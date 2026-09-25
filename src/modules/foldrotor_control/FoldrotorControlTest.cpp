/****************************************************************************
 *
 * foldrotor_control — step 3 tests:
 *  1) FR_* param registration and defaults match
 *     .claude/specs/controller_params.md's table. Pure param-registry
 *     check (same pattern as src/lib/parameters/ParameterTest.cpp) — no
 *     module instantiation, no work queue, no Gazebo.
 *  2) The quaternion->Euler conversion FoldrotorControl::Run() applies
 *     ahead of the cascade math (controller.md Interface) — exercised
 *     near the pitch = +/-90 deg singularity, per
 *     controller_params.md's Verification section ("the test must
 *     exercise near that boundary, not just a level-attitude case").
 *     Uses the exact same expression as FoldrotorControl.cpp:
 *     matrix::Eulerf(matrix::Quatf(q_raw)).
 *
 * foldrotor_control — step 4a tests:
 *  4) PositionVelocityControl: position P -> velocity PID -> inertial
 *     desired force, at the exact gains in controller_params.md's table.
 *     REWORKED 2026-09-17 to mirror mc_pos_control's PositionControl
 *     structure (see PositionVelocityControl.hpp's header comment and
 *     .claude/plans/read-mc-pos-contorl-and-can-greedy-pearl.md) — a
 *     deliberate, recorded departure from the Simulink-derived structure
 *     for this loop's saturation/anti-windup MECHANISM only; the gains
 *     are unchanged. Every expected value below is derived by hand from
 *     those gains and written out in the comment above its assertion —
 *     not read back off the implementation. Covers FF-as-P,
 *     derivative-on-measurement, the gravity feedforward (15.260017 N,
 *     the measured hover weight), and the post-rework anti-windup split:
 *     Z keeps conditional-integration freezing (now against a dynamic
 *     sphere bound); X/Y now use Rundqwist tracking anti-windup instead
 *     of the old uniform conditional-integration freeze. Pure math — no
 *     Gazebo, no uORB.
 *
 * foldrotor_control — step 4c tests:
 *  5) AttitudeRateControl: attitude P -> rate PID -> body moments
 *     (controller.md "Structure (confirmed from Simulink)"), at the
 *     gains in controller_params.md's table. Roll/pitch and yaw are
 *     covered separately because their gains differ (3.5/0.1/0.5 vs
 *     2.5/0/0). Every expected value is derived by hand from the gains
 *     and written above its assertion. Covers the decisions recorded in
 *     findings.md ("Attitude/rate-loop semantics (step 4c)"): FF-as-P,
 *     derivative-on-measurement, [-pi,pi] error wrapping, and the full
 *     mc_rate_control integral (conditional integration, i_factor,
 *     clamp, landed gate). Pure math — no Gazebo, no uORB.
 *
 * foldrotor_control — step 4b tests:
 *  3) The Inertial2Body force-path rotation (controller.md, "Identified
 *     issues (confirmed) -> 1"): identity at level attitude, proper
 *     rotation (Rt^T*Rt = I, det = +1), hand-computed cases including the
 *     sustained-yaw regime the spec says the bug surfaces in, and
 *     equivalence to PX4's own Dcm transpose (which is what makes it
 *     composable with _euler above). Pure math — no Gazebo, no uORB.
 *
 * foldrotor_control — step 4e part 1 tests:
 *  6) CascadeRateGate: the multi-rate gating mechanism (findings.md
 *     "Multi-rate cascade cadence (step 4e)") driven with synthetic
 *     timestamps — fires at the expected cadence for both the 50 Hz and
 *     250 Hz periods when driven at 1000 Hz, primes on the first call,
 *     and does not "catch up" with multiple fires after a stalled/
 *     irregular gap. Pure math — no Gazebo, no uORB.
 *  7) AttitudeRateControl's updateAttitude()/updateRate() split
 *     (AttitudeRateControl.hpp, "Interface split (step 4e part 1)"):
 *     confirms `_rate_sp` holds across repeated updateRate() calls with
 *     no intervening updateAttitude() — the behaviour the multi-rate
 *     cascade actually depends on — and that update() still reproduces
 *     the exact step 4c hand-computed nominal case via the split calls,
 *     proving the split is behaviour-preserving rather than assumed so.
 *
 * foldrotor_control — step 4e part 2 tests:
 *  8) FoldrotorAllocation (step 4d, .claude/specs/allocation.md): the
 *     runtime-derived Minv against Minv*M0 ~= I and against the spec's
 *     transcribed literal, round-trip through the forward thrust-vector
 *     map, saturation on thrust/tilt, the max_tilt regression guard,
 *     fold pinned to 0, the degenerate zero-wrench guard, and a coarse
 *     sweep asserting every output stays finite. Closes allocation.md's
 *     "Minv*M0 ~= I" and "Round-trip" missing tests. Every expected
 *     value is hand-derived and written above its assertion, per this
 *     file's established style. Pure math -- no Gazebo, no uORB.
 *  9) FoldrotorControl's actuator-mapping static methods (the wiring
 *     into Run(), not the allocator class itself): the alpha->fold
 *     negation and the newtons->normalized motor round-trip against
 *     model.sdf's rotor curve. Exercised directly, without a work queue,
 *     per the step 4e allocation plan's Part C mapping tests.
 *
 ****************************************************************************/

#include "Inertial2Body.hpp"
#include "PositionVelocityControl.hpp"
#include "AttitudeRateControl.hpp"
#include "CascadeRateGate.hpp"
#include "FoldrotorAllocation.hpp"
#include "FoldrotorControl.hpp"

#include <gtest/gtest.h>

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module_params.h>
#include <matrix/matrix/math.hpp>

#include <cmath>

class FoldrotorControlParamsTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		param_control_autosave(false);
		param_reset_all();
	}
};

namespace
{

struct ExpectedParam {
	px4::params id;
	float default_value;
};

// One entry per row in controller_params.md's parameter table.
// D gains zeroed 2026-09-22. With the derivative taken on the
// MEASUREMENT against a plant I*rate_dot = M, the D term collects as
// (I + D)*rate_dot = FF*e_r -- synthetic inertia, not damping (see
// foldrotor_control_params.yaml and findings.md 2026-09-22 (13)).
// D/I was 13%/7%/19% for roll/pitch/yaw, negligible at the 0.36-0.62
// Hz loop bandwidths, but D's gain rises with frequency and it
// supplied the bulk of the commanded moment at the 18.8 Hz actuator
// limit cycle.
const ExpectedParam kExpectedParams[] = {
	{px4::params::FR_POS_P, 0.4f},

	{px4::params::FR_VEL_XY_FF, 1.0f},
	{px4::params::FR_VEL_XY_I, 0.2f},
	{px4::params::FR_VEL_XY_D, 0.0f},

	{px4::params::FR_VEL_Z_FF, 7.0f},
	{px4::params::FR_VEL_Z_I, 7.0f},
	{px4::params::FR_VEL_Z_D, 0.0f},
	{px4::params::FR_VEL_Z_GRAV_FF, 19.6014f},
	{px4::params::FR_VEL_Z_I_LIM, 3.0f},

	{px4::params::FR_VEL_XY_I_LIM, 0.3f},

	{px4::params::FR_VEL_XY_MAX, 1.0f},
	{px4::params::FR_VEL_Z_MAX_UP, 0.3f},
	{px4::params::FR_VEL_Z_MAX_DN, 0.7f},

	{px4::params::FR_ATT_P, 4.0f},

	{px4::params::FR_RATE_R_FF, 0.49f},
	{px4::params::FR_RATE_R_I, 0.5f},
	{px4::params::FR_RATE_R_D, 0.0f},
	{px4::params::FR_RATE_R_I_LIM, 0.4f},

	// Pitch is NOT roll's gain. Both are now sized directly from the
	// MEASURED per-axis moment authority rather than scaled off each
	// other -- FF = authority / linear_range, see findings.md
	// 2026-09-21 (6) and foldrotor_control_params.yaml's descriptions.
	{px4::params::FR_PITCH_LEVER, 0.0f},
	{px4::params::FR_RATE_P_FF, 0.11f},
	{px4::params::FR_RATE_P_I, 0.112f},
	{px4::params::FR_RATE_P_D, 0.0f},
	{px4::params::FR_RATE_P_I_LIM, 0.05f},

	{px4::params::FR_RATE_YAW_FF, 0.62f},
	{px4::params::FR_RATE_YAW_I, 0.0f},
	{px4::params::FR_RATE_YAW_D, 0.0f},

	// Command-path bandwidth limit, 2026-09-22. Not a gain: the corner
	// of the first-order low-pass on the commanded wrench, sized between
	// the loop bandwidths (<=0.62 Hz) and the actuator poles (>=6.1 Hz).
	{px4::params::FR_WRENCH_LP, 5.0f},
};

} // namespace

TEST_F(FoldrotorControlParamsTest, DefaultsMatchControllerParamsSpec)
{
	for (const auto &expected : kExpectedParams) {
		const param_t handle = param_handle(expected.id);
		ASSERT_NE(handle, PARAM_INVALID);

		float value = NAN;
		ASSERT_EQ(0, param_get(handle, &value));
		EXPECT_FLOAT_EQ(expected.default_value, value);
	}
}

namespace
{

// Reproduces FoldrotorControl::Run()'s conversion exactly:
//   _euler = matrix::Eulerf(matrix::Quatf(attitude.q));
// attitude.q is a raw float[4] off the wire (vehicle_attitude_s), so the
// round trip below goes through the same raw-array constructor rather than
// staying in matrix::Quatf the whole way.
matrix::Eulerf roundTripThroughWireQuaternion(const matrix::Eulerf &euler_in)
{
	const matrix::Quatf q_in(euler_in);
	const float q_raw[4] = {q_in(0), q_in(1), q_in(2), q_in(3)};
	return matrix::Eulerf(matrix::Quatf(q_raw));
}

} // namespace

TEST(FoldrotorControlQuaternionToEulerTest, LevelAttitudeRoundTrips)
{
	const matrix::Eulerf euler_out = roundTripThroughWireQuaternion(matrix::Eulerf(0.f, 0.f, 0.f));

	EXPECT_NEAR(euler_out.phi(), 0.f, 1e-5f);
	EXPECT_NEAR(euler_out.theta(), 0.f, 1e-5f);
	EXPECT_NEAR(euler_out.psi(), 0.f, 1e-5f);
}

TEST(FoldrotorControlQuaternionToEulerTest, AwayFromSingularityRoundTripsExactly)
{
	// Non-trivial roll/pitch/yaw, far from pitch = +/-90 deg, where the
	// phi/theta/psi decomposition is unique — controller.md's cascade math
	// assumes exactly this recovered triple.
	const matrix::Eulerf euler_in(0.4f, 0.2f, 1.0f);
	const matrix::Eulerf euler_out = roundTripThroughWireQuaternion(euler_in);

	EXPECT_NEAR(euler_out.phi(), euler_in.phi(), 1e-4f);
	EXPECT_NEAR(euler_out.theta(), euler_in.theta(), 1e-4f);
	EXPECT_NEAR(euler_out.psi(), euler_in.psi(), 1e-4f);
}

TEST(FoldrotorControlQuaternionToEulerTest, NearPitchSingularityStaysFiniteAndRotationEquivalent)
{
	// controller.md / controller_params.md: ZYX Euler extraction has a
	// known singularity at pitch = +/-90 deg (gimbal lock: phi and psi
	// become non-unique, only their combination is defined). This must not
	// be assumed safe just because matrix::Eulerf is a textbook-standard
	// PX4 library utility -- verify the module's actual usage here.
	// Euler.hpp's Dcm->Euler constructor special-cases |theta - +/-pi/2| <
	// 1e-3 rad (phi forced to 0, psi absorbs it). Offset 5e-4 rad lands
	// inside that band and hits the special-cased branch; offset 1e-2 rad
	// stays in the general atan2 branch but is still close enough to be
	// numerically sensitive (cos(theta) near zero). Both regimes are
	// exercised, verified below by compiling and running this exact logic
	// against the real matrix headers (phi_out snaps to 0 at 5e-4, tracks
	// the input at 1e-2 — matches the library's documented behavior).
	const float near_singularity_offsets[] = {5e-4f, 1e-2f};
	const float signs[] = {-1.f, 1.f};

	for (float sign : signs) {
		for (float offset : near_singularity_offsets) {
			const float theta = sign * (static_cast<float>(M_PI) / 2.f - offset);
			const matrix::Eulerf euler_in(0.3f, theta, 0.6f);

			const matrix::Eulerf euler_out = roundTripThroughWireQuaternion(euler_in);

			ASSERT_TRUE(PX4_ISFINITE(euler_out.phi()));
			ASSERT_TRUE(PX4_ISFINITE(euler_out.theta()));
			ASSERT_TRUE(PX4_ISFINITE(euler_out.psi()));

			// phi/psi individually are not required to match euler_in near
			// gimbal lock -- what must hold is that the recovered triple
			// describes the *same rotation* (same DCM), since that DCM is
			// what Inertial2Body (step 4) will actually apply.
			// Brace-init, not (): matrix::Dcmf dcm_in(matrix::Quatf(euler_in))
			// parses as a function declaration (most vexing parse) — caught
			// by actually compiling this against the real matrix headers
			// before committing, not just by inspection.
			const matrix::Dcmf dcm_in{matrix::Quatf(euler_in)};
			const matrix::Dcmf dcm_out(euler_out);

			for (int i = 0; i < 3; i++) {
				for (int j = 0; j < 3; j++) {
					EXPECT_NEAR(dcm_in(i, j), dcm_out(i, j), 5e-3f)
							<< "mismatch at (" << i << "," << j << ") theta=" << theta;
				}
			}
		}
	}
}

// ---------------------------------------------------------------------------
// Step 4b: Inertial2Body force-path rotation
// (controller.md, "Identified issues (confirmed) -> 1. Missing
// inertial->body rotation on the force path"). REWORKED 2026-09-17 from
// the full 3-2-1 Euler rotation to a yaw-only 2D rotation, alongside
// PositionVelocityControl's mc_pos_control-structure rework -- see
// Inertial2Body.hpp's header comment.
//
// Pure math -- no uORB, no module instantiation, no Gazebo, same category
// as the quaternion singularity tests above.
// ---------------------------------------------------------------------------

namespace
{

// Independent 3x3 determinant by cofactor expansion. Deliberately hand-
// written rather than taken from the matrix library: the point of the
// proper-rotation check is to test the transcribed Rt against arithmetic
// we control, not against another routine from the same library the
// transcription is already being compared to.
float determinant3x3(const matrix::Dcmf &m)
{
	return m(0, 0) * (m(1, 1) * m(2, 2) - m(1, 2) * m(2, 1))
	       - m(0, 1) * (m(1, 0) * m(2, 2) - m(1, 2) * m(2, 0))
	       + m(0, 2) * (m(1, 0) * m(2, 1) - m(1, 1) * m(2, 0));
}

void expectProperRotation(const matrix::Dcmf &Rt, const char *context)
{
	// Rt^T * Rt == I (orthonormal) ...
	const matrix::SquareMatrix<float, 3> should_be_identity = Rt.transpose() * Rt;
	const matrix::SquareMatrix<float, 3> identity = matrix::eye<float, 3>();

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			EXPECT_NEAR(should_be_identity(i, j), identity(i, j), 1e-5f)
					<< "Rt^T*Rt mismatch at (" << i << "," << j << ") for " << context;
		}
	}

	// ... and det == +1, not -1: a rotation, not a reflection. Orthonormality
	// alone does not rule out a reflection, which would silently mirror an
	// axis and is exactly the kind of transcription slip this catches.
	EXPECT_NEAR(determinant3x3(Rt), 1.f, 1e-5f) << "det(Rt) != 1 for " << context;
}

} // namespace

TEST(FoldrotorControlInertial2BodyTest, LevelAttitudeIsIdentity)
{
	// At exactly level the stage must be an identity, so inserting it
	// cannot change any existing near-level result.
	const matrix::Quatf level(matrix::Eulerf(0.f, 0.f, 0.f));
	const matrix::Vector3f F_i(3.f, -4.f, 5.f);

	const matrix::Vector3f F_b = foldrotor::inertialToBody(F_i, level);

	EXPECT_NEAR(F_b(0), F_i(0), 1e-5f);
	EXPECT_NEAR(F_b(1), F_i(1), 1e-5f);
	EXPECT_NEAR(F_b(2), F_i(2), 1e-5f);

	expectProperRotation(matrix::Dcmf(level).transpose(), "level attitude");
}

// THE regression test for 2026-09-21 (7). Roll and pitch must NOT be
// ignored: a tilted vehicle asking for NED-vertical lift must receive a
// body-frame force that is tilted the opposite way, so that the delivered
// inertial force is still vertical.
//
// The yaw-only stage this replaces returned F_i unchanged here, which is
// what silently converted lift into sideways thrust and flew the vehicle
// 272 m downrange. The final assertion is the size of that defect.
TEST(FoldrotorControlInertial2BodyTest, PitchIsNotIgnored)
{
	// Nose up 22 deg (the sustained pitch measured in SITL), no roll/yaw.
	const float theta = math::radians(22.f);
	const matrix::Quatf tilted(matrix::Eulerf(0.f, theta, 0.f));

	// Hover lift: NED z is DOWN-positive, so "up" is negative.
	const matrix::Vector3f F_i(0.f, 0.f, -15.26f);
	const matrix::Vector3f F_b = foldrotor::inertialToBody(F_i, tilted);

	// R_ned_to_body for a pure pitch theta maps (0,0,-Fz) to
	// (Fz*sin(theta), 0, -Fz*cos(theta)).
	EXPECT_NEAR(F_b(0), 15.26f * std::sin(theta), 1e-4f);
	EXPECT_NEAR(F_b(1), 0.f, 1e-4f);
	EXPECT_NEAR(F_b(2), -15.26f * std::cos(theta), 1e-4f);

	// Round trip: rotating back must reproduce the commanded inertial
	// force exactly. This is the property the whole stage exists for and
	// the one the yaw-only version did not have.
	const matrix::Vector3f back = matrix::Dcmf(tilted) * F_b;
	EXPECT_NEAR(back(0), F_i(0), 1e-4f);
	EXPECT_NEAR(back(1), F_i(1), 1e-4f);
	EXPECT_NEAR(back(2), F_i(2), 1e-4f);

	// Size of the defect: the yaw-only stage was an identity at zero yaw,
	// so it would have handed F_i straight through, and the vehicle would
	// have delivered R*F_i -- leaving this much uncommanded NED-horizontal
	// force. Compare against kPosVelForceXYLimit, the authority the
	// position loop has to fight it with.
	const matrix::Vector3f delivered_by_yaw_only = matrix::Dcmf(tilted) * F_i;
	const float stray_horizontal = matrix::Vector2f(delivered_by_yaw_only(0),
				       delivered_by_yaw_only(1)).norm();
	EXPECT_GT(stray_horizontal, 5.f * FoldrotorControl::kPosVelForceXYLimit)
			<< "the yaw-only stage leaked " << stray_horizontal
			<< " N of horizontal force at 22 deg of pitch";
}

TEST(FoldrotorControlInertial2BodyTest, RollIsNotIgnored)
{
	const float phi = math::radians(30.f);
	const matrix::Quatf rolled(matrix::Eulerf(phi, 0.f, 0.f));

	const matrix::Vector3f F_i(0.f, 0.f, -15.26f);
	const matrix::Vector3f F_b = foldrotor::inertialToBody(F_i, rolled);

	// Pure roll phi maps (0,0,-Fz) to (0, -Fz*sin(phi), -Fz*cos(phi)).
	EXPECT_NEAR(F_b(0), 0.f, 1e-4f);
	EXPECT_NEAR(F_b(1), -15.26f * std::sin(phi), 1e-4f);
	EXPECT_NEAR(F_b(2), -15.26f * std::cos(phi), 1e-4f);
}

TEST(FoldrotorControlInertial2BodyTest, IsAProperRotationAtAnyAttitude)
{
	const float angles[][3] = {
		{0.f, 0.f, 0.f}, {0.f, 0.f, 1.1f}, {0.f, 0.f, -2.5f},
		{0.4f, -0.3f, 0.7f}, {1.2f, 0.9f, -2.0f}, {-1.5f, 1.4f, 3.0f},
	};

	for (const auto &a : angles) {
		const matrix::Dcmf Rt = matrix::Dcmf(matrix::Quatf(matrix::Eulerf(a[0], a[1], a[2]))).transpose();
		expectProperRotation(Rt, "attitude sweep");
	}
}

// No singularity at pitch = +/-90 deg. The pre-2026-09-17 full-Euler
// version reconstructed the rotation from phi/theta/psi and so inherited
// 3-2-1 gimbal lock there; controller.md and controller_params.md both
// still carry that as an open concern. Taking the DCM from the quaternion
// removes it -- the concern was about the PARAMETERIZATION, not about
// using the full attitude.
TEST(FoldrotorControlInertial2BodyTest, NoSingularityAtNinetyDegreePitch)
{
	for (float theta : {math::radians(89.9f), math::radians(90.f), math::radians(-90.f)}) {
		const matrix::Quatf q(matrix::Eulerf(0.f, theta, 0.7f));
		const matrix::Vector3f F_b = foldrotor::inertialToBody(matrix::Vector3f(0.f, 0.f, -15.26f), q);

		EXPECT_TRUE(std::isfinite(F_b(0)) && std::isfinite(F_b(1)) && std::isfinite(F_b(2)))
				<< "non-finite force at theta = " << theta;
		EXPECT_NEAR(F_b.norm(), 15.26f, 1e-3f) << "rotation changed the magnitude at theta = " << theta;

		expectProperRotation(matrix::Dcmf(q).transpose(), "90 deg pitch");
	}
}

TEST(FoldrotorControlInertial2BodyTest, PureYawMatchesHandComputedRotation)
{
	// Yaw behaviour is unchanged by the 2026-09-21 (7) rework and still
	// matters: at psi = +90 deg, phi = theta = 0,
	//
	//   Rt = [  0  1  0 ]
	//        [ -1  0  0 ]
	//        [  0  0  1 ]
	//
	// so F_i = (3, 4, 5) -> F_b = (4, -3, 5).
	const matrix::Quatf yawed_90(matrix::Eulerf(0.f, 0.f, static_cast<float>(M_PI) / 2.f));
	const matrix::Vector3f F_i(3.f, 4.f, 5.f);

	const matrix::Vector3f F_b = foldrotor::inertialToBody(F_i, yawed_90);

	EXPECT_NEAR(F_b(0), 4.f, 1e-5f);
	EXPECT_NEAR(F_b(1), -3.f, 1e-5f);
	EXPECT_NEAR(F_b(2), 5.f, 1e-5f);

	// Physical cross-check on the sign: nose east, so a due-north (+X NED)
	// force pushes the vehicle to its left, which is -Y in body FRD.
	const matrix::Vector3f north = foldrotor::inertialToBody(matrix::Vector3f(1.f, 0.f, 0.f), yawed_90);
	EXPECT_NEAR(north(0), 0.f, 1e-5f);
	EXPECT_NEAR(north(1), -1.f, 1e-5f);
	EXPECT_NEAR(north(2), 0.f, 1e-5f);
}

TEST(FoldrotorControlInertial2BodyTest, MatchesPx4DcmTransposeAcrossAttitudes)
{
	// Confirms the stage is exactly PX4's own standard DCM transpose, not
	// an independently hand-rolled rotation.
	const float angles[][3] = {
		{0.f, 0.f, 0.f}, {0.6f, -0.3f, 0.4f}, {0.f, 0.f, -1.1f},
		{0.2f, 1.0f, static_cast<float>(M_PI) / 2.f}, {-0.8f, -0.5f, 3.0f},
	};

	for (const auto &a : angles) {
		const matrix::Quatf q(matrix::Eulerf(a[0], a[1], a[2]));
		const matrix::Vector3f F_i(3.f, -4.f, 5.f);
		const matrix::Vector3f got = foldrotor::inertialToBody(F_i, q);
		const matrix::Vector3f expected = matrix::Dcmf(q).transpose() * F_i;

		for (int i = 0; i < 3; i++) {
			EXPECT_NEAR(got(i), expected(i), 1e-5f) << "component " << i;
		}
	}
}

/****************************************************************************
 * Step 4a — PositionVelocityControl.
 *
 * Gains throughout are exactly controller_params.md's table:
 *   FR_POS_P        = 3.0   (all axes)
 *   FR_VEL_XY_FF/I/D = 6.0 / 1.0 / 1.0
 *   FR_VEL_Z_FF/I/D  = 7.0 / 7.0 / 0.1
 *   FR_VEL_Z_GRAV_FF = 15.260017 (measured hover weight; was 9.81, fixed 2026-09-09)
 * with FR_VEL_*_FF read as the P gain on the velocity error, per the
 * decision recorded in findings.md.
 ****************************************************************************/

namespace
{

foldrotor::PositionVelocityControl makeSpecDefaultController()
{
	foldrotor::PositionVelocityControl ctrl;
	ctrl.setPositionGain(3.0f);
	ctrl.setVelocityGains(6.0f, 1.0f, 1.0f,   // FR_VEL_XY_FF / _I / _D
			      7.0f, 7.0f, 0.1f);  // FR_VEL_Z_FF  / _I / _D
	ctrl.setGravityFeedforward(15.260017f);
	return ctrl;
}

constexpr float kTol = 1e-4f;

} // namespace

// Nominal case, X and Y, hand-computed:
//   v_sp  = 3 * (pos_sp - pos) = 3 * (2, -1, 0)   = (6, -3, 0)
//   e_v   = v_sp - vel = (6-0.5, -3-0.25, 0-0)    = (5.5, -3.25, 0)
//   Fx    = 6 * 5.5                               =  33.0
//   Fy    = 6 * (-3.25)                           = -19.5
//   Fz    = 7 * 0 + 0 (integral) - 0.1 * 0 - 15.260017 = -15.260017
//   (grav_ff SUBTRACTED, not added -- sign flipped 2026-09-11,
//   PositionVelocityControl.hpp OPEN ITEM (b), REOPENED not resolved)
// The integral contributes nothing on the first call, so this isolates
// the position P stage and the velocity P stage together.
TEST(FoldrotorPositionVelocityControlTest, NominalXYMatchesHandComputedCascade)
{
	auto ctrl = makeSpecDefaultController();

	const matrix::Vector3f F = ctrl.update(matrix::Vector3f(0.f, 0.f, 0.f),
					       matrix::Vector3f(2.f, -1.f, 0.f),
					       matrix::Vector3f(0.5f, 0.25f, 0.f),
					       matrix::Vector3f(0.f, 0.f, 0.f),
					       0.01f);

	EXPECT_NEAR(F(0), 33.0f, kTol);
	EXPECT_NEAR(F(1), -19.5f, kTol);
	EXPECT_NEAR(F(2), -15.260017f, kTol);

	// And the integral advanced by I * e_v * dt, per axis:
	//   x: 1 *  5.50 * 0.01 =  0.055
	//   y: 1 * -3.25 * 0.01 = -0.0325
	//   z: 7 *  0.00 * 0.01 =  0
	EXPECT_NEAR(ctrl.getIntegral()(0), 0.055f, kTol);
	EXPECT_NEAR(ctrl.getIntegral()(1), -0.0325f, kTol);
	EXPECT_NEAR(ctrl.getIntegral()(2), 0.0f, kTol);
}

// With everything at zero, the only output is the Z gravity feedforward,
// and it appears on Z alone. This pins OPEN ITEM (b), REOPENED 2026-09-11
// (PositionVelocityControl.hpp): FR_VEL_Z_GRAV_FF is a positive weight
// measurement, but body FRD/NED is Z DOWN-positive, so a force opposing
// gravity needs a NEGATIVE Z contribution -- the value is now negative,
// not positive-down as this test previously asserted. Magnitude is still
// the measured hover weight (15.260017 N, resolved 2026-09-09; was 9.81 N
// before), per OPEN ITEM (a)'s fix -- only the sign of how it enters the
// Z axis changed. Not re-verified against a bench pass (see the header
// comment's OPEN ITEM (b) re-verification note) -- if that sign is ever
// decided to be wrong again, this test is the thing that must change with
// it — deliberately, not silently.
TEST(FoldrotorPositionVelocityControlTest, GravityFeedforwardIsLiteralWeightOnZOnly)
{
	auto ctrl = makeSpecDefaultController();

	const matrix::Vector3f F = ctrl.update(matrix::Vector3f(), matrix::Vector3f(),
					       matrix::Vector3f(), matrix::Vector3f(), 0.01f);

	EXPECT_NEAR(F(0), 0.0f, kTol);
	EXPECT_NEAR(F(1), 0.0f, kTol);
	EXPECT_NEAR(F(2), -15.260017f, kTol);
}

// Derivative acts on the measured velocity derivative, negated:
//   Fx = -1.0 * 1 = -1.0
//   Fy = -1.0 * 2 = -2.0
//   Fz = -0.1 * 3 - 15.260017 = -15.560017
//   (grav_ff SUBTRACTED -- sign flipped 2026-09-11, see
//   NominalXYMatchesHandComputedCascade's comment)
// Note the Z coefficient differs from X/Y (0.1 vs 1.0), which is why the
// axes are checked separately rather than as one shared gain.
TEST(FoldrotorPositionVelocityControlTest, DerivativeActsOnMeasurementAtPerAxisGains)
{
	auto ctrl = makeSpecDefaultController();

	const matrix::Vector3f F = ctrl.update(matrix::Vector3f(), matrix::Vector3f(),
					       matrix::Vector3f(),
					       matrix::Vector3f(1.f, 2.f, 3.f),
					       0.01f);

	EXPECT_NEAR(F(0), -1.0f, kTol);
	EXPECT_NEAR(F(1), -2.0f, kTol);
	EXPECT_NEAR(F(2), -15.560017f, kTol);
}

// The discriminating test for derivative-on-measurement vs
// derivative-on-error. A position setpoint step of 2 m with dt = 0.01
// steps the velocity setpoint by 3*2 = 6 m/s in one sample. Derivative
// on the *error* would add D * 6 / 0.01 = 600 N to Fx. Derivative on the
// measurement adds nothing, because vel_dot is zero:
//   Fx = 6 * 6 = 36.0
TEST(FoldrotorPositionVelocityControlTest, SetpointStepProducesNoDerivativeKick)
{
	auto ctrl = makeSpecDefaultController();

	// Settled at the origin: no error, so no integral is accumulated.
	ctrl.update(matrix::Vector3f(), matrix::Vector3f(), matrix::Vector3f(),
		    matrix::Vector3f(), 0.01f);
	ASSERT_NEAR(ctrl.getIntegral()(0), 0.0f, kTol);

	const matrix::Vector3f F = ctrl.update(matrix::Vector3f(),
					       matrix::Vector3f(2.f, 0.f, 0.f),
					       matrix::Vector3f(), matrix::Vector3f(),
					       0.01f);

	EXPECT_NEAR(F(0), 36.0f, kTol);
}

// X/Y integral accumulation at FR_VEL_XY_I = 1, dt = 0.1, e_v = 3:
//   call 1: Fx = 6*3 + 0.0 = 18.0 ; integral -> 1*3*0.1 = 0.3
//   call 2: Fx = 6*3 + 0.3 = 18.3 ; integral -> 0.6
//   call 3: Fx = 6*3 + 0.6 = 18.6 ; integral -> 0.9
TEST(FoldrotorPositionVelocityControlTest, IntegralAccumulatesAtXYGain)
{
	auto ctrl = makeSpecDefaultController();

	const matrix::Vector3f pos_sp(1.f, 0.f, 0.f);
	const matrix::Vector3f zero;

	EXPECT_NEAR(ctrl.update(zero, pos_sp, zero, zero, 0.1f)(0), 18.0f, kTol);
	EXPECT_NEAR(ctrl.update(zero, pos_sp, zero, zero, 0.1f)(0), 18.3f, kTol);
	EXPECT_NEAR(ctrl.update(zero, pos_sp, zero, zero, 0.1f)(0), 18.6f, kTol);
	EXPECT_NEAR(ctrl.getIntegral()(0), 0.9f, kTol);
}

// Z integral accumulation at FR_VEL_Z_I = 7 — the aggressive one — with
// the gravity feedforward riding along, dt = 0.1, e_v = 3. grav_ff is
// SUBTRACTED, not added (sign flipped 2026-09-11, see
// NominalXYMatchesHandComputedCascade's comment):
//   call 1: Fz = 7*3 + 0.0 - 15.260017 =  5.739983 ; integral -> 7*3*0.1 = 2.1
//   call 2: Fz = 7*3 + 2.1 - 15.260017 =  7.839983 ; integral -> 4.2
//   call 3: Fz = 7*3 + 4.2 - 15.260017 =  9.939983 ; integral -> 6.3
// (Integral accumulation itself does not depend on the feedforward value
// or its sign -- getIntegral()(2) is unchanged from before the flip.)
TEST(FoldrotorPositionVelocityControlTest, IntegralAccumulatesAtZGainWithGravityFeedforward)
{
	auto ctrl = makeSpecDefaultController();

	const matrix::Vector3f pos_sp(0.f, 0.f, 1.f);
	const matrix::Vector3f zero;

	EXPECT_NEAR(ctrl.update(zero, pos_sp, zero, zero, 0.1f)(2), 5.739983f, kTol);
	EXPECT_NEAR(ctrl.update(zero, pos_sp, zero, zero, 0.1f)(2), 7.839983f, kTol);
	EXPECT_NEAR(ctrl.update(zero, pos_sp, zero, zero, 0.1f)(2), 9.939983f, kTol);
	EXPECT_NEAR(ctrl.getIntegral()(2), 6.3f, kTol);
}

TEST(FoldrotorPositionVelocityControlTest, ResetIntegralClearsAccumulatedState)
{
	auto ctrl = makeSpecDefaultController();

	const matrix::Vector3f zero;
	ctrl.update(zero, matrix::Vector3f(1.f, 0.f, 1.f), zero, zero, 0.1f);
	ASSERT_GT(fabsf(ctrl.getIntegral()(0)), 0.f);
	ASSERT_GT(fabsf(ctrl.getIntegral()(2)), 0.f);

	ctrl.resetIntegral();

	EXPECT_NEAR(ctrl.getIntegral()(0), 0.0f, kTol);
	EXPECT_NEAR(ctrl.getIntegral()(1), 0.0f, kTol);
	EXPECT_NEAR(ctrl.getIntegral()(2), 0.0f, kTol);

	// And a subsequent settled call returns to the bare gravity term.
	// grav_ff SUBTRACTED, not added (sign flipped 2026-09-11, see
	// NominalXYMatchesHandComputedCascade's comment).
	const matrix::Vector3f F = ctrl.update(zero, zero, zero, zero, 0.1f);
	EXPECT_NEAR(F(0), 0.0f, kTol);
	EXPECT_NEAR(F(2), -15.260017f, kTol);
}

// Default limits are a huge ceiling with zero margin
// (PositionVelocityControl.hpp's _lim_force_max/_lim_force_xy_margin
// defaults, 2026-09-17 rework), so nothing clamps and no anti-windup
// engages for any realistic test magnitude -- the same "no-op unless
// configured" behavior the previous +/-infinity output-limit boxes gave.
TEST(FoldrotorPositionVelocityControlTest, DefaultLimitsNeitherClampNorFreeze)
{
	auto ctrl = makeSpecDefaultController();

	const matrix::Vector3f zero;
	const matrix::Vector3f pos_sp(2.f, 0.f, 0.f);

	// e_v = 6, so Fx = 36 unclamped, integral grows by 1*6*0.1 = 0.6/step.
	for (int i = 0; i < 5; i++) {
		ctrl.update(zero, pos_sp, zero, zero, 0.1f);
	}

	EXPECT_NEAR(ctrl.getIntegral()(0), 3.0f, kTol);
}

// 2026-09-17 rework: X/Y no longer use conditional-integration freezing
// (that mechanism now applies to Z only, mirroring mc_pos_control's own
// asymmetric split -- see PositionVelocityControl.hpp decision 3). X/Y
// instead use Rundqwist tracking anti-windup, which does NOT zero the
// error outright; it tempers it by arw_gain*(desired - produced) so the
// integrator keeps moving toward the equilibrium point that holds the
// output exactly at the saturation boundary, rather than freezing at
// zero (the old uniform-conditional-integration behavior) or winding up
// unboundedly (DefaultLimitsNeitherClampNorFreeze's unbounded case).
//
// Setup: combined force limit 20 N, zero margin, zero gravity
// feedforward (a fresh controller, not makeSpecDefaultController() --
// isolates the X-axis geometry from Z's constant -15.260017 N offset,
// which would otherwise eat into the sphere radius available to X).
//
// pos_sp = (2,0,0) held constant every call -> vel_sp = 6, e_v = 6
// (vel and pos never change), so e_v is the same 6 every call; only the
// integral term changes the unclamped Fx from call to call.
//
// Hand-derived recurrence (dt=0.1, FR_VEL_XY_FF=6, FR_VEL_XY_I=1,
// arw_gain = 2/6 = 1/3):
//   F_desired = 6*6 + I_prev = 36 + I_prev  (always > 20 here, so clamped)
//   F_produced = 20                          (sphere radius, X-only)
//   e_v_adjusted = 6 - (1/3)*(F_desired - F_produced)
//                = 6 - (1/3)*(16 + I_prev) = (2 - I_prev)/3
//   I_new = I_prev + 0.1*1*e_v_adjusted = I_prev*(29/30) + 1/15
//
//   I0 = 0
//   I1 = 1/15                    = 0.0666667
//   I2 = I1*29/30 + 1/15         = 0.1311111
//   I3 = I2*29/30 + 1/15         = 0.1934074
//
// I climbs toward a finite fixed point (I* = 2, where e_v_adjusted = 0)
// instead of freezing at 0 or growing without bound -- the output stays
// pinned at the 20 N sphere edge every call regardless (scale always
// renormalizes the clamped vector back to exactly the radius).
TEST(FoldrotorPositionVelocityControlTest, TrackingAntiWindupDampensIntegrationWhileSaturatedX)
{
	foldrotor::PositionVelocityControl ctrl;
	ctrl.setPositionGain(3.0f);
	ctrl.setVelocityGains(6.0f, 1.0f, 1.0f, 7.0f, 7.0f, 0.1f);
	ctrl.setForceLimits(20.f);
	ctrl.setHorizontalForceMargin(0.f);

	const matrix::Vector3f zero;
	const matrix::Vector3f pos_sp(2.f, 0.f, 0.f);

	EXPECT_NEAR(ctrl.update(zero, pos_sp, zero, zero, 0.1f)(0), 20.0f, kTol);
	EXPECT_NEAR(ctrl.getIntegral()(0), 0.0666667f, 1e-3f);

	EXPECT_NEAR(ctrl.update(zero, pos_sp, zero, zero, 0.1f)(0), 20.0f, kTol);
	EXPECT_NEAR(ctrl.getIntegral()(0), 0.1311111f, 1e-3f);

	EXPECT_NEAR(ctrl.update(zero, pos_sp, zero, zero, 0.1f)(0), 20.0f, kTol);
	EXPECT_NEAR(ctrl.getIntegral()(0), 0.1934074f, 1e-3f);
}

// Z keeps the old conditional-integration mechanism exactly (decision 3),
// just evaluated against the dynamic sphere bound instead of a static
// box -- with X/Y at zero, the sphere bound collapses to exactly the
// combined force limit, so this reproduces the pre-rework test's numbers
// unchanged. grav_ff is SUBTRACTED (2026-09-11 sign fix, unaffected by
// this rework): pos_sp_z = 2 gives e_v = 6, Fz = 7*6 - 15.260017 =
// 26.739983, above the 20 N limit -- saturates high with a positive
// error, so the integrator freezes at zero every step (unlike X/Y's
// tracking ARW above, Z's freeze is a hard zero, not a tempered value).
TEST(FoldrotorPositionVelocityControlTest, IntegratorFreezesWhileSaturatedZ)
{
	auto ctrl = makeSpecDefaultController();
	ctrl.setForceLimits(20.f);
	ctrl.setHorizontalForceMargin(0.f);

	const matrix::Vector3f zero;
	const matrix::Vector3f pos_sp(0.f, 0.f, 2.f);

	for (int i = 0; i < 5; i++) {
		EXPECT_NEAR(ctrl.update(zero, pos_sp, zero, zero, 0.1f)(2), 20.0f, kTol);
	}

	EXPECT_NEAR(ctrl.getIntegral()(2), 0.0f, kTol);

	// Reopen and probe with zero error: Fz is then integral - gravity.
	// Frozen  -> 0    - 15.260017 = -15.260017
	// Unfrozen would have been 21 - 15.260017 = 5.739983.
	ctrl.setForceLimits(1e6f);
	EXPECT_NEAR(ctrl.update(zero, zero, zero, zero, 0.1f)(2), -15.260017f, kTol);
}

// FR_VEL_Z_I_LIM = 3.0 N (2026-09-09), a separate mechanism from the
// output-limit-driven conditional integration above: it bounds the
// accumulated integral directly via setIntegratorLimit(), regardless of
// whether the output itself is saturated. Output limits are left at
// their default +/-infinity here so nothing freezes via that path --
// only the new clamp acts.
//
// e_v = 3*(pos_sp_z - 0) - 0 = 30, so the unclamped increment per 0.1 s
// step is 7*30*0.1 = 21 -- one step already blows past +/-3.
TEST(FoldrotorPositionVelocityControlTest, IntegratorClampsAtFrVelZILim)
{
	auto ctrl = makeSpecDefaultController();
	ctrl.setIntegratorLimit(matrix::Vector3f(INFINITY, INFINITY, 3.0f));

	const matrix::Vector3f zero;
	const matrix::Vector3f pos_sp(0.f, 0.f, 10.f);

	ctrl.update(zero, pos_sp, zero, zero, 0.1f);
	EXPECT_NEAR(ctrl.getIntegral()(2), 3.0f, kTol);

	// A second identical step would unclamp to 3.0 + 21 = 24.0; still
	// clamps at 3.0, not just "happened to land there once".
	ctrl.update(zero, pos_sp, zero, zero, 0.1f);
	EXPECT_NEAR(ctrl.getIntegral()(2), 3.0f, kTol);
}

// The freeze must be directional, not a blanket hold: once the axis
// leaves saturation, it has to integrate normally again, otherwise the
// integrator latches. The sphere bound is symmetric (unlike the old
// independent boxes, which could be configured asymmetrically to isolate
// this directly), so this uses Z -- which keeps the hard conditional-
// integration freeze (see IntegratorFreezesWhileSaturatedZ above) -- and
// demonstrates the unfreeze with a second call whose magnitude simply
// doesn't reach the bound, rather than an asymmetric limit.
//
//   step 1, pos_sp_z = +2: e_v = 6, Fz = 7*6 - 0 = 42 (grav_ff = 0 here,
//           a fresh controller, to keep the bound symmetric and exactly
//           equal to the configured limit) -> clamped to +20 with a
//           positive error -> frozen, integral stays 0.
//   step 2, pos_sp_z = -0.5: e_v = 3*(-0.5) - 0 = -1.5, Fz = 7*(-1.5) +
//           0 = -10.5, well inside +/-20 -> NOT saturated -> integral
//           moves by 7*(-1.5)*0.1 = -1.05, and the output is the
//           unclamped -10.5, not the previous cycle's clamped value.
TEST(FoldrotorPositionVelocityControlTest, IntegratorUnfreezesOnceZLeavesSaturation)
{
	foldrotor::PositionVelocityControl ctrl;
	ctrl.setPositionGain(3.0f);
	ctrl.setVelocityGains(6.0f, 1.0f, 1.0f, 7.0f, 7.0f, 0.1f);
	ctrl.setForceLimits(20.f);
	ctrl.setHorizontalForceMargin(0.f);

	const matrix::Vector3f zero;

	EXPECT_NEAR(ctrl.update(zero, matrix::Vector3f(0.f, 0.f, 2.f), zero, zero, 0.1f)(2),
		    20.0f, kTol);
	EXPECT_NEAR(ctrl.getIntegral()(2), 0.0f, kTol);

	EXPECT_NEAR(ctrl.update(zero, matrix::Vector3f(0.f, 0.f, -0.5f), zero, zero, 0.1f)(2),
		    -10.5f, kTol);
	EXPECT_NEAR(ctrl.getIntegral()(2), -1.05f, kTol);
}

// The output type composes directly with step 4b's rotation stage, which
// is the whole reason update() returns matrix::Vector3f. At level
// attitude Rt is the identity, so F_b == F_i — this asserts the two
// stages join without any adaptation, not that the rotation is correct
// (Inertial2Body has its own tests for that).
TEST(FoldrotorPositionVelocityControlTest, OutputComposesWithInertialToBody)
{
	auto ctrl = makeSpecDefaultController();

	const matrix::Vector3f F_i = ctrl.update(matrix::Vector3f(0.f, 0.f, 0.f),
				     matrix::Vector3f(2.f, -1.f, 0.f),
				     matrix::Vector3f(0.5f, 0.25f, 0.f),
				     matrix::Vector3f(0.f, 0.f, 0.f),
				     0.01f);

	const matrix::Quatf level(matrix::Eulerf(0.f, 0.f, 0.f));
	const matrix::Vector3f F_b = foldrotor::inertialToBody(F_i, level);

	// grav_ff SUBTRACTED, not added (sign flipped 2026-09-11, see
	// NominalXYMatchesHandComputedCascade's comment).
	EXPECT_NEAR(F_b(0), 33.0f, kTol);
	EXPECT_NEAR(F_b(1), -19.5f, kTol);
	EXPECT_NEAR(F_b(2), -15.260017f, kTol);
}

// Velocity-magnitude limiting (added 2026-09-17, see PositionVelocityControl
// .hpp's setVelocityLimits() comment and OPEN ITEM (b)). Horizontal case:
// a big position error would otherwise demand an unbounded vel_sp before it
// ever reaches the velocity PID.
//   raw vel_sp = 3 * (3, 4, 0) = (9, 12, 0), norm = 15
//   xy_max = 1.0 -> scale = 1/15 -> vel_sp_clamped = (0.6, 0.8, 0)
//   (direction preserved: 0.8/0.6 == 12/9, not a naive per-axis clamp)
//   e_v = vel_sp_clamped - vel = (0.6, 0.8, 0)
//   Fx = 6 * 0.6 = 3.6
//   Fy = 6 * 0.8 = 4.8
//   Fz = -15.260017 (pos_sp_z = 0, grav_ff only)
TEST(FoldrotorPositionVelocityControlTest, HorizontalVelocityLimitScalesSetpointPreservingDirection)
{
	auto ctrl = makeSpecDefaultController();
	ctrl.setVelocityLimits(1.0f, 100.0f, 100.0f);

	const matrix::Vector3f F = ctrl.update(matrix::Vector3f(0.f, 0.f, 0.f),
					       matrix::Vector3f(3.f, 4.f, 0.f),
					       matrix::Vector3f(0.f, 0.f, 0.f),
					       matrix::Vector3f(0.f, 0.f, 0.f),
					       0.01f);

	EXPECT_NEAR(F(0), 3.6f, kTol);
	EXPECT_NEAR(F(1), 4.8f, kTol);
	EXPECT_NEAR(F(2), -15.260017f, kTol);
}

// Vertical case is asymmetric (up_max != down_max), unlike horizontal —
// mirrors mc_pos_control's MPC_Z_VEL_MAX_UP/_DN convention. NED: negative
// pos_sp_z is a climb (up), positive is a descent (down).
//
// Climb: raw vel_sp_z = 3 * (-10) = -30, clamped to -up_max = -1.0.
//   e_v_z = -1.0 - 0 = -1.0
//   Fz = 7 * (-1.0) - 15.260017 = -22.260017
TEST(FoldrotorPositionVelocityControlTest, VerticalVelocityLimitClampsClimbAtUpMax)
{
	auto ctrl = makeSpecDefaultController();
	ctrl.setVelocityLimits(100.0f, 1.0f, 0.5f);

	const matrix::Vector3f F = ctrl.update(matrix::Vector3f(0.f, 0.f, 0.f),
					       matrix::Vector3f(0.f, 0.f, -10.f),
					       matrix::Vector3f(0.f, 0.f, 0.f),
					       matrix::Vector3f(0.f, 0.f, 0.f),
					       0.01f);

	EXPECT_NEAR(F(2), -22.260017f, kTol);
}

// Descent: raw vel_sp_z = 3 * 10 = 30, clamped to down_max = 0.5.
//   e_v_z = 0.5 - 0 = 0.5
//   Fz = 7 * 0.5 - 15.260017 = -11.760017
TEST(FoldrotorPositionVelocityControlTest, VerticalVelocityLimitClampsDescentAtDownMax)
{
	auto ctrl = makeSpecDefaultController();
	ctrl.setVelocityLimits(100.0f, 1.0f, 0.5f);

	const matrix::Vector3f F = ctrl.update(matrix::Vector3f(0.f, 0.f, 0.f),
					       matrix::Vector3f(0.f, 0.f, 10.f),
					       matrix::Vector3f(0.f, 0.f, 0.f),
					       matrix::Vector3f(0.f, 0.f, 0.f),
					       0.01f);

	EXPECT_NEAR(F(2), -11.760017f, kTol);
}

// Default (no setVelocityLimits() call) must not clamp -- same "+/-infinity
// is a no-op" contract as the other limit setters (setForceLimits(),
// setIntegratorLimit()). Reuses NominalXYMatchesHandComputedCascade's exact
// scenario/values; the point here is that no explicit setVelocityLimits()
// call was made.
TEST(FoldrotorPositionVelocityControlTest, DefaultVelocityLimitsDoNotClamp)
{
	auto ctrl = makeSpecDefaultController();

	const matrix::Vector3f F = ctrl.update(matrix::Vector3f(0.f, 0.f, 0.f),
					       matrix::Vector3f(2.f, -1.f, 0.f),
					       matrix::Vector3f(0.5f, 0.25f, 0.f),
					       matrix::Vector3f(0.f, 0.f, 0.f),
					       0.01f);

	EXPECT_NEAR(F(0), 33.0f, kTol);
	EXPECT_NEAR(F(1), -19.5f, kTol);
	EXPECT_NEAR(F(2), -15.260017f, kTol);
}

/****************************************************************************
 * Step 4c — AttitudeRateControl.
 *
 * These tests verify the controller's MATH -- the P/I/D structure, the
 * attitude->rate cascade, anti-windup and saturation -- so they drive it
 * with a deliberately matched roll/pitch gain set:
 *   FR_ATT_P           = 3.0             (all axes)
 *   roll  FF/I/D       = 3.5 / 0.1 / 0.5
 *   pitch FF/I/D       = 3.5 / 0.1 / 0.5  (equal to roll ON PURPOSE)
 *   yaw   FF/I/D       = 2.5 / 0.0 / 0.0
 * with FR_RATE_*_FF read as the P gain on the rate error, per the
 * decision recorded in findings.md.
 *
 * These are NOT the shipped defaults any more. Roll and pitch were split
 * on 2026-09-21 and pitch scaled by Iyy/Ixx (FR_RATE_P_FF = 0.17, not
 * 3.5) -- see foldrotor_control_params.yaml. Keeping the two axes equal
 * here is what lets the hand-computed expectations below stay symmetric
 * and independently checkable; the shipped defaults are pinned
 * separately by FoldrotorControlParamsTest.DefaultsMatchControllerParamsSpec.
 ****************************************************************************/

namespace
{

foldrotor::AttitudeRateControl makeMatchedGainAttitudeController()
{
	foldrotor::AttitudeRateControl ctrl;
	ctrl.setAttitudeGain(3.0f);
	ctrl.setRateGains(3.5f, 0.1f, 0.5f,   // roll  FF / I / D
			  3.5f, 0.1f, 0.5f,   // pitch FF / I / D -- equal to roll on purpose
			  2.5f, 0.0f, 0.0f);  // FR_RATE_YAW_FF / _I / _D
	return ctrl;
}

const matrix::Eulerf kLevel(0.f, 0.f, 0.f);
const matrix::Vector3f kZero3;

} // namespace

// Nominal roll/pitch case, hand-computed:
//   att_error = (0.2, -0.1, 0)
//   rate_sp   = 3 * att_error        = (0.6, -0.3, 0)
//   e_r       = rate_sp - rate       = (0.55, -0.32, 0)
//   Mx = 3.5 * 0.55  =  1.925
//   My = 3.5 * -0.32 = -1.12
//   Mz = 2.5 * 0     =  0
// The integral contributes nothing on the first call, so this isolates
// the attitude P stage and the rate P stage together.
TEST(FoldrotorAttitudeRateControlTest, NominalRollPitchMatchesHandComputedCascade)
{
	auto ctrl = makeMatchedGainAttitudeController();

	const matrix::Vector3f M = ctrl.update(kLevel,
					       matrix::Eulerf(0.2f, -0.1f, 0.f),
					       matrix::Vector3f(0.05f, 0.02f, 0.f),
					       kZero3, 0.01f);

	EXPECT_NEAR(M(0), 1.925f, 1e-5f);
	EXPECT_NEAR(M(1), -1.12f, 1e-5f);
	EXPECT_NEAR(M(2), 0.0f, 1e-5f);

	// The intermediate body-rate setpoint is the attitude P output.
	EXPECT_NEAR(ctrl.getRateSetpoint()(0), 0.6f, 1e-5f);
	EXPECT_NEAR(ctrl.getRateSetpoint()(1), -0.3f, 1e-5f);
}

// Yaw uses a different gain set, so it gets its own hand-computed case:
//   att_error_z = 0.4 ; rate_sp_z = 1.2 ; e_r_z = 1.2 - 0.1 = 1.1
//   Mz = 2.5 * 1.1 = 2.75
// And because FR_RATE_YAW_I = FR_RATE_YAW_D = 0, yaw is a pure
// proportional law: the integral must stay at exactly zero no matter how
// long a yaw error persists. That is asserted rather than assumed,
// because it is controller.md's Open question 2 and a future decision to
// make those gains nonzero has to break a test, not pass silently.
TEST(FoldrotorAttitudeRateControlTest, NominalYawMatchesHandComputedCascadeAndStaysProportional)
{
	auto ctrl = makeMatchedGainAttitudeController();

	const matrix::Eulerf euler_sp(0.f, 0.f, 0.4f);
	const matrix::Vector3f rate(0.f, 0.f, 0.1f);

	EXPECT_NEAR(ctrl.update(kLevel, euler_sp, rate, kZero3, 0.1f)(2), 2.75f, 1e-5f);

	for (int i = 0; i < 10; i++) {
		ctrl.update(kLevel, euler_sp, rate, kZero3, 0.1f);
	}

	EXPECT_NEAR(ctrl.getIntegral()(2), 0.0f, 1e-9f);
	EXPECT_NEAR(ctrl.update(kLevel, euler_sp, rate, kZero3, 0.1f)(2), 2.75f, 1e-5f);
}

// Derivative acts on the measured angular acceleration, negated, at the
// per-axis gains (roll/pitch 0.5, yaw 0.0):
//   Mx = -0.5 * 1 = -0.5
//   My = -0.5 * 2 = -1.0
//   Mz = -0.0 * 3 =  0.0
TEST(FoldrotorAttitudeRateControlTest, DerivativeActsOnMeasurementAtPerAxisGains)
{
	auto ctrl = makeMatchedGainAttitudeController();

	const matrix::Vector3f M = ctrl.update(kLevel, kLevel, kZero3,
					       matrix::Vector3f(1.f, 2.f, 3.f), 0.01f);

	EXPECT_NEAR(M(0), -0.5f, 1e-5f);
	EXPECT_NEAR(M(1), -1.0f, 1e-5f);
	EXPECT_NEAR(M(2), 0.0f, 1e-5f);
}

// The discriminating test for derivative-on-measurement vs
// derivative-on-error. An attitude setpoint step of 0.2 rad with
// dt = 0.01 steps the rate setpoint by 3*0.2 = 0.6 rad/s in one sample.
// Derivative on the *error* would add D * 0.6 / 0.01 = 0.5 * 60 = 30 N*m
// to Mx. Derivative on the measurement adds nothing, since rate_dot = 0:
//   Mx = 3.5 * 0.6 = 2.1
TEST(FoldrotorAttitudeRateControlTest, SetpointStepProducesNoDerivativeKick)
{
	auto ctrl = makeMatchedGainAttitudeController();

	// Settled level: no error, so no integral is accumulated.
	ctrl.update(kLevel, kLevel, kZero3, kZero3, 0.01f);
	ASSERT_NEAR(ctrl.getIntegral()(0), 0.0f, 1e-9f);

	const matrix::Vector3f M = ctrl.update(kLevel, matrix::Eulerf(0.2f, 0.f, 0.f),
					       kZero3, kZero3, 0.01f);

	EXPECT_NEAR(M(0), 2.1f, 1e-5f);
}

// Roll/pitch integral accumulation, including mc_rate_control's
// nonlinear i_factor. With e_r = 0.6, I = 0.1, dt = 0.1:
//   i_factor = 1 - (0.6 / radians(400))^2 = 0.99261368...
//   increment = i_factor * 0.1 * 0.6 * 0.1 = 0.0059556821...
//   call 1: Mx = 3.5*0.6 + 0.0        = 2.1
//   call 2: Mx = 3.5*0.6 + 0.00595568 = 2.10595568
//   call 3: Mx = 3.5*0.6 + 0.01191137 = 2.11191136
// Without i_factor the increment would be exactly 0.006 and the integral
// after three calls 0.018 rather than 0.01786705, so these values also
// discriminate the i_factor's presence.
TEST(FoldrotorAttitudeRateControlTest, IntegralAccumulatesAtRollPitchGainWithIFactor)
{
	auto ctrl = makeMatchedGainAttitudeController();

	const matrix::Eulerf euler_sp(0.2f, 0.f, 0.f);

	EXPECT_NEAR(ctrl.update(kLevel, euler_sp, kZero3, kZero3, 0.1f)(0), 2.1f, 1e-6f);
	EXPECT_NEAR(ctrl.update(kLevel, euler_sp, kZero3, kZero3, 0.1f)(0), 2.105955682f, 1e-6f);
	EXPECT_NEAR(ctrl.update(kLevel, euler_sp, kZero3, kZero3, 0.1f)(0), 2.111911364f, 1e-6f);

	EXPECT_NEAR(ctrl.getIntegral()(0), 0.017867046f, 1e-6f);
}

// The i_factor at a large rate error, where it dominates. With
// att_error = 2.0 rad, rate_sp = 6.0, e_r = 6.0, I = 0.1, dt = 0.1:
//   i_factor  = 1 - (6.0 / radians(400))^2 = 0.26136857...
//   increment = i_factor * 0.1 * 6.0 * 0.1 = 0.01568211...
// Against 0.06 without the i_factor — a factor of ~3.8, so this cannot
// pass by accident.
TEST(FoldrotorAttitudeRateControlTest, IFactorReducesIntegrationAtLargeRateError)
{
	auto ctrl = makeMatchedGainAttitudeController();

	ctrl.update(kLevel, matrix::Eulerf(2.0f, 0.f, 0.f), kZero3, kZero3, 0.1f);

	EXPECT_NEAR(ctrl.getIntegral()(0), 0.015682114f, 1e-6f);
}

// And the i_factor floors at zero: past a rate error of radians(400),
// 1 - (e/scale)^2 goes negative and is clamped, so integration stops
// entirely. att_error = 2.5 -> rate_sp = 7.5 > 6.98131700 = radians(400).
TEST(FoldrotorAttitudeRateControlTest, IFactorFloorsAtZeroBeyondScale)
{
	auto ctrl = makeMatchedGainAttitudeController();

	ctrl.update(kLevel, matrix::Eulerf(2.5f, 0.f, 0.f), kZero3, kZero3, 0.1f);

	EXPECT_NEAR(ctrl.getIntegral()(0), 0.0f, 1e-9f);
}

// The landed gate (rate_control.cpp:81-83): while landed the integral is
// frozen entirely, independently of saturation. Same setup as the
// accumulation test above, which reaches 0.0059556821 per step.
TEST(FoldrotorAttitudeRateControlTest, LandedFreezesIntegralEntirely)
{
	auto ctrl = makeMatchedGainAttitudeController();

	const matrix::Eulerf euler_sp(0.2f, 0.f, 0.f);

	for (int i = 0; i < 5; i++) {
		EXPECT_NEAR(ctrl.update(kLevel, euler_sp, kZero3, kZero3, 0.1f, true)(0), 2.1f, 1e-6f);
	}

	EXPECT_NEAR(ctrl.getIntegral()(0), 0.0f, 1e-9f);

	// Airborne again, the same input integrates normally.
	ctrl.update(kLevel, euler_sp, kZero3, kZero3, 0.1f, false);
	EXPECT_NEAR(ctrl.getIntegral()(0), 0.005955682f, 1e-6f);
}

// The integrator clamp (rate_control.cpp:112-114). Each step would add
// 0.0059556821, so a limit of 0.001 must bind on the very first call and
// hold there.
TEST(FoldrotorAttitudeRateControlTest, IntegratorLimitClampsAccumulation)
{
	auto ctrl = makeMatchedGainAttitudeController();
	ctrl.setIntegratorLimit(matrix::Vector3f(0.001f, 0.001f, 0.001f));

	const matrix::Eulerf euler_sp(0.2f, 0.f, 0.f);

	for (int i = 0; i < 5; i++) {
		ctrl.update(kLevel, euler_sp, kZero3, kZero3, 0.1f);
	}

	EXPECT_NEAR(ctrl.getIntegral()(0), 0.001f, 1e-7f);
}

TEST(FoldrotorAttitudeRateControlTest, ResetIntegralClearsAccumulatedState)
{
	auto ctrl = makeMatchedGainAttitudeController();

	ctrl.update(kLevel, matrix::Eulerf(0.2f, 0.1f, 0.f), kZero3, kZero3, 0.1f);
	ASSERT_GT(fabsf(ctrl.getIntegral()(0)), 0.f);
	ASSERT_GT(fabsf(ctrl.getIntegral()(1)), 0.f);

	ctrl.resetIntegral();

	EXPECT_NEAR(ctrl.getIntegral()(0), 0.0f, 1e-9f);
	EXPECT_NEAR(ctrl.getIntegral()(1), 0.0f, 1e-9f);
	EXPECT_NEAR(ctrl.getIntegral()(2), 0.0f, 1e-9f);

	// A subsequent settled call returns exactly zero moment.
	const matrix::Vector3f M = ctrl.update(kLevel, kLevel, kZero3, kZero3, 0.1f);
	EXPECT_NEAR(M(0), 0.0f, 1e-6f);
	EXPECT_NEAR(M(1), 0.0f, 1e-6f);
	EXPECT_NEAR(M(2), 0.0f, 1e-6f);
}

// Default limits are +/-infinity, so nothing clamps and no anti-windup
// engages. This is the runtime state until step 4d supplies real bounds,
// asserted so the fact stays visible.
TEST(FoldrotorAttitudeRateControlTest, DefaultLimitsNeitherClampNorFreeze)
{
	auto ctrl = makeMatchedGainAttitudeController();

	const matrix::Eulerf euler_sp(0.2f, 0.f, 0.f);

	for (int i = 0; i < 3; i++) {
		ctrl.update(kLevel, euler_sp, kZero3, kZero3, 0.1f);
	}

	// Three unclamped increments of 0.0059556821.
	EXPECT_NEAR(ctrl.getIntegral()(0), 0.017867046f, 1e-6f);
}

// Conditional integration, now driven by MEASURED saturation flags
// (2026-09-23) rather than by comparing the moment against a predicted
// limit. Roll flagged positively saturated while e_r = 0.6 is still
// positive -> min(0.6, 0) = 0 -> the integrator must not move, for any
// number of steps. Contrast with DefaultLimitsNeitherClampNorFreeze,
// which runs the identical sequence with no flags set and reaches
// 0.017867046.
//
// Note the output is NOT clamped to anything: the rate loop returns its
// raw PID moment, 2.1 N*m, exactly as rate_control.cpp does. The flag
// governs the integrator only.
TEST(FoldrotorAttitudeRateControlTest, IntegratorFreezesWhileSaturated)
{
	auto ctrl = makeMatchedGainAttitudeController();
	ctrl.setSaturationStatus(matrix::Vector3<bool>(true, true, true),
				 matrix::Vector3<bool>(false, false, false));

	const matrix::Eulerf euler_sp(0.2f, 0.f, 0.f);

	for (int i = 0; i < 5; i++) {
		EXPECT_NEAR(ctrl.update(kLevel, euler_sp, kZero3, kZero3, 0.1f)(0), 2.1f, 1e-6f);
	}

	EXPECT_NEAR(ctrl.getIntegral()(0), 0.0f, 1e-9f);
}

// The freeze must be directional: an error driving the axis back OUT of
// saturation still integrates, otherwise the integrator latches. Flagging
// only the POSITIVE direction isolates this, the same way asymmetric
// limits used to.
//
//   step 1, euler_sp = +0.2: e_r = +0.6, positively saturated ->
//           min(0.6, 0) = 0, frozen.
//   step 2, euler_sp = -0.2: e_r = -0.6, NOT negatively saturated ->
//           integrates by i_factor * 0.1 * (-0.6) * 0.1 = -0.0059556821.
//
// Both steps return the raw PID moment (+/-2.1): there is no clamp.
TEST(FoldrotorAttitudeRateControlTest, IntegratorStillMovesOutOfSaturation)
{
	auto ctrl = makeMatchedGainAttitudeController();
	ctrl.setSaturationStatus(matrix::Vector3<bool>(true, true, true),
				 matrix::Vector3<bool>(false, false, false));

	EXPECT_NEAR(ctrl.update(kLevel, matrix::Eulerf(0.2f, 0.f, 0.f), kZero3, kZero3, 0.1f)(0),
		    2.1f, 1e-6f);
	EXPECT_NEAR(ctrl.getIntegral()(0), 0.0f, 1e-9f);

	EXPECT_NEAR(ctrl.update(kLevel, matrix::Eulerf(-0.2f, 0.f, 0.f), kZero3, kZero3, 0.1f)(0),
		    -2.1f, 1e-6f);
	EXPECT_NEAR(ctrl.getIntegral()(0), -0.005955682f, 1e-6f);
}

// Yaw error wrapping to [-pi, pi]. Current psi = 3.0, setpoint
// psi = -3.0: the raw difference is -6.0 rad, but the short way round is
// -6.0 + 2*pi = +0.28318531.
//   rate_sp_z = 3 * 0.28318531 = 0.84955592
//   Mz        = 2.5 * 0.84955592 = 2.12388980
// Unwrapped this would be 2.5 * 3 * (-6.0) = -45.0 — a large command in
// the WRONG direction, which is exactly what the wrap exists to prevent.
// The sign difference alone makes this test unambiguous.
TEST(FoldrotorAttitudeRateControlTest, YawErrorWrapsAcrossPi)
{
	auto ctrl = makeMatchedGainAttitudeController();

	const matrix::Vector3f M = ctrl.update(matrix::Eulerf(0.f, 0.f, 3.0f),
					       matrix::Eulerf(0.f, 0.f, -3.0f),
					       kZero3, kZero3, 0.1f);

	EXPECT_NEAR(M(2), 2.123889804f, 1e-5f);
	EXPECT_GT(M(2), 0.f);
}

// Wrapping is applied on all three axes, not just yaw (decision 3).
// Same construction on roll: phi = 3.0 -> phi_sp = -3.0 wraps to
// +0.28318531, giving rate_sp_x = 0.84955592 and
// Mx = 3.5 * 0.84955592 = 2.97344572, positive rather than -63.0.
TEST(FoldrotorAttitudeRateControlTest, RollErrorWrapsAcrossPiToo)
{
	auto ctrl = makeMatchedGainAttitudeController();

	const matrix::Vector3f M = ctrl.update(matrix::Eulerf(3.0f, 0.f, 0.f),
					       matrix::Eulerf(-3.0f, 0.f, 0.f),
					       kZero3, kZero3, 0.1f);

	EXPECT_NEAR(M(0), 2.973445725f, 1e-5f);
	EXPECT_GT(M(0), 0.f);
}

// Axis mapping for the handoff to allocation.md's moment input: an error
// on one attitude axis must produce a moment on that axis alone, in the
// (Mx_b, My_b, Mz_b) order the allocator expects. Allocation itself does
// not exist yet (step 4d), so this pins the shape and ordering of the
// contract rather than the allocator's behaviour — the same role
// OutputComposesWithInertialToBody plays for the force path.
TEST(FoldrotorAttitudeRateControlTest, OutputIsPerAxisBodyMomentInAllocationOrder)
{
	auto ctrl = makeMatchedGainAttitudeController();

	const matrix::Vector3f M_roll = ctrl.update(kLevel, matrix::Eulerf(0.1f, 0.f, 0.f),
					kZero3, kZero3, 0.1f);
	EXPECT_GT(M_roll(0), 0.f);
	EXPECT_NEAR(M_roll(1), 0.f, 1e-6f);
	EXPECT_NEAR(M_roll(2), 0.f, 1e-6f);

	ctrl.resetIntegral();
	const matrix::Vector3f M_pitch = ctrl.update(kLevel, matrix::Eulerf(0.f, 0.1f, 0.f),
					 kZero3, kZero3, 0.1f);
	EXPECT_NEAR(M_pitch(0), 0.f, 1e-6f);
	EXPECT_GT(M_pitch(1), 0.f);
	EXPECT_NEAR(M_pitch(2), 0.f, 1e-6f);

	ctrl.resetIntegral();
	const matrix::Vector3f M_yaw = ctrl.update(kLevel, matrix::Eulerf(0.f, 0.f, 0.1f),
				       kZero3, kZero3, 0.1f);
	EXPECT_NEAR(M_yaw(0), 0.f, 1e-6f);
	EXPECT_NEAR(M_yaw(1), 0.f, 1e-6f);
	EXPECT_GT(M_yaw(2), 0.f);
}

// ---------------------------------------------------------------------
// CascadeRateGate (step 4e part 1)
// ---------------------------------------------------------------------

TEST(FoldrotorCascadeRateGateTest, FirstCallAlwaysFires)
{
	foldrotor::CascadeRateGate gate(20000); // 50 Hz
	float dt = -1.f;
	EXPECT_TRUE(gate.due(1000000, &dt));
	// dt CONTRACT revised 2026-09-11 (CascadeRateGate.hpp): the first fire
	// reports the nominal period, not 0 -- a zero dt was reaching the PID
	// D terms as a division by zero. period_us = 20000 -> 0.02 s.
	EXPECT_NEAR(dt, 0.02f, 1e-9f);
}

// Driven at 1000 Hz (1000 us/tick) with a 50 Hz gate (period 20000 us):
// due() should fire once every 20 ticks, i.e. 50 times over 1000 ticks.
TEST(FoldrotorCascadeRateGateTest, FiresAtExpectedCadence50Hz)
{
	foldrotor::CascadeRateGate gate(20000);
	int fires = 0;

	for (uint64_t now = 1000; now <= 1000000; now += 1000) {
		if (gate.due(now)) {
			fires++;
		}
	}

	EXPECT_EQ(fires, 50);
}

// Same drive, 250 Hz gate (period 4000 us) -> 250 fires over 1000 ticks.
TEST(FoldrotorCascadeRateGateTest, FiresAtExpectedCadence250Hz)
{
	foldrotor::CascadeRateGate gate(4000);
	int fires = 0;

	for (uint64_t now = 1000; now <= 1000000; now += 1000) {
		if (gate.due(now)) {
			fires++;
		}
	}

	EXPECT_EQ(fires, 250);
}

// A stalled driving topic (a big gap in `now`) must fire exactly once for
// that overdue call, not multiple times to "catch up" -- due() only ever
// evaluates the current call.
TEST(FoldrotorCascadeRateGateTest, StalledIntervalFiresOnceNotRepeatedly)
{
	foldrotor::CascadeRateGate gate(20000); // 50 Hz
	ASSERT_TRUE(gate.due(0));

	// A 200 ms gap -- 10x the nominal period.
	float dt = 0.f;
	EXPECT_TRUE(gate.due(200000, &dt));
	EXPECT_NEAR(dt, 0.2f, 1e-6f);

	// Immediately after, the gate is not due again until another full
	// period has elapsed -- it did not bank the extra time.
	EXPECT_FALSE(gate.due(200000 + 1000));
}

TEST(FoldrotorCascadeRateGateTest, NotDueBeforePeriodElapses)
{
	foldrotor::CascadeRateGate gate(20000);
	ASSERT_TRUE(gate.due(0));
	EXPECT_FALSE(gate.due(10000));
	EXPECT_TRUE(gate.due(20000));
}

// ---------------------------------------------------------------------
// AttitudeRateControl updateAttitude()/updateRate() split (step 4e part 1)
// ---------------------------------------------------------------------

// _rate_sp must survive repeated updateRate() calls with no intervening
// updateAttitude() -- exactly what running the attitude stage at 250 Hz
// and the rate stage at 1000 Hz depends on.
TEST(FoldrotorAttitudeRateControlSplitTest, RateSetpointHoldsAcrossUpdateRateCallsWithoutAttitudeUpdate)
{
	auto ctrl = makeMatchedGainAttitudeController();

	const matrix::Vector3f rate_sp = ctrl.updateAttitude(kLevel, matrix::Eulerf(0.2f, -0.1f, 0.f));
	EXPECT_NEAR(rate_sp(0), 0.6f, 1e-5f);
	EXPECT_NEAR(rate_sp(1), -0.3f, 1e-5f);

	// Three updateRate() calls, no updateAttitude() in between: the rate
	// setpoint must be identical across all three, and must be exactly
	// what updateAttitude() just computed.
	for (int i = 0; i < 3; i++) {
		ctrl.updateRate(kZero3, kZero3, 0.001f);
		EXPECT_NEAR(ctrl.getRateSetpoint()(0), 0.6f, 1e-5f);
		EXPECT_NEAR(ctrl.getRateSetpoint()(1), -0.3f, 1e-5f);
	}
}

// update() must still reproduce the exact step 4c hand-computed nominal
// roll/pitch case (see NominalRollPitchMatchesHandComputedCascade above)
// now that it is implemented as updateAttitude() + updateRate() rather
// than one inline body -- the split changed structure, not behaviour.
TEST(FoldrotorAttitudeRateControlSplitTest, UpdateStillMatchesPreSplitHandComputedCascade)
{
	auto ctrl = makeMatchedGainAttitudeController();

	const matrix::Vector3f M = ctrl.update(kLevel,
					       matrix::Eulerf(0.2f, -0.1f, 0.f),
					       matrix::Vector3f(0.05f, 0.02f, 0.f),
					       kZero3, 0.01f);

	EXPECT_NEAR(M(0), 1.925f, 1e-5f);
	EXPECT_NEAR(M(1), -1.12f, 1e-5f);
	EXPECT_NEAR(M(2), 0.0f, 1e-5f);
}

// Calling updateAttitude() then updateRate() separately, by hand, must
// match calling update() with the same inputs in one shot -- the two
// entry points are the same math, just split into two calls.
TEST(FoldrotorAttitudeRateControlSplitTest, SplitCallsMatchCombinedUpdate)
{
	auto ctrl_split = makeMatchedGainAttitudeController();
	auto ctrl_combined = makeMatchedGainAttitudeController();

	const matrix::Eulerf euler_sp(0.2f, -0.1f, 0.05f);
	const matrix::Vector3f rate(0.05f, 0.02f, -0.01f);

	ctrl_split.updateAttitude(kLevel, euler_sp);
	const matrix::Vector3f M_split = ctrl_split.updateRate(rate, kZero3, 0.01f);

	const matrix::Vector3f M_combined = ctrl_combined.update(kLevel, euler_sp, rate, kZero3, 0.01f);

	EXPECT_NEAR(M_split(0), M_combined(0), 1e-6f);
	EXPECT_NEAR(M_split(1), M_combined(1), 1e-6f);
	EXPECT_NEAR(M_split(2), M_combined(2), 1e-6f);
}

// ---------------------------------------------------------------------
// FoldrotorAllocation (step 4d, .claude/specs/allocation.md)
// ---------------------------------------------------------------------
//
// Closes allocation.md's two named-missing tests (Minv*M0 ~= I,
// round-trip) plus saturation, the max_tilt regression guard, fold
// pinned to 0, the degenerate zero-wrench guard, and a finiteness sweep.
// Pure math -- no Gazebo, no uORB. Every expected value below was
// computed by hand from M0's definition (allocation.md "The matrices")
// with a standalone numpy check, not read back off this implementation.

// allocation.md's Minv literal, recomputed 2026-09-21 for the
// sign-corrected, CoM-referenced geometry (s1y = -0.267356, s2y =
// +0.269404, s1z/s2z = -0.054924/-0.054926 m, k = -0.022274 -- see
// FoldrotorAllocation.hpp OPEN ITEM (d), was +-0.2684 m / +0.0301 m /
// +0.017). Standalone numpy inverse of the corrected M0, same as the
// original literal's provenance; allocation.md's "The matrices" section
// carries the same value, marked as superseding the old one.
static const float kSpecMinv[6][6] = {
	{ +0.5014746190f, -0.0026120310f, +0.0001223900f, -0.1535630527f, +0.0000034470f, +1.8502851798f },
	{ +0.3818241330f, +0.4999999410f, +0.0000000030f, -0.0000034470f, -22.4476968660f, +0.0000415350f },
	{ -0.0001230960f, -0.0314724260f, +0.5014746770f, -1.8502851798f, +0.0000415350f, -0.1535630527f },
	{ +0.4985253810f, +0.0026120310f, -0.0001223900f, +0.1535630527f, -0.0000034470f, -1.8502851798f },
	{ -0.3818241330f, +0.5000000590f, -0.0000000030f, +0.0000034470f, +22.4476968660f, -0.0000415350f },
	{ +0.0001230960f, +0.0314724260f, +0.4985253230f, +1.8502851798f, -0.0000415350f, +0.1535630527f },
};

TEST(FoldrotorAllocationTest, MinvTimesM0IsIdentity)
{
	foldrotor::FoldrotorAllocation alloc;
	ASSERT_TRUE(alloc.isValid());

	const matrix::SquareMatrix<float, 6> product = alloc.getMinv() * alloc.getM0();

	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 6; j++) {
			const float expected = (i == j) ? 1.f : 0.f;
			EXPECT_NEAR(product(i, j), expected, 1e-6f) << "i=" << i << " j=" << j;
		}
	}
}

// Note: as of 2026-09-09 both the runtime-derived Minv and the spec's
// transcribed literal describe the SDF-verified geometry (the mismatch
// this comment used to warn about is resolved -- FoldrotorAllocation.hpp
// OPEN ITEM (b)). This test still only keeps the code and the spec
// honest with each other; it does not independently verify the geometry
// against the SDF (that's allocation.md's still-unwritten
// "Geometry-vs-SDF test").
TEST(FoldrotorAllocationTest, DerivedInverseMatchesSpecLiteral)
{
	foldrotor::FoldrotorAllocation alloc;
	ASSERT_TRUE(alloc.isValid());

	const matrix::SquareMatrix<float, 6> &Minv = alloc.getMinv();

	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 6; j++) {
			// 3e-6f, not 1e-6f: the two largest-magnitude entries
			// (+-29.41...) lose float32 precision at the 1e-6 digit
			// through the LU-based matrix::inv() path; a relative
			// tolerance would be more principled but this repo's
			// established style (this file, throughout) uses a fixed
			// absolute EXPECT_NEAR, so widen it just enough to absorb
			// that rather than switch styles for one test.
			EXPECT_NEAR(Minv(i, j), kSpecMinv[i][j], 3e-6f) << "i=" << i << " j=" << j;
		}
	}
}

// ---------------------------------------------------------------------
// allocation.md's long-planned "Geometry-vs-SDF test", finally written
// 2026-09-21 (FoldrotorAllocation.hpp OPEN ITEM (d)). It was deliberately
// left unwritten while the geometry mismatch was a carried deferral,
// since it would have been red on arrival by design; it is written now
// because the constants it pins were just corrected.
//
// Provenance of the expected values: a forward-kinematics walk of
// Tools/simulation/gz/models/foldrotor3/model.sdf -- resolve each
// PropNLink's pose through its joint chain at zero fold/tilt, take the
// mass-weighted CoM of every link, and subtract. Done by
// sitl_testing/allocation_study/sdf_fk.py, whose output is cross-checked
// against the independent bench force/torque measurements recorded in
// force_moment_test.md (all ten cases, signs included). These are body
// FLU, the frame allocate() is fed.
//
// This is the test that would have caught the 2026-09-21 sign inversion:
// the pre-fix constants had rotor 1 on +Y at z=+0.0301 (relative to the
// base_link origin), where the SDF puts it on -Y at z=-0.0549 (relative
// to the true CoM).
TEST(FoldrotorAllocationTest, GeometryConstantsMatchSdfForwardKinematics)
{
	foldrotor::FoldrotorAllocation alloc;
	const matrix::SquareMatrix<float, 6> &M0 = alloc.getM0();

	// Rotor positions relative to the true CoM, body FLU, from the SDF.
	// 2026-09-22: re-derived after the ballast mast moved the CoM from
	// z=+0.0248 to z=-0.0471. s1z/s2z CHANGED SIGN -- the rotors now sit
	// ABOVE the CoM. Independently reproduced by sdf_fk.py.
	const float s1y = -0.267583f, s1z = +0.017010f;
	const float s2y = 0.269177f,  s2z = +0.017009f;
	// Drag/thrust ratio: model.sdf's momentConstant (0.022274), signed
	// negative because rotor 1 (Prop1) is the ccw rotor and so reacts
	// along -T. Bench cross-check: motor 1 alone gave Mz = +0.2240 N*m
	// in FRD against 10.066 N of thrust -> 0.02225, same sign.
	const float k = -0.022274f;

	// M0's moment rows must equal r x T + drag, built here with explicit
	// cross products rather than by re-transcribing the same rows the
	// implementation uses -- otherwise this test could only ever confirm
	// that a transcription matches itself.
	const matrix::Vector3f r1(0.f, s1y, s1z);
	const matrix::Vector3f r2(0.f, s2y, s2z);

	for (int axis = 0; axis < 3; axis++) {
		matrix::Vector3f e1{}, e2{};
		e1(axis) = 1.f;
		e2(axis) = 1.f;

		// Column `axis` is rotor 1's response to a unit thrust along
		// that body axis; column `axis + 3` is rotor 2's.
		const matrix::Vector3f m1 = r1.cross(e1) + e1 * k;
		const matrix::Vector3f m2 = r2.cross(e2) - e2 * k;

		for (int row = 0; row < 3; row++) {
			EXPECT_NEAR(M0(3 + row, axis), m1(row), 1e-6f) << "rotor 1, moment row " << row << ", axis " << axis;
			EXPECT_NEAR(M0(3 + row, 3 + axis), m2(row), 1e-6f) << "rotor 2, moment row " << row << ", axis " << axis;
		}
	}
}

// Force/moment DIRECTION test -- the tier allocation.md names between
// open-loop actuator tests and closed-loop validation, and the check
// that the 2026-09-21 inversion (OPEN ITEM (d)) slipped past for weeks
// because every other allocation test compared the allocator against
// its own M0.
//
// Demand a small moment on one axis at hover, run the real allocator,
// then evaluate what the resulting rotor commands PHYSICALLY produce --
// using SDF geometry and explicit cross products, never M0. A sign
// error anywhere in the constants, the row transcription, or the
// atan2 inversion shows up here as a negative ratio, which in closed
// loop is positive feedback.
//
// Demands are kept small (<= 0.05 N*m) so nothing clamps; the
// clamp-without-redistribution path is covered by the tilt/thrust
// saturation tests above.
TEST(FoldrotorAllocationTest, CommandedMomentProducesSameSignPhysicalMoment)
{
	foldrotor::FoldrotorAllocation alloc;

	// Same SDF-derived geometry as the test above.
	const matrix::Vector3f r1(0.f, -0.267356f, -0.054924f);
	const matrix::Vector3f r2(0.f, 0.269404f, -0.054926f);
	const float k_drag = 0.022274f;
	const float hover_fz = 15.260017f;

	// Physical moment of one allocator output, about the true CoM.
	auto physical_moment = [&](const foldrotor::FoldrotorAllocation::Output & o) {
		auto thrust = [](float F, float alpha, float beta) {
			return matrix::Vector3f(F * sinf(beta),
						-F * cosf(beta) * sinf(alpha),
						F * cosf(beta) * cosf(alpha));
		};
		const matrix::Vector3f T1 = thrust(o.F1, o.alpha1, o.beta1);
		const matrix::Vector3f T2 = thrust(o.F2, o.alpha2, o.beta2);
		// Rotor 1 is ccw and reacts along -T; rotor 2 is cw, +T.
		return r1.cross(T1) - T1 * k_drag + r2.cross(T2) + T2 * k_drag;
	};

	const matrix::Vector3f F_hover(0.f, 0.f, hover_fz);
	const matrix::Vector3f trim = physical_moment(alloc.allocate(F_hover, matrix::Vector3f{}));

	for (int axis = 0; axis < 3; axis++) {
		for (float magnitude : {0.05f, -0.05f}) {
			matrix::Vector3f M_cmd{};
			M_cmd(axis) = magnitude;

			const auto out = alloc.allocate(F_hover, M_cmd);
			ASSERT_FALSE(out.saturated) << "axis " << axis << " magnitude " << magnitude;

			const matrix::Vector3f delivered = physical_moment(out) - trim;
			const float ratio = delivered(axis) / magnitude;

			EXPECT_GT(ratio, 0.f) << "axis " << axis << " magnitude " << magnitude
					      << ": allocator produced the OPPOSITE moment (ratio " << ratio << ")";
			EXPECT_NEAR(ratio, 1.f, 0.05f) << "axis " << axis << " magnitude " << magnitude;
		}
	}
}

// Largest single-axis moment the REAL allocator satisfies without
// clamping, holding the vehicle's weight on Fz and everything else at
// zero. Bisected rather than hardcoded so a geometry change moves the
// bound instead of silently invalidating a literal.
//
// This replaces FoldrotorControl::momentEnvelopeAtThrust(), deleted
// 2026-09-23 along with the rate loop's output clamp. Two tests below
// used the table as a stand-in for "what the airframe can do"; they now
// ask the allocator directly, which is what the table was approximating.
static float hoverMomentAuthority(const foldrotor::FoldrotorAllocation &alloc, int axis)
{
	const matrix::Vector3f F_hover(0.f, 0.f, 19.615f);
	float lo = 0.f, hi = 20.f;

	for (int i = 0; i < 60; i++) {
		const float mid = 0.5f * (lo + hi);
		matrix::Vector3f M_cmd{};
		M_cmd(axis) = mid;

		if (alloc.allocate(F_hover, M_cmd).saturated) { hi = mid; }

		else { lo = mid; }
	}

	return lo;
}

// deliveredWrench() is the measurement the whole 2026-09-23 change rests
// on, so it is pinned in both directions.
//
// Direction 1: it must be the exact inverse of allocate() wherever
// allocate() did not clamp. If the forward map and allocateRotor()'s
// inverse ever drift apart, every saturation flag downstream becomes
// fiction, and it would drift SILENTLY -- nothing else in the module
// compares the two.
// kBodyForceXYLimit is what decides the tilt at which this vehicle stops
// being able to hold altitude, because the body-frame horizontal force at
// tilt theta is W*sin(theta) -- rotated collective, not commanded
// translation. Raised 1.0 -> 4.0 N on 2026-09-23.
//
// The criterion is controller.md's: the commanded wrench must be
// deliverable. This pins both halves -- the tilt the cap buys, and that
// the allocator can actually produce that force without clamping.
TEST(FoldrotorAllocationTest, BodyForceCapSetsTheTiltAtWhichAltitudeIsLost)
{
	foldrotor::FoldrotorAllocation alloc;

	param_t h = param_find("FR_VEL_Z_GRAV_FF");
	ASSERT_NE(h, PARAM_INVALID);
	float weight = NAN;
	param_get(h, &weight);

	const float cap = FoldrotorControl::kBodyForceXYLimit;

	// The decision: 4 N.
	EXPECT_FLOAT_EQ(cap, 4.0f);

	// theta_max = asin(cap / W) -- 11.8 deg at the post-mast weight, up
	// from 2.9 deg at the old 1.0 N.
	const float theta_max = asinf(math::constrain(cap / weight, -1.f, 1.f));
	EXPECT_NEAR(math::degrees(theta_max), 11.78f, 0.05f);

	// The 10 deg test this was raised for must fit, with margin.
	const float need_10deg = weight * sinf(math::radians(10.f));
	EXPECT_LT(need_10deg, cap)
			<< "10 deg needs " << need_10deg << " N, cap is " << cap << " N";

	// And the allocator must deliver it while holding hover lift, in
	// EVERY horizontal direction -- the envelope is not isotropic in XY,
	// so a direction-blind check passes on the easy bearings.
	for (int k = 0; k < 16; k++) {
		const float th = float(k) * (2.f * M_PI_F / 16.f);
		const auto out = alloc.allocate(matrix::Vector3f(cap * cosf(th), cap * sinf(th), weight),
						matrix::Vector3f());
		EXPECT_FALSE(out.saturated)
				<< "cap " << cap << " N not deliverable at hover lift, bearing "
				<< math::degrees(th) << " deg";
	}
}

// The 2026-09-23 unpin, and specifically its failure mode: a stale
// attitude setpoint must not leave the vehicle holding a tilt.
//
// uORB::Subscription::copy() has no "nothing new" reading -- it returns
// the last sample forever -- and mavlink_receiver.cpp:1844 publishes
// vehicle_attitude_setpoint only while OFFBOARD and only when a
// SET_ATTITUDE_TARGET arrives. Every other setpoint this module consumes
// has a continuously-running producer, so this guard has no precedent
// inside the module and is easy to drop by accident.
TEST(FoldrotorControlAttitudeSetpointTest, StaleOrUnusableSetpointFallsBackToLevel)
{
	const hrt_abstime now = 10_s;

	trajectory_setpoint_s traj{};
	traj.yaw = 0.3f;

	// A live, valid 10 deg pitch command, for contrast.
	vehicle_attitude_setpoint_s live{};
	live.timestamp = now;
	matrix::Quatf(matrix::Eulerf(0.f, math::radians(10.f), 0.f)).copyTo(live.q_d);

	EXPECT_NEAR(FoldrotorControl::resolveEulerSetpoint(live, traj, 1.0f, now).theta(),
		    math::radians(10.f), 1e-5f);

	// Stale by more than the timeout -> level, yaw from the trajectory.
	vehicle_attitude_setpoint_s stale = live;
	stale.timestamp = now - FoldrotorControl::kAttitudeSetpointTimeout - 1;
	const matrix::Eulerf from_stale = FoldrotorControl::resolveEulerSetpoint(stale, traj, 1.0f, now);
	EXPECT_NEAR(from_stale.phi(), 0.f, 1e-6f);
	EXPECT_NEAR(from_stale.theta(), 0.f, 1e-6f) << "a stale setpoint must not hold a tilt";
	EXPECT_NEAR(from_stale.psi(), 0.3f, 1e-6f);

	// Never published at all (timestamp 0) -> level.
	vehicle_attitude_setpoint_s never{};
	EXPECT_NEAR(FoldrotorControl::resolveEulerSetpoint(never, traj, 1.0f, now).theta(), 0.f, 1e-6f);

	// Fresh but unusable: NaN, and all-zero (norm 0) -> level.
	vehicle_attitude_setpoint_s nan_sp = live;
	nan_sp.q_d[2] = NAN;
	EXPECT_NEAR(FoldrotorControl::resolveEulerSetpoint(nan_sp, traj, 1.0f, now).theta(), 0.f, 1e-6f);

	vehicle_attitude_setpoint_s zero_sp{};
	zero_sp.timestamp = now;
	EXPECT_NEAR(FoldrotorControl::resolveEulerSetpoint(zero_sp, traj, 1.0f, now).theta(), 0.f, 1e-6f);

	// Fallback yaw contract: NaN in TrajectorySetpoint means "do not
	// control yaw", so the current heading is held rather than commanded
	// to a NaN-derived value.
	trajectory_setpoint_s traj_nan{};
	traj_nan.yaw = NAN;
	EXPECT_NEAR(FoldrotorControl::resolveEulerSetpoint(stale, traj_nan, 1.0f, now).psi(), 1.0f, 1e-6f);
}

// Precedence: a live attitude setpoint is authoritative for ALL THREE
// angles, yaw included. This is a decision, not a derivation -- splitting
// the source (tilt here, yaw from TrajectorySetpoint) would make the
// commanded attitude depend on which of two unsynchronised publishers
// spoke last. Pinned so the choice is visible rather than incidental.
TEST(FoldrotorControlAttitudeSetpointTest, LiveSetpointOverridesTrajectoryYaw)
{
	const hrt_abstime now = 10_s;

	trajectory_setpoint_s traj{};
	traj.yaw = 1.5f;              // would be honoured if yaw were split off

	vehicle_attitude_setpoint_s att{};
	att.timestamp = now;
	matrix::Quatf(matrix::Eulerf(math::radians(10.f), 0.f, 0.f)).copyTo(att.q_d);

	const matrix::Eulerf got = FoldrotorControl::resolveEulerSetpoint(att, traj, 0.9f, now);

	EXPECT_NEAR(got.phi(), math::radians(10.f), 1e-5f);
	EXPECT_NEAR(got.theta(), 0.f, 1e-5f);
	EXPECT_NEAR(got.psi(), 0.f, 1e-5f)
			<< "live attitude setpoint must own yaw too, not TrajectorySetpoint's 1.5";
}

// Not a test: a generator for the CoM-referenced moment analysis
// (findings.md (21), .claude/specs/force_moment_test.md). DISABLED_ so it
// never runs in the suite. It pipes a table of body-FLU wrenches through
// the REAL FoldrotorAllocation -- the analysis must not re-implement the
// allocator (.claude/CLAUDE.md rule 7) -- and writes the per-rotor
// commands for foldrotor3_tests/com_moment_analysis.py to forward-map
// through the SDF geometry. Run with FR_ALLOC_IN=cases.csv and
// FR_ALLOC_OUT=alloc.csv in the environment, passing
// --gtest_also_run_disabled_tests --gtest_filter='*DISABLED_DumpAllocation*'
// to functional-FoldrotorControl. com_moment_analysis.py does this for you.
TEST(FoldrotorAllocationTool, DISABLED_DumpAllocation)
{
	const char *in_path = getenv("FR_ALLOC_IN");
	const char *out_path = getenv("FR_ALLOC_OUT");
	ASSERT_NE(in_path, nullptr) << "set FR_ALLOC_IN";
	ASSERT_NE(out_path, nullptr) << "set FR_ALLOC_OUT";

	FILE *in = fopen(in_path, "r");
	FILE *out = fopen(out_path, "w");
	ASSERT_NE(in, nullptr);
	ASSERT_NE(out, nullptr);

	fprintf(out, "fx,fy,fz,mx,my,mz,F1,F2,alpha1,alpha2,beta1,beta2,saturated\n");

	char line[512];
	foldrotor::FoldrotorAllocation alloc;

	while (fgets(line, sizeof(line), in)) {
		float w[6];

		if (sscanf(line, "%f,%f,%f,%f,%f,%f", &w[0], &w[1], &w[2], &w[3], &w[4], &w[5]) != 6) {
			continue;       // header or blank line
		}

		const auto o = alloc.allocate(matrix::Vector3f(w[0], w[1], w[2]),
					      matrix::Vector3f(w[3], w[4], w[5]));
		fprintf(out, "%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%d\n",
			(double)w[0], (double)w[1], (double)w[2], (double)w[3], (double)w[4], (double)w[5],
			(double)o.F1, (double)o.F2, (double)o.alpha1, (double)o.alpha2,
			(double)o.beta1, (double)o.beta2, o.saturated ? 1 : 0);
	}

	fclose(in);
	fclose(out);
}

TEST(FoldrotorAllocationTest, DeliveredWrenchRoundTripsAFeasibleWrench)
{
	foldrotor::FoldrotorAllocation alloc;

	for (float fz = 6.f; fz <= 26.f; fz += 2.f) {
		for (int k = 0; k < 8; k++) {
			const float theta = float(k) * (2.f * M_PI_F / 8.f);
			const matrix::Vector3f F(0.4f * cosf(theta), 0.4f * sinf(theta), fz);
			const matrix::Vector3f M(0.3f * cosf(theta), 0.05f * sinf(theta), 0.3f * cosf(theta));

			const auto out = alloc.allocate(F, M);
			ASSERT_FALSE(out.saturated) << "test wrench must be feasible: fz " << fz;

			matrix::Vector3f F_got;
			matrix::Vector3f M_got;
			alloc.deliveredWrench(out, F_got, M_got);

			for (int a = 0; a < 3; a++) {
				EXPECT_NEAR(F_got(a), F(a), 1e-3f) << "force axis " << a << " at fz " << fz;
				EXPECT_NEAR(M_got(a), M(a), 1e-3f) << "moment axis " << a << " at fz " << fz;
			}
		}
	}
}

// Direction 2: when allocate() DOES clamp, the residual must be non-zero
// and must point along the axis that was over-asked.
//
// This is the property `saturated` cannot provide and the reason the flag
// is not used for anti-windup: it is one bit for six axes. findings.md
// 2026-09-22 (12) measured it reading 0.0% while beta1 sat pinned to its
// rail for an entire departure.
TEST(FoldrotorAllocationTest, DeliveredWrenchQuantifiesWhatSaturationOnlyFlags)
{
	foldrotor::FoldrotorAllocation alloc;

	const matrix::Vector3f F_hover(0.f, 0.f, 19.615f);
	const int pitch = 1;

	// Ask for ten times the measured pitch authority.
	const float authority = hoverMomentAuthority(alloc, pitch);
	ASSERT_GT(authority, 0.01f);

	matrix::Vector3f M_cmd{};
	M_cmd(pitch) = 10.f * authority;

	const auto out = alloc.allocate(F_hover, M_cmd);
	ASSERT_TRUE(out.saturated);

	matrix::Vector3f F_got;
	matrix::Vector3f M_got;
	alloc.deliveredWrench(out, F_got, M_got);

	const matrix::Vector3f resid = M_cmd - M_got;

	// The shortfall is real, is on the pitch axis, and carries the sign of
	// the over-ask -- the three things the anti-windup needs and the flag
	// cannot give.
	EXPECT_GT(resid(pitch), 0.1f * M_cmd(pitch))
			<< "residual must expose the undelivered pitch moment";
	EXPECT_LT(M_got(pitch), M_cmd(pitch));
	EXPECT_GT(M_got(pitch), 0.f) << "delivered pitch must not have flipped sign";
}

// Regression guard for the horizontal-force cap (kPosVelForceXYLimit,
// findings.md 2026-09-21 (5)). The sphere saturation alone permitted
// ~22 N of horizontal force at hover thrust; the allocator cannot deliver
// anything like that, because horizontal force comes from tilting alpha/
// beta and both are railed at +-kMaxTilt. This pins the actual ceiling so
// a future re-tune of the sphere cannot silently reopen the gap.
TEST(FoldrotorAllocationTest, HorizontalForceCapIsWithinTiltAuthority)
{
	foldrotor::FoldrotorAllocation alloc;

	// With NO moment reserved the allocator can manage a few newtons
	// sideways; the cap must sit below that, since the rate loop needs
	// the remaining tilt range for moments.
	float lo = 0.f, hi = 30.f;

	for (int i = 0; i < 60; i++) {
		const float mid = 0.5f * (lo + hi);
		bool ok = true;

		for (int k = 0; k < 16; k++) {
			const float theta = float(k) * (2.f * M_PI_F / 16.f);

			if (alloc.allocate(matrix::Vector3f(mid * cosf(theta), mid * sinf(theta), 17.36f),
					   matrix::Vector3f()).saturated) {
				ok = false;
				break;
			}
		}

		if (ok) { lo = mid; } else { hi = mid; }
	}

	EXPECT_LT(FoldrotorControl::kPosVelForceXYLimit, lo)
			<< "horizontal cap " << FoldrotorControl::kPosVelForceXYLimit
			<< " N exceeds the " << lo << " N the allocator can deliver with ZERO moment reserved";

	// And the sphere's own horizontal allowance at this thrust -- which is
	// what bounded the loop before this fix -- must be far above it, or
	// the cap is not the binding constraint and this fix does nothing.
	const float sphere_xy = sqrtf(28.f * 28.f - 17.36f * 17.36f);
	EXPECT_GT(sphere_xy, 4.f * FoldrotorControl::kPosVelForceXYLimit);
}

// Every loop's integrator clamp must sit BELOW the authority that loop
// actually has. This is the invariant that was violated for the whole
// 2026-09-1x/2026-09-21 flip investigation and never showed up as a test
// failure: FR_RATE_R_I_LIM was 3.8 N*m against a true roll authority of
// 1.47, and FR_VEL_XY_I_LIM was 15 N against a 1 N horizontal cap -- so
// each integrator could, on its own, wind up to many times the total the
// vehicle could ever deliver, and stay there. Anti-windup cannot save a
// loop whose clamp is above the rail.
//
// The margins here are deliberately loose (the point is the ORDERING, not
// a tuning value) but the assertions are strict inequalities, so a future
// re-tune that reintroduces the inversion fails immediately.
TEST(FoldrotorControlParamTest, IntegratorLimitsStayBelowTheirAuthority)
{
	// Measured, not predicted: momentEnvelopeAtThrust() was deleted
	// 2026-09-23 with the rest of the predicted-bound machinery. The
	// invariant is unchanged -- a clamp above the authority the loop
	// actually has cannot be saved by anti-windup.
	foldrotor::FoldrotorAllocation alloc;
	const matrix::Vector3f hover_env(hoverMomentAuthority(alloc, 0),
					 hoverMomentAuthority(alloc, 1),
					 hoverMomentAuthority(alloc, 2));

	auto default_of = [](const char *name) {
		param_t h = param_find(name);
		EXPECT_NE(h, PARAM_INVALID) << name;
		float v = NAN;
		param_get(h, &v);
		return v;
	};

	EXPECT_LT(default_of("FR_RATE_R_I_LIM"), hover_env(0))
			<< "roll rate integrator can wind past the deliverable roll moment";
	EXPECT_LT(default_of("FR_RATE_P_I_LIM"), hover_env(1))
			<< "pitch rate integrator can wind past the deliverable pitch moment";

	// The horizontal velocity integrator is bounded by the hard XY force
	// cap, not by a moment.
	EXPECT_LT(default_of("FR_VEL_XY_I_LIM"), FoldrotorControl::kPosVelForceXYLimit)
			<< "horizontal velocity integrator can wind past the horizontal force cap";

	// Vertical is the one axis with real headroom: the integrator trims
	// around FR_VEL_Z_GRAV_FF, so its clamp is compared against the slack
	// between hover weight and the ceiling above it, not against zero.
	//
	// That ceiling was the combined force sphere (28 N) until 2026-09-23,
	// when the sphere went to T/W = 2 and stopped being an authority bound
	// at all -- it now sits ABOVE what the rotors can deliver
	// (CombinedForceSphereIsTwoTimesWeightAndExceedsRotorCeiling). Comparing
	// against it would make this assertion vacuous in exactly the direction
	// this test exists to catch, so the deliverable ceiling is used instead:
	// two rotors at kMaxThrust.
	const float rotor_ceiling = 2.f * foldrotor::FoldrotorAllocation::kMaxThrust;
	EXPECT_LT(default_of("FR_VEL_Z_I_LIM"), rotor_ceiling - default_of("FR_VEL_Z_GRAV_FF"))
			<< "vertical velocity integrator can wind past the deliverable force ceiling";
}

// Regression guard for the 2026-09-23 T/W decision AND for what it gives
// up, which is the part worth a test.
//
// controller.md's position-loop contract is that the commanded wrench be
// DELIVERABLE. The combined force sphere is what enforced that inside the
// loop: at 28 N it sat under the airframe's own 30 N rotor ceiling, so the
// loop could not ask for force that does not exist. At T/W = 2 it sits
// above that ceiling and can no longer do so.
//
// This is pinned as an assertion rather than left in a comment for two
// reasons: so the suite states the cost out loud, and so that restoring
// the sphere to a bounding value is a deliberate reopening of the decision
// rather than a silent "fix" of a number that looks wrong.
TEST(FoldrotorControlParamTest, CombinedForceSphereIsTwoTimesWeightAndExceedsRotorCeiling)
{
	auto default_of = [](const char *name) {
		param_t h = param_find(name);
		EXPECT_NE(h, PARAM_INVALID) << name;
		float v = NAN;
		param_get(h, &v);
		return v;
	};

	const float weight = default_of("FR_VEL_Z_GRAV_FF");
	const float sphere = FoldrotorControl::kPosVelForceTW * weight;
	const float rotor_ceiling = 2.f * foldrotor::FoldrotorAllocation::kMaxThrust;

	// The decision itself.
	EXPECT_FLOAT_EQ(FoldrotorControl::kPosVelForceTW, 2.0f)
			<< "combined force sphere is specified as a thrust-to-weight ratio";
	EXPECT_NEAR(sphere, 39.2028f, 1e-3f)
			<< "T/W = 2 against the post-mast weight is 39.20 N";

	// The cost. The airframe's real T/W is 1.53, so the sphere is above
	// everything it can produce and is now inert.
	EXPECT_GT(sphere, rotor_ceiling)
			<< "sphere no longer bounds the command to the deliverable -- if this "
			"fails the T/W decision has been reverted, which is a decision, not a fix";
	EXPECT_LT(rotor_ceiling / weight, 2.0f)
			<< "airframe cannot actually reach T/W = 2; see findings.md 2026-09-23";
}

// controller_params.md "Command-path bandwidth limit" sits above this one;
// the criterion here is controller.md's translational dynamics contract,
// p_ddot = R*F_b/m - [0,0,g], which requires the Z feedforward to equal the
// vehicle's weight for a level hover.
//
// What this actually guards is the WEAKER, and previously unwritten,
// contract underneath it: the feedforward does not have to be exact,
// because the integrator trims the remainder -- but the remainder has to
// be something the integrator is ALLOWED to supply. FR_VEL_Z_I_LIM is a
// hard symmetric clamp on the accumulated integral in newtons
// (PositionVelocityControl.hpp's setIntegratorLimit()), so once
// |weight - grav_ff| exceeds it, the shortfall can only come from
// FR_VEL_Z_FF * e_v, which means a permanent velocity error and therefore
// a permanent position error. Nothing saturates and nothing warns; the
// vehicle simply holds the wrong altitude.
//
// That is exactly what the 2026-09-22 ballast mast caused. It added
// 0.443 kg without FR_VEL_Z_GRAV_FF being re-measured, leaving a 4.341 N
// gap against this 3.0 N bound. Log 2026-09-22/06_38_09.ulg: commanded
// -1.50 m, held -1.03 m, integrator pinned at -3.0001 N over 5778
// samples, zero allocator saturation. findings.md 2026-09-22 (14).
//
// kVehicleWeightN is model.sdf's nine link masses (2.000145 kg) times
// worlds/foldrotor.sdf's gravity (9.8), i.e. the force the simulated
// vehicle actually needs to hover.
TEST(FoldrotorControlParamTest, GravityFeedforwardResidualFitsIntegrator)
{
	auto default_of = [](const char *name) {
		param_t h = param_find(name);
		EXPECT_NE(h, PARAM_INVALID) << name;
		float v = NAN;
		param_get(h, &v);
		return v;
	};

	constexpr float kVehicleWeightN = 2.000145f * 9.8f;   // 19.6014 N

	const float grav_ff = default_of("FR_VEL_Z_GRAV_FF");
	const float i_lim   = default_of("FR_VEL_Z_I_LIM");
	const float vel_ff  = default_of("FR_VEL_Z_FF");
	const float pos_p   = default_of("FR_POS_P");

	const float residual = std::fabs(kVehicleWeightN - grav_ff);

	EXPECT_LT(residual, i_lim)
			<< "gravity feedforward is " << residual << " N from the vehicle's "
			<< kVehicleWeightN << " N weight, but FR_VEL_Z_I_LIM only permits "
			<< i_lim << " N of integral trim -- the remainder becomes a permanent "
			<< "altitude error of " << (residual - i_lim) / vel_ff / pos_p << " m";

	// Sanity on the other side: a feedforward that OVERSHOOTS the weight by
	// more than the integrator can pull back would hold the vehicle high by
	// the same mechanism, so the bound above is deliberately two-sided.
	EXPECT_GT(grav_ff, kVehicleWeightN - i_lim);
	EXPECT_LT(grav_ff, kVehicleWeightN + i_lim);

	// The steady-state altitude error this parameterisation actually
	// implies, stated as a number rather than left to be discovered in a
	// log. With the residual inside the integrator's range this is zero by
	// construction; the expectation documents that.
	const float steady_state_error_m = (residual > i_lim)
					   ? (residual - i_lim) / vel_ff / pos_p
					   : 0.f;
	EXPECT_FLOAT_EQ(steady_state_error_m, 0.f)
			<< "non-zero steady-state altitude error implied by the gains";
}

// The rate loop must stay LINEAR across the whole range of rate setpoints
// the attitude loop above it can produce. If FF * (achievable rate error)
// exceeds the moment envelope, the loop is a relay, not a controller --
// which is exactly what made the vehicle bang-bang on roll within 44 ms of
// arming (findings.md 2026-09-21 (6)): FR_RATE_R_FF = 3.5 against a 1.47
// N*m envelope railed at a rate error of only 0.42 rad/s, i.e. at ~8 deg
// of attitude error.
TEST(FoldrotorControlParamTest, RateLoopStaysLinearOverAttitudeLoopDemand)
{
	auto default_of = [](const char *name) {
		param_t h = param_find(name);
		EXPECT_NE(h, PARAM_INVALID) << name;
		float v = NAN;
		param_get(h, &v);
		return v;
	};

	// Measured, as above -- the predicted table is gone.
	foldrotor::FoldrotorAllocation alloc;
	const matrix::Vector3f hover_env(hoverMomentAuthority(alloc, 0),
					 hoverMomentAuthority(alloc, 1),
					 hoverMomentAuthority(alloc, 2));
	const float att_p = default_of("FR_ATT_P");

	// Largest rate setpoint the attitude stage can ask for at the tilt we
	// still expect to recover from. 45 deg is the design target: beyond
	// that this airframe cannot hold itself up regardless of moment.
	const float max_tilt_error = M_PI_F / 4.f;
	const float max_rate_sp = att_p * max_tilt_error;

	const char *ff_name[3] = {"FR_RATE_R_FF", "FR_RATE_P_FF", "FR_RATE_YAW_FF"};

	for (int a = 0; a < 2; a++) {   // roll, pitch -- yaw_sp is not produced this way
		const float rail_at = hover_env(a) / default_of(ff_name[a]);
		EXPECT_GT(rail_at, max_rate_sp)
				<< ff_name[a] << " rails the rate loop at " << rail_at
				<< " rad/s, below the " << max_rate_sp
				<< " rad/s the attitude loop commands at 45 deg of error";
	}
}

// fitWrenchToEnvelope() must leave the allocator with something it can
// satisfy exactly, for ANY commanded wrench -- including the physically
// impossible ones the cascade produces when the vehicle is far from level
// (at 73 deg of pitch, holding altitude asks for ~17 N of body-FORWARD
// force, which no amount of +-45 deg rotor tilt can deliver).
//
// This is the guard for the clamp-not-redistribute hazard that drove the
// whole flip investigation: an infeasible request came back as an
// arbitrary delivered wrench, worst case with My sign-flipped. The fit
// removes the hazard by construction rather than by tuning.
TEST(FoldrotorAllocationTest, FittedWrenchNeverSaturates)
{
	FoldrotorControl ctrl;

	// Sweep well beyond the envelope on every axis, including the tilted-
	// vehicle case where most of the collective has rotated into body X.
	const float fxy[] = { 0.f, 1.f, 5.f, 17.f, 40.f};
	const float fz[]  = { 0.f, 5.f, 15.26f, 20.f, 35.f, -10.f};
	const float mom[] = { 0.f, 0.15f, 1.5f, 5.f, 20.f};

	int fitted = 0;

	for (float h : fxy) {
		for (float v : fz) {
			for (float m : mom) {
				for (int k = 0; k < 8; k++) {
					const float theta = float(k) * (2.f * M_PI_F / 8.f);
					matrix::Vector3f F(h * cosf(theta), h * sinf(theta), v);
					matrix::Vector3f M(m, m * 0.1f, m);

					const bool was_saturated = ctrl.getAllocation().allocate(F, M).saturated;
					ctrl.fitWrenchToEnvelope(F, M);

					EXPECT_FALSE(ctrl.getAllocation().allocate(F, M).saturated)
							<< "fit left an infeasible wrench at |Fxy|=" << h
							<< " Fz=" << v << " M=" << m << " bearing " << k;

					if (was_saturated) { fitted++; }
				}
			}
		}
	}

	// Non-degeneracy: the sweep must actually contain infeasible cases, or
	// the assertion above is vacuous.
	EXPECT_GT(fitted, 50) << "sweep never produced an infeasible wrench";
}

// The fit's PRIORITY ordering, which is the part that matters for flight:
// vertical force is held, then moment, then horizontal force. Getting this
// backwards would trade away the ability to stay upright in order to chase
// a horizontal position setpoint.
TEST(FoldrotorAllocationTest, FitSacrificesHorizontalForceBeforeMoment)
{
	FoldrotorControl ctrl;

	// A wrench that is feasible in Fz and moment alone, but not once a
	// large horizontal force is added.
	matrix::Vector3f F(12.f, 0.f, 15.26f);
	matrix::Vector3f M(1.2f, 0.1f, 1.2f);
	const matrix::Vector3f M_commanded = M;

	ASSERT_TRUE(ctrl.getAllocation().allocate(F, M).saturated) << "test case is not infeasible";
	ctrl.fitWrenchToEnvelope(F, M);

	// Vertical force untouched ...
	EXPECT_FLOAT_EQ(F(2), 15.26f);
	// ... moment untouched ...
	EXPECT_FLOAT_EQ(M(0), M_commanded(0));
	EXPECT_FLOAT_EQ(M(1), M_commanded(1));
	EXPECT_FLOAT_EQ(M(2), M_commanded(2));
	// ... and the horizontal force is what gave way.
	EXPECT_LT(F(0), 12.f);
	EXPECT_GE(F(0), 0.f);
}

// --- Pitch tilt lever (2026-09-21 architecture change) ---------------
//
// Verification criteria from allocation.md "Pitch actuation path" and
// controller.md "Attitude/rate-loop form" item 5.

// THE defining property. Routing pitch through the body-x lever must
// drive the DIFFERENTIAL FOLD command to zero, because fold is the heavy,
// slow joint (0.0136 kg*m^2, 6.1 Hz, zeta 0.48) and tilt is the light,
// fast, well-damped one (0.0012 kg*m^2, 20.5 Hz, zeta 1.61). If this
// assertion fails the change is cosmetic: the moment is still being made
// on the actuator that cannot close a 2.77 Hz unstable pole.
TEST(FoldrotorAllocationTest, PitchLeverMovesPitchFromFoldToTilt)
{
	FoldrotorControl ctrl;
	const float lever = foldrotor::FoldrotorAllocation::pitchLeverFrd();
	const float My = 0.10f;   // FLU

	// Lever OFF: pitch is made by differential fold, and it is expensive.
	const auto off = ctrl.getAllocation().allocate(
				 matrix::Vector3f(0.f, 0.f, 19.615f), matrix::Vector3f(0.f, My, 0.f));
	const float fold_off = 0.5f * (fabsf(off.alpha1) + fabsf(off.alpha2));
	EXPECT_NEAR(math::degrees(fold_off), 12.86f, 0.5f) << "drag-path fold cost changed";
	EXPECT_LT(fabsf(off.beta1), math::radians(0.5f)) << "lever off must not use tilt";

	// Lever ON: same commanded My, Fx supplies it through the 0.0549
	// N*m/N arm.
	//
	// SIGN. allocate() is FLU (allocation.md "Frame convention"), and
	// pitchLeverFrd() is FRD, so the conversion is NOT a no-op even though
	// x itself is common to both frames: My flips between the frames, so
	//   My_flu = kS1z * Fx  ->  Fx = -My_flu / pitchLeverFrd().
	// Run() does the equivalent in FRD (+My_frd / pitchLeverFrd()) where
	// no flip is needed. Writing it out because getting it backwards
	// DOUBLES the fold command instead of cancelling it -- which is what
	// the first version of this test did, and the allocator reported 30
	// deg of fold rather than 0.
	const float fx_flu = -My / lever;
	const auto on = ctrl.getAllocation().allocate(
				matrix::Vector3f(fx_flu, 0.f, 19.615f), matrix::Vector3f(0.f, My, 0.f));

	EXPECT_LT(math::degrees(fabsf(on.alpha1)), 0.5f) << "fold should fall out entirely";
	EXPECT_LT(math::degrees(fabsf(on.alpha2)), 0.5f) << "fold should fall out entirely";
	EXPECT_NEAR(math::degrees(fabsf(on.beta1)), 16.70f, 0.5f) << "tilt should carry it instead";

	// REVERSED 2026-09-22, and this is the point. Before the ballast mast
	// the lever arm was 0.0549 N*m/N and moving pitch onto tilt was
	// CHEAPER in travel (6.8 deg vs 16.3). The mast shortened the arm to
	// 0.0170, so the same moment now costs MORE tilt than it did fold
	// (16.7 vs 12.9 deg) and 5.9 N of body-x force on top. The lever is
	// now a worse deal on every axis, which is why FR_PITCH_LEVER
	// defaults to 0. Asserting the inequality in its true direction
	// rather than deleting it keeps that trade visible.
	EXPECT_GT(fabsf(on.beta1), fold_off)
			<< "lever is expected to cost MORE travel at the mast geometry";
}

// The lever no longer buys authority at the ballast-mast geometry, and
// that is worth pinning rather than quietly dropping.
//
// Before the mast the arm was 0.0549 N*m/N, so letting Fx move raised
// max|My| from 0.342 to 1.490 N*m. The mast shortened the arm to 0.0170,
// and the arithmetic collapses: reaching even the Fx-pinned ceiling
// (0.440 N*m at the new 19.615 N hover) would demand 25.9 N of body-x
// force, far outside the actuator box. So the lever can only RELOCATE
// pitch travel now, never extend the envelope.
//
// This is the measured justification for FR_PITCH_LEVER defaulting to 0.
// The mechanism is kept because it is correct for any geometry with the
// rotors far from the CoM -- it is this particular airframe, after the
// mast, that no longer benefits.
TEST(FoldrotorAllocationTest, PitchLeverNoLongerExtendsTheEnvelopeAtMastGeometry)
{
	FoldrotorControl ctrl;
	const float lever = foldrotor::FoldrotorAllocation::pitchLeverFrd();
	const matrix::Vector3f hover(0.f, 0.f, 19.615f);

	// 0.44 N*m is just past the Fx-pinned ceiling ...
	const float My = 0.44f;
	EXPECT_TRUE(ctrl.getAllocation().allocate(hover, matrix::Vector3f(0.f, My, 0.f)).saturated)
			<< "0.44 N*m should be infeasible with Fx pinned";

	// ... and the lever cannot rescue it either, because the Fx it needs
	// is itself outside the box.
	const float fx_flu = -My / lever;
	EXPECT_GT(fabsf(fx_flu), 20.f) << "lever arm is no longer short enough to matter";
	EXPECT_TRUE(ctrl.getAllocation().allocate(matrix::Vector3f(fx_flu, 0.f, 19.615f),
			matrix::Vector3f(0.f, My, 0.f)).saturated)
			<< "the lever is not expected to extend the envelope at this geometry";
}

// REGRESSION. fitWrenchToEnvelope() sacrifices horizontal force first, by
// design. The lever component of Fx must be exempt -- it is the pitch
// moment, not a mission objective. Without the exemption the fit silently
// deletes the pitch command and the architecture change does nothing,
// which is a failure mode that would look exactly like "the lever didn't
// help" in flight.
TEST(FoldrotorAllocationTest, FitHoldsThePitchLeverAtMomentPriority)
{
	FoldrotorControl ctrl;
	// FLU, so a positive My is carried by a NEGATIVE Fx (see the sign
	// note in PitchLeverMovesPitchFromFoldToTilt).
	const float lever_fx = -0.05f / foldrotor::FoldrotorAllocation::pitchLeverFrd();
	const float extra_fx = 20.f;  // the position loop's share, infeasible

	matrix::Vector3f F(lever_fx + extra_fx, 0.f, 19.615f);
	matrix::Vector3f M(0.6f, 0.05f, 0.6f);
	const matrix::Vector3f M_commanded = M;

	ASSERT_TRUE(ctrl.getAllocation().allocate(F, M).saturated) << "test case is not infeasible";
	ctrl.fitWrenchToEnvelope(F, M, lever_fx);

	EXPECT_FALSE(ctrl.getAllocation().allocate(F, M).saturated) << "fit left an infeasible wrench";
	// The moment survived in full ...
	EXPECT_FLOAT_EQ(M(1), M_commanded(1));
	// ... and so did every newton of the lever. What is left of Fx beyond
	// the lever is the position loop's surviving share: between none of it
	// and all of it, never negative (which would mean the fit had eaten
	// into the lever).
	const float survived = (F(0) - lever_fx) / extra_fx;
	EXPECT_GE(survived, 0.f) << "the fit ate into the pitch lever";
	EXPECT_LT(survived, 1.f) << "nothing was sacrificed at all";
}

// When even the moment has to yield, the lever yields WITH it. Leaving a
// body-x force behind that no longer corresponds to any pitch request
// would be a standing horizontal disturbance of unknown origin.
TEST(FoldrotorAllocationTest, FitScalesThePitchLeverWithTheMoment)
{
	FoldrotorControl ctrl;
	const float lever_fx = -6.f;   // FLU

	// Moment far outside anything deliverable, so step 2 has to run.
	matrix::Vector3f F(lever_fx, 0.f, 19.615f);
	matrix::Vector3f M(20.f, 20.f, 20.f);

	ctrl.fitWrenchToEnvelope(F, M, lever_fx);

	EXPECT_FALSE(ctrl.getAllocation().allocate(F, M).saturated) << "fit left an infeasible wrench";
	EXPECT_LT(fabsf(F(0)), fabsf(lever_fx)) << "moment was cut but the lever was not";
	EXPECT_GE(F(0) / lever_fx, 0.f) << "the lever changed sign";
}

// Passing no lever keeps the pre-2026-09-21 behaviour exactly, so the
// existing priority guard above still describes the default path.
TEST(FoldrotorAllocationTest, FitWithoutLeverIsUnchanged)
{
	FoldrotorControl ctrl;
	matrix::Vector3f F_a(12.f, 0.f, 19.615f), F_b(12.f, 0.f, 19.615f);
	matrix::Vector3f M_a(1.2f, 0.1f, 1.2f), M_b(1.2f, 0.1f, 1.2f);

	ctrl.fitWrenchToEnvelope(F_a, M_a);
	ctrl.fitWrenchToEnvelope(F_b, M_b, 0.f);

	EXPECT_FLOAT_EQ(F_a(0), F_b(0));
	EXPECT_FLOAT_EQ(M_a(1), M_b(1));
}

// Hover, Fz = +15.26 N (Z-up positive per allocation.md's thrust vector
// -- see FoldrotorAllocation.hpp OPEN ITEM (c), NOT PX4 body FRD).
//
// The split is NEARLY even, not exactly even, as of the 2026-09-21
// geometry correction (OPEN ITEM (d)): the moment arms are now measured
// from the vehicle's true CoM, which model.sdf puts 0.001024 m off
// centre in Y, so holding Mx = 0 requires the near rotor to carry
// slightly more thrust. Hand solved via Minv*w: F1 = 7.658913 N,
// F2 = 7.601087 N (sum = 15.26 N exactly), with beta picking up a
// 3e-4 rad tilt from the 8.6e-5 m CoM x-offset.
//
// This asymmetry is the fix, not an error: the old even split left a
// standing +0.0156 N*m roll moment at hover, which is exactly the
// residual the bench measured as its static baseline
// (force_moment_test.md, predicted +0.0156 vs measured +0.0159).
TEST(FoldrotorAllocationTest, HoverProducesNearEvenSplitZeroTilt)
{
	foldrotor::FoldrotorAllocation alloc;
	// 2026-09-22: hover thrust is 19.615 N, not 15.26 -- the ballast mast
	// took the vehicle to 2.00 kg. Values re-solved through the updated
	// Minv; the near-even-but-not-even split this test exists to pin is
	// unchanged in kind, only in magnitude.
	const auto out = alloc.allocate(matrix::Vector3f(0.f, 0.f, 19.615f), matrix::Vector3f(0.f, 0.f, 0.f));

	EXPECT_NEAR(out.F1, 9.836426f, 1e-4f);
	EXPECT_NEAR(out.F2, 9.778574f, 1e-4f);
	EXPECT_NEAR(out.F1 + out.F2, 19.615f, 1e-4f);
	EXPECT_NEAR(out.alpha1, 0.f, 1e-6f);
	EXPECT_NEAR(out.alpha2, 0.f, 1e-6f);
	EXPECT_NEAR(out.beta1, 0.000244f, 1e-5f);
	EXPECT_NEAR(out.beta2, -0.000246f, 1e-5f);
	EXPECT_FALSE(out.saturated);
}

// Hover + Mx = 0.5 N*m. Hand solved via Minv*w (both rotors' Ty stays 0,
// so alpha is unaffected). Recomputed 2026-09-21 for the sign-corrected,
// CoM-referenced geometry (FoldrotorAllocation.hpp OPEN ITEM (d) -- was
// F1=8.557926 N, F2=6.702534 N, beta1=+0.006866, beta2=-0.008767 against
// the sign-inverted +-0.2684/+0.0301 geometry): F1=6.734181 N,
// F2=8.526554 N, beta1=-0.011046 rad, beta2=+0.008724 rad.
//
// Note which rotor gains thrust: rotor 1 is Motor1/Arm1, which the
// corrected constants place on body -Y in FLU, so a positive (right-hand
// about +X) roll moment is produced by loading rotor 2, not rotor 1. The
// pre-fix expectation had this backwards, which is the sign inversion
// OPEN ITEM (d) describes.
TEST(FoldrotorAllocationTest, HoverPlusRollMomentMatchesHandSolved)
{
	foldrotor::FoldrotorAllocation alloc;
	const auto out = alloc.allocate(matrix::Vector3f(0.f, 0.f, 19.615f), matrix::Vector3f(0.5f, 0.f, 0.f));

	EXPECT_NEAR(out.F1, 8.911594f, 1e-4f);
	EXPECT_NEAR(out.F2, 10.703975f, 1e-4f);
	EXPECT_NEAR(out.beta1, -0.008347f, 1e-5f);
	EXPECT_NEAR(out.beta2, 0.006949f, 1e-5f);
	EXPECT_FALSE(out.saturated);
}

// Hover + Mz = 0.2 N*m. Recomputed 2026-09-21 for the sign-corrected,
// CoM-referenced geometry (was F1=7.662495 N, F2=7.615542 N,
// beta1=-0.048448, beta2=+0.048747): F1=7.637288 N, F2=7.640883 N,
// beta1=+0.048788 rad, beta2=-0.048765 rad. Yaw is produced almost
// entirely by differential tilt (beta), so the sign correction shows up
// here as the beta pair swapping sign.
TEST(FoldrotorAllocationTest, HoverPlusYawMomentMatchesHandSolved)
{
	foldrotor::FoldrotorAllocation alloc;
	const auto out = alloc.allocate(matrix::Vector3f(0.f, 0.f, 19.615f), matrix::Vector3f(0.f, 0.f, 0.2f));

	EXPECT_NEAR(out.F1, 9.812784f, 1e-4f);
	EXPECT_NEAR(out.F2, 9.816355f, 1e-4f);
	EXPECT_NEAR(out.beta1, 0.037965f, 1e-5f);
	EXPECT_NEAR(out.beta2, -0.037952f, 1e-5f);
	EXPECT_FALSE(out.saturated);
}

// Round-trip test (allocation.md's other currently-missing test): feed
// each of the three unsaturated cases above back through the forward
// map Tx=F*sin(beta), Ty=-F*cos(beta)*sin(alpha), Tz=F*cos(beta)*cos(alpha)
// and w_reconstructed = M0*T; recovered wrench must match the original
// within tolerance. alpha happens to be 0 for all three of these cases
// (pure Fz/Mx/Mz -- no Fy, so Ty stays 0 on both rotors, see the
// per-test comments above), but the forward map includes the alpha term
// unconditionally so this generalises correctly now that alpha is
// unpinned (see RoundTripReproducesCommandedWrenchWithNonzeroTy below
// for cases that actually exercise it).
TEST(FoldrotorAllocationTest, RoundTripReproducesCommandedWrench)
{
	foldrotor::FoldrotorAllocation alloc;

	const matrix::Vector3f wrenches_F[] = {
		matrix::Vector3f(0.f, 0.f, 15.26f),
		matrix::Vector3f(0.f, 0.f, 15.26f),
		matrix::Vector3f(0.f, 0.f, 15.26f),
	};
	const matrix::Vector3f wrenches_M[] = {
		matrix::Vector3f(0.f, 0.f, 0.f),
		matrix::Vector3f(0.5f, 0.f, 0.f),
		matrix::Vector3f(0.f, 0.f, 0.2f),
	};

	for (int c = 0; c < 3; c++) {
		const auto out = alloc.allocate(wrenches_F[c], wrenches_M[c]);

		// 1e-5, not 1e-6: with the CoM-referenced geometry (OPEN ITEM
		// (d)) the pure-Mz case leaves a ~2e-6 rad residual alpha
		// rather than an exact zero, since Ty no longer cancels to
		// machine zero across the two rotors.
		ASSERT_NEAR(out.alpha1, 0.f, 1e-5f);
		ASSERT_NEAR(out.alpha2, 0.f, 1e-5f);

		auto forward = [](float F, float alpha, float beta) {
			return matrix::Vector3f(F * sinf(beta), -F * cosf(beta) * sinf(alpha), F * cosf(beta) * cosf(alpha));
		};

		const matrix::Vector3f T1 = forward(out.F1, out.alpha1, out.beta1);
		const matrix::Vector3f T2 = forward(out.F2, out.alpha2, out.beta2);

		matrix::Vector<float, 6> T;
		T(0) = T1(0); T(1) = T1(1); T(2) = T1(2);
		T(3) = T2(0); T(4) = T2(1); T(5) = T2(2);

		const matrix::Vector<float, 6> w = alloc.getM0() * T;

		EXPECT_NEAR(w(0), wrenches_F[c](0), 1e-3f) << "case " << c;
		EXPECT_NEAR(w(1), wrenches_F[c](1), 1e-3f) << "case " << c;
		EXPECT_NEAR(w(2), wrenches_F[c](2), 1e-3f) << "case " << c;
		EXPECT_NEAR(w(3), wrenches_M[c](0), 1e-3f) << "case " << c;
		EXPECT_NEAR(w(4), wrenches_M[c](1), 1e-3f) << "case " << c;
		EXPECT_NEAR(w(5), wrenches_M[c](2), 1e-3f) << "case " << c;
	}
}

// Round-trip with alpha actually nonzero on both rotors (Fy != 0 in the
// commanded wrench), now that alpha is unpinned. Expected values below
// are from an independent numeric solve of the same M0/Minv (not the
// class's own getMinv(), to avoid the check being circular) -- see
// allocation.md's alpha sign-mapping section for the bench data that
// fixed the alpha_n -> ArmNFoldJoint mapping this depends on being
// consistent with. Includes one pure-lateral case and two combined
// (Fy + Mx, Fy + Mz) cases, all unsaturated.
TEST(FoldrotorAllocationTest, RoundTripReproducesCommandedWrenchWithNonzeroTy)
{
	foldrotor::FoldrotorAllocation alloc;

	const matrix::Vector3f wrenches_F[] = {
		matrix::Vector3f(0.f, 10.f, 15.26f),
		matrix::Vector3f(0.f, -8.f, 15.26f),
		matrix::Vector3f(0.f, 6.f, 15.26f),
	};
	const matrix::Vector3f wrenches_M[] = {
		matrix::Vector3f(0.f, 0.f, 0.f),
		matrix::Vector3f(0.3f, 0.f, 0.f),
		matrix::Vector3f(0.f, 0.f, 0.15f),
	};

	for (int c = 0; c < 3; c++) {
		const auto out = alloc.allocate(wrenches_F[c], wrenches_M[c]);

		ASSERT_FALSE(out.saturated) << "case " << c;
		// Sanity: this test only means something if alpha actually moved.
		ASSERT_GT(fabsf(out.alpha1), 1e-3f) << "case " << c;
		ASSERT_GT(fabsf(out.alpha2), 1e-3f) << "case " << c;

		auto forward = [](float F, float alpha, float beta) {
			return matrix::Vector3f(F * sinf(beta), -F * cosf(beta) * sinf(alpha), F * cosf(beta) * cosf(alpha));
		};

		const matrix::Vector3f T1 = forward(out.F1, out.alpha1, out.beta1);
		const matrix::Vector3f T2 = forward(out.F2, out.alpha2, out.beta2);

		matrix::Vector<float, 6> T;
		T(0) = T1(0); T(1) = T1(1); T(2) = T1(2);
		T(3) = T2(0); T(4) = T2(1); T(5) = T2(2);

		const matrix::Vector<float, 6> w = alloc.getM0() * T;

		EXPECT_NEAR(w(0), wrenches_F[c](0), 1e-3f) << "case " << c;
		EXPECT_NEAR(w(1), wrenches_F[c](1), 1e-3f) << "case " << c;
		EXPECT_NEAR(w(2), wrenches_F[c](2), 1e-3f) << "case " << c;
		EXPECT_NEAR(w(3), wrenches_M[c](0), 1e-3f) << "case " << c;
		EXPECT_NEAR(w(4), wrenches_M[c](1), 1e-3f) << "case " << c;
		EXPECT_NEAR(w(5), wrenches_M[c](2), 1e-3f) << "case " << c;
	}
}

// F_b=(0,0,100), M_b=0 -> T1=T2=(0,0,50) unsaturated-math F=50 N on
// both rotors -- clamps to kMaxThrust=15 N, beta stays 0 (no tilt
// demanded), saturated=true. F can never go below 0 by construction
// (F = norm(Tx,Ty,Tz), which cannot be negative) -- the lower clamp
// branch exists defensively but is structurally unreachable, not tested
// via a negative-F case here.
TEST(FoldrotorAllocationTest, ThrustClampsAtFifteen)
{
	foldrotor::FoldrotorAllocation alloc;
	const auto out = alloc.allocate(matrix::Vector3f(0.f, 0.f, 100.f), matrix::Vector3f(0.f, 0.f, 0.f));

	EXPECT_NEAR(out.F1, 15.f, 1e-4f);
	EXPECT_NEAR(out.F2, 15.f, 1e-4f);
	// beta is the same 3e-4 rad CoM-offset residual as the hover case
	// above (OPEN ITEM (d)), not zero -- no tilt is demanded here.
	EXPECT_NEAR(out.beta1, 0.000244f, 1e-5f);
	EXPECT_NEAR(out.beta2, -0.000246f, 1e-5f);
	EXPECT_TRUE(out.saturated);
}

// beta (tilt) clamp. The stimulus was rebuilt 2026-09-21 for the
// sign-corrected geometry (OPEN ITEM (d)). The old wrench --
// F_b=(40,0,30), M_b=(0,0.8,0) -- no longer isolates beta: with the
// arms referenced to the true CoM, holding My against a 40 N forward
// force demands a huge differential Ty (+-49 N), so it is now ALPHA that
// rails first (1.27 rad -> 0.79) and beta only reaches 0.37 rad. That
// would still have gone green while testing the wrong channel.
//
// So this case is constructed from the thrust vectors instead, which is
// exact and hand-checkable: pick T1 = T2 = (20, 0, 15), forward-map it
// through M0 to get the wrench that demands it, and feed that in. Ty = 0
// on both rotors by construction, so alpha stays 0 and beta alone
// saturates: beta = atan2(20, 15) = 0.927295 rad > kMaxTilt, clamped to
// +0.79. F = norm(20,0,15) = 25 N also clamps to 15 N.
TEST(FoldrotorAllocationTest, TiltClampsAtPositivePointSevenNine)
{
	foldrotor::FoldrotorAllocation alloc;
	const auto out = alloc.allocate(matrix::Vector3f(40.f, 0.f, 30.f),
					matrix::Vector3f(0.023910f, 0.680380f, -0.031880f));

	EXPECT_NEAR(out.beta1, 0.79f, 1e-5f);
	EXPECT_NEAR(out.beta2, 0.79f, 1e-5f);
	EXPECT_NEAR(out.alpha1, 0.f, 1e-6f);
	EXPECT_NEAR(out.alpha2, 0.f, 1e-6f);
	EXPECT_TRUE(out.saturated);
}

// Mirror of the above: T1 = T2 = (-20, 0, 15), whose forward map is
// F_b=(-40,0,30), M_b=(0.030720, +2.197000, +0.040960). Unclamped
// beta = atan2(-20, 15) = -0.927295 rad, clamped to -kMaxTilt = -0.79.
TEST(FoldrotorAllocationTest, TiltClampsAtNegativePointSevenNine)
{
	foldrotor::FoldrotorAllocation alloc;
	const auto out = alloc.allocate(matrix::Vector3f(-40.f, 0.f, 30.f),
					matrix::Vector3f(0.023910f, -0.680380f, 0.031880f));

	EXPECT_NEAR(out.beta1, -0.79f, 1e-5f);
	EXPECT_NEAR(out.beta2, -0.79f, 1e-5f);
	EXPECT_NEAR(out.alpha1, 0.f, 1e-6f);
	EXPECT_NEAR(out.alpha2, 0.f, 1e-6f);
	EXPECT_TRUE(out.saturated);
}

// allocation.md's explicitly requested regression guard: max_tilt must
// be 0.79 rad, NOT 1.0472 rad (the allocator's original design-intent
// figure, decided-against 2026-09-06 -- see allocation.md "Tilt limit").
TEST(FoldrotorAllocationTest, MaxTiltConstantIsPointSevenNine)
{
	EXPECT_FLOAT_EQ(foldrotor::FoldrotorAllocation::kMaxTilt, 0.79f);
	EXPECT_NE(foldrotor::FoldrotorAllocation::kMaxTilt, 1.0472f);
}

// Fold unpinned 2026-09-10 (FoldrotorAllocation.hpp OPEN ITEM (a),
// bench-verified sign per allocation.md's alpha sign-mapping section).
// A pure +Y force command (F_b=(0,10,15.26), M_b=0) must now produce a
// nonzero, same-signed alpha on BOTH rotors -- checking both is the
// point (allocation.md's alpha/beta mapping is common-mode, not
// differential, so a per-arm sign flip would be a real bug, not
// symmetric noise). Expected values from an independent numeric solve
// of M0/Minv (see RoundTripReproducesCommandedWrenchWithNonzeroTy's
// comment), recomputed 2026-09-21 for the sign-corrected geometry (OPEN
// ITEM (d) -- was alpha1=-0.548187, alpha2=-0.615450):
// alpha1=-0.522854 rad, alpha2=-0.649440 rad -- still negative on both,
// matching alpha=atan2(-Ty,Tz) for the positive Ty this Fy produces on
// both rotors (Control_Alloc's own +alpha -> -Ty convention). The
// common-mode sign is what this test guards and it is unchanged; only
// how the demand splits between the two arms moved.
TEST(FoldrotorAllocationTest, PureLateralForceProducesNonzeroAlphaCorrectSign)
{
	foldrotor::FoldrotorAllocation alloc;
	const auto out = alloc.allocate(matrix::Vector3f(0.f, 10.f, 19.615f), matrix::Vector3f(0.f, 0.f, 0.f));

	EXPECT_NEAR(out.alpha1, -0.483538f, 1e-5f);
	EXPECT_NEAR(out.alpha2, -0.459943f, 1e-5f);
	EXPECT_LT(out.alpha1, 0.f);
	EXPECT_LT(out.alpha2, 0.f);
	EXPECT_FALSE(out.saturated);
}

// The atan2(0,0) guard: a zero wrench must produce all-zero, finite
// output on both rotors -- not whatever atan2's implementation-defined
// behaviour at the origin would otherwise propagate.
TEST(FoldrotorAllocationTest, DegenerateZeroWrenchProducesZeroCommands)
{
	foldrotor::FoldrotorAllocation alloc;
	const auto out = alloc.allocate(matrix::Vector3f(0.f, 0.f, 0.f), matrix::Vector3f(0.f, 0.f, 0.f));

	EXPECT_TRUE(PX4_ISFINITE(out.F1));
	EXPECT_TRUE(PX4_ISFINITE(out.F2));
	EXPECT_TRUE(PX4_ISFINITE(out.alpha1));
	EXPECT_TRUE(PX4_ISFINITE(out.alpha2));
	EXPECT_TRUE(PX4_ISFINITE(out.beta1));
	EXPECT_TRUE(PX4_ISFINITE(out.beta2));

	EXPECT_FLOAT_EQ(out.F1, 0.f);
	EXPECT_FLOAT_EQ(out.F2, 0.f);
	EXPECT_FLOAT_EQ(out.alpha1, 0.f);
	EXPECT_FLOAT_EQ(out.alpha2, 0.f);
	EXPECT_FLOAT_EQ(out.beta1, 0.f);
	EXPECT_FLOAT_EQ(out.beta2, 0.f);
}

// allocation.md's "including near-singular inputs": a coarse grid of
// wrenches sweeping through zero and through large/asymmetric values
// (including exactly the degenerate zero point) must never produce a
// non-finite output or a value outside its declared range, on any
// channel.
TEST(FoldrotorAllocationTest, NoOutputIsEverNonFinite)
{
	foldrotor::FoldrotorAllocation alloc;

	const float values[] = { -1000.f, -50.f, -1.f, 0.f, 1.f, 50.f, 1000.f };

	for (float fx : values) {
		for (float fy : values) {
			for (float fz : values) {
				const auto out = alloc.allocate(matrix::Vector3f(fx, fy, fz),
								matrix::Vector3f(0.1f, -0.2f, 0.3f));

				ASSERT_TRUE(PX4_ISFINITE(out.F1));
				ASSERT_TRUE(PX4_ISFINITE(out.F2));
				ASSERT_TRUE(PX4_ISFINITE(out.alpha1));
				ASSERT_TRUE(PX4_ISFINITE(out.alpha2));
				ASSERT_TRUE(PX4_ISFINITE(out.beta1));
				ASSERT_TRUE(PX4_ISFINITE(out.beta2));

				EXPECT_GE(out.F1, 0.f);
				EXPECT_LE(out.F1, foldrotor::FoldrotorAllocation::kMaxThrust);
				EXPECT_GE(out.F2, 0.f);
				EXPECT_LE(out.F2, foldrotor::FoldrotorAllocation::kMaxThrust);

				EXPECT_GE(out.alpha1, -foldrotor::FoldrotorAllocation::kMaxTilt);
				EXPECT_LE(out.alpha1, foldrotor::FoldrotorAllocation::kMaxTilt);
				EXPECT_GE(out.alpha2, -foldrotor::FoldrotorAllocation::kMaxTilt);
				EXPECT_LE(out.alpha2, foldrotor::FoldrotorAllocation::kMaxTilt);

				EXPECT_GE(out.beta1, -foldrotor::FoldrotorAllocation::kMaxTilt);
				EXPECT_LE(out.beta1, foldrotor::FoldrotorAllocation::kMaxTilt);
				EXPECT_GE(out.beta2, -foldrotor::FoldrotorAllocation::kMaxTilt);
				EXPECT_LE(out.beta2, foldrotor::FoldrotorAllocation::kMaxTilt);
			}
		}
	}
}

// ---------------------------------------------------------------------
// FoldrotorControl actuator-mapping static methods (step 4e part 2,
// the wiring into Run() -- not FoldrotorAllocation itself)
// ---------------------------------------------------------------------

// allocation.md (updated 2026-09-10, bench-measured): a positive
// ArmNFoldJoint angle produces NEGATIVE Y thrust for both arms, matching
// Control_Alloc's own +alpha -> -Ty convention directly -- no sign flip
// needed. alpha_n -> ArmNFoldJoint is a DIRECT mapping (superseding the
// old, kinematically-reasoned-but-bench-disproven negated mapping):
// foldToNormalizedServo(0.395) = 0.395/0.79 = 0.5.
TEST(FoldrotorControlMappingTest, AlphaMapsDirectToFoldChannel)
{
	EXPECT_NEAR(FoldrotorControl::foldToNormalizedServo(0.395f), 0.5f, 1e-5f);
	EXPECT_NEAR(FoldrotorControl::foldToNormalizedServo(-0.395f), -0.5f, 1e-5f);
	EXPECT_NEAR(FoldrotorControl::foldToNormalizedServo(0.79f), 1.f, 1e-5f);
}

// beta -> servo mapping is direct (no negation), linear on [-0.79, 0.79].
TEST(FoldrotorControlMappingTest, BetaMapsDirectToTiltChannel)
{
	EXPECT_NEAR(FoldrotorControl::tiltToNormalizedServo(0.395f), 0.5f, 1e-5f);
	EXPECT_NEAR(FoldrotorControl::tiltToNormalizedServo(-0.79f), -1.f, 1e-5f);
}

// F = motorConstant * omega^2 (model.sdf:557-559, motorConstant =
// 5.4844e-06). Round-trip: pick a thrust inside [0, 15] N, convert to a
// normalized command against a known [ec_min, ec_max] range, then invert
// the same curve by hand and confirm the normalized command lands where
// hand-computed omega/interpolation predicts.
//
// Hand computed for thrust = 7.63 N (hover-per-rotor), ec_min=308,
// ec_max=2054: omega = sqrt(7.63 / 5.4844e-6) = 1179.4994... rad/s;
// normalized = (1179.4994 - 308) / (2054 - 308) = 0.499141...
TEST(FoldrotorControlMappingTest, ThrustToNormalizedInvertsSdfCurve)
{
	const float normalized = FoldrotorControl::thrustToNormalizedMotor(7.63f, 308.f, 2054.f);
	EXPECT_NEAR(normalized, 0.499141f, 1e-4f);

	// Zero thrust: omega=0 is below SIM_GZ_EC_MIN1's idle floor (308
	// rad/s), so the raw interpolation goes negative (-308/1746 =
	// -0.1764) and the function's own [0,1] clamp brings it back to 0 --
	// zero thrust cannot command a negative normalized value.
	const float normalized_zero = FoldrotorControl::thrustToNormalizedMotor(0.f, 308.f, 2054.f);
	EXPECT_NEAR(normalized_zero, 0.f, 1e-5f);

	// Max thrust (kMaxThrust = 15 N): omega = sqrt(15/5.4844e-6) =
	// 1654.786... rad/s, normalized = (1654.786 - 308)/1746 = 0.771 --
	// well inside [0,1], not driven to the rail (2054 is model.sdf's
	// maxRotVelocity headroom above the 15 N actuator limit, not equal
	// to it).
	const float normalized_max = FoldrotorControl::thrustToNormalizedMotor(15.f, 308.f, 2054.f);
	EXPECT_NEAR(normalized_max, 0.770786f, 1e-4f);

	// Thrust beyond kMaxThrust is defensively clamped to 15 N before the
	// curve inversion, so it must produce the exact same normalized
	// command as thrust == kMaxThrust, not overshoot [0,1].
	const float normalized_over = FoldrotorControl::thrustToNormalizedMotor(1000.f, 308.f, 2054.f);
	EXPECT_NEAR(normalized_over, normalized_max, 1e-6f);
}

// ---------------------------------------------------------------------
// FRD -> allocator FLU frame transform (allocation.md open item (c),
// resolved 2026-09-08)
// ---------------------------------------------------------------------

// Direct unit test of the transform itself: all three components
// distinct and nonzero, so a wrong or partial flip (e.g. only Z negated,
// not Y -- exactly the kind of bug a hover-only test cannot see, since Y
// is zero at hover either way) shows up as a wrong Y or wrong Z, not
// just a wrong magnitude.
TEST(FoldrotorControlMappingTest, FrdToAllocatorFluNegatesYAndZOnly)
{
	const matrix::Vector3f flu = FoldrotorControl::frdToAllocatorFlu(matrix::Vector3f(1.f, 2.f, 3.f));

	EXPECT_FLOAT_EQ(flu(0), 1.f);   // X unchanged
	EXPECT_FLOAT_EQ(flu(1), -2.f);  // Y negated
	EXPECT_FLOAT_EQ(flu(2), -3.f);  // Z negated

	// The rotation is its own inverse (180 deg): applying it twice must
	// return the original vector exactly.
	const matrix::Vector3f round_trip = FoldrotorControl::frdToAllocatorFlu(flu);
	EXPECT_FLOAT_EQ(round_trip(0), 1.f);
	EXPECT_FLOAT_EQ(round_trip(1), 2.f);
	EXPECT_FLOAT_EQ(round_trip(2), 3.f);
}

// Non-hover, lateral case through the full FRD->FLU->allocate() path:
// F_b = (0, 10, -15.26) N FRD (lateral +Y, hover-weight lift), M_b =
// (0.3, 0, 0) N*m FRD -- nonzero Y on both force and (via the resulting
// Ty on each rotor) moment allocation, so this cannot pass by accident
// the way an Fz-only hover case can.
//
// Hand solved (standalone numpy, matching this file's established
// style). Recomputed 2026-09-21 for the sign-corrected, CoM-referenced
// geometry (FoldrotorAllocation.hpp OPEN ITEM (d) -- was F1=9.120778 N,
// F2=9.123882 N, beta1=-0.000013 rad, beta2=+0.000013 rad):
// F_alloc = (0, -10, 15.26), M_alloc = (0.3, -0, -0) after the
// transform; allocate() through the corrected Minv gives
// T1 = (-0.128014, -5, 6.087558), T2 = (0.128014, -5, 9.172442), so
//   F1 = 7.878757 N,  F2 = 10.447489 N
//   beta1 = -0.016249 rad,  beta2 = 0.012253 rad
// A bug that flips only Z (leaves Y unflipped) produces a materially
// different result against this same corrected geometry -- F1 =
// 9.536123 N, F2 = 8.716643 N, beta1 = +0.004265 rad,
// beta2 = -0.004666 rad, hand-checked the same way -- so this test still
// catches that specific partial-flip bug, not just a totally-missing
// transform.
TEST(FoldrotorAllocationTest, LateralFrdWrenchAllocatesConsistentlyWithFullFlip)
{
	foldrotor::FoldrotorAllocation alloc;

	const matrix::Vector3f F_b(0.f, 10.f, -19.615f);
	const matrix::Vector3f M_b(0.3f, 0.f, 0.f);

	const matrix::Vector3f F_alloc = FoldrotorControl::frdToAllocatorFlu(F_b);
	const matrix::Vector3f M_alloc = FoldrotorControl::frdToAllocatorFlu(M_b);

	const auto out = alloc.allocate(F_alloc, M_alloc);

	EXPECT_NEAR(out.F1, 10.820571f, 1e-4f);
	EXPECT_NEAR(out.F2, 11.197293f, 1e-4f);
	EXPECT_NEAR(out.beta1, -0.001622f, 1e-5f);
	EXPECT_NEAR(out.beta2, 0.001567f, 1e-5f);
}

// --- Command-path bandwidth limit (2026-09-22) -----------------------
//
// Verification criteria from controller.md "Command-path bandwidth
// limit" and the FR_WRENCH_LP param.
//
// WHAT THIS EXISTS TO CATCH. Log 2026-09-22/06_12_34.ulg measured the
// joint torque needed to TRACK the commanded angles -- J*d2(theta)/dt2
// against model.sdf's cmd_max = 5 N*m -- at 12.1 N*m rms on tilt and
// 54.9 N*m rms on fold, over cmd_max on 52% and 89% of samples. Nothing
// in the chain bounded the command's slew; these tests pin the piece
// that now does.

namespace
{

/** Step the filter n times at a fixed rate, returning the last output. */
void stepWrenchLowPass(FoldrotorControl &ctrl, matrix::Vector3f &F, matrix::Vector3f &M,
		       const matrix::Vector3f &F_in, const matrix::Vector3f &M_in,
		       float cutoff_hz, float dt, int n)
{
	for (int i = 0; i < n; i++) {
		F = F_in;
		M = M_in;
		ctrl.applyWrenchLowPass(F, M, cutoff_hz, dt);
	}
}

} // namespace

// Unity DC gain. A steady wrench must survive the filter untouched, or
// every hover trim in controller_params.md shifts the moment the filter
// is enabled. The first call seeds, so even that one is exact.
TEST(FoldrotorWrenchLowPassTest, SteadyWrenchPassesUnchanged)
{
	FoldrotorControl ctrl;

	const matrix::Vector3f F_in(0.4f, -0.2f, -19.615f);
	const matrix::Vector3f M_in(0.05f, -0.01f, 0.02f);

	matrix::Vector3f F = F_in;
	matrix::Vector3f M = M_in;
	ctrl.applyWrenchLowPass(F, M, 5.f, 0.004f);

	// Seeded, not ramped: no first-cycle transient onto the actuators.
	EXPECT_FLOAT_EQ(F(2), F_in(2));
	EXPECT_FLOAT_EQ(M(0), M_in(0));

	stepWrenchLowPass(ctrl, F, M, F_in, M_in, 5.f, 0.004f, 500);

	for (int i = 0; i < 3; i++) {
		EXPECT_NEAR(F(i), F_in(i), 1e-5f) << "force axis " << i;
		EXPECT_NEAR(M(i), M_in(i), 1e-6f) << "moment axis " << i;
	}
}

// THE defining property. At the 18.8 Hz actuator limit cycle measured in
// the log, a 5 Hz corner must actually attenuate -- first-order theory
// says 1/sqrt(1 + (18.8/5)^2) = 0.257. If this ever passes near 1.0 the
// filter is wired but inert.
TEST(FoldrotorWrenchLowPassTest, AttenuatesAtTheMeasuredLimitCycleFrequency)
{
	FoldrotorControl ctrl;

	const float dt = 0.004f;          // 250 Hz rate loop
	const float f_osc = 18.8f;        // measured yaw limit cycle
	const float cutoff = 5.f;         // FR_WRENCH_LP default

	// Drive Mz with a unit sinusoid and measure the steady-state
	// amplitude, skipping the first 2 s of settling.
	float peak = 0.f;

	for (int i = 0; i < 2500; i++) {
		const float t = float(i) * dt;
		matrix::Vector3f F(0.f, 0.f, -19.615f);
		matrix::Vector3f M(0.f, 0.f, sinf(2.f * float(M_PI) * f_osc * t));
		ctrl.applyWrenchLowPass(F, M, cutoff, dt);

		if (i > 500) {
			peak = math::max(peak, fabsf(M(2)));
		}
	}

	const float expected = 1.f / sqrtf(1.f + (f_osc / cutoff) * (f_osc / cutoff));
	EXPECT_NEAR(peak, expected, 0.06f) << "5 Hz corner must cut 18.8 Hz to ~26%";
	EXPECT_LT(peak, 0.4f);
}

// REGRESSION. The filter runs AFTER the rate loop's output clamp, and
// fitWrenchToEnvelope() plus the conditional-integration anti-windup both
// assume the moment they see respects those bounds. A first-order
// low-pass is a convex combination of past samples, so it cannot exceed
// them -- but only if the implementation never overshoots. Pin it: drive
// a full-scale square wave at the clamp and check nothing leaves the box.
TEST(FoldrotorWrenchLowPassTest, NeverExceedsTheBoundsOfItsInput)
{
	FoldrotorControl ctrl;

	const matrix::Vector3f limit(1.52f, 0.22f, 2.56f);   // kRateM*Limit
	const float dt = 0.004f;

	for (int i = 0; i < 2000; i++) {
		const float sign = (i % 7 < 3) ? 1.f : -1.f;
		matrix::Vector3f F(0.f, 0.f, -19.615f);
		matrix::Vector3f M = limit * sign;
		ctrl.applyWrenchLowPass(F, M, 5.f, dt);

		for (int a = 0; a < 3; a++) {
			EXPECT_LE(fabsf(M(a)), limit(a) + 1e-5f)
					<< "axis " << a << " left the rate loop's envelope at step " << i;
		}
	}
}

// FR_WRENCH_LP <= 0 must be a true bypass, so the unfiltered command path
// stays flyable for comparison and every pre-2026-09-22 test keeps
// exercising it. A bypassed filter must also not accumulate stale state:
// re-enabling it has to start from the live command, not from whatever it
// last held, or enabling the param in flight steps the actuators.
TEST(FoldrotorWrenchLowPassTest, NonPositiveCutoffBypassesAndReseeds)
{
	FoldrotorControl ctrl;

	// Run the filter up to a steady state at one wrench ...
	matrix::Vector3f F(0.f, 0.f, -19.615f);
	matrix::Vector3f M(0.f, 0.f, 0.f);
	const matrix::Vector3f F_a(0.f, 0.f, -19.615f);
	const matrix::Vector3f M_a(1.f, 0.f, 0.f);
	stepWrenchLowPass(ctrl, F, M, F_a, M_a, 5.f, 0.004f, 500);
	ASSERT_NEAR(M(0), 1.f, 1e-4f);

	// ... bypass it at a very different wrench ...
	matrix::Vector3f F_b(0.f, 0.f, -19.615f);
	matrix::Vector3f M_b(-1.f, 0.f, 0.f);
	ctrl.applyWrenchLowPass(F_b, M_b, 0.f, 0.004f);
	EXPECT_FLOAT_EQ(M_b(0), -1.f) << "cutoff 0 must pass the command through untouched";

	ctrl.applyWrenchLowPass(F_b, M_b, -1.f, 0.004f);
	EXPECT_FLOAT_EQ(M_b(0), -1.f) << "negative cutoff must bypass too";

	// ... and re-enabling must seed from the live command (-1), not ramp
	// down from the stale state (+1).
	matrix::Vector3f F_c(0.f, 0.f, -19.615f);
	matrix::Vector3f M_c(-1.f, 0.f, 0.f);
	ctrl.applyWrenchLowPass(F_c, M_c, 5.f, 0.004f);
	EXPECT_FLOAT_EQ(M_c(0), -1.f) << "re-enable must re-seed, not step from stale state";
}

// A dt of zero must not divide by zero or freeze the filter; it is a
// bypass like a non-positive cutoff. Run() can see dt == 0 on the first
// cycle after a gate reset.
TEST(FoldrotorWrenchLowPassTest, ZeroDtBypassesInsteadOfDividingByZero)
{
	FoldrotorControl ctrl;

	matrix::Vector3f F(1.f, 2.f, -19.615f);
	matrix::Vector3f M(0.3f, 0.1f, 0.2f);
	ctrl.applyWrenchLowPass(F, M, 5.f, 0.f);

	EXPECT_FLOAT_EQ(F(0), 1.f);
	EXPECT_FLOAT_EQ(M(2), 0.2f);
	EXPECT_TRUE(PX4_ISFINITE(M(0)));
}

// The arm-edge reset (Run()'s _wrench_lp_reset, set beside the integrator
// resets) must seed from the live command. Without it the first armed
// cycle puts a stale, pre-arm wrench on the actuators for one time
// constant -- the same class of defect as the 2026-09-09 arm-time
// integrator spike.
TEST(FoldrotorWrenchLowPassTest, ResetSeedsFromTheLiveCommand)
{
	FoldrotorControl ctrl;

	matrix::Vector3f F(0.f, 0.f, -19.615f);
	matrix::Vector3f M(0.f, 0.f, 0.f);
	const matrix::Vector3f F_pre(0.f, 0.f, -19.615f);
	const matrix::Vector3f M_pre(0.8f, 0.f, 0.f);
	stepWrenchLowPass(ctrl, F, M, F_pre, M_pre, 5.f, 0.004f, 500);
	ASSERT_NEAR(M(0), 0.8f, 1e-4f);

	ctrl.resetWrenchLowPass();

	matrix::Vector3f F_armed(0.f, 0.f, -19.615f);
	matrix::Vector3f M_armed(0.f, 0.f, 0.f);
	ctrl.applyWrenchLowPass(F_armed, M_armed, 5.f, 0.004f);

	EXPECT_FLOAT_EQ(M_armed(0), 0.f) << "reset must discard the pre-arm wrench";
}

// The filter must not change WHICH wrench is asked for, only how fast it
// changes: force and moment share one corner, so a steady wrench that
// fitWrenchToEnvelope() accepts is still accepted after filtering. Guards
// against a future split-corner change silently making the fit's input
// inconsistent with the lever it is handed.
TEST(FoldrotorWrenchLowPassTest, FilteredSteadyWrenchStaysFeasible)
{
	FoldrotorControl ctrl;

	const matrix::Vector3f F_in(0.5f, 0.3f, -15.26f);
	const matrix::Vector3f M_in(0.2f, 0.05f, 0.3f);

	matrix::Vector3f F = F_in;
	matrix::Vector3f M = M_in;
	stepWrenchLowPass(ctrl, F, M, F_in, M_in, 5.f, 0.004f, 500);

	matrix::Vector3f F_alloc = FoldrotorControl::frdToAllocatorFlu(F);
	matrix::Vector3f M_alloc = FoldrotorControl::frdToAllocatorFlu(M);
	EXPECT_FALSE(ctrl.getAllocation().allocate(F_alloc, M_alloc).saturated);
}
