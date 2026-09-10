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
 *     desired force (controller.md "Structure (confirmed from
 *     Simulink)"), at the exact gains in controller_params.md's table.
 *     Every expected value below is derived by hand from those gains
 *     and written out in the comment above its assertion — not read
 *     back off the implementation. Covers the four semantics decisions
 *     recorded in findings.md ("Velocity-loop semantics resolved by
 *     user decision (step 4a)"): FF-as-P, derivative-on-measurement,
 *     conditional-integration anti-windup, and the gravity feedforward
 *     (15.260017 N, the measured hover weight, since the 2026-09-09
 *     under-scaling fix — was 9.81 before). Pure math — no Gazebo, no
 *     uORB.
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
const ExpectedParam kExpectedParams[] = {
	{px4::params::FR_POS_P, 3.0f},

	{px4::params::FR_VEL_XY_FF, 6.0f},
	{px4::params::FR_VEL_XY_I, 1.0f},
	{px4::params::FR_VEL_XY_D, 1.0f},

	{px4::params::FR_VEL_Z_FF, 7.0f},
	{px4::params::FR_VEL_Z_I, 7.0f},
	{px4::params::FR_VEL_Z_D, 0.1f},
	{px4::params::FR_VEL_Z_GRAV_FF, 15.260017f},
	{px4::params::FR_VEL_Z_I_LIM, 3.0f},

	{px4::params::FR_ATT_P, 3.0f},

	{px4::params::FR_RATE_RP_FF, 3.5f},
	{px4::params::FR_RATE_RP_I, 0.1f},
	{px4::params::FR_RATE_RP_D, 0.5f},

	{px4::params::FR_RATE_YAW_FF, 2.5f},
	{px4::params::FR_RATE_YAW_I, 0.0f},
	{px4::params::FR_RATE_YAW_D, 0.0f},
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
// inertial->body rotation on the force path").
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
	// controller.md's explanation of why the missing rotation has not shown
	// up in testing: "Near level (phi,theta,psi ~ 0), inertial ~ body". At
	// exactly level it must be an identity, so inserting this stage cannot
	// change any existing near-level result.
	const matrix::Eulerf level(0.f, 0.f, 0.f);
	const matrix::Vector3f F_i(3.f, -4.f, 5.f);

	const matrix::Vector3f F_b = foldrotor::inertialToBody(F_i, level);

	EXPECT_FLOAT_EQ(F_b(0), F_i(0));
	EXPECT_FLOAT_EQ(F_b(1), F_i(1));
	EXPECT_FLOAT_EQ(F_b(2), F_i(2));

	expectProperRotation(foldrotor::inertialToBodyRotation(level), "level attitude");
}

TEST(FoldrotorControlInertial2BodyTest, IsAProperRotationAtNonTrivialAttitude)
{
	expectProperRotation(foldrotor::inertialToBodyRotation(matrix::Eulerf(0.f, 0.f, 0.f)),
			     "level attitude");
	expectProperRotation(foldrotor::inertialToBodyRotation(matrix::Eulerf(0.4f, -0.3f, 1.1f)),
			     "phi=0.4 theta=-0.3 psi=1.1");
	expectProperRotation(foldrotor::inertialToBodyRotation(matrix::Eulerf(-1.2f, 0.9f, -2.5f)),
			     "phi=-1.2 theta=0.9 psi=-2.5");
}

TEST(FoldrotorControlInertial2BodyTest, PureYawMatchesHandComputedRotation)
{
	// THE regime controller.md says the bug surfaces in: "sustained yaw
	// during translation". At psi = +90 deg, phi = theta = 0:
	//
	//   cpsi = 0, spsi = 1, cth = 1, sth = 0, cphi = 1, sphi = 0
	//
	// substituted into controller.md's Rt gives, by hand:
	//
	//   Rt = [  0  1  0 ]
	//        [ -1  0  0 ]
	//        [  0  0  1 ]
	//
	// so F_i = (3, 4, 5) -> F_b = (4, -3, 5). Derived independently from
	// the spec's matrix, not read back out of the implementation.
	const matrix::Eulerf yawed_90(0.f, 0.f, static_cast<float>(M_PI) / 2.f);
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

	// And the magnitude of the defect this stage fixes: without the
	// rotation, F_i was handed to the allocator as if it were body-frame.
	// At 90 deg of yaw that is a full axis swap, not a small error -- the
	// X and Y commands would be interchanged and one of them sign-flipped.
	EXPECT_GT((F_b - F_i).norm(), 4.f);
}

TEST(FoldrotorControlInertial2BodyTest, PureYaw45MatchesHandComputedRotation)
{
	// psi = 45 deg, phi = theta = 0 -> Rt = [ c s 0; -s c 0; 0 0 1 ]
	// with c = s = sqrt(2)/2, so F_i = (1, 0, 0) -> (sqrt(2)/2, -sqrt(2)/2, 0).
	const float root_half = std::sqrt(2.f) / 2.f;
	const matrix::Eulerf yawed_45(0.f, 0.f, static_cast<float>(M_PI) / 4.f);

	const matrix::Vector3f F_b = foldrotor::inertialToBody(matrix::Vector3f(1.f, 0.f, 0.f), yawed_45);

	EXPECT_NEAR(F_b(0), root_half, 1e-5f);
	EXPECT_NEAR(F_b(1), -root_half, 1e-5f);
	EXPECT_NEAR(F_b(2), 0.f, 1e-5f);
}

TEST(FoldrotorControlInertial2BodyTest, PureRollMatchesHandComputedRotation)
{
	// phi = +90 deg, theta = psi = 0 -> by hand from controller.md's Rt:
	//
	//   Rt = [ 1  0  0 ]
	//        [ 0  0  1 ]
	//        [ 0 -1  0 ]
	//
	// Rolled 90 deg right, the body Y axis points along inertial down, so a
	// pure inertial-down force (0,0,1) becomes +Y in body: (0, 1, 0).
	const matrix::Eulerf rolled_90(static_cast<float>(M_PI) / 2.f, 0.f, 0.f);

	const matrix::Vector3f F_b = foldrotor::inertialToBody(matrix::Vector3f(0.f, 0.f, 1.f), rolled_90);

	EXPECT_NEAR(F_b(0), 0.f, 1e-5f);
	EXPECT_NEAR(F_b(1), 1.f, 1e-5f);
	EXPECT_NEAR(F_b(2), 0.f, 1e-5f);
}

TEST(FoldrotorControlInertial2BodyTest, MatchesPx4DcmTransposeAcrossAttitudes)
{
	// controller.md claims its Rt is the "standard ZYX (yaw-pitch-roll)"
	// inertial->body rotation. PX4's own Dcm(const Euler&) constructor
	// (src/lib/matrix/matrix/Dcm.hpp) builds the standard 3-2-1 intrinsic
	// Tait-Bryan body->inertial DCM, so the claim is equivalent to
	//   Rt == matrix::Dcmf(euler).transpose()
	// across all attitudes. Asserting it here does two jobs: it checks the
	// spec's claim is true rather than assumed, and it proves this stage
	// composes with step 3's _euler = matrix::Eulerf(matrix::Quatf(q))
	// without any convention adaptation -- which is what step 4e relies on.
	const matrix::Eulerf attitudes[] = {
		matrix::Eulerf(0.f, 0.f, 0.f),
		matrix::Eulerf(0.4f, -0.3f, 1.1f),
		matrix::Eulerf(-1.2f, 0.9f, -2.5f),
		matrix::Eulerf(0.f, 0.f, static_cast<float>(M_PI) / 2.f),
		matrix::Eulerf(static_cast<float>(M_PI) / 2.f, 0.f, 0.f),
		matrix::Eulerf(0.15f, 1.4f, 3.0f),
	};

	for (const matrix::Eulerf &euler : attitudes) {
		const matrix::Dcmf Rt = foldrotor::inertialToBodyRotation(euler);
		const matrix::Dcmf expected = matrix::Dcmf(euler).transpose();

		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				EXPECT_NEAR(Rt(i, j), expected(i, j), 1e-5f)
						<< "mismatch at (" << i << "," << j << ") for phi=" << euler.phi()
						<< " theta=" << euler.theta() << " psi=" << euler.psi();
			}
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
//   Fz    = 7 * 0 + 0 (integral) - 0.1 * 0 + 15.260017 = 15.260017
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
	EXPECT_NEAR(F(2), 15.260017f, kTol);

	// And the integral advanced by I * e_v * dt, per axis:
	//   x: 1 *  5.50 * 0.01 =  0.055
	//   y: 1 * -3.25 * 0.01 = -0.0325
	//   z: 7 *  0.00 * 0.01 =  0
	EXPECT_NEAR(ctrl.getIntegral()(0), 0.055f, kTol);
	EXPECT_NEAR(ctrl.getIntegral()(1), -0.0325f, kTol);
	EXPECT_NEAR(ctrl.getIntegral()(2), 0.0f, kTol);
}

// With everything at zero, the only output is the Z gravity feedforward,
// and it appears on Z alone. This pins OPEN ITEM (b): the value is
// positive, i.e. positive-down in NED, exactly as controller_params.md
// records it. Magnitude is now the measured hover weight (15.260017 N,
// resolved 2026-09-09; was 9.81 N before), per OPEN ITEM (a)'s fix. If
// that sign is ever decided to be wrong, this test is the thing that must
// change with it — deliberately, not silently.
TEST(FoldrotorPositionVelocityControlTest, GravityFeedforwardIsLiteralWeightOnZOnly)
{
	auto ctrl = makeSpecDefaultController();

	const matrix::Vector3f F = ctrl.update(matrix::Vector3f(), matrix::Vector3f(),
					       matrix::Vector3f(), matrix::Vector3f(), 0.01f);

	EXPECT_NEAR(F(0), 0.0f, kTol);
	EXPECT_NEAR(F(1), 0.0f, kTol);
	EXPECT_NEAR(F(2), 15.260017f, kTol);
}

// Derivative acts on the measured velocity derivative, negated:
//   Fx = -1.0 * 1 = -1.0
//   Fy = -1.0 * 2 = -2.0
//   Fz = -0.1 * 3 + 15.260017 = 14.960017
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
	EXPECT_NEAR(F(2), 14.960017f, kTol);
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
// the gravity feedforward riding along, dt = 0.1, e_v = 3:
//   call 1: Fz = 7*3 + 0.0 + 15.260017 = 36.260017 ; integral -> 7*3*0.1 = 2.1
//   call 2: Fz = 7*3 + 2.1 + 15.260017 = 38.360017 ; integral -> 4.2
//   call 3: Fz = 7*3 + 4.2 + 15.260017 = 40.460017 ; integral -> 6.3
// (Integral accumulation itself does not depend on the feedforward value.)
TEST(FoldrotorPositionVelocityControlTest, IntegralAccumulatesAtZGainWithGravityFeedforward)
{
	auto ctrl = makeSpecDefaultController();

	const matrix::Vector3f pos_sp(0.f, 0.f, 1.f);
	const matrix::Vector3f zero;

	EXPECT_NEAR(ctrl.update(zero, pos_sp, zero, zero, 0.1f)(2), 36.260017f, kTol);
	EXPECT_NEAR(ctrl.update(zero, pos_sp, zero, zero, 0.1f)(2), 38.360017f, kTol);
	EXPECT_NEAR(ctrl.update(zero, pos_sp, zero, zero, 0.1f)(2), 40.460017f, kTol);
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
	const matrix::Vector3f F = ctrl.update(zero, zero, zero, zero, 0.1f);
	EXPECT_NEAR(F(0), 0.0f, kTol);
	EXPECT_NEAR(F(2), 15.260017f, kTol);
}

// Default limits are +/-infinity, so nothing clamps and no anti-windup
// engages. This is the runtime state until step 4d supplies real bounds,
// and it is asserted here so that fact is visible rather than assumed.
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

// Conditional integration, X axis. Limits +/-20 N; e_v = 6 gives an
// unclamped Fx of 36, so the output saturates high while the error is
// still positive -> the integrator must be frozen at zero for every one
// of the five steps.
//
// The integral is then read out two ways: directly, and by re-running
// with the limits reopened and zero error, where the output IS the
// integral (plus the Z gravity term). Contrast with
// DefaultLimitsNeitherClampNorFreeze, which runs the identical sequence
// unbounded and reaches 3.0.
TEST(FoldrotorPositionVelocityControlTest, IntegratorFreezesWhileSaturatedX)
{
	auto ctrl = makeSpecDefaultController();
	ctrl.setOutputLimits(matrix::Vector3f(-20.f, -20.f, -20.f),
			     matrix::Vector3f(20.f, 20.f, 20.f));

	const matrix::Vector3f zero;
	const matrix::Vector3f pos_sp(2.f, 0.f, 0.f);

	for (int i = 0; i < 5; i++) {
		// Output is clamped to the limit, not the unclamped 36.
		EXPECT_NEAR(ctrl.update(zero, pos_sp, zero, zero, 0.1f)(0), 20.0f, kTol);
	}

	EXPECT_NEAR(ctrl.getIntegral()(0), 0.0f, kTol);

	ctrl.setOutputLimits(matrix::Vector3f(-1e6f, -1e6f, -1e6f),
			     matrix::Vector3f(1e6f, 1e6f, 1e6f));
	const matrix::Vector3f probe = ctrl.update(zero, zero, zero, zero, 0.1f);
	EXPECT_NEAR(probe(0), 0.0f, kTol);
}

// Same mechanism on Z, where FR_VEL_Z_I = 7 makes windup the real
// concern. e_v = 3 gives an unclamped Fz of 7*3 + 15.260017 = 36.260017,
// above the 20 N limit, so the integrator freezes at zero. Unbounded, the
// same five steps would reach 7 * 3 * 0.1 * 5 = 10.5.
TEST(FoldrotorPositionVelocityControlTest, IntegratorFreezesWhileSaturatedZ)
{
	auto ctrl = makeSpecDefaultController();
	ctrl.setOutputLimits(matrix::Vector3f(-20.f, -20.f, -20.f),
			     matrix::Vector3f(20.f, 20.f, 20.f));

	const matrix::Vector3f zero;
	const matrix::Vector3f pos_sp(0.f, 0.f, 1.f);

	for (int i = 0; i < 5; i++) {
		EXPECT_NEAR(ctrl.update(zero, pos_sp, zero, zero, 0.1f)(2), 20.0f, kTol);
	}

	EXPECT_NEAR(ctrl.getIntegral()(2), 0.0f, kTol);

	// Reopen and probe with zero error: Fz is then integral + gravity.
	// Frozen  -> 0    + 15.260017 = 15.260017
	// Unfrozen would have been 10.5 + 15.260017 = 25.760017.
	ctrl.setOutputLimits(matrix::Vector3f(-1e6f, -1e6f, -1e6f),
			     matrix::Vector3f(1e6f, 1e6f, 1e6f));
	EXPECT_NEAR(ctrl.update(zero, zero, zero, zero, 0.1f)(2), 15.260017f, kTol);
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

// The freeze must be directional, not a blanket hold: an error pushing
// the output back OUT of saturation still has to integrate, otherwise
// the integrator latches. Asymmetric limits isolate this.
//
//   step 1, pos_sp = +2: e_v = +6, Fx = 36 -> clamped to the +20 upper
//           limit with a positive error, so frozen -> integral stays 0.
//   step 2, pos_sp = -2: e_v = -6, Fx = -36, well inside the -1000 lower
//           limit, so NOT saturated -> integral moves by 1*(-6)*0.1 = -0.6.
TEST(FoldrotorPositionVelocityControlTest, IntegratorStillMovesOutOfSaturation)
{
	auto ctrl = makeSpecDefaultController();
	ctrl.setOutputLimits(matrix::Vector3f(-1000.f, -1000.f, -1000.f),
			     matrix::Vector3f(20.f, 20.f, 20.f));

	const matrix::Vector3f zero;

	EXPECT_NEAR(ctrl.update(zero, matrix::Vector3f(2.f, 0.f, 0.f), zero, zero, 0.1f)(0),
		    20.0f, kTol);
	EXPECT_NEAR(ctrl.getIntegral()(0), 0.0f, kTol);

	EXPECT_NEAR(ctrl.update(zero, matrix::Vector3f(-2.f, 0.f, 0.f), zero, zero, 0.1f)(0),
		    -36.0f, kTol);
	EXPECT_NEAR(ctrl.getIntegral()(0), -0.6f, kTol);
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

	const matrix::Eulerf level(0.f, 0.f, 0.f);
	const matrix::Vector3f F_b = foldrotor::inertialToBody(F_i, level);

	EXPECT_NEAR(F_b(0), 33.0f, kTol);
	EXPECT_NEAR(F_b(1), -19.5f, kTol);
	EXPECT_NEAR(F_b(2), 15.260017f, kTol);
}

/****************************************************************************
 * Step 4c — AttitudeRateControl.
 *
 * Gains throughout are exactly controller_params.md's table:
 *   FR_ATT_P            = 3.0   (all axes)
 *   FR_RATE_RP_FF/I/D   = 3.5 / 0.1 / 0.5   (roll and pitch)
 *   FR_RATE_YAW_FF/I/D  = 2.5 / 0.0 / 0.0   (yaw)
 * with FR_RATE_*_FF read as the P gain on the rate error, per the
 * decision recorded in findings.md.
 ****************************************************************************/

namespace
{

foldrotor::AttitudeRateControl makeSpecDefaultAttitudeController()
{
	foldrotor::AttitudeRateControl ctrl;
	ctrl.setAttitudeGain(3.0f);
	ctrl.setRateGains(3.5f, 0.1f, 0.5f,   // FR_RATE_RP_FF  / _I / _D
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
	auto ctrl = makeSpecDefaultAttitudeController();

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
	auto ctrl = makeSpecDefaultAttitudeController();

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
	auto ctrl = makeSpecDefaultAttitudeController();

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
	auto ctrl = makeSpecDefaultAttitudeController();

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
	auto ctrl = makeSpecDefaultAttitudeController();

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
	auto ctrl = makeSpecDefaultAttitudeController();

	ctrl.update(kLevel, matrix::Eulerf(2.0f, 0.f, 0.f), kZero3, kZero3, 0.1f);

	EXPECT_NEAR(ctrl.getIntegral()(0), 0.015682114f, 1e-6f);
}

// And the i_factor floors at zero: past a rate error of radians(400),
// 1 - (e/scale)^2 goes negative and is clamped, so integration stops
// entirely. att_error = 2.5 -> rate_sp = 7.5 > 6.98131700 = radians(400).
TEST(FoldrotorAttitudeRateControlTest, IFactorFloorsAtZeroBeyondScale)
{
	auto ctrl = makeSpecDefaultAttitudeController();

	ctrl.update(kLevel, matrix::Eulerf(2.5f, 0.f, 0.f), kZero3, kZero3, 0.1f);

	EXPECT_NEAR(ctrl.getIntegral()(0), 0.0f, 1e-9f);
}

// The landed gate (rate_control.cpp:81-83): while landed the integral is
// frozen entirely, independently of saturation. Same setup as the
// accumulation test above, which reaches 0.0059556821 per step.
TEST(FoldrotorAttitudeRateControlTest, LandedFreezesIntegralEntirely)
{
	auto ctrl = makeSpecDefaultAttitudeController();

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
	auto ctrl = makeSpecDefaultAttitudeController();
	ctrl.setIntegratorLimit(matrix::Vector3f(0.001f, 0.001f, 0.001f));

	const matrix::Eulerf euler_sp(0.2f, 0.f, 0.f);

	for (int i = 0; i < 5; i++) {
		ctrl.update(kLevel, euler_sp, kZero3, kZero3, 0.1f);
	}

	EXPECT_NEAR(ctrl.getIntegral()(0), 0.001f, 1e-7f);
}

TEST(FoldrotorAttitudeRateControlTest, ResetIntegralClearsAccumulatedState)
{
	auto ctrl = makeSpecDefaultAttitudeController();

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
	auto ctrl = makeSpecDefaultAttitudeController();

	const matrix::Eulerf euler_sp(0.2f, 0.f, 0.f);

	for (int i = 0; i < 3; i++) {
		ctrl.update(kLevel, euler_sp, kZero3, kZero3, 0.1f);
	}

	// Three unclamped increments of 0.0059556821.
	EXPECT_NEAR(ctrl.getIntegral()(0), 0.017867046f, 1e-6f);
}

// Conditional integration. Limits +/-1.0 N*m; e_r = 0.6 gives an
// unclamped Mx of 2.1, so the axis saturates high while the error is
// still positive -> min(0.6, 0) = 0 -> the integrator must not move,
// for any number of steps. Contrast with
// DefaultLimitsNeitherClampNorFreeze, which runs the identical sequence
// unbounded and reaches 0.017867046.
TEST(FoldrotorAttitudeRateControlTest, IntegratorFreezesWhileSaturated)
{
	auto ctrl = makeSpecDefaultAttitudeController();
	ctrl.setOutputLimits(matrix::Vector3f(-1.f, -1.f, -1.f),
			     matrix::Vector3f(1.f, 1.f, 1.f));

	const matrix::Eulerf euler_sp(0.2f, 0.f, 0.f);

	for (int i = 0; i < 5; i++) {
		// Output is clamped to the limit, not the unclamped 2.1.
		EXPECT_NEAR(ctrl.update(kLevel, euler_sp, kZero3, kZero3, 0.1f)(0), 1.0f, 1e-6f);
	}

	EXPECT_NEAR(ctrl.getIntegral()(0), 0.0f, 1e-9f);
}

// The freeze must be directional: an error driving the axis back OUT of
// saturation still integrates, otherwise the integrator latches.
// Asymmetric limits isolate this.
//
//   step 1, euler_sp = +0.2: e_r = +0.6, Mx = 2.1 -> clamped to the +1.0
//           upper limit with a positive error -> min(0.6, 0) = 0, frozen.
//   step 2, euler_sp = -0.2: e_r = -0.6, Mx = -2.1, well inside the
//           -1000 lower limit -> not saturated -> integrates by
//           i_factor * 0.1 * (-0.6) * 0.1 = -0.0059556821.
TEST(FoldrotorAttitudeRateControlTest, IntegratorStillMovesOutOfSaturation)
{
	auto ctrl = makeSpecDefaultAttitudeController();
	ctrl.setOutputLimits(matrix::Vector3f(-1000.f, -1000.f, -1000.f),
			     matrix::Vector3f(1.f, 1.f, 1.f));

	EXPECT_NEAR(ctrl.update(kLevel, matrix::Eulerf(0.2f, 0.f, 0.f), kZero3, kZero3, 0.1f)(0),
		    1.0f, 1e-6f);
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
	auto ctrl = makeSpecDefaultAttitudeController();

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
	auto ctrl = makeSpecDefaultAttitudeController();

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
	auto ctrl = makeSpecDefaultAttitudeController();

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
	EXPECT_NEAR(dt, 0.f, 1e-9f);
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
	auto ctrl = makeSpecDefaultAttitudeController();

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
	auto ctrl = makeSpecDefaultAttitudeController();

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
	auto ctrl_split = makeSpecDefaultAttitudeController();
	auto ctrl_combined = makeSpecDefaultAttitudeController();

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

// allocation.md's Minv literal, recomputed 2026-09-09 for the
// SDF-verified geometry (s1y/s2y = +-0.2684 m, s1z/s2z = +0.0301 m --
// see FoldrotorAllocation.hpp OPEN ITEM (b), was +-0.15 m / +0.02 m).
// Standalone numpy inverse of the corrected M0, same as the original
// literal's provenance; allocation.md's "The matrices" section carries
// the same value, marked as superseding the old one.
static const float kSpecMinv[6][6] = {
	{ +0.5000000000f, +0.0035373791f,  0.0000000000f, +0.1175209007f,  0.0000000000f, -1.8554476330f },
	{ -0.8852941176f, +0.5000000000f,  0.0000000000f,  0.0000000000f, +29.4117647059f,  0.0000000000f },
	{  0.0000000000f, +0.0558489738f, +0.5000000000f, +1.8554476330f,  0.0000000000f, +0.1175209007f },
	{ +0.5000000000f, -0.0035373791f,  0.0000000000f, -0.1175209007f,  0.0000000000f, +1.8554476330f },
	{ +0.8852941176f, +0.5000000000f,  0.0000000000f,  0.0000000000f, -29.4117647059f,  0.0000000000f },
	{  0.0000000000f, -0.0558489738f, +0.5000000000f, -1.8554476330f,  0.0000000000f, -0.1175209007f },
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

// Hover, Fz = +15.26 N (Z-up positive per allocation.md's thrust vector
// -- see FoldrotorAllocation.hpp OPEN ITEM (c), NOT PX4 body FRD). Hand
// solved: with Fx=Fy=Mx=My=Mz=0, M0's structure splits Fz evenly and
// both moment-arm terms vanish, so F1=F2=Fz/2=7.63 N, alpha=beta=0 on
// both rotors, unsaturated.
TEST(FoldrotorAllocationTest, HoverProducesEvenSplitZeroTilt)
{
	foldrotor::FoldrotorAllocation alloc;
	const auto out = alloc.allocate(matrix::Vector3f(0.f, 0.f, 15.26f), matrix::Vector3f(0.f, 0.f, 0.f));

	EXPECT_NEAR(out.F1, 7.63f, 1e-4f);
	EXPECT_NEAR(out.F2, 7.63f, 1e-4f);
	EXPECT_NEAR(out.alpha1, 0.f, 1e-6f);
	EXPECT_NEAR(out.alpha2, 0.f, 1e-6f);
	EXPECT_NEAR(out.beta1, 0.f, 1e-5f);
	EXPECT_NEAR(out.beta2, 0.f, 1e-5f);
	EXPECT_FALSE(out.saturated);
}

// Hover + Mx = 0.5 N*m. Hand solved via Minv*w (both rotors' Ty stays 0,
// so alpha is unaffected). Recomputed 2026-09-09 for the SDF-verified
// geometry (FoldrotorAllocation.hpp OPEN ITEM (b) -- was F1=9.277405 N,
// F2=5.987374 N, beta1=+0.020103 rad, beta2=-0.031153 rad against the
// old s_y=+-0.15/s_z=+0.02 geometry): F1=8.557926 N, F2=6.702534 N,
// beta1=+0.006866 rad, beta2=-0.008767 rad.
TEST(FoldrotorAllocationTest, HoverPlusRollMomentMatchesHandSolved)
{
	foldrotor::FoldrotorAllocation alloc;
	const auto out = alloc.allocate(matrix::Vector3f(0.f, 0.f, 15.26f), matrix::Vector3f(0.5f, 0.f, 0.f));

	EXPECT_NEAR(out.F1, 8.557926f, 1e-4f);
	EXPECT_NEAR(out.F2, 6.702534f, 1e-4f);
	EXPECT_NEAR(out.beta1, 0.006866f, 1e-5f);
	EXPECT_NEAR(out.beta2, -0.008767f, 1e-5f);
	EXPECT_FALSE(out.saturated);
}

// Hover + Mz = 0.2 N*m. Recomputed 2026-09-09 for the SDF-verified
// geometry (was F1=7.732662 N, F2=7.584020 N, beta1=-0.085224 rad,
// beta2=+0.086899 rad against the old geometry): F1=7.662495 N,
// F2=7.615542 N, beta1=-0.048448 rad, beta2=+0.048747 rad.
TEST(FoldrotorAllocationTest, HoverPlusYawMomentMatchesHandSolved)
{
	foldrotor::FoldrotorAllocation alloc;
	const auto out = alloc.allocate(matrix::Vector3f(0.f, 0.f, 15.26f), matrix::Vector3f(0.f, 0.f, 0.2f));

	EXPECT_NEAR(out.F1, 7.662495f, 1e-4f);
	EXPECT_NEAR(out.F2, 7.615542f, 1e-4f);
	EXPECT_NEAR(out.beta1, -0.048448f, 1e-5f);
	EXPECT_NEAR(out.beta2, 0.048747f, 1e-5f);
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

		ASSERT_NEAR(out.alpha1, 0.f, 1e-6f);
		ASSERT_NEAR(out.alpha2, 0.f, 1e-6f);

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
	EXPECT_NEAR(out.beta1, 0.f, 1e-5f);
	EXPECT_NEAR(out.beta2, 0.f, 1e-5f);
	EXPECT_TRUE(out.saturated);
}

// F_b=(40,0,30), M_b=(0,0.8,0). Recomputed 2026-09-09 for the
// SDF-verified geometry (FoldrotorAllocation.hpp OPEN ITEM (b)): unlike
// the old s_y=+-0.15 geometry, T1's Ty no longer cancels to exactly 0 --
// hand-solved via Minv*w gives T1=(20, -11.882353, 15), T2=(20,
// 11.882353, 15), so unclamped F=27.680143 N (>15, saturates) and
// unclamped beta1=atan2(20,hypot(-11.882353,15))=0.807469 rad (46.26
// deg), clamped to +kMaxTilt=+0.79 rad -- the clamp still triggers
// either way, so the assertions below are unchanged by the geometry fix.
TEST(FoldrotorAllocationTest, TiltClampsAtPositivePointSevenNine)
{
	foldrotor::FoldrotorAllocation alloc;
	const auto out = alloc.allocate(matrix::Vector3f(40.f, 0.f, 30.f), matrix::Vector3f(0.f, 0.8f, 0.f));

	EXPECT_NEAR(out.beta1, 0.79f, 1e-5f);
	EXPECT_NEAR(out.beta2, 0.79f, 1e-5f);
	EXPECT_TRUE(out.saturated);
}

// Mirror of the above: F_b=(-40,0,30), M_b=(0,-0.8,0) -> T1=(-20,
// 11.882353, 15), T2=(-20, -11.882353, 15), unclamped
// beta1=-0.807469 rad, clamped to -kMaxTilt=-0.79 rad.
TEST(FoldrotorAllocationTest, TiltClampsAtNegativePointSevenNine)
{
	foldrotor::FoldrotorAllocation alloc;
	const auto out = alloc.allocate(matrix::Vector3f(-40.f, 0.f, 30.f), matrix::Vector3f(0.f, -0.8f, 0.f));

	EXPECT_NEAR(out.beta1, -0.79f, 1e-5f);
	EXPECT_NEAR(out.beta2, -0.79f, 1e-5f);
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
// comment): alpha1=-0.548187 rad, alpha2=-0.615450 rad -- negative,
// matching alpha=atan2(-Ty,Tz) for the positive Ty this Fy produces on
// both rotors (Control_Alloc's own +alpha -> -Ty convention).
TEST(FoldrotorAllocationTest, PureLateralForceProducesNonzeroAlphaCorrectSign)
{
	foldrotor::FoldrotorAllocation alloc;
	const auto out = alloc.allocate(matrix::Vector3f(0.f, 10.f, 15.26f), matrix::Vector3f(0.f, 0.f, 0.f));

	EXPECT_NEAR(out.alpha1, -0.548187f, 1e-5f);
	EXPECT_NEAR(out.alpha2, -0.615450f, 1e-5f);
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
// style). Recomputed 2026-09-09 for the SDF-verified geometry
// (FoldrotorAllocation.hpp OPEN ITEM (b) -- was F1=9.399402 N,
// F2=8.848980 N, beta1=0.003968 rad, beta2=-0.004215 rad against the old
// s_y=+-0.15/s_z=+0.02 geometry): F_alloc = (0, -10, 15.26), M_alloc =
// (0.3, -0, -0) after the transform; allocate() through the corrected
// Minv gives T1 = (-0.000118, -5, 7.628145), T2 = (0.000118, -5,
// 7.631855), so
//   F1 = 9.120778 N,  F2 = 9.123882 N
//   beta1 = -0.000013 rad,  beta2 = 0.000013 rad
// A bug that flips only Z (leaves Y unflipped) produces a materially
// different result against this same corrected geometry -- F1 =
// 10.073837 N, F2 = 8.212710 N, beta1 = 0.007011 rad,
// beta2 = -0.008600 rad, hand-checked the same way -- so this test still
// catches that specific partial-flip bug, not just a totally-missing
// transform, even though beta1/beta2 are individually much smaller now
// than under the old geometry.
TEST(FoldrotorAllocationTest, LateralFrdWrenchAllocatesConsistentlyWithFullFlip)
{
	foldrotor::FoldrotorAllocation alloc;

	const matrix::Vector3f F_b(0.f, 10.f, -15.26f);
	const matrix::Vector3f M_b(0.3f, 0.f, 0.f);

	const matrix::Vector3f F_alloc = FoldrotorControl::frdToAllocatorFlu(F_b);
	const matrix::Vector3f M_alloc = FoldrotorControl::frdToAllocatorFlu(M_b);

	const auto out = alloc.allocate(F_alloc, M_alloc);

	EXPECT_NEAR(out.F1, 9.120778f, 1e-4f);
	EXPECT_NEAR(out.F2, 9.123882f, 1e-4f);
	EXPECT_NEAR(out.beta1, -0.000013f, 1e-5f);
	EXPECT_NEAR(out.beta2, 0.000013f, 1e-5f);
}
