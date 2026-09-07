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
 *     conditional-integration anti-windup, and the literal +9.81
 *     gravity feedforward. Pure math — no Gazebo, no uORB.
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
 ****************************************************************************/

#include "Inertial2Body.hpp"
#include "PositionVelocityControl.hpp"
#include "AttitudeRateControl.hpp"
#include "CascadeRateGate.hpp"

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
	{px4::params::FR_VEL_Z_GRAV_FF, 9.81f},

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
 *   FR_VEL_Z_GRAV_FF = 9.81
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
	ctrl.setGravityFeedforward(9.81f);
	return ctrl;
}

constexpr float kTol = 1e-4f;

} // namespace

// Nominal case, X and Y, hand-computed:
//   v_sp  = 3 * (pos_sp - pos) = 3 * (2, -1, 0)   = (6, -3, 0)
//   e_v   = v_sp - vel = (6-0.5, -3-0.25, 0-0)    = (5.5, -3.25, 0)
//   Fx    = 6 * 5.5                               =  33.0
//   Fy    = 6 * (-3.25)                           = -19.5
//   Fz    = 7 * 0 + 0 (integral) - 0.1 * 0 + 9.81 =   9.81
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
	EXPECT_NEAR(F(2), 9.81f, kTol);

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
// +9.81, i.e. positive-down in NED, exactly as controller_params.md
// records it. If that sign is ever decided to be wrong, this test is the
// thing that must change with it — deliberately, not silently.
TEST(FoldrotorPositionVelocityControlTest, GravityFeedforwardIsLiteralPlus981OnZOnly)
{
	auto ctrl = makeSpecDefaultController();

	const matrix::Vector3f F = ctrl.update(matrix::Vector3f(), matrix::Vector3f(),
					       matrix::Vector3f(), matrix::Vector3f(), 0.01f);

	EXPECT_NEAR(F(0), 0.0f, kTol);
	EXPECT_NEAR(F(1), 0.0f, kTol);
	EXPECT_NEAR(F(2), 9.81f, kTol);
}

// Derivative acts on the measured velocity derivative, negated:
//   Fx = -1.0 * 1 = -1.0
//   Fy = -1.0 * 2 = -2.0
//   Fz = -0.1 * 3 + 9.81 = 9.51
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
	EXPECT_NEAR(F(2), 9.51f, kTol);
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
//   call 1: Fz = 7*3 + 0.0 + 9.81 = 30.81 ; integral -> 7*3*0.1 = 2.1
//   call 2: Fz = 7*3 + 2.1 + 9.81 = 32.91 ; integral -> 4.2
//   call 3: Fz = 7*3 + 4.2 + 9.81 = 35.01 ; integral -> 6.3
TEST(FoldrotorPositionVelocityControlTest, IntegralAccumulatesAtZGainWithGravityFeedforward)
{
	auto ctrl = makeSpecDefaultController();

	const matrix::Vector3f pos_sp(0.f, 0.f, 1.f);
	const matrix::Vector3f zero;

	EXPECT_NEAR(ctrl.update(zero, pos_sp, zero, zero, 0.1f)(2), 30.81f, kTol);
	EXPECT_NEAR(ctrl.update(zero, pos_sp, zero, zero, 0.1f)(2), 32.91f, kTol);
	EXPECT_NEAR(ctrl.update(zero, pos_sp, zero, zero, 0.1f)(2), 35.01f, kTol);
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
	EXPECT_NEAR(F(2), 9.81f, kTol);
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
// concern. e_v = 3 gives an unclamped Fz of 7*3 + 9.81 = 30.81, above
// the 20 N limit, so the integrator freezes at zero. Unbounded, the same
// five steps would reach 7 * 3 * 0.1 * 5 = 10.5.
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
	// Frozen  -> 0    + 9.81 = 9.81
	// Unfrozen would have been 10.5 + 9.81 = 20.31.
	ctrl.setOutputLimits(matrix::Vector3f(-1e6f, -1e6f, -1e6f),
			     matrix::Vector3f(1e6f, 1e6f, 1e6f));
	EXPECT_NEAR(ctrl.update(zero, zero, zero, zero, 0.1f)(2), 9.81f, kTol);
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
	EXPECT_NEAR(F_b(2), 9.81f, kTol);
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
