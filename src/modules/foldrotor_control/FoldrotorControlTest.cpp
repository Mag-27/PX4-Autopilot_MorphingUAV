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
 ****************************************************************************/

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
