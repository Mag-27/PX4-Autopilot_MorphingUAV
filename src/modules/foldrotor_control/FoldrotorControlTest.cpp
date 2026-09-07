/****************************************************************************
 *
 * foldrotor_control — step 3 test: FR_* param registration and defaults
 * match .claude/specs/controller_params.md's table. Pure param-registry
 * check (same pattern as src/lib/parameters/ParameterTest.cpp) — no
 * module instantiation, no work queue, no Gazebo.
 *
 ****************************************************************************/

#include <gtest/gtest.h>

#include <px4_platform_common/module_params.h>

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
