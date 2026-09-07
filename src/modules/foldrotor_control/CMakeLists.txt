############################################################################
#
# foldrotor_control — standalone position/velocity/attitude/rate/allocation
# module for the foldrotor3 fully-actuated bi-rotor vehicle.
#
# Step 3 of the implementation plan (.claude/specs/controller_params.md):
# position/velocity/attitude/rate gains are now declared as params and
# loaded via parameters_updated(); the quaternion->Euler conversion is
# wired in. No control law yet — that, plus the PositionVelocityControl /
# AttitudeRateControl / FoldrotorAllocation math libraries and their
# DEPENDS, land in step 4.
#
############################################################################

px4_add_module(
	MODULE modules__foldrotor_control
	MAIN foldrotor_control
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		FoldrotorControl.cpp
		FoldrotorControl.hpp
	MODULE_CONFIG
		foldrotor_control_params.yaml
	DEPENDS
		mathlib
		px4_work_queue
	)

px4_add_functional_gtest(SRC FoldrotorControlTest.cpp LINKLIBS modules__foldrotor_control)
