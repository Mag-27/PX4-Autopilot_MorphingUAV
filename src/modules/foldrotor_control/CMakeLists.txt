############################################################################
#
# foldrotor_control — standalone position/velocity/attitude/rate/allocation
# module for the foldrotor3 fully-actuated bi-rotor vehicle.
#
# Step 2 of the implementation plan (see .claude/plans in the associated
# Cowork session): module skeleton only. No control law, no params yet —
# those land in steps 3-5. This file will grow a MODULE_CONFIG entry and
# additional DEPENDS (the PositionVelocityControl / AttitudeRateControl /
# FoldrotorAllocation math libraries) as those land.
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
	DEPENDS
		mathlib
		px4_work_queue
	)
