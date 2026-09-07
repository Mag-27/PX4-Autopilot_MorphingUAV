############################################################################
#
# foldrotor_control — standalone position/velocity/attitude/rate/allocation
# module for the foldrotor3 fully-actuated bi-rotor vehicle.
#
# Step 3 of the implementation plan (.claude/specs/controller_params.md):
# position/velocity/attitude/rate gains are now declared as params and
# loaded via parameters_updated(); the quaternion->Euler conversion is
# wired in.
#
# Step 4b adds Inertial2Body.hpp, the inertial->body force-path rotation
# (controller.md "Identified issues (confirmed) -> 1"). Step 4a adds
# PositionVelocityControl.hpp, the position P -> velocity PID -> inertial
# desired force stages. Step 4c adds AttitudeRateControl.hpp, the
# attitude P -> rate PID -> body moment stages. All three are header-only
# pure math, listed in SRCS for visibility only — no DEPENDS beyond the
# mathlib already present. Step 4e part 1 adds CascadeRateGate.hpp (the
# multi-rate gating mechanism) and wires all four into Run(), computing a
# real wrench every cycle -- but publishes it nowhere. Allocation
# (FoldrotorAllocation, 4d) and the actuator_motors/actuator_servos
# publish are still to come.
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
		Inertial2Body.hpp
		PositionVelocityControl.hpp
		AttitudeRateControl.hpp
		CascadeRateGate.hpp
	MODULE_CONFIG
		foldrotor_control_params.yaml
	DEPENDS
		mathlib
		px4_work_queue
	)

px4_add_functional_gtest(SRC FoldrotorControlTest.cpp LINKLIBS modules__foldrotor_control)
