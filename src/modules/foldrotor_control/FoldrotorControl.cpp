/****************************************************************************
 *
 * foldrotor_control — see FoldrotorControl.hpp for scope/status.
 *
 ****************************************************************************/

#include "FoldrotorControl.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/math/Limits.hpp>

using namespace time_literals;

ModuleBase::Descriptor FoldrotorControl::desc{task_spawn, custom_command, print_usage};

FoldrotorControl::FoldrotorControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
}

FoldrotorControl::~FoldrotorControl()
{
	perf_free(_loop_perf);
}

bool
FoldrotorControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void
FoldrotorControl::parameters_updated()
{
	// No params yet — step 3 adds the position/velocity/attitude/rate gains
	// from .claude/specs/controller.md here.
}

void
FoldrotorControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup(desc);
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed.
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_updated();
	}

	vehicle_angular_velocity_s angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

		const hrt_abstime now = angular_velocity.timestamp_sample;

		// Guard against too small/too large dt, same bounds as mc_rate_control
		// (reference/px4-module-patterns.md item 2) — not yet used for any
		// control law, just proving the timing plumbing is sane.
		const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);
		_last_run = now;
		(void)dt;

		// Read (but do not yet act on) the other inputs this module will need.
		vehicle_local_position_s local_position{};
		_vehicle_local_position_sub.copy(&local_position);

		vehicle_attitude_s attitude{};
		_vehicle_attitude_sub.copy(&attitude);

		trajectory_setpoint_s trajectory_setpoint{};
		_trajectory_setpoint_sub.copy(&trajectory_setpoint);

		_vehicle_control_mode_sub.copy(&_vehicle_control_mode);

		// Skeleton heartbeat only — no actuator output yet. Rate-limited to
		// 1 Hz so this is useful in the SITL shell without flooding it.
		if (now - _last_heartbeat_log > 1_s) {
			_last_heartbeat_log = now;
			PX4_INFO("foldrotor_control alive — armed=%d offboard=%d position_ctrl=%d",
				 _vehicle_control_mode.flag_armed,
				 _vehicle_control_mode.flag_control_offboard_enabled,
				 _vehicle_control_mode.flag_control_position_enabled);
		}
	}

	perf_end(_loop_perf);
}

int FoldrotorControl::task_spawn(int argc, char *argv[])
{
	FoldrotorControl *instance = new FoldrotorControl();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

int FoldrotorControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FoldrotorControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Standalone position/velocity/attitude/rate/allocation controller for the
foldrotor3 fully-actuated bi-rotor vehicle. Replaces mc_pos_control,
mc_att_control, mc_rate_control, and control_allocator for this vehicle
only — those modules are not started for this airframe, and are not
modified by this module's existence.

Status: skeleton only (step 2 of the implementation plan). Subscribes to
its required inputs and logs a 1 Hz heartbeat; does not yet compute or
publish any actuator command.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("foldrotor_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int foldrotor_control_main(int argc, char *argv[])
{
	return ModuleBase::main(FoldrotorControl::desc, argc, argv);
}
