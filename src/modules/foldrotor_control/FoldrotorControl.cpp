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
	parameters_updated();
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
	// Per reference/px4-module-patterns.md item 4: raw _param_fr_* values
	// are read only here, never from hot control-law code (which doesn't
	// exist yet — step 4 will read _gains instead).
	_gains.pos_p = _param_fr_pos_p.get();

	_gains.vel_xy_ff = _param_fr_vel_xy_ff.get();
	_gains.vel_xy_i = _param_fr_vel_xy_i.get();
	_gains.vel_xy_d = _param_fr_vel_xy_d.get();

	_gains.vel_z_ff = _param_fr_vel_z_ff.get();
	_gains.vel_z_i = _param_fr_vel_z_i.get();
	_gains.vel_z_d = _param_fr_vel_z_d.get();
	_gains.vel_z_grav_ff = _param_fr_vel_z_grav_ff.get();

	_gains.att_p = _param_fr_att_p.get();

	_gains.rate_rp_ff = _param_fr_rate_rp_ff.get();
	_gains.rate_rp_i = _param_fr_rate_rp_i.get();
	_gains.rate_rp_d = _param_fr_rate_rp_d.get();

	_gains.rate_yaw_ff = _param_fr_rate_yaw_ff.get();
	_gains.rate_yaw_i = _param_fr_rate_yaw_i.get();
	_gains.rate_yaw_d = _param_fr_rate_yaw_d.get();

	// Push into the cascade objects. Output/integrator limits are left at
	// their +/-infinity defaults -- controller.md and controller_params.md
	// are both explicit that inventing bounds records an assumption as a
	// constraint; the real values come from the allocator (4d), not from
	// here.
	_pos_vel_control.setPositionGain(_gains.pos_p);
	_pos_vel_control.setVelocityGains(_gains.vel_xy_ff, _gains.vel_xy_i, _gains.vel_xy_d,
					  _gains.vel_z_ff, _gains.vel_z_i, _gains.vel_z_d);
	_pos_vel_control.setGravityFeedforward(_gains.vel_z_grav_ff);

	_att_rate_control.setAttitudeGain(_gains.att_p);
	_att_rate_control.setRateGains(_gains.rate_rp_ff, _gains.rate_rp_i, _gains.rate_rp_d,
				       _gains.rate_yaw_ff, _gains.rate_yaw_i, _gains.rate_yaw_d);
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
		// (reference/px4-module-patterns.md item 2).
		const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);
		_last_run = now;

		vehicle_local_position_s local_position{};
		_vehicle_local_position_sub.copy(&local_position);

		vehicle_attitude_s attitude{};
		_vehicle_attitude_sub.copy(&attitude);

		// vehicle_attitude.q is FRD-body->NED (VehicleAttitude.msg); the
		// cascade math (controller.md) is written in phi/theta/psi, so
		// convert here -- see controller.md Interface. No custom
		// conversion math: matrix::Eulerf is PX4's own 3-2-1 intrinsic
		// Tait-Bryan utility, the same convention Inertial2Body uses, and
		// already how mc_att_control/vtol_att_control/EKF2 read this
		// exact topic.
		_euler = matrix::Eulerf(matrix::Quatf(attitude.q));

		trajectory_setpoint_s trajectory_setpoint{};
		_trajectory_setpoint_sub.copy(&trajectory_setpoint);

		_vehicle_control_mode_sub.copy(&_vehicle_control_mode);

		vehicle_land_detected_s land_detected{};
		_vehicle_land_detected_sub.copy(&land_detected);
		// vehicle_land_detected_s default-constructs with landed == false;
		// if nothing has published yet this reads as "not landed", which
		// is the conservative choice for the integrator's landed gate
		// (rate_control.cpp's landed gate exists to STOP integrating, so
		// defaulting to "not landed" is the fail-safe direction: a wrong
		// "landed" default would silently disable integration instead).
		const bool landed = land_detected.landed;

		// Disarm-triggered integrator reset (step 4e decision 4). Only
		// the disarm edge; see the member comment on _armed_prev for why
		// "mode entry" has no equivalent here.
		if (_armed_prev && !_vehicle_control_mode.flag_armed) {
			_pos_vel_control.resetIntegral();
			_att_rate_control.resetIntegral();
		}

		_armed_prev = _vehicle_control_mode.flag_armed;

		// --- Position/velocity stage (4a), gated at 50 Hz, F_i -> F_b via
		// Inertial2Body (4b). ---
		float pos_vel_dt = 0.f;

		if (_pos_vel_gate.due(now, &pos_vel_dt)) {
			// Full mc_pos_control-style validity gating (step 4e
			// decision): estimator validity flags plus PX4_ISFINITE on
			// the setpoint, since TrajectorySetpoint.msg's own contract
			// is "NaN means this state should not be controlled". This
			// module has no independent velocity-setpoint path (4a's
			// PositionVelocityControl derives vel_sp from pos_sp; it
			// was not extended to accept vel_sp directly, since doing
			// so would be a new decision about that class, not this
			// diff's wiring) -- see the open item this generates in
			// controller.md.
			const bool position_valid = local_position.xy_valid && local_position.z_valid;
			const bool velocity_valid = local_position.v_xy_valid && local_position.v_z_valid;
			const bool setpoint_valid = PX4_ISFINITE(trajectory_setpoint.position[0])
						    && PX4_ISFINITE(trajectory_setpoint.position[1])
						    && PX4_ISFINITE(trajectory_setpoint.position[2]);
			const bool inputs_valid = position_valid && velocity_valid && setpoint_valid;

			if (inputs_valid) {
				if (!_pos_vel_inputs_valid_prev) {
					// Recovering from invalid input: reset rather
					// than resume from whatever the integrator held
					// during the gap.
					_pos_vel_control.resetIntegral();
				}

				const matrix::Vector3f pos(local_position.x, local_position.y, local_position.z);
				const matrix::Vector3f pos_sp(trajectory_setpoint.position);
				const matrix::Vector3f vel(local_position.vx, local_position.vy, local_position.vz);
				const matrix::Vector3f vel_dot(local_position.ax, local_position.ay, local_position.az);

				const matrix::Vector3f F_i =
					_pos_vel_control.update(pos, pos_sp, vel, vel_dot, pos_vel_dt);
				_F_b = foldrotor::inertialToBody(F_i, _euler);
			}

			// else: hold the previous _F_b rather than compute from
			// invalid/NaN input. EKF reset-counter adjustment
			// (xy_reset_counter etc.) is a separate, recorded open
			// item -- nothing is published to actuators yet, so no
			// transient from an unhandled reset can reach the vehicle.

			_pos_vel_inputs_valid_prev = inputs_valid;
		}

		// --- Attitude stage (4c's attitude-P half), gated at 250 Hz. ---
		if (_attitude_gate.due(now)) {
			// euler_sp: phi_sp/theta_sp pinned to zero (step 4e user
			// decision -- this vehicle translates by thrust vectoring,
			// not body lean; findings.md "euler_sp sourcing (step
			// 4e)"). psi_sp is the only Euler setpoint
			// TrajectorySetpoint actually carries; NaN means "don't
			// control yaw" per its own contract, so it holds the
			// current heading instead of commanding a NaN-derived
			// moment.
			const float psi_sp = PX4_ISFINITE(trajectory_setpoint.yaw) ? trajectory_setpoint.yaw : _euler.psi();
			const matrix::Eulerf euler_sp(0.f, 0.f, psi_sp);

			_att_rate_control.updateAttitude(_euler, euler_sp);
		}

		// --- Rate stage (4c's rate-PID half), every cycle -- this Run()'s
		// native ~1000 Hz, driven by vehicle_angular_velocity. ---
		const matrix::Vector3f rate(angular_velocity.xyz);
		const matrix::Vector3f rate_dot(angular_velocity.xyz_derivative);
		_M_b = _att_rate_control.updateRate(rate, rate_dot, dt, landed);

		// _F_b / _M_b now hold the full computed wrench. PUBLISHED
		// NOWHERE -- see class comment. Anti-windup/integrator-limit
		// bounds are still +/-infinity (inert) until 4d.

		if (now - _last_heartbeat_log > 1_s) {
			_last_heartbeat_log = now;
			PX4_INFO("foldrotor_control alive — armed=%d offboard=%d position_ctrl=%d "
				 "F_b=[%.2f %.2f %.2f]N M_b=[%.3f %.3f %.3f]Nm (NOT PUBLISHED)",
				 _vehicle_control_mode.flag_armed,
				 _vehicle_control_mode.flag_control_offboard_enabled,
				 _vehicle_control_mode.flag_control_position_enabled,
				 (double)_F_b(0), (double)_F_b(1), (double)_F_b(2),
				 (double)_M_b(0), (double)_M_b(1), (double)_M_b(2));
		}
	}

	perf_end(_loop_perf);
}

int FoldrotorControl::print_status()
{
	PX4_INFO("status: step 4e part 1 -- cascade wired, wrench computed, PUBLISHES NOTHING to actuators");
	PX4_INFO("armed=%d", _vehicle_control_mode.flag_armed);
	PX4_INFO("F_b = [%.3f, %.3f, %.3f] N (body/FRD)", (double)_F_b(0), (double)_F_b(1), (double)_F_b(2));
	PX4_INFO("M_b = [%.4f, %.4f, %.4f] N*m (body/FRD)", (double)_M_b(0), (double)_M_b(1), (double)_M_b(2));
	PX4_INFO("open items: output/integrator-limit bounds are +/-infinity (anti-windup inert) until 4d; "
		 "EKF reset-counter adjustment not implemented; integrator reset handles disarm only, "
		 "not \"mode entry\" (this module has no PX4 flight-mode concept)");

	return 0;
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

Status: step 4e part 1 of the implementation plan (see .claude/specs/
controller.md, controller_params.md). The full position/velocity/
attitude/rate cascade is wired into Run() and computes a wrench
(F_b, M_b) every cycle -- `status` prints it. Allocation (step 4d) does
not exist yet and NOTHING IS PUBLISHED to actuator_motors/
actuator_servos; this module is inert at the actuator boundary. The
stock mc_pos_control/mc_att_control/mc_rate_control stack is unaffected
by this module running.

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
