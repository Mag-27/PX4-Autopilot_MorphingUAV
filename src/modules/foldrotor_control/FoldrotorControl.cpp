/****************************************************************************
 *
 * foldrotor_control — see FoldrotorControl.hpp for scope/status.
 *
 ****************************************************************************/

#include "FoldrotorControl.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/math/Limits.hpp>

using namespace time_literals;

namespace
{
// model.sdf's rotor model (Tools/simulation/gz/models/foldrotor3/
// model.sdf:557-559): F = motorConstant * omega^2. Not a PX4 param --
// literal, matching allocation.md's step-4e plan.
constexpr float kMotorConstant = 5.4844e-06f;
} // namespace

ModuleBase::Descriptor FoldrotorControl::desc{task_spawn, custom_command, print_usage};

// Actuator-mapping functions used by Run() (step 4e part 2). Public
// static class methods, not a new helper file (see class comment): this
// is what lets FoldrotorControlTest.cpp exercise the mapping directly,
// without a work queue -- the step 4e allocation plan's "one structural
// concession" for the mapping tests.

float
FoldrotorControl::thrustToNormalizedMotor(float thrust_n, float ec_min, float ec_max)
{
	// Newtons -> normalized [0,1] motor command, open item O-5 (see
	// class comment). Inverts the SDF rotor curve to get the commanded
	// angular rate, then interpolates that rate onto [ec_min, ec_max] --
	// the same range SIM_GZ_EC_MIN1/MAX1 drive Gazebo's rotor plugin
	// with. NOT [-1,1]: the exact [-1,1]-vs-[0,1] semantics of
	// actuator_motors.control on this path were traced and NOT fully
	// pinned down this session -- this is the bench test's job to
	// confirm empirically before any free flight.
	const float thrust_clamped = math::constrain(thrust_n, 0.f, foldrotor::FoldrotorAllocation::kMaxThrust);
	const float omega = sqrtf(thrust_clamped / kMotorConstant);
	const float normalized = (omega - ec_min) / (ec_max - ec_min);
	return math::constrain(normalized, 0.f, 1.f);
}

float
FoldrotorControl::tiltToNormalizedServo(float beta_rad)
{
	// beta (tilt, rad) -> normalized [-1,1] servo command. Linear and
	// exact: SIM_GZ_SV_MINA/MAXA = +-45.26 deg = +-0.79 rad =
	// FoldrotorAllocation::kMaxTilt, and MixingOutput::
	// output_limit_calc_single (src/lib/mixer_module/mixer_module.cpp:565)
	// interpolates [-1,1] onto [min,max] -- so beta/kMaxTilt is correct
	// with no fudge factor. Clamped defensively regardless.
	return math::constrain(beta_rad / foldrotor::FoldrotorAllocation::kMaxTilt, -1.f, 1.f);
}

float
FoldrotorControl::foldToNormalizedServo(float alpha_rad)
{
	// fold (alpha, rad) -> normalized [-1,1] servo command, direct
	// (no negation). Bench-measured 2026-09-10 (force/torque sensor,
	// both arms, motor + own fold servo): a positive ArmNFoldJoint
	// angle produces NEGATIVE Y thrust for both Arm1 and Arm2 -- the
	// opposite of the "SDF geometry, verified 2026-09-06" claim that
	// used to justify negating here. Since Control_Alloc's own
	// convention is also +alpha -> -Ty, the mapping is direct: no sign
	// flip needed. See allocation.md's alpha sign-mapping section for
	// the measurement data.
	return math::constrain(alpha_rad / foldrotor::FoldrotorAllocation::kMaxTilt, -1.f, 1.f);
}

matrix::Vector3f
FoldrotorControl::frdToAllocatorFlu(const matrix::Vector3f &v_frd)
{
	// allocation.md OPEN ITEM (c), resolved 2026-09-08: PX4 body FRD ->
	// Control_Alloc's body FLU. 180 deg rotation about body X -- X
	// unchanged, Y and Z negate. Coefficients transcribed from
	// foldrotor3_tests/test_frame_convention.py's FLU_TO_FRD
	// (`np.diag([1.0, -1.0, -1.0])`), the project's single source of
	// truth for this transform -- not an independent derivation. The
	// rotation is its own inverse (180 deg), so the same expression
	// converts either direction.
	return matrix::Vector3f(v_frd(0), -v_frd(1), -v_frd(2));
}

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
	_gains.vel_z_i_lim = _param_fr_vel_z_i_lim.get();

	_gains.att_p = _param_fr_att_p.get();

	_gains.rate_rp_ff = _param_fr_rate_rp_ff.get();
	_gains.rate_rp_i = _param_fr_rate_rp_i.get();
	_gains.rate_rp_d = _param_fr_rate_rp_d.get();

	_gains.rate_yaw_ff = _param_fr_rate_yaw_ff.get();
	_gains.rate_yaw_i = _param_fr_rate_yaw_i.get();
	_gains.rate_yaw_d = _param_fr_rate_yaw_d.get();

	// Push into the cascade objects. Output limits are left at their
	// +/-infinity defaults -- controller.md and controller_params.md are
	// both explicit that inventing bounds records an assumption as a
	// constraint; the real values come from the allocator (4d), not from
	// here. The *integrator* clamp is a separate mechanism: FR_VEL_Z_I_LIM
	// = 3.0 N (2026-09-09, findings.md) bounds the Z integral's own
	// contribution directly, independent of the (still-inert)
	// conditional-integration anti-windup above. X/Y have no such param
	// and stay +/-infinity -- no decision has been made there.
	_pos_vel_control.setPositionGain(_gains.pos_p);
	_pos_vel_control.setVelocityGains(_gains.vel_xy_ff, _gains.vel_xy_i, _gains.vel_xy_d,
					  _gains.vel_z_ff, _gains.vel_z_i, _gains.vel_z_d);
	_pos_vel_control.setGravityFeedforward(_gains.vel_z_grav_ff);
	_pos_vel_control.setIntegratorLimit(matrix::Vector3f(INFINITY, INFINITY, _gains.vel_z_i_lim));

	_att_rate_control.setAttitudeGain(_gains.att_p);
	_att_rate_control.setRateGains(_gains.rate_rp_ff, _gains.rate_rp_i, _gains.rate_rp_d,
				       _gains.rate_yaw_ff, _gains.rate_yaw_i, _gains.rate_yaw_d);

	// SIM_GZ_EC_MIN1/MAX1 -- NOT hardcoded 308/2054 (open item O-5, see
	// class comment): read here so the airframe file
	// (4026_gz_foldrotor3) stays the single source of truth for the
	// motor rate range. param_find() rather than a DEFINE_PARAMETERS
	// entry because these are SIM_GZ_* params owned by the simulator
	// bridge module, not this module's own FR_* namespace.
	param_t ec_min1_handle = param_find("SIM_GZ_EC_MIN1");
	param_t ec_max1_handle = param_find("SIM_GZ_EC_MAX1");

	if (ec_min1_handle != PARAM_INVALID) {
		int32_t ec_min1{};
		param_get(ec_min1_handle, &ec_min1);
		_sim_gz_ec_min1 = (float)ec_min1;
	}

	if (ec_max1_handle != PARAM_INVALID) {
		int32_t ec_max1{};
		param_get(ec_max1_handle, &ec_max1);
		_sim_gz_ec_max1 = (float)ec_max1;
	}
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

		// Arm-triggered integrator reset (2026-09-09, symmetric fix to the
		// disarm-edge reset above). The wrench (_F_b/_M_b, including
		// FR_VEL_Z_I's integrator) is computed every cycle regardless of
		// arm state -- while disarmed, that integral has no physical
		// feedback to correct it against, so it silently winds toward
		// whatever the stale/pre-arm position error demands (now capped
		// at FR_VEL_Z_I_LIM = 3.0 N as of 2026-09-09, but still nonzero
		// and still stale). Without this reset, the moment real thrust
		// engages on arm it inherits that accumulated bias directly,
		// which is what produced the arm-time thrust-ceiling spike and
		// attitude failsafe recorded in findings.md's 2026-09-09 (2)
		// entry -- confirmed to be that cause, not the trajectory_setpoint
		// gap fixed alongside it.
		if (!_armed_prev && _vehicle_control_mode.flag_armed) {
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

		// --- Allocation (4d) + actuator publish (4e part 2), every
		// cycle at the rate loop's 1000 Hz. FoldrotorAllocation is
		// stateless and cheap (one 6x6 multiply plus two atan2 calls),
		// so it needs no gate of its own -- matches how the rate stage
		// is already treated. Output-limit bounds (and the
		// conditional-integration anti-windup they drive) are still
		// +/-infinity (inert) -- choosing real bounds is a control
		// decision not made in this diff (see the plan's open items).
		// FR_VEL_Z_I's own integrator clamp is a separate mechanism and
		// IS bounded now (FR_VEL_Z_I_LIM = 3.0 N, 2026-09-09); see
		// PositionVelocityControl.hpp OPEN ITEM (d).
		//
		// _F_b/_M_b are PX4 body FRD; FoldrotorAllocation's math
		// (allocation.md's Control_Alloc, class comment RESOLVED note)
		// is body FLU -- convert both immediately before allocate(),
		// per-cycle, so _F_b/_M_b themselves stay in their native FRD
		// frame for the heartbeat log / print_status() / getForceBody()
		// / getMomentBody() above.
		const matrix::Vector3f F_alloc = frdToAllocatorFlu(_F_b);
		const matrix::Vector3f M_alloc = frdToAllocatorFlu(_M_b);
		_alloc_out = _allocation.allocate(F_alloc, M_alloc);

		actuator_motors_s actuator_motors{};
		actuator_servos_s actuator_servos{};

		actuator_motors.timestamp_sample = now;
		actuator_servos.timestamp_sample = now;

		for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; i++) {
			actuator_motors.control[i] = NAN;
		}

		for (int i = 0; i < actuator_servos_s::NUM_CONTROLS; i++) {
			actuator_servos.control[i] = NAN;
		}

		// Arm/mode gate -- reuse _vehicle_control_mode.flag_armed,
		// already read above for the disarm-edge integrator reset, no
		// new subscription. Not armed: leave every channel at NaN, per
		// ActuatorMotors.msg / ActuatorServos.msg's own contract ("NaN
		// maps to disarmed") -- NOT zero, which on actuator_motors is a
		// live commanded value output_limit_calc_single maps into the
		// ESC range, not "off".
		if (_vehicle_control_mode.flag_armed) {
			// Channel mapping, 4026_gz_foldrotor3 + force_moment_bench_commands.md:
			//   motors.control[0] = Motor1 (101, Prop1Joint, +Y) <- F1
			//   motors.control[1] = Motor2 (102, Prop2Joint, -Y) <- F2
			//   servos.control[0] = Servo1 (201, Arm1FoldJoint)  <- alpha1
			//   servos.control[1] = Servo2 (202, Arm1TiltJoint)  <- beta1
			//   servos.control[2] = Servo3 (203, Arm2FoldJoint)  <- alpha2
			//   servos.control[3] = Servo4 (204, Arm2TiltJoint)  <- beta2
			actuator_motors.control[0] = thrustToNormalizedMotor(_alloc_out.F1, _sim_gz_ec_min1, _sim_gz_ec_max1);
			actuator_motors.control[1] = thrustToNormalizedMotor(_alloc_out.F2, _sim_gz_ec_min1, _sim_gz_ec_max1);

			actuator_servos.control[0] = foldToNormalizedServo(_alloc_out.alpha1);
			actuator_servos.control[1] = tiltToNormalizedServo(_alloc_out.beta1);
			actuator_servos.control[2] = foldToNormalizedServo(_alloc_out.alpha2);
			actuator_servos.control[3] = tiltToNormalizedServo(_alloc_out.beta2);
		}

		actuator_motors.timestamp = hrt_absolute_time();
		actuator_servos.timestamp = hrt_absolute_time();

		// Publish unconditionally (step 4e plan decision 1). This module
		// is the sole publisher of these topics for this airframe: the
		// stock control_allocator is no longer started for
		// 4026_gz_foldrotor3 (plan open item O-4, resolved by the
		// airframe no longer setting VEHICLE_TYPE mc) -- see
		// print_status().
		_actuator_motors_pub.publish(actuator_motors);
		_actuator_servos_pub.publish(actuator_servos);

		if (now - _last_heartbeat_log > 1_s) {
			_last_heartbeat_log = now;
			PX4_INFO("foldrotor_control alive — armed=%d offboard=%d position_ctrl=%d "
				 "F_b=[%.2f %.2f %.2f]N M_b=[%.3f %.3f %.3f]Nm "
				 "F1=%.2f F2=%.2f a1=%.3f a2=%.3f b1=%.3f b2=%.3f sat=%d",
				 _vehicle_control_mode.flag_armed,
				 _vehicle_control_mode.flag_control_offboard_enabled,
				 _vehicle_control_mode.flag_control_position_enabled,
				 (double)_F_b(0), (double)_F_b(1), (double)_F_b(2),
				 (double)_M_b(0), (double)_M_b(1), (double)_M_b(2),
				 (double)_alloc_out.F1, (double)_alloc_out.F2,
				 (double)_alloc_out.alpha1, (double)_alloc_out.alpha2,
				 (double)_alloc_out.beta1, (double)_alloc_out.beta2,
				 _alloc_out.saturated);
		}
	}

	perf_end(_loop_perf);
}

int FoldrotorControl::print_status()
{
	PX4_INFO("status: step 4e part 2 -- cascade + allocation wired, publishing actuator_motors/actuator_servos");
	PX4_INFO("armed=%d", _vehicle_control_mode.flag_armed);
	PX4_INFO("F_b = [%.3f, %.3f, %.3f] N (body/FRD)", (double)_F_b(0), (double)_F_b(1), (double)_F_b(2));
	PX4_INFO("M_b = [%.4f, %.4f, %.4f] N*m (body/FRD)", (double)_M_b(0), (double)_M_b(1), (double)_M_b(2));
	PX4_INFO("F1=%.3f N F2=%.3f N  alpha1=%.4f alpha2=%.4f rad  beta1=%.4f beta2=%.4f rad  saturated=%d",
		 (double)_alloc_out.F1, (double)_alloc_out.F2,
		 (double)_alloc_out.alpha1, (double)_alloc_out.alpha2,
		 (double)_alloc_out.beta1, (double)_alloc_out.beta2,
		 _alloc_out.saturated);
	PX4_INFO("allocation Minv valid (derived from M0 at init): %d", _allocation.isValid());
	PX4_INFO("open items (see .claude/plans/step-4e-allocation-plan.md): wrench sign/frame convention "
		 "vs allocation.md's +Z-positive thrust is RESOLVED (frdToAllocatorFlu(), see class comment); "
		 "fold (alpha) is pinned to 0, no lateral thrust-vectoring authority yet; "
		 "control_allocator (stock) is no longer started for this airframe (4026_gz_foldrotor3 "
		 "overrides VEHICLE_TYPE to \"none\"), so this module is the sole publisher of "
		 "actuator_motors/actuator_servos; newtons->normalized motor mapping is unverified against "
		 "Gazebo (bench test required before free flight); output-limit bounds are +/-infinity "
		 "(conditional-integration anti-windup still inert); FR_VEL_Z_I_LIM=3.0N bounds the Z "
		 "integrator directly (RESOLVED 2026-09-09), X/Y integrator still unbounded; EKF "
		 "reset-counter adjustment not implemented; integrator reset handles disarm only, not "
		 "\"mode entry\"");

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

Status: step 4e part 2 of the implementation plan (see .claude/specs/
controller.md, controller_params.md, allocation.md, and
.claude/plans/step-4e-allocation-plan.md). The full position/velocity/
attitude/rate cascade is wired into Run(), computes a wrench (F_b, M_b)
every cycle, allocates it (FoldrotorAllocation, step 4d), converts it
into the allocator's FLU convention (frdToAllocatorFlu(), the wrench
sign/frame resolution) and publishes actuator_motors/actuator_servos
every cycle -- `status` prints the allocated commands and every open
item. This is now the sole publisher of those topics for this airframe:
4026_gz_foldrotor3 no longer sets VEHICLE_TYPE mc, so the stock
mc_pos_control/mc_att_control/mc_rate_control/control_allocator stack is
not started for this vehicle (those modules themselves are unmodified
and still start normally for any other airframe).

UNRESOLVED before any free-flight attempt: the newtons->normalized motor
mapping against Gazebo -- must be confirmed by a bench-context check
(module armed, not flying) before closed-loop hover, per
.claude/CLAUDE.md's verification-before-validation order.

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
