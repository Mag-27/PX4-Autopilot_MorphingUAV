/****************************************************************************
 *
 * foldrotor_control — standalone PX4 module for the foldrotor3 fully-
 * actuated bi-rotor vehicle. Replaces the stock position/attitude/rate/
 * allocation stack (mc_pos_control, mc_att_control, mc_rate_control,
 * control_allocator) for this vehicle only; those modules are not modified.
 *
 * STEP 4e part 1 (.claude/specs/controller.md, controller_params.md):
 * the multi-rate cascade is wired into Run() — PositionVelocityControl
 * (4a) at 50 Hz, AttitudeRateControl's attitude stage (4c) at 250 Hz and
 * its rate stage every cycle (1000 Hz, this Run()'s native rate),
 * Inertial2Body (4b) rotating the force output between them. The full
 * wrench (_F_b, _M_b) is computed every cycle and held for inspection —
 * getForceBody()/getMomentBody() and the heartbeat log.
 *
 * STEP 4e part 2 (.claude/specs/allocation.md): FoldrotorAllocation (4d)
 * turns that wrench into per-rotor thrust/tilt commands and this module
 * now publishes actuator_motors/actuator_servos every cycle, gated on
 * vehicle_control_mode.flag_armed (NaN, per each message's own "NaN
 * maps to disarmed" contract, when not armed). Publishing is
 * unconditional, and this module is the sole publisher of these topics
 * for this airframe: 4026_gz_foldrotor3 no longer sets VEHICLE_TYPE mc,
 * so the stock control_allocator is not started for this vehicle (plan
 * open item O-4, resolved -- see print_status()). Fold (alpha) is pinned
 * to 0 in FoldrotorAllocation -- this module has no lateral
 * thrust-vectoring authority yet, see FoldrotorAllocation.hpp OPEN ITEM
 * (a). The newtons->normalized motor conversion inverts model.sdf's
 * rotor curve using SIM_GZ_EC_MIN1/MAX1, read via param_find/param_get
 * so the airframe file stays the single source of truth -- see
 * parameters_updated() and OPEN ITEM O-5 below.
 *
 * RESOLVED (2026-09-08) — the wrench sign/frame convention. Confirmed
 * root cause: Control_Alloc.m (allocation.md's math, matching the
 * source thesis derivation) was written in body FLU (Z-up);
 * _F_b/_M_b out of this module's cascade are PX4 body FRD (Z-down), per
 * controller.md. FLU<->FRD is a 180 deg rotation about body X: X
 * unchanged, Y and Z negate. Run() applies frdToAllocatorFlu() to both
 * _F_b and _M_b immediately before calling _allocation.allocate() --
 * the allocator class itself is untouched and still expects its native
 * (FLU) convention. See FoldrotorAllocation.hpp OPEN ITEM (c) and
 * allocation.md for the full argument and record.
 *
 ****************************************************************************/

#pragma once

#include "AttitudeRateControl.hpp"
#include "CascadeRateGate.hpp"
#include "FoldrotorAllocation.hpp"
#include "Inertial2Body.hpp"
#include "PositionVelocityControl.hpp"

#include <lib/perf/perf_counter.h>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>

using namespace time_literals;

class FoldrotorControl : public ModuleBase, public ModuleParams, public px4::WorkItem
{
public:
	static Descriptor desc;

	FoldrotorControl();
	~FoldrotorControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase — `foldrotor_control status` */
	int print_status() override;

	bool init();

	/** Computed body-frame wrench, held between cycles. Inspection only — see class comment. */
	const matrix::Vector3f &getForceBody() const { return _F_b; }
	const matrix::Vector3f &getMomentBody() const { return _M_b; }

	// Actuator-mapping free functions used by Run() (step 4e part 2).
	// Public static, not a new helper file (see class comment): this is
	// what lets FoldrotorControlTest.cpp exercise the mapping without a
	// work queue, per the step 4e allocation plan's Part C mapping tests.

	/** Newtons -> normalized [0,1] motor command; see FoldrotorControl.cpp for the derivation. */
	static float thrustToNormalizedMotor(float thrust_n, float ec_min, float ec_max);

	/** beta (tilt, rad) -> normalized [-1,1] servo command; linear, exact, no fudge factor. */
	static float tiltToNormalizedServo(float beta_rad);

	/** alpha (fold, rad) -> normalized [-1,1] servo command; direct mapping (bench-verified 2026-09-10, see allocation.md). */
	static float foldToNormalizedServo(float alpha_rad);

	/**
	 * PX4 body FRD -> Control_Alloc's body FLU, applied to a force or
	 * moment vector alike (allocation.md open item (c), resolved
	 * 2026-09-08). FLU<->FRD is a 180 deg rotation about body X: X is
	 * unchanged, Y and Z both negate. Same coefficients as
	 * foldrotor3_tests/test_frame_convention.py's FLU_TO_FRD
	 * (`np.diag([1.0, -1.0, -1.0])`) -- that file is this transform's
	 * source of truth; this is a transcription, not an independent
	 * derivation, and the rotation is its own inverse (180 deg) so the
	 * same function converts either direction.
	 */
	static matrix::Vector3f frdToAllocatorFlu(const matrix::Vector3f &v_frd);

private:
	void Run() override;

	void parameters_updated();

	// Plain staging area for the cascade gains, populated by
	// parameters_updated() from the FR_* params below and then pushed into
	// _pos_vel_control / _att_rate_control's setters (per
	// reference/px4-module-patterns.md item 4: raw _param_fr_* values are
	// read only in parameters_updated(), never from hot control-law code).
	// One field per controller_params.md table row.
	struct Gains {
		float pos_p{0.f};

		float vel_xy_ff{0.f};
		float vel_xy_i{0.f};
		float vel_xy_d{0.f};

		float vel_z_ff{0.f};
		float vel_z_i{0.f};
		float vel_z_d{0.f};
		float vel_z_grav_ff{0.f};
		float vel_z_i_lim{0.f};

		float att_p{0.f};

		float rate_rp_ff{0.f};
		float rate_rp_i{0.f};
		float rate_rp_d{0.f};

		float rate_yaw_ff{0.f};
		float rate_yaw_i{0.f};
		float rate_yaw_d{0.f};
	};

	Gains _gains{};

	// The cascade objects (4a, 4c) themselves — gains pushed in from
	// _gains by parameters_updated(), state (integrators, held rate_sp)
	// carried across Run() cycles.
	foldrotor::PositionVelocityControl _pos_vel_control;
	foldrotor::AttitudeRateControl _att_rate_control;

	// Multi-rate cascade cadence (step 4e part 1, user decision 2026-09-07
	// — NOT in controller.md, see findings.md "Multi-rate cascade cadence
	// (step 4e)"): position/velocity at 50 Hz, attitude at 250 Hz. The
	// rate stage runs every Run() cycle (this module's native 1000 Hz,
	// driven by vehicle_angular_velocity) and needs no gate of its own.
	// Gated on accumulated time since each stage's own last fire, against
	// the driving topic's own timestamp_sample — never a fixed
	// sample-count divider, per reference/px4-module-patterns.md item 2.
	foldrotor::CascadeRateGate _pos_vel_gate{20000}; // 50 Hz, period in us
	foldrotor::CascadeRateGate _attitude_gate{4000};  // 250 Hz, period in us

	// The computed wrench, held between the position/velocity and
	// attitude stages' slower cycles. Body/FRD, allocation.md's Interface
	// input -- see class comment's OPEN ITEM on the sign/frame
	// convention this is hand off to _allocation as-is.
	matrix::Vector3f _F_b{};
	matrix::Vector3f _M_b{};

	// Allocation (4d): stateless pure math, no setters -- see
	// FoldrotorAllocation.hpp. Minv is derived from M0 at construction.
	foldrotor::FoldrotorAllocation _allocation;

	// Held between cycles purely for print_status()/the heartbeat log;
	// not fed back into anything.
	foldrotor::FoldrotorAllocation::Output _alloc_out{};

	uORB::Publication<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};
	uORB::Publication<actuator_servos_s> _actuator_servos_pub{ORB_ID(actuator_servos)};

	// Newtons -> normalized motor command (open item O-5, see class
	// comment): inverts model.sdf's F = motorConstant * omega^2 curve,
	// then interpolates omega onto SIM_GZ_EC_MIN1/MAX1 -- NOT hardcoded
	// 308/2054, read from the params so the airframe file stays the
	// single source of truth. motorConstant/maxRotVelocity themselves
	// ARE from model.sdf (Tools/simulation/gz/models/foldrotor3/
	// model.sdf:557-559) and are not exposed as PX4 params, so those two
	// stay literal.
	float _sim_gz_ec_min1{308.f};
	float _sim_gz_ec_max1{2054.f};

	// Reset-on-recovery for the position/velocity integrator: true only
	// while vehicle_local_position's validity flags and
	// trajectory_setpoint's position are all valid/finite. On the
	// invalid->valid transition the integrator is reset rather than
	// resuming from whatever it held during the gap (mc_pos_control
	// precedent). Attitude/rate have no equivalent: vehicle_attitude and
	// vehicle_angular_velocity carry no _valid flags in this message set
	// (checked against VehicleAttitude.msg / VehicleAngularVelocity.msg,
	// 2026-09-07) -- an open item, not an oversight, recorded in
	// controller.md.
	bool _pos_vel_inputs_valid_prev{false};

	// Disarm-triggered integrator reset (step 4e decision 4, "on disarm
	// and on mode entry"). Only the disarm edge is implemented: this
	// module has no PX4 flight-mode concept of its own (it always runs
	// the same cascade), so "mode entry" has no equivalent here --
	// recorded as an open item in controller.md rather than guessed at.
	bool _armed_prev{false};

	// Driving callback: registered on the fastest input this module needs
	// (matches mc_rate_control's pattern — see reference/px4-module-patterns.md
	// item 2/3).
	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};

	// Plain (polled) subscriptions.
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	vehicle_control_mode_s _vehicle_control_mode{};

	// Quaternion->Euler conversion required ahead of the phi/theta/psi
	// cascade math (controller.md Interface): vehicle_attitude.q is
	// FRD-body->NED, converted here with PX4's own 3-2-1 intrinsic
	// Tait-Bryan utility (matrix::Eulerf), the same convention already
	// used for Inertial2Body and already the established way this exact
	// topic is read (mc_att_control, vtol_att_control, EKF2). No custom
	// conversion math — this is wiring, not derivation. Consumed by
	// Inertial2Body and AttitudeRateControl below.
	matrix::Eulerf _euler{};

	hrt_abstime _last_run{0};
	hrt_abstime _last_heartbeat_log{0};

	perf_counter_t _loop_perf;

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::FR_POS_P>)         _param_fr_pos_p,

		(ParamFloat<px4::params::FR_VEL_XY_FF>)     _param_fr_vel_xy_ff,
		(ParamFloat<px4::params::FR_VEL_XY_I>)      _param_fr_vel_xy_i,
		(ParamFloat<px4::params::FR_VEL_XY_D>)      _param_fr_vel_xy_d,

		(ParamFloat<px4::params::FR_VEL_Z_FF>)      _param_fr_vel_z_ff,
		(ParamFloat<px4::params::FR_VEL_Z_I>)       _param_fr_vel_z_i,
		(ParamFloat<px4::params::FR_VEL_Z_D>)       _param_fr_vel_z_d,
		(ParamFloat<px4::params::FR_VEL_Z_GRAV_FF>) _param_fr_vel_z_grav_ff,
		(ParamFloat<px4::params::FR_VEL_Z_I_LIM>)   _param_fr_vel_z_i_lim,

		(ParamFloat<px4::params::FR_ATT_P>)         _param_fr_att_p,

		(ParamFloat<px4::params::FR_RATE_RP_FF>)    _param_fr_rate_rp_ff,
		(ParamFloat<px4::params::FR_RATE_RP_I>)     _param_fr_rate_rp_i,
		(ParamFloat<px4::params::FR_RATE_RP_D>)     _param_fr_rate_rp_d,

		(ParamFloat<px4::params::FR_RATE_YAW_FF>)   _param_fr_rate_yaw_ff,
		(ParamFloat<px4::params::FR_RATE_YAW_I>)    _param_fr_rate_yaw_i,
		(ParamFloat<px4::params::FR_RATE_YAW_D>)    _param_fr_rate_yaw_d
	)
};
