/****************************************************************************
 *
 * foldrotor_control — standalone PX4 module for the foldrotor3 fully-
 * actuated bi-rotor vehicle. Replaces the stock position/attitude/rate/
 * allocation stack (mc_pos_control, mc_att_control, mc_rate_control,
 * control_allocator) for this vehicle only; those modules are not modified.
 *
 * STEP 4e part 1 (.claude/specs/controller.md, controller_params.md):
 * the multi-rate cascade is now wired into Run() — PositionVelocityControl
 * (4a) at 50 Hz, AttitudeRateControl's attitude stage (4c) at 250 Hz and
 * its rate stage every cycle (1000 Hz, this Run()'s native rate),
 * Inertial2Body (4b) rotating the force output between them. The full
 * wrench (_F_b, _M_b) is computed every cycle and held for inspection —
 * getForceBody()/getMomentBody() and the heartbeat log — but is
 * PUBLISHED NOWHERE. No actuator_motors/actuator_servos wiring exists;
 * that, the allocator (4d), and the airframe/rc.txt change that would
 * stop the stock mc_* stack starting are all separate, later diffs. This
 * module is provably inert at the actuator boundary today.
 *
 * Allocation (FoldrotorAllocation, 4d) does not exist yet; _F_b/_M_b are
 * the body-frame wrench allocation.md's Interface expects as input, not
 * an actuator command.
 *
 ****************************************************************************/

#pragma once

#include "AttitudeRateControl.hpp"
#include "CascadeRateGate.hpp"
#include "Inertial2Body.hpp"
#include "PositionVelocityControl.hpp"

#include <lib/perf/perf_counter.h>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionInterval.hpp>
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
	// input. Published nowhere — see class comment.
	matrix::Vector3f _F_b{};
	matrix::Vector3f _M_b{};

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

		(ParamFloat<px4::params::FR_ATT_P>)         _param_fr_att_p,

		(ParamFloat<px4::params::FR_RATE_RP_FF>)    _param_fr_rate_rp_ff,
		(ParamFloat<px4::params::FR_RATE_RP_I>)     _param_fr_rate_rp_i,
		(ParamFloat<px4::params::FR_RATE_RP_D>)     _param_fr_rate_rp_d,

		(ParamFloat<px4::params::FR_RATE_YAW_FF>)   _param_fr_rate_yaw_ff,
		(ParamFloat<px4::params::FR_RATE_YAW_I>)    _param_fr_rate_yaw_i,
		(ParamFloat<px4::params::FR_RATE_YAW_D>)    _param_fr_rate_yaw_d
	)
};
