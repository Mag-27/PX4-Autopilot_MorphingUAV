/****************************************************************************
 *
 * foldrotor_control — standalone PX4 module for the foldrotor3 fully-
 * actuated bi-rotor vehicle. Replaces the stock position/attitude/rate/
 * allocation stack (mc_pos_control, mc_att_control, mc_rate_control,
 * control_allocator) for this vehicle only; those modules are not modified.
 *
 * STEP 3 (params) of the implementation plan (.claude/specs/
 * controller_params.md): the position/velocity/attitude/rate gains from
 * controller.md are now declared as params (FR_* prefix) and loaded into
 * a plain Gains struct by parameters_updated(). The quaternion->Euler
 * conversion required ahead of the cascade math (controller.md Interface)
 * is wired into Run() using PX4's own matrix::Eulerf/Quatf. No control
 * law runs yet and nothing is published to the actuators. The cascade
 * math (PositionVelocityControl / AttitudeRateControl /
 * FoldrotorAllocation) and the actuator_motors/actuator_servos
 * publications land in steps 4-5.
 *
 ****************************************************************************/

#pragma once

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

	bool init();

private:
	void Run() override;

	void parameters_updated();

	// Plain data holder for the cascade gains, populated by
	// parameters_updated() from the FR_* params below. One field per
	// controller_params.md table row. Deliberately not the actual control
	// objects (PositionVelocityControl / AttitudeRateControl) those gains
	// will eventually live in — that class design is step 4's concern; this
	// struct exists so step 3 has a concrete target to load params into and
	// test, without presuming step 4's shape.
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

	// Driving callback: registered on the fastest input this module needs
	// (matches mc_rate_control's pattern — see reference/px4-module-patterns.md
	// item 2/3). The multi-rate cascade (position/velocity @ 50 Hz, attitude
	// @ 250 Hz, per controller.md) is gated internally by accumulated dt
	// against each stage's own timestamp, not by a fixed sample-count divider
	// — added in step 5 once the cascade math exists.
	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};

	// Plain (polled) subscriptions — read every Run(), no control action yet.
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	vehicle_control_mode_s _vehicle_control_mode{};

	// Quaternion->Euler conversion required ahead of the phi/theta/psi
	// cascade math (controller.md Interface): vehicle_attitude.q is
	// FRD-body->NED, converted here with PX4's own 3-2-1 intrinsic
	// Tait-Bryan utility (matrix::Eulerf), the same convention already
	// used for Inertial2Body and already the established way this exact
	// topic is read (mc_att_control, vtol_att_control, EKF2). No custom
	// conversion math — this is wiring, not derivation. Step 4 consumes
	// this; nothing reads it yet.
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
