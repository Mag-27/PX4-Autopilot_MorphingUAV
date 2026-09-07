/****************************************************************************
 *
 * foldrotor_control — standalone PX4 module for the foldrotor3 fully-
 * actuated bi-rotor vehicle. Replaces the stock position/attitude/rate/
 * allocation stack (mc_pos_control, mc_att_control, mc_rate_control,
 * control_allocator) for this vehicle only; those modules are not modified.
 *
 * STEP 2 (skeleton) of the implementation plan: module lifecycle, driving
 * uORB callback, and read-only subscriptions are wired up. No control law
 * runs yet and nothing is published to the actuators — Run() only proves
 * the module starts, receives data, and stops cleanly. The cascade math
 * (PositionVelocityControl / AttitudeRateControl / FoldrotorAllocation)
 * and the actuator_motors/actuator_servos publications land in later steps
 * per the plan, once the control-law source values (see .claude/specs/
 * controller.md, allocation.md) are available to transcribe.
 *
 ****************************************************************************/

#pragma once

#include <lib/perf/perf_counter.h>
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

	hrt_abstime _last_run{0};
	hrt_abstime _last_heartbeat_log{0};

	perf_counter_t _loop_perf;
};
