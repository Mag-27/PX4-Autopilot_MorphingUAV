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
 * getForceBody()/getMomentBody() and `foldrotor_control status`.
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

#include <lib/mathlib/math/filter/AlphaFilter.hpp>
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
#include <uORB/topics/debug_array.h>
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

	// Rate-loop per-axis output limits, N*m in body FRD. Public for the
	// same reason the mapping functions above are: so a test can pin them
	// without standing up a work queue. The full derivation (and why
	// these are the HOVER-constrained maxima, not the unconstrained ones)
	// lives at the point of use in FoldrotorControl.cpp's
	// parameters_updated(); revised 2026-09-21, see findings.md and
	// FoldrotorAllocation.hpp OPEN ITEM (d).
	//
	// These must never exceed what the allocator can actually deliver
	// while holding a hover: AttitudeRateControl's conditional-integration
	// anti-windup uses them to decide whether it is saturated, so a limit
	// above the true ceiling makes that detection silently blind. Guarded
	// by FoldrotorAllocationTest.RateLimitsDoNotExceedHoverMomentAuthority.
	// Peak of the scheduled envelope below, i.e. the most this vehicle can
	// ever be asked for on each axis. REVISED 2026-09-21 (5), down from
	// 3.8 / 0.30 / 3.8: those were each measured with the OTHER TWO AXES AT
	// ZERO, which is not how the rate loop uses them -- it demands all
	// three at once, every cycle. The simultaneously-deliverable envelope
	// is roughly half the single-axis one.
	// REVISED 2026-09-22 for the ballast mast geometry (hover thrust is
	// now 19.62 N, not 15.27 N, and kS1z/kS2z changed sign). Re-measured
	// with sitl_testing/allocation_study/gentable.py against the updated
	// allocator, same method as the 09-21 (5) figures they replace.
	static constexpr float kRateMxLimit = 1.52f;  ///< roll,  peak simultaneous
	static constexpr float kRateMyLimit = 0.22f;  ///< pitch, peak simultaneous
	static constexpr float kRateMzLimit = 2.56f;  ///< yaw,   peak simultaneous

	// Hard ceiling on horizontal force magnitude, N. Declared here rather
	// than beside the other position-loop limits in parameters_updated()
	// because it is a matched pair with momentEnvelopeAtThrust()'s table
	// (which is measured holding exactly this much in reserve), and the
	// regression test has to be able to see both. Rationale at the point
	// of use in FoldrotorControl.cpp.
	static constexpr float kPosVelForceXYLimit = 1.0f;

	// Hard ceiling on BODY-frame horizontal force, N. Sized from the pitch
	// authority rather than the tilt rails: a body-forward force acts
	// 5.5 cm below the CoM and pitches the vehicle nose-up, so this bounds
	// that disturbance to ~40% of the hover pitch envelope
	// (0.4 * 0.146 / 0.0549 = 1.06 N). Derivation at the point of use in
	// Run(). This is the constant that makes pitch stable.
	static constexpr float kBodyForceXYLimit = 1.0f;

	// --- Pitch tilt lever (2026-09-21, architecture change) ------------
	//
	// Bound on the body-x force the ATTITUDE loop may command in order to
	// produce pitch moment, N. Deliberately separate from
	// kBodyForceXYLimit above, which bounds the body-x force the POSITION
	// loop asks for -- the two are the same physical quantity but opposite
	// in intent, and collapsing them into one budget is what left pitch
	// with no usable actuator:
	//
	//   position-loop Fx  -> DESTABILISING. Wanting to translate while
	//                        tilted asks for body-forward force, which
	//                        pitches further nose-up (findings.md (9)).
	//                        Keep it fenced off: kBodyForceXYLimit.
	//   attitude-loop Fx  -> STABILISING. The rate loop aims the same
	//                        lever with the sign that corrects pitch.
	//                        Give it room: this constant.
	//
	// Sized to cover the full hover pitch envelope through the lever
	// alone. REVISED 2026-09-22: the mast shrank the lever arm from
	// 0.0549 to 0.0170 N*m/N, so the same coverage would now cost
	// 0.44 / 0.0170 = 26 N of body-x force -- an absurd trade. The lever
	// existed only to buy bandwidth against an UNSTABLE pitch pole; the
	// mast removes the pole, so FR_PITCH_LEVER now defaults to 0 and this
	// bound is a backstop rather than an operating point. Kept at a value
	// that cannot itself destabilise anything.
	static constexpr float kPitchLeverFxLimit = 2.0f;

	/**
	 * Moment authority simultaneously available on ALL THREE axes at a
	 * given collective Fz, N*m, body FRD.
	 *
	 * This replaces rollAuthorityAtThrust() (2026-09-21 (4)), which was
	 * right in kind but wrong in scope. Two errors it corrects:
	 *
	 * 1. It scheduled ONLY roll, on the reasoning that measured yaw
	 *    authority RISES with Fz and pitch is flat. Both of those
	 *    measurements were taken one axis at a time. Demanded together --
	 *    which is what the rate loop actually does -- all three collapse,
	 *    because they compete for the same two rotor thrust vectors. At
	 *    Fz = 17.36 N the single-axis maxima are 3.43 / 0.39 / 4.38 N*m but
	 *    the simultaneous envelope is only 1.75 / 0.17 / 2.17.
	 *
	 * 2. It modelled authority as differential thrust alone, ignoring the
	 *    +-kMaxTilt rails on alpha and beta. Those rails, not the thrust
	 *    budget, are what actually binds once any horizontal force is
	 *    commanded alongside a moment.
	 *
	 * The table is a measured bisection of the REAL allocator (feasibility
	 * = no clamp on F, alpha or beta for either rotor) over the worst-case
	 * horizontal direction, holding |Fxy| <= kPosVelForceXYLimit in
	 * reserve, then scaled by 0.85. Generated by
	 * sitl_testing/allocation_study/gentable.py; regenerate it there if the
	 * geometry constants or kMaxTilt/kMaxThrust ever change.
	 *
	 * Note the envelope is NOT monotonic: it peaks near Fz ~ 18-20 N and
	 * collapses toward zero at both ends -- at 30 N both rotors are pinned
	 * at kMaxThrust with nothing left to differentiate, and at 0 N there is
	 * no thrust to vector. This is why a single constant could not express
	 * it and why FR_VEL_Z_MAX_UP matters so much (2026-09-21 (3)).
	 *
	 * Guarded by FoldrotorAllocationTest.ScheduledMomentEnvelopeIsDeliverable.
	 */
	static matrix::Vector3f momentEnvelopeAtThrust(float fz_n);

	/**
	 * Scale a wrench down to the actuator envelope, in body FLU and in
	 * place, so FoldrotorAllocation is never handed a request it has to
	 * clamp. Priority: vertical force, then moment, then horizontal force.
	 * Rationale and the reason this is not a lookup table are at the point
	 * of definition in FoldrotorControl.cpp.
	 *
	 * Guarded by FoldrotorAllocationTest.FittedWrenchNeverSaturates.
	 */
	/**
	 * @param fx_lever_flu the body-x force the ATTITUDE loop added to make
	 *        pitch moment (see Run()'s pitch-lever block). Held at moment
	 *        priority rather than sacrificed first with the rest of the
	 *        horizontal force, because it IS the pitch moment. Pass 0 for
	 *        the plain "all of Fxy is a mission objective" behaviour.
	 */
	void fitWrenchToEnvelope(matrix::Vector3f &F_flu, matrix::Vector3f &M_flu,
				 float fx_lever_flu = 0.f) const;

	/**
	 * Bandwidth-limit the commanded wrench to what the servos can execute,
	 * in body FRD, in place. First-order, unity DC gain, one shared corner
	 * for force and moment. `cutoff_hz <= 0` or `dt <= 0` is a no-op and
	 * re-arms the seed, so the next enabled call starts from the raw
	 * command instead of stale state.
	 *
	 * Rationale, and the measurements that set the default corner, are on
	 * the _wrench_lp_* members below and in the FR_WRENCH_LP param.
	 *
	 * Guarded by FoldrotorWrenchLowPassTest.
	 */
	void applyWrenchLowPass(matrix::Vector3f &F_frd, matrix::Vector3f &M_frd,
				float cutoff_hz, float dt);

	/** Re-seed the wrench filter on the next call, as both arm edges do. */
	void resetWrenchLowPass() { _wrench_lp_reset = true; }

	/** Read-only access to the allocator, so the fit's guard test can check feasibility itself. */
	const foldrotor::FoldrotorAllocation &getAllocation() const { return _allocation; }

	/** Bisection steps used by fitWrenchToEnvelope(); 1e-4 of full scale. */
	static constexpr int kFitIterations = 14;

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
		float vel_xy_i_lim{0.f};

		float vel_z_ff{0.f};
		float vel_z_i{0.f};
		float vel_z_d{0.f};
		float vel_z_grav_ff{0.f};
		float vel_z_i_lim{0.f};

		float vel_xy_max{0.f};
		float vel_z_max_up{0.f};
		float vel_z_max_dn{0.f};

		float att_p{0.f};

		float rate_r_ff{0.f};
		float rate_r_i{0.f};
		float rate_r_d{0.f};
		float rate_r_i_lim{0.f};
		float rate_p_ff{0.f};
		float rate_p_i{0.f};
		float rate_p_d{0.f};
		float rate_p_i_lim{0.f};

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

	/**
	 * Command-path bandwidth limit, FR_WRENCH_LP (2026-09-22).
	 *
	 * WHY THIS EXISTS. Nothing between the rate loop and the servos
	 * bounded the command's SLEW, only its amplitude. The rate loop emits
	 * at 250 Hz and the allocator is an algebraic map, so whatever
	 * frequency content is in the wrench lands directly on alpha/beta.
	 * Measured in log 2026-09-22/06_12_34.ulg: the commanded tilt slewed
	 * at 2416 deg/s rms (p99 7051 deg/s), and the joint torque needed to
	 * TRACK the commanded angle -- J * d2(theta)/dt2 against model.sdf's
	 * cmd_max = 5 N*m -- was 12.1 N*m rms on tilt and 54.9 N*m rms on
	 * fold, exceeding cmd_max on 52% and 89% of samples respectively.
	 * The servos spent most of the flight saturated, and a saturated
	 * actuator inside a feedback loop contributes phase lag the linear
	 * design never accounted for: that is what sustained the 18.8 Hz
	 * yaw limit cycle (truth yaw rate 1.06 rad/s rms, 82% of its power
	 * above 10 Hz).
	 *
	 * WHY A FILTER AND NOT A SLEW LIMIT. A slew limit on alpha/beta
	 * would clip each angle independently, which distorts the delivered
	 * wrench exactly the way allocation.md's clamp-not-redistribute does
	 * -- the defect fitWrenchToEnvelope() exists to prevent. Filtering
	 * the wrench keeps the allocator's input a consistent wrench; the
	 * angles stay whatever that wrench maps to.
	 *
	 * WHY IT IS FREE. The rigid-body closed-loop bandwidths are 0.36 Hz
	 * (pitch), 0.46 Hz (roll) and 0.62 Hz (yaw); the actuators' dominant
	 * poles are 7.1 Hz (tilt) and 6.1 Hz (fold), computed from the SDF
	 * inertia about each joint axis and its p=20/d=0.5 PID. A 5 Hz
	 * corner sits an order of magnitude above every loop and below every
	 * actuator pole, costing ~7 deg of phase at the fastest loop.
	 *
	 * ORDERING. Applied AFTER the rate loop's own output clamp, so the
	 * envelope guarantee survives: a first-order low-pass of a signal
	 * bounded by +/-m_limit is itself bounded by +/-m_limit, so
	 * fitWrenchToEnvelope() and the anti-windup still see a feasible
	 * request. Unity DC gain, so no steady-state trim is altered.
	 */
	AlphaFilter<matrix::Vector3f> _wrench_lp_force{};
	AlphaFilter<matrix::Vector3f> _wrench_lp_moment{};

	/**
	 * Set on both arm edges so the next cycle seeds the filters with the
	 * current wrench instead of ramping into it. Same reasoning as the
	 * _pos_vel_gate/_attitude_gate resets beside the integrator resets:
	 * carrying pre-arm filter state into the first armed cycle would put
	 * a stale wrench on the actuators for one time constant.
	 */
	bool _wrench_lp_reset{true};

	// Allocation (4d): stateless pure math, no setters -- see
	// FoldrotorAllocation.hpp. Minv is derived from M0 at construction.
	foldrotor::FoldrotorAllocation _allocation;

	// Held between cycles purely for print_status();
	// not fed back into anything.
	foldrotor::FoldrotorAllocation::Output _alloc_out{};

	uORB::Publication<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};
	uORB::Publication<actuator_servos_s> _actuator_servos_pub{ORB_ID(actuator_servos)};

	// Allocation output (_alloc_out: F1/F2/alpha1/alpha2/beta1/beta2) plus
	// the allocator's input wrench, for live inspection over MAVLink
	// without touching the mavlink module -- debug_array is a generic,
	// already-registered stream (MavlinkStreamDebugFloatArray ->
	// DEBUG_FLOAT_ARRAY), enabled at runtime with
	// `mavlink stream -s DEBUG_FLOAT_ARRAY -r <rate>`, no core-module
	// changes. See sitl_testing/plot_hover.py.
	//
	// EXACTLY ONE debug_array publish per cycle. debug_array is
	// single-instance with queue depth 1, so a second publish in the same
	// cycle does not add a message -- it DESTROYS the first one for every
	// subscriber slower than this loop, silently. That regression shipped
	// and cost a flight-test session (see Run(), and findings.md
	// 2026-09-21). Anything else that needs publishing gets another slot
	// in this same array, not another publish() call.
	uORB::Publication<debug_array_s> _debug_array_pub{ORB_ID(debug_array)};

	// Slot layout of the single "fr_alloc" debug_array. Named constants
	// rather than bare indices because sitl_testing/plot_hover.py decodes
	// the same layout by position and there is nothing in the message to
	// catch a mismatch. DebugArray.msg ARRAY_SIZE is 58; 13 used.
	static constexpr int kDebugF1        = 0;
	static constexpr int kDebugF2        = 1;
	static constexpr int kDebugAlpha1    = 2;
	static constexpr int kDebugAlpha2    = 3;
	static constexpr int kDebugBeta1     = 4;
	static constexpr int kDebugBeta2     = 5;
	static constexpr int kDebugSaturated = 6;
	static constexpr int kDebugWrenchFx  = 7;
	static constexpr int kDebugWrenchFy  = 8;
	static constexpr int kDebugWrenchFz  = 9;
	static constexpr int kDebugWrenchMx  = 10;
	static constexpr int kDebugWrenchMy  = 11;
	static constexpr int kDebugWrenchMz  = 12;
	static constexpr int kDebugSlotsUsed = 13;

	// DIAGNOSTIC (temporary -- remove when findings.md's flip
	// investigation closes).
	//
	// Why this exists rather than just logging more to debug_array: the
	// failure being chased starts and rails within 4-8 ms of arming, and
	// debug_array cannot resolve that. The logger polls every
	// _log_interval = 3500 us (logger.cpp) and DebugArray.msg carries no
	// queue depth, so a topic published at the rate loop's 1 kHz is
	// sampled at ~250 Hz no matter what the module does. Raising it means
	// editing the logger or the msg -- both outside this standalone
	// module's boundary (.claude/CLAUDE.md "Hard constraints").
	//
	// So: capture every cycle into a ring buffer owned by this module and
	// dump it on demand with `foldrotor_control trace`. SITL-only
	// diagnostic sizing (~2 s at 1 kHz, ~140 kB); this is not intended to
	// ship to a flight board. The dump runs on the console thread while
	// Run() writes, which is a benign tear for diagnostics -- a sample may
	// be internally inconsistent, but sample TIMING is what is being
	// recovered here and that is unaffected.
	struct TraceSample {
		uint64_t t;             ///< hrt timestamp, us
		float rate[3];          ///< measured body rate, FRD rad/s
		float rate_dot[3];      ///< measured angular accel (xyz_derivative), FRD rad/s^2
		float rate_sp[3];       ///< rate setpoint out of the attitude stage, FRD
		float m_b[3];           ///< rate-loop moment output, FRD N*m
		float f_b[3];           ///< position-loop force output, FRD N
		float m_limit[3];       ///< moment authority granted this cycle, N*m FRD
		uint8_t saturated;      ///< allocator clamped a channel
		uint8_t armed;
	};
	static constexpr int kTraceLen = 2048;
	TraceSample _trace[kTraceLen] {};
	uint32_t _trace_head{0};    ///< total samples written; index = head % kTraceLen
	bool _trace_enabled{true};
	bool _trace_freeze_on_disarm{true};
	bool _trace_frozen{false};
	bool _trace_was_armed{false};
	uint32_t _trace_arm_head{0};   ///< _trace_head at the last arm edge
	bool _trace_capture_from_arm{true}; ///< freeze once the buffer fills after arming

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

	// Full NED->body rotation, used by the FORCE path only. The attitude
	// stage keeps _euler because its setpoint is an Euler triple. See
	// Inertial2Body.hpp for why the force path must not use yaw alone.
	matrix::Dcmf _R_ned_to_body{};

	hrt_abstime _last_run{0};

	perf_counter_t _loop_perf;

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::FR_POS_P>)         _param_fr_pos_p,

		(ParamFloat<px4::params::FR_VEL_XY_FF>)     _param_fr_vel_xy_ff,
		(ParamFloat<px4::params::FR_VEL_XY_I>)      _param_fr_vel_xy_i,
		(ParamFloat<px4::params::FR_VEL_XY_D>)      _param_fr_vel_xy_d,
		(ParamFloat<px4::params::FR_VEL_XY_I_LIM>)  _param_fr_vel_xy_i_lim,

		(ParamFloat<px4::params::FR_VEL_Z_FF>)      _param_fr_vel_z_ff,
		(ParamFloat<px4::params::FR_VEL_Z_I>)       _param_fr_vel_z_i,
		(ParamFloat<px4::params::FR_VEL_Z_D>)       _param_fr_vel_z_d,
		(ParamFloat<px4::params::FR_VEL_Z_GRAV_FF>) _param_fr_vel_z_grav_ff,
		(ParamFloat<px4::params::FR_VEL_Z_I_LIM>)   _param_fr_vel_z_i_lim,

		(ParamFloat<px4::params::FR_VEL_XY_MAX>)    _param_fr_vel_xy_max,
		(ParamFloat<px4::params::FR_VEL_Z_MAX_UP>)  _param_fr_vel_z_max_up,
		(ParamFloat<px4::params::FR_VEL_Z_MAX_DN>)  _param_fr_vel_z_max_dn,

		(ParamFloat<px4::params::FR_ATT_P>)         _param_fr_att_p,
		(ParamFloat<px4::params::FR_PITCH_LEVER>)  _param_fr_pitch_lever,
		(ParamFloat<px4::params::FR_WRENCH_LP>)    _param_fr_wrench_lp,

		(ParamFloat<px4::params::FR_RATE_R_FF>)     _param_fr_rate_r_ff,
		(ParamFloat<px4::params::FR_RATE_R_I>)      _param_fr_rate_r_i,
		(ParamFloat<px4::params::FR_RATE_R_D>)      _param_fr_rate_r_d,
		(ParamFloat<px4::params::FR_RATE_R_I_LIM>)  _param_fr_rate_r_i_lim,
		(ParamFloat<px4::params::FR_RATE_P_FF>)     _param_fr_rate_p_ff,
		(ParamFloat<px4::params::FR_RATE_P_I>)      _param_fr_rate_p_i,
		(ParamFloat<px4::params::FR_RATE_P_D>)      _param_fr_rate_p_d,
		(ParamFloat<px4::params::FR_RATE_P_I_LIM>)  _param_fr_rate_p_i_lim,

		(ParamFloat<px4::params::FR_RATE_YAW_FF>)   _param_fr_rate_yaw_ff,
		(ParamFloat<px4::params::FR_RATE_YAW_I>)    _param_fr_rate_yaw_i,
		(ParamFloat<px4::params::FR_RATE_YAW_D>)    _param_fr_rate_yaw_d
	)
};
