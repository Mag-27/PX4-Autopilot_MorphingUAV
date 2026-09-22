/****************************************************************************
 *
 * foldrotor_control — see FoldrotorControl.hpp for scope/status.
 *
 ****************************************************************************/

#include "FoldrotorControl.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/math/Limits.hpp>

#include <cstring>
#include <float.h>

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
FoldrotorControl::momentEnvelopeAtThrust(float fz_n)
{
	// Measured simultaneous moment envelope, bisected against the real
	// allocator on a uniform 2 N grid from 0 to 30 N. See the header for
	// the derivation and why a scalar roll-only schedule was wrong.
	// Fz(N) : Mx, My, Mz  (N*m, |Fxy| <= kPosVelForceXYLimit, x0.85)
	static constexpr int kN = 16;
	static constexpr float kStep = 2.f;
	static constexpr float kTable[kN][3] = {
		{ 0.000f,  0.000f,  0.000f},   //  0 N
		{ 0.084f,  0.008f,  0.123f},   //  2 N
		{ 0.307f,  0.026f,  0.324f},   //  4 N
		{ 0.529f,  0.045f,  0.529f},   //  6 N
		{ 0.742f,  0.064f,  0.743f},   //  8 N
		{ 0.955f,  0.083f,  0.957f},   // 10 N
		{ 1.169f,  0.102f,  1.172f},   // 12 N
		{ 1.382f,  0.121f,  1.386f},   // 14 N
		{ 1.593f,  0.140f,  1.603f},   // 16 N
		{ 1.542f,  0.180f,  2.050f},   // 18 N
		{ 1.214f,  0.190f,  2.164f},   // 20 N
		{ 0.895f,  0.176f,  1.990f},   // 22 N
		{ 0.661f,  0.151f,  1.685f},   // 24 N
		{ 0.433f,  0.120f,  1.320f},   // 26 N
		{ 0.210f,  0.080f,  0.846f},   // 28 N
		{ 0.000f,  0.000f,  0.000f},   // 30 N
	};

	// Sign-independent: Fz is DOWN-positive in FRD, so a hovering vehicle
	// presents a negative value here, but the envelope depends only on how
	// hard the rotors are driven.
	const float fz = fabsf(fz_n);
	const float idx = math::constrain(fz / kStep, 0.f, float(kN - 1));
	const int i0 = int(idx);
	const int i1 = math::min(i0 + 1, kN - 1);
	const float frac = idx - float(i0);

	matrix::Vector3f env;

	for (int a = 0; a < 3; a++) {
		env(a) = kTable[i0][a] + frac * (kTable[i1][a] - kTable[i0][a]);
	}

	return env;
}

void
FoldrotorControl::applyWrenchLowPass(matrix::Vector3f &F_frd, matrix::Vector3f &M_frd,
				     float cutoff_hz, float dt)
{
	// See the _wrench_lp_* member comment in the header for why this
	// exists; in short, nothing between the rate loop and the servos
	// bounded the command's SLEW, and the servos spent log
	// 2026-09-22/06_12_34.ulg saturated trying to follow it.
	//
	// A first-order low-pass of a signal bounded by +/-L is itself bounded
	// by +/-L (the state is a convex combination of past samples), so
	// running this AFTER the rate loop's output clamp preserves the
	// envelope guarantee that fitWrenchToEnvelope() and the
	// conditional-integration anti-windup both rely on. Unity DC gain, so
	// no steady-state trim is altered.
	if (!(cutoff_hz > 0.f) || !(dt > FLT_EPSILON)) {
		// Bypassed. Re-arm the seed so that enabling FR_WRENCH_LP in
		// flight starts from the live command rather than stepping out
		// of whatever the filter last held.
		_wrench_lp_reset = true;
		return;
	}

	_wrench_lp_force.setCutoffFreq(cutoff_hz);
	_wrench_lp_moment.setCutoffFreq(cutoff_hz);

	if (_wrench_lp_reset) {
		_wrench_lp_force.reset(F_frd);
		_wrench_lp_moment.reset(M_frd);
		_wrench_lp_reset = false;

	} else {
		_wrench_lp_force.update(F_frd, dt);
		_wrench_lp_moment.update(M_frd, dt);
	}

	F_frd = _wrench_lp_force.getState();
	M_frd = _wrench_lp_moment.getState();
}

void
FoldrotorControl::fitWrenchToEnvelope(matrix::Vector3f &F_flu, matrix::Vector3f &M_flu,
				      float fx_lever_flu) const
{
	// Scale an infeasible wrench down to the actuator envelope, in a fixed
	// priority order, so that FoldrotorAllocation never has to clamp.
	//
	// WHY THIS EXISTS. allocation.md specifies clamp-not-redistribute: if
	// the requested wrench is outside the envelope, each rotor's F/alpha/
	// beta is clamped independently and no attempt is made to recover the
	// intent. That is fine when the request is feasible and arbitrary when
	// it is not -- the delivered wrench then bears no fixed relationship to
	// the commanded one (measured worst case during the flip
	// investigation: Fy down 87%, Mx down 87%, and My SIGN-FLIPPED). This
	// does not change the allocation algorithm; it guarantees the algorithm
	// is only ever handed inputs it can satisfy exactly.
	//
	// WHY NOT A LOOKUP TABLE. momentEnvelopeAtThrust() indexes on collective
	// thrust alone, holding a fixed |Fxy| <= kPosVelForceXYLimit in reserve.
	// That assumption is only valid while the vehicle is near level. The
	// horizontal cap is applied in the INERTIAL frame, but the allocator's
	// alpha/beta rails are BODY constraints -- so once the vehicle tilts,
	// the body-frame horizontal force grows without bound (at 73 deg of
	// pitch, holding altitude needs ~17 N of body-forward force) and the
	// table's premise collapses. The table is kept for the rate loop's
	// output limits, where a cheap, monotone, always-nonzero bound is what
	// the anti-windup needs; feasibility is enforced here instead, against
	// the real allocator and the actual commanded direction.
	//
	// PRIORITY. Vertical force first (without it the vehicle falls), then
	// moment (without it the vehicle tumbles, and a tumbled vehicle cannot
	// use vertical force anyway), then horizontal force. Horizontal
	// position is the only one of the three that is merely a mission
	// objective rather than a survival condition, so it yields first.
	//
	// EXCEPT for fx_lever_flu (2026-09-21). Once the attitude loop is
	// allowed to make pitch moment out of body-x force, part of Fx is no
	// longer a mission objective -- it IS the pitch moment, just expressed
	// on the other side of the 0.0549 N*m/N lever. Sacrificing it "first"
	// would delete the pitch command and silently restore the behaviour
	// this whole change exists to remove. So the lever component rides at
	// MOMENT priority: held through step 1, and scaled with the moment in
	// step 2 because the two are the same physical quantity. Only the
	// position loop's share of Fx yields first.
	//
	// Body FRD and the allocator's FLU agree on x (the transform is a
	// 180 deg rotation about x), so fx_lever_flu needs no conversion --
	// see frdToAllocatorFlu().
	if (!_allocation.allocate(F_flu, M_flu).saturated) {
		return;
	}

	// Split Fx: the lever share is protected, the rest is sacrificeable.
	const matrix::Vector2f F_xy(F_flu(0) - fx_lever_flu, F_flu(1));

	// 1) Give up the position loop's horizontal force. Bisect the largest
	//    fraction of it that still lets the full moment (and the lever
	//    that is part of it) through.
	{
		matrix::Vector3f F_try(fx_lever_flu, 0.f, F_flu(2));

		if (!_allocation.allocate(F_try, M_flu).saturated) {
			float lo = 0.f, hi = 1.f;

			for (int i = 0; i < kFitIterations; i++) {
				const float mid = 0.5f * (lo + hi);
				F_try(0) = fx_lever_flu + F_xy(0) * mid;
				F_try(1) = F_xy(1) * mid;

				if (_allocation.allocate(F_try, M_flu).saturated) { hi = mid; }

				else { lo = mid; }
			}

			F_flu(0) = fx_lever_flu + F_xy(0) * lo;
			F_flu(1) = F_xy(1) * lo;
			return;
		}
	}

	// The position loop's horizontal force is fully spent; it stays at
	// zero from here. The lever survives into step 2 and is scaled with
	// the moment there.
	F_flu(0) = fx_lever_flu;
	F_flu(1) = 0.f;

	// 2) Give up moment, keeping the collective. The lever scales WITH the
	//    moment -- it is the same command on the other side of the lever
	//    arm, so shrinking one without the other would leave the vehicle
	//    holding a body-x force that no longer corresponds to any pitch
	//    request.
	{
		const matrix::Vector3f F_try(0.f, 0.f, F_flu(2));

		if (!_allocation.allocate(F_try, matrix::Vector3f()).saturated) {
			float lo = 0.f, hi = 1.f;

			for (int i = 0; i < kFitIterations; i++) {
				const float mid = 0.5f * (lo + hi);
				const matrix::Vector3f F_mid(fx_lever_flu * mid, 0.f, F_flu(2));

				if (_allocation.allocate(F_mid, M_flu * mid).saturated) { hi = mid; }

				else { lo = mid; }
			}

			F_flu(0) = fx_lever_flu * lo;
			M_flu *= lo;
			return;
		}
	}

	// Nothing of the moment survived; the lever goes with it.
	F_flu(0) = 0.f;

	// 3) The collective alone is out of range. Nothing can be held; scale
	//    it back to the largest deliverable value and zero the moment.
	M_flu.setZero();
	{
		const float fz = F_flu(2);
		float lo = 0.f, hi = 1.f;

		for (int i = 0; i < kFitIterations; i++) {
			const float mid = 0.5f * (lo + hi);

			if (_allocation.allocate(matrix::Vector3f(0.f, 0.f, fz * mid),
						 matrix::Vector3f()).saturated) { hi = mid; }

			else { lo = mid; }
		}

		F_flu(2) = fz * lo;
	}
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
	_gains.vel_xy_i_lim = _param_fr_vel_xy_i_lim.get();

	_gains.vel_z_ff = _param_fr_vel_z_ff.get();
	_gains.vel_z_i = _param_fr_vel_z_i.get();
	_gains.vel_z_d = _param_fr_vel_z_d.get();
	_gains.vel_z_grav_ff = _param_fr_vel_z_grav_ff.get();
	_gains.vel_z_i_lim = _param_fr_vel_z_i_lim.get();

	_gains.vel_xy_max = _param_fr_vel_xy_max.get();
	_gains.vel_z_max_up = _param_fr_vel_z_max_up.get();
	_gains.vel_z_max_dn = _param_fr_vel_z_max_dn.get();

	_gains.att_p = _param_fr_att_p.get();

	_gains.rate_r_ff = _param_fr_rate_r_ff.get();
	_gains.rate_r_i = _param_fr_rate_r_i.get();
	_gains.rate_r_d = _param_fr_rate_r_d.get();
	_gains.rate_r_i_lim = _param_fr_rate_r_i_lim.get();

	_gains.rate_p_ff = _param_fr_rate_p_ff.get();
	_gains.rate_p_i = _param_fr_rate_p_i.get();
	_gains.rate_p_d = _param_fr_rate_p_d.get();
	_gains.rate_p_i_lim = _param_fr_rate_p_i_lim.get();

	_gains.rate_yaw_ff = _param_fr_rate_yaw_ff.get();
	_gains.rate_yaw_i = _param_fr_rate_yaw_i.get();
	_gains.rate_yaw_d = _param_fr_rate_yaw_d.get();

	// PositionVelocityControl's sphere-saturation bounds (2026-09-17
	// rework, see that class's header comment and .claude/plans/
	// read-mc-pos-contorl-and-can-greedy-pearl.md): a single combined
	// force-magnitude ceiling plus a horizontal margin, replacing the
	// previous independent per-axis output-limit boxes (kPosVelFzLimit=28N/
	// kPosVelFxyLimit=20N) that let the velocity loop demand roughly
	// double the correct per-rotor hover trim before the allocator ever
	// saw it. Derived the same way those boxes were -- FoldrotorAllocation's
	// kMaxThrust (2 rotors x 15 N = 30 N ceiling, some margin held back)
	// and M0 geometry (kS1y ~= 0.2684 m roll arm, kS1z ~= 0.0301 m pitch
	// arm) -- NOT a re-derivation of Control_Alloc.m. These are placeholders
	// to stop the runaway, unverified against a logged clean hover -- see
	// findings.md OPEN ITEM and PositionVelocityControl.hpp OPEN ITEM (a).
	static constexpr float kPosVelForceLimit = 28.f;      // N, combined magnitude ceiling
	static constexpr float kPosVelForceXYMargin = 8.f;    // N, horizontal force held back when saturating Z

	// ADDED 2026-09-21 (5). The sphere above is the wrong SHAPE for this
	// vehicle, and that -- not its radius -- was the dominant cause of the
	// in-air flip.
	//
	// A sphere treats horizontal and vertical force as interchangeable: at
	// the hover-ish Fz = 17.36 N it permits |Fxy| up to
	// sqrt(28^2 - 17.36^2) = 22.0 N. But this vehicle cannot produce force
	// in an arbitrary direction. Every newton of horizontal force has to
	// come from tilting the rotors (alpha, beta), and both angles are
	// railed at +-kMaxTilt = 0.79 rad. Bisecting the REAL allocator for the
	// largest feasible |Fxy| in the worst-case horizontal direction
	// (sitl_testing/allocation_study/envelope.py):
	//
	//   moments reserved   |Fxy| feasible at Fz = 17.36 N
	//   -----------------  ------------------------------
	//   none                        6.38 N
	//   Mx = 3.8 only               0.00 N
	//   moderate, all three         0.00 N
	//
	// So the loop was permitted to ask for 22 N of a quantity whose true
	// ceiling is ~6 N with NO moment reserved at all, and ~0 N once the
	// rate loop's demand is accounted for. The position loop won that
	// contest every cycle (it is upstream), drove alpha/beta onto their
	// rails, and left the rate loop's moment to be delivered by whatever
	// angular range happened to remain -- which is how a commanded My came
	// back SIGN-FLIPPED, and why the allocator reported saturation on 95%
	// of samples.
	//
	// The fix is a hard horizontal cap, applied after the sphere, sized to
	// leave the rails free for moments. 1.0 N buys 0.64 m/s^2 of
	// horizontal acceleration -- weak, but it is genuinely what a
	// two-rotor vehicle with +-45 deg of tilt has left after it finishes
	// holding itself up and keeping itself upright. Attitude outranks
	// position here; that ordering is not tunable, it is the airframe.
	// momentEnvelopeAtThrust()'s table is measured holding exactly this
	// much horizontal force in reserve, so the two constants are a matched
	// pair -- change one and regenerate the other with
	// allocation_study/gentable.py. The constant itself is declared in
	// FoldrotorControl.hpp so the regression test can see it.

	// RESOLVED 2026-09-18 (findings.md): these were placeholders that
	// overestimated the allocator's true per-axis moment authority --
	// verified by numerically maximizing |Mx|/|My|/|Mz| over the full
	// actuator box (F1,F2 in [0,15] N, alpha1,2/beta1,2 in [-0.79,0.79]
	// rad) via FoldrotorAllocation's own forward map, cross-checked with
	// both a brute-force grid search and scipy.optimize (multiple random
	// restarts), all three methods agreeing to 5+ significant figures:
	//   true max |Mx| ~= 4.06 N*m  (was 8.0,  ~2.0x too high)
	//   true max |My| ~= 0.90 N*m  (was 1.5,  ~1.67x too high)
	//   true max |Mz| ~= 5.77 N*m  (was 6.0,  ~1.04x, negligible)
	// Because setOutputLimits()/updateIntegral()'s conditional-integration
	// anti-windup (AttitudeRateControl.hpp) uses these constants to decide
	// "am I saturated", overestimating them left the rate loop's own
	// saturation detection blind across the gap between the true ceiling
	// and the configured one -- the rate loop believed it was within
	// bounds while the allocator was already clamping individual channels
	// to their rails trying to deliver an unachievable moment, producing
	// a bang-bang limit-cycle chatter on alpha/beta traced through five
	// reproduced free-flight crashes (findings.md, 2026-09-18 entries).
	// Set here to the true achievable maxima with a small margin held
	// back, same pattern as kPosVelForceLimit/kPosVelForceXYMargin above.
	//
	// REVISED 2026-09-21 (findings.md 2026-09-21, and see
	// FoldrotorAllocation.hpp OPEN ITEM (d)). The 4.06/0.90/5.77 figures
	// above maximize each moment over the whole actuator box with the
	// FORCE left free -- i.e. they are reachable only if the vehicle is
	// willing to stop holding itself up. The number the rate loop
	// actually needs is the moment reachable *while hovering*: Fz = the
	// vehicle's weight, Fx = Fy = 0, other two moments 0. Re-measured
	// under that constraint against the sign-corrected allocator
	// (sitl_testing/allocation_study/, T7):
	//   max |Mx| ~= 3.83 N*m   (vs 4.06 unconstrained)
	//   max |My| ~= 0.34 N*m   (vs 0.90 unconstrained -- 2.6x smaller)
	//   max |Mz| ~= 3.85 N*m   (vs 5.77 unconstrained)
	// Pitch is the axis that matters: it is produced almost entirely by
	// the small drag-coupling term, so 0.2 N*m already demands the full
	// +-0.79 rad fold deflection. Leaving kRateMyLimit at 0.85 let the
	// rate loop ask for ~2.5x what hover flight can deliver, so the
	// allocator clamped while the rate loop still believed itself
	// unsaturated -- exactly the blindness the 2026-09-18 revision was
	// meant to remove, just at a smaller scale.
	// The three constants themselves are declared in FoldrotorControl.hpp
	// so the regression test can see them; this comment is their rationale.

	// Push into the cascade objects. The *integrator* clamp is a separate
	// mechanism from setOutputLimits()/conditional integration:
	// FR_VEL_Z_I_LIM/FR_VEL_XY_I_LIM/FR_RATE_R_I_LIM/FR_RATE_P_I_LIM (2026-09-09,
	// 2026-09-1x, findings.md) bound each loop's accumulated integral
	// directly. Yaw's rate integrator stays +/-infinity: FR_RATE_YAW_I is
	// 0, so there is nothing to bound.
	_pos_vel_control.setPositionGain(_gains.pos_p);
	_pos_vel_control.setVelocityGains(_gains.vel_xy_ff, _gains.vel_xy_i, _gains.vel_xy_d,
					  _gains.vel_z_ff, _gains.vel_z_i, _gains.vel_z_d);
	_pos_vel_control.setGravityFeedforward(_gains.vel_z_grav_ff);
	_pos_vel_control.setIntegratorLimit(matrix::Vector3f(_gains.vel_xy_i_lim, _gains.vel_xy_i_lim,
					    _gains.vel_z_i_lim));
	_pos_vel_control.setForceLimits(kPosVelForceLimit);
	_pos_vel_control.setHorizontalForceMargin(kPosVelForceXYMargin);
	_pos_vel_control.setHorizontalForceLimit(kPosVelForceXYLimit);
	_pos_vel_control.setVelocityLimits(_gains.vel_xy_max, _gains.vel_z_max_up, _gains.vel_z_max_dn);

	_att_rate_control.setAttitudeGain(_gains.att_p);
	_att_rate_control.setRateGains(_gains.rate_r_ff, _gains.rate_r_i, _gains.rate_r_d,
				       _gains.rate_p_ff, _gains.rate_p_i, _gains.rate_p_d,
				       _gains.rate_yaw_ff, _gains.rate_yaw_i, _gains.rate_yaw_d);
	_att_rate_control.setIntegratorLimit(matrix::Vector3f(_gains.rate_r_i_lim, _gains.rate_p_i_lim, INFINITY));

	// Seed the output limits with the envelope peaks. ALL THREE axes are
	// re-scheduled every cycle in Run() against the collective Fz actually
	// commanded -- these constants are only the peak of a curve that
	// collapses at both ends, see momentEnvelopeAtThrust() and findings.md
	// 2026-09-21 (5). This call still matters: it is what the limits are
	// before the first Run().
	_att_rate_control.setOutputLimits(matrix::Vector3f(-kRateMxLimit, -kRateMyLimit, -kRateMzLimit),
					  matrix::Vector3f(kRateMxLimit, kRateMyLimit, kRateMzLimit));

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
		const matrix::Quatf q_att(attitude.q);
		_euler = matrix::Eulerf(q_att);

		// Full body<-NED rotation, kept alongside _euler for the FORCE
		// path only (2026-09-21 (7)). The attitude stage still consumes
		// _euler, because its setpoint is expressed as phi_sp/theta_sp/
		// psi_sp; the force path must not, see inertialToBody().
		_R_ned_to_body = matrix::Dcmf(q_att).transpose();

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
		//
		// Gate reset added 2026-09-11 alongside the integrator reset:
		// resetIntegral() zeroes state INSIDE PositionVelocityControl /
		// AttitudeRateControl, but _F_b/_M_b themselves are only
		// recomputed when _pos_vel_gate / _attitude_gate next fire --
		// up to one full gate period later (20 ms / 4 ms). Until then,
		// _F_b/_M_b (and actuator_motors/servos, published every cycle
		// from whatever they currently hold) keep the PRE-reset wrench
		// even though the integrator behind it has already been zeroed.
		// CascadeRateGate::reset() makes the next due() call fire
		// immediately, same semantics as a fresh construction, so the
		// wrench is recomputed this cycle instead.
		if (_armed_prev && !_vehicle_control_mode.flag_armed) {
			_pos_vel_control.resetIntegral();
			_att_rate_control.resetIntegral();
			_pos_vel_gate.reset();
			_attitude_gate.reset();
			_wrench_lp_reset = true;
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
		//
		// Same gate-staleness reasoning as the disarm branch above: reset
		// _pos_vel_gate/_attitude_gate too, so the freshly-zeroed
		// integrator's effect reaches _F_b/_M_b this cycle rather than up
		// to one gate period after arming.
		if (!_armed_prev && _vehicle_control_mode.flag_armed) {
			_pos_vel_control.resetIntegral();
			_att_rate_control.resetIntegral();
			_pos_vel_gate.reset();
			_attitude_gate.reset();
			_wrench_lp_reset = true;
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
				_F_b = foldrotor::inertialToBody(F_i, _R_ned_to_body);

				// Cap the BODY-frame horizontal force (2026-09-21 (9)).
				// A different constraint from the inertial cap in
				// PositionVelocityControl, and the one that keeps pitch
				// stable.
				//
				// The rotors sit 5.5 cm BELOW the centre of mass
				// (FoldrotorAllocation::kS1z = -0.0549 m, confirmed
				// against model.sdf's forward kinematics). Any
				// body-forward force is therefore applied below the CoM
				// and produces a NOSE-UP moment,
				// My_frd = -kS1z * Fx = +0.055 * Fx.
				//
				// That closes a POSITIVE FEEDBACK loop through the full
				// rotation above: pitching nose-up makes the velocity
				// loop ask for body-forward force in order to keep
				// pushing up in NED, and that force pitches the vehicle
				// further nose-up. Loop gain dMy/dtheta = |kS1z| * Fz
				// ~= 0.055 * 17 = 0.94 N*m/rad against a pitch authority
				// of ~0.146 N*m, so the level equilibrium is unstable
				// beyond 0.146/0.94 = 0.155 rad = 8.9 deg. Measured: the
				// vehicle departed at ~9 deg on every run and ran to a
				// 73 deg mechanical stop.
				//
				// Capping in the INERTIAL frame (kPosVelForceXYLimit)
				// does not bound this: once tilted, the body-frame
				// horizontal force is dominated by the rotated
				// COLLECTIVE (Fx_body = Fz * sin(theta)), not by the
				// horizontal command. The cap has to be applied here,
				// after the rotation, where the destabilising moment is
				// actually set.
				//
				// Consequence, stated plainly: this airframe cannot
				// fully compensate its own attitude in the force path.
				// While tilted it will not hold altitude, because the
				// force that would do so is the same force that stops it
				// ever getting level again. Recovering attitude first
				// and accepting the altitude error is the only ordering
				// that converges.
				const float fxy_body = matrix::Vector2f(_F_b(0), _F_b(1)).norm();

				if (fxy_body > kBodyForceXYLimit && fxy_body > FLT_EPSILON) {
					const float scale = kBodyForceXYLimit / fxy_body;
					_F_b(0) *= scale;
					_F_b(1) *= scale;
				}
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
		//
		// Re-schedule ALL THREE moment limits against the collective
		// thrust the position loop is asking for THIS cycle, before the
		// rate loop runs. The kRateM*Limit constants are only the peak of
		// an envelope that collapses at both ends of the thrust range, so
		// using them unscheduled both lets the rate loop command moments
		// the allocator must clamp and blinds the conditional-integration
		// anti-windup, which keys off these same bounds. Scheduling roll
		// alone (2026-09-21 (4)) was not enough: pitch and yaw collapse
		// too once all three are demanded together, which is every cycle.
		// findings.md 2026-09-21 (5).
		//
		// _F_b is FRD, where Fz is negative-up; momentEnvelopeAtThrust()
		// takes the magnitude.
		matrix::Vector3f m_limit = momentEnvelopeAtThrust(_F_b(2));

		// The table was measured with body-x force pinned in reserve, so
		// its pitch row is the drag-path-only authority (0.342 N*m peak).
		// The tilt lever below adds |kPitchLeverFxLimit| * 0.0549 N*m on
		// top of that, and the rate loop has to know: these same bounds
		// drive the conditional-integration anti-windup, so a limit below
		// the true ceiling makes the loop believe it is saturated when it
		// is not -- the mirror image of the 09-18 defect, and just as
		// blinding. Measured relation is additive to within 1% over the
		// whole range (max|My| = 0.342 + 0.0549*Fx, checked against the
		// allocator's own forward map at the actuator box corner:
		// predicted 1.506, measured 1.490 N*m).
		m_limit(1) += _param_fr_pitch_lever.get() * kPitchLeverFxLimit
			      * foldrotor::FoldrotorAllocation::pitchLeverFrd();

		_att_rate_control.setOutputLimits(-m_limit, m_limit);

		const matrix::Vector3f rate(angular_velocity.xyz);
		const matrix::Vector3f rate_dot(angular_velocity.xyz_derivative);
		_M_b = _att_rate_control.updateRate(rate, rate_dot, dt, landed);

		// --- Pitch tilt lever (2026-09-21, architecture change) --------
		//
		// Hand the attitude loop the body-x force as a pitch actuator.
		//
		// WHY. Roll is made by differential THRUST and yaw by TILT, both
		// cheap. Pitch has no moment arm at all on a side-by-side rotor
		// pair, so M0 row 4 leaves only two ways to make it:
		//
		//   My_flu = kS1z * (T1x + T2x)  +  k * (T1y - T2y)
		//            \___ tilt lever ___/    \__ drag coupling __/
		//              |kS1z| = 0.0549          |k| = 0.0223
		//
		// The tilt-lever term is 2.5x stronger per unit thrust, but using
		// it produces net body-x force -- which the Fx row forbids when
		// Fx is pinned to whatever the position loop asked for. So the
		// allocator is forced down the drag path, and the drag path is
		// brutally stiff: 0.1 N*m of pitch costs 16.3 deg of differential
		// FOLD (d(alpha)/d(My) = 2.70 rad/N*m, Jacobian of allocate() at
		// hover). Fold is the heavy joint -- 0.0136 kg*m^2 about its
		// axis, so model.sdf's p=20/d=0.5 joint PID is a 6.1 Hz, zeta =
		// 0.48 actuator.
		//
		// That cannot work. findings.md (9) measured the pitch axis as
		// OPEN-LOOP UNSTABLE at 2.77 Hz (dMy/dtheta = 0.94 N*m/rad on
		// Iyy = 0.00309), which wants ~8.3 Hz of loop bandwidth. A 6.1 Hz
		// actuator does not provide it at ANY gain, which is why the
		// 09-18 limit correction, the 09-21 Iyy/Ixx rescaling and the
		// envelope fit each removed a real defect and pitch departed
		// anyway.
		//
		// Routing pitch through the lever instead moves it onto the TILT
		// joint, which carries 0.0012 kg*m^2 -- 11x lighter, so the same
		// PID is 20.5 Hz with zeta = 1.61. 20.5 >> 8.3: the bandwidth gap
		// closes. Travel drops too, 16.3 -> 6.8 deg per 0.1 N*m, and the
		// differential fold command for pitch goes to exactly zero.
		//
		// HOW. Commanding Fx = My / 0.0549 alongside My does not add a
		// second moment -- the allocator still realises exactly the My
		// asked for. It changes which actuator realises it: the lever
		// then supplies FR_PITCH_LEVER of the moment and the drag path
		// the remainder, so at 1.0 the differential Ty (and with it
		// alpha) falls out entirely. No change to allocate() itself
		// (.claude/CLAUDE.md rule 7) -- only to the wrench it is handed.
		//
		// THE COST, stated plainly: 1.82 N of body-x force per 0.1 N*m of
		// pitch. The vehicle translates while it corrects attitude. That
		// is the same ordering findings.md (9) already accepted --
		// attitude first, position error second -- applied to the
		// horizontal axis instead of the vertical one.
		//
		// --- Command-path bandwidth limit (2026-09-22) ---------------
		//
		// Hold the commanded wrench to something the servos can execute.
		// See the FR_WRENCH_LP block in the header for the measurement
		// that motivates it and for why this is a wrench filter rather
		// than a slew limit on alpha/beta.
		//
		// Placed after the rate loop's own output clamp but before the
		// pitch lever, frdToAllocatorFlu() and fitWrenchToEnvelope(), so
		// everything downstream -- the fit, the allocator, and the
		// debug_array the plots read -- sees the same wrench the
		// actuators are actually asked for. Filtering in FRD rather than
		// FLU is arbitrary (frdToAllocatorFlu() is linear, so the two
		// commute); FRD is where _F_b/_M_b natively live.
		//
		// NOT written back into _F_b/_M_b, for the same reason the pitch
		// lever is not: those stay each loop's own output for
		// print_status()/getForceBody()/getMomentBody() and for the
		// hold-on-invalid-input path.
		//
		// FR_WRENCH_LP <= 0 bypasses the filter entirely, which both
		// gives a way to fly the unfiltered command for comparison and
		// keeps every pre-2026-09-22 test exercising the original path.
		const float wrench_lp_hz = _param_fr_wrench_lp.get();

		matrix::Vector3f F_pos_cmd = _F_b;
		matrix::Vector3f M_cmd = _M_b;
		applyWrenchLowPass(F_pos_cmd, M_cmd, wrench_lp_hz, dt);

		// Budgeted SEPARATELY from kBodyForceXYLimit, which fences off
		// the position loop's body-x request. Same physical quantity,
		// opposite intent: the position loop's Fx is the destabilising
		// direction (tilted -> wants forward force -> pitches further),
		// the attitude loop's carries the correcting sign. One shared
		// budget is what left pitch with no actuator -- at 1.0 N that cap
		// was holding the lever to 0.055 N*m, 16% of the 0.342 N*m the
		// airframe already had.
		//
		// NOT written back into _F_b. _F_b is the POSITION loop's output
		// and is recomputed only when _pos_vel_gate fires (50 Hz) while
		// this runs every cycle (250 Hz), so `_F_b(0) += ...` would add
		// the lever five times per position update and diverge. _F_b also
		// stays the position-loop force for print_status()/getForceBody()
		// and for the next cycle's hold-on-invalid-input path.
		//
		// Derived from the FILTERED pitch moment M_cmd(1), not from
		// _M_b (2026-09-22). The lever IS the pitch moment, just
		// expressed on the other side of the 0.0549 N*m/N arm -- see
		// fitWrenchToEnvelope(), which holds it at moment priority for
		// exactly that reason. Taking it from the unfiltered moment
		// would hand fitWrenchToEnvelope() an fx_lever_flu that no
		// longer matches the Fx actually inside F_cmd, so the fit would
		// protect the wrong amount of body-x force.
		const float pitch_lever_fx =
			math::constrain(_param_fr_pitch_lever.get() * M_cmd(1)
					/ foldrotor::FoldrotorAllocation::pitchLeverFrd(),
					-kPitchLeverFxLimit, kPitchLeverFxLimit);

		matrix::Vector3f F_cmd = F_pos_cmd;
		F_cmd(0) += pitch_lever_fx;

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
		// frame for print_status() / getForceBody() / getMomentBody()
		// above.
		matrix::Vector3f F_alloc = frdToAllocatorFlu(F_cmd);
		matrix::Vector3f M_alloc = frdToAllocatorFlu(M_cmd);

		// Fit the wrench into what the actuators can actually deliver
		// BEFORE allocating it (2026-09-21 (8)). Without this the
		// allocator clamps per rotor and does not redistribute, so an
		// infeasible wrench comes back as an arbitrary one -- measured
		// worst case, a sign-flipped My.
		fitWrenchToEnvelope(F_alloc, M_alloc, pitch_lever_fx);

		_alloc_out = _allocation.allocate(F_alloc, M_alloc);

		// Publish the allocation output for live inspection over MAVLink
		// (sitl_testing/plot_hover.py), unconditional (armed or not) --
		// _alloc_out itself is computed unconditionally above, so this
		// mirrors that rather than the arm-gated actuator publish below.
		// A binary uORB publish, not PX4_INFO -- doesn't reintroduce the
		// console-spam problem the "no periodic log" decision below
		// avoided.
		// ONE message, not two. Until 2026-09-21 this published a
		// second debug_array ("fr_wrench", id 1) immediately after the
		// one below. debug_array is a single-instance topic with queue
		// depth 1, so every subscriber slower than this loop -- the
		// logger AND MavlinkStreamDebugFloatArray, which both use a
		// plain uORB::Subscription -- only ever saw the LAST write.
		// Measured in log 2026-09-21/11_19_09.ulg: 4041 debug_array
		// records, 4041 of them id 1, zero fr_alloc. The allocator
		// output was invisible in the log and over MAVLink for as long
		// as the diagnostic existed, which is why
		// sitl_testing/plot_hover.py's force and servo-angle subplots
		// came up empty.
		//
		// debug_array carries 58 floats (DebugArray.msg ARRAY_SIZE), so
		// both payloads fit in one message with room to spare. Layout,
		// mirrored in plot_hover.py and in the header's
		// kDebug* constants:
		//   [0..6]   allocator OUTPUT: F1 F2 alpha1 alpha2 beta1 beta2 saturated
		//   [7..12]  allocator INPUT wrench (FLU, post-fit): Fx Fy Fz Mx My Mz
		debug_array_s debug_array{};
		debug_array.timestamp = now;
		debug_array.id = 0;
		strncpy(debug_array.name, "fr_alloc", sizeof(debug_array.name) - 1);
		debug_array.data[kDebugF1]        = _alloc_out.F1;
		debug_array.data[kDebugF2]        = _alloc_out.F2;
		debug_array.data[kDebugAlpha1]    = _alloc_out.alpha1;
		debug_array.data[kDebugAlpha2]    = _alloc_out.alpha2;
		debug_array.data[kDebugBeta1]     = _alloc_out.beta1;
		debug_array.data[kDebugBeta2]     = _alloc_out.beta2;
		debug_array.data[kDebugSaturated] = _alloc_out.saturated ? 1.f : 0.f;

		// The allocator's INPUT wrench (F_alloc/M_alloc, FLU, after
		// fitWrenchToEnvelope()). Needed alongside the output to tell
		// whether a bad allocation came from a bad wrench upstream or
		// from the allocator itself.
		debug_array.data[kDebugWrenchFx] = F_alloc(0);
		debug_array.data[kDebugWrenchFy] = F_alloc(1);
		debug_array.data[kDebugWrenchFz] = F_alloc(2);
		debug_array.data[kDebugWrenchMx] = M_alloc(0);
		debug_array.data[kDebugWrenchMy] = M_alloc(1);
		debug_array.data[kDebugWrenchMz] = M_alloc(2);

		_debug_array_pub.publish(debug_array);


		// DIAGNOSTIC (temporary, see the TraceSample comment in the
		// header): per-cycle capture, because debug_array above is
		// sampled by the logger at ~250 Hz and the failure being chased
		// develops inside 4-8 ms. Dump with `foldrotor_control trace`.
		if (_trace_enabled && !_trace_frozen) {
			TraceSample &s = _trace[_trace_head % kTraceLen];
			s.t = now;
			const matrix::Vector3f &rate_sp = _att_rate_control.getRateSetpoint();

			for (int i = 0; i < 3; i++) {
				s.rate[i] = rate(i);
				s.rate_dot[i] = rate_dot(i);
				s.rate_sp[i] = rate_sp(i);
				s.m_b[i] = _M_b(i);
				s.f_b[i] = _F_b(i);
			}

			const bool is_armed = _vehicle_control_mode.flag_armed;
			s.m_limit[0] = m_limit(0);
			s.m_limit[1] = m_limit(1);
			s.m_limit[2] = m_limit(2);
			s.saturated = _alloc_out.saturated ? 1 : 0;
			s.armed = is_armed ? 1 : 0;
			_trace_head++;

			// Freeze on the disarm edge so a failsafe disarm leaves the
			// buffer holding the run that caused it, rather than letting
			// the post-crash idle overwrite it before it can be dumped.
			// Note _armed_prev cannot serve here -- it is already updated
			// earlier this cycle, so this needs its own edge memory.
			if (_trace_freeze_on_disarm && _trace_was_armed && !is_armed) {
				_trace_frozen = true;
			}

			// Capture-from-arm mode. The ring holds only ~2 s, so by the
			// time a run ends it contains the post-crash idle, which is
			// useless for judging the control law (the vehicle is lying on
			// the ground and every angular acceleration is contact, not
			// commanded moment). Latching the arm index and freezing once
			// the buffer has filled from there keeps the FIRST 2 s after
			// arming instead -- the only window where a moment-vs-response
			// sign check means anything.
			if (!_trace_was_armed && is_armed) {
				_trace_arm_head = _trace_head;
			}

			if (_trace_capture_from_arm && is_armed
			    && (_trace_head - _trace_arm_head) >= (uint32_t)kTraceLen) {
				_trace_frozen = true;
			}

			_trace_was_armed = is_armed;
		}

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

		// No periodic log here (2026-09-12) -- matches mc_pos_control /
		// mc_att_control / mc_rate_control, none of which log on a
		// timer. `foldrotor_control status` (below) gives the same
		// numbers on demand; a per-cycle or per-second PX4_INFO was
		// drowning out commander/EKF2 output during arming tests.
	}

	perf_end(_loop_perf);
}

int FoldrotorControl::print_status()
{
	PX4_INFO("status: step 4e part 2 -- cascade + allocation wired, publishing actuator_motors/actuator_servos");
	PX4_INFO("armed=%d", _vehicle_control_mode.flag_armed);
	PX4_INFO("F_b = [%.3f, %.3f, %.3f] N (body/FRD)", (double)_F_b(0), (double)_F_b(1), (double)_F_b(2));
	PX4_INFO("M_b = [%.4f, %.4f, %.4f] N*m (body/FRD)", (double)_M_b(0), (double)_M_b(1), (double)_M_b(2));
	// Bench-context check (2026-09-14, findings.md): confirms the new
	// setOutputLimits()/setIntegratorLimit() bounds actually hold each
	// accumulated integral, not just that the code compiles.
	const matrix::Vector3f vel_int = _pos_vel_control.getIntegral();
	const matrix::Vector3f rate_int = _att_rate_control.getIntegral();
	PX4_INFO("vel_int = [%.4f, %.4f, %.4f] N (inertial/NED; limits +/-%.1f, +/-%.1f, +/-%.1f)",
		 (double)vel_int(0), (double)vel_int(1), (double)vel_int(2),
		 (double)_gains.vel_xy_i_lim, (double)_gains.vel_xy_i_lim, (double)_gains.vel_z_i_lim);
	PX4_INFO("rate_int = [%.4f, %.4f, %.4f] N*m (body/FRD; limits +/-%.1f, +/-%.1f, inf)",
		 (double)rate_int(0), (double)rate_int(1), (double)rate_int(2),
		 (double)_gains.rate_r_i_lim, (double)_gains.rate_p_i_lim);
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
		 "Gazebo (bench test required before free flight); PositionVelocityControl now uses "
		 "mc_pos_control-style sphere saturation + tracking anti-windup (2026-09-17 rework, see "
		 "PositionVelocityControl.hpp), with a placeholder combined force limit/margin unverified "
		 "against a logged clean hover; FR_VEL_Z_I_LIM=3.0N / FR_VEL_XY_I_LIM=15.0N bound the "
		 "velocity integrators directly; EKF reset-counter adjustment not implemented; integrator "
		 "reset handles disarm only, not \"mode entry\"");

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
	// DIAGNOSTIC (temporary, see the TraceSample comment in the header).
	if (argc > 0 && !strcmp(argv[0], "trace")) {
		FoldrotorControl *obj = get_instance<FoldrotorControl>(desc);

		if (obj == nullptr) {
			PX4_ERR("module not running");
			return 1;
		}

		if (argc > 1 && !strcmp(argv[1], "reset")) {
			obj->_trace_head = 0;
			obj->_trace_frozen = false;
			PX4_INFO("trace reset, capture re-armed");
			return 0;
		}

		// CSV to stdout so it can be redirected straight into a file and
		// read with the same tooling as the .ulg exports. Oldest first.
		const uint32_t head = obj->_trace_head;
		const uint32_t count = head < (uint32_t)kTraceLen ? head : (uint32_t)kTraceLen;
		const uint32_t first = head - count;

		printf("t_us,armed,sat,mx_lim,my_lim,mz_lim,p,q,r,pdot,qdot,rdot,p_sp,q_sp,r_sp,Mx,My,Mz,Fx,Fy,Fz\n");

		for (uint32_t i = 0; i < count; i++) {
			const TraceSample &s = obj->_trace[(first + i) % kTraceLen];
			printf("%llu,%u,%u,%.4f,%.4f,%.4f,"
			       "%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f\n",
			       (unsigned long long)s.t, s.armed, s.saturated,
			       (double)s.m_limit[0], (double)s.m_limit[1], (double)s.m_limit[2],
			       (double)s.rate[0], (double)s.rate[1], (double)s.rate[2],
			       (double)s.rate_dot[0], (double)s.rate_dot[1], (double)s.rate_dot[2],
			       (double)s.rate_sp[0], (double)s.rate_sp[1], (double)s.rate_sp[2],
			       (double)s.m_b[0], (double)s.m_b[1], (double)s.m_b[2],
			       (double)s.f_b[0], (double)s.f_b[1], (double)s.f_b[2]);
		}

		PX4_INFO("dumped %u samples (frozen=%d)", (unsigned)count, (int)obj->_trace_frozen);
		return 0;
	}

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
	PRINT_MODULE_USAGE_COMMAND_DESCR("trace",
					 "Dump the per-cycle diagnostic ring buffer as CSV (temporary; see findings.md). "
					 "`trace reset` clears it and re-arms capture after a disarm freeze.");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int foldrotor_control_main(int argc, char *argv[])
{
	return ModuleBase::main(FoldrotorControl::desc, argc, argv);
}
