/****************************************************************************
 *
 * foldrotor_control — position P -> velocity PID -> inertial-frame desired
 * force (step 4a).
 *
 * REWORKED (2026-09-17) to match mc_pos_control's PositionControl class
 * structure (src/modules/mc_pos_control/PositionControl/PositionControl.
 * {hpp,cpp}), at the user's explicit request, after diagnosing that this
 * class's previous bespoke design (a literal Simulink transcription with
 * uniform conditional-integration anti-windup and independent per-axis
 * output-limit boxes) was letting the velocity loop demand roughly double
 * the correct per-rotor hover trim (~15 N/rotor observed in SITL vs a
 * correct ~7.63 N/rotor, against a bench-measured ~15.27 N total hover
 * weight) before FoldrotorAllocation ever saw it — the allocator itself is
 * unit-tested correct (FoldrotorControlTest.cpp,
 * HoverProducesEvenSplitZeroTilt). See .claude/plans/
 * read-mc-pos-contorl-and-can-greedy-pearl.md for the full diagnosis and
 * decision record.
 *
 * THIS IS A DELIBERATE, RECORDED DEPARTURE from controller.md's stated
 * source of truth ("the validated Simulink model is the source of truth
 * for the math") for this loop's STRUCTURE (P/PID form, anti-windup
 * mechanism) only — the Simulink-derived GAIN VALUES (FR_POS_P=3.0,
 * FR_VEL_*_FF/I/D) are unchanged; only the mechanism around them changed.
 * See controller.md/controller_params.md for the updated record.
 *
 * What was ported from PositionControl.cpp, and what was deliberately
 * NOT ported (the user's explicit request: skip "position to attitude
 * algebra and cascade", since foldrotor3 is fully-actuated and translates
 * by independent per-rotor thrust vectoring, not by leaning the body —
 * already flagged as "not transferable" in reference/px4-module-patterns.md):
 *
 *   PORTED:
 *   - _positionControl()-equivalent: position P -> velocity setpoint
 *     (PositionControl.cpp:127-141), minus the NaN-aware setpoint-
 *     combining machinery (this module has no independent velocity-
 *     setpoint input path). Velocity-magnitude limiting itself WAS
 *     ported (setVelocityLimits(), added 2026-09-17 -- see OPEN ITEM (b)
 *     below, now resolved) once real SITL testing showed it was load-
 *     bearing, not optional; the horizontal clamp collapses to a plain
 *     magnitude scale rather than PositionControl.cpp's full priority-
 *     blend logic, since there's no feedforward vel_sp term to blend
 *     against here.
 *   - _velocityControl()-equivalent: PID on velocity error -> desired
 *     force (PositionControl.cpp:143-150), already in this loop's native
 *     force domain (see decision 1 below) — no acceleration/mass
 *     conversion needed, unlike mc_pos_control.
 *   - Vertical-priority sphere saturation (PositionControl.cpp:163-186),
 *     made SYMMETRIC on Z rather than mc_pos_control's uni-directional
 *     (thrust-can-only-point-up) convention, since this vehicle's
 *     independent rotor tilt lets it direct force either up or down.
 *   - Z conditional-integration anti-windup (PositionControl.cpp:157-161),
 *     against the dynamic sphere bound instead of a static box.
 *   - Tracking anti-windup on X/Y, Rundqwist 1990 (PositionControl.cpp:
 *     188-199), simplified: mc_pos_control compares its acceleration
 *     setpoint against an acceleration "produced" by round-tripping the
 *     saturated thrust back through hover_thrust/g, because its loop
 *     works in acceleration+normalized-collective-thrust units. This
 *     loop's gains already act directly in newtons on both the desired
 *     and the saturated side (decision 1), so that round-trip is not
 *     needed — the "desired vs. produced" comparison is done directly in
 *     newtons. This is a unit-domain simplification of the port, not a
 *     change to the anti-windup mechanism itself.
 *
 *   NOT PORTED (deliberately, per the user's request):
 *   - _accelerationControl() / ControlMath::limitTilt() /
 *     ControlMath::thrustToAttitude() — the whole "collective thrust +
 *     body-tilt direction" attitude-setpoint synthesis. This class
 *     returns a force vector directly; nothing here produces an attitude
 *     setpoint. (AttitudeRateControl's phi_sp=theta_sp=0 pinning, in
 *     FoldrotorControl.cpp, is unrelated and predates this rework.)
 *   - updateHoverThrust()'s integrator-preserving hover-thrust-change
 *     absorption — this loop has no normalized "hover thrust" concept
 *     (FR_VEL_Z_GRAV_FF is a literal newtons weight, not normalized), so
 *     there is nothing analogous to re-derive.
 *
 * ---------------------------------------------------------------------
 * Decisions this file encodes, none of which were determined by a spec
 * on their own (see .claude/specs/findings.md and controller.md for the
 * dated record):
 *
 * 1. "FF" in controller_params.md's gain table is the *P gain on the
 *    velocity error*, in this loop's native force domain (newtons), not
 *    an acceleration feedforward on the setpoint and not a normalized
 *    thrust gain:
 *
 *        F = FF*e_v + I*integral(e_v) - D*vel_dot
 *
 *    The param keeps the name FR_VEL_*_FF because it is already
 *    published; only its meaning is pinned down. This is why the port
 *    from PositionControl.cpp's acceleration-domain _velocityControl()
 *    needs no mass/hover-thrust conversion anywhere in this class: the
 *    gains were already tuned to produce newtons directly (confirmed by
 *    the existing hand-computed unit tests, e.g. Fx = FR_VEL_XY_FF *
 *    vel_error), so introducing mass = FR_VEL_Z_GRAV_FF / CONSTANTS_ONE_G
 *    as a multiplier would double-scale the output, not match
 *    mc_pos_control's shape.
 *
 * 2. The derivative acts on the *measurement*, not the error, and is
 *    supplied as an input rather than differentiated here — matching
 *    mc_pos_control (PositionControl.cpp:150). Unchanged by this rework.
 *
 * 3. Anti-windup is now mc_pos_control's asymmetric shape (Z: conditional
 *    integration against the dynamic sphere bound; X/Y: Rundqwist
 *    tracking anti-windup) — a deliberate change from the previous
 *    uniform-conditional-integration-on-all-three-axes design, per the
 *    user's explicit "port the full mc_pos_control shape" decision.
 *
 * 4. The output is a force in newtons (allocation.md's contract), and
 *    FR_VEL_Z_GRAV_FF enters as a literal subtracted from the Z axis
 *    (`force(2) -= _grav_ff`). Body FRD/NED is Z DOWN-positive, so a
 *    force opposing gravity (points UP) needs a NEGATIVE Z contribution
 *    equal to the measured weight — unchanged by this rework, carried
 *    over from the 2026-09-11 sign fix.
 *
 * ---------------------------------------------------------------------
 * OPEN ITEMS — carried, not resolved. Do not "fix" these without a
 * decision; each one changes flight behaviour.
 *
 * (a) The combined force-magnitude limit and horizontal-force margin
 *     (setForceLimits()/setHorizontalForceMargin(), below) are first-cut
 *     placeholders, proposed the same way the previous independent boxes
 *     were (derived from FoldrotorAllocation::kMaxThrust x 2 rotors, with
 *     margin held back for moment authority) — NOT verified against a
 *     clean logged hover. See findings.md's existing open item on this;
 *     it now applies to these two numbers instead of the old boxes.
 *
 * (b) RESOLVED 2026-09-17: velocity-magnitude limiting is now ported
 *     (setVelocityLimits(), FR_VEL_XY_MAX/FR_VEL_Z_MAX_UP/FR_VEL_Z_MAX_DN)
 *     after real SITL testing (hover_setpoint.sh's 1.5 m ALT step) showed
 *     the unlimited vel_sp let a single ordinary position-setpoint step
 *     consume most of the combined force-magnitude sphere on its own,
 *     leaving the allocator no per-rotor thrust headroom to produce any
 *     attitude-loop moment and forcing fold/tilt to their rails within
 *     milliseconds of arming — see findings.md's 2026-09-17 entry. The
 *     limit VALUES themselves are still first-cut placeholders (same
 *     "not yet verified against a logged step response" status as (a)).
 *
 * (c) The Z loop's "extra summing junction not present on X/Y"
 *     (controller.md, original "Structure (confirmed from Simulink)")
 *     remains UNRESOLVED and is now moot for the *structure* question
 *     (which no longer follows the Simulink diagram at all) but may
 *     still matter if the Simulink model encoded a real physical
 *     correction — carried as-is, not resolved by this rework.
 *
 ****************************************************************************/

#pragma once

#include <matrix/matrix/math.hpp>

#include <cmath>
#include <float.h>

namespace foldrotor
{

class PositionVelocityControl
{
public:
	PositionVelocityControl() = default;

	/**
	 * Velocity-setpoint magnitude limits (m/s), mc_pos_control's
	 * setVelocityLimits()/constrainXY() equivalent — added 2026-09-17
	 * after real SITL testing showed a step in pos_sp (e.g. hover_
	 * setpoint.sh's 1.5 m ALT step) produces an unbounded vel_sp
	 * instantly, which by itself can approach the combined force-
	 * magnitude sphere (setForceLimits()) before any attitude-loop
	 * moment demand is even considered — starving the allocator of the
	 * per-rotor thrust headroom it needs to produce a moment and forcing
	 * fold/tilt to their rails to extract one anyway. See findings.md's
	 * 2026-09-17 entry and FR_VEL_XY_MAX/FR_VEL_Z_MAX_UP/FR_VEL_Z_MAX_DN
	 * in controller_params.md. Unlike mc_pos_control, there is no
	 * feedforward velocity-setpoint input to prioritize/blend against
	 * (this module's vel_sp comes only from position P) — so the
	 * horizontal clamp is a plain magnitude scale
	 * (ControlMath::constrainXY(v0, v1=0, max) collapses to exactly
	 * this), not the full priority-blend logic. Defaults to
	 * +/-infinity, i.e. a no-op, matching this class's other limit
	 * setters.
	 */
	void setVelocityLimits(float xy_max, float up_max, float down_max)
	{
		_lim_vel_xy = (xy_max > 0.f) ? xy_max : 0.f;
		_lim_vel_up = (up_max > 0.f) ? up_max : 0.f;
		_lim_vel_down = (down_max > 0.f) ? down_max : 0.f;
	}

	/**
	 * Position-loop P gain, FR_POS_P. Shared by x, y and z — a
	 * deliberate replication of the Simulink structure ("Kp=3, all
	 * axes"), not an oversight, per controller_params.md. Unchanged by
	 * the mc_pos_control-structure rework: mc_pos_control's position
	 * gain is per-axis (setPositionGains(Vector3f)); this module keeps
	 * the single shared scalar since no per-axis position gain param
	 * exists (or has been asked for).
	 */
	void setPositionGain(float p) { _pos_p = p; }

	/**
	 * Velocity-loop gains. Note the first argument of each triple is
	 * FR_VEL_*_FF, which is the P gain in this loop's native force
	 * domain (decision 1 above).
	 */
	void setVelocityGains(float xy_ff, float xy_i, float xy_d,
			      float z_ff, float z_i, float z_d)
	{
		_vel_p = matrix::Vector3f(xy_ff, xy_ff, z_ff);
		_vel_i = matrix::Vector3f(xy_i, xy_i, z_i);
		_vel_d = matrix::Vector3f(xy_d, xy_d, z_d);
	}

	/** FR_VEL_Z_GRAV_FF, subtracted from the Z axis. See decision 4 above. */
	void setGravityFeedforward(float grav_ff) { _grav_ff = grav_ff; }

	/**
	 * Combined force-magnitude ceiling (newtons), the mc_pos_control-
	 * style vertical-priority sphere saturation's radius
	 * (PositionControl.cpp:166, thrust_max_squared) — replaces the
	 * previous independent per-axis output-limit boxes. Unlike
	 * mc_pos_control's uni-directional collective thrust, this bound is
	 * SYMMETRIC (force may point either up or down on Z), matching this
	 * vehicle's independent rotor tilt authority. See OPEN ITEM (a): the
	 * value passed in by FoldrotorControl.cpp is a first-cut placeholder.
	 */
	void setForceLimits(float max) { _lim_force_max = (max > 0.f) ? max : 0.f; }

	/**
	 * Horizontal-force margin (newtons) held back when the sphere
	 * saturation is prioritizing vertical authority —
	 * PositionControl.cpp's _lim_thr_xy_margin, re-expressed directly in
	 * newtons rather than normalized thrust (this loop has no normalized
	 * thrust domain — see decision 1). See OPEN ITEM (a).
	 */
	void setHorizontalForceMargin(float margin) { _lim_force_xy_margin = (margin > 0.f) ? margin : 0.f; }

	/**
	 * Hard ceiling on horizontal force magnitude (newtons), applied AFTER
	 * the sphere saturation above and before the tracking anti-windup.
	 * Added 2026-09-21 (5); has no mc_pos_control equivalent, because
	 * mc_pos_control's vehicle has no such constraint.
	 *
	 * The sphere is the wrong shape for a vehicle whose force direction is
	 * railed at +-kMaxTilt: it permits ~22 N of horizontal force at hover
	 * thrust where the allocator can feasibly deliver ~6 N with no moments
	 * reserved and ~0 N with them. See FoldrotorControl.cpp's
	 * kPosVelForceXYLimit for the measured envelope and the reasoning.
	 *
	 * Defaults to +infinity, i.e. a no-op, matching this class's other
	 * limit setters -- so the existing sphere-saturation tests, which never
	 * call this, are unaffected.
	 */
	void setHorizontalForceLimit(float max) { _lim_force_xy_max = (max > 0.f) ? max : 0.f; }

	/**
	 * Symmetric bound on the accumulated integral itself, mirroring
	 * AttitudeRateControl::setIntegratorLimit() (mc_rate_control's
	 * _lim_int pattern) — a separate mechanism from the saturation/
	 * anti-windup above. Defaults to +/-infinity, i.e. a no-op, on all
	 * three axes. FR_VEL_Z_I_LIM = 3.0 N and FR_VEL_XY_I_LIM = 15.0 N are
	 * pushed in here by FoldrotorControl.cpp; unchanged by this rework.
	 */
	void setIntegratorLimit(const matrix::Vector3f &lim) { _lim_int = lim; }

	/** Zero the velocity integrator. 4e calls this on disarm / mode entry. */
	void resetIntegral() { _vel_int.setZero(); }

	const matrix::Vector3f &getIntegral() const { return _vel_int; }

	/**
	 * Run one cascade step.
	 *
	 * @param pos     current position, inertial/NED (m)
	 * @param pos_sp  position setpoint, inertial/NED (m)
	 * @param vel     current velocity, inertial/NED (m/s) — same frame
	 *                as pos, confirmed in controller.md's Interface
	 * @param vel_dot measured velocity derivative, inertial/NED (m/s^2).
	 *                This is the D input (decision 2), NOT differentiated
	 *                internally; 4e supplies vehicle_local_position's
	 *                ax/ay/az, as mc_pos_control does.
	 * @param dt      timestep (s), used only by the integrator
	 * @return desired force in the inertial/NED frame (N), to be handed
	 *         to foldrotor::inertialToBody() by step 4e
	 */
	matrix::Vector3f update(const matrix::Vector3f &pos, const matrix::Vector3f &pos_sp,
				const matrix::Vector3f &vel, const matrix::Vector3f &vel_dot,
				float dt)
	{
		// --- _positionControl()-equivalent (PositionControl.cpp:127-141) ---
		// Position P -> velocity setpoint. No velocity feedforward path
		// exists in this module's inputs, so none is invented here.
		matrix::Vector3f vel_sp = (pos_sp - pos) * _pos_p;

		// Velocity-magnitude limiting (added 2026-09-17, see
		// setVelocityLimits()'s comment above) -- horizontal is a plain
		// magnitude clamp (no feedforward vel_sp term exists to blend
		// against here), vertical is the asymmetric up/down clamp,
		// matching mc_pos_control's Z convention (NED: up is negative).
		const float vel_sp_xy_norm = matrix::Vector2f(vel_sp(0), vel_sp(1)).norm();

		if (vel_sp_xy_norm > _lim_vel_xy && vel_sp_xy_norm > FLT_EPSILON) {
			const float scale = _lim_vel_xy / vel_sp_xy_norm;
			vel_sp(0) *= scale;
			vel_sp(1) *= scale;
		}

		vel_sp(2) = math_constrain(vel_sp(2), -_lim_vel_up, _lim_vel_down);

		matrix::Vector3f vel_error = vel_sp - vel;

		// --- _velocityControl()-equivalent (PositionControl.cpp:143-150) ---
		// PID, already force-domain (decision 1): FF is the P gain; D is
		// on the measurement and negated (decision 2); the integral is
		// carried between calls.
		matrix::Vector3f force = vel_error.emult(_vel_p) + _vel_int - vel_dot.emult(_vel_d);

		// Gravity feedforward, Z only (decision 4). SUBTRACTED: body
		// FRD/NED is Z DOWN-positive, so a force opposing gravity needs a
		// NEGATIVE Z contribution equal to the measured weight.
		force(2) -= _grav_ff;

		// Pre-saturation force, kept for the tracking anti-windup
		// comparison below (mirrors mc_pos_control's _acc_sp).
		const matrix::Vector3f force_desired = force;

		// --- Vertical-priority sphere saturation (PositionControl.cpp:
		// 163-186), symmetric on Z (see setForceLimits()'s comment). ---
		const float force_max_squared = _lim_force_max * _lim_force_max;
		const float force_xy_norm = matrix::Vector2f(force(0), force(1)).norm();

		// How much horizontal force is already committed, keeping the
		// configured margin in reserve.
		const float allocated_horizontal = (force_xy_norm < _lim_force_xy_margin) ?
						   force_xy_norm : _lim_force_xy_margin;
		const float force_z_max_squared = force_max_squared
						  - allocated_horizontal * allocated_horizontal;
		const float force_z_max = std::sqrt((force_z_max_squared > 0.f) ? force_z_max_squared : 0.f);

		// Conditional integration on Z (decision 3 / mc_pos_control's
		// PositionControl.cpp:157-161), evaluated against the pre-clamp
		// force and the dynamic sphere bound above, before force(2) is
		// itself overwritten by the clamp below.
		const bool z_blocked_high = (force(2) >= force_z_max) && (vel_error(2) > 0.f);
		const bool z_blocked_low  = (force(2) <= -force_z_max) && (vel_error(2) < 0.f);

		if (z_blocked_high || z_blocked_low) {
			vel_error(2) = 0.f;
		}

		force(2) = math_constrain(force(2), -force_z_max, force_z_max);

		// Determine how much horizontal force is left after prioritizing Z.
		const float force_max_xy_squared = force_max_squared - force(2) * force(2);
		const float force_max_xy = std::sqrt((force_max_xy_squared > 0.f) ? force_max_xy_squared : 0.f);

		if (force_xy_norm > force_max_xy && force_xy_norm > FLT_EPSILON) {
			const float scale = force_max_xy / force_xy_norm;
			force(0) *= scale;
			force(1) *= scale;
		}

		// Hard horizontal ceiling (setHorizontalForceLimit(), 2026-09-21
		// (5)). Applied after the sphere because it is a separate physical
		// constraint -- the sphere bounds total force magnitude, this
		// bounds the DIRECTION the rotors can actually be tilted to while
		// still leaving alpha/beta range for the rate loop's moments.
		// Deliberately placed BEFORE the tracking anti-windup below so
		// that the clamp it applies is what the ARW sees as "produced",
		// which is what stops the horizontal integrator winding up against
		// a bound it can never reach.
		const float force_xy_capped = matrix::Vector2f(force(0), force(1)).norm();

		if (force_xy_capped > _lim_force_xy_max && force_xy_capped > FLT_EPSILON) {
			const float scale = _lim_force_xy_max / force_xy_capped;
			force(0) *= scale;
			force(1) *= scale;
		}

		// --- Tracking anti-windup, X/Y (Rundqwist 1990,
		// PositionControl.cpp:188-199), directly in newtons -- see this
		// file's header comment for why no hover-thrust round-trip is
		// needed here. Only engages while actually saturated. ---
		const matrix::Vector2f force_xy_desired(force_desired(0), force_desired(1));
		const matrix::Vector2f force_xy_produced(force(0), force(1));

		if (force_xy_desired.norm_squared() > force_xy_produced.norm_squared()) {
			const float arw_gain = 2.f / _vel_p(0);
			const matrix::Vector2f adjustment = arw_gain * (force_xy_desired - force_xy_produced);
			vel_error(0) -= adjustment(0);
			vel_error(1) -= adjustment(1);
		}

		// --- Integrate and clamp (separate mechanism, unchanged from
		// before this rework: FR_VEL_Z_I_LIM/FR_VEL_XY_I_LIM bound the
		// accumulated integral directly via setIntegratorLimit()). ---
		matrix::Vector3f vel_int = _vel_int + vel_error.emult(_vel_i) * dt;

		for (int i = 0; i < 3; i++) {
			if (std::isfinite(vel_int(i))) {
				_vel_int(i) = math_constrain(vel_int(i), -_lim_int(i), _lim_int(i));
			}
		}

		return force;
	}

private:
	// Local constrain so this header stays dependent on mathlib only,
	// matching Inertial2Body.hpp's minimal include surface.
	static float math_constrain(float v, float lo, float hi)
	{
		return (v < lo) ? lo : ((v > hi) ? hi : v);
	}

	float _pos_p{0.f};
	matrix::Vector3f _vel_p;
	matrix::Vector3f _vel_i;
	matrix::Vector3f _vel_d;
	float _grav_ff{0.f};

	// Sphere-saturation bounds (OPEN ITEM (a)): default to +/-infinity-
	// equivalent (a huge ceiling, zero margin) so an un-configured
	// instance (e.g. a test that never calls setForceLimits()) clamps
	// nothing, matching the previous class's "+/-infinity is a no-op"
	// default behavior.
	float _lim_force_max{1e9f};
	float _lim_force_xy_margin{0.f};
	float _lim_force_xy_max{INFINITY};

	matrix::Vector3f _lim_int{INFINITY, INFINITY, INFINITY};

	matrix::Vector3f _vel_int;

	// Velocity-setpoint magnitude limits (OPEN ITEM (b), resolved
	// 2026-09-17 -- see setVelocityLimits()). Default to +/-infinity, a
	// no-op, matching this class's other limit setters.
	float _lim_vel_xy{INFINITY};
	float _lim_vel_up{INFINITY};
	float _lim_vel_down{INFINITY};
};

} // namespace foldrotor
