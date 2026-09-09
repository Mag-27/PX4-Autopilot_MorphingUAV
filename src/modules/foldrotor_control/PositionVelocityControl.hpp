/****************************************************************************
 *
 * foldrotor_control — position P -> velocity PID -> inertial-frame desired
 * force (step 4a).
 *
 * Implements the first two stages of .claude/specs/controller.md's
 * "Structure (confirmed from Simulink)" cascade:
 *
 *     position P (FR_POS_P) -> velocity PID (FR_VEL_XY_*, FR_VEL_Z_*)
 *         -> F_i = (Fx_i, Fy_i, Fz_i), inertial/NED
 *
 * The output is deliberately matrix::Vector3f so it feeds step 4b directly:
 *
 *     const matrix::Vector3f F_b = foldrotor::inertialToBody(F_i, euler);
 *
 * Scope is just this math. No uORB, no params read here (4e pushes the
 * FR_* values in via the setters, per controller_params.md's
 * "raw _param_fr_* values should not be read directly from hot
 * control-law code"), no attitude/rate loop (4c), no allocation (4d), no
 * wiring into Run() (4e). Nothing calls this class yet.
 *
 * ---------------------------------------------------------------------
 * Decisions this file encodes, none of which were determined by a spec.
 * All four were made by the user 2026-09-07 and are recorded in
 * .claude/specs/findings.md ("Velocity-loop semantics resolved by user
 * decision (step 4a)") before being folded into controller.md.
 *
 * 1. "FF" in controller_params.md's gain table is the *P gain on the
 *    velocity error*, not a feedforward on the setpoint:
 *
 *        F = FF*e_v + I*integral(e_v) - D*vel_dot
 *
 *    The param keeps the name FR_VEL_*_FF because it is already
 *    published; only its meaning is now pinned down.
 *
 * 2. The derivative acts on the *measurement*, not the error, and is
 *    supplied as an input rather than differentiated here — matching
 *    mc_pos_control (PositionControl.cpp:150, `- _vel_dot.emult(
 *    _gain_vel_d)`, fed from states.acceleration, already filtered by
 *    the estimator). No internal D filter, also matching PX4. This may
 *    diverge from the Simulink reference if that differentiated the
 *    error: v_sp = FR_POS_P * e_p steps whenever the position setpoint
 *    moves, and derivative-on-error would kick there while this does
 *    not. Deliberate and recorded, not accidental.
 *
 * 3. Anti-windup is conditional integration on ALL THREE AXES: the
 *    error is zeroed before integration when that axis' output is
 *    saturated in the direction that would make the saturation worse.
 *
 *    Precedent, stated accurately (an earlier version of this comment
 *    overclaimed it): this is mc_pos_control's *vertical* algorithm,
 *    generalized to x and y. PositionControl.cpp:158-160 — commented
 *    "Integrator anti-windup in vertical direction", gating on
 *    _thr_sp(2) and vel_error(2) — is conditional integration, and only
 *    for Z. X/Y there use something else entirely: tracking anti-windup
 *    (Rundqwist 1990, PositionControl.cpp:188-198), which feeds the
 *    difference between desired and achievable acceleration back into
 *    the error with a gain of 2/P rather than freezing on saturation
 *    direction. mc_pos_control also clamps its Z integral outright
 *    (:146, to +/-g), which this class does not.
 *
 *    So: uniform conditional integration is a deliberate choice for
 *    this module, not a verified match to PX4's horizontal precedent.
 *    It is defensible — it is simple, symmetric across axes, and does
 *    not require the achievable-output estimate that tracking ARW needs
 *    — but switching to real tracking anti-windup on X/Y would be a
 *    design decision, and is not one that has been made. See
 *    setOutputLimits() for why the conditional-integration mechanism
 *    itself is still inert on all three axes until step 4d supplies real
 *    output bounds. **The separate integrator-clamp mechanism (below) is
 *    no longer inert on Z**: `FR_VEL_Z_I_LIM = 3.0` N, set 2026-09-09
 *    (findings.md, same date), bounds the accumulated `_vel_int(2)`
 *    itself, independent of setOutputLimits()/conditional integration.
 *    X/Y have no such param yet and stay ±infinity — no decision has
 *    been made there.
 *
 * 4. The output is a force in newtons (allocation.md's contract), and
 *    FR_VEL_Z_GRAV_FF enters as a literal added directly to the Z axis
 *    (`force(2) += _grav_ff`, no other scaling or sign step). **Resolved
 *    2026-09-09** (controller.md Open questions 3/4, findings.md
 *    2026-09-09 "(4)"/"(5)"): the vehicle's own translational dynamics,
 *    p_ddot = R_b^i * F_b/m - [0,0,g]^T, requires the force-domain
 *    feedforward on Fz to equal the vehicle's weight mg for p_ddot = 0 at
 *    a level hover — not the raw 9.81 m/s^2 acceleration literal that was
 *    there before. Value is now 15.260017 N, the 2026-09-09 Part D
 *    bench-measured static weight (a direct measurement, not a computed
 *    mass estimate). See former OPEN ITEMS (a)/(b) below, now resolved.
 *
 * ---------------------------------------------------------------------
 * OPEN ITEMS — carried, not resolved. Do not "fix" these without a
 * decision; each one changes flight behaviour.
 *
 * (a) [RESOLVED 2026-09-09] Gravity feedforward units. Was 9.81, an
 *     *acceleration*, in a force-domain loop. This airframe measures
 *     15.260017 N (findings.md, Part D bench run), so the feedforward is
 *     now that measured weight directly. Before this fix, hover leaned on
 *     the FR_VEL_Z_I = 7 integrator to make up the remaining ~5.4 N —
 *     confirmed by the 2026-09-09 Part D bench run, where F_b.z climbed
 *     from the literal 9.81 N to Z-thrust saturation from windup alone.
 *
 * (b) [RESOLVED 2026-09-09, empirically] Gravity feedforward sign. The
 *     literal is added directly and positive, and Part D's bench runs
 *     (both before and after this fix) show the *integrator* winding up
 *     in the same direction as the feedforward to reach hover weight, not
 *     opposing it — the sign was already correct for this module's
 *     runtime convention. This does not, on its own, resolve whether that
 *     convention is "really" NED with a Z-up-authored Simulink reference,
 *     or something else; it only confirms the sign this code needs is the
 *     one it already had. Do not read this as license to guess signs
 *     elsewhere — it stands on the Part D measurement, not on reasoning
 *     about the model's authored frame.
 *
 * (c) The Z loop's "extra summing junction not present on X/Y"
 *     (controller.md, "Structure (confirmed from Simulink)") is
 *     UNRESOLVED. It is not guessable from the prose description and
 *     needs the Simulink velocity-loop diagram. Z is implemented here
 *     as FF/I/D plus FR_VEL_Z_GRAV_FF as given — the junction is
 *     neither implemented nor invented. If it turns out to carry logic,
 *     this class is wrong on Z and the fix belongs here.
 *
 * (d) [RESOLVED 2026-09-09, Z only] Integrator windup bound. Until now
 *     the anti-windup mechanism (decision 3) was inert on every axis
 *     because setOutputLimits() defaulted to +/-infinity, and there was
 *     no separate clamp on the accumulated integral at all (unlike
 *     AttitudeRateControl's setIntegratorLimit()). This class now has the
 *     same setIntegratorLimit() mechanism, and FR_VEL_Z_I_LIM = 3.0 N
 *     bounds `_vel_int(2)` directly. This is a decision about how much
 *     the integrator alone may contribute to Fz, deliberately smaller
 *     than the ~15.26 N hover weight now supplied entirely by
 *     FR_VEL_Z_GRAV_FF (OPEN ITEM (a)) — the integrator only needs to
 *     cover real trim/disturbance now, not a structural feedforward gap.
 *     X and Y have no such param and remain unbounded; that is an
 *     unmade decision, not an oversight.
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
	 * Position-loop P gain, FR_POS_P. Shared by x, y and z — a
	 * deliberate replication of the Simulink structure ("Kp=3, all
	 * axes"), not an oversight, per controller_params.md.
	 */
	void setPositionGain(float p) { _pos_p = p; }

	/**
	 * Velocity-loop gains. Note the first argument of each triple is
	 * FR_VEL_*_FF, which is the P gain (decision 1 above).
	 */
	void setVelocityGains(float xy_ff, float xy_i, float xy_d,
			      float z_ff, float z_i, float z_d)
	{
		_vel_p = matrix::Vector3f(xy_ff, xy_ff, z_ff);
		_vel_i = matrix::Vector3f(xy_i, xy_i, z_i);
		_vel_d = matrix::Vector3f(xy_d, xy_d, z_d);
	}

	/** FR_VEL_Z_GRAV_FF, added to the Z axis. See OPEN ITEMS (a)/(b), resolved 2026-09-09. */
	void setGravityFeedforward(float grav_ff) { _grav_ff = grav_ff; }

	/**
	 * Per-axis output bounds used both to clamp the returned force and
	 * to drive conditional integration (decision 3).
	 *
	 * These default to +/-infinity, which makes both the clamp and the
	 * anti-windup no-ops. That is intentional: the real bounds are a
	 * property of what the allocator can actually produce, which is
	 * step 4d and does not exist yet. Inventing a limit now would be
	 * recording an assumption as a constraint (the same reasoning
	 * controller_params.md gives for leaving the params' min/max TBD).
	 *
	 * Consequence, stated plainly: until 4d/4e call this with real
	 * bounds, THE ANTI-WINDUP IS INERT AT RUNTIME and the FR_VEL_Z_I=7
	 * integrator is unbounded — the same shape of caveat as step 4b's
	 * rotation existing but not being wired in. The tests exercise the
	 * mechanism by supplying explicit bounds.
	 */
	void setOutputLimits(const matrix::Vector3f &lower, const matrix::Vector3f &upper)
	{
		_lim_lower = lower;
		_lim_upper = upper;
	}

	/**
	 * Symmetric bound on the accumulated integral itself, mirroring
	 * AttitudeRateControl::setIntegratorLimit() (mc_rate_control's
	 * _lim_int pattern) — a separate mechanism from setOutputLimits()/the
	 * conditional-integration anti-windup above. Defaults to
	 * +/-infinity, i.e. a no-op, on all three axes.
	 *
	 * FR_VEL_Z_I_LIM = 3.0 N (2026-09-09, findings.md) is now pushed into
	 * this on Z only, resolving OPEN ITEM (d) below on that axis. X/Y
	 * have no corresponding param and stay unbounded.
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
		// Position P -> velocity setpoint. controller.md: position P
		// (Kp=3, all axes) feeding the velocity loop; there is no
		// velocity feedforward path in the Simulink structure, so
		// none is invented here.
		const matrix::Vector3f vel_sp = (pos_sp - pos) * _pos_p;

		matrix::Vector3f vel_error = vel_sp - vel;

		// Velocity PID. FF is the P gain (decision 1); D is on the
		// measurement and negated (decision 2); the integral is the
		// state carried between calls.
		matrix::Vector3f force = vel_error.emult(_vel_p) + _vel_int - vel_dot.emult(_vel_d);

		// Gravity feedforward, Z only, literal value = measured weight in
		// newtons (mg), not a raw g literal. OPEN ITEMS (a), (b), resolved 2026-09-09.
		force(2) += _grav_ff;

		matrix::Vector3f force_limited;

		for (int i = 0; i < 3; i++) {
			force_limited(i) = math_constrain(force(i), _lim_lower(i), _lim_upper(i));

			// Conditional integration: freeze the axis only when the
			// output is already saturated AND the error would drive
			// it further into saturation. The error sign maps
			// directly onto the output sign here because _vel_p is
			// positive, so this stays frame-agnostic.
			const bool blocked_high = (force(i) >= _lim_upper(i)) && (vel_error(i) > 0.f);
			const bool blocked_low  = (force(i) <= _lim_lower(i)) && (vel_error(i) < 0.f);

			if (blocked_high || blocked_low) {
				vel_error(i) = 0.f;
			}
		}

		matrix::Vector3f vel_int = _vel_int + vel_error.emult(_vel_i) * dt;

		for (int i = 0; i < 3; i++) {
			// Integrator clamp (OPEN ITEM (d)), mirroring
			// AttitudeRateControl::updateIntegral()'s finiteness guard:
			// don't propagate a non-finite accumulation.
			if (std::isfinite(vel_int(i))) {
				_vel_int(i) = math_constrain(vel_int(i), -_lim_int(i), _lim_int(i));
			}
		}

		return force_limited;
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

	matrix::Vector3f _lim_lower{-INFINITY, -INFINITY, -INFINITY};
	matrix::Vector3f _lim_upper{INFINITY, INFINITY, INFINITY};
	matrix::Vector3f _lim_int{INFINITY, INFINITY, INFINITY};

	matrix::Vector3f _vel_int;
};

} // namespace foldrotor
