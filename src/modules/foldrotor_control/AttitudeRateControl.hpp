/****************************************************************************
 *
 * foldrotor_control — attitude P -> rate PID -> body moments (step 4c).
 *
 * Implements the second half of .claude/specs/controller.md's "Structure
 * (confirmed from Simulink)" cascade:
 *
 *     attitude P (FR_ATT_P) -> rate PID (FR_RATE_RP_*, FR_RATE_YAW_*)
 *         -> M_b = (Mx_b, My_b, Mz_b), body/FRD
 *
 * The output is matrix::Vector3f in body/FRD, which is what
 * allocation.md's Interface expects as its moment input — the same
 * handoff pattern as step 4a's F_i -> step 4b's F_b. Unlike the force
 * path, the moment path needs NO frame rotation: p/q/r and the body
 * moments are body-frame by convention already (controller.md).
 *
 * Scope is just this math. No uORB, no params read here (4e pushes the
 * FR_* values in via the setters), no allocation (4d). Nothing called
 * this class until 4e wired it in.
 *
 * `euler_sp` is a plain function argument, exactly as step 4a treats
 * `pos_sp`. Where it comes from — in particular whether phi_sp/theta_sp
 * are always zero for this fully-actuated vehicle, or are driven by
 * something else — was a 4e wiring question; 4e pinned phi_sp/theta_sp
 * to zero (user decision, findings.md "euler_sp sourcing (step 4e)"),
 * consistent with this vehicle translating by thrust vectoring rather
 * than body tilt — controller.md's own explanation for why the missing
 * force-path rotation stayed hidden.
 *
 * **Interface split (step 4e part 1, user decision).** `update()` below
 * does attitude-P and rate-PID in one call, always recomputing `_rate_sp`
 * from the current Euler error. 4e's multi-rate cascade (findings.md
 * "Multi-rate cascade cadence") needs the rate PID to run faster than
 * the attitude P — the two are now `updateAttitude()` and `updateRate()`,
 * called independently at their own cadences. `update()` is kept,
 * unchanged in behaviour (it now just calls both back to back), so every
 * hand-computed value already pinned by this file's existing tests stays
 * valid as a regression test of the split.
 *
 * ---------------------------------------------------------------------
 * Decisions this file encodes. All were made by the user 2026-09-07 and
 * are recorded in .claude/specs/findings.md ("Attitude/rate-loop
 * semantics (step 4c)") before being folded into controller.md.
 *
 * 1. "FF" in controller_params.md's rate gain table is the *P gain on
 *    the rate error*, carrying step 4a's velocity-loop decision forward:
 *
 *        M = FF*e_r + I*integral(e_r) - D*rate_dot
 *
 *    This was decided as its own question, not inherited by assumption,
 *    and it is worth recording that it goes AGAINST the PX4 precedent
 *    rather than with it: RateControl::update()
 *    (src/lib/rate_control/rate_control.cpp:78) computes
 *
 *        _gain_p.emult(rate_error) + _rate_int
 *            - _gain_d.emult(angular_accel) + _gain_ff.emult(rate_sp)
 *
 *    i.e. PX4's rate controller has a distinct feedforward term applied
 *    to the *setpoint*, separate from P. The competing reading was
 *    rejected because it needs a P value that appears in no spec — and
 *    because with FR_RATE_YAW_I = FR_RATE_YAW_D = 0 it would leave yaw
 *    with no error feedback whatsoever. If the Simulink rate-loop
 *    diagram ever turns up and shows a separate P, this is the decision
 *    that has to be revisited.
 *
 * 2. The derivative acts on the *measurement* and is supplied as an
 *    input, not differentiated internally: `-D*rate_dot`. This one IS a
 *    verified precedent match, checked against the files rather than
 *    asserted: MulticopterRateControl.cpp:137 reads
 *    `const Vector3f angular_accel{angular_velocity.xyz_derivative};`
 *    and passes it to _rate_control.update() (:220), where
 *    rate_control.cpp:78 applies `- _gain_d.emult(angular_accel)` —
 *    uniformly on all three axes. Step 4e should feed this class
 *    vehicle_angular_velocity's xyz_derivative for the same reason.
 *
 * 3. The Euler attitude error is wrapped to [-pi, pi] on ALL THREE axes.
 *    Not in any spec — a deliberate addition to the Simulink structure,
 *    because an unwrapped yaw error crossing +/-pi produces a ~2pi error
 *    and a large command in the WRONG direction. See the note in
 *    update() about what wrapping means at the theta = +/-pi/2 pole.
 *
 * 4. The integral follows mc_rate_control's updateIntegral() in full —
 *    conditional integration, the nonlinear i_factor, the integrator
 *    clamp, and the landed gate. See updateIntegral() below, where each
 *    piece is attributed to its line in src/lib/rate_control/.
 *
 * ---------------------------------------------------------------------
 * OPEN ITEMS — carried, not resolved.
 *
 * (a) Attitude-to-rate mapping. This class applies a direct proportional
 *     gain to the Euler angle error to produce a body rate setpoint,
 *     exactly as controller.md specifies. That implicitly assumes Euler
 *     angle rate ~= body angular rate, which is exact only for small
 *     pitch; the exact relation needs the T(Theta) transformation
 *     (controller.md, Open questions 1). T(Theta) is deliberately NOT
 *     added here — adding it silently would change the control law
 *     away from the Simulink reference, and omitting it silently would
 *     hide a known approximation. It is implemented as specified and
 *     flagged as an approximation, and the error grows with pitch.
 *
 * (b) Yaw rate loop has FR_RATE_YAW_I = FR_RATE_YAW_D = 0, implemented
 *     as given. Whether that is deliberate (consistent with the small
 *     k=0.017 drag-coupling term) or unfinished tuning is
 *     controller.md's Open question 2 and remains open. Combined with
 *     decision 1 above, yaw is a pure proportional law: no integral, no
 *     derivative, no feedforward. No nonzero gain is invented here.
 *
 * (c) controller.md records that the Simulink diagrams show unconnected,
 *     red-highlighted ports on the phi/theta branches of the attitude
 *     loop and on the qsp branch of the rate loop. Whether those are
 *     dead nodes or carry logic this class is missing CANNOT be
 *     determined from the prose description — it needs the actual
 *     diagram images. They are not assumed to be dead ends. If any of
 *     them is live, this class is incomplete and the fix belongs here.
 *
 * (d) The integrator clamp and the output bounds both default to
 *     +/-infinity because no spec supplies values for them. See
 *     setIntegratorLimit() and setOutputLimits().
 *
 ****************************************************************************/

#pragma once

#include <matrix/matrix/math.hpp>

#include <cmath>
#include <float.h>

namespace foldrotor
{

class AttitudeRateControl
{
public:
	AttitudeRateControl() = default;

	/**
	 * Attitude-loop P gain, FR_ATT_P. Shared by phi, theta and psi — a
	 * deliberate replication of the Simulink structure ("Kp=3, all
	 * axes"), per controller_params.md.
	 */
	void setAttitudeGain(float p) { _att_p = p; }

	/**
	 * Rate-loop gains. The first argument of each triple is
	 * FR_RATE_*_FF, which is the P gain on the rate error (decision 1).
	 * Roll/pitch and yaw are separate because their gains differ:
	 * 3.5/0.1/0.5 against 2.5/0/0.
	 */
	void setRateGains(float rp_ff, float rp_i, float rp_d,
			  float yaw_ff, float yaw_i, float yaw_d)
	{
		_rate_p = matrix::Vector3f(rp_ff, rp_ff, yaw_ff);
		_rate_i = matrix::Vector3f(rp_i, rp_i, yaw_i);
		_rate_d = matrix::Vector3f(rp_d, rp_d, yaw_d);
	}

	/**
	 * Per-axis moment bounds, used both to clamp the returned moment
	 * and to derive the saturation flags that drive conditional
	 * integration.
	 *
	 * Defaults to +/-infinity, so both are no-ops. Same reasoning as
	 * step 4a's PositionVelocityControl::setOutputLimits(): the real
	 * bounds are a property of what the allocator can produce, which is
	 * step 4d. This is also exactly how PX4 does it —
	 * MulticopterRateControl.cpp:196-215 builds its saturation flags
	 * from control-allocation feedback and pushes them in via
	 * RateControl::setSaturationStatus() — so deferring is the real
	 * shape of the problem, not a convenience.
	 *
	 * Consequence, stated plainly: until 4d/4e pass real bounds, THE
	 * ANTI-WINDUP IS INERT AT RUNTIME.
	 */
	void setOutputLimits(const matrix::Vector3f &lower, const matrix::Vector3f &upper)
	{
		_lim_lower = lower;
		_lim_upper = upper;
	}

	/**
	 * Symmetric bound on the accumulated integral, mc_rate_control's
	 * _lim_int (rate_control.cpp:113, `math::constrain(rate_i,
	 * -_lim_int(i), _lim_int(i))`), which PX4 drives from its
	 * MC_*RATE_MAX-style params.
	 *
	 * Defaults to +/-infinity: no FR_RATE_*_I_LIM param exists, and
	 * controller_params.md is explicit that inventing bounds records an
	 * assumption as a constraint. The mechanism is implemented and
	 * tested; the value has to come from a decision. Until one is made,
	 * THIS CLAMP IS ALSO INERT.
	 */
	void setIntegratorLimit(const matrix::Vector3f &lim) { _lim_int = lim; }

	/** Zero the rate integrator. 4e calls this on disarm / mode entry. */
	void resetIntegral() { _rate_int.setZero(); }

	const matrix::Vector3f &getIntegral() const { return _rate_int; }

	/** The intermediate body-rate setpoint, for logging and for tests. */
	const matrix::Vector3f &getRateSetpoint() const { return _rate_sp; }

	/**
	 * Attitude P only: Euler error -> body-rate setpoint. Split from the
	 * rate stage (step 4e part 1) so the two can run at different rates —
	 * findings.md "Multi-rate cascade cadence (step 4e)", folded into
	 * controller.md. `_rate_sp` is held internally and consumed by
	 * updateRate() on every call until this is called again; call this
	 * at the slower (attitude-loop) cadence and updateRate() at the
	 * faster (rate-loop) cadence.
	 *
	 * OPEN ITEM (a) unchanged by the split: this is the direct
	 * proportional mapping controller.md specifies, which treats the
	 * Euler angle error as if it were a body-rate error. That holds only
	 * for small pitch; the exact relation needs T(Theta). Implemented as
	 * specified, flagged rather than silently corrected.
	 *
	 * The error is wrapped to [-pi, pi] on all three axes (decision 3).
	 * For psi this is what stops a setpoint crossing +/-pi from
	 * commanding a near-2pi rotation the wrong way round. For phi/theta
	 * inside the ZYX convention's own ranges the wrap is a no-op; at the
	 * theta = +/-pi/2 pole the extraction is already degenerate (see
	 * controller.md's singularity note) and wrapping neither helps nor
	 * hurts.
	 *
	 * @param euler     current attitude, ZYX phi/theta/psi — step 3's
	 *                  _euler = matrix::Eulerf(matrix::Quatf(q))
	 * @param euler_sp  attitude setpoint, same convention. Sourcing is
	 *                  a 4e question, deliberately not resolved here.
	 * @return the body-rate setpoint just computed (same as
	 *         getRateSetpoint() afterward)
	 */
	const matrix::Vector3f &updateAttitude(const matrix::Eulerf &euler, const matrix::Eulerf &euler_sp)
	{
		matrix::Vector3f att_error;

		for (int i = 0; i < 3; i++) {
			att_error(i) = matrix::wrap_pi(euler_sp(i) - euler(i));
		}

		_rate_sp = att_error * _att_p;

		return _rate_sp;
	}

	/**
	 * Rate PID only: body-rate error (against the `_rate_sp` most
	 * recently set by updateAttitude()) -> body moment. Split from the
	 * attitude stage per updateAttitude()'s doc comment; intended to run
	 * every cycle at the rate-loop's native cadence even when
	 * updateAttitude() itself has not been called this cycle — `_rate_sp`
	 * simply holds its last value, which is exactly the "hold the
	 * intermediate setpoint between updates" behaviour the multi-rate
	 * cascade needs and is asserted directly in
	 * RateSetpointHoldsAcrossUpdateRateCallsWithoutAttitudeUpdate.
	 *
	 * @param rate      current body angular rate p,q,r (FRD, rad/s)
	 * @param rate_dot  measured angular acceleration (FRD, rad/s^2).
	 *                  The D input (decision 2), NOT differentiated
	 *                  internally; 4e supplies vehicle_angular_velocity's
	 *                  xyz_derivative, as mc_rate_control does.
	 * @param dt        timestep (s), used only by the integrator —
	 *                  the actual elapsed time since this was last
	 *                  called, not a nominal period.
	 * @param landed    when true the integral is frozen entirely,
	 *                  matching rate_control.cpp:81-83. 4e supplies it.
	 * @return desired moment (Mx_b, My_b, Mz_b) in body/FRD, N*m —
	 *         allocation.md's moment input
	 */
	matrix::Vector3f updateRate(const matrix::Vector3f &rate, const matrix::Vector3f &rate_dot,
				    float dt, bool landed = false)
	{
		matrix::Vector3f rate_error = _rate_sp - rate;

		// Rate PID. FF is the P gain (decision 1); D is on the
		// measurement and negated (decision 2). Note there is
		// deliberately no _gain_ff.emult(rate_sp) term here, which is
		// where this departs from rate_control.cpp:78.
		const matrix::Vector3f moment =
			rate_error.emult(_rate_p) + _rate_int - rate_dot.emult(_rate_d);

		matrix::Vector3f moment_limited;

		for (int i = 0; i < 3; i++) {
			moment_limited(i) = constrain(moment(i), _lim_lower(i), _lim_upper(i));
		}

		// rate_control.cpp:81-83 — "update integral only if we are not
		// landed".
		if (!landed) {
			updateIntegral(rate_error, moment, dt);
		}

		return moment_limited;
	}

	/**
	 * Run one attitude+rate cascade step in a single call — updateAttitude()
	 * followed immediately by updateRate(). Kept, unchanged in behaviour,
	 * as the single-rate entry point: every existing hand-computed test in
	 * FoldrotorControlTest.cpp calls this and continues to pin the exact
	 * same values it always has. 4e's multi-rate wiring calls
	 * updateAttitude()/updateRate() separately instead of this.
	 *
	 * @see updateAttitude(), updateRate() for the parameter docs.
	 */
	matrix::Vector3f update(const matrix::Eulerf &euler, const matrix::Eulerf &euler_sp,
				const matrix::Vector3f &rate, const matrix::Vector3f &rate_dot,
				float dt, bool landed = false)
	{
		updateAttitude(euler, euler_sp);
		return updateRate(rate, rate_dot, dt, landed);
	}

private:
	/**
	 * Mirrors RateControl::updateIntegral() (src/lib/rate_control/
	 * rate_control.cpp:88-116) in full, per decision 4. Each piece is
	 * PX4's, not invented here:
	 *
	 *  - conditional integration (:91-98): clip the error rather than
	 *    zero it, so an error pulling the axis OUT of saturation still
	 *    integrates. Unlike step 4a's velocity loop — where this pattern
	 *    matches PX4 only on Z — this is a genuine all-three-axes match
	 *    to what mc_rate_control does.
	 *  - the nonlinear i_factor (:100-107): fade the I gain as the rate
	 *    error grows, to stop the integral building up through a large
	 *    setpoint change. The 400-degree scale is PX4's hard-coded
	 *    constant, imported as-is; no spec provides one.
	 *  - the finiteness guard and integrator clamp (:112-114).
	 */
	void updateIntegral(matrix::Vector3f &rate_error, const matrix::Vector3f &moment, float dt)
	{
		for (int i = 0; i < 3; i++) {
			// Saturation flags derived from this step's unclamped
			// moment. PX4 instead carries them over from the previous
			// cycle's allocator feedback; using the current step keeps
			// this class self-contained and matches step 4a.
			const bool saturated_positive = (moment(i) >= _lim_upper(i));
			const bool saturated_negative = (moment(i) <= _lim_lower(i));

			// Prevent further positive/negative control saturation.
			if (saturated_positive) {
				rate_error(i) = fminf(rate_error(i), 0.f);
			}

			if (saturated_negative) {
				rate_error(i) = fmaxf(rate_error(i), 0.f);
			}

			// I term factor: reduce the I gain with increasing rate
			// error. 400 degrees, expressed in radians.
			float i_factor = rate_error(i) / kIFactorRateErrorScale;
			i_factor = fmaxf(0.f, 1.f - i_factor * i_factor);

			const float rate_i = _rate_int(i) + i_factor * _rate_i(i) * rate_error(i) * dt;

			// Do not propagate the result if out of range or invalid.
			if (std::isfinite(rate_i)) {
				_rate_int(i) = constrain(rate_i, -_lim_int(i), _lim_int(i));
			}
		}
	}

	// Local constrain so this header stays dependent on mathlib only,
	// matching Inertial2Body.hpp's and PositionVelocityControl.hpp's
	// minimal include surface.
	static float constrain(float v, float lo, float hi)
	{
		return (v < lo) ? lo : ((v > hi) ? hi : v);
	}

	// math::radians(400.f), PX4's hard-coded i_factor scale.
	static constexpr float kIFactorRateErrorScale = 6.98131700797732f;

	float _att_p{0.f};
	matrix::Vector3f _rate_p;
	matrix::Vector3f _rate_i;
	matrix::Vector3f _rate_d;

	matrix::Vector3f _lim_lower{-INFINITY, -INFINITY, -INFINITY};
	matrix::Vector3f _lim_upper{INFINITY, INFINITY, INFINITY};
	matrix::Vector3f _lim_int{INFINITY, INFINITY, INFINITY};

	matrix::Vector3f _rate_sp;
	matrix::Vector3f _rate_int;
};

} // namespace foldrotor
