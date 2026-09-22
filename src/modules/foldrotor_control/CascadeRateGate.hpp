/****************************************************************************
 *
 * foldrotor_control — multi-rate cascade gating (step 4e part 1).
 *
 * controller.md's "Structure (confirmed from Simulink)" section gives the
 * cascade order (position P -> velocity PID -> attitude P -> rate PID ->
 * allocation) but no timing. The 50 Hz / 250 Hz / 1000 Hz split used to
 * wire Run() is a user decision (2026-09-07), not a spec transcription --
 * recorded in findings.md ("Multi-rate cascade cadence") before being
 * folded into controller.md. This header is just the mechanism.
 *
 * Gating is accumulated wall-time since the stage last fired, measured
 * against the *triggering topic's own* timestamp_sample -- never a fixed
 * sample-count divider, per reference/px4-module-patterns.md item 2 and
 * the same reasoning FoldrotorControl.cpp already applies to dt. A stalled
 * or irregular driving rate (e.g. a dropped vehicle_angular_velocity
 * publication) fires the gate once as soon as the accumulated time clears
 * the period, then resumes its normal cadence -- it never "catches up" by
 * firing more than once for a single due() call.
 *
 * dt CONTRACT (revised 2026-09-11): *dt_out is never zero and never
 * unbounded.
 *   - First fire reports the nominal period rather than 0. A zero dt
 *     reaches the D terms of the velocity and rate PIDs as a division by
 *     zero -- the resulting inf/NaN lands in an integrator and persists
 *     well past the cycle that produced it. One cycle of nominal-period
 *     integration on a stage that has just been primed (error still ~0)
 *     is the cheaper failure. Same reasoning as mc_rate_control, which
 *     constrains its dt into [0.2 ms, 20 ms] rather than admitting 0.
 *   - Every fire is clamped to _max_dt_us (default 10x period), so a
 *     long stall, a paused SITL clock, or a backwards timestamp cannot
 *     hand a controller a dt large enough to step its integrator across
 *     the whole gap in one cycle.
 * Callers that previously relied on the old "first call yields 0" promise
 * must be updated; see FoldrotorControlTest.cpp.
 *
 ****************************************************************************/
#pragma once

#include <cstdint>

namespace foldrotor
{

class CascadeRateGate
{
public:
	/**
	 * @param period_us minimum elapsed time between fires, microseconds.
	 *                  Must be > 0 -- a zero period leaves the gate with
	 *                  no meaningful dt bound and is not a supported way
	 *                  to express "every cycle" (just call the stage
	 *                  unconditionally instead, as the rate loop does).
	 * @param max_dt_us upper bound on the dt reported to the caller,
	 *                  microseconds. Defaults to 10x period_us.
	 */
	explicit CascadeRateGate(uint64_t period_us, uint64_t max_dt_us = 0)
		: _period_us(period_us),
		  _max_dt_us(max_dt_us != 0 ? max_dt_us : period_us * 10u)
	{}

	/**
	 * @param now    current time, microseconds, from the driving topic's
	 *               own timestamp_sample -- not a locally-read clock
	 * @param dt_out if non-null and this call fires, receives the elapsed
	 *               time since the previous fire, seconds, clamped into
	 *               (0, _max_dt_us]. This is the real accumulated dt
	 *               (>= period) in normal operation; it saturates after a
	 *               stall rather than reporting the full gap. Untouched if
	 *               this call does not fire.
	 * @return true if this stage is due to run this cycle. The first call
	 *         always fires (there is no meaningful "last run" yet), with
	 *         *dt_out set to the nominal period -- see the dt CONTRACT
	 *         note above for why this is not 0.
	 */
	bool due(uint64_t now, float *dt_out = nullptr)
	{
		if (!_primed) {
			_primed = true;
			_last = now;

			if (dt_out) {
				*dt_out = static_cast<float>(_period_us) * 1e-6f;
			}

			return true;
		}

		// Unsigned subtraction: if `now` ever moves backwards (SITL clock
		// reset, a stale timestamp_sample) this wraps to a very large
		// value, which the _max_dt_us clamp below bounds and the
		// `_last = now` re-sync below recovers from on the next cycle.
		const uint64_t elapsed_us = now - _last;

		if (elapsed_us >= _period_us) {
			_last = now;

			if (dt_out) {
				const uint64_t dt_us = (elapsed_us > _max_dt_us) ? _max_dt_us : elapsed_us;
				*dt_out = static_cast<float>(dt_us) * 1e-6f;
			}

			return true;
		}

		return false;
	}

	/**
	 * Drop the "last fired" reference. The next due() call fires
	 * immediately and reports the nominal period, exactly as the first
	 * call after construction does. Use on disarm / on any event that
	 * already resets the downstream stage's state, so a gate that has
	 * been idle does not hand a freshly-reset integrator a saturated dt.
	 */
	void reset() { _primed = false; }

private:
	uint64_t _period_us;
	uint64_t _max_dt_us;
	uint64_t _last{0};
	bool _primed{false};
};

} // namespace foldrotor
