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
 ****************************************************************************/

#pragma once

#include <cstdint>

namespace foldrotor
{

class CascadeRateGate
{
public:
	/** @param period_us minimum elapsed time between fires, microseconds */
	explicit CascadeRateGate(uint64_t period_us) : _period_us(period_us) {}

	/**
	 * @param now    current time, microseconds, from the driving topic's
	 *               own timestamp_sample -- not a locally-read clock
	 * @param dt_out if non-null and this call fires, receives the actual
	 *               elapsed time since the previous fire, seconds -- the
	 *               real accumulated dt (>= period, larger after a
	 *               stall), not the nominal period. Untouched if this
	 *               call does not fire.
	 * @return true if this stage is due to run this cycle. The first call
	 *         always fires (there is no meaningful "last run" yet), with
	 *         *dt_out set to 0 since there is nothing to integrate over.
	 */
	bool due(uint64_t now, float *dt_out = nullptr)
	{
		if (!_primed) {
			_primed = true;
			_last = now;

			if (dt_out) {
				*dt_out = 0.f;
			}

			return true;
		}

		if ((now - _last) >= _period_us) {
			if (dt_out) {
				*dt_out = static_cast<float>(now - _last) * 1e-6f;
			}

			_last = now;
			return true;
		}

		return false;
	}

private:
	uint64_t _period_us;
	uint64_t _last{0};
	bool _primed{false};
};

} // namespace foldrotor
