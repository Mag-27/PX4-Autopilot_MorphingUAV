# Controller Parameters Specification (step 3)

## Status
Not yet implemented. `foldrotor_control::parameters_updated()`
(`FoldrotorControl.cpp:40-44`) is currently a stub:

```cpp
void
FoldrotorControl::parameters_updated()
{
	// No params yet — step 3 adds the position/velocity/attitude/rate gains
	// from .claude/specs/controller.md here.
}
```

This spec is the contract to implement that against. It is step 3 of the
module's build-out (step 2 = skeleton, done; step 4 = cascade control-law
math; step 5 = actuator_motors/actuator_servos publish). The original
implementation plan those step numbers came from is not recoverable —
confirmed absent from both this repo's `.claude/` tree and the user's
global Claude Code plans directory — so this document is now the durable
record of what step 3 means, replacing the lost plan text.

## Scope
PX4 `params.yaml` parameter definitions, plus the `DEFINE_PARAMETERS`/
`ParamFloat` wiring and `parameters_updated()` implementation pattern, for
the position/velocity/attitude/rate cascade gains in `foldrotor_control`.

**Out of scope:**
- The control-law math itself that consumes these gains (step 4).
- Allocation constants `d`, `l_arm`, `h`, `k`, `max_tilt` — these are
  `allocation.md`'s concern, also landing in step 4, not params in the
  controller sense.
- `actuator_motors`/`actuator_servos` publishing (step 5).

## Source of truth
`controller.md`'s "Structure (confirmed from Simulink)" section. Gain
values are reproduced here only as PX4 param metadata — not re-derived,
not re-tuned.

## Naming convention — flagged recommendation, not decided
Proposed prefix: **`FR_`** (e.g. `FR_POS_P`), not `MC_`.

Rationale: `foldrotor_control` *replaces* the stock `MC_*` cascade
(`mc_pos_control`/`mc_att_control`/`mc_rate_control`) for this vehicle
rather than extending it — most existing `MC_*` params (e.g.
`MC_ROLLRATE_P`) will not apply and are not read by this module. Reusing
the `MC_` prefix risks implying a compatibility/overlap that doesn't
exist, and would sit ambiguously in QGroundControl's param list next to
the stock params it's replacing. `mc_raptor` uses `MC_RAPTOR_*`, but that
module is still conceptually an MC flight mode layered on top of the
stock stack; `foldrotor_control` is not — it's a full-stack substitute.

This is a naming call, not a control-engineering one — flagged for the
user to confirm or override before `params.yaml` is written in step 4/5;
not settled silently here.

## Parameter table
Group: `Foldrotor Control`. All gains below are single shared values per
loop (matching Simulink's "Kp=3, all axes" structure) rather than PX4's
usual XY/Z split convention (`mc_pos_control`-style) — this is a
deliberate replication of the Simulink structure, not an oversight, and
should not be "fixed" to match PX4 convention without a decision to do
so.

**What `FR_VEL_*_FF` means — decided 2026-09-07 (step 4a).** The table
records FF/I/D per velocity loop with no separate P term. That is now
read as: **FF *is* the P gain, applied to the velocity error**, giving
`F = FF*e_v + I*integral(e_v) - D*vel_dot`. The competing reading (FF as
a true feedforward on `v_sp`, plus an unrecorded P) was rejected — it
requires a P value that appears in no spec. The param names are left
alone because they are already published and renaming them would churn
saved airframe configs; only the documented meaning changes. See
`controller.md`'s "Velocity-loop form" section for the full set of
decisions this belongs to (derivative-on-measurement, conditional
integration on all three axes, gravity-feedforward units and sign).

The same reading was extended to the **rate** loop's
`FR_RATE_RP_FF`/`FR_RATE_YAW_FF` on 2026-09-07 (step 4c) — as a separate
decision, not an inherited assumption. Worth recording that the rate loop
had a real precedent pointing the other way: PX4's `RateControl::update()`
(`src/lib/rate_control/rate_control.cpp:78`) carries a distinct
`_gain_ff.emult(rate_sp)` term applied to the setpoint, separate from its
P gain. The FF-as-P reading was chosen anyway, because the alternative
needs a P value no spec records and would leave yaw (I = D = 0) with no
error feedback at all.

**Missing param — new gap identified by step 4c.** `AttitudeRateControl`
implements `mc_rate_control`'s integrator clamp
(`rate_control.cpp:112-114`), but there is no `FR_RATE_*_I_LIM` row in
the table below to drive it, so it defaults to ±infinity and is inert.
PX4 drives its equivalent from params. Adding one is a decision, not a
transcription — the Simulink model supplies no value — so it is recorded
here as a gap rather than filled in.

`min`/`max`/`increment`/`decimal` are left **TBD**: `controller.md` gives
only the Simulink-confirmed default values, no tuning range. Filling
these in is a step-4/5 concern once the control law exists to tune
against — inventing bounds now would be recording an assumption as a
constraint.

| Param | Loop | Meaning | Default | Notes |
|---|---|---|---|---|
| `FR_POS_P` | Position | P gain, all axes (x,y,z) | 3.0 | Shared across axes, per Simulink |
| `FR_VEL_XY_FF` | Velocity (X/Y) | **P gain on the velocity error** (see note) | 6.0 | Name says FF; meaning is P — decided 2026-09-07 |
| `FR_VEL_XY_I` | Velocity (X/Y) | Integral gain | 1.0 | |
| `FR_VEL_XY_D` | Velocity (X/Y) | Derivative gain | 1.0 | |
| `FR_VEL_Z_FF` | Velocity (Z) | **P gain on the velocity error** (see note) | 7.0 | Name says FF; meaning is P — decided 2026-09-07 |
| `FR_VEL_Z_I` | Velocity (Z) | Integral gain | 7.0 | |
| `FR_VEL_Z_D` | Velocity (Z) | Derivative gain | 0.1 | |
| `FR_VEL_Z_GRAV_FF` | Velocity (Z) | Gravity feedforward | 9.81 | See open item below — Z loop also has an extra summing junction not present on X/Y; not represented as a separate param here, since it's a control-law structure question (step 4), not a gain value |
| `FR_ATT_P` | Attitude | P gain, all axes (phi,theta,psi) | 3.0 | Shared across axes, per Simulink |
| `FR_RATE_RP_FF` | Rate (roll/pitch, Mx_b/My_b) | **P gain on the rate error** (see note) | 3.5 | Name says FF; meaning is P — decided 2026-09-07 |
| `FR_RATE_RP_I` | Rate (roll/pitch) | Integral gain | 0.1 | |
| `FR_RATE_RP_D` | Rate (roll/pitch) | Derivative gain | 0.5 | |
| `FR_RATE_YAW_FF` | Rate (yaw, Mz_b) | **P gain on the rate error** (see note) | 2.5 | Name says FF; meaning is P — decided 2026-09-07 |
| `FR_RATE_YAW_I` | Rate (yaw) | Integral gain | 0.0 | See open item below — confirm deliberate |
| `FR_RATE_YAW_D` | Rate (yaw) | Derivative gain | 0.0 | See open item below — confirm deliberate |

## Implementation pattern
Per `reference/px4-module-patterns.md` item 4 (confirmed convention across
`mc_pos_control`/`mc_att_control`/`mc_rate_control`):
- One `DEFINE_PARAMETERS(...)` entry per row above, `(ParamFloat<px4::params::FR_...>) _param_fr_...` in `FoldrotorControl.hpp`.
- `params.yaml` metadata (`type: float`, `default`, `description.short/long`) matching the format used in `mc_rate_control/mc_rate_control_params.yaml` and `mc_raptor/module.yaml`, listed under this module's `MODULE_CONFIG` in `CMakeLists.txt` (currently absent — the skeleton `CMakeLists.txt` has no `MODULE_CONFIG` entry yet; adding the yaml file and this entry together is part of step 3).
- `parameters_updated()` fills in the stub above: `updateParams()` refreshes all `Param*` values from storage; `parameters_updated()` then pushes them into whatever local gain-holding struct/objects step 4 introduces. Raw `_param_fr_*` values should not be read directly from hot control-law code, only from inside this function — matching the pattern all three stock controllers use.

## Open items carried forward, not resolved
These are restated from `controller.md`, not decided here:

1. **Z-velocity-loop gravity feedforward + extra summing junction.**
   `controller.md`: "plus an explicit +9.81 gravity feedforward and an
   extra summing junction not present on X/Y — confirm intentional, not a
   leftover node." `FR_VEL_Z_GRAV_FF` above captures the feedforward value
   as a param; the *summing junction* is a control-law structure question
   that step 4 must resolve when the math is actually implemented — it
   does not block defining the params in this table.
   **Step 4a status (2026-09-07): still open, and two further problems
   with `FR_VEL_Z_GRAV_FF` itself were identified and are also carried
   rather than resolved** — its *units* (9.81 is an acceleration, but the
   loop output is a force in N and this airframe is ≈1.556 kg) and its
   *sign* (+9.81 on the NED Z axis points down, not up). The value is
   implemented literally as this table records it. See `controller.md`
   Open questions 3–5; a decision on any of the three changes the Z loop
   and its tests, not just a tuning number.
2. **Yaw rate loop zero I/D gain.** Step 4c implemented these as given
   and invented nothing; combined with the FF-as-P decision, yaw is now
   a pure proportional law, guarded by a regression test that asserts
   the yaw integral stays at exactly zero. `controller.md`: "confirm intentional
   (consistent with the small k=0.017 drag-coupling term) vs. unfinished
   tuning." `FR_RATE_YAW_I`/`FR_RATE_YAW_D` are defined at 0.0 either way
   — the param exists and is tunable regardless of which explanation is
   correct; this only matters once someone decides whether to change it
   from 0.

Neither open item blocks this spec's scope (param *definition*); both
block full *tuning confidence* and are step-4 concerns.

## Verification
- No independent test for param values alone beyond "the module loads
  without error and `parameters_updated()` populates its target fields
  with the configured defaults" — a reasonable step-3-only interface
  test (no Gazebo required), consistent with the working-style rule that
  every diff needs a test tied to a spec verification criterion.
- **Step 3 introduces the quaternion→Euler conversion.** Per
  `controller.md`'s Interface section: `vehicle_attitude` delivers a
  quaternion, but the gains in this table feed a Euler-angle cascade, so
  step 3's plumbing must introduce the conversion (ZYX, same convention
  as `Inertial2Body`) as soon as `vehicle_attitude` is read — not defer
  it to step 4. The conversion is `matrix::Eulerf(matrix::Quatf(q))`,
  PX4's own 3-2-1 intrinsic Tait-Bryan utility, already the established
  way this exact topic is read across `mc_att_control`,
  `vtol_att_control`, and EKF2 — not new math written here.
- **±90°-pitch singularity test — required, decided 2026-09-07, reversing
  a same-day decision to discard it.** The vehicle still isn't expected
  to reach ±90° pitch — that fact hasn't changed — but the test costs
  nothing (it's a handful of pure-math assertions, no Gazebo, no module
  instantiation) and is kept as a defensive check on this module's own
  usage of `matrix::Eulerf`, not a claim that the attitude is reachable.
  Implemented in `FoldrotorControlTest.cpp`
  (`FoldrotorControlQuaternionToEulerTest`): it round-trips through the
  exact wire-format conversion `FoldrotorControl.cpp` uses
  (`matrix::Quatf` constructed from a raw `float[4]`, matching
  `vehicle_attitude.q`), at both the special-cased band inside
  `Euler.hpp:88-94` (±90° − 5e-4 rad) and just outside it (±90° − 1e-2
  rad, still numerically sensitive since cos(theta) is near zero there).
  It asserts two things: the recovered phi/theta/psi stay finite, and
  the recovered triple describes the *same rotation* (DCM comparison,
  not raw angle equality — phi and psi individually aren't unique at
  gimbal lock, only their combination is). Verified 2026-09-07: builds
  and passes (`cmake --build build/px4_sitl_test --target
  functional-FoldrotorControl`, run directly — 4/4 tests pass, including
  this one).
- Not to be confused with the above: a **finite/NaN guard on estimator
  input** is a different check — defensive validation of what arrives,
  not singularity correctness — and belongs with the Estimator→Controller
  stale-data behavior that `system.md` still lists as TBD, not with
  step 3.
- The real verification criterion these params ultimately serve is
  `controller.md`'s existing one: given identical inputs, the PX4
  module's output must match the Simulink reference output within a
  defined tolerance. That comparison isn't possible until step 4 (the
  control law) exists to consume these gains.

## Not specified here
- Tuning ranges (`min`/`max`/`increment`/`decimal`) — step 4/5.
- The control law that reads these gains — step 4, `controller.md`.
  Steps 4a (position/velocity), 4b (inertial→body rotation) and 4c
  (attitude/rate) now exist as pure math; nothing is wired into `Run()`
  until 4e.
- Whether `FR_` is the final prefix — open until confirmed.
