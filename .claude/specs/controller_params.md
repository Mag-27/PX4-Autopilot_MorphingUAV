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

`min`/`max`/`increment`/`decimal` are left **TBD**: `controller.md` gives
only the Simulink-confirmed default values, no tuning range. Filling
these in is a step-4/5 concern once the control law exists to tune
against — inventing bounds now would be recording an assumption as a
constraint.

| Param | Loop | Meaning | Default | Notes |
|---|---|---|---|---|
| `FR_POS_P` | Position | P gain, all axes (x,y,z) | 3.0 | Shared across axes, per Simulink |
| `FR_VEL_XY_FF` | Velocity (X/Y) | Feedforward gain | 6.0 | |
| `FR_VEL_XY_I` | Velocity (X/Y) | Integral gain | 1.0 | |
| `FR_VEL_XY_D` | Velocity (X/Y) | Derivative gain | 1.0 | |
| `FR_VEL_Z_FF` | Velocity (Z) | Feedforward gain | 7.0 | |
| `FR_VEL_Z_I` | Velocity (Z) | Integral gain | 7.0 | |
| `FR_VEL_Z_D` | Velocity (Z) | Derivative gain | 0.1 | |
| `FR_VEL_Z_GRAV_FF` | Velocity (Z) | Gravity feedforward | 9.81 | See open item below — Z loop also has an extra summing junction not present on X/Y; not represented as a separate param here, since it's a control-law structure question (step 4), not a gain value |
| `FR_ATT_P` | Attitude | P gain, all axes (phi,theta,psi) | 3.0 | Shared across axes, per Simulink |
| `FR_RATE_RP_FF` | Rate (roll/pitch, Mx_b/My_b) | Feedforward gain | 3.5 | |
| `FR_RATE_RP_I` | Rate (roll/pitch) | Integral gain | 0.1 | |
| `FR_RATE_RP_D` | Rate (roll/pitch) | Derivative gain | 0.5 | |
| `FR_RATE_YAW_FF` | Rate (yaw, Mz_b) | Feedforward gain | 2.5 | |
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
2. **Yaw rate loop zero I/D gain.** `controller.md`: "confirm intentional
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
- **No ±90°-pitch singularity test — decided 2026-09-07, discarded not
  deferred.** An earlier revision of this spec required one. It is not
  required, for two reasons: (a) the vehicle cannot fly at ±90° pitch
  (user-confirmed 2026-09-07), so it guards an unreachable state; and
  (b) the gimbal-lock branch is inside `matrix::Eulerf`
  (`Euler.hpp:88-94` special-cases the pole, sets phi=0, folds phi+psi
  into psi — it does not emit NaN), and that library carries its own
  coverage under `src/lib/matrix/test/`. This module's code is a single
  delegating call, so a module-level singularity test would assert on
  someone else's already-tested branch. Recorded here with the reasoning
  so this reads as a decision, not an omission.
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
- Whether `FR_` is the final prefix — open until confirmed.
