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
`FR_RATE_R_FF`/`FR_RATE_P_FF`/`FR_RATE_YAW_FF` on 2026-09-07 (step 4c) — as a separate
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
| `FR_VEL_XY_I_LIM` | Velocity (X/Y) | Integrator windup limit (N) | 15.0 | Mirrors `FR_VEL_Z_I_LIM`. Wired into `PositionVelocityControl::setIntegratorLimit()` (`FoldrotorControl.cpp::parameters_updated()`) |
| `FR_VEL_Z_FF` | Velocity (Z) | **P gain on the velocity error** (see note) | 7.0 | Name says FF; meaning is P — decided 2026-09-07 |
| `FR_VEL_Z_I` | Velocity (Z) | Integral gain | 7.0 | |
| `FR_VEL_Z_D` | Velocity (Z) | Derivative gain | 0.1 | **Now 0.0 (2026-09-22).** Synthetic mass, not damping — see "Derivative gains zeroed" below. Its input, `vehicle_local_position.az`, carried 19.6 m/s² rms of airframe vibration, which this gain turned into 1.96 N rms of the 2.82 N rms commanded `Fz` |
| `FR_VEL_Z_GRAV_FF` | Velocity (Z) | Gravity feedforward | 15.260017 | **Resolved 2026-09-09** (was 9.81, the raw acceleration literal — see open item below): force-domain feedforward must equal the vehicle's measured weight in newtons, per `controller.md` Open questions 3/4. The Z loop's extra summing junction question is separate and still open, not represented as a param here since it's a control-law structure question (step 4), not a gain value |
| `FR_VEL_Z_I_LIM` | Velocity (Z) | Integrator windup limit (N) | 3.0 | **New 2026-09-09**, resolving the "no `FR_VEL_*_I_LIM` param" gap on this axis (see `AttitudeRateControl`'s analogous, still-unfilled `FR_RATE_*_I_LIM` gap below): bounds `FR_VEL_Z_I`'s accumulated integral directly via `PositionVelocityControl::setIntegratorLimit()`, separate from the still-inert output-limit/conditional-integration anti-windup. Chosen post-`FR_VEL_Z_GRAV_FF` fix: the integrator only needs to cover real trim/disturbance now, not a structural feedforward gap. No X/Y equivalent — not decided |
| `FR_VEL_XY_MAX` | Velocity (X/Y) | Max horizontal velocity setpoint (m/s) | 1.0 | **New 2026-09-17**, resolving `PositionVelocityControl.hpp` OPEN ITEM (b): clamps the position loop's `vel_sp` before the velocity PID (`mc_pos_control`'s `setVelocityLimits()`/`constrainXY()` equivalent). Added after real SITL testing showed an unlimited `vel_sp` from an ordinary position step could by itself approach the combined force-magnitude sphere (`FR_VEL_*` gains × unlimited velocity error), starving the allocator of per-rotor thrust headroom and forcing fold/tilt to their rails within milliseconds of arming — see `findings.md`'s 2026-09-17 entry. First-cut placeholder, **not yet verified against a logged step response** |
| `FR_VEL_Z_MAX_UP` | Velocity (Z) | Max climb velocity setpoint (m/s) | 1.0 | **New 2026-09-17**, same motivation/status as `FR_VEL_XY_MAX` |
| `FR_VEL_Z_MAX_DN` | Velocity (Z) | Max descent velocity setpoint (m/s) | 0.7 | **New 2026-09-17**, same motivation/status as `FR_VEL_XY_MAX`. Deliberately lower than `FR_VEL_Z_MAX_UP`, mirroring `mc_pos_control`'s `MPC_Z_VEL_MAX_DN < MPC_Z_VEL_MAX_UP` convention |
| `FR_ATT_P` | Attitude | P gain, all axes (phi,theta,psi) | 3.0 | Shared across axes, per Simulink |
| `FR_PITCH_LEVER` | Attitude | Fraction of pitch moment routed through the body-x tilt lever | **0.0** | **Not from Simulink** — added 2026-09-21, defaulted OFF 2026-09-22 once the ballast mast made pitch open-loop stable and shortened the lever arm 3.2x. 0 = drag path only, 1 = lever only. See allocation.md "Pitch actuation path" and findings.md (11), (12). |
| `FR_RATE_R_FF` | Rate (roll, Mx_b) | **P gain on the rate error** (see note) | 3.5 | Name says FF; meaning is P — decided 2026-09-07. Was `FR_RATE_RP_FF`, split 2026-09-21 |
| `FR_RATE_R_I` | Rate (roll) | Integral gain | 0.1 | Was `FR_RATE_RP_I` |
| `FR_RATE_R_D` | Rate (roll) | Derivative gain | 0.5 | Was `FR_RATE_RP_D`; 0.013 after the 2026-09-21 split. **Now 0.0 (2026-09-22)** — see "Derivative gains zeroed" below. Supplied 0.25 N·m rms of the 0.27 N·m commanded `Mx` |
| `FR_RATE_R_I_LIM` | Rate (roll) | Integrator windup limit (N*m) | 3.8 | Was `FR_RATE_RP_I_LIM` = 6.0; lowered to roll's hover moment authority |
| `FR_RATE_P_FF` | Rate (pitch, My_b) | **P gain on the rate error** | 0.17 | Roll's gain × Iyy/Ixx — see "Roll/pitch gain split" below. **Not flight-tuned** |
| `FR_RATE_P_I` | Rate (pitch) | Integral gain | 0.005 | Same Iyy/Ixx scaling |
| `FR_RATE_P_D` | Rate (pitch) | Derivative gain | 0.024 | Same scaling; 0.0029 after the 2026-09-21 split. The "−D/I = 162" rationale first recorded here was wrong — see the corrected pole in "Roll/pitch gain split" below. **Now 0.0 (2026-09-22)** — see "Derivative gains zeroed" |
| `FR_RATE_P_I_LIM` | Rate (pitch) | Integrator windup limit (N*m) | 0.3 | Sized to pitch's ~0.34 N*m hover authority, not roll's |
| `FR_RATE_YAW_FF` | Rate (yaw, Mz_b) | **P gain on the rate error** (see note) | 2.5 | Name says FF; meaning is P — decided 2026-09-07 |
| `FR_RATE_YAW_I` | Rate (yaw) | Integral gain | 0.0 | See open item below — confirm deliberate |
| `FR_RATE_YAW_D` | Rate (yaw) | Derivative gain | 0.0 | **Was silently raised to 0.013 during the 2026-09-21 pitch work** without updating the param description, which kept reading "zero per the Simulink reference model" — a code/spec disagreement caught 2026-09-22. **Back to 0.0**; it supplied 1.14 N·m rms of the 1.30 N·m commanded `Mz` and dominated the 18.8 Hz limit cycle. See "Derivative gains zeroed" below |
| `FR_WRENCH_LP` | Allocation input | Commanded-wrench low-pass corner (Hz) | **5.0** | **Not from Simulink** — added 2026-09-22. First-order, unity DC gain, applied to the commanded body force and moment after the rate loop's output clamp and before allocation. `<= 0` disables. See "Command-path bandwidth limit" below and `controller.md` |

## Derivative gains zeroed (2026-09-22)

`FR_RATE_R_D`, `FR_RATE_P_D`, `FR_RATE_YAW_D` and `FR_VEL_Z_D` are all
0.0 as of 2026-09-22. This is a structural consequence of the loop form
decided 2026-09-07, not a tuning preference.

The derivative acts on the **measurement**, not the error
(`controller.md` "Attitude/rate-loop form" item 2):

```
M = FF*e_r + I*∫e_r − D*ṙ        against a plant   I_body*ṙ = M
⇒  (I_body + D)*ṙ = FF*e_r + I*∫e_r
```

The D term collects onto the inertia. **It is synthetic inertia, not
damping.** Damping in this cascade comes from the rate loop's P acting on
the attitude loop's output. The same algebra applies to the velocity loop
(`F = FF*e_v + I*∫e_v − D*v̇` against `m*v̇ = F`, so `D` is synthetic mass).

What it bought, versus what it cost — measured from log
`2026-09-22/06_12_34.ulg`:

| axis | `D` | `D/I` | closed-loop BW | D term's share of the commanded moment |
|---|---|---|---|---|
| roll | 0.013 | 13% | 0.46 Hz | 0.25 of 0.27 N·m rms |
| pitch | 0.0029 | 7% | 0.36 Hz | 0.02 of 0.04 N·m rms |
| yaw | 0.013 | 19% | 0.62 Hz | **1.14 of 1.30 N·m rms** |
| vertical | 0.1 | 5% (`D/m`) | — | **1.96 of 2.82 N rms** |

A 5–19% inertia bump at a 0.4–0.6 Hz loop bandwidth is negligible, but
`|D·ṙ|` rises linearly with frequency: at the 18.8 Hz limit cycle,
`D·ω = 0.013 × 118 = 1.53 N·m` per rad/s of rate. Removing `D` slightly
*increases* the rigid-body damping ratio (yaw ζ 0.978 → 1.068) because it
removes inertia rather than damping.

`IMU_DGYRO_CUTOFF` = 20 Hz put the derivative filter's corner directly on
the 18.8 Hz limit cycle — maximum phase lag at ~0.7 gain, the worst
placement available. Left unchanged, since with `D` = 0 the D path is
inactive; it becomes relevant again only if a `D` gain is ever restored.

**If a D gain is restored,** the reason has to be something this algebra
does not cover — unmodelled actuator lag or a disturbance-rejection
requirement — and it needs a matched derivative cutoff well below the
actuator poles, not the default 20 Hz.

## Command-path bandwidth limit (2026-09-22)

`FR_WRENCH_LP` = 5.0 Hz. Nothing between the rate loop and the servos
bounded the command's **slew**, only its amplitude: the rate loop emits
at 250 Hz and the allocator is an algebraic map, so every frequency
component of the wrench lands directly on `alpha`/`beta`.

Measured from log `2026-09-22/06_12_34.ulg`, the joint torque required to
*track* the commanded angle (`J·d²θ/dt²` against `model.sdf`'s
`cmd_max = 5 N·m`):

| command | J about its axis | τ rms | τ p99 | over `cmd_max` |
|---|---|---|---|---|
| `beta` (tilt) | 0.00121 kg·m² | 12.1 N·m | 35.9 | **52% of samples** |
| `alpha` (fold) | 0.01359 kg·m² | 54.9 N·m | 175.3 | **89% of samples** |

The servos were saturated for most of the flight. A saturated actuator
inside a feedback loop contributes phase lag the linear design never
accounted for, which is what sustained the 18.8 Hz yaw limit cycle.

Corner chosen from the gap between the loops and the actuators:

| | frequency |
|---|---|
| pitch / roll / yaw closed-loop bandwidth | 0.36 / 0.46 / 0.62 Hz |
| **`FR_WRENCH_LP`** | **5.0 Hz** |
| fold servo dominant pole | 6.1 Hz (ωn 38.4 rad/s, ζ 0.48) |
| tilt servo dominant pole | 7.1 Hz (ωn 128.8 rad/s, ζ 1.61) |

An order of magnitude above every loop and below every actuator pole, so
it costs ~7° of phase at the fastest loop. Actuator poles are computed
from the SDF inertia about each joint axis against `model.sdf`'s
`p_gain=20 / d_gain=0.5` joint PID.

**Why a wrench filter and not a slew limit on `alpha`/`beta`.** Clipping
each angle independently distorts the delivered wrench in exactly the way
`allocation.md`'s clamp-not-redistribute does — the defect
`fitWrenchToEnvelope()` exists to prevent. Filtering the wrench keeps the
allocator's input a consistent wrench.

**Ordering.** Applied after the rate loop's output clamp, so the envelope
guarantee survives: a first-order low-pass of a signal bounded by
±`m_limit` is itself bounded by ±`m_limit`. Unity DC gain, so no
steady-state trim changes. The pitch lever is derived from the *filtered*
moment so that `fitWrenchToEnvelope()`'s `fx_lever_flu` matches the `Fx`
actually inside the commanded force.

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
   **Step 4a status (2026-09-07): identified two further problems with
   `FR_VEL_Z_GRAV_FF` itself** — its *units* (9.81 is an acceleration, but
   the loop output is a force in N) and its *sign* (+9.81 on the NED Z
   axis points down, not up).
   **Resolved 2026-09-09 (`controller.md` Open questions 3/4):** units
   were under-scaled — the force-domain feedforward must equal the
   measured hover weight (15.260017 N), not the raw `g` literal; sign was
   confirmed empirically correct as-implemented by the Part D bench runs
   (integrator winds up in the same direction as the feedforward, not
   opposing it). The summing-junction question (this item's other half)
   remains open — a decision there still changes the Z loop and its
   tests, not just a tuning number.

   **Windup bound resolved 2026-09-09, separately:** `FR_VEL_Z_I_LIM =
   3.0` bounds the accumulated Z integral itself (new param, table
   above), independent of the saturation/anti-windup mechanism. X/Y now
   have an equivalent param too (`FR_VEL_XY_I_LIM = 15.0`, added since).

   **Anti-windup mechanism REWORKED 2026-09-17** (`controller.md`'s
   velocity-loop-form decision 3, `PositionVelocityControl.hpp`): no
   longer uniform conditional integration on all three axes. Z keeps
   conditional integration, now against a dynamic vertical-priority
   sphere-saturation bound; X/Y switched to real tracking anti-windup
   (Rundqwist 1990), matching `mc_pos_control`'s actual asymmetric
   shape. The combined force-magnitude limit and horizontal margin that
   drive this (`FoldrotorControl.cpp`'s `kPosVelForceLimit`/
   `kPosVelForceXYMargin`) are wired in now — not inert — but are
   first-cut placeholder values, unverified against a logged clean
   hover (same open-item status the old independent boxes carried).

   **Velocity-magnitude limiting added 2026-09-17** (`PositionVelocityControl.hpp`
   OPEN ITEM (b), now resolved): real SITL testing (`hover_setpoint.sh`'s
   1.5 m ALT step) showed the previously-unbounded `vel_sp` let a single
   ordinary position-setpoint step consume nearly the entire combined
   force-magnitude sphere on its own (`FR_VEL_Z_FF=7.0 * unbounded vel_error`
   vs. the 28 N sphere cap), leaving the allocator no per-rotor thrust
   headroom to produce any attitude-loop moment and forcing fold/tilt to
   their rails within ~20 ms of arming — see `findings.md`'s 2026-09-17
   entry for the full trace. `FR_VEL_XY_MAX`/`FR_VEL_Z_MAX_UP`/
   `FR_VEL_Z_MAX_DN` (table above) now clamp `vel_sp` before it reaches the
   velocity PID, mirroring `mc_pos_control`'s `setVelocityLimits()`. Values
   are first-cut placeholders, **not yet verified against a logged step
   response** — same open-item status as the force-sphere constants above.
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


## Roll/pitch gain split (2026-09-21)

Roll and pitch shared one gain triple (`FR_RATE_RP_FF`/`_I`/`_D`) from
the Simulink transcription until 2026-09-21. **The user confirmed
(2026-09-21) that the Simulink rate loop was tuned against a different
inertia than this airframe's, and that `model.sdf`'s inertia is the one
that counts.** The shared gains therefore have no valid provenance here.

Inertia about the true CoM, from `model.sdf` (see
`src/modules/foldrotor_control/sitl_testing/allocation_study/`):

| axis | I (kg·m²) | shared P/I | hover moment authority |
|---|---|---|---|
| roll (x) | 0.0645 | 54 /s | 3.83 N·m |
| pitch (y) | **0.0031** | **1131 /s** | **0.34 N·m** |
| yaw (z) | 0.0649 | 38 /s | 3.85 N·m |

Pitch is ~21× lighter than roll and has ~11× less moment authority, so
one gain cannot suit both. Two consequences drove the split:

1. The rate loop feeds back measured angular acceleration as
   `−rate_dot · D`. **The mechanism first written here on 2026-09-21 was
   wrong and is corrected below** — it assumed a 1 kHz loop and an
   unfiltered difference, giving a root of `−D/I` (162 on pitch). Both
   assumptions were false: the loop ran at 250 Hz (`max_step_size`, and
   the IMU sensor's own `update_rate`), and `rate_dot` is low-passed at
   `IMU_DGYRO_CUTOFF`. The real pole is

   ```
   1 − α(1 + D/I),   α = dt/(τ + dt),   τ = 1/(2π·IMU_DGYRO_CUTOFF)
   ```

   which at dt = 4 ms was **−1.93 on BOTH roll and pitch** — so roll's
   own `D = 0.5` was equally unstable, and scaling pitch to match it
   scaled pitch to an unstable reference. Resolved 2026-09-21 by raising
   the simulation to 1 kHz (`worlds/foldrotor.sdf` plus the flight
   model's `imu_sensor` `update_rate`), where both axes sit at 46% of the
   stability limit and need no gain change. `FR_RATE_P_D` is still scaled
   from roll, but now for the bandwidth-matching reason (2) below, not
   for the stability reason. See findings.md 2026-09-21 (2) and (3).
   **The 1 kHz IMU rate is load-bearing: the margin runs out at
   dt ≥ 2.5 ms.**
2. `FR_RATE_RP_I_LIM = 6.0 N·m` was ~20× pitch's entire hover authority,
   so the pitch integrator could wind to a value the vehicle could never
   deliver.

Defaults are roll's values scaled by `Iyy/Ixx ≈ 0.048`, giving matched
closed-loop bandwidth rather than matched gain. **These are first-cut
values from that scaling plus a loop simulation — not flight-tuned.**
They need a clean hover to confirm, the same way
`kPosVelForceLimit`/`FR_VEL_*_MAX` still do.
