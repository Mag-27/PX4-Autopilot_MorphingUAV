# Controller Specification

## Status
**Skeleton only, as of the 2026-09-07 audit.**
`src/modules/foldrotor_control/` exists (`FoldrotorControl.cpp/.hpp`),
but it is lifecycle/uORB plumbing only: the module starts, subscribes to
its required inputs, and logs a heartbeat — it does not yet compute or
publish any control law. `parameters_updated()` is an empty stub (see
`controller_params.md`, step 3). No wrench is computed, nothing is
published to `Control_Alloc`/actuators. This spec remains the contract
to build the actual control law against, not a description of existing
behavior. The reference implementation is the Simulink model.
(`src/modules/mc_raptor` exists in this tree but is an unrelated
RL-policy flight-mode module — don't mistake it for this target.)

## Scope
Position + attitude control law. Algorithm reference: the validated
Simulink model is the source of truth for the math. This spec covers the
PX4 module's I/O contract, not a re-derivation of the control law.

## Interface
**In:** state from estimator — position (inertial/NED, confirmed),
velocity (inertial/NED, confirmed — same frame as position, per
`VehicleLocalPosition.msg`), attitude (**quaternion on the wire**, not
Euler — `vehicle_attitude` publishes `q[4]`, Hamilton convention, FRD
body → NED, per `VehicleAttitude.msg`; confirmed 2026-09-07), angular
rate (FRD body, rad/s, confirmed via `VehicleAngularVelocity.msg`).

The Simulink cascade math (position P → velocity PID → attitude P → rate
PID) is written in terms of Euler angles phi/theta/psi throughout,
including the `Inertial2Body` rotation above. Since the actual PX4
topic is a quaternion, the module must convert quaternion → Euler
internally, **same ZYX (yaw-pitch-roll) convention already used for
`Inertial2Body`**, before any of that cascade math runs. This conversion
is a new step not present in the Simulink reference (which was never
given a quaternion to begin with), but it is not new math: use
`matrix::Eulerf(matrix::Quatf(q))`, PX4's own 3-2-1 intrinsic
Tait-Bryan utility, already the established way this exact topic is
read across `mc_att_control`, `vtol_att_control`, and EKF2.

The ZYX extraction has a known singularity at pitch = ±90°. **A
module-level test for it is required — decided 2026-09-07, reversing
the same-day decision to discard it.** The vehicle still isn't expected
to reach ±90° pitch (that hasn't changed), but the test is cheap,
already written, and passing, so it's kept as a defensive check on this
module's specific usage rather than relying solely on `matrix::Eulerf`
(`Euler.hpp:88-94` special-cases the pole, sets phi=0, folds phi+psi
into psi — no NaN) carrying its own coverage under
`src/lib/matrix/test/`. The kept test (`FoldrotorControlTest.cpp`,
`FoldrotorControlQuaternionToEulerTest`) checks rotation-equivalence via
DCM comparison at the pole, not raw phi/theta/psi equality — phi and
psi individually aren't unique at gimbal lock, but the recovered triple
must describe the same rotation, since that's what `Inertial2Body`
(step 4) will actually apply. It exercises the exact wire-format round
trip (`matrix::Quatf` from a raw `float[4]`) `FoldrotorControl.cpp`
uses, not just the library in the abstract. See `controller_params.md`
for how step 3 scopes this.

**Out:** desired wrench — force (Fx_b,Fy_b,Fz_b), moment
(Mx_b,My_b,Mz_b), confirmed **body frame** (explicit `_b` suffix in the
Simulink model). Must match allocation.md's expected input.

## Structure (confirmed from Simulink)
Cascaded: position P (Kp=3, all axes) → velocity PID → attitude P
(Kp=3, all axes) → rate PID → allocation.
- Velocity loop gains: X/Y use FF=6, I=1, D=1. Z (altitude) uses
  FF=7, I=7, D=0.1, plus an explicit +9.81 gravity feedforward and an
  extra summing junction not present on X/Y — confirm intentional, not
  a leftover node.
- Rate loop gains: roll/pitch (Mx_b, My_b) use FF=3.5, I=0.1, D=0.5.
  Yaw (Mz_b) uses FF=2.5, **I=0, D=0** — yaw rate control is currently
  proportional/feedforward only. Confirm intentional.
- Several blocks (phi/theta branches in the attitude loop, qsp branch
  in the rate loop) show unconnected, red-highlighted ports in the
  Simulink diagrams — worth checking these aren't dangling logic.
  **Still unresolved after step 4c, and not resolvable from this
  document:** whether those ports are dead nodes or carry logic
  `AttitudeRateControl` is missing cannot be told from the prose
  description — it needs the actual diagram images. They are explicitly
  NOT assumed to be dead ends (OPEN ITEM (c) in
  `AttitudeRateControl.hpp`). If any is live, that class is incomplete.

### Velocity-loop form (decided 2026-09-07, step 4a)
The gain table above records FF/I/D per loop but says nothing about the
loop's *form*. Four things had to be settled before the math could be
written; all four were user decisions, not derivations, and none is
recoverable from the Simulink prose alone:

1. **"FF" is the P gain on the velocity error**, not a feedforward on
   the setpoint: `F = FF*e_v + I*integral(e_v) - D*vel_dot`. The params
   keep the name `FR_VEL_*_FF` (already published); only the meaning is
   pinned. The competing reading — FF on `v_sp` plus a separate P —
   would require a P value that exists in no spec.

   **Velocity-magnitude limiting ADDED 2026-09-17** (`PositionVelocityControl.hpp`
   OPEN ITEM (b), resolved): the velocity setpoint (`v_sp = FR_POS_P * e_p`)
   was unbounded until now — real SITL testing (`hover_setpoint.sh`'s 1.5 m
   ALT step) showed this let one ordinary position-setpoint step demand
   more instantaneous force than `FR_VEL_Z_FF` (7.0) times the sphere
   saturation could absorb without leaving the allocator any per-rotor
   thrust headroom for the attitude loop's moment — the observed symptom
   was fold/tilt slamming to their rails within ~20 ms of arming, not a
   gradual tip-over. `PositionVelocityControl::setVelocityLimits()` now
   clamps `v_sp` before the velocity PID: horizontal is a plain magnitude
   scale (no feedforward `v_sp` term exists here to blend against, unlike
   `mc_pos_control`'s `ControlMath::constrainXY`), vertical is asymmetric
   up/down (`FR_VEL_Z_MAX_UP`/`FR_VEL_Z_MAX_DN`), matching
   `mc_pos_control`'s convention. See `findings.md`'s 2026-09-17 entry for
   the full force-budget trace and `controller_params.md` for the new
   params — first-cut placeholders, not yet verified against a logged step
   response.
2. **The derivative acts on the measurement and is supplied as an
   input**, not differentiated internally: `-D*vel_dot`, following
   `mc_pos_control` (`PositionControl.cpp:150`). Step 4e feeds it
   `vehicle_local_position`'s ax/ay/az, which the estimator has already
   filtered; there is no D filter inside the module. **This may diverge
   from the Simulink reference** if that differentiated the error —
   `v_sp = FR_POS_P * e_p` steps on every position-setpoint change and
   derivative-on-error would kick there, while this does not. Deliberate
   and recorded; the Verification criterion below is what would expose
   it.
3. **Anti-windup — REWORKED 2026-09-17, now `mc_pos_control`'s actual
   asymmetric shape, not the uniform generalization described below.**
   Diagnostic work traced the velocity loop to demanding roughly double
   the correct per-rotor hover trim (~15 N/rotor observed in SITL vs a
   correct ~7.63 N/rotor) before the allocator ever saw it, and the user
   decided to stop iterating on the bespoke design and instead port
   `mc_pos_control`'s `PositionControl` structure directly (skipping its
   thrust→attitude conversion, which doesn't apply to this fully-actuated
   vehicle) — see `PositionVelocityControl.hpp`'s header comment and
   `.claude/plans/read-mc-pos-contorl-and-can-greedy-pearl.md` for the
   full diagnosis and decision record. **This is a deliberate departure
   from this document's Simulink-source-of-truth for this loop's
   STRUCTURE only — the gain values are unchanged.** Z now uses
   conditional integration against a dynamic vertical-priority
   sphere-saturation bound (`PositionControl.cpp:157-186`); X/Y now use
   real tracking anti-windup (Rundqwist 1990, `PositionControl.cpp:
   188-199`), simplified to compare desired-vs-produced force directly in
   newtons rather than mc_pos_control's acceleration/hover-thrust round
   trip, since this loop's gains already act in the force domain
   (decision 1).

   **Previous design (superseded 2026-09-17), kept for history only —
   does not describe the running code:** anti-windup was conditional
   integration on all three axes, freezing an axis' integrator when its
   output was saturated in the direction that would worsen it —
   `mc_pos_control`'s vertical algorithm generalized to x and y, not a
   match to its horizontal one (X/Y there used tracking anti-windup
   instead). The uniform choice was defensible — simple, symmetric,
   needing no achievable-output estimate — but was a choice, not a
   derivation, and the saturation bounds it depended on
   (`setOutputLimits()`, independent per-axis boxes) left the mechanism
   inert until step 4e supplied real values.

   **Separately, `FR_VEL_Z_I`'s own windup bound is now set — RESOLVED
   2026-09-09.** `FR_VEL_Z_I_LIM = 3.0` N bounds the accumulated Z
   integral directly (`PositionVelocityControl::setIntegratorLimit()`,
   mirroring `AttitudeRateControl`'s existing mechanism of the same
   name), independent of the conditional-integration/output-limit
   mechanism above. This is a decision, not a derivation: post the
   `FR_VEL_Z_GRAV_FF` fix (Open question 3, below) the integrator no
   longer has to make up a ~5.4 N structural feedforward gap on its own,
   so its remaining job is real trim/disturbance rejection, which 3 N
   comfortably covers without inventing a value from nothing. X and Y
   have no equivalent param and remain unbounded — not evaluated here.
4. **Output stays a force in newtons with `FR_VEL_Z_GRAV_FF` entering
   as a literal added directly to the NED Z axis.** Originally `+9.81`,
   exactly as `controller_params.md` recorded, carrying two known
   problems rather than silently correcting them. **Resolved 2026-09-09**
   (Open questions 3 and 4 below): under-scaled, now `15.260017` (the
   measured hover weight); sign confirmed empirically correct as-is.

**Status (step 4a, 2026-09-07; anti-windup REWORKED 2026-09-17): wired
into `Run()` at 50 Hz.**
`src/modules/foldrotor_control/PositionVelocityControl.hpp` provides
`foldrotor::PositionVelocityControl`, whose `update()` returns the
inertial/NED desired force as a `matrix::Vector3f` so it composes
directly with step 4b's `foldrotor::inertialToBody(F_i, euler)`. Tested
in `FoldrotorControlTest.cpp` (`FoldrotorPositionVelocityControlTest`, 18
tests) against hand-computed values at the spec gains: the nominal X/Y
cascade, per-axis D gains, X/Y and Z integral accumulation, integrator
reset, the default unbounded behaviour, Z's conditional-integration
freeze (both saturating and un-saturating), the `FR_VEL_Z_I_LIM` clamp,
X/Y's tracking anti-windup dampening (rather than freezing) integration
while saturated, and (added 2026-09-17) velocity-magnitude limiting —
horizontal magnitude scaling that preserves direction, asymmetric
vertical up/down clamping, and the default no-clamp case. The combined
force-magnitude limit/horizontal margin and the velocity limits are all
first-cut placeholders — see `PositionVelocityControl.hpp`'s OPEN ITEMS
(a) and (b) — not yet verified against a logged clean hover or step
response.


### Attitude/rate-loop form (decided 2026-09-07, step 4c)
Same situation as the velocity loop: FF/I/D per loop, nothing about
form. Decided by the user 2026-09-07.

1. **"FF" is the P gain on the rate error** —
   `M = FF*e_r + I*integral(e_r) - D*rate_dot` — carrying the
   velocity-loop reading forward. Recorded explicitly because **this one
   goes against the PX4 precedent rather than with it**:
   `RateControl::update()` (`src/lib/rate_control/rate_control.cpp:78`)
   computes `_gain_p.emult(rate_error) + _rate_int -
   _gain_d.emult(angular_accel) + _gain_ff.emult(rate_sp)`, i.e. PX4's
   rate controller has a distinct feedforward on the *setpoint*,
   separate from P. The competing reading was rejected because it needs
   a P value in no spec, and because with `FR_RATE_YAW_I` =
   `FR_RATE_YAW_D` = 0 it would leave yaw with no error feedback at all.
   If the Simulink rate-loop diagram turns up showing a separate P, this
   is the decision to revisit.
2. **Derivative on the measurement, supplied as an input** —
   `-D*rate_dot`. Verified against the files, not asserted:
   `MulticopterRateControl.cpp:137` reads
   `const Vector3f angular_accel{angular_velocity.xyz_derivative};` and
   passes it to `_rate_control.update()` (`:220`), where
   `rate_control.cpp:78` applies `- _gain_d.emult(angular_accel)`,
   uniformly on all three axes. Step 4e should feed this class
   `vehicle_angular_velocity`'s `xyz_derivative`.
3. **The Euler attitude error is wrapped to [-pi, pi] on all three
   axes.** Not in any spec — a deliberate addition to the Simulink
   structure, because an unwrapped yaw error crossing ±pi yields a ~2pi
   error and a large command in the *wrong* direction.
4. **The integral follows `mc_rate_control`'s `updateIntegral()` in
   full** (`rate_control.cpp:88-116`): conditional integration
   (`:91-98`, all three axes — a genuine precedent match here, unlike
   the velocity loop's), the nonlinear `i_factor` that fades the I gain
   with rate error (`:100-107`, PX4's hard-coded 400° scale, imported
   as-is), the finiteness guard and integrator clamp (`:112-114`), and
   the landed gate (`:81-83`). **Two of these are inert by default:**
   the output bounds come from allocation (4d), exactly as PX4 sources
   them from control-allocation feedback
   (`MulticopterRateControl.cpp:196-215`), and no `FR_RATE_*_I_LIM`
   param exists for the integrator clamp. Both default to ±infinity
   rather than inventing values, so until 4d/4e supply them **the rate
   anti-windup and the integrator clamp are inert at runtime.** A
   parameter for the clamp is a new, unfilled gap in
   `controller_params.md`'s table.

5. **The attitude loop owns body-x force as a pitch actuator (decided
   2026-09-21).** Previously `Fx` belonged entirely to the position
   loop and the attitude loop had no access to it, which left pitch
   actuated only through `M0`'s weak drag-coupling term. The rate
   loop's `My` output is now also expressed as a body-x force request,
   `Fx = FR_PITCH_LEVER * My_frd / 0.0549`, bounded separately from the
   position loop's own horizontal budget.

   This is a genuine widening of the attitude loop's authority, recorded
   explicitly because it breaks the clean cascade separation the rest of
   this spec assumes: the attitude loop now perturbs a quantity the
   position loop believes it controls. The vehicle translates while it
   corrects pitch. That is accepted — see allocation.md "Pitch actuation
   path" for why no gain set can stabilise the 2.77 Hz pitch pole through
   the 6.1 Hz fold actuator, and findings.md (11) for the measurements.

   The two budgets are deliberately NOT shared: the position loop's `Fx`
   is the pitch-destabilising direction (findings.md (9)), the attitude
   loop's carries the correcting sign.

   **OFF BY DEFAULT as of 2026-09-22** (`FR_PITCH_LEVER` = 0). The ballast
   mast made the pitch axis open-loop stable, which removes the reason
   this widening existed, and shortened the lever arm 3.2x, which makes it
   a bad trade anyway. The cascade separation above is therefore intact at
   the shipped defaults; the mechanism remains available and specified.
   See allocation.md "Pitch actuation path" and findings.md (12).

**Status (step 4c, 2026-09-07): implemented as pure math, not wired in.**
`src/modules/foldrotor_control/AttitudeRateControl.hpp` provides
`foldrotor::AttitudeRateControl`, whose `update()` returns
(Mx_b, My_b, Mz_b) as a `matrix::Vector3f` in body/FRD — allocation.md's
moment input, needing no rotation (the moment path is body-frame by
convention already). `euler_sp` is a plain argument; where it comes from,
including whether phi_sp/theta_sp are always zero for this fully-actuated
vehicle, is a 4e question and is deliberately unresolved. Nothing calls
the class — wiring is 4e. Tested in `FoldrotorControlTest.cpp`
(`FoldrotorAttitudeRateControlTest`, 16 tests) against hand-computed
values at the spec gains, with roll/pitch and yaw covered separately
since their gains differ: the nominal cascades, yaw staying purely
proportional, per-axis D gains, a derivative-on-measurement
discriminator, integral accumulation including the `i_factor` and its
zero floor, the landed gate, the integrator clamp, reset, conditional
integration and the directional un-saturation case, ±pi wrapping on both
yaw and roll, and the per-axis moment ordering the allocator expects.
All the anti-windup and integral mechanisms were mutation-tested.

### Run() wiring (decided 2026-09-07, step 4e part 1)
Six user decisions, made while wiring 4a/4b/4c into `Run()` and none
recoverable from this spec's prior text — detail in
`findings.md` ("Step 4e part 1 wiring decisions"):

1. **Multi-rate cascade cadence: position/velocity at 50 Hz, attitude at
   250 Hz, rate at 1000 Hz.** This is NOT derived from the Simulink
   reference or any prior section of this document — checked by grep
   before implementing, since a citation claiming "per controller.md"
   for this cadence would otherwise have gone unverified. It is a step
   4e decision, recorded here now as the source of truth for it.
2. **`AttitudeRateControl`'s interface was split** (`updateAttitude()`
   at 250 Hz, `updateRate()` at 1000 Hz) because step 4c's single
   `update()` always recomputes `rate_sp` from the current Euler error
   in one call and cannot be run at two different rates as-is.
   `update()` is kept, unchanged in behaviour, as a convenience entry
   point; step 4c's 16 hand-computed tests continue to pass against it
   unmodified.
3. **`euler_sp`: phi_sp/theta_sp pinned to zero**, resolving the "4e
   question" step 4c left open above — consistent with this vehicle
   translating by thrust vectoring, not body lean, which is this
   document's own explanation (Identified issue 1, below) for why the
   missing force-path rotation stayed hidden. psi_sp comes from
   `trajectory_setpoint.yaw`, held at the current heading when NaN.
4. **Validity/NaN gating**: full `mc_pos_control`-style — estimator
   `_valid` flags plus `PX4_ISFINITE` on the setpoint, holding the
   previous wrench on invalid input and resetting the integrator on
   recovery. **New open item this creates:** `PositionVelocityControl`
   has no independent velocity-setpoint path (`vel_sp` is always derived
   from `pos_sp`), so a NaN position with a live velocity setpoint
   cannot be honored without extending that class — not attempted here.
5. **EKF reset counters** (`xy_reset_counter` etc.): explicitly NOT
   handled — recorded as an open item. Harmless for now since nothing
   downstream of this module publishes to actuators yet.
6. **Integrator reset on disarm only.** This module has no PX4
   flight-mode concept of its own (it always runs the same cascade), so
   "reset on mode entry" has no equivalent here and is not implemented —
   recorded as an open item rather than guessed at.

**Status (step 4e part 1, 2026-09-07): the cascade is wired into
`Run()` and computes a real wrench (`_F_b`, `_M_b`) every cycle, but
publishes it nowhere.** No `actuator_motors`/`actuator_servos` wiring
exists; the allocator (4d) and the airframe/rc.txt change that would
stop the stock `mc_*` stack starting are separate, later diffs. Verified
in SITL (`px4_sitl_foldrotor` board config + SIH quadx substrate, since
this module has no dedicated Gazebo airframe of its own yet):
`foldrotor_control start`/`status`/`stop` all work, `status` prints the
live wrench, `mc_pos_control`/`mc_att_control`/`mc_rate_control` boot
and run normally alongside it (unaffected, per the Hard constraint that
this module doesn't touch them), and the boot log carries no `ERR`
lines. The logged wrench's Z force started near the literal +9.81 N
gravity feedforward and climbed toward ~10.9 N over several seconds —
`FR_VEL_Z_I`'s integrator winding up toward the known ~15.26 N hover
weight gap (Open question 3, below) with anti-windup confirmed inert
(bounds still ±infinity), exactly as `PositionVelocityControl.hpp`'s
OPEN ITEM (a) predicted. Expected behavior, not a new finding.

### Command-path bandwidth limit (added 2026-09-22)
Not from Simulink. A first-order low-pass (`FR_WRENCH_LP`, default 5 Hz,
unity DC gain) on the commanded body force and moment, applied **after**
the rate loop's output clamp and the pitch lever and **before**
`frdToAllocatorFlu()` / `fitWrenchToEnvelope()`. `<= 0` disables it and
restores the unfiltered path.

**The contract it adds:** the wrench handed to allocation must be one the
servos can physically execute, not merely one inside the moment envelope.
The envelope fit (`allocation.md`) bounds the command's *amplitude*;
nothing bounded its *slew*. The rate loop runs at 250 Hz and allocation is
algebraic, so all of the command's frequency content reached `alpha`/
`beta` directly. Measured in log `2026-09-22/06_12_34.ulg`: tracking the
commanded angles needed 12.1 N·m rms (tilt) and 54.9 N·m rms (fold)
against `model.sdf`'s `cmd_max = 5 N·m`, exceeding it on 52% and 89% of
samples. The saturated servos' phase lag sustained an 18.8 Hz yaw limit
cycle (truth yaw rate 1.06 rad/s rms, 82% of power above 10 Hz).

This does **not** change the control law's form — the loops are unchanged
and the filter has unity DC gain, so any steady-state comparison against
the Simulink reference is unaffected. It changes only how fast the
commanded wrench is allowed to move, and only above 5 Hz, which is an
order of magnitude above every closed-loop bandwidth in the cascade.

Two ordering properties this relies on, both regression-tested
(`FoldrotorWrenchLowPassTest`):
1. A first-order low-pass of a signal bounded by ±`m_limit` is itself
   bounded by ±`m_limit`, so running it after the rate loop's clamp
   preserves the feasibility guarantee that `fitWrenchToEnvelope()` and
   the conditional-integration anti-windup both rely on.
2. The pitch lever is derived from the **filtered** moment, because the
   lever *is* the pitch moment expressed across the 0.0549 N·m/N arm. A
   lever taken from the unfiltered moment would not match the `Fx`
   actually inside the commanded force, and `fitWrenchToEnvelope()` would
   protect the wrong amount of body-x force at moment priority.

Sizing, the actuator pole computation, and the reason this is a wrench
filter rather than a slew limit on `alpha`/`beta` are in
`controller_params.md` "Command-path bandwidth limit".

## Identified issues (confirmed)

### 1. Missing inertial→body rotation on the force path
Confirmed: position AND velocity (x,y,z / Vx,Vy,Vz) are both inertial.
The velocity loop's output is currently the inertial-frame desired
force, fed to `Control_Alloc` as if it were body-frame — no rotation
exists between them.

Why it hasn't shown up in testing: this vehicle is fully actuated
(independent per-rotor tilt), so it doesn't need large attitude
excursions to translate. Near level (phi,theta,psi ≈ 0), inertial ≈
body, so the error is small; the velocity loop's integral action also
partially absorbs a slowly-varying mismatch. Maneuvers "working" is
validation evidence, not verification — the bug will surface once a
maneuver drives attitude far enough from level that R(phi,theta,psi)
departs meaningfully from identity (sustained yaw during translation,
commanded roll/pitch, off-level disturbance rejection).

**Fix:** insert a rotation stage between the velocity-loop output and
`Control_Alloc`'s Fx_d,Fy_d,Fz_d inputs, using the *current estimated*
attitude (not setpoint), standard ZYX (yaw-pitch-roll) convention:

```matlab
function [Fx_b, Fy_b, Fz_b] = Inertial2Body(Fx_i, Fy_i, Fz_i, phi, theta, psi)
%#codegen
    cphi = cos(phi);   sphi = sin(phi);
    cth  = cos(theta); sth  = sin(theta);
    cpsi = cos(psi);   spsi = sin(psi);
    % Rt = inertial -> body (transpose of standard ZYX body->inertial DCM)
    Rt = [ cpsi*cth,                  spsi*cth,                 -sth;
           cpsi*sth*sphi - spsi*cphi, spsi*sth*sphi + cpsi*cphi, cth*sphi;
           cpsi*sth*cphi + spsi*sphi, spsi*sth*cphi - cpsi*sphi, cth*cphi ];
    F_body = Rt * [Fx_i; Fy_i; Fz_i];
    Fx_b = F_body(1); Fy_b = F_body(2); Fz_b = F_body(3);
end
```

Moment path (Mx_b,My_b,Mz_b) is unaffected — p,q,r and body moments are
body-frame by convention already, no rotation needed there.

**REWORKED 2026-09-17: `Rt` above is no longer what runs.** Alongside
`PositionVelocityControl`'s mc_pos_control-structure rework (see that
section above and `.claude/plans/read-mc-pos-contorl-and-can-greedy-
pearl.md`), this stage was reduced to a yaw-only 2D rotation (phi=theta=0
substituted into the `Rt` above): roll/pitch's contribution is dropped
deliberately — this vehicle translates by independent per-rotor thrust
vectoring, not by leaning the body — while yaw is kept, since it is
actively commanded and independently varies (confirmed empirically: a
SITL hover test commanded 90° of yaw while hovering). This also removes
the pitch=±90° singularity concern this stage's full rotation used to
carry (the quaternion→Euler extraction's own singularity, Interface
section above, is unrelated and unaffected — `AttitudeRateControl` still
needs the full Euler triple). See `Inertial2Body.hpp`'s header comment.

**Status (step 4b, 2026-09-07; yaw-only rework 2026-09-17): wired into
`Run()` every 50 Hz position/velocity cycle**
(`Run()` computes `F_i` via `PositionVelocityControl::update()`, then
`_F_b = foldrotor::inertialToBody(F_i, _euler)`).
`src/modules/foldrotor_control/Inertial2Body.hpp`
provides `foldrotor::inertialToBodyRotation(euler)` (now the yaw-only
reduction above) and `foldrotor::inertialToBody(F_i, euler)`. Tested in
`FoldrotorControlTest.cpp`
(`FoldrotorControlInertial2BodyTest`, 6 tests): identity at level
attitude, roll/pitch ignored at zero yaw, proper rotation (`Rt^T·Rt = I`
*and* `det = +1`, the latter
ruling out a reflection that orthonormality alone would admit), hand-
computed pure-yaw cases (90° and 45°), and equivalence to the pure-yaw
reduction of PX4's own `Dcm(Euler).transpose()` across several yaw
angles.

**The "standard ZYX" claim above was confirmed for the full rotation,
2026-09-07; since the 2026-09-17 yaw-only rework, `Rt` is exactly
`matrix::Dcmf(Eulerf(0,0,psi)).transpose()` — the same claim, reduced to
the yaw-only special case.** PX4's `Dcm(const Euler&)` constructor
(`src/lib/matrix/matrix/Dcm.hpp:121-142`) builds the standard 3-2-1
intrinsic Tait-Bryan body→inertial DCM, and transposing it at phi=theta=0
reproduces this stage's three nonzero expressions element-for-element.
This matters beyond pedantry — it confirms the yaw-only reduction is the
correct special case of the original transcription, not an independent
2D rotation invented separately, and it composes with step 3's
`_euler = matrix::Eulerf(matrix::Quatf(q))` with no convention
adaptation. Asserted across five yaw angles (with nonzero roll/pitch in
the input, to also confirm they're ignored) by
`MatchesPx4DcmYawOnlyTransposeAcrossYawAngles` rather than left as a
comment.

New dependency introduced: `Control_Alloc`'s effective correctness now
depends on attitude estimate quality, not just the desired wrench. If
phi,theta,psi is stale or wrong, allocation is wrong even when the
inertial-frame force command was correct — reflect this in the
Estimator→Controller stale-data contract once that row is filled in.

## Open questions
1. The attitude→rate mapping (image 2) uses a direct proportional gain
   on Euler angle error, which implicitly assumes Euler angle rate ≈
   body angular rate — only exact for small pitch; the exact relation
   needs a T(Θ) transformation. Not confirmed as a problem; worth the
   same kind of check once the force-path fix above is in.
   **Step 4c implemented the direct mapping exactly as specified and did
   NOT add T(Θ)** — adding it silently would move the control law away
   from the Simulink reference, and omitting it silently would bury a
   known approximation. It is flagged as OPEN ITEM (a) in
   `AttitudeRateControl.hpp`. The approximation error grows with pitch.
2. Yaw rate loop has zero I/D gain — confirm deliberate (consistent
   with the small k=0.017 drag-coupling term) vs. unfinished tuning.
   **Step 4c implemented them as given and invented nothing.** Combined
   with the FF-as-P decision above, yaw is now a *pure proportional*
   law: no integral, no derivative, no feedforward. A regression test
   (`NominalYawMatchesHandComputedCascadeAndStaysProportional`) asserts
   the yaw integral stays at exactly zero, so a future decision to make
   these gains nonzero has to break a test rather than pass silently.
3. **`FR_VEL_Z_GRAV_FF` units — RESOLVED 2026-09-09, under-scaled.**
   9.81 was an *acceleration*, but the velocity loop's output is a force
   in newtons per `allocation.md`. Derived from the vehicle's own
   translational dynamics, `p_ddot = R_b^i * F_b/m - [0,0,g]^T`: for a
   level hover (`p_ddot = 0`), the force-domain feedforward on `F_b,z`
   must equal the vehicle's actual weight `mg` in newtons, not the raw
   `g` literal. This airframe measures 15.260017 N static
   (`findings.md`, 2026-09-09 Part D bench run), so `FR_VEL_Z_GRAV_FF`'s
   default is now that measured value directly — a direct measurement,
   not a computed `m` times a separate `g` constant. Confirmed by the
   2026-09-09 Part D bench measurement showing `FR_VEL_Z_I` climbing
   ~5.4 N from windup alone to close the gap between the old 9.81 N
   literal and the true ~15.26 N hover weight — exactly the shortfall
   this predicted. It was under-scaling, not a Simulink
   acceleration-units-with-mass-folded-in-downstream design; no evidence
   for the latter was ever found.
4. **`FR_VEL_Z_GRAV_FF` sign — RESOLVED 2026-09-09, empirically, not by
   frame reasoning.** The literal is added directly and positive
   (`force(2) += _grav_ff`, no other sign step), and both the pre-fix and
   post-fix 2026-09-09 Part D bench runs show `FR_VEL_Z_I` winding up in
   the *same* direction as the feedforward to reach hover weight, never
   opposing it — so the sign this runtime already uses is the one it
   needs. This does not settle whether the underlying convention is
   really NED with the Simulink reference authored Z-up, or something
   else entirely; it only confirms the empirical sign is correct for this
   module as built. Recorded as resolved-in-practice rather than
   resolved-in-theory, so a future reader doesn't mistake this for a
   frame-convention derivation it isn't.
5. **The Z velocity loop's "extra summing junction not present on X/Y"
   (Structure, above) is still unresolved.** It is not guessable from
   the prose description and needs the Simulink velocity-loop diagram.
   Step 4a implements Z as FF/I/D plus `FR_VEL_Z_GRAV_FF` as given — the
   junction is neither implemented nor invented, and is flagged in
   `PositionVelocityControl.hpp`'s header comment as OPEN ITEM (c). If
   it carries logic, this loop is wrong on Z and the fix belongs in that
   file.
6. **No independent velocity-setpoint path.** `PositionVelocityControl`
   always derives `vel_sp` from `pos_sp`; `Run()`'s validity gating
   (step 4e part 1) can only gate on `trajectory_setpoint.position`
   being finite, so a NaN position paired with a live velocity setpoint
   is not honored. Extending the class to accept `vel_sp` directly would
   be a new decision, not made here.

   **Related, resolved 2026-09-09: `trajectory_setpoint`'s publisher.**
   This module was always meant to consume whatever `trajectory_setpoint`
   says (decision 6/item 8 below — no flight-mode concept of its own),
   but was never meant to *generate* that setpoint itself, and the
   airframe change that stopped the stock stack (`allocation.md`/plan
   O-4) also stopped `flight_mode_manager`, the thing that normally
   publishes it — leaving `trajectory_setpoint` with no publisher at all.
   `Run()`'s own validity gating (`PX4_ISFINITE` on `position[0..2]`)
   didn't catch this, because a never-published, zero-initialized
   `trajectory_setpoint_s` reads as *finite* (all-zero, not NaN) — so the
   module happily computed a real wrench against a stale `pos_sp =
   (0,0,0)` regardless of where the vehicle actually was.

   **Checked (not assumed) before fixing:** `flight_mode_manager`
   (`src/modules/flight_mode_manager/FlightModeManager.cpp`) depends only
   on `vehicle_control_mode`/`vehicle_status`/`vehicle_land_detected`/
   `vehicle_local_position`/`vehicle_command` — nothing in it requires
   `mc_pos_control` to be running; it selects a `FlightTask` from
   `nav_state` alone and is otherwise standalone. **Fix chosen: start
   `flight_mode_manager`** in `4026_gz_foldrotor3` (not gating `Run()` on
   `flag_control_*`, the alternative) — smaller diff, and it doesn't
   reverse decision 6/item 8 below (this module still has no flight-mode
   concept of its own; it just now has a legitimate setpoint to consume
   instead of nothing).

   **One caveat, load-bearing, not silently relied on:**
   `flight_mode_manager` also subscribes to `takeoff_status`, published
   *only* by `mc_pos_control` (checked: `grep` for
   `ORB_ID(takeoff_status)` publishers finds no other module). Without
   it, `_takeoff_state` never advances past `TAKEOFF_STATE_UNINITIALIZED`
   (`FlightModeManager.cpp:357`), so the active flight task is
   `reActivate()`-ed (setpoint snapshotted to current position/velocity)
   every single cycle rather than ever tracking a moving command. Verified
   in SITL: `listener trajectory_setpoint` showed `position: [-6.14,
   -10.78, 1.78]` matching `listener vehicle_local_position`'s `x: -6.12,
   y: -9.97, z: 1.44` at the same instant — a real, continuously-updated
   hold-in-place setpoint, not a coincidence. This is **sufficient for
   Part D's bench sequence** (hold current position while armed) but this
   module cannot honor any actual translation command until something
   publishes `takeoff_status`, which nothing does today — recorded here,
   not treated as solved.

   **Verification result, also recorded honestly: fixing the setpoint
   gap alone did NOT stop the arm-time attitude failsafe.** Re-running the
   same arm/disarm SITL check with `flight_mode_manager` running (even
   arming within ~2s of boot, minimizing any pre-arm settling time) still
   produced `_F_b` spiking to ~30N+ and an immediate "Attitude failure
   (roll)" failsafe disarm. Traced (via `listener vehicle_local_position`/
   `trajectory_setpoint`) to two *different*, already-documented,
   already-deferred causes, not this gap: (1) `FR_VEL_Z_I`'s integrator
   (anti-windup bounds still ±infinity, this doc's own "inert at runtime"
   note above) accumulates every cycle regardless of arm state and was
   already near 12N by arm time, pushing `_F_b.z` to the thrust ceiling
   the instant real thrust turns on; (2) `allocation.md`'s already-flagged
   `M0` geometry mismatch (s1y 0.15 vs the SDF's real 0.2684, "roll
   response ≈1.8× commanded... first suspect if roll misbehaves") — which
   is exactly the failure mode observed. This is the first time the full
   cascade has ever reached real Gazebo physics (previously blocked by
   O-4's publisher race), so this is a first discovery for Part D, not a
   regression introduced by the setpoint fix. Neither cause is touched
   here — both are pre-existing, explicitly out-of-scope for a
   startup/lifecycle diff. See `findings.md`'s 2026-09-09 entry.
7. **EKF reset counters unhandled.** `Run()` reads
   `vehicle_local_position` but does not diff `xy_reset_counter` /
   `z_reset_counter` / `vxy_reset_counter` / `vz_reset_counter` /
   `heading_reset_counter` against the previous cycle the way
   `mc_pos_control`'s `adjustSetpointForEKFResets` does. Harmless today
   (nothing reaches actuators), but needs a decision before 4d/4e's
   actuator publish lands.
8. **Integrator reset covers disarm only, not "mode entry."** This
   module has no PX4 flight-mode concept of its own — it always runs the
   same cascade — so there is no clean equivalent to implement. Left
   open rather than guessed at.

## Verification
- Given identical inputs, the PX4 module's output must match the
  Simulink reference output within a defined tolerance (numerical
  comparison test, not closed-loop)
- This is a software-correctness check only — it does not validate
  closed-loop behavior

## Not specified here
Gain values (config, not contract). Internal control law derivation
(see Simulink model / thesis notes).
