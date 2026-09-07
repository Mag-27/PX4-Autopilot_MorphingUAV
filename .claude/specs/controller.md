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
3. **Anti-windup is conditional integration on all three axes**,
   freezing an axis' integrator when its output is saturated in the
   direction that would worsen it. **This is `mc_pos_control`'s vertical
   algorithm generalized to x and y, not a match to its horizontal
   one** — `PositionControl.cpp:158-160` is conditional integration for
   Z only (its own comment says "in vertical direction"), while X/Y use
   tracking anti-windup (Rundqwist 1990, `:188-198`), feeding the
   desired-minus-achievable acceleration back into the error at a gain
   of 2/P. `mc_pos_control` additionally clamps its Z integral to ±g
   (`:146`); this module does not. The uniform choice here is
   defensible — simple, symmetric, and it needs no achievable-output
   estimate — but it is a choice, and adopting tracking ARW on X/Y
   would be a design decision that has not been made. Simulink has no
   anti-windup at all, so this remains a deliberate divergence from the
   reference either way. **It is inert at runtime today:** the
   saturation bounds are a property of what the allocator can produce,
   which is step 4d, so `PositionVelocityControl` takes them as
   caller-supplied and defaults them to +/-infinity. Until 4d/4e pass
   real bounds, the `FR_VEL_Z_I = 7` integrator is unbounded.
4. **Output stays a force in newtons with `FR_VEL_Z_GRAV_FF` entering
   as a literal `+9.81` on the NED Z axis**, exactly as
   `controller_params.md` records. Two problems with that value are
   knowingly carried rather than silently corrected — see Open
   questions 3 and 4 below.

**Status (step 4a, 2026-09-07): implemented as pure math, not wired in.**
`src/modules/foldrotor_control/PositionVelocityControl.hpp` provides
`foldrotor::PositionVelocityControl`, whose `update()` returns the
inertial/NED desired force as a `matrix::Vector3f` so it composes
directly with step 4b's `foldrotor::inertialToBody(F_i, euler)`. Nothing
calls it — wiring is 4e — so this changes no runtime behaviour yet, and
issue 1 below remains live. Tested in `FoldrotorControlTest.cpp`
(`FoldrotorPositionVelocityControlTest`, 12 tests) against hand-computed
values at the spec gains: the nominal X/Y cascade, per-axis D gains,
X/Y and Z integral accumulation, integrator reset, the default
unbounded behaviour, and conditional integration on both X and Z
including the directional case where an error pulling *out* of
saturation must still integrate.


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

**Status (step 4b, 2026-09-07): the rotation stage exists as pure math.
As of step 4e part 1, it IS called every 50 Hz position/velocity cycle**
(`Run()` computes `F_i` via `PositionVelocityControl::update()`, then
`_F_b = foldrotor::inertialToBody(F_i, _euler)`) — the fix is applied.
`src/modules/foldrotor_control/Inertial2Body.hpp`
provides `foldrotor::inertialToBodyRotation(euler)` (the `Rt` above,
transcribed literally) and `foldrotor::inertialToBody(F_i, euler)`.
**The bug this fixes has not reached the vehicle either way**, before or
after this wiring: nothing has ever been published to actuators, and
still isn't as of 4e part 1 — allocation (4d) and the actuator publish
are separate, later diffs. Tested in `FoldrotorControlTest.cpp`
(`FoldrotorControlInertial2BodyTest`, 6 tests): identity at level
attitude, proper rotation (`Rt^T·Rt = I` *and* `det = +1`, the latter
ruling out a reflection that orthonormality alone would admit), and
hand-computed pure-yaw/pure-roll cases.

**The "standard ZYX" claim above is confirmed, not assumed (2026-09-07).**
`Rt` as written here is exactly `matrix::Dcmf(euler).transpose()`: PX4's
`Dcm(const Euler&)` constructor (`src/lib/matrix/matrix/Dcm.hpp:121-142`)
builds the standard 3-2-1 intrinsic Tait-Bryan body→inertial DCM, and
transposing it reproduces the nine expressions above element-for-element.
This matters beyond pedantry — it is what makes this stage compose with
step 3's `_euler = matrix::Eulerf(matrix::Quatf(q))` with no convention
adaptation, since both are then the same ZYX convention. Asserted across
six attitudes by `MatchesPx4DcmTransposeAcrossAttitudes` rather than left
as a comment.

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
3. **`FR_VEL_Z_GRAV_FF` units.** 9.81 is an *acceleration*, but the
   velocity loop's output is a force in newtons per `allocation.md`.
   This airframe measures 15.26 N (findings.md, 2026-09-06), i.e.
   m ≈ 1.556 kg, so a force-domain gravity term would be ≈15.26 N. As
   implemented, hover leans on the `FR_VEL_Z_I = 7` integrator to make
   up the remaining ≈5.4 N. Either the Simulink loop is really in
   acceleration units with the mass folded in downstream, or the
   feedforward is under-scaled. Not resolved.
4. **`FR_VEL_Z_GRAV_FF` sign.** Position and velocity are NED
   (confirmed, Interface above), so gravity is +Z and a hover force must
   be *negative* Z. A literal `+9.81` on `Fz_i` points *down*. This
   reads like the Simulink model was authored Z-up, but that is not
   confirmed. Implemented as recorded rather than inverted on a guess —
   a silent Z inversion here is the same class of error that the
   2026-09-05 arm-axis and 2026-09-06 rotor-thrust findings were caught
   by, and it would present as a controller bug.
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
