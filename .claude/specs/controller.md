# Controller Specification

## Status
**Not yet implemented in PX4.** As of the 2026-08-27 audit, no
standalone foldrotor controller module exists in `src/`. This spec is
the contract to build against, not a description of existing code. The
reference implementation is the Simulink model.
(`src/modules/mc_raptor` exists in this tree but is an unrelated
RL-policy flight-mode module — don't mistake it for this target.)

## Scope
Position + attitude control law. Algorithm reference: the validated
Simulink model is the source of truth for the math. This spec covers the
PX4 module's I/O contract, not a re-derivation of the control law.

## Interface
**In:** state from estimator — position (inertial, confirmed),
velocity (frame open — see Open questions), attitude (Euler angles
phi/theta/psi, confirmed representation), angular rate (frame/units:
see system.md contract table)
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
2. Yaw rate loop has zero I/D gain — confirm deliberate (consistent
   with the small k=0.017 drag-coupling term) vs. unfinished tuning.

## Verification
- Given identical inputs, the PX4 module's output must match the
  Simulink reference output within a defined tolerance (numerical
  comparison test, not closed-loop)
- This is a software-correctness check only — it does not validate
  closed-loop behavior

## Not specified here
Gain values (config, not contract). Internal control law derivation
(see Simulink model / thesis notes).
