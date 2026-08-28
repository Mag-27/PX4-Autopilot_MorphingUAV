# Findings Log

Dated exploration notes about THIS codebase. Transient by design.

Two rules keep this file small:
- Folding a finding into system.md/controller.md/allocation.md is a
  separate, deliberate decision — not automatic.
- Once folded, prune the entry to a one-line pointer naming where it
  landed. Detail stays in git history.

Durable reference knowledge (how PX4 itself works) does NOT belong
here — see `reference/`.

---

## 2026-08-27 — foldrotor3 module and actuator/state interface audit

**Promoted.** Servo wiring gap and actuator surface reachability →
`allocation.md` (Status/Blocker), `system.md` (Milestone 1 precondition,
Actuators→Gazebo row). Specs-ahead-of-implementation status →
`controller.md`, `allocation.md` (Status sections).

Still open, not yet folded into any spec:
- No custom controller/allocation module exists in `src/` — foldrotor3
  is simulation-side only (model.sdf, bench fixture, airframe 4026).
- `src/modules/mc_raptor` is an unrelated RL flight-mode module in this
  tree. Not the target module; don't mistake it for one.
- Dangling reference: `model.sdf` servo-gain comment cites
  `open_loop_commands.md` for test data. No such file in the repo —
  the servo gains (p=20, i=0, d=0.5) currently have no provenance.
- SDF drag / rolling-moment / spin-up-lag terms are x500-derived
  generic placeholders, not measured for this vehicle. Known
  model-fidelity gap; matters at validation, not now.
- Estimator→Controller and Controller→Allocation contract rows are not
  verifiable until a controller module exists. Do not fill them from
  the Simulink model.

---

## 2026-08-27 — Stock mc_control stack as a structural reference

**Relocated** to `reference/px4-module-patterns.md`. This was durable
PX4 reference knowledge, not a finding about this codebase — it resolves
no contract row and does not go stale.

One item bears on an open architectural question, recorded here because
it's a decision input rather than reference material: reusing PX4's
`control_allocator` would bind the module to a normalized [-1,1]
unitless contract on `vehicle_torque_setpoint`/`vehicle_thrust_setpoint`,
plus a publish-ordering dependency (torque triggers the allocator; thrust
is read opportunistically, so torque must be published last). Relevant to
whether the custom allocator interfaces with `control_allocator` or
bypasses it — still undecided.
