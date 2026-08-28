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

**Folded.** → `allocation.md` (Status), `system.md` (Milestone 1
checklist, Actuators→Gazebo row), `controller.md` (Status, mc_raptor
note). Servo-wiring blocker independently resolved 2026-08-28 (PR #3);
`system.md` updated to match. Detail in git history / PR #2.

Estimator→Controller and Controller→Allocation contract rows remain
not-verifiable until a controller module exists — tracked in `system.md`
directly (Milestone 1 note), not duplicated here.

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
