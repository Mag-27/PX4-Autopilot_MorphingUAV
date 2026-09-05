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

## 2026-09-05 — Arm axis convention: realigned to the MATLAB lateral layout

Reported symptom: arm1/arm2 looked like they sat on the wrong sides in
the gz GUI after the `base_link`/`airframe_link` rotation fix.

**The reported symptom was not the bug.** Resolving the full SDF chain
into `base_link`, and independently reading STL vertex extents, both put
the arms on ±X — self-consistent, and not the ±Y that was read off the
GUI. `airframe_link_joint` was a *pure roll about X*, which cannot move
the X axis at all, so the suspected cause was impossible. The GUI reading
was almost certainly a link-local axis triad (one is drawn at every
joint) rather than the world triad.

**The real defect was the layout itself:** the CAD lays the arms fore/aft
along its own X, but the vehicle is a lateral side-by-side rotor pair.
Fixed by adding a **−90° yaw** to `airframe_link_joint` (now `+90° roll,
−90° yaw`). Yaw is about the already-upright vertical axis, so it cannot
disturb the roll fix. Resulting geometry, PX4 body FRD:

| | x | y | z |
|---|---|---|---|
| Arm1TiltLink / Prop1 | ~0 | **+0.2318 / +0.2684** | −0.0194 / +0.0301 |
| Arm2TiltLink / Prop2 | ~0 | **−0.2348 / −0.2684** | −0.0194 / +0.0301 |

Sign is FRD (Y right). The SDF is authored in gz FLU (Y left), 180° away;
choosing the wrong one silently inverts roll, so both the test and the
model.sdf comment state the frame explicitly.

**Folded.** → `system.md` Actuators→Gazebo row, Frame cell (was TBD).

Guarded by `foldrotor3_tests/test_frame_convention.py` — pure SDF/STL, no
Gazebo, no PX4 build (~0.15 s), asserting in FRD. Covers arm sides,
mesh/joint-origin agreement, prop mirroring, and uprightness. Confirmed
falsifiable against a mutated model. Lives in the parent repo, not the
`Tools/simulation/gz` submodule, since that submodule tracks upstream
`PX4/PX4-gazebo-models`.

**Still open — MATLAB vs Gazebo rotor geometry disagrees in magnitude.**
The axis question above is now resolved (both put rotor 1 on +Y FRD), but:

| | MATLAB (per `allocation.md` s1/s2) | Gazebo (measured) | gap |
|---|---|---|---|
| moment arm | 0.15 (`d+l_arm`) | 0.2684 | **1.79×** |
| vertical offset | `h`=0.02 | 0.0301 | **1.51×** |

`M0`/`Minv` are hand-typed literals not derived from `d`/`l_arm`/`h`
(`allocation.md`), so nothing catches this today. Not decided: which is
authoritative. `Control_Alloc.m` is **not in this repo**, so the MATLAB
column is as recorded in `allocation.md` and is itself unverified against
the Simulink model — check it before acting on these numbers.

---

## 2026-08-28 — Open-loop actuator test, all 6 channels (SITL, foldrotor3)

**Folded.** → `system.md` Milestone 1 checklist (now checked). Both
servos (`-s 1..4`) and motors (`-m 1`, `-m 2`) confirmed against
`4026_gz_foldrotor3` itself, not the bench fixture. Note: this only
confirms each channel *moves* — sign/magnitude correctness is the next
unchecked item (force/moment direction test).

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
