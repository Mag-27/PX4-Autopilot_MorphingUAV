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

## 2026-09-07 — foldrotor_control module skeleton audit (build wiring, uORB interfaces, spec drift)

**Folded.** → `controller.md` (Status, Interface — quaternion not Euler,
conversion required), `system.md` (Estimator→Controller Frame cell:
velocity/rate confirmed, attitude corrected to quaternion; "Module scope
(decided)": stock stack still auto-started via `rc.mc_apps`, must stop
before step 5), `controller_params.md` (quaternion→Euler conversion test
added to step 3 verification). Board-config precedent gap resolved by
reverting `default.px4board` and adding standalone
`boards/px4/sitl/foldrotor.px4board`. Detail in git history.

---

## 2026-09-06 — Tilt/fold actuator limit conflict, and α/β→joint mapping is undocumented

**Folded.** → `allocation.md` ("Actuator naming and tilt-limit
convention", Known requirements) and `system.md` (Allocation→Actuators
row, Sign convention and Valid range cells). Both gaps resolved by user
decision 2026-09-06: tilt limit clamped to ±0.79 rad (±45.26°, the
physical/SDF/servo range, not the allocator's original ±60° design
intent); α1/α2 = Arm1/Arm2 **fold** angle, β1/β2 = Arm1/Arm2 **tilt**
angle, by design in the vehicle dynamics. Detail (SDF pose-chain
derivation, PX4 silent-saturation code path, arm angle-sign-symmetry
trap) in git history.

---

## 2026-09-06 — Force/moment direction test PASSED; found and fixed a rotor thrust-axis defect

Ran the force/moment direction test end to end on the bench fixture. All 6
actuator channels plus both combined cases now match an SDF-derived
expectation in sign, with lift to 0.1%, roll 0.6%, yaw 0.2%.

**Folded.** → `system.md` Actuators→Gazebo row (Sign convention, was TBD) and
Milestone 1 checklist (now checked); full comparison table and methodology
corrections in `.claude/specs/force_moment_test.md`.

**The substantive finding: the rotors produced zero net thrust.**
`gz-sim-multicopter-motor-model-system` applies thrust along the rotor
*link's* local Z, not the joint axis. foldrotor3's CAD export is Y-up, so
`PropNJoint` carried `<xyz>0 1 0</xyz>` and each rotor link's local Z lay
perpendicular to its own spin axis — the thrust vector rotated with the
propeller and averaged to nothing (measured net lift 0.03 N against a
15.26 N airframe weight, with ±10.4 N swinging around the XY plane). Closed-
loop hover could never have worked, and would have presented as a controller
bug. This is the failure the verification-before-validation order exists to
catch.

Fixed by rotating the `PropNJoint` frames −90° about X so local +Z is the
spin axis, with visual/collision/inertial poses counter-rotated so nothing
physical moved — a change of frame, not of geometry. Verified: at-rest bench
reading identical before and after (15.2600 N), and lift went from 0.03 N to
−10.07 N against a −10.08 N prediction.

**A sign trap worth remembering** (cost me one wrong analysis): thrust does
*not* depend on `turningDirection`. The plugin's `td * sign(v)` cancels
because the joint velocity is itself commanded as `td * refRotVel / slowdown`.
`turningDirection` sets only the drag-torque sign. Reasoning about thrust
direction from it leads to the wrong fix. Detail in
`reference/gz-rotor-and-sensor-conventions.md`.

**Two fixture defects found and fixed**, both of which silently corrupted
results before being caught:
- `bench_mount_joint`'s parent was `world`. gz-sim's ForceTorque system
  resolves endpoints with `GetLinkFromScopedName()`, which only matches
  `Link` entities, so it skipped the sensor and advertised no topic at all.
  Fixed with a massless `mount_plate` link between two fixed joints.
- The stand sat at 0.1 m with 30 mm clearance; `-s 1 -v 0.5` drove
  `Arm1TiltLink` 18 mm through the ground plane, moving ~9.5 N off the mount
  and producing a plausible but spurious wrench. Raised to 0.5 m, guarded by
  `foldrotor3_tests/test_bench_clearance.py`.

Also: PX4's `server.config` does not load `gz-sim-forcetorque-system`, so a
`<sensor>` element alone is inert. Declared at model scope in the bench SDF
to keep the change out of PX4; this works reliably (apparent flakiness during
testing traced to stale `gz sim` servers left running between launches, not
the plugin placement).

**Still open — unexplained residual:** a single motor shows ΔFy ≈ ∓0.5 N
(~5% of thrust) that the expectation puts at zero. Opposite sign per motor,
so it cancels in the pair. Suspected tilt-joint deflection under thrust load
(the fold joints demonstrably sag ~0.03 rad), but **not verified**. Affects
no sign; recorded, not chased.

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
