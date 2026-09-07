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

## 2026-09-07 — Step 4e part 1 wiring decisions (cascade into Run(), no actuator publish)

**Folded.** → `controller.md` ("Run() wiring (decided 2026-09-07, step
4e part 1)" under Structure), `AttitudeRateControl.hpp` (interface-split
doc comment), `FoldrotorControl.hpp/.cpp` (member comments). All six
were user decisions made while wiring PositionVelocityControl (4a),
Inertial2Body (4b), and AttitudeRateControl (4c) into `Run()`; none is
recoverable from controller.md's existing prose.

Worth keeping visible rather than pruning to a bare pointer, because a
future reader could otherwise assume the cadence number came from the
Simulink reference:

1. **Multi-rate cascade cadence — 50 Hz position/velocity, 250 Hz
   attitude, 1000 Hz rate — is NOT in controller.md.** Checked by grep
   across controller.md, controller_params.md, and
   reference/px4-module-patterns.md before implementing; nothing there
   specifies cascade timing, only the stage order. This is a step 4e
   decision, recorded here and now in controller.md, not a spec
   transcription.
2. **`AttitudeRateControl`'s interface was split** into
   `updateAttitude()` (250 Hz) and `updateRate()` (1000 Hz) because
   `update()` (step 4c) always recomputes `rate_sp` from the current
   Euler error in one call — there is no way to run its attitude stage
   slower than its rate stage without this split. `update()` itself is
   unchanged in behaviour (calls both back to back), so step 4c's 16
   hand-computed tests still pass as regression tests of the split.
3. **`euler_sp`: phi_sp/theta_sp pinned to zero**, psi_sp from
   `trajectory_setpoint.yaw` (held at current heading if NaN) —
   consistent with controller.md's own explanation that this vehicle
   translates by thrust vectoring, not body lean.
4. **Validity/NaN gating**: full mc_pos_control-style — estimator
   `_valid` flags plus `PX4_ISFINITE` on the setpoint, holding the
   previous wrench and resetting the integrator on the invalid->valid
   transition. Generates one new open item: `PositionVelocityControl`
   has no independent velocity-setpoint path (it derives `vel_sp` from
   `pos_sp` only), so a NaN position with a live velocity setpoint can't
   be honored without extending that class — not done here, since that
   would be a new decision about 4a's class, not this diff's wiring.
5. **EKF reset counters (`xy_reset_counter` etc.)**: recorded as an
   explicit open item, not implemented. Nothing is published to
   actuators yet, so no reset-driven transient can reach the vehicle
   from this diff.
6. **Integrator reset on disarm only.** "Reset on disarm and on mode
   entry" (the original ask) has no clean "mode entry" equivalent: this
   module has no PX4 flight-mode concept of its own — it always runs the
   same cascade regardless of what flight mode is active. Only the
   disarm edge (`flag_armed` true->false) is implemented; the gap is
   recorded in controller.md rather than guessed at.

Verified in SITL (2026-09-07, `px4_sitl_foldrotor` board config, SIH
quadx substrate — the module has no dedicated Gazebo model wired into
its own airframe file yet, so SIH stands in for "real sensor data
flowing"): `foldrotor_control start`/`status`/`stop` all work; `status`
printed a live wrench (`F_b`, `M_b`); `mc_pos_control`/`mc_att_control`/
`mc_rate_control` booted normally alongside it (rc.mc_defaults, not
disabled); no ERR lines in the boot log. `F_b`'s Z component started
near +9.81 N (the literal gravity feedforward) and climbed toward
~10.9 N over several seconds — exactly the FR_VEL_Z_I=7 integrator
winding up toward the known ~15.26 N hover weight gap that
`PositionVelocityControl.hpp`'s OPEN ITEM (a) already predicted, with
the anti-windup confirmed inert (bounds still +/-infinity) exactly as
documented. This is the wiring behaving as specified, not a new finding.

---

## 2026-09-07 — Attitude/rate-loop semantics (step 4c)

**Folded.** → `controller.md` (new "Attitude/rate-loop form" subsection
under Structure; Open questions 1 and 2 and the red-highlighted-ports
bullet all updated with 4c status) and `controller_params.md`
(FR_RATE_*_FF documented as the P gain, plus a new recorded gap: no
`FR_RATE_*_I_LIM` param exists to drive the integrator clamp). Detail in
git history.

One item is worth keeping visible rather than pruning entirely, because
it is a decision made *against* the available precedent and a future
reader will otherwise assume it was made in ignorance of it:
`FR_RATE_*_FF` is read as the P gain even though PX4's own
`RateControl::update()` (`rate_control.cpp:78`) has a distinct
`_gain_ff.emult(rate_sp)` setpoint-feedforward term separate from P. The
Simulink rate-loop diagram would settle it; until then the FF-as-P
reading stands, and yaw is consequently a pure proportional law.

---

## 2026-09-07 — Correction: step 4a's anti-windup does NOT match mc_pos_control on X/Y

**Folded.** → `PositionVelocityControl.hpp` (decision 3 comment),
`controller.md` ("Velocity-loop form", item 3), `controller_params.md`
(wording). Pruned per the rules below, but the substance is worth one
extra line because it is a repeat-offender failure mode:

Step 4a's fold-in claimed its conditional-integration anti-windup
"matches `mc_pos_control` (`PositionControl.cpp:158-160`)". **That is
true for Z only.** Re-read from the file, not memory:
- `:158-160` — commented "Integrator anti-windup in vertical direction",
  gates on `_thr_sp(2)` / `vel_error(2)`. Conditional integration, Z only.
- `:188-198` — X/Y use **tracking anti-windup** (Rundqwist 1990):
  `arw_gain = 2/_gain_vel_p(0)`, and `vel_error.xy() -= arw_gain *
  (acc_sp_xy - acc_sp_xy_produced)`. Feeds achievable-vs-desired
  acceleration back into the error; not a saturation-direction freeze.
- `:146` — `mc_pos_control` also hard-clamps its Z integral to ±g, which
  `PositionVelocityControl` does not.

`foldrotor_control` applying uniform conditional integration to all three
axes is fine and stays as implemented — **the code is unchanged; only the
claim was wrong.** Switching X/Y to real tracking ARW would be a design
decision (it needs an achievable-output estimate the module does not have
until 4d), and is explicitly NOT being made here.

The 4a D-term citation (`:150`, derivative on measurement, negated) was
checked at the same time and **is** correct for all three axes; it stands.

**The lesson, since this is the second time:** the 4a work asserted a
precedent match for one line range while having only verified the
behaviour of another. Cite a file range only for the axes/cases actually
read. This is the same discipline the 2026-09-05 entry records for
geometry claims — see the `feedback_geometry_claims` memory.

---

## 2026-09-07 — Velocity-loop semantics resolved by user decision (step 4a)

**Folded.** → `controller.md` (new "Velocity-loop form" subsection under
Structure; new Open questions 3–5 for the gravity-feedforward units,
sign, and the still-unresolved Z summing junction) and
`controller_params.md` (FR_VEL_*_FF documented as the P gain; open item 1
extended). Detail in git history.

---

## 2026-09-07 — controller.md's Rt verified against PX4's own DCM convention (step 4b)

**Folded.** → `controller.md` ("Identified issues (confirmed) → 1",
new Status and convention-confirmation paragraphs).

Checked, rather than assumed, that the `Rt` matrix `controller.md` gives
for `Inertial2Body` really is the "standard ZYX" inertial→body rotation
it claims to be: it equals `matrix::Dcmf(euler).transpose()`
element-for-element, PX4's `Dcm(const Euler&)` being the standard 3-2-1
intrinsic Tait-Bryan body→inertial DCM (`Dcm.hpp:121-142`). **The spec's
claim holds — no correction needed.** Recorded because the check is what
licenses composing this stage directly with step 3's `_euler`, and
because a silent convention mismatch here would have surfaced only as a
sign error during yawed translation, which is the hardest class of bug
to attribute (system.md's whole premise).

Resolves no contract-table row on its own: the Controller→Allocation row
stays open, since the force components are still inertial at runtime —
the rotation exists as tested pure math but nothing calls it until 4e.

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
