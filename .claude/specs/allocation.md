# Control Allocation Specification

## Status
**Not yet implemented in PX4.** No standalone allocator module exists in
`src/` (confirmed 2026-08-27; `grep -rli foldrotor src/` returns nothing).
`src/modules/mc_raptor` exists in this tree but is an unrelated RL-policy
flight-mode module — don't mistake it for this target. Everything
committed for foldrotor3 so far is simulation-side only:
`Tools/simulation/gz/models/foldrotor3` (Gazebo model) and its
`foldrotor3_bench` bench-test fixture variant, plus the
`4026_gz_foldrotor3` PX4 airframe.

The current airframe uses `CA_AIRFRAME 0` (PX4's generic/stock multirotor
allocator), not this custom allocator — an explicit, self-documented
open-loop first pass ("Open-loop first pass: default/generic control
allocation only, no custom ActuatorEffectivenessFoldrotor. Actuators are
driven directly via actuator_test, not through the control allocator's
mixer output.") This is consistent with, not a violation of, this spec's
"not to be replaced by PX4's default allocator" requirement, which applies
once the custom module exists.

**Actuator surface reachability (resolved 2026-08-28, PR #3):** the four
fold/tilt servo joints (`servo_0..3`) had no PX4 actuator function
assigned (`SIM_GZ_SV_FUNC*` unset), so `GZMixingInterfaceServo` wasn't
publishing to them — blocking 4 of the 6 DOF this allocator will need to
drive. `SIM_GZ_SV_FUNC1..4` are now set (201-204); see `system.md`
Milestone 1 checklist.

## Scope
Custom allocation, first-class subsystem. Not to be replaced by PX4's
default allocator.

## Interface
**In:** desired wrench (force, moment) from controller — **must be body
frame; currently is not** for the force components (see controller.md:
confirmed inertial→body rotation is missing on the velocity-loop
output; fix specified there). Moment components are already body-frame.
**Out:** actuator_motors, actuator_servos commands

## Actuator naming and tilt-limit convention (resolved 2026-09-06)

Two gaps flagged in `findings.md` (2026-09-06, "Tilt/fold actuator limit
conflict, and α/β→joint mapping is undocumented") are now settled by
user decision — both are control-design calls, not derivable from SDF or
code alone.

**α/β ↔ physical joint identity — by design in the vehicle dynamics:**
- α1, α2 = `Arm1FoldJoint`, `Arm2FoldJoint` angle
- β1, β2 = `Arm1TiltJoint`, `Arm2TiltJoint` angle

Combined with the 2026-09-06 SDF-geometry findings (unchanged, still
correct — they described what each joint does, not what MATLAB calls it):
α (fold) tilts thrust into body ±Y (lateral); β (tilt) tilts thrust into
body ±X (longitudinal). Arms are angle-sign-symmetric, not mirrored:
equal positive α on both arms tilts both thrust vectors the same
direction in body frame, not a differential — any allocator logic
assuming β₂ = −β₁-style mirroring must be checked against this.

**Tilt limit — clamped to the physical range, ±45.26° (±0.79 rad):**
the SDF joint limit and `SIM_GZ_SV_MINA/MAXA` are authoritative; the
allocator's `max_tilt` is decided to be **0.79 rad, not 1.0472 rad**.
The original ±60° figure was the allocator's design intent, not
something the plant can reach — see `findings.md`'s PX4 servo-mixing
trace for how a command beyond the physical limit gets silently
saturated with no log, which is what makes this a "must fix before
building," not a cosmetic mismatch. This reduces max thrust-vectoring
authority versus the original 60° design; no SDF/hardware change is
made or implied by this decision.

## Known requirements
- Allocation matrix: constant, frozen at tilt angles α1 = α2 = 0,
  confirmed current — `Control_Alloc.m` and the forward `fcn` MATLAB
  Function block implement this directly
- **Actuator limits (updated 2026-09-06):** F1, F2 ∈ [0, 15] N; α1, β1,
  α2, β2 ∈ **[-45.26°, 45.26°] (±0.79 rad)** — was ±60° (±1.0472 rad);
  clamped to the physical/SDF/servo range per the decision above.
  Enforced by explicit clamping in `Control_Alloc`; `max_tilt` must be
  updated there when `Control_Alloc.m` is brought into this repo or
  reimplemented for PX4.
- Reactive drag torque is modeled as ∓k·(full rotor thrust vector), not
  just its nominal spin-axis (z) component — M0's moment rows apply k
  to each rotor's own-axis thrust component (e.g. k·Tx1 contributes to
  Mx). This is a simplification worth stating explicitly: confirm it
  matches how Gazebo's rotor plugin computes drag torque, or document
  the expected discrepancy. This is exactly the kind of assumption that
  lets controller and physics each be individually "correct" while
  disagreeing at the boundary. **Known current gap:** `model.sdf`'s
  drag/rolling-moment/spin-up-lag terms are x500-derived generic
  placeholders, not measured for this vehicle, so this cross-check can't
  be done precisely yet — matters at validation, not now.
- Singularity handling is currently "avoid it by construction" (fixed
  α=β=0, so M0 stays full rank by design), not "detect and regularize
  at runtime" — Minv is a precomputed literal, not an on-line
  regularized inverse

## Actuator geometry / effectiveness matrix
Confirmed current (from `Control_Alloc.m`, replaces the erased earlier
constants):
- d = 0.05 m (base distance), l_arm = 0.10 m (arm length),
  h = 0.02 m (vertical CG offset), k = 0.017 (drag/thrust ratio)
- s1y = +(d+l_arm) = 0.15, s1z = h = 0.02 (rotor 1);
  s2y = -(d+l_arm) = -0.15, s2z = h = 0.02 (rotor 2)
- M0 (6×6, constant, evaluated at α=β=0) and its inverse are both
  hand-typed literals in `Control_Alloc.m` — **not derived at runtime
  from d/l_arm/h/k**. If any geometry constant changes, Minv must be
  regenerated by hand; nothing currently checks that Minv still
  matches M0.

## Verification
- Saturation test: allocator never outputs commands outside actuator
  limits, including near-singular inputs
- Direction/sign test: known desired wrench produces actuator commands
  with the expected sign, cross-checked against the force/moment
  direction test in system.md
- **Minv·M0 ≈ I test** (currently missing): catches Minv going stale
  relative to M0 after a geometry-constant edit. Cheap, high-value.
- **Round-trip test** (currently missing): `Control_Alloc` output fed
  into `fcn` should reproduce the original desired wrench within
  tolerance, for wrenches inside the unsaturated envelope. Checked
  algebraically here — the atan2 inverse and sin/cos forward mapping
  are consistent with each other — but nothing encodes this as a test.
- **New, from the 2026-09-06 limit decision:** a test asserting the
  allocator's own `max_tilt`/clamping constant equals 0.79 rad, not
  1.0472 rad — catches this regressing if `Control_Alloc.m` is
  re-imported from an older MATLAB revision.

`open_loop_commands.md` (repo root) has the actuator_test channel map and
reference commands for exercising all 6 channels via SITL — useful for
the saturation/direction tests above once the allocator itself exists to
compare against.

## Not specified here
Actuator failure response, hardware-specific calibration (deferred —
SITL only for now).
