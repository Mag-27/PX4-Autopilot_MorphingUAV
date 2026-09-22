# Control Allocation Specification

## Status

**Implemented (step 4d/4e part 2, this session)** as
`src/modules/foldrotor_control/FoldrotorAllocation.hpp`, a stateless
pure-math class wired into `FoldrotorControl::Run()`, publishing
`actuator_motors`/`actuator_servos` every cycle. `Minv` is derived from
`M0` at construction (`matrix::inv<float,6>()`), not hardcoded — see "The
matrices" below. Fold (α) is pinned to 0 in this diff's output (no
lateral thrust-vectoring authority yet — see the plan's open item O-3).
Full detail, including everything NOT yet resolved by this
implementation, is in `.claude/plans/step-4e-allocation-plan.md`.

**Publisher race resolved (2026-09-09, plan open item O-4):**
`4026_gz_foldrotor3` no longer sets `VEHICLE_TYPE mc` (overridden back
to `none` right after sourcing `rc.mc_defaults`, so `rc.vehicle_setup`
never sources `rc.mc_apps`), so `control_allocator` — along with
`mc_pos_control`/`mc_att_control`/`mc_rate_control` — is no longer
started for this airframe. `foldrotor_control` is now the sole publisher
of `actuator_motors`/`actuator_servos`; `land_detector` (the one
`rc.mc_apps` line commander/arming needs) is started explicitly in the
airframe file instead. This has still **not** been exercised against
Gazebo/hover (Part D of the allocation plan), only unit tested
(`FoldrotorAllocationTest`, `FoldrotorControlMappingTest`) — the Part D
bench run is the next step, now that only one publisher is live.

**Geometry mismatch resolved (2026-09-09):** `FoldrotorAllocation`'s
`kS1y`/`kS2y`/`kS1z`/`kS2z` now match the SDF-verified rotor geometry
(`s1y = +0.2684`, `s2y = -0.2684`, `s1z = s2z = +0.0301`) instead of the
old `d+l_arm = 0.15`/`h = 0.02` figures. `Minv` re-derives automatically
from the corrected `M0` at construction — no other code path changes.
Fixed now (not before) because the first real SITL exposure of this
cascade (`findings.md` 2026-09-09 (2)) showed the exact "roll misbehaves"
symptom this mismatch was already flagged as the first suspect for. See
"Actuator geometry / effectiveness matrix" below for the updated table
and matrices; the previously-deferred "Geometry-vs-SDF test" can now be
written as a green regression guard, though this diff doesn't add it.

**Actuator surface reachability (resolved 2026-08-28, PR #3):** the four
fold/tilt servo joints (`servo_0..3`) had no PX4 actuator function
assigned (`SIM_GZ_SV_FUNC*` unset), so `GZMixingInterfaceServo` wasn't
publishing to them — blocking 4 of the 6 DOF this allocator will need to
drive. `SIM_GZ_SV_FUNC1..4` are now set (201-204); see `system.md`
Milestone 1 checklist.

**Source-of-truth gap:** `Control_Alloc.m` is not in this repo. The
authoritative allocator source lives only in the Simulink model, outside
version control. The constants and matrices recorded below were
transcribed from it and are unverified against the live model. Bringing
`Control_Alloc.m` (or a generated equivalent) under version control would
close this.

## Scope

Custom allocation, first-class subsystem. Not to be replaced by PX4's
default allocator.

## Interface

**In:** desired wrench (force, moment) from controller — body frame,
**Control_Alloc.m's native FLU (Forward-Left-Up, Z-up) convention, not
PX4's FRD**. See "Frame convention" below for the resolution and where
the FRD→FLU conversion actually happens (it is not inside this
allocator).

**Out:** actuator_motors, actuator_servos commands

## Frame convention (resolved 2026-09-08)

**Root cause:** `Control_Alloc.m`'s math (transcribed below, and
matching the source thesis derivation, Progress_160726.pdf slides 7–10)
was written in body FLU (Forward-Left-Up, Z-up) — that's why its thrust
vector is `[F·sinβ; −F·cosβ·sinα; F·cosβ·cosα]`, positive along **+Z**.
`FoldrotorControl::Run()`'s cascade produces `_F_b`/`_M_b` in **PX4 body
FRD** (Forward-Right-Down, Z-down), per `controller.md`. These are not
the same frame — feeding FRD values directly into this FLU-derived math
misallocates any wrench with a nonzero Y or Z component, not just hover
(a Z-up-positive hover wrench allocates cleanly; the same hover
expressed in FRD, Fz negative, allocates to α ≈ ±180°, destroyed by the
±0.79 rad clamp).

**Fix:** a single coordinate transform, applied to `_F_b`/`_M_b` in
`FoldrotorControl.cpp` immediately before `_allocation.allocate()` is
called — **not** inside `FoldrotorAllocation` itself, and **not** a
rederivation of `M0`/`Minv`. FLU↔FRD is a 180° rotation about body X
(forward): X unchanged, Y and Z both negate.

```
F_alloc = (F_b.x, -F_b.y, -F_b.z)
M_alloc = (M_b.x, -M_b.y, -M_b.z)
```

**Source of truth:** these are the same coefficients as
`foldrotor3_tests/test_frame_convention.py`'s `FLU_TO_FRD =
np.diag([1.0, -1.0, -1.0])` — that file is the project's single
definition of this transform; `FoldrotorControl::frdToAllocatorFlu()` is
a transcription of it into C++, not an independent derivation. The
rotation is its own inverse (180°), so the same expression converts
either direction.

`FoldrotorAllocation`'s Interface contract is therefore: it expects
`F_b`/`M_b` already in body FLU, matching `Control_Alloc.m`'s own frame.
The allocator class does not know about FRD and never sees it — the
conversion is entirely the caller's responsibility. See
`FoldrotorAllocation.hpp`'s file-header OPEN ITEM (c) and
`FoldrotorControl.hpp`'s class comment for the code-level record.

## Actuator naming and tilt-limit convention (resolved 2026-09-06)

Two gaps flagged in `findings.md` (2026-09-06) are settled by user
decision — both are control-design calls, not derivable from SDF or code
alone.

**α/β ↔ physical joint identity:**
- α1, α2 = `Arm1FoldJoint`, `Arm2FoldJoint` angle
- β1, β2 = `Arm1TiltJoint`, `Arm2TiltJoint` angle

α (fold) tilts thrust into body ±Y (lateral); β (tilt) tilts thrust into
body ±X (longitudinal). Arms are angle-sign-symmetric, not mirrored:
equal positive α on both arms tilts both thrust vectors the same
direction in body frame, not a differential — any allocator logic
assuming β₂ = −β₁-style mirroring must be checked against this. (`M0`'s
force rows are common-mode sums, so it carries no mirroring assumption;
the differential moments come from the ±s_y arms.)

**α maps directly onto the SDF joint convention — RESOLVED 2026-09-10,
supersedes the negated mapping below.** `Control_Alloc`'s thrust vector is
`[F·sin β; −F·cos β·sin α; F·cos β·cos α]` — note the leading minus on
the y-component, so **positive α produces −Y thrust**. The
2026-09-06 entry below claimed a positive `ArmNFoldJoint` angle produces
+Y thrust "(SDF geometry, verified 2026-09-06)" and required negating the
mapping — that claim was never actually thrust-verified, only
kinematically reasoned from the SDF, and turned out to be **wrong**.

Bench-measured 2026-09-10 (fixed-mount rig, force/torque sensor, one
motor + its own fold servo, each arm in isolation): a positive
`ArmNFoldJoint` angle produces **−Y thrust for both Arm1 and Arm2**
(Arm1: Fy +0.50 N → −3.41 N at joint ≈ +0.345 rad commanded; Arm2: Fy
−0.50 N → −4.38 N at joint ≈ +0.450 rad commanded — see
`force_moment_bench_commands.md` for the procedure). That already
matches Control_Alloc's own `+α → −Ty` convention, so **no sign flip is
needed** — the mapping is direct:
- `β_n → ArmNTiltJoint`, direct
- `α_n → ArmNFoldJoint`, **direct** (not negated)

`FoldrotorControl::foldToNormalizedServo()` implemented the negated
mapping from 2026-09-06 through 2026-09-10 (dead code while α was pinned
to 0 — see `FoldrotorAllocation.hpp` OPEN ITEM (a) — so this was never
exercised against Gazebo before now); it has been corrected to the direct
mapping as part of unpinning α. Had the negated version shipped live, the
entire lateral axis would have been backwards.

<details>
<summary>Superseded 2026-09-06 entry (kept for history)</summary>

α carries a sign inversion against the SDF joint convention: positive α
produces −Y thrust, while a positive `ArmNFoldJoint` angle produces +Y
thrust (SDF geometry, verified 2026-09-06) — mapping was `α_n →
ArmNFoldJoint`, negated. **This SDF-geometry claim was disproven by the
2026-09-10 bench measurement above.**

</details>

**The α/β mapping is exact only at zero fold deflection.** `Control_Alloc`
decomposes thrust as a rotation about two *orthogonal, fixed* axes. The
SDF's joints are not that: fold is tilt's parent, so fold carries the tilt
axis with it. At zero the tilt axis is body −Y; at the fold limit
(0.79 rad) it has rotated to `[0, −0.7039, −0.7103]`, 45° out of the body
XY plane. Fold deflection also translates the rotor — at its limit, 82 mm
inboard and 99 mm down — which invalidates the constant moment arms as
well as the axis orthogonality. Treat the mapping as a small-angle
approximation valid near α ≈ 0, consistent with the frozen-matrix
assumption below.

**Tilt limit — clamped to the physical range, ±45.26° (±0.79 rad):**
the SDF joint limit and `SIM_GZ_SV_MINA/MAXA` are authoritative; the
allocator's `max_tilt` is decided to be **0.79 rad, not 1.0472 rad**.
The original ±60° figure was the allocator's design intent, not something
the plant can reach — see `findings.md`'s PX4 servo-mixing trace for how a
command beyond the physical limit gets silently saturated with no log,
which is what makes this a "must fix before building," not a cosmetic
mismatch. This reduces max thrust-vectoring authority versus the original
60° design; no SDF/hardware change is made or implied by this decision.
The limit applies to **all four** joints, fold included — not just the
tilt pair.

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
  just its nominal spin-axis (z) component — M0's moment rows apply k to
  each rotor's own-axis thrust component (e.g. k·Tx1 contributes to Mx).
  This is a simplification worth stating explicitly: confirm it matches
  how Gazebo's rotor plugin computes drag torque, or document the
  expected discrepancy. This is exactly the kind of assumption that lets
  controller and physics each be individually "correct" while disagreeing
  at the boundary. **Known current gap:** `model.sdf`'s
  drag/rolling-moment/spin-up-lag terms are x500-derived generic
  placeholders, not measured for this vehicle, so this cross-check can't
  be done precisely yet — matters at validation, not now.
- Singularity handling is currently "avoid it by construction" (fixed
  α=β=0, so M0 stays full rank by design), not "detect and regularize at
  runtime" — Minv is a precomputed literal, not an on-line regularized
  inverse

## Actuator geometry / effectiveness matrix

`Control_Alloc.m`'s original constants (not in repo — see Status), for
provenance only, **superseded below**:
- d = 0.05 m (base distance), l_arm = 0.10 m (arm length),
  h = 0.02 m (vertical CG offset), k = 0.017 (drag/thrust ratio)
- s1y = +(d+l_arm) = 0.15, s1z = h = 0.02 (rotor 1);
  s2y = -(d+l_arm) = -0.15, s2z = h = 0.02 (rotor 2)

### Geometry mismatch — RESOLVED 2026-09-09 (was: KNOWN DEFERRED MISMATCH)

**These constants did not match the SDF.** Verified 2026-09-06 from SDF
geometry:

| quantity | `Control_Alloc.m` (old) | SDF actual (now used) | error (old) |
|---|---|---|---|
| rotor y-offset (s1y/s2y) | ±0.15 m | ±0.2684 m | −44% |
| rotor z-offset (s1z/s2z) | +0.02 m | +0.0301 m | −34% |

Consequence (while deferred): for a commanded `Mx_d`, the allocator
computed a thrust differential assuming a 0.15 m moment arm, but the
vehicle applied it through a 0.2684 m arm — **roll response was ≈1.8×
commanded**. Yaw was affected via the same s_y terms; `Fz` was unaffected
(no moment arm).

This did not show up in Simulink because the same wrong `M0` sits on both
the allocator and plant sides there and cancels. It does not cancel in
PX4/Gazebo, where the plant is the verified SDF geometry — confirmed
2026-09-09: the first real SITL exercise of this cascade produced exactly
the predicted symptom (an immediate "Attitude failure (roll)" at arm),
detailed in `findings.md`'s 2026-09-09 (2) entry and `controller.md`.

### Sign / reference-point error — RESOLVED 2026-09-21

The magnitudes fixed in 2026-09-09 (below) were right; their **signs and
reference point were not**, and the error negated `M0`'s entire moment
block, inverting all three commanded moment axes.

| quantity | pre-2026-09-21 | corrected (body FLU, rel. true CoM) |
|---|---|---|
| rotor 1 y-offset (`kS1y`) | +0.2684 m | **−0.267356 m** |
| rotor 2 y-offset (`kS2y`) | −0.2684 m | **+0.269404 m** |
| rotor z-offset (`kS1z`/`kS2z`) | +0.0301 m | **−0.054924 / −0.054926 m** |
| drag/thrust ratio (`kDragRatio`) | +0.017 | **−0.022274** |

Two independent causes, both settled by user decision 2026-09-21:

1. **Rotor labels.** `Control_Alloc.m`'s frame is Z-up with *its* rotor 1
   on +Y (user-confirmed) — i.e. FLU, the handedness this module already
   feeds the allocator. But that rotor is physically **Motor2/Arm2**;
   `FoldrotorControl.cpp` wires allocator rotor 1 → Motor1/Arm1, which
   sits on −Y in FLU. The channel wiring is bench-verified, so the
   constants were re-signed to match it rather than swapping channels.
2. **Z reference point.** `+0.0301 m` is the rotor height relative to the
   `base_link` **origin**, sign-flipped. The moment arm the vehicle
   rotates about is relative to the **CoM**, which sits 0.0248 m above
   that origin → −0.0549 m in FLU.

`kDragRatio` was separately 31% low: `Control_Alloc.m`'s 0.017 against
`model.sdf`'s `momentConstant` 0.022274 (bench-measured 0.02225). User
decision 2026-09-21: **the SDF value is the source of truth.**

Measured effect, driving the real allocator against an SDF
forward-kinematics model (`src/modules/foldrotor_control/sitl_testing/
allocation_study/`, `findings.md` 2026-09-21): a small commanded moment
at hover produced **−1.00× (Mx), −1.31× (My), −1.00× (Mz)** of what was
asked — positive feedback on all three rate loops, and the cause of the
repeated free-flight flips.

Now guarded by two tests that did not previously exist, both of which
fail against the old constants:
`FoldrotorAllocationTest.GeometryConstantsMatchSdfForwardKinematics`
(this spec's long-planned "Geometry-vs-SDF test") and
`FoldrotorAllocationTest.CommandedMomentProducesSameSignPhysicalMoment`
(the force/moment-direction tier). Both build their expectation from SDF
geometry with explicit cross products, never from `M0`.

**Status: magnitudes fixed 2026-09-09.** Deferred by user decision 2026-09-06,
re-opened and resolved once SITL evidence made it the confirmed cause
rather than a hypothetical one. `FoldrotorAllocation.hpp`'s `kS1y`/
`kS2y`/`kS1z`/`kS2z` now use the SDF-verified values directly (±0.2684 m,
+0.0301 m) rather than being derived from `Control_Alloc.m`'s
`d`/`l_arm`/`h` formula, since that formula is what produced the wrong
figures in the first place. `Minv` re-derives automatically from the
corrected `M0` (see "Done" note below) — no other code changes.

### The matrices

`M0` and `Minv` were hand-typed literals in `Control_Alloc.m` — **not
derived at runtime from d/l_arm/h/k** there. The PX4 implementation
(below) does derive `Minv` at runtime instead, which is what let the
2026-09-09 geometry fix update automatically without a manual matrix
edit.

```matlab
M0 = [ ...
     1       0       0       1       0       0;
     0       1       0       0       1       0;
     0       0       1       0       0       1;
     k      -s1z     s1y    -k      -s2z     s2y;
     s1z     k       0       s2z    -k       0;
    -s1y     0       k      -s2y     0      -k ];
```

**Old `Minv` literal (d=0.05, l_arm=0.10, h=0.02, k=0.017) — SUPERSEDED
2026-09-09, kept only as history.** Valid solely for the old, mismatched
geometry; do not use.

```matlab
% det(M0) = -3.0993e-3, cond(M0) = 58.85 -> full rank, pinv == inv
Minv = [ ...
    +0.5000000000  +0.0074597393   0.0000000000  +0.3729869674    0.0000000000   -3.2910614770;
    -0.5882352941  +0.5000000000   0.0000000000   0.0000000000   +29.4117647059   0.0000000000;
     0.0000000000  +0.0658212295  +0.5000000000  +3.2910614770    0.0000000000   +0.3729869674;
    +0.5000000000  -0.0074597393   0.0000000000  -0.3729869674    0.0000000000   +3.2910614770;
    +0.5882352941  +0.5000000000   0.0000000000   0.0000000000   -29.4117647059   0.0000000000;
     0.0000000000  -0.0658212295  +0.5000000000  -3.2910614770    0.0000000000   -0.3729869674];
```

**Superseded `Minv` literal (s1y/s2y=±0.2684, s1z/s2z=+0.0301, k=0.017)
— SUPERSEDED 2026-09-21 by the sign/reference-point fix above. Kept only
as history; its moment block has the wrong sign throughout.**

```matlab
% det(M0) = -9.836548e-3, cond(M0) = 58.88 -> full rank, pinv == inv
Minv = [ ...
    +0.5000000000  +0.0035373791   0.0000000000  +0.1175209007    0.0000000000   -1.8554476330;
    -0.8852941176  +0.5000000000   0.0000000000   0.0000000000   +29.4117647059   0.0000000000;
     0.0000000000  +0.0558489738  +0.5000000000  +1.8554476330    0.0000000000   +0.1175209007;
    +0.5000000000  -0.0035373791   0.0000000000  -0.1175209007    0.0000000000   +1.8554476330;
    +0.8852941176  +0.5000000000   0.0000000000   0.0000000000   -29.4117647059   0.0000000000;
     0.0000000000  -0.0558489738  +0.5000000000  -1.8554476330    0.0000000000   -0.1175209007];
```

**Current `Minv` literal (s1y=−0.267356, s2y=+0.269404, s1z/s2z=−0.054924/
−0.054926, k=−0.022274) — matches `FoldrotorControlTest.cpp`'s
`kSpecMinv` as of 2026-09-21.**

```matlab
% cond(M0) = 58.88 -> full rank, pinv == inv
Minv = [ ...
    +0.5018950707  +0.0084344507  +0.0001572486  -0.1535630527  +0.0000068943   +1.8502851798;
    -1.2329396653  +0.5000003787  +0.0000000071  -0.0000068943  -22.4476968660  +0.0000830693;
    -0.0001526860  +0.1016269135  +0.5018946920  -1.8502851798  +0.0000830693   -0.1535630531;
    +0.4981049293  -0.0084344507  -0.0001572486  +0.1535630527  -0.0000068943   -1.8502851798;
    +1.2329396653  +0.4999996213  -0.0000000071  +0.0000068943  +22.4476968660  -0.0000830693;
    +0.0001526860  -0.1016269135  +0.4981053080  +1.8502851798  -0.0000830693   +0.1535630531];
```

For the PX4 implementation, deriving the inverse from `M0` at
initialisation (rather than hardcoding it) is what removes this entire
class of staleness — see "Done" below, and this is exactly why the
2026-09-09 fix needed no manual `Minv` edit.

**Done (step 4e part 2):** `FoldrotorAllocation` follows this preference
exactly — `M0` is built from the named geometry constants in the
constructor and `Minv` is derived via `matrix::inv<float,6>()`, never
hardcoded. The literal above now lives only in
`FoldrotorControlTest.cpp` (`kSpecMinv`), as the reference the derived
inverse is checked against (`DerivedInverseMatchesSpecLiteral`).

## Pitch actuation path (decided 2026-09-21)

Pitch has **no moment arm** on this airframe — both rotors sit at the
same body x. `M0` row 4 therefore offers only two paths, and the
controller must choose between them:

```
My_flu = kS1z*(T1x + T2x)  +  k*(T1y - T2y)
         \__ tilt lever __/    \_ drag coupling _/
           |kS1z| = 0.0549        |k| = 0.0223
```

**Contract:** the allocator is handed a wrench in which part of `Fx` may
be a *pitch command* rather than a translation command. `allocate()` is
unchanged and still solves the wrench exactly — this is a statement about
how the caller composes the wrench, not about the algorithm (Scope: the
custom allocator is not replaced, and its math is not rewritten).

| | fold / alpha (drag path) | tilt / beta (lever path) |
|---|---|---|
| joint inertia (`model.sdf`) | 0.0136 kg*m^2 | **0.0012 kg*m^2** |
| actuator, at `p=20, d=0.5` | 6.1 Hz, zeta 0.48 | **20.5 Hz, zeta 1.61** |
| travel per 0.1 N*m, **pre-mast** | 16.3 deg | **6.8 deg** |
| travel per 0.1 N*m, **post-mast** | **12.9 deg** | 16.7 deg |
| lever arm `|kS1z|` | — | 0.0549 -> **0.0170** N*m/N |
| body-x cost per 0.1 N*m | none | 1.8 N -> **5.9 N** |

`FR_PITCH_LEVER` selects the split (0 = drag path only; 1 = lever only,
which drives the differential fold command to exactly zero).

**REVISED 2026-09-22, default 1.0 -> 0.0.** The contract above was written
when the pitch axis was open-loop UNSTABLE at 2.77 Hz and needed ~8.3 Hz
of loop bandwidth that the 6.1 Hz fold actuator could not supply at any
gain. The ballast mast (findings.md (12)) moved the CoM below the rotor
plane, so:

- the axis is now **open-loop stable** (restoring 0.33 N*m/rad against
  0.44 N*m of authority, a stable equilibrium over +-76 deg) and the
  bandwidth argument that justified the lever no longer applies;
- `kS1z` changed sign and the arm shortened 3.2x, so the lever costs
  **more** travel than the path it replaced, plus 5.9 N of body-x force;
- it can no longer extend the envelope at all — reaching the Fx-pinned
  0.44 N*m ceiling would demand 25.9 N of body-x force.

The mechanism is **kept, not removed**: it is correct for any geometry
that puts the rotors far from the CoM, and this airframe may not keep its
mast. Raise `FR_PITCH_LEVER` only with a measured reason.

**Consequences for the envelope fit.** The lever share of `Fx` rides at
*moment* priority, not horizontal-force priority — it is the pitch
moment expressed on the other side of the lever arm. It is held through
the horizontal-force step and scaled with the moment when the moment
itself must yield. Sacrificing it first would silently delete the pitch
command.

**Not yet flown.** Nothing here has been flown since the 2026-09-22
ballast mast. These figures are from `model.sdf` forward kinematics,
the allocator's own forward map, and unit tests. The SITL open-loop and
force/moment direction steps for the lever path have not been run. The
`kS1z` z-offset they all rest on is verified against the SDF but **not**
against a CoM-referenced force/torque measurement — the bench senses
about the mount (findings.md (10), still open).

## Verification

- Saturation test: allocator never outputs commands outside actuator
  limits, including near-singular inputs — **done**,
  `FoldrotorAllocationTest.ThrustClampsAtFifteen`,
  `.TiltClampsAtPositivePointSevenNine`/`.TiltClampsAtNegativePointSevenNine`,
  `.NoOutputIsEverNonFinite` (coarse sweep incl. near-singular inputs)
- Direction/sign test: known desired wrench produces actuator commands
  with the expected sign — **partially done at the unit-math level**
  (`FoldrotorAllocationTest.HoverProducesEvenSplitZeroTilt` and the
  Mx/Mz cases, plus `.LateralFrdWrenchAllocatesConsistentlyWithFullFlip`
  for a non-hover, lateral case exercising the FRD→FLU transform), cross-
  checked against the force/moment direction test in system.md; the
  Gazebo-level cross-check is Part D of
  `.claude/plans/step-4e-allocation-plan.md`, not yet run
- **Minv·M0 ≈ I test — done**, `FoldrotorAllocationTest.MinvTimesM0IsIdentity`.
  As of 2026-09-09 this checks the SDF-verified geometry (the "both
  describe the wrong vehicle" caveat that used to apply here no longer
  does — see "Geometry mismatch — RESOLVED" above); it still only proves
  `Minv`/`M0` mutual consistency, not that `M0` itself matches the SDF.
- **Geometry-vs-SDF test — still missing, no longer deliberately so.**
  Assert `M0`'s s1y/s1z/s2y/s2z against the SDF-derived rotor positions,
  the way `test_frame_convention.py` guards the frames. Would now be
  green on arrival (the geometry fix landed 2026-09-09), so writing it is
  no longer blocked by the old "deferred by user decision" reasoning —
  just not written in this diff, which was scoped to the constants and
  the tests that pin them.
- **Round-trip test — done**,
  `FoldrotorAllocationTest.RoundTripReproducesCommandedWrench`, for the
  three hand-solved unsaturated cases in the step 4e allocation plan.
- **α sign-mapping test — done at the Gazebo/bench level, 2026-09-10.**
  Bench-measured directly (force/torque sensor, both arms): a positive
  commanded α now produces −Y thrust in Gazebo, confirmed by the direct
  (not negated) `α_n → ArmNFoldJoint` mapping — see "α maps directly
  onto the SDF joint convention" above. Unit-tested at the mapping-
  expression level by
  `FoldrotorControlMappingTest.AlphaMapsDirectToFoldChannel`
  (supersedes the old `AlphaMapsNegatedToFoldChannel`, which asserted
  the since-disproven negated mapping).
- **max_tilt test — done**,
  `FoldrotorAllocationTest.MaxTiltConstantIsPointSevenNine`: asserts
  `kMaxTilt == 0.79f`, not `1.0472f`.
- **FRD→FLU frame transform test — done (2026-09-08)**,
  `FoldrotorControlMappingTest.FrdToAllocatorFluNegatesYAndZOnly` (direct
  transform check, all three axes distinct) and
  `FoldrotorAllocationTest.LateralFrdWrenchAllocatesConsistentlyWithFullFlip`
  (end-to-end through `allocate()` with a nonzero lateral force and
  moment — a hover-only case cannot distinguish a correct full flip from
  a partial one, since Y is zero at hover either way).

`open_loop_commands.md` (repo root) has the actuator_test channel map and
reference commands for exercising all 6 channels via SITL — useful for the
saturation/direction tests above once the allocator itself exists to
compare against.

## Not specified here

Actuator failure response, hardware-specific calibration (deferred — SITL
only for now).
