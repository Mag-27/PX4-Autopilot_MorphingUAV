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

## 2026-09-22 (13) — The commanded wrench was unexecutable: derivative gains were pumping vibration into it, and the ballast mast silently invalidated the roll/pitch gains

First hover-quality investigation after (12). User's report: hovering, but
heavy oscillation in fold angle, tilt angle and force. Log
`2026-09-22/06_12_34.ulg`, 98.4 s armed.

### The oscillation is real body motion, not sensor or plotting noise
Groundtruth matches sensed on every axis:

| signal | rms | peak | power > 10 Hz |
|---|---|---|---|
| yaw rate (truth) | 1.06 rad/s (61 °/s) | **18.8 Hz** | 82% |
| roll rate (truth) | 0.34 rad/s | 3.5 Hz | 25% |
| `Mz` command | 1.30 N·m (envelope ±2.56) | 18.8 Hz | 71% |
| `beta` tilt | 0.256 rad rms, 68° p2p | 18.7 Hz | 68% |
| `F1`/`F2` | 1.48 N rms on a 9.7 N mean | 19.7 Hz | 77% |

### It is ONE phenomenon, not three
Yaw moment is made by differential tilt, so `beta` *is* the yaw actuator;
tilting steals vertical force, so collective pumps to hold altitude. The
fold/tilt/force chatter are all downstream of the yaw limit cycle.

### The allocator is NOT the source — checked before blaming it
```
corr(beta1, beta required by Mz alone) = +0.993
corr(beta1, -beta2)                    = +0.976   (pure differential, as yaw wants)
|beta| > 0.58 rad                      =   0.0%   (no railing)
```
Clean, faithful, purely differential. It was rendering a noisy wrench.

### Cause A — the D gains are synthetic inertia, and they were ringing
**Folded** → `controller_params.md` "Derivative gains zeroed". The D term
acts on the measurement against a plant `I·ṙ = M`, so it collects as
`(I + D)·ṙ = P·e_r`: it adds inertia, not damping. `D/I` was 13/7/19%
(roll/pitch/yaw) at 0.36–0.62 Hz loop bandwidths, while supplying 1.14 of
1.30 N·m rms of commanded `Mz` and 1.96 of 2.82 N rms of commanded `Fz`.
Verified by reconstructing each term from the logged rate and derivative
(the roll/yaw sign difference, +0.93 vs −0.74 correlation with the logged
moment, also independently cross-checks the FLU/FRD z-negation).

### Cause B — nothing bounded the command's SLEW
**Folded** → `controller.md` "Command-path bandwidth limit" and
`controller_params.md`. Torque needed to *track* the commanded angles
(`J·d²θ/dt²` vs `cmd_max = 5 N·m`) was 12.1 N·m rms on tilt and 54.9 N·m
rms on fold, over `cmd_max` on 52% and 89% of samples. The servos were
saturated most of the flight; that nonlinear phase lag is what closed the
18.8 Hz loop. Fixed with `FR_WRENCH_LP` = 5 Hz.

### OPEN — the ballast mast invalidated the roll and pitch rate gains
**Not yet acted on.** Recomputing inertia about the CoM from `model.sdf`
(via `foldrotor3_tests/test_frame_convention.py`'s `_transform_to_body_flu()`,
per the standing rule to import those helpers rather than re-derive):

| | before mast | after mast | ratio |
|---|---|---|---|
| mass | 1.5571 kg | 2.0001 kg | — |
| Ixx | 0.06448 | 0.10247 | ×1.59 |
| **Iyy** | **0.00309** | **0.04108** | **×13.3** |
| Izz | 0.06494 | 0.06813 | ×1.05 |

0.443 kg at −0.30 m is a large parallel-axis term. `FR_RATE_P_FF = 0.11`
was sized for `Iyy = 0.00309` (finding (6)); the vehicle now has 13.3×
that, so **pitch is under-gained ~13× and roll ~1.6×**. Yaw is unchanged —
which is why yaw's defect was over-gain at high frequency and pitch's is
under-gain. Consistent with the log: pitch attitude rms 4.66° vs roll
1.25°, and pitch wandering at 0.94 Hz. Both loops are also loafing —
`My` used 0.050 of 0.22 N·m available, `Mx` 0.32 of 1.52.

Deliberately NOT changed in the same diff as the noise fix: raising gains
while the command path still saturates the servos would confound the
result. Sequenced after a re-fly on `FR_WRENCH_LP`.

The pitch tradeoff is authority, not gain — attitude error at which the
pitch rate loop rails against the 0.22 N·m envelope:
`FR_RATE_P_FF` 0.11 → 57° (never), 0.44 → 14.3°, 0.66 → 9.5°,
1.46 (inertia-matched) → 4.3°. Current pitch rms is 4.66°, so the fully
matched gain would sit on the rail roughly half the time until tracking
tightens. **User decision, not made here.**

### OPEN — carried from before
- `FR_RATE_YAW_I` still 0; yaw drifted 59° over the flight.
- `saturated` read 0.0% while `beta` swung 68° p2p — `fitWrenchToEnvelope()`
  absorbs infeasibility before the allocator's flag is set, so the flag is
  blind. Needs a travel-based indicator.
- `IMU_DGYRO_CUTOFF` = 20 Hz sat exactly on the 18.8 Hz limit cycle. Left
  alone because the D path is now inactive; relevant again only if a D
  gain is restored.

---

## 2026-09-22 (12) — Ballast mast: CoM moved below the rotor plane, pitch is now open-loop STABLE

**User's call, and the diagnosis was theirs.** The rotors sat 5.5 cm
BELOW the CoM; on a thrust-VECTORING vehicle that makes body-horizontal
force pitch-destabilising. Putting the CoM below the rotors flips that
term from destabilising to restoring. (11) fought the unstable pole with
a better actuator; this removes the pole.

### Evidence it was still the binding failure

Log `2026-09-22/05_25_26.ulg`, armed 10.12-11.73 s. **Pitch departs
alone** — roll is inside +-3 deg until pitch is already at 48 deg:

```
 t=10.80  pitch -18.2  roll +2.2     <- at the ~21 deg tipping point
 t=11.10  pitch +48.1  roll +0.9     <- pitch departs by itself
 t=11.20  pitch +79.6  roll -20.3    <- roll only follows
 t=11.40  pitch +62.9  roll -179.5   <- inverted
```

This **corrects the previous session's read** of `11_52_08.ulg` as a roll
regression. Roll was a consequence of the pitch departure, not its cause.
The (10)/(11) roll-regression item is accordingly downgraded, not closed:
nothing has bisected `360a0967`, but no log yet shows roll departing
first.

### The change

`model.sdf` gains `ballast_link`: 0.443 kg at z = -0.30 m on a fixed
joint, bringing the model to the **2.00 kg the real vehicle is estimated
at** (it was 1.557 kg -- a fidelity gap independent of the CoM question).
Disc radius 0.12 m so the vehicle can stand on it; model spawn pose
raised 0.1 -> 0.32 m to clear it.

Depth is not a free choice. At 0.443 kg the CoM reaches the rotor plane
only at z = -0.2233 m; -0.30 m buys +0.017 m of margin. **A 30 cm mast is
the real cost of this approach** and needs landing gear taller than
itself. Raising the rotors ~7 cm achieves the same sign flip with no mass
and no mast -- offered and not taken, recorded here because it stays
available.

| | before | after |
|---|---|---|
| mass / hover thrust | 1.557 kg / 15.27 N | **2.000 kg / 19.62 N** |
| CoM z (FLU) | +0.0248 | **-0.0471** |
| s_z (rotor rel. CoM) | **-0.0549 (destabilising)** | **+0.0170 (RESTORING)** |
| dM/dtheta | +0.94 N*m/rad, unstable at 2.77 Hz | **-0.33 N*m/rad, stable** |
| pitch equilibrium range | unstable beyond ~21 deg | **stable over +-76 deg** |
| Ixx / Iyy | 0.06447 / 0.00309 | 0.10247 / **0.04108** |
| Iyy/Ixx | 0.048 | 0.401 |
| T/W | 1.97 | 1.53 |

Geometry independently reproduced by `sdf_fk.py` and by the
`foldrotor3_tests` frame chain, agreeing to 5 decimals.

### Consequences worked through

- **`kS1z`/`kS2z` changed sign** (-0.054924 -> +0.017010). All four
  geometry constants updated; `Minv` literal, the moment-envelope table
  (`gentable.py`) and `kRateM*Limit` (1.52 / 0.22 / 2.56) regenerated.
- **`FR_PITCH_LEVER` now defaults to 0.** The lever arm shrank 0.0549 ->
  0.0170 N*m/N, so (11)'s mechanism inverts and weakens: the same 0.1 N*m
  now costs **16.7 deg of tilt (was 6.8) and 5.9 N of body-x force (was
  1.8)** -- more travel than the fold path it was meant to replace
  (12.9 deg). It can no longer extend the envelope either: reaching even
  the Fx-pinned 0.44 N*m ceiling would need 25.9 N of body-x force. The
  mechanism is kept, not deleted -- it is correct for any geometry with
  the rotors far from the CoM. Pinned by
  `PitchLeverNoLongerExtendsTheEnvelopeAtMastGeometry`.
- **Pitch gains are envelope-limited, not inertia-matched.** The
  Iyy/Ixx rule gives `FR_RATE_P_FF` = 0.196, but that rails the rate loop
  at 0.96 rad/s against the 1.57 rad/s the attitude loop commands at its
  45 deg design error, because the pitch moment envelope (0.188 N*m) did
  not grow with the inertia. The envelope is a hard limit and the
  matched-bandwidth rule is a heuristic, so **0.11 ships** (ceiling
  0.1197). `FR_RATE_P_I` 0.112 and `FR_RATE_P_D` 0.0029 hold roll's
  P:I:D ratios against it.
  **Stated plainly: roll and pitch no longer have matched bandwidth.**
  Pitch is deliberately slower. That is only acceptable because the mast
  made the axis open-loop stable -- it would not have been before.
  Caught by `RateLoopStaysLinearOverAttitudeLoopDemand`, which is the
  test doing exactly its job.

### NOT YET FLOWN

All of the above is model geometry, the allocator's forward map, and unit
tests. No SITL run has happened since the mast was added. In particular
the 2.00 kg / 30 cm mast is a **simulation** change: if the real airframe
cannot carry its mass that low, the sim is now easier than the vehicle
and the gains derived here do not transfer. Flying it is the next step,
not a formality.

**Still open:** the CoM-referenced pitch direction test from (10) still
does not exist, and it matters more now -- every number above depends on
`s_z`, whose sign this change deliberately inverted, and the bench senses
about the MOUNT rather than the CoM. Also unresolved: the ballast-disc
ground contact is untested, and `beta1` was seen pinned at the -45.3 deg
rail through the whole `05_25_26` departure while `saturated` read 0%,
because `fitWrenchToEnvelope()` absorbs infeasibility before the
allocator sets that flag -- the saturation flag has gone blind and needs
a travel-based indicator instead.

---

## 2026-09-21 (11) — Pitch was routed through the worst actuator the airframe has. Fixed by giving the attitude loop the body-x force.

**Resolves (10).** Its open hypothesis (pitch rate signal is vibration
noise; fix is `IMU_GYRO_CUTOFF`) is **falsified by measurement**, see the
groundtruth comparison below. Do not filter the gyro on that basis.

### What (10) was missing

Pitch has no moment arm on a side-by-side rotor pair, so `M0` row 4
leaves exactly two paths:

```
My_flu = kS1z*(T1x + T2x)  +  k*(T1y - T2y)
         \__ tilt lever __/    \_ drag coupling _/
           |kS1z| = 0.0549        |k| = 0.0223
```

The tilt lever is 2.5x stronger per unit thrust, but using it produces
net body-x force — which the `Fx` row forbids while `Fx` is pinned to the
position loop's request. So the allocator was **forced down the drag
path**, and the drag path is brutally stiff. Jacobian of `allocate()` at
hover (15.27 N, level):

| | per 0.1 N*m of commanded pitch | joint inertia | actuator |
|---|---|---|---|
| drag path (fold, alpha) | **16.3 deg** | 0.0136 kg*m^2 | **6.1 Hz, zeta 0.48** |
| lever path (tilt, beta) | **6.8 deg** | 0.0012 kg*m^2 | **20.5 Hz, zeta 1.61** |

Both joints carry the *same* `model.sdf` PID (`p_gain=20, d_gain=0.5`);
the tilt sub-chain is 11x lighter, so it is 3.4x faster and properly
damped rather than underdamped.

**This is why no gain ever worked.** (9) measured the pitch axis as
open-loop unstable at `sqrt(0.94/0.00309) = 17.4 rad/s = 2.77 Hz`,
wanting roughly 8.3 Hz of loop bandwidth. A 6.1 Hz actuator cannot
provide that at any gain. The 09-18 limit correction, the 09-21
`Iyy/Ixx` rescaling and (8)'s envelope fit each removed a real defect,
and pitch departed anyway, because none of them touched which actuator
the moment came out of.

### (10)'s vibration hypothesis, falsified

Log `2026-09-21/11_19_09.ulg`, window 16.3–19.0 s, sensed gyro vs the
simulator's exact rigid-body state:

| | `vehicle_angular_velocity` | `..._groundtruth` |
|---|---|---|
| p std | 175.5 deg/s | **138.9 deg/s** |
| q std | 155.5 deg/s | **132.9 deg/s** |

Groundtruth oscillates essentially as hard as the gyro reports, so the
motion is **real body motion, not a noisy signal**. Filtering would have
hidden it. `My` never exceeded 0.181 N*m and was at its rail on 0% of
samples — the loop was not saturated, the axis simply had no authority.

### The change

`FR_PITCH_LEVER` (0..1, default 1.0) routes that fraction of the
commanded pitch moment through the body-x lever. Commanding
`Fx = My_frd / 0.0549` alongside `My` does not add a second moment — the
allocator still realises exactly the `My` asked for. It changes *which
actuator* realises it: the lever supplies `FR_PITCH_LEVER` of the moment
and the drag path the remainder, so at 1.0 the differential fold command
falls out entirely. **`allocate()` itself is untouched** (CLAUDE.md rule
7) — only the wrench it is handed.

It also raises the authority, measured against the allocator's own
forward map with SLSQP from 300 restarts:

| | max abs(My) at hover | Fx it costs |
|---|---|---|
| Fx pinned (before) | **0.342 N*m** | 0 |
| Fx free (actuator box corner) | **1.490 N*m** | 21.2 N |

Additive to within 1%: `max|My| = 0.342 + 0.0549*Fx`. The 0.342 figure
independently reproduces (10)'s 0.34, which cross-validates the model.

**Two things had to move with it or the change is inert:**

1. `fitWrenchToEnvelope()` sacrifices horizontal force *first*, by
   design (8). The lever component is exempt — it IS the pitch moment,
   not a mission objective — and it scales *with* the moment in step 2.
   Without the exemption the fit silently deletes the pitch command, a
   failure that in flight would look exactly like "the lever didn't
   help". Guarded by `FitHoldsThePitchLeverAtMomentPriority` and
   `FitScalesThePitchLeverWithTheMoment`.
2. `kRateMyLimit` / the scheduled envelope was measured with Fx pinned,
   so it caps pitch at the old authority and — because the same bounds
   drive conditional-integration anti-windup — would make the rate loop
   believe it is saturated when it is not. The mirror image of the 09-18
   defect. Bumped by `FR_PITCH_LEVER * kPitchLeverFxLimit * 0.0549`.

`kBodyForceXYLimit` (1.0 N, from (9)) is **not** the same budget and is
unchanged. Same physical quantity, opposite intent: the position loop's
Fx is the destabilising direction, the attitude loop's carries the
correcting sign. Collapsing them is what left pitch with no actuator —
at 1.0 N that cap was holding the lever to 0.055 N*m, **16%** of the
0.342 N*m the airframe already had. New separate budget
`kPitchLeverFxLimit` = 6.2 N, sized to cover the full hover envelope
through the lever alone.

**The cost, stated plainly:** 1.82 N of unwanted body-x force per
0.1 N*m of pitch — the vehicle translates while it corrects attitude.
Same ordering (9) already accepted (attitude first, position error
second), applied to the horizontal axis instead of the vertical one.

**NOT YET FLOWN.** Everything above is bench/unit-test and model
analysis. Verification order (CLAUDE.md): interface tests pass; the SITL
open-loop and force/moment direction steps for the lever path have not
been run, and no hover has been attempted. Do not read the authority
numbers as flight-validated.

**Still open, carried over from (10):** a CoM-referenced pitch direction
test does not exist — the bench senses about the MOUNT. The 0.94
N*m/rad loop gain and the 0.0549 N*m/N lever both depend on the rotor
z-offset relative to the true CoM, so both rest on `model.sdf` forward
kinematics alone.

**Also open and unrelated to pitch:** roll regressed badly between (10)
and log `11_19_09.ulg` — (10) recorded roll at 0.1 deg and 1.1 m
horizontal departure; that log shows +-10 deg roll at +-290 deg/s and
27 m of departure. Not bisected. Suspect `360a0967` (unpin alpha /
fold sign mapping).

**Separately fixed, same session:** the module published two
`debug_array` messages per cycle (`fr_alloc` id 0, then `fr_wrench`
id 1). Single-instance topic, queue depth 1, so the second destroyed the
first for the logger and `MavlinkStreamDebugFloatArray` alike — 4041 of
4041 records in `11_19_09.ulg` were id 1, zero `fr_alloc`. The allocator
output was invisible in the log and over MAVLink, which is why
`plot_hover.py`'s force and servo-angle subplots came up empty for a
whole flight-test session. Merged into one message (slots `[0..6]`
output, `[7..12]` input wrench). `foldrotor3/model.sdf` also never
carried the `gz-sim-joint-state-publisher-system` the README claimed,
so the actual-joint-angle overlay was silently empty too; added.

---

## 2026-09-21 (10) — RESOLVED by (11): pitch departs at ~9 deg

Status after (5)-(9). What is fixed, measured:

| | before | after |
|---|---|---|
| allocator saturation | 95% | ~0% |
| roll | 179 deg flip | **0.1 deg** |
| horizontal departure | 272 m | 1.1 m |
| peak altitude | 0.42 m | 0.61 m |

**Still failing:** pitch departs at ~9 deg within ~2 s of arming and runs
to a 73.4 deg mechanical stop, on every run, repeatable to 0.2 deg. The
vehicle does not hover.

**Ruled out by measurement, not by argument:**

- *Allocator saturation* -- now ~0%, and pitch departs anyway.
- *Pitch moment sign / geometry* -- `kS1z`/`kS2z` confirmed EXACTLY against
  `model.sdf` forward kinematics (`foldrotor3_tests`). The bench
  force/torque measurement (both rotors, tilt +0.5, 20.1 N) gives
  `dFx` = +7.788 N and `dMy` = -0.2498 N*m, matching
  `expected_wrench.py`'s SDF oracle in sign and to 18% in magnitude
  (the oracle warns its magnitudes are a single-point fit). NOTE: the
  bench senses about the MOUNT, not the CoM, so it cannot by itself
  validate a CoM-referenced moment -- the agreement is between two
  mount-referenced quantities. A CoM-referenced pitch direction test
  does not exist yet and is the main gap in the verification chain.
- *Thrust mapping* -- bench-confirmed: `-v 0.6` gives 10.07 N measured vs
  `5.4844e-6 * 1355.6^2` = 10.08 N computed.
- *Inverting the commanded `My`* -- tried directly as an experiment. The
  vehicle did not stabilise; it flipped in roll instead and never flew.
  Not clean evidence either way, and NOT a justification for flipping a
  constant that three independent sources agree on.

**Hypothesis below was FALSIFIED — see (11).** Groundtruth angular rates
oscillate as hard as the sensed ones, so this is real body motion, not a
noisy signal, and `IMU_GYRO_CUTOFF` is not the fix. Kept for the record
because the reasoning was sound given what was measured at the time.

**Best remaining hypothesis, unconfirmed.** The pitch rate signal is
dominated by structure/contact vibration: `q` oscillates +/-4 rad/s with a
near-zero mean while the logged attitude sits at 0.1 deg, and `rate_dot`
reaches 292 rad/s^2 -- 6x what the 0.146 N*m pitch authority can produce,
so it cannot be commanded motion. `Iyy` = 0.00309 is 21x smaller than
`Ixx`, so the pitch axis is far more excitable than roll, which is now
clean. With `FR_RATE_P_FF` = 0.049 the loop rails at a rate error of
3.0 rad/s, i.e. **the noise alone rails it** (`My` pinned at its limit on
78% of samples). If so the fix is filtering (`IMU_GYRO_CUTOFF`, currently
40 Hz) rather than gains, and the vibration source -- plausibly the
fold/tilt servos chattering and reacting against a very light pitch axis
-- needs identifying first.

**Do not** re-tune pitch gains against this until the vibration is
characterised; the loop is not tracking a signal it can be tuned for.

---

## 2026-09-21 (9) — Rotors sit BELOW the CoM, which makes thrust-vectored horizontal force pitch-destabilising

**Measured geometry** (`model.sdf` forward kinematics, via
`foldrotor3_tests`): CoM at z = +0.0248 m, both rotors at z = -0.0301 m,
so the rotors are **5.5 cm below the centre of mass**
(`kS1z` = -0.054924, `kS2z` = -0.054926 -- both confirmed exactly, not
merely consistent).

**Consequence.** A body-forward force acts below the CoM and produces a
nose-up moment, `My_frd = -kS1z * Fx = +0.055 * Fx`. Combined with (7)'s
full rotation this closes a **positive feedback loop**: pitching nose-up
makes the velocity loop ask for body-forward force to keep pushing up in
NED, and that force pitches the vehicle further nose-up.

Loop gain `dMy/dtheta = |kS1z| * Fz ~= 0.055 * 17 = 0.94 N*m/rad`, against
a pitch authority of ~0.146 N*m. The level equilibrium is therefore
unstable beyond `0.146 / 0.94 = 0.155 rad =` **8.9 deg**, and past that no
attainable moment recovers it. Measured: the vehicle departed at ~9 deg on
every run and ran to a 73.4-73.6 deg mechanical stop, repeatable to 0.2
deg across runs.

**Mitigation applied** (`kBodyForceXYLimit`, 1.0 N): cap horizontal force
in the BODY frame, after the rotation, where the destabilising moment is
actually set. The inertial cap (`kPosVelForceXYLimit`) does not bound it,
because once tilted the body horizontal force is dominated by the rotated
collective (`Fx_body = Fz * sin(theta)`), not by the horizontal command.

**This mitigation did NOT stop the pitch departure** -- see the open
question below. It is retained because the analysis above is independently
correct and the bound is cheap, not because it was shown to fix anything.

---

## 2026-09-21 (8) — Fit the wrench to the envelope before allocating, instead of tabulating it

(5)'s `momentEnvelopeAtThrust()` table indexes on collective thrust alone
while holding a fixed `|Fxy| <= kPosVelForceXYLimit` in reserve. After (7)
that premise collapses: the horizontal cap is applied in the INERTIAL
frame but the allocator's alpha/beta rails are BODY constraints, so once
the vehicle tilts the body-frame horizontal force grows without bound (at
73 deg of pitch, holding altitude asks for ~17 N of body-forward force).
Allocator saturation came back at **68%**.

**Replaced with** `fitWrenchToEnvelope()`: before allocating, scale the
wrench down against the REAL allocator until it is feasible, in priority
order **vertical force > moment > horizontal force**. Horizontal position
is the only one of the three that is a mission objective rather than a
survival condition, so it yields first. Three bisections, 14 iterations,
exact for the actual commanded direction.

This does **not** change the allocation algorithm (rule 7) -- it
guarantees the algorithm is only ever handed inputs it can satisfy
exactly, removing the clamp-not-redistribute hazard by construction rather
than by tuning. The table is kept for the rate loop's output limits, where
a cheap, monotone, always-nonzero bound is what the anti-windup needs.

Saturation returned to ~0%. Guards: `FittedWrenchNeverSaturates` (sweeps
600 wrenches including physically impossible ones),
`FitSacrificesHorizontalForceBeforeMoment` (pins the priority order).

---

## 2026-09-21 (7) — The force path rotated by YAW ONLY, so a tilted vehicle aimed its lift sideways

**Reverses the 2026-09-17 change** that made `Inertial2Body` a yaw-only
rotation. Its premise -- "this vehicle translates by thrust vectoring, not
by leaning, so assume level" -- is true in its first half and does not
imply its second. Assuming level does not make the vehicle level; it makes
the controller blind to the roll/pitch it has.

**Measured cost:** at the sustained 22 deg of pitch the vehicle was
holding, ~17 N of commanded lift became **~6.4 N of uncommanded
NED-horizontal force**, against the 1.0 N the position loop is permitted
to answer with. The loop loses by a factor of six every cycle. The vehicle
climbed to altitude and departed **272 m downrange in 14 s**.

**Fixed** by taking the full body<-NED rotation from the attitude
quaternion's DCM. `F_b = R^T * F_i` means the delivered inertial force is
`R * F_b = F_i` at any attitude -- the property a fully-actuated vehicle
is supposed to have. **No singularity**: the pre-2026-09-17 full-Euler
version reconstructed the rotation from phi/theta/psi and inherited 3-2-1
gimbal lock at pitch = +/-90 deg; taking the DCM from the quaternion never
forms an Euler triple. That open concern in `controller.md` /
`controller_params.md` was always about the PARAMETERIZATION, not about
using the full attitude, and no longer applies.

**Result:** flyaway 272 m -> 1.1 m. Roll settled from a 179 deg flip to
**0.1 deg**.

Guards: `PitchIsNotIgnored`, `RollIsNotIgnored`,
`NoSingularityAtNinetyDegreePitch`, `MatchesPx4DcmTransposeAcrossAttitudes`.

---

## 2026-09-21 (6) — The rate loop was a relay: gains were ~7x the vehicle's real moment authority

Follows directly from (5), which removed allocator saturation and so
finally made the rate loop's own behaviour visible.

**Symptom after (5).** Saturation 95% -> 1.4%, but the vehicle still
rolled over within 1.8 s. The trace showed why: `Mx` sat at **exactly
+/-`mx_limit` on essentially every sample** from 44 ms after arming, with
`p` oscillating +/-4 rad/s. That is a relay, not a controller.

**Cause.** `FR_RATE_R_FF` was 3.5 N*m per rad/s against a deliverable
roll moment of 1.47 N*m. The loop therefore rails at a rate error of
1.47/3.5 = **0.42 rad/s**. The attitude stage above it commands
`p_sp = FR_ATT_P * e_phi = 3.0 * e_phi`, which passes 0.42 rad/s at
**e_phi = 8 degrees**. So the cascade was linear only within 8 deg of
level and bang-bang everywhere outside it. Pitch had the same defect
(`FR_RATE_P_FF` 0.17 vs 0.146 N*m deliverable -> rails at 0.86 rad/s).

These gains were never wrong in isolation -- they were scaled off each
other by inertia ratio (2026-09-21 (2)) and never checked against the
measured moment envelope, which (5) established is roughly half what the
single-axis measurements suggested.

**Second defect, same root:** every integrator clamp was above the
authority it feeds.

| param | was | authority it feeds |
|---|---|---|
| `FR_RATE_R_I_LIM` | 3.8 N*m | 1.47 N*m |
| `FR_RATE_P_I_LIM` | 0.30 N*m | 0.146 N*m |
| `FR_VEL_XY_I_LIM` | **15 N** | **1.0 N** (the new XY cap) |

An integrator that can wind to 10x the rail is not anti-wound by any
mechanism downstream of it.

**Fix.** Re-sized the whole cascade from measured authority rather than
from each other. `FF = authority / linear_range` with a 3 rad/s linear
range; `D = 0.2 * I_axis` (the old `D` was ~7.8x `Ixx`, which made the
D-term dominate the effective inertia); integrator clamps set below their
rails; `FR_ATT_P` 3.0 -> 2.0 to restore bandwidth separation over the now
slower rate loop; `FR_POS_P` 3.0 -> 0.4 because the horizontal velocity
loop's bandwidth is only ~0.6 rad/s once the XY force cap is respected,
so a 3.0 position gain inverted the cascade ordering.

**Guards added.** `RateLoopStaysLinearOverAttitudeLoopDemand` (the rate
loop must not rail inside the attitude loop's command range at 45 deg of
error) and `IntegratorLimitsStayBelowTheirAuthority`. Both fail on the
old defaults. These are the invariants that were violated for the entire
2026-09-1x/2026-09-21 investigation without ever producing a test failure.

---

## 2026-09-21 (5) — The force limit was the wrong SHAPE: a sphere on a cone-constrained vehicle

**This is the cause of the in-air flip that (4) left unexplained**, and it
supersedes (4)'s roll-only authority schedule.

**What was wrong.** `PositionVelocityControl`'s mc_pos_control-derived
sphere saturation bounds *total force magnitude* (28 N). A sphere treats
horizontal and vertical force as interchangeable, so at the hover-ish
`Fz` = 17.36 N it permitted `|Fxy|` up to
sqrt(28^2 - 17.36^2) = **22.0 N**. But this vehicle cannot produce force
in an arbitrary direction: horizontal force comes only from tilting the
rotors, and both `alpha` and `beta` are railed at +/-`kMaxTilt` = 0.79
rad. Bisecting the **real allocator** for the largest feasible `|Fxy|` in
the worst-case bearing (`allocation_study/envelope.py`):

| moments reserved | max deliverable `\|Fxy\|` at Fz = 17.36 N |
|---|---|
| none | 6.38 N |
| `Mx` = 3.8 alone | 0.00 N |
| moderate, all three | 0.00 N |

So the position loop was allowed to ask for ~4x what the vehicle can do
with *no* moments reserved, and unboundedly more than it can do once the
rate loop's demand is counted. Being upstream, it won that contest every
cycle, drove alpha/beta onto their rails, and left the rate loop's moment
to be delivered by whatever angular range remained -- which is how a
commanded `My` came back **sign-flipped**.

**Second error, same measurement.** (4)'s `rollAuthorityAtThrust()` was
right in kind but wrong in scope. It scheduled **only roll**, justified by
single-axis measurements showing yaw authority *rising* with `Fz` and
pitch flat. Those were each taken **with the other two axes at zero**,
which is never how the rate loop uses them. Demanded together, all three
collapse:

| | Mx | My | Mz |
|---|---|---|---|
| single-axis max, Fz = 17.36 | 3.43 | 0.39 | 4.38 |
| **simultaneous** | **1.75** | **0.17** | **2.17** |

Every rate limit was therefore ~2x oversized even after (4).

**Fixes.**
1. `momentEnvelopeAtThrust()` replaces `rollAuthorityAtThrust()`: a
   measured table (bisected against the real allocator, 2 N grid, 0.85
   safety factor, `allocation_study/gentable.py`) scheduling **all three**
   axes. The envelope is **non-monotonic** -- it peaks near Fz ~ 18-20 N
   and collapses at both ends -- which is why no constant could express it.
2. `setHorizontalForceLimit()` -- a hard `|Fxy|` <= 1.0 N cap applied
   after the sphere and before the tracking anti-windup, so the ARW sees
   the real bound. 1.0 N is 0.64 m/s^2: weak, but genuinely what a
   two-rotor vehicle with +/-45 deg of tilt has left after holding itself
   up and keeping itself upright. **Attitude outranks position here; that
   ordering is the airframe, not a tuning choice.**

The two constants are a **matched pair** -- the moment table is measured
holding exactly `kPosVelForceXYLimit` in reserve. Change one, regenerate
the other.

**Result: allocator saturation 95% -> 1.4%.** The flip did not stop,
which is what exposed (6).

**Guards:** `ScheduledMomentEnvelopeIsDeliverable` (every scheduled
wrench must allocate without clamping, swept over bearing and thrust) and
`HorizontalForceCapIsWithinTiltAuthority`. Both fail on the old values.

---

## 2026-09-21 (4) — Thrust-scheduled roll limit + reduced climb command: first liftoff, still flips in the air

**Changes made** (the agreed order: close the logging gap, then Fix 1 +
Fix 2, then bench, then re-fly):

1. **Per-cycle diagnostic trace** (`FoldrotorControl.{hpp,cpp}`,
   `foldrotor_control trace`). `debug_array` cannot resolve this failure:
   the logger polls every `_log_interval` = 3500 us and `DebugArray.msg`
   has no queue depth, so a 1 kHz publication is sampled at ~250 Hz.
   Raising that means editing the logger or the msg, both outside this
   module's boundary, so the module keeps its own 2048-sample ring
   buffer instead (rate, rate_dot, rate_sp, M, F, the granted roll limit,
   saturation, armed), frozen on the disarm edge and dumped as CSV.
   Confirms the loop genuinely runs at 1 kHz (samples exactly 1000 us
   apart). Temporary, same lifetime as `fr_wrench`.
2. **Fix 1 — `FoldrotorControl::rollAuthorityAtThrust()`**, called every
   cycle before the rate loop, feeding the existing `setOutputLimits()`
   so the anti-windup sees the true bound too. Only roll is scheduled;
   measured yaw authority rises with Fz and pitch is roughly flat.
   Guarded by `RollAuthorityTracksCollectiveThrust`, which sweeps Fz
   15.26 -> 28 N and **fails from Fz = 16.01 N upward** against the old
   constant (verified by reverting).
3. **Fix 2 — `FR_VEL_Z_MAX_UP` 1.0 -> 0.3 m/s**, cutting the takeoff
   collective from 22.26 to 17.36 N and restoring roll authority from
   ~2.2 to ~3.5 N·m.

**Bench force/moment direction test — PASSED, and it caught a real bug.**
The 2026-09-21 constants correction had only ever been checked against
the SDF forward-kinematics oracle, never on Gazebo physics. Run on the
bolted `foldrotor3_bench` (static reading −15.260017 N, so the fixture
senses rather than welds through):

| case | dFz exp/meas | dMx exp/meas | dMz exp/meas |
|---|---|---|---|
| `-m 1 -v 0.6` | −10.078 / −10.066 | −2.7048 / −2.7216 | +0.2245 / +0.2240 |
| `-m 2 -v 0.6` | −10.078 / −10.066 | +2.7048 / +2.7210 | −0.2245 / −0.2239 |
| both | −20.157 / −20.132 | 0 / −0.0006 | 0 / +0.0001 |

`kDragRatio` checks out directly: 0.022274 × 10.07 = 0.2243 vs 0.2240
measured. Baseline `Mx` +0.0159 matches the documented static residual.
**The constants are now bench-verified, not just oracle-verified.**

The bug it caught: the new `imu_sensor` comment in the flight
`model.sdf` contained `--`, which is illegal inside an XML comment, so
the model would not parse. `foldrotor3_tests/` failed 7 tests on it.
Running the tier in order caught it before it reached a flight.

**Re-fly result — real, measurable, still failing.** Armed 0.490 s ->
**1.080 s**, and the failure moved from pitch to roll. **It leaves the
ground for the first time**: z −0.070 -> −0.489 (42 cm) by 388 ms, having
never exceeded 0.08 m in any prior run. Then it flips in the air (pitch
+66° at 286 ms, roll −142° at 388 ms) and falls back.

The trace confirms both fixes work as designed: `mx_limit` tracks live
(3.16 / 3.39 / 3.48 / 2.50 as Fz moves) and Fz = −17.35 N, exactly the
17.36 N predicted from `FR_VEL_Z_MAX_UP` = 0.3.

**What it did NOT fix.** Saturation is essentially unchanged: 95% of
samples (was 96%), |dMy| over budget on 13% (was 19%). The position loop
still commands up to `Fy` = −24.4 N. Delivered `Fz` stays above weight
(+2 to +8 N), so lift is not the limit — the flip is. Ground contact is
*not* the dominant term after ~100 ms: the vehicle is airborne at
z = −0.35 when pitch runs away, and the measured saturation-induced pitch
error (up to 0.462 N·m = 149 rad/s² on Iyy) is by itself enough to
explain that runaway.

**Conclusion: Fix 3 (the position-loop rework,
`.claude/plans/read-mc-pos-contorl-and-can-greedy-pearl.md`) is the
remaining cause and is now warranted by measurement.** Bounding the
commanded force to the deliverable envelope is what removes the
saturation that corrupts the delivered moment.

---

## 2026-09-21 (3) — Sim raised to 1 kHz (D now stable); flight still fails, on a different and dominant cause: moment authority is thrust-dependent and the allocator is saturated 96% of the time

**Change made** (user decision, from the options in (2)): raise the sim to
1 kHz and recheck `D` there. Two files, because the world step alone was
not the binding constraint:
- `Tools/simulation/gz/worlds/foldrotor.sdf` — new; the stock `default`
  world with physics at `max_step_size` 0.001 / 1000 Hz. A dedicated
  world rather than editing `default.sdf`, which every gz vehicle in the
  repo shares. `4026_gz_foldrotor3` now defaults `PX4_GZ_WORLD` to it
  (still env-overridable).
- `Tools/simulation/gz/models/foldrotor3/model.sdf` — `imu_sensor`
  `<update_rate>` 250 → 1000. **This, not the world step, set the rate
  loop's dt**; the world step only bounds it. Confirmed by `uorb top`:
  `vehicle_angular_velocity` 250 → 1000 Hz.

**`D` rechecked at 1 kHz: no change needed.** `D` = 0.5 roll / 0.024
pitch sit at **46% of the stability limit** on both axes (pole +0.023 /
+0.021; `D_max` 1.091 / 0.052). A full cascade sim settles an 8.6° step
in 0.63 s with peak moments well inside the limits, and absorbs the
logged arm seed with a 5.1° roll excursion. Margin holds down to ~2.3 ms;
it re-diverges at dt ≥ 2.5 ms, so the 1 kHz IMU rate is load-bearing —
hence the comments in both SDFs. `allocation_study/dsweep.py`,
`dcheck1k.py`.

**Flight result: no measurable improvement.** Armed 0.490 s (was 0.504),
same `Preflight Fail: Attitude failure (pitch)`, pitch +3.2° → −46° at
86 ms → −76° at 137 ms, never climbs (z −0.07 of −1.5 commanded). The
D-term fix is necessary but not sufficient — a different bottleneck
dominates, and until it is fixed the D change cannot be validated in
flight, only in sim. **Caution: the 45 s of level attitude in the
`hover_1k.csv` trace is a *disarmed* vehicle sitting on the ground, not a
hover** — actuators go NaN at 0.51 s.

**Dominant cause — moment authority is a function of commanded `Fz`,
but `kRateMxLimit`/`kRateMyLimit`/`kRateMzLimit` are constants sized at
hover.** Bisected against the real allocator (`authvsfz.py`):

| `Fz` cmd | × hover | Mx auth | (lim 3.8) | My auth | (lim 0.30) | Mz auth | (lim 3.8) |
|---------|---------|---------|-----------|---------|------------|---------|-----------|
| 15.26 | 1.00 | 4.02 | ok | 0.357 | ok | 4.15 | ok |
| 18.00 | 1.18 | 3.39 | **OVER** | 0.421 | ok | 4.89 | ok |
| 22.26 | 1.46 | **2.20** | **OVER** | 0.468 | ok | 5.41 | ok |
| 26.83 | 1.76 | 0.92 | **OVER** | 0.328 | ok | 3.71 | **OVER** |

Roll authority *collapses* as collective thrust eats the per-rotor
[0, 15] N budget: at `Fz` = 22.26 N each rotor sits at 11.1 N with only
3.9 N of headroom, so the deliverable differential is less than half the
hover value. **The vehicle commands `Fz` = 22.26 N from the instant of
arming by design** (`FR_VEL_Z_GRAV_FF` 15.26 + `FR_VEL_Z_FF` 7.0 × the
`FR_VEL_Z_MAX_UP` 1.0 m/s cap), so the rate loop is permitted to ask for
3.8 N·m of roll while the allocator can deliver 2.20 — 1.7× over, on
every takeoff.

This is the same class of defect as 2026-09-18 (2) — rate limits
overestimating true authority — at a different operating point. That fix
measured the maxima *at hover* and `RateLimitsDoNotExceedHoverMomentAuthority`
guards only that point, so it passes while the vehicle never actually
operates there during takeoff.

**Consequence, measured** (`clampcheck.py`, replaying the logged
commanded wrenches through the real allocator and forward-mapping the
clamped solution back through `M0`): the allocator is **saturated in 96%
of samples from arming onward**, and because the policy is
clamp-not-redistribute the delivered wrench departs arbitrarily from the
commanded one. Worst logged sample: commanded
`(Fx 18.2, Fy 16.1, Fz 13.9, Mx 3.8, My 0.30, Mz 3.8)` delivered as
`(9.9, 2.1, 19.0, 0.51, −0.12, 2.18)` — `Fy` down 87%, `Mx` down 87%, and
**`My` sign-flipped**. 19% of samples have a pitch-moment error larger
than the entire pitch budget (mean |ΔMy| 0.145, max 0.416 N·m vs the
0.30 limit). That fully accounts for the pitch runaway: measured pitch
rate −19 rad/s at 86 ms implies −1.28 N·m on `Iyy`, 4.3× what the rate
loop is allowed to command — the vehicle is not getting the wrench the
controller asked for.

Lateral demand is what rails it: the position loop calls for `Fx` of
±20–25 N, 1.3–1.7× vehicle weight sideways, which drives both tilt
servos to the ±0.79 rad rails and leaves no envelope for moments.

**This is the failure mode `.claude/plans/read-mc-pos-contorl-and-can-greedy-pearl.md`
was written to address** (position loop over-commanding force, starving
the attitude loop of rotor-thrust headroom, fold/tilt railing within
milliseconds of arming) — that plan is still open and now has direct
measured support. Decision pending with the user.

---

## 2026-09-21 (2) — Rate loop runs at 250 Hz, not 1 kHz; the D term is unstable at that rate on BOTH roll and pitch

**Trigger:** the post-allocation-fix SITL hover run
(`log/2026-09-21/06_48_08.ulg`). Roll and yaw now hold where the vehicle
used to tumble, but pitch runs to −76°, it never climbs, and it fails
safe after 0.504 s.

**Root cause: the loop rate assumption is wrong.**
`Tools/simulation/gz/worlds/default.sdf` sets
`max_step_size = 0.004` / `real_time_update_rate = 250`. The Gazebo world
therefore steps at **250 Hz**, the IMU publishes at 250 Hz, and the rate
loop — driven by `vehicle_angular_velocity` — runs at **dt = 4 ms**, not
the "native ~1000 Hz" `FoldrotorControl.cpp:435` claims and not the 1 kHz
the cascade is documented at throughout `controller.md`. The 4 ms spacing
in `sensor_combined` is the real simulation rate, not log decimation — a
misreading that limited the previous session's analysis.

**Consequence.** The rate loop feeds back
`M = FF·e_r + I·∫e_r − D·rate_dot` with `rate_dot` =
`vehicle_angular_velocity.xyz_derivative`, low-passed at
`IMU_DGYRO_CUTOFF` (20 Hz in this log). That acceleration-feedback loop
has pole `1 − α(1 + D/I)`, `α = dt/(τ+dt)`, `τ = 1/(2π·20)`:

| dt | rate | pole, roll `D=0.5` | pole, pitch `D=0.024` |
|----|------|--------------------|------------------------|
| 1.00 ms | 1000 Hz | +0.023 | +0.021 |
| 1.25 ms | 800 Hz | −0.189 | −0.190 |
| 2.00 ms | 500 Hz | −0.759 | −0.761 |
| **4.00 ms** | **250 Hz (actual)** | **−1.929** | **−1.933** |

At the actual rate both axes are outside the unit circle on a *negative
real* pole — sign-alternating divergence. Stability needs
`D < I·(2/α − 1)`: **0.321 roll** (now 0.5) and **0.0154 pitch** (now
0.024). Both are over by the same **1.56×**.

**This contradicts `controller_params.md`'s "Roll/pitch gain split
(2026-09-21)" section and the `FR_RATE_P_D` param description**, which
state the discrete root is `−D/I` and that scaling pitch to roll's ~7.8
resolves it. That analysis assumed a 1 kHz loop and an *unfiltered*
difference. With the real 250 Hz rate and the 20 Hz filter, roll's own
ratio was never stable either — pitch was scaled to match an unstable
reference. Not yet corrected in the spec; pending the gain decision below.

**Evidence it is this and not contact or allocation.** The commanded
moments rail *before* the vehicle moves: at t=0 `M_flu` = (0.032, 0.000,
1.079) with rates ~0; by +4 ms `Mx` is at the +3.8 rail and by +8 ms `My`
is at +0.30, while ground truth still shows roll −0.56° / pitch +2.19° at
+36 ms. A closed-loop sim at dt = 4 ms with the real filter reproduces
the onset — rails in ~8 samples, sign-alternating — and shows the logged
seed (roll −0.88 rad/s) settling into a sustained bang-bang limit cycle
at the current `D`, decaying with `D` under the limit.
`sitl_testing/allocation_study/dsweep.py`.

**Still unexplained (separate seed question).** The first 4 ms jump —
rates going from ~0 to (−0.88, −0.24, +0.54) rad/s — is *not* produced by
the commanded wrench: that needs ~14 N·m against the 3.8 available. Ground
release at 22.3 N (1.46× weight) or rotor/servo spin-up reaction are the
candidates, unverified. It is a seed, not the flip mechanism — a stable
loop absorbs it.

**Not bugs, checked and cleared:**
- `Fz` = 22.30 N at arm is by design: `FR_VEL_Z_GRAV_FF` 15.26 +
  `FR_VEL_Z_FF` 7.0 × the `FR_VEL_Z_MAX_UP` 1.0 m/s cap = 22.26.
- The standing `Mz` = 1.079 N·m at zero yaw error is real and correct
  controller behaviour: EKF yaw reads +98.25° against ground truth
  +89.98°, an **8.28° estimator/declination bias**, which at
  `FR_ATT_P` 3.0 × `FR_RATE_YAW_FF` 2.5 predicts 1.079 N·m exactly. It
  costs ~28% of hover yaw authority at idle — worth a decision, not a
  flip cause.

---

## 2026-09-21 — Allocator moment block was sign-inverted; roll/pitch rate gains split

Folded into specs, detail in git history and in the study directory.

- **Allocation sign / reference-point error** (all three moment axes came
  out −1.00×/−1.31×/−1.00× of commanded, i.e. positive feedback on every
  rate loop) → `allocation.md`, "Sign / reference-point error — RESOLVED
  2026-09-21", plus its updated `Minv` literal. Guarded by
  `GeometryConstantsMatchSdfForwardKinematics` and
  `CommandedMomentProducesSameSignPhysicalMoment`.
- **Roll/pitch rate-gain split** (SDF `Iyy` = 0.0031 vs `Ixx` = 0.0645;
  shared `FR_RATE_RP_D = 0.5` gave a discrete root `−D/I` of 162 on
  pitch) → `controller_params.md`, "Roll/pitch gain split (2026-09-21)".
- **Rate-loop output limits** re-measured under the hover constraint
  (3.8 / 0.30 / 3.8 N·m, was 4.0 / 0.85 / 5.5) → derivation at the point
  of use in `FoldrotorControl.cpp`'s `parameters_updated()`; guarded by
  `RateLimitsDoNotExceedHoverMomentAuthority`.

Reproducible study (real allocator vs an SDF forward-kinematics oracle,
cross-checked against `force_moment_test.md`'s bench numbers, plus the
closed-loop sim of each candidate fix):
`src/modules/foldrotor_control/sitl_testing/allocation_study/`.

**Still open after this fix:** the `FR_RATE_P_*` defaults are scaled, not
flight-tuned; `kPosVelForceLimit`/`kPosVelForceXYMargin`/`FR_VEL_*_MAX`
remain unverified placeholders; the temporary `fr_wrench` diagnostic
(`debug_array` id=1) is still in `FoldrotorControl.cpp` and must come out
when this investigation closes.

---

## 2026-09-18 (2) — Rate-loop output limits overestimated true allocator moment authority; fix improves but does not resolve the free-flight flip

**Trigger:** continued diagnosis of the free-flight flip from the entry
below. Five reproduced crashes (headless `gz_foldrotor3`, MAVLink offboard,
same `-z 1.5` step every time) all showed the same signature: fold/tilt
(`alpha`/`beta`) rail-saturating and chattering between extremes within
~20-80 ms of arming. A temporary diagnostic (`debug_array` id=1,
`"fr_wrench"`, `FoldrotorControl.cpp`, publishing `F_alloc`/`M_alloc` —
the allocator's actual FLU input, not just its output which `id=0`
`"fr_alloc"` already carried) showed `My` (pitch moment) pegged at exactly
`±1.5` — the configured `kRateMyLimit` — alternating sign every few ms,
classic relay/bang-bang saturation behaviour.

**Diagnosis:** numerically maximized `|Mx|`/`|My|`/`|Mz|` over the full
actuator box (`F1,F2 ∈ [0,15]` N, `alpha1,2/beta1,2 ∈ [-0.79,0.79]` rad)
using `FoldrotorAllocation`'s own forward map — closed-form analytical
optimum, a brute-force grid search, and `scipy.optimize` with random
restarts, all agreeing to 5+ significant figures. Result: the configured
`kRateMxLimit`/`kRateMyLimit`/`kRateMzLimit` (8.0/1.5/6.0 N·m,
`FoldrotorControl.cpp`) all overestimate what the allocator can actually
deliver — true maxima are ~4.06/~0.90/~5.77 N·m (roll ~2.0x too high,
pitch ~1.67x too high, yaw ~1.04x, negligible). Because
`AttitudeRateControl::updateIntegral()`'s conditional-integration
anti-windup uses these same constants (via `setOutputLimits()`) to decide
"am I saturated," the rate loop believed it was within bounds for moment
commands between the true ceiling and the configured one, while the
allocator was already clamping individual channels to their rails trying
to deliver something physically unachievable — a genuine interface-
contract mismatch between the rate loop's notion of its own authority and
the allocator's real one.

**Fix applied:** `kRateMxLimit`/`kRateMyLimit`/`kRateMzLimit` set to
4.0/0.85/5.5 N·m (just under the computed true maxima, same margin-
holding pattern as `kPosVelForceLimit`/`kPosVelForceXYMargin`).

**Result — real but partial improvement, NOT a fix.** Re-tested via the
same MAVLink reproduction: armed duration before crash went from 0.756s
to 1.74s (>2x longer), and the wrench log confirms the rate loop now
correctly saturates at the new, true ceiling (`My` pegs at ±0.85, not
±1.5). But the vehicle still flips — the moment still chatters and
alternates sign continuously, now against the corrected ceiling instead
of the wrong one. Conditional-integration anti-windup only stops the
*integral* from winding up further once saturated; it does not stop the
*proportional* term from repeatedly demanding full-rail output if the
underlying rate error itself keeps oscillating in sign. That oscillation
predates this fix and is unexplained by it.

**Leading open hypothesis, not yet investigated:** roll and pitch share
one `FR_RATE_RP_FF=3.5` gain (`AttitudeRateControl::setRateGains()`), but
now that pitch's TRUE ceiling (0.85-0.90 N·m) is confirmed far tighter
than roll's (4.0-4.06 N·m), the same gain may simply be too stiff for
pitch's authority — meaning even the corrected ceiling saturates easily
under ordinary disturbance, sustaining the relay oscillation. Not
implemented or decided — a real tuning/architecture call, not made here.

**Not folded into a spec yet** — this entry is the record until the
underlying oscillation is understood or resolved.

---

## 2026-09-18 — Allocation diagnostic pin reverted; velocity-magnitude limiting added after real SITL crash

**Trigger:** re-running the real SITL hover test after the 2026-09-17
`PositionVelocityControl` rework (below) still showed both rotors pinned
at 15 N ("still both of them are needing 15 N?" / "still it is not
fixed"). Two distinct causes, found and fixed in sequence:

**(1) `FoldrotorAllocation.hpp`'s diagnostic pin was still active.** The
uncommitted "DIAGNOSTIC PIN" block flagged in the 2026-09-17 entry below
(forces `alpha1/alpha2/beta1/beta2` to 0 after `allocateRotor()` computes
them) was still in place, so the vehicle had zero tilt authority — no way
to vector thrust sideways or produce a differential roll/pitch/yaw
moment. A logged run (`.../log/2026-09-17/18_20_39.ulg` at the time)
showed force/servo-angle debug data (`fr_alloc`, published every cycle
regardless of arm state) staying flat/symmetric for ~11.5s while the
vehicle sat quietly (no destabilizing moment with F1≈F2 and no tilt),
then a small disturbance tipped it with nothing to correct it, driving
the vertical loop to keep demanding more Z force to fight the growing
position error until both rotors pinned at 15 N and the vehicle tumbled.
**Fix:** deleted the 4-line pin per its own comment's instruction
(`FoldrotorAllocation.hpp::allocate()`). All 66 then-existing
`Foldrotor*` tests pass, including the 8 `FoldrotorAllocationTest` cases
that had been failing because of the pin. Bolted-bench check (headless
`gz_foldrotor3_bench`) confirmed correct trim afterward: F1≈7.0-7.4N,
F2≈7.0-7.4N, `saturated=0`, no drift over 15s.

**(2) With tilt restored, a real free-flight run still crashed — faster
than before, not slower.** Pulled the actual `.ulg` log
(`.../log/2026-09-17/18_20_39.ulg`) rather than eyeballing the plot: the
armed window was only 0.7s (5.116s→5.848s in log time) before a
disarm/crash; the plot's apparent ~14s of continued "motion" after that
is passive post-crash tumbling plus `fr_alloc`'s always-on debug output,
not controlled flight. Within the 0.7s armed window, `alpha1`/`beta1`
were already rail-saturated at t=5.136s (20ms after arming), while
roll/pitch were still under 6° — ruling out "tips over, then the
yaw-only `Inertial2Body` approximation breaks down" as the cause; tilt
saturated before any attitude error existed. Traced with the actual
gains (`foldrotor_control_params.yaml`): `hover_setpoint.sh` commands an
unramped 1.5 m Z step; `vel_sp_z = 1.5 * FR_POS_P(3.0) = 4.5 m/s`
instantly; `force_z ≈ 4.5 * FR_VEL_Z_FF(7.0) ≈ 31.5N`, already past the
28N combined-force sphere on its own. That alone commits ~14N/rotor
undirected — already near each rotor's individual 15N ceiling — leaving
the allocator no thrust-magnitude headroom to satisfy any attitude-loop
moment demand except by tilting the already-maxed thrust vector to its
rail. This is exactly `PositionVelocityControl.hpp`'s OPEN ITEM (b)
(velocity-magnitude limiting, explicitly not ported in the 2026-09-17
rework) turning out to be load-bearing, not optional, once tested for
real. Confirmed with the user before implementing (control/tuning
decision, not made silently).

**Fix:** ported `mc_pos_control`'s `setVelocityLimits()`/`constrainXY()`
equivalent — `PositionVelocityControl::setVelocityLimits()` clamps
`vel_sp` right after the position-P step, before it reaches the velocity
PID. Horizontal is a plain magnitude scale, not the full priority-blend
logic (`ControlMath::constrainXY`'s `v1` feedforward term doesn't exist
in this module — no feedforward velocity-setpoint input path). Vertical
is asymmetric up/down (`FR_VEL_Z_MAX_UP`/`FR_VEL_Z_MAX_DN`), mirroring
`mc_pos_control`'s `MPC_Z_VEL_MAX_UP > MPC_Z_VEL_MAX_DN` convention. New
params: `FR_VEL_XY_MAX=1.0`, `FR_VEL_Z_MAX_UP=1.0`, `FR_VEL_Z_MAX_DN=0.7`
m/s — first-cut placeholders sized so `FF * limit` leaves headroom under
the sphere cap alongside `FR_VEL_Z_GRAV_FF`, **not yet verified against a
logged step response**. 5 new hand-computed unit tests added
(`FoldrotorPositionVelocityControlTest`: horizontal magnitude scaling
preserving direction, asymmetric vertical up/down clamp, default no-op).
`make tests TESTFILTER=Foldrotor`: 70/70 pass (8 `FoldrotorAllocationTest`
cases restored by the pin revert above, plus these 5 new tests).

**Folded into:** `controller.md`'s velocity-loop-form decision 1 (new
paragraph) and `controller_params.md`'s parameter table + open item 1.

**Not yet done:** real free-flight SITL re-test with both fixes in
place — the bolted-bench check only exercises the pure-hover case
(alpha/beta≈0), not a large step command with real tilt authority. The
user needs to run `hover_setpoint.sh`/`plot_hover.py` again to confirm
this closes the loop end-to-end.

---

## 2026-09-17 — PositionVelocityControl reworked to mc_pos_control's structure; anti-windup no longer uniform

**Trigger:** SITL hover testing (via `hover_setpoint.sh`/`plot_hover.py`)
showed both rotors pinned at the 15 N/rotor allocator ceiling
(`FoldrotorAllocation::kMaxThrust`) during a symmetric-thrust hover
command. Quantitative check: summing `model.sdf`'s link masses gives a
true vehicle weight of ~15.27 N (matching `FR_VEL_Z_GRAV_FF =
15.260017`), so a correct symmetric hover trim is ~7.63 N/rotor — roughly
half the observed value. The allocator itself was ruled out first
(`FoldrotorControlTest.cpp`'s `HoverProducesEvenSplitZeroTilt` already
passes, confirming `allocate(Fz=15.26, M=0)` → `F1=F2=7.63N`), pointing
at the upstream velocity loop / its saturation handling instead.

**Decision (user, this session):** stop iterating on the bespoke
Simulink-transcribed `PositionVelocityControl` design and port
`mc_pos_control`'s `PositionControl` class structure directly —
explicitly skipping its thrust-vector→attitude-setpoint conversion
(`ControlMath::thrustToAttitude`/`limitTilt`), since foldrotor3 is
fully-actuated and translates by independent rotor thrust vectoring, not
by leaning the body (already flagged as "not transferable" in
`reference/px4-module-patterns.md`). Full tradeoff discussion and the
decision record: `.claude/plans/read-mc-pos-contorl-and-can-greedy-pearl.md`.

**What changed:**
- `PositionVelocityControl.hpp`: anti-windup is no longer uniform
  conditional integration on all three axes. Z keeps conditional
  integration, now against a dynamic vertical-priority sphere-saturation
  bound (symmetric, not mc_pos_control's uni-directional convention,
  since this vehicle's tilt lets force point either up or down); X/Y
  switched to Rundqwist 1990 tracking anti-windup, simplified to compare
  desired-vs-produced force directly in newtons (no acceleration/
  hover-thrust round trip needed, since this loop's gains already act in
  the force domain). `setOutputLimits()` (independent per-axis boxes) is
  gone, replaced by `setForceLimits()`/`setHorizontalForceMargin()`.
- `Inertial2Body.hpp`: reduced from the full 3-2-1 Euler rotation to a
  yaw-only 2D rotation (roll/pitch dropped — assume level; yaw kept —
  actively controlled, confirmed varying by the 90°-yaw hover test
  earlier this session). Removes the pitch=±90° singularity concern this
  stage's own rotation used to carry (unrelated to, and does not affect,
  the still-live quaternion→Euler extraction singularity used by
  `AttitudeRateControl`).
- `FoldrotorControl.cpp`: `kPosVelFzLimit`/`kPosVelFxyLimit` (28N/20N
  independent boxes) replaced by `kPosVelForceLimit=28N`/
  `kPosVelForceXYMargin=8N` (combined sphere radius + margin) — same
  placeholder-value caveat as before, unverified against a logged clean
  hover.
- Tests: `FoldrotorPositionVelocityControlTest` (13 tests) and
  `FoldrotorControlInertial2BodyTest` (6 tests) rewritten against the new
  algorithms, hand-computed. `make tests TESTFILTER=Foldrotor` run
  2026-09-17: all 19 rewritten tests pass.

**Folded into:** `controller.md`'s velocity-loop-form decision 3 and the
Inertial2Body section; `controller_params.md`'s windup-bound open item.

**Unrelated, pre-existing, found while running the test suite — NOT
fixed here (out of scope, `FoldrotorAllocation` untouched by this
rework):** `FoldrotorAllocation.hpp::allocate()` has an uncommitted
"DIAGNOSTIC PIN (temporary, not a spec change)" block (predates this
session) that forces `alpha1/alpha2/beta1/beta2` to 0 after
`allocateRotor()` computes them, explicitly marked for reversion. This
currently fails 8 `FoldrotorAllocationTest` cases
(`HoverPlusRollMomentMatchesHandSolved`,
`RoundTripReproducesCommandedWrench` and others that expect nonzero
tilt/fold outputs). Flagged for the user to revert or resolve; not
touched by this entry's rework.

## 2026-09-14 (2) — Servo compliance under thrust load: existing bench numbers too small to explain the bang-banging; live actual-vs-commanded instrumentation added

**Question raised:** watching the "(1)" entry's hover-test bang-banging
plot, whether fold/tilt joints deflecting away from their commanded angle
under rotor-thrust reaction torque (genuine servo compliance — each joint
is driven by a bounded-effort `gz-sim-joint-position-controller-system`
PID, `p_gain=20, d_gain=0.5, cmd_max=5`, not an infinitely stiff joint) is
itself contributing to the observed swings, rather than just the upstream
attitude/saturation instability "(1)" already diagnoses.

**Existing bench data re-checked, not re-measured.** `open_loop_commands.md`
("Servo load test under motor thrust") already answers "how much will it
deflect" for straight, symmetric, held-steady thrust at the current final
gains: ~0.10 deg steady-state error at a held nonzero tilt angle, ~1.8 deg
fold cross-coupling at commanded=0 (re-analyzed this session from
`servo_load_test_logs/07_motorson_v0.6_fold_crosscoupling_pgain20_FINAL.csv`
via `analyze.py`), both under up to ~32 N combined thrust (~3.8x weight) —
essentially zero oscillation once the p_gain=20 fix landed. **This is far
smaller than the ~45 deg rail-to-rail swings in the hover plot**, so static
compliance under that loading does not explain the bang-banging by itself.

**Caveat, not resolved here:** that bench test used straight vertical
thrust and a fixed, held-steady commanded reference — not the asymmetric,
near-saturated, rapidly-changing reference the real cascade produces during
the divergence in "(1)". Whether compliance/tracking-lag is worse under
that specific loading is still open; not chased with a new bench test this
session (user's explicit choice — cite existing numbers, instrument the
real failure case instead).

**Instrumentation added instead:** `Tools/simulation/gz/models/foldrotor3/
model.sdf` (the flight model, not just the bench variant) now carries a
`gz-sim-joint-state-publisher-system` plugin (same one
`foldrotor3_bench/model.sdf` already had), publishing true joint angle on
`/world/<world>/model/<model>/joint_state`. `plot_hover.py` subscribes to
it live via `gz-transport13`'s Python bindings (confirmed importable this
session: `gz.transport13` + `gz.msgs.model_pb2.Model`, NOT `gz.msgs13`
which doesn't exist in this environment) and overlays actual
alpha1/alpha2/beta1/beta2 on the same commanded-angle subplot, sharing one
timeline so the two are comparable at the ~1-2 Hz scale the oscillation
shows up at. Purely additive telemetry — no `JointPositionController`,
physics, or control-path change.

**Not yet run:** the actual hover test with this instrumentation live —
the next step is re-running "(1)"'s bang-banging scenario and reading
whether actual tracks commanded closely (rules out compliance, points back
at "(1)"'s upstream diagnosis) or diverges by a large sustained margin
(new evidence compliance/tracking-lag is itself a real contributor). Record
that result here once run, don't over-interpret a single pass.

---

## 2026-09-14 — Output limits + rate integrator bound set; hover-test windup diagnosed as caused by inert anti-windup

**Broken:** `PositionVelocityControl` and `AttitudeRateControl` both have a
conditional-integration anti-windup mechanism gated on `setOutputLimits()`
(see each class's own header comment), but neither ever received real
bounds from `FoldrotorControl::parameters_updated()` -- both stayed at
their `+/-infinity` construction defaults. `AttitudeRateControl` also
never received `setIntegratorLimit()` (unlike `PositionVelocityControl`,
which got its Z-axis bound in "(6)" below). With both mechanisms inert,
nothing bounded the rate/velocity integrators during the 2026-09-1x hover
SITL test: sustained thrust saturation, roll diverging to -160deg and
staying there instead of correcting, and alpha1/alpha2 bang-banging at
the fold rail.

**Fixed:** in `parameters_updated()`:
- `_pos_vel_control.setIntegratorLimit(Vector3f(vel_xy_i_lim, vel_xy_i_lim,
  vel_z_i_lim))` -- the X/Y `INFINITY` placeholders replaced with the new
  `FR_VEL_XY_I_LIM` param (see the params-wiring commit that preceded
  this one).
- `_pos_vel_control.setOutputLimits()`: static box in inertial/NED (this
  class's output frame, ahead of `Inertial2Body`) -- Fz in [-28, 28] N,
  Fx/Fy in [-20, 20] N.
- `_att_rate_control.setIntegratorLimit(Vector3f(rate_rp_i_lim,
  rate_rp_i_lim, INFINITY))` -- yaw stays unbounded since `FR_RATE_YAW_I`
  is 0, nothing to bound.
- `_att_rate_control.setOutputLimits()`: static box in body/FRD (`M_b`'s
  native frame -- `Inertial2Body.hpp` only rotates the force path, never
  moments) -- Mx in [-8, 8] N*m, My in [-1.5, 1.5] N*m, Mz in [-6, 6] N*m.

All four numeric bounds are first-cut estimates derived from
`FoldrotorAllocation::kMaxThrust` (2 rotors x 15 N = 30 N ceiling, held
back with margin) and `FoldrotorAllocation`'s M0 geometry (`kS1y` ~=
0.2684 m roll arm, `kS1z` ~= 0.0301 m pitch arm) -- not a re-derivation of
`Control_Alloc.m`.

**OPEN ITEM: these four numbers are unverified placeholders.** They stop
the runaway (finite output limits make the conditional-integration
anti-windup live, and the new integrator clamps bound each accumulated
integral directly), but none has been checked against a logged clean
hover. Before trusting them past "stop the runaway": confirm the box
doesn't clip commanded force/moment during nominal hover (saturation flag
should stay false in steady flight), and confirm the integrator limits
don't themselves reintroduce steady-state error. Not attempted here.

**Bench-context check done, 2026-09-14 (armed, not flying — bolted
`gz_foldrotor3_bench`, per CLAUDE.md's verification-before-validation
order).** Added `vel_int`/`rate_int` printouts to `print_status()`.
Arm workaround: `NAV_DLL_ACT=0`/`NAV_RCL_ACT=0` (same as the Part D bench
runs), OFFBOARD via `offboard_hover.py --x 50 --altitude 10` — a large
sustained inertial-frame position error the bolted bench can never close,
driving `PositionVelocityControl` and `AttitudeRateControl` hard into
their new output-limit boxes for the whole ~15 s hold. Result: `F_b`
pinned at the box edges the whole time (`Fz ≈ -28.0`, `Fy ≈ -19.8..-19.9`
N, never past them), `vel_int` crept only to `[0, -0.34, 0]` N (well
inside the ±15/±15/±3 N limits — not frozen at exactly 0 on Y since that
axis wasn't fully saturated, consistent with conditional integration
rather than a blanket freeze) and `rate_int` stayed under 0.002 N*m
(nowhere near ±6/±6/inf) — no runaway, no repeat of the roll-divergence/
bang-banging in the bug report. Disarm-triggered reset also reconfirmed
working alongside the new bounds: `vel_int`/`rate_int` both dropped back
to ~0 immediately on `commander disarm -f`.

**Still not done:** the clean-hover check itself (saturation flag false,
no steady-state error from the box) — this run was deliberately a
saturating stress case, not a hover. That's the remaining gate before a
real SITL flight test.

---

## 2026-09-10 — Alpha unpinned; also surfaces unbounded F_b windup on the fixed bench (not fixed here)

**Folded into `allocation.md`** (alpha sign-mapping section, "α maps
directly onto the SDF joint convention" — RESOLVED 2026-09-10) and
`FoldrotorAllocation.hpp`/`FoldrotorControl.cpp` (OPEN ITEM (a) resolved,
`foldToNormalizedServo()` negation removed). Summary: the "SDF geometry,
verified 2026-09-06" claim that positive `ArmNFoldJoint` produces +Y
thrust was never actually thrust-verified and was wrong — bench-measured
(force/torque sensor, both arms, motor + own fold servo) positive
`ArmNFoldJoint` produces −Y thrust on both arms, matching
`Control_Alloc`'s own convention directly (no negation needed). Fold is
now unpinned in `FoldrotorAllocation::allocate()`, alpha clamped to
±kMaxTilt like beta, with unit tests
(`PureLateralForceProducesNonzeroAlphaCorrectSign`,
`RoundTripReproducesCommandedWrenchWithNonzeroTy`) added and verified
test-sensitive to a sign flip.

**Not folded, not fixed — flagging for the user.** Driving a sustained
lateral Offboard setpoint (`offboard_hover.py --y 3.0`, y=3m unreachable
target) against the fixed bench rig (`gz_foldrotor3_bench`, which by
design cannot move to close the position error) produced `F_b` growing
essentially unbounded over the ~15s hold: `Fx` climbed from single
digits to 622 N (>40x the vehicle's ~15 N weight) by the time of
disarm, still climbing, with `Fy` scaling alongside it. The 2026-09-09
`FR_VEL_Z_I_LIM` fix (`findings.md` (6) above) bounds the Z-axis
velocity-loop integrator specifically; there's no equivalent bound
evident on the X/Y (lateral) integrator path, or on the position-loop
output feeding into F_b generally. This was only exercised now because
alpha unpinning was the first time a lateral bench closed-loop test made
sense to run. Whether this is (a) expected/intentional on a
fixed-mount bench specifically (the vehicle physically cannot close the
error, so *some* growth is inherent to the rig, not the controller) or
(b) a real missing clamp that would also bite in free flight given a
large enough position error, is a controller-design question, not an
allocation one — out of scope for this diff, not resolved here. Do not
attempt free-flight hover with a large lateral setpoint step until this
is understood; a small/gentle setpoint (as the existing altitude-only
hover recipe uses) may not trigger it.

---

## 2026-09-09 (5) — FR_VEL_Z_GRAV_FF under-scaling fixed; confirms the fix, surfaces a separate takeoff-ramp-vs-fixed-bench mechanism

Fixed `controller.md` Open questions 3/4 per the derivation the user gave:
`p_ddot = R_b^i * F_b/m - [0,0,g]^T` requires the force-domain Z
feedforward to equal the vehicle's weight `mg`, not the raw acceleration
`g`. `FR_VEL_Z_GRAV_FF` default changed 9.81 → 15.260017 (the Part D
bench-measured static weight, "(4)" entry above — a direct measurement).
Sign left untouched: the literal is added directly and positive, with no
other scaling/sign step confirmed by reading `PositionVelocityControl.cpp`
(`force(2) += _grav_ff`) before touching anything, so the fix cannot
compound with something else. Re-derived every hand-computed
`PositionVelocityControlTest` value that depended on the old 9.81 literal
(9 assertions across 7 tests, one test renamed —
`GravityFeedforwardIsLiteralPlus981OnZOnly` →
`...LiteralWeightOnZOnly` — since its old name hardcoded the value being
changed). `make tests TESTFILTER=Foldrotor` green after the update.
Also updated `controller_params.md`'s param table and open-items section,
and `PositionVelocityControl.hpp`'s decision-4/OPEN ITEMS (a)/(b) comments,
to record both as resolved (units: under-scaled, confirmed; sign:
empirically correct as-implemented, not derived from frame reasoning —
recorded as resolved-in-practice, not resolved-in-theory, so a future
reader doesn't over-read it as a NED/Z-up derivation).

**Re-ran the identical Part D bench arm/hold sequence** (same
`NAV_DLL_ACT=0`/`NAV_RCL_ACT=0` workaround for the RTL-failsafe confound
documented in "(4)"). **The fix works exactly as predicted at the arm
edge:** the first heartbeat after `commander arm -f` read
`F_b=[-0.05, 0.02, 15.67]N` — within 0.4 N of the measured hover weight
immediately, versus climbing up from near-9.81N before the fix. This
directly confirms the diagnosis: the old FF was under-scaled by exactly
the ~5.4 N gap `FR_VEL_Z_I` used to have to make up from windup.

**But `F_b.z` still climbed to saturation over the following ~10 s**
(from 15.67 N up through 20.83 N at 4s, to 41.888 N / `F1=F2=15.000N`
saturated by ~10s) — **not reproducing the "settles near ~7.63N each"
prediction.** Traced to a different, already-partially-visible mechanism,
not the FF scaling: `listener trajectory_setpoint` during the climb showed
a sustained `velocity.z ≈ 0.7 m/s` setpoint (commander's own
"Takeoff detected" ramp, confirmed by the log line at the arm edge) — a
real climb command that the mechanically-fixed bench can never satisfy,
so the vertical position/velocity error (and thus `FR_VEL_Z_I`'s integral)
grows continuously for as long as the vehicle is armed on the stand,
independent of whether the feedforward is scaled correctly. This is the
same shape of confound as "(4)"'s RTL-failsafe finding, one level
downstream: fixing the datalink-loss failsafe stopped commander from
switching flight tasks, but did not stop the takeoff task itself from
issuing a ramping setpoint the bench cannot track.

**Disarm check clean:** bench `force.z` returned to
`-15.26001708 N` (bit-identical to baseline) immediately on `commander
disarm -f`.

**Not attempted this session, per the user's explicit scope limit:** the
integrator windup bound itself — a separate, still-pending decision. The
takeoff-ramp-vs-fixed-bench mechanism found here is a candidate cause for
why `F_b.z` still saturates on the bench even after this fix, but
confirming that (vs. some other residual cause) needs a bench test that
either suppresses the takeoff ramp (a `commander takeoff`/mode-specific
param, not identified yet) or accepts the ramp and only measures the
transient shape before saturation — a decision for whoever picks this up
next, not resolved here.

---

## 2026-09-09 (6) — FR_VEL_Z_I windup bound set (FR_VEL_Z_I_LIM = 3.0 N); plateau confirmed on the bench

Set the integrator windup bound left open by "(5)": `PositionVelocityControl`
had no mechanism to bound the accumulated integral itself, only the
still-inert (output-limit-driven) conditional integration. Added
`setIntegratorLimit()`, mirroring `AttitudeRateControl`'s existing method
of the same name/shape (clamps the accumulated integral directly, with the
same finiteness guard), and wired a new param `FR_VEL_Z_I_LIM = 3.0` N
through `FoldrotorControl::parameters_updated()` into
`setIntegratorLimit(Vector3f(INFINITY, INFINITY, 3.0f))` — Z only; X/Y stay
unbounded, no decision made there. **Checked before wiring, not assumed:**
read `PositionVelocityControl::update()` first to confirm the accumulated
integral (`_vel_int`), not some post-hoc contribution to the output, is
what needed bounding — same mechanism `AttitudeRateControl` already used,
so no new design was invented, just the missing sibling method. `params.yaml`,
`controller.md` (decision 3, in "Velocity-loop form"), and
`controller_params.md` (new table row + open-items note) updated to record
this as resolved. Added `IntegratorClampsAtFrVelZILim` to
`FoldrotorControlTest.cpp`, driving the Z integral with `e_v = 30` (well
past the 3 N bound in a single 0.1 s step) and asserting it clamps at
exactly 3.0 across two consecutive calls — the existing tests
(`IntegralAccumulatesAtZGainWithGravityFeedforward` reaching an unclamped
6.3, `IntegratorFreezesWhileSaturatedZ`'s 10.5) were correctly left
untouched because `makeSpecDefaultController()` never calls
`setIntegratorLimit()`, so the class default (±infinity) still governs
them — the new param is pushed only by the real module, same pattern as
`setOutputLimits()`. `make tests TESTFILTER=Foldrotor` green. Per the
user's explicit instruction, the (separate, still
open) conditional-integration/output-limit anti-windup mechanism was not
touched.

**Re-ran the same Part D bench arm/hold sequence** (rebuilt
`px4_sitl_foldrotor`, same `NAV_DLL_ACT=0`/`NAV_RCL_ACT=0` workaround).
`F_b.z` climbed from the arm-edge `15.75 N` and **plateaued at ~18.5–18.6 N**
within the first ~10 s, then held flat for a further 10+ s of observation
(`foldrotor_control status`: `F_b = [-0.040, -0.045, 18.581] N`, `F1=9.301N
F2=9.281N`, `saturated=0`) — hover weight (15.260017 N) plus the new 3 N
integrator ceiling, as predicted, and nowhere near the previous run's
41.888 N / `F1=F2=15.000N` saturation. The "(5)" takeoff-ramp mechanism
(`trajectory_setpoint.velocity.z ≈ 0.7 m/s` against a bench that cannot
move) is presumably still driving a real, uncorrected vertical error the
whole time — this fix does not address that, and was not asked to — but
the Z integrator's own contribution to `F_b.z` is now capped regardless of
how long that error persists, which is exactly the bound's job. Disarm
confirmed clean: bench sensor read `force.z = -15.26001708 N`, bit-identical
to the pre-arm/pre-test baseline.

**Not attempted, out of scope per the user's instruction:** the
conditional-integration/output-limit anti-windup mechanism itself (still
±infinity, still inert on all axes) and the takeoff-ramp-vs-fixed-bench
mechanism from "(5)" (still open, still unexplained beyond the diagnosis
already recorded there).

---

## 2026-09-09 (4) — Part D run: ground-plane spike does NOT reproduce on the bench

Ran the plan's Part D bench sequence for the first time (`.claude/plans/
step-4e-allocation-plan.md`). Two tooling/process issues found and worked
around, then a clean arm/disarm result that answers the 2026-09-09 (3)
ground-contact hypothesis.

**Tooling gap found, not silently fixed:** `servo_load_test_logs/
launch_bench.sh` and `force_moment_bench_commands.md` still hardcode
`build/px4_sitl_default`, but `foldrotor_control` is only compiled into
`px4_sitl_foldrotor` (`boards/px4/sitl/foldrotor.px4board`, minimal config
listing only `CONFIG_MODULES_FOLDROTOR_CONTROL=y`) — confirmed via `find`:
no `.o`/`.a` for the module anywhere under `px4_sitl_default`'s build tree.
Worked around this session by reproducing the script's exact launch
pattern (FIFO, env vars, `HEADLESS=1`, `PX4_SYS_AUTOSTART=4026`) by hand
against `build/px4_sitl_foldrotor/bin/px4` instead. The checked-in script
itself is unchanged — it needs a deliberate decision (parameterize the
build dir, or give `px4_sitl_foldrotor` the full module set and retarget
the docs) before the next person hits the same silent-no-op.

**Also found:** a stale `gz sim` server from an earlier session was still
running before this run started (`gazebo already running world: default`
on first launch attempt) — exactly the flakiness `findings.md`'s
2026-09-06 entry already warned about. Killed both stray processes and
relaunched clean before trusting any reading.

**First arm attempt was confounded, not the target measurement.**
`commander arm -f` with no GCS/RC connected triggered `NAV_DLL_ACT`/
`NAV_RCL_ACT`'s default RTL-on-loss failsafe (`navigation mode: Return`),
which commands a real climb-and-return trajectory the bench cannot
physically satisfy — that produced its own large, confounded excursion
(`F_b.z` up to 35.256 N, saturated, `torque.y` up to 1.07 N·m on the bench
sensor) that is **not** the phenomenon under test. Set `NAV_DLL_ACT=0`,
`NAV_RCL_ACT=0` for this SITL session (not persisted) and re-armed for a
clean `Hold`-mode static test — `commander status` confirmed `in failsafe:
no` throughout the real measurement below.

**The clean result: no roll/attitude spike, no failsafe disarm, on the
bench.** Across ~10 s post-arm, `M_b` stayed at `[~0.02, ~-0.04, ~0.0002]
N·m` throughout — never left the 0.01–0.04 N·m band, nowhere near the
ground-plane run's `M_b=[-4.529, ...]` — and no "Attitude failure" /
"Disarmed by failsafe" ever fired. The vehicle stayed armed in `Hold` for
the full window. This is strong evidence the ground-plane spike is a
ground-contact/liftoff artifact, not a controller defect — the leading
hypothesis in the 2026-09-09 (3) entry holds.

**`F_b.z` did still saturate (15 N/rotor, 31.76 N total), but this is the
already-documented, separate Z-axis issue,** not a new finding: `FR_VEL_Z_I`
has no windup bound (controller.md, "inert at runtime") and continued
climbing smoothly post-arm from the arm-edge-reset value, through hover
weight, to the thrust ceiling over several seconds — a slow monotonic
climb, not a step, and with zero coupling into roll/yaw. Confirms the
2026-09-09 (3) fix's own diagnosis: the integrator arm-edge reset and
geometry fix were correct and sufficient for the roll axis; the Z-integral
bound is a pre-existing, already-flagged gap, unrelated to this spike.

**`land_detected.landed` does NOT read "landed" on the bench once armed
and thrusting** — it correctly flips to `False` within the same arm
transient (`ground_contact/maybe_landed/landed` all `False`), matching what
it would report on the ground plane once thrust ramps up. So the rate-loop
integrator freeze gate is **not** a confound between the two test contexts
— both runs have it unfrozen during the relevant window. This was checked,
not assumed, per this session's explicit added objective.

**Numbers, for the record** (all `foldrotor_control status`, bench sensor
in gz FLU): static baseline `force.z = -15.260017 N` (5 decimal digits
identical pre- and post-test, confirming the fixture is sensed correctly);
settled pre-saturation reading `F_b=[0.008, 0.085, 16.334]N`,
`M_b=[0.0179,-0.0043,0.0002]N·m`, `F1=8.139N F2=8.197N`, `saturated=0`;
final saturated reading `F_b=[-0.178,-0.056,31.755]N`,
`M_b=[-0.0032,0.0061,-0.0001]N·m`, `F1=F2=15.000N`, `saturated=1`. Disarm:
bench `force.z` returned to exactly `-15.260017080000003 N` (bit-identical
to the pre-arm baseline), `actuator_outputs` motors dropped to the
disarmed convention value.

**Not done this session, still open:** no closed-loop lift-off was
attempted (out of scope, per `system.md`'s still-unchecked Milestone 1
box); the Z-integrator bound itself was not fixed (separate, pre-existing,
already-documented gap — a fix here needs a user decision on the bound
value, not a code guess); the `launch_bench.sh`/docs build-target mismatch
above was not fixed, only worked around.

---

## 2026-09-09 (3) — Both confirmed causes of the arm-time failure fixed

**Folded.** → `controller.md` (decision 6/item 8, arm-edge reset),
`allocation.md` (Status, "Geometry mismatch — RESOLVED", "The matrices").

Fix 1 — integrator arm-edge reset. `Run()` only reset
`_pos_vel_control`/`_att_rate_control`'s integrators on the armed→disarmed
edge (controller.md decision 6/item 8); there was no symmetric
disarmed→armed reset. Since the wrench (including `FR_VEL_Z_I`'s
integrator) is computed every cycle regardless of arm state, and its
anti-windup bounds are still ±infinity (inert, per controller.md), the
integral silently wound up while disarmed with no physical feedback to
correct it against — confirmed in the 2026-09-09 (2) entry as one of the
two causes of the arm-time thrust-ceiling spike. Added the symmetric
`!_armed_prev && flag_armed` reset, same place, same pattern as the
existing disarm-edge one.

Fix 2 — `M0` geometry correction. `FoldrotorAllocation.hpp`'s
`kS1y`/`kS2y`/`kS1z`/`kS2z` updated from the old `d+l_arm=0.15`/`h=0.02`
figures to the SDF-verified `±0.2684`/`+0.0301`, per allocation.md's
already-existing comparison table. `Minv` re-derives automatically at
construction (`matrix::inv<float,6>()`) — no matrix literal to hand-edit.
Deferred by user decision 2026-09-06; re-opened and fixed now because the
2026-09-09 (2) SITL run produced exactly the "roll misbehaves" symptom
this mismatch was already flagged as the first suspect for.

**Every hand-computed test value that depended on the old geometry was
re-derived, not left to silently pass or fail** (`FoldrotorControlTest.cpp`):
`kSpecMinv` (the `DerivedInverseMatchesSpecLiteral` reference — new
values computed via the same standalone-numpy-inverse method as the
original), `HoverPlusRollMomentMatchesHandSolved`,
`HoverPlusYawMomentMatchesHandSolved`,
`LateralFrdWrenchAllocatesConsistentlyWithFullFlip` (including its
partial-flip discriminator case — re-verified the test still
distinguishes a correct full FRD→FLU flip from a Z-only partial flip
under the new geometry, even though both betas are individually much
smaller now: ~1.3e-5 rad correct vs. ~0.007 rad for the partial-flip bug,
a ratio of >500), and the `TiltClampsAt...` tests' explanatory comments
(their assertions were unaffected — the unclamped beta is still beyond
±0.79 either way, just a different unclamped value: 0.807469 rad now vs.
0.927295 rad before). `HoverProducesEvenSplitZeroTilt`,
`ThrustClampsAtFifteen`, and `RoundTripReproducesCommandedWrench` needed
no changes — confirmed geometry-independent (pure-`Fz` cases go through
`M0`'s identity rows unaffected by `s_y`/`s_z`; the round-trip test is a
self-consistency check with no hardcoded geometry-dependent expectation).
`make tests TESTFILTER=Foldrotor` green after the update.

**Verified in SITL — re-ran the same arm/disarm check; NOT clean, honest
result recorded rather than a forced pass.** `mc_pos_control status`/
`control_allocator status` still report not running, `land_detector
status` still reports running, `foldrotor_control status` still reports
running with no errors, and disarm still works cleanly. But `_F_b`
still spiked to `[-14.72, 1.20, 35.68]N` with `_M_b=[-4.529, ...]` within
~1s of `commander arm -f`, followed immediately by "Preflight Fail:
Attitude failure (roll)" and "Disarmed by failsafe" — essentially the
same failure signature as before both fixes (previously ~[-10, 0, 33]N
range). The arm-edge integrator reset does fire (confirmed: it resets
exactly at the detected `!_armed_prev && flag_armed` transition, same
code path verified for the disarm edge), so the wound-up pre-arm
integral is not what's producing this specific spike — something
generates a large error within the same ~1s window *after* the reset,
too fast to be integrator windup alone.

**Not investigated further this session, per scope** (both fixes were
explicitly bounded to the arm-edge reset and the geometry correction;
this is a third, distinct mechanism). Leading candidate, not confirmed:
the vehicle rests directly on the Gazebo ground plane (unlike the
dedicated bench fixture `force_moment_bench_commands.md`/the 2026-09-06
force/moment test used, which mounts it on a stand) — going from zero
thrust (disarmed) to near-hover thrust in one control cycle from a
resting, ground-contacting state could produce a genuine liftoff/contact
transient that the attitude loop then has to reject, independent of any
integrator or geometry issue. Not verified against telemetry finer than
the 1 Hz heartbeat log, so this is a hypothesis to check, not a
diagnosis — flagged for whoever picks up Part D next, not chased here.

---

## 2026-09-09 (2) — trajectory_setpoint had no publisher; fixed, but doesn't explain the arm-time instability

**Folded.** → `controller.md` (Open question 6, new subsection), airframe
file (`flight_mode_manager start`, with reasoning inline).

The O-4 airframe diff correctly identified `flight_mode_manager` as
serving `mc_pos_control`/`mc_att_control` and left it unstarted along
with the rest of `rc.mc_apps`. That was an oversight, not a deliberate
call: `flight_mode_manager` is `foldrotor_control`'s only legitimate
`trajectory_setpoint` publisher, and without it the topic simply never
publishes — `Run()`'s `PX4_ISFINITE` gating doesn't catch this, since an
unpublished `trajectory_setpoint_s` reads as all-zero (finite), not NaN,
so the module silently computed a real wrench against a stale
`pos_sp=(0,0,0)`.

**Checked, not assumed:** `FlightModeManager.cpp` depends only on
`vehicle_control_mode`/`vehicle_status`/`vehicle_land_detected`/
`vehicle_local_position`/`vehicle_command` — no dependency on
`mc_pos_control` itself. Started it in the airframe file instead of
gating `Run()` on `flag_control_*` (the alternative fix) — smaller diff,
consistent with the existing decision that this module has no
flight-mode concept of its own (controller.md decision 6/item 8).

**One real caveat:** `flight_mode_manager` also reads `takeoff_status`,
published only by `mc_pos_control`. Without it, `_takeoff_state` is
stuck at `TAKEOFF_STATE_UNINITIALIZED` forever, so the active flight
task is `reActivate()`-ed (setpoint reset to current position/velocity)
every cycle instead of ever tracking motion. Verified via `listener`:
`trajectory_setpoint.position` tracked `vehicle_local_position.x/y`
closely at the same instant — a real, continuous hold-in-place setpoint.
Good enough for Part D's static bench sequence; not good enough for any
future translation command, which needs its own decision (something has
to publish `takeoff_status`, or this module needs an independent path).

**The actual arm-time failure was not this gap.** Re-ran the arm/disarm
SITL check with `flight_mode_manager` running, including a variant
arming within ~2s of boot to rule out a "vehicle free-fell/settled badly
before arming" explanation (ruled out: the pre-arm `_F_b.z` growth curve
and `vehicle_local_position.z` were consistent with a vehicle resting
normally, not falling). Every variant still produced `_F_b` jumping to
~30N+ and an immediate "Attitude failure (roll)" failsafe disarm within
~1s of `commander arm -f`, `listener` evidence: `vx=-2.02, vy=-3.35,
az=-25.86` within ~2s of arming despite `trajectory_setpoint` correctly
tracking real position.

Traced to two pre-existing, already-documented, already-deferred causes:
1. `FR_VEL_Z_I`'s integrator has no arm-edge reset (only disarm resets,
   controller.md decision 6/item 8) and no windup bound (anti-windup
   bounds are still ±infinity, "inert at runtime" per controller.md's
   Structure section) — it accumulates every cycle regardless of arm
   state, so it's already near hover-weight by the time of arming;
   real thrust turning on then pushes `_F_b.z` straight to the 30N
   ceiling.
2. `allocation.md`'s already-flagged `M0` geometry mismatch (s1y 0.15 m
   vs the SDF's real 0.2684 m, documented as "roll response ≈1.8×
   commanded... **first suspect if roll misbehaves**") — exactly the
   failure mode observed.

This is the first time the full cascade has ever reached real Gazebo
physics — previously blocked by O-4's publisher race, so nothing before
this session could have shown it. Not a regression from the setpoint
fix; a genuine first discovery that belongs to Part D. **Neither cause
is touched by this diff** — both require touching the cascade/allocation
math, out of scope for a startup/lifecycle fix. Arming and disarming via
the shell both still work mechanically (no ERR, clean state transitions)
— it's the physical behavior once real thrust reaches the vehicle that
isn't safe to fly yet, which was already the documented precondition for
Part D.

---

## 2026-09-09 — Airframe startup: stock mc control stack no longer starts (O-4 resolved)

**Folded.** → `allocation.md` (Status), `system.md` (Controller→Allocation
paragraph), `FoldrotorControl.cpp`/`.hpp` (`print_status()`,
`print_usage()`, class/Run() comments).

Read `rc.mc_apps` in full before touching the airframe file (flagged as
an unread gap in the original module plan). Contents: `control_allocator`,
`mc_rate_control`, `mc_att_control`, `mc_autotune_attitude_control`
(conditional `MC_AT_EN`), `vision_target_estimator` (conditional
`VTE_EN`), `mc_hover_thrust_estimator`, `flight_mode_manager`,
`mc_pos_control`, `land_detector start multicopter`, `mc_nn_control`
(conditional `MC_NN_EN`), `mc_raptor` (conditional `MC_RAPTOR_ENABLE`).
Everything except `land_detector` is either the stock controller stack
itself or a controller-specific helper (autotune/hover-thrust-estimator/
flight_mode_manager all serve `mc_pos_control`/`mc_att_control`, which
this module replaces). `land_detector` is the one line commander/arming
uses independently of the controller stack, so it's started explicitly
now. EKF2/sensors/commander are unconditional in `rcS`, not gated by
`VEHICLE_TYPE`, so they need no special handling.

Fix: `4026_gz_foldrotor3` still sources `rc.mc_defaults` (for its
non-controller defaults — `MAV_TYPE`, `IMU_GYRO_RATEMAX`, `RTL_*_ALT`,
`EKF2_RNG_FOG`) but immediately overrides `VEHICLE_TYPE` back to `none`,
so `rc.vehicle_setup` never sources `rc.mc_apps`. Smaller diff than not
sourcing `rc.mc_defaults` at all (which would mean duplicating those
params). `land_detector start multicopter` and `foldrotor_control start`
are added explicitly at the end of the airframe file.

Verified in SITL: `foldrotor_control status` running, no errors;
`mc_pos_control status`/`mc_att_control status`/`mc_rate_control
status`/`control_allocator status` all report not running; `land_detector
status` reports running (multicopter); `listener actuator_motors`/
`listener actuator_servos` show a single topic instance, no second
publisher.

**Arming observation, not chased — flagged for Part D.** `commander arm
-f` succeeded (`foldrotor_control`'s heartbeat showed `armed=1`), but the
vehicle was auto-disarmed by commander's own failsafe about 1s later
("Preflight Fail: Attitude failure (roll)", "Disarmed by failsafe") —
`_F_b`/`_M_b` spiked to tens of N / N·m in that same window. Plausible
cause: with `control_allocator` no longer racing it, `foldrotor_control`
is for the first time the only thing actually reaching the actuators,
and there is no `trajectory_setpoint` publisher running (`flight_mode_
manager` is part of `rc.mc_apps`, not started here) — `pos_sp` sits at
the struct's zero-initialized default, so if the spawned position isn't
also near zero the position error, and the wrench it produces, is large.
Not investigated further: this is exactly the Part D bench-verification
question (allocation.md Status, plan open item — armed-but-not-flying
force/moment check), which was already the documented next step, not
something this startup/lifecycle diff should resolve. Recorded so Part D
starts from this observation rather than rediscovering it.

---

## 2026-09-08 — Step 4e part 2: FoldrotorAllocation + actuator publish implemented

**Folded.** → `allocation.md` (Status, "The matrices" done-note,
Verification), `system.md` (Controller→Allocation and
Allocation→Actuators contract rows). Full design/rationale in
`.claude/plans/step-4e-allocation-plan.md`, implemented as-is: new
`src/modules/foldrotor_control/FoldrotorAllocation.hpp` (Minv derived
from M0 at init, not hardcoded), wired into `FoldrotorControl::Run()`
(unconditional publish, armed-gated NaN-on-disarm, α negated into the
fold channel per allocation.md, newtons→normalized motor curve inverted
from model.sdf, SIM_GZ_EC_MIN1/MAX1 read via param_find/param_get). 24
new tests (`FoldrotorAllocationTest`, `FoldrotorControlMappingTest`) —
closes allocation.md's Minv·M0≈I and round-trip missing tests. All 63
`TESTFILTER=Foldrotor` tests pass; `make px4_sitl_foldrotor` builds
clean.

**NOT done, and load-bearing:** the wrench sign/frame convention
(`FoldrotorAllocation.hpp` OPEN ITEM (c) / plan open item O-2) is
unresolved — `_F_b` is handed to `allocate()` exactly as the cascade
produces it, with no sign flip. This has not been checked against
Gazebo at all (plan Part D bench run not run this session) — do not
attempt closed-loop hover, or even a bench-armed check, before that
Part D sequence is run and the sign question above is settled by the
user.

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
