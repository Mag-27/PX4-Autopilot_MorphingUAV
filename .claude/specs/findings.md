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
