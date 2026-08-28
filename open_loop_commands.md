# foldrotor3 open-loop actuator testing

Reference commands for exercising the `foldrotor3` SITL airframe
(`4026_gz_foldrotor3`) open-loop via `actuator_test`, bypassing the control
allocator entirely (`CA_AIRFRAME` is set to generic Multirotor, not Custom —
no `ActuatorEffectivenessFoldrotor` in the loop for this pass).

## Build and launch

```
make px4_sitl_default
PX4_SYS_AUTOSTART=4026 PX4_GZ_MODEL=foldrotor3 make px4_sitl gz_foldrotor3
```

Wait for the `pxh>` prompt, then run the commands below from there.

## Channel map

| `actuator_test` index | Function | Joint | gz command topic |
|---|---|---|---|
| `-m 1` | Motor1 | `Prop1Joint` | `/foldrotor3_0/command/motor_speed` (velocity[0]) |
| `-m 2` | Motor2 | `Prop2Joint` | `/foldrotor3_0/command/motor_speed` (velocity[1]) |
| `-s 1` | Servo1 | `Arm1FoldJoint` | `/model/foldrotor3_0/servo_0` |
| `-s 2` | Servo2 | `Arm1TiltJoint` | `/model/foldrotor3_0/servo_1` |
| `-s 3` | Servo3 | `Arm2FoldJoint` | `/model/foldrotor3_0/servo_2` |
| `-s 4` | Servo4 | `Arm2TiltJoint` | `/model/foldrotor3_0/servo_3` |

## Motors

`-v` is `-1..1`, mapped linearly onto the raw `[SIM_GZ_EC_MIN, SIM_GZ_EC_MAX]
= [308, 2054]` range, which PX4 publishes straight through as the commanded
rad/s (no further unit conversion). `-v 1` = 2054 rad/s, the real
100%-throttle point derived from the T-Motor F90 KV1300 + HQ 70403 test data.

**Start low and short.** Real thrust from these constants (~7-23N per motor
at 6S) will flip an untethered, unstabilized airframe at moderate-to-high
`-v` values — this has already crashed gz-sim once in testing (an ODE
collision-bounds overflow from the resulting tumble).

```
# one motor at a time, low value, short window
actuator_test set -m 1 -v 0.1 -t 2
actuator_test set -m 2 -v 0.1 -t 2

# both together (symmetric thrust, less tumble torque), stepping up
actuator_test set -m 1 -v 0.2 -t 3
actuator_test set -m 2 -v 0.2 -t 3
```

Don't try to reverse-map the T-Motor bench-test throttle percentages to `-v`
values expecting a match — the sim's thrust model is a single-point
quadratic fit anchored at 100% throttle, not a faithful reproduction of the
real motor's nonlinear curve at partial throttle.

## Fold/tilt servos

`-v` maps onto `SIM_GZ_SV_MINA{n}/MAXA{n} = ∓45.26°`, matching the `±0.79`
rad joint limit in `model.sdf`.

```
actuator_test set -s 1 -v 0.5 -t 5   # Arm1FoldJoint
actuator_test set -s 2 -v 0.5 -t 5   # Arm1TiltJoint
actuator_test set -s 3 -v 0.5 -t 5   # Arm2FoldJoint
actuator_test set -s 4 -v 0.5 -t 5   # Arm2TiltJoint
```

These are position-controlled (`gz-sim-joint-position-controller-system`),
not velocity-controlled like the motors — no tumble risk from testing them
individually.

## Watching the response

```
# live commanded motor velocities (rad/s), both motors in one message
gz topic --echo -t /foldrotor3_0/command/motor_speed

# live commanded servo angle (rad) for a specific joint, e.g. Arm1FoldJoint
gz topic --echo -t /model/foldrotor3_0/servo_0

# list all advertised topics for this model
gz topic -l | grep foldrotor3_0

# static joint info (axis, parent/child link) -- NOT live angle/position
gz model -m foldrotor3_0 -j Prop1Joint

# usage/help
actuator_test
```

For live joint *position* rather than just the commanded value, subscribe to
`/world/default/dynamic_pose/info` and diff a link's orientation quaternion
against a known baseline — there's no direct "current joint angle" topic
exposed by gz-sim for this setup. (Update: for the bench-test model below, a
`gz-sim-joint-state-publisher-system` plugin gives a much simpler path to the
same data — see the next section.)

## Servo load test under motor thrust

Fold/tilt servos had only been exercised in isolation (motors off). This
section validates the `JointPositionController` gain
(`p_gain`/`i_gain`/`d_gain` in `model.sdf`) against gyroscopic/aerodynamic
load from spinning motors, since the eventual `ActuatorEffectivenessFoldrotor`
wrench allocator assumes commanded tilt angle ≈ actual tilt angle.

### Bench-test fixture

Spinning both motors at a nontrivial throttle on the untethered `foldrotor3`
model risks a tumble (real thrust at even moderate `-v` exceeds the
airframe's own weight — see the Motors section above, and this has already
crashed gz-sim once from an ODE collision-bounds overflow). To test under
load safely, `Tools/simulation/gz/models/foldrotor3_bench/model.sdf` is a
copy of the flight `model.sdf` with two additions (**not** present in the
flight file):

- `<joint name='bench_mount_joint' type='fixed'><parent>world</parent>
  <child>base_link</child></joint>` — rigidly bolts `base_link` (and,
  transitively, `airframe_link`) to the world. All prop/fold/tilt revolute
  joints stay free; only the airframe body is pinned, so it cannot tumble
  regardless of thrust.
- A `gz-sim-joint-state-publisher-system` plugin listing the four fold/tilt
  joints, publishing actual joint angle/velocity on
  `/world/<world>/model/<model>/joint_state` (a `gz.msgs.Model` message,
  one `joint { axis1 { position: ... velocity: ... } }` block per joint).
  This is a much more direct way to read back actual angle than the
  dynamic_pose/info quaternion-diffing described above, and only needed for
  this bench rig since the flight model has no such readback.

It reuses the flight model's meshes via `model://foldrotor3/meshes/...`
(resolved because both model directories sit on the same
`GZ_SIM_RESOURCE_PATH`), so nothing needed duplicating there.

### Launching the bench rig

`PX4_SYS_AUTOSTART=4026` bypasses the `PX4_SIM_MODEL` -> airframe-filename
lookup in `rcS`, so `PX4_SIM_MODEL` can point at `gz_foldrotor3_bench` (no
matching airframe file exists, or needs to) while still loading airframe
4026's params. This is what `make px4_sitl gz_foldrotor3` does under the
hood (see `src/modules/simulation/gz_bridge/CMakeLists.txt`), just with an
explicit model name instead of a generated make target:

```
servo_load_test_logs/launch_bench.sh [work_dir] [gz_model]
# defaults: work_dir=/tmp/foldrotor3_bench_test, gz_model=gz_foldrotor3_bench
```

This blocks in the foreground, feeding `pxh>` from `$work_dir/pxh_in` (a
FIFO) and logging to `$work_dir/px4.log`. Drive it from another shell:

```
echo 'actuator_test set -s 2 -v 0.5 -t 6' > /tmp/foldrotor3_bench_test/pxh_in
```

`actuator_test set ... -t N` is non-blocking — it schedules an internal
revert-to-off after N seconds and returns the prompt immediately, so
multiple commands (e.g. both motors, then a servo step) can be queued in
quick succession to run concurrently.

Capture the joint_state topic for the test window and extract one joint's
`axis1.position` to CSV:

```
gz topic --echo -t /world/default/model/foldrotor3_bench_0/joint_state \
    --duration 10 > capture.log
python3 servo_load_test_logs/parse_joint_state.py capture.log Arm1TiltJoint out.csv
python3 servo_load_test_logs/analyze.py out.csv <commanded_rad>
```

`analyze.py` reports step-onset time, peak/overshoot, 5%-band settling time,
and steady-state mean/error/oscillation (peak-to-peak over the last 1s).

### Test and results

Servo2 (`Arm1TiltJoint`) commanded to `-v 0.5` (+22.63 deg / 0.395 rad) via
`actuator_test`, motors either off or spun up ~1.5s beforehand (RPM settles
in <0.1s given `timeConstantUp=0.0125s`, well before the servo step) and
held for the rest of the window:

| condition | steady-state error | oscillation (pk-pk) | settling time | overshoot |
|---|---|---|---|---|
| motors off, p_gain=10 | +0.21 deg | 0.00 deg | 0.16 s | +0.21 deg |
| motors on, `-v 0.3` (~23N combined, ~2.7x weight), p_gain=10 | +0.21 deg | 0.46 deg | 0.16 s | +0.43 deg |
| motors on, `-v 0.6` (~32N combined, ~3.8x weight), p_gain=10 | +0.20 deg | 0.78 deg | 0.19 s | +0.59 deg |
| motors on, `-v 0.6`, p_gain=10, **d_gain=1.0** | **-9.3 deg** | **1.65 deg** | never | (unstable) |
| motors on, `-v 0.6`, **p_gain=20**, d_gain=0.5 (final) | +0.10 deg | 0.00 deg | 0.08 s | +0.10 deg |

Steady-state error was essentially unaffected by motor load at the default
gain (+0.2 deg either way) — the load instead showed up as a small
motor-vibration-driven limit cycle riding on top of the held position
(0 → 0.78 deg pk-pk from 0 to ~69% commanded rotor speed), plus weaker
cross-coupling onto the fold axis (`Arm1FoldJoint`, commanded to hold 0 deg):
0.26 deg pk-pk at `-v 0.3`.

**Raising `d_gain` made this worse, not better.** At `d_gain=1.0` under
`-v 0.6` load, differentiating the vibration-frequency noise apparently
saturated the position controller's `cmd_max=5` effort limit: the joint
settled into a sustained ~60Hz limit cycle around a mean ~9 deg away from
the command, never converging. Raw data:
`servo_load_test_logs/04_motorson_v0.6_tilt_pgain10_dgain1.0_FAILED.csv`.
More damping is the wrong direction for a high-frequency vibration
disturbance like this one.

**Raising `p_gain` to 20 (leaving `d_gain=0.5`) fixed it instead**: the
limit cycle disappeared entirely (0.00 deg pk-pk on both the tilt and the
cross-coupled fold joint at the worst-throttle case tested), steady-state
error dropped to +0.10 deg, and it settled faster with less overshoot than
even the original motors-off case. Verified this didn't regress the
motors-off case (peak/overshoot unchanged at +0.10 deg) and that the flight
`foldrotor3` model still spawns and responds to servo commands cleanly with
the new gain (motors off, untethered — not load-tested untethered, for the
tumble-risk reasons above).

**`p_gain=20` (d_gain/i_gain/cmd_max/cmd_min unchanged) has been applied to
all four servo plugins in the committed `model.sdf`** (and to
`foldrotor3_bench/model.sdf`, so the bench rig stays representative of the
flight file). Rationale is recorded in-line in `model.sdf`'s plugin comment.

Raw per-run CSVs (time, position, velocity) and the scripts above are in
`servo_load_test_logs/`.
