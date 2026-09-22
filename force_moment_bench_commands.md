# foldrotor3 force/moment direction test — commands

This file is itself a required deliverable of `.claude/specs/force_moment_test.md`
(see that spec's Deliverables section) — not just documentation of one.
It lists the exact commands to build, launch, drive, and capture the
fixed-mount bench force/torque sensor test.

Sibling to `open_loop_commands.md` in location and style; reuses that
file's bench-launch pattern rather than introducing new tooling.

**Prerequisite**: the SDF edits specified in `force_moment_test.md` must be
applied to `Tools/simulation/gz/models/foldrotor3_bench/model.sdf`
(submodule `Tools/simulation/gz`, branch `foldrotor3-servo-load-test`):
the `airframe_link_joint` geometry-parity fix, the `mount_plate` link that
gives `bench_mount_joint` a real parent link, the `force_torque` sensor,
the model-scoped `gz-sim-forcetorque-system` plugin, and the raised stand
height. No PX4 rebuild is needed for SDF changes — `gz_env.sh` points
`PX4_GZ_MODELS` at the source tree.

## Channel map

| `actuator_test` index | Function | Joint | Arm position |
|---|---|---|---|
| `-m 1` | Motor1 | `Prop1Joint` | Arm1, body +Y |
| `-m 2` | Motor2 | `Prop2Joint` | Arm2, body −Y |
| `-s 1` | Servo1 | `Arm1FoldJoint` | Arm1, body +Y |
| `-s 2` | Servo2 | `Arm1TiltJoint` | Arm1, body +Y |
| `-s 3` | Servo3 | `Arm2FoldJoint` | Arm2, body −Y |
| `-s 4` | Servo4 | `Arm2TiltJoint` | Arm2, body −Y |

## Expected values before you measure

Derive the expectation first, so the measurement can falsify it rather than
be rationalised after the fact:

```
python3 foldrotor3_tests/expected_wrench.py --frd
```

Pure SDF geometry, no Gazebo. Signs are the deliverable; magnitudes are
indicative only (single-point quadratic thrust fit). The interface-level
geometry guards run without Gazebo too:

```
python3 -m pytest foldrotor3_tests/ -q
```

## Build and launch

```
make px4_sitl_default
```

Reuse the existing bench-launch pattern verbatim — no new launch tooling:

```echo 'actuator_test set -m 1 -v 0.6 -t 8' > /tmp/foldrotor3_bench_test/pxh_in
servo_load_test_logs/launch_bench.sh [work_dir] [gz_model]
# defaults: work_dir=/tmp/foldrotor3_bench_test, gz_model=gz_foldrotor3_bench
```

This blocks in the foreground (`PX4_SYS_AUTOSTART=4026`,
`PX4_SIM_MODEL=gz_foldrotor3_bench` under the hood), feeding `pxh>` from
`$work_dir/pxh_in` and logging to `$work_dir/px4.log`. Drive it from
another shell by writing to that FIFO, as in `open_loop_commands.md`:

```
echo 'actuator_test set -m 1 -v 0.6 -t 8' > /tmp/foldrotor3_bench_test/pxh_in
```

**Wait for `Gazebo world is ready` and the `pxh>` prompt in `px4.log` before
sending anything.** Commands written to the FIFO before PX4 reaches the
prompt are silently lost — a capture full of unchanging baseline is the
symptom.

## Confirm the sensor is live

```
gz topic -l | grep force_torque
gz topic --echo -t /world/default/model/foldrotor3_bench_0/joint/bench_mount_joint/sensor/force_torque/forcetorque -n 1
```

Note the instance suffix: `px4-rc.gzsim` spawns the model as
`${MODEL_NAME}_${px4_instance}`, so the topic carries `foldrotor3_bench_0`,
not `foldrotor3_bench`.

At rest the reading must be `force.z ≈ −15.26 N` in world (gz FLU) — the
airframe's own weight, everything on the child side of `bench_mount_joint`.
`mount_plate` (1 g) sits on the parent side and is correctly excluded:
total model mass 1.5581 kg × 9.8 = 15.2698 N, minus 0.0098 N, = 15.2600 N.
**A near-zero static `force.z` means the fixture is welded through rather
than sensed** — stop and investigate; do not interpret any actuator result.

## Baseline is mandatory

The mount carries the full static weight and the static moment of the CoM.
Thrust and servo mass-redistribution are *deltas* on top of ~15.26 N. Every
case must be differenced against an actuators-off window captured in the
same run — start the capture, leave ~4 s quiet, then command.

## Per-actuator test commands (isolation)

One channel active at a time. **Safety note**: unlike the untethered flight
model there is no tumble risk here (the airframe is bolted to the world),
but real reaction load still passes through `bench_mount_joint` and through
whichever servo is active.

```
# motors -- -v 0.6 (~10 N per rotor) for signal-to-noise against the
# 15.26 N gravity baseline; already exercised safely on this bench in the
# servo load test.
actuator_test set -m 1 -v 0.6 -t 8
actuator_test set -m 2 -v 0.6 -t 8

# fold/tilt servos -- position-controlled, no tumble/overload risk
actuator_test set -s 1 -v 0.5 -t 8   # Arm1FoldJoint
actuator_test set -s 2 -v 0.5 -t 8   # Arm1TiltJoint
actuator_test set -s 3 -v 0.5 -t 8   # Arm2FoldJoint
actuator_test set -s 4 -v 0.5 -t 8   # Arm2TiltJoint
```

Leave ~14 s between cases so the previous one reverts and settles.

## Combined-actuator test commands

```
# both motors together (yaw-cancellation case)
actuator_test set -m 1 -v 0.6 -t 8
actuator_test set -m 2 -v 0.6 -t 8

# one motor + its own tilt servo (coupled-axis case), Motor1 + Servo2
actuator_test set -m 1 -v 0.6 -t 8
actuator_test set -s 2 -v 0.5 -t 8
```

`actuator_test set ... -t N` is non-blocking — it schedules an internal
revert-to-off after N seconds and returns the prompt immediately, so the
two commands in each combined case can be queued back-to-back to run
concurrently, as in `open_loop_commands.md`'s servo load test.

## Reading the sensor during a test window

Capture concurrently with the actuator commands, mirroring the existing
`joint_state` capture pattern:

```
gz topic --echo -t /world/default/model/foldrotor3_bench_0/joint/bench_mount_joint/sensor/force_torque/forcetorque \
    --duration 60 > capture.log
python3 servo_load_test_logs/parse_wrench.py capture.log capture.csv
```

`parse_wrench.py` emits `t,fx,fy,fz,tx,ty,tz`. The sensor publishes at
50 Hz.

**Sample rate matters for the motor cases.** The rotor-induced force is not
static — see `force_moment_test.md`'s recorded result. Comparing a single
`-n 1` sample against an expectation will mislead; take the mean over a
multi-second window and also report peak-to-peak.

Frame handling, per `force_moment_test.md`'s methodology step 5:

1. Readings are in the `<frame>parent</frame>` orientation. `mount_plate`
   and `bench_mount_joint` both have identity poses, so this is the world
   (gz FLU) orientation — confirmed empirically, not assumed, by the static
   `force.z = −15.26 N` matching the computed weight along world −Z.
2. Apply `FLU_TO_FRD` (`diag(1,-1,-1)`, as used in
   `foldrotor3_tests/test_frame_convention.py`) before comparing to any
   FRD-derived expectation.

## Safety notes

- No tumble/flip risk on this fixed bench — the tumble warning in
  `open_loop_commands.md`'s Motors section does not apply here.
- Ground contact *does* invalidate results. At the original 0.1 m stand
  height, `-s 1 -v 0.5` drove `Arm1TiltLink` 18 mm through the ground plane
  and moved ~9.5 N off the mount. The stand is now at 0.5 m and
  `foldrotor3_tests/test_bench_clearance.py` guards it; re-run that test
  after any change to the fixture height or the servo range.
