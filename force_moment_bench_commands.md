# foldrotor3 force/moment direction test — commands

A required deliverable of `.claude/specs/force_moment_test.md`. Results are
recorded there; this file is the procedure.

**Prerequisite.** The bench SDF edits must be applied to
`Tools/simulation/gz/models/foldrotor3_bench/model.sdf` (submodule
`Tools/simulation/gz`, branch `foldrotor3-servo-load-test`): the
`airframe_link_joint` parity fix, the `mount_plate` link, the `force_torque`
sensor, the model-scoped `gz-sim-forcetorque-system` plugin, the 0.5 m stand,
and the rotor thrust-axis fix. SDF changes need no PX4 rebuild — `gz_env.sh`
points `PX4_GZ_MODELS` at the source tree.

## Channel map

| Index | Function | Joint | Arm |
|---|---|---|---|
| `-m 1` | Motor1 | `Prop1Joint` | Arm1, body +Y |
| `-m 2` | Motor2 | `Prop2Joint` | Arm2, body −Y |
| `-s 1` | Servo1 | `Arm1FoldJoint` | Arm1, body +Y |
| `-s 2` | Servo2 | `Arm1TiltJoint` | Arm1, body +Y |
| `-s 3` | Servo3 | `Arm2FoldJoint` | Arm2, body −Y |
| `-s 4` | Servo4 | `Arm2TiltJoint` | Arm2, body −Y |

## 1. Predict before you measure

```
python3 foldrotor3_tests/expected_wrench.py --frd
python3 -m pytest foldrotor3_tests/ -q
```

Both are pure SDF geometry, no Gazebo. Signs are the pass criterion;
magnitudes are indicative (single-point quadratic thrust fit).

## 2. Build and launch

```
make px4_sitl_default
servo_load_test_logs/launch_bench.sh
```

The script takes no arguments. It runs in the foreground with
`PX4_SYS_AUTOSTART=4026 PX4_SIM_MODEL=gz_foldrotor3_bench`, feeds `pxh>` from
`/tmp/foldrotor3_bench_test/pxh_in`, and logs to `px4.log` in the same
directory.

**Wait for `Gazebo world is ready` and the `pxh>` prompt in `px4.log`.**
Commands written to the FIFO earlier are silently dropped; the symptom is a
capture that never leaves baseline.

**Kill any stale `gz sim` before relaunching.** A leftover server makes PX4
attach to the previous world — old model pose, or no sensor topic at all.

There is no `make px4_sitl gz_foldrotor3_bench` target and there should not
be. `gz_bridge/CMakeLists.txt` globs
`ROMFS/px4fmu_common/init.d-posix/airframes/*_gz_*`; the only match is
`4026_gz_foldrotor3`. The bench has no airframe file on purpose —
`PX4_SYS_AUTOSTART=4026` bypasses the model-name lookup so it reuses 4026's
params. `gz_foldrotor3_bench` is a `PX4_SIM_MODEL` value, not a make target.

## 3. Confirm the sensor is live

```
gz topic -l | grep force_torque
gz topic --echo -n 1 \
  -t /world/default/model/foldrotor3_bench_0/joint/bench_mount_joint/sensor/force_torque/forcetorque
```

The `_0` suffix is the PX4 instance index added by `px4-rc.gzsim`.

At rest, `force.z` must be **−15.26 N** in world (gz FLU): total model mass
1.5581 kg × 9.8 = 15.2698 N, less the 1 g `mount_plate` on the parent side.
**A near-zero static `force.z` means the joint is welded through rather than
sensed** — stop; do not interpret any actuator result.

## 4. Baseline is mandatory

Thrust and servo mass-redistribution are deltas on top of 15.26 N. Difference
every case against an actuators-off window from the *same* run: start the
capture, stay quiet ~4 s, then command. Verify the previous case has fully
reverted first — a still-spinning motor silently corrupts the baseline.

## 5. Capture

Run concurrently with the commands:

```
gz topic --echo --duration 60 \
  -t /world/default/model/foldrotor3_bench_0/joint/bench_mount_joint/sensor/force_torque/forcetorque \
  > capture.log
python3 servo_load_test_logs/parse_wrench.py capture.log capture.csv
```

Emits `t,fx,fy,fz,tx,ty,tz` at 50 Hz.

Rotor force oscillates, so **never judge a motor case from a single `-n 1`
sample** — take the mean over a multi-second window and report peak-to-peak
alongside it. Slice inside the command window (`cmd+4 .. cmd+7.5` for `-t 8`);
straddling the revert inflates peak-to-peak.

Frames: readings are in the `<frame>parent</frame>` orientation. `mount_plate`
and `bench_mount_joint` both have identity poses, so that is world gz FLU —
confirmed by the static `force.z` matching computed weight along world −Z, not
assumed. Apply `FLU_TO_FRD` = `diag(1,−1,−1)` (as in
`foldrotor3_tests/test_frame_convention.py`) before comparing against any
FRD expectation.

## 6. Isolation cases

One channel at a time, ~14 s apart so each reverts and settles.

```
actuator_test set -m 1 -v 0.6 -t 8
actuator_test set -m 2 -v 0.6 -t 8
actuator_test set -s 1 -v 0.5 -t 8   # Arm1FoldJoint
actuator_test set -s 2 -v 0.5 -t 8   # Arm1TiltJoint
actuator_test set -s 3 -v 0.5 -t 8   # Arm2FoldJoint
actuator_test set -s 4 -v 0.5 -t 8   # Arm2TiltJoint
```

`-v 0.6` gives ~10 N per rotor — enough signal against the 15.26 N baseline,
and already exercised safely on this bench in the servo load test.

## 7. Combined cases

```
# both motors: yaw and roll must cancel
actuator_test set -m 1 -v 0.6 -t 8
actuator_test set -m 2 -v 0.6 -t 8

# Motor1 + its own tilt servo: thrust vectoring
actuator_test set -m 1 -v 0.6 -t 8
actuator_test set -s 2 -v 0.5 -t 8
```

`actuator_test set ... -t N` is non-blocking — it schedules an internal revert
and returns immediately, so each pair queues back-to-back and runs
concurrently.

## Safety

- No tumble risk: the airframe is bolted to the world. The tumble warning in
  `open_loop_commands.md` does not apply here.
- Ground contact does invalidate results. At the original 0.1 m stand height,
  `-s 1 -v 0.5` drove `Arm1TiltLink` 18 mm through the ground plane and moved
  ~9.5 N off the mount. The stand is now 0.5 m;
  `foldrotor3_tests/test_bench_clearance.py` guards it. Re-run that test after
  any change to stand height or servo range.
