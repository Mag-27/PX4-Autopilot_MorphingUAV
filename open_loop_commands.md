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
exposed by gz-sim for this setup.
