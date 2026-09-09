# SITL Offboard hover recipe (foldrotor_control)

Manual launch recipe for exercising `foldrotor_control` end-to-end in Gazebo
SITL over MAVLink Offboard, without going through
`Tools/simulation/gz/simulation-gazebo`. Recorded because the sensor-plugin
gap below cost real effort to diagnose.

## Why not `make px4_sitl_default gz_<model>` or the `simulation-gazebo` helper directly

Both work, but if you ever launch `gz sim` by hand (e.g. to attach a debugger
to `px4` separately, or to inspect the server independently), the
`simulation-gazebo` Python helper downloads a **generic** server config to
`~/.simulation-gazebo/server.config`, which only loads `Physics`,
`UserCommands`, and `SceneBroadcaster`. IMU/baro/mag never publish under that
config — not because of rendering, and not because the world is paused, but
because the `Imu`/`AirPressure`/`Magnetometer`/`Sensors` system plugins were
never loaded. PX4's own launch path (`gz_env.sh.in`) sets
`GZ_SIM_SERVER_CONFIG_PATH` to `src/modules/simulation/gz_bridge/server.config`,
which declares all of them. Any hand launch of `gz sim` must set that same
variable explicitly.

## 1. Launch the Gazebo server

```bash
REPO=/home/magesvarlinux/PX4-Autopilot
export GZ_SIM_SERVER_CONFIG_PATH="$REPO/src/modules/simulation/gz_bridge/server.config"
export GZ_SIM_RESOURCE_PATH="$REPO/Tools/simulation/gz/models:$REPO/Tools/simulation/gz/worlds:$GZ_SIM_RESOURCE_PATH"
cd "$REPO"
gz sim -s -r -v 3 "$REPO/Tools/simulation/gz/worlds/default.sdf"
```

`-r` starts the world running (not paused) immediately; without it the
server sits paused until something unpauses it. Confirm sensors are actually
flowing, not just the clock:

```bash
gz topic -e -t /clock -n 2                                   # sim time ticking
gz topic -e -t /world/default/model/<model>/link/<link>/sensor/imu_sensor/imu -n 1
```

## 2. Launch PX4 SITL against it

Use `gz_foldrotor3` for free flight (the vehicle can fall) or
`gz_foldrotor3_bench` for a rig-mounted test where altitude/vz cannot be
trusted as a controller signal — the bench rig holds the vehicle up
regardless of what the controller commands.

```bash
PX4_SYS_AUTOSTART=4026 \
PX4_SIM_MODEL=gz_foldrotor3 \
PX4_GZ_WORLD=default \
GZ_IP=127.0.0.1 \
HEADLESS=1 \
./build/px4_sitl_foldrotor/bin/px4
```

(Feed stdin via a named FIFO + a `sleep infinity` holder if driving the pxh
shell non-interactively; recreate both the FIFO and the holder together, or a
new launch can end up writing to an orphaned FIFO with no reader.)

At the pxh prompt, disable the RC-loss and data-link-loss failsafes for a
headless run with no RC and only a MAVLink offboard link (there is a
`NAV_DLL_ACT` default in the airframe file already, but Offboard testing
with no telemetry heartbeat guarantees needs both off):

```
param set NAV_DLL_ACT 0
param set NAV_RCL_ACT 0
```

## 3. Drive an Offboard hover

```bash
python3 src/modules/foldrotor_control/sitl_testing/offboard_hover.py \
    --altitude 2.0 --hold-seconds 60 --log /tmp/hover_log.csv
```

This streams `SET_POSITION_TARGET_LOCAL_NED` (position-only type mask) for
2s, requests `OFFBOARD` (PX4 custom main mode 6), arms, then keeps streaming
for `--hold-seconds`. The setpoint stream must not stop before or during the
mode switch and arm, or PX4 will refuse/exit Offboard on timeout.

Useful live signals while it runs, from the pxh shell or `listener`:

- `foldrotor_control`'s own periodic log line: `armed=`, `offboard=`,
  `position_ctrl=`, `F_b=[...]N`, `M_b=[...]`
- `listener vehicle_local_position` — `z`, `vz`, drift in `x`/`y`
- `listener vehicle_attitude` — roll/pitch for divergence/oscillation

Note `vehicle_attitude_setpoint` is **not** published by this module (it
computes/consumes attitude error internally without republishing to the
standard topic), so it is not available as a thrust/attitude-setpoint signal
source.

## 4. Clean up

Kill the `gz sim` server, the `px4` process, and any FIFO-holder `sleep
infinity` process explicitly by PID — `pkill -f "gz sim"` pattern matches can
miss the actual process depending on how it was invoked.
