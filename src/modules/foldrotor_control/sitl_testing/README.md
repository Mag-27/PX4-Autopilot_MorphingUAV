# foldrotor_control SITL testing

How to fly `foldrotor_control` end-to-end in Gazebo SITL over MAVLink
Offboard, and plot the result. Restored from the recipe recorded in
`9754caa956` / `531362faec`, updated for the scripts that exist now
(`offboard_hover.py` and `plot_hover.py` are gone; their jobs are done by
`setpoint_trajectory.sh` and `plot_sitl_log.py`).

## 0. On a fresh machine

```bash
git clone --recursive -b Mag-27/foldrotor-control-skeleton-review \
    https://github.com/Mag-27/PX4-Autopilot_MorphingUAV.git PX4-Autopilot
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh          # PX4 toolchain + Gazebo, first time only
python3 -m pip install --user pymavlink pyulog matplotlib
```

The vehicle models (`foldrotor3`, `foldrotor3_bench`) and the `foldrotor`
world live in the `Tools/simulation/gz` submodule, which points at the
`Mag-27/PX4-gazebo-models` fork. In an existing clone, after pulling run
`git submodule sync --recursive && git submodule update --init --recursive`
-- without `sync` the old upstream URL is kept and the pinned commit cannot
be fetched.

## 1. Launch SITL

```bash
make px4_sitl_default gz_foldrotor3               # with the Gazebo GUI
HEADLESS=1 make px4_sitl_default gz_foldrotor3    # without
```

`foldrotor_control` is compiled into `px4_sitl_default`
(`boards/px4/sitl/default.px4board`). Airframe `4026_gz_foldrotor3` loads
the `foldrotor` world (gravity exactly -9.8) and, because it sets
`VEHICLE_TYPE none` to keep the stock mc controllers out, starts
`land_detector`, `flight_mode_manager` and `foldrotor_control` itself.

Checked 2026-09-25 on the dev machine: the log shows
`Starting gazebo with world: .../worlds/foldrotor.sdf` and
`gz_bridge ... model: foldrotor3_0`; at the `pxh>` prompt
`foldrotor_control status` reports the cascade running, `listener
sensor_accel` publishes, and `vehicle_local_position` is `xy_valid`/
`z_valid`. The `gz_frame_id ... not defined in SDF` warnings on spawn are
harmless.

**Kill any stale Gazebo server first.** A `gz sim` left running from an
earlier session makes the next launch attach to the wrong world. Check with
`ps -eo pid,cmd | grep -E 'gz sim|bin/px4' | grep -v grep` and kill by PID.

### Launching `gz sim` by hand

Only needed to run the server separately (e.g. to debug `px4` on its own).
The `simulation-gazebo` Python helper uses a generic server config that
loads only `Physics`, `UserCommands` and `SceneBroadcaster`, so IMU, baro
and mag never publish. PX4's own launch path sets
`GZ_SIM_SERVER_CONFIG_PATH` to `src/modules/simulation/gz_bridge/server.config`,
which loads them; a hand launch must do the same:

```bash
REPO=$(git rev-parse --show-toplevel)
export GZ_SIM_SERVER_CONFIG_PATH="$REPO/src/modules/simulation/gz_bridge/server.config"
export GZ_SIM_RESOURCE_PATH="$REPO/Tools/simulation/gz/models:$REPO/Tools/simulation/gz/worlds:$GZ_SIM_RESOURCE_PATH"
gz sim -s -r -v 3 "$REPO/Tools/simulation/gz/worlds/foldrotor.sdf"   # -r: start unpaused

# second terminal
PX4_SYS_AUTOSTART=4026 PX4_SIM_MODEL=gz_foldrotor3 PX4_GZ_WORLD=foldrotor \
GZ_IP=127.0.0.1 HEADLESS=1 ./build/px4_sitl_default/bin/px4
```

Confirm sensors are flowing, not just the clock:
`gz topic -e -t /clock -n 2`, then `gz topic -l | grep imu` and echo one.

## 2. Disable the link-loss failsafes (headless, no GCS/RC)

At the `pxh>` prompt:

```
param set NAV_DLL_ACT 0
param set NAV_RCL_ACT 0
```

With these at their defaults, an arm with no GCS/RC trips an RTL failsafe
whose climb-and-return confounds the run (`findings.md` 2026-09-09 (4)).

## 3. Fly it

Both scripts stream Offboard setpoints over `udpin:0.0.0.0:14540`
(`-c` to change), switch to OFFBOARD, arm, and print tracking statistics.
`--help` lists every option.

```bash
cd src/modules/foldrotor_control/sitl_testing

./setpoint_trajectory.sh -s hold -t 60            # stationary hover
./setpoint_trajectory.sh                          # 1 m step at 1.5 m (default)
./setpoint_trajectory.sh -s box -d 1.5 -t 120     # also: ramp circle figure8 climb

./attitude_setpoint_trajectory.sh                 # hover + 10 deg pitch/roll legs
./attitude_setpoint_trajectory.sh -p 5 -r 5       # gentler
```

Follow the verification order in `.claude/CLAUDE.md`: `hold` before
`step`/`box`, and attitude legs from `hold` before combining them with
translation. Read the header of `attitude_setpoint_trajectory.sh` before
interpreting its results -- yaw 0 is a 90 deg turn at this spawn, a
one-shot attitude command is a 500 ms pulse, and the position and attitude
messages compete for the Offboard mode.

Useful live signals from `pxh>`: `foldrotor_control status` (F_b, M_b,
integrators, rotor commands, `saturated`), `listener vehicle_local_position`,
`listener vehicle_attitude`.

## 4. Plot the run

The logger closes the `.ulg` on disarm or PX4 shutdown, and the scripts
leave the vehicle armed -- disarm (`commander disarm`) or `shutdown` first.

```bash
./plot_sitl_log.py                  # newest log under build/px4_sitl_default/rootfs/log
./plot_sitl_log.py path/to/run.ulg
./plot_sitl_log.py --watch          # plot each log as it is closed
./plot_sitl_log.py --show           # also open a window
```

Writes one PNG next to each `.ulg`: position (with setpoint), attitude,
commanded fold/tilt angles and commanded rotor thrusts, with allocator
clamps shaded. The angle and thrust panels come from the module's
`fr_alloc` debug_array, logged because the SITL `SDLOG_PROFILE` includes
the debug bit.

## 5. Clean up

Stop `px4` with `shutdown` (or Ctrl+C) and check no `gz sim` server is left
behind (see step 1). Kill by PID; `pkill -f "gz sim"` can miss the process
depending on how it was started.
