# foldrotor_control SITL testing

## Launching SITL

The generic `simulation-gazebo` helper (what `make px4_sitl_default
gz_foldrotor3` would normally drive) uses a Gazebo server config that omits
the sensor plugins this vehicle needs. Launch manually instead:

```
GZ_SIM_SERVER_CONFIG_PATH=<path-to-px4-gz-bridge-server.config> \
PX4_SYS_AUTOSTART=4026 \
PX4_SIM_MODEL=gz_foldrotor3 \
./build/px4_sitl_default/bin/px4
```

Bring up Gazebo (`gz sim`) separately, pointed at the foldrotor world, before
or after starting `px4`.

## Watching position/orientation/commanded allocation live

`plot_hover.py` is a passive MAVLink listener -- it does not arm, switch
modes, or send setpoints. Run it in a second terminal any time after SITL is
up, while you fly/hold hover however you normally do (QGC, RC, an offboard
script):

```
python3 src/modules/foldrotor_control/sitl_testing/plot_hover.py
```

It buffers `LOCAL_POSITION_NED`, `ATTITUDE`, and the module's own
`DEBUG_FLOAT_ARRAY` allocation output in memory. When it stops receiving
data -- either because you Ctrl+C this script directly, or because you
Ctrl+C the `px4` process in its own terminal -- it opens a matplotlib window
with four stacked plots, sharing a time axis:

- position (x/y/z, meters, NED)
- orientation (roll/pitch/yaw, degrees)
- commanded force per rotor (F1/F2, newtons), with dashed reference lines
  at the `[0, 15] N` actuator limit
- commanded servo angles per rotor (alpha = fold, beta = tilt, degrees),
  with dashed reference lines at the `+-45.3 deg` tilt limit -- and, when
  available, the faint overlaid ACTUAL joint angle (see next section)

Any time window where `FoldrotorAllocation`'s `saturated` flag was set (a
channel got clamped to its limit) is shaded on the force/angle plots.

The commanded values come from `FoldrotorControl::Run()` publishing
`debug_array_s` (name `fr_alloc`, see `FoldrotorControl.hpp`/`.cpp`) --
a generic, already-registered MAVLink stream
(`MavlinkStreamDebugFloatArray` -> `DEBUG_FLOAT_ARRAY`). No PX4 core module,
mavlink included, needed a source change to get this data out.

This is a live/interactive alternative to pulling the position/orientation
signals out of the `.ulg` PX4's own logger already records (the commanded
force/servo angles are not currently in the `.ulg` -- only surfaced via this
debug stream and `foldrotor_control status`); it doesn't replace the log.

## Actual (not just commanded) servo angle

Fold/tilt joints are driven by a bounded-effort PID
(`gz-sim-joint-position-controller-system`, `Tools/simulation/gz/models/
foldrotor3/model.sdf`), not an infinitely stiff joint -- under enough
external (rotor-thrust reaction) torque the real simulated angle can lag or
sit away from the commanded one. `foldrotor3/model.sdf` also carries a
`gz-sim-joint-state-publisher-system` plugin (same one the
`foldrotor3_bench` rig already used for this, see `open_loop_commands.md`)
publishing true joint position/velocity as a `gz.msgs.Model` on
`/world/<world>/model/<model instance>/joint_state`.

`plot_hover.py` subscribes to this directly via `gz-transport13`'s Python
bindings (`gz.transport13`, `gz.msgs.model_pb2.Model` -- both need to be
importable; confirm with `python3 -c "import gz.transport13, gz.msgs"`) and
overlays the actual alpha1/alpha2/beta1/beta2 onto the same angle subplot as
the commanded traces, sharing this script's own timeline so the two line up
even at the ~1-2 Hz timescale a bang-banging oscillation shows up at.

The default `--gz-topic` guesses the model instance name
(`foldrotor3_0`) -- confirm the live one first:

```
gz topic -l | grep joint_state
python3 src/modules/foldrotor_control/sitl_testing/plot_hover.py --gz-topic /world/default/model/<name>/joint_state
```

Pass `--gz-topic ''` to fall back to commanded-only plotting (e.g. if the
SDF plugin isn't loaded, or you're deliberately isolating from Gazebo).
