# PX4 Module Patterns

Durable reference knowledge — how PX4 itself wants a standalone module
built, scheduled, and wired. Drawn from reading the stock multicopter
control stack (`mc_pos_control`, `mc_att_control`, `mc_rate_control`,
`control_allocator`) as a **structural** reference only, not a
control-design reference: the target vehicle is fully actuated
(independent per-rotor tilt); the stock quad stack is underactuated. Every
item below is annotated as either a **PX4 convention** (required to
interoperate with commander, logger, mavlink, uORB consumers) or a
**stylistic choice** (free to differ on). This does not go stale with the
foldrotor3 project's progress — read it on demand when writing or
reviewing PX4 module code, not as project status.

Files read: `src/modules/mc_pos_control/MulticopterPositionControl.{hpp,cpp}`,
`src/modules/mc_att_control/mc_att_control.hpp` +
`mc_att_control_main.cpp`, `src/modules/mc_rate_control/MulticopterRateControl.{hpp,cpp}`,
`src/modules/control_allocator/ControlAllocator.{hpp,cpp}`,
`src/lib/control_allocation/**`, `msg/VehicleTorqueSetpoint.msg`,
`msg/VehicleThrustSetpoint.msg`, `msg/versioned/ActuatorMotors.msg`,
`msg/ControlAllocatorStatus.msg`, CMakeLists/Kconfig for all three
controllers, `boards/px4/sitl/default.px4board`,
`platforms/common/include/px4_platform_common/px4_work_queue/WorkQueueManager.hpp`.

## 1. Module lifecycle — ModuleBase/ModuleParams/ScheduledWorkItem

All three controllers and `control_allocator` follow the identical shape:

- Class inherits `ModuleBase`, `ModuleParams`, and either
  `px4::ScheduledWorkItem` (mc_pos_control, control_allocator — self-paced,
  can `ScheduleDelayed`/`ScheduleNow`) or `px4::WorkItem` (mc_att_control,
  mc_rate_control — purely callback-driven, no self-scheduling).
- Static `Descriptor desc{task_spawn, custom_command, print_usage};` at
  file scope (e.g. `MulticopterPositionControl.cpp:44`).
- `task_spawn()` — allocates the instance with `new`, stores it in
  `desc.object`, sets `desc.task_id = task_id_is_work_queue`, calls
  `instance->init()`, cleans up and returns `PX4_ERROR` on failure (e.g.
  `MulticopterRateControl.cpp:298-327`).
- `init()` — registers the driving uORB callback (see item 3) and returns
  bool; only `mc_pos_control::init()` also calls `ScheduleNow()` since it's
  a `ScheduledWorkItem` (`MulticopterPositionControl.cpp:62-73`).
- `custom_command()` — all three just return `print_usage("unknown
  command")`; none of these define custom CLI subcommands.
- `print_usage()` — uses `PRINT_MODULE_DESCRIPTION`,
  `PRINT_MODULE_USAGE_NAME`, `PRINT_MODULE_USAGE_COMMAND("start")`,
  `PRINT_MODULE_USAGE_DEFAULT_COMMANDS()`.
- `print_status()` is only overridden by `control_allocator`
  (`ControlAllocator.cpp:894-1045`) to dump the effectiveness matrix and
  actuator min/max — none of the three flight controllers override it, so
  they get `ModuleBase`'s default.
- Bottom of each `_main.cpp`: `extern "C" __EXPORT int
  <name>_main(int argc, char *argv[]) { return
  ModuleBase::main(<Class>::desc, argc, argv); }` — the NuttX/POSIX
  app entrypoint the build system hooks up.

**PX4 convention — required.** This is the only shape `ModuleBase::main()`
and the build's app-registration machinery understand. A standalone module
must follow this exact pattern (own `desc`, `task_spawn`,
`custom_command`, `print_usage`, `_main` entrypoint) to be startable from
the NSH shell / startup scripts and to behave correctly under `stop`/
`status`.

## 2. Work queue registration, scheduling, loop rate

Two distinct scheduling styles, both driven by uORB, not by a timer:

- **Callback-driven (mc_att_control, mc_rate_control, control_allocator):**
  a `uORB::SubscriptionCallbackWorkItem` on the highest-rate input topic
  registers itself as the `Run()` trigger via `registerCallback()` in
  `init()`. Effective loop rate = the publish rate of that topic (gyro rate
  for rate control, EKF attitude rate for attitude control).
  `mc_att_control` triggers on `vehicle_attitude`
  (`mc_att_control.hpp:115`); `mc_rate_control` triggers on
  `vehicle_angular_velocity` (gyro rate); `control_allocator` triggers on
  `vehicle_torque_setpoint` (see item 7 for why only torque, not thrust,
  drives the allocator).
- **Self-paced with backup (mc_pos_control):** primary trigger is
  `_local_pos_sub` (`SubscriptionCallbackWorkItem` on
  `vehicle_local_position`, `MulticopterPositionControl.hpp:104`), but
  `Run()` also calls `ScheduleDelayed(100_ms)` every cycle as a backup in
  case the local-position callback stalls (`MulticopterPositionControl.cpp:389`).
  `control_allocator` does the same with a 50ms backup, but only
  `#ifndef ENABLE_LOCKSTEP_SCHEDULER` — the backup is disabled in lockstep
  SITL because it would interfere with lockstep's deterministic
  time-advance (`ControlAllocator.cpp:101-103`, `:307-310`).
- Work queue assignment at construction: `nav_and_controllers` for
  mc_pos_control and mc_att_control; `rate_ctrl` for mc_rate_control and
  control_allocator. These are separate named work queues with their own
  priority/stack size
  (`platforms/common/include/px4_platform_common/px4_work_queue/WorkQueueManager.hpp:54,71`)
  — rate control and allocation run on a higher-priority queue than
  position/attitude.
- `dt` is never assumed fixed: every `Run()` computes
  `dt = (now - _last_run) * 1e-6f` from the *triggering topic's own
  timestamp*, then clamps it to a sane range (e.g.
  `math::constrain(dt, 0.000125f, 0.02f)` in mc_rate_control,
  `0.0002f/0.02f` in mc_att_control, `0.002f/0.04f` in mc_pos_control) —
  guards against startup glitches and stalls, not a fixed-rate assumption.

**PX4 convention — required for the pieces that touch shared queues and
lockstep.** Using `px4::WorkItem`/`ScheduledWorkItem` and registering a
`SubscriptionCallbackWorkItem` is the only supported way to get scheduled
on the work-queue threads at all. Which specific topic drives the
callback, whether to add a `ScheduleDelayed` backup, and which named queue
to join, are **stylistic / architectural choices** — but the
`#ifndef ENABLE_LOCKSTEP_SCHEDULER` guard on backup scheduling is a real
constraint if the module will ever run in lockstep SITL: an unconditional
backup schedule can fight lockstep.

## 3. uORB patterns — Sub vs SubscriptionCallback, staleness, publish throttling

- **Driving input** (the one whose new-data event should trigger `Run()`):
  `uORB::SubscriptionCallbackWorkItem`, constructed with `this` so it can
  register the callback. Exactly one per module in all four modules read
  here.
- **All other inputs:** plain `uORB::Subscription`, polled with
  `.update(&struct)` (copies only if updated, returns bool) or
  `.updated()` + `.copy(&struct)` when the code needs to branch on
  "did this change" before deciding whether to act (e.g.
  `MulticopterPositionControl.cpp:403-416` diffing
  `flag_multicopter_position_control_enabled` before vs. after copy).
- **Parameter updates:** always `uORB::SubscriptionInterval
  _parameter_update_sub{ORB_ID(parameter_update), 1_s}` — rate-limited to
  1 Hz, checked with `.updated()` at the top of `Run()`, triggers
  `updateParams()` + a module-local `parameters_updated()` that pushes
  params into the internal control-law objects.
- **Staleness / validity:** handled per-field, not via a single "is this
  message stale" check. `mc_pos_control::set_vehicle_states()` gates each
  field independently on both a `_valid` flag from the EKF *and*
  `PX4_ISFINITE`/`isAllFinite()`, and separately resets internal filters
  when a field goes invalid (`MulticopterPositionControl.cpp:319-373`). EKF
  reset counters (`vxy_reset_counter`, `vz_reset_counter`,
  `xy_reset_counter`, `z_reset_counter`, `heading_reset_counter`) are
  tracked as instance state and diffed each cycle to detect an EKF
  discontinuity and apply the corresponding `delta_*` to any in-flight
  setpoint (`adjustSetpointForEKFResets`,
  `MulticopterPositionControl.cpp:690-733`) — this is how the stack avoids
  a control transient when the estimator jumps.
- **Publish throttling:** not time-based for the control outputs
  themselves — they publish every `Run()` cycle at the driving topic's
  rate. Throttling is used only for lower-priority/expensive outputs: e.g.
  `control_allocator` publishes `control_allocator_status` only every 5ms
  regardless of its own faster `Run()` rate
  (`ControlAllocator.cpp:456-466`, "it's somewhat expensive and we use it
  for slower dynamics"), and `mc_pos_control` only republishes
  `takeoff_status` when the state or tilt limit actually changed
  (`MulticopterPositionControl.cpp:624-633`, using
  `uORB::PublicationData` to compare against the last-published value).

**PX4 convention — required.** `SubscriptionCallbackWorkItem` on exactly
the topic meant to pace the module is how PX4's uORB scheduling model
works; getting this wrong (e.g. registering the callback on a
low-rate/optional topic) silently changes your control loop's real rate.
The specific staleness/validity gating strategy (per-field flags +
finite-checks + reset-counter diffing) is **PX4 convention** in the sense
that it's how the rest of the stack (EKF2, land detector) communicates
validity — a module consuming `vehicle_local_position` or similar
EKF-derived topics should honor the same `_valid` flags and reset counters
to stay consistent. Which outputs get throttled and by how much is a
**stylistic choice**.

## 4. Parameter declaration/update patterns

- All parameters declared via `DEFINE_PARAMETERS(...)` macro in the class
  body, one `(ParamFloat<px4::params::X>) _param_x` /
  `(ParamInt<...>)` / `(ParamBool<...>)` entry per line (e.g.
  `MulticopterRateControl.hpp:137-169`). These map 1:1 to `.yaml` param
  metadata files listed under `MODULE_CONFIG` in each `CMakeLists.txt`
  (item 8).
- Update flow, identical shape in all three: check
  `_parameter_update_sub.updated()` -> `.copy()` to clear the update ->
  call `ModuleParams::updateParams()` (refreshes all `Param*` values from
  storage) -> call a module-local `parameters_updated()`/
  `parameters_update()` that pushes the new values into the actual control
  objects (`_rate_control.setPidGains(...)`,
  `_attitude_control.setProportionalGain(...)`, `_control.setPositionGains(...)`,
  etc.) — the raw `Param*` values are never read directly inside the hot
  control-law code, only inside this update function.
- `mc_pos_control` additionally does param *derivation*: several
  parameters (tilt limits, accel limits, jerk limits) are computed from a
  single `SYS_VEHICLE_RESP` "responsiveness" meta-parameter via
  `commit_no_notification()` + a final `param_notify_changes()`
  (`MulticopterPositionControl.cpp:122-151`), and several are
  cross-validated against each other with `mavlink_log_critical` +
  `events::send` warnings when a derived/dependent constraint is violated
  (e.g. hover thrust outside min/max, cruise speed above max speed,
  `MulticopterPositionControl.cpp:176-295`).

**PX4 convention — required.** `DEFINE_PARAMETERS` + `.yaml` metadata is
the only way a parameter shows up in QGroundControl, mavlink param
protocol, and logging. The "meta-parameter derives several concrete
parameters" pattern (`SYS_VEHICLE_RESP`) and the
cross-validation-with-mavlink-warning pattern are **stylistic choices**
specific to this module, not something a new module must replicate.

## 5. Signaling readiness/failure to commander; behavior on arming/nav_state change

None of the three flight controllers directly publish anything to
commander declaring "I am healthy" — there's no explicit
health/readiness uORB topic published by mc_pos_control, mc_att_control,
or mc_rate_control in the code read here. Instead:

- **Arming state** is read, not announced: `vehicle_control_mode_s
  _vehicle_control_mode` (subscribed by all three) carries
  `flag_armed` plus per-axis `flag_control_*_enabled` bits (attitude,
  rates, position, manual, offboard, allocation) that commander/the
  navigator set based on the current nav_state and arming state.
  Controllers gate all their behavior off these flags rather than
  computing arming/mode logic themselves — e.g.
  `mc_rate_control::Run()` only runs the rate controller and publishes
  torque/thrust when `_vehicle_control_mode.flag_control_rates_enabled`
  is set, and resets its integrator when `!flag_armed`
  (`MulticopterRateControl.cpp:189-194`).
- **On disarm / mode exit:** each controller resets its own internal
  state rather than signaling anything outward — e.g.
  `mc_pos_control` clears `_setpoint` and calls
  `_control.setInputSetpoint()` when
  `flag_multicopter_position_control_enabled` transitions high->low
  (`MulticopterPositionControl.cpp:410-414`); `mc_rate_control` resets
  its rate-control integrator on `!flag_armed`.
  `control_allocator` gates whether it actually publishes actuator
  outputs on `vehicle_control_mode.flag_control_allocation_enabled`
  (`_publish_controls`, `ControlAllocator.cpp:366-369`, `:719-721`) — so
  even when the allocator computes an allocation every cycle, it will
  silently *not* publish `actuator_motors`/`actuator_servos` unless
  commander/the mode manager has set that flag.
  `control_allocator` also reads `failure_detector_status` to react to
  a reported motor failure by reconfiguring its effectiveness matrix
  (`check_for_motor_failures`, `ControlAllocator.cpp:810-869`) — this is
  the one place a controller reacts to a "failure" signal from
  elsewhere in the stack, but it doesn't originate the signal.
- **The one output that does function as a health/status signal
  consumed elsewhere:** `control_allocator_status`
  (`torque_setpoint_achieved`, `thrust_setpoint_achieved`,
  `unallocated_torque/thrust`, `actuator_saturation[]`) is published by
  control_allocator every cycle at up to 5ms and is read back by
  `mc_rate_control` to drive rate-controller anti-windup
  (`_control_allocator_status_sub`, `MulticopterRateControl.cpp:196-216`)
  — this is a feedback loop *between controllers*, not a
  controller-to-commander health report.

**PX4 convention — required (for the flags), architecture-inherent
(for the "no explicit health report" observation).** Reading
`vehicle_control_mode` flags and gating behavior on them is required to
interoperate — commander/navigator are the authority on what mode the
vehicle is in and controllers must not compute this themselves. There is
no "raise a ready/fault flag to commander" API pattern visible in these
three modules to imitate; whatever the new module needs for
fault-signaling would need its own design decision (e.g. failsafe
circuit breaker, a new status topic, or leaning on existing
`failure_detector_status`) — **flagging this as a gap, not a pattern to
copy.**

## 6. Where control latency is measured/logged, what gets logged for debugging

- **Latency/cycle-time instrumentation:** every module allocates a
  `perf_counter_t` via `perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")` in
  the constructor, brackets the entire `Run()` body with
  `perf_begin(...)`/`perf_end(...)`, and frees it in the destructor with
  `perf_free(...)` (e.g. `MulticopterRateControl.hpp:123`,
  `MulticopterRateControl.cpp:53,63,113,271`). This is PX4's generic perf
  framework — counters are inspectable at runtime via the `perf` command
  and dumped by `control_allocator::print_status()`
  (`ControlAllocator.cpp:1042`); the other three modules don't override
  `print_status()` so their perf counters aren't self-printed, only
  visible via the global `perf` listing.
- **Debug/status logging:** no module here does structured
  step-by-step debug logging of internal control values beyond what's
  already captured by the uORB topics they publish (which the logger
  records if configured to). `rate_ctrl_status` is explicitly a
  debugging/tuning topic — `mc_rate_control` publishes it every cycle
  via `_controller_status_pub` (a `PublicationMulti`) carrying whatever
  `_rate_control.getRateControlStatus()` fills in
  (`MulticopterRateControl.cpp:226-229`). `actuator_controls_status_0`
  is a lower-rate (500ms integration window) power/energy-per-axis debug
  topic computed from the torque setpoint
  (`updateActuatorControlsStatus`, `MulticopterRateControl.cpp:274-296`).
  `mavlink_log_critical`/`events::send` calls are used only for
  user-facing warnings about parameter constraint violations
  (`mc_pos_control`, item 4), not general debug tracing.

**PX4 convention — required for anything you want visible in
`perf`/flight-review-style tooling.** Wrapping `Run()` in a named
`PC_ELAPSED` perf counter is the standard, expected way cycle time is
exposed. Publishing a dedicated `*_status` debug topic (like
`rate_ctrl_status`) is a **stylistic choice**, but if the new
allocation-boundary needs its own debug visibility, mirroring this
pattern (a status topic populated by the control-law class itself) is a
reasonable low-risk option rather than inventing a new mechanism.

## 7. The uORB boundary into ControlAllocator

This is the concrete interface a fully-actuated module's rate-control-
equivalent stage would also need to satisfy, if it's going to reuse
`control_allocator` rather than writing a standalone allocator (a
decision explicitly out of scope here — see `.claude/CLAUDE.md`: "Do not
substitute PX4's default control allocator for the custom one" — this
section documents the interface only, not a proposal to use it).

- **Two topics carry the boundary**, published together by
  `mc_rate_control` and consumed by `control_allocator`:
  - `vehicle_torque_setpoint` (`msg/VehicleTorqueSetpoint.msg`):
    `float32[3] xyz` — "torque setpoint about X, Y, Z body axis
    (normalized)". No explicit frame documented in-message beyond "body
    axis"; by construction (`MulticopterRateControl.cpp:219-223`,
    `_rate_control.update(rates, ...)`) X/Y/Z correspond to
    roll/pitch/yaw in FRD body frame, consistent with
    `vehicle_angular_velocity` (the input).
  - `vehicle_thrust_setpoint` (`msg/VehicleThrustSetpoint.msg`):
    `float32[3] xyz`, range `[-1, 1]`, "along X, Y, Z body axis... If
    set to NAN the motors affecting this axis are stopped." For the
    stock quad, only `xyz[2]` (down) is ever non-zero/non-NaN — a
    structural consequence of underactuation, **not transferable**
    as-is to a fully-actuated vehicle, which would legitimately want
    all three axes populated.
  - Both are **normalized, unitless, [-1, 1]** at this boundary — not
    physical torque (N.m) or force (N). This is a key contract:
    `control_allocator` explicitly documents (comment,
    `ControlAllocator.hpp:193-200`) that only `vehicle_torque_setpoint`
    drives the allocator's callback/`Run()` trigger — thrust is read
    opportunistically (`_vehicle_thrust_setpoint_sub.update()`, a plain
    `Subscription`, not a callback source) whenever torque triggers a
    run, on the assumption the two are "usually published together."
    **This ordering dependency is a real constraint**: if a fully-actuated
    module ever needs both torque and thrust honored atomically per
    control cycle, publishing torque *after* thrust (or in the same
    `Run()` call, torque last) each cycle is required to avoid a
    one-cycle-stale thrust value being paired with fresh torque — see
    the linked PR/issue refs in that comment
    (`ControlAllocator.hpp:196-200`) for the history of bugs this
    caused.
  - What `ControlAllocator::Run()` assumes is already true about this
    input (`ControlAllocator.cpp:379-396`): both topics are read as
    already-clamped-to-normalized-range setpoints; thrust NaNs are
    converted to zero (`_thrust_sp.nanToZero()`) *after* being used to
    decide which motors to stop
    (`_actuator_effectiveness->stopMotorsBasedOnThrustSetpoint`) — so
    NaN-as-"stop this axis" is a meaningful signal the allocator acts
    on, not just a defensive default. Torque has no equivalent
    NaN-handling — `mc_rate_control` explicitly replaces any non-finite
    torque axis with `0.f` before publishing
    (`MulticopterRateControl.cpp:236-238`), so `control_allocator` never
    has to handle NaN torque.
  - Saturation is **not yet applied** at this boundary — the allocator
    does saturation/clipping itself
    (`_control_allocation[i]->clipActuatorSetpoint()`,
    `ControlAllocator.cpp:449`) after allocating; `mc_rate_control`
    publishes its raw PID output. Anti-windup feedback flows the
    opposite direction, from `control_allocator_status` back into
    `mc_rate_control`'s `RateControl::setSaturationStatus()` (item 5) —
    so the rate controller *reacts to* saturation reported one cycle
    later, it does not pre-empt it.
  - Allocator's own outputs downstream: `actuator_motors`
    (`msg/versioned/ActuatorMotors.msg`) — `float32[12] control`,
    range `[-1, 1]`, "1 means maximum positive thrust... NaN maps to
    disarmed" — and `actuator_servos`, gated on
    `vehicle_control_mode.flag_control_allocation_enabled`
    (`_publish_controls`).
  - Feedback topic: `control_allocator_status`
    (`msg/ControlAllocatorStatus.msg`) — achieved flags,
    unallocated torque/thrust, per-actuator saturation direction
    (`ACTUATOR_SATURATION_UPPER/LOWER/_DYN`).

**PX4 convention — required, if reusing `control_allocator`.** The
normalized-[-1,1]-unitless contract on `vehicle_torque_setpoint`/
`vehicle_thrust_setpoint`, the torque-drives-the-callback /
thrust-read-opportunistically ordering dependency, and the
NaN-on-an-axis-means-stop-those-motors convention are all things a
producer into `control_allocator` must honor exactly — they're not
stylistic. Whether the new module even *uses* `control_allocator` for its
independent per-rotor tilt actuation is the open architectural question
flagged in `.claude/CLAUDE.md`, not resolved here.

## 8. Build wiring

- Each module's `CMakeLists.txt` calls `px4_add_module(MODULE
  modules__<name> MAIN <name> COMPILE_FLAGS ... SRCS ... MODULE_CONFIG
  <param-yaml files> DEPENDS <library targets>)` — e.g.
  `mc_rate_control/CMakeLists.txt:34-50` depends on `circuit_breaker`,
  `mathlib`, `RateControl` (its own control-law library), and
  `px4_work_queue`.
  `MODULE_CONFIG` is how the `.yaml` parameter metadata files get fed
  into the parameter-generation step (item 4) — a module with runtime
  parameters must list its yaml files here.
- Each module also has a `Kconfig` fragment
  (`menuconfig MODULES_MC_RATE_CONTROL ... depends on ...`,
  `mc_rate_control/Kconfig:1-12`) that defines the `CONFIG_MODULES_*`
  symbol used to gate inclusion.
- A board is built by enabling `CONFIG_MODULES_<NAME>=y` lines in a
  `.px4board` file — confirmed in
  `boards/px4/sitl/default.px4board`: `CONFIG_MODULES_MC_POS_CONTROL=y`,
  `CONFIG_MODULES_MC_ATT_CONTROL=y`, `CONFIG_MODULES_MC_RATE_CONTROL=y`,
  `CONFIG_MODULES_CONTROL_ALLOCATOR=y` are all present alongside ~30
  other enabled modules (EKF2, commander, land_detector, etc.). This is
  the entire mechanism — no separate registry file beyond the
  `.px4board` + each module's own `Kconfig`.

**PX4 convention — required.** This three-piece pattern
(`CMakeLists.txt` + `Kconfig` + a `CONFIG_MODULES_<NAME>=y` line in the
target `.px4board`) is the only supported way to get a new module built
and started in a SITL image — there's no alternative build path. A new
standalone module needs all three pieces, with its own new
`CONFIG_MODULES_<NAME>` symbol, added to whichever `.px4board` file
covers the SITL target being used (`default.px4board` or a
custom/derived one).

## Not transferable (flagged, not detailed)

Items that exist specifically because the stock quad must tilt its body
to translate — do not carry over to a fully-actuated, independent-tilt
vehicle:

- **`mc_pos_control` tilt-vector-to-attitude-setpoint conversion** —
  the whole point of `PositionControl`/`ControlMath` producing a thrust
  vector that gets split into a tilt quaternion + scalar thrust
  (`print_usage` docstring, `MulticopterPositionControl.cpp:777-786`)
  is a translate-by-tilting mechanism.
- **`mc_att_control` tilt-limit / stick-to-tilt-angle mapping** —
  `generate_attitude_setpoint`'s roll/pitch-to-tilt-angle geometry
  (`v_norm`, `_man_tilt_max`, axis-angle construction,
  `mc_att_control_main.cpp:144-210`) and `MC_YAW_WEIGHT` priority
  logic in `AttitudeControl::setProportionalGain` (referenced at
  `mc_att_control.hpp:158`, not read in detail here) both encode
  "yaw is lower priority than roll/pitch because roll/pitch must
  produce translation."
- **VTOL-specific tilt-correction math** — `AttitudeControlMath::
  correctTiltSetpointForYawError` (`mc_att_control_main.cpp:198`) is
  VTOL-transition-specific, out of scope entirely.
- **`ActuatorEffectivenessMultirotor` geometry / mixing math and the
  specific allocation algorithms** (`ControlAllocationPseudoInverse`,
  `ControlAllocationSequentialDesaturation`) — this is exactly the
  "their control allocation approach" a custom controller replaces; not
  to be substituted for the custom allocator regardless.
- **Takeoff/landing tilt-ramp logic** (`TakeoffHandling`,
  `MPC_TILTMAX_LND`/`MPC_TILTMAX_AIR` slew, `MulticopterPositionControl.cpp:525-528`)
  — tied to the tilt-to-translate assumption.
