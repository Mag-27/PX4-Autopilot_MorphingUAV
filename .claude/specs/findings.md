# Findings Log

Dated exploration notes. Append-only. Folding a finding into
system.md/controller.md/allocation.md is a separate, deliberate decision —
not automatic.

---

## 2026-08-27 — foldrotor3 module and actuator/state interface audit

### Scope
Audited what actually exists in the repo for the foldrotor3 vehicle: the
custom controller/allocation module, the Gazebo model's actuator
interface, and the PX4-side wiring between them. No code changes made.

### 1. Actual current interfaces at the boundary

**Custom controller/allocation module: does not exist in `src/`.**
`grep -rli foldrotor src/` returns nothing. There is no standalone PX4
module implementing position/attitude control or the custom allocator
described in controller.md/allocation.md. Everything committed so far
for foldrotor3 is simulation-side only:
- `Tools/simulation/gz/models/foldrotor3/model.sdf` + `model.config`
  (Gazebo model, tracked in the `Tools/simulation/gz` submodule)
- `Tools/simulation/gz/models/foldrotor3_bench/` (a bench-test fixture
  variant, base_link pinned to world, used for servo tuning)
- `ROMFS/px4fmu_common/init.d-posix/airframes/4026_gz_foldrotor3`
  (PX4 airframe startup script, commit `e03d5a7c23`, single-shot addition
  of the airframe file + its `CMakeLists.txt` registration — the
  "registeree" from the commit message)

(Separately, `src/modules/mc_raptor` exists in this tree — an RL-policy
flight-mode module with its own trajectories/blob submodule. It is
unrelated to the foldrotor3 controller effort; noting it here only so a
future session doesn't mistake it for the target module.)

**Gazebo model actuator interface (`model.sdf`), confirmed from source:**
- 2 rotors via `gz-sim-multicopter-motor-model-system`:
  `Prop1Joint`/`Prop2Joint`, `motorType=velocity`,
  `commandSubTopic=command/motor_speed`, `motorNumber` 0/1,
  `maxRotVelocity=2054.42` rad/s, `motorConstant=5.4844e-06`,
  `momentConstant=0.022274` (both hand-derived from thrust-stand data
  per the in-SDF comment; drag/rolling-moment/spin-up-lag terms are
  x500-derived generic placeholders, not measured for this vehicle).
- 4 fold/tilt joints via `gz-sim-joint-position-controller-system`:
  `Arm1FoldJoint`→`servo_0`, `Arm1TiltJoint`→`servo_1`,
  `Arm2FoldJoint`→`servo_2`, `Arm2TiltJoint`→`servo_3`, all
  `p_gain=20, i_gain=0, d_gain=0.5, cmd_max/min=±5`. Gains were tuned on
  the bench fixture (motors to ~69% commanded omega, ~3.8x airframe
  weight) — see "dangling reference" below.
- PX4-side subscription convention, confirmed in
  `src/modules/simulation/gz_bridge/GZMixingInterfaceServo.cpp`: servo
  topics are published as `/model/<model>/servo_<i>` for i=0..7, and a
  given index is only published if `_mixing_output.isFunctionSet(i)` —
  i.e. only if that actuator function slot is assigned on the PX4 side.

**PX4 airframe wiring (`4026_gz_foldrotor3`), confirmed from source:**
- `CA_AIRFRAME 0` (generic/stock multirotor allocator, not the custom
  allocator), `CA_ROTOR_COUNT 2`, with rotor geometry given directly via
  `CA_ROTOR0_PX/PY/KM` and `CA_ROTOR1_PX/PY/KM` (no preset geometry).
- `SIM_GZ_EC_FUNC1=101`, `SIM_GZ_EC_FUNC2=102` (Motor1/Motor2) map the
  two rotors through to `command/motor_speed`, with
  `SIM_GZ_EC_MIN/MAX{1,2}=150/1000`.
- **No `SIM_GZ_SV_FUNC*` parameters are set anywhere in this airframe
  file**, and none are inherited from `rc.mc_defaults`. Combined with the
  `isFunctionSet(i)` gating above, this means the four fold/tilt servo
  joints defined in the SDF (`servo_0..3`) currently have no PX4 actuator
  function assigned to them — PX4 will not publish to any of those
  topics yet. Comparable existing tilt airframes (e.g.
  `4020_gz_tiltrotor`) do set `SIM_GZ_SV_FUNC*`, so this looks like
  wiring that's simply not done yet rather than an intentional omission.
- The airframe file's own comment is explicit that this is a deliberate
  first pass: *"Open-loop first pass: default/generic control allocation
  only, no custom ActuatorEffectivenessFoldrotor. Actuators are driven
  directly via actuator_test, not through the control allocator's mixer
  output."*

**Dangling reference:** the servo-gain-tuning comment in `model.sdf`
points to `open_loop_commands.md` for "the full test data." No file by
that name exists anywhere in the repo (checked full tree). Either it
was never committed or it lives outside this repo.

### 2. Contradictions / gaps vs. the specs

- **controller.md and allocation.md describe a module that doesn't exist
  yet in code.** Both specs read as settled contracts for the PX4
  module's I/O (state in, wrench out / wrench in, actuator commands
  out), but there is no module to check those contracts against. This
  isn't a contradiction so much as a status mismatch worth being
  explicit about: the specs are ahead of the implementation.
- **system.md's contract table rows "Estimator → Controller" and
  "Controller → Allocation" are not yet exercisable** — no controller
  code exists to source or sink those signals. They should stay TBD, not
  be inferred from the Simulink model, until code exists.
- **system.md's "Allocation → Actuators" row implicitly assumes the
  custom allocator's actuator surface (F1,F2 thrust + α1,β1,α2,β2 tilt)
  is reachable end-to-end.** It currently isn't: only the 2 motor
  channels are wired PX4→Gazebo; the 4 tilt/fold channels that the
  allocator would need to drive have no PX4-side function assignment.
  Even a pure open-loop actuator_test of the tilt/fold axes (Milestone 1,
  system.md acceptance criterion 1) cannot currently reach Gazebo for
  those 4 DOF until `SIM_GZ_SV_FUNC*` (or equivalent) is configured.
- **system.md's "Actuators → Gazebo" row lists rate as "200 Hz
  (assumed)" and frame/units as TBD.** Nothing in the SDF or gz_bridge
  code inspected here confirms a 200 Hz figure either way — still
  genuinely open, not contradicted.
- CA_AIRFRAME 0 (stock generic allocator) is explicitly a temporary
  open-loop stand-in per the airframe file's own comment, consistent
  with allocation.md's "not to be replaced by PX4's default allocator" —
  no contradiction, just confirms the custom allocator isn't in the loop
  yet.

### 3. Which system.md contract-table row(s) this speaks to

- **Allocation → Actuators** — directly: identifies that the actuator
  surface this row describes (F1,F2,α1,β1,α2,β2) is only half-wired on
  the PX4↔Gazebo side (motors yes, tilt/fold servos no), independent of
  whether the custom allocator exists.
- **Actuators → Gazebo** — directly: confirms concrete topic names
  (`command/motor_speed`, `servo_0..3`) and motor constants; rate and
  frame remain TBD, not contradicted.
- **Controller → Allocation** and **Estimator → Controller** —
  indirectly: flags that both remain unverifiable-in-principle right now
  because no controller module exists to generate or consume those
  signals, which is a precondition gap the milestone-1 open-loop
  criteria don't yet surface.
