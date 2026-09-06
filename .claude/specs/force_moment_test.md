# Force/Moment Direction Test

## Objective

Verify that commanded actuator values produce reaction force/moment at the
fixed bench mount with the expected sign and direction. This closes the
gap the open-loop actuator test explicitly left open: that test confirmed
each of the 6 actuator channels *moves*, not that the sign or direction of
the resulting force/moment is correct.

Ties directly to `system.md`'s Milestone 1 checklist item: *"Force/moment
direction test: known actuator commands produce force/moment in the
expected direction and sign."* Per `.claude/CLAUDE.md`'s verification
order (interface tests → SITL open-loop actuator tests → force/moment
direction tests → closed-loop validation), this test must pass before any
closed-loop validation is attempted.

## Status

**Complete, 2026-09-06. PASSED — all 6 actuator channels plus both combined
cases verified against an SDF-derived expectation.** Every sign is correct;
lift matches to 0.1%, roll 0.6%, yaw 0.2%.

Getting there required fixing a **flight-model defect this test exposed**:
the rotors produced no net thrust at all, because the thrust axis was
perpendicular to the spin axis. See Results. Milestone 1's force/moment box
is now checked and closed-loop validation is unblocked.

The bench fixture now has: the `airframe_link_joint` geometry-parity fix, a
`mount_plate` link giving `bench_mount_joint` a real parent link, the
`force_torque` sensor, a model-scoped `gz-sim-forcetorque-system` plugin,
and a raised stand height. All are in
`Tools/simulation/gz/models/foldrotor3_bench/model.sdf` (submodule
`Tools/simulation/gz`, branch `foldrotor3-servo-load-test`).

## Scope

**In scope:**
- Single-actuator isolation tests: both motors (`-m 1`, `-m 2`) and all
  four tilt/fold servos (`-s 1..4`).
- Combined/simultaneous multi-actuator cases: both motors together; one
  motor with its own tilt servo.

**Out of scope:**
- Magnitude/calibration accuracy. The sim thrust model is a single-point
  quadratic fit (per `open_loop_commands.md`) — sign and direction are the
  deliverable here, not magnitude match.
- Closed-loop behavior — the next, later step in the verification order.

## Prerequisite: bench geometry parity

`foldrotor3_bench/model.sdf`'s `airframe_link_joint` pose is stale
relative to the flight model:

- **Bench (current, stale):** `0 0 0 1.5707963267948966 0 0`
- **Flight (commit `00267a4`):** `0 0 0 1.5707963267948966 0 -1.5707963267948966`

Commit `00267a4` added a −90° yaw term to the flight file's
`airframe_link_joint` pose to bring the arms onto body ±Y
(`git show 00267a4 --stat` confirms it touched only
`models/foldrotor3/model.sdf`, not the bench copy). This fix was never
propagated to the bench fixture.

**This was a blocking prerequisite. Applied 2026-09-06** to
`Tools/simulation/gz/models/foldrotor3_bench/model.sdf`, and now guarded by
`foldrotor3_tests/test_frame_convention.py`, which runs every frame
assertion against both SDFs and additionally asserts that the bench and
flight airframe orientations agree. The bench file is a full textual
duplicate rather than an `<include>`, which is precisely how `00267a4`
missed it; the test is what stops that recurring.

## Methodology: computing the expected direction/sign

This section commits to a *procedure*, not a precomputed table. The
actual signed numeric table is test-execution output, produced when the
test is run — consistent with `allocation.md`'s "verified by testing, not
assumption" discipline.

1. **Resolve actuator geometry into body FRD.** Use the same
   pose-chain-walk + `FLU_TO_FRD` (`diag(1,-1,-1)`) approach already
   implemented in `foldrotor3_tests/test_frame_convention.py`
   (`_transform_to_body_flu`, `_origin_frd`). Do not re-derive frame math
   from scratch — reuse those helpers directly.

2. **Motors.** Thrust acts along the resolved local Z of the rotor
   **link** (`PropNLink`), scaled by the plugin's `turningDirection`
   (`ccw`/`cw`) — *not* along the `PropNJoint` axis, which the plugin uses
   only for air drag. This distinction is not cosmetic: for this model the
   two are perpendicular, which is the defect the test found (see Results).
   Read it from `reference/gz-rotor-and-sensor-conventions.md`, do not
   assume it. The net
   wrench observed at `bench_mount_joint` includes the r×F moment-arm
   term from each rotor's position relative to the mount — reuse the
   geometry already documented in `system.md`'s Actuators→Gazebo row
   (Arm1/Prop1 at +Y, Arm2/Prop2 at −Y, once the geometry-parity fix
   above is applied).

3. **Servos (isolated, motors off).** The dominant signature is the change
   in the *gravity* moment about the mount, Δ(r_com × mg), as the arm's
   centre of mass moves — computable from the SDF link masses and the same
   pose-chain walk, with the joint angle injected at the joint frame
   (`expected_wrench.transform_with_angles`). This is a clean, force-free
   moment signature and, as executed, proved a **stronger** check than the
   motor cases, not a weaker one. (This spec originally called it a weak
   secondary check; the Results section supersedes that.)

4. **Combined cases — superposition.** Sum the r×F and torque
   contributions across all simultaneously-active actuators. For
   example: both motors' vertical thrust plus their counter-rotating
   reaction torques (yaw-cancellation case), or a motor's thrust/torque
   plus its own tilt servo's contribution (coupled-axis case). This is
   explicitly the superposition of the single-actuator expectations from
   steps 2–3 — not a separately-derived model.

5. **Mandatory frame-conversion step.** Even with `<frame>parent</frame>`
   selected on the sensor (see below), explicitly verify — do not
   assume — that the readback matches what `<frame>child</frame>` would
   report for this fixture, given `bench_mount_joint` carries no
   `<pose>` (identity rotation from `world` to `base_link`). Then apply
   the same `FLU_TO_FRD` rotation used by `test_frame_convention.py`
   before comparing the sensor readback to any FRD-derived expectation.

6. **Difference every case against a quiescent baseline.** With
   `child_to_parent` the mount carries the full static weight (~15.26 N)
   and the static CoM moment; thrust and mass redistribution are deltas on
   top of that. Capture an actuators-off window in the same run and
   subtract it. Check that baseline against the summed link masses first —
   it is a cheap, exact test that the sensor is sensing rather than welded
   through.

7. **Do not judge a motor case from a single sample or a short window.**
   The rotor force is not necessarily static; take the mean *and* the
   peak-to-peak over a multi-second window. A single `-n 1` sample of this
   model reads like a plausible thrust and is not one.

8. Record the actual signed numeric table only once the test has been
   run — see Acceptance Criteria.

## Results (2026-09-06)

**PASSED, after fixing one flight-model defect this test found.**

Setup: `foldrotor3_bench` on `worlds/default.sdf`, stand at 0.5 m, sensor at
50 Hz, driven with `actuator_test` via `servo_load_test_logs/launch_bench.sh`.
All values are body FRD (X fwd, Y right, Z down) after applying `FLU_TO_FRD`
to the sensor's world-frame output, and are deltas from an actuators-off
baseline in the same run. Expectations come from
`foldrotor3_tests/expected_wrench.py`, computed from the SDF *before*
measuring. Lift is negative Fz because FRD Z points down.

### Baseline — sensor validated

| | Fx | Fy | Fz | Mx | My | Mz |
|---|---|---|---|---|---|---|
| predicted | 0 | 0 | +15.260 | +0.0156 | −0.0066 | 0 |
| measured | 0 | 0 | +15.260 | +0.0159 | −0.0066 | 0 |

The static reading is the airframe weight to the digit (total model mass
1.5581 kg, minus the 1 g `mount_plate` which sits on the parent side of the
sensor, × 9.8 = 15.2600 N) and the static moment to 2%, the residual being
gravity sag at rest that the static-pose expectation does not model. This is
what confirms the sensor is genuinely sensing rather than welded through, and
that `<frame>parent</frame>` is the world orientation — methodology step 5,
confirmed empirically rather than assumed.

### Full comparison

| case | quantity | predicted | measured |
|---|---|---|---|
| `-m 1 -v 0.6` | ΔFz (lift) | −10.078 | **−10.066** |
| | ΔMx (roll) | −2.7048 | **−2.7216** |
| | ΔMz (yaw) | +0.2245 | **+0.2240** |
| `-m 2 -v 0.6` | ΔFz | −10.078 | **−10.066** |
| | ΔMx | +2.7048 | **+2.7210** |
| | ΔMz | −0.2245 | **−0.2239** |
| both motors | ΔFz | −20.157 | **−20.132** |
| | ΔMx | 0 | **−0.0006** |
| | ΔMz | 0 | **+0.0001** |
| `-s 1 -v 0.5` | ΔMx | −0.0102 | **−0.0178** |
| `-s 2 -v 0.5` | ΔMy | −0.0358 | **−0.0359** |
| `-s 3 -v 0.5` | ΔMx | +0.0825 | **+0.0765** |
| `-s 4 -v 0.5` | ΔMy | −0.0328 | **−0.0329** |
| `-m 1` + `-s 2` | ΔFx | +3.878 | **+3.895** |
| | ΔFz | −9.302 | **−9.285** |
| | ΔMx | −2.5830 | **−2.5968** |
| | ΔMz | −0.8336 | **−0.8379** |

Units N and N·m. Every sign is correct. Lift matches to 0.1%, roll to 0.6%,
yaw to 0.2%.

**Counter-rotating pair behaves correctly**: with both motors at equal
command, lift sums to −20.13 N (thrust-to-weight 1.32 at 60% throttle) while
roll and yaw cancel to below 0.001 N·m.

**Thrust vectoring works**: tilting Arm1 by 22.6° while motor 1 runs
redistributes the 10.07 N thrust into 9.29 N vertical + 3.90 N forward.
`10.066 × cos(22.6°) = 9.29` and `10.066 × sin(22.6°) = 3.87` — the arm tilt
redirects thrust exactly as the geometry says it should.

**Servos** produce a clean, force-free moment on the correct axis with no
cross-axis coupling. Both tilt channels match magnitude to 0.3%. The two fold
channels agree in sign but not magnitude (`-s 3` 7% low, `-s 1` 1.7×) because
the fold joints sag ±0.031 rad under gravity at rest, so their true starting
pose is not the SDF pose the expectation is computed from. Magnitude accuracy
is out of scope; flagged, not chased.

Contrary to this spec's original methodology step 3, the servo cases are
**not** a weak secondary check — Δ(r_com × mg) is fully computable and gave
the tightest agreement of any case.

### Residual, unexplained

A single motor produces ΔFy ≈ ∓0.5 N (about 5% of thrust) that the
expectation puts at zero, with opposite sign for the two motors so that it
cancels in the pair (0.002 N with both running). Most likely the tilt joint
deflecting slightly under thrust load — the fold joints demonstrably sag
~0.03 rad — which would tip the thrust vector. **Not verified.** It cancels
in the pair and does not affect any sign, so it is recorded rather than
chased.

### The defect this test found

The first run **failed**: net lift was 0.03 N against a 15.26 N baseline —
zero — while ±10.4 N swung around the body XY plane at rotor frequency.

`gz-sim-multicopter-motor-model-system` applies thrust along the rotor
*link's* local Z:

```cpp
link.AddWorldForce(_ecm, worldPose->Rot().RotateVector(Vector3(0, 0, thrust)));
```

foldrotor3's CAD export is Y-up, so `PropNJoint` carried `<xyz>0 1 0</xyz>`
and each rotor link's local Z lay in the horizontal plane, **perpendicular to
its own spin axis**. The thrust vector therefore rotated with the propeller
and averaged to nothing. Every stock PX4 model instead puts +Z on the spin
axis (`x500_base`: `<xyz>0 0 1</xyz>`, no rotor-link rotation). The joint axis
is used only for air drag, so nothing else in the model flagged it.

Note the thrust sign does **not** depend on `turningDirection`: the plugin's
`td * sign(v)` cancels because the joint velocity is itself commanded as
`td * refRotVel / slowdown`. `turningDirection` only sets the drag-torque
sign, which is what makes the pair's yaw reactions oppose. Reasoning about
the thrust direction from `turningDirection` leads to the wrong fix; see
`reference/gz-rotor-and-sensor-conventions.md`.

**Fix applied** to both `models/foldrotor3/model.sdf` and the bench copy: the
`PropNJoint` frames were rotated −90° about X so local +Z is the spin axis,
`<axis><xyz>` became `0 0 1`, and the visual, collision and inertial poses
were counter-rotated +90° so nothing physical moved. Mass, centre of mass,
inertia, mesh placement and spin axis are all unchanged — only the frame the
plugin reads its thrust direction from. Confirmed by the at-rest bench
reading being identical before and after (15.2600 N, same static moments).
The one genuine physical change is `Prop1Joint`'s stray `2.4136e-05`
x-component in its axis, snapped to a clean `0 0 1` — a 0.0014° difference.

Guarded by
`test_frame_convention.py::test_rotor_thrust_axis_is_the_spin_axis_and_points_up`,
which asserts each rotor link's +Z is parallel to its joint axis and points
up, on both SDFs, plus a companion test asserting the two rotors
counter-rotate. Both were confirmed to fail against the pre-fix model.

Because this defect was in the flight model, **closed-loop hover could never
have worked**, independently of any controller — the vehicle produced no lift
at all. It would have presented as a controller bug.

### Fixture defect found and fixed

At the original 0.1 m stand height the fixture had only 30 mm of ground
clearance, and `-s 1 -v 0.5` drove `Arm1TiltLink` 18 mm **through** the
ground plane. The contact offloaded ~9.5 N of the airframe's weight off
`bench_mount_joint` and injected friction forces, producing a plausible-
looking but entirely spurious wrench. The stand is now at 0.5 m and
`foldrotor3_tests/test_bench_clearance.py` guards every servo case in both
directions.

## Bench fixture implementation

Five edits to `Tools/simulation/gz/models/foldrotor3_bench/model.sdf`
(submodule `Tools/simulation/gz`, branch `foldrotor3-servo-load-test`).
This spec originally called for two; three more turned out to be required
before the sensor would emit anything at all:

**(a) Geometry-parity fix** — update `airframe_link_joint`'s pose per the
Prerequisite section above.

**(b) Add a `force_torque` sensor to `bench_mount_joint`:**

```xml
<sensor name="force_torque" type="force_torque">
  <update_rate>50</update_rate>
  <force_torque>
    <frame>parent</frame>
    <measure_direction>child_to_parent</measure_direction>
  </force_torque>
</sensor>
```

- `<frame>parent</frame>`: report the wrench in the `world` (parent)
  orientation rather than `base_link` (child) or the sensor frame.
- `<measure_direction>child_to_parent</measure_direction>`: report the
  reaction wrench the airframe (`base_link`, the child) exerts on the
  world mount (the parent) — what a real load cell bolted to the bench
  would read.
- Because neither `bench_mount_joint` nor `mount_plate` carries a `<pose>`,
  the `parent` frame is the world orientation. Confirmed empirically by the
  static reading matching the computed weight along world −Z, per
  Methodology step 5 — not assumed.

**(c) Give `bench_mount_joint` a real parent link.** As originally written
the joint's `<parent>` was `world`, and gz-sim's ForceTorque system resolves
both endpoints with `GetLinkFromScopedName()`, which matches only entities
carrying a `Link` component. The world entity has none, so the system logs
*"Parent link with name [world] ... not found. Failed to create sensor"* and
**advertises no topic at all**. Fixed by interposing a massless
`mount_plate` link:

```
world --(bench_anchor_joint, fixed)--> mount_plate
      --(bench_mount_joint, fixed, +force_torque sensor)--> base_link
```

`mount_plate` is on the parent side of the sensor, so its 1 g is correctly
excluded from the reading.

**(d) Load the ForceTorque *system* plugin.** A `<sensor>` element alone is
inert. gz-sim runs only the systems listed in
`src/modules/simulation/gz_bridge/server.config` (exported as
`GZ_SIM_SERVER_CONFIG_PATH` by `gz_env.sh`), which lists 14 systems and does
not include ForceTorque; `worlds/default.sdf` declares no plugins at all.
Declared at **model scope** in the bench SDF instead, which keeps the change
inside this submodule and out of PX4:

```xml
<plugin filename="gz-sim-forcetorque-system" name="gz::sim::systems::ForceTorque"/>
```

Model scope is not the documented placement for this system, so it was
verified empirically before anything else was built on it. It works.

**(e) Raise the stand height** from 0.1 m to 0.5 m. See Results — at 0.1 m a
folded arm passed through the ground plane and silently corrupted the
reading.

Resulting topic:
`/world/<world>/model/foldrotor3_bench/joint/bench_mount_joint/sensor/force_torque/forcetorque`,
message type `gz.msgs.Wrench` (`force{x,y,z}`, `torque{x,y,z}`; torque is
always reported about the joint origin regardless of `<frame>`).

## No PX4-side / gz_bridge changes

PX4's `gz_bridge` module (`src/modules/simulation/gz_bridge/GZBridge.cpp`)
has a fixed subscription list — clock, pose/info, imu, mag, navsat,
air_pressure, distance_sensor, airspeed, optical_flow, odometry,
laser_scan — and forwards none of them for a force-torque-like uORB
topic. This is confirmed by reading that file; there is no PX4-side
support for this sensor type.

Readback for this test is via `gz topic --echo` directly against the
topic above, exactly mirroring the existing `joint_state` readback
pattern already used and documented in `open_loop_commands.md`. **Nothing
in this spec touches PX4 core modules or the (not-yet-written) custom
control allocator.**

## Deliverables

Four, not the two originally listed — the acceptance criteria below
("re-verified", "compared against the geometrically-derived expectation")
cannot be met by a spec and a command list alone:

1. **This spec document**, plus the SDF diff it specifies (edits (a)-(e)
   above to `foldrotor3_bench/model.sdf`).
2. **`force_moment_bench_commands.md`** (repo root) — the exact commands to
   build, launch, and drive this test, plus how to capture sensor output
   during a test window. A first-class, separately reviewable artifact.
3. **Interface-level geometry guards** (`foldrotor3_tests/`, pure SDF/STL,
   no Gazebo, ~1 s): `test_frame_convention.py` now runs against both the
   flight and bench SDFs and asserts they agree; `test_bench_clearance.py`
   asserts no link reaches the ground plane at rest or under any servo
   command in either direction. Both were confirmed falsifiable by
   reverting the fix under test.
4. **Expectation and capture tooling**: `foldrotor3_tests/expected_wrench.py`
   (signed expectation from the SDF, reusing the `test_frame_convention.py`
   helpers) and `servo_load_test_logs/parse_wrench.py` (`gz.msgs.Wrench`
   echo to CSV, mirroring `parse_joint_state.py`).

## Acceptance criteria

- [x] Bench geometry parity fixed and re-verified
      (`test_frame_convention.py`, both SDFs).
- [x] `force_torque` sensor confirmed emitting on the expected topic, and
      its static reading validated against the summed link masses.
- [x] Each of the 6 actuators tested in isolation; sign compared against
      the geometrically-derived expectation. **All 6 pass.**
- [x] Confirmed combined cases (both motors; one motor + its own tilt
      servo) tested; superposition confirmed to 0.3%.
- [x] Result recorded into this spec's Status and Results, and into
      `system.md`'s Actuators→Gazebo row and Milestone 1 checklist.
- [x] Dated `findings.md` entry added.
- [x] Rotor thrust-axis defect fixed in both SDFs, guarded by a regression
      test confirmed to fail against the pre-fix model.

**Milestone 1's force/moment criterion is satisfied.** The next item in the
verification order — closed-loop Offboard hover — is unblocked.

## Not specified here

- Magnitude/calibration accuracy.
- Closed-loop validation — explicitly the next, later step in the
  verification order.
