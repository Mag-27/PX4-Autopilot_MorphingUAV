# System Specification

## Objective
Establish a trustworthy PX4 + Gazebo SITL pipeline for a custom
position/attitude controller with custom control allocation, such that a
fault can be attributed to controller, allocator, PX4 interface,
coordinate convention, actuator interface, Gazebo model, estimator, or
timing — rather than assumed by default to be a controller error.

## Module scope (decided)
The custom module replaces the full stack: position control → attitude
control → rate/allocation → actuator output. Standalone PX4 module;
existing PX4 controller/allocator modules are not modified.

**Not yet true at runtime (confirmed 2026-09-07):** `4026_gz_foldrotor3`
sources `rc.mc_defaults` (`VEHICLE_TYPE mc`), and `rc.vehicle_setup`
unconditionally sources `rc.mc_apps` for any `mc` airframe — which starts
`control_allocator`, `mc_rate_control`, `mc_att_control`, and
`mc_pos_control`. Nothing currently stops this for this airframe, and
`foldrotor_control` is not started by any rc script. This is harmless
today only because `foldrotor_control` is a skeleton that publishes
nothing. **The stock stack must stop being auto-started for this airframe
at or before step 5 (the first actuator-publish test), not only at step
6 (closed-loop hover)** — otherwise the moment `foldrotor_control` starts
publishing `actuator_motors`/`actuator_servos`, both stacks will be
commanding the same actuators simultaneously, and an open-loop
actuator-publish test would be exercising a race, not the module in
isolation.

## Architecture
Estimator → Position Controller → Attitude Controller → Desired Wrench →
Control Allocation → Actuator Commands → Gazebo

## Contract table
Fill each row from actual code + testing, not assumption. TBD = not yet
verified against the current codebase.

| Boundary | In | Out | Units | Frame | Sign convention | Rate | Valid range | Stale-data behavior | Verified by |
|---|---|---|---|---|---|---|---|---|---|
| Estimator → Controller | EKF2 state | position, velocity, attitude, angular rate | m, m/s, rad, rad/s (assumed) | position: inertial/NED (confirmed); **velocity: inertial/NED, confirmed 2026-09-07 — same frame as position, per `VehicleLocalPosition.msg`'s `vx/vy/vz` doc (was OPEN)**; **attitude: quaternion on the wire, confirmed 2026-09-07 — `vehicle_attitude.q[4]`, Hamilton, FRD body→NED (per `VehicleAttitude.msg`), not Euler as previously stated; module must convert to Euler internally (ZYX, matching `Inertial2Body`) before the phi/theta/psi cascade math applies — see controller.md Interface**; **angular rate: FRD body, confirmed 2026-09-07 via `VehicleAngularVelocity.msg`** | TBD | pos 50Hz, att 250Hz, vel 50Hz, rate 1000Hz (per Simulink; unconfirmed in PX4 module) | TBD | TBD | interface test |
| Controller → Allocation | state error | desired wrench: Fx_b,Fy_b,Fz_b,Mx_b,My_b,Mz_b | N, N·m | force components: **body required, currently inertial (confirmed bug — fix specified in controller.md)**; moment components: body (unaffected) | TBD | same as above | unconstrained (allocator saturates downstream) | n/a | comparison vs. Simulink |
| Allocation → Actuators | desired wrench | F1,F2 (thrust), α1,β1,α2,β2 (tilt) | N; rad | body, per-rotor | **α1/α2 = Arm1FoldJoint/Arm2FoldJoint angle (lateral thrust tilt, body Y); β1/β2 = Arm1TiltJoint/Arm2TiltJoint angle (longitudinal thrust tilt, body X) — by design in vehicle dynamics, decided 2026-09-06. Arms are angle-sign-symmetric, not mirrored: equal positive α (or β) on both arms tilts both thrust vectors the same body direction. Detail: allocation.md "Actuator naming and tilt-limit convention".** | same as above | F: [0,15] N; **α,β: [-0.79, 0.79] rad (±45.26°) — decided 2026-09-06, clamped to the physical/SDF/servo range; was ±1.0472 rad (±60°), the allocator's original design intent, which the plant cannot reach. Detail: allocation.md.** | TBD | saturation test |
| Actuators → Gazebo | motors: `command/motor_speed` (idx 0,1); servos: `/model/foldrotor3_0/servo_0..3` (`SIM_GZ_SV_FUNC1..4`=201-204, confirmed set as of PR #3 — was the Milestone 1 blocker below) | applied force/moment | motors rad/s; servos rad (joint position) | **body FRD (X fwd, Y right, Z down) — note the SDF is authored in gz FLU, which is 180° about X from this. Lateral side-by-side rotor pair: Arm1/Prop1 at +Y (+0.2318/+0.2684 m), Arm2/Prop2 at −Y (−0.2348/−0.2684 m), props mirrored to ±0.26838, both at z=+0.0301. Set by `airframe_link_joint` = +90° roll (CAD Y-up → body up) then −90° yaw (arm heading). Verified 2026-09-05 from SDF+STL geometry, not assumption; guarded by `foldrotor3_tests/test_frame_convention.py`** | **Verified 2026-09-06 by force/moment direction test, all 6 channels (force_moment_test.md Results). Motors: a positive `-v` gives lift (−Fz FRD), Motor1 −Mx / +Mz and Motor2 +Mx / −Mz, so the counter-rotating pair cancels roll and yaw to <0.001 N·m while lift sums. Servos: a positive `-v` gives −My on Arm1Tilt and Arm2Tilt, −Mx on Arm1Fold, +Mx on Arm2Fold, each a pure moment with no net force. Combined motor+tilt redistributes thrust per cos/sin of the tilt angle. Signs all match the SDF-derived expectation; lift to 0.1%, roll 0.6%, yaw 0.2%. Required fixing the rotor thrust axis, which was perpendicular to the spin axis — guarded by `test_frame_convention.py`.** | TBD — 200 Hz was assumed; nothing in SDF or gz_bridge confirms it | motors: maxRotVelocity 2054.42 rad/s, `SIM_GZ_EC_MIN/MAX` corrected to 308/2054 (was mismatched 150/1000, fixed PR #3) so 100% throttle actually reaches maxRotVelocity; servo angle ±45.26° (±0.79 rad, matches model.sdf joint limit), `SIM_GZ_SV_MINA/MAXA` set accordingly | TBD | force/moment direction test |

## Verification philosophy
Verification (did we build it correctly) precedes validation (does it
behave correctly). A tracking failure is investigated bottom-up through
this table before the controller itself is suspected.

## Milestone 1 acceptance criteria
- [x] **Precondition (blocker, found 2026-08-27; resolved 2026-08-28,
      PR #3):** assign PX4 actuator functions to the four fold/tilt
      servos so `GZMixingInterfaceServo` will publish to `servo_0..3`.
      `SIM_GZ_SV_FUNC1..4` are now set (201-204, mapping
      Arm1FoldJoint→servo_0, Arm1TiltJoint→servo_1,
      Arm2FoldJoint→servo_2, Arm2TiltJoint→servo_3 in
      `4026_gz_foldrotor3`) — all 6 DOF are now reachable from PX4. This
      unblocks, but does not itself satisfy, the open-loop test below.
- [x] **Open-loop actuator test (confirmed 2026-08-28):** commanded
      actuator values produce the expected Gazebo actuator response on
      `4026_gz_foldrotor3` itself — all 6 channels. Servos (`-s 1..4`,
      all four fold/tilt joints) and motors (`-m 1`, `-m 2`) both
      confirmed via `actuator_test` against the running SITL model, per
      `open_loop_commands.md`'s reference commands. This is direction/
      magnitude-agnostic — it only confirms each channel moves, not that
      the sign/magnitude is correct; that's the next item.
      `servo_load_test_logs/` bench-fixture data (tilt and fold, p-gain
      10/20 sweeps) is separate and was not used to satisfy this row.
- [x] **Force/moment direction test — RUN 2026-09-06, PASSED.** All 6
      actuator channels plus both combined cases verified against an
      SDF-derived expectation: every sign correct, lift to 0.1%, roll 0.6%,
      yaw 0.2%. Counter-rotating pair cancels roll and yaw to <0.001 N·m;
      arm tilt redirects thrust per cos/sin of the tilt angle.
      **This test exposed and fixed a flight-model defect**: the rotors
      produced *zero* net thrust because
      `gz-sim-multicopter-motor-model-system` applies thrust along the rotor
      link's local Z, and foldrotor3's Y-up CAD left that axis perpendicular
      to the spin axis — the force rotated with the propeller and averaged
      out. Closed-loop hover could never have worked, and would have looked
      like a controller bug. Fixed in `models/foldrotor3/model.sdf` and the
      bench copy; guarded by
      `foldrotor3_tests/test_frame_convention.py`. Full comparison table in
      `force_moment_test.md` Results.
- [ ] Only after all of the above: attempt closed-loop Offboard hover
      with EKF2 state feedback

Note: the Estimator→Controller and Controller→Allocation contract rows
are not verifiable until a controller module exists. Do not fill them in
from the Simulink model — that would record an assumption as a
confirmed contract.

## Out of scope (for now)
- Hardware integration
- Failsafe/arming logic changes
- Modifying existing PX4 controller/allocator modules
