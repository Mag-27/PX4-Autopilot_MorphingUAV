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

## Architecture
Estimator → Position Controller → Attitude Controller → Desired Wrench →
Control Allocation → Actuator Commands → Gazebo

## Contract table
Fill each row from actual code + testing, not assumption. TBD = not yet
verified against the current codebase.

| Boundary | In | Out | Units | Frame | Sign convention | Rate | Valid range | Stale-data behavior | Verified by |
|---|---|---|---|---|---|---|---|---|---|
| Estimator → Controller | EKF2 state | position, velocity, attitude, angular rate | m, m/s, rad, rad/s (assumed) | position: inertial (confirmed); attitude: Euler (confirmed representation); **velocity: OPEN — see controller.md Open questions** | TBD | pos 50Hz, att 250Hz, vel 50Hz, rate 1000Hz (per Simulink; unconfirmed in PX4 module) | TBD | TBD | interface test |
| Controller → Allocation | state error | desired wrench: Fx_b,Fy_b,Fz_b,Mx_b,My_b,Mz_b | N, N·m | force components: **body required, currently inertial (confirmed bug — fix specified in controller.md)**; moment components: body (unaffected) | TBD | same as above | unconstrained (allocator saturates downstream) | n/a | comparison vs. Simulink |
| Allocation → Actuators | desired wrench | F1,F2 (thrust), α1,β1,α2,β2 (tilt) | N; rad | body, per-rotor | TBD | same as above | F: [0,15] N; α,β: [-1.0472, 1.0472] rad | TBD | saturation test |
| Actuators → Gazebo | motors: `command/motor_speed` (idx 0,1); servos: `/model/foldrotor3/servo_0..3` | applied force/moment | motors rad/s; servos rad (joint position) | TBD | TBD | TBD — 200 Hz was assumed; nothing in SDF or gz_bridge confirms it | motors: maxRotVelocity 2054.42 rad/s, `SIM_GZ_EC_MIN/MAX` 150/1000; servo cmd ±5 | TBD | force/moment direction test |

## Verification philosophy
Verification (did we build it correctly) precedes validation (does it
behave correctly). A tracking failure is investigated bottom-up through
this table before the controller itself is suspected.

## Milestone 1 acceptance criteria
- [ ] **Precondition (blocker, found 2026-08-27):** assign PX4 actuator
      functions to the four fold/tilt servos so `GZMixingInterfaceServo`
      will publish to `servo_0..3`. Currently unset, so 4 of 6 DOF are
      unreachable from PX4 and the open-loop test below cannot run for
      them. Compare `4020_gz_tiltrotor`, which does set `SIM_GZ_SV_FUNC*`.
- [ ] Open-loop actuator test: commanded actuator values produce the
      expected Gazebo actuator response — **all 6 channels**, not just
      the 2 already-wired motors
- [ ] Force/moment direction test: known actuator commands produce
      force/moment in the expected direction and sign
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
