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
| Actuators → Gazebo | actuator commands | applied force/moment | N, N·m | TBD | TBD | 200 Hz (assumed) | TBD | TBD | force/moment direction test |

## Verification philosophy
Verification (did we build it correctly) precedes validation (does it
behave correctly). A tracking failure is investigated bottom-up through
this table before the controller itself is suspected.

## Milestone 1 acceptance criteria
- [ ] Open-loop actuator test: commanded actuator values produce the
      expected Gazebo actuator response
- [ ] Force/moment direction test: known actuator commands produce
      force/moment in the expected direction and sign
- [ ] Only after both pass: attempt closed-loop Offboard hover with
      EKF2 state feedback

## Out of scope (for now)
- Hardware integration
- Failsafe/arming logic changes
- Modifying existing PX4 controller/allocator modules
