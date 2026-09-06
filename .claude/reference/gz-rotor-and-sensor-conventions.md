# Gazebo rotor and force/torque sensor conventions (gz-sim 8 / Harmonic)

Durable upstream behaviour, not findings about this repo. Verified against
gz-sim 8.11.0 source and confirmed empirically in SITL, 2026-09-06.

## `gz-sim-multicopter-motor-model-system`

**Thrust is applied along the rotor *link's* local Z axis, not along the
joint axis.** From `src/systems/multicopter_motor_model/MulticopterMotorModel.cc`:

```cpp
double thrust = this->turningDirection * realMotorVelocitySign *
                realMotorVelocity * realMotorVelocity * this->motorConstant;
link.AddWorldForce(_ecm, worldPose->Rot().RotateVector(Vector3(0, 0, thrust)));
```

Consequences:

* The rotor link frame **must** be authored with local +Z along the spin
  axis. Every stock PX4 model does this: `x500_base` gives each
  `rotor_N_joint` `<axis><xyz>0 0 1</xyz></axis>` and no pose rotation.
* If a CAD export is Y-up and the rotor joint axis is `0 1 0`, the link's
  local Z ends up *perpendicular* to the spin axis. The plugin then applies
  thrust sideways, and because the link spins about the actual axis, the
  force vector rotates in the plane at rotor frequency and **averages to
  zero net lift**. The joint axis is used for air drag, not for thrust, so
  nothing else flags the mistake.
* `AddWorldForce` with no offset applies the force at the link's **centre of
  mass**, not the link origin -- so the moment arm is the CoM position.
* **Thrust does not depend on `turningDirection`.** The expression reads
  `thrust = turningDirection * sign(v) * v^2 * motorConstant`, but the joint
  velocity is itself commanded as
  `turningDirection * refMotorRotVel / rotorVelocitySlowdownSim`, so
  `sign(v) == turningDirection` and the two factors cancel:
  `thrust = +motorConstant * w^2`, always along the rotor link's local **+Z**.
  Both rotors of a counter-rotating pair therefore push along their own
  link +Z whichever way they spin — which is exactly why each rotor link
  must be authored with +Z along its spin axis, pointing up.
* The rotor drag torque is `(0, 0, -turningDirection * thrust *
  momentConstant)` in the rotor link frame, applied to the *parent* link.
  This one **does** carry `turningDirection`, and it is the only thing that
  does: it is what gives a counter-rotating pair opposing yaw reactions.
  Getting this backwards leads to the wrong fix — do not reason about the
  thrust sign from `turningDirection`.
* `rotorVelocitySlowdownSim` only scales the *visual/physical* joint speed:
  the joint is commanded `refMotorRotVel / rotorVelocitySlowdownSim` and the
  force uses `jointVelocity * rotorVelocitySlowdownSim`, so thrust is
  unaffected by it.

## `force_torque` sensors

* The sensor is declared on a `<joint>`, and requires the
  `gz-sim-forcetorque-system` system plugin to be loaded. PX4 does **not**
  load it: `src/modules/simulation/gz_bridge/server.config` lists 14 systems
  and ForceTorque is not among them. Declaring the plugin at *model* scope
  inside the model SDF works and keeps the change out of PX4.
* **Both joint endpoints must be real links.** `ForceTorque.cc` resolves them
  with `GetLinkFromScopedName()`, which matches only entities with a `Link`
  component. A joint whose `<parent>` is `world` is skipped with
  *"Parent link with name [world] ... not found. Failed to create sensor"* --
  and no topic is advertised. Interpose a massless link and use two fixed
  joints.
* `<frame>` selects the orientation only (`parent`/`child`/`sensor`, via
  `rotationParentInSensor.Inverse()` etc. in gz-sensors
  `ForceTorqueSensor.cc`); torque is always reported about the sensor origin,
  which is the joint origin when the sensor carries no `<pose>`.
* `<measure_direction>child_to_parent</measure_direction>` reports the wrench
  the child exerts on the parent (a load cell reading); `parent_to_child`
  is the same values negated.
* The reading includes the **static weight of everything on the child side**
  of the joint. Verify a new fixture by checking the quiescent force against
  the summed link masses -- it is an exact, cheap check that the sensor is
  really sensing rather than being welded through.
