#!/usr/bin/env python3
"""Expected bench-mount reaction wrench for the foldrotor3 force/moment test.

Derives, from the SDF alone, the sign and approximate magnitude of the wrench
that `bench_mount_joint`'s force_torque sensor should report for a given set of
actuator commands. This is the "expected" half of
`.claude/specs/force_moment_test.md`; the measured half comes from
`servo_load_test_logs/parse_wrench.py`.

It is deliberately independent of the simulator: nothing here reads a gz topic.
Frame math is reused from `test_frame_convention.py` rather than re-derived, per
that spec's methodology step 1.

Magnitudes are indicative only -- the sim thrust model is a single-point
quadratic fit. **Signs are the deliverable.**

Sign conventions taken from source, not assumed:

* `gz-sim-multicopter-motor-model-system` applies thrust along the *rotor
  link's local Z*, NOT along the joint axis:
  `AddWorldForce(R_link * (0, 0, thrust))`.
* Thrust is `+motorConstant * w^2` and does **not** depend on
  `turningDirection`. The source reads
  `thrust = td * sign(v) * v^2 * kf`, but the joint velocity is itself
  commanded as `td * refRotVel / slowdown`, so `sign(v) == td` and the two
  `td` factors cancel. Both rotors therefore push along their own link +Z
  whichever way they spin -- which is why the link frames must be authored
  with +Z along the spin axis, pointing up.
* The rotor drag torque is `(0, 0, -td * thrust * km)` in the rotor link
  frame, applied to the parent link. This one *does* carry `turningDirection`
  -- it is what gives the counter-rotating pair opposite yaw reactions.
* The sensor is `measure_direction=child_to_parent`, `frame=parent`, and
  `mount_plate` has an identity pose, so readings are in the world (gz FLU)
  orientation. `FLU_TO_FRD` converts to the body FRD the specs are written in.

Usage:
    python3 foldrotor3_tests/expected_wrench.py            # full case table
    python3 foldrotor3_tests/expected_wrench.py --frd      # in body FRD
"""
import argparse
import xml.etree.ElementTree as ET

import numpy as np

from test_frame_convention import (FLU_TO_FRD, MODEL_SDFS, _frames_of,
                                   _rpy_to_matrix, _transform_to_body_flu)

GRAVITY = np.array([0.0, 0.0, -9.8])  # worlds/default.sdf

# 4026_gz_foldrotor3: SIM_GZ_EC_MIN/MAX and SIM_GZ_SV_MINA/MAXA.
MOTOR_OMEGA_MIN = 308.0
MOTOR_OMEGA_MAX = 2054.0
SERVO_ANGLE_MAX = 0.79  # rad, matches the joint limit in model.sdf

SERVO_JOINTS = {
    1: "Arm1FoldJoint",
    2: "Arm1TiltJoint",
    3: "Arm2FoldJoint",
    4: "Arm2TiltJoint",
}


def _motor_params(root):
    """{motor_index: dict} from the MulticopterMotorModel plugin blocks."""
    out = {}
    for plugin in root.findall("plugin"):
        if "multicopter-motor-model" not in (plugin.get("filename") or ""):
            continue
        td = plugin.findtext("turningDirection").strip()
        out[int(plugin.findtext("motorNumber")) + 1] = {
            "link": plugin.findtext("linkName").strip(),
            "joint": plugin.findtext("jointName").strip(),
            "turning": 1.0 if td == "ccw" else -1.0,
            "kf": float(plugin.findtext("motorConstant")),
            "km": float(plugin.findtext("momentConstant")),
        }
    return out


def _inertials(root):
    """{link_name: (mass, com_offset_in_link_frame)}, excluding the mount."""
    out = {}
    for link in root.findall("link"):
        inertial = link.find("inertial")
        if inertial is None or link.get("name") == "mount_plate":
            continue  # mount_plate sits on the parent side of the sensor
        pose = inertial.find("pose")
        com = (np.array([float(x) for x in pose.text.split()[:3]])
               if pose is not None else np.zeros(3))
        out[link.get("name")] = (float(inertial.findtext("mass")), com)
    return out


def _axis_of(root, joint_name):
    joint = next(j for j in root.findall("joint") if j.get("name") == joint_name)
    axis = np.array([float(x) for x in joint.find("axis/xyz").text.split()])
    return axis / np.linalg.norm(axis)


def _rotation_about(axis, angle):
    """Rodrigues rotation, as a 4x4 so it composes with the pose transforms."""
    k = np.array([[0, -axis[2], axis[1]],
                  [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    t = np.eye(4)
    t[:3, :3] = (np.eye(3) + np.sin(angle) * k
                 + (1 - np.cos(angle)) * (k @ k))
    return t


def transform_with_angles(frames, root, name, angles):
    """`_transform_to_body_flu` with revolute joints displaced by `angles`.

    `angles` maps joint name -> radians. A joint rotation applies to
    everything below it, so it is injected at the joint's own frame during
    the same parent-chain walk the static helper does.
    """
    transform = np.eye(4)
    while name is not None:
        parent, local = frames[name]
        if name in angles:
            local = local @ _rotation_about(_axis_of(root, name), angles[name])
        transform = local @ transform
        name = parent
    return transform


def _com_world(frames, root, link, com_offset, angles):
    t = transform_with_angles(frames, root, link, angles)
    return t[:3, :3] @ com_offset + t[:3, 3]


def gravity_wrench(frames, root, inertials, angles):
    """Weight and its moment about the mount origin, in world (FLU)."""
    force = np.zeros(3)
    torque = np.zeros(3)
    for link, (mass, com) in inertials.items():
        w = mass * GRAVITY
        force += w
        torque += np.cross(_com_world(frames, root, link, com, angles), w)
    return force, torque


def motor_wrench(frames, root, motors, motor_index, value, angles):
    """Thrust, its moment about the mount origin, and rotor drag torque."""
    m = motors[motor_index]
    omega = MOTOR_OMEGA_MIN + value * (MOTOR_OMEGA_MAX - MOTOR_OMEGA_MIN)
    force, torque = rotor_wrench(frames, root, motors, motor_index,
                                 m["kf"] * omega ** 2, angles)
    return force, torque, omega


def rotor_wrench(frames, root, motors, motor_index, magnitude, angles):
    """Thrust of a given MAGNITUDE (N), its moment about the mount origin,
    and rotor drag torque -- motor_wrench() without the omega mapping.

    Split out 2026-09-25 so the CoM-referenced moment analysis can drive
    the model with FoldrotorAllocation's per-rotor thrust directly. The
    thrust point is the rotor LINK's CoM, which is where gz applies it --
    and, measured against the prop mesh, sits inside the blade band
    (+4.6..+7.4 mm above the joint, CoM +5.74 mm), so it is also the real
    vehicle's thrust point to within 0.3 mm. See findings.md (21).
    """
    m = motors[motor_index]
    t = transform_with_angles(frames, root, m["link"], angles)
    rotation = t[:3, :3]

    force = rotation @ np.array([0.0, 0.0, magnitude])
    drag = rotation @ np.array([0.0, 0.0, -m["turning"] * magnitude * m["km"]])

    # AddWorldForce with no offset applies the force at the link's CoM.
    _, com = _inertials(root)[m["link"]]
    r = rotation @ com + t[:3, 3]
    return force, np.cross(r, force) + drag


def case(frames, root, motors, inertials, active_motors, angles):
    """Total expected sensor reading (world FLU) for one actuator case."""
    force, torque = gravity_wrench(frames, root, inertials, angles)
    speeds = {}
    for index, value in active_motors.items():
        f, t, omega = motor_wrench(frames, root, motors, index, value, angles)
        force += f
        torque += t
        speeds[index] = omega
    return force, torque, speeds


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frd", action="store_true",
                        help="report in body FRD instead of world FLU")
    parser.add_argument("--motor-value", type=float, default=0.6,
                        help="actuator_test -v for the motor cases")
    parser.add_argument("--servo-value", type=float, default=0.5,
                        help="actuator_test -v for the servo cases")
    args = parser.parse_args()

    root, frames = _frames_of(MODEL_SDFS["bench"])
    motors = _motor_params(root)
    inertials = _inertials(root)

    mv, sv = args.motor_value, args.servo_value
    servo_angle = sv * SERVO_ANGLE_MAX

    cases = [("baseline (all off)", {}, {})]
    cases += [(f"-m {i} -v {mv}", {i: mv}, {}) for i in sorted(motors)]
    cases += [(f"-s {n} -v {sv}", {}, {SERVO_JOINTS[n]: servo_angle})
              for n in sorted(SERVO_JOINTS)]
    cases += [
        (f"-m 1 -m 2 -v {mv}", {1: mv, 2: mv}, {}),
        (f"-m 1 -v {mv} + -s 2 -v {sv}", {1: mv},
         {SERVO_JOINTS[2]: servo_angle}),
    ]

    base_f, base_t, _ = case(frames, root, motors, inertials, {}, {})
    rot = FLU_TO_FRD if args.frd else np.eye(3)
    label = "body FRD (X fwd, Y right, Z down)" if args.frd else "world FLU (gz)"

    print(f"Expected bench_mount_joint reaction wrench -- {label}")
    print("Absolute reading, then delta vs the all-off baseline.")
    print("Signs are the deliverable; magnitudes are indicative "
          "(single-point quadratic thrust fit).\n")
    header = (f"{'case':<26}{'Fx':>8}{'Fy':>8}{'Fz':>8}   "
              f"{'dFx':>8}{'dFy':>8}{'dFz':>8}   "
              f"{'dMx':>8}{'dMy':>8}{'dMz':>8}")
    print(header)
    print("-" * len(header))
    for name, active, angles in cases:
        f, t, speeds = case(frames, root, motors, inertials, active, angles)
        fr, tr = rot @ f, rot @ t
        df, dt = rot @ (f - base_f), rot @ (t - base_t)
        print(f"{name:<26}"
              + "".join(f"{v:8.3f}" for v in fr) + "   "
              + "".join(f"{v:8.3f}" for v in df) + "   "
              + "".join(f"{v:8.4f}" for v in dt)
              + ("   omega=" + ",".join(f"{v:.0f}" for v in speeds.values())
                 if speeds else ""))


if __name__ == "__main__":
    main()
