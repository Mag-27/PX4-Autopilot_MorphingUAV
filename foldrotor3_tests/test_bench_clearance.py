"""Ground-clearance regression test for the foldrotor3 bench fixture.

Pure SDF/STL geometry -- no Gazebo. The bench fixture bolts the airframe to
the world at a fixed height; if any in-scope fold/tilt command swings a link
below the world's ground plane, the resulting contact silently offloads part
of the airframe's weight off bench_mount_joint and injects friction forces,
corrupting the force_torque reading the force/moment direction test depends on.

This was not hypothetical: at the original 0.5 -> 0.1 m stand height, an
`actuator_test set -s 1 -v 0.5` (Arm1FoldJoint, +0.395 rad) put Arm1TiltLink
18 mm through the ground and moved ~9.5 N off the mount. See
.claude/specs/force_moment_test.md.
"""
import numpy as np
import pytest

from expected_wrench import SERVO_ANGLE_MAX, SERVO_JOINTS, transform_with_angles
from test_frame_convention import (FLU_TO_FRD, MODEL_SDFS, _frames_of,
                                   _load_stl_vertices, _pose_to_transform,
                                   _resolve_mesh_uri)

# The largest commanded deflection the force/moment test uses, in both
# directions -- the fixture must clear the ground across the whole range.
TEST_ANGLES = [+0.5 * SERVO_ANGLE_MAX, -0.5 * SERVO_ANGLE_MAX]

# Enough margin that mesh-vs-collision differences and settling transients
# cannot bring a link into contact.
MIN_CLEARANCE_M = 0.05


@pytest.fixture(scope="module")
def bench():
    root, frames = _frames_of(MODEL_SDFS["bench"])
    stand_height = float(root.find("pose").text.split()[2])
    return root, frames, stand_height


def _visual_points(visual):
    """Points bounding one <visual>, in the visual's own frame.

    Meshes use their STL vertices. Cylinders are sampled around both rims,
    which bounds their lowest point under any rotation. Added 2026-09-25 with
    the ballast mast, whose disc and mast are primitive cylinders: the
    previous mesh-only version read `.find(".//uri").text` and crashed on
    them, and a version that SKIPPED primitives instead would have silently
    stopped checking the lowest thing on the vehicle -- the 0.24 m disc
    0.3 m below base_link. Anything else raises, for the same reason.
    """
    geometry = visual.find("geometry")
    uri = geometry.find(".//uri")
    if uri is not None:
        return _load_stl_vertices(_resolve_mesh_uri(uri.text))

    cylinder = geometry.find("cylinder")
    if cylinder is not None:
        radius = float(cylinder.findtext("radius"))
        half = 0.5 * float(cylinder.findtext("length"))
        theta = np.linspace(0.0, 2.0 * np.pi, 64, endpoint=False)
        ring = np.c_[radius * np.cos(theta), radius * np.sin(theta)]
        return np.vstack([np.c_[ring, np.full(len(ring), z)] for z in (-half, half)])

    raise NotImplementedError(
        f"clearance check cannot bound visual '{visual.get('name')}' geometry "
        f"{[c.tag for c in geometry]}; add it here rather than skipping it")


def _lowest_world_z(root, frames, stand_height, angles):
    """Height above the world ground plane of the model's lowest point."""
    lowest, culprit = np.inf, None
    for link in root.findall("link"):
        name = link.get("name")
        visuals = link.findall("visual")
        if not visuals:
            continue
        transform = transform_with_angles(frames, root, name, angles)
        for visual in visuals:
            pose = visual.find("pose")
            local = np.eye(4) if pose is None else _pose_to_transform(pose.text)
            full = transform @ local
            verts = _visual_points(visual)
            in_body = (full[:3, :3] @ verts.T).T + full[:3, 3]
            in_frd = (FLU_TO_FRD @ in_body.T).T
            # FRD z is down and the model origin sits at stand_height.
            z = stand_height - in_frd[:, 2].max()
            if z < lowest:
                lowest, culprit = z, f"{name}/{visual.get('name')}"
    return lowest, culprit


def test_fixture_clears_the_ground_at_rest(bench):
    root, frames, stand_height = bench
    z, culprit = _lowest_world_z(root, frames, stand_height, {})
    assert z > MIN_CLEARANCE_M, (
        f"bench fixture sits {z:.4f} m above ground at rest ({culprit}); "
        f"raise the model <pose> z (currently {stand_height})")


@pytest.mark.parametrize("servo", sorted(SERVO_JOINTS))
@pytest.mark.parametrize("angle", TEST_ANGLES)
def test_fixture_clears_the_ground_under_servo_command(bench, servo, angle):
    root, frames, stand_height = bench
    joint = SERVO_JOINTS[servo]
    z, culprit = _lowest_world_z(root, frames, stand_height, {joint: angle})
    assert z > MIN_CLEARANCE_M, (
        f"servo {servo} ({joint}) at {angle:+.3f} rad brings {culprit} to "
        f"{z:.4f} m above ground -- ground contact will corrupt the "
        f"bench_mount_joint wrench; raise the model <pose> z "
        f"(currently {stand_height})")


def test_ballast_disc_is_the_lowest_point_and_clears_the_ground(bench):
    """The mast's disc is the lowest thing on the bench vehicle -- and clears.

    Pins that the clearance check actually SEES the ballast rather than
    passing because it looked elsewhere: at rest the culprit must be the
    disc, at the height the geometry predicts (0.5 - 0.3 - 0.01 = 0.19 m).
    """
    root, frames, stand_height = bench
    z, culprit = _lowest_world_z(root, frames, stand_height, {})
    assert culprit == "ballast_link/ballast_link_visual", (
        f"lowest point is {culprit}, not the ballast disc -- is the mast still on the bench?")
    assert z == pytest.approx(stand_height - 0.3 - 0.01, abs=1e-6)
    assert z > MIN_CLEARANCE_M
