"""Frame-convention regression tests for the foldrotor3 Gazebo model.

Pure SDF/STL geometry — no Gazebo, no PX4 build. Pins the body-frame
contract in .claude/specs/system.md so an orientation regression fails
here rather than being discovered by eye in the gz GUI.

All assertions are in PX4 body FRD (X fwd, Y right, Z down), NOT the gz
FLU frame the SDF is authored in (Y left, Z up). The two differ by 180
deg about X, so a contract stated in the wrong one silently inverts roll.
"""
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

MODELS_DIR = (Path(__file__).resolve().parent.parent
              / "Tools/simulation/gz/models")

# Both files are exercised: foldrotor3_bench is a full textual duplicate of
# the flight model (not an <include>), which is exactly how commit 00267a4's
# arm-alignment fix landed in one and not the other. Running every assertion
# against both is what stops that recurring.
MODEL_SDFS = {
    "flight": MODELS_DIR / "foldrotor3" / "model.sdf",
    "bench": MODELS_DIR / "foldrotor3_bench" / "model.sdf",
}

FLU_TO_FRD = np.diag([1.0, -1.0, -1.0])


def _rpy_to_matrix(r, p, y):
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx  # SDF <pose> RPY is extrinsic X-Y-Z


def _pose_to_transform(text):
    v = [float(x) for x in text.split()]
    t = np.eye(4)
    t[:3, :3] = _rpy_to_matrix(*v[3:6])
    t[:3, 3] = v[0:3]
    return t


def _load_stl_vertices(path):
    data = path.read_bytes()
    if data[:5].lower().lstrip() == b"solid" and b"facet" in data[:2000]:
        verts = [[float(x) for x in line.split()[1:4]]
                 for line in data.decode("ascii", "ignore").splitlines()
                 if line.split()[:1] == ["vertex"]]
        return np.array(verts)

    count = struct.unpack("<I", data[80:84])[0]
    verts = np.zeros((count * 3, 3))
    offset = 84
    for i in range(count):
        vals = struct.unpack("<12f", data[offset:offset + 48])
        verts[i * 3:i * 3 + 3] = np.array(vals[3:12]).reshape(3, 3)
        offset += 50
    return verts


def _frames_of(sdf_path):
    root = ET.parse(sdf_path).getroot().find("model")
    frames = {}
    for element in list(root.findall("joint")) + list(root.findall("link")):
        pose = element.find("pose")
        if pose is None:
            frames[element.get("name")] = (None, np.eye(4))
        else:
            frames[element.get("name")] = (pose.get("relative_to"),
                                           _pose_to_transform(pose.text))
    return root, frames


@pytest.fixture(scope="module", params=sorted(MODEL_SDFS))
def model(request):
    return _frames_of(MODEL_SDFS[request.param])


def _transform_to_body_flu(frames, name):
    transform = np.eye(4)
    while name is not None:
        parent, local = frames[name]
        transform = local @ transform
        name = parent
    return transform


def _origin_frd(frames, name):
    return FLU_TO_FRD @ _transform_to_body_flu(frames, name)[:3, 3]


def _resolve_mesh_uri(uri):
    """model://<model>/meshes/<file> -> a path under MODELS_DIR.

    The bench model reuses the flight model's meshes by URI rather than
    duplicating them, so the mesh does not necessarily live beside the SDF
    that references it.
    """
    stripped = uri.removeprefix("model://")
    return MODELS_DIR / stripped


def _mesh_bounds_frd(root, frames, link_name):
    link = next(x for x in root.findall("link") if x.get("name") == link_name)
    uri = link.find("visual").find(".//uri")
    verts = _load_stl_vertices(_resolve_mesh_uri(uri.text))
    transform = _transform_to_body_flu(frames, link_name)
    in_body = (transform[:3, :3] @ verts.T).T + transform[:3, 3]
    in_frd = (FLU_TO_FRD @ in_body.T).T
    return in_frd.min(axis=0), in_frd.max(axis=0)


def test_arm1_on_positive_y_arm2_on_negative_y(model):
    """Lateral side-by-side rotor pair: arm1 right (+Y FRD), arm2 left (-Y FRD).

    Matches s1y=+0.15 / s2y=-0.15 in the MATLAB allocator.
    """
    _, frames = model
    arm1 = _origin_frd(frames, "Arm1TiltLink")
    arm2 = _origin_frd(frames, "Arm2TiltLink")

    assert arm1[1] > 0.1, f"Arm1 must lie on body +Y (FRD), got y={arm1[1]:.4f}"
    assert arm2[1] < -0.1, f"Arm2 must lie on body -Y (FRD), got y={arm2[1]:.4f}"
    assert abs(arm1[0]) < 0.05, f"Arm1 off the Y axis: x={arm1[0]:.4f}"
    assert abs(arm2[0]) < 0.05, f"Arm2 off the Y axis: x={arm2[0]:.4f}"


def test_arm_meshes_do_not_straddle_the_origin(model):
    """Rendered geometry must agree with the joint origins, not just the frames."""
    root, frames = model
    arm1_min, _ = _mesh_bounds_frd(root, frames, "Arm1TiltLink")
    _, arm2_max = _mesh_bounds_frd(root, frames, "Arm2TiltLink")

    assert arm1_min[1] > 0.0, f"Arm1 mesh crosses into -Y: min y={arm1_min[1]:.4f}"
    assert arm2_max[1] < 0.0, f"Arm2 mesh crosses into +Y: max y={arm2_max[1]:.4f}"


def test_props_are_mirrored_about_the_body_origin(model):
    """Guards the Prop1Joint/Prop1Link.STL offset bug (both props once stacked at one point)."""
    _, frames = model
    prop1 = _origin_frd(frames, "Prop1Link")
    prop2 = _origin_frd(frames, "Prop2Link")

    assert prop1[1] > 0 > prop2[1], "props must sit on opposite sides of the body origin"
    assert abs(prop1[1] + prop2[1]) < 1e-3, (
        f"props not mirrored in Y: {prop1[1]:.5f} vs {prop2[1]:.5f}")
    assert abs(prop1[2] - prop2[2]) < 1e-3, (
        f"props not level in Z: {prop1[2]:.5f} vs {prop2[2]:.5f}")


def test_cad_y_up_maps_to_body_up(model):
    """airframe_link_joint converts the CAD export's Y-up to body up (-Z in FRD)."""
    _, frames = model
    rotation = _transform_to_body_flu(frames, "airframe_link")[:3, :3]
    cad_up = FLU_TO_FRD @ (rotation @ np.array([0.0, 1.0, 0.0]))

    assert cad_up[2] < -0.99, (
        f"model is not upright: CAD +Y maps to {cad_up} in body FRD")


def test_bench_airframe_orientation_matches_flight():
    """The bench fixture must carry the flight model's airframe alignment.

    foldrotor3_bench/model.sdf is a copy, not an <include>, so a fix applied
    to the flight file can silently miss it -- which is what happened to the
    -90 deg yaw in commit 00267a4, leaving the bench arms on the wrong axis.
    Compare the resolved rotations rather than the pose text so an equivalent
    but differently-written pose still passes.
    """
    _, flight = _frames_of(MODEL_SDFS["flight"])
    _, bench = _frames_of(MODEL_SDFS["bench"])

    flight_r = _transform_to_body_flu(flight, "airframe_link")[:3, :3]
    bench_r = _transform_to_body_flu(bench, "airframe_link")[:3, :3]

    assert np.allclose(flight_r, bench_r, atol=1e-9), (
        "bench airframe_link orientation has drifted from the flight model:\n"
        f"flight=\n{flight_r}\nbench=\n{bench_r}")


ROTORS = [("Prop1Link", "Prop1Joint"), ("Prop2Link", "Prop2Joint")]


def _joint_axis(root, joint_name):
    joint = next(j for j in root.findall("joint") if j.get("name") == joint_name)
    axis = np.array([float(x) for x in joint.find("axis/xyz").text.split()])
    return axis / np.linalg.norm(axis)


@pytest.mark.parametrize("link,joint", ROTORS)
def test_rotor_thrust_axis_is_the_spin_axis_and_points_up(model, link, joint):
    """The rotor link's local +Z must be the spin axis, pointing up.

    gz-sim-multicopter-motor-model-system applies thrust along the rotor
    *link's* local Z (`AddWorldForce(R_link * (0,0,thrust))`), not along the
    joint axis, and the thrust sign does not depend on turningDirection.
    foldrotor3's CAD export is Y-up, so this was originally violated: the
    rotor links' local Z lay in the horizontal plane, perpendicular to their
    own spin axis. The thrust vector then rotated with the propeller and
    averaged to zero -- measured net lift was 0.03 N against a 15.26 N
    airframe weight. See .claude/specs/force_moment_test.md.
    """
    root, frames = model
    rotation = _transform_to_body_flu(frames, link)[:3, :3]
    thrust_axis = FLU_TO_FRD @ (rotation @ np.array([0.0, 0.0, 1.0]))
    spin_axis = FLU_TO_FRD @ (rotation @ _joint_axis(root, joint))

    assert np.allclose(thrust_axis, spin_axis, atol=1e-3), (
        f"{link} thrust axis {thrust_axis} is not its spin axis {spin_axis}; "
        "thrust will rotate with the propeller and average to zero")
    assert thrust_axis[2] < -0.999, (
        f"{link} thrust axis {thrust_axis} does not point up (body -Z in FRD)")


def test_the_two_rotors_counter_rotate(model):
    """Opposite turningDirection is what cancels yaw on this side-by-side pair.

    It does not affect thrust direction (the two `turningDirection` factors in
    the plugin's thrust expression cancel), only the drag-torque sign.
    """
    root, _ = model
    directions = {}
    for plugin in root.findall("plugin"):
        if "multicopter-motor-model" in (plugin.get("filename") or ""):
            directions[plugin.findtext("jointName").strip()] = \
                plugin.findtext("turningDirection").strip()

    assert set(directions) == {"Prop1Joint", "Prop2Joint"}, directions
    assert directions["Prop1Joint"] != directions["Prop2Joint"], (
        f"both rotors spin the same way ({directions}); yaw reactions will add "
        "instead of cancelling")
