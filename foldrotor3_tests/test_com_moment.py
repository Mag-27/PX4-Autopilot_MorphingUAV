"""CoM-referenced moment tests: the real allocator against the SDF geometry.

Runs the C++ FoldrotorAllocation (via the DISABLED_DumpAllocation gtest) and
forward-maps its commands through the flight model.sdf about the vehicle's
moving centre of mass -- see com_moment_analysis.py and findings.md (21).
Skipped when the PX4 unit-test binary has not been built.
"""
import math

import numpy as np
import pytest

import com_moment_analysis as cma
import expected_wrench as ew
from test_frame_convention import FLU_TO_FRD, MODEL_SDFS, _frames_of

pytestmark = pytest.mark.skipif(
    not cma.TEST_BIN.exists(),
    reason="needs build/px4_sitl_test/functional-FoldrotorControl (make tests)")


@pytest.fixture(scope="module")
def delivered(tmp_path_factory):
    root, frames = _frames_of(MODEL_SDFS["flight"])
    motors, inertials = ew._motor_params(root), ew._inertials(root)
    cases = cma.build_cases()
    rows = cma.run_allocator(cases, tmp_path_factory.mktemp("alloc"))
    out = {}
    for (label, f_cmd, m_cmd), row in zip(cases, rows):
        f_del, m_del, _ = cma.delivered_about_com(frames, root, motors, inertials, row)
        out[label] = (np.array(f_cmd), np.array(m_cmd), f_del, m_del, row)
    return out


def test_allocator_and_sdf_agree_on_force(delivered):
    """Pins the allocator <-> SDF joint mapping and every sign between them.

    Force is independent of any lever arm, so it must match exactly. A
    swapped servo, a negated angle or a wrong rotor index shows up here as
    newtons of error, before it can corrupt a moment comparison.
    """
    for label, (f_cmd, _, f_del, _, row) in delivered.items():
        assert row["saturated"] == "0", f"{label}: test case must be feasible"
        assert np.allclose(f_del, f_cmd, atol=1e-3), (
            f"{label}: SDF delivers {f_del}, allocator commanded {f_cmd}")


@pytest.mark.parametrize("my", [0.02, 0.05, 0.1, 0.2, -0.02, -0.05, -0.1, -0.2])
def test_pure_pitch_moment_is_delivered_about_the_com(delivered, my):
    """force_moment_test.md CoM acceptance: sign right, within 10% or 5 mN*m.

    Holds today (gain 1.00, constant +3.6 mN*m offset): the allocator's pitch
    MOMENT path is sound. The defect is in the force path, below.
    """
    f_cmd, m_cmd, _, m_del, _ = delivered[f"pure My {my:+.2f}"]
    cmd, got = (FLU_TO_FRD @ m_cmd)[1], (FLU_TO_FRD @ m_del)[1]
    assert np.sign(got) == np.sign(cmd)
    assert abs(got - cmd) <= max(0.1 * abs(cmd), 0.005)


@pytest.mark.xfail(strict=True, reason=(
    "KNOWN DEFECT, findings.md (21): FoldrotorAllocation's constant M0 assumes "
    "arms flat and thrust at the prop joint. Body-x force makes it command "
    "differential fold, which moves the rotors and the CoM, so ~0.06 N*m of "
    "pitch per newton leaks through -- 1.15 N*m/rad of spurious restoring "
    "stiffness in hover. strict=True: when the allocator is fixed this XPASSes "
    "and fails, forcing this marker to be removed."))
@pytest.mark.parametrize("fx", [1.0, 2.0, -1.0, -2.0])
def test_pure_body_x_force_produces_no_pitch_about_the_com(delivered, fx):
    """force_moment_test.md CoM acceptance: |My_com| <= 5 mN*m for pure Fx."""
    _, _, _, m_del, _ = delivered[f"pure Fx {fx:+.0f} N"]
    assert abs((FLU_TO_FRD @ m_del)[1]) <= 0.005


def test_geometric_pitch_stiffness_is_restoring_and_near_the_flight_fit(delivered):
    """Characterises the defect above so a change to it cannot go unnoticed.

    The unintended pitch moment per radian of tilt must stay restoring and
    close to the 1.15 N*m/rad this analysis found (89% of the 1.289 N*m/rad
    identified from flight). If the allocator is fixed this should approach
    zero -- update it together with the xfail above, not separately.
    """
    th, err = [], []
    for label, (_, m_cmd, _, m_del, _) in delivered.items():
        if label.startswith("tilt comp") and abs(float(label.split()[3])) <= 2:
            th.append(math.radians(float(label.split()[3])))
            err.append(((FLU_TO_FRD @ m_del) - (FLU_TO_FRD @ m_cmd))[1])
    restoring = -np.polyfit(th, err, 1)[0]
    assert restoring == pytest.approx(1.149, abs=0.02)
