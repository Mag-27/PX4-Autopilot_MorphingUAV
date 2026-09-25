#!/usr/bin/env python3
"""CoM-referenced moment analysis, Phase 0 (offline, no Gazebo).

Question: for a wrench commanded through the REAL FoldrotorAllocation,
what force and moment does the vehicle's own geometry deliver about its
(moving) centre of mass -- and does it match what the allocator intended?

Pipeline:
  1. Build a table of body-FLU wrenches (the allocator's input frame).
  2. Run them through the C++ allocator via the DISABLED_DumpAllocation
     gtest -- the allocator is never re-implemented here (.claude/CLAUDE.md
     rule 7).
  3. Forward-map each rotor command through the SDF with
     expected_wrench.rotor_wrench(): full pose chain, joint angles
     injected, thrust at the rotor link CoM (= the real blade plane to
     within 0.3 mm; see that function's docstring).
  4. Resolve about the vehicle CoM computed AT THOSE JOINT ANGLES (the arms
     carry mass, so folding moves the CoM by centimetres).

Guard: force does not depend on any lever arm, so the SDF-delivered force
must equal the commanded force. If it does not, the joint mapping or a
sign convention between allocator and SDF is wrong and every moment below
is meaningless -- the script stops rather than print them.

See findings.md (21).
"""
import argparse
import csv
import math
import os
import subprocess
import sys
from pathlib import Path

import numpy as np

import expected_wrench as ew
from test_frame_convention import FLU_TO_FRD, MODEL_SDFS, _frames_of

REPO = Path(__file__).resolve().parent.parent
TEST_BIN = REPO / "build/px4_sitl_test/functional-FoldrotorControl"

WEIGHT = 19.6014        # FR_VEL_Z_GRAV_FF, N -- the hover lift, FLU +z
TRIM_MY_FLU = 0.0226    # steady pitch moment held in hover, log 2026-09-25/05_30_05
IDENTIFIED_STIFFNESS = 1.289   # N*m/rad restoring, output-error fit, findings (19)

JOINT_OF = {"alpha1": "Arm1FoldJoint", "beta1": "Arm1TiltJoint",
            "alpha2": "Arm2FoldJoint", "beta2": "Arm2TiltJoint"}


def tilt_compensation_flu(theta):
    """Body force that keeps lift vertical at pitch theta (FRD, nose-up +).

    Same operation as FoldrotorControl's inertialToBody(): F_b = R^T F_i with
    F_i = (0, 0, -W) NED and R the standard ZYX DCM for Euler(0, theta, 0).
    Returned in the allocator's FLU frame.
    """
    c, s = math.cos(theta), math.sin(theta)
    r_nb = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    f_frd = r_nb.T @ np.array([0.0, 0.0, -WEIGHT])
    return FLU_TO_FRD @ f_frd


def build_cases():
    cases = [("hover", (0, 0, WEIGHT), (0, 0, 0)),
             ("hover + measured pitch trim", (0, 0, WEIGHT), (0, TRIM_MY_FLU, 0))]
    for my in (0.02, 0.05, 0.1, 0.2):
        for sign in (+1, -1):
            cases.append((f"pure My {sign * my:+.2f}", (0, 0, WEIGHT), (0, sign * my, 0)))
    for fx in (1.0, 2.0, 3.0, 4.0):
        for sign in (+1, -1):
            cases.append((f"pure Fx {sign * fx:+.0f} N", (sign * fx, 0, WEIGHT), (0, 0, 0)))
    for label, f, m in (("pure Fy +1 N", (0, 1, WEIGHT), (0, 0, 0)),
                        ("pure Mx +0.2", (0, 0, WEIGHT), (0.2, 0, 0)),
                        ("pure Mz +0.2", (0, 0, WEIGHT), (0, 0, 0.2))):
        cases.append((label, f, m))
    for deg in (-10, -5, -2, -1, 1, 2, 5, 10):
        f = tilt_compensation_flu(math.radians(deg))
        cases.append((f"tilt comp theta {deg:+d} deg", tuple(f), (0, 0, 0)))
    return cases


def run_allocator(cases, workdir):
    cin, cout = workdir / "fr_alloc_in.csv", workdir / "fr_alloc_out.csv"
    with open(cin, "w") as fh:
        fh.write("fx,fy,fz,mx,my,mz\n")
        for _, f, m in cases:
            fh.write(",".join(f"{v:.7f}" for v in (*f, *m)) + "\n")
    env = dict(os.environ, FR_ALLOC_IN=str(cin), FR_ALLOC_OUT=str(cout))
    subprocess.run([str(TEST_BIN), "--gtest_also_run_disabled_tests",
                    "--gtest_filter=*DISABLED_DumpAllocation*"],
                   env=env, check=True, capture_output=True)
    with open(cout) as fh:
        rows = list(csv.DictReader(fh))
    assert len(rows) == len(cases), f"allocator returned {len(rows)} rows for {len(cases)} cases"
    return rows


def delivered_about_com(frames, root, motors, inertials, row):
    angles = {JOINT_OF[k]: float(row[k]) for k in JOINT_OF}
    rotor_of = {m["link"]: i for i, m in motors.items()}
    force, torque_mount = np.zeros(3), np.zeros(3)
    for link, thrust in (("Prop1Link", float(row["F1"])), ("Prop2Link", float(row["F2"]))):
        f, t = ew.rotor_wrench(frames, root, motors, rotor_of[link], thrust, angles)
        force += f
        torque_mount += t
    mass = sum(v[0] for v in inertials.values())
    com = sum(v[0] * ew._com_world(frames, root, k, v[1], angles)
              for k, v in inertials.items()) / mass
    return force, torque_mount - np.cross(com, force), com


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--workdir", default=os.environ.get("TMPDIR", "/tmp"))
    args = ap.parse_args()
    if not TEST_BIN.exists():
        sys.exit(f"{TEST_BIN} missing -- run `make tests TESTFILTER=Foldrotor` first")

    root, frames = _frames_of(MODEL_SDFS["flight"])
    motors, inertials = ew._motor_params(root), ew._inertials(root)
    cases = build_cases()
    rows = run_allocator(cases, Path(args.workdir))

    results = []
    worst_force = 0.0
    for (label, f_cmd, m_cmd), row in zip(cases, rows):
        f_del, m_del, com = delivered_about_com(frames, root, motors, inertials, row)
        f_err = np.abs(f_del - np.array(f_cmd)).max()
        worst_force = max(worst_force, f_err)
        results.append((label, np.array(f_cmd), np.array(m_cmd), f_del, m_del, com, row, f_err))

    print(f"GUARD: worst |F_delivered - F_commanded| over {len(cases)} cases = {worst_force:.2e} N")
    if worst_force > 1e-3:
        for r in results:
            print(f"   {r[0]:30s} F_cmd {np.round(r[1], 4)}  F_sdf {np.round(r[3], 4)}  err {r[7]:.2e}")
        sys.exit("STOP: allocator and SDF disagree on FORCE -- joint mapping or sign is wrong; "
                 "moment results would be meaningless.")
    print("       allocator -> SDF joint mapping and signs confirmed by force; moments follow.\n")

    to_frd = lambda v: FLU_TO_FRD @ v
    print(f"{'case':30s} {'sat':>3s} {'a1-a2 deg':>9s} {'b1 deg':>7s} | "
          f"{'My cmd FRD':>10s} {'My del FRD':>10s} {'My err':>8s} | {'Mx err':>7s} {'Mz err':>7s} | CoM z mm")
    for label, f_cmd, m_cmd, f_del, m_del, com, row, _ in results:
        mc, md = to_frd(m_cmd), to_frd(m_del)
        e = md - mc
        dfold = math.degrees(float(row["alpha1"]) - float(row["alpha2"]))
        print(f"{label:30s} {row['saturated']:>3s} {dfold:+9.2f} {math.degrees(float(row['beta1'])):+7.2f} | "
              f"{mc[1]:+10.4f} {md[1]:+10.4f} {e[1]:+8.4f} | {e[0]:+7.4f} {e[2]:+7.4f} | {1000 * com[2]:+7.2f}")

    # Equivalent pitch stiffness from the tilt-compensation cases: the moment
    # the geometry delivers that the allocator did NOT intend, per radian of
    # pitch. Restoring (opposing theta) is reported positive, matching the
    # sign of the flight-identified 1.289 N*m/rad.
    th, err = [], []
    for label, f_cmd, m_cmd, f_del, m_del, com, row, _ in results:
        if label.startswith("tilt comp"):
            th.append(math.radians(float(label.split()[3])))
            err.append((to_frd(m_del) - to_frd(m_cmd))[1])
    th, err = np.array(th), np.array(err)
    slope = np.polyfit(th, err, 1)[0]
    small = np.abs(th) <= math.radians(2.0)
    slope_small = np.polyfit(th[small], err[small], 1)[0]
    print(f"\nunintended pitch moment vs pitch angle (tilt-compensation cases):")
    print(f"   slope over +-10 deg : {slope:+.4f} N*m/rad  -> restoring stiffness {-slope:+.4f}")
    print(f"   slope over +-2 deg  : {slope_small:+.4f} N*m/rad  -> restoring stiffness {-slope_small:+.4f}")
    print(f"   flight-identified   : restoring {IDENTIFIED_STIFFNESS:+.3f} N*m/rad "
          f"-> geometry accounts for {100 * (-slope_small) / IDENTIFIED_STIFFNESS:.0f}%")


if __name__ == "__main__":
    main()
