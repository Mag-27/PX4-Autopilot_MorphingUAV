#!/usr/bin/env python3
"""Headless tests for plot_sitl_log -- no SITL, Gazebo or ULog file required.

Run: python3 -m pytest src/modules/foldrotor_control/sitl_testing/test_plot_sitl_log.py
"""
import math
import unittest
from types import SimpleNamespace

import matplotlib
matplotlib.use("Agg")

import plot_sitl_log as psl  # noqa: E402


def _fake_ulog():
    """Minimal stand-in for pyulog.ULog with the five topics extract() reads."""
    t0 = 1_000_000
    ts = [t0 + i * 100_000 for i in range(10)]  # 10 samples, 0.1 s apart
    yaw = math.radians(90.0)
    q = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))

    da = {"timestamp": ts + [ts[-1]], "id": [0] * 10 + [1]}
    for slot in range(58):
        da[f"data[{slot}]"] = [0.0] * 11
    da["data[0]"] = [5.0] * 10 + [99.0]            # F1; the id=1 row must be dropped
    da["data[1]"] = [6.0] * 11                     # F2
    da["data[2]"] = [math.radians(10.0)] * 11      # alpha1
    da["data[5]"] = [math.radians(-20.0)] * 11     # beta2
    da["data[6]"] = [0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0]  # saturated

    topics = {
        "vehicle_local_position": {"timestamp": ts, "x": [1.0] * 10, "y": [2.0] * 10, "z": [-1.5] * 10},
        "trajectory_setpoint": {"timestamp": ts[:2], "position[0]": [0.0] * 2,
                                "position[1]": [0.0] * 2, "position[2]": [-1.5] * 2},
        "vehicle_attitude": {"timestamp": ts, "q[0]": [q[0]] * 10, "q[1]": [q[1]] * 10,
                             "q[2]": [q[2]] * 10, "q[3]": [q[3]] * 10},
        "debug_array": da,
        "vehicle_status": {"timestamp": [ts[0], ts[3], ts[7]], "arming_state": [1, 2, 1]},
    }
    return SimpleNamespace(
        start_timestamp=t0,
        data_list=[SimpleNamespace(name=n, multi_id=0, data=d) for n, d in topics.items()])


class ExtractTest(unittest.TestCase):
    def setUp(self):
        self.s = psl.extract(_fake_ulog())

    def test_time_base_is_seconds_since_log_start(self):
        self.assertAlmostEqual(self.s["pos"][0][0], 0.0)
        self.assertAlmostEqual(self.s["pos"][0][-1], 0.9)

    def test_alloc_slots_decoded_and_other_ids_dropped(self):
        a = self.s["alloc"]
        self.assertEqual(len(a["t"]), 10)
        self.assertEqual(a["F1"], [5.0] * 10)
        self.assertEqual(a["F2"], [6.0] * 10)
        self.assertAlmostEqual(math.degrees(a["alpha1"][0]), 10.0)
        self.assertAlmostEqual(math.degrees(a["beta2"][0]), -20.0)
        self.assertEqual(a["saturated"][:4], [False, False, True, True])

    def test_attitude_converted_to_euler(self):
        _, r, p, y = self.s["att"]
        self.assertAlmostEqual(r[0], 0.0)
        self.assertAlmostEqual(p[0], 0.0)
        self.assertAlmostEqual(math.degrees(y[0]), 90.0)

    def test_arm_edges(self):
        edges = [(round(t, 3), armed) for t, armed in self.s["arm_times"]]
        self.assertEqual(edges, [(0.3, True), (0.7, False)])

    def test_missing_topics_do_not_raise(self):
        s = psl.extract(SimpleNamespace(start_timestamp=0, data_list=[]))
        self.assertIsNone(s["alloc"])
        psl.build_figure(s)


class HelpersTest(unittest.TestCase):
    def test_quat_to_euler_pure_pitch(self):
        th = math.radians(15.0)
        _, p, _ = psl.quat_to_euler(math.cos(th / 2), 0.0, math.sin(th / 2), 0.0)
        self.assertAlmostEqual(math.degrees(p), 15.0)

    def test_disarmed_spans(self):
        self.assertEqual(psl.disarmed_spans([(3.0, True), (7.0, False)], 10.0), [(0.0, 3.0), (7.0, 10.0)])
        self.assertEqual(psl.disarmed_spans([], 10.0), [(0.0, 10.0)])


class BuildFigureTest(unittest.TestCase):
    def test_four_panels_with_requested_series(self):
        fig = psl.build_figure(psl.extract(_fake_ulog()))
        self.assertEqual(len(fig.axes), 4)
        labels = [set(ln.get_label() for ln in ax.lines) for ax in fig.axes]
        self.assertTrue({"x (N)", "y (E)", "z (D)"} <= labels[0])
        self.assertTrue({"roll", "pitch", "yaw"} <= labels[1])
        self.assertTrue({"alpha1 (fold)", "alpha2 (fold)", "beta1 (tilt)", "beta2 (tilt)"} <= labels[2])
        self.assertTrue({"F1", "F2"} <= labels[3])


if __name__ == "__main__":
    unittest.main()
