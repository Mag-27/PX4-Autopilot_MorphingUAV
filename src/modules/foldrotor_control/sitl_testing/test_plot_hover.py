#!/usr/bin/env python3
"""Headless test for plot_hover.build_figure -- no SITL/MAVLink required."""
import math
import unittest

import matplotlib
matplotlib.use("Agg")

from plot_hover import build_figure  # noqa: E402


def _cmd_sample(t, saturated=False):
    return (t, 5.0 + t, 6.0 - t, 0.1 * t, -0.1 * t, 0.05 * t, -0.05 * t, saturated)


def _act_sample(t):
    return (t, 0.09 * t, -0.09 * t, 0.04 * t, -0.04 * t)


def _wrench_sample(t):
    return (t, 0.3 * t, -0.02 * t, 0.1 * t)


class BuildFigureTest(unittest.TestCase):
    def test_four_axes_with_expected_data_line_counts(self):
        pos_buffer = [(t, math.sin(t), math.cos(t), -2.0) for t in range(10)]
        att_buffer = [(t, 0.01 * t, -0.01 * t, 0.0) for t in range(10)]
        cmd_buffer = [_cmd_sample(t) for t in range(10)]

        fig = build_figure(pos_buffer, att_buffer, cmd_buffer)

        self.assertEqual(len(fig.axes), 4)
        pos_ax, att_ax, force_ax, angle_ax = fig.axes
        self.assertEqual(len(pos_ax.lines), 3)
        self.assertEqual(len(att_ax.lines), 3)
        # 2 data lines (F1, F2) + 2 reference lines (max thrust, zero floor)
        self.assertEqual(len(force_ax.lines), 4)
        # 4 data lines (alpha1/2, beta1/2) + 2 reference lines (+/- tilt limit)
        self.assertEqual(len(angle_ax.lines), 6)

    def test_actual_angle_traces_are_added_when_act_buffer_given(self):
        cmd_buffer = [_cmd_sample(t) for t in range(10)]
        act_buffer = [_act_sample(t) for t in range(10)]

        fig = build_figure([], [], cmd_buffer, act_buffer)

        _pos_ax, _att_ax, _force_ax, angle_ax = fig.axes
        # 4 commanded + 4 actual data lines + 2 reference lines
        self.assertEqual(len(angle_ax.lines), 10)

    def test_missing_act_buffer_does_not_add_actual_traces(self):
        cmd_buffer = [_cmd_sample(t) for t in range(10)]

        fig_default = build_figure([], [], cmd_buffer)
        fig_empty = build_figure([], [], cmd_buffer, [])

        for fig in (fig_default, fig_empty):
            _pos_ax, _att_ax, _force_ax, angle_ax = fig.axes
            self.assertEqual(len(angle_ax.lines), 6)

    def test_wrench_buffer_adds_a_fifth_moment_subplot(self):
        """The commanded moment shares the time axis with the servo angles.

        Regression guard for the reason this subplot exists: alpha is an
        extremely stiff function of My (d(alpha)/d(My) ~= 2.9 rad/N*m at
        hover), so a servo-angle trace cannot be read on its own.
        """
        cmd_buffer = [_cmd_sample(t) for t in range(10)]
        wrench_buffer = [_wrench_sample(t) for t in range(10)]

        fig = build_figure([], [], cmd_buffer, [], wrench_buffer)

        self.assertEqual(len(fig.axes), 5)
        moment_ax = fig.axes[4]
        # Mx, My, Mz + the zero reference line
        self.assertEqual(len(moment_ax.lines), 4)
        # x label moves to the new bottom subplot, and only there
        self.assertEqual(moment_ax.get_xlabel(), "time [s]")
        self.assertEqual(fig.axes[3].get_xlabel(), "")

    def test_no_wrench_buffer_keeps_the_four_row_figure(self):
        cmd_buffer = [_cmd_sample(t) for t in range(10)]

        for fig in (build_figure([], [], cmd_buffer),
                    build_figure([], [], cmd_buffer, [], [])):
            self.assertEqual(len(fig.axes), 4)
            self.assertEqual(fig.axes[3].get_xlabel(), "time [s]")

    def test_saturated_window_is_shaded(self):
        cmd_buffer = [_cmd_sample(t, saturated=(2 <= t <= 4)) for t in range(10)]

        fig = build_figure([], [], cmd_buffer)

        _pos_ax, _att_ax, force_ax, angle_ax = fig.axes
        self.assertEqual(len(force_ax.patches), 1)
        self.assertEqual(len(angle_ax.patches), 1)

    def test_empty_buffers_do_not_raise(self):
        fig = build_figure([], [], [])
        self.assertEqual(len(fig.axes), 4)
        pos_ax, att_ax, force_ax, angle_ax = fig.axes
        self.assertEqual(len(pos_ax.lines), 0)
        self.assertEqual(len(att_ax.lines), 0)
        # reference lines are drawn even with no commanded data to plot
        self.assertEqual(len(force_ax.lines), 2)
        self.assertEqual(len(angle_ax.lines), 2)


if __name__ == "__main__":
    unittest.main()
