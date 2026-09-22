#!/usr/bin/env python3
"""Live position/orientation/allocation plotter for foldrotor_control SITL testing.

Connects to an already-running SITL instance over MAVLink, buffers
LOCAL_POSITION_NED, ATTITUDE and the module's DEBUG_FLOAT_ARRAY allocation
output, and pops open a matplotlib window with position/orientation/
commanded-force/commanded-servo-angle traces as soon as it stops receiving
data -- whether that's because you Ctrl+C'd this script or because the px4
process itself was Ctrl+C'd in its own terminal.

Passive only: does not arm, switch modes, or send setpoints. Run it
alongside whatever is actually flying the vehicle (QGC, RC, an offboard
script). See README.md for the SITL launch recipe this is meant to run
against, and FoldrotorControl.hpp/.cpp for where the DEBUG_FLOAT_ARRAY
("fr_alloc") is published. ONE message per cycle carries both halves:
data[0:7]  = F1, F2, alpha1, alpha2, beta1, beta2, saturated  (allocator OUT)
data[7:13] = Fx, Fy, Fz, Mx, My, Mz (FLU, post-fit)           (allocator IN)

That single-message layout is not cosmetic. debug_array is a single-instance
uORB topic with queue depth 1, so two publishes in one cycle do not produce
two messages -- the second destroys the first for every subscriber slower
than the control loop, which is the logger and the MAVLink stream both. A
"fr_wrench" diagnostic published that way silently blanked these plots for a
whole flight-test session (findings.md 2026-09-21). Keep it to one publish;
the array has 58 slots.

No core PX4 module (mavlink included) is modified to get this:
DEBUG_FLOAT_ARRAY is an already-registered generic MAVLink stream.
"""
import argparse
import math
import os
import time

# Must be set before importing pymavlink: DEBUG_FLOAT_ARRAY only exists in
# the MAVLink 2 message set.
os.environ.setdefault("MAVLINK20", "1")

import gz.transport13  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from gz.msgs.model_pb2 import Model  # noqa: E402
from pymavlink import mavutil  # noqa: E402

HEARTBEAT_LOSS_TIMEOUT_S = 3.0
ALLOC_ARRAY_NAME = "fr_alloc"

# allocation.md's alpha/beta <-> joint naming, used to label the
# gz-sim-joint-state-publisher-system "joint_state" topic's per-joint
# axis1.position readback (Tools/simulation/gz/models/foldrotor3/model.sdf).
JOINT_TO_LABEL = {
    "Arm1FoldJoint": "alpha1",
    "Arm1TiltJoint": "beta1",
    "Arm2FoldJoint": "alpha2",
    "Arm2TiltJoint": "beta2",
}

# allocation.md actuator limits (FoldrotorAllocation.hpp kMaxThrust/kMaxTilt)
# -- drawn as reference lines so a saturated command is visible, not just
# implied by the "saturated" flag.
MAX_THRUST_N = 15.0
MAX_TILT_DEG = math.degrees(0.79)

# references/palette.md categorical slots 1-3 (blue, orange, aqua). Slots
# 1/2 double as the per-rotor hue, held consistent across the force and
# servo-angle subplots so "rotor 1" means the same color everywhere on the
# page; slot 3 is used only for the position/orientation z/yaw traces.
AXIS1_COLOR = "#2a78d6"
AXIS2_COLOR = "#eb6834"
AXIS3_COLOR = "#1baf7a"
ROTOR1_COLOR = AXIS1_COLOR
ROTOR2_COLOR = AXIS2_COLOR
LIMIT_COLOR = "#898781"       # muted ink, reference lines
GRID_COLOR = "#e1e0d9"        # hairline gridline
SATURATED_COLOR = "#ec835a"   # status/serious -- saturation-window shading


def _saturated_spans(cmd_buffer):
    """Contiguous (t_start, t_end) windows where the saturated flag is set."""
    spans = []
    span_start = None
    for t, _f1, _f2, _a1, _a2, _b1, _b2, saturated in cmd_buffer:
        if saturated and span_start is None:
            span_start = t
        elif not saturated and span_start is not None:
            spans.append((span_start, t))
            span_start = None
    if span_start is not None:
        spans.append((span_start, cmd_buffer[-1][0]))
    return spans


def build_figure(pos_buffer, att_buffer, cmd_buffer, act_buffer=None, wrench_buffer=None):
    """Build a position/orientation/force/servo-angle figure from buffered samples.

    pos_buffer: list of (t, x, y, z) tuples (meters, NED).
    att_buffer: list of (t, roll, pitch, yaw) tuples (radians).
    cmd_buffer: list of (t, F1, F2, alpha1, alpha2, beta1, beta2, saturated)
        tuples (F in newtons; alpha/beta in radians; saturated is bool-like) --
        the allocator's COMMANDED servo angles.
    wrench_buffer: optional list of (t, Mx, My, Mz) tuples (N*m, body FLU) --
        the moment the rate loop asked the allocator for. Plotted against
        the servo angles below it because the two are tightly coupled: at
        hover d(alpha)/d(My) = 2.9 rad/N*m, i.e. 0.1 N*m of pitch command
        moves each fold joint ~17 deg in opposite directions. Reading the
        angle subplot without this one cannot tell a big servo command from
        a big moment command. None or empty omits the subplot entirely.
    act_buffer: optional list of (t, alpha1, beta1, alpha2, beta2) tuples
        (radians) -- the ACTUAL simulated joint angle, read back from Gazebo's
        gz-sim-joint-state-publisher-system (see JOINT_TO_LABEL). None or
        empty draws the commanded traces only, unchanged from before this was
        added.
    Pure function, no MAVLink/gz-transport dependency -- safe to unit test
    headlessly. One y-axis per subplot throughout (never a dual-axis chart).
    """
    n_rows = 5 if wrench_buffer else 4
    fig, axes = plt.subplots(n_rows, 1, sharex=True, figsize=(9, 2.75 * n_rows))
    pos_ax, att_ax, force_ax, angle_ax = axes[0], axes[1], axes[2], axes[3]
    moment_ax = axes[4] if wrench_buffer else None

    if pos_buffer:
        t_pos = [s[0] for s in pos_buffer]
        pos_ax.plot(t_pos, [s[1] for s in pos_buffer], color=AXIS1_COLOR, label="x")
        pos_ax.plot(t_pos, [s[2] for s in pos_buffer], color=AXIS2_COLOR, label="y")
        pos_ax.plot(t_pos, [s[3] for s in pos_buffer], color=AXIS3_COLOR, label="z")
    pos_ax.set_title("foldrotor_control SITL hover")
    pos_ax.set_ylabel("position [m]\n(NED)")
    pos_ax.legend(loc="upper right", ncol=3, frameon=False)
    pos_ax.grid(True, linewidth=0.5, color=GRID_COLOR)

    if att_buffer:
        t_att = [s[0] for s in att_buffer]
        att_ax.plot(t_att, [math.degrees(s[1]) for s in att_buffer], color=AXIS1_COLOR, label="roll")
        att_ax.plot(t_att, [math.degrees(s[2]) for s in att_buffer], color=AXIS2_COLOR, label="pitch")
        att_ax.plot(t_att, [math.degrees(s[3]) for s in att_buffer], color=AXIS3_COLOR, label="yaw")
    att_ax.set_ylabel("orientation\n[deg]")
    att_ax.legend(loc="upper right", ncol=3, frameon=False)
    att_ax.grid(True, linewidth=0.5, color=GRID_COLOR)

    if cmd_buffer:
        t_cmd = [s[0] for s in cmd_buffer]
        force_ax.plot(t_cmd, [s[1] for s in cmd_buffer], color=ROTOR1_COLOR, label="F1")
        force_ax.plot(t_cmd, [s[2] for s in cmd_buffer], color=ROTOR2_COLOR, label="F2")
        angle_ax.plot(t_cmd, [math.degrees(s[3]) for s in cmd_buffer],
                      color=ROTOR1_COLOR, linestyle="-", label="alpha1 (fold)")
        angle_ax.plot(t_cmd, [math.degrees(s[5]) for s in cmd_buffer],
                      color=ROTOR1_COLOR, linestyle="--", label="beta1 (tilt)")
        angle_ax.plot(t_cmd, [math.degrees(s[4]) for s in cmd_buffer],
                      color=ROTOR2_COLOR, linestyle="-", label="alpha2 (fold)")
        angle_ax.plot(t_cmd, [math.degrees(s[6]) for s in cmd_buffer],
                      color=ROTOR2_COLOR, linestyle="--", label="beta2 (tilt)")

        for span_start, span_end in _saturated_spans(cmd_buffer):
            force_ax.axvspan(span_start, span_end, color=SATURATED_COLOR, alpha=0.15, linewidth=0)
            angle_ax.axvspan(span_start, span_end, color=SATURATED_COLOR, alpha=0.15, linewidth=0)

    if act_buffer:
        t_act = [s[0] for s in act_buffer]
        # Same rotor color and fold(-)/tilt(--) linestyle as the commanded
        # traces above, but faint -- so "actual" reads as an overlay on
        # "commanded", not a second independent signal to parse.
        angle_ax.plot(t_act, [math.degrees(s[1]) for s in act_buffer],
                      color=ROTOR1_COLOR, linestyle="-", alpha=0.4, linewidth=1.5,
                      label="alpha1 (actual)")
        angle_ax.plot(t_act, [math.degrees(s[2]) for s in act_buffer],
                      color=ROTOR1_COLOR, linestyle="--", alpha=0.4, linewidth=1.5,
                      label="beta1 (actual)")
        angle_ax.plot(t_act, [math.degrees(s[3]) for s in act_buffer],
                      color=ROTOR2_COLOR, linestyle="-", alpha=0.4, linewidth=1.5,
                      label="alpha2 (actual)")
        angle_ax.plot(t_act, [math.degrees(s[4]) for s in act_buffer],
                      color=ROTOR2_COLOR, linestyle="--", alpha=0.4, linewidth=1.5,
                      label="beta2 (actual)")

    force_ax.axhline(MAX_THRUST_N, color=LIMIT_COLOR, linestyle=":", linewidth=1, label="limit")
    force_ax.axhline(0.0, color=LIMIT_COLOR, linestyle=":", linewidth=1)
    force_ax.set_ylabel("commanded\nforce [N]")
    force_ax.legend(loc="upper right", ncol=3, frameon=False)
    force_ax.grid(True, linewidth=0.5, color=GRID_COLOR)

    angle_ax.axhline(MAX_TILT_DEG, color=LIMIT_COLOR, linestyle=":", linewidth=1, label="limit")
    angle_ax.axhline(-MAX_TILT_DEG, color=LIMIT_COLOR, linestyle=":", linewidth=1)
    angle_ax.set_ylabel("servo angle [deg]\n(commanded, +actual if available)")
    angle_ax.legend(loc="upper right", ncol=4, frameon=False, fontsize="x-small")
    angle_ax.grid(True, linewidth=0.5, color=GRID_COLOR)

    if moment_ax is not None:
        t_w = [s[0] for s in wrench_buffer]
        moment_ax.plot(t_w, [s[1] for s in wrench_buffer], color=AXIS1_COLOR, label="Mx (roll)")
        moment_ax.plot(t_w, [s[2] for s in wrench_buffer], color=AXIS2_COLOR, label="My (pitch)")
        moment_ax.plot(t_w, [s[3] for s in wrench_buffer], color=AXIS3_COLOR, label="Mz (yaw)")
        moment_ax.axhline(0.0, color=LIMIT_COLOR, linestyle=":", linewidth=1)
        moment_ax.set_ylabel("commanded\nmoment [N*m]")
        moment_ax.legend(loc="upper right", ncol=3, frameon=False)
        moment_ax.grid(True, linewidth=0.5, color=GRID_COLOR)

    axes[-1].set_xlabel("time [s]")

    fig.tight_layout()
    return fig


def _decode_array_name(raw_name):
    if isinstance(raw_name, bytes):
        raw_name = raw_name.decode("utf-8", errors="ignore")
    return raw_name.split("\x00", 1)[0]


def _make_joint_state_callback(act_buffer, t0):
    """Build the gz-transport callback for the "joint_state" topic.

    Closes over t0 (set once, before the subscriber is created, from the
    same wall clock the MAVLink loop below uses) so commanded and actual
    traces share one timeline -- required for the two streams to line up
    at the ~1-2 Hz oscillation timescale this is meant to diagnose.
    Buffers one (t, alpha1, beta1, alpha2, beta2) tuple per message, only
    once all four joints named in JOINT_TO_LABEL have been seen in it.
    """
    def callback(msg):
        t = time.time() - t0
        angles = {}
        for joint in msg.joint:
            label = JOINT_TO_LABEL.get(joint.name)
            if label:
                angles[label] = joint.axis1.position
        if len(angles) == len(JOINT_TO_LABEL):
            act_buffer.append((t, angles["alpha1"], angles["beta1"],
                               angles["alpha2"], angles["beta2"]))
    return callback


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--connect", default="udpin:0.0.0.0:14550")
    parser.add_argument("--rate-hz", type=float, default=30.0)
    parser.add_argument(
        "--gz-topic", default="/world/default/model/foldrotor3_0/joint_state",
        help="gz-transport topic publishing actual joint angle/velocity "
             "(gz-sim-joint-state-publisher-system, added to "
             "Tools/simulation/gz/models/foldrotor3/model.sdf). Confirm the "
             "live model instance name with 'gz topic -l | grep joint_state' "
             "-- it need not match this default. Pass '' to skip actual-angle "
             "capture and plot commanded angles only.")
    args = parser.parse_args()

    master = mavutil.mavlink_connection(args.connect)
    print("waiting for heartbeat...")
    master.wait_heartbeat(timeout=10)
    print(f"heartbeat from sys={master.target_system} comp={master.target_component}")

    interval_us = 1e6 / args.rate_hz
    for msg_id in (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
                   mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
                   mavutil.mavlink.MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY):
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            msg_id, interval_us, 0, 0, 0, 0, 0)

    pos_buffer = []
    att_buffer = []
    cmd_buffer = []
    wrench_buffer = []
    act_buffer = []
    # Set once, eagerly, rather than on first MAVLink message: act_buffer's
    # gz-transport callback (fired from gz-transport's own thread) needs the
    # same origin the MAVLink loop below uses, and may start receiving before
    # the first MAVLink message arrives.
    t0 = time.time()
    last_msg_time = time.time()

    gz_node = None
    if args.gz_topic:
        gz_node = gz.transport13.Node()
        if not gz_node.subscribe(Model, args.gz_topic, _make_joint_state_callback(act_buffer, t0)):
            print(f"warning: failed to subscribe to {args.gz_topic!r} -- "
                  "actual servo angle will not be plotted. Check the topic "
                  "name with 'gz topic -l | grep joint_state'.")

    print("buffering LOCAL_POSITION_NED / ATTITUDE / DEBUG_FLOAT_ARRAY / "
          "gz joint_state -- Ctrl+C here, or Ctrl+C the px4 process, to plot")
    try:
        while True:
            msg = master.recv_match(
                type=["LOCAL_POSITION_NED", "ATTITUDE", "DEBUG_FLOAT_ARRAY"],
                blocking=True, timeout=1.0)
            now = time.time()
            if msg is None:
                if now - last_msg_time > HEARTBEAT_LOSS_TIMEOUT_S:
                    print("no data received -- SITL appears to have stopped")
                    break
                continue

            last_msg_time = now
            t = now - t0

            msg_type = msg.get_type()
            if msg_type == "LOCAL_POSITION_NED":
                pos_buffer.append((t, msg.x, msg.y, msg.z))
            elif msg_type == "ATTITUDE":
                att_buffer.append((t, msg.roll, msg.pitch, msg.yaw))
            elif msg_type == "DEBUG_FLOAT_ARRAY" and _decode_array_name(msg.name) == ALLOC_ARRAY_NAME:
                d = msg.data
                cmd_buffer.append((t, d[0], d[1], d[2], d[3], d[4], d[5], bool(d[6])))
                wrench_buffer.append((t, d[10], d[11], d[12]))
    except KeyboardInterrupt:
        print("\ninterrupted -- plotting buffered data")

    if not pos_buffer and not att_buffer and not cmd_buffer and not act_buffer:
        print("no data buffered, nothing to plot")
        return

    if cmd_buffer and not act_buffer:
        print("note: commanded angles plotted, but NO actual joint angle was "
              f"received on {args.gz_topic!r}. gz-transport subscribe() "
              "succeeds on a topic nobody publishes, so this is silent -- "
              "check 'gz topic -l | grep joint_state' and that model.sdf "
              "carries gz-sim-joint-state-publisher-system.")

    build_figure(pos_buffer, att_buffer, cmd_buffer, act_buffer, wrench_buffer)
    plt.show()


if __name__ == "__main__":
    main()
