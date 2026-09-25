#!/usr/bin/env python3
"""Post-flight plots for a foldrotor_control SITL run, read from its ULog.

One PNG per log, written next to the .ulg (same stem), with four panels on
a shared time axis:

  1. position     vehicle_local_position x/y/z (NED, m), trajectory_setpoint
                  overlaid dotted when present
  2. orientation  vehicle_attitude roll/pitch/yaw (deg, FRD body w.r.t. NED)
  3. servo angles commanded alpha1/alpha2 (fold) and beta1/beta2 (tilt), deg
  4. thrust       commanded F1/F2, N

Panels 3 and 4 come from the module's "fr_alloc" debug_array -- the
allocator OUTPUT in physical units, before the normalisation into
actuator_servos/actuator_motors. Slot layout mirrors the kDebug* constants
in FoldrotorControl.hpp; there is nothing in the message to catch a
mismatch, so keep the two in step. Windows where the allocator reported a
clamp (kDebugSaturated) are shaded.

debug_array is only logged when SDLOG_PROFILE includes the debug bit (32);
the SITL default already does. With SDLOG_MODE 1 a log is closed on disarm
or PX4 shutdown -- the trajectory scripts leave the vehicle armed, so the
log for a run is complete only once you disarm or stop px4.

Usage:
  ./plot_sitl_log.py                    # newest log under the SITL rootfs
  ./plot_sitl_log.py path/to/run.ulg    # specific log(s)
  ./plot_sitl_log.py --watch            # plot every log as it is closed
  ./plot_sitl_log.py --show             # also open a window
"""
import argparse
import math
import sys
import time
from pathlib import Path

import matplotlib

if "--show" not in sys.argv:
    matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[4]
DEFAULT_LOG_DIR = REPO_ROOT / "build" / "px4_sitl_default" / "rootfs" / "log"

ALLOC_ARRAY_ID = 0  # FoldrotorControl.cpp: debug_array.id = 0, name "fr_alloc"

# FoldrotorControl.hpp kDebug* slots.
SLOT_F1, SLOT_F2 = 0, 1
SLOT_ALPHA1, SLOT_ALPHA2 = 2, 3
SLOT_BETA1, SLOT_BETA2 = 4, 5
SLOT_SATURATED = 6

# FoldrotorAllocation.hpp kMaxThrust / kMaxTilt (alpha is clamped to the
# same bound as beta). Reference lines only.
MAX_THRUST_N = 15.0
MAX_TILT_DEG = math.degrees(0.79)

# Categorical slots 1-3 (validated: CVD adjacent dE 9.2, slot 3 below 3:1
# contrast, so every series is legend-labelled). Rotor 1/2 keep the same
# hue across the servo and thrust panels; fold is solid, tilt dashed.
AXIS1_COLOR = "#2a78d6"
AXIS2_COLOR = "#eb6834"
AXIS3_COLOR = "#1baf7a"
ROTOR1_COLOR = AXIS1_COLOR
ROTOR2_COLOR = AXIS2_COLOR
LIMIT_COLOR = "#898781"
GRID_COLOR = "#e1e0d9"
SATURATED_COLOR = "#ec835a"
ARM_COLOR = "#52514e"
DISARMED_COLOR = "#898781"


def quat_to_euler(q0, q1, q2, q3):
    """Hamilton quaternion (w, x, y, z) -> (roll, pitch, yaw) in radians.

    Same ZYX convention as matrix::Eulerf, so the numbers match what PX4
    itself reports for vehicle_attitude.
    """
    roll = math.atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q0 * q2 - q3 * q1))))
    yaw = math.atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3))
    return roll, pitch, yaw


def _saturated_spans(t, flags):
    spans = []
    start = None
    for ti, sat in zip(t, flags):
        if sat and start is None:
            start = ti
        elif not sat and start is not None:
            spans.append((start, ti))
            start = None
    if start is not None and t:
        spans.append((start, t[-1]))
    return spans


def disarmed_spans(arm_times, t_end):
    """(t_start, t_end) windows where the vehicle was disarmed.

    arm_times is extract()'s list of (t, armed) edges; the log is taken to
    start disarmed at t = 0.
    """
    spans = []
    start = 0.0
    for t, armed in arm_times:
        if armed and start is not None:
            spans.append((start, t))
            start = None
        elif not armed and start is None:
            start = t
    if start is not None and t_end > start:
        spans.append((start, t_end))
    return spans


def extract(ulog):
    """Pull the plotted series out of a pyulog.ULog into plain lists.

    Returns a dict of series keyed by name; every time base is seconds
    since the log's start. Missing topics give empty lists rather than
    raising, so a log from a run that never armed still plots.
    """
    t0 = ulog.start_timestamp

    def topic(name):
        for d in ulog.data_list:
            if d.name == name and d.multi_id == 0:
                return d.data
        return None

    def secs(ts):
        return [(x - t0) * 1e-6 for x in ts]

    out = {"pos": None, "pos_sp": None, "att": None, "alloc": None, "arm_times": []}

    lp = topic("vehicle_local_position")
    if lp is not None:
        out["pos"] = (secs(lp["timestamp"]), list(lp["x"]), list(lp["y"]), list(lp["z"]))

    ts = topic("trajectory_setpoint")
    if ts is not None:
        out["pos_sp"] = (secs(ts["timestamp"]), list(ts["position[0]"]),
                         list(ts["position[1]"]), list(ts["position[2]"]))

    va = topic("vehicle_attitude")
    if va is not None:
        eul = [quat_to_euler(*q) for q in zip(va["q[0]"], va["q[1]"], va["q[2]"], va["q[3]"])]
        out["att"] = (secs(va["timestamp"]), [e[0] for e in eul], [e[1] for e in eul], [e[2] for e in eul])

    da = topic("debug_array")
    if da is not None:
        keep = [i for i, v in enumerate(da["id"]) if v == ALLOC_ARRAY_ID]
        t_all = secs(da["timestamp"])

        def col(slot):
            c = da[f"data[{slot}]"]
            return [float(c[i]) for i in keep]

        out["alloc"] = {
            "t": [t_all[i] for i in keep],
            "F1": col(SLOT_F1), "F2": col(SLOT_F2),
            "alpha1": col(SLOT_ALPHA1), "alpha2": col(SLOT_ALPHA2),
            "beta1": col(SLOT_BETA1), "beta2": col(SLOT_BETA2),
            "saturated": [v > 0.5 for v in col(SLOT_SATURATED)],
        }

    vs = topic("vehicle_status")
    if vs is not None:
        armed_prev = False
        for t, state in zip(secs(vs["timestamp"]), vs["arming_state"]):
            armed = state == 2  # vehicle_status_s::ARMING_STATE_ARMED
            if armed != armed_prev:
                out["arm_times"].append((t, armed))
            armed_prev = armed

    return out


def build_figure(series, title="foldrotor_control SITL"):
    """Four-panel figure from extract()'s dict. Pure; safe to test headlessly."""
    fig, (pos_ax, att_ax, ang_ax, thr_ax) = plt.subplots(4, 1, sharex=True, figsize=(10, 11))

    if series["pos"]:
        t, x, y, z = series["pos"]
        pos_ax.plot(t, x, color=AXIS1_COLOR, linewidth=1.5, label="x (N)")
        pos_ax.plot(t, y, color=AXIS2_COLOR, linewidth=1.5, label="y (E)")
        pos_ax.plot(t, z, color=AXIS3_COLOR, linewidth=1.5, label="z (D)")
    if series["pos_sp"]:
        t, x, y, z = series["pos_sp"]
        for vals, color in ((x, AXIS1_COLOR), (y, AXIS2_COLOR), (z, AXIS3_COLOR)):
            pos_ax.plot(t, vals, color=color, linestyle=":", linewidth=1.2)
        pos_ax.plot([], [], color=LIMIT_COLOR, linestyle=":", label="setpoint")
    pos_ax.set_ylabel("position [m]\n(NED)")
    pos_ax.set_title(title)

    if series["att"]:
        t, r, p, y = series["att"]
        att_ax.plot(t, [math.degrees(v) for v in r], color=AXIS1_COLOR, linewidth=1.5, label="roll")
        att_ax.plot(t, [math.degrees(v) for v in p], color=AXIS2_COLOR, linewidth=1.5, label="pitch")
        att_ax.plot(t, [math.degrees(v) for v in y], color=AXIS3_COLOR, linewidth=1.5, label="yaw")
    att_ax.set_ylabel("orientation [deg]\n(FRD w.r.t. NED)")

    a = series["alloc"]
    if a and a["t"]:
        t = a["t"]
        deg = lambda vals: [math.degrees(v) for v in vals]  # noqa: E731
        ang_ax.plot(t, deg(a["alpha1"]), color=ROTOR1_COLOR, linewidth=1.5, label="alpha1 (fold)")
        ang_ax.plot(t, deg(a["beta1"]), color=ROTOR1_COLOR, linewidth=1.5, linestyle="--", label="beta1 (tilt)")
        ang_ax.plot(t, deg(a["alpha2"]), color=ROTOR2_COLOR, linewidth=1.5, label="alpha2 (fold)")
        ang_ax.plot(t, deg(a["beta2"]), color=ROTOR2_COLOR, linewidth=1.5, linestyle="--", label="beta2 (tilt)")
        thr_ax.plot(t, a["F1"], color=ROTOR1_COLOR, linewidth=1.5, label="F1")
        thr_ax.plot(t, a["F2"], color=ROTOR2_COLOR, linewidth=1.5, label="F2")
        spans = _saturated_spans(t, a["saturated"])
        for s0, s1 in spans:
            ang_ax.axvspan(s0, s1, color=SATURATED_COLOR, alpha=0.15, linewidth=0)
            thr_ax.axvspan(s0, s1, color=SATURATED_COLOR, alpha=0.15, linewidth=0)
        if spans:
            thr_ax.axvspan(0, 0, color=SATURATED_COLOR, alpha=0.15, label="allocator clamped")
    else:
        for ax in (ang_ax, thr_ax):
            ax.text(0.5, 0.5, "no fr_alloc debug_array in log\n(SDLOG_PROFILE debug bit off?)",
                    transform=ax.transAxes, ha="center", va="center", color=LIMIT_COLOR)

    ang_ax.axhline(MAX_TILT_DEG, color=LIMIT_COLOR, linestyle=":", linewidth=1, label="limit")
    ang_ax.axhline(-MAX_TILT_DEG, color=LIMIT_COLOR, linestyle=":", linewidth=1)
    ang_ax.set_ylabel("commanded servo\nangle [deg]")

    thr_ax.axhline(MAX_THRUST_N, color=LIMIT_COLOR, linestyle=":", linewidth=1, label="limit")
    thr_ax.axhline(0.0, color=LIMIT_COLOR, linestyle=":", linewidth=1)
    thr_ax.set_ylabel("commanded\nthrust [N]")
    thr_ax.set_xlabel("time since log start [s]")

    for t, armed in series["arm_times"]:
        for ax in (pos_ax, att_ax, ang_ax, thr_ax):
            ax.axvline(t, color=ARM_COLOR, linewidth=0.8, linestyle="-." if armed else ":")
        pos_ax.annotate("arm" if armed else "disarm", (t, 1.0), xycoords=("data", "axes fraction"),
                        xytext=(3, -10), textcoords="offset points", fontsize="x-small", color=ARM_COLOR)

    # The allocator runs while disarmed, but FoldrotorControl publishes NaN
    # (motors off / servos disarmed) to actuator_motors/servos then -- so the
    # commanded values in those windows never reach the vehicle.
    t_end = max(ax.dataLim.x1 for ax in (pos_ax, att_ax, ang_ax, thr_ax))
    off = disarmed_spans(series["arm_times"], t_end)
    for s0, s1 in off:
        for ax in (ang_ax, thr_ax):
            ax.axvspan(s0, s1, color=DISARMED_COLOR, alpha=0.12, linewidth=0)
    if off:
        thr_ax.axvspan(0, 0, color=DISARMED_COLOR, alpha=0.12, label="disarmed\n(not sent)")

    for ax in (pos_ax, att_ax, ang_ax, thr_ax):
        ax.grid(True, linewidth=0.5, color=GRID_COLOR)
        ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), frameon=False, fontsize="small")

    fig.tight_layout()
    return fig


def plot_log(path, show=False):
    from pyulog import ULog

    topics = ["vehicle_local_position", "trajectory_setpoint", "vehicle_attitude",
              "debug_array", "vehicle_status"]
    series = extract(ULog(str(path), topics))
    fig = build_figure(series, title=f"foldrotor_control SITL -- {path.parent.name}/{path.name}")
    out = path.with_suffix(".png")
    fig.savefig(out, dpi=120)
    print(f">> wrote {out}")
    if show:
        plt.show()
    plt.close(fig)
    return out


def _all_logs(log_dir):
    return sorted(log_dir.glob("*/*.ulg"), key=lambda p: p.stat().st_mtime)


def watch(log_dir, settle_s, poll_s):
    """Plot every log that has stopped growing and has no up-to-date PNG."""
    print(f">> watching {log_dir} (Ctrl+C to stop)")
    sizes = {}
    failed = set()
    while True:
        now = time.time()
        for p in _all_logs(log_dir):
            if p in failed:
                continue
            png = p.with_suffix(".png")
            st = p.stat()
            if png.exists() and png.stat().st_mtime >= st.st_mtime:
                continue
            prev = sizes.get(p)
            sizes[p] = (st.st_size, prev[1] if prev and prev[0] == st.st_size else now)
            if now - sizes[p][1] >= settle_s:
                try:
                    plot_log(p)
                except Exception as e:  # a truncated log must not kill the watcher
                    print(f">> {p}: {e}", file=sys.stderr)
                    failed.add(p)
        time.sleep(poll_s)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="*", type=Path, help="ULog file(s); default: newest in --log-dir")
    ap.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    ap.add_argument("--watch", action="store_true", help="plot each new log once it is closed")
    ap.add_argument("--settle", type=float, default=5.0,
                    help="--watch: seconds a log must stop growing before it is plotted")
    ap.add_argument("--show", action="store_true", help="open an interactive window too")
    args = ap.parse_args()

    if args.watch:
        try:
            watch(args.log_dir, args.settle, poll_s=1.0)
        except KeyboardInterrupt:
            pass
        return

    logs = args.logs
    if not logs:
        found = _all_logs(args.log_dir)
        if not found:
            sys.exit(f"no .ulg under {args.log_dir}")
        logs = [found[-1]]
    for p in logs:
        plot_log(p, show=args.show)


if __name__ == "__main__":
    main()
