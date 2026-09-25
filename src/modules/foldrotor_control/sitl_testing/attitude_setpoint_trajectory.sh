#!/usr/bin/env bash
#
# attitude_setpoint_trajectory.sh -- stream offboard POSITION setpoints and
# interleave one ATTITUDE setpoint per motion leg, alternating pitch and
# roll. The companion to setpoint_trajectory.sh, which commands position
# only.
#
# Only possible since the 2026-09-23 unpin (findings.md (17)). Before it,
# FoldrotorControl pinned phi_sp/theta_sp to zero and this script would
# have been a no-op. It now reads vehicle_attitude_setpoint, which
# mavlink_receiver publishes from SET_ATTITUDE_TARGET while in OFFBOARD.
#
# Legs alternate, one attitude command each:
#   leg 1: roll  0, pitch P    leg 2: roll R, pitch 0    leg 3: ...
#
# THREE THINGS THAT WILL SURPRISE YOU. Read these before reading a result.
#
# 1. YAW 0 IS NOT "LEAVE YAW ALONE". A live attitude setpoint is
#    authoritative for all three angles (controller.md, step 4e item 3),
#    so the yaw in it overrides trajectory_setpoint.yaw. This model spawns
#    at identity in the gz world and GZBridge composes a fixed +90 deg
#    yaw, so PX4 reads ~+90 deg at arm -- commanding yaw 0 commands a
#    ~90 deg TURN. Default here is therefore --yaw 90 (hold the spawn
#    heading) so the tilt test is not also a yaw test. Pass --yaw 0 for
#    a literal zero.
#
# 2. ONE-SHOT MEANS A 500 ms PULSE, NOT A HELD TILT.
#    FoldrotorControl::kAttitudeSetpointTimeout is 500 ms: a setpoint
#    older than that is discarded and the module falls back to level. So
#    one message per leg produces a half-second attitude pulse, then a
#    return to level for the rest of the leg. That is a perfectly good
#    step-response test and it is what --att oneshot (the default) does.
#    Use --att stream to hold the tilt for the whole leg instead.
#
# 3. THE TWO MESSAGES FIGHT OVER THE MODE. Each handler publishes a fresh
#    offboard_control_mode with only its own field set
#    (mavlink_receiver.cpp:1818 and :1274), and commander's offboard
#    branch is a strict if/else on the latest one
#    (control_mode.cpp:114-131). While attitude=true is newest, the
#    position flags drop and mavlink_receiver stops forwarding
#    trajectory_setpoint. At 20 Hz the window is ~50 ms per attitude
#    message, which is why one-shot is the default -- --att stream widens
#    it to the whole leg and the position setpoint stops arriving.
#
# Shapes (the position half; legs are what carry the attitude commands):
#   hold      stationary, attitude legs only            (DEFAULT)
#   step      two-point step response in north
#   box       square circuit in XY
#
# hold is the default deliberately: it exercises attitude with no
# translation to confound it, which is the next rung on the verification
# ladder in .claude/CLAUDE.md. Move to step/box once hold is clean.
#
# Usage:
#   ./attitude_setpoint_trajectory.sh                       # hold, 10 deg legs
#   ./attitude_setpoint_trajectory.sh -p 5 -r 5             # gentler
#   ./attitude_setpoint_trajectory.sh --att stream -w 8
#   ./attitude_setpoint_trajectory.sh -s box -d 1.0 --csv /tmp/att.csv
#   ./attitude_setpoint_trajectory.sh --att none            # position only
#
set -euo pipefail

SHAPE=hold
DIST=1.0
ALT=1.5
PITCH_DEG=10
ROLL_DEG=10
YAW_DEG=90        # spawn heading; see note 1
DWELL=10          # seconds per leg
SETTLE=8          # stationary hover before the first attitude command
DURATION=60
RATE=20
CONN="udpin:0.0.0.0:14540"
DO_ARM=1
ATT_MODE=oneshot  # oneshot | stream | none
CSV=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        -s|--shape)    SHAPE="$2"; shift 2 ;;
        -d|--dist)     DIST="$2"; shift 2 ;;
        -z|--alt)      ALT="$2"; shift 2 ;;
        -p|--pitch)    PITCH_DEG="$2"; shift 2 ;;
        -r|--roll)     ROLL_DEG="$2"; shift 2 ;;
        -y|--yaw)      YAW_DEG="$2"; shift 2 ;;
        -w|--dwell)    DWELL="$2"; shift 2 ;;
        --settle)      SETTLE="$2"; shift 2 ;;
        -t|--time)     DURATION="$2"; shift 2 ;;
        --rate)        RATE="$2"; shift 2 ;;
        -c|--conn)     CONN="$2"; shift 2 ;;
        --att)         ATT_MODE="$2"; shift 2 ;;
        --csv)         CSV="$2"; shift 2 ;;
        --no-arm)      DO_ARM=0; shift ;;
        -h|--help)     sed -n '3,60p' "$0"; exit 0 ;;
        *) echo "unknown arg: $1" >&2; exit 1 ;;
    esac
done

case "$SHAPE" in hold|step|box) ;; *) echo "unknown shape: $SHAPE" >&2; exit 1 ;; esac
case "$ATT_MODE" in oneshot|stream|none) ;; *) echo "unknown --att: $ATT_MODE" >&2; exit 1 ;; esac

python3 -c 'import pymavlink' 2>/dev/null || {
    echo ">> installing pymavlink"
    python3 -m pip install --user --quiet pymavlink
}

echo ">> shape=${SHAPE} alt=${ALT} m | legs: pitch ${PITCH_DEG} deg / roll ${ROLL_DEG} deg, yaw ${YAW_DEG} deg"
echo ">> att=${ATT_MODE} dwell=${DWELL} s | ${DURATION} s | ${RATE} Hz | ${CONN} | arm=${DO_ARM}"

SHAPE="$SHAPE" DIST="$DIST" ALT="$ALT" PITCH_DEG="$PITCH_DEG" ROLL_DEG="$ROLL_DEG" \
YAW_DEG="$YAW_DEG" DWELL="$DWELL" SETTLE="$SETTLE" DURATION="$DURATION" RATE="$RATE" \
CONN="$CONN" DO_ARM="$DO_ARM" ATT_MODE="$ATT_MODE" CSV="$CSV" \
python3 - <<'PYEOF'
import math, os, time
from pymavlink import mavutil

SHAPE    = os.environ["SHAPE"]
DIST     = float(os.environ["DIST"])
ALT      = float(os.environ["ALT"])
PITCH    = math.radians(float(os.environ["PITCH_DEG"]))
ROLL     = math.radians(float(os.environ["ROLL_DEG"]))
YAW      = math.radians(float(os.environ["YAW_DEG"]))
DWELL    = float(os.environ["DWELL"])
SETTLE   = float(os.environ["SETTLE"])
DURATION = float(os.environ["DURATION"])
RATE     = float(os.environ["RATE"])
CONN     = os.environ["CONN"]
DO_ARM   = os.environ["DO_ARM"] == "1"
ATT_MODE = os.environ["ATT_MODE"]
CSV      = os.environ["CSV"]

Z = -ALT
DT = 1.0 / RATE

# Module-side staleness guard (FoldrotorControl::kAttitudeSetpointTimeout).
# Mirrored here only so the report can say whether a pulse was expected to
# decay -- the module enforces it regardless of what this script believes.
ATT_TIMEOUT = 0.5

# SET_POSITION_TARGET_LOCAL_NED: position + yaw, everything else ignored.
# Velocity is masked off deliberately -- foldrotor_control reads only
# trajectory_setpoint.position[0..2] and .yaw, so a velocity field would be
# ignored by the module even if flight_mode_manager forwarded it.
POS_TYPE_MASK = 0b0000101111111000

# SET_ATTITUDE_TARGET: ignore the three body rates (bits 0,1,2), send
# attitude + throttle. THROTTLE_IGNORE must stay CLEAR: mavlink_receiver
# requires has_thrust before it will publish anything
# (mavlink_receiver.cpp:1816). The thrust value itself is discarded by
# foldrotor_control, which reads only q_d -- it is here to satisfy that
# check, not to command anything.
ATT_TYPE_MASK = 0b00000111
ATT_THRUST = 0.5


def euler_to_quat(roll, pitch, yaw):
    """ZYX (3-2-1) Hamilton quaternion, w-first -- the convention
    matrix::Quatf(matrix::Eulerf(...)) uses, which is what the module
    inverts on the far side."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return [cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy]


def leg_waypoints():
    if SHAPE == "step":
        return [(0.0, 0.0, Z), (DIST, 0.0, Z)]
    if SHAPE == "box":
        return [(0.0, 0.0, Z), (DIST, 0.0, Z), (DIST, DIST, Z), (0.0, DIST, Z)]
    return [(0.0, 0.0, Z)]


WAYPOINTS = leg_waypoints()


def leg_index(t):
    return int(t / DWELL) if t >= 0.0 else -1


def position_target(t):
    if t < 0.0 or SHAPE == "hold":
        return (0.0, 0.0, Z)
    return WAYPOINTS[leg_index(t) % len(WAYPOINTS)]


def attitude_for_leg(i):
    """Alternating: even legs pitch, odd legs roll. Yaw is the same on
    every leg -- only the tilt axis alternates."""
    if i % 2 == 0:
        return (0.0, PITCH, YAW)
    return (ROLL, 0.0, YAW)


m = mavutil.mavlink_connection(CONN)
print(">> waiting for heartbeat...")
m.wait_heartbeat()
print(f">> connected: sys {m.target_system} comp {m.target_component}")


def send_gcs_heartbeat():
    # commander refuses to arm with "No connection to the GCS" unless
    # something identifying itself as one sends heartbeats.
    m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                         mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)


send_gcs_heartbeat()

for msg_id in (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
               mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE):
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        msg_id, int(1e6 / RATE), 0, 0, 0, 0, 0)

t0 = time.time()
sp = [0.0, 0.0, Z]


def send_position():
    m.mav.set_position_target_local_ned_send(
        int((time.time() - t0) * 1000),
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        POS_TYPE_MASK,
        sp[0], sp[1], sp[2],
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        YAW, 0.0)


def send_attitude(roll, pitch, yaw):
    m.mav.set_attitude_target_send(
        int((time.time() - t0) * 1000),
        m.target_system, m.target_component,
        ATT_TYPE_MASK,
        euler_to_quat(roll, pitch, yaw),
        0.0, 0.0, 0.0,
        ATT_THRUST)


print(">> priming setpoint stream (1 s)")
last_hb = time.time()
for _ in range(int(RATE)):
    send_position()
    if time.time() - last_hb > 1.0:
        last_hb = time.time()
        send_gcs_heartbeat()
    time.sleep(DT)

m.mav.command_long_send(
    m.target_system, m.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6, 0, 0, 0, 0, 0)
print(">> OFFBOARD requested")

if DO_ARM:
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0)
    print(">> ARM requested")

    ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
    if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
        if ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print(f"!! ARM REJECTED, result={ack.result} -- reason is in the pxh shell")

print(f">> settling at origin, z={Z:+.2f} m NED, for {SETTLE:.0f} s")
if ATT_MODE == "oneshot":
    print(f">> then one attitude command per {DWELL:g} s leg; each decays to level "
          f"after {ATT_TIMEOUT:.1f} s (module timeout)")
elif ATT_MODE == "stream":
    print(f">> then attitude held for the whole {DWELL:g} s leg "
          "-- position setpoints stop arriving while it streams")
else:
    print(">> attitude disabled (--att none): position only")

samples = []       # (t, leg, roll_cmd, pitch_cmd, roll, pitch, yaw, n, e, d)
csv_fh = open(CSV, "w") if CSV else None
if csv_fh:
    csv_fh.write("t,leg,roll_cmd,pitch_cmd,roll,pitch,yaw,n,e,d\n")

start = time.time()
end = start + SETTLE + DURATION
last_print = 0.0
pos = None
att = None
cur_leg = -1
att_cmd = (0.0, 0.0, YAW)
att_sent_at = -1e9

try:
    while time.time() < end:
        now = time.time()
        elapsed = now - start
        t_traj = elapsed - SETTLE

        sp[0], sp[1], sp[2] = position_target(t_traj)

        leg = leg_index(t_traj)

        if leg != cur_leg and leg >= 0:
            cur_leg = leg
            att_cmd = attitude_for_leg(leg)

            if ATT_MODE == "oneshot":
                send_attitude(*att_cmd)
                att_sent_at = now
                print(f"   [leg {leg}] ATT once: roll {math.degrees(att_cmd[0]):+5.1f} "
                      f"pitch {math.degrees(att_cmd[1]):+5.1f} yaw {math.degrees(att_cmd[2]):+5.1f} deg")
            elif ATT_MODE == "stream":
                print(f"   [leg {leg}] ATT hold: roll {math.degrees(att_cmd[0]):+5.1f} "
                      f"pitch {math.degrees(att_cmd[1]):+5.1f} yaw {math.degrees(att_cmd[2]):+5.1f} deg")

        if ATT_MODE == "stream" and leg >= 0:
            send_attitude(*att_cmd)
        else:
            send_position()

        while True:
            msg = m.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE"], blocking=False)
            if msg is None:
                break
            if msg.get_type() == "LOCAL_POSITION_NED":
                pos = msg
            else:
                att = msg

        if pos is not None and att is not None and t_traj >= 0.0:
            # For one-shot, the command is only in force for the timeout
            # window; outside it the module is commanding level and a
            # residual tilt is the vehicle still returning, not tracking.
            live = (ATT_MODE == "stream") or (now - att_sent_at < ATT_TIMEOUT)
            row = (t_traj, leg,
                   math.degrees(att_cmd[0]) if live else 0.0,
                   math.degrees(att_cmd[1]) if live else 0.0,
                   math.degrees(att.roll), math.degrees(att.pitch), math.degrees(att.yaw),
                   pos.x, pos.y, pos.z)
            samples.append(row)
            if csv_fh:
                csv_fh.write(",".join(f"{v:.4f}" for v in row) + "\n")

        if now - last_hb > 1.0:
            last_hb = now
            send_gcs_heartbeat()

        if now - last_print > 2.0 and att is not None and pos is not None:
            last_print = now
            phase = "settle" if t_traj < 0.0 else f"leg {leg}"
            print(f"   [{phase:>7}] att=({math.degrees(att.roll):+6.2f},"
                  f"{math.degrees(att.pitch):+6.2f},{math.degrees(att.yaw):+7.2f}) deg  "
                  f"pos=({pos.x:+5.2f},{pos.y:+5.2f},{pos.z:+5.2f})")

        time.sleep(DT)
except KeyboardInterrupt:
    print("\n>> interrupted")

if csv_fh:
    csv_fh.close()
    print(f">> wrote {len(samples)} samples to {CSV}")

if samples:
    print("")
    print(">> per-leg attitude response (peak |angle| reached while the command was live)")
    print(">>   leg  axis    cmd     peak    at-end   |  alt drift")
    legs = sorted({int(s[1]) for s in samples if s[1] >= 0})

    for lg in legs:
        rows = [s for s in samples if int(s[1]) == lg]
        live_rows = [s for s in rows if abs(s[2]) > 1e-6 or abs(s[3]) > 1e-6]
        if not live_rows:
            continue

        axis = "pitch" if lg % 2 == 0 else "roll"
        idx = 5 if axis == "pitch" else 4
        cmd = live_rows[0][3] if axis == "pitch" else live_rows[0][2]
        peak_row = max(live_rows, key=lambda s: abs(s[idx]))
        peak = peak_row[idx]
        at_end = live_rows[-1][idx]
        dz = rows[-1][9] - rows[0][9]

        # If the largest angle seen is the LAST sample of the window, the
        # response had not turned over yet: the command expired mid-slew
        # and "peak" is just where it got to, not a steady state. With
        # --att oneshot and FR_ATT_P = 2.0 this is the normal case, since
        # the rate setpoint is only FR_ATT_P * error and 500 ms of it is
        # barely the time needed to cover the commanded angle at all.
        rising = (peak_row is live_rows[-1]) and (abs(peak) > 0.1)
        note = "  <- still rising, not settled" if rising else ""
        print(f">>   {lg:>3}  {axis:<6} {cmd:+6.1f}  {peak:+6.1f}  {at_end:+6.1f}   |  {dz:+5.2f} m{note}")

    # Module-side constants, for turning the commanded angle into a share
    # of the authority it actually costs. Measured 2026-09-23 against the
    # real allocator at hover lift; see findings.md (17).
    RESTORING = 0.334        # N*m/rad, body-horizontal force on the lever
    AUTH_PITCH = 0.440       # N*m, hover single-axis
    AUTH_ROLL = 2.82         # N*m
    CAP_TILT = math.degrees(math.asin(4.0 / 19.6014))   # kBodyForceXYLimit

    if ATT_MODE == "oneshot":
        # Deliberately no gain in this message: FR_ATT_P lives on the vehicle
        # and a number hardcoded here goes stale the moment it is retuned
        # (it did, 2026-09-24 -- this printed the old 2.0 on a 4.0 flight).
        print("")
        print(f">> NOTE: a one-shot command is live for only {ATT_TIMEOUT:.1f} s. A leg marked")
        print(">> 'still rising' ran out of window mid-slew and measured a SLEW, not")
        print(">> tracking. Use --att stream to hold the command and see where it")
        print(">> settles -- with -s hold that costs nothing, because the position")
        print(">> setpoint is constant and the module simply keeps the last one.")
        print(">> For damping, look past the window: the CSV is 20 Hz, the console")
        print(">> prints are 2 s apart and WILL alias a ~1 Hz oscillation away.")

    print("")
    print(">> Expect the two axes to differ -- that is the airframe, not a fault.")

    for label, ang, auth in (("pitch", PITCH, AUTH_PITCH), ("roll", ROLL, AUTH_ROLL)):
        cost = RESTORING * ang
        print(f">>   holding {math.degrees(ang):4.1f} deg of {label:<5} costs {cost:5.3f} N*m"
              f" = {100.0 * cost / auth:4.1f}% of its {auth:.2f} N*m hover authority")

    print(">> Pitch also carries a deliberately slower gain (FR_RATE_P_FF 0.11")
    print(">> vs roll's 0.49), so it should be the slower axis as well.")
    print(">>")
    print(f">> Altitude: kBodyForceXYLimit = 4.0 N holds tilt up to {CAP_TILT:.1f} deg")
    print(">> without sinking. Past that the force path clips and the vehicle")
    print(">> keeps the angle while losing height.")

    if max(math.degrees(PITCH), math.degrees(ROLL)) > CAP_TILT:
        print(f"!! COMMANDED TILT EXCEEDS {CAP_TILT:.1f} deg -- altitude loss above is")
        print("!! expected behaviour of the cap, not a tracking failure.")

    print(">>")
    print(">> Cross-check against the module: debug_array 'fr_alloc' slots 13-15")
    print(">> carry the measured allocator residual (asked - delivered, N*m FRD).")
    print(">> Non-zero there during a leg means the allocator could not deliver")
    print(">> the moment the rate loop asked for -- the condition the `saturated`")
    print(">> flag reads 0% through. That is the number to watch.")

print(">> done -- vehicle still armed, disarm in pxh with: commander disarm")
PYEOF
