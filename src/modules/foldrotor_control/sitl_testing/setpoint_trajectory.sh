#!/usr/bin/env bash
#
# setpoint_trajectory.sh -- stream offboard POSITION TRACKING trajectories
# to PX4 SITL, as hover_setpoint.sh does for a stationary hover.
#
# Same transport and the same caveats as hover_setpoint.sh:
#
#   SET_POSITION_TARGET_LOCAL_NED -> commander -> flight_mode_manager
#       -> trajectory_setpoint -> _trajectory_setpoint_sub
#
# flight_mode_manager MUST be running. The foldrotor3 airframe no longer
# sets VEHICLE_TYPE mc, so rc.mc_apps does not start it -- run
# `flight_mode_manager start` in the pxh shell first, or nothing will
# publish trajectory_setpoint and this script will stream into a void.
#
# Shapes:
#   step      two-point step response, dwelling at each end   (DEFAULT)
#   box       square circuit in XY, dwelling at each corner
#   ramp      constant-velocity straight line out and back
#   circle    constant-speed circle, entered from the origin, so it is
#             centred at (0, DIST) east and reaches 2*DIST east
#   figure8   lemniscate; peak speed is sqrt(2)*SPEED at the crossing
#   climb     vertical step between ALT and ALT+DZ
#   hold      stationary, i.e. hover_setpoint.sh with error statistics
#
# Usage:
#   ./setpoint_trajectory.sh                          # 1 m step, 1.5 m, 60 s
#   ./setpoint_trajectory.sh -s box -d 1.5 -t 120
#   ./setpoint_trajectory.sh -s circle -d 1.0 -v 0.3
#   ./setpoint_trajectory.sh -s ramp -v 0.4 --csv /tmp/track.csv
#   ./setpoint_trajectory.sh --no-arm                 # stream only
#
set -euo pipefail

SHAPE=step
DIST=1.0          # step size / box side / circle radius / ramp length (m)
ALT=1.5
DZ=0.5            # climb shape: vertical step size (m)
SPEED=0.3         # commanded path speed for the continuous shapes (m/s)
DWELL=10          # seconds held at each waypoint for the discrete shapes
SETTLE=8          # seconds of stationary hover before the trajectory starts
DURATION=60
RATE=20
CONN="udpin:0.0.0.0:14540"
DO_ARM=1
YAW_DEG=90        # see the YAW note in the python block
POS_P=0.4         # FR_POS_P, used ONLY to predict the lag printed below
CSV=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        -s|--shape)    SHAPE="$2"; shift 2 ;;
        -d|--dist)     DIST="$2"; shift 2 ;;
        -z|--alt)      ALT="$2"; shift 2 ;;
        --dz)          DZ="$2"; shift 2 ;;
        -v|--speed)    SPEED="$2"; shift 2 ;;
        -w|--dwell)    DWELL="$2"; shift 2 ;;
        --settle)      SETTLE="$2"; shift 2 ;;
        -t|--time)     DURATION="$2"; shift 2 ;;
        -r|--rate)     RATE="$2"; shift 2 ;;
        -c|--conn)     CONN="$2"; shift 2 ;;
        -y|--yaw)      YAW_DEG="$2"; shift 2 ;;
        --pos-p)       POS_P="$2"; shift 2 ;;
        --csv)         CSV="$2"; shift 2 ;;
        --no-arm)      DO_ARM=0; shift ;;
        -h|--help)     sed -n '3,31p' "$0"; exit 0 ;;
        *) echo "unknown arg: $1" >&2; exit 1 ;;
    esac
done

case "$SHAPE" in
    step|box|ramp|circle|figure8|climb|hold) ;;
    *) echo "unknown shape: $SHAPE (see --help)" >&2; exit 1 ;;
esac

python3 -c 'import pymavlink' 2>/dev/null || {
    echo ">> installing pymavlink"
    python3 -m pip install --user --quiet pymavlink
}

echo ">> shape=${SHAPE} dist=${DIST} m alt=${ALT} m speed=${SPEED} m/s"
echo ">> ${DURATION} s | ${RATE} Hz | ${CONN} | arm=${DO_ARM}"

SHAPE="$SHAPE" DIST="$DIST" ALT="$ALT" DZ="$DZ" SPEED="$SPEED" DWELL="$DWELL" \
SETTLE="$SETTLE" DURATION="$DURATION" RATE="$RATE" CONN="$CONN" DO_ARM="$DO_ARM" \
YAW_DEG="$YAW_DEG" POS_P="$POS_P" CSV="$CSV" \
python3 - <<'PYEOF'
import math, os, time
from pymavlink import mavutil

SHAPE    = os.environ["SHAPE"]
DIST     = float(os.environ["DIST"])
ALT      = float(os.environ["ALT"])
DZ       = float(os.environ["DZ"])
SPEED    = float(os.environ["SPEED"])
DWELL    = float(os.environ["DWELL"])
SETTLE   = float(os.environ["SETTLE"])
DURATION = float(os.environ["DURATION"])
RATE     = float(os.environ["RATE"])
CONN     = os.environ["CONN"]
DO_ARM   = os.environ["DO_ARM"] == "1"
POS_P    = float(os.environ["POS_P"])
CSV      = os.environ["CSV"]

# NED: down is positive, so a hover ALT metres up is z = -ALT.
Z = -ALT
DT = 1.0 / RATE

# gz world is ENU, PX4 is NED; GZBridge::rotateQuaternion composes a fixed
# +90 deg yaw between them (GZBridge.cpp:886-901). This model spawns at
# identity orientation in the gz world, so PX4's own yaw estimate reads
# ~+90 deg at arm, not 0. Command that same heading here so a tracking
# test doesn't also have to fight a 90 deg yaw correction.
YAW = math.radians(float(os.environ["YAW_DEG"]))

# type_mask: ignore vel/accel/yaw_rate, use position + yaw.
#   bits 0-2 pos, 3-5 vel, 6-8 accel, 10 yaw, 11 yaw_rate
#
# Velocity is masked off DELIBERATELY, not as a simplification.
# foldrotor_control reads only trajectory_setpoint.position[0..2] and .yaw
# (FoldrotorControl.cpp:665, :753) -- it has no velocity-feedforward input
# at all, so anything sent in the velocity fields would be ignored by the
# module even if flight_mode_manager forwarded it.
TYPE_MASK = 0b0000101111111000

# ---------------------------------------------------------------------
# WHAT THIS CAN AND CANNOT MEASURE
#
# The position loop is a pure proportional law with no feedforward:
#
#     vel_sp = (pos_sp - pos) * FR_POS_P     (PositionVelocityControl.hpp:293)
#
# so holding a constant path speed v REQUIRES a standing position error of
# v / FR_POS_P metres. That lag is structural, not a tracking failure --
# at FR_POS_P = 0.4 and v = 0.3 m/s it is 0.75 m, which looks alarming on
# a plot and means nothing is wrong. It is printed below so it can be
# subtracted, and the summary reports both raw and lag-compensated error.
#
# Consequences for choosing a shape:
#   - step / box / climb dwell long enough to reach steady state, where
#     the lag is zero by construction. Use these to measure the loop.
#   - ramp / circle / figure8 never reach steady state. Use these to ask
#     whether the vehicle stays STABLE while translating, not how small
#     the error is.
#
# vel_sp is then clamped to FR_VEL_XY_MAX (1.0 m/s) and
# FR_VEL_Z_MAX_UP/DN, so the loop is only linear while
# |pos_sp - pos| < FR_VEL_XY_MAX / FR_POS_P = 2.5 m. A step larger than
# that spends its first seconds on the velocity rail.
# ---------------------------------------------------------------------
lag = SPEED / POS_P if POS_P > 0 else float("inf")
continuous = SHAPE in ("ramp", "circle", "figure8")
print(f">> position loop is pure P (FR_POS_P={POS_P}), no feedforward")
if continuous:
    print(f">> EXPECT {lag:.2f} m of steady-state lag at {SPEED} m/s -- structural, not a fault")
else:
    print(f">> discrete shape: dwell {DWELL:.0f} s reaches steady state, lag -> 0")
if DIST > 2.5:
    print(f"!! dist {DIST} m > 2.5 m: vel_sp will hit the FR_VEL_XY_MAX rail on each leg")


def leg_waypoints():
    """Corner list for the dwell-based shapes, in NED metres."""
    if SHAPE == "step":
        return [(0.0, 0.0, Z), (DIST, 0.0, Z)]
    if SHAPE == "box":
        return [(0.0, 0.0, Z), (DIST, 0.0, Z), (DIST, DIST, Z), (0.0, DIST, Z)]
    if SHAPE == "climb":
        return [(0.0, 0.0, Z), (0.0, 0.0, Z - DZ)]
    return [(0.0, 0.0, Z)]


WAYPOINTS = leg_waypoints()


def target(t):
    """Position setpoint (north, east, down) at trajectory time t seconds."""
    if t < 0.0:
        return (0.0, 0.0, Z)          # settle phase: hold the origin

    if SHAPE == "hold":
        return (0.0, 0.0, Z)

    if SHAPE in ("step", "box", "climb"):
        return WAYPOINTS[int(t / DWELL) % len(WAYPOINTS)]

    if SHAPE == "ramp":
        # Out and back along +north at constant speed.
        leg = DIST / SPEED if SPEED > 0 else 1.0
        phase = (t / leg) % 2.0
        x = DIST * phase if phase < 1.0 else DIST * (2.0 - phase)
        return (x, 0.0, Z)

    if SHAPE == "circle":
        # Starts at the origin moving +north, so there is no jump at t=0.
        # That puts the CENTRE at (0, DIST): the path reaches 2*DIST east,
        # which is what counts against the 2.5 m linear-range note above.
        w = SPEED / DIST if DIST > 0 else 0.0
        return (DIST * math.sin(w * t), DIST * (1.0 - math.cos(w * t)), Z)

    if SHAPE == "figure8":
        # Lemniscate through the origin. Path speed varies along it:
        # SPEED sets the nominal DIST*w, and the peak is sqrt(2)*SPEED at
        # the crossing, where the north and east rates add.
        w = SPEED / DIST if DIST > 0 else 0.0
        return (DIST * math.sin(w * t), 0.5 * DIST * math.sin(2.0 * w * t), Z)

    return (0.0, 0.0, Z)


m = mavutil.mavlink_connection(CONN)
print(">> waiting for heartbeat...")
m.wait_heartbeat()
print(f">> connected: sys {m.target_system} comp {m.target_component}")

# PX4's commander refuses to arm with "No connection to the GCS" unless
# something identifying itself as a ground control station sends
# heartbeats -- this script only *received* PX4's heartbeat above, it
# never sent one of its own. Repeat at ~1 Hz for as long as it runs.
def send_gcs_heartbeat():
    m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                         mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)


send_gcs_heartbeat()

# Ask for LOCAL_POSITION_NED faster than the template's opportunistic
# 2 s poll: the error summary at the end needs a real sample rate, not
# whatever happened to be in the buffer.
m.mav.command_long_send(
    m.target_system, m.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
    mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
    int(1e6 / RATE), 0, 0, 0, 0, 0)

t0 = time.time()
sp = [0.0, 0.0, Z]


def setpoint():
    m.mav.set_position_target_local_ned_send(
        int((time.time() - t0) * 1000),   # time_boot_ms
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        TYPE_MASK,
        sp[0], sp[1], sp[2],   # x, y, z
        0.0, 0.0, 0.0,         # vx, vy, vz
        0.0, 0.0, 0.0,         # afx, afy, afz
        YAW, 0.0)              # yaw, yaw_rate


# PX4 rejects OFFBOARD unless setpoints are already streaming. Prime it.
print(">> priming setpoint stream (1 s)")
last_hb = time.time()
for _ in range(int(RATE)):
    setpoint()
    if time.time() - last_hb > 1.0:
        last_hb = time.time()
        send_gcs_heartbeat()
    time.sleep(DT)

# custom_mode 6 = PX4_CUSTOM_MAIN_MODE_OFFBOARD
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
            print(f"!! ARM REJECTED, result={ack.result}")
            print("!! read the reason in the pxh shell -- with VEHICLE_TYPE mc")
            print("!! removed, a commander preflight check is the usual cause.")

print(f">> settling at origin, z={Z:+.2f} m NED, for {SETTLE:.0f} s")
print(f">> then {SHAPE} for {DURATION:.0f} s (ctrl-c to stop)")

samples = []          # (t, sp_n, sp_e, sp_d, n, e, d)
csv_fh = open(CSV, "w") if CSV else None
if csv_fh:
    csv_fh.write("t,sp_n,sp_e,sp_d,n,e,d\n")

start = time.time()
end = start + SETTLE + DURATION
last_print = 0.0
pos = None
try:
    while time.time() < end:
        now = time.time()
        elapsed = now - start
        sp[0], sp[1], sp[2] = target(elapsed - SETTLE)
        setpoint()

        # Drain every pending message, or the socket buffer backs up and
        # the position samples fall behind the setpoints they pair with.
        while True:
            msg = m.recv_match(type="LOCAL_POSITION_NED", blocking=False)
            if msg is None:
                break
            pos = msg

        if pos is not None and elapsed > SETTLE:
            row = (elapsed - SETTLE, sp[0], sp[1], sp[2], pos.x, pos.y, pos.z)
            samples.append(row)
            if csv_fh:
                csv_fh.write(",".join(f"{v:.4f}" for v in row) + "\n")

        if now - last_hb > 1.0:
            last_hb = now
            send_gcs_heartbeat()

        if now - last_print > 2.0 and pos is not None:
            last_print = now
            en = math.hypot(sp[0] - pos.x, sp[1] - pos.y)
            ez = sp[2] - pos.z
            phase = "settle" if elapsed < SETTLE else SHAPE
            print(f"   [{phase:>7}] sp=({sp[0]:+5.2f},{sp[1]:+5.2f},{sp[2]:+5.2f}) "
                  f"pos=({pos.x:+5.2f},{pos.y:+5.2f},{pos.z:+5.2f}) "
                  f"|e_xy|={en:5.2f}  e_z={ez:+5.2f}")

        time.sleep(DT)
except KeyboardInterrupt:
    print("\n>> interrupted")

if csv_fh:
    csv_fh.close()
    print(f">> wrote {len(samples)} samples to {CSV}")

if samples:
    exy = [math.hypot(s[1] - s[4], s[2] - s[5]) for s in samples]
    ez = [s[3] - s[6] for s in samples]
    n = len(exy)
    rms_xy = math.sqrt(sum(e * e for e in exy) / n)
    rms_z = math.sqrt(sum(e * e for e in ez) / n)
    print("")
    print(f">> {SHAPE}: {n} samples over {samples[-1][0]:.1f} s")
    print(f">>   horizontal error  rms {rms_xy:5.3f} m   max {max(exy):5.3f} m")
    print(f">>   vertical error    rms {rms_z:5.3f} m   max {max(abs(e) for e in ez):5.3f} m")
    if continuous:
        # The pure-P lag is a pursuit error along the path, so subtracting
        # it in magnitude is only indicative -- the CSV is there for a
        # proper cross-correlation if the number matters.
        print(f">>   predicted structural lag at {SPEED} m/s: {lag:5.3f} m")
        print(f">>   excess over lag: {max(0.0, rms_xy - lag):5.3f} m rms"
              "  (indicative -- see --csv for a proper lag-compensated fit)")
    else:
        print(f">>   dwell shape: this IS the tracking error, no lag to subtract")

print(">> done -- vehicle still armed, disarm in pxh with: commander disarm")
PYEOF
