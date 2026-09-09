#!/usr/bin/env python3
"""Drive a closed-loop Offboard hover over MAVLink for foldrotor_control SITL testing.

Streams a held SET_POSITION_TARGET_LOCAL_NED setpoint (position-only), switches
to OFFBOARD, arms, then keeps streaming. See ../sitl_testing/README.md for the
full launch recipe this script is meant to run against.
"""
import argparse
import time

from pymavlink import mavutil

TYPE_MASK_POS_ONLY = 0b0000111111111000  # only x,y,z position bits enabled


def send_sp(master, x=0.0, y=0.0, z=0.0):
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        TYPE_MASK_POS_ONLY,
        x, y, z,
        0, 0, 0,
        0, 0, 0,
        0, 0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--connect", default="udpin:0.0.0.0:14550")
    parser.add_argument("--altitude", type=float, default=2.0,
                         help="hold altitude in meters (positive = up; sent as negative NED z)")
    parser.add_argument("--hold-seconds", type=float, default=60.0)
    parser.add_argument("--log", default=None, help="optional path to write CSV log of local position/attitude")
    args = parser.parse_args()

    z_sp = -abs(args.altitude)

    master = mavutil.mavlink_connection(args.connect)
    print("waiting for heartbeat...")
    master.wait_heartbeat(timeout=10)
    print(f"heartbeat from sys={master.target_system} comp={master.target_component}")

    logf = open(args.log, "w") if args.log else None
    if logf:
        logf.write("t,x,y,z,vz,roll,pitch,yaw\n")

    print("streaming setpoint for 2s before mode switch...")
    t0 = time.time()
    while time.time() - t0 < 2.0:
        send_sp(master, z=z_sp)
        time.sleep(0.05)

    print("requesting OFFBOARD mode...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        6, 0, 0, 0, 0, 0)  # PX4 custom main mode 6 = OFFBOARD

    time.sleep(0.5)

    print("arming...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0)

    print(f"streaming setpoint + logging for {args.hold_seconds:.0f}s...")
    t_start = time.time()
    last_local_pos = None
    last_attitude = None
    while time.time() - t_start < args.hold_seconds:
        send_sp(master, z=z_sp)
        msg = master.recv_match(type=['ATTITUDE', 'LOCAL_POSITION_NED'], blocking=False)
        if msg is not None:
            if msg.get_type() == 'LOCAL_POSITION_NED':
                last_local_pos = msg
            elif msg.get_type() == 'ATTITUDE':
                last_attitude = msg
            if logf and last_local_pos is not None and last_attitude is not None:
                logf.write(f"{time.time() - t_start:.3f},{last_local_pos.x:.4f},"
                           f"{last_local_pos.y:.4f},{last_local_pos.z:.4f},"
                           f"{last_local_pos.vz:.4f},{last_attitude.roll:.4f},"
                           f"{last_attitude.pitch:.4f},{last_attitude.yaw:.4f}\n")
        time.sleep(0.02)

    if logf:
        logf.close()
    print("done streaming")


if __name__ == "__main__":
    main()
