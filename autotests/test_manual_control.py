#!/usr/bin/env python3
"""
Send MANUAL_CONTROL messages to the SITL MavlinkGateway to verify
that the receive-side parsing and RC forwarding work correctly.

Usage (while SITL is running):
    source ../../venvs/fprime311/bin/activate
    python3 autotests/test_manual_control.py

What to check in GDS telemetry:
    - rcMsgCount should increment at ~10 Hz
    - rcRoll should sweep between -1.0 and +1.0 (sine wave)
    - mavMsgsRecvd should increment

Also listens on port 14550 for telemetry coming back from SITL to
verify bidirectional communication.

Press Ctrl+C to stop.
"""

import math
import time
import socket
import struct
import sys

from pymavlink import mavutil

TARGET_HOST = "127.0.0.1"
TARGET_PORT = 14540       # MavlinkGateway bind port
LISTEN_PORT = 14550       # Where MavlinkGateway sends telemetry
SEND_HZ = 10
SYS_ID = 255             # GCS system ID (standard for GCS)
COMP_ID = 190             # MAV_COMP_ID_MISSIONPLANNER


def main():
    # --- Sender: sends MANUAL_CONTROL to SITL ---
    print(f"Opening sender to {TARGET_HOST}:{TARGET_PORT} ...")
    sender = mavutil.mavlink_connection(
        f"udpout:{TARGET_HOST}:{TARGET_PORT}",
        source_system=SYS_ID,
        source_component=COMP_ID,
    )

    # --- Listener: receives telemetry from SITL on port 14550 ---
    print(f"Opening listener on 0.0.0.0:{LISTEN_PORT} ...")
    try:
        listener = mavutil.mavlink_connection(
            f"udpin:0.0.0.0:{LISTEN_PORT}",
            source_system=SYS_ID,
            source_component=COMP_ID,
        )
        has_listener = True
        print("Listener ready.\n")
    except Exception as e:
        print(f"Could not bind listener on {LISTEN_PORT} (MissionPlanner using it?): {e}")
        print("Will still send — check GDS for rcMsgCount/rcRoll.\n")
        has_listener = False

    print(f"Sending MANUAL_CONTROL at {SEND_HZ} Hz")
    print("Roll sweeps as sine wave (-1000..+1000), pitch/yaw=0, throttle=500")
    print("-----------------------------------------------------------\n")

    t0 = time.monotonic()
    count = 0
    telem_count = 0
    last_attitude_roll = None

    try:
        while True:
            t = time.monotonic() - t0

            # Sine sweep on roll: period = 4 seconds
            roll = int(1000 * math.sin(2 * math.pi * t / 4.0))
            pitch = 0
            throttle = 500   # mid-range (0..1000)
            yaw = 0

            sender.mav.manual_control_send(
                target=1,           # target system (our SITL is sysid 1)
                x=pitch,            # x = pitch  (-1000..+1000)
                y=roll,             # y = roll   (-1000..+1000)
                z=throttle,         # z = throttle (0..1000)
                r=yaw,              # r = yaw    (-1000..+1000)
                buttons=0,
            )
            count += 1

            # Non-blocking: check for telemetry back from SITL
            if has_listener:
                while True:
                    msg = listener.recv_match(blocking=False)
                    if msg is None:
                        break
                    telem_count += 1
                    if msg.get_type() == "ATTITUDE":
                        last_attitude_roll = math.degrees(msg.roll)

            if count % SEND_HZ == 0:
                elapsed = time.monotonic() - t0
                telem_str = f"telem_msgs={telem_count}"
                if last_attitude_roll is not None:
                    telem_str += f"  att_roll={last_attitude_roll:+6.1f}°"
                print(f"  [{elapsed:6.1f}s] sent={count}  roll_cmd={roll:+5d}  |  {telem_str}")

            time.sleep(1.0 / SEND_HZ)

    except KeyboardInterrupt:
        elapsed = time.monotonic() - t0
        print(f"\nStopped. Sent {count} MANUAL_CONTROL msgs in {elapsed:.1f}s, received {telem_count} telem msgs back.")
        if telem_count > 0:
            print("PASS: Bidirectional MAVLink works. SITL received and responded.")
        else:
            print("NOTE: No telemetry received on listener. Check if port 14550 was free.")
        if count > 0:
            print("\nCheck GDS telemetry for rcMsgCount > 0 and rcRoll sweeping.")


if __name__ == "__main__":
    main()
