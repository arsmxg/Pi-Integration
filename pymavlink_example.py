#!/usr/bin/env python3
from pymavlink import mavutil
import sys
import time

# Change this to your USB serial device
# Common examples:
# Linux: /dev/ttyACM0 or /dev/ttyUSB0
# Mac:   /dev/tty.usbmodem* or /dev/tty.usbserial*
# Win:   COM3, COM4, etc.
DEVICE = "/dev/ttyACM0"
BAUD = 115200
HEARTBEAT_TIMEOUT = 10


def main():
    print(f"Connecting to {DEVICE} at {BAUD} baud...")

    try:
        master = mavutil.mavlink_connection(DEVICE, baud=BAUD)
    except Exception as e:
        print(f"Failed to open port: {e}")
        sys.exit(1)

    print("Waiting for heartbeat...")
    hb = master.wait_heartbeat(timeout=HEARTBEAT_TIMEOUT)

    if hb is None:
        print(f"No heartbeat received within {HEARTBEAT_TIMEOUT} seconds.")
        sys.exit(2)

    print("Connected.")
    print(f"System ID: {master.target_system}")
    print(f"Component ID: {master.target_component}")

    # Try reading a few MAVLink messages
    print("\nListening for messages for 5 seconds...\n")
    end_time = time.time() + 5

    while time.time() < end_time:
        msg = master.recv_match(blocking=True, timeout=1)
        if msg is not None:
            print(msg)

    print("\nConnection test complete.")


if __name__ == "__main__":
    main()