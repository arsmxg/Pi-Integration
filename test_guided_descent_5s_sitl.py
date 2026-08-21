#!/usr/bin/env python3
"""
Test 2 (SITL): 5-Second 1 m/s Guided Descent & Auto-Release (SITL UDP)
======================================================================
Purpose:
  Safely test GUIDED mode vertical rate control in SITL simulation (ArduPlane).
  Commands a smooth 1.0 m/s descent for exactly 5.0 seconds, then immediately
  releases control and restores the aircraft's previous flight mode (e.g. AUTO, FBWA, CRUISE).

  Waypoint Retention:
  When engaging from AUTO mode (e.g. flying leg from WP 5 to WP 6), the script
  remembers the active target waypoint (WP 6) and explicitly restores that target
  waypoint upon reverting back to AUTO, preventing ArduPlane from turning back to WP 5.

Safety Interlocks:
  1. Dynamic Relative Altitude Floor:
     Sets floor = h_0 - 10.0m upon engagement. If aircraft descends below this floor,
     control is released immediately.
  2. Pilot Throttle Override:
     If the pilot advances the physical RC throttle (> 1800 us), the script instantly
     aborts and hands back control to AUTO / prior mode with target waypoint preserved.
  3. Pilot Mode Switch Override:
     If the pilot flips the flight mode switch on the transmitter away from GUIDED,
     the script stands down immediately without overriding pilot manual selection.
  4. Precise 5-Second Time Limit:
     Strict timer releases control after 5.0 seconds of descent.

Usage Examples:
  # SITL Default (UDP 14550):
  python3 test_guided_descent_5s_sitl.py

  # Custom UDP / TCP Port:
  python3 test_guided_descent_5s_sitl.py --connect udp:127.0.0.1:14550
  python3 test_guided_descent_5s_sitl.py --connect udpin:0.0.0.0:14550
  python3 test_guided_descent_5s_sitl.py --connect tcp:127.0.0.1:5762
  python3 test_guided_descent_5s_sitl.py --release-mode AUTO
"""

import argparse
import math
import sys
import time
from typing import Optional

from pymavlink import mavutil

# Default Connection Settings for SITL (UDP)
SITL_CONNECTION = 'tcp:127.0.0.1:5762'

# ArduPlane Flight Modes
MODE_MANUAL = 0
MODE_FBWA = 5
MODE_FBWB = 6
MODE_CRUISE = 7
MODE_AUTO = 10
MODE_RTL = 11
MODE_GUIDED = 15

PLANE_MODES = {
    0: "MANUAL",
    1: "CIRCLE",
    2: "STABILIZE",
    3: "TRAINING",
    4: "ACRO",
    5: "FBWA",
    6: "FBWB",
    7: "CRUISE",
    8: "AUTOTUNE",
    10: "AUTO",
    11: "RTL",
    12: "LOITER",
    13: "TAKEOFF",
    14: "AVOID_ADSB",
    15: "GUIDED",
    16: "INITIALISING",
    17: "QSTABILIZE",
    18: "QHOVER",
    19: "QLOITER",
    20: "QLAND",
    21: "QRTL",
    22: "QAUTOTUNE",
    23: "QACRO",
    24: "THERMAL",
    25: "LOITER_ALT_QLAND",
}

PLANE_NAME_TO_MODE = {v: k for k, v in PLANE_MODES.items()}


class Telemetry:
    """Telemetry cache for live sensor streams."""

    WANTED = {
        'GLOBAL_POSITION_INT': 33,
        'ATTITUDE': 30,
        'RC_CHANNELS': 65,
        'VFR_HUD': 74,
        'HEARTBEAT': 0,
        'MISSION_CURRENT': 42,
    }

    def __init__(self, master, throttle_channel: int = 3):
        self.master = master
        self.throttle_channel = throttle_channel
        self.msgs = {}
        self.stamps = {}

    def request_streams(self, rate_hz: float = 10.0):
        for name, msgid in self.WANTED.items():
            if name == 'HEARTBEAT':
                continue
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                msgid, 1e6 / rate_hz, 0, 0, 0, 0, 0
            )

    def pump(self):
        while True:
            msg = self.master.recv_match(blocking=False)
            if msg is None:
                return
            if msg.get_srcSystem() != self.master.target_system:
                continue
            if msg.get_srcComponent() != mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1:
                continue
            t = msg.get_type()
            if t in self.WANTED:
                self.msgs[t] = msg
                self.stamps[t] = time.time()

    def get(self, name: str, max_age: Optional[float] = None):
        msg = self.msgs.get(name)
        if msg is None:
            return None
        if max_age is not None and (time.time() - self.stamps[name] > max_age):
            return None
        return msg

    def agl(self) -> Optional[float]:
        gpi = self.get('GLOBAL_POSITION_INT', 1.0)
        return (gpi.relative_alt / 1000.0) if gpi is not None else None

    def vertical_velocity(self) -> Optional[float]:
        gpi = self.get('GLOBAL_POSITION_INT', 1.0)
        return -(gpi.vz / 100.0) if gpi is not None else None  # Positive up, negative down (m/s)

    def groundspeed(self) -> Optional[float]:
        hud = self.get('VFR_HUD', 1.0)
        if hud is not None:
            return hud.groundspeed
        gpi = self.get('GLOBAL_POSITION_INT')
        return (math.hypot(gpi.vx, gpi.vy) / 100.0) if gpi is not None else None

    def heading_deg(self) -> Optional[float]:
        hud = self.get('VFR_HUD', 1.0)
        if hud is not None:
            return float(hud.heading)
        gpi = self.get('GLOBAL_POSITION_INT')
        return (gpi.hdg / 100.0) if (gpi is not None and gpi.hdg != 65535) else None

    def mode(self) -> Optional[int]:
        hb = self.get('HEARTBEAT', 3.0)
        return hb.custom_mode if hb is not None else None

    def is_armed(self) -> bool:
        hb = self.get('HEARTBEAT', 3.0)
        if hb is None:
            return False
        return bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    def throttle_pwm(self) -> Optional[int]:
        rc = self.get('RC_CHANNELS', 1.0)
        if rc is None:
            return None
        return getattr(rc, f'chan{self.throttle_channel}_raw', None)

    def current_waypoint(self) -> Optional[int]:
        mc = self.get('MISSION_CURRENT', 3.0)
        return mc.seq if mc is not None else None


def set_flight_mode(master, mode_number: int):
    """Command autopilot flight mode."""
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_number,
        0, 0, 0, 0, 0
    )


def set_mission_current(master, seq: int):
    """Set active mission target waypoint index over MAVLink."""
    master.mav.mission_set_current_send(
        master.target_system, master.target_component, seq
    )


def send_target_altitude(master, alt_target_m: float):
    """Command target altitude in GUIDED mode."""
    master.mav.command_int_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_ALTITUDE,
        0, 0,
        0, 0, 0, 0,  # Direct tracking without slew resets
        0, 0, alt_target_m
    )


def send_target_heading(master, heading_deg: float):
    """Hold ground-track heading in GUIDED mode."""
    master.mav.command_int_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL,
        mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_HEADING,
        0, 0,
        0,  # Course-over-ground
        heading_deg % 360.0,
        2.0,  # Max lateral accel limit
        0, 0, 0, 0
    )


def main():
    parser = argparse.ArgumentParser(description="Execute a 5-second 1 m/s descent test in GUIDED mode (SITL UDP) and release control.")
    parser.add_argument("--connect", default=SITL_CONNECTION, help=f"MAVLink connection string (e.g. {SITL_CONNECTION}, udpin:0.0.0.0:14550, tcp:127.0.0.1:5762, default: {SITL_CONNECTION})")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate (if serial port, default: 115200)")
    parser.add_argument("--descent-rate", type=float, default=1.0, help="Descent rate in m/s (default 1.0 m/s)")
    parser.add_argument("--duration", type=float, default=5.0, help="Descent duration in seconds (default 5.0 s)")
    parser.add_argument("--go-around-thr", type=int, default=1800, help="Pilot throttle PWM abort threshold (default 1800 us)")
    parser.add_argument("--throttle-ch", type=int, default=3, help="Throttle RC channel (default 3)")
    parser.add_argument("--release-mode", default="RESTORE", help="Flight mode to set upon test completion: 'RESTORE' (previous mode), 'AUTO', 'FBWA', 'CRUISE', 'RTL'")
    parser.add_argument("--rate", type=float, default=10.0, help="Control loop rate in Hz (default 10 Hz)")
    args = parser.parse_args()

    print("=" * 72)
    print("   5-Second 1 m/s GUIDED Mode Descent Test & Auto-Release          ")
    print("   [SITL UDP Configuration: ArduPlane Simulation]                  ")
    print("=" * 72)
    print(f"Connection String  : {args.connect} (baud={args.baud})")
    print(f"Target Sink Rate   : {args.descent_rate:.1f} m/s")
    print(f"Test Duration      : {args.duration:.1f} seconds (Expected Alt Drop: {args.descent_rate * args.duration:.1f} m)")
    print(f"Pilot Override Thr : > {args.go_around_thr} us (Channel {args.throttle_ch})")
    print(f"Release Flight Mode: {args.release_mode}")
    print("=" * 72)

    master = mavutil.mavlink_connection(
        args.connect,
        baud=args.baud,
        source_system=1,
        source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER
    )

    print("Connecting and waiting for Heartbeat...")
    master.wait_heartbeat()
    print(f"Heartbeat received (System #{master.target_system})")

    telem = Telemetry(master, throttle_channel=args.throttle_ch)
    telem.request_streams(rate_hz=args.rate)

    print("Collecting initial telemetry state (waiting up to 3s)...")
    t0 = time.time()
    while time.time() - t0 < 3.0:
        telem.pump()
        if telem.agl() is not None and telem.mode() is not None:
            # If in AUTO, also wait briefly for MISSION_CURRENT if not yet received
            if telem.mode() != MODE_AUTO or telem.current_waypoint() is not None:
                break
        time.sleep(0.05)

    initial_agl = telem.agl()
    initial_mode_num = telem.mode()
    initial_mode_name = PLANE_MODES.get(initial_mode_num, f"MODE_{initial_mode_num}")
    initial_hdg = telem.heading_deg() or 0.0
    saved_target_wp = telem.current_waypoint()

    if initial_agl is None or initial_mode_num is None:
        print("[!] ERROR: Failed to receive relative altitude or flight mode from autopilot.")
        sys.exit(1)

    print(f"\n[PRE-CHECK INITIAL STATE]")
    print(f" -> Initial Flight Mode : {initial_mode_name} (Mode #{initial_mode_num})")
    if saved_target_wp is not None:
        print(f" -> Active Target WP    : WP #{saved_target_wp} (will restore upon return to AUTO)")
    print(f" -> Starting Altitude   : {initial_agl:.2f} m AGL")
    print(f" -> Initial Heading     : {initial_hdg:.1f}°")
    print(f" -> Armed Status        : {'ARMED' if telem.is_armed() else 'DISARMED'}")

    # Determine handback flight mode
    if args.release_mode.upper() == "RESTORE":
        handback_mode_num = initial_mode_num
        handback_mode_name = initial_mode_name
    else:
        target_name = args.release_mode.upper()
        if target_name in PLANE_NAME_TO_MODE:
            handback_mode_num = PLANE_NAME_TO_MODE[target_name]
            handback_mode_name = target_name
        else:
            print(f"[!] Warning: Unknown release mode '{args.release_mode}'. Defaulting to initial mode {initial_mode_name}.")
            handback_mode_num = initial_mode_num
            handback_mode_name = initial_mode_name

    # Set Dynamic Relative Altitude Floor (start_alt - 10m)
    altitude_floor = initial_agl - 10.0
    expected_end_alt = initial_agl - (args.descent_rate * args.duration)

    print(f" -> Dynamic Alt Floor   : {altitude_floor:.2f} m AGL (10.0 m below start altitude)")
    print(f" -> Planned End Alt     : {expected_end_alt:.2f} m AGL")
    print(f" -> Handback Target Mode: {handback_mode_name} (#{handback_mode_num})")
    print("-" * 72)

    # Safety confirmation prompt / countdown
    print("\n[!] Engaging GUIDED mode and starting 5-second descent...")

    # Step 1: Engage GUIDED Mode
    set_flight_mode(master, MODE_GUIDED)
    t_mode_wait = time.time()
    while telem.mode() != MODE_GUIDED and time.time() - t_mode_wait < 3.0:
        telem.pump()
        time.sleep(0.02)

    if telem.mode() != MODE_GUIDED:
        print("[!] ERROR: Flight controller failed to confirm GUIDED mode. Aborting test.")
        sys.exit(1)

    print(f"[*] GUIDED mode confirmed! Test actively running for {args.duration:.1f}s.")
    print("-" * 72)
    print(f" {'t(s)':<6} | {'Target Alt':<11} | {'Live AGL':<10} | {'Sink Rate':<10} | {'Thr Stick':<10} | {'Mode'}")
    print("-" * 72)

    # Step 2: Run 5-Second Descent Loop
    start_time = time.time()
    last_cmd_time = 0.0
    last_print_time = 0.0
    loop_period = 1.0 / args.rate
    abort_reason = None

    while True:
        telem.pump()
        now = time.time()
        t_elapsed = now - start_time

        if t_elapsed >= args.duration:
            break

        current_agl = telem.agl()
        current_mode = telem.mode()
        current_vz = telem.vertical_velocity()
        current_thr = telem.throttle_pwm()
        current_hdg = telem.heading_deg() or initial_hdg

        # Safety Check 1: Pilot RC Throttle Override
        if current_thr is not None and current_thr > args.go_around_thr:
            abort_reason = f"Pilot Throttle Override detected ({current_thr} us > {args.go_around_thr} us)"
            break

        # Safety Check 2: Pilot Manual Mode Switch Override
        if current_mode is not None and current_mode != MODE_GUIDED:
            abort_reason = f"Pilot Mode Switch detected (switched to {PLANE_MODES.get(current_mode, current_mode)})"
            break

        # Safety Check 3: Dynamic Altitude Floor Exceeded
        if current_agl is not None and current_agl <= altitude_floor:
            abort_reason = f"Altitude Floor reached ({current_agl:.2f} m <= {altitude_floor:.2f} m)"
            break

        # Send altitude & heading guidance setpoints at configured rate
        if (now - last_cmd_time) >= loop_period:
            # Target altitude drops linearly at configured descent rate
            target_alt = initial_agl - (args.descent_rate * t_elapsed)
            send_target_altitude(master, target_alt)
            send_target_heading(master, initial_hdg)
            last_cmd_time = now

        # Print live telemetry at 5 Hz
        if (now - last_print_time) >= 0.20:
            target_alt = initial_agl - (args.descent_rate * t_elapsed)
            agl_str = f"{current_agl:6.2f} m" if current_agl is not None else "  N/A   "
            vz_str = f"{current_vz:6.2f} m/s" if current_vz is not None else "  N/A   "
            thr_str = f"{current_thr} us" if current_thr is not None else "  N/A  "
            mode_str = PLANE_MODES.get(current_mode, str(current_mode))
            print(f" {t_elapsed:5.2f}s | {target_alt:7.2f} m   | {agl_str:<10} | {vz_str:<10} | {thr_str:<10} | {mode_str}")
            last_print_time = now

        time.sleep(0.01)

    # Step 3: Test Completed / Handback Control
    total_elapsed = time.time() - start_time
    print("-" * 72)
    if abort_reason:
        print(f"\n[!] TEST ABORTED EARLY at t={total_elapsed:.2f}s: {abort_reason}")
    else:
        print(f"\n[+] 5.0-Second Descent Completed Successfully (Duration: {total_elapsed:.2f}s)!")

    # Release control logic
    if abort_reason and "Pilot Mode Switch detected" in abort_reason:
        print(f"[*] Standing down: Aircraft already in {PLANE_MODES.get(telem.mode(), telem.mode())} by pilot manual switch.")
    else:
        # If handback is AUTO and we have a remembered target waypoint, restore mission index first
        if handback_mode_num == MODE_AUTO and saved_target_wp is not None:
            print(f"[*] Pre-setting active AUTO target waypoint to WP #{saved_target_wp}...")
            set_mission_current(master, saved_target_wp)
            time.sleep(0.05)

        print(f"[*] RELEASING CONTROL -> Switching flight mode to {handback_mode_name} (#{handback_mode_num})...")
        set_flight_mode(master, handback_mode_num)

        # Re-send mission_set_current after mode change to guarantee active waypoint lock
        if handback_mode_num == MODE_AUTO and saved_target_wp is not None:
            time.sleep(0.05)
            set_mission_current(master, saved_target_wp)

        # Wait briefly for confirmation of mode restoration
        t_release = time.time()
        while time.time() - t_release < 3.0:
            telem.pump()
            if telem.mode() == handback_mode_num:
                if handback_mode_num != MODE_AUTO or (telem.current_waypoint() == saved_target_wp):
                    break
            time.sleep(0.05)

    final_agl = telem.agl()
    final_mode_num = telem.mode()
    final_mode_name = PLANE_MODES.get(final_mode_num, f"MODE_{final_mode_num}")
    final_wp = telem.current_waypoint()

    print("\n" + "=" * 72)
    print(" DESCENT TEST FLIGHT RESULTS SUMMARY")
    print("=" * 72)
    print(f" - Initial Altitude    : {initial_agl:.2f} m AGL")
    print(f" - Final Altitude      : {final_agl:.2f} m AGL" if final_agl is not None else " - Final Altitude      : N/A")
    if final_agl is not None:
        delta_alt = initial_agl - final_agl
        avg_sink = delta_alt / max(0.1, total_elapsed)
        print(f" - Total Alt Drop      : {delta_alt:.2f} m (Target: {args.descent_rate * min(total_elapsed, args.duration):.2f} m)")
        print(f" - Average Sink Rate   : {avg_sink:.2f} m/s (Target: {args.descent_rate:.2f} m/s)")
    print(f" - Handback Flight Mode: Confirmed in {final_mode_name} (#{final_mode_num})")
    if final_mode_num == MODE_AUTO and final_wp is not None:
        wp_status = f" (WP #{saved_target_wp} preserved)" if final_wp == saved_target_wp else ""
        print(f" - Active Target WP    : WP #{final_wp}{wp_status}")
    print("=" * 72)
    print("Test script finished. Aircraft control released.\n")


if __name__ == "__main__":
    main()
