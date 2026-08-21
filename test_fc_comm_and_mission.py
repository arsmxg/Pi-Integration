#!/usr/bin/env python3
"""
Test 1: Flight Controller Communication, Parameters & Mission Validator
========================================================================
Purpose:
  Pre-flight / ground test script for Cube Orange+ (ArduPlane 4.3+ / 4.5.6).
  Verifies bidirectional MAVLink communication, queries and validates all
  flare-relevant TECS/landing parameters, downloads the flight controller mission,
  and verifies the approach & landing geometry.

Usage Examples:
  # SITL Default:
  python3 test_fc_comm_and_mission.py --connect tcp:127.0.0.1:5762

  # Real Aircraft via USB/Serial:
  python3 test_fc_comm_and_mission.py --connect /dev/ttyUSB0 --baud 115200
  python3 test_fc_comm_and_mission.py --connect /dev/ttyAMA0 --baud 921600

  # Real Aircraft via UDP Telemetry:
  python3 test_fc_comm_and_mission.py --connect udp:127.0.0.1:14550
"""

import argparse
import math
import sys
import time
from typing import Dict, List, Optional

from pymavlink import mavutil

# ArduPlane Flight Mode Mapping
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

R_EARTH = 6378137.0


def haversine_dist(lat1_deg: float, lon1_deg: float, lat2_deg: float, lon2_deg: float) -> float:
    """Calculate horizontal distance in meters between two GPS coordinates."""
    lat1, lon1 = math.radians(lat1_deg), math.radians(lon1_deg)
    lat2, lon2 = math.radians(lat2_deg), math.radians(lon2_deg)
    dlat, dlon = lat2 - lat1, lon2 - lon1
    a = math.sin(dlat / 2.0) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2
    return 2.0 * R_EARTH * math.asin(math.sqrt(max(0.0, min(a, 1.0))))


def fetch_param(master, name: str, timeout: float = 2.0) -> Optional[float]:
    """Read a single parameter from the autopilot over MAVLink."""
    master.mav.param_request_read_send(
        master.target_system, master.target_component,
        name.encode('ascii'), -1
    )
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
        if msg is not None and msg.param_id.strip('\x00') == name:
            return msg.param_value
    return None


def fetch_all_relevant_params(master) -> Dict[str, Optional[float]]:
    """Query all parameters relevant to the exponential flare controller."""
    params_to_query = [
        ('TECS_VERT_ACC', 'Max vertical acceleration [m/s^2]', 1.5),
        ('LAND_FLARE_SEC', 'Target flare duration [s]', 3.0),
        ('LAND_FLARE_ALT', 'Minimum backup flare altitude floor [m]', 3.0),
        ('TECS_LAND_SINK', 'Target touchdown sink rate [m/s]', 0.30),
        ('TRIM_ARSPD_CM', 'Cruise/Approach airspeed target [cm/s]', 1500.0),
        ('ARSPD_FBW_MIN', 'Minimum FBW airspeed [m/s]', 9.0),
        ('ARSPD_FBW_MAX', 'Maximum FBW airspeed [m/s]', 22.0),
        ('TECS_SINK_MAX', 'TECS maximum sink rate [m/s]', 5.0),
        ('TECS_PITCH_MAX', 'TECS max pitch angle [deg]', 20.0),
        ('TECS_PITCH_MIN', 'TECS min pitch angle [deg]', -15.0),
        ('TKOFF_ALT', 'Takeoff climbout altitude [m]', 30.0),
        ('RTL_ALT', 'Return to launch altitude [m]', 50.0),
        ('FLTMODE_CH', 'Flight mode channel number', 5.0),
    ]

    results = {}
    print("\n" + "=" * 72)
    print(" 1. AUTOPILOT PARAMETER EXTRACTION & VALIDATION")
    print("=" * 72)
    print(f" {'Parameter':<16} | {'Value':<10} | {'Default':<10} | {'Description'}")
    print("-" * 72)

    for param_name, desc, default_val in params_to_query:
        val = fetch_param(master, param_name, timeout=1.5)
        results[param_name] = val
        val_str = f"{val:8.2f}" if val is not None else "  [N/A]  "
        print(f" {param_name:<16} | {val_str:<10} | {default_val:<10.2f} | {desc}")

    return results


def download_mission(master) -> List:
    """Download the full mission plan from the flight controller."""
    print("\n" + "=" * 72)
    print(" 2. FLIGHT CONTROLLER MISSION DOWNLOAD")
    print("=" * 72)
    master.waypoint_request_list_send()
    count_msg = master.recv_match(type='MISSION_COUNT', blocking=True, timeout=5.0)
    if count_msg is None:
        raise RuntimeError("Timeout requesting MISSION_COUNT from autopilot.")

    print(f"Found {count_msg.count} mission items on flight controller.")
    if count_msg.count == 0:
        return []

    wps = []
    for i in range(count_msg.count):
        master.mav.mission_request_int_send(master.target_system, master.target_component, i)
        wp = master.recv_match(type='MISSION_ITEM_INT', blocking=True, timeout=5.0)
        if wp is None:
            raise RuntimeError(f"Timeout fetching mission item index {i}")
        wps.append(wp)

    master.mav.mission_ack_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_MISSION_ACCEPTED
    )
    return wps


def print_mission_table(wps: List):
    """Print a clean summary table of the downloaded mission."""
    cmd_names = {
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT: "NAV_WAYPOINT",
        mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT: "NAV_SPLINE_WP",
        mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS: "NAV_LOITER_TURNS",
        mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME: "NAV_LOITER_TIME",
        mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM: "NAV_LOITER_UNLIM",
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH: "NAV_RTL",
        mavutil.mavlink.MAV_CMD_NAV_LAND: "NAV_LAND",
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF: "NAV_TAKEOFF",
        mavutil.mavlink.MAV_CMD_DO_LAND_START: "DO_LAND_START",
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO: "DO_SET_SERVO",
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED: "DO_CHANGE_SPEED",
    }

    print("-" * 72)
    print(f" {'Seq':<4} | {'Command Name':<16} | {'Lat (deg)':<11} | {'Lon (deg)':<11} | {'Alt (m)':<8} | {'Frame'}")
    print("-" * 72)
    for i, w in enumerate(wps):
        cname = cmd_names.get(w.command, f"CMD_{w.command}")
        lat = f"{w.x / 1e7:10.6f}" if (w.x != 0) else "   ---    "
        lon = f"{w.y / 1e7:10.6f}" if (w.y != 0) else "   ---    "
        alt = f"{w.z:7.1f}"
        print(f" #{i:<3} | {cname:<16} | {lat} | {lon} | {alt} | {w.frame}")
    print("-" * 72)


def analyze_landing_geometry(wps: List, params: Dict[str, Optional[float]]):
    """Analyze the landing sequence, approach glideslope, and dynamic flare altitude."""
    print("\n" + "=" * 72)
    print(" 3. APPROACH & FLARE GEOMETRY ANALYSIS")
    print("=" * 72)

    if not wps:
        print("[!] No mission items available to analyze.")
        return

    # 1. Locate DO_LAND_START
    do_land_idx = next((i for i, w in enumerate(wps) if w.command == mavutil.mavlink.MAV_CMD_DO_LAND_START), None)

    # 2. Locate Next WP after DO_LAND_START
    do_land_next_idx = None
    if do_land_idx is not None:
        nav_commands = {
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT,
            mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,
            mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
            mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        }
        do_land_next_idx = next(
            (i for i in range(do_land_idx + 1, len(wps)) if wps[i].command in nav_commands),
            (do_land_idx + 1) if (do_land_idx + 1 < len(wps)) else None
        )

    # 3. Locate NAV_LAND
    land_idx = next((i for i, w in enumerate(wps) if w.command == mavutil.mavlink.MAV_CMD_NAV_LAND), None)

    if land_idx is None:
        print("[!] WARNING: Mission does NOT contain a NAV_LAND item (MAV_CMD_NAV_LAND).")
        return

    # 4. Locate NAV_WAYPOINT before NAV_LAND
    approach_idx = next((i for i in range(land_idx - 1, -1, -1)
                         if wps[i].command == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT), None)

    if approach_idx is None:
        print("[!] WARNING: No NAV_WAYPOINT found preceding NAV_LAND.")
        return

    approach_wp = wps[approach_idx]
    land_wp = wps[land_idx]

    lat1, lon1 = approach_wp.x / 1e7, approach_wp.y / 1e7
    lat2, lon2 = land_wp.x / 1e7, land_wp.y / 1e7
    alt1, alt2 = approach_wp.z, land_wp.z

    # Runway track bearing
    r_lat1, r_lon1 = math.radians(lat1), math.radians(lon1)
    r_lat2, r_lon2 = math.radians(lat2), math.radians(lon2)
    dlon = r_lon2 - r_lon1
    x = math.sin(dlon) * math.cos(r_lat2)
    y = math.cos(r_lat1) * math.sin(r_lat2) - math.sin(r_lat1) * math.cos(r_lat2) * math.cos(dlon)
    track_rad = math.atan2(x, y)
    track_deg = (math.degrees(track_rad) + 360.0) % 360.0

    # Approach distance & slope
    app_dist = haversine_dist(lat1, lon1, lat2, lon2)
    alt_drop = alt1 - alt2
    if app_dist > 1.0 and alt_drop > 0:
        gradient = alt_drop / app_dist
        gs_deg = math.degrees(math.atan(gradient))
    else:
        gradient = 0.0524
        gs_deg = 3.0

    # Param fallbacks
    vert_acc = params.get('TECS_VERT_ACC') or 1.5
    flare_sec = params.get('LAND_FLARE_SEC') or 3.0
    flare_alt_backup = params.get('LAND_FLARE_ALT') or 3.0
    td_sink = params.get('TECS_LAND_SINK') or 0.30
    trim_arspd_cm = params.get('TRIM_ARSPD_CM')
    app_spd = (trim_arspd_cm / 100.0) if (trim_arspd_cm and trim_arspd_cm > 0) else 15.0

    # Dynamic Flare Calculations
    hdot_app = -(app_spd * gradient)
    hdot_td = -abs(td_sink)
    tau_accel = abs(hdot_app) / vert_acc
    delta_sink = max(0.1, abs(hdot_app) - abs(hdot_td))
    h_flare_accel = tau_accel * delta_sink

    sink_ratio = abs(hdot_app) / max(0.05, abs(hdot_td))
    tau_time = flare_sec / math.log(sink_ratio) if sink_ratio > 1.05 else flare_sec
    h_flare_time = tau_time * delta_sink

    h_flare = max(h_flare_accel, h_flare_time, flare_alt_backup)
    tau_exp = h_flare / delta_sink
    initial_accel = abs(hdot_app) / tau_exp
    actual_duration = tau_exp * math.log(sink_ratio) if sink_ratio > 1.0 else flare_sec
    ground_run = app_spd * actual_duration
    engage_dist_to_land = h_flare / gradient

    print(f" -> DO_LAND_START Item      : #{do_land_idx if do_land_idx is not None else '(None)'}")
    print(f" -> Go-Around Target WP     : #{do_land_next_idx if do_land_next_idx is not None else '(None)'}")
    print(f" -> Approach Waypoint Leg   : #{approach_idx} -> NAV_LAND #{land_idx}")
    print(f" -> Approach Leg Distance   : {app_dist:.1f} m (Alt Drop: {alt_drop:.1f} m)")
    print(f" -> Runway Centerline Track : {track_deg:.1f}°")
    print(f" -> Approach Glideslope     : {gs_deg:.2f}° ({gradient * 100:.1f}%)")
    print(f" -> Approach Sink Rate      : {hdot_app:.2f} m/s (at {app_spd:.1f} m/s groundspeed)")
    print(f" -> Touchdown Target Sink   : {hdot_td:.2f} m/s")
    print("-" * 72)
    print(f" -> Flare Alt (Accel Bound) : {h_flare_accel:.2f} m AGL (Limit: {vert_acc:.2f} m/s²)")
    print(f" -> Flare Alt (Time Bound)  : {h_flare_time:.2f} m AGL (Target: {flare_sec:.1f} s)")
    print(f" -> Flare Alt (Backup Floor): {flare_alt_backup:.2f} m AGL")
    print(f" => DECIDED FLARE HEIGHT    : {h_flare:.2f} m AGL")
    print(f" -> Initial Acceleration    : {initial_accel:.2f} m/s² (Bounded <= {vert_acc:.2f} m/s²)")
    print(f" -> Predicted Ground Run    : {ground_run:.1f} m over {actual_duration:.2f} s")
    print(f" -> Trigger Point Distance  : {engage_dist_to_land:.1f} m ahead of NAV_LAND")


def test_live_telemetry(master, timeout_s: float = 3.0):
    """Test receipt of essential live telemetry messages."""
    print("\n" + "=" * 72)
    print(" 4. LIVE TELEMETRY STREAM TEST")
    print("=" * 72)

    # Request streams at 10 Hz
    stream_ids = [
        (33, "GLOBAL_POSITION_INT"),
        (30, "ATTITUDE"),
        (65, "RC_CHANNELS"),
        (74, "VFR_HUD"),
        (42, "MISSION_CURRENT"),
        (1,  "SYS_STATUS"),
    ]

    for msgid, name in stream_ids:
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            msgid, 1e6 / 10.0, 0, 0, 0, 0, 0
        )

    deadline = time.time() + timeout_s
    received = {}
    while time.time() < deadline:
        msg = master.recv_match(blocking=True, timeout=0.2)
        if msg is None:
            continue
        mtype = msg.get_type()
        if mtype not in received:
            received[mtype] = msg
        if len(received) >= len(stream_ids):
            break

    for _, name in stream_ids:
        msg = received.get(name)
        status = "OK" if msg is not None else "TIMEOUT (No Data)"
        detail = ""
        if msg is not None:
            if name == "GLOBAL_POSITION_INT":
                detail = f"RelAlt: {msg.relative_alt / 1000.0:.2f}m, Vz: {-msg.vz / 100.0:.2f}m/s"
            elif name == "VFR_HUD":
                detail = f"AS: {msg.airspeed:.1f}m/s, GS: {msg.groundspeed:.1f}m/s, Hdg: {msg.heading}°"
            elif name == "RC_CHANNELS":
                detail = f"Ch3 (Thr): {getattr(msg, 'chan3_raw', 0)}us, Ch5 (Mode): {getattr(msg, 'chan5_raw', 0)}us"
            elif name == "ATTITUDE":
                detail = f"Roll: {math.degrees(msg.roll):.1f}°, Pitch: {math.degrees(msg.pitch):.1f}°"
            elif name == "SYS_STATUS":
                detail = f"Batt: {msg.voltage_battery / 1000.0:.2f}V, Curr: {msg.current_battery / 100.0:.1f}A"
        print(f" [{status:^7}] {name:<22} : {detail}")

    return len(received) == len(stream_ids)


def main():
    parser = argparse.ArgumentParser(description="Test communication, parameters, and mission geometry with Cube Orange+ flight controller.")
    parser.add_argument("--connect", default="tcp:127.0.0.1:5762", help="MAVLink connection string (e.g. /dev/ttyUSB0, /dev/ttyAMA0, tcp:127.0.0.1:5762, udp:127.0.0.1:14550)")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate (if using serial port, e.g. 57600, 115200, 921600)")
    args = parser.parse_args()

    print("=" * 72)
    print("   Cube Orange+ Communication, Parameters & Mission Validator       ")
    print("=" * 72)
    print(f"Connecting to: {args.connect} (baud={args.baud})...")

    try:
        master = mavutil.mavlink_connection(
            args.connect,
            baud=args.baud,
            source_system=1,
            source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER
        )
        print("Waiting for autopilot HEARTBEAT...")
        hb = master.wait_heartbeat(timeout=10)
        if hb is None:
            print("[!] ERROR: No heartbeat received from flight controller.")
            sys.exit(1)

        mode_name = PLANE_MODES.get(hb.custom_mode, f"MODE_{hb.custom_mode}")
        armed_str = "ARMED" if (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) else "DISARMED"
        print(f"Heartbeat received from System #{master.target_system}, Component #{master.target_component}")
        print(f"Autopilot Type: {hb.autopilot}, Vehicle Type: {hb.type}, Current Mode: {mode_name} ({armed_str})")

        # 1. Parameter Extraction
        params = fetch_all_relevant_params(master)

        # 2. Mission Download
        wps = download_mission(master)
        print_mission_table(wps)

        # 3. Landing & Flare Geometry Analysis
        analyze_landing_geometry(wps, params)

        # 4. Telemetry Stream Validation
        telem_ok = test_live_telemetry(master)

        print("\n" + "=" * 72)
        print(" 5. PRE-FLIGHT READINESS SUMMARY")
        print("=" * 72)
        print(f" - MAVLink Heartbeat Link     : PASS (System #{master.target_system})")
        print(f" - Parameters Query           : {'PASS' if params.get('TECS_VERT_ACC') is not None else 'WARNING (used defaults)'}")
        print(f" - Mission Plan Download      : {'PASS' if len(wps) > 0 else 'FAIL (Empty mission)'}")
        print(f" - Telemetry Streams          : {'PASS' if telem_ok else 'PARTIAL'}")
        print("=" * 72)
        print("Pre-flight ground check complete.\n")

    except Exception as e:
        print(f"\n[!] ERROR encountered: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
