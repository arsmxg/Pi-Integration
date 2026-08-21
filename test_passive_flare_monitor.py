#!/usr/bin/env python3
"""
Test 3: Passive Shadow Monitor for Exponential Flare Controller
===============================================================
Purpose:
  Runs alongside normal flight/approach operations on the aircraft WITHOUT
  EVER sending control commands (zero mode switches, zero altitude setpoints,
  zero airspeed/heading setpoints, zero disarm commands).

  Verifies:
    1. Mission geometry parsing and dynamic flare height sizing.
    2. Step-by-step state machine transitions (WAIT_APPROACH -> FLARE -> ROLLOUT -> STOPPED).
    3. Analytical trajectory computations (h_ref, hdot_ref, az_ref, XTK, target course).
    4. Pilot RC go-around trigger recognition (throttle > threshold) and intended WP target.
    5. Pilot flight mode switch recognition and controller stand-down logic.
    6. Optional CSV flight-data logging for post-flight telemetry overlay.

Usage Examples:
  # Default (Connects to /dev/ttyACM0 @ 115200 baud, logs live flight data to passive_test.csv):
  python3 test_passive_flare_monitor.py

  # SITL Passive Monitor:
  python3 test_passive_flare_monitor.py --connect tcp:127.0.0.1:5762

  # Custom Serial Port / Custom Log:
  python3 test_passive_flare_monitor.py --connect /dev/ttyUSB0 --baud 115200
  python3 test_passive_flare_monitor.py --connect /dev/ttyAMA0 --baud 921600 --log-csv custom_flight_log.csv
"""

import argparse
import csv
import math
import os
import sys
import time
from dataclasses import dataclass
from typing import Optional, Tuple

from pymavlink import mavutil

# ArduPlane Custom Flight Modes
MODE_MANUAL = 0
MODE_FBWA = 5
MODE_FBWB = 6
MODE_CRUISE = 7
MODE_AUTO = 10
MODE_RTL = 11
MODE_TAKEOFF = 13
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

# Default Connection & Logging Settings
DEVICE = "/dev/ttyACM0"
BAUD = 115200
DEFAULT_CSV_LOG = "passive_test.csv"

R_EARTH = 6378137.0


@dataclass
class Config:
    connection: str = DEVICE
    baud: int = BAUD

    # Autopilot Defaults / Fallbacks
    default_vert_acc: float = 1.5         # Max vertical acceleration [m/s^2] (TECS_VERT_ACC)
    default_flare_sec: float = 3.0        # Target flare duration [s] (LAND_FLARE_SEC)
    default_flare_alt_backup: float = 3.0 # Minimum backup flare altitude [m] (LAND_FLARE_ALT)
    default_td_sink: float = 0.30         # Target touchdown sink rate [m/s] (TECS_LAND_SINK)
    default_approach_speed: float = 15.0  # Nominal approach groundspeed [m/s]

    # Termination, Rollout & Disarm
    touchdown_agl: float = 0.30           # Altitude declaring ground contact [m]
    rollout_stop_gs: float = 1.5          # Groundspeed declaring aircraft stopped [m/s]
    auto_disarm_delay_s: float = 30.0     # Time after stopping before auto-disarm [s]

    # Lateral Guidance
    t_intercept: float = 4.0              # Cross-track closure time constant [s]
    v_close_max: float = 3.0              # Max lateral closure velocity [m/s]
    max_course_corr_deg: float = 20.0     # Maximum course correction angle [deg]
    hdg_accel_limit: float = 2.0          # Lateral acceleration limit [m/s^2]

    # Pilot Authority
    go_around_thr_pwm: int = 1800         # RC throttle PWM threshold for go-around (> 80% stick)
    throttle_channel: int = 3             # RC throttle channel index

    # Loop Rate
    loop_hz: float = 20.0


@dataclass
class MissionApproachGeometry:
    track_rad: float
    track_deg: float
    approach_lat: float
    approach_lon: float
    approach_alt: float
    approach_wp_seq: int
    land_lat: float
    land_lon: float
    land_alt: float
    land_seq: int
    approach_dist_m: float
    approach_alt_drop_m: float
    glideslope_deg: float
    glideslope_gradient: float
    do_land_start_seq: Optional[int]
    do_land_start_next_seq: Optional[int]
    takeoff_alt_m: Optional[float]


@dataclass
class DynamicExponentialProfile:
    h_flare: float
    h_flare_accel: float
    h_flare_time: float
    h_flare_backup: float
    max_vert_acc: float
    hdot_approach: float
    hdot_td: float
    tau_exp: float
    h_infty: float
    flare_duration_s: float
    flare_ground_dist_m: float
    flare_to_land_dist_m: float


class Telemetry:
    """Telemetry receiver and cache populated from MAVLink streams."""

    WANTED = {
        'GLOBAL_POSITION_INT': 33,
        'ATTITUDE': 30,
        'MISSION_CURRENT': 42,
        'RC_CHANNELS': 65,
        'VFR_HUD': 74,
        'HEARTBEAT': 0,
    }

    def __init__(self, master, throttle_channel: int = 3):
        self.master = master
        self.throttle_channel = throttle_channel
        self.msgs = {}
        self.stamps = {}

    def request_streams(self):
        for name, msgid in self.WANTED.items():
            if name == 'HEARTBEAT':
                continue
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                msgid, 1e6 / 10.0, 0, 0, 0, 0, 0
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
        return -(gpi.vz / 100.0) if gpi is not None else None  # + = up, - = down (m/s)

    def groundspeed(self) -> Optional[float]:
        hud = self.get('VFR_HUD', 1.0)
        if hud is not None:
            return hud.groundspeed
        gpi = self.get('GLOBAL_POSITION_INT')
        return (math.hypot(gpi.vx, gpi.vy) / 100.0) if gpi is not None else None

    def airspeed(self) -> Optional[float]:
        hud = self.get('VFR_HUD', 1.0)
        return hud.airspeed if hud is not None else None

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
        return getattr(rc, f'chan{self.throttle_channel}_raw', None) if rc is not None else None

    def roll_deg(self) -> Optional[float]:
        att = self.get('ATTITUDE', 1.0)
        return math.degrees(att.roll) if att is not None else None

    def pitch_deg(self) -> Optional[float]:
        att = self.get('ATTITUDE', 1.0)
        return math.degrees(att.pitch) if att is not None else None

    def heading_deg(self) -> Optional[float]:
        hud = self.get('VFR_HUD', 1.0)
        if hud is not None:
            return float(hud.heading)
        gpi = self.get('GLOBAL_POSITION_INT')
        return (gpi.hdg / 100.0) if (gpi is not None and gpi.hdg != 65535) else None

    def lat_lon(self) -> Tuple[Optional[float], Optional[float]]:
        gpi = self.get('GLOBAL_POSITION_INT', 1.0)
        if gpi is not None and (gpi.lat != 0 or gpi.lon != 0):
            return gpi.lat / 1e7, gpi.lon / 1e7
        return None, None


def fetch_param(master, name: str, timeout: float = 2.0) -> Optional[float]:
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


def haversine_dist(lat1_deg: float, lon1_deg: float, lat2_deg: float, lon2_deg: float) -> float:
    lat1, lon1 = math.radians(lat1_deg), math.radians(lon1_deg)
    lat2, lon2 = math.radians(lat2_deg), math.radians(lon2_deg)
    dlat, dlon = lat2 - lat1, lon2 - lon1
    a = math.sin(dlat / 2.0) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2
    return 2.0 * R_EARTH * math.asin(math.sqrt(max(0.0, min(a, 1.0))))


def extract_mission_geometry(master) -> MissionApproachGeometry:
    print('\n[1/3] Downloading flight mission from autopilot...')
    master.waypoint_request_list_send()
    count_msg = master.recv_match(type='MISSION_COUNT', blocking=True, timeout=5)
    if count_msg is None:
        raise RuntimeError('Timeout requesting MISSION_COUNT from autopilot.')

    wps = []
    for i in range(count_msg.count):
        master.mav.mission_request_int_send(master.target_system, master.target_component, i)
        wp = master.recv_match(type='MISSION_ITEM_INT', blocking=True, timeout=5)
        if wp is None:
            raise RuntimeError(f'Timeout fetching mission item {i}')
        wps.append(wp)

    master.mav.mission_ack_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_MISSION_ACCEPTED
    )

    # 1. Search for DO_LAND_START
    do_land_start_idx = next((i for i, w in enumerate(wps)
                              if w.command == mavutil.mavlink.MAV_CMD_DO_LAND_START), None)

    # 2. Locate navigation waypoint immediately after DO_LAND_START
    do_land_start_next_idx = None
    if do_land_start_idx is not None:
        nav_commands = {
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT,
            mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,
            mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
            mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        }
        do_land_start_next_idx = next(
            (i for i in range(do_land_start_idx + 1, len(wps)) if wps[i].command in nav_commands),
            (do_land_start_idx + 1) if (do_land_start_idx + 1 < len(wps)) else None
        )

    # 3. Takeoff altitude
    takeoff_wp = next((w for w in wps if w.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF), None)
    takeoff_alt = takeoff_wp.z if takeoff_wp is not None else fetch_param(master, 'TKOFF_ALT')

    # 4. Locate NAV_LAND
    land_idx = next((i for i, w in enumerate(wps)
                     if w.command == mavutil.mavlink.MAV_CMD_NAV_LAND), None)
    if land_idx is None or land_idx == 0:
        raise RuntimeError('Mission does not contain a NAV_LAND item with a preceding waypoint.')

    # 5. Locate immediate NAV_WAYPOINT before NAV_LAND
    approach_wp_idx = next((i for i in range(land_idx - 1, -1, -1)
                            if wps[i].command == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT), None)
    if approach_wp_idx is None:
        raise RuntimeError('No NAV_WAYPOINT found before NAV_LAND in the mission plan.')

    approach_wp = wps[approach_wp_idx]
    land_wp = wps[land_idx]

    lat1, lon1 = approach_wp.x / 1e7, approach_wp.y / 1e7
    lat2, lon2 = land_wp.x / 1e7, land_wp.y / 1e7
    alt1, alt2 = approach_wp.z, land_wp.z

    r_lat1, r_lon1 = math.radians(lat1), math.radians(lon1)
    r_lat2, r_lon2 = math.radians(lat2), math.radians(lon2)
    dlon = r_lon2 - r_lon1
    x = math.sin(dlon) * math.cos(r_lat2)
    y = math.cos(r_lat1) * math.sin(r_lat2) - math.sin(r_lat1) * math.cos(r_lat2) * math.cos(dlon)
    track_rad = math.atan2(x, y)
    track_deg = (math.degrees(track_rad) + 360.0) % 360.0

    approach_dist = haversine_dist(lat1, lon1, lat2, lon2)
    alt_drop = alt1 - alt2
    if approach_dist < 1.0 or alt_drop <= 0:
        gradient = 0.0524
        gs_deg = 3.0
    else:
        gradient = alt_drop / approach_dist
        gs_deg = math.degrees(math.atan(gradient))

    geom = MissionApproachGeometry(
        track_rad=track_rad,
        track_deg=track_deg,
        approach_lat=lat1,
        approach_lon=lon1,
        approach_alt=alt1,
        approach_wp_seq=approach_wp_idx,
        land_lat=lat2,
        land_lon=lon2,
        land_alt=alt2,
        land_seq=land_idx,
        approach_dist_m=approach_dist,
        approach_alt_drop_m=alt_drop,
        glideslope_deg=gs_deg,
        glideslope_gradient=gradient,
        do_land_start_seq=do_land_start_idx,
        do_land_start_next_seq=do_land_start_next_idx,
        takeoff_alt_m=takeoff_alt,
    )

    print(f' -> Mission Land Seq       : #{geom.land_seq}')
    print(f' -> Approach Waypoint Seq  : #{geom.approach_wp_seq}')
    print(f' -> DO_LAND_START Sequence : {"#" + str(geom.do_land_start_seq) if geom.do_land_start_seq is not None else "(none found)"}')
    print(f' -> Go-Around Target WP    : {"#" + str(geom.do_land_start_next_seq) if geom.do_land_start_next_seq is not None else "(default)"}')
    print(f' -> Approach Leg Distance  : {geom.approach_dist_m:.1f} m (Alt Drop: {geom.approach_alt_drop_m:.1f} m)')
    print(f' -> Runway Track Bearing   : {geom.track_deg:.1f}°')
    print(f' -> Approach Glideslope    : {geom.glideslope_deg:.2f}° ({geom.glideslope_gradient * 100:.1f}%)')
    return geom


def build_dynamic_exponential_profile(master, geom: MissionApproachGeometry, cfg: Config) -> DynamicExponentialProfile:
    print('\n[2/3] Extracting autopilot constraints & calculating dynamic flare height...')

    vert_acc_param = fetch_param(master, 'TECS_VERT_ACC')
    max_vert_acc = vert_acc_param if (vert_acc_param is not None and vert_acc_param > 0) else cfg.default_vert_acc

    flare_sec_param = fetch_param(master, 'LAND_FLARE_SEC')
    flare_sec = flare_sec_param if (flare_sec_param is not None and flare_sec_param > 0) else cfg.default_flare_sec

    flare_alt_backup_param = fetch_param(master, 'LAND_FLARE_ALT')
    flare_alt_backup = flare_alt_backup_param if (flare_alt_backup_param is not None and flare_alt_backup_param > 0) else cfg.default_flare_alt_backup

    td_sink_param = fetch_param(master, 'TECS_LAND_SINK')
    td_sink = td_sink_param if (td_sink_param is not None and td_sink_param > 0) else cfg.default_td_sink

    cruise_aspd = fetch_param(master, 'TRIM_ARSPD_CM')
    approach_speed = (cruise_aspd / 100.0) if (cruise_aspd is not None and cruise_aspd > 0) else cfg.default_approach_speed

    hdot_approach = -(approach_speed * geom.glideslope_gradient)
    hdot_td = -abs(td_sink)
    hdot_app = -abs(hdot_approach)

    # Dynamic Flare Height Calculations
    tau_accel = abs(hdot_app) / max_vert_acc
    delta_sink = max(0.1, abs(hdot_app) - abs(hdot_td))
    h_flare_accel = tau_accel * delta_sink

    sink_ratio = abs(hdot_app) / max(0.05, abs(hdot_td))
    tau_time = (flare_sec / math.log(sink_ratio)) if sink_ratio > 1.05 else flare_sec
    h_flare_time = tau_time * delta_sink

    h_flare = max(h_flare_accel, h_flare_time, flare_alt_backup)
    tau_exp = h_flare / delta_sink
    h_infty = -tau_exp * abs(hdot_td)
    actual_duration_s = (tau_exp * math.log(sink_ratio)) if sink_ratio > 1.0 else flare_sec
    flare_ground_dist = approach_speed * actual_duration_s
    flare_to_land_dist = h_flare / geom.glideslope_gradient
    initial_accel = abs(hdot_app) / tau_exp

    profile = DynamicExponentialProfile(
        h_flare=h_flare,
        h_flare_accel=h_flare_accel,
        h_flare_time=h_flare_time,
        h_flare_backup=flare_alt_backup,
        max_vert_acc=max_vert_acc,
        hdot_approach=hdot_app,
        hdot_td=hdot_td,
        tau_exp=tau_exp,
        h_infty=h_infty,
        flare_duration_s=actual_duration_s,
        flare_ground_dist_m=flare_ground_dist,
        flare_to_land_dist_m=flare_to_land_dist,
    )

    print(f' -> Max Vertical Acceleration Limit (TECS_VERT_ACC) : {profile.max_vert_acc:.2f} m/s²')
    print(f' -> Target Flare Duration           (LAND_FLARE_SEC) : {flare_sec:.2f} s')
    print(f' -> Flare Altitude Backup Floor     (LAND_FLARE_ALT) : {profile.h_flare_backup:.2f} m')
    print(f' -> Nominal Approach Sink Rate      (hdot_approach)  : {profile.hdot_approach:.2f} m/s')
    print(f' -> Target Touchdown Sink Rate      (TECS_LAND_SINK) : {profile.hdot_td:.2f} m/s')
    print(f' -------------------------------------------------------------')
    print(f' -> Sized for Accel Bound : {profile.h_flare_accel:.2f} m AGL')
    print(f' -> Sized for Flare Sec   : {profile.h_flare_time:.2f} m AGL')
    print(f' => DECIDED FLARE HEIGHT  : {profile.h_flare:.2f} m AGL (Engage Point {profile.flare_to_land_dist_m:.1f}m ahead of land WP)')
    print(f' -> Trajectory Time Const : {profile.tau_exp:.2f} s (Initial az: {initial_accel:.2f} m/s² <= {profile.max_vert_acc:.2f} m/s²)')
    print(f' -> Expected Flare Run    : {profile.flare_ground_dist_m:.1f} m over {profile.flare_duration_s:.2f} s')
    return profile


def cross_track_m(lat_deg: float, lon_deg: float, track_rad: float, ref_lat_deg: float, ref_lon_deg: float) -> float:
    ref_lat = math.radians(ref_lat_deg)
    dn = (lat_deg - ref_lat_deg) * (math.pi / 180.0) * R_EARTH
    de = (lon_deg - ref_lon_deg) * (math.pi / 180.0) * R_EARTH * math.cos(ref_lat)
    tn, te = math.cos(track_rad), math.sin(track_rad)
    return tn * de - te * dn


def compute_live_exponential_trajectory(t_elapsed: float, h_0: float, hdot_0: float,
                                       hdot_td: float, max_vert_acc: float) -> Tuple[float, float, float]:
    abs_hdot_0 = max(abs(hdot_td) + 0.1, abs(hdot_0))
    abs_hdot_td = abs(hdot_td)

    tau_accel = abs_hdot_0 / max_vert_acc
    tau_geom = h_0 / (abs_hdot_0 - abs_hdot_td)
    tau = max(tau_accel, tau_geom)

    h_infty = -tau * abs_hdot_td
    decay = math.exp(-t_elapsed / tau)
    h_ref = (h_0 - h_infty) * decay + h_infty
    hdot_ref = -abs_hdot_0 * decay
    accel_ref = (abs_hdot_0 / tau) * decay

    return max(0.0, h_ref), hdot_ref, accel_ref


def get_unique_filename(base_path: str = "passive_test.csv") -> str:
    """Generate a unique filename by appending an incrementing index if the file exists."""
    if not os.path.exists(base_path):
        return base_path
    root, ext = os.path.splitext(base_path)
    counter = 1
    while True:
        candidate = f"{root}_{counter}{ext}"
        if not os.path.exists(candidate):
            return candidate
        counter += 1


def main():
    parser = argparse.ArgumentParser(description="Passive shadow monitor for exponential flare controller.")
    parser.add_argument("--connect", default=DEVICE, help=f"MAVLink connection string (e.g. {DEVICE}, tcp:127.0.0.1:5762, default: {DEVICE})")
    parser.add_argument("--baud", type=int, default=BAUD, help=f"Serial baud rate (if serial port, default: {BAUD})")
    parser.add_argument("--go-around-thr", type=int, default=1800, help="Pilot throttle PWM threshold for go-around (default 1800 us)")
    parser.add_argument("--throttle-ch", type=int, default=3, help="Throttle RC channel (default 3)")
    parser.add_argument("--log-csv", default=DEFAULT_CSV_LOG, help=f"CSV output file path (default: {DEFAULT_CSV_LOG}, auto-increments if file exists)")
    args = parser.parse_args()

    cfg = Config(
        connection=args.connect,
        baud=args.baud,
        go_around_thr_pwm=args.go_around_thr,
        throttle_channel=args.throttle_ch
    )

    print("=" * 72)
    print("   Passive Shadow Monitor: Exponential Flare & Go-Around Detector  ")
    print("   [STRICTLY READ-ONLY: ZERO ACTUATION OR CONTROL COMMANDS SENT]   ")
    print("=" * 72)
    print(f"Connecting to {cfg.connection} (baud={cfg.baud})...")

    master = mavutil.mavlink_connection(
        cfg.connection,
        baud=cfg.baud,
        source_system=1,
        source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER
    )
    master.wait_heartbeat()
    print(f"Heartbeat received from system #{master.target_system}")

    # Extract mission and profile
    geom = extract_mission_geometry(master)
    profile = build_dynamic_exponential_profile(master, geom, cfg)

    # Initialize telemetry
    telem = Telemetry(master, throttle_channel=cfg.throttle_channel)
    telem.request_streams()

    # CSV Logger Setup
    csv_file = None
    csv_writer = None
    actual_csv_path = None
    if args.log_csv:
        actual_csv_path = get_unique_filename(args.log_csv)
        csv_file = open(actual_csv_path, mode='w', newline='')
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow([
            'timestamp', 'state', 'mode', 'mission_seq',
            'lat', 'lon', 'agl_m', 'vz_mps',
            'roll_deg', 'pitch_deg', 'heading_deg',
            'groundspeed_mps', 'airspeed_mps', 'thr_pwm',
            'xtk_m', 'target_course_deg',
            'h_ref_m', 'hdot_ref_mps', 'az_ref_mps2',
            'alt_err_m', 'sink_err_mps'
        ])
        csv_file.flush()
        print(f"\n[i] Telemetry logging enabled -> {actual_csv_path}")

    state = 'WAIT_APPROACH'
    flare_start_time = None
    flare_h0 = None
    flare_hdot0 = None
    stopped_start_time = None
    last_print_time = 0.0
    last_countdown_print = 0.0
    last_mode = None

    print(f'\n' + '=' * 72)
    print(f'PASSIVE SHADOW MONITOR ACTIVE')
    print(f'Waiting for final approach (AUTO mode on seq #{geom.land_seq}, AGL <= {profile.h_flare:.1f} m)...')
    print('=' * 72)

    try:
        while True:
            telem.pump()
            now = time.time()
            agl = telem.agl()
            gpi = telem.get('GLOBAL_POSITION_INT')
            mode_num = telem.mode()
            mode_name = PLANE_MODES.get(mode_num, f"MODE_{mode_num}")
            thr = telem.throttle_pwm()
            mc = telem.get('MISSION_CURRENT')
            mc_seq = mc.seq if mc is not None else None
            vz = telem.vertical_velocity()
            vg = telem.groundspeed() or cfg.default_approach_speed
            ias = telem.airspeed()
            roll = telem.roll_deg()
            pitch = telem.pitch_deg()
            hdg = telem.heading_deg()
            lat_cur, lon_cur = telem.lat_lon()

            # Cross-track & lateral course calculations (All States)
            xtk = 0.0
            target_course_deg = geom.track_deg
            if lat_cur is not None and lon_cur is not None:
                xtk = cross_track_m(lat_cur, lon_cur, geom.track_rad, geom.land_lat, geom.land_lon)
                v_close = max(-cfg.v_close_max, min(-xtk / cfg.t_intercept, cfg.v_close_max))
                corr_rad = math.asin(max(-0.5, min(v_close / max(vg, 5.0), 0.5)))
                max_corr = math.radians(cfg.max_course_corr_deg)
                corr_rad = max(-max_corr, min(corr_rad, max_corr))
                target_course_deg = (math.degrees(geom.track_rad + corr_rad) + 360.0) % 360.0

            h_ref = None
            hdot_ref = None
            accel_ref = None
            alt_err = None
            sink_err = None

            # ----------------------------------------------------------------
            # 1. Mode Change Monitoring (All States)
            # ----------------------------------------------------------------
            if last_mode is not None and mode_num != last_mode:
                old_str = PLANE_MODES.get(last_mode, str(last_mode))
                print(f"\n[SHADOW DETECT] >>> Flight mode changed: {old_str} -> {mode_name}")
                if state in ('SHADOW_FLARE', 'SHADOW_ROLLOUT', 'SHADOW_STOPPED'):
                    print(f"[SHADOW DETECT] -> Flare controller would abort due to pilot mode change.")
            last_mode = mode_num

            # ----------------------------------------------------------------
            # 2. Pilot Throttle Go-Around Detection (All States)
            # ----------------------------------------------------------------
            if thr is not None and thr > cfg.go_around_thr_pwm:
                target_seq = geom.do_land_start_next_seq or geom.do_land_start_seq or geom.approach_wp_seq
                print(f"\n[SHADOW DETECT] >>> PILOT THROTTLE ADVANCE DETECTED (Throttle: {thr} us > {cfg.go_around_thr_pwm} us)")
                print(f" [PASSIVE SIMULATION]:")
                print(f"  -> Simulated set active mission item to: Seq #{target_seq}")
                print(f"  -> Simulated switch flight mode to: AUTO")
                print(f"  -> [PASSIVE: Zero commands sent to autopilot]")
                time.sleep(0.5)

            # ----------------------------------------------------------------
            # 3. State: WAIT_APPROACH
            # ----------------------------------------------------------------
            if state == 'WAIT_APPROACH':
                on_final = (mode_num == MODE_AUTO and mc_seq == geom.land_seq)
                descending = (gpi is not None and gpi.vz > 20)  # > 0.20 m/s downward

                if on_final and descending and agl is not None and agl <= profile.h_flare:
                    flare_h0 = agl
                    live_vz = vz if (vz is not None and vz < -0.2) else profile.hdot_approach
                    flare_hdot0 = live_vz
                    flare_start_time = now
                    state = 'SIM_FLARE'

                    print(f"\n[SHADOW DETECT] >>> FLARE TRIGGER CRITERIA MET at AGL {flare_h0:.2f} m (live sink: {flare_hdot0:.2f} m/s)")
                    print(f" [PASSIVE SIMULATION]:")
                    print(f"  -> Simulated switch flight mode to: GUIDED")
                    print(f"  -> Simulated set airspeed setpoint to: 0.0 m/s (Idle Throttle)")
                    print(f"  -> Simulated command runway track bearing: {geom.track_deg:.1f}°")
                    print(f"  -> [PASSIVE: Maintaining read-only monitoring]")
                    print("-" * 72)
                    print(f" {'t(s)':<5} | {'Live AGL':<9} | {'h_ref':<8} | {'AltErr':<7} | {'Live Vz':<9} | {'hdot_ref':<9} | {'az_ref':<8} | {'XTK':<7} | {'Thr'}")
                    print("-" * 72)

                elif (now - last_print_time) >= 1.0:
                    agl_str = f"{agl:5.1f}m" if agl is not None else " N/A "
                    vz_str = f"{vz:5.1f}m/s" if vz is not None else " N/A "
                    thr_str = f"{thr}us" if thr is not None else "N/A"
                    wp_str = f"#{mc_seq}" if mc_seq is not None else "N/A"
                    print(f"[WAIT_APPROACH] Mode: {mode_name:<8} | WP: {wp_str:<4} | AGL: {agl_str} (Flare at {profile.h_flare:.1f}m) | "
                          f"Vz: {vz_str} | Thr: {thr_str} | GS: {vg:.1f}m/s")
                    last_print_time = now

            # ----------------------------------------------------------------
            # 4. State: SHADOW_FLARE
            # ----------------------------------------------------------------
            elif state == 'SHADOW_FLARE':
                t_elapsed = now - flare_start_time
                h_ref, hdot_ref, accel_ref = compute_live_exponential_trajectory(
                    t_elapsed, flare_h0, flare_hdot0, profile.hdot_td, profile.max_vert_acc
                )
                alt_err = (agl - h_ref) if (agl is not None and h_ref is not None) else None
                sink_err = (vz - hdot_ref) if (vz is not None and hdot_ref is not None) else None

                # Print trajectory comparison at 1 Hz
                if (now - last_print_time) >= 1.0:
                    agl_str = f"{agl:6.2f}m" if agl is not None else "  N/A  "
                    vz_str = f"{vz:6.2f}m/s" if vz is not None else "  N/A  "
                    thr_str = f"{thr}us" if thr is not None else "N/A"
                    alt_err_str = f"{alt_err:+6.2f}m" if alt_err is not None else "  N/A  "
                    print(f" {t_elapsed:4.1f}s | {agl_str:<9} | {h_ref:6.2f}m | {alt_err_str} | {vz_str:<9} | {hdot_ref:6.2f}m/s | {accel_ref:5.2f}m/s² | {xtk:+5.1f}m | {thr_str}")
                    last_print_time = now

                # Touchdown detection
                if agl is not None and agl <= cfg.touchdown_agl:
                    print(f"\n[SHADOW DETECT] >>> Touchdown detected (AGL {agl:.2f} m <= {cfg.touchdown_agl:.2f} m) -> Entering SHADOW_ROLLOUT")
                    print(f" [PASSIVE SIMULATION]:")
                    print(f"  -> Simulated command runway heading hold ({geom.track_deg:.1f}°) and 0m altitude")
                    state = 'SHADOW_ROLLOUT'

            # ----------------------------------------------------------------
            # 5. State: SHADOW_ROLLOUT
            # ----------------------------------------------------------------
            elif state == 'SHADOW_ROLLOUT':
                if (now - last_print_time) >= 1.0:
                    print(f"[SHADOW_ROLLOUT] Groundspeed: {vg:.1f} m/s (Stop threshold: < {cfg.rollout_stop_gs:.1f} m/s)")
                    last_print_time = now

                if vg < cfg.rollout_stop_gs:
                    stopped_start_time = now
                    last_countdown_print = now
                    state = 'SHADOW_STOPPED'
                    print(f"\n[SHADOW DETECT] >>> Aircraft stopped on runway (Vg: {vg:.1f} m/s < {cfg.rollout_stop_gs:.1f} m/s)")
                    print(f" [PASSIVE SIMULATION]:")
                    print(f"  -> Starting 30-second delayed auto-disarm countdown in shadow mode.")

            # ----------------------------------------------------------------
            # 6. State: SHADOW_STOPPED
            # ----------------------------------------------------------------
            elif state == 'SHADOW_STOPPED':
                if not telem.is_armed():
                    print("\n[SHADOW DETECT] >>> Aircraft disarmed. Landing workflow complete.")
                    state = 'DONE'
                    break

                elapsed_stopped = now - stopped_start_time
                remaining = max(0.0, cfg.auto_disarm_delay_s - elapsed_stopped)

                if (now - last_countdown_print) >= 5.0 and remaining > 0:
                    print(f"[SHADOW_STOPPED] Aircraft stopped. Simulated auto-disarm in {remaining:.0f}s...")
                    last_countdown_print = now

                if elapsed_stopped >= cfg.auto_disarm_delay_s:
                    print(f"\n[SHADOW DETECT] >>> 30 seconds elapsed after stop -> [PASSIVE: WOULD COMMAND AIRCRAFT DISARM]")
                    state = 'DONE'
                    break

            # ----------------------------------------------------------------
            # CSV Telemetry Logging (All States)
            # ----------------------------------------------------------------
            if csv_writer:
                csv_writer.writerow([
                    f"{now:.3f}",
                    state,
                    mode_name,
                    mc_seq if mc_seq is not None else "",
                    f"{lat_cur:.7f}" if lat_cur is not None else "",
                    f"{lon_cur:.7f}" if lon_cur is not None else "",
                    f"{agl:.2f}" if agl is not None else "",
                    f"{vz:.2f}" if vz is not None else "",
                    f"{roll:.1f}" if roll is not None else "",
                    f"{pitch:.1f}" if pitch is not None else "",
                    f"{hdg:.1f}" if hdg is not None else "",
                    f"{vg:.2f}" if vg is not None else "",
                    f"{ias:.2f}" if ias is not None else "",
                    thr if thr is not None else "",
                    f"{xtk:.2f}" if xtk is not None else "",
                    f"{target_course_deg:.1f}" if target_course_deg is not None else "",
                    f"{h_ref:.2f}" if h_ref is not None else "",
                    f"{hdot_ref:.2f}" if hdot_ref is not None else "",
                    f"{accel_ref:.2f}" if accel_ref is not None else "",
                    f"{alt_err:.2f}" if alt_err is not None else "",
                    f"{sink_err:.2f}" if sink_err is not None else "",
                ])
                csv_file.flush()

            time.sleep(1.0 / cfg.loop_hz)

    except KeyboardInterrupt:
        print("\n[!] Passive monitor stopped by user.")
    finally:
        if csv_file:
            csv_file.flush()
            csv_file.close()
            print(f"[i] Telemetry log saved to {actual_csv_path}")

    print(f"\nPassive shadow monitor finished. Final state: {state}")


if __name__ == '__main__':
    main()

