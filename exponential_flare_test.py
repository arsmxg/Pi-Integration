"""
Smooth Acceleration-Bounded Exponential Flare Controller for ArduPlane
======================================================================
Target firmware: Stock ArduPlane 4.3+ / 4.5.6 (no custom TECS modifications).

Key Concepts:
  1. Acceleration-Bounded Trajectory:
     Extracts TECS_VERT_ACC as the strict upper bound for vertical acceleration
     demanded at flare initiation (a_z,max = |hdot_approach| / tau <= TECS_VERT_ACC).
  2. Dynamic Flare Height Decision:
     Computes the required flare initiation altitude dynamically from:
       - Approach glideslope angle (from mission NAV_WAYPOINT -> NAV_LAND leg).
       - Approach descent rate (hdot_approach = -V_g * tan(gamma)).
       - Maximum vertical acceleration constraint (TECS_VERT_ACC).
       - Desired flare duration (LAND_FLARE_SEC).
       - Backup flare altitude (LAND_FLARE_ALT) used as a minimum floor.
     On steeper approaches, the aircraft automatically flares higher up to absorb
     vertical momentum without pitch jerk or G-loading exceedances.
  3. C0, C1, and C2 Continuous Transition:
     Anchors the analytical exponential trajectory to the vehicle's live state
     (h_0, hdot_0) at the instant of GUIDED mode engagement, ensuring zero position
     step, zero velocity step, and acceleration bounded by TECS_VERT_ACC.
  4. Mission-Aware Go-Around:
     When full throttle is commanded, the script identifies MAV_CMD_DO_LAND_START
     in the loaded mission plan, sets the active mission item to DO_LAND_START,
     and transitions immediately to AUTO mode, allowing the aircraft to climb to
     the planned altitude and cleanly repeat the landing circuit without orbit loops.
  5. 30-Second Auto-Disarm Delay on Stop:
     Once the aircraft completes rollout and comes to a full stop on the runway,
     a 30-second countdown begins. If the pilot remains in GUIDED mode, the aircraft
     automatically disarms after 30 seconds. If the pilot switches flight modes,
     the auto-disarm is cancelled and the script stands down.
"""

import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple

from pymavlink import mavutil


# ----------------------------------------------------------------------------
# Configuration
# ----------------------------------------------------------------------------
@dataclass
class Config:
    connection: str = 'tcp:127.0.0.1:5762'

    # Autopilot Defaults / Fallbacks (overridden by autopilot parameters)
    default_vert_acc: float = 1.5        # Max vertical acceleration [m/s^2] (TECS_VERT_ACC)
    default_flare_sec: float = 3.0       # Target flare duration [s] (LAND_FLARE_SEC)
    default_flare_alt_backup: float = 3.0# Minimum backup flare altitude [m] (LAND_FLARE_ALT)
    default_td_sink: float = 0.30        # Target touchdown sink rate [m/s] (TECS_LAND_SINK)
    default_approach_speed: float = 15.0 # Nominal approach groundspeed [m/s]

    # Termination, Rollout & Disarm
    touchdown_agl: float = 0.30          # Altitude declaring ground contact [m]
    rollout_stop_gs: float = 1.5         # Groundspeed declaring aircraft stopped [m/s]
    auto_disarm_delay_s: float = 30.0    # Time after stopping before auto-disarm [s]
    disarm_on_stop: bool = True          # Enable 30-second delayed auto-disarm

    # Lateral Guidance
    t_intercept: float = 4.0             # Cross-track closure time constant [s]
    v_close_max: float = 3.0             # Max lateral closure velocity [m/s]
    max_course_corr_deg: float = 20.0    # Maximum course correction angle [deg]
    hdg_accel_limit: float = 2.0         # Lateral acceleration limit [m/s^2]

    # Speed Management
    flare_aspd_margin: float = 0.5       # Target airspeed = AIRSPEED_MIN + this [m/s]

    # Pilot Authority & Failsafes
    go_around_thr_pwm: int = 1900        # RC throttle PWM threshold for go-around
    throttle_channel: int = 3            # RC throttle channel

    # Rates
    cmd_hz: float = 10.0                 # Guidance command rate [Hz]
    loop_hz: float = 20.0                # Telemetry polling rate [Hz]


CFG = Config()

# Plane Custom Flight Modes
MODE_AUTO, MODE_TAKEOFF, MODE_GUIDED = 10, 13, 15
HEADING_TYPE_COG = 0
SPEED_TYPE_AIRSPEED = 0
R_EARTH = 6378137.0


# ----------------------------------------------------------------------------
# Data Structures
# ----------------------------------------------------------------------------
@dataclass
class MissionApproachGeometry:
    """Calculated geometry and waypoint references from the loaded autopilot mission."""
    track_rad: float                 # Runway centerline bearing [rad]
    track_deg: float                 # Runway centerline bearing [deg]
    approach_lat: float              # Waypoint before land [deg]
    approach_lon: float              # Waypoint before land [deg]
    approach_alt: float              # Waypoint before land altitude [m]
    approach_wp_seq: int             # Mission sequence of waypoint before land
    land_lat: float                  # NAV_LAND waypoint latitude [deg]
    land_lon: float                  # NAV_LAND waypoint longitude [deg]
    land_alt: float                  # NAV_LAND waypoint altitude [m]
    land_seq: int                    # Mission sequence index of NAV_LAND
    approach_dist_m: float           # Horizontal distance of approach leg [m]
    approach_alt_drop_m: float       # Vertical drop on approach leg [m]
    glideslope_deg: float            # Approach glide slope angle [deg]
    glideslope_gradient: float       # Glide slope gradient (tan gamma)
    do_land_start_seq: Optional[int] # Sequence index of MAV_CMD_DO_LAND_START (if present)
    do_land_start_next_seq: Optional[int] # First NAV waypoint after DO_LAND_START
    takeoff_alt_m: Optional[float]   # Planned takeoff / climbout altitude [m]


@dataclass
class DynamicExponentialProfile:
    """Analytical parameters defining the acceleration-bounded flare trajectory."""
    h_flare: float                   # Decided flare initiation altitude [m]
    h_flare_accel: float             # Min flare altitude from acceleration limit [m]
    h_flare_time: float              # Flare altitude from LAND_FLARE_SEC [m]
    h_flare_backup: float            # Backup flare altitude floor from LAND_FLARE_ALT [m]
    max_vert_acc: float              # Upper bound vertical acceleration [m/s^2] (TECS_VERT_ACC)
    hdot_approach: float             # Required approach sink rate (negative) [m/s]
    hdot_td: float                   # Target touchdown sink rate (negative) [m/s]
    tau_exp: float                   # Nominal exponential time constant [s]
    h_infty: float                   # Nominal asymptotic target depth [m]
    flare_duration_s: float          # Expected flare duration [s]
    flare_ground_dist_m: float       # Expected ground run during flare [m]
    flare_to_land_dist_m: float      # Distance from flare initiation point to NAV_LAND [m]


class Telemetry:
    """Latest-value telemetry cache populated from MAVLink streams."""

    WANTED = {
        'GLOBAL_POSITION_INT': 33,
        'ATTITUDE': 30,
        'MISSION_CURRENT': 42,
        'RC_CHANNELS': 65,
        'VFR_HUD': 74,
        'HEARTBEAT': 0,
    }

    def __init__(self, master):
        self.master = master
        self.msgs = {}
        self.stamps = {}

    def request_streams(self):
        """Request required MAVLink telemetry packets at 10 Hz."""
        for name, msgid in self.WANTED.items():
            if name == 'HEARTBEAT':
                continue
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                msgid, 1e6 / 10.0, 0, 0, 0, 0, 0)

    def pump(self):
        """Drain MAVLink receive buffer and cache latest messages."""
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
        """Get filtered EKF relative altitude above home/ground in meters."""
        gpi = self.get('GLOBAL_POSITION_INT', 1.0)
        if gpi is not None:
            return gpi.relative_alt / 1000.0
        return None

    def vertical_velocity(self) -> Optional[float]:
        """Get current vertical velocity in m/s (negative for descent)."""
        gpi = self.get('GLOBAL_POSITION_INT', 1.0)
        if gpi is not None:
            return -(gpi.vz / 100.0) # gpi.vz is cm/s, +down -> convert to +up m/s
        return None

    def groundspeed(self) -> Optional[float]:
        """Get groundspeed in m/s from VFR_HUD or velocity vectors."""
        hud = self.get('VFR_HUD', 1.0)
        if hud is not None:
            return hud.groundspeed
        gpi = self.get('GLOBAL_POSITION_INT')
        if gpi is not None:
            return math.hypot(gpi.vx, gpi.vy) / 100.0
        return None

    def mode(self) -> Optional[int]:
        """Get current custom flight mode number."""
        hb = self.get('HEARTBEAT', 3.0)
        return hb.custom_mode if hb is not None else None

    def is_armed(self) -> bool:
        """Check if vehicle is currently armed."""
        hb = self.get('HEARTBEAT', 3.0)
        if hb is None:
            return False
        return bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    def throttle_pwm(self) -> Optional[int]:
        """Get RC transmitter throttle channel PWM value."""
        rc = self.get('RC_CHANNELS', 1.0)
        if rc is None:
            return None
        return getattr(rc, f'chan{CFG.throttle_channel}_raw', None)


# ----------------------------------------------------------------------------
# Parameter & Geometry Extraction Helpers
# ----------------------------------------------------------------------------
def fetch_param(master, name: str, timeout: float = 2.0) -> Optional[float]:
    """Read a parameter from the autopilot over MAVLink."""
    master.mav.param_request_read_send(
        master.target_system, master.target_component,
        name.encode('ascii'), -1)
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
        if msg is not None and msg.param_id.strip('\x00') == name:
            return msg.param_value
    return None


def haversine_dist(lat1_deg: float, lon1_deg: float, lat2_deg: float, lon2_deg: float) -> float:
    """Calculate horizontal distance in meters between two GPS coordinates."""
    lat1, lon1 = math.radians(lat1_deg), math.radians(lon1_deg)
    lat2, lon2 = math.radians(lat2_deg), math.radians(lon2_deg)
    dlat, dlon = lat2 - lat1, lon2 - lon1
    a = math.sin(dlat / 2.0)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0)**2
    return 2.0 * R_EARTH * math.asin(math.sqrt(max(0.0, min(a, 1.0))))


def extract_mission_geometry(master) -> MissionApproachGeometry:
    """Download the autopilot mission, locate DO_LAND_START, NAV_LAND, and approach geometry."""
    print('\n[1/3] Downloading mission from autopilot...')
    master.waypoint_request_list_send()
    count_msg = master.recv_match(type='MISSION_COUNT', blocking=True, timeout=5)
    if count_msg is None:
        raise RuntimeError('Timeout requesting MISSION_COUNT from autopilot.')

    wps = []
    for i in range(count_msg.count):
        master.mav.mission_request_int_send(
            master.target_system, master.target_component, i)
        wp = master.recv_match(type='MISSION_ITEM_INT', blocking=True, timeout=5)
        if wp is None:
            raise RuntimeError(f'Timeout fetching mission item {i}')
        wps.append(wp)

    master.mav.mission_ack_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_MISSION_ACCEPTED)

    # 1. Search for DO_LAND_START item (command 189)
    do_land_start_idx = next((i for i, w in enumerate(wps)
                              if w.command == mavutil.mavlink.MAV_CMD_DO_LAND_START), None)
    do_land_start_next_idx = (do_land_start_idx + 1) if (do_land_start_idx is not None and do_land_start_idx + 1 < len(wps)) else None

    # 2. Search for NAV_TAKEOFF item (command 22) or read TKOFF_ALT param
    takeoff_wp = next((w for w in wps if w.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF), None)
    takeoff_alt = takeoff_wp.z if takeoff_wp is not None else fetch_param(master, 'TKOFF_ALT')

    # 3. Locate NAV_LAND
    land_idx = next((i for i, w in enumerate(wps)
                     if w.command == mavutil.mavlink.MAV_CMD_NAV_LAND), None)
    if land_idx is None or land_idx == 0:
        raise RuntimeError('Mission does not contain a NAV_LAND item with a preceding waypoint.')

    # 4. Locate immediate NAV_WAYPOINT before NAV_LAND
    approach_wp_idx = next((i for i in range(land_idx - 1, -1, -1)
                            if wps[i].command == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT), None)
    if approach_wp_idx is None:
        raise RuntimeError('No NAV_WAYPOINT found before NAV_LAND in the mission plan.')

    approach_wp = wps[approach_wp_idx]
    land_wp = wps[land_idx]

    lat1, lon1 = approach_wp.x / 1e7, approach_wp.y / 1e7
    lat2, lon2 = land_wp.x / 1e7, land_wp.y / 1e7
    alt1, alt2 = approach_wp.z, land_wp.z

    # Centerline bearing calculation
    r_lat1, r_lon1 = math.radians(lat1), math.radians(lon1)
    r_lat2, r_lon2 = math.radians(lat2), math.radians(lon2)
    dlon = r_lon2 - r_lon1
    x = math.sin(dlon) * math.cos(r_lat2)
    y = math.cos(r_lat1) * math.sin(r_lat2) - math.sin(r_lat1) * math.cos(r_lat2) * math.cos(dlon)
    track_rad = math.atan2(x, y)
    track_deg = (math.degrees(track_rad) + 360.0) % 360.0

    # Distance and glideslope calculation
    approach_dist = haversine_dist(lat1, lon1, lat2, lon2)
    alt_drop = alt1 - alt2
    if approach_dist < 1.0 or alt_drop <= 0:
        gradient = 0.0524  # Default 3 degree slope
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
    print(f' -> Planned Takeoff Alt    : {geom.takeoff_alt_m:.1f} m' if geom.takeoff_alt_m else ' -> Planned Takeoff Alt    : (default)')
    print(f' -> Approach Leg Distance  : {geom.approach_dist_m:.1f} m')
    print(f' -> Approach Alt Drop      : {geom.approach_alt_drop_m:.1f} m')
    print(f' -> Runway Track Bearing   : {geom.track_deg:.1f} deg')
    print(f' -> Approach Glideslope    : {geom.glideslope_deg:.2f} deg ({geom.glideslope_gradient * 100:.1f}%)')
    return geom


def build_dynamic_exponential_profile(master, geom: MissionApproachGeometry) -> DynamicExponentialProfile:
    """Extract UAV limits (TECS_VERT_ACC, LAND_FLARE_SEC) and compute dynamic flare height."""
    print('\n[2/3] Extracting autopilot constraints & calculating dynamic flare height...')

    # 1. Maximum Vertical Acceleration Limit (TECS_VERT_ACC)
    vert_acc_param = fetch_param(master, 'TECS_VERT_ACC')
    max_vert_acc = vert_acc_param if (vert_acc_param is not None and vert_acc_param > 0) else CFG.default_vert_acc

    # 2. Flare Duration Parameter (LAND_FLARE_SEC)
    flare_sec_param = fetch_param(master, 'LAND_FLARE_SEC')
    flare_sec = flare_sec_param if (flare_sec_param is not None and flare_sec_param > 0) else CFG.default_flare_sec

    # 3. Flare Altitude Floor / Backup (LAND_FLARE_ALT)
    flare_alt_backup_param = fetch_param(master, 'LAND_FLARE_ALT')
    flare_alt_backup = flare_alt_backup_param if (flare_alt_backup_param is not None and flare_alt_backup_param > 0) else CFG.default_flare_alt_backup

    # 4. Target Touchdown Sink Rate (TECS_LAND_SINK)
    td_sink_param = fetch_param(master, 'TECS_LAND_SINK')
    td_sink = td_sink_param if (td_sink_param is not None and td_sink_param > 0) else CFG.default_td_sink

    # 5. Approach Speed
    cruise_aspd = fetch_param(master, 'TRIM_ARSPD_CM')
    if cruise_aspd is not None and cruise_aspd > 0:
        approach_speed = cruise_aspd / 100.0
    else:
        approach_speed = CFG.default_approach_speed

    # Required steady-state approach sink rate from mission glideslope:
    hdot_approach = -(approach_speed * geom.glideslope_gradient)
    hdot_td = -abs(td_sink)
    hdot_app = -abs(hdot_approach)

    # ------------------------------------------------------------------------
    # Dynamic Flare Height Calculations:
    # ------------------------------------------------------------------------
    # Constraint A: Acceleration Bound (a_z(0) = |hdot_app| / tau <= max_vert_acc)
    tau_accel = abs(hdot_app) / max_vert_acc
    delta_sink = max(0.1, abs(hdot_app) - abs(hdot_td))
    h_flare_accel = tau_accel * delta_sink

    # Constraint B: Flare Time Parameter (LAND_FLARE_SEC)
    sink_ratio = abs(hdot_app) / max(0.05, abs(hdot_td))
    if sink_ratio > 1.05:
        tau_time = flare_sec / math.log(sink_ratio)
    else:
        tau_time = flare_sec
    h_flare_time = tau_time * delta_sink

    # Constraint C: Minimum Altitude Backup Floor
    h_flare_backup_floor = flare_alt_backup

    # Final Decided Flare Initiation Height:
    # On steep descents, h_flare_accel scales up to prevent pitch acceleration spikes.
    h_flare = max(h_flare_accel, h_flare_time, h_flare_backup_floor)

    # Resulting Nominal Time Constant and Trajectory Geometry:
    tau_exp = h_flare / delta_sink
    h_infty = -tau_exp * abs(hdot_td)
    actual_duration_s = tau_exp * math.log(sink_ratio) if sink_ratio > 1.0 else flare_sec
    flare_ground_dist = approach_speed * actual_duration_s
    flare_to_land_dist = h_flare / geom.glideslope_gradient

    # Peak demanded acceleration at entry
    initial_accel = abs(hdot_app) / tau_exp

    profile = DynamicExponentialProfile(
        h_flare=h_flare,
        h_flare_accel=h_flare_accel,
        h_flare_time=h_flare_time,
        h_flare_backup=h_flare_backup_floor,
        max_vert_acc=max_vert_acc,
        hdot_approach=hdot_app,
        hdot_td=hdot_td,
        tau_exp=tau_exp,
        h_infty=h_infty,
        flare_duration_s=actual_duration_s,
        flare_ground_dist_m=flare_ground_dist,
        flare_to_land_dist_m=flare_to_land_dist,
    )

    print(f' -> Max Vertical Acceleration Limit (TECS_VERT_ACC) : {profile.max_vert_acc:.2f} m/s^2')
    print(f' -> Flare Time Parameter Target     (LAND_FLARE_SEC) : {flare_sec:.2f} s')
    print(f' -> Flare Backup Floor Altitude     (LAND_FLARE_ALT) : {profile.h_flare_backup:.2f} m')
    print(f' -> Mission Required Approach Sink  (hdot_approach)  : {profile.hdot_approach:.2f} m/s')
    print(f' -> Target Touchdown Sink Rate      (TECS_LAND_SINK) : {profile.hdot_td:.2f} m/s')
    print(f' -------------------------------------------------------------')
    print(f' -> Sized for Accel Limit : {profile.h_flare_accel:.2f} m AGL')
    print(f' -> Sized for Flare Sec   : {profile.h_flare_time:.2f} m AGL')
    print(f' -> DECIDED FLARE HEIGHT  : {profile.h_flare:.2f} m AGL (Engage Point {profile.flare_to_land_dist_m:.1f}m ahead of land point)')
    print(f' -> Resulting tau_exp     : {profile.tau_exp:.2f} s (Initial Accel: {initial_accel:.2f} m/s^2 <= {profile.max_vert_acc:.2f} m/s^2)')
    print(f' -> Predicted Flare Run   : {profile.flare_ground_dist_m:.1f} m over {profile.flare_duration_s:.2f} s')
    return profile


# ----------------------------------------------------------------------------
# Trajectory Computation & MAVLink Actuation
# ----------------------------------------------------------------------------
def cross_track_m(lat_deg: float, lon_deg: float, track_rad: float,
                  ref_lat_deg: float, ref_lon_deg: float) -> float:
    """Signed cross-track distance in meters from runway centerline (+ = right)."""
    ref_lat = math.radians(ref_lat_deg)
    dn = (lat_deg - ref_lat_deg) * (math.pi / 180.0) * R_EARTH
    de = (lon_deg - ref_lon_deg) * (math.pi / 180.0) * R_EARTH * math.cos(ref_lat)
    tn, te = math.cos(track_rad), math.sin(track_rad)
    return tn * de - te * dn


def compute_live_exponential_trajectory(t_elapsed: float, h_0: float, hdot_0: float,
                                       hdot_td: float, max_vert_acc: float) -> Tuple[float, float, float]:
    """Compute continuous exponential flare trajectory anchored to live initial conditions (h_0, hdot_0).

    Guarantees:
      1. h_ref(0) = h_0        (zero position jump)
      2. hdot_ref(0) = hdot_0  (zero velocity jump)
      3. a_z(0) <= max_vert_acc (acceleration bounded by TECS_VERT_ACC)

    Returns:
      (h_ref, hdot_ref, a_z)
    """
    abs_hdot_0 = max(abs(hdot_td) + 0.1, abs(hdot_0))
    abs_hdot_td = abs(hdot_td)

    # Ensure time constant satisfies acceleration bound
    tau_accel = abs_hdot_0 / max_vert_acc
    tau_geom = h_0 / (abs_hdot_0 - abs_hdot_td)
    tau = max(tau_accel, tau_geom)

    # Asymptotic depth
    h_infty = -tau * abs_hdot_td

    decay = math.exp(-t_elapsed / tau)
    h_ref = (h_0 - h_infty) * decay + h_infty
    hdot_ref = -abs_hdot_0 * decay
    accel_ref = (abs_hdot_0 / tau) * decay

    return max(0.0, h_ref), hdot_ref, accel_ref


def send_target_altitude(master, alt_target_m: float):
    """Command TECS target altitude in GUIDED mode without slew resets.

    Setting param3 = 0 updates guided_state.target_alt directly, allowing TECS's
    internal energy balance and pitch damping loops to smoothly track the trajectory.
    """
    master.mav.command_int_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_ALTITUDE,
        0, 0,
        0, 0, 0, 0,  # p3 = 0 (direct target tracking without slew resets)
        0, 0, alt_target_m
    )


def send_course(master, course_deg: float):
    """Command Course-Over-Ground (COG) ground-track heading in GUIDED mode."""
    master.mav.command_int_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL,
        mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_HEADING,
        0, 0,
        HEADING_TYPE_COG,
        course_deg % 360.0,
        CFG.hdg_accel_limit,
        0, 0, 0, 0
    )


def send_airspeed(master, aspd_mps: float):
    """Command equivalent airspeed target in GUIDED mode."""
    master.mav.command_int_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL,
        mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_SPEED,
        0, 0,
        SPEED_TYPE_AIRSPEED,
        aspd_mps,
        0, 0, 0, 0, 0
    )


def set_mode(master, mode: int):
    """Change autopilot flight mode."""
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode,
        0, 0, 0, 0, 0
    )


def set_mission_current(master, seq: int):
    """Set active mission item index over MAVLink."""
    master.mav.mission_set_current_send(
        master.target_system, master.target_component, seq
    )


def execute_go_around(master, geom: MissionApproachGeometry, thr_pwm: int):
    """Execute a clean mission-aware go-around to DO_LAND_START in AUTO mode."""
    if geom.do_land_start_seq is not None:
        target_seq = geom.do_land_start_seq
        print(f'\n[!] GO-AROUND (throttle {thr_pwm} us): Setting mission current to DO_LAND_START (seq #{target_seq}) -> Switching to AUTO mode')
        set_mission_current(master, target_seq)
        set_mode(master, MODE_AUTO)
    elif geom.approach_wp_seq is not None:
        target_seq = geom.approach_wp_seq
        print(f'\n[!] GO-AROUND (throttle {thr_pwm} us): Setting mission current to approach WP (seq #{target_seq}) -> Switching to AUTO mode')
        set_mission_current(master, target_seq)
        set_mode(master, MODE_AUTO)
    else:
        print(f'\n[!] GO-AROUND (throttle {thr_pwm} us): No DO_LAND_START in mission -> Switching to AUTO mode')
        set_mode(master, MODE_AUTO)


# ----------------------------------------------------------------------------
# Main Execution Loop
# ----------------------------------------------------------------------------
def main():
    print('====================================================================')
    print('   ArduPlane Acceleration-Bounded Exponential Flare Controller      ')
    print('====================================================================')

    print(f'Connecting to {CFG.connection}...')
    master = mavutil.mavlink_connection(
        CFG.connection, source_system=1,
        source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER)
    master.wait_heartbeat()
    print(f'Heartbeat received from system {master.target_system}')

    # Extract mission geometry and analytical flare profile
    geom = extract_mission_geometry(master)
    profile = build_dynamic_exponential_profile(master, geom)

    # Initialize telemetry
    telem = Telemetry(master)
    telem.request_streams()

    # Query minimum stall airspeed for throttle management
    aspd_min = fetch_param(master, 'AIRSPEED_MIN')
    if aspd_min is None:
        aspd_min = fetch_param(master, 'ARSPD_FBW_MIN')
    flare_aspd = (aspd_min + CFG.flare_aspd_margin) if aspd_min else None
    print(f'\n[3/3] Flare Airspeed Setpoint: {flare_aspd:.1f} m/s' if flare_aspd else 'Flare Airspeed: (default)')

    state = 'WAIT_APPROACH'
    flare_start_time = None
    flare_h0 = None
    flare_hdot0 = None
    stopped_start_time = None
    last_cmd_time = 0.0
    last_print_time = 0.0
    last_countdown_print = 0.0
    non_guided_since = None

    print(f'\nWaiting for final approach (AUTO mode on mission seq #{geom.land_seq}, AGL <= {profile.h_flare:.1f} m)...')

    while state not in ('DONE', 'GO_AROUND', 'ABORT'):
        telem.pump()
        now = time.time()
        agl = telem.agl()
        gpi = telem.get('GLOBAL_POSITION_INT')
        mode = telem.mode()

        # --------------------------------------------------------------------
        # Pilot Authority: Go-Around & Mode Override Check
        # --------------------------------------------------------------------
        if state in ('FLARE', 'ROLLOUT', 'STOPPED'):
            thr = telem.throttle_pwm()
            if thr is not None and thr > CFG.go_around_thr_pwm:
                execute_go_around(master, geom, thr)
                state = 'GO_AROUND'
                break

            # If pilot manually changes flight mode out of GUIDED: Stand down immediately
            if mode is not None and mode != MODE_GUIDED:
                if non_guided_since is None:
                    non_guided_since = now
                elif now - non_guided_since >= 1.0:
                    print(f'\n[!] External mode switch detected ({mode}). Auto-control stood down.')
                    state = 'ABORT'
                    break
            else:
                non_guided_since = None

        # --------------------------------------------------------------------
        # State: WAIT_APPROACH
        # --------------------------------------------------------------------
        if state == 'WAIT_APPROACH':
            mc = telem.get('MISSION_CURRENT')
            on_final = (mode == MODE_AUTO and mc is not None and mc.seq == geom.land_seq)
            descending = (gpi is not None and gpi.vz > 20)  # > 0.20 m/s downward

            if on_final and descending and agl is not None and agl <= profile.h_flare:
                # Capture live vehicle state at engagement instant
                flare_h0 = agl
                live_vz = telem.vertical_velocity()
                flare_hdot0 = live_vz if (live_vz is not None and live_vz < -0.2) else profile.hdot_approach

                print(f'\n[>>>] FLARE ENGAGED at AGL {flare_h0:.2f} m (live sink: {flare_hdot0:.2f} m/s) -> GUIDED')
                set_mode(master, MODE_GUIDED)

                # Wait for GUIDED confirmation
                t0 = time.time()
                while telem.mode() != MODE_GUIDED and time.time() - t0 < 3.0:
                    telem.pump()
                    time.sleep(0.02)

                if telem.mode() != MODE_GUIDED:
                    print('Failed to confirm GUIDED mode. Aborting.')
                    state = 'ABORT'
                    break

                if flare_aspd is not None:
                    send_airspeed(master, flare_aspd)

                # Seed centerline heading immediately
                send_course(master, geom.track_deg)

                flare_start_time = time.time()
                state = 'FLARE'
                print('Smooth acceleration-bounded exponential flare active.')

        # --------------------------------------------------------------------
        # State: FLARE (Acceleration-Bounded Trajectory Tracking)
        # --------------------------------------------------------------------
        elif state == 'FLARE':
            if (now - last_cmd_time) >= (1.0 / CFG.cmd_hz) and gpi is not None and agl is not None:
                t_elapsed = now - flare_start_time
                h_ref, hdot_ref, accel_ref = compute_live_exponential_trajectory(
                    t_elapsed, flare_h0, flare_hdot0, profile.hdot_td, profile.max_vert_acc
                )
                vg = telem.groundspeed() or CFG.default_approach_speed

                # 1. Vertical Guidance: Stream smooth reference altitude
                send_target_altitude(master, h_ref)

                # 2. Lateral Guidance: Cross-track correction to runway centerline
                lat_cur = gpi.lat / 1e7
                lon_cur = gpi.lon / 1e7
                xtk = cross_track_m(lat_cur, lon_cur, geom.track_rad, geom.land_lat, geom.land_lon)

                v_close = max(-CFG.v_close_max, min(-xtk / CFG.t_intercept, CFG.v_close_max))
                corr_rad = math.asin(max(-0.5, min(v_close / max(vg, 5.0), 0.5)))
                max_corr = math.radians(CFG.max_course_corr_deg)
                corr_rad = max(-max_corr, min(corr_rad, max_corr))

                target_course_deg = (math.degrees(geom.track_rad + corr_rad) + 360.0) % 360.0
                send_course(master, target_course_deg)

                last_cmd_time = now

                # Telemetry printout (1 Hz)
                if (now - last_print_time) >= 1.0:
                    print(f't: {t_elapsed:4.1f}s | AGL: {agl:5.2f}m | h_ref: {h_ref:5.2f}m | '
                          f'hdot_ref: {hdot_ref:5.2f}m/s | az: {accel_ref:4.2f}m/s^2 | Vg: {vg:4.1f}m/s | XTK: {xtk:+5.1f}m')
                    last_print_time = now

            # Touchdown detection
            if agl is not None and agl <= CFG.touchdown_agl:
                print(f'\nTouchdown detected (AGL {agl:.2f} m) -> Entering ROLLOUT')
                state = 'ROLLOUT'

        # --------------------------------------------------------------------
        # State: ROLLOUT (Holding Ground Track until Stopped)
        # --------------------------------------------------------------------
        elif state == 'ROLLOUT':
            if (now - last_cmd_time) >= (1.0 / CFG.cmd_hz):
                # Hold down on runway (target altitude 0.0m) and maintain centerline
                send_target_altitude(master, 0.0)
                send_course(master, geom.track_deg)
                last_cmd_time = now

            gs = telem.groundspeed()
            if gs is not None and gs < CFG.rollout_stop_gs:
                stopped_start_time = now
                last_countdown_print = now
                state = 'STOPPED'
                print(f'\n[i] Aircraft stopped on runway (Vg: {gs:.1f} m/s).')
                if CFG.disarm_on_stop:
                    print(f'[i] 30-second auto-disarm timer started. (Switching out of GUIDED mode cancels auto-disarm).')
                else:
                    state = 'DONE'

        # --------------------------------------------------------------------
        # State: STOPPED (30-Second Auto-Disarm Countdown in GUIDED Mode)
        # --------------------------------------------------------------------
        elif state == 'STOPPED':
            if (now - last_cmd_time) >= (1.0 / CFG.cmd_hz):
                # Keep holding 0.0m altitude demand and runway heading
                send_target_altitude(master, 0.0)
                send_course(master, geom.track_deg)
                last_cmd_time = now

            # Check if vehicle was already disarmed by pilot or autopilot
            if not telem.is_armed():
                print('\n[i] Vehicle disarmed. Landing workflow complete.')
                state = 'DONE'
                break

            elapsed_stopped = now - stopped_start_time
            remaining = max(0.0, CFG.auto_disarm_delay_s - elapsed_stopped)

            # Print countdown status every 5 seconds
            if (now - last_countdown_print) >= 5.0 and remaining > 0:
                print(f'[i] Aircraft stopped in GUIDED mode. Auto-disarming in {remaining:.0f}s...')
                last_countdown_print = now

            # 30-second timer elapsed
            if elapsed_stopped >= CFG.auto_disarm_delay_s:
                if CFG.disarm_on_stop and mode == MODE_GUIDED:
                    print(f'\n[!] 30 seconds elapsed after stop in GUIDED mode -> Disarming aircraft.')
                    master.mav.command_long_send(
                        master.target_system, master.target_component,
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                        0, 0, 0, 0, 0, 0, 0)
                    # Fallback force disarm if normal disarm is rejected on ground
                    time.sleep(0.5)
                    telem.pump()
                    if telem.is_armed():
                        master.mav.command_long_send(
                            master.target_system, master.target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                            0, 21196, 0, 0, 0, 0, 0)
                state = 'DONE'

        time.sleep(1.0 / CFG.loop_hz)

    print(f'State machine exited. Final state: {state}')


if __name__ == '__main__':
    main()
