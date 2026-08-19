"""
Exponential Flare Trajectory Generator for ArduPlane SITL / Companion Computer
==============================================================================
Target firmware: Stock ArduPlane 4.3+ / 4.5.6 (no custom TECS firmware mods).

Description:
  Implements a pure exponential flare trajectory (no variable-tau law) by
  extracting key parameters directly from the UAV autopilot and the loaded
  mission plan:
    1. Mission Approach Glide Slope: Derived from the NAV_WAYPOINT immediately
       preceding the NAV_LAND command and the NAV_LAND waypoint itself.
    2. Approach Sink Rate: Required steady-state descent rate on final approach
       based on approach speed and glide path angle.
    3. Final Touchdown Sink Rate: Extracted from autopilot parameters (TECS_LAND_SINK).
    4. Flare Initiation Altitude & Flare Distance: Extracted from LAND_FLARE_ALT /
       geometry, determining the distance from the flare point to the runway landing point.
    5. Analytical Exponential Profile:
         tau_exp  = h_flare / (|hdot_approach| - |hdot_touchdown|)
         h_infty  = -tau_exp * |hdot_touchdown|
         h_ref(t) = (h_flare - h_infty) * exp(-t / tau_exp) + h_infty
         hdot(t)  = -|hdot_approach| * exp(-t / tau_exp)

  Unlike variable-rate slew resets that induce pitch oscillations, this script
  generates a continuous, smooth reference altitude trajectory h_ref(t) that
  TECS tracks with optimal pitch damping and energy management.
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

    # Fallback / Default Parameters (overridden by autopilot params if available)
    default_flare_alt: float = 6.0       # Flare initiation AGL [m]
    default_td_sink: float = 0.30        # Target touchdown sink rate [m/s]
    default_approach_speed: float = 15.0 # Nominal approach groundspeed [m/s]

    # Termination & Rollout
    touchdown_agl: float = 0.30          # Altitude declaring ground contact [m]
    rollout_stop_gs: float = 2.0         # Groundspeed declaring rollout complete [m/s]
    disarm_on_stop: bool = True          # Force-disarm after rollout (SITL)

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
# Mission & Parameter Extraction
# ----------------------------------------------------------------------------
@dataclass
class MissionApproachGeometry:
    """Calculated geometry from the loaded autopilot mission."""
    track_rad: float                 # Runway centerline bearing [rad]
    track_deg: float                 # Runway centerline bearing [deg]
    approach_lat: float              # Waypoint before land [deg]
    approach_lon: float              # Waypoint before land [deg]
    approach_alt: float              # Waypoint before land altitude [m]
    land_lat: float                  # NAV_LAND waypoint latitude [deg]
    land_lon: float                  # NAV_LAND waypoint longitude [deg]
    land_alt: float                  # NAV_LAND waypoint altitude [m]
    land_seq: int                    # Mission sequence index of NAV_LAND
    approach_dist_m: float           # Horizontal distance of approach leg [m]
    approach_alt_drop_m: float       # Vertical drop on approach leg [m]
    glideslope_deg: float            # Approach glide slope angle [deg]
    glideslope_gradient: float       # Glide slope gradient (tan gamma)


@dataclass
class ExponentialFlareProfile:
    """Analytical parameters defining the exponential flare trajectory."""
    h_flare: float                   # Initiation altitude [m]
    hdot_approach: float             # Initial sink rate at flare entry (negative) [m/s]
    hdot_td: float                   # Target touchdown sink rate (negative) [m/s]
    tau_exp: float                   # Exponential decay time constant [s]
    h_infty: float                   # Asymptotic target altitude below ground [m]
    flare_duration_s: float          # Expected flare duration to reach ground [s]
    flare_ground_distance_m: float   # Expected horizontal distance during flare [m]
    flare_point_to_land_dist_m: float# Distance from flare initiation to NAV_LAND point [m]


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
    """Download the autopilot mission and extract approach glideslope and centerline geometry."""
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

    # Locate NAV_LAND
    land_idx = next((i for i, w in enumerate(wps)
                     if w.command == mavutil.mavlink.MAV_CMD_NAV_LAND), None)
    if land_idx is None or land_idx == 0:
        raise RuntimeError('Mission does not contain a NAV_LAND item with a preceding waypoint.')

    # Locate immediate NAV_WAYPOINT before NAV_LAND
    approach_wp = next((wps[i] for i in range(land_idx - 1, -1, -1)
                        if wps[i].command == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT), None)
    if approach_wp is None:
        raise RuntimeError('No NAV_WAYPOINT found before NAV_LAND in the mission plan.')

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
        land_lat=lat2,
        land_lon=lon2,
        land_alt=alt2,
        land_seq=land_idx,
        approach_dist_m=approach_dist,
        approach_alt_drop_m=alt_drop,
        glideslope_deg=gs_deg,
        glideslope_gradient=gradient,
    )

    print(f' -> Mission Land Seq    : #{geom.land_seq}')
    print(f' -> Approach Leg Dist   : {geom.approach_dist_m:.1f} m')
    print(f' -> Approach Alt Drop   : {geom.approach_alt_drop_m:.1f} m')
    print(f' -> Runway Track Bearing: {geom.track_deg:.1f} deg')
    print(f' -> Approach Glideslope : {geom.glideslope_deg:.2f} deg ({geom.glideslope_gradient * 100:.1f}%)')
    return geom


def build_exponential_flare_profile(master, geom: MissionApproachGeometry) -> ExponentialFlareProfile:
    """Query autopilot parameters and construct the exponential flare trajectory."""
    print('\n[2/3] Extracting autopilot landing parameters...')

    # 1. Touchdown Sink Rate Target
    td_sink_param = fetch_param(master, 'TECS_LAND_SINK')
    td_sink = td_sink_param if (td_sink_param is not None and td_sink_param > 0) else CFG.default_td_sink

    # 2. Flare Initiation Altitude
    flare_alt_param = fetch_param(master, 'LAND_FLARE_ALT')
    if flare_alt_param is not None and flare_alt_param > 0:
        flare_alt = flare_alt_param
    else:
        flare_alt = CFG.default_flare_alt

    # 3. Approach Groundspeed / Airspeed
    cruise_aspd = fetch_param(master, 'TRIM_ARSPD_CM')
    if cruise_aspd is not None and cruise_aspd > 0:
        approach_speed = cruise_aspd / 100.0
    else:
        approach_speed = CFG.default_approach_speed

    # Calculate required approach sink rate from mission glideslope:
    # hdot_approach = -V_ground * tan(gamma)
    hdot_approach = -(approach_speed * geom.glideslope_gradient)

    # Negative sink rates
    hdot_td = -abs(td_sink)
    hdot_app = -abs(hdot_approach)

    # Exponential time constant:
    # tau_exp = h_flare / (|hdot_approach| - |hdot_td|)
    delta_sink = abs(hdot_app) - abs(hdot_td)
    if delta_sink <= 0.1:
        delta_sink = 0.5
    tau_exp = flare_alt / delta_sink

    # Asymptotic target below ground level:
    h_infty = -tau_exp * abs(hdot_td)

    # Predicted duration and distances
    duration_s = tau_exp * math.log(abs(hdot_app) / abs(hdot_td))
    flare_ground_dist = approach_speed * duration_s

    # Distance from flare initiation point on glideslope to the NAV_LAND waypoint:
    # dist_to_land = h_flare / tan(gamma)
    flare_to_land_dist = flare_alt / geom.glideslope_gradient

    profile = ExponentialFlareProfile(
        h_flare=flare_alt,
        hdot_approach=hdot_app,
        hdot_td=hdot_td,
        tau_exp=tau_exp,
        h_infty=h_infty,
        flare_duration_s=duration_s,
        flare_ground_distance_m=flare_ground_dist,
        flare_point_to_land_dist_m=flare_to_land_dist,
    )

    print(f' -> Flare Engage Altitude     (h_flare)        : {profile.h_flare:.2f} m')
    print(f' -> Required Approach Sink    (hdot_approach)  : {profile.hdot_approach:.2f} m/s')
    print(f' -> Target Touchdown Sink     (TECS_LAND_SINK) : {profile.hdot_td:.2f} m/s')
    print(f' -> Flare Point to Land Point (Distance)       : {profile.flare_point_to_land_dist_m:.1f} m')
    print(f' -> Exponential Time Constant (tau_exp)        : {profile.tau_exp:.2f} s')
    print(f' -> Asymptotic Target Depth   (h_infty)        : {profile.h_infty:.2f} m')
    print(f' -> Predicted Flare Duration  (T_flare)        : {profile.flare_duration_s:.2f} s')
    print(f' -> Predicted Flare Ground Run(D_flare)        : {profile.flare_ground_distance_m:.1f} m')
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


def compute_exponential_trajectory(t_elapsed: float, profile: ExponentialFlareProfile) -> Tuple[float, float]:
    """Compute analytical exponential flare reference altitude and sink rate at time t."""
    decay = math.exp(-t_elapsed / profile.tau_exp)
    # h_ref(t) = (h_flare - h_infty) * exp(-t/tau) + h_infty
    h_ref = (profile.h_flare - profile.h_infty) * decay + profile.h_infty
    # hdot_ref(t) = hdot_approach * exp(-t/tau)
    hdot_ref = profile.hdot_approach * decay
    return max(0.0, h_ref), hdot_ref


def send_target_altitude(master, alt_target_m: float):
    """Command TECS target altitude in GUIDED mode without slew resets.

    Setting param3 = 0 updates guided_state.target_alt cleanly, allowing TECS's
    internal controller to track the smooth continuous reference altitude.
    """
    master.mav.command_int_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_ALTITUDE,
        0, 0,
        0, 0, 0, 0,  # p3 = 0 (direct target tracking without slew reset)
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


# ----------------------------------------------------------------------------
# Main Execution Loop
# ----------------------------------------------------------------------------
def main():
    print('====================================================================')
    print('         ArduPlane Exponential Flare Trajectory Controller          ')
    print('====================================================================')

    print(f'Connecting to {CFG.connection}...')
    master = mavutil.mavlink_connection(
        CFG.connection, source_system=1,
        source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER)
    master.wait_heartbeat()
    print(f'Heartbeat received from system {master.target_system}')

    # Extract mission geometry and analytical flare profile
    geom = extract_mission_geometry(master)
    profile = build_exponential_flare_profile(master, geom)

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
    last_cmd_time = 0.0
    last_print_time = 0.0
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
        if state in ('FLARE', 'ROLLOUT'):
            thr = telem.throttle_pwm()
            if thr is not None and thr > CFG.go_around_thr_pwm:
                print(f'\n[!] GO-AROUND: Throttle {thr} us > {CFG.go_around_thr_pwm} us -> Switching to TAKEOFF')
                set_mode(master, MODE_TAKEOFF)
                state = 'GO_AROUND'
                break

            # If pilot manually changes flight mode out of GUIDED: Stand down
            if mode is not None and mode != MODE_GUIDED:
                if non_guided_since is None:
                    non_guided_since = now
                elif now - non_guided_since >= 1.5:
                    print(f'\n[!] External mode switch detected ({mode}). Standing down.')
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
                print(f'\n[>>>] FLARE ENGAGED at AGL {agl:.2f} m -> Switching to GUIDED')
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
                print('Exponential flare trajectory active.')

        # --------------------------------------------------------------------
        # State: FLARE (Exponential Reference Trajectory Tracking)
        # --------------------------------------------------------------------
        elif state == 'FLARE':
            if (now - last_cmd_time) >= (1.0 / CFG.cmd_hz) and gpi is not None and agl is not None:
                t_elapsed = now - flare_start_time
                h_ref, hdot_ref = compute_exponential_trajectory(t_elapsed, profile)
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
                          f'hdot_ref: {hdot_ref:5.2f}m/s | Vg: {vg:4.1f}m/s | XTK: {xtk:+5.1f}m')
                    last_print_time = now

            # Touchdown detection
            if agl is not None and agl <= CFG.touchdown_agl:
                print(f'\nTouchdown detected (AGL {agl:.2f} m) -> Entering ROLLOUT')
                state = 'ROLLOUT'

        # --------------------------------------------------------------------
        # State: ROLLOUT
        # --------------------------------------------------------------------
        elif state == 'ROLLOUT':
            if (now - last_cmd_time) >= (1.0 / CFG.cmd_hz):
                # Hold down on runway (target altitude 0.0m) and maintain centerline
                send_target_altitude(master, 0.0)
                send_course(master, geom.track_deg)
                last_cmd_time = now

            gs = telem.groundspeed()
            if gs is not None and gs < CFG.rollout_stop_gs:
                if CFG.disarm_on_stop:
                    print('Rollout complete -> Force disarming (SITL)')
                    master.mav.command_long_send(
                        master.target_system, master.target_component,
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                        0, 21196, 0, 0, 0, 0, 0)
                state = 'DONE'

        time.sleep(1.0 / CFG.loop_hz)

    print(f'State machine exited. Final state: {state}')


if __name__ == '__main__':
    main()
