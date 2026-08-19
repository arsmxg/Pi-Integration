"""
Variable Tau Flare via Companion Computer (SITL validation)
===========================================================
Target firmware: stock ArduPlane 4.5.6 (no custom TECS modifications).

Concept of operations
---------------------
1. WAIT_APPROACH: aircraft flies the mission in AUTO. The script downloads the
   mission, finds the NAV_LAND item and the final NAV_WAYPOINT before it, and
   derives the runway centerline (track bearing + land point). It then watches
   AGL while the NAV_LAND item is the active mission item.
2. FLARE: when AGL <= FLARE_ENGAGE_AGL and the aircraft is descending, the
   script switches to GUIDED and closes two loops at CMD_HZ:
     Lateral : cross-track error vs. the centerline -> ground-course command
               via MAV_CMD_GUIDED_CHANGE_HEADING (HEADING_TYPE_COURSE_OVER_GROUND).
               Roll is produced by the firmware's guidedHeading AC_PID, so
               tracking behaves like AUTO rather than a loiter orbit.
     Vertical: variable tau law
                   tau   = TAU_O * V_GO / V_G
                   h_B   = tau * TD_SINK
                   hdot  = -(h_AGL + h_B) / tau
               commanded through MAV_CMD_GUIDED_CHANGE_ALTITUDE, whose param3
               is a height-demand slew rate (m/s). Resending each cycle with a
               target below the aircraft makes it a height-rate interface into
               TECS. TECS remains the inner loop throughout.
   Airspeed demand is dropped to AIRSPEED_MIN + margin once at engage so TECS
   runs the descent near idle throttle.
3. ROLLOUT: below TOUCHDOWN_AGL the script keeps a small down demand to hold
   the aircraft on the runway, then (optionally) force-disarms once
   groundspeed decays.
4. GO_AROUND: at any point after engage, transmitter throttle above
   GO_AROUND_THR_PWM commands mode TAKEOFF and the script exits the flare.

Pilot authority: STICK_MIXING = 1 (default) applies in GUIDED exactly as in
AUTO, so aileron/elevator inputs mix into the demanded attitude with no
script involvement. The pilot's mode switch also overrides at any time; the
script detects an external mode change and stands down.

SITL setup
----------
  sim_vehicle.py -v ArduPlane --console --map
  # optional SITL rangefinder (script falls back to relative_alt without it):
  param set RNGFND1_TYPE 100
  param set RNGFND1_MIN_CM 5
  param set RNGFND1_MAX_CM 4000
  param set RNGFND1_ORIENT 25
  # reboot, load a mission ending in NAV_WAYPOINT -> NAV_LAND, arm, mode AUTO
  # run this script any time before final; it waits for the engage condition
"""

import math
import time
from dataclasses import dataclass

from pymavlink import mavutil


# ----------------------------------------------------------------------------
# Configuration
# ----------------------------------------------------------------------------
@dataclass
class Config:
    connection: str = 'tcp:127.0.0.1:5762'

    # Variable tau law (mirrors FLARE_TAU_O / FLARE_VGO from the firmware mod)
    tau_o: float = 4.0            # nominal time constant [s]
    v_go: float = 15.0            # nominal approach groundspeed [m/s]
    td_sink: float = 0.25         # touchdown sink rate bias term [m/s]

    # Engagement / termination
    flare_engage_agl: float = 8.0   # AGL to switch AUTO -> GUIDED [m]
    touchdown_agl: float = 0.30     # AGL treated as ground contact [m]
    rollout_stop_gs: float = 2.0    # groundspeed to declare rollout done [m/s]
    disarm_on_stop: bool = True     # force-disarm after rollout (SITL only)

    # Command shaping
    sink_max: float = 4.0           # steepest allowed demand [m/s]
    vg_min: float = 5.0             # groundspeed clamp for tau [m/s]
    vg_max: float = 40.0
    flare_aspd_margin: float = 0.5  # commanded airspeed = AIRSPEED_MIN + this

    # Lateral guidance
    t_intercept: float = 4.0        # time constant to close cross-track [s]
    v_close_max: float = 3.0        # max lateral closure velocity [m/s]
    max_course_corr_deg: float = 25.0
    hdg_accel_limit: float = 2.5    # lateral accel limit -> bank limit [m/s^2]

    # Go-around
    go_around_thr_pwm: int = 1900   # RC throttle PWM threshold
    throttle_channel: int = 3

    # Rates
    cmd_hz: float = 5.0             # heading/altitude command rate
    loop_hz: float = 20.0           # telemetry pump rate


CFG = Config()

# Plane custom mode numbers
MODE_AUTO, MODE_TAKEOFF, MODE_GUIDED = 10, 13, 15
HEADING_TYPE_COG = 0
SPEED_TYPE_AIRSPEED = 0
R_EARTH = 6378137.0


# ----------------------------------------------------------------------------
# Telemetry cache
# ----------------------------------------------------------------------------
class Telemetry:
    """Latest-value cache for the handful of messages the loops consume."""

    WANTED = {
        'GLOBAL_POSITION_INT': 33,
        'ATTITUDE': 30,
        'MISSION_CURRENT': 42,
        'RC_CHANNELS': 65,
        'VFR_HUD': 74,
        'DISTANCE_SENSOR': 132,
        'RANGEFINDER': 173,
        'HEARTBEAT': 0,
    }

    def __init__(self, master):
        """Initialize the telemetry cache.

        Args:
            master: mavutil MAVLink connection instance.
        """
        self.master = master
        self.msgs = {}
        self.stamps = {}

    def request_streams(self):
        """Request required MAVLink telemetry message streams from the autopilot at 10 Hz."""
        for name, msgid in self.WANTED.items():
            if name == 'HEARTBEAT':
                continue
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                msgid, 1e6 / 10.0, 0, 0, 0, 0, 0)

    def pump(self):
        """Read and cache incoming MAVLink messages in a non-blocking loop.

        Filters messages to ensure they originate from the targeted autopilot
        (ignoring GCS/MAVProxy heartbeats) and caches WANTED message types with timestamps.
        """
        while True:
            msg = self.master.recv_match(blocking=False)
            if msg is None:
                return
            # MAVProxy and any other GCS link get forwarded onto this
            # connection; their HEARTBEATs carry custom_mode 0
            if msg.get_srcSystem() != self.master.target_system:
                continue
            if msg.get_srcComponent() != mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1:
                continue
            t = msg.get_type()
            if t in self.WANTED:
                self.msgs[t] = msg
                self.stamps[t] = time.time()

    def get(self, name, max_age=None):
        """Retrieve the latest cached message of a given type.

        Args:
            name (str): MAVLink message type name (e.g., 'ATTITUDE', 'VFR_HUD').
            max_age (float, optional): Maximum age in seconds before message is considered stale.

        Returns:
            mavlink message object or None if absent or expired.
        """
        msg = self.msgs.get(name)
        if msg is None:
            return None
        if max_age is not None and time.time() - self.stamps[name] > max_age:
            return None
        return msg

    # -- derived quantities ---------------------------------------------------
    def agl(self):
        """Get the filtered EKF altitude above ground/home in meters.

        Uses the EKF's fused state estimate from GLOBAL_POSITION_INT.relative_alt.
        Directly querying the EKF avoids algebraic pitch-attitude coupling (where
        flaring up causes geometric cos(pitch) shrinkage, falsely indicating a lower
        AGL and triggering pitch-induced oscillations).

        Returns:
            float: EKF estimated relative altitude in meters, or None if unavailable.
        """
        gpi = self.get('GLOBAL_POSITION_INT', 1.0)
        if gpi is not None:
            return gpi.relative_alt / 1000.0
        return None

    def groundspeed(self):
        """Get the current groundspeed in meters per second.

        Extracts groundspeed from VFR_HUD or derives it from GLOBAL_POSITION_INT velocity vectors.

        Returns:
            float: Current groundspeed in m/s, or None if unavailable.
        """
        hud = self.get('VFR_HUD', 1.0)
        if hud is not None:
            return hud.groundspeed
        gpi = self.get('GLOBAL_POSITION_INT')
        if gpi is not None:
            return math.hypot(gpi.vx, gpi.vy) / 100.0
        return None

    def mode(self):
        """Get the current custom flight mode number from the autopilot HEARTBEAT.

        Returns:
            int: Custom flight mode ID (e.g. 10 for AUTO, 15 for GUIDED), or None.
        """
        hb = self.get('HEARTBEAT', 3.0)
        return hb.custom_mode if hb is not None else None

    def throttle_pwm(self):
        """Get the pilot's transmitter throttle channel PWM value.

        Used for pilot go-around detection during automated flare/rollout.

        Returns:
            int: Throttle channel PWM in microseconds (typically 1000-2000), or None.
        """
        rc = self.get('RC_CHANNELS', 1.0)
        if rc is None:
            return None
        return getattr(rc, f'chan{CFG.throttle_channel}_raw', None)


# ----------------------------------------------------------------------------
# Mission geometry
# ----------------------------------------------------------------------------
def download_centerline(master):
    """Download the autopilot mission and determine runway centerline geometry.

    Fetches all mission items, locates the NAV_LAND waypoint and its immediate
    preceding NAV_WAYPOINT, then computes the approach track bearing (radians)
    and touchdown coordinate references.

    Args:
        master: MAVLink connection instance.

    Returns:
        tuple: (track_rad, land_lat_deg, land_lon_deg, land_seq)
            - track_rad (float): Runway centerline heading in radians.
            - land_lat_deg (float): Latitude of the NAV_LAND point in degrees.
            - land_lon_deg (float): Longitude of the NAV_LAND point in degrees.
            - land_seq (int): Mission sequence index of the NAV_LAND command.

    Raises:
        RuntimeError: If mission download times out or required waypoints are missing.
    """
    print('Downloading mission...')
    master.waypoint_request_list_send()
    count = master.recv_match(type='MISSION_COUNT', blocking=True, timeout=5)
    if count is None:
        raise RuntimeError('No MISSION_COUNT received')

    wps = []
    for i in range(count.count):
        master.mav.mission_request_int_send(
            master.target_system, master.target_component, i)
        wp = master.recv_match(type='MISSION_ITEM_INT', blocking=True, timeout=5)
        if wp is None:
            raise RuntimeError(f'Timed out fetching mission item {i}')
        wps.append(wp)
    master.mav.mission_ack_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_MISSION_ACCEPTED)

    land_idx = next((i for i, w in enumerate(wps)
                     if w.command == mavutil.mavlink.MAV_CMD_NAV_LAND), None)
    if land_idx is None or land_idx == 0:
        raise RuntimeError('Mission has no NAV_LAND with a preceding waypoint')

    approach = next((wps[i] for i in range(land_idx - 1, -1, -1)
                     if wps[i].command == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT),
                    None)
    if approach is None:
        raise RuntimeError('No NAV_WAYPOINT found before NAV_LAND')

    land = wps[land_idx]
    lat1, lon1 = math.radians(approach.x / 1e7), math.radians(approach.y / 1e7)
    lat2, lon2 = math.radians(land.x / 1e7), math.radians(land.y / 1e7)
    dlon = lon2 - lon1
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    track = math.atan2(x, y)
    print(f'Centerline track {math.degrees(track) % 360:.1f} deg, '
          f'NAV_LAND at mission seq {land.seq}')
    return track, land.x / 1e7, land.y / 1e7, land.seq


def cross_track_m(lat_deg, lon_deg, track_rad, ref_lat_deg, ref_lon_deg):
    """Compute signed cross-track distance in meters from runway centerline.

    Uses equirectangular projection centered on the landing threshold reference.

    Args:
        lat_deg (float): Aircraft current latitude in degrees.
        lon_deg (float): Aircraft current longitude in degrees.
        track_rad (float): Runway centerline heading in radians.
        ref_lat_deg (float): Reference touchdown point latitude in degrees.
        ref_lon_deg (float): Reference touchdown point longitude in degrees.

    Returns:
        float: Signed cross-track error in meters.
               Positive indicates the aircraft is to the right of centerline
               when facing along the approach direction.
    """
    ref_lat = math.radians(ref_lat_deg)
    dn = (lat_deg - ref_lat_deg) * math.pi / 180.0 * R_EARTH
    de = (lon_deg - ref_lon_deg) * math.pi / 180.0 * R_EARTH * math.cos(ref_lat)
    tn, te = math.cos(track_rad), math.sin(track_rad)
    # z-component of track x position (NE frame): positive when right of track
    return tn * de - te * dn


# ----------------------------------------------------------------------------
# MAVLink command helpers (all OFFBOARD_GUIDED slew commands, COMMAND_INT)
# ----------------------------------------------------------------------------
def cmd_int(master, command, p1=0, p2=0, p3=0, p4=0, x=0, y=0, z=0,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL):
    """Send a MAVLink COMMAND_INT packet to the autopilot.

    Helper function wrapping `command_int_send` for offboard guided control commands.

    Args:
        master: MAVLink connection instance.
        command (int): MAVLink command ID (MAV_CMD enum).
        p1-p4 (float): Command-specific parameters 1 to 4.
        x, y (int): Scaled coordinate values (e.g. latitude/longitude * 1e7) if applicable.
        z (float): Altitude or z-axis parameter.
        frame (int): Coordinate frame (e.g., MAV_FRAME_GLOBAL, MAV_FRAME_GLOBAL_RELATIVE_ALT).
    """
    master.mav.command_int_send(
        master.target_system, master.target_component,
        frame, command, 0, 0, p1, p2, p3, p4, x, y, z)


def send_course(master, course_deg):
    """Command the autopilot's ground-course heading in GUIDED mode.

    Sends MAV_CMD_GUIDED_CHANGE_HEADING with course-over-ground mode (COG)
    and lateral acceleration limits to prevent excessive bank angles during flare.

    Args:
        master: MAVLink connection instance.
        course_deg (float): Desired ground course heading in degrees (0-360).
    """
    cmd_int(master, mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_HEADING,
            p1=HEADING_TYPE_COG, p2=course_deg % 360.0,
            p3=CFG.hdg_accel_limit)


def send_height_rate(master, rel_alt_m, hdot_mps):
    """Command TECS altitude slew downward at a desired sink rate (|hdot| m/s).

    Uses MAV_CMD_GUIDED_CHANGE_ALTITUDE with param3 specifying the height slew rate.
    Because ArduPlane resets its slew origin on every accepted command, continuously
    transmitting a target below current altitude transforms this into an effective
    vertical sink rate interface to TECS.

    Args:
        master: MAVLink connection instance.
        rel_alt_m (float): Current relative altitude in meters.
        hdot_mps (float): Desired vertical sink rate in m/s (negative for descent).
    """
    target = rel_alt_m - 5.0
    # handler rejects z == 0.0 and z == -1.0 exactly
    if abs(target) < 0.05:
        target = -0.05
    if abs(target + 1.0) < 0.05:
        target = -1.1
    cmd_int(master, mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_ALTITUDE,
            p3=abs(hdot_mps), z=target,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT)


def send_airspeed(master, aspd_mps):
    """Set the target equivalent airspeed demand in GUIDED mode.

    Uses MAV_CMD_GUIDED_CHANGE_SPEED to throttle down to approach/flare airspeed
    (AIRSPEED_MIN + margin) so TECS executes the descent near idle throttle.

    Args:
        master: MAVLink connection instance.
        aspd_mps (float): Demanded airspeed target in m/s.
    """
    cmd_int(master, mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_SPEED,
            p1=SPEED_TYPE_AIRSPEED, p2=aspd_mps, p3=0)


def set_mode(master, mode):
    """Request the autopilot switch to a specific custom flight mode.

    Sends MAV_CMD_DO_SET_MODE with MAV_MODE_FLAG_CUSTOM_MODE_ENABLED.

    Args:
        master: MAVLink connection instance.
        mode (int): Custom flight mode number (e.g. MODE_GUIDED=15, MODE_TAKEOFF=13).
    """
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode,
        0, 0, 0, 0, 0)


def fetch_param(master, name, timeout=2.0):
    """Request and read a named parameter value from the autopilot over MAVLink.

    Args:
        master: MAVLink connection instance.
        name (str): Autopilot parameter name (e.g., 'AIRSPEED_MIN', 'ARSPD_FBW_MIN').
        timeout (float): Max wait time in seconds for PARAM_VALUE response.

    Returns:
        float: Parameter value if received, or None on timeout.
    """
    master.mav.param_request_read_send(
        master.target_system, master.target_component,
        name.encode('ascii'), -1)
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
        if msg is not None and msg.param_id.strip('\x00') == name:
            return msg.param_value
    return None


# ----------------------------------------------------------------------------
# Flare law
# ----------------------------------------------------------------------------
def variable_tau_hdot(agl_m, vg_mps):
    """Calculate target vertical sink rate (hdot) using the Variable-Tau guidance law.

    Implements:
        tau  = tau_o * (v_go / V_g)      # Velocity-scaled time constant
        h_B  = tau * td_sink             # Touchdown bias altitude
        hdot = -(h_AGL + h_B) / tau      # Commanded descent sink rate

    Clamps groundspeed to [vg_min, vg_max] and clamps output sink rate
    between -td_sink (soft touchdown rate) and -sink_max (steepest descent limit).

    Args:
        agl_m (float): Current altitude above ground level in meters.
        vg_mps (float): Current aircraft groundspeed in meters per second.

    Returns:
        tuple: (hdot_cmd, tau)
            - hdot_cmd (float): Commanded vertical rate in m/s (negative for descent).
            - tau (float): Instantaneous time constant in seconds.
    """
    vg = min(max(vg_mps, CFG.vg_min), CFG.vg_max)
    tau = CFG.tau_o * CFG.v_go / vg
    h_b = tau * CFG.td_sink
    hdot = -(agl_m + h_b) / tau
    return max(-CFG.sink_max, min(hdot, -CFG.td_sink)), tau


# ----------------------------------------------------------------------------
# Main state machine
# ----------------------------------------------------------------------------
def main():
    """Execute the companion-computer variable-tau flare integration loop.

    Workflow:
    1. Connects to the SITL MAVLink TCP endpoint and downloads the active landing mission.
    2. Computes the runway centerline heading and landing threshold coordinates.
    3. Requests high-rate telemetry and queries stall/minimum airspeed parameters.
    4. Runs state machine transitions:
       - WAIT_APPROACH: Monitors AUTO mission progression on final approach until AGL <= flare_engage_agl.
       - FLARE: Switches to GUIDED mode, applies Variable-Tau sink rate demands to TECS, and tracks centerline.
       - ROLLOUT: Below touchdown AGL, maintains hold-down altitude demand until groundspeed stops, then disarms.
       - GO_AROUND: Triggers if pilot throttle stick exceeds threshold, immediately commanding TAKEOFF mode.
       - ABORT: Stands down if pilot switches flight mode externally.
    """
    print(f'Connecting to {CFG.connection}...')
    master = mavutil.mavlink_connection(
        CFG.connection, source_system=1,
        source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER)
    master.wait_heartbeat()
    print(f'Heartbeat from system {master.target_system}')

    track_rad, land_lat, land_lon, land_seq = download_centerline(master)

    telem = Telemetry(master)
    telem.request_streams()

    aspd_min = fetch_param(master, 'AIRSPEED_MIN')
    if aspd_min is None:
        aspd_min = fetch_param(master, 'ARSPD_FBW_MIN')
    flare_aspd = (aspd_min + CFG.flare_aspd_margin) if aspd_min else None
    print(f'Flare airspeed target: {flare_aspd if flare_aspd else "not set"}')

    state = 'WAIT_APPROACH'
    last_cmd = 0.0
    last_print = 0.0
    non_guided_since = None
    print('Waiting for AUTO landing approach '
          f'(engage at AGL <= {CFG.flare_engage_agl} m)...')

    while state not in ('DONE', 'GO_AROUND', 'ABORT'):
        telem.pump()
        now = time.time()
        agl = telem.agl()
        gpi = telem.get('GLOBAL_POSITION_INT')
        mode = telem.mode()

        # ---- pilot go-around wins in every armed state ----------------------
        if state in ('FLARE', 'ROLLOUT'):
            thr = telem.throttle_pwm()
            if thr is not None and thr > CFG.go_around_thr_pwm:
                print(f'GO-AROUND: throttle {thr} us > '
                      f'{CFG.go_around_thr_pwm} us, commanding TAKEOFF')
                set_mode(master, MODE_TAKEOFF)
                state = 'GO_AROUND'
                break
            # pilot changed mode out of GUIDED some other way: stand down
            if mode is not None and mode != MODE_GUIDED:
                if non_guided_since is None:
                    non_guided_since = now
                elif now - non_guided_since >= 1.5:
                    print(f'External mode change (custom_mode {mode}), '
                          'standing down')
                    state = 'ABORT'
                    break
            else:
                non_guided_since = None

        if state == 'WAIT_APPROACH':
            mc = telem.get('MISSION_CURRENT')
            on_final = (mode == MODE_AUTO and mc is not None
                        and mc.seq == land_seq)
            descending = gpi is not None and gpi.vz > 20  # cm/s, +down
            if on_final and descending and agl is not None \
                    and agl <= CFG.flare_engage_agl:
                print(f'ENGAGE at AGL {agl:.2f} m: switching to GUIDED')
                set_mode(master, MODE_GUIDED)
                t0 = time.time()
                while telem.mode() != MODE_GUIDED and time.time() - t0 < 3.0:
                    telem.pump()
                    time.sleep(0.02)
                if telem.mode() != MODE_GUIDED:
                    print('GUIDED not confirmed, aborting')
                    state = 'ABORT'
                    break
                if flare_aspd is not None:
                    send_airspeed(master, flare_aspd)
                # seed lateral control immediately so _enter()'s
                # loiter-at-current-location target never drives roll
                send_course(master, math.degrees(track_rad))
                state = 'FLARE'
                print('FLARE active')

        elif state == 'FLARE':
            if now - last_cmd >= 1.0 / CFG.cmd_hz and gpi is not None \
                    and agl is not None:
                vg = telem.groundspeed() or CFG.v_go
                hdot, tau = variable_tau_hdot(agl, vg)
                send_height_rate(master, gpi.relative_alt / 1000.0, hdot)

                xtk = cross_track_m(gpi.lat / 1e7, gpi.lon / 1e7,
                                    track_rad, land_lat, land_lon)
                v_close = max(-CFG.v_close_max,
                              min(-xtk / CFG.t_intercept, CFG.v_close_max))
                corr = math.asin(max(-0.5, min(v_close / max(vg, CFG.vg_min),
                                               0.5)))
                corr = max(-math.radians(CFG.max_course_corr_deg),
                            min(corr, math.radians(CFG.max_course_corr_deg)))
                send_course(master, math.degrees(track_rad + corr))
                last_cmd = now

                if now - last_print >= 1.0:
                    print(f'AGL {agl:5.2f} m  Vg {vg:4.1f}  tau {tau:4.2f} s  '
                          f'hdot_cmd {hdot:5.2f} m/s  xtk {xtk:+5.1f} m')
                    last_print = now

            if agl is not None and agl <= CFG.touchdown_agl:
                print('Touchdown detected, entering rollout')
                state = 'ROLLOUT'

        elif state == 'ROLLOUT':
            if now - last_cmd >= 1.0 / CFG.cmd_hz and gpi is not None:
                # hold a small down demand and centerline through rollout
                send_height_rate(master, gpi.relative_alt / 1000.0, 0.5)
                send_course(master, math.degrees(track_rad))
                last_cmd = now
            gs = telem.groundspeed()
            if gs is not None and gs < CFG.rollout_stop_gs:
                if CFG.disarm_on_stop:
                    print('Rollout complete, force-disarming (SITL)')
                    master.mav.command_long_send(
                        master.target_system, master.target_component,
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                        0, 21196, 0, 0, 0, 0, 0)
                state = 'DONE'

        time.sleep(1.0 / CFG.loop_hz)

    print(f'Final state: {state}')


if __name__ == '__main__':
    main()
