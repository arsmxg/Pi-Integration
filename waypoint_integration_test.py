"""
PyMAVLink SITL Test: Single-Shot Glide Slope Flare
"""
import time
import math
from pymavlink import mavutil

connection_string = 'tcp:127.0.0.1:5762'
print(f"Connecting to SITL at {connection_string}...")
master = mavutil.mavlink_connection(connection_string)

master.wait_heartbeat()
print(f"Heartbeat received from system {master.target_system}")

def get_landing_track_angle(master):
    print("Requesting mission items...")
    master.waypoint_request_list_send()
    count_msg = master.recv_match(type='MISSION_COUNT', blocking=True, timeout=5)
    
    if not count_msg:
        return None

    waypoints = []
    for i in range(count_msg.count):
        master.mav.mission_request_int_send(master.target_system, master.target_component, i)
        wp_msg = master.recv_match(type='MISSION_ITEM_INT', blocking=True, timeout=5)
        if wp_msg:
            waypoints.append(wp_msg)

    land_idx = -1
    for i, wp in enumerate(waypoints):
        if wp.command == mavutil.mavlink.MAV_CMD_NAV_LAND:
            land_idx = i
            break

    if land_idx <= 0:
        return None

    wp_approach = None
    for i in range(land_idx - 1, -1, -1):
        if waypoints[i].command == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
            wp_approach = waypoints[i]
            break

    if not wp_approach:
        return None

    wp_land = waypoints[land_idx]

    lat1 = math.radians(wp_approach.x / 1e7)
    lon1 = math.radians(wp_approach.y / 1e7)
    lat2 = math.radians(wp_land.x / 1e7)
    lon2 = math.radians(wp_land.y / 1e7)

    d_lon = lon2 - lon1
    x = math.sin(d_lon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(d_lon))
    bearing_rad = math.atan2(x, y)
    
    bearing_deg = (math.degrees(bearing_rad) + 360) % 360
    print(f"True Mission Track Found: {bearing_deg:.1f} degrees")
    return bearing_rad

def set_guided_mode(master):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,                                                  
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 15,                                                 
        0, 0, 0, 0, 0                                       
    )
    print("Mode switch command sent.")

def send_glide_slope_waypoint(master, lat_int, lon_int, alt_m):
    """
    Sends the 3D target coordinate EXACTLY ONCE to establish the L1 leg.
    """
    now_ms = int(time.time() * 1000) & 0xFFFFFFFF
    type_mask = 0b1111111111111000 # Use x, y, z
    
    msg = master.mav.set_position_target_global_int_encode(
        now_ms, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, type_mask,
        lat_int, lon_int, alt_m, 
        0, 0, 0, 0, 0, 0, 0, 0
    )
    master.mav.send(msg)

# 1. Fetch the mission track angle
mission_track_rad = get_landing_track_angle(master)
if mission_track_rad is None:
    exit()

# 2. Engage GUIDED mode
set_guided_mode(master)

print("Waiting for GUIDED mode confirmation...")
while True:
    ack_msg = master.recv_match(type='HEARTBEAT', blocking=True)
    if ack_msg and ack_msg.custom_mode == 15:
        print("Confirmed: Aircraft is in GUIDED mode.")
        break

# Capture the exact position and velocity to calculate the glide slope
print("Locking initial state...")
pos_msg = None
while not pos_msg:
    pos_msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)

lat_init = pos_msg.lat 
lon_init = pos_msg.lon 
alt_init_m = pos_msg.relative_alt / 1000.0 

current_vx = pos_msg.vx / 100.0
current_vy = pos_msg.vy / 100.0
v_ground = math.sqrt(current_vx**2 + current_vy**2)

# Prevent division by zero if testing on the ground
if v_ground < 5.0:
    v_ground = 15.0

# 3. Project the Ghost Waypoint (2000 meters ahead)
distance_m = 2000.0
R_earth = 6378137.0
lat_rad = math.radians(lat_init / 1e7)

dx = distance_m * math.cos(mission_track_rad)
dy = distance_m * math.sin(mission_track_rad)

d_lat_deg = math.degrees(dx / R_earth)
d_lon_deg = math.degrees(dy / (R_earth * math.cos(lat_rad)))

target_lat_int = int(lat_init + (d_lat_deg * 1e7))
target_lon_int = int(lon_init + (d_lon_deg * 1e7))

# --- THE FIX: Calculate the required target altitude for a 2.0 m/s sink rate ---
target_sink_rate_ms = 2.0
time_to_target = distance_m / v_ground
altitude_drop = target_sink_rate_ms * time_to_target
target_alt_m = alt_init_m - altitude_drop

print(f"Locked Glide Slope: 2000m ahead at {target_alt_m:.1f}m altitude.")

# Send the MAVLink command exactly once
send_glide_slope_waypoint(master, target_lat_int, target_lon_int, target_alt_m)

# 4. Monitor the descent (No MAVLink spamming)
duration_seconds = 15
print(f"Monitoring 2.0 m/s TECS descent for {duration_seconds} seconds...")

for i in range(duration_seconds):
    time.sleep(1.0)

print("Flare test complete.")