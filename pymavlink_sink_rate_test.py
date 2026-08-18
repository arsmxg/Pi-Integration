"""
PyMAVLink SITL Test: 3D Velocity Vector using GLOBAL_INT 
"""
import time
from pymavlink import mavutil

connection_string = 'tcp:127.0.0.1:5762'
print(f"Connecting to SITL at {connection_string}...")
master = mavutil.mavlink_connection(connection_string)

master.wait_heartbeat()
print(f"Heartbeat received from system {master.target_system} component {master.target_component}")

def set_guided_mode(master):
    """
    Commands the flight controller to switch into GUIDED mode using COMMAND_LONG.
    ArduPlane GUIDED custom mode is 15.
    """
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,                                                  
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  
        15,                                                 
        0, 0, 0, 0, 0                                       
    )
    print("Mode switch command sent.")

def send_velocity_command(master, vx, vy, vz):
    """
    Sends a GLOBAL_INT command specifying only the 3D velocity vector.
    """
    now_ms = int(time.time() * 1000) & 0xFFFFFFFF
    
    # 0b1111111111000111 -> controls ONLY vx, vy, and vz. Ignores pos, accel, and yaw.
    type_mask = 0b1111111111000111
    
    msg = master.mav.set_position_target_global_int_encode(
        time_boot_ms=now_ms,
        target_system=master.target_system, 
        target_component=master.target_component,
        coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        type_mask=type_mask,
        lat_int=0, lon_int=0, alt=0,            # Positions (ignored)
        vx=vx, vy=vy, vz=vz,                    # Velocities in m/s
        afx=0, afy=0, afz=0,                    # Accelerations (ignored)
        yaw=0, yaw_rate=0                       # Yaw (ignored)
    )
    
    master.mav.send(msg)

# 1. Engage GUIDED mode
set_guided_mode(master)

# 2. Wait for GUIDED mode confirmation
print("Waiting for GUIDED mode confirmation...")
while True:
    ack_msg = master.recv_match(type='HEARTBEAT', blocking=True)
    if ack_msg and ack_msg.custom_mode == 15:
        print("Confirmed: Aircraft is in GUIDED mode.")
        break

# 3. Execute the Control Loop
duration_seconds = 15
frequency_hz = 10
target_sink_rate_ms = 2.0  # Positive is down

print(f"Streaming target sink rate ({target_sink_rate_ms} m/s) for {duration_seconds} seconds...")

# Fallback velocities
current_vx = 15.0 
current_vy = 0.0

for i in range(duration_seconds * frequency_hz):
    # Read the live telemetry to echo the forward trajectory
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=0.1)
    
    if msg:
        # GLOBAL_POSITION_INT velocity is in cm/s; convert to m/s
        current_vx = msg.vx / 100.0
        current_vy = msg.vy / 100.0

    send_velocity_command(master, current_vx, current_vy, target_sink_rate_ms)
    time.sleep(1.0 / frequency_hz)

print("Sink rate test complete.")