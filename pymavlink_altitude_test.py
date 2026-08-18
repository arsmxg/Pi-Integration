"""
PyMAVLink SITL Test: Altitude Target for ArduPlane GUIDED Mode
"""
import time
from pymavlink import mavutil

connection_string = 'tcp:127.0.0.1:5762'
print(f"Connecting to SITL at {connection_string}...")
master = mavutil.mavlink_connection(connection_string)

master.wait_heartbeat()
print(f"Heartbeat received from system (system {master.target_system} component {master.target_component})")

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

def send_altitude_command(master, target_alt_meters):
    """
    Sends a GLOBAL_INT command specifying only the target altitude.
    """
    # Create a timestamp in milliseconds for time_boot_ms
    now_ms = int(time.time() * 1000) & 0xFFFFFFFF
    
    # type_mask = 0b1111111111111011 -> controls ONLY z (altitude)
    msg = master.mav.set_position_target_global_int_encode(
        time_boot_ms=now_ms,
        target_system=master.target_system, 
        target_component=master.target_component,
        coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        type_mask=0b1111111111111011,
        lat_int=0, lon_int=0,                   # Positions (ignored)
        alt=target_alt_meters,                  # Target altitude in meters AGL
        vx=0, vy=0, vz=0,                       # Velocities (ignored)
        afx=0, afy=0, afz=0,                    # Accelerations (ignored)
        yaw=0, yaw_rate=0                       # Yaw (ignored)
    )
    
    # Send the encoded message
    master.mav.send(msg)

# 1. Engage GUIDED mode
set_guided_mode(master)

# 2. Safety Check: Wait for the flight controller to confirm the mode switch
print("Waiting for GUIDED mode confirmation...")
while True:
    ack_msg = master.recv_match(type='HEARTBEAT', blocking=True)
    if ack_msg and ack_msg.custom_mode == 15:
        print("Confirmed: Aircraft is in GUIDED mode.")
        break

# 3. Execute the Control Loop
duration_seconds = 15
frequency_hz = 10
target_alt = 30.0  # Set your target altitude here (meters AGL)

print(f"Streaming target altitude ({target_alt} m) for {duration_seconds} seconds...")

for i in range(duration_seconds * frequency_hz):
    send_altitude_command(master, target_alt)
    time.sleep(1.0 / frequency_hz)

print("Altitude test complete.")