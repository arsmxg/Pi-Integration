# test sending sink rate targets in SITL
import time
from pymavlink import mavutil

# might be 5762
connection_string = 'tcp:127.0.0.1:5760'
print(f"Connecting to SITL at {connection_string}...")
master = mavutil.mavlink_connection(connection_string)

master.wait_heartbeat()
print(f"Heartbeat received from system (system {master.target_system} component {master.target_component})")

def set_guided_mode(master):
    # switch to guided
    mode_id = master.mode_mapping()['GUIDED']
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    print("Mode switch command sent: GUIDED")

def send_vz_command(master, vz):
    # send sink rate targets
    # Bitmask to indicate which fields should be ignored by the vehicle.
    # 3559 (0b110111100111) enables Velocity and ignores Position, Acceleration, and Yaw.
    type_mask = 3559 
    
    master.mav.set_position_target_local_ned_send(
        0,                                   # time_boot_ms (not used)
        master.target_system, 
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # coordinate_frame
        type_mask,                           # type_mask
        0, 0, 0,                             # x, y, z positions (ignored)
        0, 0, vz,                            # vx, vy, vz velocity in m/s
        0, 0, 0,                             # afx, afy, afz accelerations (ignored)
        0, 0                                 # yaw, yaw_rate (ignored)
    )


set_guided_mode(master)

time.sleep(1)

duration_seconds = 15
frequency_hz = 10
target_sink_rate_ms = 2.0  # 2.0 m/s downward velocity

print(f"Streaming Vz target: {target_sink_rate_ms} m/s for {duration_seconds} seconds...")

for i in range(duration_seconds * frequency_hz):
    send_vz_command(master, target_sink_rate_ms)
    time.sleep(1.0 / frequency_hz)

print("Vz test complete.")