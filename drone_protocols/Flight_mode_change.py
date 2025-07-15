#change flight mode
from pymavlink import mavutil

mode = 'GUIDED'  
connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
connection.wait_heartbeat()

mode_id = connection.mode_mapping()[mode]
connection.mav.set_mode_send(
    connection.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id
)
print(f"Mode changed to {mode}")
