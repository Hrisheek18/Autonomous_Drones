#for land
from pymavlink import mavutil
import time
connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
connection.wait_heartbeat()

# Set mode to ALT_HOLD
mode = 'ALT_HOLD'
mode_id = connection.mode_mapping()[mode]

connection.mav.set_mode_send(
    connection.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id
)
print("Mode set to ALT_HOLD")
time.sleep(2)
connection.mav.command_long_send(
    connection.target_system,
    connection.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    0,
    0, 0, 0, 0, 0, 0, 0
)
print("Landing command sent")
