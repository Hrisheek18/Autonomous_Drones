import time
from pymavlink import mavutil

connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=921600)

connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))

# Arm the vehicle
print("Arming motors")
msg = connection.mav.command_long_encode(
    connection.target_system,
    connection.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,1, 0, 0, 0, 0, 0, 0
)
connection.write(msg)

time.sleep(5)

# Disarm the vehicle
print("Disarming motors")
msg = connection.mav.command_long_encode(
    connection.target_system,
    connection.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,0,0, 0, 0, 0, 0, 0
)
connection.write(msg)

# Close the connection (optional)
connection.close()