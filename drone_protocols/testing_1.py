from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
master.wait_heartbeat()
print(" Heartbeat received")

mode = 'GUIDED'
mode_id = master.mode_mapping()[mode]
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id
)
print(" Mode set to GUIDED")
time.sleep(2)

# Arming 
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0
)
print(" Arming sent")
time.sleep(3)

# takeoff to 2 meters
altitude = 2
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,
    0, 0, 0, 0, 0, 0,
    altitude
)
print(" Takeoff command sent to 2 meters")
time.sleep(8)  

# Yaw control to 90 degrees
yaw_deg = 90
yaw_rad = yaw_deg * 3.14159 / 180.0

master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_CONDITION_YAW,
    0,
    yaw_deg,   # Target yaw angle
    20,        # Yaw speed (deg/s)
    1,         # Direction (1 = CW, -1 = CCW)
    1,         # Relative (1 = relative, 0 = absolute)
    0, 0, 0
)
print(" Yaw set to 90 deg")
time.sleep(5)

# Landing
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    0,
    0, 0, 0, 0, 0, 0, 0
)
print(" Landing the drone")
time.sleep(10)

# Disarming
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0
)
print(" Drone disarmed")
