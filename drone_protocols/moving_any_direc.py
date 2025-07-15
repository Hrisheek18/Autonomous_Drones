#give protocols to move in any direction by changing vx,vy,vz
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

def send_velocity(vx, vy, vz):
    connection.mav.set_position_target_local_ned_send(
        int(time.time() * 1e6),
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0
    )

print("Moving forward...")
for _ in range(20):
    send_velocity(1.0, 0, 0)
    time.sleep(0.1)

#for hovering we can keep vx,vy,vz=0 