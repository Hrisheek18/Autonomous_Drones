#arming

from pymavlink import mavutil

connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
connection.wait_heartbeat()

connection.mav.command_long_send(
    connection.target_system,
    connection.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0
)
print("Drone Arming Sent")

#dis arming

from pymavlink import mavutil

connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
connection.wait_heartbeat()

connection.mav.command_long_send(
    connection.target_system,
    connection.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0
)
print("Drone Disarming Sent")
#change flight mode
from pymavlink import mavutil

mode = 'GUIDED'  # Can be STABILIZE, AUTO, RTL, etc.
connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
connection.wait_heartbeat()

mode_id = connection.mode_mapping()[mode]
connection.mav.set_mode_send(
    connection.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id
)
print(f"Mode changed to {mode}")

#takeoff
from pymavlink import mavutil

altitude = 5  # meters
connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
connection.wait_heartbeat()

connection.mav.command_long_send(
    connection.target_system,
    connection.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,
    0, 0, 0, 0, 0, 0, altitude
)
print(f"Takeoff command to {altitude}m sent")

#land
from pymavlink import mavutil

connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
connection.wait_heartbeat()

connection.mav.command_long_send(
    connection.target_system,
    connection.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    0,
    0, 0, 0, 0, 0, 0, 0
)
print("Landing command sent")

#give protocols to move in any direction by changing vx,vy,vz
from pymavlink import mavutil
import time

connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
connection.wait_heartbeat()

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
#RTL
from pymavlink import mavutil

connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
connection.wait_heartbeat()

connection.set_mode_apm("RTL")
print("Return to Launch mode set")

    #telem data

from pymavlink import mavutil
import time

# Connect to the Pixhawk (update port and baud if necessary)
connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)

# Wait for heartbeat to ensure communication is established
print("Waiting for heartbeat...")
connection.wait_heartbeat()
print(f"Heartbeat received from system {connection.target_system}, component {connection.target_component}")

print("\nReading telemetry data (press Ctrl+C to stop):\n")

# Continuous loop to read telemetry data
while True:
    # Receive only GLOBAL_POSITION_INT messages
    msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)

    # Extract and convert relevant data
    altitude = msg.relative_alt / 1000.0    # in meters
    latitude = msg.lat / 1e7                # in degrees
    longitude = msg.lon / 1e7               # in degrees

    # Display data
    print(f"Altitude: {altitude:.2f} m | Latitude: {latitude:.7f} | Longitude: {longitude:.7f}")

    # Delay between readings
    time.sleep(1)
