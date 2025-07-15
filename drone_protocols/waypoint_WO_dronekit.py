#waypoint

from pymavlink import mavutil
import time

# Connect to the Pixhawk
connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))

# Set mode to ALT_HOLD
mode = 'ALT_HOLD'
mode_id = connection.mode_mapping()[mode]

connection.mav.set_mode_send(
    connection.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id
)

# Function to send waypoints
def send_waypoints():
    # Clear existing mission
    connection.mav.mission_clear_all_send(connection.target_system, connection.target_component)
    time.sleep(1)

    #home = [19.0760, 72.8777, 10]      
    #wp1 =  [19.0761, 72.8778, 10]
    #wp2 =  [19.0762, 72.8777, 10]
    #wp3 =  [19.0763, 72.8776, 10]

    waypoints = [home, wp1, wp2, wp3]

    # Send count of waypoints
    connection.mav.mission_count_send(connection.target_system, connection.target_component, len(waypoints))

    # Wait for MISSION_REQUEST and respond
    for i, wp in enumerate(waypoints):
        msg = connection.recv_match(type='MISSION_REQUEST', blocking=True)
        print(f"MISSION_REQUEST received for seq: {msg.seq}")

        connection.mav.mission_item_send(
            connection.target_system,
            connection.target_component,
            i,                                       # Sequence
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 0,                                    # current, autocontinue
            0, 0, 0, 0,                              # params 1-4
            wp[0], wp[1], wp[2]                      # latitude, longitude, altitude
        )
        print(f"Sent waypoint {i}: {wp}")

# Arm and switch to AUTO mode
def set_mode_and_arm():
    # Set AUTO mode
    mode_id = connection.mode_mapping()['AUTO']
    connection.mav.set_mode_send(
        connection.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    print("Mode set to AUTO")
    time.sleep(1)

    # Arm the drone
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    print("Arming command sent")
    time.sleep(5)

# Start the mission
def start_mission():
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START,
        0,
        0, 0, 0, 0, 0, 0, 0
    )
    print("Mission start command sent")

# Main Execution
send_waypoints()
set_mode_and_arm()
start_mission()

# Monitor mission progress
while True:
    msg = connection.recv_match(type=['MISSION_CURRENT', 'GLOBAL_POSITION_INT'], blocking=True)
    if msg.get_type() == 'MISSION_CURRENT':
        print(f"Currently on waypoint {msg.seq}")
    elif msg.get_type() == 'GLOBAL_POSITION_INT':
        print(f"Altitude: {msg.relative_alt/1000.0}m, Lat: {msg.lat/1e7}, Lon: {msg.lon/1e7}")
    time.sleep(1)
