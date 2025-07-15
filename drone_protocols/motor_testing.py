#testing the motors

from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
master.wait_heartbeat()
print(f"Connected to system {master.target_system}, component {master.target_component}")

mode_id = master.mode_mapping()['MANUAL']
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id
)
time.sleep(2)

master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0
)
print("Arming sent")
time.sleep(2)

for _ in range(100):  
    master.mav.set_actuator_control_target_send(
        int(time.time() * 1e6),
        master.target_system,
        master.target_component,
        0, 
        [0.5, 0, 0, 0, 0, 0, 0, 0]  # Only motor 1 spins at 50% CW
    )
    time.sleep(0.1)
for _ in range(100):  
    master.mav.set_actuator_control_target_send(
        int(time.time() * 1e6), 
        master.target_system,
        master.target_component,
        0, 
        [0, -0.5, 0, 0, 0, 0, 0, 0]  # Only motor 1 spins at 50% ACW
    )
    time.sleep(0.1)
for _ in range(100): 
    master.mav.set_actuator_control_target_send(
        int(time.time() * 1e6),  
        master.target_system,
        master.target_component,
        0, 
        [0, 0, 0.5, 0, 0, 0, 0, 0]  # Only motor 1 spins at 50% CW
    )
    time.sleep(0.1)
for _ in range(100): 
    master.mav.set_actuator_control_target_send(
        int(time.time() * 1e6),  
        master.target_system,
        master.target_component,
        0,  
        [0, 0, 0, -0.5, 0, 0, 0, 0]  # Only motor 1 spins at 50% ACW
    )
    time.sleep(0.1)

master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0
)
print("Disarming done")
