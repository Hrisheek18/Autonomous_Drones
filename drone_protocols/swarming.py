#swarming

# swarm_simple.py
from pymavlink import mavutil
import time

drones = {
    'drone1': mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600),
    'drone2': mavutil.mavlink_connection('/dev/ttyUSB1', baud=57600),
}
for m in drones.values():
    m.wait_heartbeat()
print("Connected to both drones")

# Arm & GUIDED
for name, m in drones.items():
    m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1,0,0,0,0,0,0)
    m.mav.set_mode_send(m.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, m.mode_mapping()['GUIDED'])
time.sleep(5)

# Assign formation targets (relative offsets)
targets = {
    'drone1': (0, 0, -10),
    'drone2': (0, 10, -10),
}
def goto_offset(m, dx, dy, dz):
    m.mav.set_position_target_local_ned_send(
        int(time.time()*1e6), m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111000111,
        dx, dy, dz, 0,0,0,0,0,0,0,0
    )

for name, (dx, dy, dz) in targets.items():
    goto_offset(drones[name], dx, dy, dz)

print("Formation target sent")
time.sleep(20)

# Disarm
for m in drones.values():
    m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0,0,0,0,0,0,0)
print("SWARM: Disarm sent")
