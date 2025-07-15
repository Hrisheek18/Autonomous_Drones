# drone2_receiver_pymavlink.py
import socket
import time
from pymavlink import mavutil

# Listen for incoming coordinates from Drone 1
UDP_IP = "0.0.0.0"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
print("[✓] Waiting for object coordinates...")

data, addr = sock.recvfrom(1024)
print(f"[✓] Received from Drone 1: {data.decode()}")

lat, lon, alt = map(float, data.decode().split(','))

# Connect to Pixhawk via serial or UDP
# Example for serial: '/dev/ttyAMA0' or USB: '/dev/ttyUSB0'
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)

# Wait for heartbeat
master.wait_heartbeat()
print(f"[✓] Connected to Pixhawk")

# Set mode to GUIDED
master.set_mode_apm("GUIDED")

# Arm the drone
master.arducopter_arm()
print("[✓] Arming...")
master.motors_armed_wait()
print("[✓] Armed")

# Send position target (GUIDED mode)
master.mav.set_position_target_global_int_send(
    int(time.time()*1e6),
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
    0b0000111111111000,  # Only position enabled
    int(lat * 1e7),       # Latitude (scaled)
    int(lon * 1e7),       # Longitude (scaled)
    alt,                  # Altitude (meters)
    0, 0, 0,              # Velocity
    0, 0, 0,              # Acceleration
    0, 0                  # Yaw, yaw rate
)

print(f"[✓] Sent command to go to object at {lat}, {lon}, alt {alt}")
