import time
import cv2
import numpy as np
from pymavlink import mavutil

# Connect to Pixhawk over serial
master = mavutil.mavlink_connection('/dev/serial0', baud=57600)
master.wait_heartbeat()
print("Connected to Pixhawk")

# Function to set mode
def set_mode(mode):
    mode_id = master.mode_mapping()[mode]  
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

# Arm and takeoff function
def arm_and_takeoff(target_altitude):
    set_mode("GUIDED")
    time.sleep(2)

    # Arm motors
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    print("Arming...")
    time.sleep(5)

    # Takeoff
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, target_altitude
    )
    print(f"Taking off to {target_altitude} meters...")
    time.sleep(7)

# Function to send body-frame velocity and yaw rate
def send_velocity(vx, vy, vz, yaw_rate):
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,  # Only velocity and yaw rate enabled
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, yaw_rate
    )

# Start flight
arm_and_takeoff(2)  # 2 meters altitude

# Start camera
cap = cv2.VideoCapture(0)

# Run until 'q' is pressed
while True:
    ret, frame = cap.read()
    if not ret:
        print("Frame not received.")
        continue

    frame = cv2.flip(frame, 1)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Detect red object
    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])
    mask = cv2.inRange(hsv, lower_red, upper_red)

    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    h, w = frame.shape[:2]
    cx = w // 2
    object_found = False

    if contours:
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) > 500:
            x, y, obj_w, obj_h = cv2.boundingRect(largest)
            cx = x + obj_w // 2
            object_found = True
            cv2.rectangle(frame, (x, y), (x+obj_w, y+obj_h), (0, 255, 0), 2)

    # Calculate horizontal offset from center
    error_x = cx - (w // 2)
    threshold = 40

    if object_found:
        print("Object detected")

        # Set yaw rate based on object's position
        if abs(error_x) > threshold:
            yaw_rate = 0.2 if error_x > 0 else -0.2
        else:
            yaw_rate = 0

        # Move forward and adjust yaw
        send_velocity(0.2, 0, 0, yaw_rate)
    else:
        print("Searching...")
        send_velocity(0, 0, 0, 0)

    # Show the frame
    cv2.imshow("Object Tracking", frame)

    # Exit when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Stop and land when exiting
cap.release()
cv2.destroyAllWindows()
print("Landing...")

master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    0, 0, 0, 0, 0, 0, 0, 0
)
