from pymavlink import mavutil
from pynput import keyboard
import time

# Connect to the flight controller
print("Connecting to Pixhawk...")
master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
master.wait_heartbeat()
print("Connected!")

def arm_and_takeoff(altitude=2):
    master.set_mode("GUIDED")

    print("Arming motors...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    master.motors_armed_wait()
    print("Motors armed.")

    print(f"Taking off to {altitude} meters...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, altitude
    )
    time.sleep(5)

def land():
    print("Landing...")
    master.set_mode("LAND")

def disarm():
    print("Disarming motors...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    master.motors_disarmed_wait()

def send_velocity(vx, vy, vz, yaw_rate=0):
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,  # Only velocity + yaw rate enabled
        0, 0, 0,             # x, y, z positions (not used)
        vx, vy, vz,          # velocity
        0, 0, 0,             # acceleration (not used)
        0, yaw_rate
    )

def on_press(key):
    try:
        k = key.char.lower()
        if k == 't':
            arm_and_takeoff(2)
        elif k == 'l':
            land()
        elif k == 'w':
            send_velocity(0.5, 0, 0)  # forward
        elif k == 's':
            send_velocity(-0.5, 0, 0) # backward
        elif k == 'a':
            send_velocity(0, -0.5, 0) # left
        elif k == 'd':
            send_velocity(0, 0.5, 0)  # right
        elif k == 'q':
            send_velocity(0, 0, 0, yaw_rate=-30) # yaw left
        elif k == 'e':
            send_velocity(0, 0, 0, yaw_rate=30)  # yaw right
        elif k == ' ':
            send_velocity(0, 0, 0)  # stop
        elif k == 'x':
            disarm()
            print("Exiting...")
            return False
    except AttributeError:
        pass

print("Keyboard control started.")
print("Press 't' to takeoff, 'l' to land, 'x' to disarm + exit.")

with keyboard.Listener(on_press=on_press) as listener:
    listener.join()
