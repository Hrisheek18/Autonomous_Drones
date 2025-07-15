from simple_pid import PID
from modules import drone
import time
import os  #Added to handle directory creation

# Constants
MAX_SPEED = 4      # m/s
MAX_YAW = 15       # degrees/s

# PID Tuning
P_YAW = 0.02
I_YAW = 0
D_YAW = 0

P_ROLL = 0.22
I_ROLL = 0
D_ROLL = 0

# Global State
pidYaw = None
pidRoll = None
movementYawAngle = 0
movementRollAngle = 0
inputValueYaw = 0
inputValueVelocityX = 0
flight_altitude = 2.0

debug_yaw = None
debug_velocity = None

def configure_PID(control_type):
    global pidRoll, pidYaw
    print(f"Configuring {control_type} control")

    if control_type == 'PID':
        pidYaw = PID(P_YAW, I_YAW, D_YAW, setpoint=0)
        pidRoll = PID(P_ROLL, I_ROLL, D_ROLL, setpoint=0)
    else:
        pidYaw = PID(P_YAW, 0, 0, setpoint=0)
        pidRoll = PID(P_ROLL, 0, 0, setpoint=0)

    pidYaw.output_limits = (-MAX_YAW, MAX_YAW)
    pidRoll.output_limits = (-MAX_SPEED, MAX_SPEED)

def connect_drone(drone_location):
    drone.connect_drone(drone_location)

def setXdelta(XDelta):
    global inputValueYaw
    inputValueYaw = XDelta

def getMovementYawAngle():
    return movementYawAngle

def setZDelta(ZDelta):
    global inputValueVelocityX
    inputValueVelocityX = ZDelta

def getMovementVelocityXCommand():
    return movementRollAngle

def set_flight_altitude(alt):
    global flight_altitude
    flight_altitude = alt

def initialize_debug_logs(DEBUG_FILEPATH):
    global debug_yaw, debug_velocity

    #Ensure directory exists
    directory = os.path.dirname(DEBUG_FILEPATH)
    if directory != '' and not os.path.exists(directory):
        os.makedirs(directory)

    debug_yaw = open(DEBUG_FILEPATH + "_yaw.txt", "a")
    debug_yaw.write("P: I: D: Error: command:\n")

    debug_velocity = open(DEBUG_FILEPATH + "_velocity.txt", "a")
    debug_velocity.write("P: I: D: Error: command:\n")

def debug_writer_YAW(value):
    if debug_yaw:
        debug_yaw.write(f"0,0,0,{inputValueYaw},{value}\n")

def debug_writer_ROLL(value):
    if debug_velocity:
        debug_velocity.write(f"0,0,0,{inputValueVelocityX},{value}\n")

def control_drone():
    global movementYawAngle, movementRollAngle

    # Yaw Control
    if inputValueYaw == 0 or pidYaw is None:
        movementYawAngle = 0.0
        drone.send_movement_command_YAW(0.0)
    else:
        yaw_output = pidYaw(inputValueYaw)
        movementYawAngle = -float(yaw_output if yaw_output is not None else 0.0)
        drone.send_movement_command_YAW(movementYawAngle)
        debug_writer_YAW(movementYawAngle)

    # Roll (Forward-Backward) Control
    if inputValueVelocityX == 0 or pidRoll is None:
        movementRollAngle = 0.0
        drone.send_movement_command_XYA(0.0, 0.0, flight_altitude)
    else:
        roll_output = pidRoll(inputValueVelocityX)
        movementRollAngle = -float(roll_output if roll_output is not None else 0.0)
        drone.send_movement_command_XYA(movementRollAngle, 0.0, flight_altitude)
        debug_writer_ROLL(movementRollAngle)

def arm_and_takeoff(altitude):
    drone.arm_and_takeoff(altitude)

def land():
    drone.land()

def print_drone_report():
    print("Battery Info:", drone.get_battery_info())
    print("EKF OK:", drone.get_EKF_status())

def stop_drone():
    drone.send_movement_command_YAW(0)
    drone.send_movement_command_XYA(0, 0, flight_altitude)

def set_system_state(state):
    print(f"System state set to {state}")
def disarm():
    from modules import drone
    try:
        drone.disarm()
        print("[INFO] Disarm command sent.")
    except Exception as e:
        print(f"[WARN] Disarm failed or drone already disarmed: {e}")


