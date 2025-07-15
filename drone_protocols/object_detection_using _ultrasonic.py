import time
import RPi.GPIO as GPIO
from pymavlink import mavutil
import serial

MAX_DISTANCE = 200  
HEARTBEAT_INTERVAL = 1.0 
UART_PORT = '/dev/serial0'  # UART port for Pixhawk (or '/dev/ttyS0' or '/dev/ttyUSB0')
BAUD_RATE = 57600  

SENSORS = {
    'front': {'trigger': 23, 'echo': 24},
    'back': {'trigger': 17, 'echo': 18},
    'left': {'trigger': 27, 'echo': 22},
    'right': {'trigger': 5, 'echo': 6}
}

GPIO.setmode(GPIO.BCM)
for sensor in SENSORS.values():
    GPIO.setup(sensor['trigger'], GPIO.OUT)
    GPIO.setup(sensor['echo'], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Initialize MAVLink connection
connection = mavutil.mavlink_connection(UART_PORT, baud=BAUD_RATE)

def measure_distance(trigger_pin, echo_pin):
    GPIO.output(trigger_pin, True)
    time.sleep(0.00001)  
    GPIO.output(trigger_pin, False)
    
    start_time = time.time()
    while GPIO.input(echo_pin) == 0 and (time.time() - start_time) < 0.1:
        pass
    pulse_start = time.time()
    
    while GPIO.input(echo_pin) == 1 and (time.time() - pulse_start) < 0.1:
        pass
    pulse_end = time.time()
    
    # Calculate distance
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  
    distance = round(distance, 2)
    
    if distance <= 0 or distance > MAX_DISTANCE:
        return 0
    return distance

def calculate_pitch_roll(distances):
    pitch = 1500  
    roll = 1500  
    
    front_dist = distances['front']
    back_dist = distances['back']
    if front_dist != 0 and front_dist < MAX_DISTANCE:
        pitch = 1500 + 30 + ((200 - front_dist) )  # Forward pitch
    elif back_dist != 0 and back_dist < MAX_DISTANCE:
        pitch = 1500 - (30 + ((200 - back_dist) ))  # Backward pitch
    
    left_dist = distances['left']
    right_dist = distances['right']
    if left_dist != 0 and left_dist < MAX_DISTANCE:
        roll = 1500 + 30 + ((200 - left_dist) )  # Left roll
    elif right_dist != 0 and right_dist < MAX_DISTANCE:
        roll = 1500 - (30 + ((200 - right_dist) ))  # Right roll
    
    return int(pitch), int(roll)

def send_heartbeat():
    connection.mav.heartbeat_send(
        type=mavutil.mavlink.MAV_TYPE_QUADROTOR,
        autopilot=mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
        base_mode=0,
        custom_mode=1,
        system_status=0,
        mavlink_version=3
    )

def send_rc_override(pitch, roll):
    if pitch != 1500 or roll != 1500:  
        connection.mav.rc_channels_override_send(
            connection.target_system,  
            0,  
            roll,  # Channel 1 (roll)
            pitch,  # Channel 2 (pitch)
            0,  # Channel 3 (throttle)
            0,  # Channel 4 (yaw)
            0, 0, 0, 0, 0, 0  
        )
    else:
        connection.mav.rc_channels_override_send(
            connection.target_system, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0  # Reset channels
        )

def main():
    last_heartbeat = 0
    while True:
        current_time = time.time()
        if current_time - last_heartbeat > HEARTBEAT_INTERVAL:
            send_heartbeat()
            last_heartbeat = current_time
        
        distances = {}
        for name, pins in SENSORS.items():
            distances[name] = measure_distance(pins['trigger'], pins['echo'])
        
        pitch, roll = calculate_pitch_roll(distances)
        
        send_rc_override(pitch, roll)
        
        print(f"Distances: Front={distances['front']}cm, Back={distances['back']}cm, "
              f"Left={distances['left']}cm, Right={distances['right']}cm, "
              f"Pitch={pitch}, Roll={roll}")
        
        time.sleep(0.1)

if __name__ == "__main__":
    main()