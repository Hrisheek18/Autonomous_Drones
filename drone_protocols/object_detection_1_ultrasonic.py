import time
import RPi.GPIO as GPIO
from pymavlink import mavutil
import serial

# Constants
MAX_DISTANCE = 70  # Max distance for ultrasonic sensor (cm)
TRIGGER_PIN = 23   # GPIO pin for trigger (BCM numbering)
ECHO_PIN = 24      # GPIO pin for echo (BCM numbering)
HEARTBEAT_INTERVAL = 1.0  # Heartbeat every 1 second
UART_PORT = '/dev/serial0'  # UART port for Pixhawk (or '/dev/ttyS0' or '/dev/ttyUSB0')
BAUD_RATE = 57600  # Match ESP32's 57600 baud

# GPIO setup for ultrasonic sensor
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIGGER_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Initialize MAVLink connection
connection = mavutil.mavlink_connection(UART_PORT, baud=BAUD_RATE)

def measure_distance():
    """Measure distance using HC-SR04 ultrasonic sensor with separate trigger/echo pins."""
    # Send trigger pulse
    GPIO.output(TRIGGER_PIN, True)
    time.sleep(0.00001)  # 10us pulse
    GPIO.output(TRIGGER_PIN, False)
    
    # Wait for echo start
    start_time = time.time()
    while GPIO.input(ECHO_PIN) == 0 and (time.time() - start_time) < 0.1:
        pass
    pulse_start = time.time()
    
    # Wait for echo end
    while GPIO.input(ECHO_PIN) == 1 and (time.time() - pulse_start) < 0.1:
        pass
    pulse_end = time.time()
    
    # Calculate distance
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # Speed of sound: 343m/s = 17150cm/s
    distance = round(distance, 2)
    
    if distance <= 0 or distance > MAX_DISTANCE:
        return 0
    return distance

def calculate_pitch(distance):
    """Calculate pitch value based on distance."""
    if distance != 0 and distance < MAX_DISTANCE:
        pitch_back = 1500 + 30 + ((70 - distance) * 6)  # Same formula as original
        return int(pitch_back)
    return 1500  # Neutral pitch

def send_heartbeat():
    """Send MAVLink heartbeat message."""
    connection.mav.heartbeat_send(
        type=mavutil.mavlink.MAV_TYPE_QUADROTOR,
        autopilot=mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
        base_mode=0,
        custom_mode=1,
        system_status=0,
        mavlink_version=3
    )

def send_rc_override(pitch):
    """Send RC channel override for pitch (channel 2)."""
    if pitch != 1500:  # Only send override if pitch is not neutral (indicating valid distance)
        connection.mav.rc_channels_override_send(
            connection.target_system,  # Target system ID (1 for Pixhawk)
            0,  # Target component ID
            0,  # Channel 1 (roll)
            pitch,  # Channel 2 (pitch)
            0,  # Channel 3 (throttle)
            0,  # Channel 4 (yaw)
            0, 0, 0, 0, 0, 0  # Channels 5-10
        )
    else:
        connection.mav.rc_channels_override_send(
            connection.target_system, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0  # Reset channels
        )

def main():
    last_heartbeat = 0
    while True:
        # Send heartbeat every 1 second
        current_time = time.time()
        if current_time - last_heartbeat > HEARTBEAT_INTERVAL:
            send_heartbeat()
            last_heartbeat = current_time
        
        # Measure distance and calculate pitch
        distance = measure_distance()
        output_pitch = calculate_pitch(distance)
        
        # Send RC override
        send_rc_override(output_pitch)
        
        # Log data for debugging
        print(f"Distance: {distance} cm, Pitch: {output_pitch}")
        
        # Small delay to avoid overwhelming CPU
        time.sleep(0.1)

if __name__ == "__main__":
    main()