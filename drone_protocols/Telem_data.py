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
