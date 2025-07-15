#RTL
from pymavlink import mavutil

connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
connection.wait_heartbeat()

connection.set_mode_apm("RTL")
print("Return to Launch mode set")
