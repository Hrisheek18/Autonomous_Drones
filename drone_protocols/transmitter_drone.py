# drone1_sender.py
import socket

# GPS coordinates of detected object
object_lat = 12.3456789
object_lon = 76.5432100
object_alt = 10.0

# IP and port of Drone 2's Raspberry Pi
UDP_IP = "192.168.4.2"  # change to Pi2's IP
UDP_PORT = 5005

# Create socket and send data
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
message = f"{object_lat},{object_lon},{object_alt}"
sock.sendto(message.encode(), (UDP_IP, UDP_PORT))
print(f"[✓] Sent object coordinates: {message}")
