import socket
import time

PORT = 5005
TIMEOUT = 0.5

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", PORT))
sock.settimeout(0.1)

distance = 0.0
angle = 0.0
last_packet_time = 0

print(f"Listening for Head Pi on UDP port {PORT}...")

while True:
    try:
        data, address = sock.recvfrom(1024)

        message = data.decode().strip()

        distance_str, angle_str = message.split(",")

        distance = float(distance_str)
        angle = float(angle_str)

        last_packet_time = time.monotonic()

        print(
            f"Head Pi: distance={distance:.2f} ft  "
            f"angle={angle:.1f} deg"
        )

    except socket.timeout:
        pass

    # Communication failsafe
    if time.monotonic() - last_packet_time > TIMEOUT:
        distance = 0.0
        angle = 0.0

        print("HEAD PI COMMUNICATION LOST")