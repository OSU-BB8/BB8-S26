import socket
import serial
import time

# ============================================================
# SETTINGS
# ============================================================

# Head Pi -> Body Pi UDP
UDP_PORT = 5005

# Body Pi -> Teensy
TEENSY_PORT = "/dev/ttyACM0"
TEENSY_BAUD = 115200

# If we haven't heard from the head for this long,
# command BB-8 to stay still.
HEAD_TIMEOUT = 0.5       # seconds

# How often to send commands to the Teensy
SEND_RATE = 20.0         # Hz


# ============================================================
# UDP SETUP
# ============================================================

udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_socket.bind(("0.0.0.0", UDP_PORT))

# Non-blocking so we can keep talking to the Teensy
# even when no UDP packet is arriving.
udp_socket.setblocking(False)

print(f"[UDP] Listening on port {UDP_PORT}")


# ============================================================
# TEENSY SETUP
# ============================================================

print(f"[TEENSY] Connecting to {TEENSY_PORT}...")

teensy = serial.Serial(
    TEENSY_PORT,
    TEENSY_BAUD,
    timeout=0
)

# Teensy often resets when serial port opens
time.sleep(2.0)

print("[TEENSY] Connected")


# ============================================================
# TARGET STATE
# ============================================================

target_distance = 0.0
target_angle = 0.0

last_head_packet = None

head_connected = False


# ============================================================
# TEENSY COMMAND
# ============================================================

def send_target(distance, angle):
    """
    Send distance and angle to Teensy.
    """

    message = f"{distance:.2f} {angle:.2f}\n"

    teensy.write(message.encode())


def send_still():
    """
    Command BB-8 to remain still.
    """

    send_target(0.0, 0.0)


# ============================================================
# MAIN LOOP
# ============================================================

send_interval = 1.0 / SEND_RATE
last_send = 0.0

print("[SYSTEM] Waiting for Head Pi...")


try:

    while True:

        now = time.monotonic()

        # ----------------------------------------------------
        # CHECK FOR UDP DATA
        # ----------------------------------------------------

        try:

            while True:

                data, address = udp_socket.recvfrom(1024)

                message = data.decode().strip()

                # Expected:
                #
                # 12.5,-30.0

                parts = message.split(",")

                if len(parts) != 2:
                    print(f"[UDP] Invalid packet: {message}")
                    continue

                try:

                    new_distance = float(parts[0])
                    new_angle = float(parts[1])

                except ValueError:

                    print(f"[UDP] Invalid numbers: {message}")
                    continue


                # Packet is valid
                target_distance = new_distance
                target_angle = new_angle

                last_head_packet = now


                # Connection restored
                if not head_connected:

                    print(
                        f"[HEAD] Connected: {address[0]}"
                    )

                    head_connected = True


        except BlockingIOError:
            pass


        # ----------------------------------------------------
        # CHECK HEAD PI TIMEOUT
        # ----------------------------------------------------

        if last_head_packet is None:

            head_alive = False

        else:

            head_alive = (
                now - last_head_packet
                <= HEAD_TIMEOUT
            )


        if not head_alive:

            if head_connected:

                print(
                    f"[HEAD] COMMUNICATION LOST "
                    f"(>{HEAD_TIMEOUT:.2f}s)"
                )

                print("[SAFETY] Commanding STILL")

                head_connected = False


        # ----------------------------------------------------
        # SEND COMMAND TO TEENSY
        # ----------------------------------------------------

        if now - last_send >= send_interval:

            if head_alive:

                send_target(
                    target_distance,
                    target_angle
                )

                print(
                    f"\rTARGET | "
                    f"D={target_distance:6.2f} ft | "
                    f"A={target_angle:7.2f} deg",
                    end="",
                    flush=True
                )

            else:

                send_still()

            last_send = now


        # Prevent CPU from running at 100%
        time.sleep(0.001)


# ============================================================
# SHUTDOWN
# ============================================================

except KeyboardInterrupt:

    print("\n[SHUTDOWN] Stopping BB-8")

    # Send several still commands just to be safe
    for _ in range(5):

        send_still()
        time.sleep(0.02)


finally:

    udp_socket.close()
    teensy.close()

    print("[SHUTDOWN] Complete")