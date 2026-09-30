import socket
import threading
import time

BODY_PI_IP = "10.42.0.27"
PORT = 5005
SEND_RATE = 0.05       # 20 Hz

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

current_target = None
target_lock = threading.Lock()
running = True


def sender_loop():
    global running

    while running:
        with target_lock:
            target = current_target

        if target is not None:
            distance, angle = target

            message = f"{distance:.2f},{angle:.2f}"
            sock.sendto(
                message.encode(),
                (BODY_PI_IP, PORT)
            )

        time.sleep(SEND_RATE)


# Start continuous sender
sender = threading.Thread(
    target=sender_loop,
    daemon=True
)
sender.start()


print("BB-8 Head Pi Target Sender")
print("Enter: distance angle")
print("Example: 12.5 -37.2")
print("The target will be continuously sent at 20 Hz.")
print("Type 'stop' to stop sending.")
print("Ctrl+C to quit.")


try:
    while True:
        command = input("> ").strip()

        if command.lower() == "stop":
            with target_lock:
                current_target = None

            print("Stopped sending target.")
            continue

        parts = command.split()

        if len(parts) != 2:
            print("Enter two numbers: distance_ft angle_deg")
            continue

        try:
            distance = float(parts[0])
            angle = float(parts[1])

        except ValueError:
            print("Invalid input. Enter two numbers.")
            continue

        with target_lock:
            current_target = (distance, angle)

        print(
            f"Target set: "
            f"D={distance:.2f} ft, "
            f"A={angle:.1f} deg"
        )

except KeyboardInterrupt:
    print("\nStopping sender.")

finally:
    running = False
    sender.join(timeout=0.2)
    sock.close()