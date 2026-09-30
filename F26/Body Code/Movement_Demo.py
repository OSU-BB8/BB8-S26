#!/usr/bin/env python3

import serial
import time

# -----------------------------
# Teensy serial connection
# -----------------------------

PORT = "/dev/serial0"
BAUD = 115200

SEND_PERIOD = 0.05     # 20 Hz

MOVE_TIME = 1
STOP_TIME = 0.2

DISTANCE = 20


# -----------------------------
# Demo sequence
# -----------------------------

movements = [
    ("FORWARD",       DISTANCE,    0),
    ("FORWARD",       DISTANCE,    0),
    ("FORWARD",       DISTANCE,    0),
    ("FORWARD",       DISTANCE,    0),
    ("FRONT RIGHT",   DISTANCE,   10),
    ("FRONT RIGHT",   DISTANCE,   20),
    ("FRONT RIGHT",   DISTANCE,   30),
    ("FRONT RIGHT",   DISTANCE,   40),
    ("RIGHT",         DISTANCE,   90),
    ("RIGHT",         DISTANCE,   90),
    ("RIGHT",         DISTANCE,   90),
    ("RIGHT",         DISTANCE,   90),
    ("FORWARD",       DISTANCE,    0),
    ("FORWARD",       DISTANCE,    0),
    ("FORWARD",       DISTANCE,    0),
    ("FORWARD",       DISTANCE,    0),
    ("FRONT LEFT",          DISTANCE,  -10),
    ("FRONT LEFT",          DISTANCE,  -20),
    ("FRONT LEFT",          DISTANCE,  -30),
    ("FRONT LEFT",          DISTANCE,  -40),
    ("LEFT",    DISTANCE,  -90),
    ("LEFT",    DISTANCE,  -90),
    ("LEFT",    DISTANCE,  -90),
    ("LEFT",    DISTANCE,  -90),
]


def send_target(ser, distance, angle):

    packet = f"T,{distance:.3f},{angle:.3f}\n"

    ser.write(packet.encode("ascii"))


def run_target(ser, name, distance, angle, duration):

    print(
        f"{name:<15} | "
        f"D={distance:5.1f} ft | "
        f"A={angle:6.1f} deg"
    )

    start = time.monotonic()

    while time.monotonic() - start < duration:

        send_target(
            ser,
            distance,
            angle
        )

        time.sleep(SEND_PERIOD)


def stop_bb8(ser, duration):

    print("STOP")

    # Explicitly tell Teensy to enter stopped state
    ser.write(b"STOP\n")
    ser.flush()

    time.sleep(duration)


def main():

    print("Opening Teensy serial connection...")

    ser = serial.Serial(
        port=PORT,
        baudrate=BAUD,
        timeout=0.05,
        write_timeout=0.1
    )

    # Give serial connection a moment to initialize
    time.sleep(1)

    print()
    print("==============================")
    print(" BB-8 MOVEMENT DEMONSTRATION")
    print("==============================")
    print()
    print("Press Ctrl+C to stop.")
    print()

    try:

        while True:

            for name, distance, angle in movements:

                run_target(
                    ser,
                    name,
                    distance,
                    angle,
                    MOVE_TIME
                )

                # stop_bb8(
                #     ser,
                #     STOP_TIME
                # )

    except KeyboardInterrupt:

        print("\nDemo interrupted.")

    finally:

        print("STOPPING BB-8")

        try:
            ser.write(b"STOP\n")
            ser.flush()
        except Exception:
            pass

        ser.close()

        print("Demo ended.")


if __name__ == "__main__":
    main()