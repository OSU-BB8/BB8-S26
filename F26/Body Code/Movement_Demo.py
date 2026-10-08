#!/usr/bin/env python3

import serial
import time

# -----------------------------
# Teensy serial connection
# -----------------------------

PORT = "/dev/serial0"
BAUD = 115200

SEND_PERIOD = 0.05     # 20 Hz

MOVE_TIME = 0.2
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
    ("FORWARD",       DISTANCE,    0),
    ("FORWARD",       DISTANCE,    0),
    ("FORWARD",       DISTANCE,    0),
    ("FORWARD",       DISTANCE,    0),
    ("FORWARD",       DISTANCE,    0),
    ("FORWARD",       DISTANCE,    0),
    ("FORWARD",       DISTANCE,    0),
    ("FORWARD",       DISTANCE,    0),
    ("FORWARD",       DISTANCE,    0),
    ("FORWARD",       DISTANCE,    0),
    ("FORWARD",       DISTANCE,    0),
    ("FORWARD",       DISTANCE,    0),
    ("FORWARD",       DISTANCE,    0),
    ("FORWARD",       DISTANCE,    0),
    ("FORWARD",       DISTANCE,    0),
    ("FORWARD",       DISTANCE,    0),
    ("FRONT RIGHT",   DISTANCE,   0),
    ("FRONT RIGHT",   DISTANCE,   5),
    ("FRONT RIGHT",   DISTANCE,   10),
    ("FRONT RIGHT",   DISTANCE,   15),
    ("FRONT RIGHT",   DISTANCE,   20),
    ("FRONT RIGHT",   DISTANCE,   25),
    ("FRONT RIGHT",   DISTANCE,   30),
    ("FRONT RIGHT",   DISTANCE,   35),
    ("FRONT RIGHT",   DISTANCE,   40),
    ("FRONT RIGHT",   DISTANCE,   45),
    ("FRONT RIGHT",   DISTANCE,   50),
    ("FRONT RIGHT",   DISTANCE,   55),
    ("FRONT RIGHT",   DISTANCE,   60),
    ("FRONT RIGHT",   DISTANCE,   65),
    ("FRONT RIGHT",   DISTANCE,   70),
    ("FRONT RIGHT",   DISTANCE,   75),
    ("RIGHT",         DISTANCE,   90),
    ("RIGHT",         DISTANCE,   90),
    ("RIGHT",         DISTANCE,   90),
    ("RIGHT",         DISTANCE,   90),
    ("RIGHT",         DISTANCE,   90),
    ("RIGHT",         DISTANCE,   90),
    ("RIGHT",         DISTANCE,   90),
    ("RIGHT",         DISTANCE,   90),
    ("RIGHT",         DISTANCE,   90),
    ("RIGHT",         DISTANCE,   90),
    ("RIGHT",         DISTANCE,   90),
    ("RIGHT",         DISTANCE,   90),
    ("RIGHT",         DISTANCE,   90),
    ("RIGHT",         DISTANCE,   90),
    ("RIGHT",         DISTANCE,   90),
    ("RIGHT",         DISTANCE,   90),
    ("FORWARD",       DISTANCE,    0),
    ("FORWARD",       DISTANCE,    0),
    ("FORWARD",       DISTANCE,    0),
    ("FORWARD",       DISTANCE,    0),
    ("FORWARD",       DISTANCE,    0),
    ("FORWARD",       DISTANCE,    0),
    ("FORWARD",       DISTANCE,    0),
    ("FORWARD",       DISTANCE,    0),
    ("FORWARD",       DISTANCE,    0),
    ("FORWARD",       DISTANCE,    0),
    ("FORWARD",       DISTANCE,    0),
    ("FORWARD",       DISTANCE,    0),
    ("FORWARD",       DISTANCE,    0),
    ("FORWARD",       DISTANCE,    0),
    ("FORWARD",       DISTANCE,    0),
    ("FORWARD",       DISTANCE,    0),
    ("FORWARD",       DISTANCE,    0),
    ("FORWARD",       DISTANCE,    0),
    ("FORWARD",       DISTANCE,    0),
    ("FORWARD",       DISTANCE,    0),
    ("FRONT LEFT",    DISTANCE,  -5),
    ("FRONT LEFT",    DISTANCE,  -10),
    ("FRONT LEFT",    DISTANCE,  -15),
    ("FRONT LEFT",    DISTANCE,  -20),
    ("FRONT LEFT",    DISTANCE,  -25),
    ("FRONT LEFT",    DISTANCE,  -30),
    ("FRONT LEFT",    DISTANCE,  -35),
    ("FRONT LEFT",    DISTANCE,  -40),
    ("FRONT LEFT",    DISTANCE,  -45),
    ("FRONT LEFT",    DISTANCE,  -50),
    ("FRONT LEFT",    DISTANCE,  -55),
    ("FRONT LEFT",    DISTANCE,  -60),
    ("FRONT LEFT",    DISTANCE,  -65),
    ("FRONT LEFT",    DISTANCE,  -70),
    ("FRONT LEFT",    DISTANCE,  -75),
    ("LEFT",    DISTANCE,  -90),
    ("LEFT",    DISTANCE,  -90),
    ("LEFT",    DISTANCE,  -90),
    ("LEFT",    DISTANCE,  -90),
    ("LEFT",    DISTANCE,  -90),
    ("LEFT",    DISTANCE,  -90),
    ("LEFT",    DISTANCE,  -90),
    ("LEFT",    DISTANCE,  -90),
    ("LEFT",    DISTANCE,  -90),
    ("LEFT",    DISTANCE,  -90),
    ("LEFT",    DISTANCE,  -90),
    ("LEFT",    DISTANCE,  -90),
    ("LEFT",    DISTANCE,  -90),
    ("LEFT",    DISTANCE,  -90),
    ("LEFT",    DISTANCE,  -90),
    ("LEFT",    DISTANCE,  -90),
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