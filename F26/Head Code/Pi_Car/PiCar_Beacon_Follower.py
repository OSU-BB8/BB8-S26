#!/usr/bin/env python3
"""
PiCar-X UWB beacon follower.

Behavior:
- Reads Makerfabs UWB beacon data from serial.
- Uses beacon distance and lateral X offset to steer toward the beacon.
- Drives forward until TARGET_DISTANCE_CM.
- Backs up if it gets too close.
- Stops if UWB data becomes stale.
- Stops for an ultrasonic obstacle.
- Prints beacon data and drive commands to the terminal.

This is intentionally UWB-only for initial testing.
"""

import json
import math
import serial
import threading
import time

from picarx import Picarx


# ============================================================
# SETTINGS
# ============================================================

# PiCar-X
DRIVE_SPEED = 50
BACKUP_SPEED = 20
MAX_STEERING = 30

# Desired beacon following distance
TARGET_DISTANCE_CM = 150
DISTANCE_TOLERANCE_CM = 15

# Do not steer for tiny left/right errors
UWB_X_TOLERANCE_CM = 15

# If calculated angle is larger than this, command max steering.
# Smaller values make steering more aggressive.
ANGLE_FOR_MAX_STEERING_DEG = 45.0

# Ultrasonic emergency stop
STOP_DISTANCE_CM = 20

# UWB receiver
UWB_PORT = "/dev/ttyACM0"
UWB_BAUD = 115200
UWB_TIMEOUT_SECONDS = 1.0

# Terminal print rate
PRINT_PERIOD = 0.10


# ============================================================
# START PICAR-X
# ============================================================

px = Picarx()
px.stop()
px.set_dir_servo_angle(0)


# ============================================================
# SHARED UWB DATA
# ============================================================

uwb_lock = threading.Lock()

uwb_distance_cm = None
uwb_x_cm = None
uwb_y_cm = None
uwb_angle_deg = None
uwb_last_update = 0.0
uwb_raw_line = None


# ============================================================
# UWB READER
# ============================================================

def calculate_angle_deg(x_cm, y_cm, distance_cm):
    """
    Estimate target angle from the UWB X/Y coordinates.

    0 degrees = straight ahead
    negative/positive = opposite steering directions

    If Ycm is available, atan2(X, Y) gives a useful target angle.
    If Ycm is unavailable, estimate angle from X and distance.
    """

    if x_cm is None:
        return 0.0

    if y_cm is not None:
        return math.degrees(math.atan2(float(x_cm), float(y_cm)))

    if distance_cm is not None and float(distance_cm) > 0:
        ratio = float(x_cm) / float(distance_cm)
        ratio = max(-1.0, min(1.0, ratio))
        return math.degrees(math.asin(ratio))

    return 0.0


def uwb_reader_thread():
    global uwb_distance_cm
    global uwb_x_cm
    global uwb_y_cm
    global uwb_angle_deg
    global uwb_last_update
    global uwb_raw_line

    try:
        ser = serial.Serial(
            UWB_PORT,
            UWB_BAUD,
            timeout=0.1
        )

        print(f"[UWB] Serial opened: {UWB_PORT} @ {UWB_BAUD}")

    except Exception as exc:
        print(f"[UWB] SERIAL FAILED TO OPEN: {exc}")
        return

    while True:
        try:
            line = ser.readline().decode(
                "utf-8",
                errors="ignore"
            ).strip()

            if not line:
                continue

            # Uncomment this if you want EVERY serial line printed:
            # print("[UWB RAW]", line)

            if "{" not in line:
                continue

            json_str = line[line.index("{"):]

            packet = json.loads(json_str)

            if "TWR" not in packet:
                continue

            data = packet["TWR"]

            distance_cm = float(data["D"])
            x_cm = float(data["Xcm"])

            # Some firmware versions provide Ycm, some may not.
            y_value = data.get("Ycm")
            y_cm = float(y_value) if y_value is not None else None

            angle_deg = calculate_angle_deg(
                x_cm,
                y_cm,
                distance_cm
            )

            with uwb_lock:
                uwb_distance_cm = distance_cm
                uwb_x_cm = x_cm
                uwb_y_cm = y_cm
                uwb_angle_deg = angle_deg
                uwb_last_update = time.monotonic()
                uwb_raw_line = line

        except (ValueError, KeyError, json.JSONDecodeError):
            # Bad/incomplete serial packet; ignore it.
            continue

        except serial.SerialException as exc:
            print(f"[UWB] Serial error: {exc}")
            return

        except Exception as exc:
            print(f"[UWB] Unexpected read error: {exc}")
            time.sleep(0.1)


threading.Thread(
    target=uwb_reader_thread,
    daemon=True
).start()


# ============================================================
# CONTROL FUNCTIONS
# ============================================================

def get_steering(angle_deg, x_cm):
    """
    Convert beacon angle into PiCar-X steering.

    X tolerance prevents the car from constantly hunting when
    the beacon is approximately centered.
    """

    if abs(x_cm) <= UWB_X_TOLERANCE_CM:
        return 0

    steering = -(
        angle_deg / ANGLE_FOR_MAX_STEERING_DEG
    ) * MAX_STEERING

    steering = max(
        -MAX_STEERING,
        min(MAX_STEERING, steering)
    )

    return int(round(steering))


def stop_and_center():
    px.stop()
    px.set_dir_servo_angle(0)


# ============================================================
# MAIN LOOP
# ============================================================

print("==============================================")
print(" PiCar-X UWB BEACON FOLLOWER")
print("==============================================")
print(f"Target distance : {TARGET_DISTANCE_CM} cm")
print(f"Drive speed     : {DRIVE_SPEED}")
print(f"Max steering    : +/-{MAX_STEERING} deg")
print(f"UWB timeout     : {UWB_TIMEOUT_SECONDS:.2f} s")
print("Press CTRL+C to stop")
print("==============================================")

last_print = 0.0
last_state = None

try:
    while True:

        now = time.monotonic()

        # ----------------------------------------------------
        # READ ULTRASONIC SENSOR
        # ----------------------------------------------------

        try:
            obstacle_distance = px.ultrasonic.read()
        except Exception:
            obstacle_distance = 999

        # ----------------------------------------------------
        # COPY CURRENT UWB DATA
        # ----------------------------------------------------

        with uwb_lock:
            distance_cm = uwb_distance_cm
            x_cm = uwb_x_cm
            y_cm = uwb_y_cm
            angle_deg = uwb_angle_deg
            last_update = uwb_last_update
            raw_line = uwb_raw_line

        uwb_age = now - last_update

        uwb_fresh = (
            distance_cm is not None
            and x_cm is not None
            and angle_deg is not None
            and uwb_age < UWB_TIMEOUT_SECONDS
        )

        # ----------------------------------------------------
        # SAFETY: OBSTACLE
        # ----------------------------------------------------

        if (
            obstacle_distance > 0
            and obstacle_distance < STOP_DISTANCE_CM
        ):
            stop_and_center()

            steering = 0
            state = "OBSTACLE STOP"

        # ----------------------------------------------------
        # FAILSAFE: BEACON LOST
        # ----------------------------------------------------

        elif not uwb_fresh:
            stop_and_center()

            steering = 0
            state = "BEACON LOST - STOPPED"

        # ----------------------------------------------------
        # BEACON FOLLOWING
        # ----------------------------------------------------

        else:
            steering = get_steering(
                angle_deg,
                x_cm
            )

            px.set_dir_servo_angle(steering)

            # Too far away -> drive toward beacon
            if distance_cm > (
                TARGET_DISTANCE_CM
                + DISTANCE_TOLERANCE_CM
            ):
                px.forward(DRIVE_SPEED)
                state = "FORWARD"

            # Too close -> back away
            elif distance_cm < (
                TARGET_DISTANCE_CM
                - DISTANCE_TOLERANCE_CM
            ):
                px.backward(BACKUP_SPEED)
                state = "BACKUP"

            # Correct distance -> remain stopped
            else:
                px.stop()
                state = "TARGET REACHED"

        # ----------------------------------------------------
        # TERMINAL DEBUG
        # ----------------------------------------------------

        if now - last_print >= PRINT_PERIOD:

            if uwb_fresh:
                y_text = (
                    f"{y_cm:7.1f}"
                    if y_cm is not None
                    else "   N/A "
                )

                print(
                    f"[BEACON] "
                    f"D={distance_cm:7.1f} cm | "
                    f"X={x_cm:7.1f} cm | "
                    f"Y={y_text} cm | "
                    f"Angle={angle_deg:7.1f} deg | "
                    f"Steer={steering:3d} deg | "
                    f"State={state} | "
                    f"Ultrasonic={obstacle_distance:6.1f} cm"
                )

            else:
                if last_update == 0:
                    age_text = "never"
                else:
                    age_text = f"{uwb_age:.2f}s"

                print(
                    f"[BEACON] NO FRESH DATA | "
                    f"Age={age_text} | "
                    f"State={state}"
                )

            last_print = now

        # Print state transitions separately so they are obvious.
        if state != last_state:
            print(f"[DRIVE] {state}")
            last_state = state

        time.sleep(0.02)


except KeyboardInterrupt:
    print("\nStopping PiCar-X...")


finally:
    stop_and_center()
    print("PiCar-X stopped safely.")
