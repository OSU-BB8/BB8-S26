#!/usr/bin/env python3

import serial
import socket
import json
import time
import statistics
import math
from collections import deque


# ============================================================
# SETTINGS
# ============================================================

# UWB receiver
BEACON_PORT = "/dev/ttyACM0"
BEACON_BAUD = 115200

# Body Pi
BODY_PI_IP = "10.42.0.27"
UDP_PORT = 5005

# Unit conversion
CM_TO_FT = 0.0328084

# UWB P angle correction
ANGLE_DIVISOR = 3.0


# ============================================================
# FILTER SETTINGS
# ============================================================

# Number of recent measurements used for averaging
WINDOW_SIZE = 15

# Reject measurements too far from the current group
MAX_DISTANCE_ERROR_FT = 2.0
MAX_ANGLE_ERROR_DEG = 30.0

# Need this many good samples before sending a target
MIN_GOOD_SAMPLES = 5


# ============================================================
# BEACON LOSS SETTINGS
# ============================================================

# If no valid UWB packet is received for this long,
# consider the beacon lost.
BEACON_TIMEOUT = 0.5


# ============================================================
# TRANSMISSION SETTINGS
# ============================================================

# Send averaged position to Body Pi at 5 Hz
SEND_RATE_HZ = 5
SEND_PERIOD = 1.0 / SEND_RATE_HZ


# ============================================================
# SAMPLE STORAGE
# ============================================================

distance_samples = deque(maxlen=WINDOW_SIZE)
angle_samples = deque(maxlen=WINDOW_SIZE)


# ============================================================
# BEACON STATE
# ============================================================

last_beacon_time = None

# Start in lost state until the first valid UWB packet arrives
beacon_lost = True


# ============================================================
# ANGLE FUNCTIONS
# ============================================================

def angle_difference(a, b):
    """
    Return the smallest difference between two angles.

    Example:
        179 and -179 are only 2 degrees apart.
    """

    return (a - b + 180) % 360 - 180


def circular_average(angles):
    """
    Average angles correctly across the +/-180 degree boundary.
    """

    if not angles:
        return None

    sin_sum = sum(
        math.sin(math.radians(a))
        for a in angles
    )

    cos_sum = sum(
        math.cos(math.radians(a))
        for a in angles
    )

    return math.degrees(
        math.atan2(
            sin_sum,
            cos_sum
        )
    )


# ============================================================
# FILTER
# ============================================================

def get_filtered_target():

    # Wait until enough samples exist
    if len(distance_samples) < MIN_GOOD_SAMPLES:
        return None


    # --------------------------------------------------------
    # DISTANCE FILTER
    # --------------------------------------------------------

    median_distance = statistics.median(
        distance_samples
    )

    good_distances = [
        distance
        for distance in distance_samples
        if abs(
            distance - median_distance
        ) <= MAX_DISTANCE_ERROR_FT
    ]


    # --------------------------------------------------------
    # ANGLE FILTER
    # --------------------------------------------------------

    angle_center = circular_average(
        angle_samples
    )

    good_angles = [
        angle
        for angle in angle_samples
        if abs(
            angle_difference(
                angle,
                angle_center
            )
        ) <= MAX_ANGLE_ERROR_DEG
    ]


    # --------------------------------------------------------
    # MAKE SURE ENOUGH GOOD DATA REMAINS
    # --------------------------------------------------------

    if len(good_distances) < MIN_GOOD_SAMPLES:
        return None

    if len(good_angles) < MIN_GOOD_SAMPLES:
        return None


    # --------------------------------------------------------
    # AVERAGE GOOD DATA
    # --------------------------------------------------------

    average_distance = statistics.mean(
        good_distances
    )

    average_angle = circular_average(
        good_angles
    )


    return (
        average_distance,
        average_angle,
        len(good_distances),
        len(good_angles)
    )


# ============================================================
# CONNECTION SETUP
# ============================================================

print("Opening UWB receiver...")


# IMPORTANT:
#
# Keep this timeout short.
#
# readline() must return frequently enough for the program
# to check whether the beacon has stopped sending data.

beacon = serial.Serial(
    BEACON_PORT,
    BEACON_BAUD,
    timeout=0.05
)


sock = socket.socket(
    socket.AF_INET,
    socket.SOCK_DGRAM
)


print()
print("======================================")
print(" BB-8 FILTERED UWB BEACON SENDER")
print("======================================")
print()

print(f"Beacon port:     {BEACON_PORT}")
print(f"Body Pi:         {BODY_PI_IP}:{UDP_PORT}")
print()

print(f"Filter window:   {WINDOW_SIZE} samples")
print(f"Minimum samples: {MIN_GOOD_SAMPLES}")

print(
    f"Distance filter: +/- "
    f"{MAX_DISTANCE_ERROR_FT} ft"
)

print(
    f"Angle filter:    +/- "
    f"{MAX_ANGLE_ERROR_DEG} deg"
)

print(
    f"Beacon timeout:  "
    f"{BEACON_TIMEOUT} sec"
)

print(
    f"Send rate:       "
    f"{SEND_RATE_HZ} Hz"
)

print()
print("Waiting for UWB beacon...")
print()


# ============================================================
# MAIN LOOP
# ============================================================

last_send_time = 0


try:

    while True:

        # ----------------------------------------------------
        # READ UWB
        # ----------------------------------------------------

        raw = beacon.readline()


        # ====================================================
        # NEW SERIAL DATA RECEIVED
        # ====================================================

        if raw:

            line = raw.decode(
                "utf-8",
                errors="ignore"
            ).strip()


            # ------------------------------------------------
            # FIND START OF JSON
            # ------------------------------------------------

            json_start = line.find("{")


            if json_start != -1:

                json_text = line[
                    json_start:
                ]


                try:

                    data = json.loads(
                        json_text
                    )


                    # ========================================
                    # VALID TWR PACKET
                    # ========================================

                    if "TWR" in data:

                        twr = data["TWR"]


                        # ------------------------------------
                        # READ RAW UWB DATA
                        # ------------------------------------

                        distance_cm = float(
                            twr["D"]
                        )

                        raw_angle = float(
                            twr["P"]
                        )


                        # ====================================
                        # VALID BEACON PACKET RECEIVED
                        # ====================================

                        now = time.monotonic()

                        last_beacon_time = now


                        # ------------------------------------
                        # BEACON JUST RETURNED
                        # ------------------------------------

                        if beacon_lost:

                            print()
                            print(
                                ">>> BEACON SIGNAL "
                                "RESTORED <<<"
                            )

                            print(
                                "Collecting fresh "
                                "measurements..."
                            )
                            print()


                            # Throw away old measurements
                            distance_samples.clear()
                            angle_samples.clear()


                        beacon_lost = False


                        # ====================================
                        # CONVERT UWB DATA
                        # ====================================

                        # centimeters -> feet
                        distance_ft = (
                            distance_cm
                            * CM_TO_FT
                        )


                        # UWB P reading is approximately
                        # 3x the real angle.
                        angle_deg = (
                            raw_angle
                            / ANGLE_DIVISOR
                        )


                        # ====================================
                        # BASIC SANITY CHECK
                        # ====================================

                        if (
                            0 <= distance_ft <= 50
                            and
                            -180 <= angle_deg <= 180
                        ):

                            distance_samples.append(
                                distance_ft
                            )

                            angle_samples.append(
                                angle_deg
                            )


                except (
                    json.JSONDecodeError,
                    KeyError,
                    ValueError,
                    TypeError
                ):

                    # Ignore malformed UWB packets
                    pass


        # ====================================================
        # BEACON WATCHDOG
        # ====================================================

        now = time.monotonic()


        if last_beacon_time is not None:

            time_since_beacon = (
                now - last_beacon_time
            )


            if (
                time_since_beacon
                > BEACON_TIMEOUT
            ):

                # Only print this once when the
                # transition to lost occurs.

                if not beacon_lost:

                    print()
                    print(
                        ">>> BEACON LOST <<<"
                    )

                    print(
                        f"No valid UWB data for "
                        f"{time_since_beacon:.2f} sec."
                    )

                    print(
                        "Stopping transmission "
                        "to Body Pi."
                    )
                    print()


                    # Remove all old measurements.
                    #
                    # This prevents BB-8 from acting
                    # on old beacon information when
                    # the beacon reconnects.

                    distance_samples.clear()
                    angle_samples.clear()


                beacon_lost = True


        # ====================================================
        # SEND TO BODY PI
        # ====================================================

        now = time.monotonic()


        if (
            now - last_send_time
            >= SEND_PERIOD
        ):


            # ------------------------------------------------
            # ONLY SEND WHILE BEACON IS CONNECTED
            # ------------------------------------------------

            if not beacon_lost:

                result = get_filtered_target()


                if result is not None:

                    (
                        distance_ft,
                        angle_deg,
                        good_distance_count,
                        good_angle_count
                    ) = result


                    # ----------------------------------------
                    # BUILD MESSAGE
                    # ----------------------------------------

                    message = (
                        f"{distance_ft:.3f},"
                        f"{angle_deg:.2f}"
                    )


                    # ----------------------------------------
                    # SEND UDP
                    # ----------------------------------------

                    sock.sendto(
                        message.encode("ascii"),
                        (
                            BODY_PI_IP,
                            UDP_PORT
                        )
                    )


                    # ----------------------------------------
                    # TERMINAL DEBUG
                    # ----------------------------------------

                    print(
                        f"SEND -> "
                        f"D={distance_ft:5.2f} ft | "
                        f"A={angle_deg:6.1f} deg | "
                        f"Samples: "
                        f"D {good_distance_count:2d}/"
                        f"{len(distance_samples):2d} "
                        f"A {good_angle_count:2d}/"
                        f"{len(angle_samples):2d}"
                    )


                else:

                    print(
                        "FILTER -> "
                        "Collecting samples..."
                    )


            last_send_time = now


# ============================================================
# CTRL+C
# ============================================================

except KeyboardInterrupt:

    print()
    print("Stopping UWB sender...")


# ============================================================
# CLEANUP
# ============================================================

finally:

    beacon.close()
    sock.close()

    print("Beacon sender stopped.")