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
WINDOW_SIZE = 100

# Reject measurements too far from the current group
MAX_DISTANCE_ERROR_FT = 4.0
MAX_ANGLE_ERROR_DEG = 15.0

# Need this many good samples before accepting a new target
MIN_GOOD_SAMPLES = 20


# ============================================================
# BEACON LOSS SETTINGS
# ============================================================

# If no valid UWB packet is received for this long,
# consider the beacon lost.
BEACON_TIMEOUT = 3.0


# ============================================================
# TRANSMISSION SETTINGS
# ============================================================

# Send averaged position to Body Pi at 5 Hz
SEND_RATE_HZ = 5
SEND_PERIOD = 1.0 / SEND_RATE_HZ


# ============================================================
# HOLD-LAST-TARGET SETTINGS
# ============================================================

# If fresh good data is temporarily unavailable, continue
# sending the previous good target for this many send cycles.
#
# At 5 Hz:
#   20 cycles = 4 seconds
#
# After this expires, UDP transmission stops and the Body Pi
# will eventually put BB-8 into PAUSE.
HOLD_LAST_TARGET_CYCLES = 20


# ============================================================
# SAMPLE STORAGE
# ============================================================

distance_samples = deque(maxlen=WINDOW_SIZE)
angle_samples = deque(maxlen=WINDOW_SIZE)


# ============================================================
# BEACON STATE
# ============================================================

# Time that the most recent valid UWB TWR packet arrived
last_beacon_time = None

# Start lost until first valid beacon packet arrives
beacon_lost = True

# Most recent successfully filtered target
#
# Format:
# (distance_ft, angle_deg)
last_good_target = None

# Number of consecutive send cycles for which we've had
# to reuse last_good_target
hold_cycles = 0


# ============================================================
# ANGLE FUNCTIONS
# ============================================================

def angle_difference(a, b):
    """
    Return the smallest signed difference between two angles.

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
    """
    Filter the current rolling sample windows.

    Returns:
        (
            average_distance,
            average_angle,
            number_of_good_distance_samples,
            number_of_good_angle_samples
        )

    Returns None if there are not enough good samples.
    """

    # --------------------------------------------------------
    # WAIT FOR ENOUGH RAW SAMPLES
    # --------------------------------------------------------

    if len(distance_samples) < MIN_GOOD_SAMPLES:
        return None

    if len(angle_samples) < MIN_GOOD_SAMPLES:
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

    if angle_center is None:
        return None

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


# Keep serial timeout short.
#
# This allows readline() to return frequently so the program
# can detect when UWB packets completely stop arriving.

beacon = serial.Serial(
    BEACON_PORT,
    BEACON_BAUD,
    timeout=0.05
)


sock = socket.socket(
    socket.AF_INET,
    socket.SOCK_DGRAM
)


# ============================================================
# STARTUP INFORMATION
# ============================================================

print()
print("======================================")
print(" BB-8 FILTERED UWB BEACON SENDER")
print("======================================")
print()

print(f"Beacon port:        {BEACON_PORT}")
print(f"Body Pi:            {BODY_PI_IP}:{UDP_PORT}")
print()

print(f"Filter window:      {WINDOW_SIZE} samples")
print(f"Minimum samples:    {MIN_GOOD_SAMPLES}")

print(
    f"Distance filter:    +/- "
    f"{MAX_DISTANCE_ERROR_FT} ft"
)

print(
    f"Angle filter:       +/- "
    f"{MAX_ANGLE_ERROR_DEG} deg"
)

print(
    f"Beacon timeout:     "
    f"{BEACON_TIMEOUT} sec"
)

print(
    f"Send rate:          "
    f"{SEND_RATE_HZ} Hz"
)

print(
    f"Hold target cycles: "
    f"{HOLD_LAST_TARGET_CYCLES}"
)

print(
    f"Hold target time:   "
    f"{HOLD_LAST_TARGET_CYCLES / SEND_RATE_HZ:.1f} sec"
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

        # ====================================================
        # READ UWB SERIAL
        # ====================================================

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
                                ">>> BEACON SIGNAL RESTORED <<<"
                            )

                            print(
                                "Collecting fresh measurements..."
                            )
                            print()


                            # Throw away measurements from
                            # before the signal was lost.
                            distance_samples.clear()
                            angle_samples.clear()


                            # IMPORTANT:
                            #
                            # Do NOT clear last_good_target.
                            #
                            # While the new filter fills up,
                            # we can continue using the previous
                            # known-good target if there are
                            # hold cycles remaining.


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
                        # 3x the actual angle.
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

                # --------------------------------------------
                # BEACON JUST BECAME LOST
                # --------------------------------------------

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
                        "Holding previous good target "
                        "temporarily..."
                    )
                    print()


                    # Remove old filter data.
                    #
                    # last_good_target is intentionally
                    # NOT cleared.

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

            target_to_send = None
            using_held_target = False


            # =================================================
            # TRY TO GET A NEW GOOD TARGET
            # =================================================

            if not beacon_lost:

                result = get_filtered_target()


                if result is not None:

                    (
                        distance_ft,
                        angle_deg,
                        good_distance_count,
                        good_angle_count
                    ) = result


                    # -----------------------------------------
                    # SAVE NEW KNOWN-GOOD TARGET
                    # -----------------------------------------

                    last_good_target = (
                        distance_ft,
                        angle_deg
                    )


                    # New good data resets the hold counter.
                    hold_cycles = 0


                    target_to_send = (
                        distance_ft,
                        angle_deg
                    )


                    print(
                        f"GOOD -> "
                        f"D={distance_ft:5.2f} ft | "
                        f"A={angle_deg:6.1f} deg | "
                        f"Confidence: "
                        f"D {(100 * good_distance_count / len(distance_samples)):3.1f}% | "
                        f"A {(100 * good_angle_count / len(distance_samples)):3.1f}%"
                    )


            # =================================================
            # NO NEW GOOD TARGET
            # =================================================

            if target_to_send is None:


                # ---------------------------------------------
                # HOLD LAST KNOWN-GOOD TARGET
                # ---------------------------------------------

                if (
                    last_good_target is not None
                    and
                    hold_cycles < HOLD_LAST_TARGET_CYCLES
                ):

                    target_to_send = last_good_target

                    hold_cycles += 1

                    using_held_target = True


                    distance_ft, angle_deg = (
                        last_good_target
                    )


                    print(
                        f"HOLD "
                        f"{hold_cycles:02d}/"
                        f"{HOLD_LAST_TARGET_CYCLES} -> "
                        f"D={distance_ft:5.2f} ft | "
                        f"A={angle_deg:6.1f} deg"
                    )


                # ---------------------------------------------
                # HOLD PERIOD EXPIRED
                # ---------------------------------------------

                else:

                    if last_good_target is None:

                        print(
                            "WAIT -> "
                            "No good target available yet"
                        )

                    else:

                        print(
                            f"PAUSE -> "
                            f"No good beacon target for "
                            f"{HOLD_LAST_TARGET_CYCLES} cycles"
                        )


            # =================================================
            # SEND TARGET
            # =================================================

            if target_to_send is not None:

                distance_ft, angle_deg = (
                    target_to_send
                )


                message = (
                    f"{distance_ft:.3f},"
                    f"{angle_deg:.2f}"
                )


                sock.sendto(
                    message.encode("ascii"),
                    (
                        BODY_PI_IP,
                        UDP_PORT
                    )
                )


            # =================================================
            # UPDATE SEND TIMER
            # =================================================

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