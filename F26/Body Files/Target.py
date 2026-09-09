# ============================================================
# Leash_Mode.py
#
# Autonomous BB-8 "Leash Mode"
#
# Inputs from beacon:
#     distance_ft  = distance from BB-8 to person
#     angle_deg    = angle of person relative to BB-8
#
# Angle convention:
#      0 deg = directly in front
#     +angle = person is to the RIGHT
#     -angle = person is to the LEFT
#
# Movement strategy:
#
#     FAR + ALIGNED
#         -> drive straight
#
#     FAR + OFF-ANGLE
#         -> drive in a smooth arc using pendulum steering
#
#     CLOSE + OFF-ANGLE
#         -> stop and pivot using pirouette motor
#
#     INSIDE LEASH ZONE
#         -> stop
#
# If beacon signal is lost:
#         -> STOP
#
# ============================================================


import os

# Must be set before importing Movement_Functions
os.environ["GPIOZERO_PIN_FACTORY"] = "pigpio"

import time
import math

from Movement_Functions import BB8Movement
from Test_Target import TargetTestUI

test_beacon = TargetTestUI()


# ============================================================
# USER TUNING VARIABLES
# ============================================================


# ------------------------------------------------------------
# 1. LEASH DISTANCE
# ------------------------------------------------------------

# BB-8 stops following once it gets this close.
#
# Using two thresholds creates hysteresis:
#
#   > FOLLOW_START_DISTANCE_FT -> start following
#   < FOLLOW_STOP_DISTANCE_FT  -> stop following
#
# This prevents BB-8 from constantly starting/stopping at 10 ft.

FOLLOW_START_DISTANCE_FT = 10.0
FOLLOW_STOP_DISTANCE_FT = 8.5


# Never intentionally move toward the person below this distance.
HARD_MIN_DISTANCE_FT = 6.0


# Ignore obviously bad beacon readings.
MAX_VALID_DISTANCE_FT = 100.0


# ------------------------------------------------------------
# 2. DRIVE SPEED
# ------------------------------------------------------------

# Start LOW during testing.
#
# Existing PS5 code normally uses around 0.3.

MAX_DRIVE_SPEED = 0.30
MIN_DRIVE_SPEED = 0.12


# Distance above FOLLOW_STOP_DISTANCE where maximum speed is reached.
#
# Example:
#
#   FOLLOW_STOP_DISTANCE = 8.5
#   FULL_SPEED_DISTANCE   = 18
#
# At 9 ft  -> slow
# At 18 ft -> full MAX_DRIVE_SPEED

FULL_SPEED_DISTANCE_FT = 18.0


# Limits how quickly commanded drive speed changes.
#
# Smaller = smoother acceleration.
# Units are command units / second.

DRIVE_ACCEL_RATE = 0.40
DRIVE_DECEL_RATE = 0.70


# ------------------------------------------------------------
# 3. HEADING / ANGLE SETTINGS
# ------------------------------------------------------------

# If beacon is inside this angle, treat it as straight ahead.
ANGLE_DEADBAND_DEG = 6.0


# When close to the person, pivot if heading error exceeds this.
CLOSE_PIVOT_START_ANGLE_DEG = 22.0


# Once pivoting, continue until target is inside this angle.
#
# Smaller than PIVOT_START gives hysteresis.

PIVOT_STOP_ANGLE_DEG = 8.0


# ------------------------------------------------------------
# 4. DISTANCE-BASED MOVEMENT STYLE
# ------------------------------------------------------------

# If farther away than this, BB-8 prefers a smooth arc instead
# of stopping and pivoting.

ARC_PREFERRED_DISTANCE_FT = 14.0


# If closer than this, use pivoting for large heading errors.

CLOSE_NAV_DISTANCE_FT = 14.0

# If target is far enough behind BB-8, always pivot first.
REAR_PIVOT_ANGLE_DEG = 100.0


# ------------------------------------------------------------
# 5. PENDULUM ARC STEERING
# ------------------------------------------------------------

# Pendulum center.
SWING_CENTER_DEG = 90.0


# Maximum steering offset from center.
#
# Current Movement_Functions clamps the servo command to
# approximately 70-117 degrees, so +/- 14 gives room.

MAX_SWING_OFFSET_DEG = 14.0


# Converts heading error into pendulum shift.
#
# Example:
#     20 degree target error * 0.35 = 7 degree swing offset

SWING_KP = 0.35


# If BB-8 arcs the WRONG direction, change this from +1 to -1.
SWING_DIRECTION = -1


# Don't make tiny pendulum adjustments.
MIN_SWING_OFFSET_DEG = 1.5


# Maximum pendulum change per second.
#
# This makes the arc enter/exit smoothly rather than snapping.

SWING_SLEW_RATE_DEG_PER_SEC = 35.0


# ------------------------------------------------------------
# 6. PIROUETTE / PIVOT SETTINGS
# ------------------------------------------------------------

# Converts heading error into steer() command.

PIVOT_KP = 0.022


# Minimum command that actually causes useful rotation.

MIN_PIVOT_COMMAND = 0.35


# Maximum reaction-wheel command.

MAX_PIVOT_COMMAND = 0.85


# If pivot turns the WRONG direction, change this to -1.

PIVOT_DIRECTION = 1


# ------------------------------------------------------------
# 7. BEACON FILTERING
# ------------------------------------------------------------

# Exponential moving average.
#
# Higher number:
#     more responsive
#     more jitter
#
# Lower number:
#     smoother
#     more delay

DISTANCE_FILTER_ALPHA = 0.25
ANGLE_FILTER_ALPHA = 0.30


# If no valid beacon packet arrives for this long:
# STOP IMMEDIATELY.

BEACON_TIMEOUT_SEC = 0.50


# ------------------------------------------------------------
# 8. CONTROL LOOP
# ------------------------------------------------------------

LOOP_HZ = 50.0
LOOP_TIME = 1.0 / LOOP_HZ


# How frequently status is printed to terminal.

PRINT_INTERVAL_SEC = 0.25


# ============================================================
# BEACON INTERFACE
# ============================================================

def get_beacon_data():
    """
    Replace the contents of this function with the actual
    interface for your chosen UWB AoA beacon.

    This function MUST return either:

        (distance_ft, angle_deg)

    Example:

        return 14.7, -22.5

    Meaning:

        person is 14.7 ft away
        person is 22.5 degrees LEFT of BB-8

    Or return:

        None

    if no fresh/valid beacon data is available.


    ----------------------------------------------------------
    IMPORTANT
    ----------------------------------------------------------

    This placeholder intentionally returns None so BB-8
    CANNOT MOVE until the real beacon interface is added.
    """

    return None


# ============================================================
# GENERAL HELPERS
# ============================================================

def clamp(value, minimum, maximum):

    return max(minimum, min(maximum, value))


def move_toward(current, target, max_change):
    """
    Slew-rate limiter.
    """

    if target > current:
        return min(current + max_change, target)

    if target < current:
        return max(current - max_change, target)

    return current


def normalize_angle(angle_deg):
    """
    Normalize angle to -180...+180 degrees.
    """

    while angle_deg > 180:
        angle_deg -= 360

    while angle_deg < -180:
        angle_deg += 360

    return angle_deg


# ============================================================
# BEACON FILTER
# ============================================================

class BeaconFilter:

    def __init__(self):

        self.filtered_distance = None
        self.filtered_angle = None


    def reset(self):

        self.filtered_distance = None
        self.filtered_angle = None


    def update(self, distance_ft, angle_deg):

        angle_deg = normalize_angle(angle_deg)

        # First measurement
        if self.filtered_distance is None:

            self.filtered_distance = distance_ft
            self.filtered_angle = angle_deg

            return (
                self.filtered_distance,
                self.filtered_angle
            )


        # Distance EMA
        self.filtered_distance = (
            DISTANCE_FILTER_ALPHA * distance_ft
            + (1.0 - DISTANCE_FILTER_ALPHA)
            * self.filtered_distance
        )


        # ----------------------------------------------------
        # Angle EMA with wrap-around protection
        # ----------------------------------------------------

        angle_error = normalize_angle(
            angle_deg - self.filtered_angle
        )

        self.filtered_angle = normalize_angle(
            self.filtered_angle
            + ANGLE_FILTER_ALPHA * angle_error
        )


        return (
            self.filtered_distance,
            self.filtered_angle
        )


# ============================================================
# NAVIGATION CONTROLLER
# ============================================================

class LeashController:

    def __init__(self, bb8):

        self.bb8 = bb8

        # --------------------------
        # Navigation state
        # --------------------------

        self.following = False

        self.pivoting = False


        # --------------------------
        # Smooth command tracking
        # --------------------------

        self.current_drive_command = 0.0

        self.current_swing_command = SWING_CENTER_DEG


        # --------------------------
        # Beacon safety
        # --------------------------

        self.last_valid_beacon_time = None


        # --------------------------
        # Status
        # --------------------------

        self.state = "STARTUP"
        bb8.spin_head(-0.1)


    # ========================================================
    # DISTANCE CONTROL
    # ========================================================

    def calculate_drive_speed(self, distance_ft):
        """
        Calculate forward speed based on distance.

        Close -> slow
        Far   -> faster
        """

        if distance_ft <= FOLLOW_STOP_DISTANCE_FT:
            return 0.0


        if distance_ft >= FULL_SPEED_DISTANCE_FT:
            return MAX_DRIVE_SPEED


        usable_range = (
            FULL_SPEED_DISTANCE_FT
            - FOLLOW_STOP_DISTANCE_FT
        )


        progress = (
            distance_ft
            - FOLLOW_STOP_DISTANCE_FT
        ) / usable_range


        speed = (
            MIN_DRIVE_SPEED
            + progress
            * (MAX_DRIVE_SPEED - MIN_DRIVE_SPEED)
        )


        return clamp(
            speed,
            MIN_DRIVE_SPEED,
            MAX_DRIVE_SPEED
        )


    # ========================================================
    # ARC STEERING
    # ========================================================

    def calculate_swing_target(self, angle_deg):
        """
        Convert heading error into pendulum steering.

        Larger angle -> larger center-of-gravity shift.
        """

        if abs(angle_deg) <= ANGLE_DEADBAND_DEG:
            return SWING_CENTER_DEG


        offset = (
            angle_deg
            * SWING_KP
            * SWING_DIRECTION
        )


        offset = clamp(
            offset,
            -MAX_SWING_OFFSET_DEG,
            MAX_SWING_OFFSET_DEG
        )


        # Prevent useless tiny commands
        if 0 < abs(offset) < MIN_SWING_OFFSET_DEG:

            offset = math.copysign(
                MIN_SWING_OFFSET_DEG,
                offset
            )


        return SWING_CENTER_DEG + offset


    # ========================================================
    # PIROUETTE CONTROL
    # ========================================================

    def calculate_pivot_command(self, angle_deg):
        """
        Convert heading error into reaction-wheel command.
        """

        if abs(angle_deg) <= PIVOT_STOP_ANGLE_DEG:

            return 0.0


        command = (
            angle_deg
            * PIVOT_KP
            * PIVOT_DIRECTION
        )


        # Minimum useful turn strength
        if 0 < abs(command) < MIN_PIVOT_COMMAND:

            command = math.copysign(
                MIN_PIVOT_COMMAND,
                command
            )


        return clamp(
            command,
            -MAX_PIVOT_COMMAND,
            MAX_PIVOT_COMMAND
        )


    # ========================================================
    # SMOOTH HARDWARE COMMANDS
    # ========================================================

    def command_drive(self, target, dt):

        target = clamp(
            target,
            -MAX_DRIVE_SPEED,
            MAX_DRIVE_SPEED
        )


        if abs(target) > abs(self.current_drive_command):

            rate = DRIVE_ACCEL_RATE

        else:

            rate = DRIVE_DECEL_RATE


        max_change = rate * dt


        self.current_drive_command = move_toward(
            self.current_drive_command,
            target,
            max_change
        )


        self.bb8.drive(
            self.current_drive_command
        )


    def command_swing(self, target, dt):

        target = clamp(
            target,
            SWING_CENTER_DEG - MAX_SWING_OFFSET_DEG,
            SWING_CENTER_DEG + MAX_SWING_OFFSET_DEG
        )


        max_change = (
            SWING_SLEW_RATE_DEG_PER_SEC
            * dt
        )


        self.current_swing_command = move_toward(
            self.current_swing_command,
            target,
            max_change
        )


        self.bb8.set_swing(
            self.current_swing_command
        )


    # ========================================================
    # NORMAL STOP
    # ========================================================

    def stop_motion(self, dt):

        self.bb8.steer(0.0)

        self.command_drive(
            0.0,
            dt
        )

        self.command_swing(
            SWING_CENTER_DEG,
            dt
        )


    # ========================================================
    # EMERGENCY / SIGNAL-LOSS STOP
    # ========================================================

    def emergency_stop(self):

        self.bb8.drive(0.0)

        self.bb8.steer(0.0)

        self.bb8.set_swing(
            SWING_CENTER_DEG
        )


        self.current_drive_command = 0.0
        self.current_swing_command = SWING_CENTER_DEG

        self.following = False
        self.pivoting = False


    # ========================================================
    # MAIN NAVIGATION DECISION
    # ========================================================

    def update(self, distance_ft, angle_deg, dt):

        angle_deg = normalize_angle(angle_deg)


        # ----------------------------------------------------
        # Validate data
        # ----------------------------------------------------

        if not math.isfinite(distance_ft):

            self.state = "INVALID DISTANCE"
            self.emergency_stop()
            return


        if not math.isfinite(angle_deg):

            self.state = "INVALID ANGLE"
            self.emergency_stop()
            return


        if distance_ft < 0:

            self.state = "INVALID DISTANCE"
            self.emergency_stop()
            return


        if distance_ft > MAX_VALID_DISTANCE_FT:

            self.state = "OUT OF RANGE"
            self.emergency_stop()
            return


        self.last_valid_beacon_time = time.time()


        # ====================================================
        # HARD MINIMUM DISTANCE
        # ====================================================

        if distance_ft <= HARD_MIN_DISTANCE_FT:

            self.state = "TOO CLOSE"

            self.following = False
            self.pivoting = False

            self.stop_motion(dt)

            return


        # ====================================================
        # FOLLOW DISTANCE HYSTERESIS
        # ====================================================

        if not self.following:

            # Don't restart until target gets outside 10 ft
            if distance_ft < FOLLOW_START_DISTANCE_FT:

                self.state = "IN LEASH ZONE"

                self.stop_motion(dt)

                return


            self.following = True


        else:

            # Once following, continue until 8.5 ft
            if distance_ft <= FOLLOW_STOP_DISTANCE_FT:

                self.following = False
                self.pivoting = False

                self.state = "TARGET REACHED"

                self.stop_motion(dt)

                return


        # ====================================================
        # PIVOT STATE HYSTERESIS
        # ====================================================

        if self.pivoting:

            # Stay in pivot mode until well aligned
            if abs(angle_deg) <= PIVOT_STOP_ANGLE_DEG:

                self.pivoting = False

            else:

                self.state = "PIVOT"

                # Stop translational motion while pivoting
                self.command_drive(
                    0.0,
                    dt
                )

                # Center pendulum
                self.command_swing(
                    SWING_CENTER_DEG,
                    dt
                )

                pivot_command = (
                    self.calculate_pivot_command(
                        angle_deg
                    )
                )

                self.bb8.steer(
                    pivot_command
                )

                return
        # ====================================================
        # TARGET FAR BEHIND
        #
        # If the target is significantly behind BB-8,
        # pivot first regardless of distance.
        # ====================================================

        if abs(angle_deg) >= REAR_PIVOT_ANGLE_DEG:

            self.state = "PIVOT"

            self.pivoting = True

            # Stop driving while rotating
            self.command_drive(
                0.0,
                dt
            )

            # Center pendulum while rotating
            self.command_swing(
                SWING_CENTER_DEG,
                dt
            )

            pivot_command = (
                self.calculate_pivot_command(
                    angle_deg
                )
            )

            self.bb8.steer(
                pivot_command
            )

            return

        # ====================================================
        # TARGET STRAIGHT AHEAD
        # ====================================================

        if abs(angle_deg) <= ANGLE_DEADBAND_DEG:

            self.state = "STRAIGHT"

            self.bb8.steer(0.0)

            self.command_swing(
                SWING_CENTER_DEG,
                dt
            )

            speed = self.calculate_drive_speed(
                distance_ft
            )

            self.command_drive(
                speed,
                dt
            )

            return


        # ====================================================
        # CLOSE + LARGE ANGLE
        #
        # Pivot before driving.
        # ====================================================

        if (
            distance_ft <= CLOSE_NAV_DISTANCE_FT
            and
            abs(angle_deg)
            >= CLOSE_PIVOT_START_ANGLE_DEG
        ):

            self.state = "PIVOT"

            self.pivoting = True

            self.command_drive(
                0.0,
                dt
            )

            self.command_swing(
                SWING_CENTER_DEG,
                dt
            )

            pivot_command = (
                self.calculate_pivot_command(
                    angle_deg
                )
            )

            self.bb8.steer(
                pivot_command
            )

            return


        # ====================================================
        # ARC MODE
        #
        # Target is far enough away that BB-8 can take a
        # sweeping curve rather than stopping to rotate.
        # ====================================================

        self.state = "ARC"

        # Reaction wheel is NOT used during normal arc driving.
        self.bb8.steer(0.0)


        swing_target = (
            self.calculate_swing_target(
                angle_deg
            )
        )


        self.command_swing(
            swing_target,
            dt
        )


        speed = self.calculate_drive_speed(
            distance_ft
        )


        # ----------------------------------------------------
        # Reduce speed for sharp arcs
        # ----------------------------------------------------

        angle_strength = clamp(
            abs(angle_deg) / 90.0,
            0.0,
            1.0
        )


        turn_speed_factor = (
            1.0
            - 0.45 * angle_strength
        )


        speed *= turn_speed_factor


        speed = max(
            MIN_DRIVE_SPEED,
            speed
        )


        self.command_drive(
            speed,
            dt
        )


    # ========================================================
    # WATCHDOG
    # ========================================================

    def check_beacon_timeout(self):

        if self.last_valid_beacon_time is None:

            self.state = "WAITING FOR BEACON"
            self.emergency_stop()

            return True


        if (
            time.time()
            - self.last_valid_beacon_time
            > BEACON_TIMEOUT_SEC
        ):

            self.state = "BEACON LOST"
            self.emergency_stop()

            return True


        return False


# ============================================================
# MAIN PROGRAM
# ============================================================

def main():

    print("")
    print("========================================")
    print("        BB-8 LEASH MODE STARTING")
    print("========================================")
    print("")


    # --------------------------------------------------------
    # Initialize BB-8 hardware
    # --------------------------------------------------------

    bb8 = BB8Movement()


    # Make absolutely sure everything begins stopped.
    bb8.stop_all()


    controller = LeashController(bb8)

    beacon_filter = BeaconFilter()


    # Enable motor/servo power.
    #
    # This does NOT mean BB-8 immediately moves.
    # It will only move after receiving valid beacon data.

    bb8.enable_system()


    print("Hardware enabled.")
    print("Waiting for valid beacon data...")
    print("")


    last_loop_time = time.time()

    last_print_time = 0.0


    try:

        while True:

            loop_start = time.time()


            # =================================================
            # DELTA TIME
            # =================================================

            dt = loop_start - last_loop_time
            last_loop_time = loop_start


            # Protect against weird timing after pauses/debugging

            dt = clamp(
                dt,
                0.001,
                0.10
            )


            # =================================================
            # GET BEACON DATA
            # =================================================

            test_beacon.update()

            distance_ft, angle_deg = test_beacon.get_target()

            # No target entered yet, or "stop" was entered
            if distance_ft is not None and angle_deg is not None:

                try:
                    distance_ft = float(distance_ft)
                    angle_deg = float(angle_deg)

                    # -----------------------------------------
                    # Filter measurement
                    # -----------------------------------------

                    (
                        filtered_distance,
                        filtered_angle
                    ) = beacon_filter.update(
                        distance_ft,
                        angle_deg
                    )

                    # -----------------------------------------
                    # Run navigation
                    # -----------------------------------------

                    controller.update(
                        filtered_distance,
                        filtered_angle,
                        dt
                    )

                    # -----------------------------------------
                    # Status output
                    # -----------------------------------------

                    if (
                        loop_start - last_print_time
                        >= PRINT_INTERVAL_SEC
                    ):
                        print(
                            f"{controller.state:18} | "
                            f"D={filtered_distance:5.1f} ft | "
                            f"A={filtered_angle:6.1f} deg | "
                            f"Drive={controller.current_drive_command:+.2f} | "
                            f"Swing={controller.current_swing_command:5.1f}"
                        )

                        last_print_time = loop_start

                except (TypeError, ValueError):
                    print("[WARNING] Invalid test target.")


            # =================================================
            # SIGNAL-LOSS WATCHDOG
            # =================================================

            controller.check_beacon_timeout()


            # =================================================
            # LOOP TIMING
            # =================================================

            elapsed = (
                time.time()
                - loop_start
            )


            if elapsed < LOOP_TIME:

                time.sleep(
                    LOOP_TIME - elapsed
                )


    # ========================================================
    # USER STOP
    # ========================================================

    except KeyboardInterrupt:

        print("")
        print("Leash mode stopped by user.")


    # ========================================================
    # UNEXPECTED ERROR
    # ========================================================

    except Exception as error:

        print("")
        print(
            f"[CRITICAL ERROR] {error}"
        )


    # ========================================================
    # CLEANUP
    # ========================================================

    finally:

        print(
            "Stopping BB-8..."
        )

        bb8.stop_all()

        bb8.rest_all_servos()

        bb8.disable_system()

        print(
            "Motors and relays disabled."
        )


# ============================================================
# PROGRAM ENTRY POINT
# ============================================================

if __name__ == "__main__":

    main()