# ============================================================
# Target.py
#
# Autonomous BB-8 "Leash Mode"
#
# Current input:
#     Test_Target.py fake beacon
#
# Future input:
#     UWB AoA beacon
#
# Beacon values:
#     distance_ft = distance from BB-8 to person, in FEET
#     angle_deg   = angle of person relative to BB-8
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
#     FAR + OFF ANGLE
#         -> drive in an arc using pendulum steering
#
#     CLOSE + LARGE ANGLE
#         -> stop and pivot with pirouette/reaction wheel
#
#     TARGET FAR BEHIND
#         -> pivot first regardless of distance
#
#     LOST BEACON
#         -> stop immediately
#
# ============================================================


import os

# Must be set BEFORE importing Movement_Functions
os.environ["GPIOZERO_PIN_FACTORY"] = "pigpio"

import time
import math

from Movement_Functions import BB8Movement
from Test_Target import TargetTestUI


# ============================================================
# TEST BEACON
# ============================================================

test_beacon = TargetTestUI()


# ============================================================
# USER TUNING VARIABLES
# ============================================================


# ------------------------------------------------------------
# 1. LEASH DISTANCE
# ------------------------------------------------------------

# Start following once the person moves this far away.
FOLLOW_START_DISTANCE_FT = 10.0

# Once following, stop when the person gets this close.
#
# The difference between START and STOP provides hysteresis
# so BB-8 doesn't rapidly start/stop around one distance.
FOLLOW_STOP_DISTANCE_FT = 8.5

# Absolute minimum allowed distance.
HARD_MIN_DISTANCE_FT = 6.0

# Reject obviously bad readings.
MAX_VALID_DISTANCE_FT = 100.0


# ------------------------------------------------------------
# 2. DRIVE SPEED
# ------------------------------------------------------------

# Maximum forward command.
#
# You tested 0.70 successfully.
# Reduce this during early autonomous testing if desired.
MAX_DRIVE_SPEED = 0.30

# Minimum useful moving speed.
MIN_DRIVE_SPEED = 0.12

# At this distance or farther, BB-8 reaches MAX_DRIVE_SPEED.
FULL_SPEED_DISTANCE_FT = 18.0

# Acceleration/deceleration rate.
#
# Units:
# command per second
#
# Example:
# 0.40 means approximately 1.75 seconds from 0 -> 0.70.
DRIVE_ACCEL_RATE = 0.40
DRIVE_DECEL_RATE = 0.70

# Smallest speed change worth sending to gpiozero.
#
# THIS IS IMPORTANT:
# Once the requested speed stops changing, we stop resending
# the same command to the motor driver.
DRIVE_COMMAND_EPSILON = 0.001


# ------------------------------------------------------------
# 3. ANGLE / HEADING
# ------------------------------------------------------------

# Consider target straight ahead inside +/- this amount.
ANGLE_DEADBAND_DEG = 6.0

# When close to the person, start pivoting above this error.
CLOSE_PIVOT_START_ANGLE_DEG = 22.0

# Once pivoting, stop pivoting below this error.
PIVOT_STOP_ANGLE_DEG = 8.0

# If target is farther behind than this angle,
# pivot regardless of distance.
REAR_PIVOT_ANGLE_DEG = 65.0


# ------------------------------------------------------------
# 4. DISTANCE-BASED NAVIGATION STYLE
# ------------------------------------------------------------

# Inside this range, a sufficiently large heading error
# causes BB-8 to pivot before driving.
CLOSE_NAV_DISTANCE_FT = 14.0


# ------------------------------------------------------------
# 5. PENDULUM ARC STEERING
# ------------------------------------------------------------

SWING_CENTER_DEG = 90.0

# Maximum pendulum steering offset from center.
MAX_SWING_OFFSET_DEG = 14.0

# Angle-to-pendulum gain.
#
# Example:
# 20 degree heading error * 0.35 = 7 degree servo offset
SWING_KP = 0.35

# Reverse this if arc steering goes the wrong direction.
SWING_DIRECTION = -1

# Ignore tiny pendulum offsets.
MIN_SWING_OFFSET_DEG = 1.5

# Maximum servo movement rate.
SWING_SLEW_RATE_DEG_PER_SEC = 35.0

# Do not resend essentially identical servo commands.
SWING_COMMAND_EPSILON_DEG = 0.05


# ------------------------------------------------------------
# 6. PIROUETTE / REACTION WHEEL
# ------------------------------------------------------------

# Converts heading error into bb8.steer() command.
PIVOT_KP = 0.022

# Minimum useful reaction wheel command.
MIN_PIVOT_COMMAND = 0.1

# Maximum reaction wheel command.
MAX_PIVOT_COMMAND = 1

# Reverse if pivoting turns the wrong direction.
PIVOT_DIRECTION = -1

# Don't resend essentially identical turn commands.
STEER_COMMAND_EPSILON = 0.01


# ------------------------------------------------------------
# 7. BEACON FILTER
# ------------------------------------------------------------

# EMA smoothing:
#
# Higher = more responsive / noisier
# Lower  = smoother / slower
DISTANCE_FILTER_ALPHA = 0.25
ANGLE_FILTER_ALPHA = 0.30

# Stop if no valid beacon data arrives for this long.
BEACON_TIMEOUT_SEC = 0.50


# ------------------------------------------------------------
# 8. CONTROL LOOP
# ------------------------------------------------------------

LOOP_HZ = 50.0
LOOP_TIME = 1.0 / LOOP_HZ

PRINT_INTERVAL_SEC = 0.25


# ============================================================
# FUTURE REAL BEACON INTERFACE
# ============================================================

def get_beacon_data():
    """
    Future interface for the real UWB AoA beacon.

    Return:

        (distance_ft, angle_deg)

    Example:

        return 14.7, -22.5

    meaning:

        person is 14.7 ft away
        person is 22.5 degrees LEFT

    Return None if no fresh beacon packet is available.

    This function is currently NOT used while Test_Target.py
    is providing fake target information.
    """

    return None


# ============================================================
# GENERAL HELPERS
# ============================================================

def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def move_toward(current, target, max_change):
    """
    Move current toward target by no more than max_change.
    """

    if target > current:
        return min(current + max_change, target)

    if target < current:
        return max(current - max_change, target)

    return current


def normalize_angle(angle_deg):
    """
    Normalize angle to -180 ... +180 degrees.
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

        # First valid reading
        if self.filtered_distance is None:

            self.filtered_distance = distance_ft
            self.filtered_angle = angle_deg

            return (
                self.filtered_distance,
                self.filtered_angle
            )


        # --------------------------
        # Distance EMA
        # --------------------------

        self.filtered_distance = (
            DISTANCE_FILTER_ALPHA * distance_ft
            + (1.0 - DISTANCE_FILTER_ALPHA)
            * self.filtered_distance
        )


        # --------------------------
        # Angle EMA
        #
        # Use wrap-safe angle error.
        # --------------------------

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
# LEASH CONTROLLER
# ============================================================

class LeashController:

    def __init__(self, bb8):

        self.bb8 = bb8


        # ----------------------------------------------------
        # NAVIGATION STATE
        # ----------------------------------------------------

        self.following = False
        self.pivoting = False

        self.state = "STARTUP"


        # ----------------------------------------------------
        # DESIRED / SMOOTHED COMMANDS
        # ----------------------------------------------------

        self.current_drive_command = 0.0
        self.current_swing_command = SWING_CENTER_DEG
        self.current_steer_command = 0.0


        # ----------------------------------------------------
        # LAST COMMAND ACTUALLY SENT TO HARDWARE
        #
        # These are separate because the controller runs at
        # 50 Hz, but we do NOT want to resend an unchanged
        # motor command 50 times per second.
        # ----------------------------------------------------

        self.last_drive_hardware_command = None
        self.last_swing_hardware_command = None
        self.last_steer_hardware_command = None


        # ----------------------------------------------------
        # BEACON SAFETY
        # ----------------------------------------------------

        self.last_valid_beacon_time = None


    # ========================================================
    # DRIVE HARDWARE INTERFACE
    # ========================================================

    def send_drive_command(self, command, force=False):
        """
        Send drive command only if it actually changed.

        This prevents the repeated gpiozero writes that caused
        the main drive motors to periodically stutter.
        """

        command = clamp(
            command,
            -MAX_DRIVE_SPEED,
            MAX_DRIVE_SPEED
        )


        if (
            force
            or self.last_drive_hardware_command is None
            or abs(
                command
                - self.last_drive_hardware_command
            ) >= DRIVE_COMMAND_EPSILON
        ):

            self.bb8.drive(command)

            self.last_drive_hardware_command = command


    # ========================================================
    # STEER HARDWARE INTERFACE
    # ========================================================

    def send_steer_command(self, command, force=False):
        """
        Avoid repeatedly sending an unchanged reaction-wheel
        command when it isn't necessary.
        """

        command = clamp(command, -1.0, 1.0)

        if (
            force
            or self.last_steer_hardware_command is None
            or abs(
                command
                - self.last_steer_hardware_command
            ) >= STEER_COMMAND_EPSILON
        ):
            self.bb8.steer(command)
            self.current_steer_command = command
            self.last_steer_hardware_command = command


    # ========================================================
    # SWING HARDWARE INTERFACE
    # ========================================================

    def send_swing_command(self, degrees, force=False):
        """
        Avoid repeatedly writing the exact same PCA9685 servo
        position when nothing has changed.
        """

        degrees = clamp(
            degrees,
            SWING_CENTER_DEG - MAX_SWING_OFFSET_DEG,
            SWING_CENTER_DEG + MAX_SWING_OFFSET_DEG
        )


        if (
            force
            or self.last_swing_hardware_command is None
            or abs(
                degrees
                - self.last_swing_hardware_command
            ) >= SWING_COMMAND_EPSILON_DEG
        ):

            self.bb8.set_swing(degrees)

            self.last_swing_hardware_command = degrees


    # ========================================================
    # DISTANCE CONTROL
    # ========================================================

    def calculate_drive_speed(self, distance_ft):
        """
        Calculate forward speed from target distance.

        Close -> slower
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
            * (
                MAX_DRIVE_SPEED
                - MIN_DRIVE_SPEED
            )
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
        Convert target angle into pendulum shift.
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


        # Avoid tiny ineffective offsets
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
        Convert target angle into reaction-wheel command.
        """

        if abs(angle_deg) <= PIVOT_STOP_ANGLE_DEG:
            return 0.0


        command = (
            angle_deg
            * PIVOT_KP
            * PIVOT_DIRECTION
        )


        # Enforce minimum useful command
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
    # SMOOTH DRIVE COMMAND
    # ========================================================

    def command_drive(self, target, dt):
        """
        Slew the requested drive command toward target.

        IMPORTANT:
        An unchanged final command is NOT continuously resent
        to gpiozero. This prevents the stuttering found during
        testing.
        """

        target = clamp(
            target,
            -MAX_DRIVE_SPEED,
            MAX_DRIVE_SPEED
        )


        # Determine acceleration vs deceleration
        if abs(target) > abs(self.current_drive_command):

            rate = DRIVE_ACCEL_RATE

        else:

            rate = DRIVE_DECEL_RATE


        max_change = rate * dt


        new_command = move_toward(
            self.current_drive_command,
            target,
            max_change
        )


        # Snap very tiny differences exactly to target
        if abs(new_command - target) < DRIVE_COMMAND_EPSILON:
            new_command = target


        # Store controller state
        self.current_drive_command = new_command


        # Hardware write occurs ONLY when command changed
        self.send_drive_command(
            self.current_drive_command
        )


    # ========================================================
    # SMOOTH PENDULUM COMMAND
    # ========================================================

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


        new_command = move_toward(
            self.current_swing_command,
            target,
            max_change
        )


        if (
            abs(new_command - target)
            < SWING_COMMAND_EPSILON_DEG
        ):
            new_command = target


        self.current_swing_command = new_command


        self.send_swing_command(
            self.current_swing_command
        )


    # ========================================================
    # NORMAL STOP
    # ========================================================

    def stop_motion(self, dt):
        """
        Controlled stop.

        Drive ramps down instead of immediately jumping to zero.
        """

        self.send_steer_command(0.0)

        self.command_drive(
            0.0,
            dt
        )

        self.command_swing(
            SWING_CENTER_DEG,
            dt
        )


    # ========================================================
    # EMERGENCY STOP
    # ========================================================

    def emergency_stop(self):
        """
        Immediate stop used for invalid/lost beacon data.
        """

        # Force the commands through even if our cached value
        # happens to already say zero.
        self.send_drive_command(
            0.0,
            force=True
        )

        self.send_steer_command(
            0.0,
            force=True
        )

        self.send_swing_command(
            SWING_CENTER_DEG,
            force=True
        )


        self.current_drive_command = 0.0
        self.current_swing_command = SWING_CENTER_DEG

        self.following = False
        self.pivoting = False


    # ========================================================
    # MAIN NAVIGATION UPDATE
    # ========================================================

    def update(self, distance_ft, angle_deg, dt):

        angle_deg = normalize_angle(angle_deg)


        # ----------------------------------------------------
        # VALIDATE BEACON DATA
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


        # This packet is valid.
        self.last_valid_beacon_time = time.time()


        # ====================================================
        # HARD MINIMUM DISTANCE
        # ====================================================

        if distance_ft <= HARD_MIN_DISTANCE_FT:

            self.state = "TOO CLOSE"

            self.following = False
            self.pivoting = False

            # Immediate stop because the person is very close.
            self.emergency_stop()

            return


        # ====================================================
        # FOLLOW DISTANCE HYSTERESIS
        # ====================================================

        if not self.following:

            # Don't start moving again until target is at
            # least FOLLOW_START_DISTANCE_FT away.
            if distance_ft < FOLLOW_START_DISTANCE_FT:

                self.state = "IN LEASH ZONE"

                self.stop_motion(dt)

                return


            self.following = True


        else:

            # Once following, continue until target gets
            # inside FOLLOW_STOP_DISTANCE_FT.
            if distance_ft <= FOLLOW_STOP_DISTANCE_FT:

                self.following = False
                self.pivoting = False

                self.state = "TARGET REACHED"

                self.stop_motion(dt)

                return


        # ====================================================
        # EXISTING PIVOT STATE
        # ====================================================

        if self.pivoting:

            # Don't bounce in/out of pivot mode.
            if abs(angle_deg) <= PIVOT_STOP_ANGLE_DEG:

                self.pivoting = False

                # Explicitly stop reaction wheel once aligned.
                self.send_steer_command(0.0)

            else:

                self.state = "PIVOT"

                # Stop forward motion
                self.command_drive(
                    0.0,
                    dt
                )

                # Center pendulum
                self.command_swing(
                    SWING_CENTER_DEG,
                    dt
                )

                # Turn body
                pivot_command = (
                    self.calculate_pivot_command(
                        angle_deg
                    )
                )

                self.send_steer_command(
                    pivot_command
                )

                return


        # ====================================================
        # TARGET FAR BEHIND
        # ====================================================

        if abs(angle_deg) >= REAR_PIVOT_ANGLE_DEG:

            self.state = "PIVOT"
            self.pivoting = True

            # Stop translational movement
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

            self.send_steer_command(
                pivot_command
            )

            return


        # ====================================================
        # TARGET STRAIGHT AHEAD
        # ====================================================

        if abs(angle_deg) <= ANGLE_DEADBAND_DEG:

            self.state = "STRAIGHT"

            # Pirouette motor should be off.
            self.send_steer_command(0.0)

            # Center the pendulum.
            self.command_swing(
                SWING_CENTER_DEG,
                dt
            )

            # Calculate distance-based drive speed.
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
        # ====================================================

        if (
            distance_ft <= CLOSE_NAV_DISTANCE_FT
            and
            abs(angle_deg)
            >= CLOSE_PIVOT_START_ANGLE_DEG
        ):

            self.state = "PIVOT"

            self.pivoting = True


            # Stop forward motion.
            self.command_drive(
                0.0,
                dt
            )


            # Center pendulum.
            self.command_swing(
                SWING_CENTER_DEG,
                dt
            )


            pivot_command = (
                self.calculate_pivot_command(
                    angle_deg
                )
            )


            self.send_steer_command(
                pivot_command
            )

            return


        # ====================================================
        # ARC MODE
        # ====================================================

        self.state = "ARC"


        # Reaction wheel is NOT needed for normal arc.
        self.send_steer_command(0.0)


        # Calculate pendulum steering.
        swing_target = (
            self.calculate_swing_target(
                angle_deg
            )
        )


        self.command_swing(
            swing_target,
            dt
        )


        # Calculate forward speed.
        speed = self.calculate_drive_speed(
            distance_ft
        )


        # ----------------------------------------------------
        # SLOW DOWN DURING SHARPER ARCS
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
    # BEACON WATCHDOG
    # ========================================================

    def check_beacon_timeout(self):

        # No valid beacon has ever been received.
        if self.last_valid_beacon_time is None:

            self.state = "WAITING FOR BEACON"

            self.emergency_stop()

            return True


        # Beacon was present but became stale.
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
    # INITIALIZE HARDWARE
    # --------------------------------------------------------

    bb8 = BB8Movement()


    # Make sure everything starts stopped.
    bb8.stop_all()


    controller = LeashController(bb8)

    beacon_filter = BeaconFilter()


    # Enable motor / servo power.
    bb8.enable_system()


    print("Hardware enabled.")
    print("")
    print("TEST TARGET MODE")
    print("")
    print("Enter:")
    print("    distance_ft angle_deg")
    print("")
    print("Examples:")
    print("    20 0")
    print("    20 20")
    print("    12 30")
    print("    12 90")
    print("")
    print("Use 'stop' in Test_Target to remove target.")
    print("")


    last_loop_time = time.time()

    last_print_time = 0.0


    try:

        while True:

            loop_start = time.time()


            # =================================================
            # DELTA TIME
            # =================================================

            dt = (
                loop_start
                - last_loop_time
            )

            last_loop_time = loop_start


            # Protect against weird timing after a pause.
            dt = clamp(
                dt,
                0.001,
                0.10
            )


            # =================================================
            # GET TEST BEACON DATA
            # =================================================

            test_beacon.update()

            distance_ft, angle_deg = (
                test_beacon.get_target()
            )


            # =================================================
            # VALID TARGET
            # =================================================

            if (
                distance_ft is not None
                and
                angle_deg is not None
            ):

                try:

                    distance_ft = float(
                        distance_ft
                    )

                    angle_deg = float(
                        angle_deg
                    )


                    # -----------------------------------------
                    # FILTER BEACON DATA
                    # -----------------------------------------

                    (
                        filtered_distance,
                        filtered_angle
                    ) = beacon_filter.update(
                        distance_ft,
                        angle_deg
                    )


                    # -----------------------------------------
                    # NAVIGATION UPDATE
                    # -----------------------------------------

                    controller.update(
                        filtered_distance,
                        filtered_angle,
                        dt
                    )


                    # -----------------------------------------
                    # STATUS
                    # -----------------------------------------

                    if (
                        loop_start
                        - last_print_time
                        >= PRINT_INTERVAL_SEC
                    ):

                        print(
                            f"{controller.state:18} | "
                            f"D={filtered_distance:5.1f} ft | "
                            f"A={filtered_angle:4.1f} deg | "
                            f"Drive={controller.current_drive_command:+.2f} | "
                            f"Swing={controller.current_swing_command:5.1f} | "
                            f"Pivot={controller.current_steer_command:5.1f}"
                        )

                        last_print_time = loop_start


                except (TypeError, ValueError):

                    print(
                        "[WARNING] Invalid test target."
                    )


            # =================================================
            # NO TARGET
            # =================================================

            else:

                # If the test target is removed, watchdog
                # will stop BB-8 after BEACON_TIMEOUT_SEC.
                pass


            # =================================================
            # BEACON WATCHDOG
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
    # ERROR
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

        print("")
        print("Stopping BB-8...")


        # Direct hardware cleanup rather than relying on
        # cached controller command state.
        bb8.stop_all()

        bb8.rest_all_servos()

        bb8.disable_system()


        print(
            "Motors and relays disabled."
        )


# ============================================================
# PROGRAM ENTRY
# ============================================================

if __name__ == "__main__":

    main()