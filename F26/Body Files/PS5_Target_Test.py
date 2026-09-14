# ============================================================
# PS5_Target_Test.py
#
# BB-8 pathfinding test using a PS5 controller.
#
# LEFT STICK = virtual target:
#     Stick direction -> target angle
#     Stick magnitude -> target distance (0 to 20 ft)
#
# Angle convention matches Target.py:
#      0 deg = directly in front
#     +angle = target to the RIGHT
#     -angle = target to the LEFT
#
# Controls:
#     PS button   = Arm BB-8
#     Menu button = Emergency stop / disarm
#     X button    = Reset motors
#
# This is a standalone TEST program. It intentionally duplicates
# the Target.py navigation behavior instead of importing Target.py.
# ============================================================

import os
os.environ["GPIOZERO_PIN_FACTORY"] = "pigpio"

import sys
import time
import math
import pygame

from Movement_Functions import BB8Movement


# ============================================================
# CONTROLLER BUTTON MAPPINGS
# ============================================================

PS_BUTTON = 10
MENU_BUTTON = 9
X_BUTTON = 0


# ============================================================
# VIRTUAL TARGET SETTINGS
# ============================================================

MAX_STICK_DISTANCE_FT = 20.0

# Raw joystick noise around center is ignored.
STICK_DEADZONE = 0.10


# ============================================================
# TARGET / PATHFINDING SETTINGS
# These match the Target.py tuning values.
# ============================================================

FOLLOW_START_DISTANCE_FT = 10.0
FOLLOW_STOP_DISTANCE_FT = 8.5
HARD_MIN_DISTANCE_FT = 6.0
MAX_VALID_DISTANCE_FT = 100.0

MAX_DRIVE_SPEED = 0.30
MIN_DRIVE_SPEED = 0.12
FULL_SPEED_DISTANCE_FT = 18.0

DRIVE_ACCEL_RATE = 0.40
DRIVE_DECEL_RATE = 0.70

ANGLE_DEADBAND_DEG = 6.0

CLOSE_PIVOT_START_ANGLE_DEG = 22.0
PIVOT_STOP_ANGLE_DEG = 8.0

CLOSE_NAV_DISTANCE_FT = 14.0

# Always pivot if target is mostly behind BB-8.
REAR_PIVOT_ANGLE_DEG = 100.0

SWING_CENTER_DEG = 90.0
MAX_SWING_OFFSET_DEG = 14.0
SWING_KP = 0.35
SWING_DIRECTION = 1
MIN_SWING_OFFSET_DEG = 1.5
SWING_SLEW_RATE_DEG_PER_SEC = 35.0

PIVOT_KP = 0.022
MIN_PIVOT_COMMAND = 0.35
MAX_PIVOT_COMMAND = 0.85
PIVOT_DIRECTION = 1

# No beacon filtering is needed for a joystick, but a little smoothing
# helps keep target angle/distance from jumping around.
DISTANCE_FILTER_ALPHA = 0.35
ANGLE_FILTER_ALPHA = 0.40

TARGET_LOOP_TIME = 0.02       # 50 Hz
PRINT_INTERVAL_SEC = 0.25


# ============================================================
# GENERAL HELPERS
# ============================================================

def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def move_toward(current, target, max_change):
    if target > current:
        return min(current + max_change, target)

    if target < current:
        return max(current - max_change, target)

    return current


def normalize_angle(angle_deg):
    while angle_deg > 180.0:
        angle_deg -= 360.0

    while angle_deg < -180.0:
        angle_deg += 360.0

    return angle_deg


def apply_radial_deadzone(x, y, deadzone):
    """
    Apply a radial deadzone while preserving stick direction.

    The remaining magnitude is rescaled so:
        deadzone -> 0
        full stick -> 1
    """

    magnitude = math.hypot(x, y)

    if magnitude <= deadzone:
        return 0.0, 0.0, 0.0

    magnitude = min(magnitude, 1.0)

    scaled_magnitude = (
        (magnitude - deadzone)
        / (1.0 - deadzone)
    )

    unit_x = x / magnitude
    unit_y = y / magnitude

    return (
        unit_x * scaled_magnitude,
        unit_y * scaled_magnitude,
        scaled_magnitude
    )


def stick_to_target(joystick):
    """
    Convert left-stick position into a virtual target.

    pygame:
        axis 0 = left stick X
        axis 1 = left stick Y

    Desired target coordinates:
        up    ->   0 deg
        right -> +90 deg
        down  -> +/-180 deg
        left  -> -90 deg

    Returns:
        distance_ft, angle_deg, magnitude
    """

    raw_x = joystick.get_axis(0)
    raw_y = joystick.get_axis(1)

    x, y, magnitude = apply_radial_deadzone(
        raw_x,
        raw_y,
        STICK_DEADZONE
    )

    if magnitude <= 0.0:
        return 0.0, 0.0, 0.0

    # pygame Y is negative when stick is pushed upward.
    angle_deg = math.degrees(
        math.atan2(x, -y)
    )

    angle_deg = normalize_angle(angle_deg)

    distance_ft = (
        magnitude * MAX_STICK_DISTANCE_FT
    )

    return distance_ft, angle_deg, magnitude


# ============================================================
# JOYSTICK TARGET FILTER
# ============================================================

class TargetFilter:

    def __init__(self):
        self.filtered_distance = None
        self.filtered_angle = None

    def reset(self):
        self.filtered_distance = None
        self.filtered_angle = None

    def update(self, distance_ft, angle_deg):

        angle_deg = normalize_angle(angle_deg)

        if self.filtered_distance is None:
            self.filtered_distance = distance_ft
            self.filtered_angle = angle_deg
            return self.filtered_distance, self.filtered_angle

        self.filtered_distance = (
            DISTANCE_FILTER_ALPHA * distance_ft
            + (1.0 - DISTANCE_FILTER_ALPHA)
            * self.filtered_distance
        )

        angle_error = normalize_angle(
            angle_deg - self.filtered_angle
        )

        self.filtered_angle = normalize_angle(
            self.filtered_angle
            + ANGLE_FILTER_ALPHA * angle_error
        )

        return self.filtered_distance, self.filtered_angle


# ============================================================
# PATHFINDING CONTROLLER
# ============================================================

class PathController:

    def __init__(self, bb8):
        self.bb8 = bb8

        self.following = False
        self.pivoting = False

        self.current_drive_command = 0.0
        self.current_swing_command = SWING_CENTER_DEG
        self.current_steer_command = 0.0

        self.state = "STARTUP"

    # --------------------------------------------------------
    # DISTANCE CONTROL
    # --------------------------------------------------------

    def calculate_drive_speed(self, distance_ft):

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

    # --------------------------------------------------------
    # ARC STEERING
    # --------------------------------------------------------

    def calculate_swing_target(self, angle_deg):

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

        if 0 < abs(offset) < MIN_SWING_OFFSET_DEG:
            offset = math.copysign(
                MIN_SWING_OFFSET_DEG,
                offset
            )

        return SWING_CENTER_DEG + offset

    # --------------------------------------------------------
    # PIROUETTE CONTROL
    # --------------------------------------------------------

    def calculate_pivot_command(self, angle_deg):

        if abs(angle_deg) <= PIVOT_STOP_ANGLE_DEG:
            return 0.0

        command = (
            angle_deg
            * PIVOT_KP
            * PIVOT_DIRECTION
        )

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

    # --------------------------------------------------------
    # SMOOTH HARDWARE COMMANDS
    # --------------------------------------------------------

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
            SWING_SLEW_RATE_DEG_PER_SEC * dt
        )

        self.current_swing_command = move_toward(
            self.current_swing_command,
            target,
            max_change
        )

        self.bb8.set_swing(
            self.current_swing_command
        )

    def stop_motion(self, dt):

        self.bb8.steer(0.0)
        self.current_steer_command = 0.0

        self.command_drive(
            0.0,
            dt
        )

        self.command_swing(
            SWING_CENTER_DEG,
            dt
        )

    def emergency_stop(self):

        self.bb8.drive(0.0)
        self.bb8.steer(0.0)
        self.current_steer_command = 0.0
        self.bb8.set_swing(SWING_CENTER_DEG)

        self.current_drive_command = 0.0
        self.current_swing_command = SWING_CENTER_DEG

        self.following = False
        self.pivoting = False
        self.state = "STOPPED"

    # --------------------------------------------------------
    # MAIN PATHFINDING DECISION
    # --------------------------------------------------------

    def update(self, distance_ft, angle_deg, dt):

        angle_deg = normalize_angle(angle_deg)

        # ----------------------------------------------------
        # STICK CENTERED
        # ----------------------------------------------------

        if distance_ft <= HARD_MIN_DISTANCE_FT:
            self.state = "STICK CENTER"

            self.following = False
            self.pivoting = False

            self.stop_motion(dt)
            return

        # ----------------------------------------------------
        # FOLLOW DISTANCE HYSTERESIS
        # Same behavior as Target.py.
        # ----------------------------------------------------

        if not self.following:

            if distance_ft < FOLLOW_START_DISTANCE_FT:
                self.state = "IN LEASH ZONE"
                self.stop_motion(dt)
                return

            self.following = True

        else:

            if distance_ft <= FOLLOW_STOP_DISTANCE_FT:
                self.following = False
                self.pivoting = False

                self.state = "TARGET REACHED"
                self.stop_motion(dt)
                return

        # ----------------------------------------------------
        # EXISTING PIVOT HYSTERESIS
        # ----------------------------------------------------

        if self.pivoting:

            if abs(angle_deg) <= PIVOT_STOP_ANGLE_DEG:
                self.pivoting = False

            else:
                self.state = "PIVOT"

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
                self.current_steer_command = pivot_command

                return

        # ----------------------------------------------------
        # TARGET FAR BEHIND
        #
        # Force a reaction-wheel pivot regardless of distance.
        # ----------------------------------------------------

        if abs(angle_deg) >= REAR_PIVOT_ANGLE_DEG:

            self.state = "REAR PIVOT"
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
            self.current_steer_command = pivot_command

            return

        # ----------------------------------------------------
        # TARGET STRAIGHT AHEAD
        # ----------------------------------------------------

        if abs(angle_deg) <= ANGLE_DEADBAND_DEG:

            self.state = "STRAIGHT"

            self.bb8.steer(0.0)
            self.current_steer_command = 0.0

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

        # ----------------------------------------------------
        # CLOSE + LARGE ANGLE
        # ----------------------------------------------------

        if (
            distance_ft <= CLOSE_NAV_DISTANCE_FT
            and
            abs(angle_deg) >= CLOSE_PIVOT_START_ANGLE_DEG
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
            self.current_steer_command = pivot_command

            return

        # ----------------------------------------------------
        # ARC MODE
        # ----------------------------------------------------

        self.state = "ARC"

        # No reaction wheel during normal arc driving.
        self.bb8.steer(0.0)
        self.current_steer_command = 0.0

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


# ============================================================
# CONTROLLER CONNECTION
# ============================================================

def connect_controller():

    pygame.init()
    pygame.joystick.init()

    print("Waiting for PS5 controller...")

    joystick = None

    while joystick is None:

        pygame.joystick.quit()
        pygame.joystick.init()

        if pygame.joystick.get_count() > 0:

            try:
                joystick = pygame.joystick.Joystick(0)
                joystick.init()

            except pygame.error:
                joystick = None

        if joystick is None:
            time.sleep(1.0)

    print(
        f"Controller connected: "
        f"{joystick.get_name()}"
    )

    try:
        joystick.rumble(
            0.5,
            0.5,
            400
        )
    except Exception:
        pass

    return joystick


# ============================================================
# MAIN PROGRAM
# ============================================================

def main():

    bb8 = BB8Movement()

    bb8.stop_all()
    bb8.disable_system()

    joystick = connect_controller()

    controller = PathController(bb8)
    target_filter = TargetFilter()

    system_armed = False
    last_print_time = 0.0
    last_time = time.time()

    print("")
    print("========================================")
    print("     BB-8 PS5 PATHFINDING TEST")
    print("========================================")
    print("")
    print("LEFT STICK = VIRTUAL TARGET")
    print("")
    print("Stick up:       target in front")
    print("Stick right:    target to the right")
    print("Stick left:     target to the left")
    print("Stick down:     target behind")
    print("")
    print("Full stick = 20 ft target distance")
    print("")
    print("PS BUTTON:   Arm / Start BB-8")
    print("MENU BUTTON: Emergency Stop / Disarm")
    print("X BUTTON:    Reset Motors")
    print("")
    print("BB-8 is currently DISARMED.")
    print("Press PS button to arm.")
    print("")

    try:

        while True:

            loop_start = time.time()

            dt = loop_start - last_time
            last_time = loop_start

            dt = clamp(
                dt,
                0.001,
                0.10
            )

            # ================================================
            # CONTROLLER EVENTS
            # ================================================

            for event in pygame.event.get():

                if event.type == pygame.JOYBUTTONDOWN:

                    print(
                        f"Controller button "
                        f"{event.button} pressed"
                    )

                    # ----------------------------------------
                    # PS BUTTON - ARM
                    # ----------------------------------------

                    if event.button == PS_BUTTON:

                        if not system_armed:

                            print("")
                            print("ARMING BB-8...")

                            bb8.stop_all()
                            bb8.enable_system()

                            controller.emergency_stop()
                            target_filter.reset()

                            system_armed = True

                            try:
                                joystick.rumble(
                                    0.8,
                                    0.8,
                                    200
                                )
                            except Exception:
                                pass

                            print("BB-8 ARMED")
                            print("")

                    # ----------------------------------------
                    # MENU BUTTON - E-STOP / DISARM
                    # ----------------------------------------

                    elif event.button == MENU_BUTTON:

                        print("")
                        print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                        print("!!! EMERGENCY STOP !!!")
                        print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

                        controller.emergency_stop()
                        bb8.stop_all()
                        bb8.disable_system()

                        system_armed = False

                        try:
                            joystick.rumble(
                                1.0,
                                1.0,
                                150
                            )

                            time.sleep(0.20)

                            joystick.rumble(
                                1.0,
                                1.0,
                                150
                            )

                        except Exception:
                            pass

                        print("BB-8 DISARMED")
                        print("Press PS button to re-arm.")
                        print("")

                    # ----------------------------------------
                    # X BUTTON - RESET MOTORS
                    # ----------------------------------------

                    elif event.button == X_BUTTON:

                        if system_armed:

                            controller.emergency_stop()
                            bb8.stop_all()
                            target_filter.reset()

                            print(
                                "Motors Reset to Zero..."
                            )

            # ================================================
            # DISARMED
            # ================================================

            if not system_armed:

                elapsed = time.time() - loop_start

                if elapsed < TARGET_LOOP_TIME:
                    time.sleep(
                        TARGET_LOOP_TIME - elapsed
                    )

                continue

            # ================================================
            # LEFT STICK -> VIRTUAL TARGET
            # ================================================

            (
                raw_distance_ft,
                raw_angle_deg,
                stick_magnitude
            ) = stick_to_target(
                joystick
            )

            # When stick is centered, bypass filter so BB-8
            # responds immediately instead of coasting on old
            # virtual-target data.
            if stick_magnitude <= 0.0:

                target_filter.reset()

                filtered_distance = 0.0
                filtered_angle = 0.0

            else:

                (
                    filtered_distance,
                    filtered_angle
                ) = target_filter.update(
                    raw_distance_ft,
                    raw_angle_deg
                )

            # ================================================
            # PATHFINDING
            # ================================================

            controller.update(
                filtered_distance,
                filtered_angle,
                dt
            )

            # ================================================
            # STATUS
            # ================================================

            if (
                loop_start - last_print_time
                >= PRINT_INTERVAL_SEC
            ):

                print(
                    f"{controller.state:14} | "
                    f"Stick={stick_magnitude:4.2f} | "
                    f"D={filtered_distance:5.1f} ft | "
                    f"A={filtered_angle:7.1f} deg | "
                    f"Drive={controller.current_drive_command:+.2f} | "
                    f"Swing={controller.current_swing_command:5.1f} | "
                    f"Pivot={controller.current_steer_command:5.1f}"
                )

                last_print_time = loop_start

            # ================================================
            # LOOP TIMING
            # ================================================

            elapsed = time.time() - loop_start

            if elapsed < TARGET_LOOP_TIME:
                time.sleep(
                    TARGET_LOOP_TIME - elapsed
                )

    except KeyboardInterrupt:

        print("")
        print(
            "Shutting down safely "
            "(user initiated)..."
        )

    except Exception as error:

        print("")
        print(
            f"[CRITICAL ERROR] {error}"
        )

    finally:

        bb8.stop_all()
        bb8.rest_all_servos()
        bb8.disable_system()

        print("Relays Powered OFF")

        pygame.quit()
        sys.exit()


if __name__ == "__main__":
    main()
