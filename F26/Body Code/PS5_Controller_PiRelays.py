#!/usr/bin/env python3
"""
BB-8 PS5 controller -> Teensy target sender

Recent BB-8 architecture:
    Raspberry Pi:
        - Reads the PS5 controller
        - Converts left-stick position into a virtual target
        - Continuously sends target distance/angle to Teensy

    Teensy:
        - Owns drive motors, reaction wheel, swing servos, relays, IMU,
          head leveling, navigation, acceleration limiting, and failsafes

Pi -> Teensy:
    T,<distance_ft>,<angle_deg>\n

Controls:
    Left stick:
        Direction = target angle
        Magnitude = target distance (0 to MAX_TARGET_DISTANCE_FT)

    X:
        Emergency motor stop while held; centered stick otherwise sends T,0,0

    PS button:
        Emergency stop and exit

The latest target is continuously sent at 20 Hz so the Teensy watchdog
does not enter COMMS_LOST while the controller is being used.
"""

import math
import sys
import threading
import time

import pygame
import serial
from BB8_Relays import BB8Relays


# =========================
# CONFIGURATION
# =========================

TEENSY_PORT = "/dev/serial0"
TEENSY_BAUD = 115200
SEND_PERIOD = 0.05                 # 20 Hz

MAX_TARGET_DISTANCE_FT = 20.0
STICK_DEADZONE = 0.15

# DualSense / pygame mappings used by the previous controller program.
LEFT_STICK_X_AXIS = 0
LEFT_STICK_Y_AXIS = 1
X_BUTTON = 0
PS_BUTTON = 10


# =========================
# TEENSY SERIAL LINK
# =========================

class TeensyLink:
    def __init__(self, port=TEENSY_PORT, baud=TEENSY_BAUD):
        self.ser = serial.Serial(
            port=port,
            baudrate=baud,
            timeout=0.05,
            write_timeout=0.1,
        )

        self.running = True
        self.latest_status = None
        self.last_status_at = None
        self.write_lock = threading.Lock()

        self.reader = threading.Thread(
            target=self._reader_loop,
            daemon=True,
        )
        self.reader.start()

    def send_target(self, distance_ft, angle_deg):
        packet = f"T,{float(distance_ft):.3f},{float(angle_deg):.3f}\n"
        with self.write_lock:
            self.ser.write(packet.encode("ascii"))

    def send_stop(self):
        with self.write_lock:
            self.ser.write(b"STOP\n")
            self.ser.flush()

    def close(self):
        self.running = False

        try:
            self.send_stop()
        except (serial.SerialException, OSError):
            pass

        try:
            self.ser.close()
        except (serial.SerialException, OSError):
            pass

    def _reader_loop(self):
        while self.running:
            try:
                raw = self.ser.readline()
                if not raw:
                    continue

                line = raw.decode("ascii", errors="replace").strip()

                if line.startswith("S,"):
                    self.latest_status = line
                    self.last_status_at = time.monotonic()
                    print("[TEENSY]", line)
                elif line:
                    print("[TEENSY]", line)

            except (serial.SerialException, OSError) as exc:
                if self.running:
                    print("Teensy serial read error:", exc)
                    time.sleep(0.5)


# =========================
# CONTROLLER HELPERS
# =========================

def apply_radial_deadzone(x, y, deadzone):
    """
    Radial deadzone with rescaling.

    Inside deadzone -> (0, 0, 0)
    Outside deadzone -> magnitude smoothly maps deadzone..1 to 0..1.
    """
    magnitude = math.hypot(x, y)

    if magnitude <= deadzone:
        return 0.0, 0.0, 0.0

    magnitude = min(magnitude, 1.0)
    scaled_magnitude = (magnitude - deadzone) / (1.0 - deadzone)

    if magnitude > 0.0:
        scale = scaled_magnitude / magnitude
        x *= scale
        y *= scale

    return x, y, scaled_magnitude


def stick_to_target(joystick):
    """
    Convert left stick into the same distance/angle target format used
    by the beacon-following code.

    Forward = 0 deg
    Right   = +90 deg
    Back    = +/-180 deg
    Left    = -90 deg
    """
    x = joystick.get_axis(LEFT_STICK_X_AXIS)
    y = -joystick.get_axis(LEFT_STICK_Y_AXIS)  # pygame Y is negative upward

    x, y, magnitude = apply_radial_deadzone(x, y, STICK_DEADZONE)

    if magnitude <= 0.0:
        return 0.0, 0.0

    distance_ft = magnitude * MAX_TARGET_DISTANCE_FT

    # atan2(x, y) intentionally uses Y as the forward reference.
    angle_deg = math.degrees(math.atan2(x, y))

    return distance_ft, angle_deg


# =========================
# MAIN
# =========================

def main():
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() < 1:
        print("No controller found. Please connect your DualSense.")
        pygame.quit()
        return 1

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print(f"Controller detected: {joystick.get_name()}")
    print()
    print("--- BB-8 PS5 TARGET CONTROL ---")
    print("Left stick: virtual target direction + distance")
    print(f"Full stick: {MAX_TARGET_DISTANCE_FT:.1f} ft target")
    print("X button: stop and disable relays while held")
    print("PS button: emergency stop and exit")
    print()

    try:
        link = TeensyLink()
    except serial.SerialException as exc:
        print(f"Could not open Teensy serial port {TEENSY_PORT}: {exc}")
        pygame.quit()
        return 1

    try:
        relays = BB8Relays()
    except Exception:
        link.close()
        pygame.quit()
        raise

    print("Waiting for Teensy status before enabling relays...")
    last_print = 0.0
    x_was_down = False
    ps_was_down = False
    x_latched = False

    # Start safely. No motion command is active until the stick leaves deadzone.
    target_active = False

    try:
        while True:
            loop_start = time.monotonic()

            # Required so pygame refreshes controller state.
            pygame.event.pump()

            ps_down = bool(joystick.get_button(PS_BUTTON))
            x_down = bool(joystick.get_button(X_BUTTON))

            # PS = exit and drop power. X = stop while held, with no automatic
            # restart until X is released and a fresh stick command is present.
            if ps_down and not ps_was_down:
                print("\n[ESTOP] PS button pressed.")
                link.send_stop()
                relays.disable()
                break

            if x_down:
                if not x_latched:
                    link.send_stop()
                    relays.disable()
                    print("[STOP] X held; relays off.")
                x_latched = True
                time.sleep(SEND_PERIOD)
                ps_was_down = ps_down
                x_was_down = x_down
                continue
            x_latched = False

            distance_ft, angle_deg = stick_to_target(joystick)
            if distance_ft <= 0.0:
                distance_ft, angle_deg = 0.0, 0.0
                target_active = False
            else:
                target_active = True

            # Don't energize motor power unless Teensy is demonstrably alive.
            now = time.monotonic()
            teensy_alive = (link.last_status_at is not None and
                            now - link.last_status_at < 1.0)
            if not teensy_alive:
                if relays.enabled:
                    print("[FAILSAFE] Teensy status lost; relays OFF")
                relays.disable()
                time.sleep(SEND_PERIOD)
                ps_was_down = ps_down
                x_was_down = x_down
                continue

            try:
                if not relays.enabled:
                    link.send_stop()   # Clear stale Teensy navigation state
                    relays.enable()    # Includes 150 ms contact settling
                link.send_target(distance_ft, angle_deg)
            except (serial.SerialException, OSError) as exc:
                print("Teensy serial write error:", exc)
                relays.disable()
                break

            if target_active and now - last_print >= 0.25:
                print(f"[TARGET] D={distance_ft:5.2f} ft A={angle_deg:7.2f} deg")
                last_print = now

            ps_was_down = ps_down
            x_was_down = x_down

            elapsed = time.monotonic() - loop_start
            if elapsed < SEND_PERIOD:
                time.sleep(SEND_PERIOD - elapsed)

    except KeyboardInterrupt:
        print("\nKeyboard interrupt -> stopping BB-8.")

    except pygame.error as exc:
        print(f"\nController error: {exc}")

    finally:
        try:
            link.send_stop()
        except (serial.SerialException, OSError):
            pass

        relays.close()
        link.close()
        pygame.quit()
        print("BB-8 stopped safely.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
