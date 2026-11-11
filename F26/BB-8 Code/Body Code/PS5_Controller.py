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
RIGHT_STICK_Y_AXIS = 4  # Typical DualSense pygame mapping; verify on your Pi
TRIANGLE_BUTTON = 3
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

    def send_manual(self, drive, swing):
        packet = f"M,{float(drive):.3f},{float(swing):.3f}\n"
        with self.write_lock:
            self.ser.write(packet.encode("ascii"))
