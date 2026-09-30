#!/usr/bin/env python3
"""
BB-8 Raspberry Pi -> Teensy target sender

Protocol:
    Pi -> Teensy:
        T,<distance_ft>,<angle_deg>\n
    Teensy -> Pi:
        S,<state>,<distance>,<angle>,<pitch>,<roll>,<drive>,<swing>,<pivot>

Use this class from your UWB / controller code by calling:
    link.send_target(distance_ft, angle_deg)

IMPORTANT:
Keep sending the current target at >= 5-10 Hz even if it has not changed.
The Teensy intentionally stops and disables motor power if packets stop.
"""

import serial
import threading
import time


class TeensyLink:
    def __init__(self, port="/dev/serial0", baud=115200):
        self.ser = serial.Serial(
            port=port,
            baudrate=baud,
            timeout=0.05,
            write_timeout=0.1,
        )

        self.running = True
        self.latest_status = None

        self.reader = threading.Thread(
            target=self._reader_loop,
            daemon=True,
        )
        self.reader.start()

    def send_target(self, distance_ft, angle_deg):
        packet = f"T,{float(distance_ft):.3f},{float(angle_deg):.3f}\n"
        self.ser.write(packet.encode("ascii"))

    def stop(self):
        try:
            self.ser.write(b"STOP\n")
            self.ser.flush()
        finally:
            self.running = False

    def _reader_loop(self):
        while self.running:
            try:
                raw = self.ser.readline()
                if not raw:
                    continue

                line = raw.decode("ascii", errors="replace").strip()

                if line.startswith("S,"):
                    self.latest_status = line
                    print("[TEENSY]", line)
                elif line:
                    print("[TEENSY]", line)

            except serial.SerialException as exc:
                print("Serial error:", exc)
                time.sleep(0.5)


def main():
    link = TeensyLink()

    print("BB-8 Pi/Teensy continuous test sender")
    print("Enter: distance angle")
    print("Example: 15 -30")
    print("The latest target is sent continuously at 20 Hz.")
    print("Type 'stop' to stop motors and return to WAIT_PI.")
    print("Ctrl+C to quit.")

    target_lock = threading.Lock()
    current_target = [None]
    sender_running = [True]

    def sender_loop():
        # 20 Hz heartbeat. This is comfortably faster than the Teensy's
        # 500 ms communications watchdog.
        while sender_running[0]:
            with target_lock:
                target = current_target[0]

            if target is not None:
                try:
                    link.send_target(*target)
                except serial.SerialException as exc:
                    print("Serial write error:", exc)

            time.sleep(0.05)

    sender = threading.Thread(target=sender_loop, daemon=True)
    sender.start()

    try:
        while True:
            command = input("> ").strip()

            if command.lower() == "stop":
                with target_lock:
                    current_target[0] = None
                try:
                    link.ser.write(b"STOP\n")
                    link.ser.flush()
                except serial.SerialException as exc:
                    print("Serial write error:", exc)
                print("STOP sent. Enter a new distance/angle to resume.")
                continue

            parts = command.split()
            if len(parts) != 2:
                print("Enter two numbers: distance_ft angle_deg")
                continue

            try:
                distance = float(parts[0])
                angle = float(parts[1])
            except ValueError:
                print("Invalid numbers.")
                continue

            with target_lock:
                current_target[0] = (distance, angle)

            print(
                f"Target set: D={distance:.1f} ft, A={angle:.1f} deg "
                "(sending continuously at 20 Hz)"
            )

    except KeyboardInterrupt:
        print("\nStopping BB-8.")
    finally:
        sender_running[0] = False
        with target_lock:
            current_target[0] = None
        try:
            link.stop()
        except Exception:
            pass


if __name__ == "__main__":
    main()
