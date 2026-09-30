#!/usr/bin/env python3
"""
BB-8 Body Pi: Head Pi UDP receiver -> Teensy target sender

Head Pi -> Body Pi UDP:
    <distance_ft>,<angle_deg>

Body Pi -> Teensy:
    T,<distance_ft>,<angle_deg>\n

If Head Pi packets stop for HEAD_TIMEOUT seconds, the Body Pi sends STOP
to the Teensy and does not resume until a new valid Head Pi packet arrives.
"""

import socket
import serial
import threading
import time

UDP_PORT = 5005
HEAD_TIMEOUT = 0.5

TEENSY_PORT = "/dev/serial0"
TEENSY_BAUD = 115200
SEND_PERIOD = 0.05       # 20 Hz
UDP_SOCKET_TIMEOUT = 0.1


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

        self.reader = threading.Thread(
            target=self._reader_loop,
            daemon=True,
        )
        self.reader.start()

    def send_target(self, distance_ft, angle_deg):
        packet = f"T,{float(distance_ft):.3f},{float(angle_deg):.3f}\n"
        self.ser.write(packet.encode("ascii"))

    def send_stop(self):
        self.ser.write(b"STOP\n")
        self.ser.flush()

    def close(self):
        try:
            self.send_stop()
        except serial.SerialException:
            pass

        self.running = False

        try:
            self.ser.close()
        except serial.SerialException:
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
                    print("[TEENSY]", line)
                elif line:
                    print("[TEENSY]", line)

            except serial.SerialException as exc:
                if self.running:
                    print("Teensy serial read error:", exc)
                    time.sleep(0.5)


def main():
    link = TeensyLink()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", UDP_PORT))
    sock.settimeout(UDP_SOCKET_TIMEOUT)

    target_lock = threading.Lock()
    current_target = [None]
    sender_running = [True]

    # None means we have not received a valid packet yet.
    last_packet_time = [None]
    head_connected = [False]

    def sender_loop():
        """Continuously resend the newest valid target to the Teensy at 20 Hz."""
        while sender_running[0]:
            with target_lock:
                target = current_target[0]

            if target is not None:
                try:
                    link.send_target(*target)
                except serial.SerialException as exc:
                    print("Teensy serial write error:", exc)

            time.sleep(SEND_PERIOD)

    sender = threading.Thread(target=sender_loop, daemon=True)
    sender.start()

    print("BB-8 Body Pi receiver started")
    print(f"Listening for Head Pi UDP packets on port {UDP_PORT}")
    print(f"Forwarding valid targets to Teensy at {1 / SEND_PERIOD:.0f} Hz")
    print(f"Head Pi timeout: {HEAD_TIMEOUT:.2f} s")
    print("Waiting for Head Pi...")

    try:
        while True:
            try:
                data, address = sock.recvfrom(1024)
                message = data.decode("ascii", errors="strict").strip()

                parts = message.split(",")
                if len(parts) != 2:
                    print(f"[HEAD] Ignoring malformed packet: {message!r}")
                    continue

                distance = float(parts[0])
                angle = float(parts[1])

                now = time.monotonic()

                with target_lock:
                    current_target[0] = (distance, angle)
                    last_packet_time[0] = now

                if not head_connected[0]:
                    print(f"[HEAD] Communication established from {address[0]}")
                    head_connected[0] = True

                # print(
                #     f"[HEAD] D={distance:.2f} ft  "
                #     f"A={angle:.1f} deg"
                #)

            except socket.timeout:
                pass
            except (UnicodeDecodeError, ValueError) as exc:
                print(f"[HEAD] Invalid packet ignored: {exc}")

            # Head Pi communication failsafe.
            with target_lock:
                last_rx = last_packet_time[0]

            timed_out = (
                last_rx is None
                or (time.monotonic() - last_rx) > HEAD_TIMEOUT
            )

            if timed_out and head_connected[0]:
                # Stop the sender thread from continuing to resend the stale target.
                with target_lock:
                    current_target[0] = None

                try:
                    link.send_stop()
                except serial.SerialException as exc:
                    print("Could not send STOP to Teensy:", exc)

                head_connected[0] = False
                print("[FAILSAFE] HEAD PI COMMUNICATION LOST -> STOP sent to Teensy")

    except KeyboardInterrupt:
        print("\nStopping BB-8.")

    finally:
        sender_running[0] = False

        with target_lock:
            current_target[0] = None

        try:
            link.send_stop()
        except serial.SerialException:
            pass

        link.close()
        sock.close()
        sender.join(timeout=0.2)

        print("Body Pi receiver stopped safely.")


if __name__ == "__main__":
    main()
