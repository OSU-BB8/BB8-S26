#!/usr/bin/env python3
"""BB-8 Body Pi: Head UDP -> Teensy targets, with Pi-controlled relays.

Head Pi UDP port 5005: distance_ft,angle_deg
Teensy UART 115200: T,distance_ft,angle_deg\n or STOP\n
"""
import math
import socket
import threading
import time
import serial
from BB8_Relays import BB8Relays

UDP_PORT = 5005
HEAD_TIMEOUT = 0.5
TEENSY_PORT = '/dev/serial0'
TEENSY_BAUD = 115200
SEND_PERIOD = 0.05
TEENSY_STATUS_TIMEOUT = 1.0


class TeensyLink:
    def __init__(self):
        self.ser = serial.Serial(TEENSY_PORT, TEENSY_BAUD,
                                 timeout=0.05, write_timeout=0.1)
        self.running = threading.Event()
        self.running.set()
        self.write_lock = threading.Lock()
        self.last_status_at = None
        self.reader = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader.start()

    def send_target(self, distance, angle):
        with self.write_lock:
            self.ser.write(f'T,{distance:.3f},{angle:.3f}\n'.encode('ascii'))

    def send_stop(self):
        with self.write_lock:
            self.ser.write(b'STOP\n')
            self.ser.flush()

    def status_recent(self):
        t = self.last_status_at
        return t is not None and time.monotonic() - t < TEENSY_STATUS_TIMEOUT

    def _reader_loop(self):
        while self.running.is_set():
            try:
                raw = self.ser.readline()
                if not raw:
                    continue
                line = raw.decode('ascii', errors='replace').strip()
                if line:
                    # Current Teensy firmware uses 'S=WAIT_PI,...', not 'S,...'.
                    # Accept either status format without requiring a state
                    # transition out of WAIT_PI before sending first target.
                    if line.startswith(('S=', 'S,')):
                        self.last_status_at = time.monotonic()
                    print('[TEENSY]', line)
            except (serial.SerialException, OSError, TypeError) as exc:
                if self.running.is_set():
                    print('[TEENSY] Serial read error:', exc)
                    self.running.clear()

    def close(self):
        # End reads and join BEFORE closing the underlying file descriptor.
        self.running.clear()
        self.reader.join(timeout=0.5)
        try:
            self.ser.close()
        except (serial.SerialException, OSError):
            pass


def main():
    relays = BB8Relays()
    link = None
    sock = None
    worker = None
    running = threading.Event()
    running.set()
    lock = threading.Lock()
    target = None
    last_head_at = None
    connected = False
    teensy_status_warning = False

    try:
        link = TeensyLink()
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('0.0.0.0', UDP_PORT))
        sock.settimeout(0.1)

        def sender_loop():
            nonlocal teensy_status_warning
            while running.is_set():
                with lock:
                    fresh = (target is not None and last_head_at is not None and
                             time.monotonic() - last_head_at <= HEAD_TIMEOUT)
                    command = target if fresh else None

                # Require recent Teensy STATUS, not a specific navigation state.
                # WAIT_PI is a valid ready state. 'S=' is accepted above.
                ready = link.status_recent()
                if not ready:
                    if not teensy_status_warning:
                        print('[WAIT] No recent Teensy status; relays remain OFF')
                        teensy_status_warning = True
                    relays.disable()
                else:
                    teensy_status_warning = False
                    if command is not None:
                        try:
                            if not relays.enabled:
                                # Arm with a stationary command BEFORE enabling
                                # physical motor power. Never apply a movement
                                # target until the relay contacts have settled.
                                link.send_stop()  # clear any old command
                                link.send_target(0.0, 0.0)
                                relays.enable()  # includes 150 ms contact delay
                            # Recheck freshness after the relay delay.
                            with lock:
                                still_fresh = (target is not None and
                                    last_head_at is not None and
                                    time.monotonic() - last_head_at <= HEAD_TIMEOUT)
                                latest = target
                            if still_fresh and link.status_recent():
                                # First actual drive target is sent only AFTER
                                # the relays are closed; subsequent targets are
                                # resent every 50 ms by this same worker.
                                link.send_target(*latest)
                            else:
                                link.send_stop()
                                relays.disable()
                        except (serial.SerialException, OSError) as exc:
                            print('[FAILSAFE] Teensy write error:', exc)
                            relays.disable()
                time.sleep(SEND_PERIOD)

        worker = threading.Thread(target=sender_loop, daemon=True)
        worker.start()
        print('BB-8 Body Pi receiver started')
        print(f'Listening for Head Pi UDP packets on port {UDP_PORT}')
        print(f'Forwarding valid targets to Teensy at {1 / SEND_PERIOD:.0f} Hz')
        print(f'Head Pi timeout: {HEAD_TIMEOUT:.2f} s')
        print('Waiting for Head Pi...')

        while True:
            try:
                data, address = sock.recvfrom(1024)
                message = data.decode('ascii', errors='strict').strip()
                parts = message.split(',')
                if len(parts) != 2:
                    raise ValueError('expected distance,angle')
                distance, angle = map(float, parts)
                if not (math.isfinite(distance) and math.isfinite(angle) and
                        0 <= distance <= 100 and -180 <= angle <= 180):
                    raise ValueError('out-of-range or non-finite target')
                with lock:
                    target = (distance, angle)
                    last_head_at = time.monotonic()
                if not connected:
                    print(f'[HEAD] Communication established from {address[0]}')
                    connected = True
            except socket.timeout:
                pass
            except (UnicodeDecodeError, ValueError) as exc:
                print('[HEAD] Invalid packet ignored:', exc)

            with lock:
                expired = (last_head_at is None or
                           time.monotonic() - last_head_at > HEAD_TIMEOUT)
                if expired:
                    target = None
            if expired and connected:
                # Stop sender FIRST, then disable power.
                try:
                    link.send_stop()
                except (serial.SerialException, OSError) as exc:
                    print('[FAILSAFE] STOP write error:', exc)
                relays.disable()
                connected = False
                print('[FAILSAFE] HEAD PI COMMUNICATION LOST -> STOP, relays OFF')

    except KeyboardInterrupt:
        print('\nStopping BB-8.')
    finally:
        running.clear()
        with lock:
            target = None
        if worker is not None:
            worker.join(timeout=1.0)
        if link is not None:
            try:
                link.send_stop()
            except (serial.SerialException, OSError):
                pass
        relays.close()
        if link is not None:
            link.close()
        if sock is not None:
            sock.close()
        print('Body Pi receiver stopped safely.')


if __name__ == '__main__':
    main()
