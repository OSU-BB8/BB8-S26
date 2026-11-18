#!/usr/bin/env python3
"""BB-8 test stand safe receiver. Default DRY RUN, no GPIO is energized.

Status is sent by bb8_body_mirror.py. Movement packets require adding a
sender to the main movement scripts. The actuator adapter is intentionally
unimplemented until the original motor-driver wiring/mode is verified.
"""
import argparse
import json
import socket
import time

TIMEOUT = 0.6
MODES = {'PS5', 'BEACON', 'APRILTAG', 'DEMO'}

class Stand:
    def __init__(self):
        self.enabled = False
        self.mode = None
        self.last_status = 0.0
        self.last_motion = 0.0
        self.last_seq = -1
        self.last_output = None

    def stop(self):
        if self.last_output != ('STOP',):
            print('[TEST] STOP (GPIO disabled; dry-run only)', flush=True)
        self.last_output = ('STOP',)

    def handle(self, packet, now):
        if packet.get('protocol') != 'bb8-mirror-v1':
            return
        if packet.get('kind') == 'status':
            seq = packet.get('seq')
            if not isinstance(seq, int) or seq <= self.last_seq:
                return
            self.last_seq = seq
            self.last_status = now
            self.enabled = packet.get('enabled') is True and packet.get('state') == 'RUNNING'
            self.mode = packet.get('mode') if packet.get('mode') in MODES else None
            if not self.enabled or self.mode is None:
                self.stop()
        elif packet.get('kind') == 'motion':
            if not self.enabled or now - self.last_status > TIMEOUT:
                return
            if packet.get('mode') != self.mode:
                return
            # Packet format and scaling must be finalized against the actual
            # PS5_Controller.py and Beacon_Target_Control.py implementations.
            self.last_motion = now
            print('[TEST] Motion preview:', packet, flush=True)

    def watchdog(self, now):
        if now - self.last_status > TIMEOUT or now - self.last_motion > TIMEOUT:
            self.stop()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5015)
    parser.add_argument('--token', required=True, help='Same token as main Body Pi config.json')
    args = parser.parse_args()
    stand = Stand()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', args.port))
    sock.settimeout(.1)
    print('[TEST] Listening on UDP port', args.port, '(DRY RUN)', flush=True)
    try:
        while True:
            try:
                data, addr = sock.recvfrom(2048)
                if len(data) > 1500:
                    continue
                msg = json.loads(data)
                if isinstance(msg, dict) and msg.get('token') == args.token:
                    stand.handle(msg, time.monotonic())
            except (ValueError, UnicodeError):
                pass
            except socket.timeout:
                pass
            stand.watchdog(time.monotonic())
    except KeyboardInterrupt:
        pass
    finally:
        stand.stop()
        sock.close()

if __name__ == '__main__':
    main()
