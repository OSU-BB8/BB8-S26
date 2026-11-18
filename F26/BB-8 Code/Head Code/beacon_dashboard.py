#!/usr/bin/env python3
"""Read-only beacon dashboard. No GPIO, serial ports, or movement commands."""
import argparse
from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import socket
import threading
import time
from urllib.parse import urlsplit

PAGE = Path(__file__).with_name('beacon_dashboard.html')
STALE_SECONDS = 0.75
# Mirror the target-following thresholds in BB8_Teensy_Controller.ino.
STOP_RADIUS_FT = 4.0
START_DISTANCE_FT = 5.0


def parse_telemetry(raw):
    fields = raw.decode('ascii').strip().split(',')
    if len(fields) != 9 or fields[8] not in ('GOOD', 'HOLD'):
        raise ValueError('Expected nine-field beacon display packet')
    values = [float(value) for value in fields[:8]]
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Non-finite telemetry')
    distance, angle, dc, ac, *counts = values
    if not (0 <= distance <= 100 and -180 <= angle <= 180):
        raise ValueError('Invalid position')
    if not (0 <= dc <= 100 and 0 <= ac <= 100) or any(v < 0 for v in counts):
        raise ValueError('Invalid confidence/count')
    return dict(distance_ft=distance, angle_deg=angle,
                distance_confidence=dc, angle_confidence=ac, source_state=fields[8])


def route_preview(distance, angle):
    """Geometric preview only: firmware has no route or odometry telemetry."""
    radians = math.radians(angle)
    x, y = distance * math.sin(radians), distance * math.cos(radians)
    travel = max(0.0, distance - STOP_RADIUS_FT)
    endpoint = [travel * math.sin(radians), travel * math.cos(radians)]
    if distance <= STOP_RADIUS_FT:
        action = 'Within stopping circle'
    elif distance < START_DISTANCE_FT:
        action = 'Start/stop hysteresis zone'
    elif abs(angle) >= 65 or (distance <= 10 and abs(angle) >= 30):
        action = 'Pivot, then approach'
    elif abs(angle) <= 6:
        action = 'Straight approach'
    else:
        action = 'Arc approach'
    return dict(beacon=[x, y], endpoint=endpoint, travel_ft=travel, action=action,
                stop_radius_ft=STOP_RADIUS_FT)


class Telemetry:
    def __init__(self):
        self.lock = threading.Lock()
        self.latest = None
        self.received_at = None

    def update(self, raw):
        packet = parse_telemetry(raw)
        with self.lock:
            self.latest, self.received_at = packet, time.monotonic()

    def snapshot(self):
        with self.lock:
            if self.latest is None:
                return dict(connection='waiting', target=None, age_s=None)
            age = time.monotonic() - self.received_at
            packet = dict(self.latest)
        state = 'stale' if age > STALE_SECONDS else packet['source_state'].lower()
        # Never present a stale/held measurement as a current planned route.
        return dict(connection=state, age_s=round(age, 2), target=packet,
                    route=route_preview(packet['distance_ft'], packet['angle_deg'])
                    if state == 'good' else None)


def receive(sock, telemetry, stopped):
    while not stopped.is_set():
        try:
            raw, _ = sock.recvfrom(2048)
            telemetry.update(raw)
        except socket.timeout:
            continue
        except (ValueError, UnicodeError):
            continue


class Handler(BaseHTTPRequestHandler):
    def __init__(self, *args, telemetry, **kwargs):
        self.telemetry = telemetry
        super().__init__(*args, **kwargs)

    def do_GET(self):
        path = urlsplit(self.path).path
        if path == '/api/telemetry':
            content = json.dumps(self.telemetry.snapshot(), allow_nan=False).encode()
            mime = 'application/json'
        elif path in ('/', '/index.html'):
            content = PAGE.read_bytes()
            mime = 'text/html; charset=utf-8'
        else:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header('Content-Type', mime)
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Content-Length', str(len(content)))
        self.end_headers()
        try:
            self.wfile.write(content)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, *_):
        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8090)
    parser.add_argument('--udp-port', type=int, default=5006)
    args = parser.parse_args()
    telemetry, stopped = Telemetry(), threading.Event()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        # Telemetry stays on Head Pi; browsers only need the HTTP port.
        sock.bind(('127.0.0.1', args.udp_port))
        sock.settimeout(0.2)
        with ThreadingHTTPServer((args.host, args.port),
                partial(Handler, telemetry=telemetry)) as server:
            worker = threading.Thread(target=receive, args=(sock, telemetry, stopped), daemon=True)
            worker.start()
            print(f'Beacon dashboard listening on {args.host}:{args.port}', flush=True)
            try:
                server.serve_forever(poll_interval=0.2)
            except KeyboardInterrupt:
                pass
            finally:
                stopped.set()
                worker.join(timeout=1)


if __name__ == '__main__':
    main()
