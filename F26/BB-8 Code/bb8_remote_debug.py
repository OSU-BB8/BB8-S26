#!/usr/bin/env python3
"""Wi-Fi remote emulator for BB-8; STATUS prints the Body Pi response."""
import json
import socket
import time
from pathlib import Path

c = json.loads((Path(__file__).resolve().parent / 'config.json').read_text())
address = (c['body_host'], c['body_port'])
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(1.5)
print('Remote emulator: ps5, beacon, apriltag, stop, estop, status, quit')
try:
    while True:
        try:
            cmd = input('remote> ').strip().upper()
        except (EOFError, KeyboardInterrupt):
            break
        if cmd == 'QUIT':
            break
        if cmd not in ('PS5', 'BEACON', 'APRILTAG', 'STOP', 'ESTOP', 'STATUS'):
            print('Unknown command')
            continue
        packet = {'token': c['token'],
                  'command': 'START' if cmd in ('PS5', 'BEACON', 'APRILTAG') else cmd,
                  'mode': cmd}
        if cmd == 'STATUS':
            # Clear any delayed replies before making a new request.
            s.setblocking(False)
            try:
                while True:
                    s.recvfrom(4096)
            except BlockingIOError:
                pass
            finally:
                s.settimeout(1.5)
            for attempt in range(3):
                s.sendto(json.dumps(packet).encode(), address)
                try:
                    while True:
                        data, sender = s.recvfrom(4096)
                        if sender[0] != c['body_host']:
                            continue
                        reply = json.loads(data)
                        if reply.get('type') == 'STATUS_REPLY':
                            print('\n--- BB-8 STATUS ---')
                            for key, value in reply.items():
                                if key != 'type':
                                    print(f'{key.replace("_", " ").title()}: {value}')
                            print('-------------------\n')
                            break
                    break
                except (socket.timeout, ValueError, UnicodeDecodeError):
                    if attempt == 2:
                        print('No status response from Body Pi. Check its service, IP and UDP port.')
        else:
            for _ in range(3):
                s.sendto(json.dumps(packet).encode(), address)
                time.sleep(0.07)
            print(f'Sent {cmd} to {address[0]}:{address[1]}')
finally:
    s.close()
