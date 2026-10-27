#!/usr/bin/env python3
"""BB-8 movement demo with Body Pi-controlled relays.
Run from the same folder as BB8_Relays.py. No other process may own /dev/serial0.
"""
import time
import serial
from BB8_Relays import BB8Relays

PORT = '/dev/serial0'
BAUD = 115200
SEND_PERIOD = 0.05  # 20 Hz, Teensy watchdog is 500 ms
MOVE_TIME = 1.0
DISTANCE = 20.0

movements = [
    ('FORWARD', DISTANCE, 0), ('FORWARD', DISTANCE, 0),
    ('FORWARD', DISTANCE, 0), ('FORWARD', DISTANCE, 0),
    ('FRONT RIGHT', DISTANCE, 5), ('FRONT RIGHT', DISTANCE, 10),
    ('FRONT RIGHT', DISTANCE, 15), ('FRONT RIGHT', DISTANCE, 20),
    ('RIGHT', DISTANCE, 90), ('RIGHT', DISTANCE, 90),
    ('RIGHT', DISTANCE, 90), ('RIGHT', DISTANCE, 90),
    ('FORWARD', DISTANCE, 0), ('FORWARD', DISTANCE, 0),
    ('FORWARD', DISTANCE, 0), ('FORWARD', DISTANCE, 0),
    ('FRONT LEFT', DISTANCE, -5), ('FRONT LEFT', DISTANCE, -10),
    ('FRONT LEFT', DISTANCE, -15), ('FRONT LEFT', DISTANCE, -20),
    ('LEFT', DISTANCE, -90), ('LEFT', DISTANCE, -90),
    ('LEFT', DISTANCE, -90), ('LEFT', DISTANCE, -90),
]

def send_target(ser, distance, angle):
    ser.write(f'T,{distance:.3f},{angle:.3f}\n'.encode('ascii'))

def run_target(ser, name, distance, angle, duration):
    print(f'{name:<15} | D={distance:5.1f} ft | A={angle:6.1f} deg')
    end = time.monotonic() + duration
    while time.monotonic() < end:
        send_target(ser, distance, angle)
        time.sleep(min(SEND_PERIOD, max(0, end - time.monotonic())))

def main():
    relays = BB8Relays()  # default OFF
    ser = None
    try:
        print('Opening Teensy serial connection...')
        ser = serial.Serial(PORT, BAUD, timeout=0.05, write_timeout=0.1)
        time.sleep(0.3)
        # Make sure the Teensy cannot run an old command.
        ser.write(b'STOP\n')
        ser.flush()
        print('Enabling relays and allowing contacts to settle...')
        relays.enable()  # 150 ms delay inside helper
        # Arm Teensy with a non-moving target, not a movement command.
        # This keeps the Teensy in a known, safe zero-drive state.
        run_target(ser, 'ARM / IDLE', 0.0, 0.0, 0.25)
        print('BB-8 movement demo running; Ctrl+C stops.')
        while True:
            for name, distance, angle in movements:
                run_target(ser, name, distance, angle, MOVE_TIME)
    except KeyboardInterrupt:
        print('\nDemo interrupted.')
    finally:
        print('Stopping motors and disabling relays...')
        if ser is not None:
            try:
                ser.write(b'STOP\n')
                ser.flush()
            except (serial.SerialException, OSError):
                pass
        relays.disable()
        if ser is not None:
            ser.close()
        relays.close()
        print('Demo ended.')

if __name__ == '__main__':
    main()
