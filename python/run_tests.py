"""Run repeatable motor PID step tests over Arduino USB serial.

Example: python run_tests.py --port COM3 --speeds 50 100 150 --duration 8
Use Ctrl+C to abort (a STOP is sent in finally).
"""
import argparse
import csv
from datetime import datetime
from pathlib import Path
import time

import serial

FIELDS = [
    'time_ms',
    'target_rpm',
    'actual_rpm',
    'pwm',
    'error_rpm',
    'pid_raw'
]

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True, help='Serial port e.g. COM3 or /dev/ttyACM0')
    parser.add_argument('--speeds', type=float, nargs='+', default=[50, 100, 150])
    parser.add_argument('--duration', type=float, default=8, help='Seconds at each speed')
    parser.add_argument('--baseline', type=float, default=2, help='Seconds stopped at start')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--output', type=Path, default=Path('results'))
    args = parser.parse_args()
    if any(s <= 0 or s > 170 for s in args.speeds):
        parser.error('All speed targets must be between 0 and 170 RPM')
    args.output.mkdir(parents=True, exist_ok=True)
    destination = args.output / f'pid_test_{datetime.now():%Y%m%d_%H%M%S}.csv'
    phases = [('baseline', 0.0, args.baseline)] + [
        (f'step_{i}', rpm, args.duration) for i, rpm in enumerate(args.speeds, 1)
    ] + [('cooldown', 0.0, 2.0)]

    rows = []
    with serial.Serial(args.port, args.baud, timeout=0.1, write_timeout=0.5) as ser:
        # Opening a serial port usually resets an Arduino Uno.
        ready = False
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            line = ser.readline().decode('ascii', errors='replace').strip()
            if line.startswith('READY,MOTOR_PID_V2'):
                ready = True
                break
        if not ready:
            raise RuntimeError('Arduino V2 READY not received. Check port, firmware, and baud rate.')
        print('Connected to', args.port)

        def send(command):
            ser.write((command + '\n').encode('ascii'))
            ser.flush()
            print('>', command)

        try:
            #send('CLEAR')
            last_ping = time.monotonic()
            for phase_name, speed, seconds in phases:
                send('STOP' if speed == 0 else f'SET,{speed:g}')
                phase_start = time.monotonic()
                print('Phase:', phase_name, '| target:', speed, 'RPM')
                while time.monotonic() - phase_start < seconds:
                    if time.monotonic() - last_ping >= 0.5:
                       # send('PING')
                        last_ping = time.monotonic()
                    line = ser.readline().decode('ascii', errors='replace').strip()
                    if not line:
                        continue
                    if line.startswith('FAULT,') or line.startswith('ERR,'):
                        raise RuntimeError(f'Firmware alert: {line}')
                    if not line.startswith('DATA,'):
                        continue
                    fields = line.split(',')
                    if len(fields) != len(FIELDS) + 1:
                        continue
                    try:
                        sample = dict(zip(FIELDS, [float(x) for x in fields[1:]]))
                    except ValueError:
                        continue
                    sample.update(phase=phase_name, phase_time_s=time.monotonic() - phase_start)
                    rows.append(sample)
        finally:
            try:
                send('STOP')
            except (serial.SerialException, OSError):
                pass  # Arduino watchdog is backup if disconnected
            if rows:
                with destination.open('w', newline='') as f:
                    writer = csv.DictWriter(f, fieldnames=['phase', 'phase_time_s'] + FIELDS)
                    writer.writeheader()
                    writer.writerows(rows)
                print(f'Saved {len(rows)} samples to {destination}')
    if rows:
        from analyze_results import analyze
        analyze(destination)


if __name__ == '__main__':
    main()
