#!/usr/bin/env python3
"""
MAVLink telemetry & parameter logger for Baseline and Attack datasets.
Connects to a running SITL instance and logs key flight state and parameter
messages to a timestamped CSV for rule-based and ML anomaly detection.
"""

import csv
import time
import os
import signal
import argparse
from pymavlink import mavutil

parser = argparse.ArgumentParser()
parser.add_argument('--label', default='baseline',
                    help='Dataset label prefix (e.g., baseline, attack_pid_drift, attack_ekf_trust)')
parser.add_argument('--seed', type=int, default=0)
parser.add_argument('--output-dir', default=None,
                    help='Optional custom output directory')
args = parser.parse_args()

# --- Configuration ---
CONNECTION_STRING = 'udp:127.0.0.1:14550'
LOG_LABEL = f'{args.label}_seed{args.seed}'

# Automatically route baseline vs. attack logs if --output-dir is not specified
if args.output_dir:
    OUTPUT_DIR = args.output_dir
elif args.label == 'baseline':
    OUTPUT_DIR = '/mnt/hgfs/Dataset/BaselineDatasets'
else:
    OUTPUT_DIR = '/mnt/hgfs/Dataset/AttackDatasets'

MESSAGE_TYPES = [
    'ATTITUDE',
    'EKF_STATUS_REPORT',
    'VFR_HUD',
    'GLOBAL_POSITION_INT',
    'GPS_RAW_INT',
    'PARAM_VALUE',
]

# Only log numeric fields from GPS_RAW_INT Relevant to EKF comparison & failsafes
GPS_RAW_FIELDS = {'lat', 'lon', 'alt', 'eph', 'epv', 'vel', 'fix_type', 'satellites_visible'}

running = True


def handle_sigterm(signum, frame):
    """Allow batch_run.py's terminate() call to close the CSV cleanly."""
    global running
    running = False


signal.signal(signal.SIGTERM, handle_sigterm)


def request_streams(mav, rate_hz=4):
    """Ensure ArduPilot actively streams all required telemetry groups."""
    mav.mav.request_data_stream_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        rate_hz,
        1
    )


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    timestamp = time.strftime('%Y%m%d_%H%M%S')
    filename = f'{OUTPUT_DIR}/{LOG_LABEL}_{timestamp}.csv'

    print(f'Connecting to {CONNECTION_STRING} ...')
    mav = mavutil.mavlink_connection(CONNECTION_STRING)
    mav.wait_heartbeat()
    print(f'Heartbeat received from system {mav.target_system}, component {mav.target_component}')

    request_streams(mav, rate_hz=4)

    last_t_boot_ms = ''

    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['t_wall', 't_boot_ms', 'msg_type', 'field', 'value'])

        print(f'Logging to {filename} ... Ctrl+C to stop.')
        try:
            while running:
                msg = mav.recv_match(type=MESSAGE_TYPES, blocking=True, timeout=1)
                if msg is None:
                    continue

                t_wall = time.time()
                msg_dict = msg.to_dict()
                msg_type = msg_dict.pop('mavpackettype', msg.get_type())

                # Track monotonic simulation boot time in milliseconds
                if 'time_boot_ms' in msg_dict:
                    last_t_boot_ms = msg_dict.pop('time_boot_ms')
                elif 'time_usec' in msg_dict:
                    last_t_boot_ms = int(msg_dict.pop('time_usec') / 1000)

                t_boot = last_t_boot_ms

                # Format PARAM_VALUE as field=<param_name>, value=<float>
                if msg_type == 'PARAM_VALUE':
                    param_name = str(msg_dict.get('param_id', '')).strip('\x00')
                    param_val = msg_dict.get('param_value', 0.0)
                    if param_name:
                        writer.writerow([t_wall, t_boot, msg_type, param_name, param_val])
                elif msg_type == 'GPS_RAW_INT':
                    for field, value in msg_dict.items():
                        if field in GPS_RAW_FIELDS:
                            writer.writerow([t_wall, t_boot, msg_type, field, value])
                else:
                    for field, value in msg_dict.items():
                        writer.writerow([t_wall, t_boot, msg_type, field, value])

                f.flush()
        except KeyboardInterrupt:
            pass
        finally:
            f.flush()
            print(f'\nStopped. Saved to {filename}')


if __name__ == '__main__':
    main()