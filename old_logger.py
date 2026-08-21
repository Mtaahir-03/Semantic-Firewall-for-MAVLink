#!/usr/bin/env python3
"""
Baseline MAVLink telemetry logger.
Connects to a running SITL instance and logs key flight state
messages to a timestamped CSV for later analysis / ML training.
"""

import csv
import time
from pymavlink import mavutil

# --- Configuration ---
CONNECTION_STRING = 'udp:127.0.0.1:14550'  # MAVProxy's --out endpoint
LOG_LABEL = 'baseline'                      # change to attack name during Phase 2
OUTPUT_DIR = '/mnt/hgfs/Dataset/BaselineDatasets'

# Messages we care about for control-aware attack detection later:
# ATTITUDE (roll/pitch/yaw + rates), PID tuning data if enabled,
# EKF status, and raw IMU for completeness.
MESSAGE_TYPES = ['ATTITUDE', 'EKF_STATUS_REPORT', 'VFR_HUD', 'GLOBAL_POSITION_INT']


def main():
    import os
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    timestamp = time.strftime('%Y%m%d_%H%M%S')
    filename = f'{OUTPUT_DIR}/{LOG_LABEL}_{timestamp}.csv'

    print(f'Connecting to {CONNECTION_STRING} ...')
    mav = mavutil.mavlink_connection(CONNECTION_STRING)
    mav.wait_heartbeat()
    print(f'Heartbeat received from system {mav.target_system}, component {mav.target_component}')

    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['t_wall', 't_boot_ms', 'msg_type', 'field', 'value'])

        print(f'Logging to {filename} ... Ctrl+C to stop.')
        try:
            while True:
                msg = mav.recv_match(type=MESSAGE_TYPES, blocking=True, timeout=5)
                if msg is None:
                    continue
                t_wall = time.time()
                msg_dict = msg.to_dict()
                t_boot = msg_dict.get('time_boot_ms', '')
                msg_type = msg_dict.pop('mavpackettype', msg.get_type())
                for field, value in msg_dict.items():
                    if field == 'time_boot_ms':
                        continue
                    writer.writerow([t_wall, t_boot, msg_type, field, value])
                f.flush()
        except KeyboardInterrupt:
            print(f'\nStopped. Saved to {filename}')


if __name__ == '__main__':
    main()
