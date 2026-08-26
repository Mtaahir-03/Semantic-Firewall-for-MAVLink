#!/usr/bin/env python3
"""
Batch runner: launches logger.py + mission.py as subprocesses for N runs,
each with a different seed, producing a labeled set of baseline flight logs.

Assumes Gazebo + sim_vehicle.py are ALREADY running and stable —
this script only automates the logger/mission pairing, not the sim stack itself.
"""
from pymavlink import mavutil
import subprocess
import time
import sys

N_RUNS = 5            # start small to test, scale up once confirmed working
LABEL = 'baseline'
SETTLE_TIME = 3        # seconds to let the logger connect before starting mission
COOLDOWN_TIME = 5      # seconds between runs, lets the vehicle settle/reset

def wait_for_controller_ready(timeout=60):
    print('Waiting for controller to finish rebooting after reset...')
    try:
        mav = mavutil.mavlink_connection('udp:127.0.0.1:14550', source_system=250)
        mav.wait_heartbeat(timeout=timeout)
        print('Heartbeat received — now waiting for EKF/GPS to settle...')
        # Give the EKF/GPS genuine time to align, not just a heartbeat.
        # This matches the real init sequence (barometer cal, EKF align, GPS lock)
        # we've seen take 8-12s in your logs.
        time.sleep(15)
        mav.close()
        print('Controller ready for next run.')
        return True
    except Exception as e:
        print(f'WARNING: controller did not come back cleanly: {e}')
        return False

def reset_world():
    subprocess.run([
        'gz', 'service', '-s', '/world/iris_runway/control',
        '--reqtype', 'gz.msgs.WorldControl',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '3000',
        '--req', 'reset: {all: true}'
    ])
    wait_for_controller_ready()

def run_once(seed):
    print(f'\n=== Run {seed} ===')

    logger_proc = subprocess.Popen(
        [sys.executable, 'logger.py', '--label', LABEL, '--seed', str(seed)]
    )
    time.sleep(SETTLE_TIME)

    mission_proc = subprocess.run(
        [sys.executable, 'mission.py', '--seed', str(seed)]
    )

    logger_proc.terminate()
    logger_proc.wait(timeout=10)

    success = mission_proc.returncode == 0
    if not success:
        print(f'WARNING: run {seed} FAILED (exit code {mission_proc.returncode}) — '
              f'discard this CSV. Resetting world to recover clean state...')
        reset_world()
    else:
        print(f'Run {seed} complete — vehicle landed normally, no reset needed.')

    return success

def main():
    results = {}
    for seed in range(N_RUNS):
        results[seed] = run_once(seed)
        time.sleep(COOLDOWN_TIME)

    print('\nBatch complete.')
    failed = [s for s, ok in results.items() if not ok]
    if failed:
        print(f'Failed seeds (delete these CSVs): {failed}')
    else:
        print('All runs succeeded.')


if __name__ == '__main__':
    main()
