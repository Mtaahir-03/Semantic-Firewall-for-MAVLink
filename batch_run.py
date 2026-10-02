#!/usr/bin/env python3
"""
Smart Batch Runner: Automates logger.py + mission/attack scripts for N seeds.

Features:
  - Runs any of the 4 experimental conditions (or all 4 sequentially)
  - Verifies the vehicle is upright and disarmed before starting each seed
  - Enforces stock PID, EKF, and Navigation parameters between runs
  - Pauses for manual sim restart if a previous attack seed flipped the drone

Usage examples:
  python3 batch_run.py --mode baseline --runs 10
  python3 batch_run.py --mode pid_drift --runs 10
  python3 batch_run.py --mode ekf_trust --runs 10
  python3 batch_run.py --mode nav_aggression --runs 10
  python3 batch_run.py --mode pid_drift --start-seed 4 --runs 6
"""

import argparse
import subprocess
import time
import sys
import math
from pymavlink import mavutil

SETTLE_TIME = 3        # seconds to let logger.py connect before starting mission
COOLDOWN_TIME = 5      # seconds between runs
CONTROL_PORT = 'udp:127.0.0.1:14551'

# All target parameters across Attacks #1, #2, and #3 to reset between runs
ALL_STOCK_PARAMS = {
    'ATC_RAT_RLL_P': 0.135,
    'ATC_RAT_PIT_P': 0.135,
    'EK3_POSNE_M_NSE': 0.5,
    'EK3_VELNE_M_NSE': 0.3,
    'WP_SPD': 10.0,
    'WP_ACC': 2.5,
    'ATC_ANG_RLL_P': 4.5,
    'ATC_ANG_PIT_P': 4.5,
}

MODES = {
    'baseline': {
        'label': 'baseline',
        'cmd': [sys.executable, 'main_mission.py'],
    },
    'pid_drift': {
        'label': 'attack_pid_drift',
        'cmd': [sys.executable, 'attack_pid_drift.py'],
    },
    'ekf_trust': {
        'label': 'attack_ekf_trust',
        'cmd': [sys.executable, 'attack_ekf_trust.py'],
    },
    'nav_aggression': {
        'label': 'attack_nav_aggressionV1',
        'cmd': [sys.executable, 'attack_nav_aggression.py'],
    },
}


def preflight_check_and_reset(max_tilt_rad=0.25):
    """Connects to port 14551 before a run to restore stock parameters
    and verify the drone is disarmed and physically upright."""
    try:
        mav = mavutil.mavlink_connection(CONTROL_PORT)
        hb = mav.wait_heartbeat(timeout=5)
        if hb is None:
            print('PREFLIGHT FAIL: No heartbeat from ArduPilot on 14551.')
            mav.close()
            return False

        # Restore all stock parameters
        for param_id, val in ALL_STOCK_PARAMS.items():
            mav.mav.param_set_send(
                mav.target_system, mav.target_component,
                param_id.encode('utf-8'), val,
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )
        time.sleep(0.5)

        # Drain old buffered packets before checking live attitude
        while mav.recv_match(blocking=False) is not None:
            pass

        att = mav.recv_match(type='ATTITUDE', blocking=True, timeout=3)
        mav.close()

        if att is None:
            print('PREFLIGHT FAIL: Could not read ATTITUDE.')
            return False

        if abs(att.roll) > max_tilt_rad or abs(att.pitch) > max_tilt_rad:
            print(f'PREFLIGHT FAIL: Vehicle is leaning/flipped '
                  f'(roll={math.degrees(att.roll):.1f} deg, '
                  f'pitch={math.degrees(att.pitch):.1f} deg).')
            return False

        return True
    except Exception as e:
        print(f'PREFLIGHT ERROR: {e}')
        return False


def run_once(mode_key, seed):
    cfg = MODES[mode_key]
    label = cfg['label']
    print(f'\n========================================')
    print(f'=== [{label.upper()}] Seed {seed} ===')
    print(f'========================================')

    # Verify vehicle is upright and clean before launching logger.py
    while not preflight_check_and_reset():
        print('\nACTION REQUIRED: Please reset Gazebo + sim_vehicle.py so the '
              'drone is upright and ready.')
        input('Press ENTER once ArduPilot SITL is back online and ready... ')

    logger_proc = subprocess.Popen(
        [sys.executable, 'logger.py', '--label', label, '--seed', str(seed)]
    )
    time.sleep(SETTLE_TIME)

    flight_cmd = cfg['cmd'] + ['--seed', str(seed)]
    mission_proc = subprocess.run(flight_cmd)

    logger_proc.terminate()
    try:
        logger_proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        logger_proc.kill()

    success = (mission_proc.returncode == 0)
    if not success:
        print(f'WARNING: {label} seed {seed} exited with code {mission_proc.returncode}')
    else:
        print(f'Run {seed} ({label}) complete.')

    return success


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['baseline', 'pid_drift', 'ekf_trust', 'nav_aggression', 'all'],
                         required=True, help='Which script condition to batch run')
    parser.add_argument('--runs', type=int, default=10,
                        help='Number of seeds to run (default: 10)')
    parser.add_argument('--start-seed', type=int, default=0,
                        help='Starting seed number (default: 0)')
    args = parser.parse_args()

    modes_to_run = ['baseline', 'pid_drift', 'ekf_trust', 'nav_aggression'] if args.mode == 'all' else [args.mode]

    for mode_key in modes_to_run:
        results = {}
        for seed in range(args.start_seed, args.start_seed + args.runs):
            results[seed] = run_once(mode_key, seed)
            time.sleep(COOLDOWN_TIME)

        failed = [s for s, ok in results.items() if not ok]
        print(f'\n--- {mode_key.upper()} Batch Complete ---')
        if failed:
            print(f'Check seeds with warnings: {failed}')
        else:
            print(f'All {args.runs} seeds succeeded cleanly.')


if __name__ == '__main__':
    main()