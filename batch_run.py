#!/usr/bin/env python3
"""
Batch runner: launches logger.py + mission.py as subprocesses for N runs,
each with a different seed, producing a labeled set of baseline flight logs.

Assumes Gazebo + sim_vehicle.py are ALREADY running and stable —
this script only automates the logger/mission pairing, not the sim stack itself.
"""

import subprocess
import time
import sys

N_RUNS = 5            # start small to test, scale up once confirmed working
LABEL = 'baseline'
SETTLE_TIME = 3        # seconds to let the logger connect before starting mission
COOLDOWN_TIME = 5      # seconds between runs, lets the vehicle settle/reset


def run_once(seed):
    print(f'\n=== Run {seed} ===')

    logger_proc = subprocess.Popen(
        [sys.executable, 'logger.py', '--label', LABEL, '--seed', str(seed)]
    )
    time.sleep(SETTLE_TIME)

    mission_proc = subprocess.run(
        [sys.executable, 'main_mission.py', '--seed', str(seed)]
    )

    if mission_proc.returncode != 0:
        print(f'WARNING: mission.py exited with code {mission_proc.returncode} on seed {seed}')

    logger_proc.terminate()
    logger_proc.wait(timeout=10)
    print(f'Run {seed} complete.')


def main():
    for seed in range(N_RUNS):
        run_once(seed)
        time.sleep(COOLDOWN_TIME)
    print('\nBatch complete.')


if __name__ == '__main__':
    main()
