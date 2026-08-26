#!/usr/bin/env python3
"""
Scripted autonomous mission with randomized variation.
Accepts a --seed argument so each batch run differs slightly,
producing a more representative 'normal flight' baseline dataset.
"""
import math
import time
import random
import argparse
from pymavlink import mavutil

CONNECTION_STRING = 'udp:127.0.0.1:14551'

HOME_LAT = -35.363262
HOME_LON = 149.165237


def wait_ack(mav, command_name):
    ack = mav.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    if ack is None:
        print(f'No ACK received for {command_name}')
    else:
        print(f'{command_name}: {mavutil.mavlink.enums["MAV_RESULT"][ack.result].name}')


def set_mode(mav, mode_name):
    mode_id = mav.mode_mapping()[mode_name]
    mav.mav.set_mode_send(mav.target_system,
                           mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                           mode_id)
    time.sleep(1)


def arm(mav):
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0)
    wait_ack(mav, 'ARM')


def takeoff(mav, alt):
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, alt)
    wait_ack(mav, 'TAKEOFF')


def goto(mav, lat, lon, alt):
    mav.mav.set_position_target_global_int_send(
        0, mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,
        int(lat * 1e7), int(lon * 1e7), alt,
        0, 0, 0, 0, 0, 0, 0, 0)


def wait_until_altitude(mav, target_alt, tolerance=0.5, timeout=30):
    start = time.time()
    while time.time() - start < timeout:
        msg = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        if msg:
            alt = msg.relative_alt / 1000.0
            if abs(alt - target_alt) < tolerance:
                return True
    return False


def land(mav):
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0)
    wait_ack(mav, 'LAND')


def generate_waypoints(seed, n_points=3):
    """Generate a randomized but sensible waypoint loop around home."""
    rng = random.Random(seed)
    waypoints = []
    for _ in range(n_points):
        # small random offsets, ~50-150m in lat/lon terms
        d_lat = rng.uniform(-0.0012, 0.0012)
        d_lon = rng.uniform(-0.0012, 0.0012)
        waypoints.append((HOME_LAT + d_lat, HOME_LON + d_lon))
    return waypoints

def get_distance_metres(lat1, lon1, lat2, lon2):
    """Calculate the distance in meters between two GPS coordinates."""
    R = 6378137.0  # Radius of Earth in meters
    d_lat = math.radians(lat2 - lat1)
    d_lon = math.radians(lon2 - lon1)
    
    a = (math.sin(d_lat / 2) * math.sin(d_lat / 2) +
         math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
         math.sin(d_lon / 2) * math.sin(d_lon / 2))
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    return R * c

def wait_until_position(mav, target_lat, target_lon, tolerance=2.0, timeout=60):
    """Blocks execution until the drone is within 'tolerance' meters of the target."""
    start = time.time()
    while time.time() - start < timeout:
        msg = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        if msg:
            # GLOBAL_POSITION_INT reports coordinates in degrees * 1e7
            current_lat = msg.lat / 1e7
            current_lon = msg.lon / 1e7
            
            dist = get_distance_metres(current_lat, current_lon, target_lat, target_lon)
            if dist < tolerance:
                return True
    return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--alt', type=float, default=None,
                         help='Takeoff altitude; randomized if not set')
    args = parser.parse_args()

    rng = random.Random(args.seed)
    takeoff_alt = args.alt if args.alt is not None else rng.uniform(8, 15)
    dwell_time = rng.uniform(6, 10)
    waypoints = generate_waypoints(args.seed)

    print(f'[seed={args.seed}] takeoff_alt={takeoff_alt:.1f}m, dwell={dwell_time:.1f}s, '
          f'waypoints={len(waypoints)}')

    mav = mavutil.mavlink_connection(CONNECTION_STRING)
    mav.wait_heartbeat()
    print(f'Connected: system {mav.target_system}')

    set_mode(mav, 'GUIDED')
    arm(mav)
    takeoff(mav, takeoff_alt)
    wait_until_altitude(mav, takeoff_alt)
    print(f'Reached {takeoff_alt:.1f}m, holding for {dwell_time:.1f}s')
    time.sleep(dwell_time)

    for lat, lon in waypoints:
        print(f'Flying to {lat:.6f}, {lon:.6f}')
        goto(mav, lat, lon, takeoff_alt)
        
        # Block until the drone physically reaches the coordinate
        if wait_until_position(mav, lat, lon):
            print(f'Reached waypoint, holding for {dwell_time:.1f}s')
        else:
            print('WARNING: Waypoint timeout. Moving to next coordinate.')
            
        # Execute the hover duration AFTER arriving
        time.sleep(dwell_time)

    print('Returning to land')
    land(mav)
    time.sleep(10)
    print('Mission complete')


if __name__ == '__main__':
    main()
