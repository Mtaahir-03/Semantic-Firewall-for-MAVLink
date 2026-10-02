#!/usr/bin/env python3
"""
Week 5 ML-Based Anomaly Detector (Isolation Forest on Flight State Telemetry).

Trains an unsupervised Isolation Forest strictly on clean BaselineDatasets
sliding-window flight kinematics (attitude angles, angular rates, groundspeed,
and EKF-vs-GPS divergence), then evaluates Detection Rate, False Positive Rate,
and Time-to-Detect (TTD) across all AttackDatasets using the exact same
metrics as evaluate_detector.py.

Usage:
    python3 ml_detector.py --batch-dir /mnt/hgfs/Dataset
"""

import argparse
import csv
import glob
import math
import os
import re
import statistics
import numpy as np
import pandas as pd
from sklearn.ensemble import IsolationForest
from sklearn.preprocessing import StandardScaler

WINDOW_SIZE_S = 2.0      # 2-second sliding window
STEP_SIZE_S = 1.0        # evaluate every 1 second
CONSECUTIVE_WINDOWS = 2  # require 2 consecutive anomalous windows to trigger alert


def haversine_m(lat1, lon1, lat2, lon2):
    R = 6378137.0
    d_lat = math.radians(lat2 - lat1)
    d_lon = math.radians(lon2 - lon1)
    a = (math.sin(d_lat / 2) ** 2 +
         math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
         math.sin(d_lon / 2) ** 2)
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def load_attack_start(path):
    if path is None or not os.path.exists(path):
        return None
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            if row.get('event') == 'attack_start':
                return float(row['t_wall'])
    return None


def find_matching_ground_truth(telemetry_filename, gt_dir):
    stem = os.path.basename(telemetry_filename)
    m = re.match(r'(.+_seed\d+)_\d{8}_\d{6}\.csv$', stem)
    if not m:
        return None
    prefix = m.group(1)
    candidates = glob.glob(os.path.join(gt_dir, f'{prefix}_*.csv'))
    return candidates[0] if candidates else None


def extract_sliding_windows(csv_path):
    """Pivots raw logger.py telemetry into 2.0s sliding-window feature vectors."""
    df = pd.read_csv(csv_path)
    df['value'] = pd.to_numeric(df['value'], errors='coerce')
    df = df.dropna(subset=['t_wall', 'value'])

    # Extract key physical state streams
    streams = {}
    for msg, fld, scale, col_name in [
        ('ATTITUDE', 'roll', 1.0, 'roll'),
        ('ATTITUDE', 'pitch', 1.0, 'pitch'),
        ('ATTITUDE', 'rollspeed', 1.0, 'rollspeed'),
        ('ATTITUDE', 'pitchspeed', 1.0, 'pitchspeed'),
        ('ATTITUDE', 'yawspeed', 1.0, 'yawspeed'),
        ('VFR_HUD', 'groundspeed', 1.0, 'groundspeed'),
        ('VFR_HUD', 'climb', 1.0, 'climb'),
        ('GLOBAL_POSITION_INT', 'relative_alt', 1e-3, 'alt'),
        ('GLOBAL_POSITION_INT', 'lat', 1e-7, 'ekf_lat'),
        ('GLOBAL_POSITION_INT', 'lon', 1e-7, 'ekf_lon'),
        ('GPS_RAW_INT', 'lat', 1e-7, 'gps_lat'),
        ('GPS_RAW_INT', 'lon', 1e-7, 'gps_lon'),
    ]:
        sub = df[(df['msg_type'] == msg) & (df['field'] == fld)][['t_wall', 'value']].copy()
        sub['value'] *= scale
        streams[col_name] = sub.sort_values('t_wall')

    if streams['roll'].empty:
        return [], np.empty((0, 14))

    t_min = df['t_wall'].min()
    t_max = df['t_wall'].max()

    timestamps = []
    features = []

    t_cur = t_min + WINDOW_SIZE_S
    while t_cur <= t_max:
        t_start = t_cur - WINDOW_SIZE_S

        # Only evaluate windows where the drone is airborne (alt > 1.0m)
        alt_w = streams['alt'][(streams['alt']['t_wall'] >= t_start) &
                               (streams['alt']['t_wall'] <= t_cur)]['value']
        if len(alt_w) == 0 or alt_w.mean() < 1.0:
            t_cur += STEP_SIZE_S
            continue

        row_feats = []
        valid = True
        for col in ['roll', 'pitch', 'rollspeed', 'pitchspeed', 'yawspeed', 'groundspeed', 'climb']:
            s = streams[col][(streams[col]['t_wall'] >= t_start) &
                             (streams[col]['t_wall'] <= t_cur)]['value']
            if len(s) < 2:
                valid = False
                break
            row_feats.extend([s.std(), s.abs().max()])

        # Add EKF vs GPS horizontal divergence feature
        ekf_la = streams['ekf_lat'][(streams['ekf_lat']['t_wall'] >= t_start) & (streams['ekf_lat']['t_wall'] <= t_cur)]['value']
        ekf_lo = streams['ekf_lon'][(streams['ekf_lon']['t_wall'] >= t_start) & (streams['ekf_lon']['t_wall'] <= t_cur)]['value']
        gps_la = streams['gps_lat'][(streams['gps_lat']['t_wall'] >= t_start) & (streams['gps_lat']['t_wall'] <= t_cur)]['value']
        gps_lo = streams['gps_lon'][(streams['gps_lon']['t_wall'] >= t_start) & (streams['gps_lon']['t_wall'] <= t_cur)]['value']

        if valid and len(ekf_la) > 0 and len(gps_la) > 0:
            div_m = haversine_m(ekf_la.mean(), ekf_lo.mean(), gps_la.mean(), gps_lo.mean())
            row_feats.append(div_m)
            timestamps.append(t_cur)
            features.append(row_feats)

        t_cur += STEP_SIZE_S

    return timestamps, np.array(features)


def detect_anomalies(timestamps, preds, attack_start=None):
    """Applies a consecutive-window filter to binary predictions (-1 = anomaly)."""
    alerts = []
    streak = 0
    for t, p in zip(timestamps, preds):
        if p == -1:
            streak += 1
            if streak >= CONSECUTIVE_WINDOWS:
                alerts.append(t)
        else:
            streak = 0

    if attack_start is None:
        return {'false_positives': len(alerts), 'detected': False, 'ttd': None}

    pre = [t for t in alerts if t < attack_start]
    post = [t for t in alerts if t >= attack_start]
    return {
        'false_positives': len(pre),
        'detected': len(post) > 0,
        'ttd': (post[0] - attack_start) if post else None,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--batch-dir', required=True,
                        help='Root dataset directory (/mnt/hgfs/Dataset)')
    parser.add_argument('--contamination', type=float, default=0.01,
                        help='Isolation Forest contamination parameter (default: 0.01)')
    args = parser.parse_args()

    baseline_dir = os.path.join(args.batch_dir, 'BaselineDatasets')
    attack_dir = os.path.join(args.batch_dir, 'AttackDatasets')
    gt_dir = os.path.join(args.batch_dir, 'AttackGroundTruth')

    print('Extracting sliding-window features from BaselineDatasets...')
    baseline_runs = []
    for path in sorted(glob.glob(os.path.join(baseline_dir, '*.csv'))):
        ts, X = extract_sliding_windows(path)
        if len(ts) > 0:
            baseline_runs.append({'name': os.path.basename(path), 'ts': ts, 'X': X})

    # Leave-One-Run-Out cross-validation on Baseline to measure true FPR
    baseline_evals = []
    for i, test_run in enumerate(baseline_runs):
        train_X = np.vstack([r['X'] for j, r in enumerate(baseline_runs) if j != i])
        scaler = StandardScaler().fit(train_X)
        clf = IsolationForest(n_estimators=200, contamination=args.contamination, random_state=42)
        clf.fit(scaler.transform(train_X))
        preds = clf.predict(scaler.transform(test_run['X']))
        baseline_evals.append(detect_anomalies(test_run['ts'], preds, None))

    # Train final model on all 10 Baseline runs for Attack evaluation
    all_train_X = np.vstack([r['X'] for r in baseline_runs])
    scaler = StandardScaler().fit(all_train_X)
    clf = IsolationForest(n_estimators=200, contamination=args.contamination, random_state=42)
    clf.fit(scaler.transform(all_train_X))

    print('Evaluating Isolation Forest on AttackDatasets...\n')
    attack_results = []
    for path in sorted(glob.glob(os.path.join(attack_dir, '*.csv'))):
        stem = os.path.basename(path)
        gt_path = find_matching_ground_truth(path, gt_dir)
        attack_start = load_attack_start(gt_path)
        m = re.match(r'(.+)_seed\d+_\d{8}_\d{6}\.csv$', stem)
        category = m.group(1) if m else 'unknown_attack'

        ts, X = extract_sliding_windows(path)
        preds = clf.predict(scaler.transform(X)) if len(ts) > 0 else []
        res = detect_anomalies(ts, preds, attack_start)
        res['label'] = stem
        res['category'] = category
        attack_results.append(res)

        status = f"DETECTED (TTD: {res['ttd']:.2f}s)" if res['detected'] else "MISSED DETECTION"
        print(f"=== {stem} ===\n  Windows: {len(ts)} | Pre-attack FP: {res['false_positives']} | {status}")

    print('\n' + '=' * 70)
    print('ISOLATION FOREST (ML DETECTOR) SUMMARY')
    print('=' * 70)
    clean_base = sum(1 for r in baseline_evals if r['false_positives'] == 0)
    total_base_fp = sum(r['false_positives'] for r in baseline_evals)
    print(f"\nBaseline runs (Leave-One-Out CV): {len(baseline_evals)}")
    print(f"  Clean (zero false positives): {clean_base}/{len(baseline_evals)}")
    print(f"  Total false positive alerts: {total_base_fp}")

    for cat in sorted(set(r['category'] for r in attack_results)):
        cat_r = [r for r in attack_results if r['category'] == cat]
        n = len(cat_r)
        det = sum(1 for r in cat_r if r['detected'])
        ttds = [r['ttd'] for r in cat_r if r['ttd'] is not None]
        fps = sum(r['false_positives'] for r in cat_r)
        print(f"\n{cat}:")
        print(f"  Runs evaluated: {n}")
        print(f"  Detection rate: {det}/{n} ({100*det/n:.0f}%)")
        print(f"  Pre-attack false positives: {fps}")
        if ttds:
            print(f"  Time-to-detect -- mean: {statistics.mean(ttds):.2f}s, "
                  f"median: {statistics.median(ttds):.2f}s, "
                  f"min: {min(ttds):.2f}s, max: {max(ttds):.2f}s"
                  + (f", sd: {statistics.stdev(ttds):.2f}s" if len(ttds) > 1 else ""))


if __name__ == '__main__':
    main()