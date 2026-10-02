#!/usr/bin/env python3
"""
Batch evaluation harness for rule_detector.py.

Single-run mode (unchanged from before):
    python3 evaluate_detector.py --telemetry X.csv [--ground-truth Y.csv] --label "..."

Batch mode (NEW):
    python3 evaluate_detector.py --batch-dir /mnt/hgfs/Dataset

Batch mode expects the standard directory layout:
    <batch-dir>/BaselineDatasets/*.csv           (telemetry only, no ground truth)
    <batch-dir>/AttackDatasets/*.csv              (telemetry, one per run)
    <batch-dir>/AttackGroundTruth/*.csv           (matching ground truth, same
                                                    seed/label/timestamp convention)

It pairs each AttackDatasets telemetry file with its AttackGroundTruth file by
matching on the shared "<label>_seed<N>_<timestamp>" stem, runs the detector
on every file, and prints a single summary table: Detection Rate, False
Positive Rate, and Time-to-Detect statistics, broken down by attack label.
"""

import argparse
import csv
import glob
import os
import re
import statistics
import sys
from rule_detector import RuleBasedDetector


def load_telemetry_param_events(path):
    events = []
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row.get('msg_type') != 'PARAM_VALUE':
                continue
            try:
                t = float(row['t_wall'])
                value = float(row['value'])
            except (ValueError, KeyError, TypeError):
                continue
            events.append((t, row['field'], value))
    events.sort(key=lambda e: e[0])
    return events


def load_attack_start(path):
    if path is None or not os.path.exists(path):
        return None
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row.get('event') == 'attack_start':
                return float(row['t_wall'])
    return None


def evaluate_run(telemetry_path, ground_truth_path, label):
    events = load_telemetry_param_events(telemetry_path)
    if not events:
        return {'label': label, 'error': 'no PARAM_VALUE events found',
                'n_events': 0, 'alerts': []}

    attack_start = load_attack_start(ground_truth_path)
    is_attack_run = attack_start is not None

    detector = RuleBasedDetector()
    all_alerts = []
    for t, param, value in events:
        all_alerts.extend(detector.process_param_event(t, param, value))

    result = {
        'label': label,
        'n_events': len(events),
        'is_attack_run': is_attack_run,
        'attack_start': attack_start,
        'alerts': all_alerts,
    }

    if is_attack_run:
        pre = [a for a in all_alerts if a['t'] < attack_start]
        post = [a for a in all_alerts if a['t'] >= attack_start]
        result['false_positives'] = len(pre)
        result['detected'] = len(post) > 0
        result['ttd'] = (post[0]['t'] - attack_start) if post else None
        result['trigger_rule'] = post[0]['rule'] if post else None
    else:
        result['false_positives'] = len(all_alerts)
        result['detected'] = None
        result['ttd'] = None
        result['trigger_rule'] = None

    return result


def print_single(result):
    print(f"=== {result['label']} ===")
    if 'error' in result:
        print(f"  ERROR: {result['error']}")
        return
    print(f"  PARAM_VALUE events: {result['n_events']}")
    print(f"  Run type: {'ATTACK' if result['is_attack_run'] else 'BASELINE'}")
    print(f"  Total alerts: {len(result['alerts'])}")
    print(f"  False positives: {result['false_positives']}")
    if result['is_attack_run']:
        if result['detected']:
            print(f"  DETECTED -- TTD: {result['ttd']:.2f}s "
                  f"(rule: {result['trigger_rule']})")
        else:
            print(f"  MISSED DETECTION")
    print()


def find_matching_ground_truth(telemetry_filename, gt_dir):
    """Match e.g. attack_pid_drift_seed2_20260925_110843.csv (telemetry) to
    attack_pid_drift_seed2_20260925_110852.csv (ground truth) by shared
    label+seed prefix, tolerating a different trailing timestamp."""
    stem = os.path.basename(telemetry_filename)
    m = re.match(r'(.+_seed\d+)_\d{8}_\d{6}\.csv$', stem)
    if not m:
        return None
    prefix = m.group(1)
    candidates = glob.glob(os.path.join(gt_dir, f'{prefix}_*.csv'))
    return candidates[0] if candidates else None


def run_batch(batch_dir):
    baseline_dir = os.path.join(batch_dir, 'BaselineDatasets')
    attack_dir = os.path.join(batch_dir, 'AttackDatasets')
    gt_dir = os.path.join(batch_dir, 'AttackGroundTruth')

    results = []

    for path in sorted(glob.glob(os.path.join(baseline_dir, '*.csv'))):
        r = evaluate_run(path, None, os.path.basename(path))
        r['category'] = 'baseline'
        results.append(r)

    for path in sorted(glob.glob(os.path.join(attack_dir, '*.csv'))):
        gt_path = find_matching_ground_truth(path, gt_dir)
        stem = os.path.basename(path)
        m = re.match(r'(.+)_seed\d+_\d{8}_\d{6}\.csv$', stem)
        category = m.group(1) if m else 'unknown_attack'
        r = evaluate_run(path, gt_path, stem)
        r['category'] = category
        if gt_path is None:
            r['error'] = r.get('error', '') + ' | no matching ground truth found'
        results.append(r)
        print_single(r)

    # Baseline summary
    baseline_results = [r for r in results if r['category'] == 'baseline']
    print('=' * 70)
    print('SUMMARY')
    print('=' * 70)
    if baseline_results:
        total_fp = sum(r.get('false_positives', 0) for r in baseline_results)
        clean = sum(1 for r in baseline_results if r.get('false_positives', 0) == 0)
        print(f"\nBaseline runs: {len(baseline_results)}")
        print(f"  Clean (zero false positives): {clean}/{len(baseline_results)}")
        print(f"  Total false positive alerts: {total_fp}")

    # Per-attack-category summary
    categories = sorted(set(r['category'] for r in results if r['category'] != 'baseline'))
    for cat in categories:
        cat_results = [r for r in results if r['category'] == cat and 'error' not in r]
        if not cat_results:
            continue
        n = len(cat_results)
        detected = sum(1 for r in cat_results if r.get('detected'))
        ttds = [r['ttd'] for r in cat_results if r.get('ttd') is not None]
        fps = sum(r.get('false_positives', 0) for r in cat_results)

        print(f"\n{cat}:")
        print(f"  Runs evaluated: {n}")
        print(f"  Detection rate: {detected}/{n} ({100*detected/n:.0f}%)")
        print(f"  Pre-attack false positives: {fps}")
        if ttds:
            print(f"  Time-to-detect -- mean: {statistics.mean(ttds):.2f}s, "
                  f"median: {statistics.median(ttds):.2f}s, "
                  f"min: {min(ttds):.2f}s, max: {max(ttds):.2f}s"
                  + (f", sd: {statistics.stdev(ttds):.2f}s" if len(ttds) > 1 else ""))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--telemetry')
    parser.add_argument('--ground-truth', default=None)
    parser.add_argument('--label', default='run')
    parser.add_argument('--batch-dir', default=None,
                         help='Root dataset dir containing BaselineDatasets/, '
                              'AttackDatasets/, AttackGroundTruth/ subfolders')
    args = parser.parse_args()

    if args.batch_dir:
        run_batch(args.batch_dir)
    elif args.telemetry:
        r = evaluate_run(args.telemetry, args.ground_truth, args.label)
        print_single(r)
        if r.get('alerts'):
            print('All alerts:')
            for a in r['alerts']:
                rel = f" (t+{a['t']-r['attack_start']:.1f}s)" if r.get('attack_start') else ''
                print(f"  [{a['rule']}] t={a['t']:.2f}{rel} param={a['param']} "
                      f"value={a['value']:.4f} -- {a['detail']}")
    else:
        print('Provide either --telemetry (single run) or --batch-dir (full sweep)')
        sys.exit(1)


if __name__ == '__main__':
    main()