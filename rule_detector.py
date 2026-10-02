#!/usr/bin/env python3
"""
Physics/Policy-Based Rule Detector -- Phase 3 semantic firewall prototype.
"""

from collections import deque, defaultdict

LEGAL_RANGES = {
    'ATC_RAT_RLL_P':   (0.0, 0.35),
    'ATC_RAT_PIT_P':   (0.0, 0.35),
    'EK3_POSNE_M_NSE': (0.05, 10.0),
    'EK3_VELNE_M_NSE': (0.05, 10.0),
    'WPNAV_SPEED':     (20.0, 2000.0),   # cm/s
    'WPNAV_ACCEL':     (10.0, 981.0),    # cm/s^2
    'WP_SPD':          (0.5, 30.0),      # m/s
    'WP_ACC':          (0.2, 15.0),      # m/s^2
    'ATC_ANG_RLL_P':   (3.0, 12.0),
    'ATC_ANG_PIT_P':   (3.0, 12.0),
}

SAFETY_CRITICAL_PARAMS = set(LEGAL_RANGES.keys())

# Rule 2 tuning
WINDOW_SECONDS = 30.0
MAX_CHANGES_PER_WINDOW = 3

# Rule 3 tuning -- cumulative relative drift from first-seen value
DRIFT_THRESHOLD = 0.15  # 15%
EPSILON = 1e-6          # float rounding guard for exact 32-bit float boundaries


class RuleBasedDetector:
    def __init__(self):
        self.change_history = defaultdict(deque)   # param -> deque of change timestamps
        self.baseline_value = {}                   # param -> first value seen
        self.last_value = {}                       # param -> most recent value seen
        self.alerts = []

    def process_param_event(self, t, param, value):
        """Feed one PARAM_VALUE observation. Returns list of new alerts
        triggered by this event (may be empty)."""
        if param not in SAFETY_CRITICAL_PARAMS:
            return []

        new_alerts = []

        # Rule 1: static bounds
        lo, hi = LEGAL_RANGES[param]
        if not (lo - EPSILON <= value <= hi + EPSILON):
            new_alerts.append(self._alert(t, param, value, 'STATIC_BOUNDS',
                f'{value:.4f} outside legal range [{lo}, {hi}]'))

        # First observation of this parameter establishes the session baseline
        if param not in self.baseline_value:
            self.baseline_value[param] = value
            self.last_value[param] = value
            self.alerts.extend(new_alerts)
            return new_alerts

        # Only count Rule 2 (CHANGE_FREQUENCY) when the parameter value is
        # actually modified (ignores duplicate PARAM_VALUE broadcasts on read)
        val_changed = abs(value - self.last_value[param]) > EPSILON
        self.last_value[param] = value

        if val_changed:
            hist = self.change_history[param]
            hist.append(t)
            while hist and t - hist[0] > WINDOW_SECONDS:
                hist.popleft()
            if len(hist) > MAX_CHANGES_PER_WINDOW:
                new_alerts.append(self._alert(t, param, value, 'CHANGE_FREQUENCY',
                    f'{len(hist)} value modifications within {WINDOW_SECONDS:.0f}s '
                    f'(limit {MAX_CHANGES_PER_WINDOW})'))

        # Rule 3: cumulative drift from first-seen value this session
        base = self.baseline_value[param]
        if base != 0 and val_changed:
            rel_drift = abs(value - base) / abs(base)
            if rel_drift > DRIFT_THRESHOLD + EPSILON:
                new_alerts.append(self._alert(t, param, value, 'CUMULATIVE_DRIFT',
                    f'{rel_drift*100:.1f}% cumulative drift from baseline '
                    f'{base:.4f} (limit {DRIFT_THRESHOLD*100:.0f}%)'))

        self.alerts.extend(new_alerts)
        return new_alerts

    def _alert(self, t, param, value, rule, detail):
        return {'t': t, 'param': param, 'value': value, 'rule': rule, 'detail': detail}

    def reset(self):
        """Call between flights/sessions so drift baselines don't leak
        across runs when replaying multiple CSVs in one process."""
        self.change_history.clear()
        self.baseline_value.clear()
        self.last_value.clear()
        self.alerts = []