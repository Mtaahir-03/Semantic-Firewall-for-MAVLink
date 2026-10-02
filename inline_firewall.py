#!/usr/bin/env python3
"""
Inline MAVLink Semantic Firewall Proxy (Week 6)

Sits between the Flight Controller (14551) and the Companion Computer (14552).
Inspects all PARAM_SET commands in real-time. Logs decision latency and outcome
to CSV for offline analysis.
"""

import time
import sys
import csv
import os
import signal
from collections import deque
from pymavlink import mavutil
from rule_detectorV1 import RuleBasedDetector

FC_PORT = 'udp:127.0.0.1:14551'
CC_PORT = 'udpout:127.0.0.1:14552'
LOG_DIR = '/mnt/hgfs/Dataset/FirewallLogs'

running = True


def handle_sigint(signum, frame):
    """Graceful shutdown to flush and close the CSV on Ctrl+C."""
    global running
    running = False


signal.signal(signal.SIGINT, handle_sigint)


class InlineDetector(RuleBasedDetector):
    """Wraps RuleBasedDetector to inspect packets without corrupting state if dropped."""
    def inspect_and_filter(self, t, param, value):
        old_hist = list(self.change_history.get(param, []))
        old_base = self.baseline_value.get(param, None)
        old_last = self.last_value.get(param, None)
        old_alerts = list(self.alerts)
        
        new_alerts = self.process_param_event(t, param, value)
        
        if new_alerts:
            self.change_history[param] = deque(old_hist)
            if old_base is None:
                self.baseline_value.pop(param, None)
                self.last_value.pop(param, None)
            else:
                self.baseline_value[param] = old_base
                self.last_value[param] = old_last
            self.alerts = old_alerts
            
        return new_alerts


def main():
    os.makedirs(LOG_DIR, exist_ok=True)
    log_path = f'{LOG_DIR}/firewall_log_{time.strftime("%Y%m%d_%H%M%S")}.csv'
    log_file = open(log_path, 'w', newline='')
    log_writer = csv.writer(log_file)
    log_writer.writerow(['t_wall', 'param', 'value', 'verdict', 'latency_us', 'rule'])

    print("========================================")
    print("🛡️️  INLINE SEMANTIC FIREWALL ACTIVE  🛡️")
    print("========================================")
    print(f"Facing Flight Controller : {FC_PORT}")
    print(f"Facing Companion Computer: {CC_PORT} (Set attack scripts to this port!)")
    print(f"Logging firewall decisions to: {log_path}")
    
    fc_conn = mavutil.mavlink_connection(FC_PORT)
    cc_conn = mavutil.mavlink_connection(CC_PORT)
    detector = InlineDetector()
    
    print("\nWaiting for heartbeat from Flight Controller...")
    fc_conn.wait_heartbeat()
    print("Heartbeat received. Proxy routing established.\n")
    
    try:
        while running:
            # 1. Route FC -> CC (Telemetry, ACKs)
            fc_msg = fc_conn.recv_match(blocking=False)
            if fc_msg:
                cc_conn.write(fc_msg.get_msgbuf())
                
                # Keep firewall state synced with legitimate FC parameter broadcasts
                if fc_msg.get_type() == 'PARAM_VALUE':
                    msg_dict = fc_msg.to_dict()
                    param_name = str(msg_dict.get('param_id', '')).strip('\x00')
                    param_val = msg_dict.get('param_value', 0.0)
                    detector.inspect_and_filter(time.time(), param_name, param_val)

            # 2. Inspect & Route CC -> FC (Commands, PARAM_SET)
            cc_msg = cc_conn.recv_match(blocking=False)
            if cc_msg:
                msg_type = cc_msg.get_type()
                
                if msg_type == 'PARAM_SET':
                    start_time = time.perf_counter()
                    msg_dict = cc_msg.to_dict()
                    param_name = str(msg_dict.get('param_id', '')).strip('\x00')
                    param_val = msg_dict.get('param_value', 0.0)
                    
                    alerts = detector.inspect_and_filter(time.time(), param_name, param_val)
                    
                    if alerts:
                        drop_packet = True
                    else:
                        drop_packet = False
                        fc_conn.write(cc_msg.get_msgbuf())
                    
                    # Stop timer AFTER the write/drop execution completes
                    latency_us = (time.perf_counter() - start_time) * 1e6
                    
                    # Log the decision
                    verdict = 'BLOCKED' if drop_packet else 'PASSED'
                    triggering_rule = alerts[0]['rule'] if alerts else ''
                    log_writer.writerow([time.time(), param_name, param_val, verdict, latency_us, triggering_rule])
                    log_file.flush()
                    
                    if drop_packet:
                        print(f"[BLOCKED] {param_name} = {param_val:.4f} (Inspect Latency: {latency_us:.1f} µs)")
                        for a in alerts:
                            print(f"  └─> {a['rule']}: {a['detail']}")
                    else:
                        print(f"[PASSED]  {param_name} = {param_val:.4f} (Inspect Latency: {latency_us:.1f} µs)")
                
                else:
                    # Pass all non-PARAM_SET messages through instantly
                    fc_conn.write(cc_msg.get_msgbuf())
                    
            # Prevent 100% CPU lockup (adds up to 1ms scheduling jitter)
            time.sleep(0.001)

    except KeyboardInterrupt:
        pass
    finally:
        log_file.close()
        print(f"\nFirewall stopped. Log saved to {log_path}")

if __name__ == '__main__':
    main()