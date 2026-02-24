#!/usr/bin/env python3
"""Extended TAS sweep: varying both cycle time and open ratio.

Tests longer cycles where buffer overflow may occur.
"""
import socket
import time
import subprocess
import os
import statistics
import json

LIDAR_PORT = 7502
KETI_TSN_DIR = '/home/kim/keti-tsn-cli-new'
MEASURE_DURATION = 5
SETTLE_TIME = 2

# Extended configs: (label, cycle_us, open_us)
CONFIGS = [
    # Baseline
    ("Baseline (no TAS)", 0, 0),

    # 1ms cycle (1kHz) - different open ratios
    ("1ms / 80% open", 1000, 800),
    ("1ms / 50% open", 1000, 500),
    ("1ms / 20% open", 1000, 200),

    # 5ms cycle (200Hz)
    ("5ms / 80% open", 5000, 4000),
    ("5ms / 50% open", 5000, 2500),
    ("5ms / 20% open", 5000, 1000),

    # 10ms cycle (100Hz) - ~13 pkts per cycle
    ("10ms / 80% open", 10000, 8000),
    ("10ms / 50% open", 10000, 5000),
    ("10ms / 20% open", 10000, 2000),

    # 50ms cycle (20Hz) - ~64 pkts per cycle
    ("50ms / 80% open", 50000, 40000),
    ("50ms / 50% open", 50000, 25000),
    ("50ms / 20% open", 50000, 10000),
]

def set_tas(cycle_us, open_us):
    yaml_path = os.path.join(KETI_TSN_DIR, 'lidar-tas', '_sweep_config.yaml')
    if cycle_us == 0:
        content = '''- ? "/ietf-interfaces:interfaces/interface[name='1']/ieee802-dot1q-bridge:bridge-port/ieee802-dot1q-sched-bridge:gate-parameter-table"
  : gate-enabled: false
    admin-control-list:
      gate-control-entry: []
    config-change: true
'''
    else:
        close_us = cycle_us - open_us
        content = f'''- ? "/ietf-interfaces:interfaces/interface[name='1']/ieee802-dot1q-bridge:bridge-port/ieee802-dot1q-sched-bridge:gate-parameter-table"
  : gate-enabled: true
    admin-gate-states: 255
    admin-cycle-time:
      numerator: {cycle_us * 1000}
      denominator: 1000000000
    admin-base-time:
      seconds: 0
      nanoseconds: 0
    admin-control-list:
      gate-control-entry:
        - index: 0
          operation-name: set-gate-states
          gate-states-value: 255
          time-interval-value: {open_us * 1000}
        - index: 1
          operation-name: set-gate-states
          gate-states-value: 254
          time-interval-value: {close_us * 1000}
    config-change: true
'''
    with open(yaml_path, 'w') as f:
        f.write(content)
    result = subprocess.run(
        ['./keti-tsn', 'patch', yaml_path],
        cwd=KETI_TSN_DIR, capture_output=True, text=True, timeout=30
    )
    return 'Failed' not in result.stdout

def measure(duration):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)
    sock.bind(('0.0.0.0', LIDAR_PORT))
    sock.settimeout(0.5)

    timestamps = []
    t_start = time.monotonic()

    while True:
        now = time.monotonic()
        if now - t_start >= duration:
            break
        try:
            data, addr = sock.recvfrom(65535)
            timestamps.append(time.perf_counter())
        except socket.timeout:
            continue

    sock.close()

    if len(timestamps) < 10:
        return {'packets': len(timestamps), 'pps': 0, 'completeness': 0,
                'gap_mean': 0, 'gap_stdev': 0, 'gap_min': 0, 'gap_max': 0,
                'burst_pct': 0, 'p99_gap': 0}

    gaps = [(timestamps[i+1] - timestamps[i]) * 1e6 for i in range(len(timestamps)-1)]
    elapsed = timestamps[-1] - timestamps[0]
    pps = len(timestamps) / elapsed if elapsed > 0 else 0
    completeness = min(100, (pps / 1280) * 100)
    burst_pct = sum(1 for g in gaps if g < 50) / len(gaps) * 100 if gaps else 0
    sorted_gaps = sorted(gaps)
    p99_gap = sorted_gaps[int(len(sorted_gaps) * 0.99)] if sorted_gaps else 0

    return {
        'packets': len(timestamps),
        'pps': round(pps, 1),
        'completeness': round(completeness, 1),
        'gap_mean': round(statistics.mean(gaps), 1),
        'gap_stdev': round(statistics.stdev(gaps), 1) if len(gaps) > 1 else 0,
        'gap_min': round(min(gaps), 1),
        'gap_max': round(max(gaps), 1),
        'p99_gap': round(p99_gap, 1),
        'burst_pct': round(burst_pct, 1),
    }

def main():
    print("=" * 80)
    print("Extended TAS Sweep — LAN9662 802.1Qbv + Ouster LiDAR")
    print("=" * 80)
    print(f"LiDAR: OS-1-16, 10Hz, 128 pkts/frame, ~781µs/pkt, 3328B/pkt")
    print(f"Measure: {MEASURE_DURATION}s per config, {SETTLE_TIME}s settle")
    print()

    results = []
    for i, (label, cycle_us, open_us) in enumerate(CONFIGS):
        print(f"[{i+1}/{len(CONFIGS)}] {label}")
        print(f"  Config...", end=' ', flush=True)
        ok = set_tas(cycle_us, open_us)
        print("OK" if ok else "FAIL")
        if not ok:
            continue

        time.sleep(SETTLE_TIME)
        print(f"  Measure...", end=' ', flush=True)
        r = measure(MEASURE_DURATION)
        r['label'] = label
        r['cycle_us'] = cycle_us
        r['open_us'] = open_us
        r['close_us'] = cycle_us - open_us if cycle_us > 0 else 0
        r['open_pct'] = round(open_us / cycle_us * 100, 1) if cycle_us > 0 else 100
        results.append(r)
        print(f"Complete={r['completeness']}%  PPS={r['pps']}  "
              f"Jitter={r['gap_stdev']}µs  MaxGap={r['gap_max']}µs  Burst={r['burst_pct']}%")
        print()

    # Summary
    print("\n" + "=" * 80)
    print(f"{'Config':<22s} {'Open%':>5s} {'Cmpl%':>6s} {'PPS':>6s} "
          f"{'Mean':>7s} {'StdDev':>7s} {'P99':>7s} {'Max':>7s} {'Burst':>6s}")
    print("-" * 80)
    for r in results:
        print(f"{r['label']:<22s} {r['open_pct']:5.0f} {r['completeness']:6.1f} {r['pps']:6.0f} "
              f"{r['gap_mean']:7.0f} {r['gap_stdev']:7.0f} {r['p99_gap']:7.0f} {r['gap_max']:7.0f} "
              f"{r['burst_pct']:6.1f}")

    # Save
    out = '/home/kim/lidar/tas_sweep_extended_results.json'
    with open(out, 'w') as f:
        json.dump(results, f, indent=2)
    print(f"\nSaved to {out}")

    # Restore
    print("Restoring baseline...", end=' ', flush=True)
    set_tas(0, 0)
    print("Done")

if __name__ == '__main__':
    main()
