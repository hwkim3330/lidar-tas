#!/usr/bin/env python3
"""TAS sweep test: measure LiDAR packet reception under various TAS gate schedules.

Uses keti-tsn-cli to configure the LAN9662 switch, then measures packets.
"""
import socket
import time
import subprocess
import sys
import os
import statistics
import json

LIDAR_PORT = 7502
KETI_TSN_DIR = '/home/kim/keti-tsn-cli-new'
MEASURE_DURATION = 5  # seconds per config
SETTLE_TIME = 2  # seconds after config change

# TAS configs to test: (open_us, close_us) within a cycle
# All cycles are open+close, TAS on Port 1 egress
TAS_CONFIGS = [
    # (label, open_us, close_us)
    ("No TAS (baseline)", 0, 0),  # TAS disabled
    ("900/100 (90%)", 900, 100),
    ("800/200 (80%)", 800, 200),
    ("600/400 (60%)", 600, 400),
    ("500/500 (50%)", 500, 500),
    ("400/600 (40%)", 400, 600),
    ("200/800 (20%)", 200, 800),
]

def write_yaml(filepath, content):
    with open(filepath, 'w') as f:
        f.write(content)

def run_keti_tsn(yaml_file):
    """Run keti-tsn patch command."""
    result = subprocess.run(
        ['./keti-tsn', 'patch', yaml_file],
        cwd=KETI_TSN_DIR,
        capture_output=True, text=True, timeout=30
    )
    if 'Failed' in result.stdout:
        print(f"  WARNING: Some patches failed")
        print(result.stdout[-200:])
    return result.returncode == 0

def set_tas(open_us, close_us):
    """Configure TAS on Port 1."""
    yaml_path = os.path.join(KETI_TSN_DIR, 'lidar-tas', '_sweep_config.yaml')

    if open_us == 0 and close_us == 0:
        # Disable TAS
        content = '''- ? "/ietf-interfaces:interfaces/interface[name='1']/ieee802-dot1q-bridge:bridge-port/ieee802-dot1q-sched-bridge:gate-parameter-table"
  : gate-enabled: false
    admin-control-list:
      gate-control-entry: []
    config-change: true
'''
    else:
        cycle_ns = (open_us + close_us) * 1000
        open_ns = open_us * 1000
        close_ns = close_us * 1000
        content = f'''- ? "/ietf-interfaces:interfaces/interface[name='1']/ieee802-dot1q-bridge:bridge-port/ieee802-dot1q-sched-bridge:gate-parameter-table"
  : gate-enabled: true
    admin-gate-states: 255
    admin-cycle-time:
      numerator: {cycle_ns}
      denominator: 1000000000
    admin-base-time:
      seconds: 0
      nanoseconds: 0
    admin-control-list:
      gate-control-entry:
        - index: 0
          operation-name: set-gate-states
          gate-states-value: 255
          time-interval-value: {open_ns}
        - index: 1
          operation-name: set-gate-states
          gate-states-value: 254
          time-interval-value: {close_ns}
    config-change: true
'''

    write_yaml(yaml_path, content)
    return run_keti_tsn(yaml_path)

def measure_packets(duration):
    """Count received LiDAR packets and measure timing."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
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
                'burst_pct': 0}

    gaps = [(timestamps[i+1] - timestamps[i]) * 1e6 for i in range(len(timestamps)-1)]
    elapsed = timestamps[-1] - timestamps[0]
    pps = len(timestamps) / elapsed if elapsed > 0 else 0
    expected_pps = 1280  # 128 pkts * 10 fps
    completeness = min(100, (pps / expected_pps) * 100)
    burst_pct = sum(1 for g in gaps if g < 50) / len(gaps) * 100 if gaps else 0

    return {
        'packets': len(timestamps),
        'pps': pps,
        'completeness': completeness,
        'gap_mean': statistics.mean(gaps),
        'gap_stdev': statistics.stdev(gaps) if len(gaps) > 1 else 0,
        'gap_min': min(gaps),
        'gap_max': max(gaps),
        'burst_pct': burst_pct,
    }

def main():
    print("=" * 70)
    print("LiDAR TAS Sweep Test — LAN9662 802.1Qbv")
    print("=" * 70)
    print(f"Measure duration: {MEASURE_DURATION}s per config")
    print(f"Settle time: {SETTLE_TIME}s after config change")
    print()

    results = []

    for i, (label, open_us, close_us) in enumerate(TAS_CONFIGS):
        print(f"[{i+1}/{len(TAS_CONFIGS)}] {label}")

        # Apply config
        print(f"  Configuring TAS...", end=' ', flush=True)
        ok = set_tas(open_us, close_us)
        if not ok:
            print("FAILED — skipping")
            continue
        print("OK")

        # Settle
        print(f"  Settling {SETTLE_TIME}s...", end=' ', flush=True)
        time.sleep(SETTLE_TIME)
        print("OK")

        # Measure
        print(f"  Measuring {MEASURE_DURATION}s...", end=' ', flush=True)
        result = measure_packets(MEASURE_DURATION)
        result['label'] = label
        result['open_us'] = open_us
        result['close_us'] = close_us
        results.append(result)

        ratio = open_us / (open_us + close_us) * 100 if (open_us + close_us) > 0 else 100
        print(f"Done")
        print(f"  → Completeness: {result['completeness']:.1f}%  "
              f"PPS: {result['pps']:.0f}  "
              f"Gap: {result['gap_mean']:.0f}±{result['gap_stdev']:.0f}µs  "
              f"[{result['gap_min']:.0f}-{result['gap_max']:.0f}]  "
              f"Burst: {result['burst_pct']:.1f}%")
        print()

    # Summary table
    print("\n" + "=" * 70)
    print(f"{'Config':<22s} {'Open%':>5s} {'Complete%':>9s} {'PPS':>6s} "
          f"{'GapMean':>8s} {'GapSD':>7s} {'GapMax':>7s} {'Burst%':>7s}")
    print("-" * 70)
    for r in results:
        ratio = r['open_us'] / (r['open_us'] + r['close_us']) * 100 if (r['open_us'] + r['close_us']) > 0 else 100
        print(f"{r['label']:<22s} {ratio:5.0f} {r['completeness']:9.1f} {r['pps']:6.0f} "
              f"{r['gap_mean']:8.0f} {r['gap_stdev']:7.0f} {r['gap_max']:7.0f} {r['burst_pct']:7.1f}")

    # Save results
    out_path = '/home/kim/lidar/tas_sweep_results.json'
    with open(out_path, 'w') as f:
        json.dump(results, f, indent=2)
    print(f"\nResults saved to {out_path}")

    # Restore baseline (TAS off)
    print("\nRestoring baseline (TAS off)...", end=' ', flush=True)
    set_tas(0, 0)
    print("Done")

if __name__ == '__main__':
    main()
