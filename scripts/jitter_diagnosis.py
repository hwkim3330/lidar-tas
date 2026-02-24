#!/usr/bin/env python3
"""
LiDAR Jitter 원인 진단
- TAS all-open 상태에서 발생하는 jitter 원인 분석
- 스위치 경유 vs 직결 비교 (가능 시)
- 패킷 간격 히스토그램, 주기성, burst 패턴 분석
"""

import socket
import time
import json
import statistics
import subprocess
import os
import numpy as np

LIDAR_HOST = "192.168.6.11"
LIDAR_PORT = 7502
PKT_SIZE = 3328
PKTS_PER_FRAME = 128
EXPECTED_GAP_US = 781  # 100ms / 128 pkts

KETI_TSN_DIR = '/home/kim/keti-tsn-cli-new'


def capture_packets(duration_sec=10, label=""):
    """Capture raw packet timestamps for duration_sec."""
    print(f"\n{'='*60}")
    print(f"  Capturing: {label}")
    print(f"  Duration: {duration_sec}s")
    print(f"{'='*60}")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 16 * 1024 * 1024)
    sock.bind(('0.0.0.0', LIDAR_PORT))
    sock.settimeout(2.0)

    timestamps = []
    start = time.perf_counter()
    deadline = start + duration_sec

    while time.perf_counter() < deadline:
        try:
            data, _ = sock.recvfrom(65535)
            if len(data) == PKT_SIZE:
                timestamps.append(time.perf_counter())
        except socket.timeout:
            continue

    sock.close()
    elapsed = time.perf_counter() - start
    print(f"  Captured {len(timestamps)} packets in {elapsed:.1f}s")
    return timestamps


def analyze_gaps(timestamps, label=""):
    """Detailed inter-packet gap analysis."""
    if len(timestamps) < 10:
        print(f"  Not enough packets for analysis")
        return None

    gaps_us = [(timestamps[i+1] - timestamps[i]) * 1e6
               for i in range(len(timestamps)-1)]
    gaps = np.array(gaps_us)

    # Basic stats
    result = {
        'label': label,
        'num_packets': len(timestamps),
        'duration_sec': timestamps[-1] - timestamps[0],
        'pps': len(timestamps) / (timestamps[-1] - timestamps[0]),
        'gap_mean_us': float(np.mean(gaps)),
        'gap_median_us': float(np.median(gaps)),
        'gap_stdev_us': float(np.std(gaps)),
        'gap_min_us': float(np.min(gaps)),
        'gap_max_us': float(np.max(gaps)),
        'gap_p1_us': float(np.percentile(gaps, 1)),
        'gap_p5_us': float(np.percentile(gaps, 5)),
        'gap_p25_us': float(np.percentile(gaps, 25)),
        'gap_p75_us': float(np.percentile(gaps, 75)),
        'gap_p95_us': float(np.percentile(gaps, 95)),
        'gap_p99_us': float(np.percentile(gaps, 99)),
    }

    # Histogram bins
    bins_fine = [0, 10, 50, 100, 200, 400, 600, 700, 750, 780, 800, 850,
                 900, 1000, 1200, 1500, 2000, 5000, 10000, 50000, 100000]
    hist, _ = np.histogram(gaps, bins=bins_fine)
    result['histogram'] = {f"{bins_fine[i]}-{bins_fine[i+1]}": int(hist[i])
                           for i in range(len(hist)) if hist[i] > 0}

    # Burst analysis (gaps < 50us = back-to-back at line rate)
    burst_gaps = gaps[gaps < 50]
    normal_gaps = gaps[(gaps >= 500) & (gaps <= 1200)]
    large_gaps = gaps[gaps > 1200]

    result['burst_count'] = len(burst_gaps)
    result['burst_pct'] = float(len(burst_gaps) / len(gaps) * 100)
    result['normal_count'] = len(normal_gaps)
    result['normal_pct'] = float(len(normal_gaps) / len(gaps) * 100)
    result['large_gap_count'] = len(large_gaps)
    result['large_gap_pct'] = float(len(large_gaps) / len(gaps) * 100)

    if len(burst_gaps) > 0:
        result['burst_gap_mean_us'] = float(np.mean(burst_gaps))
    if len(normal_gaps) > 0:
        result['normal_gap_mean_us'] = float(np.mean(normal_gaps))
        result['normal_gap_stdev_us'] = float(np.std(normal_gaps))
    if len(large_gaps) > 0:
        result['large_gap_mean_us'] = float(np.mean(large_gaps))
        result['large_gap_max_us'] = float(np.max(large_gaps))

    # Frame boundary detection (gap > 2x expected = likely frame boundary)
    frame_boundaries = np.where(gaps > EXPECTED_GAP_US * 1.5)[0]
    result['frame_boundary_count'] = len(frame_boundaries)

    # Periodicity analysis: check if large gaps repeat at ~128 pkt intervals
    if len(frame_boundaries) > 2:
        fb_intervals = np.diff(frame_boundaries)
        result['frame_interval_mean_pkts'] = float(np.mean(fb_intervals))
        result['frame_interval_stdev_pkts'] = float(np.std(fb_intervals))

    # Consecutive burst detection (runs of <50us gaps)
    burst_runs = []
    run_len = 0
    for g in gaps:
        if g < 50:
            run_len += 1
        else:
            if run_len > 0:
                burst_runs.append(run_len)
            run_len = 0
    if run_len > 0:
        burst_runs.append(run_len)

    if burst_runs:
        result['burst_run_count'] = len(burst_runs)
        result['burst_run_mean'] = float(np.mean(burst_runs))
        result['burst_run_max'] = int(max(burst_runs))

    # Check for 1ms periodicity (TAS cycle signature)
    # Count gaps that fall near 1ms multiples
    for period_us in [1000, 5000, 10000]:
        near_period = np.sum((np.abs(gaps - period_us) < 100) |
                             (np.abs(gaps - period_us*2) < 100))
        if near_period > 5:
            result[f'near_{period_us}us_count'] = int(near_period)

    # Per-frame jitter: split packets into frames and analyze each
    frame_jitters = []
    frame_start = 0
    for fb in frame_boundaries:
        frame_gaps = gaps[frame_start:fb]
        if len(frame_gaps) > 5:
            frame_jitters.append(float(np.std(frame_gaps)))
        frame_start = fb + 1

    if frame_jitters:
        result['per_frame_jitter_mean'] = float(np.mean(frame_jitters))
        result['per_frame_jitter_stdev'] = float(np.std(frame_jitters))
        result['per_frame_jitter_min'] = float(np.min(frame_jitters))
        result['per_frame_jitter_max'] = float(np.max(frame_jitters))

    # Raw gap samples for visualization (every Nth gap)
    step = max(1, len(gaps) // 2000)
    result['gap_samples'] = [float(g) for g in gaps[::step]]

    print_analysis(result)
    return result


def print_analysis(r):
    print(f"\n  --- {r['label']} ---")
    print(f"  Packets: {r['num_packets']:,}  |  PPS: {r['pps']:.0f}  |  Duration: {r['duration_sec']:.1f}s")
    print(f"  Gap: mean={r['gap_mean_us']:.1f}us  median={r['gap_median_us']:.1f}us  stdev={r['gap_stdev_us']:.1f}us")
    print(f"  Gap: min={r['gap_min_us']:.1f}us  max={r['gap_max_us']:.1f}us")
    print(f"  Percentiles: P1={r['gap_p1_us']:.0f}  P5={r['gap_p5_us']:.0f}  P25={r['gap_p25_us']:.0f}  "
          f"P50={r['gap_median_us']:.0f}  P75={r['gap_p75_us']:.0f}  P95={r['gap_p95_us']:.0f}  P99={r['gap_p99_us']:.0f}")
    print(f"  Burst (<50us): {r['burst_count']} ({r['burst_pct']:.1f}%)")
    print(f"  Normal (500-1200us): {r['normal_count']} ({r['normal_pct']:.1f}%)")
    print(f"  Large (>1200us): {r['large_gap_count']} ({r['large_gap_pct']:.1f}%)")
    if 'burst_run_max' in r:
        print(f"  Burst runs: {r['burst_run_count']}x, avg={r['burst_run_mean']:.1f}, max={r['burst_run_max']}")
    if 'per_frame_jitter_mean' in r:
        print(f"  Per-frame jitter: mean={r['per_frame_jitter_mean']:.1f}us  stdev={r['per_frame_jitter_stdev']:.1f}us  "
              f"min={r['per_frame_jitter_min']:.1f}us  max={r['per_frame_jitter_max']:.1f}us")
    print(f"  Histogram:")
    for k, v in r.get('histogram', {}).items():
        pct = v / (r['num_packets']-1) * 100
        bar = '#' * int(pct / 2)
        print(f"    {k:>15}us: {v:>5} ({pct:5.1f}%) {bar}")


def apply_tas_config(cycle_us, open_us, label=""):
    """Apply TAS config via keti-tsn-cli."""
    close_us = cycle_us - open_us
    yaml_path = os.path.join(KETI_TSN_DIR, 'lidar-tas', '_diag_config.yaml')

    if close_us <= 0:
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
          time-interval-value: {cycle_us * 1000}
    config-change: true
'''
    else:
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

    print(f"\n  Applying TAS: {label} (cycle={cycle_us}us, open={open_us}us)...")
    result = subprocess.run(
        ['./keti-tsn', 'patch', yaml_path],
        cwd=KETI_TSN_DIR, capture_output=True, text=True, timeout=30
    )
    ok = 'Failed' not in result.stdout and result.returncode == 0
    print(f"  {'OK' if ok else 'FAILED'}")
    time.sleep(2)  # settle
    return ok


def main():
    # Kill the web server temporarily to free the UDP port
    print("Stopping web server to free UDP port 7502...")
    os.system("fuser -k 7502/udp 2>/dev/null")
    os.system("fuser -k 8080/tcp 2>/dev/null")
    time.sleep(2)

    all_results = []

    # ── Test 1: TAS all-open 1ms cycle (current default) ──
    apply_tas_config(1000, 1000, "All-open 1ms cycle")
    ts = capture_packets(10, "TAS All-Open (1ms cycle)")
    r = analyze_gaps(ts, "TAS All-Open (1ms cycle)")
    if r: all_results.append(r)

    # ── Test 2: TAS all-open 10ms cycle ──
    apply_tas_config(10000, 10000, "All-open 10ms cycle")
    ts = capture_packets(10, "TAS All-Open (10ms cycle)")
    r = analyze_gaps(ts, "TAS All-Open (10ms cycle)")
    if r: all_results.append(r)

    # ── Test 3: TAS all-open 100ms cycle (longer than 1 LiDAR frame) ──
    apply_tas_config(100000, 100000, "All-open 100ms cycle")
    ts = capture_packets(10, "TAS All-Open (100ms cycle)")
    r = analyze_gaps(ts, "TAS All-Open (100ms cycle)")
    if r: all_results.append(r)

    # ── Test 4: TAS 1ms / 80% open (light gating) ──
    apply_tas_config(1000, 800, "1ms/80%")
    ts = capture_packets(10, "TAS 1ms/80% open")
    r = analyze_gaps(ts, "TAS 1ms/80% open")
    if r: all_results.append(r)

    # ── Test 5: TAS 1ms / 50% open ──
    apply_tas_config(1000, 500, "1ms/50%")
    ts = capture_packets(10, "TAS 1ms/50% open")
    r = analyze_gaps(ts, "TAS 1ms/50% open")
    if r: all_results.append(r)

    # ── Test 6: TAS 5ms / 80% open ──
    apply_tas_config(5000, 4000, "5ms/80%")
    ts = capture_packets(10, "TAS 5ms/80% open")
    r = analyze_gaps(ts, "TAS 5ms/80% open")
    if r: all_results.append(r)

    # ── Test 7: TAS 10ms / 50% open ──
    apply_tas_config(10000, 5000, "10ms/50%")
    ts = capture_packets(10, "TAS 10ms/50% open")
    r = analyze_gaps(ts, "TAS 10ms/50% open")
    if r: all_results.append(r)

    # ── Test 8: TAS 50ms / 20% open (worst case) ──
    apply_tas_config(50000, 10000, "50ms/20%")
    ts = capture_packets(10, "TAS 50ms/20% open")
    r = analyze_gaps(ts, "TAS 50ms/20% open")
    if r: all_results.append(r)

    # Restore all-open
    apply_tas_config(1000, 1000, "Restore all-open")

    # Save results
    output_path = '/home/kim/lidar-tas/data/jitter_diagnosis.json'
    # Remove gap_samples for summary, keep in separate file
    summary = []
    for r in all_results:
        s = {k: v for k, v in r.items() if k != 'gap_samples'}
        summary.append(s)

    with open(output_path, 'w') as f:
        json.dump(all_results, f, indent=2)
    print(f"\nResults saved to {output_path}")

    # Print summary comparison
    print(f"\n{'='*80}")
    print(f"  SUMMARY: Jitter Diagnosis")
    print(f"{'='*80}")
    print(f"{'Config':<30} {'Pkts':>6} {'Mean':>8} {'StdDev':>8} {'P99':>8} {'Max':>8} {'Burst%':>7} {'BurstMax':>8}")
    print(f"{'-'*30} {'-'*6} {'-'*8} {'-'*8} {'-'*8} {'-'*8} {'-'*7} {'-'*8}")
    for r in all_results:
        bm = r.get('burst_run_max', 0)
        print(f"{r['label']:<30} {r['num_packets']:>6} {r['gap_mean_us']:>7.0f}u {r['gap_stdev_us']:>7.0f}u "
              f"{r['gap_p99_us']:>7.0f}u {r['gap_max_us']:>7.0f}u {r['burst_pct']:>6.1f}% {bm:>8}")

    print(f"\n  Key Question: Does jitter change with TAS cycle time when all gates are open?")
    open_results = [r for r in all_results if 'All-Open' in r['label']]
    if len(open_results) >= 2:
        stdevs = [r['gap_stdev_us'] for r in open_results]
        if max(stdevs) - min(stdevs) < 50:
            print(f"  → Jitter is SIMILAR across cycle times → TAS cycle NOT the cause")
            print(f"  → Likely: USB Ethernet adapter or store-and-forward switch latency")
        else:
            print(f"  → Jitter VARIES with cycle time → TAS gate transitions are a factor")
            for r in open_results:
                print(f"    {r['label']}: stdev={r['gap_stdev_us']:.0f}us")


if __name__ == '__main__':
    main()
