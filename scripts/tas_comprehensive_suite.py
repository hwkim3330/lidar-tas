#!/usr/bin/env python3
"""
LiDAR TAS 종합 테스트 스위트
6개 카테고리, 총 43개 설정 자동 테스트

Categories:
1. Sub-Millisecond Gating (10 tests, 10s each)
2. Buffer Capacity Mapping (13 tests, 10s each)
3. Frame-Aligned Gating (5 tests, 10s each)
4. Multi-GCL Entries (4 tests, 10s each)
5. Endurance (5 tests, 60s each)
6. Boundary Conditions (6 tests, 10s each)
"""

import socket
import time
import json
import subprocess
import os
import sys
import numpy as np

LIDAR_PORT = 7502
PKT_SIZE = 3328
PKTS_PER_FRAME = 128
EXPECTED_GAP_US = 781  # 100ms / 128 pkts

KETI_TSN_DIR = '/home/kim/keti-tsn-cli-new'
OUTPUT_PATH = '/home/kim/lidar-tas/data/comprehensive_results.json'


def capture_packets(duration_sec=10, label=""):
    """Capture raw packet timestamps for duration_sec."""
    print(f"\n{'='*60}")
    print(f"  Capturing: {label}")
    print(f"  Duration: {duration_sec}s")
    print(f"{'='*60}")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 32 * 1024 * 1024)
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


def analyze_gaps(timestamps, label="", duration_sec=10):
    """Comprehensive inter-packet gap analysis."""
    if len(timestamps) < 10:
        print(f"  Not enough packets for analysis ({len(timestamps)})")
        return {
            'label': label,
            'num_packets': len(timestamps),
            'completeness': len(timestamps) / (PKTS_PER_FRAME * 10 * (duration_sec / 10)) * 100 if duration_sec else 0,
            'pps': 0,
            'gap_mean_us': 0, 'gap_stdev_us': 0, 'gap_max_us': 0,
            'gap_p50_us': 0, 'gap_p99_us': 0,
            'burst_pct': 0, 'normal_pct': 0, 'large_pct': 0,
            'burst_run_max': 0,
            'loss_pattern': 'total_loss' if len(timestamps) == 0 else 'severe_loss',
        }

    gaps = np.array([(timestamps[i+1] - timestamps[i]) * 1e6
                     for i in range(len(timestamps)-1)])

    # Expected packets for this duration
    expected_pkts = PKTS_PER_FRAME * 10 * (duration_sec / 10)  # 1280/s * duration
    completeness = min(100.0, len(timestamps) / expected_pkts * 100)

    result = {
        'label': label,
        'num_packets': len(timestamps),
        'duration_sec': float(timestamps[-1] - timestamps[0]),
        'pps': len(timestamps) / (timestamps[-1] - timestamps[0]),
        'completeness': round(completeness, 1),
        'gap_mean_us': float(np.mean(gaps)),
        'gap_median_us': float(np.median(gaps)),
        'gap_stdev_us': float(np.std(gaps)),
        'gap_min_us': float(np.min(gaps)),
        'gap_max_us': float(np.max(gaps)),
        'gap_p50_us': float(np.percentile(gaps, 50)),
        'gap_p99_us': float(np.percentile(gaps, 99)),
    }

    # Burst / Normal / Large classification
    burst_gaps = gaps[gaps < 50]
    normal_gaps = gaps[(gaps >= 500) & (gaps <= 1200)]
    large_gaps = gaps[gaps > 1200]

    result['burst_pct'] = round(float(len(burst_gaps) / len(gaps) * 100), 1)
    result['normal_pct'] = round(float(len(normal_gaps) / len(gaps) * 100), 1)
    result['large_pct'] = round(float(len(large_gaps) / len(gaps) * 100), 1)

    # Consecutive burst detection
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

    result['burst_run_max'] = int(max(burst_runs)) if burst_runs else 0

    # Frame boundary detection
    frame_boundaries = np.where(gaps > EXPECTED_GAP_US * 1.5)[0]
    result['frame_boundary_count'] = len(frame_boundaries)

    # Per-frame completeness
    frame_sizes = []
    frame_start = 0
    for fb in frame_boundaries:
        frame_sizes.append(fb - frame_start)
        frame_start = fb + 1
    if frame_start < len(gaps):
        frame_sizes.append(len(gaps) - frame_start + 1)

    if frame_sizes:
        frame_completes = [min(100.0, fs / PKTS_PER_FRAME * 100) for fs in frame_sizes]
        result['per_frame_completeness_mean'] = round(float(np.mean(frame_completes)), 1)
        result['per_frame_completeness_min'] = round(float(np.min(frame_completes)), 1)

    # Loss pattern detection
    if completeness >= 99.5:
        result['loss_pattern'] = 'none'
    elif completeness >= 80:
        # Check if loss is periodic
        if len(large_gaps) > 5:
            large_positions = np.where(gaps > 5000)[0]
            if len(large_positions) > 2:
                intervals = np.diff(large_positions)
                cv = np.std(intervals) / np.mean(intervals) if np.mean(intervals) > 0 else 999
                result['loss_pattern'] = 'periodic' if cv < 0.3 else 'distributed'
            else:
                result['loss_pattern'] = 'burst'
        else:
            result['loss_pattern'] = 'minor'
    else:
        result['loss_pattern'] = 'burst' if result['burst_pct'] > 30 else 'distributed'

    # Gap samples for visualization (downsample to 2000)
    step = max(1, len(gaps) // 2000)
    result['gap_samples'] = [float(g) for g in gaps[::step]]

    print_analysis(result)
    return result


def print_analysis(r):
    print(f"  Packets: {r['num_packets']:,}  PPS: {r.get('pps',0):.0f}  "
          f"Completeness: {r['completeness']:.1f}%")
    if r.get('gap_mean_us'):
        print(f"  Gap: mean={r['gap_mean_us']:.1f}µs  stdev={r['gap_stdev_us']:.1f}µs  "
              f"P99={r.get('gap_p99_us',0):.0f}µs  max={r['gap_max_us']:.0f}µs")
        print(f"  Burst: {r['burst_pct']:.1f}%  Normal: {r['normal_pct']:.1f}%  "
              f"Large: {r['large_pct']:.1f}%  BurstMax: {r['burst_run_max']}")
    print(f"  Loss pattern: {r['loss_pattern']}")


def apply_tas_2entry(cycle_us, open_us, label=""):
    """Apply standard 2-entry TAS config (open + close)."""
    close_us = cycle_us - open_us
    entries = []
    if close_us <= 0:
        entries.append({'gate': 255, 'time_ns': cycle_us * 1000})
    else:
        entries.append({'gate': 255, 'time_ns': open_us * 1000})
        entries.append({'gate': 254, 'time_ns': close_us * 1000})
    return apply_tas_multi(cycle_us, entries, label)


def apply_tas_multi(cycle_us, entries, label=""):
    """Apply multi-entry TAS config.
    entries: list of {'gate': int, 'time_ns': int}
    """
    yaml_path = os.path.join(KETI_TSN_DIR, 'lidar-tas', '_comp_config.yaml')
    os.makedirs(os.path.dirname(yaml_path), exist_ok=True)

    cycle_ns = cycle_us * 1000

    gcl_entries = ""
    for i, entry in enumerate(entries):
        gcl_entries += f"""        - index: {i}
          operation-name: set-gate-states
          gate-states-value: {entry['gate']}
          time-interval-value: {entry['time_ns']}
"""

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
{gcl_entries}    config-change: true
'''

    with open(yaml_path, 'w') as f:
        f.write(content)

    entries_desc = " → ".join(
        f"{'OPEN' if e['gate']==255 else 'CLOSE'}({e['time_ns']/1000:.0f}µs)"
        for e in entries
    )
    print(f"\n  Applying TAS: {label}")
    print(f"  Cycle: {cycle_us}µs  Entries: {entries_desc}")

    result = subprocess.run(
        ['./keti-tsn', 'patch', yaml_path],
        cwd=KETI_TSN_DIR, capture_output=True, text=True, timeout=30
    )
    ok = 'Failed' not in result.stdout and result.returncode == 0
    print(f"  {'OK' if ok else 'FAILED'}")
    if not ok:
        print(f"  stdout: {result.stdout[:200]}")
        print(f"  stderr: {result.stderr[:200]}")
    time.sleep(2)  # settle
    return ok


def run_test(label, cycle_us, open_us=None, entries=None, duration=10, category=""):
    """Run a single test config and return result."""
    info = {
        'label': label,
        'category': category,
        'cycle_us': cycle_us,
        'duration': duration,
    }

    if entries:
        # Multi-GCL
        info['entries'] = [{'gate': e['gate'], 'time_ns': e['time_ns']} for e in entries]
        open_total = sum(e['time_ns'] for e in entries if e['gate'] == 255)
        info['open_pct'] = round(open_total / (cycle_us * 1000) * 100, 1)
        ok = apply_tas_multi(cycle_us, entries, label)
    else:
        # Standard 2-entry
        info['open_us'] = open_us
        info['close_us'] = cycle_us - open_us
        info['open_pct'] = round(open_us / cycle_us * 100, 1)
        ok = apply_tas_2entry(cycle_us, open_us, label)

    if not ok:
        print(f"  ⚠ Config failed, skipping capture")
        info['status'] = 'config_failed'
        return info

    ts = capture_packets(duration, label)
    result = analyze_gaps(ts, label, duration)
    result.update(info)
    result['status'] = 'ok'
    return result


def category_1_sub_ms():
    """Sub-Millisecond Gating — cycle shorter than LiDAR packet interval."""
    print(f"\n{'#'*70}")
    print(f"  CATEGORY 1: Sub-Millisecond Gating")
    print(f"  Cycles shorter than LiDAR packet interval (781µs)")
    print(f"{'#'*70}")

    configs = [
        (100,  80),  (100,  50),
        (250,  80),  (250,  50),
        (500,  80),  (500,  50),
        (750,  80),  (750,  50),
        (781,  80),  (781,  50),
    ]

    results = []
    for cycle, pct in configs:
        open_us = int(cycle * pct / 100)
        label = f"{cycle}µs/{pct}%"
        r = run_test(label, cycle, open_us, duration=10, category="sub_ms")
        results.append(r)
    return results


def category_2_buffer():
    """Buffer Capacity Mapping — find exact buffer threshold."""
    print(f"\n{'#'*70}")
    print(f"  CATEGORY 2: Buffer Capacity Mapping")
    print(f"  Finding exact buffer overflow threshold")
    print(f"{'#'*70}")

    results = []

    # Close time sweep at 20% open
    for cycle_ms in [10, 15, 20, 25, 30, 35, 40, 45, 50]:
        cycle_us = cycle_ms * 1000
        open_us = int(cycle_us * 0.20)
        label = f"{cycle_ms}ms/20%"
        r = run_test(label, cycle_us, open_us, duration=10, category="buffer")
        results.append(r)

    # Open % sweep at 50ms cycle
    for pct in [25, 30, 35, 40]:
        cycle_us = 50000
        open_us = int(cycle_us * pct / 100)
        label = f"50ms/{pct}%"
        r = run_test(label, cycle_us, open_us, duration=10, category="buffer")
        results.append(r)

    return results


def category_3_frame_aligned():
    """Frame-Aligned Gating — 100ms = exactly 1 LiDAR frame."""
    print(f"\n{'#'*70}")
    print(f"  CATEGORY 3: Frame-Aligned Gating")
    print(f"  100ms cycle = 1 LiDAR frame (128 packets)")
    print(f"{'#'*70}")

    results = []
    for pct in [90, 80, 70, 60, 50]:
        cycle_us = 100000
        open_us = int(cycle_us * pct / 100)
        label = f"100ms/{pct}%"
        r = run_test(label, cycle_us, open_us, duration=10, category="frame_aligned")
        results.append(r)
    return results


def category_4_multi_gcl():
    """Multi-GCL Entries — complex gate schedules."""
    print(f"\n{'#'*70}")
    print(f"  CATEGORY 4: Multi-GCL Entries")
    print(f"  2+ gate control list entries per cycle")
    print(f"{'#'*70}")

    results = []

    # 10ms 3-gate: open-close-open (2ms/6ms/2ms) = 40% open
    entries_3a = [
        {'gate': 255, 'time_ns': 2000000},   # 2ms open
        {'gate': 254, 'time_ns': 6000000},   # 6ms close
        {'gate': 255, 'time_ns': 2000000},   # 2ms open
    ]
    r = run_test("10ms 3-gate (2/6/2)", 10000, entries=entries_3a, duration=10, category="multi_gcl")
    results.append(r)

    # 10ms 3-gate: narrow pulse (1ms/8ms/1ms) = 20% open
    entries_3b = [
        {'gate': 255, 'time_ns': 1000000},   # 1ms open
        {'gate': 254, 'time_ns': 8000000},   # 8ms close
        {'gate': 255, 'time_ns': 1000000},   # 1ms open
    ]
    r = run_test("10ms 3-gate (1/8/1)", 10000, entries=entries_3b, duration=10, category="multi_gcl")
    results.append(r)

    # 20ms 4-gate: alternating (5/5/5/5ms) = 50% open
    entries_4a = [
        {'gate': 255, 'time_ns': 5000000},   # 5ms open
        {'gate': 254, 'time_ns': 5000000},   # 5ms close
        {'gate': 255, 'time_ns': 5000000},   # 5ms open
        {'gate': 254, 'time_ns': 5000000},   # 5ms close
    ]
    r = run_test("20ms 4-gate (5/5/5/5)", 20000, entries=entries_4a, duration=10, category="multi_gcl")
    results.append(r)

    # 100ms 4-gate: frame split (30/20/30/20ms) = 60% open
    entries_4b = [
        {'gate': 255, 'time_ns': 30000000},  # 30ms open
        {'gate': 254, 'time_ns': 20000000},  # 20ms close
        {'gate': 255, 'time_ns': 30000000},  # 30ms open
        {'gate': 254, 'time_ns': 20000000},  # 20ms close
    ]
    r = run_test("100ms 4-gate (30/20/30/20)", 100000, entries=entries_4b, duration=10, category="multi_gcl")
    results.append(r)

    return results


def category_5_endurance():
    """Endurance — 60-second measurements for statistical confidence."""
    print(f"\n{'#'*70}")
    print(f"  CATEGORY 5: Endurance (60s each)")
    print(f"  Long-duration stability testing")
    print(f"{'#'*70}")

    configs = [
        (1000,   500,  "1ms/50%"),
        (10000,  2000, "10ms/20%"),
        (50000,  10000,"50ms/20%"),
        (781,    390,  "781µs/50%"),
        (100000, 70000,"100ms/70%"),
    ]

    results = []
    for cycle, open_us, label in configs:
        r = run_test(label, cycle, open_us, duration=60, category="endurance")
        results.append(r)
    return results


def category_6_boundary():
    """Boundary Conditions — extreme parameter values."""
    print(f"\n{'#'*70}")
    print(f"  CATEGORY 6: Boundary Conditions")
    print(f"  Extreme parameters")
    print(f"{'#'*70}")

    results = []

    # Near-100% open with tiny close times
    for pct in [99, 98, 95, 90]:
        cycle_us = 1000
        open_us = int(cycle_us * pct / 100)
        label = f"1ms/{pct}%"
        r = run_test(label, cycle_us, open_us, duration=10, category="boundary")
        results.append(r)

    # Large cycle, moderate open
    r = run_test("200ms/80%", 200000, 160000, duration=10, category="boundary")
    results.append(r)

    # Very large cycle, 50% open (massive buffer test)
    r = run_test("500ms/50%", 500000, 250000, duration=10, category="boundary")
    results.append(r)

    return results


def main():
    print("="*70)
    print("  LiDAR TAS 종합 테스트 스위트")
    print("  6 categories, ~43 configs")
    print("="*70)

    # Kill processes on the UDP port
    print("\nStopping processes on port 7502...")
    os.system("fuser -k 7502/udp 2>/dev/null")
    time.sleep(2)

    all_results = {
        'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
        'categories': {},
        'summary': {},
    }

    start_time = time.time()

    # Run categories
    categories = [
        ('sub_ms', '1. Sub-Millisecond Gating', category_1_sub_ms),
        ('buffer', '2. Buffer Capacity Mapping', category_2_buffer),
        ('frame_aligned', '3. Frame-Aligned Gating', category_3_frame_aligned),
        ('multi_gcl', '4. Multi-GCL Entries', category_4_multi_gcl),
        ('endurance', '5. Endurance (60s)', category_5_endurance),
        ('boundary', '6. Boundary Conditions', category_6_boundary),
    ]

    total_tests = 0
    total_ok = 0

    for key, title, func in categories:
        print(f"\n\n{'*'*70}")
        print(f"  Starting: {title}")
        print(f"{'*'*70}")

        results = func()
        all_results['categories'][key] = results
        ok_count = sum(1 for r in results if r.get('status') == 'ok')
        total_tests += len(results)
        total_ok += ok_count
        print(f"\n  {title}: {ok_count}/{len(results)} tests completed")

    elapsed = time.time() - start_time

    # Restore all-open
    print("\n\nRestoring all-open TAS...")
    apply_tas_2entry(1000, 1000, "Restore all-open")

    # Summary
    all_results['summary'] = {
        'total_tests': total_tests,
        'completed': total_ok,
        'elapsed_sec': round(elapsed, 0),
        'elapsed_min': round(elapsed / 60, 1),
    }

    # Remove gap_samples from main results to reduce size, save separately
    all_flat = []
    for cat_key, results in all_results['categories'].items():
        for r in results:
            r_clean = {k: v for k, v in r.items() if k != 'gap_samples'}
            all_flat.append(r_clean)

    all_results['flat'] = all_flat

    # Save
    os.makedirs(os.path.dirname(OUTPUT_PATH), exist_ok=True)
    with open(OUTPUT_PATH, 'w') as f:
        json.dump(all_results, f, indent=2)
    print(f"\nResults saved to {OUTPUT_PATH}")

    # Print final summary table
    print(f"\n{'='*100}")
    print(f"  FINAL SUMMARY — {total_ok}/{total_tests} tests in {elapsed/60:.1f} minutes")
    print(f"{'='*100}")
    print(f"{'Label':<30} {'Cat':<12} {'Pkts':>6} {'Cmpl%':>6} {'GapSD':>8} {'P99':>8} {'Max':>8} {'Burst%':>7} {'Loss':>10}")
    print("-"*100)
    for r in all_flat:
        if r.get('status') != 'ok':
            print(f"{r['label']:<30} {r['category']:<12} {'FAILED':>6}")
            continue
        print(f"{r['label']:<30} {r['category']:<12} "
              f"{r['num_packets']:>6} {r['completeness']:>5.1f}% "
              f"{r.get('gap_stdev_us',0):>7.0f}µ {r.get('gap_p99_us',0):>7.0f}µ "
              f"{r.get('gap_max_us',0):>7.0f}µ {r.get('burst_pct',0):>6.1f}% "
              f"{r.get('loss_pattern','?'):>10}")


if __name__ == '__main__':
    main()
