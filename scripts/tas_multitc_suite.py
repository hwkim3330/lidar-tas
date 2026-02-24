#!/usr/bin/env python3
"""
ROII-Style Multi-TC TAS 스케줄링 테스트
Per-TC gate bitmask를 사용한 자동차 네트워크 시뮬레이션

실제 트래픽: Ouster LiDAR → TC0 (무태깅, LAN9662 기본 매핑)
Gate bitmask: 0x01=TC0, 0x04=TC2, 0x20=TC5, 0x40=TC6, 0xFF=AllOpen

Categories:
A. TC0 Allocation Sweep (8 tests) — TC0 시간 할당 변화
B. Fragmentation Effect (6 tests) — 동일 시간, 분산 vs 집중
C. Cycle Time Scaling (6 tests) — ROII 비율 고정, 주기 변경
D. ROII Realistic Profiles (5 tests) — 자동차 센서 프로필
E. Guard Band Effect (5 tests) — Guard band(0xFF) 유무 비교
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
OUTPUT_PATH = '/home/kim/lidar-tas/data/multitc_results.json'

# Gate bitmask constants
GATE_TC0 = 0x01       # 00000001 — TC0 only (LiDAR)
GATE_TC2 = 0x04       # 00000100 — TC2 only (Control)
GATE_TC5 = 0x20       # 00100000 — TC5 only (Camera)
GATE_TC6 = 0x40       # 01000000 — TC6 only (Radar)
GATE_ALL = 0xFF       # 11111111 — All TCs open (guard band)
GATE_NO_TC0 = 0xFE    # 11111110 — All except TC0

# TC name mapping for display
TC_NAMES = {
    GATE_TC0: 'TC0(LiDAR)',
    GATE_TC2: 'TC2(Control)',
    GATE_TC5: 'TC5(Camera)',
    GATE_TC6: 'TC6(Radar)',
    GATE_ALL: 'Guard(ALL)',
    GATE_NO_TC0: 'NoTC0',
}

def gate_desc(gate):
    """Human-readable gate description."""
    return TC_NAMES.get(gate, f'0x{gate:02X}')


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

    expected_pkts = PKTS_PER_FRAME * 10 * (duration_sec / 10)
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


def apply_tas_multi(cycle_us, entries, label=""):
    """Apply multi-entry TAS config with per-TC gate bitmasks.
    entries: list of {'gate': int, 'time_ns': int}
    """
    yaml_path = os.path.join(KETI_TSN_DIR, 'lidar-tas', '_multitc_config.yaml')
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
        f"{gate_desc(e['gate'])}({e['time_ns']/1e6:.2f}ms)"
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


def us_to_ns(us):
    """Convert microseconds to nanoseconds."""
    return int(us * 1000)


def ms_to_ns(ms):
    """Convert milliseconds to nanoseconds."""
    return int(ms * 1000000)


def ms_to_us(ms):
    """Convert milliseconds to microseconds."""
    return int(ms * 1000)


def run_test(label, cycle_us, entries, duration=10, category=""):
    """Run a single multi-TC test config and return result."""
    info = {
        'label': label,
        'category': category,
        'cycle_us': cycle_us,
        'duration': duration,
        'entries': [{'gate': e['gate'], 'gate_hex': f"0x{e['gate']:02X}",
                     'gate_desc': gate_desc(e['gate']),
                     'time_ns': e['time_ns'],
                     'time_ms': round(e['time_ns'] / 1e6, 3)} for e in entries],
    }

    # Compute effective TC0 open time
    tc0_open_ns = sum(e['time_ns'] for e in entries if e['gate'] & 0x01)
    cycle_ns = cycle_us * 1000
    info['tc0_open_pct'] = round(tc0_open_ns / cycle_ns * 100, 1)
    info['tc0_open_ms'] = round(tc0_open_ns / 1e6, 3)

    ok = apply_tas_multi(cycle_us, entries, label)

    if not ok:
        print(f"  Config failed, skipping capture")
        info['status'] = 'config_failed'
        return info

    ts = capture_packets(duration, label)
    result = analyze_gaps(ts, label, duration)
    result.update(info)
    result['status'] = 'ok'
    return result


# ══════════════════════════════════════════════════════════════
# CATEGORY A: TC0 Allocation Sweep
# 10ms cycle, 3-entry: TC0(LiDAR) → TC6(Radar) → TC2(Control)
# TC0 time from 1ms to 8ms, remainder split between TC6 and TC2
# ══════════════════════════════════════════════════════════════

def category_a_tc0_sweep():
    """TC0 allocation sweep: vary TC0 time from 1ms to 8ms in 10ms cycle."""
    print(f"\n{'#'*70}")
    print(f"  CATEGORY A: TC0 Allocation Sweep")
    print(f"  10ms cycle: TC0(0x01) → TC6(0x40) → TC2(0x04)")
    print(f"{'#'*70}")

    results = []
    configs = [
        # (tc0_ms, tc6_ms, tc2_ms)
        (1, 5, 4),
        (2, 4, 4),
        (3, 4, 3),
        (4, 3, 3),
        (5, 3, 2),
        (6, 2, 2),
        (7, 2, 1),
        (8, 1, 1),
    ]

    for tc0, tc6, tc2 in configs:
        label = f"TC0={tc0}ms ({tc0*10}%)"
        entries = [
            {'gate': GATE_TC0, 'time_ns': ms_to_ns(tc0)},
            {'gate': GATE_TC6, 'time_ns': ms_to_ns(tc6)},
            {'gate': GATE_TC2, 'time_ns': ms_to_ns(tc2)},
        ]
        r = run_test(label, 10000, entries, duration=10, category="tc0_sweep")
        results.append(r)

    return results


# ══════════════════════════════════════════════════════════════
# CATEGORY B: Fragmentation Effect
# 10ms cycle, 4ms total TC0 — split into different numbers of windows
# ══════════════════════════════════════════════════════════════

def category_b_fragmentation():
    """Fragmentation: same total TC0 time (4ms), different split patterns."""
    print(f"\n{'#'*70}")
    print(f"  CATEGORY B: Fragmentation Effect")
    print(f"  10ms cycle, 4ms total TC0 — concentrated vs fragmented")
    print(f"{'#'*70}")

    results = []

    # B1: 1 contiguous block: TC0(4) + TC6(6)
    r = run_test("1×4ms (contiguous)", 10000, [
        {'gate': GATE_TC0, 'time_ns': ms_to_ns(4)},
        {'gate': GATE_TC6, 'time_ns': ms_to_ns(6)},
    ], duration=10, category="fragmentation")
    results.append(r)

    # B2: 2 blocks: TC0(2) + TC6(3) + TC0(2) + TC2(3)
    r = run_test("2×2ms (split)", 10000, [
        {'gate': GATE_TC0, 'time_ns': ms_to_ns(2)},
        {'gate': GATE_TC6, 'time_ns': ms_to_ns(3)},
        {'gate': GATE_TC0, 'time_ns': ms_to_ns(2)},
        {'gate': GATE_TC2, 'time_ns': ms_to_ns(3)},
    ], duration=10, category="fragmentation")
    results.append(r)

    # B3: 3 blocks: TC0(1.33) + TC6(2) + TC0(1.33) + TC2(2) + TC0(1.34) + TC6(2)
    # Round to ns: 1333333 + 2000000 + 1333333 + 2000000 + 1333334 + 2000000 = 10000000
    r = run_test("3×1.3ms (fragmented)", 10000, [
        {'gate': GATE_TC0, 'time_ns': 1333333},
        {'gate': GATE_TC6, 'time_ns': 2000000},
        {'gate': GATE_TC0, 'time_ns': 1333333},
        {'gate': GATE_TC2, 'time_ns': 2000000},
        {'gate': GATE_TC0, 'time_ns': 1333334},
        {'gate': GATE_TC6, 'time_ns': 2000000},
    ], duration=10, category="fragmentation")
    results.append(r)

    # B4: Fast alternation TC0(2)+TC6(2) × 2 + TC0(2)  (but 8 max GCL entries)
    # TC0(1)+TC6(1) × 5 = 5 windows of TC0
    r = run_test("5×1ms (high freq)", 10000, [
        {'gate': GATE_TC0, 'time_ns': ms_to_ns(1)},
        {'gate': GATE_TC6, 'time_ns': ms_to_ns(1)},
        {'gate': GATE_TC0, 'time_ns': ms_to_ns(1)},
        {'gate': GATE_TC6, 'time_ns': ms_to_ns(1)},
        {'gate': GATE_TC0, 'time_ns': ms_to_ns(1)},
        {'gate': GATE_TC6, 'time_ns': ms_to_ns(1)},
        {'gate': GATE_TC0, 'time_ns': ms_to_ns(1)},
        {'gate': GATE_TC6, 'time_ns': ms_to_ns(3)},
    ], duration=10, category="fragmentation")
    results.append(r)

    # B5: TC0(0.5)+TC6(0.5) × 8 = 8 windows of TC0, 500µs each (max GCL = 8)
    r = run_test("4×0.5ms (ultra-fast)", 10000, [
        {'gate': GATE_TC0, 'time_ns': us_to_ns(500)},
        {'gate': GATE_TC6, 'time_ns': us_to_ns(500)},
        {'gate': GATE_TC0, 'time_ns': us_to_ns(500)},
        {'gate': GATE_TC6, 'time_ns': us_to_ns(500)},
        {'gate': GATE_TC0, 'time_ns': us_to_ns(500)},
        {'gate': GATE_TC6, 'time_ns': us_to_ns(500)},
        {'gate': GATE_TC0, 'time_ns': us_to_ns(500)},
        {'gate': GATE_TC6, 'time_ns': us_to_ns(6500)},
    ], duration=10, category="fragmentation")
    results.append(r)

    # B6: 2ms contiguous TC0 (half of B1) for comparison
    r = run_test("1×2ms (half baseline)", 10000, [
        {'gate': GATE_TC0, 'time_ns': ms_to_ns(2)},
        {'gate': GATE_TC6, 'time_ns': ms_to_ns(8)},
    ], duration=10, category="fragmentation")
    results.append(r)

    return results


# ══════════════════════════════════════════════════════════════
# CATEGORY C: Cycle Time Scaling
# ROII ratio fixed (TC0:40%, TC6:30%, TC2:20%, Guard:10%)
# Only cycle time changes
# ══════════════════════════════════════════════════════════════

def category_c_cycle_scaling():
    """Cycle time scaling with fixed ROII ratio."""
    print(f"\n{'#'*70}")
    print(f"  CATEGORY C: Cycle Time Scaling")
    print(f"  Fixed ratio: TC0=40%, TC6=30%, TC2=20%, Guard=10%")
    print(f"{'#'*70}")

    results = []
    cycles_ms = [2, 5, 10, 20, 50, 100]

    for cycle_ms in cycles_ms:
        tc0_ns = ms_to_ns(cycle_ms * 0.4)
        tc6_ns = ms_to_ns(cycle_ms * 0.3)
        tc2_ns = ms_to_ns(cycle_ms * 0.2)
        guard_ns = ms_to_ns(cycle_ms * 0.1)

        label = f"{cycle_ms}ms cycle (ROII)"
        entries = [
            {'gate': GATE_TC0,  'time_ns': tc0_ns},
            {'gate': GATE_TC6,  'time_ns': tc6_ns},
            {'gate': GATE_TC2,  'time_ns': tc2_ns},
            {'gate': GATE_ALL,  'time_ns': guard_ns},
        ]
        r = run_test(label, ms_to_us(cycle_ms), entries, duration=10, category="cycle_scaling")
        results.append(r)

    return results


# ══════════════════════════════════════════════════════════════
# CATEGORY D: ROII Realistic Profiles
# 10ms cycle, named automotive sensor profiles
# ══════════════════════════════════════════════════════════════

def category_d_roii_profiles():
    """ROII realistic automotive sensor profiles."""
    print(f"\n{'#'*70}")
    print(f"  CATEGORY D: ROII Realistic Profiles")
    print(f"  10ms cycle, automotive sensor scheduling")
    print(f"{'#'*70}")

    results = []

    profiles = [
        # (name, tc0_ms, tc6_ms, tc5_ms, tc2_ms, guard_ms)
        ("ROII-Standard", 4.0, 2.5, 1.5, 1.0, 1.0),
        ("LiDAR-Heavy",   6.0, 1.5, 1.0, 0.5, 1.0),
        ("Radar-Heavy",   2.0, 5.0, 1.0, 1.0, 1.0),
        ("Camera-Heavy",  2.0, 1.0, 5.0, 1.0, 1.0),
        ("Equal-Share",   2.0, 2.0, 2.0, 2.0, 2.0),
    ]

    for name, tc0, tc6, tc5, tc2, guard in profiles:
        entries = [
            {'gate': GATE_TC0,  'time_ns': ms_to_ns(tc0)},
            {'gate': GATE_TC6,  'time_ns': ms_to_ns(tc6)},
            {'gate': GATE_TC5,  'time_ns': ms_to_ns(tc5)},
            {'gate': GATE_TC2,  'time_ns': ms_to_ns(tc2)},
            {'gate': GATE_ALL,  'time_ns': ms_to_ns(guard)},
        ]
        r = run_test(name, 10000, entries, duration=10, category="roii_profiles")
        results.append(r)

    return results


# ══════════════════════════════════════════════════════════════
# CATEGORY E: Guard Band Effect
# 10ms cycle, TC0=4ms fixed, guard band (0xFF) varies
# Guard band opens ALL TCs including TC0
# ══════════════════════════════════════════════════════════════

def category_e_guard_band():
    """Guard band effect: vary guard time with fixed TC0."""
    print(f"\n{'#'*70}")
    print(f"  CATEGORY E: Guard Band Effect")
    print(f"  10ms cycle, TC0=4ms fixed, guard band(0xFF) varies")
    print(f"  Guard opens ALL TCs → effective TC0 = TC0_slot + guard_slot")
    print(f"{'#'*70}")

    results = []

    configs = [
        # (guard_ms, tc6_ms) — tc0=4ms fixed, total=10ms
        (0, 6),     # No guard: TC0(4) + TC6(6)
        (1, 5),     # 1ms guard: TC0(4) + Guard(1) + TC6(5) → eff TC0=5ms
        (2, 4),     # 2ms guard: TC0(4) + Guard(2) + TC6(4) → eff TC0=6ms
        (3, 3),     # 3ms guard: TC0(4) + Guard(3) + TC6(3) → eff TC0=7ms
        (6, 0),     # Max guard: TC0(4) + Guard(6) → eff TC0=10ms (100%)
    ]

    for guard_ms, tc6_ms in configs:
        eff_tc0 = 4 + guard_ms
        label = f"Guard={guard_ms}ms (eff TC0={eff_tc0}ms)"
        entries = [
            {'gate': GATE_TC0, 'time_ns': ms_to_ns(4)},
        ]
        if guard_ms > 0:
            entries.append({'gate': GATE_ALL, 'time_ns': ms_to_ns(guard_ms)})
        if tc6_ms > 0:
            entries.append({'gate': GATE_TC6, 'time_ns': ms_to_ns(tc6_ms)})

        r = run_test(label, 10000, entries, duration=10, category="guard_band")
        results.append(r)

    return results


def main():
    print("="*70)
    print("  ROII-Style Multi-TC TAS 스케줄링 테스트")
    print("  5 categories, ~30 configs")
    print("  Per-TC gate bitmask: TC0=0x01, TC6=0x40, TC5=0x20, TC2=0x04")
    print("="*70)

    # Kill processes on the UDP port
    print("\nStopping processes on port 7502...")
    os.system("fuser -k 7502/udp 2>/dev/null")
    time.sleep(2)

    all_results = {
        'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
        'categories': {},
        'summary': {},
        'gate_bitmask_legend': {
            '0x01': 'TC0 (LiDAR)',
            '0x04': 'TC2 (Control)',
            '0x20': 'TC5 (Camera)',
            '0x40': 'TC6 (Radar)',
            '0xFF': 'All TCs (Guard Band)',
            '0xFE': 'All except TC0',
        },
    }

    start_time = time.time()

    categories = [
        ('tc0_sweep',     'A. TC0 Allocation Sweep',  category_a_tc0_sweep),
        ('fragmentation', 'B. Fragmentation Effect',   category_b_fragmentation),
        ('cycle_scaling', 'C. Cycle Time Scaling',     category_c_cycle_scaling),
        ('roii_profiles', 'D. ROII Realistic Profiles', category_d_roii_profiles),
        ('guard_band',    'E. Guard Band Effect',       category_e_guard_band),
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
    apply_tas_multi(1000, [{'gate': GATE_ALL, 'time_ns': ms_to_ns(1)}], "Restore all-open")

    # Summary
    all_results['summary'] = {
        'total_tests': total_tests,
        'completed': total_ok,
        'elapsed_sec': round(elapsed, 0),
        'elapsed_min': round(elapsed / 60, 1),
    }

    # Flatten results (without gap_samples to reduce size)
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
    print(f"\n{'='*110}")
    print(f"  FINAL SUMMARY — {total_ok}/{total_tests} tests in {elapsed/60:.1f} minutes")
    print(f"{'='*110}")
    print(f"{'Label':<30} {'Cat':<15} {'TC0%':>5} {'Pkts':>6} {'Cmpl%':>6} {'GapSD':>8} {'P99':>8} {'Burst%':>7} {'Loss':>10}")
    print("-"*110)
    for r in all_flat:
        if r.get('status') != 'ok':
            print(f"{r['label']:<30} {r['category']:<15} {'FAILED':>6}")
            continue
        print(f"{r['label']:<30} {r['category']:<15} "
              f"{r.get('tc0_open_pct',0):>4.0f}% "
              f"{r['num_packets']:>6} {r['completeness']:>5.1f}% "
              f"{r.get('gap_stdev_us',0):>7.0f}µ {r.get('gap_p99_us',0):>7.0f}µ "
              f"{r.get('burst_pct',0):>6.1f}% "
              f"{r.get('loss_pattern','?'):>10}")


if __name__ == '__main__':
    main()
