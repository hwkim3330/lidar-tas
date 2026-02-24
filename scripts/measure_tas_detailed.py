#!/usr/bin/env python3
"""Detailed LiDAR packet timing measurement under TAS.

Records inter-packet gaps to detect TAS-induced burstiness.
"""
import socket
import time
import sys
import statistics

LIDAR_PORT = 7502
DURATION = 3

def measure(duration=DURATION):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # Increase receive buffer to avoid kernel drops
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    sock.bind(('0.0.0.0', LIDAR_PORT))
    sock.settimeout(0.5)

    print(f"Listening on UDP port {LIDAR_PORT} for {duration}s...")

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
        print("Not enough packets received")
        return

    # Compute inter-packet gaps in microseconds
    gaps = [(timestamps[i+1] - timestamps[i]) * 1e6 for i in range(len(timestamps)-1)]

    # Statistics
    mean_gap = statistics.mean(gaps)
    median_gap = statistics.median(gaps)
    stdev_gap = statistics.stdev(gaps) if len(gaps) > 1 else 0
    min_gap = min(gaps)
    max_gap = max(gaps)

    pkt_count = len(timestamps)
    elapsed = timestamps[-1] - timestamps[0]
    pps = pkt_count / elapsed if elapsed > 0 else 0

    print(f"\n--- Packet Timing Analysis ({pkt_count} packets, {elapsed:.2f}s) ---")
    print(f"Packets/sec: {pps:.1f}")
    print(f"Inter-packet gap (µs):")
    print(f"  Mean:   {mean_gap:.1f}")
    print(f"  Median: {median_gap:.1f}")
    print(f"  StdDev: {stdev_gap:.1f}")
    print(f"  Min:    {min_gap:.1f}")
    print(f"  Max:    {max_gap:.1f}")

    # Histogram of gaps
    buckets = [0]*10
    labels = ['<100', '100-200', '200-400', '400-600', '600-800',
              '800-1000', '1000-1500', '1500-2000', '2000-5000', '>5000']
    thresholds = [100, 200, 400, 600, 800, 1000, 1500, 2000, 5000, float('inf')]
    for g in gaps:
        for i, t in enumerate(thresholds):
            if g < t:
                buckets[i] += 1
                break

    print(f"\nGap Distribution:")
    for label, count in zip(labels, buckets):
        bar = '#' * min(count * 50 // max(max(buckets), 1), 50)
        print(f"  {label:>10s}: {count:5d} {bar}")

    # Detect bursts (packets arriving < 50µs apart)
    burst_count = sum(1 for g in gaps if g < 50)
    print(f"\nBurst packets (<50µs gap): {burst_count} ({burst_count*100/len(gaps):.1f}%)")

    # Check for TAS-like periodicity in large gaps
    large_gaps = [g for g in gaps if g > 500]
    if large_gaps:
        print(f"Large gaps (>500µs): {len(large_gaps)} occurrences")
        print(f"  Mean: {statistics.mean(large_gaps):.1f}µs")
        if len(large_gaps) > 1:
            # inter-large-gap intervals
            large_gap_indices = [i for i, g in enumerate(gaps) if g > 500]
            inter_large = [large_gap_indices[j+1] - large_gap_indices[j]
                          for j in range(len(large_gap_indices)-1)]
            if inter_large:
                print(f"  Pkts between large gaps: mean={statistics.mean(inter_large):.1f}")

if __name__ == '__main__':
    dur = int(sys.argv[1]) if len(sys.argv) > 1 else DURATION
    measure(dur)
