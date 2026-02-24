#!/usr/bin/env python3
"""Measure LiDAR packet reception under TAS gating.

Counts UDP packets on port 7502 for a fixed duration,
reports packets/sec and frame completeness.
"""
import socket
import time
import sys

LIDAR_PORT = 7502
DURATION = 5  # seconds
EXPECTED_PKT_SIZE = 3328  # Ouster col packet
PKTS_PER_FRAME = 128  # 2048 cols / 16 cols per packet
FRAMES_PER_SEC = 10

def measure(duration=DURATION):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', LIDAR_PORT))
    sock.settimeout(0.5)

    print(f"Listening on UDP port {LIDAR_PORT} for {duration}s...")

    pkt_count = 0
    pkt_sizes = {}
    t_start = time.monotonic()
    t_first = None
    t_last = None

    while True:
        now = time.monotonic()
        if now - t_start >= duration:
            break
        try:
            data, addr = sock.recvfrom(65535)
            if t_first is None:
                t_first = now
            t_last = now
            pkt_count += 1
            sz = len(data)
            pkt_sizes[sz] = pkt_sizes.get(sz, 0) + 1
        except socket.timeout:
            continue

    sock.close()

    elapsed = (t_last - t_first) if (t_first and t_last) else duration
    pps = pkt_count / elapsed if elapsed > 0 else 0
    expected_pps = PKTS_PER_FRAME * FRAMES_PER_SEC  # 1280 pps
    completeness = (pps / expected_pps) * 100 if expected_pps > 0 else 0

    print(f"\n--- Results ({elapsed:.2f}s) ---")
    print(f"Total packets: {pkt_count}")
    print(f"Packets/sec: {pps:.1f} (expected: {expected_pps})")
    print(f"Completeness: {completeness:.1f}%")
    print(f"Packet sizes: {dict(sorted(pkt_sizes.items()))}")

    return pkt_count, pps, completeness

if __name__ == '__main__':
    dur = int(sys.argv[1]) if len(sys.argv) > 1 else DURATION
    measure(dur)
