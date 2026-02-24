# LiDAR TAS 실험

Ouster OS-1-16 LiDAR + Microchip LAN9662 TSN 스위치를 사용한 IEEE 802.1Qbv (TAS) 종합 실험.

**GitHub Pages**: https://hwkim3330.github.io/lidar-tas/

## 실험 구성

```
Ouster OS-1-16  →  LAN9662 (TAS)  →  PC
192.168.6.11       Port2 → Port1     192.168.6.1
UDP 7502           802.1Qbv          Python socket
```

## LiDAR 스펙

| 항목 | 값 |
|------|-----|
| 모델 | Ouster OS-1-16-A0 |
| 해상도 | 2048 × 16 |
| 회전 | 10 Hz |
| 패킷/프레임 | 128 (3,328 bytes/pkt) |
| 패킷 간격 | ~781 µs |
| 데이터 레이트 | ~34 Mbps |

## 종합 실험 결과

### 전체 요약

| 항목 | 결과 |
|------|------|
| 총 테스트 수 | 86 (기본 13 + 종합 43 + Multi-TC 30) |
| 카테고리 | 13 (기본 8 + Multi-TC 5) |
| 100% 수신 | 36/43 (종합), 25/30 (Multi-TC), 12/13 (기본) |
| Total Loss (0%) | 12개 설정 |
| 측정 시간 | ~19분 (종합 13min + Multi-TC 6min) |

### 기본 Sweep (13 configs)

| Config | Cycle | Open% | Completeness | Gap SD | Burst% |
|--------|-------|-------|-------------|--------|--------|
| Baseline | — | 100% | **100%** | 159µs | 0.1% |
| 1ms/80% | 1ms | 80% | **100%** | 240µs | 0.6% |
| 1ms/50% | 1ms | 50% | **100%** | 344µs | 5.3% |
| 1ms/20% | 1ms | 20% | **100%** | 422µs | 22.0% |
| 5ms/80% | 5ms | 80% | **100%** | 450µs | 10.0% |
| 5ms/50% | 5ms | 50% | **100%** | 168µs | 0.2% |
| 5ms/20% | 5ms | 20% | **100%** | 1652µs | 68.9% |
| 10ms/80% | 10ms | 80% | **100%** | 180µs | 0.3% |
| 10ms/50% | 10ms | 50% | **100%** | 1431µs | 41.4% |
| 10ms/20% | 10ms | 20% | **100%** | 158µs | 0.2% |
| 50ms/80% | 50ms | 80% | **100%** | 1267µs | 17.2% |
| 50ms/50% | 50ms | 50% | **100%** | 171µs | 0.1% |
| 50ms/20% | 50ms | 20% | **51.6%** | 6922µs | 55.1% |

### Cat 1: Sub-Millisecond Gating (10 tests)

LiDAR 패킷 간격(781µs)보다 짧은 TAS cycle. **10/10 전부 100% 수신.**

| Config | Cycle | Open% | Gap SD | Burst% |
|--------|-------|-------|--------|--------|
| 100µs/80% | 100µs | 80% | 140µs | 0.1% |
| 100µs/50% | 100µs | 50% | 159µs | 0.2% |
| 250µs/80% | 250µs | 80% | 177µs | 0.2% |
| 250µs/50% | 250µs | 50% | **90µs** | 0.0% |
| 500µs/80% | 500µs | 80% | 227µs | 0.3% |
| 500µs/50% | 500µs | 50% | 310µs | 1.1% |
| 750µs/80% | 750µs | 80% | 206µs | 0.4% |
| 750µs/50% | 750µs | 50% | 226µs | 1.0% |
| 781µs/80% | 781µs | 80% | 180µs | 0.3% |
| 781µs/50% | 781µs | 50% | 206µs | 0.8% |

### Cat 2: Buffer Capacity Mapping (13 tests)

Close time 증가에 따른 버퍼 오버플로우 임계점 탐색.

| Config | Close | Completeness | 비고 |
|--------|-------|-------------|------|
| 10ms/20% | 8ms | **100%** | 안전 |
| 15ms/20% | 12ms | **100%** | 안전 |
| 20ms/20% | 16ms | **100%** | 경계 (burst 73%) |
| 25ms/20% | 20ms | **0%** | Total Loss |
| 30ms/20% | 24ms | **72%** | 부분 수신 |
| 35ms/20% | 28ms | **0%** | Total Loss |
| 40ms/20% | 32ms | **58%** | 부분 수신 |
| 45ms/20% | 36ms | **0%** | Total Loss |
| 50ms/20% | 40ms | **51%** | 부분 수신 |
| 50ms/25% | 37.5ms | **100%** | 동기화 |
| 50ms/30% | 35ms | **61%** | 부분 수신 |
| 50ms/35% | 32.5ms | **100%** | 동기화 |
| 50ms/40% | 30ms | **72%** | 부분 수신 |

### Cat 3: Frame-Aligned Gating (5 tests)

100ms cycle = LiDAR 1프레임. **5/5 전부 100% 수신.**

| Config | Close | Gap SD | Burst% |
|--------|-------|--------|--------|
| 100ms/90% | 10ms | 168µs | 0.2% |
| 100ms/80% | 20ms | 1827µs | 14.5% |
| 100ms/70% | 30ms | 177µs | 0.2% |
| 100ms/60% | 40ms | 4068µs | 18.5% |
| 100ms/50% | 50ms | 167µs | 0.1% |

### Cat 4: Multi-GCL Entries (4 tests)

| Config | Schedule | Open% | Completeness |
|--------|----------|-------|-------------|
| 10ms 3-gate (2/6/2) | O(2ms)+C(6ms)+O(2ms) | 40% | **100%** |
| 10ms 3-gate (1/8/1) | O(1ms)+C(8ms)+O(1ms) | 20% | **0%** |
| 20ms 4-gate (5/5/5/5) | O(5)+C(5)+O(5)+C(5) | 50% | **100%** |
| 100ms 4-gate (30/20/30/20) | O(30)+C(20)+O(30)+C(20) | 60% | **0%** |

### Cat 5: Endurance — 60초 (5 tests)

| Config | Duration | Packets | Completeness | Gap SD |
|--------|----------|---------|-------------|--------|
| 1ms/50% | 60s | 76,801 | **100%** | 358µs |
| 10ms/20% | 60s | 76,800 | **100%** | 2293µs |
| 50ms/20% | 60s | 0 | **0%** | — |
| 781µs/50% | 60s | 76,800 | **100%** | 196µs |
| 100ms/70% | 60s | 65,661 | **100%** | 2875µs |

### Cat 6: Boundary Conditions (6 tests)

| Config | Close | Completeness | Gap SD |
|--------|-------|-------------|--------|
| 1ms/99% | 10µs | **100%** | 152µs |
| 1ms/98% | 20µs | **100%** | 164µs |
| 1ms/95% | 50µs | **100%** | 199µs |
| 1ms/90% | 100µs | **100%** | 217µs |
| 200ms/80% | 40ms | **100%** | 2672µs |
| 500ms/50% | 250ms | **100%** | 178µs |

## 핵심 발견

### 1. Beat Frequency 동기화

TAS cycle과 LiDAR 패킷 주기(781µs)의 beat frequency에 따라 결과가 극단적으로 갈림:
- 25ms/35ms/45ms (20% open) → **0% (total loss)**
- 30ms/40ms/50ms (20% open) → **51~72% (부분 수신)**
- 500ms/50% (250ms close) → **100% (동기화)**

이 효과는 **비결정적**이므로 안전 설계에 의존 불가.

### 2. 버퍼 한계

LAN9662 TC0 큐 버퍼 ≈ 20~25 패킷 (close 16~20ms에서 전환점).
버퍼 초과 시 패킷 드롭 발생.

### 3. Sub-ms 안전성

100µs~781µs cycle 전부 100% 수신. 패킷 간격보다 짧은 gate cycle은 확률적 지연만 추가.

### 4. 장시간 불안정성

50ms/20%: 10초 → 51.6%, 60초 → 0%. 동기화 위상이 시간에 따라 변동하여 장기 운영에서 불안정.

## TSN 설계 가이드라인

### DO
- Close time ≤ 10ms 유지 (안전 영역)
- Sub-ms cycle 자유 사용 (100~781µs 전부 안전)
- LiDAR 프레임 주기(100ms)의 정수배 cycle 활용
- Open ≥ 50% 할당
- Multi-GCL 사용 시 개별 open window ≥ 2ms 확보
- RX 버퍼 16MB+ 설정

### DON'T
- Close time ≥ 20ms 사용 금지 (버퍼 오버플로우)
- Multi-GCL에서 1ms 이하 open window 사용 금지
- Beat frequency 동기화에 의존한 설계 금지
- 단기(10초) 테스트 결과를 장기 운영에 외삽 금지

## Jitter 원인 진단

All-open 상태에서도 stdev ~170µs의 jitter가 관측되어 원인 분석 수행:

| 설정 | Gap StdDev | P99 | Burst% | 비고 |
|------|-----------|-----|--------|------|
| All-Open (1ms cycle) | 173 µs | 1271 µs | 0.3% | baseline |
| All-Open (10ms cycle) | 182 µs | 1282 µs | 0.2% | baseline |
| All-Open (100ms cycle) | 167 µs | 1251 µs | 0.2% | baseline |

**결론**: TAS cycle time 변경에 무관하게 jitter 동일 → **TAS gate 전환이 원인 아님**

원인:
- **USB Ethernet 어댑터**: URB 스케줄링 간격 (125µs~1ms)
- **스위치 Store-and-Forward**: 3,328B 패킷 수신 후 전달 시 가변 큐잉 지연
- **OS 커널 스케줄링**: socket.recvfrom() 유저스페이스 전환 지연

## Multi-TC TAS 스케줄링 (ROII-Style)

Per-TC gate bitmask를 사용한 자동차 자율주행 네트워크 시뮬레이션. 실제 LiDAR(TC0) 트래픽에 ROII 센서 프로필 적용.

### Gate Bitmask

| Gate Value | Binary | Open TC | 용도 |
|-----------|--------|---------|------|
| 0x01 | 00000001 | TC0 | LiDAR 전용 |
| 0x40 | 01000000 | TC6 | Radar 시뮬 |
| 0x20 | 00100000 | TC5 | Camera 시뮬 |
| 0x04 | 00000100 | TC2 | Control 시뮬 |
| 0xFF | 11111111 | 전체 | Guard band |

### Cat A: TC0 Allocation Sweep (8 tests)

10ms cycle, TC0(0x01)→TC6(0x40)→TC2(0x04). TC0 할당 1~8ms sweep.

| Config | TC0 | TC0% | Completeness | Gap SD | Burst% |
|--------|-----|------|-------------|--------|--------|
| TC0=1ms | 1ms | 10% | **100%** | 2565µs | 79.5% |
| TC0=2ms | 2ms | 20% | **0%** | — | — |
| TC0=3ms | 3ms | 30% | **100%** | 2023µs | 63.9% |
| TC0=4ms | 4ms | 40% | **0%** | — | — |
| TC0=5ms | 5ms | 50% | **100%** | 1450µs | 45.0% |
| TC0=6ms | 6ms | 60% | **100%** | 175µs | 0.2% |
| TC0=7ms | 7ms | 70% | **100%** | 892µs | 22.8% |
| TC0=8ms | 8ms | 80% | **100%** | 159µs | 0.1% |

TC0=2ms, 4ms에서 beat frequency로 total loss. TC0% 증가가 반드시 개선을 의미하지 않음.

### Cat B: Fragmentation Effect (6 tests)

동일 TC0 총 시간(4ms)을 분산 vs 집중 배치.

| Config | Windows | Completeness | Gap SD | Burst% |
|--------|---------|-------------|--------|--------|
| 1×4ms (contiguous) | 1 | **100%** | 1749µs | 51.2% |
| 2×2ms (split) | 2 | **100%** | **181µs** | 0.2% |
| 3×1.3ms (fragmented) | 3 | **100%** | 1032µs | 45.5% |
| 5×1ms (high freq) | 5 | **100%** | **162µs** | 0.2% |
| 4×0.5ms (ultra-fast) | 4 | **100%** | 1844µs | 56.8% |
| 1×2ms (half baseline) | 1 | **0%** | — | — |

**핵심**: 분산 배치(2×2ms, 5×1ms)가 집중 배치(1×4ms)보다 jitter 10배 감소!

### Cat C: Cycle Time Scaling (6 tests)

ROII 비율 고정 (TC0=40%, TC6=30%, TC2=20%, Guard=10%), cycle만 변경.

| Cycle | TC0 | Completeness | Gap SD | Burst% |
|-------|-----|-------------|--------|--------|
| 2ms | 0.8ms | **100%** | 685µs | 24.6% |
| 5ms | 2ms | **100%** | 1039µs | 34.1% |
| 10ms | 4ms | **100%** | 1452µs | 43.7% |
| 20ms | 8ms | **100%** | **160µs** | 0.1% |
| 50ms | 20ms | **100%** | 3515µs | 33.7% |
| 100ms | 40ms | **100%** | **175µs** | 0.3% |

20ms와 100ms에서 baseline급 안정. 50ms에서 close=25ms로 burst 증가.

### Cat D: ROII Realistic Profiles (5 tests)

10ms cycle, 자동차 센서 프로필. 5-entry GCL: TC0→TC6→TC5→TC2→Guard.

| Profile | TC0 | TC6 | TC5 | TC2 | Guard | Eff TC0% | Completeness | Gap SD |
|---------|-----|-----|-----|-----|-------|----------|-------------|--------|
| ROII-Standard | 4ms | 2.5ms | 1.5ms | 1ms | 1ms | 50% | **100%** | 1451µs |
| LiDAR-Heavy | 6ms | 1.5ms | 1ms | 0.5ms | 1ms | 70% | **100%** | **186µs** |
| Radar-Heavy | 2ms | 5ms | 1ms | 1ms | 1ms | 30% | **100%** | 2005µs |
| Camera-Heavy | 2ms | 1ms | 5ms | 1ms | 1ms | 30% | **0%** | — |
| Equal-Share | 2ms | 2ms | 2ms | 2ms | 2ms | 40% | **100%** | 1739µs |

LiDAR-Heavy(TC0=6ms+Guard=70%)가 가장 안정. Camera-Heavy에서 beat frequency로 total loss.

### Cat E: Guard Band Effect (5 tests)

TC0=4ms 고정, Guard(0xFF) 크기 변화. Guard는 TC0 포함.

| Guard | Eff TC0 | Completeness | Gap SD |
|-------|---------|-------------|--------|
| 0ms | 4ms (40%) | **100%** | 159µs |
| 1ms | 5ms (50%) | **100%** | 1452µs |
| 2ms | 6ms (60%) | **0%** | — |
| 3ms | 7ms (70%) | **100%** | 882µs |
| 6ms | 10ms (100%) | **100%** | 172µs |

Guard=2ms(eff 60%)에서 total loss — beat frequency. Guard band 자체가 TC0 open 역할.

### Multi-TC 핵심 발견

1. **TC0% 증가 ≠ 안정성 증가**: beat frequency가 지배적
2. **분산 배치 >> 집중 배치**: 2×2ms(181µs) vs 1×4ms(1749µs) — 10배 개선
3. **Guard band = TC0 확장**: 0xFF는 TC0 포함이므로 effective TC0 시간 증가
4. **LiDAR-Heavy 프로필 권장**: TC0=6ms + Guard=1ms (70% effective)

### Multi-TC 설계 가이드라인

- TC0 슬롯을 2개 이상으로 분산 배치
- Guard band(0xFF)를 TC0 직후에 배치
- LiDAR TC에 ≥ 60% effective open 할당
- 20ms 또는 100ms cycle 사용 (beat frequency 동기화)

## 실시간 서버

3D 포인트 클라우드 뷰어 + 실시간 TAS 제어 웹 UI:

```bash
pip3 install flask flask-cors ouster-sdk numpy requests
python3 scripts/lidar_tas_server.py
# → http://localhost:8080
```

## 파일 구조

```
├── index.html                    # 결과 시각화 (GitHub Pages)
├── data/
│   ├── sweep_results.json        # Extended sweep (1ms ~ 50ms)
│   ├── sweep_1ms_results.json    # 1ms cycle sweep
│   ├── jitter_diagnosis.json     # Jitter 원인 진단 결과
│   ├── comprehensive_results.json # 종합 테스트 (43 configs)
│   └── multitc_results.json      # Multi-TC 테스트 (30 configs)
├── configs/
│   ├── tas-enable.yaml           # TAS 활성화 (keti-tsn-cli용)
│   ├── tas-disable.yaml          # TAS 비활성화 (all-open)
│   └── fetch-tas.yaml            # TAS 상태 조회
├── scripts/
│   ├── lidar_tas_server.py       # 실시간 3D 뷰어 + TAS 제어 서버
│   ├── tas_comprehensive_suite.py # 종합 테스트 스위트 (6 categories, 43 configs)
│   ├── tas_multitc_suite.py      # Multi-TC 테스트 (5 categories, 30 configs)
│   ├── jitter_diagnosis.py       # Jitter 원인 진단 (8개 설정 비교)
│   ├── tas_sweep.py              # 1ms cycle sweep
│   ├── tas_sweep_extended.py     # Extended sweep
│   ├── measure_tas.py            # 단순 패킷 카운트
│   ├── measure_tas_detailed.py   # 상세 타이밍 측정
│   └── xdp_gate.c                # XDP eBPF gate (참고용)
└── README.md
```

## 주의사항

- TAS 비활성화 시 `gate-enabled: false`를 사용하면 안 됨 — gate가 닫힌 상태로 고정됨
- 대신 `gate-enabled: true` + 단일 all-open GCL 항목 사용 (tas-disable.yaml 참고)
- LAN9662 TC0 큐 버퍼 크기 ~20-25 패킷 추정, 이를 초과하면 드롭 발생

## 도구

- **keti-tsn-cli**: https://github.com/hrkim-KETI/keti-tsn-cli
- **Ouster SDK**: `pip3 install ouster-sdk`
