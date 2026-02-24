# LiDAR TAS 실험

Ouster OS-1-16 LiDAR + Microchip LAN9662 TSN 스위치를 사용한 IEEE 802.1Qbv (TAS) 실험.

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

## 실험 결과 요약

13개 TAS 설정 테스트 (cycle: 1ms ~ 50ms, open: 20% ~ 80%):

- **12/13 설정에서 100% 패킷 수신** — 스위치 내부 버퍼링 효과
- **패킷 손실 발생**: 50ms cycle / 20% open → **51.6% completeness**
- **최대 Burst**: 5ms / 20% open → **68.9%** (gate open 시 일괄 방출)
- **최대 Inter-Packet Gap**: 50ms / 20% open → **41ms**

## 실시간 서버

3D 포인트 클라우드 뷰어 + 실시간 TAS 제어 웹 UI:

```bash
# 의존성 설치
pip3 install flask flask-cors ouster-sdk numpy requests

# 서버 실행
python3 scripts/lidar_tas_server.py
# → http://localhost:8080
```

기능:
- Three.js 3D LiDAR 포인트 클라우드 실시간 렌더링
- LAN9662 스위치에 실시간 TAS 설정 변경 (keti-tsn-cli 연동)
- Cycle time (1/5/10/50ms) × Open % (100/80/50/20) 프리셋
- 프레임 Completeness, Jitter, Burst, PPS 실시간 모니터링
- EMA 스무딩으로 안정적인 통계 표시
- TAS Sweep 자동 테스트 (13개 설정 순차 적용)

## 파일 구조

```
├── index.html                    # 결과 시각화 (GitHub Pages)
├── data/
│   ├── sweep_results.json        # Extended sweep (1ms ~ 50ms)
│   └── sweep_1ms_results.json    # 1ms cycle sweep
├── configs/
│   ├── tas-enable.yaml           # TAS 활성화 (keti-tsn-cli용)
│   ├── tas-disable.yaml          # TAS 비활성화 (all-open)
│   └── fetch-tas.yaml            # TAS 상태 조회
├── scripts/
│   ├── lidar_tas_server.py       # 실시간 3D 뷰어 + TAS 제어 서버
│   ├── tas_sweep.py              # 1ms cycle sweep
│   ├── tas_sweep_extended.py     # Extended sweep
│   ├── measure_tas.py            # 단순 패킷 카운트
│   ├── measure_tas_detailed.py   # 상세 타이밍 측정
│   └── xdp_gate.c                # XDP eBPF gate (참고용)
└── README.md
```

## 사용법

### TAS 설정 (keti-tsn-cli)

```bash
# keti-tsn-cli 설치
cd /path/to/keti-tsn-cli-new
npm install
./keti-tsn download   # YANG catalog 다운로드 (최초 1회)

# TAS 활성화
./keti-tsn patch configs/tas-enable.yaml

# TAS 상태 조회
./keti-tsn fetch configs/fetch-tas.yaml

# TAS 비활성화 (all gates open)
./keti-tsn patch configs/tas-disable.yaml
```

### Sweep 테스트

```bash
python3 scripts/tas_sweep_extended.py
```

## 주의사항

- TAS 비활성화 시 `gate-enabled: false`를 사용하면 안 됨 — gate가 닫힌 상태로 고정됨
- 대신 `gate-enabled: true` + 단일 all-open GCL 항목 사용 (tas-disable.yaml 참고)
- LAN9662 TC0 큐 버퍼 크기 ~30-40 패킷 추정, 이를 초과하면 드롭 발생

## 도구

- **keti-tsn-cli**: https://github.com/hrkim-KETI/keti-tsn-cli
- **Ouster SDK**: `pip3 install ouster-sdk`
