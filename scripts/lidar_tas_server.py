#!/usr/bin/env python3
"""
Ouster LiDAR + LAN9662 802.1Qbv TAS — Real Hardware Test + 3D Web Viewer

Uses keti-tsn-cli to configure real TAS on LAN9662 switch.
Measures actual packet reception, jitter, burst through the switch.

Setup:
  Ouster OS-1-16 → LAN9662 Port2 → Port1 → PC (USB Ethernet)
  192.168.6.11      TAS egress P1      192.168.6.1

Usage:
  python3 lidar_tas_test.py
  → http://localhost:8080
"""

import json
import socket
import subprocess
import threading
import time
import os
import statistics
import numpy as np
import requests
from collections import deque
from flask import Flask, render_template_string, jsonify, request as flask_request
from flask_cors import CORS
import ouster.sdk.core as core

LIDAR_HOST = "192.168.6.11"
LIDAR_PORT = 7502
KETI_TSN_DIR = '/home/kim/keti-tsn-cli-new'

app = Flask(__name__)
CORS(app)

# ── Global State ──
lock = threading.Lock()
running = True
lidar_connected = False

latest_points = None
latest_frame_id = 0

# TAS state (mirrors what's on the switch)
tas_state = {
    'enabled': False,
    'cycle_us': 1000,
    'open_us': 1000,
    'close_us': 0,
    'open_pct': 100,
    'mode': 'single',
    'entries': [{'gate': 255, 'duration_us': 1000}],
}

# Stats — raw per-frame + EMA smoothed for display
current_stats = {
    'points_per_frame': 0, 'total_points': 0, 'valid_ratio': 0,
    'fps': 0, 'bandwidth_mbps': 0, 'frame_completeness': 1.0,
    'pkts_per_frame': 0, 'valid_cols': 0, 'total_cols': 2048,
    'gap_mean_us': 0, 'gap_stdev_us': 0, 'gap_max_us': 0,
    'burst_pct': 0, 'pps': 0,
}
# EMA smoothed stats for display (avoids flickering)
EMA_ALPHA = 0.15  # lower = smoother, ~7 frame window
smoothed_stats = dict(current_stats)
stats_history = deque(maxlen=300)


# ── keti-tsn-cli TAS Control ──
def _patch_tas_yaml(content):
    """Patch live TAS YAML through keti-tsn-cli."""
    yaml_path = os.path.join(KETI_TSN_DIR, 'lidar-tas', '_live_config.yaml')
    with open(yaml_path, 'w') as f:
        f.write(content)

    result = subprocess.run(
        ['./keti-tsn', 'patch', yaml_path],
        cwd=KETI_TSN_DIR, capture_output=True, text=True, timeout=30
    )
    return 'Failed' not in result.stdout and result.returncode == 0


def _normalize_entries(cycle_us, entries):
    """Normalize/validate entry list."""
    if cycle_us <= 0:
        raise ValueError("cycle_us must be > 0")
    if not entries:
        raise ValueError("entries must not be empty")

    normalized = []
    total = 0
    for e in entries:
        gate = int(e.get('gate', 255))
        dur = int(e.get('duration_us', 0))
        if gate < 0 or gate > 255:
            raise ValueError("gate must be 0..255")
        if dur < 0:
            raise ValueError("duration_us must be >= 0")
        if dur == 0:
            continue
        normalized.append({'gate': gate, 'duration_us': dur})
        total += dur

    if not normalized:
        raise ValueError("all entry durations are 0")
    if total != cycle_us:
        raise ValueError(f"sum(duration_us)={total} must equal cycle_us={cycle_us}")
    return normalized


def apply_tas_entries(cycle_us, entries):
    """Apply arbitrary TAS gate-control list in microseconds."""
    normalized = _normalize_entries(cycle_us, entries)
    lines = [
        "- ? \"/ietf-interfaces:interfaces/interface[name='1']/ieee802-dot1q-bridge:bridge-port/ieee802-dot1q-sched-bridge:gate-parameter-table\"",
        "  : gate-enabled: true",
        "    admin-gate-states: 255",
        "    admin-cycle-time:",
        f"      numerator: {cycle_us * 1000}",
        "      denominator: 1000000000",
        "    admin-base-time:",
        "      seconds: 0",
        "      nanoseconds: 0",
        "    admin-control-list:",
        "      gate-control-entry:",
    ]
    for i, e in enumerate(normalized):
        lines.extend([
            f"        - index: {i}",
            "          operation-name: set-gate-states",
            f"          gate-states-value: {e['gate']}",
            f"          time-interval-value: {e['duration_us'] * 1000}",
        ])
    lines.append("    config-change: true")
    content = "\n".join(lines) + "\n"

    ok = _patch_tas_yaml(content)
    if ok:
        open_us = sum(e['duration_us'] for e in normalized if e['gate'] == 255)
        close_us = max(0, cycle_us - open_us)
        tas_state['enabled'] = close_us > 0
        tas_state['cycle_us'] = cycle_us
        tas_state['open_us'] = open_us
        tas_state['close_us'] = close_us
        tas_state['open_pct'] = round(open_us / cycle_us * 100)
        tas_state['entries'] = normalized
        tas_state['mode'] = 'multi' if len(normalized) > 2 else 'single'
    return ok


def apply_tas(cycle_us, open_us):
    """Apply classic 2-entry TAS config to LAN9662 via keti-tsn-cli."""
    open_us = max(0, min(int(open_us), int(cycle_us)))
    close_us = int(cycle_us) - open_us
    if close_us <= 0:
        entries = [{'gate': 255, 'duration_us': int(cycle_us)}]
    else:
        entries = [
            {'gate': 255, 'duration_us': open_us},
            {'gate': 254, 'duration_us': close_us},
        ]
    return apply_tas_entries(int(cycle_us), entries)


# ── HTML Template ──
HTML_TEMPLATE = '''
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>LiDAR TAS — LAN9662 802.1Qbv</title>
    <link href="https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700;800&display=swap" rel="stylesheet">
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        :root {
            --bg: #0f1117; --card: #1a1d27; --card2: #22262f;
            --border: rgba(99,102,241,0.2);
            --text: #e2e8f0; --text2: #94a3b8; --text3: #64748b;
            --blue: #6366f1; --cyan: #06b6d4; --green: #10b981;
            --orange: #f59e0b; --red: #ef4444; --purple: #8b5cf6;
        }
        body { font-family: 'Inter', sans-serif; background: var(--bg); color: var(--text); overflow: hidden; }
        .app { display: grid; grid-template-columns: 1fr 400px; grid-template-rows: 56px 1fr; height: 100vh; }

        .header {
            grid-column: 1 / -1;
            display: flex; align-items: center; justify-content: space-between;
            padding: 0 20px; gap: 16px;
            background: var(--card); border-bottom: 1px solid var(--border);
        }
        .header h1 { font-size: 1.1rem; font-weight: 800; color: var(--blue); }
        .header-stats { display: flex; gap: 16px; }
        .hstat { text-align: center; }
        .hstat-val { font-size: 1.15rem; font-weight: 800; color: var(--cyan); font-variant-numeric: tabular-nums; }
        .hstat-label { font-size: 0.6rem; color: var(--text3); text-transform: uppercase; letter-spacing: 0.05em; }

        #canvas3d { width: 100%; height: 100%; background: #0a0b10; cursor: grab; }
        #canvas3d:active { cursor: grabbing; }

        .sidebar {
            background: var(--card); border-left: 1px solid var(--border);
            overflow-y: auto; padding: 14px;
            display: flex; flex-direction: column; gap: 10px;
        }
        .panel { background: var(--card2); border-radius: 10px; padding: 14px; border: 1px solid var(--border); }
        .panel-title { font-size: 0.72rem; font-weight: 700; color: var(--text2); text-transform: uppercase; letter-spacing: 0.05em; margin-bottom: 10px; }

        .gate-presets { display: grid; grid-template-columns: repeat(4, 1fr); gap: 5px; margin-bottom: 10px; }
        .gate-btn {
            padding: 7px 4px; border-radius: 7px; font-size: 0.72rem; font-weight: 700;
            border: 1px solid var(--border); background: var(--card); color: var(--text2);
            cursor: pointer; transition: all 0.15s; text-align: center;
        }
        .gate-btn:hover { border-color: var(--blue); color: var(--text); }
        .gate-btn.active { background: var(--blue); color: #fff; border-color: var(--blue); }
        .gate-btn.green { background: var(--green); color: #fff; border-color: var(--green); }
        .gate-btn:disabled { opacity: 0.5; cursor: wait; }

        .slider-group { margin-bottom: 6px; }
        .slider-header { display: flex; justify-content: space-between; font-size: 0.75rem; margin-bottom: 3px; }
        .slider-val { font-weight: 700; color: var(--cyan); font-variant-numeric: tabular-nums; }
        input[type=range] { width: 100%; height: 4px; -webkit-appearance: none; background: var(--border); border-radius: 2px; }
        input[type=range]::-webkit-slider-thumb { -webkit-appearance: none; width: 14px; height: 14px; background: var(--blue); border-radius: 50%; cursor: pointer; }

        .meter-bar { height: 22px; border-radius: 11px; background: var(--card); overflow: hidden; border: 1px solid var(--border); position: relative; }
        .meter-fill { height: 100%; border-radius: 11px; transition: width 0.3s, background 0.3s; }
        .meter-text { position: absolute; top: 50%; left: 50%; transform: translate(-50%,-50%); font-size: 0.72rem; font-weight: 700; color: #fff; text-shadow: 0 1px 2px rgba(0,0,0,0.5); }
        .meter-label { display: flex; justify-content: space-between; font-size: 0.68rem; color: var(--text3); margin-top: 2px; }

        .gate-viz { height: 30px; border-radius: 6px; overflow: hidden; display: flex; margin: 8px 0; border: 1px solid var(--border); }
        .gate-open-seg { background: var(--green); }
        .gate-close-seg { background: var(--red); opacity: 0.7; }

        .results-table { width: 100%; border-collapse: collapse; font-size: 0.7rem; }
        .results-table th { text-align: left; padding: 4px 5px; color: var(--text3); border-bottom: 1px solid var(--border); font-weight: 600; }
        .results-table td { padding: 4px 5px; border-bottom: 1px solid rgba(99,102,241,0.1); font-variant-numeric: tabular-nums; }
        .pass { color: var(--green); font-weight: 700; }
        .warn { color: var(--orange); font-weight: 700; }
        .fail { color: var(--red); font-weight: 700; }

        .chart-wrap { position: relative; height: 140px; }
        canvas { width: 100% !important; }

        .run-btn {
            width: 100%; padding: 9px; border-radius: 8px; font-size: 0.8rem; font-weight: 700;
            cursor: pointer; background: linear-gradient(135deg, var(--blue), var(--purple));
            color: #fff; border: none; transition: filter 0.2s;
        }
        .run-btn:hover { filter: brightness(1.15); }
        .run-btn:disabled { opacity: 0.5; cursor: not-allowed; }

        .status-dot { display: inline-block; width: 8px; height: 8px; border-radius: 50%; margin-right: 6px; }
        .status-dot.live { background: var(--green); box-shadow: 0 0 6px var(--green); animation: blink 1.5s infinite; }
        .status-dot.gated { background: var(--orange); box-shadow: 0 0 6px var(--orange); animation: blink 0.5s infinite; }
        @keyframes blink { 0%,100% { opacity: 1; } 50% { opacity: 0.4; } }

        .info-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 3px 12px; font-size: 0.72rem; }
        .info-k { color: var(--text3); }
        .info-v { color: var(--cyan); font-weight: 600; text-align: right; font-variant-numeric: tabular-nums; }

        .timing-desc {
            font-size: 0.7rem; color: var(--text2); line-height: 1.4;
            background: rgba(99,102,241,0.06); padding: 8px 10px; border-radius: 6px;
            border: 1px solid rgba(99,102,241,0.1); margin-top: 6px;
        }
        .hw-badge {
            display: inline-block; padding: 2px 7px; border-radius: 4px; font-size: 0.6rem;
            font-weight: 800; color: #fff; vertical-align: middle;
            margin-left: 6px; letter-spacing: 0.05em; background: var(--green);
        }
        .applying { animation: pulse 0.5s infinite; }
        @keyframes pulse { 0%,100%{opacity:1}50%{opacity:0.3} }
    </style>
</head>
<body>
<div class="app">
    <div class="header">
        <div style="display:flex;align-items:center;gap:12px;">
            <span class="status-dot live" id="statusDot"></span>
            <h1>LiDAR TAS <span class="hw-badge">LAN9662 HW</span></h1>
            <span style="font-size:0.72rem;color:var(--text3);">802.1Qbv Real Switch</span>
        </div>
        <div class="header-stats">
            <div class="hstat"><div class="hstat-val" id="hFps">0</div><div class="hstat-label">FPS</div></div>
            <div class="hstat"><div class="hstat-val" id="hPoints">0</div><div class="hstat-label">Points</div></div>
            <div class="hstat"><div class="hstat-val" id="hComplete">100%</div><div class="hstat-label">Complete</div></div>
            <div class="hstat"><div class="hstat-val" id="hJitter">0</div><div class="hstat-label">&micro;s Jitter</div></div>
            <div class="hstat"><div class="hstat-val" id="hBurst">0%</div><div class="hstat-label">Burst</div></div>
        </div>
    </div>

    <div><div id="canvas3d"></div></div>

    <div class="sidebar">
        <!-- TAS Gate Control -->
        <div class="panel">
            <div class="panel-title">802.1Qbv TAS Gate Control <span class="hw-badge">HW</span></div>
            <div id="applyStatus" style="display:none;font-size:0.7rem;color:var(--orange);margin-bottom:6px;" class="applying">Applying to switch...</div>

            <div style="font-size:0.72rem;color:var(--text3);margin-bottom:6px;">Cycle Time</div>
            <div class="gate-presets" id="cyclePresets">
                <button class="gate-btn active" onclick="setCycle(1000,this)">1ms</button>
                <button class="gate-btn" onclick="setCycle(781,this)">781us</button>
                <button class="gate-btn" onclick="setCycle(5000,this)">5ms</button>
                <button class="gate-btn" onclick="setCycle(10000,this)">10ms</button>
                <button class="gate-btn" onclick="setCycle(50000,this)">50ms</button>
            </div>

            <div style="font-size:0.72rem;color:var(--text3);margin-bottom:6px;">Gate Open %</div>
            <div class="gate-presets" id="openPresets">
                <button class="gate-btn green active" onclick="setOpen(100,this)">100%</button>
                <button class="gate-btn" onclick="setOpen(80,this)">80%</button>
                <button class="gate-btn" onclick="setOpen(50,this)">50%</button>
                <button class="gate-btn" onclick="setOpen(20,this)">20%</button>
            </div>

            <div class="gate-viz" id="gateViz">
                <div class="gate-open-seg" style="flex:1;"></div>
            </div>

            <div class="slider-group">
                <div class="slider-header">
                    <span>Fine Tune: Open Duration</span>
                    <span class="slider-val" id="openVal">1000 &micro;s</span>
                </div>
                <input type="range" id="openSlider" min="100" max="50000" step="100" value="1000"
                    oninput="updateSliderDisplay()">
            </div>
            <button class="run-btn" id="applyBtn" onclick="applyGate()" style="margin-top:4px;background:var(--green);">
                Apply to Switch
            </button>
            <div class="gate-presets" style="margin-top:8px;">
                <button class="gate-btn green" onclick="applyPreset781150()">781/150</button>
                <button class="gate-btn" onclick="applyPreset78130Center()">781/30 center</button>
                <button class="gate-btn" onclick="applyPreset781Block()">781/block</button>
                <button class="gate-btn" onclick="syncUIToState()">sync</button>
            </div>
            <div style="font-size:0.72rem;color:var(--text3);margin-top:8px;margin-bottom:4px;">3-slot (open-close-open)</div>
            <div class="info-grid" style="grid-template-columns: auto 1fr auto 1fr;">
                <span class="info-k">O1</span><input id="slotOpen1" type="number" min="0" step="1" value="15" style="width:100%;padding:4px 6px;border-radius:6px;border:1px solid var(--border);background:var(--card);color:var(--text);font-size:0.72rem;">
                <span class="info-k">C</span><input id="slotClose" type="number" min="0" step="1" value="751" style="width:100%;padding:4px 6px;border-radius:6px;border:1px solid var(--border);background:var(--card);color:var(--text);font-size:0.72rem;">
                <span class="info-k">O2</span><input id="slotOpen2" type="number" min="0" step="1" value="15" style="width:100%;padding:4px 6px;border-radius:6px;border:1px solid var(--border);background:var(--card);color:var(--text);font-size:0.72rem;">
                <span class="info-k">Cycle</span><input id="slotCycle" type="number" min="1" step="1" value="781" style="width:100%;padding:4px 6px;border-radius:6px;border:1px solid var(--border);background:var(--card);color:var(--text);font-size:0.72rem;">
            </div>
            <button class="run-btn" id="applyMultiBtn" onclick="applyThreeSlot()" style="margin-top:6px;background:#0ea5e9;">
                Apply 3-slot
            </button>

            <div class="timing-desc" id="timingDesc">
                Gate 100% open &mdash; all LiDAR packets pass through
            </div>
        </div>

        <!-- Frame Completeness -->
        <div class="panel">
            <div class="panel-title">Frame Completeness</div>
            <div>
                <div class="meter-bar">
                    <div class="meter-fill" id="meterFill" style="width:100%;background:var(--green);"></div>
                    <div class="meter-text" id="meterText">100%</div>
                </div>
                <div class="meter-label">
                    <span id="pktInfo">1280 pps</span>
                    <span id="colInfo">2048/2048 cols</span>
                </div>
            </div>
        </div>

        <!-- Jitter Stats -->
        <div class="panel">
            <div class="panel-title">Packet Timing</div>
            <div class="info-grid" id="timingGrid">
                <span class="info-k">Gap Mean</span><span class="info-v" id="igMean">—</span>
                <span class="info-k">Gap StdDev</span><span class="info-v" id="igSD">—</span>
                <span class="info-k">Gap Max</span><span class="info-v" id="igMax">—</span>
                <span class="info-k">Burst (<50&micro;s)</span><span class="info-v" id="igBurst">—</span>
                <span class="info-k">PPS</span><span class="info-v" id="igPPS">—</span>
            </div>
        </div>

        <!-- Chart -->
        <div class="panel">
            <div class="panel-title">Completeness & Jitter History</div>
            <div class="chart-wrap"><canvas id="liveChart"></canvas></div>
        </div>

        <!-- Auto Test -->
        <div class="panel">
            <div class="panel-title">TAS Sweep Test (HW)</div>
            <p style="font-size:0.68rem;color:var(--text3);margin-bottom:6px;">
                Configures real TAS on LAN9662 and measures LiDAR reception.<br>
                Tests: 1ms/5ms/10ms/50ms cycles × 80%/50%/20% open.
            </p>
            <button class="run-btn" id="runTestBtn" onclick="runSweep()">Run TAS Sweep</button>
            <div id="testProgress" style="font-size:0.7rem;color:var(--text2);margin-top:4px;"></div>
        </div>

        <!-- Results -->
        <div class="panel" id="resultsPanel" style="display:none;">
            <div class="panel-title">Sweep Results</div>
            <table class="results-table">
                <thead><tr><th>Config</th><th>Cmpl%</th><th>Jitter</th><th>Burst</th><th>Max Gap</th><th></th></tr></thead>
                <tbody id="resultsBody"></tbody>
            </table>
        </div>

        <!-- Sensor -->
        <div class="panel">
            <div class="panel-title">Setup</div>
            <div class="info-grid">
                <span class="info-k">LiDAR</span><span class="info-v">OS-1-16-A0</span>
                <span class="info-k">Resolution</span><span class="info-v">2048 x 16</span>
                <span class="info-k">Rate</span><span class="info-v">10 Hz / 128 pkt</span>
                <span class="info-k">Switch</span><span class="info-v">LAN9662</span>
                <span class="info-k">Firmware</span><span class="info-v">VelocitySP v2025.06</span>
                <span class="info-k">Config Tool</span><span class="info-v">keti-tsn-cli</span>
            </div>
        </div>

        <!-- View -->
        <div class="panel">
            <div class="panel-title">View</div>
            <div class="slider-group">
                <div class="slider-header"><span>Point Size</span><span class="slider-val" id="ptSizeVal">2</span></div>
                <input type="range" id="ptSize" min="1" max="8" value="2"
                    oninput="document.getElementById('ptSizeVal').textContent=this.value">
            </div>
            <select id="colorMode" style="width:100%;padding:5px 8px;border-radius:6px;border:1px solid var(--border);background:var(--card);color:var(--text);font-size:0.75rem;">
                <option value="height">Height</option>
                <option value="distance">Distance</option>
            </select>
        </div>
    </div>
</div>

<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
<script>
    let scene, camera, renderer, controls, pointCloud, chart;
    let currentCycle = 1000, currentOpenPct = 100;

    function setCycle(us, btn=null) {
        document.querySelectorAll('#cyclePresets .gate-btn').forEach(b => b.classList.remove('active'));
        if (btn) btn.classList.add('active');
        currentCycle = us;
        updateSliderRange();
        updateGateViz();
    }

    function setOpen(pct, btn) {
        document.querySelectorAll('#openPresets .gate-btn').forEach(b => { b.classList.remove('active','green'); });
        btn.classList.add('active');
        if (pct === 100) btn.classList.add('green');
        currentOpenPct = pct;
        document.getElementById('openSlider').value = Math.round(currentCycle * pct / 100);
        updateSliderDisplay();
        updateGateViz();
    }

    function updateSliderRange() {
        const slider = document.getElementById('openSlider');
        slider.max = currentCycle;
        slider.value = Math.min(slider.value, currentCycle);
        updateSliderDisplay();
    }

    function updateSliderDisplay() {
        const openUs = parseInt(document.getElementById('openSlider').value);
        const label = openUs >= 1000 ? (openUs/1000).toFixed(1)+'ms' : openUs+'&micro;s';
        document.getElementById('openVal').innerHTML = label;
        updateGateViz();
    }

    function updateGateViz() {
        const openUs = parseInt(document.getElementById('openSlider').value);
        const closeUs = currentCycle - openUs;
        const viz = document.getElementById('gateViz');
        if (closeUs <= 0) {
            viz.innerHTML = '<div class="gate-open-seg" style="flex:1;"></div>';
        } else {
            viz.innerHTML = `<div class="gate-open-seg" style="flex:${openUs};"></div><div class="gate-close-seg" style="flex:${closeUs};"></div>`;
        }
        const desc = document.getElementById('timingDesc');
        const pct = (openUs / currentCycle * 100).toFixed(0);
        if (closeUs <= 0) {
            desc.innerHTML = 'Gate 100% open &mdash; all LiDAR packets pass through';
        } else {
            const cycleLabel = currentCycle >= 1000 ? (currentCycle/1000)+'ms' : currentCycle+'&micro;s';
            desc.innerHTML = `Cycle ${cycleLabel}: Open ${pct}% (${openUs}&micro;s) / Close ${closeUs}&micro;s<br>` +
                `Switch buffers ~${(closeUs/781).toFixed(1)} pkts during close, releases on open`;
        }
    }

    async function applyMultiGate(cycleUs, entries, label) {
        const status = document.getElementById('applyStatus');
        status.style.display = 'block';
        status.className = 'applying';
        status.style.color = 'var(--orange)';
        status.textContent = 'Applying to switch...';
        try {
            const r = await fetch('/api/gate_multi', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({cycle_us: cycleUs, entries: entries})
            });
            const d = await r.json();
            if (!d.ok) throw new Error(d.error || 'unknown');
            status.className = '';
            status.style.color = 'var(--green)';
            status.textContent = `Applied: ${label} (${d.desc})`;
            setTimeout(() => { status.style.display = 'none'; }, 3000);
            syncUIToState();
        } catch (e) {
            status.className = '';
            status.style.color = 'var(--red)';
            status.textContent = 'Error: ' + e.message;
        }
    }

    function applyPreset781150() {
        setCycle(781);
        document.getElementById('openSlider').value = 150;
        updateSliderDisplay();
        applyGate();
    }

    function applyPreset78130Center() {
        const entries = [
            {gate: 255, duration_us: 15},
            {gate: 254, duration_us: 751},
            {gate: 255, duration_us: 15}
        ];
        applyMultiGate(781, entries, '781/30 center');
    }

    function applyPreset781Block() {
        const entries = [{gate: 254, duration_us: 781}];
        applyMultiGate(781, entries, '781 block');
    }

    async function applyThreeSlot() {
        const cycleUs = parseInt(document.getElementById('slotCycle').value);
        const o1 = parseInt(document.getElementById('slotOpen1').value);
        const c = parseInt(document.getElementById('slotClose').value);
        const o2 = parseInt(document.getElementById('slotOpen2').value);
        const total = o1 + c + o2;
        if (total !== cycleUs) {
            const status = document.getElementById('applyStatus');
            status.style.display = 'block';
            status.className = '';
            status.style.color = 'var(--red)';
            status.textContent = `3-slot sum mismatch: O1+C+O2=${total}, cycle=${cycleUs}`;
            return;
        }
        const entries = [
            {gate: 255, duration_us: o1},
            {gate: 254, duration_us: c},
            {gate: 255, duration_us: o2}
        ];
        await applyMultiGate(cycleUs, entries, `3-slot ${o1}/${c}/${o2}`);
    }

    async function syncUIToState() {
        try {
            const s = await (await fetch('/api/tas_state')).json();
            currentCycle = parseInt(s.cycle_us || 1000);
            const openUs = parseInt(s.open_us || currentCycle);
            document.getElementById('openSlider').max = currentCycle;
            document.getElementById('openSlider').value = Math.min(openUs, currentCycle);
            document.getElementById('slotCycle').value = currentCycle;

            if (Array.isArray(s.entries) && s.entries.length >= 3) {
                document.getElementById('slotOpen1').value = s.entries[0].duration_us || 0;
                document.getElementById('slotClose').value = s.entries[1].duration_us || 0;
                document.getElementById('slotOpen2').value = s.entries[2].duration_us || 0;
            }
            updateSliderDisplay();
        } catch (_) {}
    }

    async function applyGate() {
        const openUs = parseInt(document.getElementById('openSlider').value);
        const btn = document.getElementById('applyBtn');
        const status = document.getElementById('applyStatus');
        btn.disabled = true;
        status.style.display = 'block';
        try {
            const r = await fetch('/api/gate', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({cycle_us: currentCycle, open_us: openUs})
            });
            const d = await r.json();
            status.style.display = 'none';
            if (d.ok) {
                status.style.display = 'block';
                status.className = '';
                status.style.color = 'var(--green)';
                status.textContent = 'Applied: ' + d.desc;
                setTimeout(() => { status.style.display = 'none'; }, 3000);
            } else {
                status.style.color = 'var(--red)';
                status.textContent = 'Failed: ' + (d.error || 'unknown');
            }
        } catch(e) {
            status.style.color = 'var(--red)';
            status.textContent = 'Error: ' + e.message;
        }
        btn.disabled = false;
    }

    async function runSweep() {
        const btn = document.getElementById('runTestBtn');
        const prog = document.getElementById('testProgress');
        btn.disabled = true; prog.textContent = 'Starting...';
        try {
            const r0 = await (await fetch('/api/run_sweep', {method: 'POST'})).json();
            if (r0.status === 'error') { prog.textContent = r0.message; btn.disabled = false; return; }
            const poll = setInterval(async () => {
                const r = await (await fetch('/api/sweep_status')).json();
                if (r.running) { prog.textContent = r.progress; }
                else { clearInterval(poll); btn.disabled = false; prog.textContent = 'Done!'; showResults(r.results); }
            }, 1000);
        } catch(e) { prog.textContent = 'Error: '+e.message; btn.disabled = false; }
    }

    function showResults(results) {
        if (!results || !results.length) return;
        document.getElementById('resultsPanel').style.display = 'block';
        document.getElementById('resultsBody').innerHTML = results.map(r => {
            const cls = r.completeness >= 99.5 ? 'pass' : r.completeness >= 90 ? 'warn' : 'fail';
            const icon = r.completeness >= 99.5 ? '\\u2705' : r.completeness >= 90 ? '\\u26a0\\ufe0f' : '\\u274c';
            return `<tr>
                <td>${r.label}</td>
                <td class="${cls}">${r.completeness.toFixed(1)}%</td>
                <td>${r.jitter.toFixed(0)}&micro;s</td>
                <td>${r.burst.toFixed(1)}%</td>
                <td>${r.max_gap >= 1000 ? (r.max_gap/1000).toFixed(1)+'ms' : r.max_gap.toFixed(0)+'&micro;s'}</td>
                <td>${icon}</td>
            </tr>`;
        }).join('');
    }

    function initScene() {
        const c = document.getElementById('canvas3d');
        scene = new THREE.Scene();
        scene.background = new THREE.Color(0x0a0b10);
        camera = new THREE.PerspectiveCamera(60, c.clientWidth/c.clientHeight, 0.1, 500);
        camera.position.set(15, 12, 15);
        renderer = new THREE.WebGLRenderer({antialias: true});
        renderer.setSize(c.clientWidth, c.clientHeight);
        renderer.setPixelRatio(window.devicePixelRatio);
        c.appendChild(renderer.domElement);
        controls = new THREE.OrbitControls(camera, renderer.domElement);
        controls.enableDamping = true; controls.dampingFactor = 0.08;
        const grid = new THREE.GridHelper(40, 40, 0x6366f1, 0x1e1e2e);
        grid.material.opacity = 0.25; grid.material.transparent = true;
        scene.add(grid);
        for (let r = 5; r <= 20; r += 5) {
            const ring = new THREE.RingGeometry(r-0.03, r+0.03, 64);
            const m = new THREE.MeshBasicMaterial({color:0x6366f1, side:THREE.DoubleSide, transparent:true, opacity:0.15});
            const mesh = new THREE.Mesh(ring, m); mesh.rotation.x = Math.PI/2; scene.add(mesh);
        }
        scene.add(new THREE.AxesHelper(3));
        pointCloud = new THREE.Points(new THREE.BufferGeometry(), new THREE.PointsMaterial({size:0.08, vertexColors:true, sizeAttenuation:true}));
        scene.add(pointCloud);
        window.addEventListener('resize', () => { camera.aspect=c.clientWidth/c.clientHeight; camera.updateProjectionMatrix(); renderer.setSize(c.clientWidth,c.clientHeight); });
        (function anim() { requestAnimationFrame(anim); controls.update(); renderer.render(scene, camera); })();
    }

    function updatePC(pts) {
        if (!pts || !pts.length) return;
        const n = pts.length, pos = new Float32Array(n*3), col = new Float32Array(n*3);
        const mode = document.getElementById('colorMode').value;
        for (let i = 0; i < n; i++) {
            const [x,y,z] = pts[i];
            pos[i*3]=x; pos[i*3+1]=z; pos[i*3+2]=y;
            let h; if (mode==='distance') { h=0.65-Math.min(Math.sqrt(x*x+y*y)/25,1)*0.65; } else { h=Math.max(0,Math.min(1,(z+2)/6))*0.75; }
            const a=Math.min(0.5-Math.abs(h%1-0.5),0.5); const f=n2=>{const k=(n2+h*12)%12;return 0.55-a*Math.max(-1,Math.min(k-3,9-k,1));};
            col[i*3]=f(0); col[i*3+1]=f(8); col[i*3+2]=f(4);
        }
        pointCloud.geometry.setAttribute('position', new THREE.BufferAttribute(pos, 3));
        pointCloud.geometry.setAttribute('color', new THREE.BufferAttribute(col, 3));
        pointCloud.material.size = document.getElementById('ptSize').value * 0.04;
    }

    function initChart() {
        chart = new Chart(document.getElementById('liveChart').getContext('2d'), {
            type: 'line',
            data: { labels: [], datasets: [
                {label:'Complete%', data:[], borderColor:'#10b981', backgroundColor:'rgba(16,185,129,0.1)', fill:true, tension:0.3, pointRadius:0, borderWidth:1.5, yAxisID:'y'},
                {label:'Jitter &micro;s', data:[], borderColor:'#f59e0b', tension:0.3, pointRadius:0, borderWidth:1.5, yAxisID:'y1'}
            ]},
            options: { responsive:true, maintainAspectRatio:false, animation:{duration:0},
                scales: { x:{display:false},
                    y:{min:0, max:105, grid:{color:'rgba(255,255,255,0.05)'}, ticks:{color:'#10b981',font:{size:9},callback:v=>v+'%'}},
                    y1:{position:'right', min:0, grid:{display:false}, ticks:{color:'#f59e0b',font:{size:9},callback:v=>v}}},
                plugins:{legend:{labels:{color:'#94a3b8',font:{size:9}}}}
            }
        });
    }

    let lastFrameId = 0;

    function pollPoints() {
        fetch('/api/points?max=32768').then(r=>r.json()).then(data => {
            if (data.frame_id !== lastFrameId && data.points && data.points.length) {
                updatePC(data.points);
                lastFrameId = data.frame_id;
            }
        }).catch(()=>{}).finally(()=>setTimeout(pollPoints, 150));
    }

    function updateDisplay(s) {
        document.getElementById('hFps').textContent = (s.fps||0).toFixed(1);
        document.getElementById('hPoints').textContent = Math.round(s.points_per_frame||0).toLocaleString();
        const comp = (s.frame_completeness||1)*100;
        document.getElementById('hComplete').textContent = comp.toFixed(1)+'%';
        document.getElementById('hJitter').textContent = (s.gap_stdev_us||0).toFixed(0);
        document.getElementById('hBurst').textContent = (s.burst_pct||0).toFixed(1)+'%';
        document.getElementById('statusDot').className = 'status-dot ' + (s.gap_stdev_us > 200 ? 'gated' : 'live');

        const fill = document.getElementById('meterFill');
        fill.style.width = comp+'%';
        fill.style.background = comp >= 99 ? 'var(--green)' : comp >= 90 ? 'var(--orange)' : 'var(--red)';
        document.getElementById('meterText').textContent = comp.toFixed(1)+'%';
        document.getElementById('pktInfo').textContent = Math.round(s.pps||0)+' pps';
        document.getElementById('colInfo').textContent = Math.round(s.valid_cols||0)+'/'+(s.total_cols||2048)+' cols';

        document.getElementById('igMean').textContent = (s.gap_mean_us||0).toFixed(0)+' \\u00b5s';
        document.getElementById('igSD').textContent = (s.gap_stdev_us||0).toFixed(0)+' \\u00b5s';
        document.getElementById('igMax').textContent = (s.gap_max_us||0).toFixed(0)+' \\u00b5s';
        document.getElementById('igBurst').textContent = (s.burst_pct||0).toFixed(1)+'%';
        document.getElementById('igPPS').textContent = Math.round(s.pps||0);
    }

    function pollStats() {
        fetch('/api/stats').then(r=>r.json()).then(s => {
            updateDisplay(s);
            const now = new Date(); const ts = now.getMinutes()+':'+String(now.getSeconds()).padStart(2,'0');
            const comp = (s.frame_completeness||1)*100;
            chart.data.labels.push(ts);
            chart.data.datasets[0].data.push(comp);
            chart.data.datasets[1].data.push(s.gap_stdev_us||0);
            if (chart.data.labels.length > 120) { chart.data.labels.shift(); chart.data.datasets.forEach(d=>d.data.shift()); }
            chart.update('none');
        }).catch(()=>{}).finally(()=>setTimeout(pollStats, 250));
    }

    initScene(); initChart(); syncUIToState(); pollPoints(); pollStats();
</script>
</body>
</html>
'''


# ── LiDAR Thread ──
def lidar_thread():
    global latest_points, latest_frame_id, running, current_stats, lidar_connected

    while running:
        lidar_connected = False
        print(f"Fetching metadata from {LIDAR_HOST}...")
        try:
            meta_raw = requests.get(
                f'http://{LIDAR_HOST}/api/v1/sensor/metadata', timeout=5
            ).text
            info = core.SensorInfo(meta_raw)
            print(f"Sensor: {info.prod_line}, S/N: {info.sn}")

            pf = core.PacketFormat.from_info(info)
            batcher = core.ScanBatcher(info)
            xyzlut = core.XYZLut(info)

            W = info.format.columns_per_frame
            H = info.format.pixels_per_column
            pkt_size = pf.lidar_packet_size
            total_pkts = W // pf.columns_per_packet
            print(f"Format: {W}x{H}, {total_pkts} pkts/frame, {pkt_size}B/pkt")

            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)
            sock.bind(('0.0.0.0', LIDAR_PORT))
            sock.settimeout(2.0)
            print(f"Listening on UDP {LIDAR_PORT} — real HW TAS mode")
            lidar_connected = True

            scan = core.LidarScan(H, W, info.format.udp_profile_lidar)
            frame_times = deque(maxlen=20)
            last_time = time.time()

            # Per-frame packet timestamps for jitter analysis
            pkt_timestamps = []

            while running:
                try:
                    data, addr = sock.recvfrom(65535)
                except socket.timeout:
                    continue

                if len(data) != pkt_size:
                    continue

                pkt_timestamps.append(time.perf_counter())

                pkt_obj = core.LidarPacket(pkt_size)
                pkt_obj.buf[:] = np.frombuffer(data, dtype=np.uint8)
                done = batcher(pkt_obj, scan)

                if done:
                    xyz = xyzlut(scan)
                    status = scan.status
                    valid_cols = int(np.count_nonzero(status))

                    xyz_flat = xyz.reshape(-1, 3)
                    distances = np.linalg.norm(xyz_flat, axis=1)
                    valid = (distances > 0.3) & (distances < 100)
                    xyz_valid = xyz_flat[valid]

                    now = time.time()
                    frame_times.append(now - last_time)
                    last_time = now
                    fps = 1.0 / (sum(frame_times) / len(frame_times)) if frame_times else 0

                    completeness = valid_cols / W

                    # Jitter analysis
                    gap_mean = 0
                    gap_stdev = 0
                    gap_max = 0
                    burst_pct = 0
                    pps = 0
                    if len(pkt_timestamps) > 2:
                        gaps = [(pkt_timestamps[i+1] - pkt_timestamps[i]) * 1e6
                                for i in range(len(pkt_timestamps)-1)]
                        gap_mean = statistics.mean(gaps)
                        gap_stdev = statistics.stdev(gaps) if len(gaps) > 1 else 0
                        gap_max = max(gaps)
                        burst_pct = sum(1 for g in gaps if g < 50) / len(gaps) * 100
                        elapsed = pkt_timestamps[-1] - pkt_timestamps[0]
                        pps = len(pkt_timestamps) / elapsed if elapsed > 0 else 0

                    raw = {
                        'points_per_frame': len(xyz_valid),
                        'total_points': W * H,
                        'valid_ratio': len(xyz_valid) / (W * H),
                        'fps': fps,
                        'bandwidth_mbps': (len(pkt_timestamps) * pkt_size * 8 * fps) / 1_000_000,
                        'frame_completeness': completeness,
                        'pkts_per_frame': len(pkt_timestamps),
                        'valid_cols': valid_cols,
                        'total_cols': W,
                        'gap_mean_us': gap_mean,
                        'gap_stdev_us': gap_stdev,
                        'gap_max_us': gap_max,
                        'burst_pct': burst_pct,
                        'pps': pps,
                    }
                    current_stats = raw

                    # EMA smoothing for display
                    a = EMA_ALPHA
                    for k in ('fps', 'frame_completeness', 'gap_mean_us',
                              'gap_stdev_us', 'gap_max_us', 'burst_pct',
                              'pps', 'bandwidth_mbps', 'points_per_frame',
                              'pkts_per_frame', 'valid_cols'):
                        smoothed_stats[k] = a * raw[k] + (1 - a) * smoothed_stats.get(k, raw[k])
                    # These don't need smoothing
                    smoothed_stats['total_points'] = raw['total_points']
                    smoothed_stats['total_cols'] = raw['total_cols']
                    smoothed_stats['valid_ratio'] = raw['valid_ratio']

                    stats_history.append(dict(current_stats))

                    with lock:
                        latest_points = xyz_valid.tolist()
                        latest_frame_id += 1

                    pkt_timestamps = []
                    scan = core.LidarScan(H, W, info.format.udp_profile_lidar)

            sock.close()

        except Exception as e:
            lidar_connected = False
            print(f"LiDAR Error: {e}")
            import traceback
            traceback.print_exc()

        if running:
            print("Reconnecting in 3 seconds...")
            time.sleep(3)


# ── TAS Sweep ──
sweep_running = False
sweep_progress = ""
sweep_results = []

SWEEP_CONFIGS = [
    ("Baseline", 0, 0),
    ("1ms/80%", 1000, 800),
    ("1ms/50%", 1000, 500),
    ("1ms/20%", 1000, 200),
    ("5ms/80%", 5000, 4000),
    ("5ms/50%", 5000, 2500),
    ("5ms/20%", 5000, 1000),
    ("10ms/80%", 10000, 8000),
    ("10ms/50%", 10000, 5000),
    ("10ms/20%", 10000, 2000),
    ("50ms/80%", 50000, 40000),
    ("50ms/50%", 50000, 25000),
    ("50ms/20%", 50000, 10000),
]

def run_sweep_thread():
    global sweep_running, sweep_progress, sweep_results

    sweep_running = True
    sweep_results = []

    for i, (label, cycle_us, open_us) in enumerate(SWEEP_CONFIGS):
        sweep_progress = f"[{i+1}/{len(SWEEP_CONFIGS)}] {label}: configuring..."

        if cycle_us == 0:
            ok = apply_tas(1000, 1000)  # all-open
        else:
            ok = apply_tas(cycle_us, open_us)

        if not ok:
            sweep_progress = f"[{i+1}] {label}: config FAILED, skipping"
            time.sleep(1)
            continue

        sweep_progress = f"[{i+1}/{len(SWEEP_CONFIGS)}] {label}: settling..."
        time.sleep(3)

        sweep_progress = f"[{i+1}/{len(SWEEP_CONFIGS)}] {label}: measuring..."
        # Collect stats over 3 seconds
        completions = []
        jitters = []
        bursts = []
        max_gaps = []
        for _ in range(30):
            time.sleep(0.1)
            s = dict(current_stats)
            completions.append(s['frame_completeness'] * 100)
            jitters.append(s['gap_stdev_us'])
            bursts.append(s['burst_pct'])
            max_gaps.append(s['gap_max_us'])

        sweep_results.append({
            'label': label,
            'cycle_us': cycle_us,
            'open_us': open_us,
            'completeness': statistics.mean(completions) if completions else 0,
            'jitter': statistics.mean(jitters) if jitters else 0,
            'burst': statistics.mean(bursts) if bursts else 0,
            'max_gap': max(max_gaps) if max_gaps else 0,
        })
        print(f"[Sweep] {label}: complete={statistics.mean(completions):.1f}% jitter={statistics.mean(jitters):.0f}")

    # Restore baseline
    sweep_progress = "Restoring baseline..."
    apply_tas(1000, 1000)
    sweep_running = False
    sweep_progress = "Done!"
    print("[Sweep] Complete!")


# ── Routes ──
@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/api/points')
def get_points():
    max_pts = int(flask_request.args.get('max', 32768))
    with lock:
        if latest_points is None:
            return jsonify({'points': [], 'frame_id': 0, 'stats': smoothed_stats})
        pts = latest_points
        fid = latest_frame_id
    if len(pts) > max_pts:
        step = max(1, len(pts) // max_pts)
        pts = pts[::step][:max_pts]
    return jsonify({'points': pts, 'frame_id': fid, 'stats': smoothed_stats})

@app.route('/api/stats')
def get_stats():
    return jsonify(smoothed_stats)

@app.route('/api/stats_raw')
def get_stats_raw():
    return jsonify(current_stats)

@app.route('/api/gate', methods=['POST'])
def set_gate():
    d = flask_request.json or {}
    cycle_us = max(1, int(d.get('cycle_us', 1000)))
    open_us = int(d.get('open_us', cycle_us))
    open_us = max(0, min(open_us, cycle_us))
    close_us = cycle_us - open_us

    print(f"[API] Applying TAS: cycle={cycle_us}us, open={open_us}us, close={close_us}us")
    ok = apply_tas(cycle_us, open_us)

    if ok:
        desc = f"cycle={cycle_us}us, open={open_us}us ({open_us*100//cycle_us}%)"
        return jsonify({'ok': True, 'desc': desc, **tas_state})
    else:
        return jsonify({'ok': False, 'error': 'keti-tsn-cli patch failed'})


@app.route('/api/gate_multi', methods=['POST'])
def set_gate_multi():
    d = flask_request.json or {}
    cycle_us = max(1, int(d.get('cycle_us', 1000)))
    entries = d.get('entries', [])
    if not isinstance(entries, list):
        return jsonify({'ok': False, 'error': 'entries must be a list'})

    try:
        normalized = _normalize_entries(cycle_us, entries)
    except ValueError as e:
        return jsonify({'ok': False, 'error': str(e)})

    print(f"[API] Applying TAS multi: cycle={cycle_us}us, entries={normalized}")
    ok = apply_tas_entries(cycle_us, normalized)
    if not ok:
        return jsonify({'ok': False, 'error': 'keti-tsn-cli patch failed'})

    open_us = sum(e['duration_us'] for e in normalized if e['gate'] == 255)
    desc = f"cycle={cycle_us}us, open={open_us}us ({open_us*100//cycle_us}%), entries={len(normalized)}"
    return jsonify({'ok': True, 'desc': desc, **tas_state})

@app.route('/api/tas_state')
def get_tas_state():
    return jsonify(tas_state)

@app.route('/api/run_sweep', methods=['POST'])
def api_run_sweep():
    if sweep_running:
        return jsonify({'status': 'already_running'})
    if not lidar_connected:
        return jsonify({'status': 'error', 'message': 'LiDAR not connected'})
    threading.Thread(target=run_sweep_thread, daemon=True).start()
    return jsonify({'status': 'running'})

@app.route('/api/sweep_status')
def api_sweep_status():
    return jsonify({'running': sweep_running, 'progress': sweep_progress, 'results': sweep_results})


def main():
    global running
    print("=" * 60)
    print("  LiDAR TAS — LAN9662 802.1Qbv Real Hardware")
    print("  keti-tsn-cli for switch configuration")
    print("=" * 60)
    print(f"  LiDAR:   OS-1-16-A0 @ {LIDAR_HOST}")
    print(f"  Switch:  LAN9662 via /dev/ttyACM0")
    print(f"  Mode:    Real HW TAS (not simulation)")
    print(f"  Web UI:  http://localhost:8080")
    print("=" * 60)

    # Ensure all gates open at start
    print("Setting TAS to all-open...")
    apply_tas(1000, 1000)

    threading.Thread(target=lidar_thread, daemon=True).start()

    try:
        app.run(host='0.0.0.0', port=8080, debug=False, threaded=True)
    except KeyboardInterrupt:
        running = False
        print("\nRestoring TAS baseline...")
        apply_tas(1000, 1000)
        print("Shutdown.")


if __name__ == '__main__':
    main()
