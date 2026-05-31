#!/usr/bin/env python3
"""Local web console for controlling and observing the tennis robot simulation."""

from __future__ import annotations

import argparse
import base64
import json
import math
import sys
import threading
import time
from collections import Counter
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "ball_detector"))

from control_bus import RobotCommandStore, RobotSensorStore, RobotStatusStore, SUPPORTED_MODES  # noqa: E402

try:
    import cv2
    from perception import detect_largest_ball, TENNIS_BALL_DIAMETER_M
    _VISION_AVAILABLE = True
except ImportError:
    _VISION_AVAILABLE = False

WEBCAM_FOV_DEG = 60.0  # typical webcam horizontal FOV; tune if distance estimates are off


class WebcamManager:
    _cap: object = None
    _lock = threading.Lock()

    @classmethod
    def get_frame(cls) -> tuple[bool, object]:
        if not _VISION_AVAILABLE:
            return False, None
        with cls._lock:
            if cls._cap is None or not cls._cap.isOpened():
                cls._cap = cv2.VideoCapture(0)
            if not cls._cap.isOpened():
                return False, None
            return cls._cap.read()

    @classmethod
    def release(cls) -> None:
        with cls._lock:
            if cls._cap is not None:
                cls._cap.release()
                cls._cap = None


HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Tennis Robot Console</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #0c1116;
      --panel: #111922;
      --panel-2: #151f2a;
      --line: #273442;
      --ink: #eef4f8;
      --muted: #91a2b2;
      --accent: #2fd08f;
      --accent-2: #57a6ff;
      --warn: #ffbd5a;
      --danger: #ff6b5f;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      background: var(--bg);
      color: var(--ink);
      font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      letter-spacing: 0;
    }
    .shell {
      min-height: 100vh;
      display: grid;
      grid-template-columns: 248px minmax(0, 1fr);
    }
    aside {
      border-right: 1px solid var(--line);
      background: #0f161d;
      padding: 22px 18px;
      position: sticky;
      top: 0;
      height: 100vh;
    }
    .brand {
      margin-bottom: 28px;
    }
    .brand h1 {
      margin: 0;
      font-size: 22px;
      line-height: 1.1;
    }
    .brand p {
      margin: 8px 0 0;
      color: var(--muted);
      font-size: 13px;
      line-height: 1.4;
    }
    nav {
      display: grid;
      gap: 6px;
    }
    nav button {
      border: 0;
      width: 100%;
      text-align: left;
      color: var(--muted);
      background: transparent;
      padding: 11px 12px;
      border-radius: 7px;
      font: inherit;
      cursor: pointer;
      transition: background 140ms ease, color 140ms ease;
    }
    nav button:hover,
    nav button.active {
      background: var(--panel-2);
      color: var(--ink);
    }
    .connection {
      position: absolute;
      left: 18px;
      right: 18px;
      bottom: 18px;
      border-top: 1px solid var(--line);
      padding-top: 16px;
      color: var(--muted);
      font-size: 13px;
      line-height: 1.5;
    }
    .dot {
      display: inline-block;
      width: 8px;
      height: 8px;
      border-radius: 50%;
      margin-right: 8px;
      background: var(--danger);
      box-shadow: 0 0 0 4px rgba(255, 107, 95, 0.12);
    }
    .dot.live {
      background: var(--accent);
      box-shadow: 0 0 0 4px rgba(47, 208, 143, 0.12);
    }
    main {
      padding: 28px;
      min-width: 0;
    }
    header {
      display: flex;
      justify-content: space-between;
      gap: 20px;
      align-items: flex-start;
      border-bottom: 1px solid var(--line);
      padding-bottom: 22px;
      margin-bottom: 24px;
    }
    header h2 {
      margin: 0;
      font-size: 28px;
      line-height: 1.1;
    }
    header p {
      margin: 8px 0 0;
      color: var(--muted);
      line-height: 1.5;
      max-width: 760px;
    }
    .timestamp {
      color: var(--muted);
      font-size: 13px;
      white-space: nowrap;
    }
    section.view {
      display: none;
      animation: rise 180ms ease;
    }
    section.view.active {
      display: block;
    }
    @keyframes rise {
      from { opacity: 0; transform: translateY(6px); }
      to { opacity: 1; transform: translateY(0); }
    }
    .grid {
      display: grid;
      gap: 14px;
    }
    .kpis {
      grid-template-columns: repeat(4, minmax(150px, 1fr));
      margin-bottom: 22px;
    }
    .metric,
    .panel {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
    }
    .metric {
      padding: 16px;
      min-height: 96px;
    }
    .metric span {
      display: block;
      color: var(--muted);
      font-size: 12px;
      text-transform: uppercase;
      margin-bottom: 10px;
    }
    .metric strong {
      display: block;
      font-size: 26px;
      line-height: 1;
      word-break: break-word;
    }
    .metric small {
      display: block;
      margin-top: 9px;
      color: var(--muted);
      font-size: 12px;
    }
    .two {
      grid-template-columns: minmax(0, 1.1fr) minmax(320px, 0.9fr);
    }
    .panel {
      padding: 18px;
      overflow: hidden;
    }
    .panel h3 {
      margin: 0 0 14px;
      font-size: 15px;
      color: var(--ink);
    }
    .controls {
      display: flex;
      gap: 10px;
      flex-wrap: wrap;
    }
    .command {
      border: 0;
      border-radius: 7px;
      color: #07110d;
      background: var(--accent);
      padding: 12px 16px;
      font-weight: 800;
      cursor: pointer;
      transition: transform 130ms ease, filter 130ms ease;
    }
    .command:hover { transform: translateY(-1px); filter: brightness(1.05); }
    .command[value="collect_pattern"] { background: #2fd08f; color: #06130d; }
    .command[value="collect_one"] { background: var(--warn); color: #1b1204; }
    .command[value="survey"] { background: var(--accent-2); color: #06101d; }
    .command[value="scan_side"] { background: #1acdcd; color: #051717; }
    .command[value="idle"] { background: var(--danger); color: #1b0604; }
    .command[value="map_left_side"] { background: #a855f7; color: #0b0514; }
    .map-cell {
      background: var(--panel-2);
      border: 1px solid var(--line);
      border-radius: 6px;
      padding: 14px 8px;
      font-size: 22px;
      font-weight: 700;
      font-variant-numeric: tabular-nums;
      min-height: 56px;
      display: flex;
      align-items: center;
      justify-content: center;
      transition: background 0.35s, color 0.35s;
    }
    .kv {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 10px 16px;
      font-size: 14px;
    }
    .kv div {
      border-top: 1px solid var(--line);
      padding-top: 10px;
      min-width: 0;
    }
    .kv span {
      display: block;
      color: var(--muted);
      font-size: 12px;
      margin-bottom: 4px;
    }
    .kv strong {
      font-weight: 650;
      word-break: break-word;
    }
    table {
      width: 100%;
      border-collapse: collapse;
      font-size: 13px;
    }
    th, td {
      text-align: left;
      padding: 11px 10px;
      border-bottom: 1px solid var(--line);
      vertical-align: top;
    }
    th {
      color: var(--muted);
      font-weight: 600;
      font-size: 12px;
      text-transform: uppercase;
    }
    tr.latest td {
      color: var(--accent);
    }
    .log {
      display: grid;
      gap: 8px;
      max-height: 520px;
      overflow: auto;
    }
    .event {
      display: grid;
      grid-template-columns: 92px 96px minmax(0, 1fr);
      gap: 10px;
      padding: 10px 0;
      border-bottom: 1px solid var(--line);
      color: var(--muted);
      font-size: 13px;
    }
    .event strong { color: var(--ink); }
    .json {
      margin: 0;
      overflow: auto;
      max-height: 560px;
      color: #b7c6d5;
      font-size: 12px;
      line-height: 1.55;
      background: #090d12;
      border-radius: 7px;
      padding: 14px;
    }
    .map-panel {
      margin-top: 14px;
    }
    .court-map {
      width: 100%;
      aspect-ratio: 2 / 1;
      display: block;
      background: #7a3329;
      border: 1px solid var(--line);
      border-radius: 8px;
    }
    .legend {
      display: flex;
      flex-wrap: wrap;
      gap: 14px;
      margin-top: 12px;
      color: var(--muted);
      font-size: 12px;
    }
    .legend span::before {
      content: "";
      display: inline-block;
      width: 9px;
      height: 9px;
      border-radius: 50%;
      margin-right: 6px;
      background: var(--accent);
    }
    .legend .across::before { background: #8793a0; }
    .legend .pending::before { background: #d7e85f; opacity: 0.4; }
    .legend .depth::before { background: #2fd08f; box-shadow: 0 0 0 2px rgba(87,166,255,0.5); }
    .legend .route::before { border-radius: 2px; background: var(--accent-2); }
    .legend .robot::before { background: var(--warn); }
    .legend .fov::before { background: rgba(87,166,255,0.15); border: 1px solid rgba(87,166,255,0.4); border-radius: 2px; }
    .sensor-grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 14px;
      margin-top: 14px;
    }
    .sensor-view {
      background: #090d12;
      border: 1px solid var(--line);
      border-radius: 8px;
      overflow: hidden;
      min-height: 180px;
    }
    .sensor-view h4 {
      margin: 0;
      padding: 10px 12px;
      border-bottom: 1px solid var(--line);
      color: var(--muted);
      font-size: 12px;
      text-transform: uppercase;
    }
    .sensor-view img {
      display: block;
      width: 100%;
      aspect-ratio: 16 / 9;
      object-fit: contain;
      background: #05080b;
    }
    .sensor-view canvas {
      display: block;
      width: 100%;
      aspect-ratio: 1 / 1;
      background: #05080b;
    }
    .sensor-meta {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 8px;
      padding: 10px 12px 12px;
      border-top: 1px solid var(--line);
      color: var(--muted);
      font-size: 12px;
    }
    .sensor-meta strong {
      display: block;
      margin-top: 3px;
      color: var(--ink);
      font-size: 13px;
      font-weight: 650;
    }
    .sensor-empty {
      display: grid;
      place-items: center;
      aspect-ratio: 16 / 9;
      color: var(--muted);
      font-size: 13px;
    }
    @media (max-width: 900px) {
      .shell { grid-template-columns: 1fr; }
      aside { position: static; height: auto; }
      .connection { position: static; margin-top: 18px; }
      main { padding: 20px; }
      header { display: block; }
      .timestamp { margin-top: 12px; }
      .kpis, .two, .sensor-grid { grid-template-columns: 1fr; }
      .event { grid-template-columns: 1fr; }
    }
  </style>
</head>
<body>
  <div class="shell">
    <aside>
      <div class="brand">
        <h1>Tennis Robot Console</h1>
        <p>Remote control and live diagnostics for the Webots controller.</p>
      </div>
      <nav aria-label="Console sections">
        <button class="active" data-view="dashboard">Dashboard</button>
        <button data-view="control">Control</button>
        <button data-view="sensors">Sensor Views</button>
        <button data-view="telemetry">Telemetry</button>
        <button data-view="stats">Command Stats</button>
        <button data-view="history">History</button>
        <button data-view="webcam">Webcam</button>
      </nav>
      <div class="connection">
        <div><span id="liveDot" class="dot"></span><span id="connectionText">Waiting for robot status</span></div>
        <div id="commandFile">Command file: runtime/robot_command.json</div>
      </div>
    </aside>
    <main>
      <header>
        <div>
          <h2 id="viewTitle">Dashboard</h2>
          <p id="viewHelp">Observe the robot mode, collector state, current target, and command stream while the simulation runs.</p>
        </div>
        <div id="lastRefresh" class="timestamp">Not refreshed yet</div>
      </header>

      <section id="dashboard" class="view active">
        <div class="grid kpis">
          <div class="metric"><span>Requested Mode</span><strong id="kRequested">idle</strong><small id="kSource">source default</small></div>
          <div class="metric"><span>Actual State</span><strong id="kState">idle</strong><small id="kActual">mode idle</small></div>
          <div class="metric"><span>Balls Collected</span><strong id="kBalls">0</strong><small id="kUptime">uptime 0.0s</small></div>
          <div class="metric"><span>Ball Detection</span><strong id="kDetection">hidden</strong><small id="kDistance">distance none</small></div>
        </div>
        <div class="grid two">
          <div class="panel">
            <h3>Robot Snapshot</h3>
            <div id="snapshot" class="kv"></div>
          </div>
          <div class="panel">
            <h3>Latest Commands</h3>
            <div id="latestEvents" class="log"></div>
          </div>
        </div>
      </section>

      <section id="control" class="view">
        <div class="grid two">
          <div class="panel">
            <h3>Mode Command</h3>
            <form id="commandForm" class="controls">
              <button class="command" type="submit" name="mode" value="collect">Start Collection</button>
              <button class="command" type="submit" name="mode" value="collect_pattern">Collect Pattern</button>
              <button class="command" type="submit" name="mode" value="search">Search Pattern</button>
              <button class="command" type="submit" name="mode" value="collect_one">Collect One</button>
              <button class="command" type="submit" name="mode" value="scan_side">Scan This Side</button>
              <button class="command" type="submit" name="mode" value="map_left_side">Map Left Side</button>
              <button class="command" type="submit" name="mode" value="survey">Survey Court</button>
              <button class="command" type="submit" name="mode" value="idle">Stop</button>
            </form>
          </div>
          <div class="panel">
            <h3>Selected Mode</h3>
            <div id="selectedMode" class="kv"></div>
          </div>
        </div>
        <div class="panel map-panel">
          <h3>Collection Map</h3>
          <canvas id="courtMap" class="court-map" width="1200" height="600"></canvas>
          <div class="legend">
            <span>Confirmed ball</span>
            <span class="depth">OAK depth</span>
            <span class="pending">Pending (building confidence)</span>
            <span class="across">Across net</span>
            <span class="route">Planned route</span>
            <span class="robot">Robot</span>
            <span class="fov">Camera FOV</span>
          </div>
        </div>
      </section>

      <section id="sensors" class="view">
        <div class="panel">
          <h3>360° LiDAR Ground Scan</h3>
          <p style="color:var(--muted);font-size:13px;margin:0 0 14px;">Real-time RPLIDAR C1 scan — 500 samples/rev, 12 m range. Use <strong>Scan This Side</strong> to capture a stationary snapshot.</p>
          <div id="lidarScanView" class="sensor-empty" style="background:#090d12;border:1px solid var(--line);border-radius:8px;min-height:420px;">waiting for LiDAR scan</div>
          <div id="lidarScanMeta" style="display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:8px;padding:12px 0 4px;color:var(--muted);font-size:12px;"></div>
          <div id="scanSideProgress" style="display:none;margin-top:12px;padding:12px 14px;background:rgba(26,205,205,0.08);border:1px solid rgba(26,205,205,0.28);border-radius:7px;font-size:13px;color:#1acdcd;">
            <strong>Scan in progress</strong> — <span id="scanSideElapsed">0.0</span>s / <span id="scanSideDuration">12</span>s
            <div style="margin-top:6px;height:4px;background:rgba(26,205,205,0.18);border-radius:2px;overflow:hidden;">
              <div id="scanSideBar" style="height:100%;background:#1acdcd;width:0%;transition:width 0.3s;border-radius:2px;"></div>
            </div>
          </div>
        </div>
        <div class="grid two" style="margin-top:14px;">
          <div class="panel">
            <h3>Front Camera</h3>
            <div id="sensorsCameraView" class="sensor-empty">waiting for image</div>
          </div>
          <div class="panel">
            <h3>OAK-D Depth</h3>
            <div id="sensorsDepthView" class="sensor-empty">waiting for depth</div>
          </div>
        </div>
        <div class="panel" style="margin-top:14px;">
          <h3>IR Intake Sensors</h3>
          <p style="color:var(--muted);font-size:13px;margin:0 0 14px;">Collection trigger fires when either sensor exceeds threshold. Value 1000 = object detected, 0 = clear.</p>
          <div id="irIntakeView" style="display:grid;grid-template-columns:1fr 1fr;gap:16px;">
            <div id="irLeftPanel" style="padding:12px;border-radius:8px;border:1px solid var(--line);">
              <div style="font-size:12px;color:var(--muted);margin-bottom:6px;">LEFT</div>
              <div id="irLeftValue" style="font-size:24px;font-weight:700;font-variant-numeric:tabular-nums;">—</div>
              <div style="margin-top:8px;height:8px;background:rgba(255,255,255,0.07);border-radius:4px;overflow:hidden;">
                <div id="irLeftBar" style="height:100%;width:0%;border-radius:4px;transition:width 0.15s,background 0.15s;"></div>
              </div>
            </div>
            <div id="irRightPanel" style="padding:12px;border-radius:8px;border:1px solid var(--line);">
              <div style="font-size:12px;color:var(--muted);margin-bottom:6px;">RIGHT</div>
              <div id="irRightValue" style="font-size:24px;font-weight:700;font-variant-numeric:tabular-nums;">—</div>
              <div style="margin-top:8px;height:8px;background:rgba(255,255,255,0.07);border-radius:4px;overflow:hidden;">
                <div id="irRightBar" style="height:100%;width:0%;border-radius:4px;transition:width 0.15s,background 0.15s;"></div>
              </div>
            </div>
          </div>
          <div id="irTriggeredBadge" style="margin-top:12px;padding:8px 14px;border-radius:6px;font-size:13px;font-weight:600;text-align:center;background:rgba(255,255,255,0.04);color:var(--muted);">
            TRIGGERED: —
          </div>
        </div>
        <div id="mapMissionPanel" class="panel" style="margin-top:14px;">
          <h3>Half-Court Mapping Grid &nbsp;<span id="mapMissionStatus" style="font-size:13px;font-weight:400;color:var(--muted);">idle</span></h3>
          <div style="display:grid;grid-template-columns:88px repeat(3,1fr);gap:6px;margin-top:14px;font-size:13px;text-align:center;align-items:center;">
            <div></div>
            <div style="color:var(--muted);font-size:11px;text-transform:uppercase;letter-spacing:.04em;">Αριστερά</div>
            <div style="color:var(--muted);font-size:11px;text-transform:uppercase;letter-spacing:.04em;">Κέντρο</div>
            <div style="color:var(--muted);font-size:11px;text-transform:uppercase;letter-spacing:.04em;">Δεξιά</div>
            <div style="color:var(--muted);font-size:11px;text-align:right;padding-right:10px;">Φράχτης</div>
            <div id="mapCell00" class="map-cell">—</div><div id="mapCell01" class="map-cell">—</div><div id="mapCell02" class="map-cell">—</div>
            <div style="color:var(--muted);font-size:11px;text-align:right;padding-right:10px;">Μέση</div>
            <div id="mapCell10" class="map-cell">—</div><div id="mapCell11" class="map-cell">—</div><div id="mapCell12" class="map-cell">—</div>
            <div style="color:var(--muted);font-size:11px;text-align:right;padding-right:10px;">Φιλέ</div>
            <div id="mapCell20" class="map-cell">—</div><div id="mapCell21" class="map-cell">—</div><div id="mapCell22" class="map-cell">—</div>
          </div>
          <div id="mapMissionProgress" style="display:none;margin-top:12px;">
            <div style="display:flex;justify-content:space-between;font-size:12px;color:var(--muted);margin-bottom:5px;">
              <span id="mapMissionPhaseLabel"></span><span id="mapMissionPosesLabel"></span>
            </div>
            <div style="height:4px;background:rgba(255,255,255,0.08);border-radius:2px;overflow:hidden;">
              <div id="mapMissionBar" style="height:100%;width:0%;background:#a855f7;border-radius:2px;transition:width 0.5s;"></div>
            </div>
          </div>
          <div id="mapMissionTotals" style="margin-top:10px;color:var(--muted);font-size:12px;"></div>
        </div>
      </section>

      <section id="telemetry" class="view">
        <div class="grid two">
          <div class="panel">
            <h3>Live Telemetry</h3>
            <div id="telemetryKv" class="kv"></div>
          </div>
          <div class="panel">
            <h3>Raw Status</h3>
            <pre id="rawStatus" class="json">{}</pre>
          </div>
        </div>
      </section>

      <section id="stats" class="view">
        <div class="grid kpis">
          <div class="metric"><span>Total Commands</span><strong id="sTotal">0</strong><small>from local history</small></div>
          <div class="metric"><span>Collect Commands</span><strong id="sCollect">0</strong><small>collect / pattern / one</small></div>
          <div class="metric"><span>Survey Commands</span><strong id="sSurvey">0</strong><small>requested mode survey</small></div>
          <div class="metric"><span>Stop Commands</span><strong id="sIdle">0</strong><small>requested mode idle</small></div>
        </div>
        <div class="panel">
          <h3>Per Command Statistics</h3>
          <table>
            <thead><tr><th>Mode</th><th>Count</th><th>Share</th><th>Last Sequence</th><th>Last Source</th></tr></thead>
            <tbody id="statsRows"></tbody>
          </table>
        </div>
      </section>

      <section id="history" class="view">
        <div class="panel">
          <h3>Command History</h3>
          <table>
            <thead><tr><th>Time</th><th>Mode</th><th>Sequence</th><th>Source</th></tr></thead>
            <tbody id="historyRows"></tbody>
          </table>
        </div>
      </section>

      <section id="webcam" class="view">
        <div class="grid kpis" style="grid-template-columns: repeat(3, minmax(150px, 1fr)); margin-bottom: 18px;">
          <div class="metric"><span>Distance</span><strong id="wcDistance">—</strong><small>monocular estimate</small></div>
          <div class="metric"><span>Bearing</span><strong id="wcBearing">—</strong><small>horizontal angle</small></div>
          <div class="metric"><span>Diameter</span><strong id="wcDiameter">—</strong><small>apparent pixels</small></div>
        </div>
        <div class="panel">
          <h3>Webcam Feed <span id="wcStatus" style="font-weight:400;color:var(--muted);font-size:13px;">— initializing</span></h3>
          <div id="wcFeedWrap" style="position:relative;background:#05080b;border-radius:6px;overflow:hidden;min-height:240px;display:flex;align-items:center;justify-content:center;">
            <img id="wcFrame" style="display:none;max-width:100%;border-radius:6px;" alt="webcam feed">
            <div id="wcEmpty" style="color:var(--muted);font-size:13px;">waiting for webcam&hellip;</div>
          </div>
          <p style="margin:12px 0 0;color:var(--muted);font-size:12px;">HSV range: H 25–72 (yellow-green). Real tennis balls (H&nbsp;25–40) are in range. Distance uses monocular focal-length formula — assumes <strong style="color:var(--ink);">60° horizontal FOV</strong>; adjust <code>WEBCAM_FOV_DEG</code> in <code>scripts/control_panel.py</code> to calibrate.</p>
        </div>
      </section>
    </main>
  </div>

  <script>
    const titles = {
      dashboard: ["Dashboard", "Observe the robot mode, collector state, current target, and command stream while the simulation runs."],
      control: ["Control", "Send high-level commands to the running Webots controller."],
      sensors: ["Sensor Views", "Live RPLIDAR C1 360° ground scan, front camera, OAK-D depth image, and half-court mapping grid. Press Map Left Side to start a mapping mission; the 3×3 grid updates live after each scan pose."],
      telemetry: ["Telemetry", "Inspect live robot pose, detection, command output, survey data, and raw status."],
      stats: ["Command Stats", "Review per-mode command counts and recent command usage."],
      history: ["History", "Audit the local command stream written by this console and controller startup."],
      webcam: ["Webcam", "Live webcam feed with HSV tennis ball detection and monocular distance estimation. No Webots needed."]
    };
    let diagnostics = { command: {}, robot: {}, history: [], stats: {} };
    let sensors = {};

    function fmt(value, suffix = "") {
      if (value === null || value === undefined || Number.isNaN(value)) return "none";
      if (typeof value === "number") return `${value.toFixed(Math.abs(value) >= 10 ? 1 : 2)}${suffix}`;
      return String(value);
    }
    function timeText(seconds) {
      if (!seconds) return "0.0s";
      if (seconds < 90) return `${seconds.toFixed(1)}s`;
      return `${Math.floor(seconds / 60)}m ${Math.floor(seconds % 60)}s`;
    }
    function dateText(ts) {
      if (!ts) return "none";
      return new Date(ts * 1000).toLocaleTimeString();
    }
    function setKv(id, rows) {
      document.getElementById(id).innerHTML = rows.map(([label, value]) => (
        `<div><span>${label}</span><strong>${value}</strong></div>`
      )).join("");
    }
    function setView(name) {
      document.querySelectorAll("nav button").forEach(btn => btn.classList.toggle("active", btn.dataset.view === name));
      document.querySelectorAll("section.view").forEach(view => view.classList.toggle("active", view.id === name));
      document.getElementById("viewTitle").textContent = titles[name][0];
      document.getElementById("viewHelp").textContent = titles[name][1];
      if (name === "webcam") startWebcam(); else stopWebcam();
    }
    document.querySelectorAll("nav button").forEach(btn => btn.addEventListener("click", () => setView(btn.dataset.view)));

    let _wcInterval = null;
    function startWebcam() {
      if (_wcInterval) return;
      refreshWebcam();
      _wcInterval = setInterval(refreshWebcam, 125);
    }
    function stopWebcam() {
      if (_wcInterval) { clearInterval(_wcInterval); _wcInterval = null; }
    }
    async function refreshWebcam() {
      try {
        const res = await fetch("/api/webcam/frame", { cache: "no-store" });
        renderWebcam(await res.json());
      } catch (_) {}
    }
    function renderWebcam(data) {
      const img = document.getElementById("wcFrame");
      const empty = document.getElementById("wcEmpty");
      const status = document.getElementById("wcStatus");
      if (!data.available) {
        img.style.display = "none";
        empty.style.display = "";
        empty.textContent = data.error || "webcam unavailable";
        status.textContent = "— unavailable";
        status.style.color = "var(--danger)";
        return;
      }
      img.src = data.data_url;
      img.style.display = "block";
      empty.style.display = "none";
      if (data.detected) {
        status.textContent = "— ball detected";
        status.style.color = "var(--accent)";
        document.getElementById("wcDistance").textContent = data.distance_m != null ? `${data.distance_m.toFixed(2)} m` : "—";
        const b = data.bearing_deg;
        document.getElementById("wcBearing").textContent = b != null ? `${b >= 0 ? "+" : ""}${b.toFixed(1)}°` : "—";
        document.getElementById("wcDiameter").textContent = data.diameter_px != null ? `${Math.round(data.diameter_px)} px` : "—";
      } else {
        status.textContent = "— no ball";
        status.style.color = "var(--muted)";
        ["wcDistance", "wcBearing", "wcDiameter"].forEach(id => { document.getElementById(id).textContent = "—"; });
      }
    }

    document.getElementById("commandForm").addEventListener("submit", async event => {
      event.preventDefault();
      const mode = event.submitter.value;
      await fetch("/api/command", {
        method: "POST",
        headers: { "Content-Type": "application/x-www-form-urlencoded" },
        body: new URLSearchParams({ mode })
      });
      if (mode === "scan_side" || mode === "map_left_side") setView("sensors");
      await refresh();
    });

    async function refresh() {
      const response = await fetch("/api/diagnostics", { cache: "no-store" });
      diagnostics = await response.json();
      const sensorResponse = await fetch("/api/sensors", { cache: "no-store" });
      sensors = await sensorResponse.json();
      render();
    }
    function render() {
      const command = diagnostics.command || {};
      const robot = diagnostics.robot || {};
      const obs = robot.observation || {};
      const out = robot.command || {};
      const pose = robot.robot || {};
      const survey = robot.survey || {};
      const search = robot.search || {};
      const mounts = robot.sensor_mounts || {};
      const oakDepth = robot.oak_depth || {};
      const scan = robot.scan || {};
      const collectOne = robot.collect_one || {};
      const collectPattern = robot.collect_pattern || {};
      const balls = robot.balls || {};
      const completion = robot.completion || {};
      const connected = !!robot.connected;

      document.getElementById("liveDot").classList.toggle("live", connected);
      document.getElementById("connectionText").textContent = connected ? "Robot status live" : `Robot status stale (${fmt(robot.age_s, "s")})`;
      document.getElementById("lastRefresh").textContent = `Refreshed ${new Date().toLocaleTimeString()}`;
      document.getElementById("commandFile").textContent = `Sequence ${command.sequence ?? 0} from ${command.source ?? "default"}`;

      document.getElementById("kRequested").textContent = command.mode || "idle";
      document.getElementById("kSource").textContent = `source ${command.source || "default"}`;
      document.getElementById("kState").textContent = robot.collector_state || "idle";
      document.getElementById("kActual").textContent = `mode ${robot.actual_mode || "idle"}`;
      document.getElementById("kBalls").textContent = robot.balls_collected ?? 0;
      document.getElementById("kUptime").textContent = `remaining ${balls.same_side_remaining ?? "?"} same-side`;
      document.getElementById("kDetection").textContent = obs.visible ? "visible" : "hidden";
      document.getElementById("kDistance").textContent = `OAK-D Depth ${fmt(oakDepth.range_m ?? obs.distance_m, "m")} bearing ${fmt(obs.bearing_deg, "deg")}`;

      setKv("snapshot", [
        ["Robot position", `${fmt(pose.x_m, "m")}, ${fmt(pose.y_m, "m")}`],
        ["Robot yaw", fmt((pose.yaw_rad || 0) * 180 / Math.PI, "deg")],
        ["Collector state", robot.collector_state || "idle"],
        ["Search", `${search.search_state || "idle"} / Zone ${search.zone_id || "?"}`],
        ["Collect pattern", collectPattern.phase || "idle"],
        ["Coverage", fmt(search.coverage_pct, "%")],
        ["LiDAR height", fmt(mounts.front_lidar?.world_z_m, "m")],
        ["OAK-D height", fmt(mounts.front_camera?.world_z_m, "m")],
        ["OAK-D Depth", oakDepth.used_for_current_observation ? `${fmt(oakDepth.range_m, "m")} used` : (oakDepth.available ? "available" : "unavailable")],
        ["Collect one", collectOne.phase || "idle"],
        ["Side complete", completion.current_side_complete ? "yes" : "no"],
        ["Remaining balls", `${balls.same_side_remaining ?? "?"} same-side / ${balls.total_remaining ?? "?"} total`],
        ["Across net", balls.across_net_remaining ?? "none"],
        ["Intake", out.intake_enabled ? "enabled" : "disabled"],
        ["Base command", `${fmt(out.linear_speed_m_s, "m/s")} / ${fmt(out.angular_speed_rad_s, "rad/s")}`],
        ["Lift wheel", fmt(out.lift_wheel_speed)],
        ["Survey waypoint", `${(survey.waypoint_index ?? 0) + 1}/${survey.waypoint_count ?? 0}`],
        ["Front range", fmt(survey.front_range_m, "m")]
      ]);
      setKv("selectedMode", [
        ["Requested mode", command.mode || "idle"],
        ["Actual mode", robot.actual_mode || "idle"],
        ["Sequence", command.sequence ?? 0],
        ["Updated", dateText(command.updated_at)],
        ["Source", command.source || "default"],
        ["Controller state", robot.collector_state || "idle"],
        ["Search state", search.search_state || "idle"],
        ["Search target", `${fmt(search.target_x_m, "m")}, ${fmt(search.target_y_m, "m")}`],
        ["Collect pattern", `${collectPattern.phase || "idle"} / failures ${collectPattern.failures ?? 0}`],
        ["Collect one phase", collectOne.phase || "idle"]
      ]);
      setKv("telemetryKv", [
        ["Telemetry enabled", robot.telemetry_enabled ? "yes" : "no"],
        ["Vision enabled", robot.vision_enabled ? "yes" : "no"],
        ["Route overlay", robot.route_visualization_enabled ? "yes" : "no"],
        ["Status age", fmt(robot.age_s, "s")],
        ["Visible candidates", balls.visible_candidates ?? 0],
        ["Nearest same-side", fmt(balls.nearest_same_side_distance_m, "m")],
        ["Loop count", robot.loop_count ?? 0],
        ["Ball visible", obs.visible ? "yes" : "no"],
        ["OAK-D Depth range", oakDepth.used_for_current_observation ? fmt(oakDepth.range_m, "m") : "not used"],
        ["OAK-D Depth limits", `${fmt(oakDepth.min_range_m, "m")} - ${fmt(oakDepth.max_range_m, "m")}`],
        ["Ball world", `${fmt(obs.world_x_m, "m")}, ${fmt(obs.world_y_m, "m")}`],
        ["Confidence", fmt(obs.confidence)],
        ["Animation", robot.collection_animation_active ? "active" : "idle"],
        ["Scan progress", `${fmt((scan.progress || 0) * 100, "%")} (${fmt(scan.elapsed_s, "s")}/${fmt(scan.full_turn_s, "s")})`],
        ["Scan best target", scan.best_visible ? `${fmt(scan.best_distance_m, "m")} @ ${fmt((scan.best_bearing_rad || 0) * 180 / Math.PI, "deg")}` : "none"],
        ["Search path", search.path_status || "idle"],
        ["Search resume", search.resume_marker || "none"],
        ["Survey state", survey.state || "idle"],
        ["Survey target", `${fmt(survey.target_x_m, "m")}, ${fmt(survey.target_y_m, "m")}`]
      ]);
      document.getElementById("rawStatus").textContent = JSON.stringify(robot, null, 2);

      // auto-navigate to sensors view when scan_side is active
      const scanSide = robot.scan_side || {};
      const scanProgressEl = document.getElementById("scanSideProgress");
      if (scanSide.active) {
        if (scanProgressEl) scanProgressEl.style.display = "block";
        const elEl = document.getElementById("scanSideElapsed");
        const durEl = document.getElementById("scanSideDuration");
        const barEl = document.getElementById("scanSideBar");
        if (elEl) elEl.textContent = fmt(scanSide.elapsed_s);
        if (durEl) durEl.textContent = fmt(scanSide.duration_s);
        if (barEl) barEl.style.width = `${Math.round((scanSide.progress || 0) * 100)}%`;
        const activeView = document.querySelector("section.view.active");
        if (activeView && activeView.id !== "sensors") setView("sensors");
      } else {
        if (scanProgressEl) scanProgressEl.style.display = "none";
      }

      renderHistory();
      renderStats();
      renderCourtMap();
      renderSensors();
      renderMapMission(robot.map_mission || {});

      // Auto-navigate to sensors when mapping mission is active
      const mapMission = robot.map_mission || {};
      if (mapMission.active && !mapMission.complete) {
        const activeView = document.querySelector("section.view.active");
        if (activeView && activeView.id !== "sensors") setView("sensors");
      }
    }
    function renderMapMission(m) {
      const statusEl = document.getElementById("mapMissionStatus");
      if (statusEl) {
        if (m.complete) statusEl.textContent = "— complete";
        else if (m.active) statusEl.textContent = `— ${m.phase_label || m.phase}`;
        else statusEl.textContent = "idle";
        statusEl.style.color = m.active ? "#a855f7" : (m.complete ? "var(--accent)" : "var(--muted)");
      }

      const hasData = m.active || m.complete;
      const grid = m.grid || [[0,0,0],[0,0,0],[0,0,0]];
      for (let r = 0; r < 3; r++) {
        for (let c = 0; c < 3; c++) {
          const el = document.getElementById(`mapCell${r}${c}`);
          if (!el) continue;
          if (!hasData) {
            el.textContent = "—";
            el.style.background = "var(--panel-2)";
            el.style.color = "var(--muted)";
            continue;
          }
          const count = (grid[r] || [])[c] ?? 0;
          el.textContent = count;
          if (count === 0) {
            el.style.background = "rgba(255,255,255,0.03)";
            el.style.color = "rgba(145,162,178,0.5)";
          } else if (count <= 2) {
            el.style.background = "rgba(87,166,255,0.13)";
            el.style.color = "#57a6ff";
          } else if (count <= 5) {
            el.style.background = "rgba(255,189,90,0.13)";
            el.style.color = "#ffbd5a";
          } else {
            el.style.background = "rgba(47,208,143,0.15)";
            el.style.color = "#2fd08f";
          }
        }
      }

      const progressEl = document.getElementById("mapMissionProgress");
      if (progressEl) {
        if (m.active && !m.complete) {
          progressEl.style.display = "block";
          const done = m.scan_poses_done ?? 0;
          const total = m.scan_poses_total ?? 5;
          const pct = total > 0 ? Math.round(done / total * 100) : 0;
          const phaseEl = document.getElementById("mapMissionPhaseLabel");
          const posesEl = document.getElementById("mapMissionPosesLabel");
          const barEl   = document.getElementById("mapMissionBar");
          if (phaseEl) phaseEl.textContent = m.phase_label || m.phase || "";
          if (posesEl) posesEl.textContent = `${done} / ${total} poses`;
          if (barEl)   barEl.style.width   = `${pct}%`;
        } else {
          progressEl.style.display = "none";
        }
      }

      const totalsEl = document.getElementById("mapMissionTotals");
      if (totalsEl && hasData) {
        const flat = grid.flat ? grid.flat() : [].concat(...grid);
        const gridSum = flat.reduce((a, b) => a + b, 0);
        totalsEl.textContent = `Candidates detected: ${m.total_candidates ?? 0} · Grid sum: ${gridSum} · Elapsed: ${fmt(m.elapsed_s, "s")}`;
      } else if (totalsEl) {
        totalsEl.textContent = "";
      }
    }
    function renderSensor(id, sensor) {
      const target = document.getElementById(id);
      if (!target) return;
      if (!sensor || !sensor.data_url) {
        target.className = "sensor-empty";
        target.textContent = "not available";
        return;
      }
      target.className = "";
      target.innerHTML = `<img src="${sensor.data_url}" alt="${id}">`;
    }
    function renderLidarSensor(id, sensor) {
      const target = document.getElementById(id);
      if (!target) return;
      const ranges = Array.isArray(sensor?.ranges_m) ? sensor.ranges_m : [];
      if (!ranges.length) {
        renderSensor(id, sensor);
        return;
      }
      const canvasId = `${id}Canvas`;
      const validRanges = ranges.filter(value => Number.isFinite(value) && value > 0);
      const minRange = Number.isFinite(sensor.min_range_m) ? sensor.min_range_m : 0.05;
      const maxRange = Number.isFinite(sensor.max_range_m) ? sensor.max_range_m : Math.max(1, ...validRanges);
      const nearest = validRanges.length ? Math.min(...validRanges) : null;
      const blocked = validRanges.filter(value => value < Math.min(1.2, maxRange)).length;
      target.className = "";
      target.innerHTML = `
        <canvas id="${canvasId}" width="520" height="520" aria-label="LiDAR 360 degree scan"></canvas>
        <div class="sensor-meta">
          <div>front<strong>${fmt(sensor.front_range_m, "m")}</strong></div>
          <div>nearest<strong>${fmt(nearest, "m")}</strong></div>
          <div>hits<strong>${validRanges.length}/${ranges.length}</strong></div>
        </div>
      `;
      drawLidarScan(document.getElementById(canvasId), ranges, minRange, maxRange, blocked);
    }
    function drawLidarScan(canvas, ranges, minRange, maxRange, candidates) {
      if (!canvas) return;
      const ctx = canvas.getContext("2d");
      const width = canvas.width;
      const height = canvas.height;
      const cx = width / 2;
      const cy = height / 2;
      const radius = Math.min(width, height) * 0.43;
      const span = Math.max(0.001, maxRange - minRange);
      const candList = Array.isArray(candidates) ? candidates : [];
      const blockedCount = ranges.filter(v => Number.isFinite(v) && v > 0 && v < Math.min(1.5, maxRange)).length;

      ctx.clearRect(0, 0, width, height);
      ctx.fillStyle = "#05080b";
      ctx.fillRect(0, 0, width, height);

      // range rings
      ctx.strokeStyle = "rgba(145,162,178,0.22)";
      ctx.lineWidth = 1;
      [0.25, 0.5, 0.75, 1].forEach(scale => {
        ctx.beginPath();
        ctx.arc(cx, cy, radius * scale, 0, Math.PI * 2);
        ctx.stroke();
        const rLabel = (minRange + span * scale).toFixed(1);
        ctx.fillStyle = "rgba(145,162,178,0.5)";
        ctx.font = "11px Inter, system-ui, sans-serif";
        ctx.fillText(`${rLabel}m`, cx + radius * scale + 3, cy - 3);
      });

      // forward indicator
      ctx.strokeStyle = "rgba(87,166,255,0.55)";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(cx, cy);
      ctx.lineTo(cx, cy - radius);
      ctx.stroke();
      ctx.fillStyle = "rgba(255,189,90,0.95)";
      ctx.beginPath();
      ctx.moveTo(cx, cy - radius - 12);
      ctx.lineTo(cx - 7, cy - radius + 3);
      ctx.lineTo(cx + 7, cy - radius + 3);
      ctx.closePath();
      ctx.fill();

      // LiDAR range points
      // Webots Lidar: index 0 = backward (-x), index n/2 = forward (+x), scan CW from above
      // Canvas convention: forward = top, left = left (so x = cx - sin(θ), y = cy - cos(θ))
      ranges.forEach((value, index) => {
        if (!Number.isFinite(value) || value <= 0) return;
        const theta = (index / ranges.length) * Math.PI * 2 - Math.PI;
        const clamped = Math.max(minRange, Math.min(maxRange, value));
        const r = ((clamped - minRange) / span) * radius;
        const x = cx - Math.sin(theta) * r;
        const y = cy - Math.cos(theta) * r;
        const near = value < Math.min(1.2, maxRange);
        ctx.fillStyle = near ? "rgba(255,189,90,0.95)" : "rgba(47,208,143,0.88)";
        ctx.fillRect(x - 1.7, y - 1.7, 3.4, 3.4);
      });

      // Ball candidates overlay
      // robot_x_m = forward (+x), robot_y_m = left (+y) — same canvas transform as LiDAR points
      candList.forEach((cand, i) => {
        const rx = cand.robot_x_m;
        const ry = cand.robot_y_m;
        const dist = Math.hypot(rx, ry);
        if (dist < 0.1 || dist > maxRange) return;
        const bearing = Math.atan2(ry, rx);
        const r = Math.min(1.0, (dist - minRange) / span) * radius;
        const x = cx - Math.sin(bearing) * r;
        const y = cy - Math.cos(bearing) * r;
        // Pulsing ring
        ctx.beginPath();
        ctx.arc(x, y, 11, 0, Math.PI * 2);
        ctx.strokeStyle = "rgba(255,189,90,0.35)";
        ctx.lineWidth = 3;
        ctx.stroke();
        // Filled dot
        ctx.beginPath();
        ctx.arc(x, y, 6, 0, Math.PI * 2);
        ctx.fillStyle = "#ffbd5a";
        ctx.fill();
        ctx.strokeStyle = "#07110d";
        ctx.lineWidth = 1.5;
        ctx.stroke();
        // Label
        ctx.fillStyle = "#ffbd5a";
        ctx.font = "bold 11px Inter, system-ui, sans-serif";
        ctx.fillText(`${dist.toFixed(1)}m`, x + 9, y - 5);
      });

      // Robot dot
      ctx.fillStyle = "#eef4f8";
      ctx.beginPath();
      ctx.arc(cx, cy, 7, 0, Math.PI * 2);
      ctx.fill();

      // Status text
      ctx.fillStyle = "rgba(145,162,178,0.9)";
      ctx.font = "12px Inter, system-ui, sans-serif";
      ctx.fillText(`${blockedCount} near`, 14, height - 16);
      if (candList.length > 0) {
        ctx.fillStyle = "#ffbd5a";
        ctx.fillText(`${candList.length} candidate${candList.length > 1 ? "s" : ""}`, 14, height - 32);
      }
    }
    function renderSensors() {
      renderSensor("sensorsCameraView", sensors.front_camera);
      renderSensor("sensorsDepthView", sensors.front_depth);
      renderLidarSensorFull("lidarScanView", sensors.front_lidar, sensors.lidar_candidates || []);
      renderIrIntake(sensors.ir_intake);
    }
    function renderIrIntake(ir) {
      const threshold = ir?.threshold ?? 500;
      function renderOne(valueId, barId, panelId, value, available) {
        const valEl = document.getElementById(valueId);
        const barEl = document.getElementById(barId);
        const panelEl = document.getElementById(panelId);
        if (!valEl || !barEl || !panelEl) return;
        if (!available || value === null || value === undefined) {
          valEl.textContent = "N/A";
          barEl.style.width = "0%";
          barEl.style.background = "rgba(255,255,255,0.15)";
          panelEl.style.borderColor = "var(--line)";
          return;
        }
        const pct = Math.min(100, Math.round((value / 1000) * 100));
        const triggered = value > threshold;
        valEl.textContent = Math.round(value);
        barEl.style.width = `${pct}%`;
        barEl.style.background = triggered ? "#2fd08f" : "rgba(145,162,178,0.45)";
        panelEl.style.borderColor = triggered ? "rgba(47,208,143,0.55)" : "var(--line)";
      }
      renderOne("irLeftValue", "irLeftBar", "irLeftPanel", ir?.left, ir?.left_available ?? (ir !== undefined));
      renderOne("irRightValue", "irRightBar", "irRightPanel", ir?.right, ir?.right_available ?? (ir !== undefined));
      const badge = document.getElementById("irTriggeredBadge");
      if (badge) {
        const triggered = !!ir?.triggered;
        const available = ir?.left_available || ir?.right_available;
        badge.textContent = available ? (triggered ? "TRIGGERED: YES — collection gate open" : "TRIGGERED: NO — ball not in intake zone") : "TRIGGERED: sensors not available";
        badge.style.background = triggered ? "rgba(47,208,143,0.15)" : (available ? "rgba(255,255,255,0.04)" : "rgba(255,80,80,0.10)");
        badge.style.color = triggered ? "#2fd08f" : (available ? "var(--muted)" : "#ff6060");
        badge.style.border = triggered ? "1px solid rgba(47,208,143,0.35)" : "none";
      }
    }
    function renderLidarSensorFull(id, sensor, candidates) {
      const target = document.getElementById(id);
      if (!target) return;
      const ranges = Array.isArray(sensor?.ranges_m) ? sensor.ranges_m : [];
      if (!ranges.length) {
        target.className = "sensor-empty";
        target.style.cssText = "background:#090d12;border:1px solid var(--line);border-radius:8px;min-height:420px;";
        target.textContent = "waiting for LiDAR scan";
        return;
      }
      const canvasId = "lidarFullCanvas";
      const validRanges = ranges.filter(v => Number.isFinite(v) && v > 0);
      const minRange = Number.isFinite(sensor.min_range_m) ? sensor.min_range_m : 0.05;
      const maxRange = Number.isFinite(sensor.max_range_m) ? sensor.max_range_m : Math.max(1, ...validRanges);
      const nearest = validRanges.length ? Math.min(...validRanges) : null;
      const blocked = validRanges.filter(v => v < Math.min(1.5, maxRange)).length;
      const candList = Array.isArray(candidates) ? candidates : [];
      target.className = "";
      target.style.cssText = "background:#090d12;border:1px solid var(--line);border-radius:8px;overflow:hidden;";
      target.innerHTML = `<canvas id="${canvasId}" width="700" height="700" style="display:block;width:100%;max-height:520px;object-fit:contain;" aria-label="LiDAR 360 degree scan"></canvas>`;
      const metaEl = document.getElementById("lidarScanMeta");
      if (metaEl) metaEl.innerHTML = [
        ["front", `<strong style="color:var(--ink);display:block;font-size:13px;">${fmt(sensor.front_range_m, "m")}</strong>`],
        ["nearest", `<strong style="color:var(--ink);display:block;font-size:13px;">${fmt(nearest, "m")}</strong>`],
        ["hits", `<strong style="color:var(--ink);display:block;font-size:13px;">${validRanges.length}/${ranges.length}</strong>`],
        ["candidates", `<strong style="color:${candList.length > 0 ? "#ffbd5a" : "var(--muted)"};display:block;font-size:13px;">${candList.length}</strong>`],
      ].map(([label, val]) => `<div style="border-top:1px solid var(--line);padding-top:8px;">${label}${val}</div>`).join("");
      drawLidarScan(document.getElementById(canvasId), ranges, minRange, maxRange, candList);
    }
    function renderCourtMap() {
      const canvas = document.getElementById("courtMap");
      if (!canvas) return;
      const ctx = canvas.getContext("2d");
      const map = (diagnostics.robot || {}).map || {};
      const court = map.court || { min_x: -11.885, max_x: 11.885, min_y: -5.485, max_y: 5.485, net_x: 0 };
      const width = canvas.width;
      const height = canvas.height;
      const pad = 42;
      const sx = x => pad + (x - court.min_x) / (court.max_x - court.min_x) * (width - pad * 2);
      const sy = y => height - pad - (y - court.min_y) / (court.max_y - court.min_y) * (height - pad * 2);
      const scaleM = (width - pad * 2) / (court.max_x - court.min_x);

      ctx.clearRect(0, 0, width, height);
      ctx.fillStyle = "#7a3329";
      ctx.fillRect(0, 0, width, height);
      ctx.fillStyle = "#8f3f32";
      ctx.fillRect(pad, pad, width - pad * 2, height - pad * 2);

      ctx.strokeStyle = "rgba(255,255,255,0.78)";
      ctx.lineWidth = 3;
      ctx.strokeRect(pad, pad, width - pad * 2, height - pad * 2);
      ctx.beginPath();
      ctx.moveTo(sx(court.net_x || 0), pad);
      ctx.lineTo(sx(court.net_x || 0), height - pad);
      ctx.strokeStyle = "rgba(18,24,30,0.95)";
      ctx.lineWidth = 5;
      ctx.stroke();

      ctx.strokeStyle = "rgba(255,255,255,0.55)";
      ctx.lineWidth = 2;
      [-6.4, 6.4].forEach(x => {
        ctx.beginPath();
        ctx.moveTo(sx(x), pad);
        ctx.lineTo(sx(x), height - pad);
        ctx.stroke();
      });
      [-4.115, 4.115, 0].forEach(y => {
        ctx.beginPath();
        ctx.moveTo(pad, sy(y));
        ctx.lineTo(width - pad, sy(y));
        ctx.stroke();
      });

      const bounds = map.active_bounds;
      if (bounds) {
        ctx.fillStyle = "rgba(87,166,255,0.06)";
        ctx.fillRect(sx(bounds.min_x), sy(bounds.max_y), sx(bounds.max_x) - sx(bounds.min_x), sy(bounds.min_y) - sy(bounds.max_y));
      }

      // camera FOV cone
      const robot = map.robot || {};
      if (robot.x_m !== undefined && robot.y_m !== undefined) {
        const rx = sx(robot.x_m);
        const ry = sy(robot.y_m);
        const yaw = robot.yaw_rad || 0;
        const fov = map.camera_fov_rad || 1.05;
        const fovRange = (map.camera_max_range_m || 4.5) * scaleM;
        ctx.save();
        ctx.translate(rx, ry);
        ctx.rotate(-yaw);
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.arc(0, 0, fovRange, -fov / 2, fov / 2);
        ctx.closePath();
        ctx.fillStyle = "rgba(87,166,255,0.07)";
        ctx.fill();
        ctx.strokeStyle = "rgba(87,166,255,0.22)";
        ctx.lineWidth = 1.5;
        ctx.setLineDash([6, 4]);
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.restore();
      }

      const route = map.route || [];
      if (route.length > 1) {
        ctx.beginPath();
        route.forEach((point, index) => {
          const x = sx(point.x_m);
          const y = sy(point.y_m);
          if (index === 0) ctx.moveTo(x, y);
          else ctx.lineTo(x, y);
        });
        ctx.strokeStyle = "#57a6ff";
        ctx.lineWidth = 5;
        ctx.lineJoin = "round";
        ctx.lineCap = "round";
        ctx.stroke();
      }

      const activeTargetId = map.active_target_id;
      const allBalls = map.balls || [];

      // pending balls (below seen_count threshold) — drawn first so confirmed render on top
      allBalls.filter(b => !b.confirmed).forEach(ball => {
        const x = sx(ball.x_m);
        const y = sy(ball.y_m);
        ctx.globalAlpha = 0.35;
        ctx.beginPath();
        ctx.arc(x, y, 5, 0, Math.PI * 2);
        ctx.fillStyle = ball.side === "across_net" ? "#8793a0" : "#d7e85f";
        ctx.fill();
        ctx.setLineDash([3, 3]);
        ctx.strokeStyle = "rgba(255,255,255,0.4)";
        ctx.lineWidth = 1;
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.globalAlpha = 1.0;
      });

      // confirmed balls
      allBalls.filter(b => b.confirmed).forEach(ball => {
        const x = sx(ball.x_m);
        const y = sy(ball.y_m);
        const isActive = ball.id === activeTargetId;
        const radius = ball.planned ? 9 : 7;

        // active target ring
        if (isActive) {
          ctx.beginPath();
          ctx.arc(x, y, radius + 6, 0, Math.PI * 2);
          ctx.strokeStyle = "#2fd08f";
          ctx.lineWidth = 2.5;
          ctx.stroke();
        }

        ctx.beginPath();
        ctx.arc(x, y, radius, 0, Math.PI * 2);
        ctx.fillStyle = ball.side === "across_net" ? "#8793a0" : (ball.visible_candidate ? "#2fd08f" : "#d7e85f");
        ctx.fill();
        ctx.strokeStyle = ball.source === "oak_depth" ? "#57a6ff" : (ball.planned ? "#ffffff" : "rgba(0,0,0,0.45)");
        ctx.lineWidth = ball.source === "oak_depth" ? 3 : (ball.planned ? 3 : 1);
        ctx.stroke();

        if (ball.order) {
          ctx.fillStyle = "#07110d";
          ctx.font = "bold 12px system-ui";
          ctx.textAlign = "center";
          ctx.textBaseline = "middle";
          ctx.fillText(String(ball.order), x, y);
        }
      });

      // robot arrow
      if (robot.x_m !== undefined && robot.y_m !== undefined) {
        const x = sx(robot.x_m);
        const y = sy(robot.y_m);
        const yaw = robot.yaw_rad || 0;
        ctx.save();
        ctx.translate(x, y);
        ctx.rotate(-yaw);
        ctx.fillStyle = "#ffbd5a";
        ctx.beginPath();
        ctx.moveTo(16, 0);
        ctx.lineTo(-10, -9);
        ctx.lineTo(-10, 9);
        ctx.closePath();
        ctx.fill();
        ctx.restore();
      }

      // summary box
      const metrics = map.metrics || {};
      const confirmedCount = allBalls.filter(b => b.confirmed && b.side !== "across_net").length;
      const depthCount = allBalls.filter(b => b.confirmed && b.source === "oak_depth" && b.side !== "across_net").length;
      const pendingCount = allBalls.filter(b => !b.confirmed && b.side !== "across_net").length;
      const plannedCount = metrics.balls_collectable ?? 0;
      const summaryLine1 = `confirmed ${confirmedCount} · depth ${depthCount} · planned ${plannedCount}${pendingCount > 0 ? ` · pending ${pendingCount}` : ""}`;
      const summaryLine2 = metrics.total_distance_m != null
        ? `distance ${fmt(metrics.total_distance_m, "m")} · replans ${metrics.planned_replans ?? 0}`
        : "no route planned";
      const boxW = Math.max(280, ctx.measureText(summaryLine1).width + 28);
      ctx.fillStyle = "rgba(12,17,22,0.76)";
      ctx.fillRect(pad + 10, pad + 10, boxW, 64);
      ctx.fillStyle = "#eef4f8";
      ctx.font = "16px system-ui";
      ctx.textAlign = "left";
      ctx.fillText(summaryLine1, pad + 24, pad + 36);
      ctx.fillStyle = "#91a2b2";
      ctx.font = "13px system-ui";
      ctx.fillText(summaryLine2, pad + 24, pad + 58);
    }
    function renderHistory() {
      const history = diagnostics.history || [];
      document.getElementById("latestEvents").innerHTML = history.slice(-8).reverse().map((row, index) => (
        `<div class="event"><span>${dateText(row.updated_at)}</span><strong>${row.mode}</strong><span>sequence ${row.sequence} · ${row.source}${index === 0 ? " · latest" : ""}</span></div>`
      )).join("") || "<div class='event'><span>none</span><strong>No commands</strong><span>Waiting for input</span></div>";
      document.getElementById("historyRows").innerHTML = history.slice().reverse().map((row, index) => (
        `<tr class="${index === 0 ? "latest" : ""}"><td>${dateText(row.updated_at)}</td><td>${row.mode}</td><td>${row.sequence}</td><td>${row.source}</td></tr>`
      )).join("");
    }
    function renderStats() {
      const stats = diagnostics.stats || {};
      const total = stats.total || 0;
      const byMode = stats.by_mode || {};
      document.getElementById("sTotal").textContent = total;
      document.getElementById("sCollect").textContent = (byMode.collect || 0) + (byMode.collect_pattern || 0) + (byMode.collect_one || 0);
      document.getElementById("sSurvey").textContent = byMode.survey || 0;
      document.getElementById("sIdle").textContent = byMode.idle || 0;
      document.getElementById("statsRows").innerHTML = ["collect", "collect_pattern", "search", "collect_one", "scan_side", "map_left_side", "survey", "idle"].map(mode => {
        const count = byMode[mode] || 0;
        const latest = (stats.latest_by_mode || {})[mode] || {};
        const share = total ? `${Math.round(count * 100 / total)}%` : "0%";
        return `<tr><td>${mode}</td><td>${count}</td><td>${share}</td><td>${latest.sequence ?? "none"}</td><td>${latest.source ?? "none"}</td></tr>`;
      }).join("");
    }
    refresh();
    setInterval(refresh, 1000);
  </script>
</body>
</html>
"""


class ControlPanelHandler(BaseHTTPRequestHandler):
    store: RobotCommandStore
    status_store: RobotStatusStore
    sensor_store: RobotSensorStore

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        if path in {"/", "/index.html"}:
            self._send_html(HTML)
            return
        if path == "/api/status":
            self._send_json(self.store.read().to_mapping())
            return
        if path == "/api/robot-status":
            self._send_json(self.status_store.read())
            return
        if path == "/api/sensors":
            self._send_json(self.sensor_store.read())
            return
        if path == "/api/history":
            self._send_json({"history": self.store.read_history()})
            return
        if path == "/api/diagnostics":
            self._send_json(self._diagnostics())
            return
        if path == "/api/webcam/frame":
            self._handle_webcam_frame()
            return
        if path == "/favicon.ico":
            self.send_response(HTTPStatus.NO_CONTENT)
            self.end_headers()
            return
        self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        if path not in {"/command", "/api/command"}:
            self.send_error(HTTPStatus.NOT_FOUND)
            return

        length = int(self.headers.get("Content-Length", "0"))
        body = self.rfile.read(length).decode("utf-8")
        mode = parse_qs(body).get("mode", ["idle"])[0]
        if mode not in SUPPORTED_MODES:
            self.send_error(HTTPStatus.BAD_REQUEST, "Unsupported mode")
            return

        command = self.store.write(mode)
        if path == "/api/command":
            self._send_json(command.to_mapping())
            return
        self.send_response(HTTPStatus.SEE_OTHER)
        self.send_header("Location", "/")
        self.end_headers()

    def _handle_webcam_frame(self) -> None:
        if not _VISION_AVAILABLE:
            self._send_json({"available": False, "error": "cv2 / perception not installed"})
            return
        ok, frame = WebcamManager.get_frame()
        if not ok or frame is None:
            self._send_json({"available": False, "error": "no webcam or read failed"})
            return

        h, w = frame.shape[:2]
        detection = detect_largest_ball(frame)
        result: dict[str, object] = {"available": True, "detected": detection is not None, "width": w, "height": h}

        if detection:
            cv2.rectangle(
                frame,
                (detection.x, detection.y),
                (detection.x + detection.width, detection.y + detection.height),
                (0, 220, 100), 2,
            )
            focal_px = (w / 2) / math.tan(math.radians(WEBCAM_FOV_DEG / 2))
            diam_px = detection.apparent_diameter_px
            distance_m = (TENNIS_BALL_DIAMETER_M * focal_px) / max(1.0, diam_px)
            normalized_x = (detection.center_x - w / 2) / (w / 2)
            bearing_rad = math.atan(normalized_x * math.tan(math.radians(WEBCAM_FOV_DEG / 2)))
            bearing_deg = math.degrees(bearing_rad)
            label = f"{distance_m:.2f}m"
            cv2.putText(frame, label, (detection.x, max(20, detection.y - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 220, 100), 2)
            result.update({
                "distance_m": round(distance_m, 3),
                "bearing_deg": round(bearing_deg, 1),
                "diameter_px": round(diam_px, 1),
            })

        _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 82])
        result["data_url"] = "data:image/jpeg;base64," + base64.b64encode(buf.tobytes()).decode()
        self._send_json(result)

    def log_message(self, format: str, *args: object) -> None:
        if urlparse(self.path).path in {"/api/status", "/api/robot-status", "/api/sensors", "/api/diagnostics", "/api/webcam/frame", "/favicon.ico"}:
            return
        print(f"{self.address_string()} - {format % args}")

    def _diagnostics(self) -> dict[str, object]:
        history = self.store.read_history(200)
        by_mode = Counter(str(row.get("mode", "unknown")) for row in history)
        latest_by_mode: dict[str, dict[str, object]] = {}
        for row in history:
            mode = str(row.get("mode", "unknown"))
            latest_by_mode[mode] = row
        robot_status = self.status_store.read()
        robot_updated_at = float(robot_status.get("updated_at", 0.0) or 0.0)
        robot_status["age_s"] = time.time() - robot_updated_at if robot_updated_at > 0 else None
        robot_status["stale"] = robot_updated_at <= 0 or robot_status["age_s"] > 3.0
        robot_status["connected"] = bool(robot_status.get("connected")) and not robot_status["stale"]
        return {
            "generated_at": time.time(),
            "command": self.store.read().to_mapping(),
            "robot": robot_status,
            "history": history[-50:],
            "stats": {
                "total": len(history),
                "by_mode": dict(by_mode),
                "latest_by_mode": latest_by_mode,
            },
        }

    def _send_html(self, html: str) -> None:
        payload = html.encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Cache-Control", "no-store, max-age=0")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def _send_json(self, data: dict[str, object]) -> None:
        payload = json.dumps(data).encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store, max-age=0")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the tennis robot remote console.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8081)
    parser.add_argument("--command-file", type=Path, default=None)
    parser.add_argument("--status-file", type=Path, default=None)
    args = parser.parse_args()

    ControlPanelHandler.store = RobotCommandStore(args.command_file) if args.command_file else RobotCommandStore.from_env()
    ControlPanelHandler.status_store = RobotStatusStore(args.status_file) if args.status_file else RobotStatusStore.from_env()
    ControlPanelHandler.sensor_store = RobotSensorStore.from_env()
    server = ThreadingHTTPServer((args.host, args.port), ControlPanelHandler)
    print(f"remote console listening on http://{args.host}:{args.port}")
    print(f"command file: {ControlPanelHandler.store.path}")
    print(f"status file: {ControlPanelHandler.status_store.path}")
    print(f"sensor file: {ControlPanelHandler.sensor_store.path}")
    server.serve_forever()


if __name__ == "__main__":
    main()
