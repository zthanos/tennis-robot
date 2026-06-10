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
sys.path.insert(0, str(ROOT / "scripts"))

from tennis_robot.control_bus import RobotCommandStore, RobotSensorStore, RobotStatusStore  # noqa: E402
from db_store import TennisRobotDB  # noqa: E402

try:
    import cv2
    from tennis_robot.perception import detect_largest_ball, TENNIS_BALL_DIAMETER_M
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
    .command[value="map_court"] { background: var(--accent-2); color: #06101d; }
    .command[value="idle"] { background: var(--danger); color: #1b0604; }
    .command:disabled { opacity: 0.32; cursor: not-allowed; transform: none !important; filter: none !important; }
    .dpad {
      display: flex;
      flex-direction: column;
      align-items: center;
      gap: 4px;
      margin-bottom: 14px;
    }
    .dpad-middle {
      display: flex;
      align-items: center;
      gap: 4px;
    }
    .dpad-up, .dpad-down, .dpad-left, .dpad-right, .dpad-center {
      width: 52px;
      height: 52px;
      padding: 0;
      font-size: 18px;
      display: flex;
      align-items: center;
      justify-content: center;
    }
    .dpad-center {
      font-size: 13px;
      font-weight: 800;
      background: #4f6e8a;
      color: #eaf2ff;
      width: 52px;
      height: 52px;
    }
    .mission-btns {
      display: flex;
      gap: 8px;
      flex-wrap: wrap;
    }
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
        <p>Remote control and live diagnostics for the Gazebo + ROS 2 simulation.</p>
      </div>
      <nav aria-label="Console sections">
        <button class="active" data-view="dashboard">Dashboard</button>
        <button data-view="control">Control</button>
        <button data-view="sensors">Sensor Views</button>
        <button data-view="telemetry">Telemetry</button>
        <button data-view="stats">Command Stats</button>
        <button data-view="history">History</button>
        <button data-view="webcam">Webcam</button>
        <button data-view="vendors">Vendors</button>
        <button data-view="surveys">Court Maps</button>
      </nav>
      <div class="connection">
        <div><span id="liveDot" class="dot"></span><span id="connectionText">Waiting for robot status</span></div>
        <div id="commandFile">Command file: runtime/robot_command.json</div>
        <div id="activeSessionSidebar" style="margin-top:8px;font-size:12px;color:var(--muted);">Session: none</div>
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
              <div class="dpad">
                <button class="command dpad-up" type="submit" name="mode" value="move_forward">&#9650;</button>
                <div class="dpad-middle">
                  <button class="command dpad-left" type="submit" name="mode" value="move_left">&#9664;</button>
                  <button class="command dpad-center" type="submit" name="mode" value="turn_180">180°</button>
                  <button class="command dpad-right" type="submit" name="mode" value="move_right">&#9654;</button>
                </div>
                <button class="command dpad-down" type="submit" name="mode" value="move_backward">&#9660;</button>
              </div>
              <div class="mission-btns">
                <button class="command" type="submit" name="mode" value="map_court">Map Court</button>
                <button class="command" type="submit" name="mode" value="collect_pattern">Collect</button>
                <button class="command" type="submit" name="mode" value="idle">Stop</button>
              </div>
            </form>
            <div id="commandHint" style="font-size:12px;color:var(--warn);margin-top:8px;min-height:16px;"></div>
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
        <div class="panel" style="margin-top:14px;">
          <h3>Obstacle Survey Debug &nbsp;<span id="obsState" style="font-size:13px;font-weight:400;color:var(--muted);">idle</span></h3>
          <div id="obsKv" class="kv" style="margin-top:8px;margin-bottom:10px;"></div>
          <div id="obsLog" style="font-family:monospace;font-size:11px;line-height:1.5;background:#090d12;border:1px solid var(--line);border-radius:6px;padding:8px 10px;max-height:260px;overflow-y:auto;white-space:pre;"></div>
          <details style="margin-top:12px;">
            <summary style="cursor:pointer;font-size:12px;color:var(--muted);user-select:none;">Previous runs (history)</summary>
            <div id="obsRunsTable" style="margin-top:8px;font-size:12px;"></div>
          </details>
        </div>
        <div id="surveyBoundaryPanel" class="panel" style="margin-top:14px;">
          <h3>Court Map Boundary &nbsp;<span id="surveyBoundaryStatus" style="font-size:13px;font-weight:400;color:var(--muted);">no court map data</span></h3>
          <div id="surveyBoundaryKv" class="kv" style="margin-top:10px;"></div>
        </div>
        <div class="panel" style="margin-top:14px;">
          <h3>Map Court Discovery &nbsp;<span id="surveyDiscoveryStatus" style="font-size:13px;font-weight:400;color:var(--muted);">waiting for points</span></h3>
          <canvas id="surveyDiscoveryMap" width="1000" height="520" style="display:block;width:100%;max-height:520px;background:#090d12;border:1px solid var(--line);border-radius:8px;"></canvas>
          <div id="surveyDiscoveryMeta" class="kv" style="margin-top:10px;"></div>
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
            <h3>Recent Events</h3>
            <div id="telemetryEvents" class="log"></div>
          </div>
        </div>
        <div class="panel" style="margin-top:14px;">
            <h3>Raw Status</h3>
            <pre id="rawStatus" class="json">{}</pre>
        </div>
      </section>

      <section id="stats" class="view">
        <div class="grid kpis">
          <div class="metric"><span>Total Commands</span><strong id="sTotal">0</strong><small>from local history</small></div>
          <div class="metric"><span>Map Court Commands</span><strong id="sSurvey">0</strong><small>requested mode map_court</small></div>
          <div class="metric"><span>Collect Commands</span><strong id="sCollect">0</strong><small>requested mode map_left_side</small></div>
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

      <section id="surveys" class="view">
        <div class="panel">
          <h3>Court Map History</h3>
          <p style="color:var(--muted);font-size:13px;margin:0 0 14px;">Αποτελέσματα χαρτογράφησης γηπέδου (Court Knowledge Model) — κάθε ολοκληρωμένο Map Court αποθηκεύεται αυτόματα.</p>
          <table id="surveyTable">
            <thead>
              <tr>
                <th>#</th><th>Ημ/νία</th><th>Vendor</th><th>Court</th><th>Surface</th>
                <th>Status</th><th>Length (m)</th><th>Width (m)</th>
                <th>W fence</th><th>E fence</th><th>S fence</th><th>N fence</th>
                <th>Pts</th>
              </tr>
            </thead>
            <tbody id="surveyRows"></tbody>
          </table>
          <div id="surveyEmpty" style="color:var(--muted);font-size:13px;padding:14px 0;display:none;">Δεν υπάρχουν χαρτογραφήσεις ακόμα. Τρέξε Map Court για να ξεκινήσει η καταγραφή.</div>
        </div>
      </section>

      <section id="vendors" class="view">
        <div class="grid" style="gap:14px;">
          <div class="panel" style="grid-column:1/-1;">
            <h3>Active Session</h3>
            <div id="activeSessionDisplay" style="margin-bottom:14px;padding:12px 16px;background:var(--panel-2);border-radius:7px;border:1px solid var(--line);font-size:14px;">
              <span style="color:var(--muted);font-size:13px;">No active session — select a vendor and court below.</span>
            </div>
            <form id="activeSessionForm" style="display:flex;gap:12px;flex-wrap:wrap;align-items:flex-end;">
              <div>
                <label style="display:block;font-size:12px;color:var(--muted);margin-bottom:6px;text-transform:uppercase;">Vendor</label>
                <select id="activeVendorSelect" style="background:var(--panel-2);border:1px solid var(--line);color:var(--ink);padding:9px 12px;border-radius:7px;font:inherit;min-width:200px;"></select>
              </div>
              <div>
                <label style="display:block;font-size:12px;color:var(--muted);margin-bottom:6px;text-transform:uppercase;">Court / Γήπεδο</label>
                <select id="activeCourtSelect" style="background:var(--panel-2);border:1px solid var(--line);color:var(--ink);padding:9px 12px;border-radius:7px;font:inherit;min-width:200px;"></select>
              </div>
              <button type="submit" class="command">Set Active</button>
            </form>
          </div>
          <div class="grid two" style="gap:14px;">
            <div class="panel">
              <h3>Vendors / Χώροι</h3>
              <div id="vendorList" style="margin-bottom:16px;"></div>
              <form id="addVendorForm">
                <div style="margin-bottom:10px;">
                  <label style="display:block;font-size:12px;color:var(--muted);margin-bottom:5px;text-transform:uppercase;">Name *</label>
                  <input id="vendorName" type="text" required placeholder="π.χ. Athens Tennis Club" style="width:100%;background:var(--panel-2);border:1px solid var(--line);color:var(--ink);padding:9px 12px;border-radius:7px;font:inherit;">
                </div>
                <div style="margin-bottom:12px;">
                  <label style="display:block;font-size:12px;color:var(--muted);margin-bottom:5px;text-transform:uppercase;">Address</label>
                  <input id="vendorAddress" type="text" placeholder="προαιρετικό" style="width:100%;background:var(--panel-2);border:1px solid var(--line);color:var(--ink);padding:9px 12px;border-radius:7px;font:inherit;">
                </div>
                <button type="submit" class="command" style="background:var(--accent-2);color:#06101d;">Add Vendor</button>
              </form>
            </div>
            <div class="panel">
              <h3>Courts / Γήπεδα</h3>
              <div id="courtList" style="margin-bottom:16px;"></div>
              <form id="addCourtForm">
                <div style="margin-bottom:10px;">
                  <label style="display:block;font-size:12px;color:var(--muted);margin-bottom:5px;text-transform:uppercase;">Vendor *</label>
                  <select id="courtVendorSelect" required style="width:100%;background:var(--panel-2);border:1px solid var(--line);color:var(--ink);padding:9px 12px;border-radius:7px;font:inherit;"><option value="">— select vendor —</option></select>
                </div>
                <div style="margin-bottom:10px;">
                  <label style="display:block;font-size:12px;color:var(--muted);margin-bottom:5px;text-transform:uppercase;">Court Name *</label>
                  <input id="courtName" type="text" required placeholder="π.χ. Γήπεδο 1" style="width:100%;background:var(--panel-2);border:1px solid var(--line);color:var(--ink);padding:9px 12px;border-radius:7px;font:inherit;">
                </div>
                <div style="margin-bottom:10px;">
                  <label style="display:block;font-size:12px;color:var(--muted);margin-bottom:5px;text-transform:uppercase;">Surface</label>
                  <select id="courtSurface" style="width:100%;background:var(--panel-2);border:1px solid var(--line);color:var(--ink);padding:9px 12px;border-radius:7px;font:inherit;">
                    <option value="clay">Clay (χώμα)</option>
                    <option value="hard">Hard (σκληρό)</option>
                    <option value="grass">Grass (χορτάρι)</option>
                    <option value="carpet">Carpet (χαλί)</option>
                  </select>
                </div>
                <div style="margin-bottom:12px;">
                  <label style="display:block;font-size:12px;color:var(--muted);margin-bottom:5px;text-transform:uppercase;">Notes</label>
                  <input id="courtNotes" type="text" placeholder="προαιρετικό" style="width:100%;background:var(--panel-2);border:1px solid var(--line);color:var(--ink);padding:9px 12px;border-radius:7px;font:inherit;">
                </div>
                <button type="submit" class="command" style="background:var(--accent-2);color:#06101d;">Add Court</button>
              </form>
            </div>
          </div>
        </div>
      </section>
    </main>
  </div>

  <script>
    const titles = {
      dashboard: ["Dashboard", "Observe the robot mode, collector state, current target, and command stream while the simulation runs."],
      control: ["Control", "Send high-level commands to the running controller."],
      sensors: ["Sensor Views", "Live RPLIDAR C1 360° ground scan, front camera, OAK-D depth image, court boundary from last Map Court run, and half-court mapping grid. Run Map Court first to measure boundaries; then Collect Left Side uses them automatically."],
      telemetry: ["Telemetry", "Inspect live robot pose, detection, command output, survey data, and raw status."],
      stats: ["Command Stats", "Review per-mode command counts and recent command usage."],
      history: ["History", "Audit the local command stream written by this console and controller startup."],
      webcam: ["Webcam", "Live webcam feed with HSV tennis ball detection and monocular distance estimation."],
      vendors: ["Vendors", "Manage vendors, venues and courts. Set the active session so survey results are tagged with the correct location."],
      surveys: ["Court Map History", "Ιστορικό χαρτογράφησης γηπέδου αποθηκευμένο στη DuckDB — ένα row ανά Map Court run."]
    };
    let diagnostics = { command: {}, robot: {}, history: [], stats: {} };
    let sensors = {};
    let lastSurveyDiscovery = null;

    function fmt(value, suffix = "") {
      if (value === null || value === undefined || Number.isNaN(value)) return "none";
      if (typeof value === "number") return `${value.toFixed(Math.abs(value) >= 10 ? 1 : 2)}${suffix}`;
      return String(value);
    }
    function canonicalFenceBounds(bounds) {
      const corners = (bounds?.canonical_fence_model || {}).corners || {};
      const pts = Object.values(corners).filter(p =>
        Number.isFinite(p?.x_m) && Number.isFinite(p?.y_m)
      );
      if (!pts.length) return {};
      const xs = pts.map(p => p.x_m);
      const ys = pts.map(p => p.y_m);
      return {
        west_x: Math.min(...xs),
        east_x: Math.max(...xs),
        south_y: Math.min(...ys),
        north_y: Math.max(...ys),
      };
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
    function escapeHtml(value) {
      return String(value)
        .replaceAll("&", "&amp;")
        .replaceAll("<", "&lt;")
        .replaceAll(">", "&gt;")
        .replaceAll('"', "&quot;")
        .replaceAll("'", "&#039;");
    }
    function setKv(id, rows) {
      document.getElementById(id).innerHTML = rows.map(([label, value]) => (
        `<div><span>${label}</span><strong>${value}</strong></div>`
      )).join("");
    }
    function renderTelemetryEvents(events) {
      const target = document.getElementById("telemetryEvents");
      if (!target) return;
      if (!events || events.length === 0) {
        target.innerHTML = `<div style="color:var(--muted);font-size:13px;">No telemetry events yet.</div>`;
        return;
      }
      const hidden = new Set(["t_s", "wall_time_s", "type", "severity"]);
      target.innerHTML = events.slice(-24).reverse().map(event => {
        const details = Object.entries(event)
          .filter(([key, value]) => !hidden.has(key) && value !== null && value !== undefined)
          .map(([key, value]) => `${key}=${Array.isArray(value) || typeof value === "object" ? JSON.stringify(value) : value}`)
          .join(" · ");
        const severity = event.severity || "info";
        const color = severity === "error" ? "var(--danger)" : severity === "warning" ? "var(--warn)" : "var(--accent-2)";
        return `<div class="event">
          <span>${fmt(event.t_s, "s")}</span>
          <strong style="color:${color};">${escapeHtml(severity)}</strong>
          <div><strong>${escapeHtml(event.type || "event")}</strong><br><span>${escapeHtml(details || "no details")}</span></div>
        </div>`;
      }).join("");
    }
    function setView(name) {
      document.querySelectorAll("nav button").forEach(btn => btn.classList.toggle("active", btn.dataset.view === name));
      document.querySelectorAll("section.view").forEach(view => view.classList.toggle("active", view.id === name));
      document.getElementById("viewTitle").textContent = titles[name][0];
      document.getElementById("viewHelp").textContent = titles[name][1];
      if (name === "webcam") startWebcam(); else stopWebcam();
      if (name === "vendors") loadVendors();
      if (name === "surveys") loadSurveys();
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

    const DPAD_MODES = new Set(["move_forward", "move_backward", "move_left", "move_right"]);
    const AUTONOMOUS_MODES = new Set(["map_court", "map_left_side", "collect_pattern", "collect", "collect_one", "search", "scan_side"]);

    function updateCommandButtons() {
      const active = vendorData.active || {};
      const hasSession = !!(active.vendor_id && active.court_id);
      const cb = diagnostics.court_boundary;
      const hasSurvey = hasSession && !!(cb && cb.survey_complete && cb.court_id === active.court_id);
      const actualMode = (diagnostics.robot || {}).actual_mode || "idle";
      const isAutonomous = AUTONOMOUS_MODES.has(actualMode);

      const btnMapCourt = document.querySelector('#commandForm [value="map_court"]');
      const btnCollect  = document.querySelector('#commandForm [value="collect_pattern"]');
      const hintEl      = document.getElementById("commandHint");

      if (btnMapCourt) btnMapCourt.disabled = !hasSession || isAutonomous;
      if (btnCollect)  btnCollect.disabled  = !hasSurvey || isAutonomous;

      const MANUAL_MODES = new Set([...DPAD_MODES, "turn_180"]);
      document.querySelectorAll("#commandForm .command").forEach(btn => {
        if (MANUAL_MODES.has(btn.value)) btn.disabled = isAutonomous;
      });

      if (hintEl) {
        if (isAutonomous)    hintEl.textContent = `Αυτόνομη λειτουργία ενεργή (${actualMode}) — πάτα Stop για έλεγχο`;
        else if (!hasSession) hintEl.textContent = "Επίλεξε vendor και γήπεδο πρώτα (Vendors →)";
        else if (!hasSurvey) hintEl.textContent = "Collect χρειάζεται Map Court για το ενεργό γήπεδο";
        else                 hintEl.textContent = "";
      }
    }

    let _dpadTimer = null;

    async function _sendRawCommand(mode) {
      await fetch("/api/command", {
        method: "POST",
        headers: { "Content-Type": "application/x-www-form-urlencoded" },
        body: new URLSearchParams({ mode })
      });
    }

    async function _stopDpad() {
      if (_dpadTimer !== null) {
        clearInterval(_dpadTimer);
        _dpadTimer = null;
        await _sendRawCommand("idle");
        await refresh();
      }
    }

    document.querySelectorAll("#commandForm .command").forEach(btn => {
      if (!DPAD_MODES.has(btn.value)) return;
      btn.addEventListener("pointerdown", e => {
        e.preventDefault();
        if (_dpadTimer !== null) return;
        btn.setPointerCapture(e.pointerId);
        _sendRawCommand(btn.value);
        _dpadTimer = setInterval(() => _sendRawCommand(btn.value), 120);
      });
      btn.addEventListener("pointerup", () => _stopDpad());
      btn.addEventListener("pointercancel", () => _stopDpad());
    });

    document.getElementById("commandForm").addEventListener("submit", async event => {
      event.preventDefault();
      const mode = event.submitter?.value;
      if (!mode || DPAD_MODES.has(mode)) return;
      await _sendRawCommand(mode);
      const switchModes = new Set(["collect", "collect_one", "collect_pattern", "search", "scan_side", "map_court", "map_left_side"]);
      if (switchModes.has(mode)) setView("sensors");
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
      const diag = robot.diagnostics || {};
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
        ["Map court state", survey.state || "idle"],
        ["Map court event", survey.navigation?.last_event || "none"],
        ["Front range", fmt(survey.navigation?.front_lidar_range_m ?? survey.front_range_m, "m")]
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
        ["Mission health", diag.health || "unknown"],
        ["Health reasons", (diag.reasons || []).join(", ") || "none"],
        ["Last event", diag.last_event ? `${diag.last_event.type} @ ${fmt(diag.last_event.t_s, "s")}` : "none"],
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
        ["Map court state", survey.state || "idle"],
        ["Map court distance", fmt(survey.navigation?.distance_traveled_m, "m")]
      ]);
      renderTelemetryEvents(robot.timeline_events || []);
      document.getElementById("rawStatus").textContent = JSON.stringify(robot, null, 2);

      renderHistory();
      renderStats();
      renderCourtMap();
      renderSensors();
      renderSurveyBoundary(robot.survey || {});
      renderSurveyDiscovery(robot.survey || {});
      renderObstacleSurveyDebug((robot.survey || {}).navigation || {});
      renderObstacleRunsHistory(diagnostics.obstacle_runs || []);
      renderMapMission(robot.map_mission || {});
      updateCommandButtons();

      // Auto-navigate to sensors when mapping mission or survey is active
      const mapMission = robot.map_mission || {};
      if (mapMission.active && !mapMission.complete) {
        const activeView = document.querySelector("section.view.active");
        if (activeView && activeView.id !== "sensors") setView("sensors");
      }
    }
    function renderSurveyBoundary(survey) {
      const statusEl = document.getElementById("surveyBoundaryStatus");
      const kvEl = document.getElementById("surveyBoundaryKv");
      if (!statusEl || !kvEl) return;
      const bounds = survey.bounds;
      const inProgress = survey.state && survey.state !== "done";
      const isComplete = bounds && (bounds.status === "SUCCESS" || bounds.survey_complete);
      if (isComplete) {
        const cg = bounds.court_geometry || {};
        const fg = canonicalFenceBounds(bounds);
        const w  = fg.west_x  != null ? fg.west_x.toFixed(2)  : "—";
        const e  = fg.east_x  != null ? fg.east_x.toFixed(2)  : "—";
        const s  = fg.south_y != null ? fg.south_y.toFixed(2) : "—";
        const n  = fg.north_y != null ? fg.north_y.toFixed(2) : "—";
        const len = cg.length_m != null ? cg.length_m.toFixed(2) : "—";
        const wid = cg.width_m  != null ? cg.width_m.toFixed(2)  : "—";
        statusEl.textContent = `— SUCCESS · ${bounds.sample_count ?? "?"} samples`;
        statusEl.style.color = "var(--accent)";
        setKv("surveyBoundaryKv", [
          ["Court length (E-W)", `${len} m`],
          ["Court width (N-S)", `${wid} m`],
          ["West fence x",  `${w} m`],
          ["East fence x",  `${e} m`],
          ["South fence y", `${s} m`],
          ["North fence y", `${n} m`],
        ]);
      } else if (inProgress) {
        statusEl.textContent = `— mapping · ${survey.sample_count ?? 0} samples`;
        statusEl.style.color = "var(--accent-2)";
        setKv("surveyBoundaryKv", [
          ["State", survey.state || "—"],
          ["Event", (survey.navigation || {}).last_event || "none"],
          ["Distance", fmt((survey.navigation || {}).distance_traveled_m, "m")],
          ["Samples collected", survey.sample_count ?? 0],
        ]);
      } else {
        statusEl.textContent = "no court map data";
        statusEl.style.color = "var(--muted)";
        kvEl.innerHTML = "";
      }
    }
    function renderSurveyDiscovery(survey) {
      const canvas = document.getElementById("surveyDiscoveryMap");
      const statusEl = document.getElementById("surveyDiscoveryStatus");
      if (!canvas) return;
      const ctx = canvas.getContext("2d");
      const nav = survey.navigation || {};
      const points = Array.isArray(nav.map_points) ? nav.map_points : [];
      const bounds = survey.bounds || diagnostics.court_boundary || {};
      const net = nav.net_boundary || (bounds.boundaries || {}).net || (bounds.court_features || {}).net_boundary || bounds.net;
      const width = canvas.width;
      const height = canvas.height;
      const pad = 36;

      ctx.clearRect(0, 0, width, height);
      ctx.fillStyle = "#090d12";
      ctx.fillRect(0, 0, width, height);

      const hasPoints = points.some(p => Number.isFinite(p.x_m) && Number.isFinite(p.y_m));
      const hasNet = !!(net && ((net.post_a && net.post_b) || net.center || net.midpoint));
      if (!hasPoints && !hasNet) {
        ctx.fillStyle = "rgba(145,162,178,0.7)";
        ctx.font = "14px system-ui";
        ctx.fillText("waiting for Map Court LiDAR points", pad, pad + 10);
        if (statusEl) statusEl.textContent = "waiting for points";
        setKv("surveyDiscoveryMeta", [
          ["State", survey.state || "idle"],
          ["Points", nav.map_point_count ?? 0],
          ["Net boundary", net ? "detected" : "none"],
        ]);
        return;
      }

      const fg = canonicalFenceBounds(bounds);
      const worldExtents = nav.scan_coverage?.world_extents || bounds.lidar_boundary_estimate?.combined || {};
      const hasFence = Number.isFinite(fg.west_x) && Number.isFinite(fg.east_x) && Number.isFinite(fg.south_y) && Number.isFinite(fg.north_y);
      const hasExtents = Number.isFinite(worldExtents.min_x_m) && Number.isFinite(worldExtents.max_x_m) && Number.isFinite(worldExtents.min_y_m) && Number.isFinite(worldExtents.max_y_m);
      let minX = hasFence ? fg.west_x - 0.5 : (hasExtents ? worldExtents.min_x_m - 1.0 : -14.0);
      let maxX = hasFence ? fg.east_x + 0.5 : (hasExtents ? worldExtents.max_x_m + 1.0 : 14.0);
      let minY = hasFence ? fg.south_y - 0.5 : (hasExtents ? worldExtents.min_y_m - 1.0 : -7.5);
      let maxY = hasFence ? fg.north_y + 0.5 : (hasExtents ? worldExtents.max_y_m + 1.0 : 7.5);
      const netCenter = net?.center || net?.midpoint || (net?.post_a && net?.post_b ? {
        x_m: (net.post_a.x_m + net.post_b.x_m) / 2,
        y_m: (net.post_a.y_m + net.post_b.y_m) / 2,
      } : null);
      const courtCorners = (() => {
        if (!net?.post_a || !net?.post_b || !netCenter) return [];
        const ax = net.post_a.x_m, ay = net.post_a.y_m;
        const bx = net.post_b.x_m, by = net.post_b.y_m;
        const dx = bx - ax, dy = by - ay;
        const postSpan = Math.hypot(dx, dy);
        if (!Number.isFinite(postSpan) || postSpan < 0.5) return [];
        const tx = dx / postSpan, ty = dy / postSpan;
        const nx = -ty, ny = tx;
        const courtWidth = Number.isFinite(net.length_m) && net.length_m > 6 ? net.length_m : 10.97;
        const halfW = courtWidth / 2;
        const halfL = 23.77 / 2;
        return [
          { x_m: netCenter.x_m + nx * halfL + tx * halfW, y_m: netCenter.y_m + ny * halfL + ty * halfW },
          { x_m: netCenter.x_m + nx * halfL - tx * halfW, y_m: netCenter.y_m + ny * halfL - ty * halfW },
          { x_m: netCenter.x_m - nx * halfL - tx * halfW, y_m: netCenter.y_m - ny * halfL - ty * halfW },
          { x_m: netCenter.x_m - nx * halfL + tx * halfW, y_m: netCenter.y_m - ny * halfL + ty * halfW },
        ];
      })();
      [...points, net?.post_a, net?.post_b, netCenter, ...courtCorners, (diagnostics.robot || {}).robot]
        .filter(Boolean)
        .forEach(p => {
          if (!Number.isFinite(p.x_m) || !Number.isFinite(p.y_m)) return;
          minX = Math.min(minX, p.x_m - 0.5);
          maxX = Math.max(maxX, p.x_m + 0.5);
          minY = Math.min(minY, p.y_m - 0.5);
          maxY = Math.max(maxY, p.y_m + 0.5);
        });
      const spanX = Math.max(1, maxX - minX);
      const spanY = Math.max(1, maxY - minY);
      const scale = Math.min((width - pad * 2) / spanX, (height - pad * 2) / spanY);
      const ox = (width - spanX * scale) / 2;
      const oy = (height - spanY * scale) / 2;
      const sx = x => ox + (x - minX) * scale;
      const sy = y => height - (oy + (y - minY) * scale);

      ctx.strokeStyle = "rgba(255,255,255,0.07)";
      ctx.lineWidth = 1;
      for (let gx = Math.ceil(minX); gx <= Math.floor(maxX); gx += 1) {
        ctx.beginPath(); ctx.moveTo(sx(gx), sy(minY)); ctx.lineTo(sx(gx), sy(maxY)); ctx.stroke();
      }
      for (let gy = Math.ceil(minY); gy <= Math.floor(maxY); gy += 1) {
        ctx.beginPath(); ctx.moveTo(sx(minX), sy(gy)); ctx.lineTo(sx(maxX), sy(gy)); ctx.stroke();
      }

      if (hasFence || hasExtents) {
        const bx0 = hasFence ? fg.west_x : worldExtents.min_x_m;
        const bx1 = hasFence ? fg.east_x : worldExtents.max_x_m;
        const by0 = hasFence ? fg.south_y : worldExtents.min_y_m;
        const by1 = hasFence ? fg.north_y : worldExtents.max_y_m;
        ctx.strokeStyle = hasFence ? "rgba(255,255,255,0.38)" : "rgba(47,208,143,0.42)";
        ctx.lineWidth = 2;
        ctx.setLineDash(hasFence ? [] : [8, 6]);
        ctx.strokeRect(sx(bx0), sy(by1), sx(bx1) - sx(bx0), sy(by0) - sy(by1));
        ctx.setLineDash([]);
      }
      if (courtCorners.length === 4) {
        ctx.strokeStyle = "rgba(255,255,255,0.22)";
        ctx.lineWidth = 2;
        ctx.beginPath();
        courtCorners.forEach((p, i) => {
          if (i === 0) ctx.moveTo(sx(p.x_m), sy(p.y_m));
          else ctx.lineTo(sx(p.x_m), sy(p.y_m));
        });
        ctx.closePath();
        ctx.stroke();
      }
      ctx.fillStyle = "rgba(145,162,178,0.8)";
      ctx.font = "12px system-ui";
      ctx.fillText("world boundary map", pad, 20);

      ctx.fillStyle = "rgba(47,208,143,0.55)";
      points.forEach(p => {
        if (!Number.isFinite(p.x_m) || !Number.isFinite(p.y_m)) return;
        ctx.fillRect(sx(p.x_m) - 1.2, sy(p.y_m) - 1.2, 2.4, 2.4);
      });

      if (net && net.post_a && net.post_b) {
        const a = net.post_a;
        const b = net.post_b;
        ctx.strokeStyle = "rgba(80,220,255,0.95)";
        ctx.lineWidth = 4;
        ctx.beginPath();
        ctx.moveTo(sx(a.x_m), sy(a.y_m));
        ctx.lineTo(sx(b.x_m), sy(b.y_m));
        ctx.stroke();
        ctx.fillStyle = "#50dcff";
        [[a, "post A"], [b, "post B"]].forEach(([p, label]) => {
          ctx.beginPath();
          ctx.arc(sx(p.x_m), sy(p.y_m), 5, 0, Math.PI * 2);
          ctx.fill();
          ctx.fillText(label, sx(p.x_m) + 7, sy(p.y_m) - 7);
        });
        ctx.font = "bold 13px system-ui";
        const c = netCenter || { x_m: (a.x_m + b.x_m) / 2, y_m: (a.y_m + b.y_m) / 2 };
        const clearance = net.front_clearance_m ?? net.distance_m;
        ctx.fillText(`NET ${fmt(net.length_m, "m")} · clearance ${fmt(clearance, "m")}`, sx(c.x_m) + 8, sy(c.y_m) - 8);
      } else if (netCenter) {
        const c = netCenter;
        ctx.fillStyle = "#50dcff";
        ctx.beginPath();
        ctx.arc(sx(c.x_m), sy(c.y_m), 6, 0, Math.PI * 2);
        ctx.fill();
        ctx.font = "bold 13px system-ui";
        ctx.fillText(`NET ${fmt(net.distance_m, "m")}`, sx(c.x_m) + 8, sy(c.y_m) - 8);
      }

      const robot = (diagnostics.robot || {}).robot || {};
      if (Number.isFinite(robot.x_m) && Number.isFinite(robot.y_m)) {
        ctx.fillStyle = "#ffbd5a";
        ctx.beginPath();
        ctx.arc(sx(robot.x_m), sy(robot.y_m), 5, 0, Math.PI * 2);
        ctx.fill();
        ctx.fillText("robot", sx(robot.x_m) + 8, sy(robot.y_m) + 4);
      }

      if (statusEl) {
        const pointCount = nav.map_point_count ?? points.length;
        statusEl.textContent = `${pointCount} world points · net ${net ? "localized" : "not fitted"}`;
        statusEl.style.color = net ? "var(--accent)" : "var(--warn)";
      }
      setKv("surveyDiscoveryMeta", [
        ["State", survey.state || "idle"],
        ["Event", nav.last_event || "none"],
        ["Points", nav.map_point_count ?? points.length],
        ["Shown sample", points.length],
        ["Frame", "world/court coordinates"],
        ["Sensor frame", nav.sensor_frame || "none"],
        ["LiDAR coverage", `F ${fmt(nav.scan_coverage?.front_m, "m")} · R ${fmt(nav.scan_coverage?.rear_m, "m")} · L ${fmt(nav.scan_coverage?.left_m, "m")} · Rt ${fmt(nav.scan_coverage?.right_m, "m")}`],
        ["World extents", nav.scan_coverage?.world_extents ? `x ${fmt(nav.scan_coverage.world_extents.min_x_m, "m")}..${fmt(nav.scan_coverage.world_extents.max_x_m, "m")} · y ${fmt(nav.scan_coverage.world_extents.min_y_m, "m")}..${fmt(nav.scan_coverage.world_extents.max_y_m, "m")}` : "none"],
        ["Net boundary", net ? `${fmt(net.length_m, "m")} @ front ${fmt(net.front_clearance_m ?? net.distance_m, "m")}` : "not fitted"],
        ["Net source", net?.source || nav.net_boundary_source || "none"],
      ]);
    }
    function renderSurveyDiscovery(survey) {
      const canvas = document.getElementById("surveyDiscoveryMap");
      const statusEl = document.getElementById("surveyDiscoveryStatus");
      const metaEl = document.getElementById("surveyDiscoveryMeta");
      if (!canvas) return;
      const ctx = canvas.getContext("2d");
      const liveNav = survey.navigation || {};
      const liveBounds = survey.bounds || {};
      const persistedBounds = diagnostics.court_boundary || {};
      const livePoints = Array.isArray(liveNav.map_points) ? liveNav.map_points : [];
      const persistedPoints = Array.isArray(persistedBounds.point_cloud_sample) ? persistedBounds.point_cloud_sample : [];
      const liveNet = liveNav.net_boundary || liveBounds.net || (liveBounds.boundaries || {}).net || (liveBounds.court_features || {}).net_boundary;
      const persistedNet = persistedBounds.net || (persistedBounds.boundaries || {}).net || (persistedBounds.court_features || {}).net_boundary;
      const hasLiveDiscovery = livePoints.length > 0 || !!liveNet;
      const hasPersistedDiscovery = persistedBounds.survey_complete && (persistedPoints.length > 0 || !!persistedNet);
      const source = hasLiveDiscovery ? {
        nav: liveNav,
        bounds: liveBounds,
        points: livePoints,
        net: liveNet,
        sourceLabel: "live"
      } : (lastSurveyDiscovery || (hasPersistedDiscovery ? {
        nav: {},
        bounds: persistedBounds,
        points: persistedPoints,
        net: persistedNet,
        sourceLabel: "saved"
      } : {
        nav: liveNav,
        bounds: liveBounds.survey_complete ? liveBounds : {},
        points: [],
        net: null,
        sourceLabel: "expected"
      }));
      if (hasLiveDiscovery) {
        lastSurveyDiscovery = source;
      }
      const nav = source.nav || {};
      const bounds = source.bounds || {};
      const net = source.net;
      const points = source.points || [];
      const width = canvas.width;
      const height = canvas.height;
      const plotW = width - 270;
      const pad = 52;
      const robot = (diagnostics.robot || {}).robot || {};
      const fg = canonicalFenceBounds(bounds);
      const worldExtents = nav.scan_coverage?.world_extents || bounds.lidar_boundary_estimate?.combined || {};
      const hasFence = Number.isFinite(fg.west_x) && Number.isFinite(fg.east_x) && Number.isFinite(fg.south_y) && Number.isFinite(fg.north_y);
      const hasExtents = Number.isFinite(worldExtents.min_x_m) && Number.isFinite(worldExtents.max_x_m) && Number.isFinite(worldExtents.min_y_m) && Number.isFinite(worldExtents.max_y_m);
      const netCenter = net?.center || net?.midpoint || (net?.post_a && net?.post_b ? {
        x_m: (net.post_a.x_m + net.post_b.x_m) / 2,
        y_m: (net.post_a.y_m + net.post_b.y_m) / 2,
      } : null);
      const courtGeometry = bounds.court_geometry || {};
      const courtLength = Number.isFinite(courtGeometry.length_m) ? courtGeometry.length_m : 23.77;
      const courtWidth = Number.isFinite(courtGeometry.width_m) ? courtGeometry.width_m : (
        Number.isFinite(net?.length_m) && net.length_m > 6 ? net.length_m : 10.97
      );
      const extSpanX = hasExtents ? worldExtents.max_x_m - worldExtents.min_x_m : 0;
      const extSpanY = hasExtents ? worldExtents.max_y_m - worldExtents.min_y_m : 0;
      const extentsCanContainCourt = (
        (extSpanX >= courtLength && extSpanY >= courtWidth)
        || (extSpanY >= courtLength && extSpanX >= courtWidth)
      );
      let fenceRect = hasFence ? {
        minX: fg.west_x, maxX: fg.east_x, minY: fg.south_y, maxY: fg.north_y, source: "verified"
      } : (hasExtents && extentsCanContainCourt ? {
        minX: worldExtents.min_x_m, maxX: worldExtents.max_x_m, minY: worldExtents.min_y_m, maxY: worldExtents.max_y_m, source: "lidar boundary estimate"
      } : {
        minX: -16.5, maxX: 16.5, minY: -8.5, maxY: 8.5, source: hasExtents ? "expected bounds + live scan" : "expected Gazebo bounds"
      });

      // Override fenceRect with measured two-point survey geometry.
      const twoPointGeo = persistedBounds.geometry;
      if (persistedBounds.survey_type === "two_point_net_baseline" && twoPointGeo) {
        const netP   = twoPointGeo.net_world_pos;
        const fenceP = twoPointGeo.fence_world_pos;
        const btf    = twoPointGeo.baseline_to_fence_m;
        if (netP && fenceP && Number.isFinite(btf)) {
          const dx = fenceP.x_m - netP.x_m;
          const dy = fenceP.y_m - netP.y_m;
          const surveyDist = Math.hypot(dx, dy);
          const ux = dx / surveyDist, uy = dy / surveyDist; // unit vector net→fence
          const px = -uy, py = ux;                          // perpendicular
          const halfLen = courtLength / 2;
          const halfWid = courtWidth / 2;
          // Unknown side margin: use current fenceRect or Gazebo default.
          const knownSideMargin = Math.abs(px) > 0.5
            ? (fenceRect.maxX - fenceRect.minX) / 2 - halfLen
            : (fenceRect.maxY - fenceRect.minY) / 2 - halfWid;
          const sideMargin = Number.isFinite(knownSideMargin) && knownSideMargin > 0
            ? knownSideMargin : 3.01;
          // Build 4 corner points of the outer fence rectangle.
          const corners = [
            [netP.x_m + ux*(halfLen+btf) + px*(halfWid+sideMargin), netP.y_m + uy*(halfLen+btf) + py*(halfWid+sideMargin)],
            [netP.x_m + ux*(halfLen+btf) - px*(halfWid+sideMargin), netP.y_m + uy*(halfLen+btf) - py*(halfWid+sideMargin)],
            [netP.x_m - ux*(halfLen+btf) + px*(halfWid+sideMargin), netP.y_m - uy*(halfLen+btf) + py*(halfWid+sideMargin)],
            [netP.x_m - ux*(halfLen+btf) - px*(halfWid+sideMargin), netP.y_m - uy*(halfLen+btf) - py*(halfWid+sideMargin)],
          ];
          fenceRect = {
            minX: Math.min(...corners.map(c => c[0])),
            maxX: Math.max(...corners.map(c => c[0])),
            minY: Math.min(...corners.map(c => c[1])),
            maxY: Math.max(...corners.map(c => c[1])),
            source: "two_point_survey",
          };
        }
      }
      const axisAlignedFromFence = (
        fenceRect.source !== "verified"
        || !net?.post_a
        || !net?.post_b
        || Math.abs(fenceRect.maxX - fenceRect.minX) < Math.abs(fenceRect.maxY - fenceRect.minY)
      );
      const courtCorners = (() => {
        if (axisAlignedFromFence) return [];
        if (!net?.post_a || !net?.post_b || !netCenter) return [];
        const ax = net.post_a.x_m, ay = net.post_a.y_m;
        const bx = net.post_b.x_m, by = net.post_b.y_m;
        const dx = bx - ax, dy = by - ay;
        const span = Math.hypot(dx, dy);
        if (!Number.isFinite(span) || span < 0.5) return [];
        const tx = dx / span, ty = dy / span;
        const nx = -ty, ny = tx;
        const halfL = courtLength / 2;
        const halfW = courtWidth / 2;
        return [
          { x_m: netCenter.x_m + nx * halfL + tx * halfW, y_m: netCenter.y_m + ny * halfL + ty * halfW },
          { x_m: netCenter.x_m + nx * halfL - tx * halfW, y_m: netCenter.y_m + ny * halfL - ty * halfW },
          { x_m: netCenter.x_m - nx * halfL - tx * halfW, y_m: netCenter.y_m - ny * halfL - ty * halfW },
          { x_m: netCenter.x_m - nx * halfL + tx * halfW, y_m: netCenter.y_m - ny * halfL + ty * halfW },
        ];
      })();
      const fallbackCourtCorners = [
        { x_m: -courtLength / 2, y_m: courtWidth / 2 },
        { x_m: courtLength / 2, y_m: courtWidth / 2 },
        { x_m: courtLength / 2, y_m: -courtWidth / 2 },
        { x_m: -courtLength / 2, y_m: -courtWidth / 2 },
      ];
      const displayCourtCorners = courtCorners.length === 4 ? courtCorners : fallbackCourtCorners;
      const courtRect = displayCourtCorners.length === 4 ? {
        minX: Math.min(...displayCourtCorners.map(p => p.x_m)),
        maxX: Math.max(...displayCourtCorners.map(p => p.x_m)),
        minY: Math.min(...displayCourtCorners.map(p => p.y_m)),
        maxY: Math.max(...displayCourtCorners.map(p => p.y_m)),
      } : {
        minX: -courtLength / 2, maxX: courtLength / 2,
        minY: -courtWidth / 2, maxY: courtWidth / 2,
      };
      const margin = fenceRect ? {
        top: fenceRect.maxY - courtRect.maxY,
        bottom: courtRect.minY - fenceRect.minY,
        left: courtRect.minX - fenceRect.minX,
        right: fenceRect.maxX - courtRect.maxX,
      } : {};
      const marginValues = Object.values(margin).filter(Number.isFinite);
      const validPoints = points.filter(p => Number.isFinite(p.x_m) && Number.isFinite(p.y_m));
      let minX = fenceRect ? fenceRect.minX - 0.7 : courtRect.minX - 1.0;
      let maxX = fenceRect ? fenceRect.maxX + 0.7 : courtRect.maxX + 1.0;
      let minY = fenceRect ? fenceRect.minY - 0.7 : courtRect.minY - 1.0;
      let maxY = fenceRect ? fenceRect.maxY + 0.7 : courtRect.maxY + 1.0;
      [...validPoints, net?.post_a, net?.post_b, netCenter, ...displayCourtCorners, robot].filter(Boolean).forEach(p => {
        if (!Number.isFinite(p.x_m) || !Number.isFinite(p.y_m)) return;
        minX = Math.min(minX, p.x_m - 0.5);
        maxX = Math.max(maxX, p.x_m + 0.5);
        minY = Math.min(minY, p.y_m - 0.5);
        maxY = Math.max(maxY, p.y_m + 0.5);
      });
      const spanX = Math.max(1, maxX - minX);
      const spanY = Math.max(1, maxY - minY);
      const scale = Math.min((plotW - pad * 2) / spanX, (height - pad * 2) / spanY);
      const ox = (plotW - spanX * scale) / 2;
      const oy = (height - spanY * scale) / 2;
      const sx = x => ox + (x - minX) * scale;
      const sy = y => height - (oy + (y - minY) * scale);

      ctx.clearRect(0, 0, width, height);
      ctx.fillStyle = "#090d12";
      ctx.fillRect(0, 0, width, height);
      ctx.strokeStyle = "rgba(255,255,255,0.07)";
      ctx.lineWidth = 1;
      for (let gx = Math.ceil(minX); gx <= Math.floor(maxX); gx += 1) {
        ctx.beginPath(); ctx.moveTo(sx(gx), sy(minY)); ctx.lineTo(sx(gx), sy(maxY)); ctx.stroke();
      }
      for (let gy = Math.ceil(minY); gy <= Math.floor(maxY); gy += 1) {
        ctx.beginPath(); ctx.moveTo(sx(minX), sy(gy)); ctx.lineTo(sx(maxX), sy(gy)); ctx.stroke();
      }

      ctx.fillStyle = "rgba(145,162,178,0.28)";
      validPoints.forEach(p => ctx.fillRect(sx(p.x_m) - 1, sy(p.y_m) - 1, 2, 2));
      if (fenceRect) {
        ctx.strokeStyle = "rgba(168,85,247,0.92)";
        ctx.lineWidth = 3;
        ctx.setLineDash(fenceRect.source === "verified" ? [] : [8, 6]);
        ctx.strokeRect(sx(fenceRect.minX), sy(fenceRect.maxY), sx(fenceRect.maxX) - sx(fenceRect.minX), sy(fenceRect.minY) - sy(fenceRect.maxY));
        ctx.setLineDash([]);
      }
      if (displayCourtCorners.length === 4) {
        ctx.fillStyle = "rgba(47,208,143,0.06)";
        ctx.beginPath();
        displayCourtCorners.forEach((p, i) => i === 0 ? ctx.moveTo(sx(p.x_m), sy(p.y_m)) : ctx.lineTo(sx(p.x_m), sy(p.y_m)));
        ctx.closePath();
        ctx.fill();
        ctx.strokeStyle = courtCorners.length === 4 ? "rgba(47,208,143,0.9)" : "rgba(47,208,143,0.48)";
        ctx.lineWidth = 2.5;
        ctx.setLineDash(courtCorners.length === 4 ? [] : [7, 5]);
        ctx.stroke();
        ctx.setLineDash([]);
      }
      if (net?.post_a && net?.post_b) {
        ctx.strokeStyle = "rgba(80,220,255,0.95)";
        ctx.lineWidth = 4;
        ctx.beginPath();
        ctx.moveTo(sx(net.post_a.x_m), sy(net.post_a.y_m));
        ctx.lineTo(sx(net.post_b.x_m), sy(net.post_b.y_m));
        ctx.stroke();
      }

      function drawDistance(x1, y1, x2, y2, label) {
        if (![x1, y1, x2, y2].every(Number.isFinite)) return;
        ctx.strokeStyle = "rgba(255,189,90,0.92)";
        ctx.fillStyle = "#ffbd5a";
        ctx.lineWidth = 1.5;
        ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();
        ctx.font = "bold 13px system-ui";
        ctx.fillText(label, (x1 + x2) / 2 + 6, (y1 + y2) / 2 - 6);
      }
      if (fenceRect) {
        const midX = (courtRect.minX + courtRect.maxX) / 2;
        const midY = (courtRect.minY + courtRect.maxY) / 2;
        drawDistance(sx(fenceRect.minX), sy(midY), sx(courtRect.minX), sy(midY), fmt(margin.left, "m"));
        drawDistance(sx(courtRect.maxX), sy(midY), sx(fenceRect.maxX), sy(midY), fmt(margin.right, "m"));
        drawDistance(sx(midX), sy(courtRect.maxY), sx(midX), sy(fenceRect.maxY), fmt(margin.top, "m"));
        drawDistance(sx(midX), sy(fenceRect.minY), sx(midX), sy(courtRect.minY), fmt(margin.bottom, "m"));
      }
      if (Number.isFinite(robot.x_m) && Number.isFinite(robot.y_m)) {
        const rx = sx(robot.x_m), ry = sy(robot.y_m);
        ctx.fillStyle = "rgba(87,166,255,0.15)";
        ctx.beginPath(); ctx.arc(rx, ry, 28, 0, Math.PI * 2); ctx.fill();
        ctx.fillStyle = "#57a6ff";
        ctx.beginPath(); ctx.arc(rx, ry, 6, 0, Math.PI * 2); ctx.fill();
      }

      ctx.fillStyle = "rgba(238,244,248,0.9)";
      ctx.font = "bold 13px system-ui";
      ctx.fillText("Court Map (Top-Down Occupancy Grid)", 18, 24);
      const legend = [["#57a6ff", "LiDAR Points"], ["#a855f7", "Outer Boundary"], ["#2fd08f", "Court Boundary"], ["#ffbd5a", "Distances"], ["#57a6ff", "Robot Pose"]];
      let lx = 24;
      ctx.font = "11px system-ui";
      legend.forEach(([color, label]) => {
        ctx.fillStyle = color; ctx.fillRect(lx, 45, 10, 10);
        ctx.fillStyle = "rgba(238,244,248,0.82)"; ctx.fillText(label, lx + 16, 54);
        lx += ctx.measureText(label).width + 42;
      });

      const panelX = plotW + 12;
      ctx.fillStyle = "rgba(10,18,26,0.76)";
      ctx.strokeStyle = "rgba(255,255,255,0.10)";
      ctx.beginPath(); ctx.roundRect(panelX, 78, width - panelX - 18, 260, 8); ctx.fill(); ctx.stroke();
      ctx.fillStyle = "rgba(238,244,248,0.92)";
      ctx.font = "bold 12px system-ui";
      ctx.fillText("Distances to Fences", panelX + 16, 104);
      const distRows = [
        ["Top", margin.top], ["Bottom", margin.bottom], ["Left", margin.left], ["Right", margin.right],
        ["Min Distance", marginValues.length ? Math.min(...marginValues) : null],
        ["Max Distance", marginValues.length ? Math.max(...marginValues) : null],
        ["Avg Distance", marginValues.length ? marginValues.reduce((a, b) => a + b, 0) / marginValues.length : null],
      ];
      let rowY = 136;
      distRows.forEach(([label, value], idx) => {
        if (idx === 4) rowY += 16;
        ctx.fillStyle = "rgba(145,162,178,0.9)"; ctx.font = "11px system-ui"; ctx.fillText(label, panelX + 16, rowY);
        ctx.fillStyle = "rgba(238,244,248,0.96)"; ctx.font = "bold 11px system-ui"; ctx.fillText(fmt(value, "m"), panelX + 120, rowY);
        rowY += 28;
      });

      if (statusEl) {
        const pointCount = nav.map_point_count ?? validPoints.length;
        statusEl.textContent = `${pointCount} LiDAR points - ${fenceRect.source} - ${courtCorners.length === 4 ? "court boundary fitted" : "court expected"}`;
        statusEl.style.color = fenceRect.source === "verified" && courtCorners.length === 4 ? "var(--accent)" : "var(--warn)";
      }
      const fenceLength = fenceRect ? fenceRect.maxX - fenceRect.minX : null;
      const fenceWidth = fenceRect ? fenceRect.maxY - fenceRect.minY : null;
      const segments = [
        ["F-Top", fenceLength, "Fence", fenceRect.source === "verified"],
        ["F-Bottom", fenceLength, "Fence", fenceRect.source === "verified"],
        ["F-Left", fenceWidth, "Fence", fenceRect.source === "verified"],
        ["F-Right", fenceWidth, "Fence", fenceRect.source === "verified"],
        ["C-Baseline Far", courtWidth, "Court", courtCorners.length === 4],
        ["C-Baseline Near", courtWidth, "Court", courtCorners.length === 4],
        ["C-Side Left", courtLength, "Court", courtCorners.length === 4],
        ["C-Side Right", courtLength, "Court", courtCorners.length === 4],
      ];
      if (metaEl) {
        metaEl.innerHTML = `
          <div><span>Outer Boundary</span><strong>${fenceRect ? `${fmt(fenceLength, "m")} x ${fmt(fenceWidth, "m")}` : "pending"}</strong></div>
          <div><span>Court Boundary</span><strong>${fmt(courtLength, "m")} x ${fmt(courtWidth, "m")}</strong></div>
          <div><span>Boundary Source</span><strong>${fenceRect?.source || "none"}</strong></div>
          <div><span>Confidence</span><strong>${fmt((bounds.diagnostics || {}).confidence, "")}</strong></div>
          <div style="grid-column:1 / -1;display:block;">
            <div style="display:grid;grid-template-columns:1fr 1fr 1fr 1fr;gap:8px 14px;align-items:center;">
              <strong>ID</strong><strong>Length</strong><strong>Type</strong><strong>Status</strong>
              ${segments.map(([id, len, type, ok]) => `
                <span>${id}</span><span>${fmt(len, "m")}</span><span>${type}</span>
                <span style="color:${ok ? "var(--accent)" : "var(--muted)"}">${ok ? "Verified" : "Pending"}</span>
              `).join("")}
            </div>
          </div>
        `;
      }
    }
    const _obsLogLines = [];
    const OBS_LOG_MAX = 200;

    function renderObstacleSurveyDebug(nav) {
      const obs = nav.obstacle_survey;
      const stateEl = document.getElementById("obsState");
      const kvEl   = document.getElementById("obsKv");
      const logEl  = document.getElementById("obsLog");
      if (!stateEl || !kvEl || !logEl) return;

      if (!obs) {
        stateEl.textContent = "idle";
        stateEl.style.color = "var(--muted)";
        return;
      }

      const stateColor = {
        scan_in_place: "var(--accent-2)",
        approach_obstacle: "var(--warn)",
        done: obs.failure_reason ? "var(--danger)" : "var(--accent)",
      }[obs.state] || "var(--muted)";
      stateEl.textContent = obs.state || "idle";
      stateEl.style.color = stateColor;

      const fr = obs.front_range_m != null ? obs.front_range_m + " m" : "—";
      const nearest = (obs.debug_log && obs.debug_log.length)
        ? (obs.debug_log[obs.debug_log.length - 1].nearest_m ?? "—") + " m" : "—";
      setKv("obsKv", [
        ["State",          obs.state || "—"],
        ["Event",          obs.last_event || "—"],
        ["Front range",    fr],
        ["Nearest (360°)", nearest],
        ["Scans",          obs.scan_count ?? "—"],
        ["Traveled",       (obs.distance_traveled_m ?? "—") + " m"],
        ["Elapsed",        (obs.elapsed_s ?? "—") + " s"],
        ["Obstacle",       obs.obstacle_type ?? "—"],
        ["Confidence",     obs.obstacle_confidence != null ? (obs.obstacle_confidence * 100).toFixed(0) + "%" : "—"],
        ["Failure",        obs.failure_reason || "—"],
      ]);

      // Append new log entries (avoid duplicates by tracking last t seen)
      const entries = Array.isArray(obs.debug_log) ? obs.debug_log : [];
      const lastSeen = _obsLogLines.length ? _obsLogLines[_obsLogLines.length - 1]._t : -1;
      let newAdded = false;
      for (const e of entries) {
        if (e.t > lastSeen) {
          const vis = e.vision ? ` vis:${e.vision}` : "";
          const tgt = e.target_heading_deg != null ? ` tgt:${e.target_heading_deg}°` : "";
          const err = e.heading_err_deg != null ? ` err:${e.heading_err_deg}°` : "";
          const front = e.front_m != null ? e.front_m : "inf";
          const near  = e.nearest_m != null ? e.nearest_m : "inf";
          const line = `[${e.t.toFixed(2).padStart(6)}s] ${e.state.padEnd(20)} ${e.decision.padEnd(24)} front:${String(front).padStart(5)}m near:${String(near).padStart(5)}m bear:${String(e.bearing_deg).padStart(6)}°${tgt}${err} cmd:(${e.cmd_lin.toFixed(2)},${e.cmd_ang.toFixed(2)})${vis}`;
          _obsLogLines.push({ _t: e.t, text: line });
          newAdded = true;
        }
      }
      if (_obsLogLines.length > OBS_LOG_MAX) _obsLogLines.splice(0, _obsLogLines.length - OBS_LOG_MAX);
      if (newAdded) {
        logEl.textContent = _obsLogLines.map(l => l.text).join("\\n");
        logEl.scrollTop = logEl.scrollHeight;
      }
    }

    function renderObstacleRunsHistory(runs) {
      const el = document.getElementById("obsRunsTable");
      if (!el || !Array.isArray(runs) || runs.length === 0) return;
      const statusColor = s => s === "SUCCESS" ? "var(--accent)" : "var(--danger)";
      const fmtTime = ts => ts ? new Date(ts * 1000).toLocaleTimeString() : "—";
      let html = `<table style="width:100%;border-collapse:collapse;font-size:11px;">
        <thead><tr style="color:var(--muted);text-align:left;border-bottom:1px solid var(--line);">
          <th style="padding:3px 6px;">#</th>
          <th style="padding:3px 6px;">Time</th>
          <th style="padding:3px 6px;">Status</th>
          <th style="padding:3px 6px;">Obstacle</th>
          <th style="padding:3px 6px;">Dist</th>
          <th style="padding:3px 6px;">Elapsed</th>
          <th style="padding:3px 6px;">Traveled</th>
          <th style="padding:3px 6px;">Failure</th>
        </tr></thead><tbody>`;
      for (const r of runs) {
        html += `<tr style="border-bottom:1px solid var(--line)20;">
          <td style="padding:3px 6px;color:var(--muted);">${r.id}</td>
          <td style="padding:3px 6px;">${fmtTime(r.started_at)}</td>
          <td style="padding:3px 6px;color:${statusColor(r.status)};">${r.status || "—"}</td>
          <td style="padding:3px 6px;">${r.obstacle_type || "—"}</td>
          <td style="padding:3px 6px;">${r.obstacle_dist_m != null ? r.obstacle_dist_m.toFixed(2) + "m" : "—"}</td>
          <td style="padding:3px 6px;">${r.elapsed_s != null ? r.elapsed_s.toFixed(1) + "s" : "—"}</td>
          <td style="padding:3px 6px;">${r.distance_traveled_m != null ? r.distance_traveled_m.toFixed(1) + "m" : "—"}</td>
          <td style="padding:3px 6px;color:var(--danger);">${r.failure_reason || ""}</td>
        </tr>`;
      }
      html += "</tbody></table>";
      el.innerHTML = html;
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
      // LiDAR: index 0 = backward (-x), index n/2 = forward (+x), scan CW from above
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

      // Survey fence measurements overlay — prefer persistent court_boundary.json over in-memory survey state
      const survBounds = diagnostics.court_boundary || ((diagnostics.robot || {}).survey || {}).bounds;
      if (survBounds && survBounds.survey_complete) {
        const fg = canonicalFenceBounds(survBounds);
        ctx.save();
        ctx.setLineDash([7, 4]);
        ctx.lineWidth = 1.8;
        ctx.strokeStyle = "rgba(255,189,90,0.75)";
        ctx.fillStyle = "rgba(255,189,90,0.92)";
        ctx.font = "bold 11px system-ui";

        if (fg.west_x != null) {
          const fx = sx(fg.west_x);
          ctx.beginPath(); ctx.moveTo(fx, pad); ctx.lineTo(fx, height - pad); ctx.stroke();
          ctx.textAlign = "right";
          ctx.fillText(`W ${fg.west_x.toFixed(1)}m`, fx - 3, pad + 20);
        }
        if (fg.east_x != null) {
          const fx = sx(fg.east_x);
          ctx.beginPath(); ctx.moveTo(fx, pad); ctx.lineTo(fx, height - pad); ctx.stroke();
          ctx.textAlign = "left";
          ctx.fillText(`E ${fg.east_x.toFixed(1)}m`, fx + 3, pad + 20);
        }
        if (fg.south_y != null) {
          const fy = sy(fg.south_y);
          ctx.beginPath(); ctx.moveTo(pad, fy); ctx.lineTo(width - pad, fy); ctx.stroke();
          ctx.textAlign = "left";
          ctx.fillText(`S ${fg.south_y.toFixed(1)}m`, pad + 6, fy - 4);
        }
        if (fg.north_y != null) {
          const fy = sy(fg.north_y);
          ctx.beginPath(); ctx.moveTo(pad, fy); ctx.lineTo(width - pad, fy); ctx.stroke();
          ctx.textAlign = "left";
          ctx.fillText(`N ${fg.north_y.toFixed(1)}m`, pad + 6, fy + 13);
        }

        // Width × depth badge bottom-right
        if (fg.east_x != null && fg.west_x != null && fg.north_y != null && fg.south_y != null) {
          const w = (fg.east_x - fg.west_x).toFixed(1);
          const d = (fg.north_y - fg.south_y).toFixed(1);
          const label = `map court: ${w} × ${d} m`;
          ctx.setLineDash([]);
          ctx.textAlign = "right";
          const lw = ctx.measureText(label).width;
          ctx.fillStyle = "rgba(12,17,22,0.72)";
          ctx.fillRect(width - pad - lw - 16, height - pad - 26, lw + 14, 20);
          ctx.fillStyle = "rgba(255,189,90,0.92)";
          ctx.fillText(label, width - pad - 4, height - pad - 11);
        }

        ctx.setLineDash([]);
        ctx.restore();
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
      document.getElementById("sSurvey").textContent = byMode.map_court || 0;
      document.getElementById("sCollect").textContent = byMode.map_left_side || 0;
      document.getElementById("sIdle").textContent = byMode.idle || 0;
      document.getElementById("statsRows").innerHTML = ["map_court", "map_left_side", "idle"].map(mode => {
        const count = byMode[mode] || 0;
        const latest = (stats.latest_by_mode || {})[mode] || {};
        const share = total ? `${Math.round(count * 100 / total)}%` : "0%";
        return `<tr><td>${mode}</td><td>${count}</td><td>${share}</td><td>${latest.sequence ?? "none"}</td><td>${latest.source ?? "none"}</td></tr>`;
      }).join("");
    }
    refresh();
    setInterval(refresh, 1000);

    // ── Vendors ───────────────────────────────────────────────────────────────
    let vendorData = { vendors: [], courts: [], active: {} };

    function escHtml(s) {
      return String(s).replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;").replace(/"/g, "&quot;");
    }

    async function loadVendors() {
      try {
        const res = await fetch("/api/vendors", { cache: "no-store" });
        vendorData = await res.json();
      } catch (_) {}
      renderVendors();
    }

    async function saveVendors(data) {
      try {
        await fetch("/api/vendors", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify(data)
        });
        vendorData = data;
      } catch (_) {}
      renderVendors();
    }

    function renderVendors() {
      const { vendors, courts, active } = vendorData;

      // Sidebar session indicator
      const sidebarEl = document.getElementById("activeSessionSidebar");
      if (sidebarEl) {
        if (active && active.vendor_id) {
          const v = vendors.find(x => x.id === active.vendor_id);
          const c = courts.find(x => x.id === active.court_id);
          sidebarEl.textContent = `Session: ${v ? v.name : "?"} · ${c ? c.name : "?"}`;
          sidebarEl.style.color = "var(--accent)";
        } else {
          sidebarEl.textContent = "Session: none";
          sidebarEl.style.color = "var(--muted)";
        }
      }

      // Active session display banner
      const displayEl = document.getElementById("activeSessionDisplay");
      if (displayEl) {
        if (active && active.vendor_id) {
          const v = vendors.find(x => x.id === active.vendor_id);
          const c = courts.find(x => x.id === active.court_id);
          displayEl.innerHTML = `
            <strong style="color:var(--accent);font-size:15px;">${escHtml(v ? v.name : "Unknown vendor")}</strong>
            <span style="color:var(--muted);margin:0 8px;">/</span>
            <span style="color:var(--ink);">${escHtml(c ? c.name : "Unknown court")}</span>
            ${c && c.surface ? `<span style="color:var(--muted);font-size:12px;margin-left:10px;">${escHtml(c.surface)}</span>` : ""}
            ${v && v.address ? `<div style="color:var(--muted);font-size:12px;margin-top:4px;">${escHtml(v.address)}</div>` : ""}
          `;
        } else {
          displayEl.innerHTML = `<span style="color:var(--muted);font-size:13px;">No active session — select a vendor and court below.</span>`;
        }
      }

      // Vendor dropdowns (activeVendorSelect + courtVendorSelect)
      ["activeVendorSelect", "courtVendorSelect"].forEach(id => {
        const sel = document.getElementById(id);
        if (!sel) return;
        const prevVal = sel.value;
        sel.innerHTML = `<option value="">— select vendor —</option>` +
          vendors.map(v => `<option value="${escHtml(v.id)}"${v.id === prevVal ? " selected" : ""}>${escHtml(v.name)}</option>`).join("");
      });

      // Pre-select active vendor and refresh court dropdown
      const avSel = document.getElementById("activeVendorSelect");
      if (avSel && !avSel.value && active && active.vendor_id) avSel.value = active.vendor_id;
      updateCourtSelect("activeCourtSelect", avSel ? avSel.value : "");

      // Vendor list
      const vendorList = document.getElementById("vendorList");
      if (vendorList) {
        if (!vendors.length) {
          vendorList.innerHTML = `<div style="color:var(--muted);font-size:13px;padding:8px 0;">Δεν υπάρχουν vendors ακόμα.</div>`;
        } else {
          vendorList.innerHTML = vendors.map(v => `
            <div style="display:flex;justify-content:space-between;align-items:flex-start;padding:10px 0;border-bottom:1px solid var(--line);">
              <div>
                <strong>${escHtml(v.name)}</strong>
                ${v.address ? `<div style="font-size:12px;color:var(--muted);margin-top:2px;">${escHtml(v.address)}</div>` : ""}
              </div>
              <button onclick="deleteVendor('${escHtml(v.id)}')" style="background:rgba(255,107,95,0.12);border:1px solid rgba(255,107,95,0.28);color:var(--danger);border-radius:5px;padding:5px 10px;cursor:pointer;font:inherit;font-size:12px;flex-shrink:0;margin-left:12px;">Διαγραφή</button>
            </div>
          `).join("");
        }
      }

      // Court list
      const courtList = document.getElementById("courtList");
      if (courtList) {
        if (!courts.length) {
          courtList.innerHTML = `<div style="color:var(--muted);font-size:13px;padding:8px 0;">Δεν υπάρχουν γήπεδα ακόμα.</div>`;
        } else {
          courtList.innerHTML = courts.map(c => {
            const v = vendors.find(x => x.id === c.vendor_id);
            const isActive = active && active.court_id === c.id;
            return `
              <div style="display:flex;justify-content:space-between;align-items:flex-start;padding:10px ${isActive ? "8px" : "0"};border-bottom:1px solid var(--line);${isActive ? "background:rgba(47,208,143,0.05);border-radius:6px;" : ""}">
                <div>
                  <strong>${escHtml(c.name)}</strong>${isActive ? ` <span style="font-size:11px;color:var(--accent);">● active</span>` : ""}
                  <div style="font-size:12px;color:var(--muted);margin-top:2px;">${escHtml(v ? v.name : "—")} · ${escHtml(c.surface || "—")}</div>
                  ${c.notes ? `<div style="font-size:12px;color:var(--muted);">${escHtml(c.notes)}</div>` : ""}
                </div>
                <button onclick="deleteCourt('${escHtml(c.id)}')" style="background:rgba(255,107,95,0.12);border:1px solid rgba(255,107,95,0.28);color:var(--danger);border-radius:5px;padding:5px 10px;cursor:pointer;font:inherit;font-size:12px;flex-shrink:0;margin-left:12px;">Διαγραφή</button>
              </div>
            `;
          }).join("");
        }
      }
      updateCommandButtons();
    }

    function updateCourtSelect(selectId, vendorId) {
      const sel = document.getElementById(selectId);
      if (!sel) return;
      const { courts, active } = vendorData;
      const list = vendorId ? courts.filter(c => c.vendor_id === vendorId) : courts;
      sel.innerHTML = `<option value="">— select court —</option>` +
        list.map(c => `<option value="${escHtml(c.id)}"${c.id === (active && active.court_id) ? " selected" : ""}>${escHtml(c.name)}</option>`).join("");
    }

    function deleteVendor(id) {
      const d = JSON.parse(JSON.stringify(vendorData));
      d.vendors = d.vendors.filter(v => v.id !== id);
      d.courts = d.courts.filter(c => c.vendor_id !== id);
      if (d.active && d.active.vendor_id === id) d.active = {};
      saveVendors(d);
    }

    function deleteCourt(id) {
      const d = JSON.parse(JSON.stringify(vendorData));
      d.courts = d.courts.filter(c => c.id !== id);
      if (d.active && d.active.court_id === id) d.active = Object.assign({}, d.active, { court_id: null });
      saveVendors(d);
    }

    document.getElementById("activeVendorSelect").addEventListener("change", function () {
      updateCourtSelect("activeCourtSelect", this.value);
    });

    document.getElementById("activeSessionForm").addEventListener("submit", function (e) {
      e.preventDefault();
      const vendorId = document.getElementById("activeVendorSelect").value;
      const courtId = document.getElementById("activeCourtSelect").value;
      if (!vendorId || !courtId) return;
      const d = JSON.parse(JSON.stringify(vendorData));
      d.active = { vendor_id: vendorId, court_id: courtId };
      saveVendors(d);
    });

    document.getElementById("addVendorForm").addEventListener("submit", function (e) {
      e.preventDefault();
      const name = document.getElementById("vendorName").value.trim();
      const address = document.getElementById("vendorAddress").value.trim();
      if (!name) return;
      const d = JSON.parse(JSON.stringify(vendorData));
      d.vendors.push({ id: "v" + Date.now(), name, address });
      saveVendors(d);
      this.reset();
    });

    document.getElementById("addCourtForm").addEventListener("submit", function (e) {
      e.preventDefault();
      const vendor_id = document.getElementById("courtVendorSelect").value;
      const name = document.getElementById("courtName").value.trim();
      const surface = document.getElementById("courtSurface").value;
      const notes = document.getElementById("courtNotes").value.trim();
      if (!vendor_id || !name) return;
      const d = JSON.parse(JSON.stringify(vendorData));
      d.courts.push({ id: "c" + Date.now(), vendor_id, name, surface, notes });
      saveVendors(d);
      this.reset();
    });

    loadVendors();

    // ── Survey history ────────────────────────────────────────────────────────
    async function loadSurveys() {
      try {
        const res = await fetch("/api/surveys", { cache: "no-store" });
        const data = await res.json();
        renderSurveys(data.surveys || []);
      } catch (_) {}
    }

    function renderSurveys(surveys) {
      const tbody = document.getElementById("surveyRows");
      const empty = document.getElementById("surveyEmpty");
      const table = document.getElementById("surveyTable");
      if (!surveys.length) {
        table.style.display = "none";
        empty.style.display = "block";
        return;
      }
      table.style.display = "";
      empty.style.display = "none";
      tbody.innerHTML = surveys.map(s => {
        const dt = s.surveyed_at ? new Date(s.surveyed_at * 1000).toLocaleString() : "—";
        const status = s.status || "SUCCESS";
        const statusEl = status === "SUCCESS"
          ? `<span style="color:var(--accent);font-size:11px;font-weight:600;">SUCCESS</span>`
          : `<span style="color:var(--danger);font-size:11px;font-weight:600;">FAILED</span>`;
        const lengthM = s.court_length_m != null ? s.court_length_m.toFixed(2) : (
          s.east_x != null && s.west_x != null ? (s.east_x - s.west_x).toFixed(2) : "—");
        const widthM = s.court_width_m != null ? s.court_width_m.toFixed(2) : (
          s.north_y != null && s.south_y != null ? (s.north_y - s.south_y).toFixed(2) : "—");
        const w  = s.west_x  != null ? s.west_x.toFixed(2)  : "—";
        const e  = s.east_x  != null ? s.east_x.toFixed(2)  : "—";
        const sY = s.south_y != null ? s.south_y.toFixed(2) : "—";
        const n  = s.north_y != null ? s.north_y.toFixed(2) : "—";
        return `<tr>
          <td style="color:var(--muted);">${s.id}</td>
          <td>${dt}</td>
          <td>${escHtml(s.vendor_name || "—")}</td>
          <td>${escHtml(s.court_name || "—")}</td>
          <td style="color:var(--muted);">${escHtml(s.surface || "—")}</td>
          <td>${statusEl}</td>
          <td style="color:var(--accent-2);font-weight:600;font-variant-numeric:tabular-nums;">${lengthM} m</td>
          <td style="color:var(--accent-2);font-weight:600;font-variant-numeric:tabular-nums;">${widthM} m</td>
          <td style="font-variant-numeric:tabular-nums;color:var(--muted);">${w}</td>
          <td style="font-variant-numeric:tabular-nums;color:var(--muted);">${e}</td>
          <td style="font-variant-numeric:tabular-nums;color:var(--muted);">${sY}</td>
          <td style="font-variant-numeric:tabular-nums;color:var(--muted);">${n}</td>
          <td style="color:var(--muted);">${s.point_count ?? "—"}</td>
        </tr>`;
      }).join("");
    }
  </script>
</body>
</html>
"""


class ControlPanelHandler(BaseHTTPRequestHandler):
    store: RobotCommandStore
    status_store: RobotStatusStore
    sensor_store: RobotSensorStore
    db: TennisRobotDB

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
        if path == "/api/vendors":
            self._send_json(self.db.read_all())
            return
        if path == "/api/surveys":
            self._send_json({"surveys": self.db.surveys()})
            return
        if path == "/favicon.ico":
            self.send_response(HTTPStatus.NO_CONTENT)
            self.end_headers()
            return
        self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        if path == "/api/vendors":
            self._handle_vendors_post()
            return
        if path not in {"/command", "/api/command"}:
            self.send_error(HTTPStatus.NOT_FOUND)
            return

        length = int(self.headers.get("Content-Length", "0"))
        body = self.rfile.read(length).decode("utf-8")
        mode = parse_qs(body).get("mode", ["idle"])[0]
        # Validation is handled inside RobotCommandStore.write(); unknown modes map to idle.
        command = self.store.write(mode)
        if path == "/api/command":
            self._send_json(command.to_mapping())
            return
        self.send_response(HTTPStatus.SEE_OTHER)
        self.send_header("Location", "/")
        self.end_headers()

    def _handle_vendors_post(self) -> None:
        length = int(self.headers.get("Content-Length", "0"))
        try:
            data = json.loads(self.rfile.read(length).decode("utf-8"))
        except (json.JSONDecodeError, ValueError):
            self.send_error(HTTPStatus.BAD_REQUEST, "Invalid JSON")
            return
        if not isinstance(data, dict):
            self.send_error(HTTPStatus.BAD_REQUEST, "Expected JSON object")
            return
        self.db.write_all(data)
        self._send_json({"ok": True})

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
        if urlparse(self.path).path in {"/api/status", "/api/robot-status", "/api/sensors", "/api/diagnostics", "/api/webcam/frame", "/api/vendors", "/api/surveys", "/favicon.ico"}:
            return
        print(f"{self.address_string()} - {format % args}")

    def _read_court_boundary(self) -> dict | None:
        path = ROOT / "runtime" / "court_boundary.json"
        if not path.exists():
            return None
        try:
            bounds = json.loads(path.read_text(encoding="utf-8"))
        except Exception:
            return None
        if bounds and (bounds.get("survey_complete") or bounds.get("status") == "SUCCESS"):
            self.db.import_survey(bounds)
        return bounds

    def _maybe_save_obstacle_run(self, robot_status: dict) -> None:
        """Persist a completed ObstacleSurvey run to DB (idempotent)."""
        nav = ((robot_status.get("survey") or {}).get("navigation") or {})
        obs = nav.get("obstacle_survey")
        if not obs:
            return
        result = obs.get("result") or {}
        if result.get("status") not in ("SUCCESS", "FAILED"):
            return
        try:
            self.db.save_obstacle_run(obs)
        except Exception:
            pass

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
        self._maybe_save_obstacle_run(robot_status)
        return {
            "generated_at": time.time(),
            "command": self.store.read().to_mapping(),
            "robot": robot_status,
            "court_boundary": self._read_court_boundary(),
            "obstacle_runs": self.db.obstacle_runs(20),
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
        try:
            self.wfile.write(payload)
        except (BrokenPipeError, ConnectionResetError):
            pass


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the tennis robot remote console.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8081)
    parser.add_argument("--command-file", type=Path, default=None)
    parser.add_argument("--status-file", type=Path, default=None)
    parser.add_argument("--db-file", type=Path, default=None)
    args = parser.parse_args()

    ControlPanelHandler.store = RobotCommandStore(args.command_file) if args.command_file else RobotCommandStore.from_env()
    ControlPanelHandler.status_store = RobotStatusStore(args.status_file) if args.status_file else RobotStatusStore.from_env()
    ControlPanelHandler.sensor_store = RobotSensorStore.from_env()
    ControlPanelHandler.db = TennisRobotDB(args.db_file) if args.db_file else TennisRobotDB()
    server = ThreadingHTTPServer((args.host, args.port), ControlPanelHandler)
    print(f"remote console listening on http://{args.host}:{args.port}")
    print(f"command file: {ControlPanelHandler.store.path}")
    print(f"status file: {ControlPanelHandler.status_store.path}")
    print(f"sensor file: {ControlPanelHandler.sensor_store.path}")
    server.serve_forever()


if __name__ == "__main__":
    main()
