#!/usr/bin/env python3
"""Local web console for controlling and observing the tennis robot simulation."""

from __future__ import annotations

import argparse
import json
import sys
import time
from collections import Counter
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "ball_detector"))

from control_bus import RobotCommandStore, RobotSensorStore, RobotStatusStore, SUPPORTED_MODES  # noqa: E402


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
    .command[value="survey"] { background: var(--accent-2); color: #06101d; }
    .command[value="idle"] { background: var(--danger); color: #1b0604; }
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
    .legend .route::before { border-radius: 2px; background: var(--accent-2); }
    .legend .robot::before { background: var(--warn); }
    .sensor-grid {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
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
        <button data-view="telemetry">Telemetry</button>
        <button data-view="stats">Command Stats</button>
        <button data-view="history">History</button>
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
            <span>Same-side ball</span>
            <span class="across">Across net</span>
            <span class="route">Planned route</span>
            <span class="robot">Robot</span>
          </div>
        </div>
        <div class="panel map-panel">
          <h3>Sensor Views</h3>
          <div class="sensor-grid">
            <div class="sensor-view"><h4>Front Camera</h4><div id="frontCameraView" class="sensor-empty">waiting for image</div></div>
            <div class="sensor-view"><h4>Collector Camera</h4><div id="collectorCameraView" class="sensor-empty">waiting for image</div></div>
            <div class="sensor-view"><h4>Front Depth</h4><div id="frontDepthView" class="sensor-empty">waiting for range image</div></div>
          </div>
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
          <div class="metric"><span>Collect Commands</span><strong id="sCollect">0</strong><small>requested mode collect</small></div>
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
    </main>
  </div>

  <script>
    const titles = {
      dashboard: ["Dashboard", "Observe the robot mode, collector state, current target, and command stream while the simulation runs."],
      control: ["Control", "Send high-level commands to the running Webots controller."],
      telemetry: ["Telemetry", "Inspect live robot pose, detection, command output, survey data, and raw status."],
      stats: ["Command Stats", "Review per-mode command counts and recent command usage."],
      history: ["History", "Audit the local command stream written by this console and controller startup."]
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
    }
    document.querySelectorAll("nav button").forEach(btn => btn.addEventListener("click", () => setView(btn.dataset.view)));

    document.getElementById("commandForm").addEventListener("submit", async event => {
      event.preventDefault();
      const mode = event.submitter.value;
      await fetch("/api/command", {
        method: "POST",
        headers: { "Content-Type": "application/x-www-form-urlencoded" },
        body: new URLSearchParams({ mode })
      });
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
      document.getElementById("kDistance").textContent = `distance ${fmt(obs.distance_m, "m")} bearing ${fmt(obs.bearing_deg, "deg")}`;

      setKv("snapshot", [
        ["Robot position", `${fmt(pose.x_m, "m")}, ${fmt(pose.y_m, "m")}`],
        ["Robot yaw", fmt((pose.yaw_rad || 0) * 180 / Math.PI, "deg")],
        ["Collector state", robot.collector_state || "idle"],
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
        ["Controller state", robot.collector_state || "idle"]
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
        ["Ball world", `${fmt(obs.world_x_m, "m")}, ${fmt(obs.world_y_m, "m")}`],
        ["Confidence", fmt(obs.confidence)],
        ["Animation", robot.collection_animation_active ? "active" : "idle"],
        ["Survey state", survey.state || "idle"],
        ["Survey target", `${fmt(survey.target_x_m, "m")}, ${fmt(survey.target_y_m, "m")}`]
      ]);
      document.getElementById("rawStatus").textContent = JSON.stringify(robot, null, 2);

      renderHistory();
      renderStats();
      renderCourtMap();
      renderSensors();
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
    function renderSensors() {
      renderSensor("frontCameraView", sensors.front_camera);
      renderSensor("collectorCameraView", sensors.collector_camera);
      renderSensor("frontDepthView", sensors.front_depth);
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
        ctx.fillStyle = "rgba(87,166,255,0.08)";
        ctx.fillRect(sx(bounds.min_x), sy(bounds.max_y), sx(bounds.max_x) - sx(bounds.min_x), sy(bounds.min_y) - sy(bounds.max_y));
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

      (map.balls || []).forEach(ball => {
        const x = sx(ball.x_m);
        const y = sy(ball.y_m);
        ctx.beginPath();
        ctx.arc(x, y, ball.planned ? 9 : 7, 0, Math.PI * 2);
        ctx.fillStyle = ball.side === "across_net" ? "#8793a0" : (ball.visible_candidate ? "#2fd08f" : "#d7e85f");
        ctx.fill();
        ctx.strokeStyle = ball.planned ? "#ffffff" : "rgba(0,0,0,0.45)";
        ctx.lineWidth = ball.planned ? 3 : 1;
        ctx.stroke();
        if (ball.order) {
          ctx.fillStyle = "#07110d";
          ctx.font = "bold 12px system-ui";
          ctx.textAlign = "center";
          ctx.textBaseline = "middle";
          ctx.fillText(String(ball.order), x, y);
        }
      });

      const robot = map.robot || {};
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

      const metrics = map.metrics || {};
      ctx.fillStyle = "rgba(12,17,22,0.76)";
      ctx.fillRect(pad + 10, pad + 10, 310, 64);
      ctx.fillStyle = "#eef4f8";
      ctx.font = "16px system-ui";
      ctx.textAlign = "left";
      ctx.fillText(`planned ${metrics.balls_collectable ?? 0} / detected ${metrics.balls_detected ?? ((map.balls || []).length)}`, pad + 24, pad + 36);
      ctx.fillStyle = "#91a2b2";
      ctx.font = "13px system-ui";
      ctx.fillText(`distance ${fmt(metrics.total_distance_m, "m")} · replans ${metrics.planned_replans ?? 0}`, pad + 24, pad + 58);
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
      document.getElementById("sCollect").textContent = byMode.collect || 0;
      document.getElementById("sSurvey").textContent = byMode.survey || 0;
      document.getElementById("sIdle").textContent = byMode.idle || 0;
      document.getElementById("statsRows").innerHTML = ["collect", "survey", "idle"].map(mode => {
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

    def log_message(self, format: str, *args: object) -> None:
        if urlparse(self.path).path in {"/api/status", "/api/robot-status", "/api/sensors", "/api/diagnostics", "/favicon.ico"}:
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
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def _send_json(self, data: dict[str, object]) -> None:
        payload = json.dumps(data).encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "application/json")
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
