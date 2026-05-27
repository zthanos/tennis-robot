#!/usr/bin/env python3
"""Browser-based tennis robot route simulation panel."""

from __future__ import annotations

import argparse
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


HTML = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Tennis Robot Route Simulator</title>
  <style>
    :root {
      color-scheme: light;
      --ink: #18202b;
      --muted: #66717f;
      --line: #d7dde5;
      --panel: #f7f8fa;
      --court: #b95d3f;
      --court-dark: #9f4d35;
      --accent: #16715d;
      --warning: #b66a16;
      --danger: #b33a32;
      --ball: #d7ff48;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      background: #eef1f4;
      color: var(--ink);
      font-family: Arial, Helvetica, sans-serif;
    }
    main {
      display: grid;
      grid-template-columns: minmax(620px, 1fr) 420px;
      min-height: 100vh;
    }
    .workspace {
      display: grid;
      grid-template-rows: auto 1fr auto;
      gap: 14px;
      padding: 20px;
    }
    header {
      display: flex;
      align-items: flex-end;
      justify-content: space-between;
      gap: 16px;
    }
    h1 {
      margin: 0 0 4px;
      font-size: 25px;
      letter-spacing: 0;
    }
    p {
      margin: 0;
      color: var(--muted);
      line-height: 1.45;
    }
    .actions {
      display: flex;
      gap: 8px;
      flex-wrap: wrap;
      justify-content: flex-end;
    }
    button {
      border: 1px solid transparent;
      border-radius: 7px;
      padding: 10px 13px;
      background: var(--accent);
      color: white;
      font-weight: 700;
      cursor: pointer;
    }
    button.secondary {
      background: white;
      color: var(--ink);
      border-color: var(--line);
    }
    button.danger { background: var(--danger); }
    canvas {
      width: 100%;
      height: 100%;
      min-height: 610px;
      background: var(--court);
      border: 1px solid #c8d0da;
      border-radius: 8px;
      box-shadow: 0 16px 50px rgba(24, 32, 43, 0.12);
    }
    .legend {
      display: flex;
      flex-wrap: wrap;
      gap: 14px;
      color: var(--muted);
      font-size: 13px;
    }
    .swatch {
      display: inline-flex;
      align-items: center;
      gap: 6px;
    }
    .swatch::before {
      content: "";
      width: 12px;
      height: 12px;
      border-radius: 50%;
      background: var(--dot, #222);
      display: inline-block;
    }
    aside {
      border-left: 1px solid var(--line);
      background: white;
      overflow: auto;
      padding: 20px;
    }
    .group {
      padding: 16px 0;
      border-bottom: 1px solid var(--line);
    }
    .group:first-child { padding-top: 0; }
    h2 {
      margin: 0 0 12px;
      font-size: 15px;
      letter-spacing: 0;
    }
    label {
      display: grid;
      grid-template-columns: 1fr auto;
      gap: 10px;
      align-items: center;
      margin: 12px 0;
      color: var(--muted);
      font-size: 13px;
    }
    input[type="range"] { width: 170px; }
    input[type="number"], select {
      width: 108px;
      border: 1px solid var(--line);
      border-radius: 6px;
      padding: 7px 8px;
      color: var(--ink);
      background: white;
    }
    .metrics {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 9px;
    }
    .metric {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 11px;
      min-height: 70px;
    }
    .metric span {
      display: block;
      color: var(--muted);
      font-size: 12px;
      margin-bottom: 7px;
    }
    .metric strong {
      font-size: 20px;
      letter-spacing: 0;
    }
    table {
      width: 100%;
      border-collapse: collapse;
      font-size: 12px;
    }
    th, td {
      border-bottom: 1px solid var(--line);
      padding: 7px 4px;
      text-align: right;
      white-space: nowrap;
    }
    th:first-child, td:first-child { text-align: left; }
    th { color: var(--muted); font-weight: 700; }
    .log {
      max-height: 180px;
      overflow: auto;
      background: #101820;
      color: #d7e7df;
      border-radius: 8px;
      padding: 10px;
      font-family: Consolas, monospace;
      font-size: 12px;
      line-height: 1.45;
    }
    @media (max-width: 1040px) {
      main { grid-template-columns: 1fr; }
      aside { border-left: 0; border-top: 1px solid var(--line); }
      canvas { min-height: 520px; }
    }
  </style>
</head>
<body>
  <main>
    <section class="workspace">
      <header>
        <div>
          <h1>Tennis Robot Route Simulator</h1>
          <p>Map court limits, scan by phase, plan a collection route, refresh during collection, then measure time.</p>
        </div>
        <div class="actions">
          <button id="runBtn">Run</button>
          <button id="pauseBtn" class="secondary">Pause</button>
          <button id="randomBtn" class="secondary">Randomize</button>
          <button id="jsonBtn" class="secondary">JSON</button>
          <button id="csvBtn" class="secondary">CSV</button>
          <button id="resetBtn" class="danger">Reset</button>
        </div>
      </header>
      <canvas id="court" width="1280" height="820" aria-label="Tennis court simulation"></canvas>
      <div class="legend">
        <span class="swatch" style="--dot: var(--ball)">Balls</span>
        <span class="swatch" style="--dot: #1d4ed8">Robot</span>
        <span class="swatch" style="--dot: #3d4654">Fixed obstacles</span>
        <span class="swatch" style="--dot: #c2410c">People safety zones</span>
        <span class="swatch" style="--dot: #152e5f">Planned path</span>
      </div>
    </section>
    <aside>
      <section class="group">
        <h2>Scenario</h2>
        <label>Playable area
          <select id="areaMode">
            <option value="half">Left half</option>
            <option value="two-phase">Two-phase halves</option>
            <option value="full">Continuous full court</option>
          </select>
        </label>
        <label>Balls <input id="ballCount" type="number" min="5" max="80" value="48"></label>
        <label>Ball distribution
          <select id="ballDistribution">
            <option value="realistic">Realistic bias</option>
            <option value="uniform">Uniform scatter</option>
          </select>
        </label>
        <label>Seed <input id="seed" type="number" min="1" max="99999" value="37"></label>
        <label>People <input id="peopleCount" type="number" min="0" max="8" value="3"></label>
        <label>Bag / basket obstacles <input id="fixedCount" type="number" min="0" max="8" value="3"></label>
      </section>

      <section class="group">
        <h2>Robot Model</h2>
        <label>Travel speed <input id="speed" type="range" min="0.25" max="1.80" step="0.05" value="0.85"></label>
        <label>Pickup time <input id="pickupTime" type="range" min="0.4" max="3.0" step="0.1" value="1.2"></label>
        <label>Scan time <input id="scanTime" type="range" min="2" max="18" step="0.5" value="7"></label>
        <label>Re-scan every N balls <input id="rescanEvery" type="number" min="0" max="20" value="5"></label>
        <label>Safety buffer <input id="buffer" type="range" min="0.15" max="1.25" step="0.05" value="0.55"></label>
      </section>

      <section class="group">
        <h2>Telemetry</h2>
        <div class="metrics">
          <div class="metric"><span>Collected</span><strong id="mCollected">0 / 0</strong></div>
          <div class="metric"><span>Total time</span><strong id="mTime">0.0s</strong></div>
          <div class="metric"><span>Distance</span><strong id="mDistance">0.0m</strong></div>
          <div class="metric"><span>Replans</span><strong id="mReplans">0</strong></div>
          <div class="metric"><span>Blocked balls</span><strong id="mBlocked">0</strong></div>
          <div class="metric"><span>Avg leg speed</span><strong id="mSpeed">0.00m/s</strong></div>
          <div class="metric"><span>Safety stops</span><strong id="mStops">0</strong></div>
          <div class="metric"><span>Near misses</span><strong id="mNearMisses">0</strong></div>
        </div>
      </section>

      <section class="group">
        <h2>Run Log</h2>
        <div id="log" class="log"></div>
      </section>

      <section class="group">
        <h2>Leg Telemetry</h2>
        <table>
          <thead><tr><th>Leg</th><th>Phase</th><th>Ball</th><th>Dist</th><th>Travel</th><th>Mode</th></tr></thead>
          <tbody id="legs"></tbody>
        </table>
      </section>
    </aside>
  </main>

  <script>
    const COURT = { length: 23.77, width: 10.97, singlesWidth: 8.23, service: 6.40 };
    const GRID = 0.25;
    const ROBOT_RADIUS = 0.36;
    const canvas = document.getElementById("court");
    const ctx = canvas.getContext("2d");
    const els = Object.fromEntries([...document.querySelectorAll("input, select")].map(el => [el.id, el]));
    const metrics = {
      collected: document.getElementById("mCollected"),
      time: document.getElementById("mTime"),
      distance: document.getElementById("mDistance"),
      replans: document.getElementById("mReplans"),
      blocked: document.getElementById("mBlocked"),
      speed: document.getElementById("mSpeed"),
      stops: document.getElementById("mStops"),
      nearMisses: document.getElementById("mNearMisses"),
      log: document.getElementById("log"),
      legs: document.getElementById("legs"),
    };

    let scenario = null;
    let plan = null;
    let sim = null;
    let running = false;
    let lastFrame = 0;

    function rng(seed) {
      let s = seed >>> 0;
      return () => {
        s = (1664525 * s + 1013904223) >>> 0;
        return s / 4294967296;
      };
    }

    function gaussian(r) {
      const u1 = Math.max(r(), 1e-9);
      const u2 = r();
      return Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2);
    }

    function clamp(n, min, max) {
      return Math.max(min, Math.min(max, n));
    }

    function val(id) {
      const el = els[id];
      return el.type === "number" || el.type === "range" ? Number(el.value) : el.value;
    }

    function bounds() {
      const half = COURT.length / 2;
      if (val("areaMode") === "half") return { minX: -half, maxX: -0.35, minY: -COURT.width / 2, maxY: COURT.width / 2 };
      return { minX: -half, maxX: half, minY: -COURT.width / 2, maxY: COURT.width / 2 };
    }

    function halfBounds(side) {
      const half = COURT.length / 2;
      if (side === "left") return { minX: -half, maxX: -0.35, minY: -COURT.width / 2, maxY: COURT.width / 2 };
      return { minX: 0.35, maxX: half, minY: -COURT.width / 2, maxY: COURT.width / 2 };
    }

    function phaseStart(area) {
      const fromLeft = Math.abs(area.minX) >= Math.abs(area.maxX);
      return { x: fromLeft ? area.minX + 1.15 : area.maxX - 1.15, y: 0 };
    }

    function inBounds(p, area, margin = 0) {
      return p.x >= area.minX + margin && p.x <= area.maxX - margin && p.y >= area.minY + margin && p.y <= area.maxY - margin;
    }

    function planningPhases(sc) {
      if (val("areaMode") !== "two-phase") {
        return [{ index: 1, name: val("areaMode") === "half" ? "Left half" : "Full court", bounds: sc.bounds, start: sc.robotStart }];
      }
      const left = halfBounds("left");
      const right = halfBounds("right");
      return [
        { index: 1, name: "Phase A left half", bounds: left, start: phaseStart(left) },
        { index: 2, name: "Phase B right half", bounds: right, start: phaseStart(right) },
      ];
    }

    function randomPlayableArea(r, b) {
      if (val("areaMode") === "two-phase" || (val("areaMode") === "full" && val("ballDistribution") === "realistic")) {
        return r() < 0.5 ? halfBounds("left") : halfBounds("right");
      }
      return b;
    }

    function sampleBallPosition(r, b) {
      const area = randomPlayableArea(r, b);
      const margin = 0.55;
      if (val("ballDistribution") === "uniform") {
        return {
          x: area.minX + margin + r() * (area.maxX - area.minX - margin * 2),
          y: area.minY + margin + r() * (area.maxY - area.minY - margin * 2),
        };
      }

      const zone = r();
      const leftSide = Math.abs(area.minX) > Math.abs(area.maxX);
      const netX = leftSide ? area.maxX : area.minX;
      const backX = leftSide ? area.minX : area.maxX;
      let x;
      let y;

      if (zone < 0.46) {
        x = netX + (leftSide ? -1 : 1) * Math.abs(gaussian(r)) * 1.05;
        y = gaussian(r) * COURT.width * 0.26;
      } else if (zone < 0.82) {
        x = backX + (leftSide ? 1 : -1) * Math.abs(gaussian(r)) * 1.20;
        y = gaussian(r) * COURT.width * 0.30;
      } else if (zone < 0.94) {
        const serviceX = leftSide ? -COURT.service : COURT.service;
        x = serviceX + gaussian(r) * 1.65;
        y = gaussian(r) * COURT.width * 0.24;
      } else {
        x = area.minX + margin + r() * (area.maxX - area.minX - margin * 2);
        y = area.minY + margin + r() * (area.maxY - area.minY - margin * 2);
      }

      return {
        x: clamp(x, area.minX + margin, area.maxX - margin),
        y: clamp(y, area.minY + margin, area.maxY - margin),
      };
    }

    function makeScenario() {
      const r = rng(val("seed"));
      const b = bounds();
      const obstacles = [];
      const people = [];
      const robotStart = { x: b.minX + 1.15, y: 0 };
      const net = { kind: "rect", label: "net", x: 0, y: 0, w: 0.18, h: COURT.width + 0.8 };
      obstacles.push(net);

      for (let i = 0; i < val("peopleCount"); i++) {
        const angle = r() * Math.PI * 2;
        const speed = 0.10 + r() * 0.22;
        let person = null;
        for (let attempt = 0; attempt < 60; attempt++) {
          const candidate = {
            kind: "circle",
            label: `person-${i + 1}`,
            x: b.minX + 1.5 + r() * (b.maxX - b.minX - 3.0),
            y: b.minY + 1.0 + r() * (b.maxY - b.minY - 2.0),
            radius: 0.42,
            human: true,
            vx: Math.cos(angle) * speed,
            vy: Math.sin(angle) * speed,
          };
          if (dist(candidate, robotStart) > 1.8 && isFree(candidate.x, candidate.y, obstacles, 0.4)) {
            person = candidate;
            break;
          }
        }
        if (!person) continue;
        people.push(person);
        obstacles.push(person);
      }
      for (let i = 0; i < val("fixedCount"); i++) {
        for (let attempt = 0; attempt < 60; attempt++) {
          const obstacle = {
            kind: "circle",
            label: `bag-${i + 1}`,
            x: b.minX + 1.2 + r() * (b.maxX - b.minX - 2.4),
            y: b.minY + 0.8 + r() * (b.maxY - b.minY - 1.6),
            radius: 0.32 + r() * 0.25,
            human: false,
          };
          if (dist(obstacle, robotStart) > 1.5 && isFree(obstacle.x, obstacle.y, obstacles, 0.25)) {
            obstacles.push(obstacle);
            break;
          }
        }
      }

      const balls = [];
      let attempts = 0;
      while (balls.length < val("ballCount") && attempts < val("ballCount") * 90) {
        attempts++;
        const position = sampleBallPosition(r, b);
        const ball = {
          id: balls.length + 1,
          x: position.x,
          y: position.y,
          collected: false,
          blocked: false,
        };
        if (isFree(ball.x, ball.y, obstacles, 0.18)) balls.push(ball);
      }

      return {
        bounds: b,
        robotStart,
        obstacles,
        people,
        balls,
      };
    }

    function isFree(x, y, obstacles, extra = 0, area = bounds()) {
      const b = area;
      const pad = ROBOT_RADIUS + val("buffer") + extra;
      if (x < b.minX + pad || x > b.maxX - pad || y < b.minY + pad || y > b.maxY - pad) return false;
      return !obstacles.some(o => collides(x, y, o, pad));
    }

    function collides(x, y, o, pad) {
      if (o.kind === "circle") return Math.hypot(x - o.x, y - o.y) <= o.radius + pad + (o.human ? 0.35 : 0);
      const dx = Math.max(Math.abs(x - o.x) - o.w / 2, 0);
      const dy = Math.max(Math.abs(y - o.y) - o.h / 2, 0);
      return Math.hypot(dx, dy) <= pad;
    }

    function segmentClear(a, b, obstacles, area = bounds()) {
      const dist = Math.hypot(a.x - b.x, a.y - b.y);
      const steps = Math.max(2, Math.ceil(dist / 0.18));
      for (let i = 0; i <= steps; i++) {
        const t = i / steps;
        const x = a.x + (b.x - a.x) * t;
        const y = a.y + (b.y - a.y) * t;
        if (!isFree(x, y, obstacles, 0, area)) return false;
      }
      return true;
    }

    function worldToGrid(p, b) {
      return {
        gx: Math.max(0, Math.min(Math.round((b.maxX - b.minX) / GRID), Math.round((p.x - b.minX) / GRID))),
        gy: Math.max(0, Math.min(Math.round((b.maxY - b.minY) / GRID), Math.round((p.y - b.minY) / GRID))),
      };
    }

    function gridToWorld(g, b) {
      return { x: b.minX + g.gx * GRID, y: b.minY + g.gy * GRID };
    }

    function pathfind(start, goal, sc) {
      if (segmentClear(start, goal, sc.obstacles, sc.bounds)) return { distance: dist(start, goal), path: [start, goal], mode: "direct" };

      const b = sc.bounds;
      const cols = Math.round((b.maxX - b.minX) / GRID) + 1;
      const rows = Math.round((b.maxY - b.minY) / GRID) + 1;
      const s = worldToGrid(start, b);
      const g = worldToGrid(goal, b);
      const key = (x, y) => `${x},${y}`;
      const open = [{ ...s, f: 0, cost: 0 }];
      const came = new Map();
      const cost = new Map([[key(s.gx, s.gy), 0]]);
      const dirs = [[1,0],[-1,0],[0,1],[0,-1],[1,1],[1,-1],[-1,1],[-1,-1]];
      let found = null;

      while (open.length) {
        open.sort((a, b) => a.f - b.f);
        const cur = open.shift();
        if (cur.gx === g.gx && cur.gy === g.gy) { found = cur; break; }
        for (const [dx, dy] of dirs) {
          const nx = cur.gx + dx, ny = cur.gy + dy;
          if (nx < 0 || ny < 0 || nx >= cols || ny >= rows) continue;
          const wp = gridToWorld({ gx: nx, gy: ny }, b);
          if (!isFree(wp.x, wp.y, sc.obstacles, 0, sc.bounds)) continue;
          const step = Math.hypot(dx, dy) * GRID;
          const nk = key(nx, ny);
          const nextCost = cur.cost + step;
          if (!cost.has(nk) || nextCost < cost.get(nk)) {
            cost.set(nk, nextCost);
            came.set(nk, key(cur.gx, cur.gy));
            const h = Math.hypot(nx - g.gx, ny - g.gy) * GRID;
            open.push({ gx: nx, gy: ny, cost: nextCost, f: nextCost + h });
          }
        }
      }

      if (!found) return { distance: Infinity, path: [], mode: "blocked" };
      const nodes = [];
      let k = key(found.gx, found.gy);
      while (k) {
        const [gx, gy] = k.split(",").map(Number);
        nodes.push(gridToWorld({ gx, gy }, b));
        k = came.get(k);
      }
      nodes.reverse();
      nodes[0] = start;
      nodes[nodes.length - 1] = goal;
      return { distance: pathDistance(nodes), path: simplifyPath(nodes, sc.obstacles, sc.bounds), mode: "avoid" };
    }

    function simplifyPath(nodes, obstacles, area = bounds()) {
      if (nodes.length <= 2) return nodes;
      const out = [nodes[0]];
      let anchor = 0;
      while (anchor < nodes.length - 1) {
        let next = nodes.length - 1;
        while (next > anchor + 1 && !segmentClear(nodes[anchor], nodes[next], obstacles, area)) next--;
        out.push(nodes[next]);
        anchor = next;
      }
      return out;
    }

    function dist(a, b) { return Math.hypot(a.x - b.x, a.y - b.y); }
    function pathDistance(path) { return path.slice(1).reduce((sum, p, i) => sum + dist(path[i], p), 0); }

    function planRoute(sc) {
      sc.balls.forEach(b => b.blocked = false);
      const phasePlans = planningPhases(sc).map(phase => planPhaseRoute(sc, phase));
      const legs = phasePlans.flatMap(phase => phase.legs);
      const plannedBalls = new Set(legs.map(leg => leg.ball));
      sc.balls.forEach(b => {
        const ownedByPhase = phasePlans.some(phase => inBounds(b, phase.bounds, 0));
        b.blocked = ownedByPhase && !plannedBalls.has(b);
      });

      const telemetry = buildTelemetry(sc, phasePlans);
      return { legs, phases: phasePlans, telemetry };
    }

    function planPhaseRoute(sc, phase) {
      const phaseScenario = { ...sc, bounds: phase.bounds, robotStart: phase.start };
      const candidates = sc.balls.filter(ball => (
        inBounds(ball, phase.bounds, 0.08) && isFree(ball.x, ball.y, sc.obstacles, 0.08, phase.bounds)
      ));
      let current = phase.start;
      const remaining = [...candidates];
      const legs = [];
      while (remaining.length) {
        let bestIndex = -1;
        let best = null;
        for (let i = 0; i < remaining.length; i++) {
          const candidate = remaining[i];
          const pf = pathfind(current, candidate, phaseScenario);
          if (pf.distance < Infinity && (!best || pf.distance < best.distance)) {
            best = { ...pf, ball: candidate };
            bestIndex = i;
          }
        }
        if (!best) break;
        legs.push({
          ...best,
          phaseIndex: phase.index,
          phaseName: phase.name,
          phaseBounds: phase.bounds,
        });
        current = best.ball;
        remaining.splice(bestIndex, 1);
      }
      return { ...phase, candidates, legs };
    }

    function buildTelemetry(sc, phasePlans) {
      const speed = val("speed");
      const pickupTime = val("pickupTime");
      const scanTime = val("scanTime");
      const rescanEvery = val("rescanEvery");
      const legs = phasePlans.flatMap(phase => phase.legs);
      let time = 0;
      let distance = 0;
      let replans = 0;
      const legRows = [];
      const events = [];
      let legNumber = 1;

      phasePlans.forEach((phase, phaseIndex) => {
        if (phaseIndex > 0) {
          events.push({ t: round(time), type: "phase", detail: `handover to ${phase.name}` });
        }
        events.push({ t: round(time), type: "scan", detail: `${phase.name} initial scan mapped ${phase.candidates.length} reachable balls` });
        time += scanTime;
        replans++;

        phase.legs.forEach((leg, index) => {
          const travel = leg.distance / speed;
          distance += leg.distance;
          time += travel + pickupTime;
          if (leg.mode === "avoid") replans++;
          legRows.push({
            leg: legNumber++,
            phase: phase.index,
            ball: leg.ball.id,
            distance_m: round(leg.distance),
            travel_s: round(travel),
            mode: leg.mode,
          });
          events.push({ t: round(time), type: "pickup", detail: `${phase.name}: ball ${leg.ball.id} collected` });
          if (rescanEvery > 0 && (index + 1) % rescanEvery === 0 && index < phase.legs.length - 1) {
            time += scanTime;
            replans++;
            events.push({ t: round(time), type: "rescan", detail: `${phase.name} refresh scan after ${index + 1} balls` });
          }
        });
      });

      if (!phasePlans.length) {
        events.push({ t: 0, type: "scan", detail: "no collection phases available" });
      }

      return {
        court: { length_m: COURT.length, width_m: COURT.width, area_mode: val("areaMode"), bounds_m: sc.bounds },
        robot: {
          speed_m_s: speed,
          pickup_time_s: pickupTime,
          scan_time_s: scanTime,
          safety_buffer_m: val("buffer"),
          ball_distribution: val("ballDistribution"),
        },
        summary: {
          phases: phasePlans.length,
          balls_detected: sc.balls.length,
          balls_collectable: legs.length,
          balls_blocked: sc.balls.filter(b => b.blocked).length,
          total_time_s: round(time),
          total_distance_m: round(distance),
          replans,
          scan_events: events.filter(event => event.type === "scan" || event.type === "rescan").length,
          average_leg_speed_m_s: legs.length ? round(distance / Math.max(1, time - scanTime * phasePlans.length)) : 0,
        },
        legs: legRows,
        events,
      };
    }

    function round(n) { return Math.round(n * 100) / 100; }

    function resetSimulation(randomizeSeed = false) {
      if (randomizeSeed) els.seed.value = Math.floor(Math.random() * 90000) + 1000;
      scenario = makeScenario();
      plan = planRoute(scenario);
      sim = {
        legIndex: 0,
        segmentIndex: 0,
        pos: { ...scenario.robotStart },
        elapsed: 0,
        collected: 0,
        distance: 0,
        runtimeReplans: 0,
        safetyStops: 0,
        nearMisses: 0,
        stopCooldown: 0,
        replanCooldown: 0,
        eventLog: [...plan.telemetry.events],
        state: "ready",
      };
      running = false;
      window.simTelemetry = plan.telemetry;
      renderTelemetry();
      draw();
    }

    function start() {
      if (!scenario) resetSimulation();
      running = true;
      lastFrame = performance.now();
      requestAnimationFrame(tick);
    }

    function tick(now) {
      if (!running) return;
      const dt = Math.min(0.08, (now - lastFrame) / 1000);
      lastFrame = now;
      advance(dt);
      draw();
      requestAnimationFrame(tick);
    }

    function advance(dt) {
      sim.elapsed += dt;
      movePeople(dt);
      const leg = plan.legs[sim.legIndex];
      if (!leg) { running = false; sim.state = "complete"; return; }
      const path = leg.path;
      const target = path[sim.segmentIndex + 1];
      if (!target) {
        leg.ball.collected = true;
        sim.collected++;
        sim.legIndex++;
        sim.segmentIndex = 0;
        const nextLeg = plan.legs[sim.legIndex];
        if (nextLeg && nextLeg.phaseIndex !== leg.phaseIndex) {
          sim.pos = { ...nextLeg.path[0] };
          logRuntimeEvent("phase", `handover to ${nextLeg.phaseName}`);
        }
        sim.state = "pickup";
        return;
      }

      const safety = safetyCheck(sim.pos, target);
      if (safety.stop) {
        sim.state = "safety-stop";
        sim.safetyStops++;
        sim.stopCooldown = 0.45;
        logRuntimeEvent("stop", `${safety.reason}; route paused`);
        updateLiveMetrics();
        return;
      }
      if (safety.nearMiss) {
        sim.nearMisses++;
      }
      if (sim.stopCooldown > 0) {
        sim.stopCooldown = Math.max(0, sim.stopCooldown - dt);
        sim.state = "safety-hold";
        updateLiveMetrics();
        return;
      }
      if (sim.replanCooldown > 0) {
        sim.replanCooldown = Math.max(0, sim.replanCooldown - dt);
      }
      if (!segmentClear(sim.pos, target, scenario.obstacles, leg.phaseBounds || scenario.bounds)) {
        if (sim.replanCooldown > 0) {
          sim.state = "safety-hold";
          updateLiveMetrics();
          return;
        }
        if (replanFromCurrent()) {
          sim.state = "runtime-replan";
          sim.runtimeReplans++;
          sim.replanCooldown = 0.6;
          logRuntimeEvent("replan", "dynamic obstacle entered the active path");
        } else {
          running = false;
          sim.state = "blocked";
          logRuntimeEvent("blocked", "no safe route remains from current position");
        }
        updateLiveMetrics();
        return;
      }

      sim.state = leg.mode === "avoid" ? "avoid" : "travel";
      const maxMove = val("speed") * dt;
      const d = dist(sim.pos, target);
      if (d <= maxMove) {
        sim.distance += d;
        sim.pos = { ...target };
        sim.segmentIndex++;
      } else {
        const k = maxMove / d;
        sim.pos = { x: sim.pos.x + (target.x - sim.pos.x) * k, y: sim.pos.y + (target.y - sim.pos.y) * k };
        sim.distance += maxMove;
      }
      updateLiveMetrics();
    }

    function movePeople(dt) {
      const b = scenario.bounds;
      scenario.people.forEach(p => {
        p.x += p.vx * dt;
        p.y += p.vy * dt;
        const margin = p.radius + val("buffer") + 0.45;
        if (p.x < b.minX + margin || p.x > b.maxX - margin) {
          p.vx *= -1;
          p.x = Math.max(b.minX + margin, Math.min(b.maxX - margin, p.x));
        }
        if (p.y < b.minY + margin || p.y > b.maxY - margin) {
          p.vy *= -1;
          p.y = Math.max(b.minY + margin, Math.min(b.maxY - margin, p.y));
        }
      });
    }

    function safetyCheck(pos, target) {
      const lookahead = Math.min(1.15, Math.max(0.45, val("speed") * 1.2));
      const pathDist = dist(pos, target);
      const steps = Math.max(2, Math.ceil(Math.min(pathDist, lookahead) / 0.12));
      let nearMiss = false;
      for (let i = 0; i <= steps; i++) {
        const t = steps === 0 ? 0 : i / steps;
        const k = pathDist === 0 ? 0 : Math.min(1, (lookahead * t) / pathDist);
        const probe = { x: pos.x + (target.x - pos.x) * k, y: pos.y + (target.y - pos.y) * k };
        for (const o of scenario.obstacles) {
          const clearance = obstacleClearance(probe, o);
          if (clearance < ROBOT_RADIUS + 0.08) {
            return { stop: true, nearMiss, reason: `${o.label} inside emergency clearance` };
          }
          if (clearance < ROBOT_RADIUS + val("buffer") + 0.18) {
            nearMiss = true;
          }
        }
      }
      return { stop: false, nearMiss, reason: "" };
    }

    function obstacleClearance(p, o) {
      if (o.kind === "circle") {
        return Math.hypot(p.x - o.x, p.y - o.y) - o.radius - (o.human ? 0.35 : 0);
      }
      const dx = Math.max(Math.abs(p.x - o.x) - o.w / 2, 0);
      const dy = Math.max(Math.abs(p.y - o.y) - o.h / 2, 0);
      return Math.hypot(dx, dy);
    }

    function replanFromCurrent() {
      const activeLeg = plan.legs[sim.legIndex];
      const activePhase = activeLeg ? activeLeg.phaseIndex : 1;
      const activeBounds = activeLeg ? activeLeg.phaseBounds : scenario.bounds;
      const phaseScenario = { ...scenario, bounds: activeBounds, robotStart: { ...sim.pos } };
      const remaining = plan.legs
        .slice(sim.legIndex)
        .filter(leg => leg.phaseIndex === activePhase)
        .map(leg => leg.ball)
        .filter(ball => !ball.collected);
      let current = { ...sim.pos };
      const legs = [];
      while (remaining.length) {
        let best = null;
        let bestIndex = -1;
        for (let i = 0; i < remaining.length; i++) {
          const candidate = remaining[i];
          const pf = pathfind(current, candidate, phaseScenario);
          if (pf.distance < Infinity && (!best || pf.distance < best.distance)) {
            best = { ...pf, ball: candidate };
            bestIndex = i;
          }
        }
        if (!best) break;
        legs.push({
          ...best,
          phaseIndex: activeLeg.phaseIndex,
          phaseName: activeLeg.phaseName,
          phaseBounds: activeLeg.phaseBounds,
        });
        current = best.ball;
        remaining.splice(bestIndex, 1);
      }
      if (!legs.length && remaining.length) return false;
      const laterPhaseLegs = plan.legs.slice(sim.legIndex).filter(leg => leg.phaseIndex !== activePhase);
      plan.legs = [...plan.legs.slice(0, sim.legIndex), ...legs, ...laterPhaseLegs];
      sim.segmentIndex = 0;
      return true;
    }

    function logRuntimeEvent(type, detail) {
      const event = { t: round(sim.elapsed), type, detail };
      sim.eventLog.push(event);
      plan.telemetry.events = sim.eventLog;
      metrics.log.innerHTML = sim.eventLog.slice(-24).map(e => `${e.t.toFixed(1)}s  ${e.type.padEnd(7)} ${e.detail}`).join("<br>");
    }

    function renderTelemetry() {
      const s = plan.telemetry.summary;
      metrics.collected.textContent = `0 / ${s.balls_collectable}`;
      metrics.time.textContent = `${s.total_time_s.toFixed(1)}s`;
      metrics.distance.textContent = `${s.total_distance_m.toFixed(1)}m`;
      metrics.replans.textContent = s.replans;
      metrics.blocked.textContent = s.balls_blocked;
      metrics.speed.textContent = `${s.average_leg_speed_m_s.toFixed(2)}m/s`;
      metrics.stops.textContent = "0";
      metrics.nearMisses.textContent = "0";
      metrics.log.innerHTML = plan.telemetry.events.map(e => `${e.t.toFixed(1)}s  ${e.type.padEnd(7)} ${e.detail}`).join("<br>");
      metrics.legs.innerHTML = plan.telemetry.legs.slice(0, 18).map(l => (
        `<tr><td>${l.leg}</td><td>${l.phase}</td><td>${l.ball}</td><td>${l.distance_m.toFixed(1)}m</td><td>${l.travel_s.toFixed(1)}s</td><td>${l.mode}</td></tr>`
      )).join("");
    }

    function updateLiveMetrics() {
      const s = plan.telemetry.summary;
      metrics.collected.textContent = `${sim.collected} / ${s.balls_collectable}`;
      metrics.time.textContent = `${sim.elapsed.toFixed(1)}s`;
      metrics.distance.textContent = `${sim.distance.toFixed(1)}m`;
      metrics.replans.textContent = s.replans + sim.runtimeReplans;
      metrics.stops.textContent = sim.safetyStops;
      metrics.nearMisses.textContent = sim.nearMisses;
    }

    function downloadTelemetry(format) {
      const telemetry = {
        ...plan.telemetry,
        runtime: {
          elapsed_s: round(sim.elapsed),
          distance_m: round(sim.distance),
          collected: sim.collected,
          runtime_replans: sim.runtimeReplans,
          safety_stops: sim.safetyStops,
          near_misses: sim.nearMisses,
          state: sim.state,
        },
      };
      const stamp = new Date().toISOString().replace(/[:.]/g, "-");
      if (format === "json") {
        downloadFile(`tennis-route-telemetry-${stamp}.json`, JSON.stringify(telemetry, null, 2), "application/json");
        return;
      }
      const rows = [
        ["leg", "phase", "ball", "distance_m", "travel_s", "mode"],
        ...telemetry.legs.map(l => [l.leg, l.phase, l.ball, l.distance_m, l.travel_s, l.mode]),
        [],
        ["summary", "value"],
        ["balls_detected", telemetry.summary.balls_detected],
        ["balls_collectable", telemetry.summary.balls_collectable],
        ["balls_blocked", telemetry.summary.balls_blocked],
        ["total_time_s", telemetry.summary.total_time_s],
        ["total_distance_m", telemetry.summary.total_distance_m],
        ["replans", telemetry.summary.replans],
        ["scan_events", telemetry.summary.scan_events],
        ["average_leg_speed_m_s", telemetry.summary.average_leg_speed_m_s],
        [],
        ["runtime", "value"],
        ["elapsed_s", telemetry.runtime.elapsed_s],
        ["runtime_distance_m", telemetry.runtime.distance_m],
        ["collected", telemetry.runtime.collected],
        ["runtime_replans", telemetry.runtime.runtime_replans],
        ["safety_stops", telemetry.runtime.safety_stops],
        ["near_misses", telemetry.runtime.near_misses],
        ["state", telemetry.runtime.state],
      ];
      downloadFile(`tennis-route-telemetry-${stamp}.csv`, rows.map(csvRow).join("\n"), "text/csv");
    }

    function csvRow(row) {
      return row.map(value => `"${String(value).replace(/"/g, '""')}"`).join(",");
    }

    function downloadFile(name, text, type) {
      const url = URL.createObjectURL(new Blob([text], { type }));
      const a = document.createElement("a");
      a.href = url;
      a.download = name;
      a.click();
      URL.revokeObjectURL(url);
    }

    function scale() {
      const margin = 54;
      const sx = (canvas.width - margin * 2) / COURT.length;
      const sy = (canvas.height - margin * 2) / COURT.width;
      const k = Math.min(sx, sy);
      return { k, cx: canvas.width / 2, cy: canvas.height / 2 };
    }

    function pt(p) {
      const s = scale();
      return { x: s.cx + p.x * s.k, y: s.cy - p.y * s.k };
    }

    function draw() {
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      drawCourt();
      drawBounds();
      drawObstacles();
      drawRoute();
      drawBalls();
      drawRobot();
      drawOverlay();
    }

    function drawCourt() {
      ctx.fillStyle = getComputedStyle(document.documentElement).getPropertyValue("--court");
      ctx.fillRect(0, 0, canvas.width, canvas.height);
      ctx.save();
      ctx.globalAlpha = 0.18;
      ctx.fillStyle = getComputedStyle(document.documentElement).getPropertyValue("--court-dark");
      for (let i = 0; i < 18; i++) {
        ctx.fillRect(40 + i * 70, 80 + (i % 5) * 120, 34, 520);
      }
      ctx.restore();
      lineRect(-COURT.length / 2, -COURT.width / 2, COURT.length, COURT.width, "#f5efe4", 3);
      lineRect(-COURT.length / 2, -COURT.singlesWidth / 2, COURT.length, COURT.singlesWidth, "#f5efe4", 2);
      drawLine({ x: 0, y: -COURT.width / 2 - 0.3 }, { x: 0, y: COURT.width / 2 + 0.3 }, "#2c2f35", 5);
      drawLine({ x: -COURT.service, y: -COURT.singlesWidth / 2 }, { x: -COURT.service, y: COURT.singlesWidth / 2 }, "#f5efe4", 2);
      drawLine({ x: COURT.service, y: -COURT.singlesWidth / 2 }, { x: COURT.service, y: COURT.singlesWidth / 2 }, "#f5efe4", 2);
      drawLine({ x: -COURT.service, y: 0 }, { x: COURT.service, y: 0 }, "#f5efe4", 2);
    }

    function drawBounds() {
      const b = scenario.bounds;
      ctx.save();
      ctx.setLineDash([10, 8]);
      lineRect(b.minX, b.minY, b.maxX - b.minX, b.maxY - b.minY, "#0f513f", 3);
      ctx.restore();
    }

    function drawObstacles() {
      scenario.obstacles.forEach(o => {
        if (o.label === "net") return;
        const p = pt(o);
        const s = scale();
        ctx.beginPath();
        ctx.arc(p.x, p.y, (o.radius + val("buffer") + (o.human ? 0.35 : 0)) * s.k, 0, Math.PI * 2);
        ctx.fillStyle = o.human ? "rgba(194, 65, 12, 0.22)" : "rgba(61, 70, 84, 0.20)";
        ctx.fill();
        ctx.beginPath();
        ctx.arc(p.x, p.y, o.radius * s.k, 0, Math.PI * 2);
        ctx.fillStyle = o.human ? "#c2410c" : "#3d4654";
        ctx.fill();
      });
    }

    function drawRoute() {
      ctx.save();
      ctx.lineWidth = 3;
      ctx.strokeStyle = "#152e5f";
      ctx.globalAlpha = 0.82;
      ctx.beginPath();
      let started = false;
      let phaseIndex = null;
      plan.legs.forEach(leg => {
        leg.path.forEach(p => {
          const q = pt(p);
          if (!started || phaseIndex !== leg.phaseIndex) {
            ctx.moveTo(q.x, q.y);
            started = true;
            phaseIndex = leg.phaseIndex;
          }
          else ctx.lineTo(q.x, q.y);
        });
      });
      ctx.stroke();
      ctx.restore();
    }

    function drawBalls() {
      const s = scale();
      scenario.balls.forEach(b => {
        const p = pt(b);
        ctx.beginPath();
        ctx.arc(p.x, p.y, Math.max(4, 0.09 * s.k), 0, Math.PI * 2);
        ctx.fillStyle = b.collected ? "rgba(215,255,72,0.25)" : b.blocked ? "#6b7280" : "#d7ff48";
        ctx.fill();
        ctx.strokeStyle = "rgba(24,32,43,0.35)";
        ctx.stroke();
      });
    }

    function drawRobot() {
      const p = pt(sim.pos);
      const s = scale();
      ctx.save();
      ctx.translate(p.x, p.y);
      ctx.fillStyle = "#1d4ed8";
      ctx.strokeStyle = "white";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(0, 0, ROBOT_RADIUS * s.k, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
      ctx.restore();
    }

    function drawOverlay() {
      ctx.fillStyle = "rgba(255,255,255,0.88)";
      ctx.fillRect(18, 18, 300, 86);
      ctx.fillStyle = "#18202b";
      ctx.font = "bold 18px Arial";
      ctx.fillText(`State: ${sim.state}`, 34, 47);
      ctx.font = "14px Arial";
      ctx.fillText(`Court: ${COURT.length}m x ${COURT.width}m`, 34, 72);
      ctx.fillText(`Move area: ${val("areaMode")}`, 34, 94);
    }

    function lineRect(x, y, w, h, color, width) {
      const a = pt({ x, y });
      const b = pt({ x: x + w, y: y + h });
      ctx.strokeStyle = color;
      ctx.lineWidth = width;
      ctx.strokeRect(a.x, b.y, b.x - a.x, a.y - b.y);
    }

    function drawLine(a, b, color, width) {
      const p = pt(a), q = pt(b);
      ctx.strokeStyle = color;
      ctx.lineWidth = width;
      ctx.beginPath();
      ctx.moveTo(p.x, p.y);
      ctx.lineTo(q.x, q.y);
      ctx.stroke();
    }

    document.getElementById("runBtn").addEventListener("click", start);
    document.getElementById("pauseBtn").addEventListener("click", () => { running = false; });
    document.getElementById("randomBtn").addEventListener("click", () => resetSimulation(true));
    document.getElementById("jsonBtn").addEventListener("click", () => downloadTelemetry("json"));
    document.getElementById("csvBtn").addEventListener("click", () => downloadTelemetry("csv"));
    document.getElementById("resetBtn").addEventListener("click", () => resetSimulation(false));
    Object.values(els).forEach(el => el.addEventListener("change", () => resetSimulation(false)));
    resetSimulation(false);
  </script>
</body>
</html>
"""


class SimulationPanelHandler(BaseHTTPRequestHandler):
    def do_GET(self) -> None:
        if self.path in {"/", "/index.html"}:
            payload = HTML.encode("utf-8")
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
            return
        if self.path == "/favicon.ico":
            self.send_response(HTTPStatus.NO_CONTENT)
            self.end_headers()
            return
        self.send_error(HTTPStatus.NOT_FOUND)

    def log_message(self, format: str, *args: object) -> None:
        if self.path != "/favicon.ico":
            print(f"{self.address_string()} - {format % args}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the browser route simulation panel.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8082)
    args = parser.parse_args()

    server = ThreadingHTTPServer((args.host, args.port), SimulationPanelHandler)
    print(f"simulation panel listening on http://{args.host}:{args.port}")
    server.serve_forever()


if __name__ == "__main__":
    main()
