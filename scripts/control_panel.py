#!/usr/bin/env python3
"""Tiny Python web control panel for the tennis robot simulation."""

from __future__ import annotations

import argparse
import json
import sys
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "ball_detector"))

from control_bus import RobotCommandStore, SUPPORTED_MODES  # noqa: E402


HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Tennis Robot Control</title>
  <style>
    :root {
      color-scheme: light;
      --ink: #18212f;
      --muted: #5f6b7a;
      --line: #d8dee8;
      --surface: #f6f8fb;
      --accent: #1d7f5c;
      --danger: #bb3b30;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      display: grid;
      place-items: center;
      background: var(--surface);
      color: var(--ink);
      font-family: Arial, Helvetica, sans-serif;
    }
    main {
      width: min(760px, calc(100vw - 32px));
      background: white;
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 28px;
      box-shadow: 0 20px 60px rgba(24, 33, 47, 0.10);
    }
    h1 {
      margin: 0 0 8px;
      font-size: 30px;
      letter-spacing: 0;
    }
    p {
      margin: 0;
      color: var(--muted);
      line-height: 1.5;
    }
    .status {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 12px;
      margin: 28px 0;
    }
    .metric {
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 14px;
      min-height: 78px;
    }
    .metric span {
      display: block;
      color: var(--muted);
      font-size: 13px;
      margin-bottom: 8px;
    }
    .metric strong {
      font-size: 22px;
      text-transform: capitalize;
    }
    form {
      display: flex;
      gap: 12px;
      flex-wrap: wrap;
    }
    button {
      border: 0;
      border-radius: 8px;
      padding: 14px 18px;
      min-width: 156px;
      color: white;
      font-size: 16px;
      font-weight: 700;
      cursor: pointer;
    }
    button[value="collect"] { background: var(--accent); }
    button[value="survey"] { background: #2f6494; }
    button[value="idle"] { background: var(--danger); }
    .roadmap {
      margin-top: 24px;
      padding-top: 20px;
      border-top: 1px solid var(--line);
    }
    .roadmap code {
      background: #edf1f5;
      border-radius: 4px;
      padding: 2px 5px;
    }
    @media (max-width: 640px) {
      main { padding: 22px; }
      .status { grid-template-columns: 1fr; }
      button { width: 100%; }
    }
  </style>
</head>
<body>
  <main>
    <h1>Tennis Robot Control</h1>
    <p>Send a high-level command to the Webots controller through the shared command file.</p>

    <section class="status" aria-label="Robot command status">
      <div class="metric"><span>Requested mode</span><strong id="mode">Idle</strong></div>
      <div class="metric"><span>Sequence</span><strong id="sequence">0</strong></div>
      <div class="metric"><span>Source</span><strong id="source">Default</strong></div>
    </section>

    <form method="post" action="/command">
      <button type="submit" name="mode" value="collect">Start collection</button>
      <button type="submit" name="mode" value="survey">Survey court</button>
      <button type="submit" name="mode" value="idle">Stop</button>
    </form>

    <p class="roadmap">
      Survey mode writes court measurements to <code>runtime/court_survey.csv</code>.
      Later this command shape can grow into launcher commands and saved training programs.
    </p>
  </main>
  <script>
    async function refreshStatus() {
      const response = await fetch('/api/status', { cache: 'no-store' });
      const status = await response.json();
      document.querySelector('#mode').textContent = status.mode;
      document.querySelector('#sequence').textContent = status.sequence;
      document.querySelector('#source').textContent = status.source;
    }
    refreshStatus();
    setInterval(refreshStatus, 3000);
  </script>
</body>
</html>
"""


class ControlPanelHandler(BaseHTTPRequestHandler):
    store: RobotCommandStore

    def do_GET(self) -> None:
        if self.path == "/" or self.path == "/index.html":
            self._send_html(HTML)
            return
        if self.path == "/api/status":
            self._send_json(self.store.read().to_mapping())
            return
        if self.path == "/favicon.ico":
            self.send_response(HTTPStatus.NO_CONTENT)
            self.end_headers()
            return
        self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:
        if self.path not in {"/command", "/api/command"}:
            self.send_error(HTTPStatus.NOT_FOUND)
            return

        length = int(self.headers.get("Content-Length", "0"))
        body = self.rfile.read(length).decode("utf-8")
        mode = parse_qs(body).get("mode", ["idle"])[0]
        if mode not in SUPPORTED_MODES:
            self.send_error(HTTPStatus.BAD_REQUEST, "Unsupported mode")
            return

        command = self.store.write(mode)
        if self.path == "/api/command":
            self._send_json(command.to_mapping())
            return
        self.send_response(HTTPStatus.SEE_OTHER)
        self.send_header("Location", "/")
        self.end_headers()

    def log_message(self, format: str, *args: object) -> None:
        if self.path in {"/api/status", "/favicon.ico"}:
            return
        print(f"{self.address_string()} - {format % args}")

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
    parser = argparse.ArgumentParser(description="Run the tennis robot web control panel.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8081)
    parser.add_argument("--command-file", type=Path, default=None)
    args = parser.parse_args()

    ControlPanelHandler.store = RobotCommandStore(args.command_file) if args.command_file else RobotCommandStore.from_env()
    server = ThreadingHTTPServer((args.host, args.port), ControlPanelHandler)
    print(f"control panel listening on http://{args.host}:{args.port}")
    print(f"command file: {ControlPanelHandler.store.path}")
    server.serve_forever()


if __name__ == "__main__":
    main()
