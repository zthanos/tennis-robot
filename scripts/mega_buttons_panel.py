#!/usr/bin/env python3
"""Web panel on the Pi for the drive_test_mega stage-A wiring checks.

Bench stage A (USB only, no motors; docs/hardware/bench-bringup-no-motors-el.md).
The page proves each connection to the Mega on its own: USB link, E-stop D33,
START D32, ARMED LED D34, the four encoders (A and B channels) and the
MPU6050.  This server owns the serial port, keeps the heartbeat and exposes
the live telemetry plus an event log; the checks themselves run in
mega_buttons_panel.html.

Safety: the only commands are allow-listed (ARM, DISARM, ZERO, heartbeat
pause).  CMD is never sent.  After every Mega reset the server sends DISARM,
a placeholder CPR (so START can arm) and CFG PWM 0, so outputs stay at zero
even when ARMED.  The PING heartbeat runs only while a browser keeps polling:
closing the page lets the firmware raise FAULT,HOST_TIMEOUT within 0.3 s.
"""

from __future__ import annotations

import argparse
import json
import signal
import threading
import time
from collections import deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlparse


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_HTML = ROOT / "scripts" / "mega_buttons_panel.html"
STATES = {0: "DISARMED", 1: "ARMED", 2: "ESTOPPED", 3: "FAULTED"}
HEARTBEAT_PERIOD_S = 0.1
BROWSER_STALE_S = 1.0
# Placeholder CPR: only unlocks ARM; no CMD is ever sent, so its value is
# irrelevant.  The real CPR is measured in stage A3.
BENCH_CPR = 1000.0
# Sent after every reset: no motion is possible from this panel.
SAFE_CONFIG = ("DISARM", f"CFG CPR {BENCH_CPR:g}", "CFG PWM 0")

ACTIONS: dict[str, tuple[str, ...]] = {
    "arm": ("ARM",),
    "disarm": ("DISARM",),
    "zero": ("ZERO", "ZERO_GYRO"),
}


class MegaButtonsController:
    def __init__(self, port: str, baud: int, startup_delay: float = 2.0) -> None:
        self.port = port
        self.baud = baud
        self.startup_delay = startup_delay
        self._serial: Any = None
        self._serial_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._connected_event = threading.Event()
        self._threads: list[threading.Thread] = []
        self._events: deque[dict[str, Any]] = deque(maxlen=600)
        self._seq = 0
        self._state: int | None = None
        self._estop_open: bool | None = None
        self._enc = [0, 0, 0, 0]
        self._gz = 0.0
        self._max_loop_gap_us = 0
        self._telemetry_times: deque[float] = deque(maxlen=200)
        self._heartbeat_paused = False
        self._last_poll = 0.0
        self._last_error = ""
        self._ready_line = ""
        self._started_at = time.time()

    # -- lifecycle -----------------------------------------------------------
    def start(self) -> None:
        for target in (self._reader_loop, self._heartbeat_loop):
            thread = threading.Thread(target=target, daemon=True)
            thread.start()
            self._threads.append(thread)

    def close(self) -> None:
        self._stop_event.set()
        try:
            self._write("DISARM")
        except Exception:
            pass
        for thread in self._threads:
            thread.join(timeout=2.0)
        self._close_serial()

    # -- events --------------------------------------------------------------
    def _event(self, kind: str, text: str) -> None:
        # Caller holds _state_lock.
        self._seq += 1
        self._events.append({
            "seq": self._seq,
            "at": time.time(),
            "kind": kind,
            "text": text,
            "estop_open": self._estop_open,
            "heartbeat_paused": self._heartbeat_paused,
        })

    def _set_state(self, state: int) -> None:
        if state != self._state:
            self._state = state
            self._event("state", STATES.get(state, str(state)))

    def _set_estop(self, estop_open: bool) -> None:
        if estop_open != self._estop_open:
            self._estop_open = estop_open
            self._event("estop", "OPEN" if estop_open else "OK")

    def _handle_line(self, line: str) -> bool:
        """Record one line; returns True on a Mega boot (READY)."""
        with self._state_lock:
            if line.startswith("T,"):
                parts = line.split(",")
                if len(parts) < 25:
                    return False
                try:
                    self._enc = [int(value) for value in parts[10:14]]
                    self._gz = float(parts[18])
                    self._max_loop_gap_us = int(parts[23])
                    self._set_estop(parts[22] == "1")
                    self._set_state(int(parts[2]))
                    self._telemetry_times.append(time.monotonic())
                except ValueError:
                    pass
                return False
            if line == "PONG":
                return False
            ready = line.startswith("READY,")
            if ready:
                self._ready_line = line
                self._state = None
            self._event("rx", line)
            return ready

    def _write(self, command: str, log: bool = True) -> None:
        with self._serial_lock:
            if self._serial is None:
                raise RuntimeError("Mega serial is not connected")
            self._serial.write((command + "\n").encode("ascii"))
            self._serial.flush()
        if log:
            with self._state_lock:
                self._event("tx", command)

    def _send_safe_config(self) -> None:
        for command in SAFE_CONFIG:
            self._write(command)

    # -- threads -------------------------------------------------------------
    def _reader_loop(self) -> None:
        while not self._stop_event.is_set():
            if self._serial is None:
                try:
                    import serial  # type: ignore[import-not-found]

                    candidate = serial.Serial(self.port, self.baud, timeout=0.1, exclusive=True)
                    with self._serial_lock:
                        self._serial = candidate
                    with self._state_lock:
                        self._last_error = ""
                        self._event("info", f"port {self.port} opened (Mega reset)")
                    self._stop_event.wait(self.startup_delay)
                    self._connected_event.set()
                    self._send_safe_config()
                except Exception as exc:  # USB may appear later.
                    with self._state_lock:
                        self._last_error = str(exc)
                    self._connected_event.clear()
                    self._close_serial()
                    self._stop_event.wait(2.0)
                    continue
            try:
                with self._serial_lock:
                    port = self._serial
                raw = port.readline() if port is not None else b""
                if raw:
                    line = raw.decode("ascii", "replace").strip()
                    if line and self._handle_line(line) and self._connected_event.is_set():
                        # The Mega rebooted on its own: CPR/PWM are back to defaults.
                        self._send_safe_config()
            except Exception as exc:
                with self._state_lock:
                    self._last_error = str(exc)
                    self._event("info", f"serial error: {exc}")
                self._connected_event.clear()
                self._close_serial()

    def _heartbeat_loop(self) -> None:
        while not self._stop_event.wait(HEARTBEAT_PERIOD_S):
            with self._state_lock:
                browser_alive = time.monotonic() - self._last_poll < BROWSER_STALE_S
                paused = self._heartbeat_paused
            if not self._connected_event.is_set() or paused or not browser_alive:
                continue
            try:
                self._write("PING", log=False)
            except Exception:
                pass

    def _close_serial(self) -> None:
        with self._serial_lock:
            port, self._serial = self._serial, None
        if port is not None:
            try:
                port.close()
            except Exception:
                pass

    # -- API -----------------------------------------------------------------
    def snapshot(self, since: int = 0) -> dict[str, Any]:
        with self._state_lock:
            now = time.monotonic()
            self._last_poll = now
            times = self._telemetry_times
            telemetry_hz = sum(1 for at in times if now - at <= 1.0)
            return {
                "now": time.time(),
                "started_at": self._started_at,
                "connected": self._connected_event.is_set(),
                "port": self.port,
                "state": STATES.get(self._state) if self._state is not None else None,
                "estop_open": self._estop_open,
                "encoders": self._enc,
                "gz": self._gz,
                "max_loop_gap_us": self._max_loop_gap_us,
                "telemetry_hz": telemetry_hz,
                "telemetry_age_s": round(now - times[-1], 3) if times else None,
                "heartbeat_paused": self._heartbeat_paused,
                "ready": self._ready_line,
                "last_error": self._last_error,
                "seq": self._seq,
                "events": [event for event in self._events if event["seq"] > since],
            }

    def action(self, name: str) -> None:
        if name in ("pause_heartbeat", "resume_heartbeat"):
            with self._state_lock:
                self._heartbeat_paused = name == "pause_heartbeat"
                self._event("info", "heartbeat " + ("paused" if self._heartbeat_paused else "resumed"))
            return
        commands = ACTIONS.get(name)
        if commands is None:
            raise ValueError(f"unknown action: {name}")
        for command in commands:
            self._write(command)


class MegaButtonsServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address: tuple[str, int], controller: MegaButtonsController,
                 html_path: Path) -> None:
        super().__init__(address, MegaButtonsHandler)
        self.controller = controller
        self.html_path = html_path


class MegaButtonsHandler(BaseHTTPRequestHandler):
    server: MegaButtonsServer

    def log_message(self, fmt: str, *args: Any) -> None:
        pass  # the page polls several times a second

    def _json(self, payload: Any, status: int = HTTPStatus.OK) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        if parsed.path == "/":
            body = self.server.html_path.read_bytes()
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
            return
        if parsed.path == "/api/status":
            try:
                since = max(0, int(parse_qs(parsed.query).get("since", ["0"])[0]))
            except ValueError:
                since = 0
            self._json(self.server.controller.snapshot(since))
            return
        self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        if parsed.path != "/api/action":
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        try:
            name = parse_qs(parsed.query).get("name", [""])[0]
            self.server.controller.action(name)
            self._json({"ok": True})
        except ValueError as exc:
            self._json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)
        except RuntimeError as exc:
            self._json({"error": str(exc)}, HTTPStatus.SERVICE_UNAVAILABLE)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8083)
    parser.add_argument("--serial-port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=500000)
    parser.add_argument("--html", type=Path, default=DEFAULT_HTML)
    args = parser.parse_args()

    controller = MegaButtonsController(args.serial_port, args.baud)
    controller.start()
    server = MegaButtonsServer((args.host, args.port), controller, args.html)

    def stop_server(_signum: int, _frame: Any) -> None:
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, stop_server)
    signal.signal(signal.SIGTERM, stop_server)
    print(f"Mega wiring panel: http://{args.host}:{args.port} "
          f"(Mega {args.serial_port} @ {args.baud})")
    try:
        server.serve_forever(poll_interval=0.25)
    finally:
        controller.close()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
