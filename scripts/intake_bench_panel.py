#!/usr/bin/env python3
"""Safe, allow-listed web panel for the dual-intake Mega bench firmware.

The panel deliberately exposes no arbitrary serial console.  Every motion is
bounded by both this process and the limits in 07_dual_intake_mega_bench.ino.
Opening the serial port resets the Mega; STOP + DISARM are always the first
commands sent after that reset.
"""

from __future__ import annotations

import argparse
import json
import signal
import threading
import time
from collections import deque
from dataclasses import asdict, dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlparse


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_HTML = ROOT / "scripts" / "intake_bench_panel.html"
STATE_NAMES = {
    0: "DISARMED",
    1: "ARMED_IDLE",
    2: "RUNNING",
    3: "WAITING_FOR_BALL",
    4: "FAULTED",
}


@dataclass(frozen=True)
class Scenario:
    scenario_id: str
    title: str
    group: str
    description: str
    command: str | None
    power: str
    duration_ms: int = 0
    motion: bool = False
    planned: bool = False
    wiring: tuple[str, ...] = ()


SCENARIOS: dict[str, Scenario] = {
    "status": Scenario(
        "status",
        "Ανάγνωση κατάστασης",
        "Έλεγχοι",
        "Διαβάζει Mega, encoders και IR χωρίς κίνηση.",
        "STATUS",
        "OUT OFF ή ON",
    ),
    "zero_encoders": Scenario(
        "zero_encoders",
        "Μηδενισμός encoders",
        "Έλεγχοι",
        "Μηδενίζει μόνο τους μετρητές encoder.",
        "ZERO",
        "OUT OFF",
    ),
    "physical_right_inward": Scenario(
        "physical_right_inward",
        "Φυσικός δεξιός — σύντομος παλμός",
        "Μοτέρ",
        "Driver D44/D45, enable D40. Σωστή intake φορά μέσω D45.",
        "PULSE L -60 250",
        "OUT ON",
        duration_ms=250,
        motion=True,
    ),
    "physical_left_inward": Scenario(
        "physical_left_inward",
        "Φυσικός αριστερός — σύντομος παλμός",
        "Μοτέρ",
        "Driver D46/D11, enable D42. Σωστή intake φορά μέσω D46.",
        "PULSE R 60 250",
        "OUT ON",
        duration_ms=250,
        motion=True,
    ),
    "both_inward_short": Scenario(
        "both_inward_short",
        "Και τα δύο — 1 δευτερόλεπτο",
        "Μοτέρ",
        "Και οι δύο τροχοί προς τα μέσα. Bench mode χωρίς encoder gate.",
        "RUN_BENCH 60 1000",
        "OUT ON",
        duration_ms=1000,
        motion=True,
    ),
    "both_inward_duration": Scenario(
        "both_inward_duration",
        "Και τα δύο — 4 δευτερόλεπτα",
        "Προχωρημένα",
        "Η δοκιμή διάρκειας που κάναμε χωρίς μπάλα. Hard timeout 4 s.",
        "RUN_BENCH 60 4000",
        "OUT ON",
        duration_ms=4000,
        motion=True,
    ),
    "right_logic_measure": Scenario(
        "right_logic_measure",
        "Μέτρηση EN / LPWM δεξιού",
        "Διαγνωστικά",
        "Παλμός 1,5 s για μέτρηση R_EN, L_EN ή LPWM με ισχύ μοτέρ κλειστή.",
        "PULSE L -30 1500",
        "OUT OFF",
        duration_ms=1500,
        motion=True,
    ),
    "left_logic_measure": Scenario(
        "left_logic_measure",
        "Μέτρηση EN / RPWM αριστερού",
        "Διαγνωστικά",
        "Παλμός 1,5 s για λογικές μετρήσεις με ισχύ μοτέρ κλειστή.",
        "PULSE R 30 1500",
        "OUT OFF",
        duration_ms=1500,
        motion=True,
    ),
    "swapped_motor_isolation": Scenario(
        "swapped_motor_isolation",
        "Επόμενη δοκιμή: ύποπτο μοτέρ στον καλό driver",
        "Επόμενη δοκιμή",
        "Δοκιμάζει το ύποπτο φυσικό δεξιό μοτέρ από τον γνωστό καλό driver.",
        "PULSE R 60 250",
        "OUT ON",
        duration_ms=250,
        motion=True,
        planned=True,
        wiring=(
            "OUT OFF, τροφοδοτικό εκτός πρίζας και USB αποσυνδεδεμένο πριν την καλωδίωση.",
            "Αποσύνδεσε το κανονικό μοτέρ από M+/M− του καλού driver και μόνωσε τα άκρα του.",
            "Σύνδεσε μόνο το ύποπτο μοτέρ στα M+/M− του καλού driver (D46/D11/D42).",
            "Άφησε τον ύποπτο driver χωρίς μοτέρ. Μην ενώσεις εξόδους δύο drivers.",
        ),
    ),
    "ir_auto_bench": Scenario(
        "ir_auto_bench",
        "IR κύκλος μίας μπάλας",
        "IR — αργότερα",
        "Περιμένει entry IR, κινεί προς τα μέσα και σταματά σε exit IR ή 4 s.",
        "AUTO_BENCH 60 4000",
        "OUT ON",
        motion=True,
        planned=True,
        wiring=(
            "Και τα δύο IR στερεωμένα και clear πριν την εκκίνηση.",
            "Μπάλα έτοιμη αλλά χέρια έξω από τους τροχούς.",
            "Το panel ακυρώνει αυτόματα την αναμονή μετά από 30 s.",
        ),
    ),
}


def parse_data_line(line: str) -> dict[str, Any] | None:
    fields = line.split(",")
    if len(fields) != 11 or fields[0] != "DATA":
        return None
    try:
        state = int(fields[2])
        return {
            "mega_ms": int(fields[1]),
            "state": state,
            "state_name": STATE_NAMES.get(state, f"UNKNOWN_{state}"),
            "left_pwm": int(fields[3]),
            "right_pwm": int(fields[4]),
            "left_count": int(fields[5]),
            "right_count": int(fields[6]),
            "left_rpm": float(fields[7]),
            "right_rpm": float(fields[8]),
            "entry_broken": bool(int(fields[9])),
            "exit_broken": bool(int(fields[10])),
        }
    except ValueError:
        return None


class SerialBenchController:
    def __init__(self, port: str, baud: int, startup_delay: float = 2.5) -> None:
        self.port = port
        self.baud = baud
        self.startup_delay = startup_delay
        self._serial: Any = None
        self._serial_lock = threading.Lock()
        self._action_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._connected_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._lines: deque[dict[str, Any]] = deque(maxlen=240)
        self._line_seq = 0
        self._telemetry: dict[str, Any] = {
            "state": 0,
            "state_name": "DISCONNECTED",
            "left_pwm": 0,
            "right_pwm": 0,
            "left_count": 0,
            "right_count": 0,
            "left_rpm": 0.0,
            "right_rpm": 0.0,
            "entry_broken": False,
            "exit_broken": False,
        }
        self._last_error = ""
        self._last_rx_monotonic = 0.0
        self._action_generation = 0

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop_event.set()
        self.safe_stop()
        if self._thread:
            self._thread.join(timeout=3.0)
        self._close_serial()

    def _append_line(self, line: str, direction: str = "rx") -> None:
        with self._state_lock:
            self._line_seq += 1
            self._lines.append(
                {
                    "seq": self._line_seq,
                    "direction": direction,
                    "line": line,
                    "at": time.time(),
                }
            )
            if direction == "rx":
                self._last_rx_monotonic = time.monotonic()
                telemetry = parse_data_line(line)
                if telemetry:
                    self._telemetry = telemetry
                elif line.startswith("FAULT,"):
                    self._telemetry["state"] = 4
                    self._telemetry["state_name"] = "FAULTED"
                elif line.startswith("OK,DISARMED"):
                    self._telemetry["state"] = 0
                    self._telemetry["state_name"] = "DISARMED"

    def _write(self, command: str) -> None:
        payload = (command + "\n").encode("ascii")
        with self._serial_lock:
            if self._serial is None:
                raise RuntimeError("Mega serial is not connected")
            self._serial.write(payload)
            self._serial.flush()
        self._append_line(command, "tx")

    def _reader_loop(self) -> None:
        while not self._stop_event.is_set():
            if self._serial is None:
                try:
                    import serial  # type: ignore[import-not-found]

                    candidate = serial.Serial(
                        self.port, self.baud, timeout=0.2, exclusive=True
                    )
                    time.sleep(self.startup_delay)
                    with self._serial_lock:
                        self._serial = candidate
                    self._connected_event.set()
                    with self._state_lock:
                        self._last_error = ""
                    self._write("STOP")
                    self._write("DISARM")
                    self._write("STATUS")
                except Exception as exc:  # USB can appear after systemd starts.
                    with self._state_lock:
                        self._last_error = str(exc)
                    self._connected_event.clear()
                    self._close_serial()
                    self._stop_event.wait(2.0)
                    continue

            try:
                with self._serial_lock:
                    serial_port = self._serial
                raw = serial_port.readline() if serial_port is not None else b""
                if raw:
                    line = raw.decode("ascii", "replace").strip()
                    if line:
                        self._append_line(line)
            except Exception as exc:
                with self._state_lock:
                    self._last_error = str(exc)
                self._connected_event.clear()
                self._close_serial()

    def _close_serial(self) -> None:
        with self._serial_lock:
            serial_port, self._serial = self._serial, None
        if serial_port is not None:
            try:
                serial_port.close()
            except Exception:
                pass

    def wait_connected(self, timeout: float = 0.5) -> bool:
        return self._connected_event.wait(timeout)

    def snapshot(self, since: int = 0) -> dict[str, Any]:
        with self._state_lock:
            lines = [entry.copy() for entry in self._lines if entry["seq"] > since]
            telemetry = self._telemetry.copy()
            last_error = self._last_error
            last_rx_age = (
                None
                if self._last_rx_monotonic == 0.0
                else round(time.monotonic() - self._last_rx_monotonic, 2)
            )
            seq = self._line_seq
        return {
            "connected": self._connected_event.is_set(),
            "port": self.port,
            "telemetry": telemetry,
            "last_error": last_error,
            "last_rx_age_s": last_rx_age,
            "seq": seq,
            "lines": lines,
        }

    def safe_stop(self) -> None:
        with self._state_lock:
            self._action_generation += 1  # Cancels any older AUTO watchdog.
        if not self._connected_event.is_set():
            return
        try:
            self._write("STOP")
            self._write("DISARM")
            self._write("STATUS")
        except Exception:
            self._connected_event.clear()
            self._close_serial()

    def run_scenario(self, scenario_id: str, confirmed: bool) -> dict[str, Any]:
        scenario = SCENARIOS.get(scenario_id)
        if scenario is None:
            raise ValueError("Unknown scenario")
        if scenario.motion and not confirmed:
            raise ValueError("Safety checklist is not confirmed")
        if not self.wait_connected(1.0):
            raise RuntimeError(f"Mega is not connected on {self.port}")

        with self._action_lock:
            with self._state_lock:
                self._action_generation += 1
            start_seq = self.snapshot()["seq"]
            try:
                if scenario.scenario_id == "status":
                    self._write("STATUS")
                    time.sleep(0.25)
                elif scenario.scenario_id == "zero_encoders":
                    self._write("STOP")
                    self._write("DISARM")
                    self._write("ZERO")
                    self._write("STATUS")
                    time.sleep(0.25)
                elif scenario.command:
                    self._write("STOP")
                    self._write("DISARM")
                    self._write("ARM")
                    self._write(scenario.command)
                    if scenario.scenario_id == "ir_auto_bench":
                        self._start_auto_watchdog(30.0)
                        time.sleep(0.25)
                    else:
                        time.sleep((scenario.duration_ms / 1000.0) + 0.4)
                        self._write("STOP")
                        self._write("DISARM")
                        self._write("STATUS")
                        time.sleep(0.25)
                return self.snapshot(since=start_seq)
            except Exception:
                self.safe_stop()
                raise

    def _start_auto_watchdog(self, timeout_s: float) -> None:
        with self._state_lock:
            generation = self._action_generation

        def watchdog() -> None:
            if self._stop_event.wait(timeout_s):
                return
            with self._state_lock:
                still_current = generation == self._action_generation
            if still_current:
                self.safe_stop()

        threading.Thread(target=watchdog, daemon=True).start()


class IntakeBenchServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(
        self,
        address: tuple[str, int],
        handler: type[BaseHTTPRequestHandler],
        controller: SerialBenchController,
        html_path: Path,
    ) -> None:
        super().__init__(address, handler)
        self.controller = controller
        self.html_path = html_path


class IntakeBenchHandler(BaseHTTPRequestHandler):
    server: IntakeBenchServer

    def log_message(self, fmt: str, *args: Any) -> None:
        print(f"[intake-bench] {self.address_string()} {fmt % args}")

    def _json(self, payload: Any, status: int = HTTPStatus.OK) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self) -> dict[str, Any]:
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError as exc:
            raise ValueError("Invalid Content-Length") from exc
        if length <= 0 or length > 4096:
            raise ValueError("Invalid request body size")
        try:
            value = json.loads(self.rfile.read(length).decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ValueError("Invalid JSON") from exc
        if not isinstance(value, dict):
            raise ValueError("JSON body must be an object")
        return value

    def do_GET(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        if parsed.path == "/":
            try:
                body = self.server.html_path.read_bytes()
            except OSError as exc:
                self.send_error(HTTPStatus.INTERNAL_SERVER_ERROR, str(exc))
                return
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Frame-Options", "DENY")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.wfile.write(body)
            return
        if parsed.path == "/api/status":
            query = parse_qs(parsed.query)
            try:
                since = max(0, int(query.get("since", ["0"])[0]))
            except ValueError:
                since = 0
            self._json(self.server.controller.snapshot(since=since))
            return
        if parsed.path == "/api/scenarios":
            self._json({"scenarios": [asdict(item) for item in SCENARIOS.values()]})
            return
        self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        if parsed.path == "/api/stop":
            self.server.controller.safe_stop()
            self._json(self.server.controller.snapshot())
            return
        if parsed.path != "/api/action":
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        try:
            payload = self._read_json()
            scenario_id = payload.get("scenario_id")
            if not isinstance(scenario_id, str):
                raise ValueError("scenario_id is required")
            result = self.server.controller.run_scenario(
                scenario_id, confirmed=payload.get("confirmed") is True
            )
            self._json(result)
        except ValueError as exc:
            self._json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)
        except RuntimeError as exc:
            self._json({"error": str(exc)}, HTTPStatus.SERVICE_UNAVAILABLE)
        except Exception as exc:
            self.server.controller.safe_stop()
            self._json({"error": str(exc)}, HTTPStatus.INTERNAL_SERVER_ERROR)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8082)
    parser.add_argument("--serial-port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--html", type=Path, default=DEFAULT_HTML)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    controller = SerialBenchController(args.serial_port, args.baud)
    controller.start()
    server = IntakeBenchServer(
        (args.host, args.port), IntakeBenchHandler, controller, args.html
    )

    def stop_server(_signum: int, _frame: Any) -> None:
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, stop_server)
    signal.signal(signal.SIGTERM, stop_server)
    print(
        f"Intake bench panel: http://{args.host}:{args.port} "
        f"(Mega {args.serial_port} @ {args.baud})"
    )
    try:
        server.serve_forever(poll_interval=0.25)
    finally:
        controller.close()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
