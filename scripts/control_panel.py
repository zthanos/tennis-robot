#!/usr/bin/env python3
"""Local web console for controlling and observing the tennis robot simulation."""

from __future__ import annotations

import argparse
import base64
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from collections import Counter
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse
import re


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


class CourtSurveyLaunchManager:
    def __init__(self, command_store: RobotCommandStore | None = None) -> None:
        self._lock = threading.Lock()
        self._process: subprocess.Popen | None = None
        self._started_at: float | None = None
        self._last_exit_code: int | None = None
        self._last_finished_at: float | None = None
        self._command_store = command_store
        self._log_path = ROOT / "runtime" / "court_survey_control_panel.log"

    def set_command_store(self, command_store: RobotCommandStore) -> None:
        self._command_store = command_store

    def start(self) -> dict[str, object]:
        with self._lock:
            if self._process is not None and self._process.poll() is None:
                return self.status()
            self._log_path.parent.mkdir(parents=True, exist_ok=True)
            log_file = self._log_path.open("ab")
            command = (
                "source /opt/ros/humble/setup.bash; "
                "source /ros2_ws/install/setup.bash; "
                "ros2 launch tennis_robot court_survey.launch.py"
            )
            self._process = subprocess.Popen(
                ["bash", "-lc", command],
                cwd=str(ROOT),
                stdout=log_file,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            self._started_at = time.time()
            self._last_exit_code = None
            threading.Thread(target=self._watch_process, daemon=True).start()
            return self.status()

    def stop(self) -> dict[str, object]:
        with self._lock:
            process = self._process
            if process is not None and process.poll() is None:
                self._terminate_process_group(process)
            return self.status()

    def status(self) -> dict[str, object]:
        process = self._process
        running = process is not None and process.poll() is None
        return {
            "running": running,
            "pid": process.pid if running and process is not None else None,
            "started_at": self._started_at,
            "last_exit_code": self._last_exit_code,
            "last_finished_at": self._last_finished_at,
            "log_path": str(self._log_path),
        }

    def _watch_process(self) -> None:
        process = self._process
        if process is None:
            return
        while process.poll() is None:
            if self._completed_boundary_after_start():
                self._terminate_process_group(process)
                break
            time.sleep(0.5)
        exit_code = self._wait_for_exit(process)
        with self._lock:
            if self._process is process:
                self._last_exit_code = exit_code
                self._last_finished_at = time.time()
                self._process = None
        if self._command_store is not None:
            self._command_store.write("idle")

    def _completed_boundary_after_start(self) -> bool:
        if self._started_at is None:
            return False
        path = ROOT / "runtime" / "court_boundary.json"
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return False
        surveyed_at = float(data.get("surveyed_at") or 0.0)
        if surveyed_at < self._started_at - 1.0:
            return False
        return data.get("status") in {"SUCCESS", "OK", "FAILED"} or bool(data.get("survey_complete"))

    def _terminate_process_group(self, process: subprocess.Popen) -> None:
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            return
        except OSError:
            process.terminate()

    def _wait_for_exit(self, process: subprocess.Popen) -> int:
        try:
            return process.wait(timeout=15.0)
        except subprocess.TimeoutExpired:
            pass
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            return process.poll() or 0
        except OSError:
            process.terminate()
        try:
            return process.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            pass
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            return process.poll() or 0
        except OSError:
            process.kill()
        return process.wait(timeout=5.0)


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


class PathHistoryStore:
    def __init__(self, path: Path, max_points: int = 2000, min_step_m: float = 0.04) -> None:
        self.path = path
        self.max_points = max_points
        self.min_step_m = min_step_m
        self._lock = threading.Lock()
        self.path.parent.mkdir(parents=True, exist_ok=True)

    @classmethod
    def from_env(cls) -> "PathHistoryStore":
        default_path = ROOT / "runtime" / "robot_path.json"
        return cls(Path(os.getenv("TENNIS_ROBOT_PATH_FILE", str(default_path))))

    def read(self) -> list[dict[str, float]]:
        with self._lock:
            return self._read_unlocked()

    def update(self, pose: dict[str, object]) -> None:
        x_m = self._as_float(pose.get("x_m"))
        y_m = self._as_float(pose.get("y_m"))
        if x_m is None or y_m is None:
            return
        yaw_rad = self._as_float(pose.get("yaw_rad"))
        point = {"x_m": x_m, "y_m": y_m, "t": time.time()}
        if yaw_rad is not None:
            point["yaw_rad"] = yaw_rad
        with self._lock:
            points = self._read_unlocked()
            if points and math.hypot(points[-1]["x_m"] - x_m, points[-1]["y_m"] - y_m) < self.min_step_m:
                points[-1] = point
            else:
                points.append(point)
            if len(points) > self.max_points:
                points = points[-self.max_points:]
            self.path.write_text(json.dumps(points), encoding="utf-8")

    def clear(self) -> None:
        with self._lock:
            self.path.write_text("[]", encoding="utf-8")

    def _read_unlocked(self) -> list[dict[str, float]]:
        try:
            data = json.loads(self.path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return []
        if not isinstance(data, list):
            return []
        points: list[dict[str, float]] = []
        for item in data:
            if not isinstance(item, dict):
                continue
            x_m = self._as_float(item.get("x_m"))
            y_m = self._as_float(item.get("y_m"))
            if x_m is None or y_m is None:
                continue
            point = {"x_m": x_m, "y_m": y_m}
            for key in ("yaw_rad", "t"):
                value = self._as_float(item.get(key))
                if value is not None:
                    point[key] = value
            points.append(point)
        return points

    @staticmethod
    def _as_float(value: object) -> float | None:
        try:
            result = float(value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(result):
            return None
        return result



HTML_PATH = ROOT / "scripts" / "control_panel.html"
# Split SPA assets (style.css, app.js, views/<name>.html) served under /static/.
STATIC_DIR = ROOT / "scripts" / "control_panel"
_STATIC_CONTENT_TYPES = {
    ".css": "text/css; charset=utf-8",
    ".js": "application/javascript; charset=utf-8",
    ".html": "text/html; charset=utf-8",
}


def _load_control_panel_html() -> str:
    try:
        return HTML_PATH.read_text(encoding="utf-8")
    except OSError:
        return "<!doctype html><html><body><h1>Tennis Robot Console</h1><p>control_panel.html missing.</p></body></html>"

class ControlPanelHandler(BaseHTTPRequestHandler):
    store: RobotCommandStore
    status_store: RobotStatusStore
    sensor_store: RobotSensorStore
    db: TennisRobotDB
    survey_launch: CourtSurveyLaunchManager
    path_store: PathHistoryStore

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        if path in {"/", "/index.html"}:
            self._send_html(_load_control_panel_html())
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
        if path == "/api/path":
            self._send_json({"points": self.path_store.read()})
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
        if path == "/api/survey-archive":
            self._send_json({"surveys": self.db.survey_archive()})
            return
        if path.startswith("/static/"):
            self._send_static(path[len("/static/"):])
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
        if path == "/api/path/clear":
            self.path_store.clear()
            self._send_json({"ok": True})
            return
        if path not in {"/command", "/api/command"}:
            self.send_error(HTTPStatus.NOT_FOUND)
            return

        length = int(self.headers.get("Content-Length", "0"))
        body = self.rfile.read(length).decode("utf-8")
        mode = parse_qs(body).get("mode", ["idle"])[0]
        if mode == "map_court":
            self.survey_launch.start()
        elif mode == "idle":
            self.survey_launch.stop()
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
        if urlparse(self.path).path in {"/api/status", "/api/robot-status", "/api/sensors", "/api/diagnostics", "/api/webcam/frame", "/api/vendors", "/api/surveys", "/api/survey-archive", "/favicon.ico"}:
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
        # Persist completed surveys (v1 SUCCESS or v2 OK/FAILED) tagged to the
        # active court. FAILED runs are kept too — they are the audit trail for
        # spotting process errors. import_survey prunes each court to the last 10.
        done = bool(bounds) and (
            bounds.get("survey_complete") or bounds.get("status") in ("SUCCESS", "OK", "FAILED")
        )
        if done:
            act = self.db.active_session()
            self.db.import_survey(bounds, court_id=act.get("court_id"), vendor_id=act.get("vendor_id"))
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

    def _read_live_court_survey(self) -> dict:
        """Read the live occupancy map written by court_survey_mission_node.

        Clean + fail-loud: the mission node writes runtime/court_survey_live.json
        with real map-frame LiDAR points, robot pose, the locked net and waypoints.
        If that file is missing or stale we return an explicit error and NO
        fabricated waypoints/net, so a problem is visible rather than masked.
        """
        status = self.survey_launch.status()
        live_path = ROOT / "runtime" / "court_survey_live.json"
        if not live_path.exists():
            return {
                "running": status.get("running", False),
                "error": "court_survey_live.json missing (mission node not writing yet)",
                "map_points": [], "navigation_points": [],
            }
        try:
            data = json.loads(live_path.read_text())
        except (OSError, ValueError) as exc:
            return {
                "running": status.get("running", False),
                "error": f"court_survey_live.json unreadable: {exc}",
                "map_points": [], "navigation_points": [],
            }
        age = time.time() - float(data.get("updated_at", 0.0) or 0.0)
        data["age_s"] = age
        data["stale"] = age > 3.0
        data["running"] = status.get("running", data.get("running", False))
        return data

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
        if not robot_status["stale"]:
            nested = robot_status.get("robot") if isinstance(robot_status.get("robot"), dict) else {}
            self.path_store.update({
                "x_m": nested.get("x_m", robot_status.get("robot_x_m")),
                "y_m": nested.get("y_m", robot_status.get("robot_y_m")),
                "yaw_rad": nested.get("yaw_rad", robot_status.get("robot_yaw_rad")),
            })
        return {
            "generated_at": time.time(),
            "command": self.store.read().to_mapping(),
            "robot": robot_status,
            "robot_path": self.path_store.read(),
            "court_boundary": self._read_court_boundary(),
            "court_survey_live": self._read_live_court_survey(),
            "court_survey_launch": self.survey_launch.status(),
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

    def _send_static(self, rel_path: str) -> None:
        # Serve split SPA assets (CSS, JS, view partials) from STATIC_DIR,
        # guarding against path traversal outside that directory.
        candidate = (STATIC_DIR / rel_path).resolve()
        try:
            candidate.relative_to(STATIC_DIR.resolve())
        except ValueError:
            self.send_error(HTTPStatus.FORBIDDEN)
            return
        if not candidate.is_file():
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        content_type = _STATIC_CONTENT_TYPES.get(
            candidate.suffix.lower(), "application/octet-stream"
        )
        try:
            payload = candidate.read_bytes()
        except OSError:
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store, max-age=0")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        try:
            self.wfile.write(payload)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _send_json(self, data: dict[str, object]) -> None:
        payload = json.dumps(data).encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store, max-age=0")
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
    ControlPanelHandler.survey_launch = CourtSurveyLaunchManager(ControlPanelHandler.store)
    ControlPanelHandler.path_store = PathHistoryStore.from_env()
    server = ThreadingHTTPServer((args.host, args.port), ControlPanelHandler)
    print(f"remote console listening on http://{args.host}:{args.port}")
    print(f"command file: {ControlPanelHandler.store.path}")
    print(f"status file: {ControlPanelHandler.status_store.path}")
    print(f"sensor file: {ControlPanelHandler.sensor_store.path}")
    server.serve_forever()


if __name__ == "__main__":
    main()
