"""RosService — all ROS 2 CLI interaction for the console.

Consolidates two previously separate concerns that both shell out to ros2:
  * the Court Survey launch process (``ros2 launch``), and
  * Nav Test goals (``ros2 action send_goal /navigate_to_pose``) with cancel.

The survey process and the nav process are independent and tracked separately
so each can be started/stopped/cancelled without affecting the other. Goal
*bounds* validation does NOT live here — it is survey knowledge and belongs to
SurveyService; RosService only talks to ros2.
"""

from __future__ import annotations

import os
import shlex
import shutil
import signal
import subprocess
import threading
import time
import json
import socket
from dataclasses import dataclass

from .config import ConsoleConfig, text_from_subprocess_output


@dataclass
class NavPreflight:
    timed_out: bool
    ready: bool
    output: str


class RosService:
    # Nav goals may traverse the whole court; keep the client blocking for a
    # while, but the goal lives on the Nav2 server even past this timeout.
    NAV_SEND_TIMEOUT_S = 120.0
    NAV_PREFLIGHT_TIMEOUT_S = 8.0
    # How long a successful preflight is trusted, so back-to-back sends don't
    # each pay for an extra `ros2 action info` round-trip.
    PREFLIGHT_CACHE_S = 10.0

    def __init__(self, config: ConsoleConfig, command_store=None) -> None:
        self._cfg = config
        self._command_store = command_store
        # When ros2 is already on PATH (env sourced, e.g. inside the container)
        # invoke it directly — no `bash -lc` + double `source`, which adds
        # several seconds of latency to every nav command.
        self._ros2_on_path = shutil.which("ros2") is not None
        self._preflight_ok_until = 0.0
        self._collector_state = {"running": False, "speed": 10.0, "manual_override": False}

        # Survey launch process.
        self._survey_lock = threading.Lock()
        self._survey_process: subprocess.Popen | None = None
        self._survey_started_at: float | None = None
        self._survey_last_exit_code: int | None = None
        self._survey_last_finished_at: float | None = None

        # Nav-test send_goal process.
        self._nav_lock = threading.Lock()
        self._nav_process: subprocess.Popen | None = None

    def collector_control(self, action: str) -> dict[str, object]:
        allowed = {"start", "stop", "speed_up", "speed_down", "release"}
        if action not in allowed:
            return {"ok": False, "message": "Unknown collector action."}
        bridge = os.getenv("COLLECTOR_SERIAL_BRIDGE", "").strip()
        if bridge:
            host, port_text = bridge.rsplit(":", 1)
            try:
                with socket.create_connection((host, int(port_text)), timeout=3.0) as connection:
                    connection.sendall(
                        (json.dumps({"action": action}, separators=(",", ":")) + "\n").encode()
                    )
                    response = connection.makefile("r", encoding="utf-8").readline()
                state = json.loads(response)
                if state.get("ok"):
                    self._collector_state.update(
                        running=bool(state.get("running")),
                        speed=float(state.get("speed", 0)),
                        manual_override=True,
                    )
                return {**state, "manual_override": True, "action": action}
            except (OSError, ValueError, json.JSONDecodeError) as exc:
                return {"ok": False, "message": f"Collector serial bridge unavailable: {exc}"}

        payload = json.dumps({"action": action}, separators=(",", ":"))
        result = subprocess.run(
            self._ros2_argv([
                "topic", "pub", "--once", "/collector/manual_control",
                "std_msgs/msg/String", f"data: '{payload}'",
            ]),
            cwd=str(self._cfg.root),
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=8.0,
            check=False,
        )
        state = self._collector_state
        if action == "start":
            state.update(running=True, manual_override=True)
        elif action == "stop":
            state.update(running=False, manual_override=True)
        elif action == "speed_up":
            state["speed"] = min(40.0, float(state["speed"]) + 2.0)
        elif action == "speed_down":
            state["speed"] = max(-40.0, float(state["speed"]) - 2.0)
        elif action == "release":
            state["manual_override"] = False
        return {"ok": result.returncode == 0, **state, "action": action}

    def collector_status(self) -> dict[str, object]:
        return {"ok": True, **self._collector_state}

    def _ros2_argv(self, args: list[str]) -> list[str]:
        """Build subprocess argv for a ros2 command. Directly (`ros2 …`,
        inheriting the already-sourced env) when ros2 is on PATH; otherwise via
        a login shell that sources the ROS setup first."""
        if self._ros2_on_path:
            return ["ros2", *args]
        inner = self._cfg.ros_prelude + " ".join(shlex.quote(a) for a in ["ros2", *args])
        return ["bash", "-lc", inner]

    # ------------------------------------------------------------------
    # Court Survey launch
    # ------------------------------------------------------------------
    def start_survey(self) -> dict[str, object]:
        with self._survey_lock:
            if self._survey_process is not None and self._survey_process.poll() is None:
                return self.survey_status()
            self._cfg.survey_log_path.parent.mkdir(parents=True, exist_ok=True)
            log_file = self._cfg.survey_log_path.open("ab")
            self._survey_process = subprocess.Popen(
                self._ros2_argv(["launch", "tennis_robot", "court_survey.launch.py"]),
                cwd=str(self._cfg.root),
                stdout=log_file,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            self._survey_started_at = time.time()
            self._survey_last_exit_code = None
            threading.Thread(target=self._watch_survey, daemon=True).start()
            return self.survey_status()

    def stop_survey(self) -> dict[str, object]:
        with self._survey_lock:
            process = self._survey_process
            if process is not None and process.poll() is None:
                self._terminate_process_group(process)
            return self.survey_status()

    def survey_status(self) -> dict[str, object]:
        process = self._survey_process
        running = process is not None and process.poll() is None
        return {
            "running": running,
            "pid": process.pid if running and process is not None else None,
            "started_at": self._survey_started_at,
            "last_exit_code": self._survey_last_exit_code,
            "last_finished_at": self._survey_last_finished_at,
            "log_path": str(self._cfg.survey_log_path),
        }

    def _watch_survey(self) -> None:
        process = self._survey_process
        if process is None:
            return
        while process.poll() is None:
            if self._completed_boundary_after_start():
                self._terminate_process_group(process)
                break
            time.sleep(0.5)
        exit_code = self._wait_for_exit(process)
        with self._survey_lock:
            if self._survey_process is process:
                self._survey_last_exit_code = exit_code
                self._survey_last_finished_at = time.time()
                self._survey_process = None
        if self._command_store is not None:
            self._command_store.write("idle")

    def _completed_boundary_after_start(self) -> bool:
        if self._survey_started_at is None:
            return False
        try:
            import json
            data = json.loads(self._cfg.court_boundary_path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return False
        surveyed_at = float(data.get("surveyed_at") or 0.0)
        if surveyed_at < self._survey_started_at - 1.0:
            return False
        return data.get("status") in {"SUCCESS", "OK", "FAILED"} or bool(data.get("survey_complete"))

    # ------------------------------------------------------------------
    # Nav Test goals
    # ------------------------------------------------------------------
    def nav_preflight(self) -> NavPreflight:
        # Skip the round-trip if a recent preflight already confirmed readiness.
        if time.time() < self._preflight_ok_until:
            return NavPreflight(timed_out=False, ready=True, output="(cached)")
        try:
            result = subprocess.run(
                self._ros2_argv(["action", "info", "/navigate_to_pose"]),
                cwd=str(self._cfg.root),
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                timeout=self.NAV_PREFLIGHT_TIMEOUT_S,
                check=False,
            )
        except subprocess.TimeoutExpired as exc:
            return NavPreflight(timed_out=True, ready=False, output=text_from_subprocess_output(exc.stdout))
        output = text_from_subprocess_output(result.stdout)
        ready = result.returncode == 0 and "Action servers: 0" not in output
        if ready:
            self._preflight_ok_until = time.time() + self.PREFLIGHT_CACHE_S
        return NavPreflight(timed_out=False, ready=ready, output=output)

    def nav_send_goal(self, goal_json: str) -> tuple[subprocess.CompletedProcess | None, bool]:
        """Run ``ros2 action send_goal`` for the serialized goal. Returns
        (completed_process, timed_out); the process group is tracked so
        nav_cancel() can interrupt it."""
        process = subprocess.Popen(
            self._ros2_argv([
                "action", "send_goal", "/navigate_to_pose",
                "nav2_msgs/action/NavigateToPose", goal_json,
            ]),
            cwd=str(self._cfg.root),
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        self._set_nav_process(process)
        try:
            stdout, _ = process.communicate(timeout=self.NAV_SEND_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            # Goal still active on the Nav2 server. Keep it tracked for cancel
            # and clear the reference when it eventually exits.
            threading.Thread(target=self._reap_nav, args=(process,), daemon=True).start()
            return None, True
        self._set_nav_process(None)
        return (
            subprocess.CompletedProcess(process.args, process.returncode, stdout=stdout, stderr=None),
            False,
        )

    def nav_cancel(self) -> bool:
        """SIGINT the running send_goal process group; the ros2 action CLI
        issues a goal cancel on interrupt. True if a goal was active."""
        with self._nav_lock:
            process = self._nav_process
            if process is None or process.poll() is not None:
                return False
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                return False
            except OSError:
                process.terminate()
            return True

    def _set_nav_process(self, process: subprocess.Popen | None) -> None:
        with self._nav_lock:
            self._nav_process = process

    def _reap_nav(self, process: subprocess.Popen) -> None:
        try:
            process.wait()
        finally:
            with self._nav_lock:
                if self._nav_process is process:
                    self._nav_process = None

    # ------------------------------------------------------------------
    # process-group lifecycle helpers (shared)
    # ------------------------------------------------------------------
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
