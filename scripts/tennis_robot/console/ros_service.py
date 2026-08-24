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
import math
import re
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
    # A single-shot publish is not reliably delivered (see _publish_command).
    COMMAND_PUBLISH_TIMES = 5
    COMMAND_PUBLISH_RATE_HZ = 10
    # How long the short-lived publisher stays alive AFTER its last message, so
    # the reliable-delivery handshake can finish before the writer is destroyed.
    # Sized from the distributed PC<->Pi LAN, not from loopback: see
    # _publish_command.
    COMMAND_PUBLISH_KEEP_ALIVE_S = 1.0
    # Basket lift carriage. The speed must match the basket_joint velocity limit
    # in urdf/components/basket.urdf.xacro (and its ros2_control command range);
    # this service drives the joint at its rated speed and closes the position
    # loop from /joint_states.
    # Must match behavior_server.max_rotational_vel in config/nav2_params.yaml.
    SPIN_ANGULAR_SPEED_RAD_S = 0.25
    BASKET_LIFT_SPEED_MPS = 0.12
    BASKET_POSITION_TOLERANCE_M = 0.005
    BASKET_STATIONARY_TOLERANCE_MPS = 0.03

    def __init__(self, config: ConsoleConfig, command_store=None) -> None:
        self._cfg = config
        self._command_store = command_store
        # When ros2 is already on PATH (env sourced, e.g. inside the container)
        # invoke it directly — no `bash -lc` + double `source`, which adds
        # several seconds of latency to every nav command.
        self._ros2_on_path = shutil.which("ros2") is not None
        self._preflight_ok_until = 0.0
        self._collector_state = {"running": False, "speed": 10.0, "manual_override": False}
        self._joint_cache_at = 0.0
        self._joint_cache: dict[str, tuple[float, float]] = {}

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
        # Same delivery caveat as every other actuator command here — a "stop"
        # that silently never arrives is the worst possible failure mode.
        published = self._publish_command(
            "/collector/manual_control", "std_msgs/msg/String", f"data: '{payload}'"
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
        return {"ok": published, **state, "action": action}

    def collector_status(self) -> dict[str, object]:
        return {"ok": True, **self._collector_state}

    # ------------------------------------------------------------------
    # Throwing Mode actuator/navigation port
    # ------------------------------------------------------------------
    def runtime_kind(self) -> str:
        return os.getenv("TENNIS_ROBOT_RUNTIME", "hardware").strip().lower()

    def _distributed_brain(self) -> bool:
        return os.getenv("TENNIS_ROBOT_DISTRIBUTED_BRAIN", "false").strip().lower() \
            in {"1", "true", "yes"}

    def _publish_command(self, topic: str, msg_type: str, payload: str,
                         timeout_s: float = 12.0) -> bool:
        """Assert one actuator command, as a short burst that outlives delivery.

        `ros2 topic pub --once` tears its publisher down as soon as the message
        is handed to DDS, and the controller frequently never sees it: measured
        against a live Gazebo controller_manager, a single --once velocity
        command left basket_joint at rest, while five copies drove it through
        full travel. Commands are idempotent (a controller latches the last
        one), so repeating is free.

        Repeating is NOT enough once the controller is on another machine.
        `--times` already waits for a matching subscription, so the burst is
        written — but the process then exits immediately, and across the LAN the
        reliable handshake has not completed, so every sample is dropped. This
        is the distributed form of the same bug: measured Pi -> PC against a live
        flywheel_velocity_controller, `--times 5 -r 10` (~0.5 s of writer life)
        delivered NOTHING and the joint stayed at 0.0 rad/s, while the identical
        burst with `--keep-alive 3` reached 55.0 rad/s. Sweeping the value: 0.25 s
        still dropped commands, 0.5 s and 1.0 s delivered 3/3 each.

        So the writer is kept alive past its last message. This is a delivery
        guarantee, not a timeout: without it the command never arrives at all,
        and no amount of waiting on the feedback side would recover it.
        """
        try:
            result = subprocess.run(
                self._ros2_argv([
                    "topic", "pub",
                    "--times", str(self.COMMAND_PUBLISH_TIMES),
                    "-r", str(self.COMMAND_PUBLISH_RATE_HZ),
                    "--keep-alive", str(self.COMMAND_PUBLISH_KEEP_ALIVE_S),
                    topic, msg_type, payload,
                ]),
                cwd=str(self._cfg.root), text=True, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, timeout=timeout_s, check=False,
            )
        except (OSError, subprocess.TimeoutExpired):
            return False
        return result.returncode == 0

    def _controller_active(self, name: str) -> bool:
        try:
            result = subprocess.run(
                self._ros2_argv([
                    "service", "call", "/controller_manager/list_controllers",
                    "controller_manager_msgs/srv/ListControllers", "{}",
                ]),
                cwd=str(self._cfg.root), text=True, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, timeout=5.0, check=False,
            )
        except (OSError, subprocess.TimeoutExpired):
            return False
        if result.returncode != 0:
            return False
        match = re.search(
            rf"\bname(?:=|:)\s*['\"]?{re.escape(name)}['\"]?(.*?)(?=\bname(?:=|:)|$)",
            result.stdout, flags=re.DOTALL,
        )
        return bool(match and re.search(
            r"\bstate(?:=|:)\s*['\"]?active\b", match.group(1)
        ))

    def flywheel_available(self) -> bool:
        if self._distributed_brain():
            return self._joint_state("flywheel_left_joint") is not None \
                and self._joint_state("flywheel_right_joint") is not None
        return self._controller_active("flywheel_velocity_controller")

    def _read_joint_states(self) -> dict[str, tuple[float, float]]:
        now = time.monotonic()
        if now - self._joint_cache_at <= 1.0:
            return self._joint_cache
        try:
            import yaml
        except ImportError:
            return {}
        try:
            result = subprocess.run(
                self._ros2_argv(["topic", "echo", "--once", "/joint_states"]),
                cwd=str(self._cfg.root), text=True, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, timeout=5.0, check=False,
            )
            if result.returncode != 0:
                return {}
            # Jazzy may prepend ANSI diagnostics and best-effort packet-loss
            # notices before the actual YAML document. Parse only from the
            # message header onward; transport warnings are not joint data.
            output = re.sub(r"\x1b\[[0-?]*[ -/]*[@-~]", "", result.stdout)
            document_start = output.find("header:")
            if document_start < 0:
                return {}
            document = yaml.safe_load(output[document_start:].split("---", 1)[0])
            names = list(document.get("name") or [])
            positions = list(document.get("position") or [])
            velocities = list(document.get("velocity") or [])
            states = {
                str(name): (float(positions[index]), float(velocities[index]))
                for index, name in enumerate(names)
                if index < len(positions) and index < len(velocities)
            }
            self._joint_cache = states
            self._joint_cache_at = now
            return states
        except (OSError, subprocess.TimeoutExpired, ValueError, yaml.YAMLError,
                TypeError, AttributeError, IndexError):
            return {}

    def _joint_state(self, joint_name: str) -> tuple[float, float] | None:
        return self._read_joint_states().get(joint_name)

    def _basket_lift_travel(self) -> float:
        # Must match the basket_joint upper limit built into the model; the
        # generator reads the same variable with the same default.
        return float(os.getenv("BASKET_LIFT_TRAVEL_M", "0.100"))

    def _basket_controller_ready(self) -> bool:
        if self._distributed_brain():
            # controller_manager lives with Gazebo on the PC; bridged
            # /joint_states feedback is the only local evidence available.
            return self._joint_state("basket_joint") is not None
        return self._controller_active("basket_velocity_controller")

    def _command_basket_velocity(self, velocity_mps: float) -> bool:
        return self._publish_command(
            "/basket_velocity_controller/commands",
            "std_msgs/msg/Float64MultiArray", f"data: [{velocity_mps}]",
        )

    def stop_basket(self) -> bool:
        """Unconditionally halt the carriage. Safe to call from any state."""
        stopped = self._command_basket_velocity(0.0)
        self._joint_cache_at = 0.0
        return stopped

    def set_basket_position(self, raised: bool, timeout_s: float | None = None) -> bool:
        if self.runtime_kind() != "simulation":
            # Physical actuator driver + limit switches are not implemented.
            return False
        if not self._basket_controller_ready():
            return False
        travel = self._basket_lift_travel()
        target = travel if raised else 0.0
        if timeout_s is None:
            timeout_s = travel / self.BASKET_LIFT_SPEED_MPS + 5.0
        # The move runs closed-loop inside one rclpy process. Doing it here from
        # `ros2 topic echo` samples cannot work: each sample costs ~1.5 s, which
        # is 180 mm of travel at the carriage's rated speed, so the loop would
        # overshoot the endpoint and park the joint on a hard stop — and a
        # prismatic joint resting on its limit is exactly what gz-sim/DART
        # refuses to drive afterwards.
        # The mover's exit code IS the confirmation: it checks tracking velocity,
        # endpoint and settling against continuous feedback. Re-checking here
        # with a one-shot `ros2 topic echo` would only add a flakier duplicate
        # of a weaker test, and a dropped sample would report a good move as a
        # failure.
        moved = self._run_ros_module(
            "tennis_robot.basket_lift_mover",
            ["--target", f"{target}", "--timeout-s", f"{timeout_s}"],
            timeout_s=timeout_s + 20.0,
        )
        self._joint_cache_at = 0.0
        return moved

    def _python_module_argv(self, module: str, args: list[str]) -> list[str]:
        """argv running a tennis_robot module under an interpreter with rclpy.

        Same shape as _ros2_argv: direct when ROS is already sourced in this
        process's environment, otherwise through a login shell that sources it.
        """
        command = ["python3", "-m", module, *args]
        if self._ros2_on_path:
            return command
        inner = self._cfg.ros_prelude + " ".join(shlex.quote(a) for a in command)
        return ["bash", "-lc", inner]

    def _run_ros_module(self, module: str, args: list[str], timeout_s: float) -> bool:
        try:
            result = subprocess.run(
                self._python_module_argv(module, args),
                cwd=str(self._cfg.root), text=True, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, timeout=timeout_s, check=False,
            )
        except (OSError, subprocess.TimeoutExpired):
            return False
        return result.returncode == 0

    def basket_position_state(self) -> str:
        if self.runtime_kind() != "simulation":
            return "UNKNOWN"
        state = self._joint_state("basket_joint")
        if state is None:
            return "UNKNOWN"
        travel = self._basket_lift_travel()
        position, velocity = state
        if abs(velocity) > self.BASKET_STATIONARY_TOLERANCE_MPS:
            return "RAISING" if velocity > 0.0 else "LOWERING"
        # Deliberately looser than the tolerance set_basket_position settles to,
        # so a confirmed move never reports back as UNKNOWN.
        endpoint_tolerance = 2.0 * self.BASKET_POSITION_TOLERANCE_M
        if abs(position) <= endpoint_tolerance:
            return "LOWERED"
        if abs(position - travel) <= endpoint_tolerance:
            return "RAISED"
        return "UNKNOWN"

    def set_flywheel_speed(self, ball_speed_mps: float) -> bool:
        if not math.isfinite(ball_speed_mps) or ball_speed_mps < 0.0:
            return False
        if not self.flywheel_available():
            return ball_speed_mps == 0.0
        wheel_rad_s = min(320.0, ball_speed_mps / 0.10)
        return self._publish_command(
            "/flywheel_velocity_controller/commands",
            "std_msgs/msg/Float64MultiArray",
            f"data: [{wheel_rad_s}, {-wheel_rad_s}]",
        )

    def wait_flywheel_ready(self, ball_speed_mps: float, timeout_s: float = 8.0) -> bool:
        target = min(320.0, ball_speed_mps / 0.10)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            left = self._joint_state("flywheel_left_joint")
            right = self._joint_state("flywheel_right_joint")
            if left and right and abs(left[1] - target) <= 0.10 * max(target, 1.0) \
                    and abs(right[1] + target) <= 0.10 * max(target, 1.0):
                return True
            time.sleep(0.1)
        return False

    def request_ball_feed(self, *, session_id: str, throw_id: str,
                          target_zone: str, throw_speed_mps: float,
                          throw_angle_deg: float,
                          publish_at_unix: float | None = None) -> bool:
        # Phase 1 has no feeder actuator yet.  Publish an explicit event request
        # for the simulator/future feeder node; the UI labels it as placeholder
        # event simulation and never calls it measured physical telemetry.
        payload = json.dumps({
            "session_id": session_id,
            "throw_id": throw_id,
            "target_zone": target_zone,
            "throw_speed_mps": throw_speed_mps,
            "throw_angle_deg": throw_angle_deg,
            "count": 1,
        }, separators=(",", ":"))
        # A feed request is a discrete event, so it must NOT go through the
        # repeat-burst publisher used for idempotent setpoints: that turned one
        # throw into several requests, and still lost whole throws when more
        # than one node subscribed. Publish exactly once, after a subscriber is
        # actually matched, and let the consumer de-duplicate on throw_id.
        args = ["--topic", "/throwing/feed_request", "--json", payload]
        timeout_s = 20.0
        if publish_at_unix is not None:
            # The publisher holds the matched connection until this instant, so
            # its variable discovery cost (2.0-3.6 s measured) stops landing in
            # the emission time and the launch-to-launch cadence is the
            # configured period rather than period + jitter.
            args += ["--publish-at", f"{publish_at_unix}"]
            timeout_s = max(timeout_s, publish_at_unix - time.time() + 20.0)
        return self._run_ros_module(
            "tennis_robot.reliable_event_publish", args, timeout_s=timeout_s,
        )

    def navigate_to_pose(self, x_m: float, y_m: float, yaw_rad: float) -> dict[str, object]:
        preflight = self.nav_preflight()
        if preflight.timed_out or not preflight.ready:
            return {"succeeded": False, "message": "Nav2 NavigateToPose is not ready"}
        goal = {
            "pose": {"header": {"frame_id": "map"}, "pose": {
                "position": {"x": x_m, "y": y_m, "z": 0.0},
                "orientation": {"z": math.sin(yaw_rad / 2.0), "w": math.cos(yaw_rad / 2.0)},
            }}
        }
        result, timed_out = self.nav_send_goal(json.dumps(goal, separators=(",", ":")))
        if timed_out or result is None:
            return {"succeeded": False, "message": "navigation timed out"}
        output = text_from_subprocess_output(result.stdout)
        succeeded = result.returncode == 0 and (
            "status: SUCCEEDED" in output or "Goal succeeded" in output
        )
        return {"succeeded": succeeded, "message": output[-1000:]}

    def align_heading(self, target_yaw_rad: float, tolerance_rad: float,
                      timeout_s: float = 45.0) -> bool:
        """Rotate in place to an absolute map-frame heading.

        NavigateToPose cannot deliver a final heading here: the shared
        general_goal_checker sets yaw_goal_tolerance 3.14 so the collection
        lanes do not spin at each lane end, Regulated Pure Pursuit performs no
        final rotation, and Nav2's Spin behaviour aborts with COLLISION_AHEAD
        against a costmap measured clear (with no opt-out field in this Jazzy
        build). The rotation therefore runs closed-loop in its own process, the
        same shape as the basket mover — see tennis_robot.heading_aligner.
        """
        return self._run_ros_module(
            "tennis_robot.heading_aligner",
            ["--target-yaw", f"{target_yaw_rad}",
             "--tolerance-rad", f"{tolerance_rad}",
             "--timeout-s", f"{timeout_s}"],
            timeout_s=timeout_s + 20.0,
        )

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
