"""ConsoleApp — application/use-case layer.

Holds the services and exposes use-case methods to the HTTP controller. This is
where cross-service orchestration lives (e.g. nav_test = bounds check via
SurveyService then preflight+send via RosService; build_diagnostics = assemble
stores + services and persist completed surveys). The controller contains no
business logic; the services contain no knowledge of each other.

Use-case results that have several HTTP outcomes return a small NavTestOutcome
value object with a transport-agnostic ``kind``; the controller maps kind to an
HTTP status. No HTTP types leak into this layer.
"""

from __future__ import annotations

import json
import math
import time
from collections import Counter
from dataclasses import dataclass

from .camera_service import CameraService
from .config import ConsoleConfig, text_from_subprocess_output
from .database_service import DatabaseService
from .path_service import PathService
from .ros_service import RosService
from .survey_service import SurveyService


@dataclass
class NavTestOutcome:
    kind: str  # out_of_bounds | preflight_timeout | not_ready | timeout | sent
    payload: dict


class ConsoleApp:
    def __init__(
        self,
        *,
        config: ConsoleConfig,
        command_store,
        status_store,
        sensor_store,
        ros: RosService,
        survey: SurveyService,
        path: PathService,
        camera: CameraService,
        db: DatabaseService,
    ) -> None:
        self.config = config
        self.command_store = command_store
        self.status_store = status_store
        self.sensor_store = sensor_store
        self.ros = ros
        self.survey = survey
        self.path = path
        self.camera = camera
        self.db = db

    # ------------------------------------------------------------------
    # simple reads (exposed so the controller stays thin)
    # ------------------------------------------------------------------
    def command_status(self) -> dict:
        return self.command_store.read().to_mapping()

    def robot_status(self) -> dict:
        return self.status_store.read()

    def sensors(self) -> dict:
        return self.sensor_store.read()

    def history(self) -> dict:
        return {"history": self.command_store.read_history()}

    def path_points(self) -> dict:
        return {"points": self.path.read()}

    def vendors(self) -> dict:
        return self.db.read_all()

    def surveys(self) -> dict:
        return {"surveys": self.db.surveys()}

    def survey_archive(self) -> dict:
        return {"surveys": self.db.survey_archive()}

    def webcam_frame(self) -> dict:
        return self.camera.frame()

    # ------------------------------------------------------------------
    # commands
    # ------------------------------------------------------------------
    def set_command(self, mode: str):
        """Apply a high-level command. Starting/stopping the survey launch is
        tied to the map_court/idle modes. Returns the written command object."""
        if mode == "map_court":
            self.ros.start_survey()
        elif mode == "idle":
            self.ros.stop_survey()
        # Validation happens inside the command store; unknown modes map to idle.
        return self.command_store.write(mode)

    def clear_path(self) -> None:
        self.path.clear()

    def save_vendors(self, data: dict) -> None:
        self.db.write_all(data)

    # ------------------------------------------------------------------
    # nav test (orchestration across SurveyService + RosService)
    # ------------------------------------------------------------------
    def nav_test(self, x_m: float, y_m: float, yaw_rad: float) -> NavTestOutcome:
        within, bounds = self.survey.check_bounds(x_m, y_m)
        goal = {"x_m": x_m, "y_m": y_m, "yaw_rad": yaw_rad}
        if not within:
            return NavTestOutcome("out_of_bounds", {
                "ok": False, "out_of_bounds": True,
                "message": "Goal is outside the surveyed court bounds.",
                "goal": goal, "bounds": bounds,
            })

        preflight = self.ros.nav_preflight()
        if preflight.timed_out:
            return NavTestOutcome("preflight_timeout", {
                "ok": False,
                "message": "Nav2 preflight timed out; NavigateToPose is not ready.",
                "output": preflight.output[-4000:],
            })
        if not preflight.ready:
            return NavTestOutcome("not_ready", {
                "ok": False,
                "message": "Nav2 NavigateToPose action server is not active yet.",
                "output": preflight.output[-4000:],
            })

        result, timed_out = self.ros.nav_send_goal(self._goal_json(x_m, y_m, yaw_rad))
        if timed_out:
            return NavTestOutcome("timeout", {
                "ok": False, "timeout": True,
                "message": "NavigateToPose is still running; use Cancel to abort the goal.",
                "goal": goal,
            })

        output = text_from_subprocess_output(result.stdout)
        return NavTestOutcome("sent", {
            "ok": result.returncode == 0,
            "returncode": result.returncode,
            "goal": goal,
            "accepted": "Goal accepted" in output,
            "canceled": "status: CANCELED" in output or "Goal was canceled" in output,
            "succeeded": "status: SUCCEEDED" in output or "Goal succeeded" in output,
            "output": output[-4000:],
        })

    def nav_cancel(self) -> dict:
        canceled = self.ros.nav_cancel()
        return {
            "ok": True,
            "canceled": canceled,
            "message": "Cancel requested." if canceled else "No active Nav Test goal to cancel.",
        }

    @staticmethod
    def _goal_json(x_m: float, y_m: float, yaw_rad: float) -> str:
        half = yaw_rad / 2.0
        goal = {
            "pose": {
                "header": {"frame_id": "map"},
                "pose": {
                    "position": {"x": x_m, "y": y_m, "z": 0.0},
                    "orientation": {"z": math.sin(half), "w": math.cos(half)},
                },
            }
        }
        return json.dumps(goal, separators=(",", ":"))

    # ------------------------------------------------------------------
    # diagnostics (assembles everything for the dashboard)
    # ------------------------------------------------------------------
    def build_diagnostics(self) -> dict[str, object]:
        history = self.command_store.read_history(200)
        by_mode = Counter(str(row.get("mode", "unknown")) for row in history)
        latest_by_mode: dict[str, dict[str, object]] = {}
        for row in history:
            latest_by_mode[str(row.get("mode", "unknown"))] = row

        robot_status = self.status_store.read()
        robot_updated_at = float(robot_status.get("updated_at", 0.0) or 0.0)
        robot_status["age_s"] = time.time() - robot_updated_at if robot_updated_at > 0 else None
        robot_status["stale"] = robot_updated_at <= 0 or robot_status["age_s"] > 3.0
        robot_status["connected"] = bool(robot_status.get("connected")) and not robot_status["stale"]
        self._maybe_save_obstacle_run(robot_status)
        if not robot_status["stale"]:
            nested = robot_status.get("robot") if isinstance(robot_status.get("robot"), dict) else {}
            self.path.update({
                "x_m": nested.get("x_m", robot_status.get("robot_x_m")),
                "y_m": nested.get("y_m", robot_status.get("robot_y_m")),
                "yaw_rad": nested.get("yaw_rad", robot_status.get("robot_yaw_rad")),
            })

        launch_status = self.ros.survey_status()
        return {
            "generated_at": time.time(),
            "command": self.command_store.read().to_mapping(),
            "robot": robot_status,
            "robot_path": self.path.read(),
            "court_boundary": self._diag_court_boundary(),
            "court_survey_live": self.survey.read_live_survey(launch_status),
            "court_survey_launch": launch_status,
            "obstacle_runs": self.db.obstacle_runs(20),
            "history": history[-50:],
            "stats": {
                "total": len(history),
                "by_mode": dict(by_mode),
                "latest_by_mode": latest_by_mode,
            },
        }

    def _diag_court_boundary(self) -> dict | None:
        """Read the persisted boundary and, when a survey has completed, import
        it into the DB tagged to the active court (pruned to last 10 elsewhere)."""
        bounds = self.survey.read_court_boundary()
        if self.survey.survey_is_complete(bounds):
            act = self.db.active_session()
            self.db.import_survey(bounds, court_id=act.get("court_id"), vendor_id=act.get("vendor_id"))
        return bounds

    def _maybe_save_obstacle_run(self, robot_status: dict) -> None:
        """Persist a completed ObstacleSurvey run to DB (idempotent)."""
        nav = ((robot_status.get("survey") or {}).get("navigation") or {})
        obs = nav.get("obstacle_survey")
        if not obs:
            return
        if (obs.get("result") or {}).get("status") not in ("SUCCESS", "FAILED"):
            return
        try:
            self.db.save_obstacle_run(obs)
        except Exception:
            pass
