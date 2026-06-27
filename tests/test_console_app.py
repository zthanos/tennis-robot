"""Tests for the refactored console application layer (tennis_robot.console).

These exercise ConsoleApp orchestration and the HTTP route table with fakes, so
they run without ROS 2, a webcam, or DuckDB. Run: `uv run pytest tests/test_console_app.py`.
"""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
from http import HTTPStatus
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from tennis_robot.console.app import ConsoleApp  # noqa: E402
from tennis_robot.console.camera_service import CameraService  # noqa: E402
from tennis_robot.console.config import ConsoleConfig  # noqa: E402
from tennis_robot.console.path_service import PathService  # noqa: E402
from tennis_robot.console.ros_service import NavPreflight  # noqa: E402
from tennis_robot.console.server import ControlPanelHandler  # noqa: E402
from tennis_robot.console.survey_service import SurveyService  # noqa: E402


class _Cmd:
    def to_mapping(self):
        return {"mode": "idle"}


class _FakeStore:
    def read(self):
        return _Cmd()

    def read_history(self, n=None):
        return []

    def write(self, mode):
        self.last = mode
        return _Cmd()


class _FakeStatus:
    def read(self):
        return {"updated_at": 0.0}


class _FakeDB:
    def __init__(self):
        self.imported = []

    def read_all(self):
        return {}

    def write_all(self, d):
        self.written = d

    def active_session(self):
        return {"court_id": 1, "vendor_id": 2}

    def surveys(self):
        return []

    def survey_archive(self):
        return []

    def import_survey(self, bounds, court_id=None, vendor_id=None):
        self.imported.append((court_id, vendor_id))

    def obstacle_runs(self, limit=20):
        return []

    def save_obstacle_run(self, obs):
        pass


class _FakeRos:
    def __init__(self):
        self.preflight = NavPreflight(timed_out=False, ready=True, output="Action servers: 1")
        self.send_result = (
            subprocess.CompletedProcess([], 0, stdout="Goal accepted\nstatus: SUCCEEDED"),
            False,
        )
        self.started = self.stopped = False

    def survey_status(self):
        return {"running": False}

    def start_survey(self):
        self.started = True
        return {}

    def stop_survey(self):
        self.stopped = True
        return {}

    def nav_preflight(self):
        return self.preflight

    def nav_send_goal(self, goal_json):
        return self.send_result

    def nav_cancel(self):
        return True


def _make(root: Path, ros: _FakeRos, db: _FakeDB) -> ConsoleApp:
    cfg = ConsoleConfig(root=root)
    cfg.runtime_dir.mkdir(parents=True, exist_ok=True)
    return ConsoleApp(
        config=cfg,
        command_store=_FakeStore(),
        status_store=_FakeStatus(),
        sensor_store=_FakeStore(),
        ros=ros,
        survey=SurveyService(cfg),
        path=PathService(cfg.robot_path_path),
        camera=CameraService(),
        db=db,
    )


def _write_boundary(root: Path, status: str = "OK") -> None:
    boundary = {
        "schema": "court_knowledge_model/v2",
        "status": status,
        "surveyed_at": 1.0,
        "fence": {"corners": [
            {"x_m": -8.582, "y_m": -8.549},
            {"x_m": 24.568, "y_m": -8.593},
            {"x_m": 24.591, "y_m": 8.657},
            {"x_m": -8.559, "y_m": 8.701},
        ]},
    }
    (root / "runtime" / "court_boundary.json").write_text(json.dumps(boundary))


def test_nav_test_out_of_bounds():
    with tempfile.TemporaryDirectory() as td:
        root = Path(td)
        app = _make(root, _FakeRos(), _FakeDB())
        _write_boundary(root)
        out = app.nav_test(100.0, 100.0, 0.0)
        assert out.kind == "out_of_bounds"
        assert out.payload["out_of_bounds"] is True


def test_nav_test_preflight_states():
    with tempfile.TemporaryDirectory() as td:
        root = Path(td)
        ros = _FakeRos()
        app = _make(root, ros, _FakeDB())
        _write_boundary(root)

        ros.preflight = NavPreflight(timed_out=False, ready=False, output="Action servers: 0")
        assert app.nav_test(0.0, 0.0, 0.0).kind == "not_ready"

        ros.preflight = NavPreflight(timed_out=True, ready=False, output="")
        assert app.nav_test(0.0, 0.0, 0.0).kind == "preflight_timeout"


def test_nav_test_send_outcomes():
    with tempfile.TemporaryDirectory() as td:
        root = Path(td)
        ros = _FakeRos()
        app = _make(root, ros, _FakeDB())
        _write_boundary(root)

        out = app.nav_test(0.0, 0.0, 1.57)
        assert out.kind == "sent"
        assert out.payload["ok"] is True and out.payload["succeeded"] is True

        ros.send_result = (None, True)
        assert app.nav_test(0.0, 0.0, 0.0).kind == "timeout"

        ros.send_result = (subprocess.CompletedProcess([], 1, stdout="error"), False)
        out = app.nav_test(0.0, 0.0, 0.0)
        assert out.kind == "sent" and out.payload["ok"] is False


def test_nav_test_allowed_without_survey():
    with tempfile.TemporaryDirectory() as td:
        # No boundary file -> bounds unknown -> goal allowed (preflight gates).
        app = _make(Path(td), _FakeRos(), _FakeDB())
        assert app.nav_test(999.0, 999.0, 0.0).kind == "sent"


def test_goal_json_quaternion():
    gj = json.loads(ConsoleApp._goal_json(1.0, 2.0, 0.0))
    assert gj["pose"]["header"]["frame_id"] == "map"
    assert gj["pose"]["pose"]["orientation"]["w"] == 1.0
    assert gj["pose"]["pose"]["orientation"]["z"] == 0.0


def test_build_diagnostics_and_db_import():
    with tempfile.TemporaryDirectory() as td:
        root = Path(td)
        db = _FakeDB()
        app = _make(root, _FakeRos(), db)
        _write_boundary(root, status="OK")
        diag = app.build_diagnostics()
        for key in ("generated_at", "command", "robot", "robot_path", "court_boundary",
                    "court_survey_live", "court_survey_launch", "obstacle_runs", "history", "stats"):
            assert key in diag
        # completed survey is imported tagged to the active court
        assert db.imported == [(1, 2)]


def test_set_command_drives_survey_launch():
    with tempfile.TemporaryDirectory() as td:
        ros = _FakeRos()
        app = _make(Path(td), ros, _FakeDB())
        app.set_command("map_court")
        app.set_command("idle")
        assert ros.started and ros.stopped


def test_get_routes_map_to_app_methods():
    for name in ControlPanelHandler.GET_JSON_ROUTES.values():
        assert hasattr(ConsoleApp, name), name


def test_nav_status_mapping():
    by_kind = ControlPanelHandler._NAV_STATUS_BY_KIND
    assert by_kind["out_of_bounds"] == HTTPStatus.BAD_REQUEST
    assert by_kind["preflight_timeout"] == HTTPStatus.SERVICE_UNAVAILABLE
    assert by_kind["not_ready"] == HTTPStatus.SERVICE_UNAVAILABLE
    assert by_kind["timeout"] == HTTPStatus.GATEWAY_TIMEOUT
