#!/usr/bin/env python3
"""Local web console for controlling and observing the tennis robot simulation.

Thin entrypoint: it builds the dependency graph (config -> stores + services ->
ConsoleApp -> ConsoleServer) and starts serving. All behaviour lives in the
``tennis_robot.console`` package; see that package's docstring / CLAUDE.md for
the layered architecture (Controller -> App -> Services, dependency-injected).
"""

from __future__ import annotations

import argparse
import errno
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from tennis_robot.control_bus import RobotCommandStore, RobotSensorStore, RobotStatusStore  # noqa: E402
from db_store import TennisRobotDB  # noqa: E402

from tennis_robot.console.app import ConsoleApp  # noqa: E402
from tennis_robot.console.camera_service import CameraService  # noqa: E402
from tennis_robot.console.config import ConsoleConfig  # noqa: E402
from tennis_robot.console.database_service import DatabaseService  # noqa: E402
from tennis_robot.console.path_service import PathService  # noqa: E402
from tennis_robot.console.ros_service import RosService  # noqa: E402
from tennis_robot.console.server import ConsoleServer  # noqa: E402
from tennis_robot.console.survey_service import SurveyService  # noqa: E402


def build_app(config: ConsoleConfig, args: argparse.Namespace) -> ConsoleApp:
    """Compose stores + services into a ConsoleApp (single wiring point)."""
    command_store = RobotCommandStore(args.command_file) if args.command_file else RobotCommandStore.from_env()
    status_store = RobotStatusStore(args.status_file) if args.status_file else RobotStatusStore.from_env()
    sensor_store = RobotSensorStore.from_env()
    db = TennisRobotDB(args.db_file) if args.db_file else TennisRobotDB()

    return ConsoleApp(
        config=config,
        command_store=command_store,
        status_store=status_store,
        sensor_store=sensor_store,
        ros=RosService(config, command_store=command_store),
        survey=SurveyService(config),
        path=PathService.from_config(config),
        camera=CameraService(),
        db=DatabaseService(db),
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the tennis robot remote console.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8081)
    parser.add_argument("--command-file", type=Path, default=None)
    parser.add_argument("--status-file", type=Path, default=None)
    parser.add_argument("--db-file", type=Path, default=None)
    args = parser.parse_args()

    config = ConsoleConfig(root=ROOT, host=args.host, port=args.port)
    # control_bus derives its DEFAULT_* paths from its own __file__, which (via
    # the tennis_robot path shim) resolves to the ROS source tree, not the
    # project root. Pin the IPC files to the real <project>/runtime/ so the
    # console and the controller node agree. setdefault respects any override
    # already set in the environment.
    os.environ.setdefault("ROBOT_COMMAND_FILE", str(config.runtime_dir / "robot_command.json"))
    os.environ.setdefault("ROBOT_COMMAND_HISTORY_FILE", str(config.runtime_dir / "robot_command_history.jsonl"))
    os.environ.setdefault("ROBOT_STATUS_FILE", str(config.runtime_dir / "robot_status.json"))
    os.environ.setdefault("ROBOT_SENSOR_FILE", str(config.runtime_dir / "robot_sensors.json"))

    app = build_app(config, args)
    try:
        server = ConsoleServer((config.host, config.port), app)
    except OSError as exc:
        if exc.errno == errno.EADDRINUSE:
            print(
                f"port {config.port} is already in use — the docker control panel is "
                f"probably serving it.\nOpen http://{config.host}:{config.port} to use it, "
                f"or rerun with --port <N> for a separate instance.",
                file=sys.stderr,
            )
            raise SystemExit(1)
        raise

    print(f"remote console listening on http://{config.host}:{config.port}")
    print(f"command file: {app.command_store.path}")
    print(f"status file: {app.status_store.path}")
    print(f"sensor file: {app.sensor_store.path}")
    server.serve_forever()


if __name__ == "__main__":
    main()
