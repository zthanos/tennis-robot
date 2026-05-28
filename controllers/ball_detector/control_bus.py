"""Small file-backed command bus shared by the web UI and Webots controller."""

from __future__ import annotations

import json
import os
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any


SUPPORTED_MODES = {"idle", "collect", "survey"}
PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_COMMAND_FILE = PROJECT_ROOT / "runtime" / "robot_command.json"
DEFAULT_COMMAND_HISTORY_FILE = PROJECT_ROOT / "runtime" / "robot_command_history.jsonl"
DEFAULT_STATUS_FILE = PROJECT_ROOT / "runtime" / "robot_status.json"
DEFAULT_SENSOR_FILE = PROJECT_ROOT / "runtime" / "robot_sensors.json"


@dataclass(frozen=True)
class RobotControlCommand:
    mode: str = "idle"
    sequence: int = 0
    updated_at: float = 0.0
    source: str = "default"

    @classmethod
    def from_mapping(cls, data: dict[str, Any]) -> "RobotControlCommand":
        mode = str(data.get("mode", "idle")).strip().lower()
        if mode not in SUPPORTED_MODES:
            mode = "idle"
        return cls(
            mode=mode,
            sequence=int(data.get("sequence", 0)),
            updated_at=float(data.get("updated_at", 0.0)),
            source=str(data.get("source", "unknown")),
        )

    def to_mapping(self) -> dict[str, Any]:
        return asdict(self)


class RobotCommandStore:
    def __init__(self, path: Path = DEFAULT_COMMAND_FILE, history_path: Path | None = None) -> None:
        self.path = path
        self.history_path = history_path or path.with_name("robot_command_history.jsonl")

    @classmethod
    def from_env(cls) -> "RobotCommandStore":
        path = Path(os.getenv("ROBOT_COMMAND_FILE", str(DEFAULT_COMMAND_FILE)))
        history_path = Path(os.getenv("ROBOT_COMMAND_HISTORY_FILE", str(DEFAULT_COMMAND_HISTORY_FILE)))
        return cls(path, history_path)

    def read(self) -> RobotControlCommand:
        try:
            with self.path.open("r", encoding="utf-8") as handle:
                data = json.load(handle)
        except FileNotFoundError:
            return RobotControlCommand()
        except (OSError, json.JSONDecodeError, TypeError, ValueError):
            return RobotControlCommand(source="invalid")
        if not isinstance(data, dict):
            return RobotControlCommand(source="invalid")
        return RobotControlCommand.from_mapping(data)

    def write(self, mode: str, source: str = "web-ui") -> RobotControlCommand:
        current = self.read()
        command = RobotControlCommand(
            mode=mode if mode in SUPPORTED_MODES else "idle",
            sequence=current.sequence + 1,
            updated_at=time.time(),
            source=source,
        )
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp_path = self.path.with_name(f"{self.path.name}.{os.getpid()}.{time.time_ns()}.tmp")
        with tmp_path.open("w", encoding="utf-8") as handle:
            json.dump(command.to_mapping(), handle, indent=2, sort_keys=True)
            handle.write("\n")
        tmp_path.replace(self.path)
        self._append_history(command)
        return command

    def read_history(self, limit: int = 100) -> list[dict[str, Any]]:
        try:
            with self.history_path.open("r", encoding="utf-8") as handle:
                lines = handle.readlines()
        except FileNotFoundError:
            return []
        except OSError:
            return []

        rows: list[dict[str, Any]] = []
        for line in lines[-max(1, limit) :]:
            try:
                data = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(data, dict):
                rows.append(data)
        return rows

    def _append_history(self, command: RobotControlCommand) -> None:
        try:
            self.history_path.parent.mkdir(parents=True, exist_ok=True)
            with self.history_path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(command.to_mapping(), sort_keys=True))
                handle.write("\n")
        except OSError:
            return None


class RobotStatusStore:
    def __init__(self, path: Path = DEFAULT_STATUS_FILE) -> None:
        self.path = path

    @classmethod
    def from_env(cls) -> "RobotStatusStore":
        path = Path(os.getenv("ROBOT_STATUS_FILE", str(DEFAULT_STATUS_FILE)))
        return cls(path)

    def read(self) -> dict[str, Any]:
        try:
            with self.path.open("r", encoding="utf-8") as handle:
                data = json.load(handle)
        except FileNotFoundError:
            return {"connected": False}
        except (OSError, json.JSONDecodeError, TypeError, ValueError):
            return {"connected": False, "source": "invalid"}
        if not isinstance(data, dict):
            return {"connected": False, "source": "invalid"}
        data["connected"] = True
        return data

    def write(self, status: dict[str, Any]) -> None:
        payload = dict(status)
        payload["updated_at"] = time.time()
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            tmp_path = self.path.with_name(f"{self.path.name}.{os.getpid()}.{time.time_ns()}.tmp")
            with tmp_path.open("w", encoding="utf-8") as handle:
                json.dump(payload, handle, indent=2, sort_keys=True)
                handle.write("\n")
            tmp_path.replace(self.path)
        except OSError:
            try:
                tmp_path.unlink(missing_ok=True)
            except OSError:
                pass


class RobotSensorStore(RobotStatusStore):
    def __init__(self, path: Path = DEFAULT_SENSOR_FILE) -> None:
        super().__init__(path)

    @classmethod
    def from_env(cls) -> "RobotSensorStore":
        path = Path(os.getenv("ROBOT_SENSOR_FILE", str(DEFAULT_SENSOR_FILE)))
        return cls(path)
