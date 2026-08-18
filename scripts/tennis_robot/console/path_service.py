"""PathService — robot path-history persistence (runtime/robot_path.json).

Owns the on-disk path trail: append-with-min-step, cap, read. Pure file I/O for
one artifact; injected with its target path.
"""

from __future__ import annotations

import json
import math
import os
import threading
import time
from pathlib import Path

from .config import ConsoleConfig


class PathService:
    def __init__(self, path: Path, max_points: int = 2000, min_step_m: float = 0.04) -> None:
        self.path = path
        self.max_points = max_points
        self.min_step_m = min_step_m
        self._lock = threading.Lock()
        self.path.parent.mkdir(parents=True, exist_ok=True)

    @classmethod
    def from_config(cls, config: ConsoleConfig) -> "PathService":
        default_path = config.robot_path_path
        return cls(Path(os.getenv("TENNIS_ROBOT_PATH_FILE", str(default_path))))

    def read(self) -> list[dict[str, float]]:
        with self._lock:
            return self._read_unlocked()

    @staticmethod
    def display_sample(
        points: list[dict[str, float]], max_points: int = 200
    ) -> list[dict[str, float]]:
        """Return a deterministic, endpoint-preserving canvas projection.

        The persisted/full audit remains available from ``/api/path``.  A
        browser canvas does not benefit from retransmitting all 2,000 points
        every second, especially when many are sub-pixel at the rendered zoom.
        """

        if max_points < 2:
            raise ValueError("max_points must be >= 2")
        if len(points) <= max_points:
            return list(points)
        last = len(points) - 1
        indexes = [round(index * last / (max_points - 1)) for index in range(max_points)]
        return [points[index] for index in indexes]

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
