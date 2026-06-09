"""Pure-Python motion helpers shared by ROS nodes and replay tests."""

from __future__ import annotations

import math
from dataclasses import dataclass


def angle_delta_rad(a: float, b: float) -> float:
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class TurnTracker:
    """Track completion of a commanded turn from yaw feedback."""

    target_heading_rad: float | None = None
    target_delta_rad: float = 0.0
    direction: float = 0.0
    last_yaw_rad: float | None = None
    progress_rad: float = 0.0
    settle_count: int = 0

    def reset(
        self,
        target_heading_rad: float | None = None,
        target_delta_rad: float = 0.0,
        direction: float = 0.0,
    ) -> None:
        self.target_heading_rad = target_heading_rad
        self.target_delta_rad = max(0.0, target_delta_rad)
        self.direction = direction
        self.last_yaw_rad = None
        self.progress_rad = 0.0
        self.settle_count = 0

    def update(self, yaw_rad: float) -> None:
        if self.target_heading_rad is None:
            self.last_yaw_rad = None
            self.progress_rad = 0.0
            return
        if self.last_yaw_rad is None:
            self.last_yaw_rad = yaw_rad
            return
        step = angle_delta_rad(yaw_rad, self.last_yaw_rad)
        if abs(step) <= math.radians(35.0):
            progress = abs(step) if self.direction == 0.0 else step * self.direction
            self.progress_rad = max(0.0, self.progress_rad + progress)
            if self.target_delta_rad > 0.0:
                self.progress_rad = min(self.progress_rad, self.target_delta_rad)
        self.last_yaw_rad = yaw_rad

    def heading_error_rad(self, yaw_rad: float) -> float:
        if self.target_heading_rad is None:
            return 0.0
        return angle_delta_rad(self.target_heading_rad, yaw_rad)

    def complete(
        self,
        yaw_rad: float,
        heading_tolerance_rad: float,
        settle_ticks: int = 3,
    ) -> bool:
        if self.target_heading_rad is None:
            return True
        if abs(self.heading_error_rad(yaw_rad)) <= heading_tolerance_rad:
            self.settle_count += 1
        else:
            self.settle_count = 0
        return self.settle_count >= settle_ticks
