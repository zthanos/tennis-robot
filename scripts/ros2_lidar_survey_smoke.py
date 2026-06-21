#!/usr/bin/env python3
"""Smoke test for the ROS 2-only LiDAR Map Court behavior."""

from __future__ import annotations

import math
import sys
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "ball_detector"))
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "tennis_robot"))

from survey import SurveyVision  # noqa: E402
from tennis_robot.lidar_survey import (  # noqa: E402
    LidarSurveyConfig,
    LidarSurveyState,
    Ros2LidarCourtSurvey,
)


def scan_from_points(
    robot_x: float,
    robot_y: float,
    yaw: float,
    points: list[tuple[float, float]],
    count: int = 360,
) -> list[float]:
    ranges = [math.inf] * count
    cos_y = math.cos(-yaw)
    sin_y = math.sin(-yaw)
    for wx, wy in points:
        dx = wx - robot_x
        dy = wy - robot_y
        lx = cos_y * dx - sin_y * dy
        ly = sin_y * dx + cos_y * dy
        distance = math.hypot(lx, ly)
        if not (0.35 <= distance <= 12.0):
            continue
        theta = math.atan2(ly, lx)
        index = int(round(((theta + math.pi) / (2.0 * math.pi)) * count)) % count
        if distance < ranges[index]:
            ranges[index] = distance
    return ranges


def court_points() -> list[tuple[float, float]]:
    pts: list[tuple[float, float]] = []
    for x in [-11.8, -6.0, 0.0, 6.0, 11.8]:
        pts.extend([(x, -5.5), (x, 5.5)])
    for y in [-5.5, -2.5, 0.0, 2.5, 5.5]:
        pts.extend([(-11.8, y), (11.8, y)])
    for jitter in [-0.08, -0.03, 0.03, 0.08]:
        pts.extend([(0.0 + jitter, -5.5), (0.0 + jitter, 5.5)])
    return pts


def step_to(behavior: Ros2LidarCourtSurvey, pose: tuple[float, float, float], target: tuple[float, float]) -> tuple[float, float, float]:
    x, y, yaw = pose
    scan = scan_from_points(x, y, yaw, court_points())
    cmd = behavior.update(x, y, yaw, scan, 0.032, SurveyVision())
    dx = target[0] - x
    dy = target[1] - y
    if math.hypot(dx, dy) > 0.001:
        yaw = math.atan2(dy, dx)
        x += math.cos(yaw) * min(0.25, math.hypot(dx, dy))
        y += math.sin(yaw) * min(0.25, math.hypot(dx, dy))
    assert cmd.base.linear_speed_m_s >= 0.0
    return (x, y, yaw)


def main() -> None:
    output_path = Path(tempfile.gettempdir()) / "ros2_lidar_survey_smoke.json"
    output_path.unlink(missing_ok=True)
    cfg = LidarSurveyConfig(
        initial_scan_duration_s=0.2,
        far_scan_duration_s=0.2,
        net_confirm_duration_s=0.2,
        output_file=output_path,
    )
    behavior = Ros2LidarCourtSurvey(cfg)
    pose = (-5.0, 0.0, 0.0)

    for _ in range(10):
        behavior.update(*pose, scan_from_points(*pose, court_points()), 0.032, SurveyVision())
    assert behavior.state == LidarSurveyState.APPROACH_NET
    assert behavior.telemetry()["net_frame"] is not None
    assert behavior._approach_target is not None

    for _ in range(40):
        pose = step_to(behavior, pose, behavior._approach_target)
        if behavior.state == LidarSurveyState.CONFIRM_NET_VISUAL:
            break
    assert behavior.state == LidarSurveyState.CONFIRM_NET_VISUAL

    behavior.update(*pose, scan_from_points(*pose, court_points()), 0.032, SurveyVision(obstacle_class="net"))
    assert behavior.state == LidarSurveyState.CROSS_TO_FAR_SIDE
    assert behavior._cross_target is not None

    for _ in range(60):
        pose = step_to(behavior, pose, behavior._cross_target)
        if behavior.state == LidarSurveyState.FAR_SIDE_SCAN:
            break
    assert behavior.state == LidarSurveyState.FAR_SIDE_SCAN

    for _ in range(10):
        behavior.update(*pose, scan_from_points(*pose, court_points()), 0.032, SurveyVision())
    behavior.update(*pose, scan_from_points(*pose, court_points()), 0.032, SurveyVision())
    assert behavior.state == LidarSurveyState.DONE
    assert output_path.exists()
    assert behavior.court_bounds is not None
    assert behavior.court_bounds["survey_complete"] is True
    output_path.unlink(missing_ok=True)
    print("ros2 lidar survey smoke ok: lidar boundaries -> net visual confirm -> far-side lidar -> done")


if __name__ == "__main__":
    main()
