#!/usr/bin/env python3
"""Exercise ball detection and monocular range estimation on a synthetic frame."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "ball_detector"))

from perception import (  # noqa: E402
    CameraMount,
    RobotPose2D,
    detect_largest_ball,
    estimate_ball_observation,
    observation_to_robot_xy,
    observation_to_world,
    robot_xy_to_world,
)


def main() -> None:
    frame = np.zeros((360, 640, 3), dtype=np.uint8)
    cv2.circle(frame, (360, 190), 18, (30, 230, 210), -1)

    detection = detect_largest_ball(frame)
    if detection is None:
        raise SystemExit("expected synthetic tennis ball to be detected")

    observation = estimate_ball_observation(detection, frame.shape[1], camera_fov_rad=1.05)
    if not 1.0 < observation.distance_m < 1.6:
        raise SystemExit(f"unexpected distance estimate: {observation.distance_m:.3f}m")
    if not 0.03 < observation.bearing_rad < 0.09:
        raise SystemExit(f"unexpected bearing estimate: {math.degrees(observation.bearing_rad):.2f}deg")

    robot_xy = observation_to_robot_xy(observation, CameraMount(x_m=0.42, y_m=0.0))
    if not 1.42 < robot_xy[0] < 2.02:
        raise SystemExit(f"unexpected robot-frame x: {robot_xy[0]:.3f}m")
    if not 0.04 < robot_xy[1] < 0.16:
        raise SystemExit(f"unexpected robot-frame y: {robot_xy[1]:.3f}m")

    world_xy = robot_xy_to_world(robot_xy[0], robot_xy[1], RobotPose2D(x_m=-8.0, y_m=0.0, yaw_rad=0.0))
    if not -6.58 < world_xy[0] < -5.98:
        raise SystemExit(f"unexpected world-frame x: {world_xy[0]:.3f}m")
    if not 0.04 < world_xy[1] < 0.16:
        raise SystemExit(f"unexpected world-frame y: {world_xy[1]:.3f}m")

    world_observation = observation_to_world(
        observation,
        RobotPose2D(x_m=-8.0, y_m=0.0, yaw_rad=math.pi / 2),
        CameraMount(x_m=0.42, y_m=0.0),
    )
    if not -8.16 < world_observation.world_x_m < -8.04:
        raise SystemExit(f"unexpected turned world x: {world_observation.world_x_m:.3f}m")
    if not 1.42 < world_observation.world_y_m < 2.02:
        raise SystemExit(f"unexpected turned world y: {world_observation.world_y_m:.3f}m")

    print(
        "perception smoke ok: "
        f"bbox=({detection.x},{detection.y},{detection.width},{detection.height}) "
        f"mono_distance={observation.distance_m:.2f}m "
        f"bearing={math.degrees(observation.bearing_rad):.1f}deg "
        f"world=({world_xy[0]:.2f},{world_xy[1]:.2f})"
    )


if __name__ == "__main__":
    main()
