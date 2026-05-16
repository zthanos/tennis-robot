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

from perception import detect_largest_ball, estimate_ball_observation  # noqa: E402


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

    print(
        "perception smoke ok: "
        f"bbox=({detection.x},{detection.y},{detection.width},{detection.height}) "
        f"distance={observation.distance_m:.2f}m "
        f"bearing={math.degrees(observation.bearing_rad):.1f}deg"
    )


if __name__ == "__main__":
    main()
