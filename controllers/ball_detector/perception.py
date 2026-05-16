"""Tennis ball perception helpers shared by controllers and smoke tests."""

from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np


TENNIS_BALL_DIAMETER_M = 0.067
MIN_BALL_AREA_PX = 20

# Tennis balls vary from yellow-green to green depending on lighting.
HSV_LOWER = np.array([25, 80, 80], dtype=np.uint8)
HSV_UPPER = np.array([85, 255, 255], dtype=np.uint8)


@dataclass(frozen=True)
class BallDetection:
    x: int
    y: int
    width: int
    height: int

    @property
    def area_px(self) -> int:
        return self.width * self.height

    @property
    def center_x(self) -> float:
        return self.x + self.width / 2

    @property
    def center_y(self) -> float:
        return self.y + self.height / 2

    @property
    def apparent_diameter_px(self) -> float:
        return (self.width + self.height) / 2


@dataclass(frozen=True)
class BallObservation:
    detection: BallDetection
    bearing_rad: float
    distance_m: float

    @property
    def x_m(self) -> float:
        return self.distance_m * math.cos(self.bearing_rad)

    @property
    def y_m(self) -> float:
        return self.distance_m * math.sin(self.bearing_rad)


def detect_largest_ball(frame: np.ndarray) -> BallDetection | None:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = [
        BallDetection(*cv2.boundingRect(contour))
        for contour in contours
        if cv2.contourArea(contour) > MIN_BALL_AREA_PX
    ]
    if not candidates:
        return None
    return max(candidates, key=lambda detection: detection.area_px)


def estimate_ball_observation(
    detection: BallDetection,
    frame_width_px: int,
    camera_fov_rad: float,
    ball_diameter_m: float = TENNIS_BALL_DIAMETER_M,
) -> BallObservation:
    focal_length_px = frame_width_px / (2 * math.tan(camera_fov_rad / 2))
    distance_m = ball_diameter_m * focal_length_px / detection.apparent_diameter_px
    normalized_x = (detection.center_x - frame_width_px / 2) / (frame_width_px / 2)
    bearing_rad = math.atan(normalized_x * math.tan(camera_fov_rad / 2))
    return BallObservation(detection=detection, bearing_rad=bearing_rad, distance_m=distance_m)
