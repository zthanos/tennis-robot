"""Tennis ball perception helpers shared by controllers and smoke tests."""

from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np


TENNIS_BALL_DIAMETER_M = 0.067
MIN_BALL_AREA_PX = 6
MIN_BALL_ASPECT_RATIO = 0.55
MAX_BALL_ASPECT_RATIO = 1.8

# Tennis balls vary from bright yellow-green to shadowed/desaturated green in Webots.
HSV_RANGES = (
    (np.array([25, 80, 80], dtype=np.uint8), np.array([85, 255, 255], dtype=np.uint8)),
    (np.array([25, 35, 45], dtype=np.uint8), np.array([95, 180, 170], dtype=np.uint8)),
)


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
    distance_source: str = "unknown"

    @property
    def x_m(self) -> float:
        return self.distance_m * math.cos(self.bearing_rad)

    @property
    def y_m(self) -> float:
        return self.distance_m * math.sin(self.bearing_rad)


@dataclass(frozen=True)
class CameraMount:
    x_m: float = 0.31
    y_m: float = 0.0
    yaw_rad: float = 0.0


@dataclass(frozen=True)
class RobotPose2D:
    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class BallWorldObservation:
    observation: BallObservation
    robot_x_m: float
    robot_y_m: float
    world_x_m: float
    world_y_m: float

    @property
    def court_x_m(self) -> float:
        return self.world_x_m

    @property
    def court_y_m(self) -> float:
        return self.world_y_m


def detect_largest_ball(frame: np.ndarray) -> BallDetection | None:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lower, upper in HSV_RANGES:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates: list[BallDetection] = []
    for contour in contours:
        if cv2.contourArea(contour) <= MIN_BALL_AREA_PX:
            continue
        x, y, width, height = cv2.boundingRect(contour)
        aspect_ratio = width / max(1, height)
        if not MIN_BALL_ASPECT_RATIO <= aspect_ratio <= MAX_BALL_ASPECT_RATIO:
            continue
        candidates.append(BallDetection(x, y, width, height))
    if not candidates:
        return None
    return max(candidates, key=lambda detection: detection.area_px)


def estimate_depth_ball_observation(
    detection: BallDetection,
    depth_frame_m: np.ndarray,
    frame_width_px: int,
    frame_height_px: int,
    camera_fov_rad: float,
    roi_scale: float = 0.55,
) -> BallObservation | None:
    """Estimate ball bearing/range from an RGB detection and aligned depth frame."""

    if depth_frame_m.size == 0:
        return None
    depth_height, depth_width = depth_frame_m.shape[:2]
    scale_x = depth_width / max(1, frame_width_px)
    scale_y = depth_height / max(1, frame_height_px)
    cx = detection.center_x * scale_x
    cy = detection.center_y * scale_y
    half_w = max(1, int(detection.width * scale_x * roi_scale / 2))
    half_h = max(1, int(detection.height * scale_y * roi_scale / 2))
    x0 = max(0, int(round(cx)) - half_w)
    x1 = min(depth_width, int(round(cx)) + half_w + 1)
    y0 = max(0, int(round(cy)) - half_h)
    y1 = min(depth_height, int(round(cy)) + half_h + 1)
    roi = depth_frame_m[y0:y1, x0:x1]
    valid = roi[np.isfinite(roi) & (roi > 0)]
    if valid.size == 0:
        return None

    normalized_x = (detection.center_x - frame_width_px / 2) / (frame_width_px / 2)
    bearing_rad = math.atan(normalized_x * math.tan(camera_fov_rad / 2))
    return BallObservation(
        detection=detection,
        bearing_rad=bearing_rad,
        distance_m=float(np.median(valid)),
        distance_source="oak_depth",
    )


def observation_to_robot_xy(
    observation: BallObservation,
    camera_mount: CameraMount = CameraMount(),
) -> tuple[float, float]:
    """Project a camera-relative ball observation into the robot base frame."""

    bearing_rad = observation.bearing_rad + camera_mount.yaw_rad
    return (
        camera_mount.x_m + observation.distance_m * math.cos(bearing_rad),
        camera_mount.y_m + observation.distance_m * math.sin(bearing_rad),
    )


def robot_xy_to_world(
    robot_x_m: float,
    robot_y_m: float,
    robot_pose: RobotPose2D,
) -> tuple[float, float]:
    """Transform robot-base XY coordinates into Webots/court world coordinates."""

    cos_yaw = math.cos(robot_pose.yaw_rad)
    sin_yaw = math.sin(robot_pose.yaw_rad)
    return (
        robot_pose.x_m + cos_yaw * robot_x_m - sin_yaw * robot_y_m,
        robot_pose.y_m + sin_yaw * robot_x_m + cos_yaw * robot_y_m,
    )


def observation_to_world(
    observation: BallObservation,
    robot_pose: RobotPose2D,
    camera_mount: CameraMount = CameraMount(),
) -> BallWorldObservation:
    robot_x_m, robot_y_m = observation_to_robot_xy(observation, camera_mount)
    world_x_m, world_y_m = robot_xy_to_world(robot_x_m, robot_y_m, robot_pose)
    return BallWorldObservation(
        observation=observation,
        robot_x_m=robot_x_m,
        robot_y_m=robot_y_m,
        world_x_m=world_x_m,
        world_y_m=world_y_m,
    )
