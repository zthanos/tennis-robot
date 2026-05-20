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
    distance_source: str = "monocular"

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
    return BallObservation(
        detection=detection,
        bearing_rad=bearing_rad,
        distance_m=distance_m,
        distance_source="monocular",
    )


def estimate_depth_ball_observation(
    detection: BallDetection,
    depth_frame_m: np.ndarray,
    rgb_frame_width_px: int,
    rgb_frame_height_px: int,
    camera_fov_rad: float,
) -> BallObservation | None:
    """Estimate ball distance from a depth/range frame aligned to the RGB camera."""

    if depth_frame_m.ndim != 2:
        raise ValueError("depth_frame_m must be a 2D array of meter distances")

    depth_height, depth_width = depth_frame_m.shape
    scale_x = depth_width / rgb_frame_width_px
    scale_y = depth_height / rgb_frame_height_px
    x0 = max(0, int(detection.x * scale_x))
    x1 = min(depth_width, int((detection.x + detection.width) * scale_x))
    y0 = max(0, int(detection.y * scale_y))
    y1 = min(depth_height, int((detection.y + detection.height) * scale_y))
    if x0 >= x1 or y0 >= y1:
        return None

    crop = depth_frame_m[y0:y1, x0:x1]
    valid = crop[np.isfinite(crop) & (crop > 0)]
    if valid.size == 0:
        return None

    normalized_x = (detection.center_x - rgb_frame_width_px / 2) / (rgb_frame_width_px / 2)
    bearing_rad = math.atan(normalized_x * math.tan(camera_fov_rad / 2))
    return BallObservation(
        detection=detection,
        bearing_rad=bearing_rad,
        distance_m=float(np.median(valid)),
        distance_source="depth",
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
