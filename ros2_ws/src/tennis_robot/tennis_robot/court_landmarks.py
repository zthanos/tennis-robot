"""Court landmark detection and coordinate transforms — pure Python, no ROS.

Provides:
  CourtLandmarks          — dataclass holding net pose + corner detections
  build_court_landmarks() — detect landmarks from a camera frame pair
  landmarks_to_robot_frame() — project camera-relative poses to robot base frame
  robot_landmarks_to_map() — transform robot-frame landmarks to map frame

Coordinate conventions
----------------------
Camera frame  : X right, Z forward (bearing = atan2(X, Z))
Robot frame   : X forward, Y left (ROS base_link convention)
Map frame     : ROS REP-103 (X forward, Y left, Z up)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from tennis_robot.perception import (
    CourtCornerDetection,
    NetPose,
    detect_court_corner,
    detect_net_pose,
)


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

@dataclass
class LandmarkPose:
    """2-D pose of a landmark in a chosen reference frame."""

    x_m: float
    y_m: float
    bearing_rad: float      # heading from the robot toward the landmark
    distance_m: float
    confidence: float
    label: str              # "net" | "corner"


@dataclass
class CourtLandmarks:
    """All landmarks detected in a single camera frame pair.

    net:     detected net pose, or None if not visible.
    corners: list of detected L-intersections (0-4 expected on a full court).
    Each entry is in *camera* frame when returned from build_court_landmarks().
    Call landmarks_to_robot_frame() to project to base_link, then
    robot_landmarks_to_map() to get map-frame poses suitable for Nav2.
    """

    net: LandmarkPose | None = None
    corners: list[LandmarkPose] = field(default_factory=list)
    frame_id: str = "camera_link"   # updated by transform helpers

    @property
    def all_landmarks(self) -> list[LandmarkPose]:
        out = []
        if self.net is not None:
            out.append(self.net)
        out.extend(self.corners)
        return out

    def has_net(self) -> bool:
        return self.net is not None

    def has_corners(self) -> bool:
        return len(self.corners) > 0


# ---------------------------------------------------------------------------
# Camera mount parameters (matches CameraMount in perception.py)
# ---------------------------------------------------------------------------

CAMERA_FORWARD_OFFSET_M: float = 0.31   # camera X offset from base_link origin
CAMERA_LATERAL_OFFSET_M: float = 0.00   # camera Y offset (centred)
CAMERA_YAW_OFFSET_RAD: float = 0.00     # camera yaw vs base_link heading
CAMERA_FOV_RAD: float = math.radians(69.0)   # OAK-D horizontal FoV


# ---------------------------------------------------------------------------
# Build from raw frames
# ---------------------------------------------------------------------------

def build_court_landmarks(
    frame: np.ndarray | None,
    depth_frame: np.ndarray | None = None,
    camera_fov_rad: float = CAMERA_FOV_RAD,
    depth_min_m: float = 0.1,
    depth_max_m: float = 12.0,
    max_corners: int = 4,
) -> CourtLandmarks:
    """Detect net and court corners from a BGR frame + aligned depth frame.

    Returns a CourtLandmarks in camera frame (frame_id = "camera_link").
    Poses without valid depth have distance_m=0.0 and low confidence.
    """
    if frame is None or frame.size == 0:
        return CourtLandmarks()

    # --- net ---
    net_lm: LandmarkPose | None = None
    net_raw: NetPose | None = detect_net_pose(
        frame, depth_frame, camera_fov_rad, depth_min_m, depth_max_m,
    )
    if net_raw is not None:
        dist = net_raw.distance_m if net_raw.distance_m is not None else 0.0
        net_lm = LandmarkPose(
            x_m=dist * math.sin(net_raw.bearing_rad),
            y_m=dist * math.cos(net_raw.bearing_rad),
            bearing_rad=net_raw.bearing_rad,
            distance_m=dist,
            confidence=net_raw.confidence,
            label="net",
        )

    # --- corners (run detect_court_corner repeatedly by masking already-found
    #     corners — simple: collect the single best per call; for PoC one call
    #     is sufficient since the robot sees at most one corner at a time) ---
    corner_lms: list[LandmarkPose] = []
    corner_raw: CourtCornerDetection | None = detect_court_corner(
        frame, depth_frame, camera_fov_rad, depth_min_m, depth_max_m,
    )
    if corner_raw is not None:
        dist = corner_raw.distance_m if corner_raw.distance_m is not None else 0.0
        corner_lms.append(LandmarkPose(
            x_m=dist * math.sin(corner_raw.bearing_rad),
            y_m=dist * math.cos(corner_raw.bearing_rad),
            bearing_rad=corner_raw.bearing_rad,
            distance_m=dist,
            confidence=corner_raw.confidence,
            label="corner",
        ))

    return CourtLandmarks(
        net=net_lm,
        corners=corner_lms[:max_corners],
        frame_id="camera_link",
    )


# ---------------------------------------------------------------------------
# Camera → robot base frame
# ---------------------------------------------------------------------------

def _camera_to_robot(lm: LandmarkPose) -> LandmarkPose:
    """Project a camera-frame LandmarkPose into ROS base_link frame.

    Camera convention: x_m = lateral (right+), y_m = depth (forward+).
    Robot base_link: x forward, y left.
    """
    # Rotate by camera yaw offset and translate by mount offset
    cos_yaw = math.cos(CAMERA_YAW_OFFSET_RAD)
    sin_yaw = math.sin(CAMERA_YAW_OFFSET_RAD)
    # camera lateral (x_cam) → robot: positive x_cam is robot right (−y_robot)
    robot_x = (
        CAMERA_FORWARD_OFFSET_M
        + cos_yaw * lm.y_m   # depth along camera Z = robot X
        - sin_yaw * lm.x_m
    )
    robot_y = (
        CAMERA_LATERAL_OFFSET_M
        - sin_yaw * lm.y_m
        - cos_yaw * lm.x_m   # camera right = robot -Y
    )
    dist = math.hypot(robot_x, robot_y)
    bearing = math.atan2(-robot_y, robot_x)   # bearing from robot heading
    return LandmarkPose(
        x_m=robot_x,
        y_m=robot_y,
        bearing_rad=bearing,
        distance_m=dist,
        confidence=lm.confidence,
        label=lm.label,
    )


def landmarks_to_robot_frame(landmarks: CourtLandmarks) -> CourtLandmarks:
    """Return a new CourtLandmarks with poses in base_link frame."""
    return CourtLandmarks(
        net=_camera_to_robot(landmarks.net) if landmarks.net else None,
        corners=[_camera_to_robot(c) for c in landmarks.corners],
        frame_id="base_link",
    )


# ---------------------------------------------------------------------------
# Robot frame → map frame
# ---------------------------------------------------------------------------

def _robot_to_map(
    lm: LandmarkPose,
    robot_x_m: float,
    robot_y_m: float,
    robot_yaw_rad: float,
) -> LandmarkPose:
    cos_yaw = math.cos(robot_yaw_rad)
    sin_yaw = math.sin(robot_yaw_rad)
    map_x = robot_x_m + cos_yaw * lm.x_m - sin_yaw * lm.y_m
    map_y = robot_y_m + sin_yaw * lm.x_m + cos_yaw * lm.y_m
    dist = math.hypot(map_x - robot_x_m, map_y - robot_y_m)
    bearing = math.atan2(map_y - robot_y_m, map_x - robot_x_m)
    return LandmarkPose(
        x_m=map_x,
        y_m=map_y,
        bearing_rad=bearing,
        distance_m=dist,
        confidence=lm.confidence,
        label=lm.label,
    )


def robot_landmarks_to_map(
    landmarks: CourtLandmarks,
    robot_x_m: float,
    robot_y_m: float,
    robot_yaw_rad: float,
) -> CourtLandmarks:
    """Return a new CourtLandmarks with poses in map frame.

    Input landmarks must already be in base_link frame (call
    landmarks_to_robot_frame() first).
    """
    return CourtLandmarks(
        net=(
            _robot_to_map(landmarks.net, robot_x_m, robot_y_m, robot_yaw_rad)
            if landmarks.net else None
        ),
        corners=[
            _robot_to_map(c, robot_x_m, robot_y_m, robot_yaw_rad)
            for c in landmarks.corners
        ],
        frame_id="map",
    )
