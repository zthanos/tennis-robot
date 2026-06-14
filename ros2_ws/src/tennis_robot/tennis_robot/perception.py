"""Tennis ball perception helpers shared by controllers and smoke tests."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import cv2
import numpy as np

from tennis_robot.survey import SurveyVision


TENNIS_BALL_DIAMETER_M = 0.067
MIN_BALL_AREA_PX = 6
# Upper bound keeps all balls down to 0.34 m (~75 k px) while rejecting bushes/vegetation
# which subtend hundreds of thousands of pixels even at 8 m.
MAX_BALL_AREA_PX = 90_000
MIN_BALL_ASPECT_RATIO = 0.55
MAX_BALL_ASPECT_RATIO = 1.8

# Tennis balls in Webots render as bright chartreuse (H≈35-60 in OpenCV 0-179).
# Upper hue capped at 72 to exclude pure-green Webots vegetation (H≈65-90).
# Second range covers shadowed/partially occluded balls; min saturation raised to
# 55 to avoid matching dark-green foliage with low chroma.
HSV_RANGES = (
    (np.array([25, 80, 80], dtype=np.uint8), np.array([72, 255, 255], dtype=np.uint8)),
    (np.array([25, 55, 50], dtype=np.uint8), np.array([72, 180, 175], dtype=np.uint8)),
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


@dataclass(frozen=True)
class CourtLineDetection:
    offset_m: float
    heading_error_rad: float
    confidence: float
    corner_detected: bool = False
    corner_confidence: float = 0.0


@dataclass(frozen=True)
class ObstacleDetection:
    label: str  # "net" | "fence"
    confidence: float


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
        area = cv2.contourArea(contour)
        if area <= MIN_BALL_AREA_PX or area > MAX_BALL_AREA_PX:
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

    # Use the 20th percentile instead of median: for small ball detections the ROI
    # contains background pixels at large depth; the lower percentile picks the
    # near surface (ball face) rather than the background-contaminated median.
    distance_m = float(np.percentile(valid, 20) if valid.size >= 5 else np.min(valid))
    normalized_x = (detection.center_x - frame_width_px / 2) / (frame_width_px / 2)
    bearing_rad = math.atan(normalized_x * math.tan(camera_fov_rad / 2))
    return BallObservation(
        detection=detection,
        bearing_rad=bearing_rad,
        distance_m=distance_m,
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


def detect_court_line(frame: np.ndarray) -> CourtLineDetection | None:
    """Detect the nearest horizontal court line and any corner intersection.

    Uses white HSV mask + Hough lines in the lower half of the frame.
    Returns None when no sufficiently long horizontal line is found.
    """
    h, w = frame.shape[:2]
    if h < 40 or w < 40:
        return None

    roi_y0 = int(h * 0.45)
    roi = frame[roi_y0:h, :]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    white = cv2.inRange(hsv, np.array([0, 0, 145], dtype=np.uint8), np.array([179, 90, 255], dtype=np.uint8))
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    white = cv2.morphologyEx(white, cv2.MORPH_OPEN, kernel)
    edges = cv2.Canny(white, 40, 120)

    min_len = max(24, w // 8)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180.0, threshold=35, minLineLength=min_len, maxLineGap=18)
    if lines is None:
        return None

    candidates: list[tuple[float, float, float, float]] = []
    for raw in lines[:, 0, :]:
        x1, y1, x2, y2 = (float(v) for v in raw)
        length = math.hypot(x2 - x1, y2 - y1)
        if length < min_len:
            continue
        angle = math.atan2(y2 - y1, x2 - x1)
        horizontal_error = min(abs(angle), abs(abs(angle) - math.pi))
        if horizontal_error > math.radians(35.0):
            continue
        mid_x = (x1 + x2) * 0.5
        candidates.append((length, angle, mid_x, horizontal_error))

    if not candidates:
        return None

    length, angle, mid_x, horizontal_error = max(candidates, key=lambda c: c[0])
    px_error = (mid_x - w * 0.5) / max(1.0, w * 0.5)
    offset_m = 0.50 + px_error * 0.45
    confidence = max(0.0, min(1.0, (length / max(1.0, w)) * (1.0 - horizontal_error / math.radians(35.0))))

    vertical_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 12))
    horizontal_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (18, 1))
    v_edges = cv2.morphologyEx(edges, cv2.MORPH_OPEN, vertical_kernel)
    h_edges = cv2.morphologyEx(edges, cv2.MORPH_OPEN, horizontal_kernel)
    corner_mask = cv2.bitwise_and(
        cv2.dilate(v_edges, np.ones((5, 5), dtype=np.uint8)),
        cv2.dilate(h_edges, np.ones((5, 5), dtype=np.uint8)),
    )
    corner_score = float(np.count_nonzero(corner_mask)) / float(corner_mask.size)

    return CourtLineDetection(
        offset_m=float(offset_m),
        heading_error_rad=float(-angle),
        confidence=float(confidence),
        corner_detected=corner_score > 0.0025,
        corner_confidence=min(1.0, corner_score * 120.0),
    )


def detect_obstacle_class(frame: np.ndarray) -> ObstacleDetection | None:
    """Classify a close-range obstacle as 'net' or 'fence'.

    Looks for a repeating grid pattern (horizontal + vertical edges) in
    the upper portion of the frame. Returns None when no grid is found.
    """
    h, w = frame.shape[:2]
    if h < 20 or w < 20:
        return None

    roi_y0 = int(h * 0.06)
    roi_y1 = int(h * 0.74)
    roi = frame[roi_y0:roi_y1, int(w * 0.05):int(w * 0.95)]
    if roi.size == 0:
        return None

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    if float(np.mean(gray < 85)) < 0.035:
        return None

    edges = cv2.Canny(gray, 45, 130)
    h_pattern = cv2.morphologyEx(edges, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_RECT, (18, 1)))
    v_pattern = cv2.morphologyEx(edges, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_RECT, (1, 10)))
    h_score = float(np.count_nonzero(h_pattern)) / float(h_pattern.size)
    v_score = float(np.count_nonzero(v_pattern)) / float(v_pattern.size)
    if h_score < 0.012 or v_score < 0.006:
        return None

    combined = cv2.bitwise_or(h_pattern, v_pattern)
    ys, _ = np.nonzero(combined)
    if ys.size == 0:
        return None

    top_frac = float(roi_y0 + int(np.min(ys))) / float(h)
    bottom_frac = float(roi_y0 + int(np.max(ys))) / float(h)
    height_frac = bottom_frac - top_frac

    # Check HORIZONTAL edge density in the lower third of the ROI.
    # Net: lower third has no horizontal wires — court surface below net is clear.
    # Fence: wire mesh extends throughout — horizontal wires visible even at bottom.
    # Use only h_pattern (not combined) to avoid false positives from vertical
    # court lines (service line stripe) which would appear in v_pattern only.
    roi_h = combined.shape[0]
    lower_third_h = h_pattern[2 * roi_h // 3:]
    lower_density = float(np.count_nonzero(lower_third_h)) / max(1, lower_third_h.size)

    large_extent = (top_frac < 0.08 and height_frac > 0.42) or height_frac > 0.58
    if large_extent:
        # At close range the net mesh fills the lower ROI too, so lower_density
        # is higher than when viewed from far away.  Use a more generous threshold
        # (0.020) so we don't flip to "fence" as the robot approaches the net.
        # A real fence has much denser horizontal wire in the lower third (>0.025).
        if lower_density < 0.020:
            return ObstacleDetection("net", min(1.0, (h_score + v_score) * 20))
        return ObstacleDetection("fence", min(1.0, h_score * 30))
    if 0.08 <= height_frac <= 0.52 and top_frac >= 0.08 and bottom_frac <= 0.82:
        return ObstacleDetection("net", min(1.0, (h_score + v_score) * 20))
    return None


@dataclass(frozen=True)
class CourtCornerDetection:
    """A detected court corner (L-intersection of baseline + sideline).

    bearing_rad: horizontal angle from camera centre — positive = right.
    distance_m:  depth estimate at the corner pixel; None if depth unavailable.
    pixel_x:     corner pixel column in the *original* (not ROI-cropped) frame.
    pixel_y:     corner pixel row in the original frame.
    confidence:  0–1 score derived from line lengths and intersection geometry.
    """

    pixel_x: int
    pixel_y: int
    bearing_rad: float
    distance_m: float | None
    confidence: float


@dataclass(frozen=True)
class NetPose:
    """Bearing and distance to the net as seen from the camera."""

    bearing_rad: float
    distance_m: float | None
    confidence: float


# ---------------------------------------------------------------------------
# Internal geometry helpers
# ---------------------------------------------------------------------------

def _hough_segments(
    frame: np.ndarray,
    min_len_fraction: float = 0.06,
) -> list[tuple[float, float, float, float]]:
    """Return Hough line segments from a BGR frame (white-line mask + Canny)."""
    h, w = frame.shape[:2]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    white = cv2.inRange(
        hsv,
        np.array([0, 0, 140], dtype=np.uint8),
        np.array([179, 90, 255], dtype=np.uint8),
    )
    white = cv2.morphologyEx(white, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    edges = cv2.Canny(white, 40, 120)
    min_len = max(20, int(w * min_len_fraction))
    lines = cv2.HoughLinesP(
        edges, 1, np.pi / 180.0, threshold=30,
        minLineLength=min_len, maxLineGap=14,
    )
    if lines is None:
        return []
    return [(float(x1), float(y1), float(x2), float(y2)) for x1, y1, x2, y2 in lines[:, 0, :]]


def _segment_angle(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.atan2(y2 - y1, x2 - x1)


def _line_intersection(
    x1: float, y1: float, x2: float, y2: float,
    x3: float, y3: float, x4: float, y4: float,
) -> tuple[float, float] | None:
    """Return intersection point of two infinite lines, or None if parallel."""
    dx1, dy1 = x2 - x1, y2 - y1
    dx2, dy2 = x4 - x3, y4 - y3
    denom = dx1 * dy2 - dy1 * dx2
    if abs(denom) < 1e-6:
        return None
    t = ((x3 - x1) * dy2 - (y3 - y1) * dx2) / denom
    return x1 + t * dx1, y1 + t * dy1


def _is_l_intersection(
    ix: float, iy: float,
    segs_h: list[tuple[float, float, float, float]],
    segs_v: list[tuple[float, float, float, float]],
    frame_w: int,
    frame_h: int,
    extend_px: float = 12.0,
) -> bool:
    """Return True only if intersection (ix,iy) is an L (exactly 2 quadrants
    have line coverage), not a T (3) or + (4).

    Strategy: for each of the 4 quadrants around the intersection, check
    whether any segment endpoint falls within `extend_px` in that quadrant.
    L-corners have exactly 2 *adjacent* occupied quadrants.
    """
    if not (0 <= ix < frame_w and 0 <= iy < frame_h):
        return False

    # Quadrant occupancy: 0=top-right, 1=bottom-right, 2=bottom-left, 3=top-left
    occupied = [False, False, False, False]

    def _check(px: float, py: float) -> None:
        dx, dy = px - ix, py - iy
        if abs(dx) < 2 and abs(dy) < 2:
            return
        if dx >= 0 and dy <= 0:
            occupied[0] = True
        elif dx >= 0 and dy > 0:
            occupied[1] = True
        elif dx < 0 and dy > 0:
            occupied[2] = True
        else:
            occupied[3] = True

    for x1, y1, x2, y2 in segs_h + segs_v:
        _check(x1, y1)
        _check(x2, y2)
        # Also sample midpoint for long segments
        _check((x1 + x2) * 0.5, (y1 + y2) * 0.5)

    n_occupied = sum(occupied)
    if n_occupied != 2:
        return False
    # Ensure the two occupied quadrants are adjacent (not diagonal)
    for i in range(4):
        if occupied[i] and occupied[(i + 1) % 4]:
            return True
    return False


def _depth_at_pixel(
    depth_frame: np.ndarray,
    px: int,
    py: int,
    frame_w: int,
    frame_h: int,
    depth_min_m: float = 0.1,
    depth_max_m: float = 12.0,
    roi_px: int = 8,
) -> float | None:
    """Sample the OAK-D depth frame around (px, py) and return median in metres."""
    dh, dw = depth_frame.shape[:2]
    # Scale pixel coords from RGB frame to depth frame dimensions
    sx = dw / max(1, frame_w)
    sy = dh / max(1, frame_h)
    cx = int(round(px * sx))
    cy = int(round(py * sy))
    x0, x1 = max(0, cx - roi_px), min(dw, cx + roi_px + 1)
    y0, y1 = max(0, cy - roi_px), min(dh, cy + roi_px + 1)
    roi = depth_frame[y0:y1, x0:x1]
    valid = roi[np.isfinite(roi) & (roi >= depth_min_m) & (roi <= depth_max_m)]
    if valid.size == 0:
        return None
    return float(np.median(valid))


# ---------------------------------------------------------------------------
# Public detection functions
# ---------------------------------------------------------------------------

def detect_court_corner(
    frame: np.ndarray,
    depth_frame: np.ndarray | None = None,
    camera_fov_rad: float = math.radians(69.0),
    depth_min_m: float = 0.1,
    depth_max_m: float = 12.0,
) -> CourtCornerDetection | None:
    """Detect the most prominent L-intersection of court lines in *frame*.

    Uses white-line Hough segments; classifies intersections as L (corner),
    T (service-line junction), or + (centre) by quadrant occupancy, and
    returns only L-type intersections.

    bearing_rad: positive = right of camera centre.
    distance_m:  OAK-D depth at the intersection pixel; None if unavailable.
    """
    h, w = frame.shape[:2]
    if h < 60 or w < 60:
        return None

    segs = _hough_segments(frame)
    if len(segs) < 2:
        return None

    # Separate horizontal and vertical segments (±35° tolerance)
    segs_h: list[tuple[float, float, float, float]] = []
    segs_v: list[tuple[float, float, float, float]] = []
    for seg in segs:
        angle = abs(_segment_angle(*seg))
        h_err = min(angle, abs(angle - math.pi))  # closeness to 0° / 180°
        v_err = abs(angle - math.pi / 2)           # closeness to 90°
        if h_err < math.radians(35):
            segs_h.append(seg)
        elif v_err < math.radians(35):
            segs_v.append(seg)

    if not segs_h or not segs_v:
        return None

    best: CourtCornerDetection | None = None
    best_score = 0.0

    for sh in segs_h:
        lh = math.hypot(sh[2] - sh[0], sh[3] - sh[1])
        for sv in segs_v:
            lv = math.hypot(sv[2] - sv[0], sv[3] - sv[1])
            pt = _line_intersection(*sh, *sv)
            if pt is None:
                continue
            ix, iy = pt
            if not (0 <= ix < w and 0 <= iy < h):
                continue
            if not _is_l_intersection(ix, iy, segs_h, segs_v, w, h):
                continue

            # Confidence: normalised product of line lengths
            score = min(1.0, (lh / w) * (lv / h) * 20.0)
            if score <= best_score:
                continue

            px_int, py_int = int(round(ix)), int(round(iy))
            dist = (
                _depth_at_pixel(depth_frame, px_int, py_int, w, h, depth_min_m, depth_max_m)
                if depth_frame is not None and depth_frame.size > 0
                else None
            )
            normalised_x = (ix - w * 0.5) / (w * 0.5)
            bearing = math.atan(normalised_x * math.tan(camera_fov_rad / 2))

            best = CourtCornerDetection(
                pixel_x=px_int,
                pixel_y=py_int,
                bearing_rad=float(bearing),
                distance_m=dist,
                confidence=float(score),
            )
            best_score = score

    return best


def detect_net_pose(
    frame: np.ndarray,
    depth_frame: np.ndarray | None = None,
    camera_fov_rad: float = math.radians(69.0),
    depth_min_m: float = 0.1,
    depth_max_m: float = 10.0,
) -> NetPose | None:
    """Return bearing + depth to the net when it fills the frame.

    Uses detect_obstacle_class() for label; estimates bearing from the
    horizontal centre-of-mass of the detected grid pattern, and distance
    from the centre depth sector of the OAK-D depth frame.
    """
    obstacle = detect_obstacle_class(frame)
    if obstacle is None or obstacle.label != "net":
        return None

    h, w = frame.shape[:2]

    # Bearing: horizontal CoM of the net grid edges
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 45, 130)
    h_pattern = cv2.morphologyEx(
        edges, cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_RECT, (18, 1)),
    )
    v_pattern = cv2.morphologyEx(
        edges, cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_RECT, (1, 10)),
    )
    combined = cv2.bitwise_or(h_pattern, v_pattern)
    ys, xs = np.nonzero(combined)
    if xs.size > 0:
        com_x = float(np.mean(xs))
        normalised_x = (com_x - w * 0.5) / (w * 0.5)
    else:
        normalised_x = 0.0
    bearing = math.atan(normalised_x * math.tan(camera_fov_rad / 2))

    # Distance: centre column of depth frame, lower-mid rows
    distance: float | None = None
    if depth_frame is not None and depth_frame.size > 0:
        dh, dw = depth_frame.shape[:2]
        x0, x1 = dw // 3, (2 * dw) // 3
        y0, y1 = int(dh * 0.3), int(dh * 0.75)
        roi = depth_frame[y0:y1, x0:x1]
        valid = roi[np.isfinite(roi) & (roi >= depth_min_m) & (roi <= depth_max_m)]
        if valid.size >= 4:
            distance = float(np.percentile(valid, 20))

    return NetPose(
        bearing_rad=float(bearing),
        distance_m=distance,
        confidence=obstacle.confidence,
    )


def build_survey_vision(
    frame: np.ndarray | None,
    depth_frame: np.ndarray | None = None,
    depth_min_range: float = 0.1,
    depth_max_range: float = 10.0,
) -> SurveyVision:
    """Build a SurveyVision from a camera frame and optional aligned depth frame.

    Always returns a SurveyVision — never None. Callers check line_detected.
    depth_min_range / depth_max_range come from the depth camera's reported range limits.
    """
    line = detect_court_line(frame) if frame is not None else None
    obstacle = detect_obstacle_class(frame) if frame is not None else None
    obstacle_class = obstacle.label if obstacle is not None else None

    line_kwargs: dict = {
        "line_detected": False,
        "line_offset_m": None,
        "line_heading_error_rad": None,
        "line_confidence": 0.0,
        "corner_detected": False,
        "corner_confidence": 0.0,
    }
    if line is not None:
        line_kwargs = {
            "line_detected": True,
            "line_offset_m": line.offset_m,
            "line_heading_error_rad": line.heading_error_rad,
            "line_confidence": line.confidence,
            "corner_detected": line.corner_detected,
            "corner_confidence": line.corner_confidence,
        }

    if depth_frame is None or depth_frame.size == 0:
        return SurveyVision(obstacle_class=obstacle_class, **line_kwargs)

    h, w = depth_frame.shape[:2]
    if h < 4 or w < 6:
        return SurveyVision(obstacle_class=obstacle_class, **line_kwargs)

    y0, y1 = int(h * 0.56), int(h * 0.92)

    def _sector(x0: int, x1: int) -> tuple[float | None, int]:
        roi = depth_frame[y0:y1, max(0, x0):min(w, x1)]
        valid = roi[np.isfinite(roi) & (roi >= depth_min_range) & (roi <= depth_max_range)]
        if valid.size == 0:
            return None, 0
        return float(np.percentile(valid, 20)), int(valid.size)

    left, ln = _sector(0, w // 3)
    center, cn = _sector(w // 3, (2 * w) // 3)
    right, rn = _sector((2 * w) // 3, w)

    return SurveyVision(
        center_m=center,
        left_m=left,
        right_m=right,
        valid_count=ln + cn + rn,
        obstacle_class=obstacle_class,
        **line_kwargs,
    )
