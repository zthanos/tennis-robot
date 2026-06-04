"""ROS 2-only LiDAR-led Map Court behavior.

This replaces the camera-line survey in the ROS 2 controller. LiDAR builds the
court boundary estimate and locates the net from its support posts. Camera
survey vision is used only to confirm the net after the robot moves closer.
"""

from __future__ import annotations

import json
import math
import os
import time
from dataclasses import dataclass
from enum import Enum
from pathlib import Path

from collector import BaseCommand
from config_utils import _env_float
from survey import SurveyVision


PROJECT_ROOT = Path(os.getenv("TENNIS_ROBOT_ROOT", "/workspace"))
DEFAULT_BOUNDARY_FILE = PROJECT_ROOT / "runtime" / "court_boundary.json"


class LidarSurveyState(str, Enum):
    INITIAL_SCAN = "initial_lidar_scan"
    APPROACH_NET = "approach_net_for_visual_confirmation"
    CONFIRM_NET_VISUAL = "confirm_net_visual"
    CROSS_TO_FAR_SIDE = "cross_to_far_side"
    FAR_SIDE_SCAN = "far_side_lidar_scan"
    VALIDATE_SURVEY = "validate_lidar_survey"
    DONE = "done"


@dataclass(frozen=True)
class LidarSurveyConfig:
    drive_speed_m_s: float = 0.35
    turn_speed_rad_s: float = 0.8
    safety_stop_range_m: float = 0.35
    safety_slow_range_m: float = 0.75
    lidar_min_range_m: float = 0.35
    lidar_max_range_m: float = 12.0
    initial_scan_duration_s: float = 2.0
    initial_scan_timeout_s: float = 18.0
    initial_scan_min_points: float = 80.0
    far_scan_duration_s: float = 2.0
    far_scan_timeout_s: float = 18.0
    far_scan_min_points: float = 80.0
    net_confirm_duration_s: float = 1.5
    net_confirm_standoff_m: float = 1.6
    net_post_min_separation_m: float = 8.5
    net_post_max_separation_m: float = 13.0
    net_post_max_cluster_diameter_m: float = 0.45
    cross_post_clearance_m: float = 1.0
    far_side_reveal_m: float = 3.2
    target_tolerance_m: float = 0.25
    heading_tolerance_rad: float = math.radians(8.0)
    expected_court_length_m: float = 23.77
    expected_court_width_m: float = 10.97
    output_file: Path = DEFAULT_BOUNDARY_FILE

    @classmethod
    def from_env(cls) -> "LidarSurveyConfig":
        d = cls()
        return cls(
            drive_speed_m_s=_env_float("ROS2_SURVEY_DRIVE_SPEED_M_S", d.drive_speed_m_s),
            turn_speed_rad_s=_env_float("ROS2_SURVEY_TURN_SPEED_RAD_S", d.turn_speed_rad_s),
            safety_stop_range_m=_env_float("ROS2_SURVEY_SAFETY_STOP_RANGE_M", d.safety_stop_range_m),
            safety_slow_range_m=_env_float("ROS2_SURVEY_SAFETY_SLOW_RANGE_M", d.safety_slow_range_m),
            lidar_min_range_m=_env_float("ROS2_SURVEY_LIDAR_MIN_RANGE_M", d.lidar_min_range_m),
            lidar_max_range_m=_env_float("ROS2_SURVEY_LIDAR_MAX_RANGE_M", d.lidar_max_range_m),
            initial_scan_duration_s=_env_float("ROS2_SURVEY_INITIAL_SCAN_DURATION_S", d.initial_scan_duration_s),
            initial_scan_timeout_s=_env_float("ROS2_SURVEY_INITIAL_SCAN_TIMEOUT_S", d.initial_scan_timeout_s),
            initial_scan_min_points=_env_float("ROS2_SURVEY_INITIAL_SCAN_MIN_POINTS", d.initial_scan_min_points),
            far_scan_duration_s=_env_float("ROS2_SURVEY_FAR_SCAN_DURATION_S", d.far_scan_duration_s),
            far_scan_timeout_s=_env_float("ROS2_SURVEY_FAR_SCAN_TIMEOUT_S", d.far_scan_timeout_s),
            far_scan_min_points=_env_float("ROS2_SURVEY_FAR_SCAN_MIN_POINTS", d.far_scan_min_points),
            net_confirm_duration_s=_env_float("ROS2_SURVEY_NET_CONFIRM_DURATION_S", d.net_confirm_duration_s),
            net_confirm_standoff_m=_env_float("ROS2_SURVEY_NET_CONFIRM_STANDOFF_M", d.net_confirm_standoff_m),
            net_post_min_separation_m=_env_float("ROS2_SURVEY_NET_POST_MIN_SEPARATION_M", d.net_post_min_separation_m),
            net_post_max_separation_m=_env_float("ROS2_SURVEY_NET_POST_MAX_SEPARATION_M", d.net_post_max_separation_m),
            net_post_max_cluster_diameter_m=_env_float(
                "ROS2_SURVEY_NET_POST_MAX_CLUSTER_DIAMETER_M",
                d.net_post_max_cluster_diameter_m,
            ),
            cross_post_clearance_m=_env_float("ROS2_SURVEY_CROSS_POST_CLEARANCE_M", d.cross_post_clearance_m),
            far_side_reveal_m=_env_float("ROS2_SURVEY_FAR_SIDE_REVEAL_M", d.far_side_reveal_m),
            target_tolerance_m=_env_float("ROS2_SURVEY_TARGET_TOLERANCE_M", d.target_tolerance_m),
            expected_court_length_m=_env_float("SURVEY_EXPECTED_COURT_LENGTH_M", d.expected_court_length_m),
            expected_court_width_m=_env_float("SURVEY_EXPECTED_COURT_WIDTH_M", d.expected_court_width_m),
            output_file=Path(os.getenv("SURVEY_OUTPUT_FILE", str(d.output_file))),
        )


@dataclass(frozen=True)
class LidarSurveyCommand:
    state: LidarSurveyState
    base: BaseCommand
    sample_count: int


@dataclass(frozen=True)
class NetFrame:
    post_a: tuple[float, float]
    post_b: tuple[float, float]
    midpoint: tuple[float, float]
    tangent: tuple[float, float]
    near_normal: tuple[float, float]
    far_normal: tuple[float, float]
    confidence: float


class Ros2LidarCourtSurvey:
    def __init__(self, config: LidarSurveyConfig | None = None) -> None:
        self.config = config or LidarSurveyConfig()
        self.state = LidarSurveyState.INITIAL_SCAN
        self.sample_count = 0
        self._state_elapsed_s = 0.0
        self._started_at: float | None = None
        self._last_event = "none"
        self._failure_reason: str | None = None
        self._court_bounds: dict | None = None
        self._initial_points: list[tuple[float, float]] = []
        self._initial_local_points: list[tuple[float, float]] = []
        self._far_points: list[tuple[float, float]] = []
        self._far_local_points: list[tuple[float, float]] = []
        self._net_frame: NetFrame | None = None
        self._approach_target: tuple[float, float] | None = None
        self._cross_target: tuple[float, float] | None = None
        self._net_visual_confirmed = False
        self._last_front_range_m = math.inf
        self._last_pose: tuple[float, float] | None = None
        self._distance_traveled_m = 0.0

    @classmethod
    def from_env(cls) -> "Ros2LidarCourtSurvey":
        return cls(LidarSurveyConfig.from_env())

    def reset(self) -> None:
        self.__init__(self.config)

    @property
    def court_bounds(self) -> dict | None:
        return self._court_bounds

    def update(
        self,
        x_m: float,
        y_m: float,
        yaw_rad: float,
        lidar_ranges: list[float] | None,
        dt_s: float,
        vision: SurveyVision | None,
    ) -> LidarSurveyCommand:
        if self._started_at is None:
            self._started_at = time.time()
            self._enter(LidarSurveyState.INITIAL_SCAN, "ros2_lidar_map_court_started")
        if self.state == LidarSurveyState.DONE:
            return self._cmd(BaseCommand(0.0, 0.0))

        self._state_elapsed_s += max(0.0, dt_s)
        self._update_distance(x_m, y_m)
        self._last_front_range_m = self._front_range(lidar_ranges)
        self._accumulate_scan(x_m, y_m, yaw_rad, lidar_ranges)

        command = self._step(x_m, y_m, yaw_rad, vision)
        return self._cmd(self._apply_safety(command))

    def telemetry(self) -> dict:
        elapsed = 0.0 if self._started_at is None else time.time() - self._started_at
        points = self._initial_points + self._far_points
        local_points = self._initial_local_points + self._far_local_points
        target = self._active_target()
        target_distance = None if target is None or self._last_pose is None else self._distance(self._last_pose, target)
        return {
            "state": self.state.value,
            "navigation_source": "ros2_lidar_boundary_survey",
            "path_driver": "lidar_net_posts_and_boundary_reveal",
            "camera_role": "net_visual_confirmation_only",
            "fallback_enabled": False,
            "last_event": self._last_event,
            "failure_reason": self._failure_reason,
            "sample_count": self.sample_count,
            "map_points": self._point_sample(points, 1200),
            "map_point_count": len(points),
            "lidar_points": self._point_sample(local_points, 600),
            "lidar_point_count": len(local_points),
            "display_frame": "world",
            "sensor_frame": "lidar_local",
            "initial_point_count": len(self._initial_points),
            "far_point_count": len(self._far_points),
            "front_lidar_range_m": None if math.isinf(self._last_front_range_m) else round(self._last_front_range_m, 3),
            "net_visual_confirmed": self._net_visual_confirmed,
            "net_frame": self._net_frame_dict(),
            "net_boundary": self._net_boundary_dict(),
            "net_boundary_source": "lidar_net_posts",
            "approach_target": self._point_dict(self._approach_target),
            "cross_target": self._point_dict(self._cross_target),
            "active_target": self._point_dict(target),
            "distance_to_target_m": None if target_distance is None else round(target_distance, 3),
            "distance_traveled_m": round(self._distance_traveled_m, 2),
            "scan_coverage": self._scan_coverage(points),
            "elapsed_s": round(elapsed, 1),
            "state_elapsed_s": round(self._state_elapsed_s, 1),
        }

    def _step(self, x_m: float, y_m: float, yaw_rad: float, vision: SurveyVision | None) -> BaseCommand:
        if self.state == LidarSurveyState.INITIAL_SCAN:
            enough_points = len(self._initial_points) >= int(self.config.initial_scan_min_points)
            if self._state_elapsed_s < self.config.initial_scan_duration_s or not enough_points:
                if self._state_elapsed_s >= self.config.initial_scan_timeout_s:
                    self._fail("LIDAR_SCAN_UNAVAILABLE")
                elif not enough_points:
                    self._last_event = "waiting_for_initial_lidar_points"
                return BaseCommand(0.0, 0.0)
            self._net_frame = self._detect_net_frame(self._initial_points, (x_m, y_m))
            if self._net_frame is None:
                self._fail("NET_POSTS_NOT_FOUND")
                return BaseCommand(0.0, 0.0)
            self._approach_target = self._target_before_net()
            self._enter(LidarSurveyState.APPROACH_NET, "net_posts_localized_from_lidar")
            return BaseCommand(0.0, 0.0)

        if self.state == LidarSurveyState.APPROACH_NET:
            if self._approach_target is None or self._distance((x_m, y_m), self._approach_target) <= self.config.target_tolerance_m:
                self._enter(LidarSurveyState.CONFIRM_NET_VISUAL, "net_confirmation_pose_reached")
                return BaseCommand(0.0, 0.0)
            return self._drive_to_target(x_m, y_m, yaw_rad, self._approach_target)

        if self.state == LidarSurveyState.CONFIRM_NET_VISUAL:
            if self._vision_confirms_net(vision):
                self._net_visual_confirmed = True
                self._cross_target = self._target_across_net((x_m, y_m))
                self._enter(LidarSurveyState.CROSS_TO_FAR_SIDE, "net_visually_confirmed")
                return BaseCommand(0.0, 0.0)
            if self._state_elapsed_s >= self.config.net_confirm_duration_s:
                self._fail("NET_VISUAL_CONFIRMATION_FAILED")
            return BaseCommand(0.0, 0.0)

        if self.state == LidarSurveyState.CROSS_TO_FAR_SIDE:
            if self._cross_target is None or self._distance((x_m, y_m), self._cross_target) <= self.config.target_tolerance_m:
                self._enter(LidarSurveyState.FAR_SIDE_SCAN, "far_side_reveal_pose_reached")
                return BaseCommand(0.0, 0.0)
            return self._drive_to_target(x_m, y_m, yaw_rad, self._cross_target)

        if self.state == LidarSurveyState.FAR_SIDE_SCAN:
            enough_points = len(self._far_points) >= int(self.config.far_scan_min_points)
            if self._state_elapsed_s >= self.config.far_scan_duration_s and enough_points:
                self._enter(LidarSurveyState.VALIDATE_SURVEY, "far_side_lidar_boundaries_captured")
            elif self._state_elapsed_s >= self.config.far_scan_timeout_s:
                self._fail("FAR_SIDE_LIDAR_SCAN_UNAVAILABLE")
            elif not enough_points:
                self._last_event = "waiting_for_far_side_lidar_points"
            return BaseCommand(0.0, 0.0)

        if self.state == LidarSurveyState.VALIDATE_SURVEY:
            self._finalize()
            return BaseCommand(0.0, 0.0)

        return BaseCommand(0.0, 0.0)

    def _accumulate_scan(
        self,
        x_m: float,
        y_m: float,
        yaw_rad: float,
        ranges: list[float] | None,
    ) -> None:
        if self.state not in {LidarSurveyState.INITIAL_SCAN, LidarSurveyState.FAR_SIDE_SCAN}:
            return
        points = self._scan_points_world(x_m, y_m, yaw_rad, ranges)
        local_points = self._scan_points_local(ranges)
        if not points:
            return
        target = self._initial_points if self.state == LidarSurveyState.INITIAL_SCAN else self._far_points
        local_target = self._initial_local_points if self.state == LidarSurveyState.INITIAL_SCAN else self._far_local_points
        target.extend(points)
        local_target.extend(local_points)
        if len(target) > 6000:
            del target[: len(target) - 6000]
        if len(local_target) > 6000:
            del local_target[: len(local_target) - 6000]
        self.sample_count += 1

    def _scan_points_local(self, ranges: list[float] | None) -> list[tuple[float, float]]:
        if not ranges or len(ranges) < 10:
            return []
        n = len(ranges)
        step = max(1, n // 240)
        points: list[tuple[float, float]] = []
        for i in range(0, n, step):
            r = ranges[i]
            if not math.isfinite(r) or r < self.config.lidar_min_range_m or r > self.config.lidar_max_range_m:
                continue
            angle = (i / n) * 2.0 * math.pi - math.pi
            points.append((r * math.cos(angle), r * math.sin(angle)))
        return points

    def _scan_points_world(
        self,
        x_m: float,
        y_m: float,
        yaw_rad: float,
        ranges: list[float] | None,
    ) -> list[tuple[float, float]]:
        if not ranges or len(ranges) < 10:
            return []
        cos_y = math.cos(yaw_rad)
        sin_y = math.sin(yaw_rad)
        n = len(ranges)
        step = max(1, n // 240)
        points: list[tuple[float, float]] = []
        for i in range(0, n, step):
            r = ranges[i]
            if not math.isfinite(r) or r < self.config.lidar_min_range_m or r > self.config.lidar_max_range_m:
                continue
            angle = (i / n) * 2.0 * math.pi - math.pi
            lx = r * math.cos(angle)
            ly = r * math.sin(angle)
            points.append((x_m + cos_y * lx - sin_y * ly, y_m + sin_y * lx + cos_y * ly))
        return points

    def _detect_net_frame(self, points: list[tuple[float, float]], robot: tuple[float, float]) -> NetFrame | None:
        clusters = self._clusters(points)
        best: tuple[float, tuple[float, float], tuple[float, float]] | None = None
        for i, a in enumerate(clusters):
            for b in clusters[i + 1:]:
                sep = self._distance(a, b)
                if not (self.config.net_post_min_separation_m <= sep <= self.config.net_post_max_separation_m):
                    continue
                mid = ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5)
                robot_dist = self._distance(mid, robot)
                score = sep - 0.12 * robot_dist
                if best is None or score > best[0]:
                    best = (score, a, b)
        if best is None:
            return None
        a, b = best[1], best[2]
        mid = ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5)
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        length = max(0.001, math.hypot(dx, dy))
        tangent = (dx / length, dy / length)
        n1 = (-tangent[1], tangent[0])
        to_robot = (robot[0] - mid[0], robot[1] - mid[1])
        near = n1 if self._dot(n1, to_robot) >= 0.0 else (-n1[0], -n1[1])
        far = (-near[0], -near[1])
        confidence = min(0.95, 0.55 + abs(length - self.config.expected_court_width_m) * -0.03 + 0.35)
        return NetFrame(a, b, mid, tangent, near, far, round(max(0.5, confidence), 2))

    def _clusters(self, points: list[tuple[float, float]]) -> list[tuple[float, float]]:
        buckets: dict[tuple[int, int], list[tuple[float, float]]] = {}
        for x, y in points:
            key = (round(x * 2.0), round(y * 2.0))
            buckets.setdefault(key, []).append((x, y))
        centers: list[tuple[float, float]] = []
        for pts in buckets.values():
            if len(pts) < 2:
                continue
            cx = sum(p[0] for p in pts) / len(pts)
            cy = sum(p[1] for p in pts) / len(pts)
            diameter = max(self._distance((cx, cy), p) for p in pts) * 2.0
            if diameter <= self.config.net_post_max_cluster_diameter_m:
                centers.append((cx, cy))
        return centers

    def _target_before_net(self) -> tuple[float, float]:
        assert self._net_frame is not None
        n = self._net_frame.near_normal
        mid = self._net_frame.midpoint
        return (mid[0] + n[0] * self.config.net_confirm_standoff_m, mid[1] + n[1] * self.config.net_confirm_standoff_m)

    def _target_across_net(self, robot: tuple[float, float]) -> tuple[float, float]:
        assert self._net_frame is not None
        a = self._net_frame.post_a
        b = self._net_frame.post_b
        post = a if self._distance(robot, a) <= self._distance(robot, b) else b
        tangent = self._net_frame.tangent
        sign = 1.0 if self._dot((robot[0] - post[0], robot[1] - post[1]), tangent) >= 0.0 else -1.0
        far = self._net_frame.far_normal
        return (
            post[0] + tangent[0] * sign * self.config.cross_post_clearance_m + far[0] * self.config.far_side_reveal_m,
            post[1] + tangent[1] * sign * self.config.cross_post_clearance_m + far[1] * self.config.far_side_reveal_m,
        )

    def _drive_to_target(self, x_m: float, y_m: float, yaw_rad: float, target: tuple[float, float]) -> BaseCommand:
        dx = target[0] - x_m
        dy = target[1] - y_m
        desired = math.atan2(dy, dx)
        err = self._angle_delta(desired, yaw_rad)
        turn = max(-self.config.turn_speed_rad_s, min(self.config.turn_speed_rad_s, err * 1.8))
        if abs(err) > self.config.heading_tolerance_rad:
            return BaseCommand(0.0, turn)
        return BaseCommand(self.config.drive_speed_m_s, turn)

    def _vision_confirms_net(self, vision: SurveyVision | None) -> bool:
        if vision is None:
            return False
        label = (vision.obstacle_class or "").strip().lower()
        if label in {"net", "net_post", "post", "posts"}:
            return True
        return vision.center_m is not None and 0.5 <= vision.center_m <= 3.0 and vision.valid_count >= 80

    def _finalize(self) -> None:
        now = time.time()
        elapsed = 0.0 if self._started_at is None else now - self._started_at
        world = self._world_extents(self._initial_points + self._far_points)
        status = "SUCCESS"
        reason = None
        if self._failure_reason:
            status = "FAILED"
            reason = self._failure_reason
        elif self._net_frame is None:
            status = "FAILED"
            reason = "NET_POSTS_NOT_FOUND"
        elif not self._net_visual_confirmed:
            status = "FAILED"
            reason = "NET_VISUAL_CONFIRMATION_FAILED"
        elif world["point_count"] < 80:
            status = "FAILED"
            reason = "LIDAR_BOUNDARY_POINTS_INSUFFICIENT"

        bounds = {
            "mapped_at": now,
            "surveyed_at": now,
            "status": status,
            "failure_reason": reason,
            "survey_complete": status == "SUCCESS",
            "court_geometry": {
                "length_m": self.config.expected_court_length_m,
                "width_m": self.config.expected_court_width_m,
                "method": "ros2_lidar_net_posts_and_boundary_reveal",
            },
            "net": self._net_frame_dict(),
            "lidar_boundary_estimate": {
                "initial_side": self._world_extents(self._initial_points),
                "far_side": self._world_extents(self._far_points),
                "combined": world,
            },
            "external_boundary_map": {
                "candidates": self._boundary_candidates(world),
                "point_count": world["point_count"],
            },
            "internal_objects": [
                {"label": "net", "classification": "internal_divider", **self._net_frame_dict()}
            ] if self._net_frame is not None else [],
            "diagnostics": {
                "confidence": self._confidence(status, world),
                "path_driver": "ros2_lidar_boundary_survey",
                "camera_role": "net_visual_confirmation_only",
                "fallback_enabled": False,
                "final_state": self.state.value,
                "last_event": self._last_event,
                "elapsed_s": round(elapsed, 1),
                "sample_count": self.sample_count,
            },
            "navigation": {
                "source": "ros2_lidar_boundary_survey",
                "legacy_camera_line_fallback": False,
                "final_state": self.state.value,
            },
            "point_cloud_sample": self._point_sample(self._initial_points + self._far_points),
            "sample_count": self.sample_count,
            "point_count": world["point_count"],
        }
        self._court_bounds = bounds
        self.config.output_file.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.config.output_file.with_suffix(".tmp.json")
        tmp.write_text(json.dumps(bounds, indent=2) + "\n", encoding="utf-8")
        tmp.replace(self.config.output_file)
        self._enter(LidarSurveyState.DONE, f"ros2_lidar_map_court_{status.lower()}")

    def _fail(self, reason: str) -> None:
        self._failure_reason = reason
        self._finalize()

    def _enter(self, state: LidarSurveyState, event: str) -> None:
        self.state = state
        self._last_event = event
        self._state_elapsed_s = 0.0

    def _cmd(self, base: BaseCommand) -> LidarSurveyCommand:
        return LidarSurveyCommand(self.state, base, self.sample_count)

    def _apply_safety(self, command: BaseCommand) -> BaseCommand:
        if command.linear_speed_m_s <= 0.0:
            return command
        if self._last_front_range_m <= self.config.safety_stop_range_m:
            return BaseCommand(0.0, 0.0)
        if self._last_front_range_m <= self.config.safety_slow_range_m:
            return BaseCommand(command.linear_speed_m_s * 0.35, command.angular_speed_rad_s)
        return command

    def _front_range(self, ranges: list[float] | None) -> float:
        if not ranges:
            return math.inf
        n = len(ranges)
        center = n // 2
        half = max(1, n // 16)
        values = [
            ranges[(center + i) % n]
            for i in range(-half, half + 1)
            if math.isfinite(ranges[(center + i) % n])
        ]
        return min(values) if values else math.inf

    def _update_distance(self, x_m: float, y_m: float) -> None:
        if self._last_pose is None:
            self._last_pose = (x_m, y_m)
            return
        step = self._distance(self._last_pose, (x_m, y_m))
        if 0.001 <= step <= 1.0:
            self._distance_traveled_m += step
        self._last_pose = (x_m, y_m)

    def _active_target(self) -> tuple[float, float] | None:
        if self.state == LidarSurveyState.APPROACH_NET:
            return self._approach_target
        if self.state == LidarSurveyState.CROSS_TO_FAR_SIDE:
            return self._cross_target
        return None

    def _world_extents(self, points: list[tuple[float, float]]) -> dict:
        if not points:
            return {"point_count": 0}
        xs = sorted(p[0] for p in points)
        ys = sorted(p[1] for p in points)
        min_x = self._quantile(xs, 0.02)
        max_x = self._quantile(xs, 0.98)
        min_y = self._quantile(ys, 0.02)
        max_y = self._quantile(ys, 0.98)
        return {
            "min_x_m": round(min_x, 3),
            "max_x_m": round(max_x, 3),
            "min_y_m": round(min_y, 3),
            "max_y_m": round(max_y, 3),
            "span_x_m": round(max_x - min_x, 3),
            "span_y_m": round(max_y - min_y, 3),
            "point_count": len(points),
        }

    def _scan_coverage(self, points: list[tuple[float, float]]) -> dict:
        world = self._world_extents(points)
        coverage = {
            "front_m": None,
            "rear_m": None,
            "left_m": None,
            "right_m": None,
            "world_extents": world if world.get("point_count", 0) > 0 else None,
        }
        if self._last_pose is None or not points:
            return coverage
        x, y = self._last_pose
        front = [px - x for px, _py in points if px >= x]
        rear = [x - px for px, _py in points if px < x]
        left = [py - y for _px, py in points if py >= y]
        right = [y - py for _px, py in points if py < y]
        coverage.update({
            "front_m": round(max(front), 3) if front else None,
            "rear_m": round(max(rear), 3) if rear else None,
            "left_m": round(max(left), 3) if left else None,
            "right_m": round(max(right), 3) if right else None,
        })
        return coverage

    def _boundary_candidates(self, extents: dict) -> list[dict]:
        if not extents or extents.get("point_count", 0) <= 0:
            return []
        return [
            {"label": "boundary_min_x", "classification": "external_boundary_candidate", "x_m": extents["min_x_m"], "y_m": 0.0, "confidence": 0.65},
            {"label": "boundary_max_x", "classification": "external_boundary_candidate", "x_m": extents["max_x_m"], "y_m": 0.0, "confidence": 0.65},
            {"label": "boundary_min_y", "classification": "external_boundary_candidate", "x_m": 0.0, "y_m": extents["min_y_m"], "confidence": 0.65},
            {"label": "boundary_max_y", "classification": "external_boundary_candidate", "x_m": 0.0, "y_m": extents["max_y_m"], "confidence": 0.65},
        ]

    def _confidence(self, status: str, world: dict) -> float:
        score = 0.2
        if self._net_frame is not None:
            score += 0.3 * self._net_frame.confidence
        if self._net_visual_confirmed:
            score += 0.2
        if len(self._initial_points) >= 80:
            score += 0.1
        if len(self._far_points) >= 80:
            score += 0.1
        if world.get("point_count", 0) >= 160:
            score += 0.1
        if status == "FAILED":
            score = min(score, 0.45)
        return round(min(0.95, score), 2)

    def _net_frame_dict(self) -> dict | None:
        if self._net_frame is None:
            return None
        return {
            "post_a": self._point_dict(self._net_frame.post_a),
            "post_b": self._point_dict(self._net_frame.post_b),
            "midpoint": self._point_dict(self._net_frame.midpoint),
            "confidence": self._net_frame.confidence,
        }

    def _net_boundary_dict(self) -> dict | None:
        if self._net_frame is None:
            return None
        a = self._net_frame.post_a
        b = self._net_frame.post_b
        center = self._net_frame.midpoint
        distance = None if self._last_pose is None else self._distance(self._last_pose, center)
        return {
            "post_a": self._point_dict(a),
            "post_b": self._point_dict(b),
            "center": self._point_dict(center),
            "lidar_local": self._net_local_dict(center),
            "length_m": round(self._distance(a, b), 3),
            "distance_m": None if distance is None else round(distance, 3),
            "front_clearance_m": None if math.isinf(self._last_front_range_m) else round(self._last_front_range_m, 3),
            "confidence": self._net_frame.confidence,
            "source": "lidar_net_posts",
        }

    def _net_local_dict(self, point: tuple[float, float]) -> dict | None:
        if self._last_pose is None:
            return None
        dx = point[0] - self._last_pose[0]
        dy = point[1] - self._last_pose[1]
        distance = math.hypot(dx, dy)
        return {
            "x_m": round(dx, 3),
            "y_m": round(dy, 3),
            "distance_m": round(distance, 3),
            "bearing_rad": round(math.atan2(dy, dx), 4),
        }

    @staticmethod
    def _point_dict(point: tuple[float, float] | None) -> dict | None:
        if point is None:
            return None
        return {"x_m": round(point[0], 3), "y_m": round(point[1], 3)}

    @staticmethod
    def _point_sample(points: list[tuple[float, float]], limit: int = 1000) -> list[dict]:
        if not points:
            return []
        buckets: dict[tuple[int, int], tuple[float, float]] = {}
        for x, y in points:
            buckets.setdefault((round(x * 5), round(y * 5)), (x, y))
        points = list(buckets.values())
        stride = max(1, len(points) // limit)
        return [{"x_m": round(x, 3), "y_m": round(y, 3)} for x, y in points[::stride]][-limit:]

    @staticmethod
    def _distance(a: tuple[float, float], b: tuple[float, float]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    @staticmethod
    def _dot(a: tuple[float, float], b: tuple[float, float]) -> float:
        return a[0] * b[0] + a[1] * b[1]

    @staticmethod
    def _angle_delta(a: float, b: float) -> float:
        return (a - b + math.pi) % (2.0 * math.pi) - math.pi

    @staticmethod
    def _quantile(values: list[float], q: float) -> float:
        idx = min(len(values) - 1, max(0, int(round((len(values) - 1) * q))))
        return values[idx]
