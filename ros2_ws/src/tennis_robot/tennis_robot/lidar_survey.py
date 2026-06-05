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
    FIND_BOUNDARY = "find_boundary"
    FIND_BASELINE = "find_baseline"
    TURN_TO_SIDELINE = "turn_to_left_sideline"
    DRIVE_SIDELINE = "drive_left_sideline"
    TURN_TO_LONG_SIDE = "turn_to_long_side"
    DRIVE_LONG_SIDE = "drive_long_side"
    TURN_TO_FAR_SHORT = "turn_to_far_short_side"
    DRIVE_FAR_SHORT = "drive_far_short_side"
    TURN_TO_RETURN = "turn_to_return_long"
    DRIVE_RETURN = "drive_return_long"
    RETURN_TO_CENTER = "return_to_center"
    INITIAL_SCAN = "initial_lidar_scan"
    APPROACH_NET = "approach_net_for_visual_confirmation"
    CONFIRM_NET_VISUAL = "confirm_net_visual"
    CROSS_TO_FAR_SIDE = "cross_to_far_side"
    FAR_SIDE_SCAN = "far_side_lidar_scan"
    VALIDATE_SURVEY = "validate_lidar_survey"
    DONE = "done"


@dataclass(frozen=True)
class LidarSurveyConfig:
    drive_speed_m_s: float = 0.60
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
    net_post_min_boundary_inset_m: float = 1.0
    cross_post_clearance_m: float = 1.0
    far_side_reveal_m: float = 3.2
    target_tolerance_m: float = 0.25
    heading_tolerance_rad: float = math.radians(8.0)
    expected_court_length_m: float = 23.77
    expected_court_width_m: float = 10.97
    sideline_drive_stop_range_m: float = 2.5
    sideline_drive_timeout_s: float = 300.0
    sideline_sector_half_deg: float = 25.0
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
            net_post_min_boundary_inset_m=_env_float(
                "ROS2_SURVEY_NET_POST_MIN_BOUNDARY_INSET_M",
                d.net_post_min_boundary_inset_m,
            ),
            cross_post_clearance_m=_env_float("ROS2_SURVEY_CROSS_POST_CLEARANCE_M", d.cross_post_clearance_m),
            far_side_reveal_m=_env_float("ROS2_SURVEY_FAR_SIDE_REVEAL_M", d.far_side_reveal_m),
            target_tolerance_m=_env_float("ROS2_SURVEY_TARGET_TOLERANCE_M", d.target_tolerance_m),
            expected_court_length_m=_env_float("SURVEY_EXPECTED_COURT_LENGTH_M", d.expected_court_length_m),
            expected_court_width_m=_env_float("SURVEY_EXPECTED_COURT_WIDTH_M", d.expected_court_width_m),
            sideline_drive_stop_range_m=_env_float("SURVEY_SIDELINE_DRIVE_STOP_M", d.sideline_drive_stop_range_m),
            sideline_drive_timeout_s=_env_float("SURVEY_SIDELINE_DRIVE_TIMEOUT_S", d.sideline_drive_timeout_s),
            sideline_sector_half_deg=_env_float("SURVEY_SIDELINE_SECTOR_HALF_DEG", d.sideline_sector_half_deg),
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
        self.state = LidarSurveyState.FIND_BOUNDARY
        self.sample_count = 0
        self._obstacle_survey: "ObstacleSurvey | None" = None
        self._baseline_survey: "ObstacleSurvey | None" = None
        self._first_obstacle: dict | None = None   # net/fence found in FIND_BOUNDARY
        self._second_obstacle: dict | None = None  # baseline/fence found in FIND_BASELINE
        self._sideline_heading: float | None = None   # world heading during DRIVE_SIDELINE
        self._long_side_heading: float | None = None  # world heading during DRIVE_LONG_SIDE
        self._far_short_heading: float | None = None  # world heading during DRIVE_FAR_SHORT
        self._return_heading: float | None = None     # world heading during DRIVE_RETURN
        self._left_range_samples: list[float] = []    # cross-track LiDAR during short-side drive
        self._right_range_samples: list[float] = []
        self._long_left_range_samples: list[float] = []    # cross-track LiDAR during long-side drive
        self._far_short_range_samples: list[float] = []    # cross-track LiDAR during far short side
        self._return_range_samples: list[float] = []       # cross-track LiDAR during return leg
        # Per-phase line-crossing: front range the first time a court line is detected
        self._near_baseline_to_fence_m: float | None = None   # Phase 2
        self._left_sideline_to_fence_m: float | None = None   # Phase 4
        self._left_sideline_line_crossed: bool = False
        self._far_baseline_to_fence_m: float | None = None    # Phase 6
        self._far_baseline_crossed: bool = False
        self._right_sideline_to_fence_m: float | None = None  # Phase 8
        self._right_sideline_line_crossed: bool = False
        self._center_target: tuple[float, float] | None = None  # drive-back waypoint
        self._center_yaw: float | None = None
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
        self._lidar_angle_min: float = -math.pi
        self._lidar_angle_increment: float = 2.0 * math.pi / 360
        self._net_detection_debug: dict = {}
        self._last_scan_ranges: list[float] = []
        self._last_scan_pose: tuple[float, float, float] = (0.0, 0.0, 0.0)

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
        lidar_angle_min: float = -math.pi,
        lidar_angle_increment: float | None = None,
    ) -> LidarSurveyCommand:
        if self._started_at is None:
            self._started_at = time.time()
            self._obstacle_survey = ObstacleSurvey()
            self._enter(LidarSurveyState.FIND_BOUNDARY, "ros2_lidar_map_court_started")
        if self.state == LidarSurveyState.DONE:
            return self._cmd(BaseCommand(0.0, 0.0))

        self._dt_s = max(0.0, dt_s)
        self._state_elapsed_s += self._dt_s
        self._update_distance(x_m, y_m)
        if lidar_ranges:
            n = len(lidar_ranges)
            self._lidar_angle_min = lidar_angle_min
            self._lidar_angle_increment = (
                lidar_angle_increment if lidar_angle_increment is not None
                else 2.0 * math.pi / max(1, n)
            )
        self._last_front_range_m = self._front_range(lidar_ranges)
        if lidar_ranges:
            self._last_scan_ranges = lidar_ranges
            self._last_scan_pose = (x_m, y_m, yaw_rad)
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
            "net_detection_debug": self._net_detection_debug,
            "obstacle_survey": (
                self._obstacle_survey.telemetry()
                if self._obstacle_survey is not None else None
            ),
            "baseline_survey": (
                self._baseline_survey.telemetry()
                if self._baseline_survey is not None else None
            ),
            "first_obstacle": self._first_obstacle,
            "second_obstacle": self._second_obstacle,
            "sideline_heading_deg": (
                None if self._sideline_heading is None
                else round(math.degrees(self._sideline_heading) % 360, 1)
            ),
            "long_side_heading_deg": (
                None if self._long_side_heading is None
                else round(math.degrees(self._long_side_heading) % 360, 1)
            ),
            "left_fence_sample_count": len(self._left_range_samples),
            "right_fence_sample_count": len(self._right_range_samples),
            "long_left_fence_sample_count": len(self._long_left_range_samples),
            "sideline_to_fence_m": self._sideline_to_fence_m,
            "net_passed": self._net_passed,
            "far_baseline_crossed": self._far_baseline_crossed,
            "far_baseline_to_fence_m": self._far_baseline_to_fence_m,
        }

    def _step(self, x_m: float, y_m: float, yaw_rad: float, vision: SurveyVision | None) -> BaseCommand:

        # ── Phase 1: find net (nearest boundary in any direction) ─────────────
        if self.state == LidarSurveyState.FIND_BOUNDARY:
            if self._obstacle_survey is None:
                self._obstacle_survey = ObstacleSurvey()
            obs_cmd = self._obstacle_survey.update(
                x_m, y_m, yaw_rad,
                self._last_scan_ranges or None, getattr(self, "_dt_s", 0.0),
                vision=vision,
                lidar_angle_min=self._lidar_angle_min,
                lidar_angle_increment=self._lidar_angle_increment,
            )
            if self._obstacle_survey.state == ObstacleSurveyState.DONE:
                result = self._obstacle_survey.result or {}
                if result.get("status") == "SUCCESS":
                    self._first_obstacle = {
                        "type": result.get("obstacle_type"),
                        "distance_m": result.get("obstacle_distance_m"),
                        "world_pos": result.get("obstacle_world_pos"),
                        "vision_class": result.get("vision_class"),
                    }
                    # ── Phase 2: turn 180° and find baseline/fence ────────────
                    _bl_cfg = ObstacleSurveyConfig(
                        drive_speed_m_s=0.70,
                        stop_at_range_m=2.50,
                        approach_timeout_s=300.0,
                        safety_stop_range_m=0.35,
                    )
                    self._baseline_survey = ObstacleSurvey(_bl_cfg)
                    self._baseline_survey._target_world_heading = yaw_rad + math.pi
                    self._baseline_survey._approach_bearing_rad = yaw_rad + math.pi
                    self._baseline_survey._pre_turn_front_range_m = self._last_front_range_m
                    self._baseline_survey._enter(ObstacleSurveyState.APPROACH, "turning_for_baseline")
                    self._enter(LidarSurveyState.FIND_BASELINE, "net_found_turning_180_for_baseline")
                else:
                    self._fail("BOUNDARY_NOT_FOUND")
            return obs_cmd.base

        # ── Phase 2: turn 180° then find baseline / fence ─────────────────────
        if self.state == LidarSurveyState.FIND_BASELINE:
            if self._baseline_survey is None:
                self._fail("BASELINE_SURVEY_NOT_INITIALIZED")
                return BaseCommand(0.0, 0.0)
            # Pass only line-detection vision (obstacle_class stripped) so the
            # camera cannot trigger a premature stop during the 180° rotation,
            # but the baseline line crossing IS recorded for baseline_to_fence_m.
            line_vision = None
            if vision is not None:
                line_vision = SurveyVision(
                    line_detected=vision.line_detected,
                    line_offset_m=vision.line_offset_m,
                    line_heading_error_rad=vision.line_heading_error_rad,
                    line_confidence=vision.line_confidence,
                )
            bl_cmd = self._baseline_survey.update(
                x_m, y_m, yaw_rad,
                self._last_scan_ranges or None, getattr(self, "_dt_s", 0.0),
                vision=line_vision,
                lidar_angle_min=self._lidar_angle_min,
                lidar_angle_increment=self._lidar_angle_increment,
            )
            if self._baseline_survey.state == ObstacleSurveyState.DONE:
                result = self._baseline_survey.result or {}
                self._second_obstacle = {
                    "type": result.get("obstacle_type"),
                    "distance_m": result.get("obstacle_distance_m"),
                    "world_pos": result.get("obstacle_world_pos"),
                    "vision_class": result.get("vision_class"),
                    "status": result.get("status"),
                }
                # Front range when baseline line was crossed = baseline-to-fence gap
                self._near_baseline_to_fence_m = self._baseline_survey._court_line_range_m
                # 90° left from the direction we drove to baseline
                baseline_bearing = self._baseline_survey._approach_bearing_rad or 0.0
                self._sideline_heading = baseline_bearing + math.pi / 2
                self._enter(LidarSurveyState.TURN_TO_SIDELINE, "baseline_found_turning_90_left")
            return bl_cmd.base

        # ── Phase 3: 90° left turn toward sideline ─────────────────────────────
        if self.state == LidarSurveyState.TURN_TO_SIDELINE:
            if self._sideline_heading is None:
                self._fail("SIDELINE_HEADING_NOT_SET")
                return BaseCommand(0.0, 0.0)
            err = self._angle_delta(self._sideline_heading, yaw_rad)
            if abs(err) <= self.config.heading_tolerance_rad:
                self._enter(LidarSurveyState.DRIVE_SIDELINE, "aligned_for_sideline_drive")
                return BaseCommand(0.0, 0.0)
            return BaseCommand(0.0, math.copysign(self.config.turn_speed_rad_s, err))

        # ── Phase 4: drive along left sideline; measure fence distances ─────────
        if self.state == LidarSurveyState.DRIVE_SIDELINE:
            if self._state_elapsed_s >= self.config.sideline_drive_timeout_s:
                self._finalize_full_survey("sideline_drive_timeout")
                return BaseCommand(0.0, 0.0)
            if self._last_scan_ranges:
                half = math.radians(self.config.sideline_sector_half_deg)
                left_r = self._sector_median_range(self._last_scan_ranges, -math.pi / 2, half)
                right_r = self._sector_median_range(self._last_scan_ranges, math.pi / 2, half)
                if math.isfinite(left_r):
                    self._left_range_samples.append(left_r)
                    if len(self._left_range_samples) > 300:
                        del self._left_range_samples[:150]
                if math.isfinite(right_r):
                    self._right_range_samples.append(right_r)
                    if len(self._right_range_samples) > 300:
                        del self._right_range_samples[:150]
                # Record left-sideline-to-fence gap when inner sideline line first seen
                if (
                    vision is not None
                    and vision.line_detected
                    and not self._left_sideline_line_crossed
                    and not math.isinf(self._last_front_range_m)
                ):
                    self._left_sideline_to_fence_m = round(self._last_front_range_m, 3)
                    self._left_sideline_line_crossed = True
            front = self._last_front_range_m
            if not math.isinf(front) and front <= self.config.sideline_drive_stop_range_m:
                # Short leg complete — turn 90° left again for the long leg
                self._long_side_heading = (self._sideline_heading or 0.0) + math.pi / 2
                self._enter(LidarSurveyState.TURN_TO_LONG_SIDE, "side_fence_reached_turning_90_left")
                return BaseCommand(0.0, 0.0)
            return self._drive_straight_heading(yaw_rad, self._sideline_heading)

        # ── Phase 5: second 90° left turn (corner: side fence → long leg) ────────
        if self.state == LidarSurveyState.TURN_TO_LONG_SIDE:
            if self._long_side_heading is None:
                self._fail("LONG_SIDE_HEADING_NOT_SET")
                return BaseCommand(0.0, 0.0)
            err = self._angle_delta(self._long_side_heading, yaw_rad)
            if abs(err) <= self.config.heading_tolerance_rad:
                self._enter(LidarSurveyState.DRIVE_LONG_SIDE, "aligned_for_long_side_drive")
                return BaseCommand(0.0, 0.0)
            return BaseCommand(0.0, math.copysign(self.config.turn_speed_rad_s, err))

        # ── Phase 6: drive the long side (past net, toward opposite baseline) ───
        if self.state == LidarSurveyState.DRIVE_LONG_SIDE:
            if self._state_elapsed_s >= self.config.sideline_drive_timeout_s:
                self._finalize_full_survey("long_side_drive_timeout")
                return BaseCommand(0.0, 0.0)
            if self._last_scan_ranges:
                half = math.radians(self.config.sideline_sector_half_deg)
                left_r = self._sector_median_range(self._last_scan_ranges, -math.pi / 2, half)
                if math.isfinite(left_r):
                    self._long_left_range_samples.append(left_r)
                    if len(self._long_left_range_samples) > 400:
                        del self._long_left_range_samples[:200]
                # Detect far baseline crossing (first court line after passing net)
                if (
                    vision is not None
                    and vision.line_detected
                    and not self._far_baseline_crossed
                    and not math.isinf(self._last_front_range_m)
                ):
                    self._far_baseline_to_fence_m = round(self._last_front_range_m, 3)
                    self._far_baseline_crossed = True
            front = self._last_front_range_m
            if not math.isinf(front) and front <= self.config.sideline_drive_stop_range_m:
                self._far_short_heading = (self._long_side_heading or 0.0) + math.pi / 2
                self._enter(LidarSurveyState.TURN_TO_FAR_SHORT, "far_baseline_fence_reached_turning_90_left")
                return BaseCommand(0.0, 0.0)
            return self._drive_straight_heading(yaw_rad, self._long_side_heading)

        # ── Phase 7: third 90° left turn (far baseline → far short side) ────────
        if self.state == LidarSurveyState.TURN_TO_FAR_SHORT:
            if self._far_short_heading is None:
                self._fail("FAR_SHORT_HEADING_NOT_SET")
                return BaseCommand(0.0, 0.0)
            err = self._angle_delta(self._far_short_heading, yaw_rad)
            if abs(err) <= self.config.heading_tolerance_rad:
                self._enter(LidarSurveyState.DRIVE_FAR_SHORT, "aligned_for_far_short_drive")
                return BaseCommand(0.0, 0.0)
            return BaseCommand(0.0, math.copysign(self.config.turn_speed_rad_s, err))

        # ── Phase 8: drive far short side (opposite side of court) ──────────────
        if self.state == LidarSurveyState.DRIVE_FAR_SHORT:
            if self._state_elapsed_s >= self.config.sideline_drive_timeout_s:
                self._finalize_full_survey("far_short_drive_timeout")
                return BaseCommand(0.0, 0.0)
            if self._last_scan_ranges:
                half = math.radians(self.config.sideline_sector_half_deg)
                left_r = self._sector_median_range(self._last_scan_ranges, -math.pi / 2, half)
                if math.isfinite(left_r):
                    self._far_short_range_samples.append(left_r)
                    if len(self._far_short_range_samples) > 300:
                        del self._far_short_range_samples[:150]
                # Record right-sideline-to-fence gap when inner sideline line first seen
                if (
                    vision is not None
                    and vision.line_detected
                    and not self._right_sideline_line_crossed
                    and not math.isinf(self._last_front_range_m)
                ):
                    self._right_sideline_to_fence_m = round(self._last_front_range_m, 3)
                    self._right_sideline_line_crossed = True
            front = self._last_front_range_m
            if not math.isinf(front) and front <= self.config.sideline_drive_stop_range_m:
                self._return_heading = (self._far_short_heading or 0.0) + math.pi / 2
                self._enter(LidarSurveyState.TURN_TO_RETURN, "far_side_fence_reached_turning_90_left")
                return BaseCommand(0.0, 0.0)
            return self._drive_straight_heading(yaw_rad, self._far_short_heading)

        # ── Phase 9: fourth 90° left turn (far side fence → return long side) ───
        if self.state == LidarSurveyState.TURN_TO_RETURN:
            if self._return_heading is None:
                self._fail("RETURN_HEADING_NOT_SET")
                return BaseCommand(0.0, 0.0)
            err = self._angle_delta(self._return_heading, yaw_rad)
            if abs(err) <= self.config.heading_tolerance_rad:
                self._enter(LidarSurveyState.DRIVE_RETURN, "aligned_for_return_drive")
                return BaseCommand(0.0, 0.0)
            return BaseCommand(0.0, math.copysign(self.config.turn_speed_rad_s, err))

        # ── Phase 10: return long side (net passes on RIGHT) — full loop done ───
        if self.state == LidarSurveyState.DRIVE_RETURN:
            if self._state_elapsed_s >= self.config.sideline_drive_timeout_s:
                self._finalize_full_survey("return_drive_timeout")
                return BaseCommand(0.0, 0.0)
            if self._last_scan_ranges:
                half = math.radians(self.config.sideline_sector_half_deg)
                left_r = self._sector_median_range(self._last_scan_ranges, -math.pi / 2, half)
                if math.isfinite(left_r):
                    self._return_range_samples.append(left_r)
                    if len(self._return_range_samples) > 400:
                        del self._return_range_samples[:200]
            front = self._last_front_range_m
            if not math.isinf(front) and front <= self.config.sideline_drive_stop_range_m:
                self._finalize_full_survey(None)
                return BaseCommand(0.0, 0.0)
            return self._drive_straight_heading(yaw_rad, self._return_heading)

        # ── Phases below are disabled (full LiDAR court survey — future work) ──
        # if self.state == LidarSurveyState.RETURN_TO_CENTER: ...
        # if self.state == LidarSurveyState.INITIAL_SCAN: ...
        # if self.state == LidarSurveyState.APPROACH_NET: ...
        # if self.state == LidarSurveyState.CONFIRM_NET_VISUAL: ...
        # if self.state == LidarSurveyState.CROSS_TO_FAR_SIDE: ...
        # if self.state == LidarSurveyState.FAR_SIDE_SCAN: ...
        # if self.state == LidarSurveyState.VALIDATE_SURVEY: self._finalize()

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
            angle = self._lidar_angle_min + i * self._lidar_angle_increment
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
            angle = self._lidar_angle_min + i * self._lidar_angle_increment
            lx = r * math.cos(angle)
            ly = r * math.sin(angle)
            points.append((x_m + cos_y * lx - sin_y * ly, y_m + sin_y * lx + cos_y * ly))
        return points

    def _detect_net_frame(self, points: list[tuple[float, float]], robot: tuple[float, float]) -> NetFrame | None:
        frame = self._detect_net_as_line(points, robot)
        if frame is not None:
            return frame
        # Cluster detection: augment accumulated (downsampled) points with the
        # full-resolution last scan so thin posts (1 beam wide) aren't missed.
        full_pts = self._full_res_scan_points_world()
        combined = points + full_pts if full_pts else points
        return self._detect_net_from_clusters(combined, robot)

    def _full_res_scan_points_world(self) -> list[tuple[float, float]]:
        """Return world-frame points from the last scan at full resolution (step=1)."""
        ranges = self._last_scan_ranges
        if not ranges or len(ranges) < 10:
            return []
        x_m, y_m, yaw_rad = self._last_scan_pose
        cos_y = math.cos(yaw_rad)
        sin_y = math.sin(yaw_rad)
        pts: list[tuple[float, float]] = []
        for i, r in enumerate(ranges):
            if not math.isfinite(r) or r < self.config.lidar_min_range_m or r > self.config.lidar_max_range_m:
                continue
            angle = self._lidar_angle_min + i * self._lidar_angle_increment
            lx = r * math.cos(angle)
            ly = r * math.sin(angle)
            pts.append((x_m + cos_y * lx - sin_y * ly, y_m + sin_y * lx + cos_y * ly))
        return pts

    def _detect_net_as_line(self, points: list[tuple[float, float]], robot: tuple[float, float]) -> NetFrame | None:
        """Detect net as a continuous line feature (LiDAR hits the net mesh, not just posts)."""
        extents = self._world_extents(points)
        dbg: dict = {
            "method": "line",
            "total_points": extents.get("point_count", 0),
            "span_x_m": round(extents.get("span_x_m", 0.0), 2),
            "span_y_m": round(extents.get("span_y_m", 0.0), 2),
        }
        if extents.get("point_count", 0) < 40:
            dbg["reject"] = "too_few_points"
            self._net_detection_debug = dbg
            return None
        span_x = extents.get("span_x_m", 0.0)
        span_y = extents.get("span_y_m", 0.0)
        if span_x <= 1.0 or span_y <= 1.0:
            dbg["reject"] = "span_too_small"
            self._net_detection_debug = dbg
            return None

        # If span_y substantially exceeds the court play-width, those are side-fence
        # returns and the court long axis must be X — regardless of span_x.
        # This handles the case where end fences are at or near LiDAR range limit,
        # making span_x appear artificially small.
        long_axis_x = (
            span_x >= span_y
            or span_y > self.config.expected_court_width_m * 1.3
        )
        half_court_width = self.config.expected_court_width_m * 0.5
        dbg["long_axis"] = "x" if long_axis_x else "y"

        if long_axis_x:
            cx = (extents["min_x_m"] + extents["max_x_m"]) * 0.5
            cy = (extents["min_y_m"] + extents["max_y_m"]) * 0.5
            long_win = max(2.0, span_x * 0.07)
            cross_lim = half_court_width + 1.0
            net_pts = [(x, y) for x, y in points if abs(x - cx) <= long_win and abs(y - cy) <= cross_lim]
            dbg.update({"cx": round(cx, 2), "cy": round(cy, 2), "long_win_m": round(long_win, 2), "cross_lim_m": round(cross_lim, 2), "net_pts": len(net_pts)})
            if len(net_pts) < 15:
                dbg["reject"] = "too_few_net_pts"
                self._net_detection_debug = dbg
                return None
            vals = sorted(p[1] for p in net_pts)
            lo, hi = self._quantile(vals, 0.03), self._quantile(vals, 0.97)
            avg_x = sum(p[0] for p in net_pts) / len(net_pts)
            post_a, post_b = (avg_x, lo), (avg_x, hi)
        else:
            cx = (extents["min_x_m"] + extents["max_x_m"]) * 0.5
            cy = (extents["min_y_m"] + extents["max_y_m"]) * 0.5
            long_win = max(2.0, span_y * 0.07)
            cross_lim = half_court_width + 1.0
            net_pts = [(x, y) for x, y in points if abs(y - cy) <= long_win and abs(x - cx) <= cross_lim]
            dbg.update({"cx": round(cx, 2), "cy": round(cy, 2), "long_win_m": round(long_win, 2), "cross_lim_m": round(cross_lim, 2), "net_pts": len(net_pts)})
            if len(net_pts) < 15:
                dbg["reject"] = "too_few_net_pts"
                self._net_detection_debug = dbg
                return None
            vals = sorted(p[0] for p in net_pts)
            lo, hi = self._quantile(vals, 0.03), self._quantile(vals, 0.97)
            avg_y = sum(p[1] for p in net_pts) / len(net_pts)
            post_a, post_b = (lo, avg_y), (hi, avg_y)

        sep = self._distance(post_a, post_b)
        dbg["separation_m"] = round(sep, 3)
        if not (self.config.net_post_min_separation_m <= sep <= self.config.net_post_max_separation_m):
            dbg["reject"] = f"separation_out_of_range ({self.config.net_post_min_separation_m}..{self.config.net_post_max_separation_m})"
            self._net_detection_debug = dbg
            return None
        dbg["result"] = "ok"
        self._net_detection_debug = dbg
        return self._build_net_frame(post_a, post_b, robot, sep)

    def _detect_net_from_clusters(self, points: list[tuple[float, float]], robot: tuple[float, float]) -> NetFrame | None:
        """Detect net from discrete post-like point clusters (fallback for real posts)."""
        clusters = self._clusters(points)
        extents = self._world_extents(points)
        if extents.get("point_count", 0) <= 0:
            return None
        best: tuple[float, tuple[float, float], tuple[float, float]] | None = None
        for i, a in enumerate(clusters):
            inset_a = self._boundary_inset_m(a, extents)
            if inset_a < self.config.net_post_min_boundary_inset_m:
                continue
            for b in clusters[i + 1:]:
                inset_b = self._boundary_inset_m(b, extents)
                if inset_b < self.config.net_post_min_boundary_inset_m:
                    continue
                sep = self._distance(a, b)
                if not (self.config.net_post_min_separation_m <= sep <= self.config.net_post_max_separation_m):
                    continue
                mid = ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5)
                robot_dist = self._distance(mid, robot)
                inset_score = min(inset_a, inset_b)
                width_error = abs(sep - self.config.expected_court_width_m)
                score = inset_score * 2.0 - width_error * 1.5 - 0.12 * robot_dist
                if best is None or score > best[0]:
                    best = (score, a, b)
        if best is None:
            return None
        return self._build_net_frame(best[1], best[2], robot)

    def _build_net_frame(
        self,
        post_a: tuple[float, float],
        post_b: tuple[float, float],
        robot: tuple[float, float],
        sep: float | None = None,
    ) -> NetFrame:
        if sep is None:
            sep = self._distance(post_a, post_b)
        mid = ((post_a[0] + post_b[0]) * 0.5, (post_a[1] + post_b[1]) * 0.5)
        dx = post_b[0] - post_a[0]
        dy = post_b[1] - post_a[1]
        length = max(0.001, math.hypot(dx, dy))
        tangent = (dx / length, dy / length)
        n1 = (-tangent[1], tangent[0])
        to_robot = (robot[0] - mid[0], robot[1] - mid[1])
        near = n1 if self._dot(n1, to_robot) >= 0.0 else (-n1[0], -n1[1])
        far = (-near[0], -near[1])
        width_error = abs(sep - self.config.expected_court_width_m)
        confidence = min(0.95, 0.55 + (0.35 if width_error < 1.0 else 0.0) - width_error * 0.03)
        return NetFrame(post_a, post_b, mid, tangent, near, far, round(max(0.5, confidence), 2))

    @staticmethod
    def _boundary_inset_m(point: tuple[float, float], extents: dict) -> float:
        x, y = point
        distances = [
            x - extents["min_x_m"],
            extents["max_x_m"] - x,
            y - extents["min_y_m"],
            extents["max_y_m"] - y,
        ]
        return min(distances)

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

    def _build_two_point_geometry(self) -> tuple[str, dict]:
        """Compute Phase 1+2 geometry dict; returns (status, geometry_dict)."""
        baseline_ok = (
            self._second_obstacle is not None
            and self._second_obstacle.get("status") == "SUCCESS"
        )
        status = "SUCCESS" if (self._first_obstacle and baseline_ok) else "PARTIAL"
        net_pos = (self._first_obstacle or {}).get("world_pos")
        fence_pos = (self._second_obstacle or {}).get("world_pos")
        line_to_fence_m = (self._second_obstacle or {}).get("line_to_fence_m")
        net_to_fence_m = None
        if net_pos and fence_pos:
            net_to_fence_m = round(math.hypot(
                fence_pos["x_m"] - net_pos["x_m"],
                fence_pos["y_m"] - net_pos["y_m"],
            ), 3)
        baseline_to_fence_m = line_to_fence_m
        net_to_baseline_m = None
        if net_to_fence_m is not None and baseline_to_fence_m is not None:
            net_to_baseline_m = round(net_to_fence_m - baseline_to_fence_m, 3)
        geo = {
            "net_world_pos": net_pos,
            "fence_world_pos": fence_pos,
            "net_to_fence_m": net_to_fence_m,
            "baseline_to_fence_m": baseline_to_fence_m,
            "net_to_baseline_m": net_to_baseline_m,
        }
        return status, geo

    def _write_bounds(self, bounds: dict) -> None:
        self._court_bounds = bounds
        self.config.output_file.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.config.output_file.with_suffix(".tmp.json")
        tmp.write_text(json.dumps(bounds, indent=2) + "\n", encoding="utf-8")
        tmp.replace(self.config.output_file)

    def _finalize_two_point(self) -> None:
        """Fallback: write two-point result and go to DONE (used if Phase 3/4 fail)."""
        now = time.time()
        elapsed = 0.0 if self._started_at is None else now - self._started_at
        status, geo = self._build_two_point_geometry()
        bounds = {
            "surveyed_at": now,
            "status": status,
            "survey_complete": status == "SUCCESS",
            "survey_type": "two_point_net_baseline",
            "net": self._first_obstacle,
            "baseline": self._second_obstacle,
            "geometry": geo,
            "elapsed_s": round(elapsed, 1),
            "court_geometry": {
                "length_m": self.config.expected_court_length_m,
                "width_m": self.config.expected_court_width_m,
                "method": "two_point_visual_survey",
            },
        }
        self._write_bounds(bounds)
        self._enter(LidarSurveyState.DONE, f"two_point_survey_{status.lower()}")

    def _finalize_full_survey(self, failure: str | None) -> None:
        """Write full perimeter survey result (Phases 1–4) and go to DONE."""
        now = time.time()
        elapsed = 0.0 if self._started_at is None else now - self._started_at
        _status, geo = self._build_two_point_geometry()

        # Sideline measurements
        def _median(samples: list[float]) -> float | None:
            if not samples:
                return None
            s = sorted(samples)
            return round(s[len(s) // 2], 3)

        # Doubles detection: compare measured sideline-to-fence against the
        # known singles-alley width (1.37 m).  If BOTH inner sidelines are
        # more than 1.8 m from their respective fences → there is room for a
        # doubles alley → court is played as doubles.
        left_sl = self._left_sideline_to_fence_m
        right_sl = self._right_sideline_to_fence_m
        DOUBLES_ALLEY_M = 1.37
        is_doubles: bool | None = None
        if left_sl is not None and right_sl is not None:
            is_doubles = (left_sl > DOUBLES_ALLEY_M + 0.4) and (right_sl > DOUBLES_ALLEY_M + 0.4)
        elif left_sl is not None:
            is_doubles = left_sl > DOUBLES_ALLEY_M + 0.4

        sideline_status = "SUCCESS" if (left_sl is not None) else "PARTIAL"
        overall_status = "SUCCESS" if (_status == "SUCCESS" and sideline_status == "SUCCESS") else "PARTIAL"
        if failure:
            overall_status = "PARTIAL"

        bounds = {
            "surveyed_at": now,
            "status": overall_status,
            "survey_complete": overall_status == "SUCCESS",
            "survey_type": "full_perimeter",
            "failure_reason": failure,
            "net": self._first_obstacle,
            "baseline": self._second_obstacle,
            "geometry": geo,
            "boundary_distances": {
                "near_baseline_to_fence_m": self._near_baseline_to_fence_m,
                "far_baseline_to_fence_m": self._far_baseline_to_fence_m,
                "left_sideline_to_fence_m": self._left_sideline_to_fence_m,
                "right_sideline_to_fence_m": self._right_sideline_to_fence_m,
                "far_baseline_crossed": self._far_baseline_crossed,
            },
            "is_doubles": is_doubles,
            "elapsed_s": round(elapsed, 1),
            "court_geometry": {
                "length_m": self.config.expected_court_length_m,
                "width_m": self.config.expected_court_width_m,
                "method": "full_perimeter_survey",
            },
        }
        self._write_bounds(bounds)
        self._enter(LidarSurveyState.DONE, f"full_perimeter_survey_{overall_status.lower()}")

    # ── Full LiDAR court survey finalize (disabled — future work) ─────────────
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

    def _sector_median_range(
        self,
        ranges: list[float],
        center_rad: float,
        half_rad: float,
    ) -> float:
        """Median valid range in the angular sector [center−half, center+half]."""
        vals: list[float] = []
        for i, r in enumerate(ranges):
            if not math.isfinite(r) or r < self.config.lidar_min_range_m or r > self.config.lidar_max_range_m:
                continue
            angle = self._lidar_angle_min + i * self._lidar_angle_increment
            angle = (angle + math.pi) % (2.0 * math.pi) - math.pi
            if center_rad - half_rad <= angle <= center_rad + half_rad:
                vals.append(r)
        if not vals:
            return math.inf
        vals.sort()
        return vals[len(vals) // 2]

    def _drive_straight_heading(self, yaw_rad: float, target_heading: float) -> BaseCommand:
        """Drive forward while keeping the given world heading (P-controller)."""
        err = self._angle_delta(target_heading, yaw_rad)
        turn = max(
            -self.config.turn_speed_rad_s * 0.5,
            min(self.config.turn_speed_rad_s * 0.5, err * 1.8),
        )
        return BaseCommand(self.config.drive_speed_m_s, turn)

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


# ─── Baseline-first boundary scan ─────────────────────────────────────────────


class BoundaryScanState(str, Enum):
    FIND_WALL = "find_perpendicular_wall"
    APPROACH  = "approach_boundary"
    DONE      = "done"


@dataclass(frozen=True)
class BoundaryScanConfig:
    drive_speed_m_s: float = 0.30
    turn_speed_rad_s: float = 0.70
    safety_stop_range_m: float = 0.40
    safety_slow_range_m: float = 0.80
    lidar_min_range_m: float = 0.35
    lidar_max_range_m: float = 12.0
    # Angular half-width of the forward and rear sectors.
    front_half_angle_deg: float = 22.0
    rear_half_angle_deg: float = 22.0
    find_wall_min_scans: int = 3
    find_wall_timeout_s: float = 12.0
    approach_timeout_s: float = 90.0
    heading_kp: float = 1.5
    heading_tolerance_rad: float = math.radians(8.0)
    output_file: Path = DEFAULT_BOUNDARY_FILE

    @classmethod
    def from_env(cls) -> "BoundaryScanConfig":
        d = cls()
        return cls(
            drive_speed_m_s=_env_float("BOUNDARY_SCAN_DRIVE_SPEED_M_S", d.drive_speed_m_s),
            turn_speed_rad_s=_env_float("BOUNDARY_SCAN_TURN_SPEED_RAD_S", d.turn_speed_rad_s),
            safety_stop_range_m=_env_float("BOUNDARY_SCAN_SAFETY_STOP_M", d.safety_stop_range_m),
            safety_slow_range_m=_env_float("BOUNDARY_SCAN_SAFETY_SLOW_M", d.safety_slow_range_m),
            lidar_min_range_m=_env_float("BOUNDARY_SCAN_LIDAR_MIN_M", d.lidar_min_range_m),
            lidar_max_range_m=_env_float("BOUNDARY_SCAN_LIDAR_MAX_M", d.lidar_max_range_m),
            front_half_angle_deg=_env_float("BOUNDARY_SCAN_FRONT_HALF_DEG", d.front_half_angle_deg),
            rear_half_angle_deg=_env_float("BOUNDARY_SCAN_REAR_HALF_DEG", d.rear_half_angle_deg),
            find_wall_min_scans=int(_env_float("BOUNDARY_SCAN_MIN_SCANS", float(d.find_wall_min_scans))),
            find_wall_timeout_s=_env_float("BOUNDARY_SCAN_FIND_TIMEOUT_S", d.find_wall_timeout_s),
            approach_timeout_s=_env_float("BOUNDARY_SCAN_APPROACH_TIMEOUT_S", d.approach_timeout_s),
            heading_kp=_env_float("BOUNDARY_SCAN_HEADING_KP", d.heading_kp),
            heading_tolerance_rad=_env_float("BOUNDARY_SCAN_HEADING_TOL_RAD", d.heading_tolerance_rad),
            output_file=Path(os.getenv("SURVEY_OUTPUT_FILE", str(d.output_file))),
        )


@dataclass(frozen=True)
class BoundaryScanCommand:
    state: BoundaryScanState
    base: BaseCommand
    front_range_m: float


class BaselineBoundarySurvey:
    """Drive toward the nearest perpendicular (end) fence, stop at the baseline,
    and record the boundary and the baseline-to-fence gap.

    Threshold derived entirely from the initial LiDAR scan — no hardcoded
    court dimensions.  The robot is placed AT the rear baseline before starting:

      D_rear  = median rear-sector range  → direct reading of the rear fence
                (hardware guarantees D_rear < 12 m, so no obstacle between
                robot and rear fence)
      D_front = median forward-sector range → distance to the far fence
                (may be beyond hardware range initially)

    The baseline-to-fence gap at the rear equals D_rear (robot starts at
    baseline).  For a symmetric court the front gap is the same, so the robot
    stops when D_front ≤ D_rear.

    State machine:
      FIND_WALL  → accumulate ≥find_wall_min_scans scans; measure D_rear.
                   Fails with REAR_FENCE_NOT_VISIBLE if D_rear = inf.
      APPROACH   → drive straight (heading-corrected); stop when
                   D_front ≤ D_rear (front baseline reached).
      DONE       → motors off, JSON result written.
    """

    def __init__(self, config: BoundaryScanConfig | None = None) -> None:
        self.config = config or BoundaryScanConfig()
        self.state = BoundaryScanState.FIND_WALL
        self._scan_count = 0
        self._state_elapsed_s = 0.0
        self._started_at: float | None = None
        self._last_event = "none"
        self._failure_reason: str | None = None
        self._initial_front_range_m: float = math.inf
        self._initial_rear_range_m: float = math.inf
        self._baseline_detect_range_m: float = math.inf  # computed from scan
        self._current_front_range_m: float = math.inf
        self._current_ranges: list[float] = []
        self._baseline_front_range_m: float | None = None
        self._approach_yaw: float | None = None
        self._last_pose: tuple[float, float] | None = None
        self._pose_at_baseline: tuple[float, float] | None = None
        self._distance_traveled_m = 0.0
        self._lidar_angle_min: float = -math.pi
        self._lidar_angle_increment: float = 2.0 * math.pi / 360
        self._result: dict | None = None

    @classmethod
    def from_env(cls) -> "BaselineBoundarySurvey":
        return cls(BoundaryScanConfig.from_env())

    def reset(self) -> None:
        self.__init__(self.config)

    @property
    def result(self) -> dict | None:
        return self._result

    def update(
        self,
        x_m: float,
        y_m: float,
        yaw_rad: float,
        lidar_ranges: list[float] | None,
        dt_s: float,
        lidar_angle_min: float = -math.pi,
        lidar_angle_increment: float | None = None,
    ) -> BoundaryScanCommand:
        if self._started_at is None:
            self._started_at = time.time()
        if self.state == BoundaryScanState.DONE:
            return BoundaryScanCommand(self.state, BaseCommand(0.0, 0.0), self._current_front_range_m)

        self._state_elapsed_s += max(0.0, dt_s)
        self._update_distance(x_m, y_m)

        if lidar_ranges:
            n = len(lidar_ranges)
            self._lidar_angle_min = lidar_angle_min
            self._lidar_angle_increment = (
                lidar_angle_increment if lidar_angle_increment is not None
                else 2.0 * math.pi / max(1, n)
            )
            self._current_front_range_m = self._sector_range(
                lidar_ranges, 0.0, math.radians(self.config.front_half_angle_deg)
            )
            self._current_ranges = lidar_ranges
            self._scan_count += 1

        cmd = self._step(x_m, y_m, yaw_rad)
        return BoundaryScanCommand(self.state, self._apply_safety(cmd), self._current_front_range_m)

    def telemetry(self) -> dict:
        elapsed = 0.0 if self._started_at is None else time.time() - self._started_at
        return {
            "state": self.state.value,
            "survey_type": "baseline_boundary_scan",
            "last_event": self._last_event,
            "failure_reason": self._failure_reason,
            "scan_count": self._scan_count,
            "front_range_m": None if math.isinf(self._current_front_range_m) else round(self._current_front_range_m, 3),
            "initial_front_range_m": None if math.isinf(self._initial_front_range_m) else round(self._initial_front_range_m, 3),
            "initial_rear_range_m": None if math.isinf(self._initial_rear_range_m) else round(self._initial_rear_range_m, 3),
            "baseline_detect_threshold_m": None if math.isinf(self._baseline_detect_range_m) else round(self._baseline_detect_range_m, 3),
            "baseline_front_range_m": None if self._baseline_front_range_m is None else round(self._baseline_front_range_m, 3),
            "distance_traveled_m": round(self._distance_traveled_m, 2),
            "elapsed_s": round(elapsed, 1),
            "result": self._result,
        }

    # ── state machine ─────────────────────────────────────────────────────────

    def _step(self, x_m: float, y_m: float, yaw_rad: float) -> BaseCommand:
        if self.state == BoundaryScanState.FIND_WALL:
            if self._state_elapsed_s >= self.config.find_wall_timeout_s:
                self._finish("LIDAR_FRONT_WALL_NOT_FOUND")
                return BaseCommand(0.0, 0.0)
            if self._scan_count < self.config.find_wall_min_scans or math.isinf(self._current_front_range_m):
                return BaseCommand(0.0, 0.0)
            # Rear fence is always within hardware range (< 12 m).
            # Median is sufficient — no obstacle between robot and rear fence.
            rear_range = self._sector_range_pct(
                self._current_ranges, math.pi, math.radians(self.config.rear_half_angle_deg), 0.50,
            )
            if math.isinf(rear_range):
                self._finish("REAR_FENCE_NOT_VISIBLE")
                return BaseCommand(0.0, 0.0)
            self._initial_front_range_m = self._current_front_range_m
            self._initial_rear_range_m = rear_range
            # Robot starts at the rear baseline → D_rear = back-court depth.
            # Symmetric court: stop when D_front ≤ D_rear (front baseline reached).
            self._baseline_detect_range_m = max(
                self.config.safety_slow_range_m + 0.1,
                rear_range,
            )
            # Already at or past the front baseline — record immediately.
            if self._current_front_range_m <= self._baseline_detect_range_m:
                self._record_baseline(x_m, y_m)
                return BaseCommand(0.0, 0.0)
            self._approach_yaw = yaw_rad
            self._enter(BoundaryScanState.APPROACH, "perpendicular_wall_found")
            return BaseCommand(0.0, 0.0)

        if self.state == BoundaryScanState.APPROACH:
            if self._state_elapsed_s >= self.config.approach_timeout_s:
                self._finish("APPROACH_TIMEOUT")
                return BaseCommand(0.0, 0.0)
            if not math.isinf(self._current_front_range_m) and self._current_front_range_m <= self._baseline_detect_range_m:
                self._record_baseline(x_m, y_m)
                return BaseCommand(0.0, 0.0)
            return self._drive_straight(yaw_rad)

        return BaseCommand(0.0, 0.0)

    def _drive_straight(self, yaw_rad: float) -> BaseCommand:
        if self._approach_yaw is None:
            return BaseCommand(self.config.drive_speed_m_s, 0.0)
        err = (self._approach_yaw - yaw_rad + math.pi) % (2.0 * math.pi) - math.pi
        turn = max(-self.config.turn_speed_rad_s, min(self.config.turn_speed_rad_s, err * self.config.heading_kp))
        return BaseCommand(self.config.drive_speed_m_s, turn)

    def _record_baseline(self, x_m: float, y_m: float) -> None:
        self._baseline_front_range_m = self._current_front_range_m
        self._pose_at_baseline = (x_m, y_m)
        self._finish(None)

    def _finish(self, failure: str | None) -> None:
        self._failure_reason = failure
        status = "FAILED" if failure else "SUCCESS"
        elapsed = 0.0 if self._started_at is None else time.time() - self._started_at
        d_front = None if math.isinf(self._initial_front_range_m) else round(self._initial_front_range_m, 3)
        d_rear = None if math.isinf(self._initial_rear_range_m) else round(self._initial_rear_range_m, 3)
        total_span = None if (d_front is None or d_rear is None) else round(d_front + d_rear, 3)
        result = {
            "survey_type": "baseline_boundary_scan",
            "status": status,
            "failure_reason": failure,
            "survey_complete": status == "SUCCESS",
            "surveyed_at": time.time(),
            "measured_front_range_m": d_front,
            "measured_rear_range_m": d_rear,
            "measured_total_span_m": total_span,
            "derived_baseline_detect_threshold_m": (
                None if math.isinf(self._baseline_detect_range_m)
                else round(self._baseline_detect_range_m, 3)
            ),
            "baseline_range_m": (
                None if self._baseline_front_range_m is None
                else round(self._baseline_front_range_m, 3)
            ),
            "boundary_distance_from_baseline_m": (
                None if self._baseline_front_range_m is None
                else round(self._baseline_front_range_m, 3)
            ),
            "robot_position_at_baseline": (
                {"x_m": round(self._pose_at_baseline[0], 3), "y_m": round(self._pose_at_baseline[1], 3)}
                if self._pose_at_baseline else None
            ),
            "distance_traveled_m": round(self._distance_traveled_m, 2),
            "elapsed_s": round(elapsed, 1),
        }
        self._result = result
        self.config.output_file.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.config.output_file.with_suffix(".tmp.json")
        tmp.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        tmp.replace(self.config.output_file)
        self._enter(BoundaryScanState.DONE, f"baseline_boundary_scan_{status.lower()}")

    # ── helpers ───────────────────────────────────────────────────────────────

    def _apply_safety(self, command: BaseCommand) -> BaseCommand:
        if command.linear_speed_m_s <= 0.0:
            return command
        if self._current_front_range_m <= self.config.safety_stop_range_m:
            return BaseCommand(0.0, 0.0)
        if self._current_front_range_m <= self.config.safety_slow_range_m:
            return BaseCommand(command.linear_speed_m_s * 0.35, command.angular_speed_rad_s)
        return command

    def _enter(self, state: BoundaryScanState, event: str) -> None:
        self.state = state
        self._last_event = event
        self._state_elapsed_s = 0.0

    def _sector_range(self, ranges: list[float], center_rad: float, half_rad: float) -> float:
        """Median range of beams in the angular sector [center−half, center+half]."""
        return self._sector_range_pct(ranges, center_rad, half_rad, 0.5)

    def _sector_range_pct(
        self, ranges: list[float], center_rad: float, half_rad: float, pct: float
    ) -> float:
        """pct-th percentile range of beams in [center−half, center+half].
        Use pct=0.5 for median (closest dense wall), pct=0.9 to see through
        sparse obstacles like the net and reach the far fence."""
        lo = center_rad - half_rad
        hi = center_rad + half_rad
        values: list[float] = []
        for i, r in enumerate(ranges):
            if not math.isfinite(r) or r < self.config.lidar_min_range_m or r > self.config.lidar_max_range_m:
                continue
            angle = self._lidar_angle_min + i * self._lidar_angle_increment
            angle = (angle + math.pi) % (2.0 * math.pi) - math.pi  # normalise to (−π, π]
            if lo <= angle <= hi:
                values.append(r)
        if not values:
            return math.inf
        values.sort()
        idx = min(len(values) - 1, max(0, int(round((len(values) - 1) * pct))))
        return values[idx]

    def _update_distance(self, x_m: float, y_m: float) -> None:
        if self._last_pose is None:
            self._last_pose = (x_m, y_m)
            return
        step = math.hypot(x_m - self._last_pose[0], y_m - self._last_pose[1])
        if 0.001 <= step <= 1.0:
            self._distance_traveled_m += step
        self._last_pose = (x_m, y_m)


# ─── Obstacle-first survey ────────────────────────────────────────────────────


class ObstacleSurveyState(str, Enum):
    SCAN_IN_PLACE = "scan_in_place"
    APPROACH = "approach_obstacle"
    DONE = "done"


@dataclass(frozen=True)
class ObstacleSurveyConfig:
    drive_speed_m_s: float = 0.50
    turn_speed_rad_s: float = 0.80
    safety_stop_range_m: float = 0.35
    safety_slow_range_m: float = 0.75
    # Stop this far from the obstacle (after approach).
    stop_at_range_m: float = 0.50
    # If the forward sector reads closer than this during the initial scan,
    # the robot is already adjacent to a boundary — skip the approach entirely.
    nearby_threshold_m: float = 2.00
    lidar_min_range_m: float = 0.30
    lidar_max_range_m: float = 12.0
    scan_min_count: int = 3
    scan_timeout_s: float = 8.0
    approach_timeout_s: float = 180.0
    heading_kp: float = 1.5
    heading_tolerance_rad: float = math.radians(8.0)
    # Fraction of infinite beams in the front sector that signals a net.
    net_sparse_threshold: float = 0.50
    front_sector_half_deg: float = 30.0
    # Angular span threshold: wider than this → "fence", narrower → "post".
    wide_obstacle_span_deg: float = 60.0
    output_file: Path = DEFAULT_BOUNDARY_FILE

    @classmethod
    def from_env(cls) -> "ObstacleSurveyConfig":
        d = cls()
        return cls(
            drive_speed_m_s=_env_float("OBSTACLE_SURVEY_DRIVE_SPEED_M_S", d.drive_speed_m_s),
            turn_speed_rad_s=_env_float("OBSTACLE_SURVEY_TURN_SPEED_RAD_S", d.turn_speed_rad_s),
            safety_stop_range_m=_env_float("OBSTACLE_SURVEY_SAFETY_STOP_M", d.safety_stop_range_m),
            safety_slow_range_m=_env_float("OBSTACLE_SURVEY_SAFETY_SLOW_M", d.safety_slow_range_m),
            stop_at_range_m=_env_float("OBSTACLE_SURVEY_STOP_AT_RANGE_M", d.stop_at_range_m),
            nearby_threshold_m=_env_float("OBSTACLE_SURVEY_NEARBY_THRESHOLD_M", d.nearby_threshold_m),
            lidar_min_range_m=_env_float("OBSTACLE_SURVEY_LIDAR_MIN_M", d.lidar_min_range_m),
            lidar_max_range_m=_env_float("OBSTACLE_SURVEY_LIDAR_MAX_M", d.lidar_max_range_m),
            scan_min_count=int(_env_float("OBSTACLE_SURVEY_SCAN_MIN_COUNT", float(d.scan_min_count))),
            scan_timeout_s=_env_float("OBSTACLE_SURVEY_SCAN_TIMEOUT_S", d.scan_timeout_s),
            approach_timeout_s=_env_float("OBSTACLE_SURVEY_APPROACH_TIMEOUT_S", d.approach_timeout_s),
            heading_kp=_env_float("OBSTACLE_SURVEY_HEADING_KP", d.heading_kp),
            heading_tolerance_rad=_env_float("OBSTACLE_SURVEY_HEADING_TOL_RAD", d.heading_tolerance_rad),
            net_sparse_threshold=_env_float("OBSTACLE_SURVEY_NET_SPARSE_THRESHOLD", d.net_sparse_threshold),
            front_sector_half_deg=_env_float("OBSTACLE_SURVEY_FRONT_SECTOR_HALF_DEG", d.front_sector_half_deg),
            wide_obstacle_span_deg=_env_float("OBSTACLE_SURVEY_WIDE_SPAN_DEG", d.wide_obstacle_span_deg),
            output_file=Path(os.getenv("SURVEY_OUTPUT_FILE", str(d.output_file))),
        )


@dataclass(frozen=True)
class ObstacleSurveyCommand:
    state: ObstacleSurveyState
    base: BaseCommand
    front_range_m: float
    obstacle_type: str | None


class ObstacleSurvey:
    """Find and stop at the nearest court boundary or obstacle.

    SCAN_IN_PLACE: Accumulate ≥scan_min_count static LiDAR scans.  If any beam
    in any direction is closer than nearby_threshold_m, the robot is already
    adjacent to something — classify and stop immediately.

    APPROACH: Nothing nearby.  Drive forward at low speed; on every tick run
    LiDAR + visual recognition.  Stop when front range ≤ stop_at_range_m and
    classify the obstacle.

    Classification priority:
      1. vision.obstacle_class ("net", "fence", "post", …) — highest confidence
      2. LiDAR sparsity in front sector: ≥net_sparse_threshold fraction of
         infinite returns → "net"
      3. Angular span of near front returns: ≥wide_obstacle_span_deg → "fence",
         smaller → "post"
      4. Fallback → "unknown"

    Writes court_boundary.json with obstacle_type, distance_m, pose, confidence.
    """

    def __init__(self, config: ObstacleSurveyConfig | None = None) -> None:
        self.config = config or ObstacleSurveyConfig()
        self.state = ObstacleSurveyState.SCAN_IN_PLACE
        self._scan_count = 0
        self._state_elapsed_s = 0.0
        self._started_at: float | None = None
        self._last_event = "none"
        self._failure_reason: str | None = None
        self._current_ranges: list[float] = []
        self._current_front_range_m: float = math.inf
        self._target_world_heading: float | None = None  # set when obstacle found in scan
        self._last_pose: tuple[float, float] | None = None
        self._distance_traveled_m = 0.0
        self._obstacle_type: str | None = None
        self._obstacle_distance_m: float | None = None
        self._obstacle_confidence: float = 0.0
        self._pose_at_stop: tuple[float, float] | None = None
        self._obstacle_world_x: float | None = None
        self._obstacle_world_y: float | None = None
        self._lidar_angle_min: float = -math.pi
        self._lidar_angle_increment: float = 2.0 * math.pi / 360
        self._result: dict | None = None
        self._debug_log: list[dict] = []   # rotating, max 80 entries
        self._last_cmd: tuple[float, float] = (0.0, 0.0)
        self._court_line_range_m: float | None = None  # front range when court line first detected
        self._approach_bearing_rad: float | None = None  # world heading toward obstacle
        self._pre_turn_front_range_m: float = math.inf  # front range BEFORE the 180° turn

    @classmethod
    def from_env(cls) -> "ObstacleSurvey":
        return cls(ObstacleSurveyConfig.from_env())

    def reset(self) -> None:
        self.__init__(self.config)

    @property
    def result(self) -> dict | None:
        return self._result

    def update(
        self,
        x_m: float,
        y_m: float,
        yaw_rad: float,
        lidar_ranges: list[float] | None,
        dt_s: float,
        vision: "SurveyVision | None" = None,
        lidar_angle_min: float = -math.pi,
        lidar_angle_increment: float | None = None,
    ) -> ObstacleSurveyCommand:
        if self._started_at is None:
            self._started_at = time.time()
        if self.state == ObstacleSurveyState.DONE:
            return ObstacleSurveyCommand(
                self.state, BaseCommand(0.0, 0.0), self._current_front_range_m, self._obstacle_type
            )

        self._state_elapsed_s += max(0.0, dt_s)
        self._update_distance(x_m, y_m)

        if lidar_ranges:
            n = len(lidar_ranges)
            self._lidar_angle_min = lidar_angle_min
            self._lidar_angle_increment = (
                lidar_angle_increment if lidar_angle_increment is not None
                else 2.0 * math.pi / max(1, n)
            )
            self._current_front_range_m = self._sector_min(
                lidar_ranges, 0.0, math.radians(self.config.front_sector_half_deg)
            )
            self._current_ranges = lidar_ranges
            self._scan_count += 1

        cmd = self._step(x_m, y_m, yaw_rad, vision)
        safe_cmd = self._apply_safety(cmd)
        self._last_cmd = (safe_cmd.linear_speed_m_s, safe_cmd.angular_speed_rad_s)
        self._append_debug(yaw_rad, vision)
        return ObstacleSurveyCommand(
            self.state,
            safe_cmd,
            self._current_front_range_m,
            self._obstacle_type,
        )

    def telemetry(self) -> dict:
        elapsed = 0.0 if self._started_at is None else time.time() - self._started_at
        return {
            "state": self.state.value,
            "survey_type": "obstacle_survey",
            "last_event": self._last_event,
            "failure_reason": self._failure_reason,
            "scan_count": self._scan_count,
            "front_range_m": (
                None if math.isinf(self._current_front_range_m)
                else round(self._current_front_range_m, 3)
            ),
            "obstacle_type": self._obstacle_type,
            "obstacle_distance_m": self._obstacle_distance_m,
            "obstacle_confidence": self._obstacle_confidence,
            "distance_traveled_m": round(self._distance_traveled_m, 2),
            "elapsed_s": round(elapsed, 1),
            "result": self._result,
            "debug_log": list(self._debug_log),
        }

    # ── state machine ─────────────────────────────────────────────────────────

    def _step(
        self, x_m: float, y_m: float, yaw_rad: float, vision: "SurveyVision | None"
    ) -> BaseCommand:
        if self.state == ObstacleSurveyState.SCAN_IN_PLACE:
            if self._state_elapsed_s >= self.config.scan_timeout_s:
                self._finish(x_m, y_m, vision, "SCAN_TIMEOUT")
                return BaseCommand(0.0, 0.0)
            if self._scan_count < self.config.scan_min_count:
                return BaseCommand(0.0, 0.0)
            bearing, nearest = self._nearest_any_direction()
            if not math.isinf(nearest) and nearest <= self.config.nearby_threshold_m:
                # Already at a boundary — classify and stop.
                otype, conf = self._classify(vision)
                self._record_obstacle(otype, nearest, conf, x_m, y_m)
                self._finish(x_m, y_m, vision, None)
                return BaseCommand(0.0, 0.0)
            if not math.isinf(nearest):
                # Obstacle found somewhere in 360° — steer toward it.
                self._pre_turn_front_range_m = self._current_front_range_m
                self._target_world_heading = yaw_rad + bearing
                self._approach_bearing_rad = self._target_world_heading
                self._enter(ObstacleSurveyState.APPROACH, "boundary_found_navigating_toward_it")
            else:
                # Nothing detected — drive straight forward.
                self._target_world_heading = None
                self._approach_bearing_rad = yaw_rad
                self._enter(ObstacleSurveyState.APPROACH, "no_boundary_found_driving_forward")
            return BaseCommand(0.0, 0.0)

        if self.state == ObstacleSurveyState.APPROACH:
            if self._state_elapsed_s >= self.config.approach_timeout_s:
                self._finish(x_m, y_m, vision, "APPROACH_TIMEOUT")
                return BaseCommand(0.0, 0.0)
            # Only accept vision when heading is aligned — prevents firing while
            # the robot is still rotating to face the target direction.
            heading_aligned = self._is_heading_aligned(yaw_rad)
            front = self._current_front_range_m
            # All stops (vision + LiDAR range) require heading alignment first.
            # This prevents premature stop while the robot is still rotating,
            # because during a turn the front sector sweeps over obstacles that
            # are not actually in the travel direction.
            if heading_aligned:
                # Record the front range the first time the court line is seen.
                # This distance = baseline-to-fence gap (how far the fence is
                # from where the robot was when it crossed the baseline).
                if (
                    self._court_line_range_m is None
                    and vision is not None
                    and vision.line_detected
                    and not math.isinf(front)
                ):
                    self._court_line_range_m = round(front, 3)
                # Primary: vision recognised the obstacle.
                if vision is not None and vision.obstacle_class:
                    dist = front if not math.isinf(front) else None
                    otype, conf = self._classify(vision)
                    self._record_obstacle(otype, dist, conf, x_m, y_m)
                    self._finish(x_m, y_m, vision, None)
                    return BaseCommand(0.0, 0.0)
                # LiDAR range stop (no vision needed).
                if not math.isinf(front) and front <= self.config.stop_at_range_m:
                    otype, conf = self._classify(vision)
                    self._record_obstacle(otype, front, conf, x_m, y_m)
                    self._finish(x_m, y_m, vision, None)
                    return BaseCommand(0.0, 0.0)
            # Safety absolute stop — always active, even during rotation.
            if not math.isinf(front) and front <= self.config.safety_stop_range_m:
                otype, conf = self._classify(vision)
                self._record_obstacle(otype, front, conf, x_m, y_m)
                self._finish(x_m, y_m, vision, None)
                return BaseCommand(0.0, 0.0)
            return self._drive_to_heading(yaw_rad)

        return BaseCommand(0.0, 0.0)

    # ── classification ────────────────────────────────────────────────────────

    def _classify(self, vision: "SurveyVision | None") -> tuple[str, float]:
        if vision is not None and vision.obstacle_class:
            label = vision.obstacle_class.strip().lower()
            if label in {"net", "net_post", "post", "posts"}:
                return "net", 0.90
            if label in {"fence", "wall", "enclosure", "boundary"}:
                return "fence", 0.90
        sparsity = self._front_sparsity()
        if sparsity >= self.config.net_sparse_threshold:
            return "net", round(min(0.90, 0.50 + sparsity * 0.40), 2)
        span_deg = self._front_obstacle_span_deg()
        if span_deg >= self.config.wide_obstacle_span_deg:
            return "fence", 0.70
        if span_deg > 0.0:
            return "post", 0.60
        return "unknown", 0.30

    def _front_sparsity(self) -> float:
        """Fraction of beams in the front sector that are infinite (net-like)."""
        if not self._current_ranges:
            return 0.0
        half = math.radians(self.config.front_sector_half_deg)
        total = inf_count = 0
        for i, r in enumerate(self._current_ranges):
            angle = self._lidar_angle_min + i * self._lidar_angle_increment
            angle = (angle + math.pi) % (2.0 * math.pi) - math.pi
            if -half <= angle <= half:
                total += 1
                if not math.isfinite(r) or r > self.config.lidar_max_range_m:
                    inf_count += 1
        return inf_count / total if total > 0 else 0.0

    def _front_obstacle_span_deg(self) -> float:
        """Angular extent (degrees) of near valid returns in the front sector."""
        if not self._current_ranges:
            return 0.0
        half = math.radians(self.config.front_sector_half_deg)
        near_limit = self.config.stop_at_range_m * 2.0
        angles: list[float] = []
        for i, r in enumerate(self._current_ranges):
            if not math.isfinite(r) or r < self.config.lidar_min_range_m or r > near_limit:
                continue
            angle = self._lidar_angle_min + i * self._lidar_angle_increment
            angle = (angle + math.pi) % (2.0 * math.pi) - math.pi
            if -half <= angle <= half:
                angles.append(angle)
        if len(angles) < 2:
            return 0.0
        return math.degrees(max(angles) - min(angles))

    def _nearest_any_direction(self) -> tuple[float, float]:
        """(bearing_rad, range_m) of the nearest valid obstacle across 360°.

        bearing_rad is in the robot frame (0 = forward).  Returns (0.0, inf)
        when no valid beam is found.
        """
        if not self._current_ranges:
            return (0.0, math.inf)
        best_r = math.inf
        best_angle = 0.0
        for i, r in enumerate(self._current_ranges):
            if not math.isfinite(r) or r < self.config.lidar_min_range_m or r > self.config.lidar_max_range_m:
                continue
            if r < best_r:
                best_r = r
                angle = self._lidar_angle_min + i * self._lidar_angle_increment
                best_angle = (angle + math.pi) % (2.0 * math.pi) - math.pi
        return (best_angle, best_r)

    def _sector_min(self, ranges: list[float], center_rad: float, half_rad: float) -> float:
        """Minimum valid range in the angular sector [center-half, center+half]."""
        lo = center_rad - half_rad
        hi = center_rad + half_rad
        values: list[float] = []
        for i, r in enumerate(ranges):
            if not math.isfinite(r) or r < self.config.lidar_min_range_m or r > self.config.lidar_max_range_m:
                continue
            angle = self._lidar_angle_min + i * self._lidar_angle_increment
            angle = (angle + math.pi) % (2.0 * math.pi) - math.pi
            if lo <= angle <= hi:
                values.append(r)
        return min(values) if values else math.inf

    def _is_heading_aligned(self, yaw_rad: float) -> bool:
        """True when the robot has completed its turn toward the target heading.

        Uses dual verification:
        1. Odometry yaw error within tolerance (primary).
        2. LiDAR cross-check: if a 180° turn was requested, verify that the
           front range has changed significantly from before the turn (the
           original obstacle is now in the rear, a new one is in front).
           This catches odometry drift during large turns.
        """
        if self._target_world_heading is None:
            return True
        err = (self._target_world_heading - yaw_rad + math.pi) % (2.0 * math.pi) - math.pi
        odom_aligned = abs(err) <= self.config.heading_tolerance_rad * 2
        if not odom_aligned:
            return False
        # For a large turn (≥120°), also verify via LiDAR that the obstacle
        # previously in front is no longer in the front sector.
        if self._pre_turn_front_range_m < math.inf and not math.isinf(self._current_front_range_m):
            original_was_close = self._pre_turn_front_range_m < 5.0
            now_front_changed = abs(self._current_front_range_m - self._pre_turn_front_range_m) > 1.5
            if original_was_close and not now_front_changed:
                return False  # front range unchanged — still facing same direction
        return True

    def _drive_to_heading(self, yaw_rad: float) -> BaseCommand:
        """Steer toward _target_world_heading if set, otherwise drive straight.

        Large error (> tolerance): rotate in place at full turn speed.
        Small error (≤ tolerance): drive forward with proportional correction
        so the residual angle is eliminated while moving.
        """
        if self._target_world_heading is None:
            return BaseCommand(self.config.drive_speed_m_s, 0.0)
        err = (self._target_world_heading - yaw_rad + math.pi) % (2.0 * math.pi) - math.pi
        if abs(err) > self.config.heading_tolerance_rad:
            # Rotate in place until roughly aligned.
            turn = math.copysign(self.config.turn_speed_rad_s, err)
            return BaseCommand(0.0, turn)
        # Within tolerance: drive forward and keep correcting the residual angle.
        turn = max(-self.config.turn_speed_rad_s * 0.5,
                   min(self.config.turn_speed_rad_s * 0.5,
                       err * self.config.heading_kp))
        return BaseCommand(self.config.drive_speed_m_s, turn)

    def _append_debug(self, yaw_rad: float, vision: "SurveyVision | None") -> None:
        elapsed = 0.0 if self._started_at is None else time.time() - self._started_at
        bearing, nearest = self._nearest_any_direction()
        target_deg = (
            None if self._target_world_heading is None
            else round(math.degrees(self._target_world_heading) % 360, 1)
        )
        heading_err = None
        if self._target_world_heading is not None:
            e = (self._target_world_heading - yaw_rad + math.pi) % (2.0 * math.pi) - math.pi
            heading_err = round(math.degrees(e), 1)

        decision = self._last_event
        lin, ang = self._last_cmd
        if lin > 0:
            decision = "driving" if ang == 0.0 else "driving+steering"
        elif ang != 0.0:
            decision = "rotating_to_face_target"
        elif self.state == ObstacleSurveyState.SCAN_IN_PLACE:
            decision = "scanning"

        entry = {
            "t": round(elapsed, 2),
            "state": self.state.value,
            "decision": decision,
            "front_m": None if math.isinf(self._current_front_range_m) else round(self._current_front_range_m, 2),
            "nearest_m": None if math.isinf(nearest) else round(nearest, 2),
            "bearing_deg": round(math.degrees(bearing), 1),
            "target_heading_deg": target_deg,
            "heading_err_deg": heading_err,
            "vision": vision.obstacle_class if vision else None,
            "cmd_lin": round(lin, 3),
            "cmd_ang": round(ang, 3),
            "scans": self._scan_count,
        }
        self._debug_log.append(entry)
        if len(self._debug_log) > 80:
            del self._debug_log[:len(self._debug_log) - 80]

    # ── helpers ───────────────────────────────────────────────────────────────

    def _record_obstacle(
        self, otype: str, dist: float | None, conf: float, x_m: float, y_m: float
    ) -> None:
        self._obstacle_type = otype
        self._obstacle_distance_m = round(dist, 3) if dist is not None else None
        self._obstacle_confidence = conf
        self._pose_at_stop = (x_m, y_m)
        if dist is not None and math.isfinite(dist) and self._approach_bearing_rad is not None:
            self._obstacle_world_x = x_m + math.cos(self._approach_bearing_rad) * dist
            self._obstacle_world_y = y_m + math.sin(self._approach_bearing_rad) * dist

    def _finish(
        self, x_m: float, y_m: float, vision: "SurveyVision | None", failure: str | None
    ) -> None:
        self._failure_reason = failure
        status = "FAILED" if failure else "SUCCESS"
        elapsed = 0.0 if self._started_at is None else time.time() - self._started_at
        result = {
            "survey_type": "obstacle_survey",
            "status": status,
            "failure_reason": failure,
            "survey_complete": status == "SUCCESS",
            "surveyed_at": time.time(),
            "obstacle_type": self._obstacle_type,
            "obstacle_distance_m": self._obstacle_distance_m,
            "obstacle_confidence": self._obstacle_confidence,
            "robot_pose_at_stop": (
                {"x_m": round(self._pose_at_stop[0], 3), "y_m": round(self._pose_at_stop[1], 3)}
                if self._pose_at_stop else None
            ),
            "obstacle_world_pos": (
                {"x_m": round(self._obstacle_world_x, 3), "y_m": round(self._obstacle_world_y, 3)}
                if self._obstacle_world_x is not None else None
            ),
            "vision_class": vision.obstacle_class if vision else None,
            "line_to_fence_m": self._court_line_range_m,
            "distance_traveled_m": round(self._distance_traveled_m, 2),
            "elapsed_s": round(elapsed, 1),
        }
        self._result = result
        self.config.output_file.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.config.output_file.with_suffix(".tmp.json")
        tmp.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        tmp.replace(self.config.output_file)
        self._enter(ObstacleSurveyState.DONE, f"obstacle_survey_{status.lower()}")

    def _drive_straight(self, yaw_rad: float) -> BaseCommand:
        if self._approach_yaw is None:
            return BaseCommand(self.config.drive_speed_m_s, 0.0)
        err = (self._approach_yaw - yaw_rad + math.pi) % (2.0 * math.pi) - math.pi
        turn = max(
            -self.config.turn_speed_rad_s,
            min(self.config.turn_speed_rad_s, err * self.config.heading_kp),
        )
        return BaseCommand(self.config.drive_speed_m_s, turn)

    def _apply_safety(self, command: BaseCommand) -> BaseCommand:
        if command.linear_speed_m_s <= 0.0:
            return command
        if self._current_front_range_m <= self.config.safety_stop_range_m:
            return BaseCommand(0.0, 0.0)
        if self._current_front_range_m <= self.config.safety_slow_range_m:
            return BaseCommand(command.linear_speed_m_s * 0.35, command.angular_speed_rad_s)
        return command

    def _enter(self, state: ObstacleSurveyState, event: str) -> None:
        self.state = state
        self._last_event = event
        self._state_elapsed_s = 0.0

    def _update_distance(self, x_m: float, y_m: float) -> None:
        if self._last_pose is None:
            self._last_pose = (x_m, y_m)
            return
        step = math.hypot(x_m - self._last_pose[0], y_m - self._last_pose[1])
        if 0.001 <= step <= 1.0:
            self._distance_traveled_m += step
        self._last_pose = (x_m, y_m)
