"""Runtime-agnostic court-line survey state machine.

The survey path is driven by camera observations of the painted outer court
line. LiDAR is used for safety and to enrich the survey result with fences,
walls, free space, and internal objects. It must not become the perimeter
driver.
"""

from __future__ import annotations

import json
import math
import os
import time
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path

from collector import BaseCommand
from config_utils import _env_float
from navigator import Navigator, navigator_from_survey_config


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_BOUNDARY_FILE = PROJECT_ROOT / "runtime" / "court_boundary.json"
_VENDORS_FILE = PROJECT_ROOT / "runtime" / "vendors.json"



class SurveyState(str, Enum):
    FIND_COURT_LINE = "find_court_line"
    ALIGN_OUTSIDE_LINE = "align_outside_line"
    FOLLOW_LINE_WITH_OFFSET = "follow_line_with_offset"
    DETECT_CORNER = "detect_corner"
    TURN_AT_CORNER = "turn_at_corner"
    MAP_EXTERNAL_BOUNDARY = "map_external_boundary"
    COMPLETE_LOOP = "complete_loop"
    VALIDATE_SURVEY = "validate_survey"
    RECOVER_LINE = "recover_line"
    DONE = "done"


@dataclass(frozen=True)
class SurveyConfig:
    drive_speed_m_s: float = 0.35
    turn_speed_rad_s: float = 1.0
    search_turn_speed_rad_s: float = 0.45
    recovery_turn_speed_rad_s: float = 0.35
    line_offset_m: float = 0.50
    line_offset_tolerance_m: float = 0.10
    line_heading_tolerance_deg: float = 7.0
    line_heading_gain: float = 1.6
    line_offset_gain: float = 0.9
    corner_min_spacing_m: float = 3.0
    safety_stop_range_m: float = 0.32
    safety_slow_range_m: float = 0.65
    min_fence_range_m: float = 0.35
    max_fence_range_m: float = 12.0
    external_boundary_min_distance_from_line_m: float = 1.0
    turn_angle_deg: float = 90.0
    phase_timeout_s: float = 90.0
    total_timeout_s: float = 540.0
    min_loop_distance_m: float = 45.0
    loop_closure_radius_m: float = 1.35
    expected_court_length_m: float = 23.77
    expected_court_width_m: float = 10.97
    lidar_local_x_m: float = -0.20
    lidar_local_y_m: float = 0.0
    lidar_subsample: int = 8

    @classmethod
    def from_env(cls) -> "SurveyConfig":
        d = cls()
        return cls(
            drive_speed_m_s=_env_float("SURVEY_DRIVE_SPEED_M_S", d.drive_speed_m_s),
            turn_speed_rad_s=_env_float("SURVEY_TURN_SPEED_RAD_S", d.turn_speed_rad_s),
            search_turn_speed_rad_s=_env_float("SURVEY_SEARCH_TURN_SPEED_RAD_S", d.search_turn_speed_rad_s),
            recovery_turn_speed_rad_s=_env_float("SURVEY_RECOVERY_TURN_SPEED_RAD_S", d.recovery_turn_speed_rad_s),
            line_offset_m=_env_float("SURVEY_LINE_OFFSET_M", d.line_offset_m),
            line_offset_tolerance_m=_env_float("SURVEY_LINE_OFFSET_TOLERANCE_M", d.line_offset_tolerance_m),
            line_heading_tolerance_deg=_env_float(
                "SURVEY_LINE_HEADING_TOLERANCE_DEG",
                d.line_heading_tolerance_deg,
            ),
            line_heading_gain=_env_float("SURVEY_LINE_HEADING_GAIN", d.line_heading_gain),
            line_offset_gain=_env_float("SURVEY_LINE_OFFSET_GAIN", d.line_offset_gain),
            corner_min_spacing_m=_env_float("SURVEY_CORNER_MIN_SPACING_M", d.corner_min_spacing_m),
            safety_stop_range_m=_env_float("SURVEY_SAFETY_STOP_RANGE_M", d.safety_stop_range_m),
            safety_slow_range_m=_env_float("SURVEY_SAFETY_SLOW_RANGE_M", d.safety_slow_range_m),
            min_fence_range_m=_env_float("SURVEY_MIN_FENCE_RANGE_M", d.min_fence_range_m),
            max_fence_range_m=_env_float("SURVEY_MAX_FENCE_RANGE_M", d.max_fence_range_m),
            external_boundary_min_distance_from_line_m=_env_float(
                "SURVEY_EXTERNAL_BOUNDARY_MIN_DISTANCE_FROM_LINE_M",
                d.external_boundary_min_distance_from_line_m,
            ),
            turn_angle_deg=_env_float("SURVEY_TURN_ANGLE_DEG", d.turn_angle_deg),
            phase_timeout_s=_env_float("SURVEY_PHASE_TIMEOUT_S", d.phase_timeout_s),
            total_timeout_s=_env_float("SURVEY_TOTAL_TIMEOUT_S", d.total_timeout_s),
            min_loop_distance_m=_env_float("SURVEY_MIN_LOOP_DISTANCE_M", d.min_loop_distance_m),
            loop_closure_radius_m=_env_float("SURVEY_LOOP_CLOSURE_RADIUS_M", d.loop_closure_radius_m),
            expected_court_length_m=_env_float("SURVEY_EXPECTED_COURT_LENGTH_M", d.expected_court_length_m),
            expected_court_width_m=_env_float("SURVEY_EXPECTED_COURT_WIDTH_M", d.expected_court_width_m),
            lidar_local_x_m=_env_float("SURVEY_LIDAR_LOCAL_X_M", d.lidar_local_x_m),
            lidar_local_y_m=_env_float("SURVEY_LIDAR_LOCAL_Y_M", d.lidar_local_y_m),
            lidar_subsample=int(_env_float("SURVEY_LIDAR_SUBSAMPLE", float(d.lidar_subsample))),
        )


@dataclass(frozen=True)
class SurveyVision:
    """Camera-derived court geometry plus optional depth/object summary."""

    center_m: float | None = None
    left_m: float | None = None
    right_m: float | None = None
    valid_count: int = 0
    obstacle_class: str | None = None
    line_detected: bool = False
    line_offset_m: float | None = None
    line_heading_error_rad: float | None = None
    line_confidence: float = 0.0
    corner_detected: bool = False
    corner_confidence: float = 0.0


@dataclass(frozen=True)
class SurveyCommand:
    state: SurveyState
    base: BaseCommand
    sample_count: int
    vision: SurveyVision | None = None


@dataclass
class LidarObject:
    label: str
    classification: str
    x_m: float
    y_m: float
    distance_to_court_line_m: float | None = None
    confidence: float = 0.6


@dataclass
class LidarSurveyMap:
    external_boundary_candidates: list[LidarObject] = field(default_factory=list)
    internal_obstacles: list[LidarObject] = field(default_factory=list)
    raw_points: list[tuple[float, float]] = field(default_factory=list)

    def point_sample(self, limit: int = 800) -> list[dict[str, float]]:
        if not self.raw_points:
            return []
        stride = max(1, len(self.raw_points) // limit)
        return [
            {"x_m": round(x, 3), "y_m": round(y, 3)}
            for x, y in self.raw_points[-limit * stride::stride]
        ][-limit:]


class LidarObstacleClassifier:
    """Classifies LiDAR returns without allowing the net to define the perimeter."""

    def __init__(self, config: SurveyConfig) -> None:
        self.config = config

    def classify(
        self,
        world_x: float,
        world_y: float,
        line_distance_m: float | None,
        vision_class: str | None,
    ) -> LidarObject:
        label = (vision_class or "").strip().lower()
        if label == "net":
            return LidarObject("net", "internal_obstacle", world_x, world_y, line_distance_m, 0.95)
        if label in {"post", "net_post", "posts"}:
            return LidarObject("post", "internal_obstacle", world_x, world_y, line_distance_m, 0.85)
        if label in {"fence", "wall", "enclosure"}:
            return LidarObject(label, "external_boundary_candidate", world_x, world_y, line_distance_m, 0.9)
        if line_distance_m is not None and line_distance_m >= self.config.external_boundary_min_distance_from_line_m:
            return LidarObject("fence_or_wall", "external_boundary_candidate", world_x, world_y, line_distance_m, 0.55)
        return LidarObject("temporary_object", "internal_obstacle", world_x, world_y, line_distance_m, 0.5)


class LidarBoundaryMapper:
    def __init__(self, config: SurveyConfig) -> None:
        self.config = config
        self.map = LidarSurveyMap()
        self.classifier = LidarObstacleClassifier(config)

    def update(
        self,
        ranges: list[float] | None,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        line_offset_m: float | None,
        vision_class: str | None,
    ) -> dict:
        if not ranges or len(ranges) < 10:
            return {"hit_count": 0}
        n = len(ranges)
        cos_y = math.cos(robot_yaw)
        sin_y = math.sin(robot_yaw)
        local_xs: list[float] = []
        local_ys: list[float] = []
        hit_count = 0
        for i in range(0, n, self.config.lidar_subsample):
            r = ranges[i]
            if not math.isfinite(r) or r < self.config.min_fence_range_m or r > self.config.max_fence_range_m:
                continue
            angle = (i / n) * 2.0 * math.pi - math.pi
            lx = self.config.lidar_local_x_m + r * math.cos(angle)
            ly = self.config.lidar_local_y_m + r * math.sin(angle)
            wx = robot_x + cos_y * lx - sin_y * ly
            wy = robot_y + sin_y * lx + cos_y * ly
            local_xs.append(lx)
            local_ys.append(ly)
            self.map.raw_points.append((wx, wy))
            if hit_count % 6 == 0:
                line_distance = None if line_offset_m is None else max(0.0, abs(ly) - abs(line_offset_m))
                obj = self.classifier.classify(wx, wy, line_distance, vision_class)
                if obj.classification == "external_boundary_candidate":
                    self.map.external_boundary_candidates.append(obj)
                else:
                    self.map.internal_obstacles.append(obj)
            hit_count += 1
        if not local_xs:
            return {"hit_count": 0}
        return {
            "hit_count": hit_count,
            "local_extents": {
                "min_x_m": round(min(local_xs), 3),
                "max_x_m": round(max(local_xs), 3),
                "min_y_m": round(min(local_ys), 3),
                "max_y_m": round(max(local_ys), 3),
            },
        }

    def result(self) -> dict:
        external = self._dedupe(self.map.external_boundary_candidates)
        internal = self._dedupe(self.map.internal_obstacles)
        fence_distances = [
            obj.distance_to_court_line_m
            for obj in external
            if obj.distance_to_court_line_m is not None and obj.distance_to_court_line_m > 0.0
        ]
        return {
            "external_boundary_candidates": [self._object_dict(obj) for obj in external],
            "internal_obstacles": [self._object_dict(obj) for obj in internal],
            "free_space_between_court_lines_and_fences": {
                "min_m": round(min(fence_distances), 3) if fence_distances else None,
                "avg_m": round(sum(fence_distances) / len(fence_distances), 3) if fence_distances else None,
                "sample_count": len(fence_distances),
            },
            "point_cloud_sample": self.map.point_sample(1200),
            "point_count": len(self.map.raw_points),
        }

    def _dedupe(self, objects: list[LidarObject], limit: int = 80) -> list[LidarObject]:
        buckets: dict[tuple[str, int, int], LidarObject] = {}
        for obj in objects:
            key = (obj.label, round(obj.x_m * 2), round(obj.y_m * 2))
            if key not in buckets or obj.confidence > buckets[key].confidence:
                buckets[key] = obj
        return list(buckets.values())[-limit:]

    @staticmethod
    def _object_dict(obj: LidarObject) -> dict:
        return {
            "label": obj.label,
            "classification": obj.classification,
            "x_m": round(obj.x_m, 3),
            "y_m": round(obj.y_m, 3),
            "distance_to_court_line_m": (
                None if obj.distance_to_court_line_m is None else round(obj.distance_to_court_line_m, 3)
            ),
            "confidence": round(obj.confidence, 2),
        }


class CourtSurveyBehavior:
    """Camera-led outer-court-line survey FSM."""

    def __init__(
        self,
        config: SurveyConfig | None = None,
        output_path: Path = DEFAULT_BOUNDARY_FILE,
    ) -> None:
        self.config = config or SurveyConfig()
        self._nav = navigator_from_survey_config(self.config)
        self.output_path = output_path
        self.state = SurveyState.FIND_COURT_LINE
        self.sample_count = 0
        self._started_at: float | None = None
        self._phase_started_at: float | None = None
        self._initialized = False
        self._mapper = LidarBoundaryMapper(self.config)
        self._court_bounds: dict | None = None
        self._last_vision: SurveyVision | None = None
        self._front_range_m = math.inf
        self._left_range_m: float | None = None
        self._right_range_m: float | None = None
        self._scan_coverage: dict = {}
        self._last_event = "none"
        self._failure_reason: str | None = None
        self._timed_out = False
        self._loop_closed = False
        self._start_pose: tuple[float, float] | None = None
        self._last_pose: tuple[float, float] | None = None
        self._phase_start_pose: tuple[float, float] | None = None
        self._distance_traveled_m = 0.0
        self._turn_start_yaw: float | None = None
        self._turn_last_yaw: float | None = None
        self._turn_accumulated_rad = 0.0
        self._corners: list[dict[str, float]] = []
        self._current_line_index = 0
        self._line_lost_count = 0
        self._phase_visit_counts: dict[SurveyState, int] = {}

    @classmethod
    def from_env(cls) -> "CourtSurveyBehavior":
        path = Path(os.getenv("SURVEY_OUTPUT_FILE", str(DEFAULT_BOUNDARY_FILE)))
        return cls(SurveyConfig.from_env(), path)

    def reset(self) -> None:
        self.__init__(self.config, self.output_path)

    @property
    def court_bounds(self) -> dict | None:
        return self._court_bounds

    def current_target(self) -> None:
        return None

    def telemetry(self) -> dict:
        elapsed = 0.0 if self._started_at is None else time.time() - self._started_at
        phase_elapsed = 0.0 if self._phase_started_at is None else time.time() - self._phase_started_at
        mapper = self._mapper.result()
        return {
            "state": self.state.value,
            "navigation_source": "camera_outer_court_line_fsm",
            "sensor_only_navigation": True,
            "mapping_pose_source": "platform_localization_estimate",
            "path_driver": "camera_court_line",
            "lidar_role": "obstacle_mapping_and_safety",
            "last_event": self._last_event,
            "failure_reason": self._failure_reason,
            "timed_out": self._timed_out,
            "front_lidar_range_m": None if math.isinf(self._front_range_m) else round(self._front_range_m, 3),
            "left_lidar_range_m": self._left_range_m,
            "right_lidar_range_m": self._right_range_m,
            "scan_coverage": self._scan_coverage,
            "line_detected": self._line_detected(),
            "line_offset_m": None if self._last_vision is None else self._last_vision.line_offset_m,
            "line_confidence": 0.0 if self._last_vision is None else self._last_vision.line_confidence,
            "corner_count": len(self._corners),
            "detected_corners": self._corners,
            "external_boundary_candidate_count": len(mapper["external_boundary_candidates"]),
            "internal_obstacle_count": len(mapper["internal_obstacles"]),
            "loop_closed": self._loop_closed,
            "distance_traveled_m": round(self._distance_traveled_m, 2),
            "phase_distance_m": round(self._phase_distance_m(), 2),
            "turn_progress_deg": round(math.degrees(self._turn_accumulated_rad), 1),
            "turn_target_deg": self.config.turn_angle_deg if self.state == SurveyState.TURN_AT_CORNER else None,
            "elapsed_s": round(elapsed, 1),
            "phase_elapsed_s": round(phase_elapsed, 1),
            "sample_count": self.sample_count,
        }

    def update(
        self,
        x_m: float,
        y_m: float,
        yaw_rad: float,
        lidar_ranges: list[float] | None,
        dt_s: float,  # noqa: ARG002 - behavior API compatibility
        vision: SurveyVision | None = None,
    ) -> SurveyCommand:
        now = time.time()
        if not self._initialized:
            self._initialized = True
            self._started_at = now
            self._phase_started_at = now
            self._start_pose = (x_m, y_m)
            self._last_pose = (x_m, y_m)
            self._enter(SurveyState.FIND_COURT_LINE, "map_court_started", x_m, y_m, yaw_rad)

        if self.state == SurveyState.DONE:
            return self._cmd(self._nav.stop(), vision)

        self._update_distance_traveled(x_m, y_m)
        self._update_sensors(lidar_ranges, vision, x_m, y_m, yaw_rad)

        if self._overall_timeout(now):
            self._timed_out = True
            self._fail("Timed out before camera-led court survey completed")
            return self._cmd(self._nav.stop(), vision)
        if self._phase_timeout(now):
            self._fail(f"Phase timeout in {self.state.value}; last_event={self._last_event}")
            return self._cmd(self._nav.stop(), vision)

        return self._cmd(self._apply_safety(self._step(x_m, y_m, yaw_rad)), vision)

    def _step(self, x_m: float, y_m: float, yaw_rad: float) -> BaseCommand:
        if self.state == SurveyState.FIND_COURT_LINE:
            if self._line_detected():
                self._enter(SurveyState.ALIGN_OUTSIDE_LINE, "court_line_detected", x_m, y_m, yaw_rad)
                return self._nav.stop()
            return self._nav.turn(self.config.search_turn_speed_rad_s)

        if self.state == SurveyState.ALIGN_OUTSIDE_LINE:
            if not self._line_detected():
                self._line_lost_count += 1
                if self._line_lost_count >= 5:
                    self._enter(SurveyState.RECOVER_LINE, "line_lost_during_align", x_m, y_m, yaw_rad)
                    return self._nav.stop()
                return self._nav.stop()
            else:
                self._line_lost_count = 0
            if self._line_aligned():
                self._enter(SurveyState.FOLLOW_LINE_WITH_OFFSET, "outside_offset_aligned", x_m, y_m, yaw_rad)
                return self._nav.stop()
            return self._line_follow_command(linear_scale=0.35)

        if self.state == SurveyState.FOLLOW_LINE_WITH_OFFSET:
            if not self._line_detected():
                self._line_lost_count += 1
                if self._line_lost_count >= 8:
                    self._enter(SurveyState.RECOVER_LINE, "line_lost", x_m, y_m, yaw_rad)
                    return self._nav.stop()
            else:
                self._line_lost_count = 0
            if self._corner_seen(x_m, y_m):
                self._enter(SurveyState.DETECT_CORNER, "court_corner_detected", x_m, y_m, yaw_rad)
                return self._nav.stop()
            if self._loop_reference_reached(x_m, y_m):
                self._enter(SurveyState.COMPLETE_LOOP, "loop_closed", x_m, y_m, yaw_rad)
                return self._nav.stop()
            return self._line_follow_command(linear_scale=1.0)

        if self.state == SurveyState.DETECT_CORNER:
            self._record_corner(x_m, y_m)
            self._enter(SurveyState.TURN_AT_CORNER, "corner_recorded", x_m, y_m, yaw_rad)
            return self._nav.stop()

        if self.state == SurveyState.TURN_AT_CORNER:
            if self._turn_complete(yaw_rad):
                self._current_line_index += 1
                self._enter(SurveyState.MAP_EXTERNAL_BOUNDARY, "corner_turn_complete", x_m, y_m, yaw_rad)
                return self._nav.stop()
            return self._nav.turn(self.config.turn_speed_rad_s)

        if self.state == SurveyState.MAP_EXTERNAL_BOUNDARY:
            next_state = SurveyState.FOLLOW_LINE_WITH_OFFSET if self._line_detected() else SurveyState.FIND_COURT_LINE
            self._enter(next_state, "external_boundary_mapped", x_m, y_m, yaw_rad)
            return self._nav.stop()

        if self.state == SurveyState.RECOVER_LINE:
            if self._line_detected():
                self._enter(SurveyState.ALIGN_OUTSIDE_LINE, "line_reacquired", x_m, y_m, yaw_rad)
                return self._nav.stop()
            return self._nav.turn(self.config.recovery_turn_speed_rad_s)

        if self.state == SurveyState.COMPLETE_LOOP:
            self._loop_closed = True
            self._enter(SurveyState.VALIDATE_SURVEY, "full_perimeter_followed", x_m, y_m, yaw_rad)
            return self._nav.stop()

        if self.state == SurveyState.VALIDATE_SURVEY:
            self._finish()
            return self._nav.stop()

        return self._nav.stop()

    def _line_follow_command(self, linear_scale: float) -> BaseCommand:
        heading_error = 0.0 if self._last_vision is None or self._last_vision.line_heading_error_rad is None else self._last_vision.line_heading_error_rad
        offset = self.config.line_offset_m if self._last_vision is None or self._last_vision.line_offset_m is None else self._last_vision.line_offset_m
        offset_error = offset - self.config.line_offset_m
        turn = heading_error * self.config.line_heading_gain + offset_error * self.config.line_offset_gain
        cmd = self._nav.drive(self.config.drive_speed_m_s * linear_scale, turn)
        return self._nav.clamp_turn(cmd, 0.75 * self.config.turn_speed_rad_s)

    def _apply_safety(self, command: BaseCommand) -> BaseCommand:
        return self._nav.apply_safety(command, self._front_range_m)

    def _line_detected(self) -> bool:
        return self._last_vision is not None and self._last_vision.line_detected and self._last_vision.line_confidence >= 0.15

    def _line_aligned(self) -> bool:
        if self._last_vision is None:
            return False
        heading = abs(self._last_vision.line_heading_error_rad or 0.0)
        offset = self._last_vision.line_offset_m
        if offset is None:
            return False
        return (
            heading <= math.radians(self.config.line_heading_tolerance_deg)
            and abs(offset - self.config.line_offset_m) <= self.config.line_offset_tolerance_m
        )

    def _corner_seen(self, x_m: float, y_m: float) -> bool:
        if self._last_vision is None or not self._last_vision.corner_detected:
            return False
        if self._last_vision.corner_confidence < 0.35:
            return False
        if not self._corners:
            return True
        last = self._corners[-1]
        return math.hypot(x_m - last["x_m"], y_m - last["y_m"]) >= self.config.corner_min_spacing_m

    def _record_corner(self, x_m: float, y_m: float) -> None:
        if self._corners and math.hypot(x_m - self._corners[-1]["x_m"], y_m - self._corners[-1]["y_m"]) < 0.25:
            return
        self._corners.append({"x_m": round(x_m, 3), "y_m": round(y_m, 3), "index": len(self._corners)})

    def _turn_complete(self, yaw_rad: float) -> bool:
        if self._turn_start_yaw is None:
            self._turn_start_yaw = yaw_rad
            self._turn_last_yaw = yaw_rad
            return False
        if self._turn_last_yaw is None:
            self._turn_last_yaw = yaw_rad
            return False
        step = abs(_wrap(yaw_rad - self._turn_last_yaw))
        if step <= math.radians(25.0):
            self._turn_accumulated_rad += step
        self._turn_last_yaw = yaw_rad
        return self._turn_accumulated_rad >= math.radians(max(0.0, self.config.turn_angle_deg - 1.0))

    def _loop_reference_reached(self, x_m: float, y_m: float) -> bool:
        if self._start_pose is None or len(self._corners) < 4:
            return False
        if self._distance_traveled_m < self.config.min_loop_distance_m:
            return False
        return math.hypot(x_m - self._start_pose[0], y_m - self._start_pose[1]) <= self.config.loop_closure_radius_m

    def _enter(self, state: SurveyState, event: str, x_m: float, y_m: float, yaw_rad: float) -> None:
        self.state = state
        self._last_event = event
        self._phase_started_at = time.time()
        self._phase_start_pose = (x_m, y_m)
        if state in {SurveyState.ALIGN_OUTSIDE_LINE, SurveyState.FOLLOW_LINE_WITH_OFFSET, SurveyState.RECOVER_LINE}:
            self._line_lost_count = 0
        if state == SurveyState.TURN_AT_CORNER:
            self._turn_start_yaw = yaw_rad
            self._turn_last_yaw = yaw_rad
            self._turn_accumulated_rad = 0.0
        else:
            self._turn_start_yaw = None
            self._turn_last_yaw = None
            self._turn_accumulated_rad = 0.0
        self._phase_visit_counts[state] = self._phase_visit_counts.get(state, 0) + 1
        print(f"survey: {event} -> {state.value}")

    def _finish(self) -> None:
        self._finalize()
        self.state = SurveyState.DONE

    def _fail(self, reason: str) -> None:
        self._failure_reason = reason
        self._finish()

    def _overall_timeout(self, now: float) -> bool:
        return self._started_at is not None and now - self._started_at > self.config.total_timeout_s

    def _phase_timeout(self, now: float) -> bool:
        return self._phase_started_at is not None and now - self._phase_started_at > self.config.phase_timeout_s

    def _phase_distance_m(self) -> float:
        if self._phase_start_pose is None or self._last_pose is None:
            return 0.0
        return math.hypot(self._last_pose[0] - self._phase_start_pose[0], self._last_pose[1] - self._phase_start_pose[1])

    def _update_sensors(
        self,
        lidar_ranges: list[float] | None,
        vision: SurveyVision | None,
        x_m: float,
        y_m: float,
        yaw_rad: float,
    ) -> None:
        self._last_vision = vision
        if lidar_ranges:
            self._front_range_m = self._sector_percentile(lidar_ranges, 0.50, 1 / 8)
            self._right_range_m = self._nullable_sector(lidar_ranges, 0.25, 1 / 10)
            self._left_range_m = self._nullable_sector(lidar_ranges, 0.75, 1 / 10)
            self._scan_coverage = self._mapper.update(
                lidar_ranges,
                x_m,
                y_m,
                yaw_rad,
                None if vision is None else vision.line_offset_m,
                None if vision is None else vision.obstacle_class,
            )
            self.sample_count += 1

    def _nullable_sector(self, ranges: list[float], center_ratio: float, half_width_ratio: float) -> float | None:
        value = self._sector_percentile(ranges, center_ratio, half_width_ratio)
        return None if math.isinf(value) else value

    def _sector_percentile(self, ranges: list[float], center_ratio: float, half_width_ratio: float) -> float:
        n = len(ranges)
        if n < 10:
            return math.inf
        center = int(n * center_ratio)
        half = max(1, int(n * half_width_ratio))
        vals: list[float] = []
        for offset in range(-half, half + 1):
            value = ranges[(center + offset) % n]
            if math.isfinite(value) and self.config.min_fence_range_m < value < self.config.max_fence_range_m:
                vals.append(value)
        if not vals:
            return math.inf
        vals.sort()
        return vals[max(0, int(len(vals) * 0.30) - 1)]

    def _update_distance_traveled(self, x_m: float, y_m: float) -> None:
        if self._last_pose is None:
            self._last_pose = (x_m, y_m)
            return
        step = math.hypot(x_m - self._last_pose[0], y_m - self._last_pose[1])
        if 0.001 <= step <= 1.0:
            self._distance_traveled_m += step
        self._last_pose = (x_m, y_m)

    def _finalize(self) -> None:
        elapsed = 0.0 if self._started_at is None else time.time() - self._started_at
        now = time.time()
        geometry = self._court_geometry_from_corners()
        mapper = self._mapper.result()
        dimension_ok = (
            geometry["length_m"] is not None
            and geometry["width_m"] is not None
            and geometry["length_m"] >= 0.75 * self.config.expected_court_length_m
            and geometry["width_m"] >= 0.75 * self.config.expected_court_width_m
        )
        if self._failure_reason:
            status = "FAILED"
            failure_reason = self._failure_reason
        elif not self._loop_closed:
            status = "FAILED"
            failure_reason = "Survey did not close the outer court-line loop"
        elif len(self._corners) < 4:
            status = "FAILED"
            failure_reason = f"Only {len(self._corners)} court corners detected"
        elif not dimension_ok:
            status = "PARTIAL"
            failure_reason = "Court dimensions are low-confidence; reporting measured loop"
        else:
            status = "SUCCESS"
            failure_reason = None

        bounds: dict = {
            "mapped_at": now,
            "status": status,
            "failure_reason": failure_reason,
            "timed_out": self._timed_out,
            "court_geometry": geometry,
            "detected_court_corners": self._corners,
            "external_boundary_map": {
                "candidates": mapper["external_boundary_candidates"],
                "point_count": mapper["point_count"],
            },
            "free_space_between_court_lines_and_fences": mapper["free_space_between_court_lines_and_fences"],
            "internal_objects": mapper["internal_obstacles"],
            "diagnostics": {
                "confidence": self._confidence(status, geometry, mapper),
                "path_driver": "camera_court_line",
                "lidar_role": "obstacle_mapping_and_safety",
                "net_policy": "net is internal and ignored for perimeter completion",
                "final_state": self.state.value,
                "last_event": self._last_event,
                "loop_closed": self._loop_closed,
                "distance_traveled_m": round(self._distance_traveled_m, 2),
                "elapsed_s": round(elapsed, 1),
                "sample_count": self.sample_count,
            },
            "navigation": {
                "source": "camera_outer_court_line_fsm",
                "sensor_only": True,
                "mapping_pose_source": "platform_localization_estimate",
                "final_state": self.state.value,
                "loop_closed": self._loop_closed,
            },
            "point_cloud_sample": mapper["point_cloud_sample"],
            "surveyed_at": now,
            "survey_complete": status == "SUCCESS",
            "sample_count": self.sample_count,
            "point_count": mapper["point_count"],
        }
        self._attach_active_session(bounds)
        self._court_bounds = bounds
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.output_path.with_suffix(".tmp.json")
        with tmp.open("w", encoding="utf-8") as f:
            json.dump(bounds, f, indent=2)
            f.write("\n")
        tmp.replace(self.output_path)
        print(f"map court: {status} in {elapsed:.1f}s -> {self.output_path}")

    def _court_geometry_from_corners(self) -> dict:
        if len(self._corners) < 4:
            return {
                "length_m": None,
                "width_m": None,
                "orientation_deg": None,
                "method": "insufficient_corners",
            }
        pts = [(corner["x_m"], corner["y_m"]) for corner in self._corners[:4]]
        sides = [
            math.hypot(pts[(i + 1) % 4][0] - pts[i][0], pts[(i + 1) % 4][1] - pts[i][1])
            for i in range(4)
        ]
        pair_a = (sides[0] + sides[2]) * 0.5
        pair_b = (sides[1] + sides[3]) * 0.5
        length = max(pair_a, pair_b)
        width = min(pair_a, pair_b)
        dx = pts[1][0] - pts[0][0]
        dy = pts[1][1] - pts[0][1]
        return {
            "length_m": round(length, 3),
            "width_m": round(width, 3),
            "orientation_deg": round(math.degrees(math.atan2(dy, dx)), 2),
            "method": "camera_detected_outer_line_corners",
            "expected_tennis_length_m": self.config.expected_court_length_m,
            "expected_tennis_width_m": self.config.expected_court_width_m,
        }

    def _confidence(self, status: str, geometry: dict, mapper: dict) -> float:
        score = 0.25
        if self._loop_closed:
            score += 0.25
        score += min(0.2, len(self._corners) * 0.05)
        if geometry["length_m"] is not None and geometry["width_m"] is not None:
            score += 0.15
        if mapper["external_boundary_candidates"]:
            score += 0.1
        if status == "SUCCESS":
            score += 0.05
        return round(min(1.0, score), 2)

    def _attach_active_session(self, bounds: dict) -> None:
        try:
            if not _VENDORS_FILE.exists():
                return
            vdata = json.loads(_VENDORS_FILE.read_text(encoding="utf-8"))
            active = vdata.get("active") or {}
            if not active.get("vendor_id"):
                return
            vmap = {v["id"]: v for v in vdata.get("vendors", [])}
            cmap = {c["id"]: c for c in vdata.get("courts", [])}
            vendor = vmap.get(active["vendor_id"], {})
            court = cmap.get(active.get("court_id", ""), {})
            bounds["vendor_id"] = active["vendor_id"]
            bounds["court_id"] = active.get("court_id")
            bounds["vendor_name"] = vendor.get("name", "")
            bounds["court_name"] = court.get("name", "")
            bounds["court_surface"] = court.get("surface", "")
        except Exception:
            return

    def _cmd(self, base: BaseCommand, vision: SurveyVision | None = None) -> SurveyCommand:
        return SurveyCommand(self.state, base, self.sample_count, vision)


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi
