"""Runtime-agnostic Map Court finite-state machine.

Map Court is the only court-survey implementation. It must not use simulator
waypoints or pre-recorded court coordinates. The traversal is driven by named
sensor events from OAK-D depth, LiDAR sectors, and a platform localization
estimate used only for mapping and loop-closure bookkeeping.
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


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_BOUNDARY_FILE = PROJECT_ROOT / "runtime" / "court_boundary.json"
_VENDORS_FILE = PROJECT_ROOT / "runtime" / "vendors.json"

_LIDAR_LOCAL_X = -0.20
_LIDAR_LOCAL_Y = 0.0
_SUBSAMPLE = 8
_MIN_POINTS = 500
_TIMEOUT_S = 540.0
_MIN_LOOP_DISTANCE_M = 35.0
_LOOP_CLOSURE_RADIUS_M = 1.2
_MIN_COURT_LENGTH_M = 23.0
_MIN_COURT_WIDTH_M = 10.0


class SurveyState(str, Enum):
    FIND_FIRST_OBSTACLE = "find_first_obstacle"
    APPROACH_NET = "approach_net"
    TURN_LEFT_AT_NET = "turn_left_at_net"
    FOLLOW_NET_TO_FENCE = "follow_net_to_fence"
    TURN_LEFT_AT_FENCE_1 = "turn_left_at_fence_1"
    FOLLOW_FENCE_TO_NEXT_FENCE = "follow_fence_to_next_fence"
    TURN_LEFT_AT_FENCE_2 = "turn_left_at_fence_2"
    FOLLOW_FENCE_TO_NET = "follow_fence_to_net"
    CROSS_NET_ON_RIGHT_SIDE = "cross_net_on_right_side"
    FOLLOW_SECOND_HALF_PERIMETER = "follow_second_half_perimeter"
    COMPLETE_AT_FIRST_NET_TURN_REFERENCE = "complete_at_first_net_turn_reference"
    DONE = "done"


@dataclass(frozen=True)
class SurveyConfig:
    drive_speed_m_s: float = 0.35
    turn_speed_rad_s: float = 1.0
    corridor_speed_m_s: float = 0.22
    min_fence_range_m: float = 0.35
    max_fence_range_m: float = 12.0
    net_detect_range_m: float = 5.5
    net_standoff_m: float = 0.45
    first_obstacle_classify_range_m: float = 1.6
    first_obstacle_min_travel_m: float = 1.2
    fence_turn_range_m: float = 0.85
    desired_side_clearance_m: float = 0.9
    side_clearance_gain: float = 0.85
    turn_angle_deg: float = 90.0
    gap_cross_distance_m: float = 1.9
    phase_timeout_s: float = 90.0

    @classmethod
    def from_env(cls) -> "SurveyConfig":
        d = cls()
        return cls(
            drive_speed_m_s=_env_float("SURVEY_DRIVE_SPEED_M_S", d.drive_speed_m_s),
            turn_speed_rad_s=_env_float("SURVEY_TURN_SPEED_RAD_S", d.turn_speed_rad_s),
            corridor_speed_m_s=_env_float("SURVEY_CORRIDOR_SPEED_M_S", d.corridor_speed_m_s),
            net_detect_range_m=_env_float("SURVEY_NET_DETECT_RANGE_M", d.net_detect_range_m),
            net_standoff_m=_env_float("SURVEY_NET_STANDOFF_M", d.net_standoff_m),
            first_obstacle_classify_range_m=_env_float(
                "SURVEY_FIRST_OBSTACLE_CLASSIFY_RANGE_M",
                d.first_obstacle_classify_range_m,
            ),
            first_obstacle_min_travel_m=_env_float(
                "SURVEY_FIRST_OBSTACLE_MIN_TRAVEL_M",
                d.first_obstacle_min_travel_m,
            ),
            fence_turn_range_m=_env_float("SURVEY_FENCE_TURN_RANGE_M", d.fence_turn_range_m),
            desired_side_clearance_m=_env_float("SURVEY_DESIRED_SIDE_CLEARANCE_M", d.desired_side_clearance_m),
            side_clearance_gain=_env_float("SURVEY_SIDE_CLEARANCE_GAIN", d.side_clearance_gain),
            turn_angle_deg=_env_float("SURVEY_TURN_ANGLE_DEG", d.turn_angle_deg),
            gap_cross_distance_m=_env_float("SURVEY_GAP_CROSS_DISTANCE_M", d.gap_cross_distance_m),
            phase_timeout_s=_env_float("SURVEY_PHASE_TIMEOUT_S", d.phase_timeout_s),
        )


@dataclass(frozen=True)
class SurveyVision:
    """OAK-D depth clearance summary in robot frame."""

    center_m: float | None = None
    left_m: float | None = None
    right_m: float | None = None
    valid_count: int = 0
    obstacle_class: str | None = None


@dataclass(frozen=True)
class SurveyCommand:
    state: SurveyState
    base: BaseCommand
    sample_count: int
    vision: SurveyVision | None = None


class CourtSurveyBehavior:
    """Map Court FSM: first obstacle, left-turn perimeter, right-side net crossing."""

    def __init__(
        self,
        config: SurveyConfig | None = None,
        output_path: Path = DEFAULT_BOUNDARY_FILE,
    ) -> None:
        self.config = config or SurveyConfig()
        self.output_path = output_path
        self.state = SurveyState.FIND_FIRST_OBSTACLE
        self.sample_count = 0
        self._started_at: float | None = None
        self._phase_started_at: float | None = None
        self._initialized = False
        self._map_xs: list[float] = []
        self._map_ys: list[float] = []
        self._court_bounds: dict | None = None
        self._front_range_m = math.inf
        self._left_range_m: float | None = None
        self._right_range_m: float | None = None
        self._oak_range_m: float | None = None
        self._last_vision: SurveyVision | None = None
        self._oak_brake_active = False
        self._net_detection_source = "none"
        self._front_obstacle_kind = "unknown"
        self._front_obstacle_source = "none"
        self._first_obstacle_kind: str | None = None
        self._last_event = "none"
        self._failure_reason: str | None = None
        self._timed_out = False
        self._loop_closed = False
        self._start_pose: tuple[float, float] | None = None
        self._first_net_turn_pose: tuple[float, float] | None = None
        self._loop_reference_pose: tuple[float, float] | None = None
        self._last_pose: tuple[float, float] | None = None
        self._distance_traveled_m = 0.0
        self._turn_start_yaw: float | None = None
        self._turn_last_yaw: float | None = None
        self._turn_accumulated_rad = 0.0
        self._gap_start_pose: tuple[float, float] | None = None
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
        return {
            "state": self.state.value,
            "navigation_source": "map_court_sensor_fsm",
            "sensor_only_navigation": True,
            "mapping_pose_source": "platform_localization_estimate",
            "last_event": self._last_event,
            "failure_reason": self._failure_reason,
            "timed_out": self._timed_out,
            "front_lidar_range_m": None if math.isinf(self._front_range_m) else self._front_range_m,
            "left_lidar_range_m": self._left_range_m,
            "right_lidar_range_m": self._right_range_m,
            "oak_range_m": self._oak_range_m,
            "oak_brake_active": self._oak_brake_active,
            "net_detection_source": self._net_detection_source,
            "front_obstacle_kind": self._front_obstacle_kind,
            "front_obstacle_source": self._front_obstacle_source,
            "first_obstacle_kind": self._first_obstacle_kind,
            "loop_closed": self._loop_closed,
            "distance_traveled_m": round(self._distance_traveled_m, 2),
            "turn_progress_deg": round(math.degrees(self._turn_accumulated_rad), 1),
            "turn_target_deg": self.config.turn_angle_deg if self.state.name.startswith("TURN_LEFT") else None,
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
            self._enter(SurveyState.FIND_FIRST_OBSTACLE, "map_court_started", x_m, y_m, yaw_rad)

        if self.state == SurveyState.DONE:
            return self._cmd(BaseCommand(0.0, 0.0), vision)

        self._update_sensors(lidar_ranges, vision, x_m, y_m, yaw_rad)
        self._update_distance_traveled(x_m, y_m)

        if self._overall_timeout(now):
            self._timed_out = True
            self._fail("Timed out before Map Court FSM completed")
            return self._cmd(BaseCommand(0.0, 0.0), vision)
        if self._phase_timeout(now):
            self._fail(f"Phase timeout in {self.state.value}; last_event={self._last_event}")
            return self._cmd(BaseCommand(0.0, 0.0), vision)

        command = self._step(x_m, y_m, yaw_rad)
        return self._cmd(command, vision)

    def _step(self, x_m: float, y_m: float, yaw_rad: float) -> BaseCommand:
        cfg = self.config

        if self.state == SurveyState.FIND_FIRST_OBSTACLE:
            if self._front_obstacle_ready_for_classification():
                obstacle_kind = self._classify_front_obstacle()
                self._first_obstacle_kind = obstacle_kind
                if obstacle_kind == "net":
                    self._net_detection_source = self._front_obstacle_source
                    if self._near_net():
                        self._first_net_turn_pose = (x_m, y_m)
                        self._loop_reference_pose = (x_m, y_m)
                        self._enter(SurveyState.TURN_LEFT_AT_NET, "first_obstacle_net_near", x_m, y_m, yaw_rad)
                    else:
                        self._enter(SurveyState.APPROACH_NET, "first_obstacle_net", x_m, y_m, yaw_rad)
                    return BaseCommand(0.0, 0.0)
                if obstacle_kind == "fence":
                    self._loop_reference_pose = (x_m, y_m)
                    self._enter(SurveyState.TURN_LEFT_AT_FENCE_1, "first_obstacle_fence", x_m, y_m, yaw_rad)
                    return BaseCommand(0.0, 0.0)
                self._fail("Unable to classify the first obstacle as net or fence")
                return BaseCommand(0.0, 0.0)
            return self._drive_until_first_obstacle()

        if self.state == SurveyState.APPROACH_NET:
            if self._front_obstacle_ready_for_classification() and self._classify_front_obstacle() == "fence":
                self._first_obstacle_kind = "fence"
                self._loop_reference_pose = (x_m, y_m)
                self._enter(SurveyState.TURN_LEFT_AT_FENCE_1, "approach_reclassified_as_fence", x_m, y_m, yaw_rad)
                return BaseCommand(0.0, 0.0)
            if self._near_net():
                self._first_net_turn_pose = (x_m, y_m)
                self._loop_reference_pose = (x_m, y_m)
                self._enter(SurveyState.TURN_LEFT_AT_NET, "near_net", x_m, y_m, yaw_rad)
                return BaseCommand(0.0, 0.0)
            return self._drive_toward_observed_net()

        if self.state == SurveyState.TURN_LEFT_AT_NET:
            if self._turn_complete(yaw_rad):
                self._enter(SurveyState.FOLLOW_NET_TO_FENCE, "left_turn_complete", x_m, y_m, yaw_rad)
                return BaseCommand(0.0, 0.0)
            return self._left_turn_command()

        if self.state == SurveyState.FOLLOW_NET_TO_FENCE:
            if self._near_fence():
                self._enter(SurveyState.TURN_LEFT_AT_FENCE_1, "near_fence", x_m, y_m, yaw_rad)
                return BaseCommand(0.0, 0.0)
            return self._drive_parallel_to_boundary()

        if self.state == SurveyState.TURN_LEFT_AT_FENCE_1:
            if self._turn_complete(yaw_rad):
                self._enter(SurveyState.FOLLOW_FENCE_TO_NEXT_FENCE, "corner_detected", x_m, y_m, yaw_rad)
                return BaseCommand(0.0, 0.0)
            return self._left_turn_command()

        if self.state == SurveyState.FOLLOW_FENCE_TO_NEXT_FENCE:
            if self._first_obstacle_kind == "fence" and self._loop_reference_reached(x_m, y_m):
                self._enter(
                    SurveyState.COMPLETE_AT_FIRST_NET_TURN_REFERENCE,
                    "loop_closed",
                    x_m,
                    y_m,
                    yaw_rad,
                )
                return BaseCommand(0.0, 0.0)
            if self._near_fence():
                next_turn = SurveyState.TURN_LEFT_AT_FENCE_1 if self._first_obstacle_kind == "fence" else SurveyState.TURN_LEFT_AT_FENCE_2
                self._enter(next_turn, "near_fence", x_m, y_m, yaw_rad)
                return BaseCommand(0.0, 0.0)
            return self._drive_parallel_to_boundary()

        if self.state == SurveyState.TURN_LEFT_AT_FENCE_2:
            if self._turn_complete(yaw_rad):
                self._enter(SurveyState.FOLLOW_FENCE_TO_NET, "corner_detected", x_m, y_m, yaw_rad)
                return BaseCommand(0.0, 0.0)
            return self._left_turn_command()

        if self.state == SurveyState.FOLLOW_FENCE_TO_NET:
            if self._net_detected() and self._phase_elapsed_s() > 2.0:
                self._enter(SurveyState.CROSS_NET_ON_RIGHT_SIDE, "right_side_net_gap_detected", x_m, y_m, yaw_rad)
                return BaseCommand(0.0, 0.0)
            return self._drive_parallel_to_boundary()

        if self.state == SurveyState.CROSS_NET_ON_RIGHT_SIDE:
            if self._gap_start_pose is None:
                self._gap_start_pose = (x_m, y_m)
            if self._distance_from(self._gap_start_pose, x_m, y_m) >= cfg.gap_cross_distance_m:
                self._enter(SurveyState.FOLLOW_SECOND_HALF_PERIMETER, "gap_crossed", x_m, y_m, yaw_rad)
                return BaseCommand(0.0, 0.0)
            return self._drive_gap_centered()

        if self.state == SurveyState.FOLLOW_SECOND_HALF_PERIMETER:
            if self._loop_reference_reached(x_m, y_m):
                self._enter(
                    SurveyState.COMPLETE_AT_FIRST_NET_TURN_REFERENCE,
                    "loop_closed",
                    x_m,
                    y_m,
                    yaw_rad,
                )
                return BaseCommand(0.0, 0.0)
            return self._drive_parallel_to_boundary()

        if self.state == SurveyState.COMPLETE_AT_FIRST_NET_TURN_REFERENCE:
            self._loop_closed = True
            self._finish()
            return BaseCommand(0.0, 0.0)

        return BaseCommand(0.0, 0.0)

    def _drive_until_first_obstacle(self) -> BaseCommand:
        turn = 0.0
        if self._left_range_m is not None and self._left_range_m < 0.45:
            turn = -0.25 * self.config.turn_speed_rad_s
        elif self._right_range_m is not None and self._right_range_m < 0.45:
            turn = 0.25 * self.config.turn_speed_rad_s
        return BaseCommand(self.config.drive_speed_m_s, turn)

    def _left_turn_command(self) -> BaseCommand:
        return BaseCommand(0.0, self.config.turn_speed_rad_s)

    def _drive_toward_observed_net(self) -> BaseCommand:
        speed = self.config.drive_speed_m_s
        turn = self._avoidance_turn()
        return BaseCommand(speed, turn)

    def _drive_parallel_to_boundary(self) -> BaseCommand:
        front = self._front_obstacle_range()
        if front <= self.config.fence_turn_range_m:
            return BaseCommand(0.0, self.config.turn_speed_rad_s)
        side = self._right_range_m
        if side is None:
            turn = -0.25 * self.config.turn_speed_rad_s
        else:
            error = side - self.config.desired_side_clearance_m
            turn = -error * self.config.side_clearance_gain
            turn = max(-0.65 * self.config.turn_speed_rad_s, min(0.65 * self.config.turn_speed_rad_s, turn))
        return BaseCommand(self.config.drive_speed_m_s, turn)

    def _drive_gap_centered(self) -> BaseCommand:
        left = self._left_range_m
        right = self._right_range_m
        turn = 0.0
        if left is not None and right is not None:
            turn = max(
                -0.45 * self.config.turn_speed_rad_s,
                min(0.45 * self.config.turn_speed_rad_s, (right - left) * 0.5),
            )
        return BaseCommand(self.config.corridor_speed_m_s, turn)

    def _avoidance_turn(self) -> float:
        if self._left_range_m is not None and self._left_range_m < 0.65:
            return -0.4 * self.config.turn_speed_rad_s
        if self._right_range_m is not None and self._right_range_m < 0.65:
            return 0.4 * self.config.turn_speed_rad_s
        return 0.0

    def _net_detected(self) -> bool:
        if self.config.net_standoff_m < self._front_range_m <= self.config.net_detect_range_m:
            self._net_detection_source = "lidar_front"
            return True
        if self._oak_range_m is not None and self.config.net_standoff_m < self._oak_range_m <= self.config.net_detect_range_m:
            self._net_detection_source = "oak_depth"
            return True
        self._net_detection_source = "none"
        return False

    def _front_obstacle_ready_for_classification(self) -> bool:
        if self._explicit_obstacle_class() in {"net", "fence"}:
            return self._front_obstacle_range() <= self.config.first_obstacle_classify_range_m
        if self._front_range_m <= self.config.first_obstacle_classify_range_m:
            return True
        if self._oak_range_m is None or self._oak_range_m > self.config.first_obstacle_classify_range_m:
            return False
        return self._distance_traveled_m >= self.config.first_obstacle_min_travel_m

    def _classify_front_obstacle(self) -> str:
        explicit = self._explicit_obstacle_class()
        if explicit in {"net", "fence"}:
            self._front_obstacle_kind = explicit
            self._front_obstacle_source = "oak_visual_classifier"
            return explicit

        oak_close = self._oak_range_m is not None and self._oak_range_m <= self.config.first_obstacle_classify_range_m
        lidar_close = self._front_range_m <= self.config.first_obstacle_classify_range_m

        if oak_close and (not lidar_close or self._front_range_m > (self._oak_range_m or 0.0) + 0.75):
            self._front_obstacle_kind = "net"
            self._front_obstacle_source = "oak_depth_no_lidar_front_return"
            return "net"
        if lidar_close:
            self._front_obstacle_kind = "fence"
            self._front_obstacle_source = "lidar_front"
            return "fence"
        if oak_close:
            self._front_obstacle_kind = "net"
            self._front_obstacle_source = "oak_depth"
            return "net"

        self._front_obstacle_kind = "unknown"
        self._front_obstacle_source = "none"
        return "unknown"

    def _explicit_obstacle_class(self) -> str | None:
        if self._last_vision is None or not self._last_vision.obstacle_class:
            return None
        return self._last_vision.obstacle_class.strip().lower()

    def _near_net(self) -> bool:
        return self._front_obstacle_range() <= self.config.net_standoff_m

    def _near_fence(self) -> bool:
        return self._front_obstacle_range() <= self.config.fence_turn_range_m

    def _front_obstacle_range(self) -> float:
        candidates = [self._front_range_m]
        if self._oak_range_m is not None:
            candidates.append(self._oak_range_m)
        return min(candidates)

    def _turn_complete(self, yaw_rad: float) -> bool:
        if self._turn_start_yaw is None:
            self._turn_start_yaw = yaw_rad
            self._turn_last_yaw = yaw_rad
            return False
        if self._turn_last_yaw is None:
            self._turn_last_yaw = yaw_rad
            return False
        step = abs(_wrap(yaw_rad - self._turn_last_yaw))
        if step <= math.radians(20.0):
            self._turn_accumulated_rad += step
        self._turn_last_yaw = yaw_rad
        target = math.radians(max(0.0, self.config.turn_angle_deg - 1.0))
        return self._turn_accumulated_rad >= target

    def _loop_reference_reached(self, x_m: float, y_m: float) -> bool:
        reference = self._loop_reference_pose or self._first_net_turn_pose
        if reference is None:
            return False
        if self._distance_traveled_m < _MIN_LOOP_DISTANCE_M:
            return False
        return self._distance_from(reference, x_m, y_m) <= _LOOP_CLOSURE_RADIUS_M

    def _enter(
        self,
        state: SurveyState,
        event: str,
        x_m: float,
        y_m: float,
        yaw_rad: float,
    ) -> None:
        self.state = state
        self._last_event = event
        self._phase_started_at = time.time()
        if state.name.startswith("TURN_LEFT"):
            self._turn_start_yaw = yaw_rad
            self._turn_last_yaw = yaw_rad
            self._turn_accumulated_rad = 0.0
        else:
            self._turn_start_yaw = None
            self._turn_last_yaw = None
            self._turn_accumulated_rad = 0.0
        self._gap_start_pose = (x_m, y_m) if state == SurveyState.CROSS_NET_ON_RIGHT_SIDE else None
        self._phase_visit_counts[state] = self._phase_visit_counts.get(state, 0) + 1
        print(f"survey: {event} -> {state.value}")

    def _finish(self) -> None:
        self._finalize()
        self.state = SurveyState.DONE

    def _fail(self, reason: str) -> None:
        self._failure_reason = reason
        self._finish()

    def _overall_timeout(self, now: float) -> bool:
        return self._started_at is not None and now - self._started_at > _TIMEOUT_S

    def _phase_timeout(self, now: float) -> bool:
        return self._phase_started_at is not None and now - self._phase_started_at > self.config.phase_timeout_s

    def _phase_elapsed_s(self) -> float:
        return 0.0 if self._phase_started_at is None else time.time() - self._phase_started_at

    def _update_sensors(
        self,
        lidar_ranges: list[float] | None,
        vision: SurveyVision | None,
        x_m: float,
        y_m: float,
        yaw_rad: float,
    ) -> None:
        self._last_vision = vision
        self._oak_range_m = None if vision is None else vision.center_m
        self._oak_brake_active = self._oak_range_m is not None and self._oak_range_m <= self.config.net_standoff_m
        if lidar_ranges:
            self._accumulate(lidar_ranges, x_m, y_m, yaw_rad)
            self._front_range_m = self._sector_percentile(lidar_ranges, 0.50, 1 / 8)
            self._right_range_m = self._nullable_sector(lidar_ranges, 0.25, 1 / 10)
            self._left_range_m = self._nullable_sector(lidar_ranges, 0.75, 1 / 10)

    def _nullable_sector(self, ranges: list[float], center_ratio: float, half_width_ratio: float) -> float | None:
        value = self._sector_percentile(ranges, center_ratio, half_width_ratio)
        return None if math.isinf(value) else value

    def _sector_percentile(self, ranges: list[float], center_ratio: float, half_width_ratio: float) -> float:
        n = len(ranges)
        if n < 10:
            return math.inf
        center = int(n * center_ratio)
        half = max(1, int(n * half_width_ratio))
        lo = max(0, center - half)
        hi = min(n, center + half)
        vals = [
            ranges[i]
            for i in range(lo, hi)
            if math.isfinite(ranges[i]) and self.config.min_fence_range_m < ranges[i] < self.config.max_fence_range_m
        ]
        if not vals:
            return math.inf
        vals.sort()
        return vals[max(0, int(len(vals) * 0.30) - 1)]

    def _accumulate(
        self,
        ranges: list[float],
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
    ) -> None:
        n = len(ranges)
        if n < 10:
            return
        cos_y = math.cos(robot_yaw)
        sin_y = math.sin(robot_yaw)
        added = 0
        for i in range(0, n, _SUBSAMPLE):
            r = ranges[i]
            if not math.isfinite(r) or r < self.config.min_fence_range_m or r > self.config.max_fence_range_m:
                continue
            angle = (i / n) * 2.0 * math.pi - math.pi
            lx = _LIDAR_LOCAL_X + r * math.cos(angle)
            ly = _LIDAR_LOCAL_Y + r * math.sin(angle)
            self._map_xs.append(robot_x + cos_y * lx - sin_y * ly)
            self._map_ys.append(robot_y + sin_y * lx + cos_y * ly)
            added += 1
        if added:
            self.sample_count += 1

    def _update_distance_traveled(self, x_m: float, y_m: float) -> None:
        if self._last_pose is None:
            self._last_pose = (x_m, y_m)
            return
        step = self._distance_from(self._last_pose, x_m, y_m)
        if 0.001 <= step <= 1.0:
            self._distance_traveled_m += step
        self._last_pose = (x_m, y_m)

    @staticmethod
    def _distance_from(pose: tuple[float, float], x_m: float, y_m: float) -> float:
        return math.hypot(x_m - pose[0], y_m - pose[1])

    def _finalize(self) -> None:
        n = len(self._map_xs)
        elapsed = 0.0 if self._started_at is None else time.time() - self._started_at
        now = time.time()

        if n >= _MIN_POINTS:
            xs = sorted(self._map_xs)
            ys = sorted(self._map_ys)
            west_x = round(xs[int(n * 0.05)], 3)
            east_x = round(xs[int(n * 0.95)], 3)
            south_y = round(ys[int(n * 0.05)], 3)
            north_y = round(ys[int(n * 0.95)], 3)
            length_m = round(east_x - west_x, 3)
            width_m = round(north_y - south_y, 3)
            dimension_ok = length_m >= _MIN_COURT_LENGTH_M and width_m >= _MIN_COURT_WIDTH_M
            if self._failure_reason:
                status = "FAILED"
                failure_reason = self._failure_reason
            elif not self._loop_closed:
                status = "FAILED"
                failure_reason = "Map Court FSM did not close the required loop"
            elif not dimension_ok:
                status = "FAILED"
                failure_reason = f"Measured boundary extents too small for a tennis court: {length_m:.2f} x {width_m:.2f} m"
            else:
                status = "SUCCESS"
                failure_reason = None
        else:
            west_x = east_x = south_y = north_y = None
            length_m = width_m = None
            status = "FAILED"
            failure_reason = self._failure_reason or (
                f"Insufficient LiDAR coverage: {n} points accumulated (minimum {_MIN_POINTS} required)"
            )

        bounds: dict = {
            "mapped_at": now,
            "status": status,
            "failure_reason": failure_reason,
            "timed_out": self._timed_out,
            "court_geometry": {
                "length_m": length_m,
                "width_m": width_m,
                "orientation_deg": None,
            },
            "fence_geometry": {
                "west_x": west_x,
                "east_x": east_x,
                "south_y": south_y,
                "north_y": north_y,
                "clearance": {
                    "west_m": None,
                    "east_m": None,
                    "south_m": None,
                    "north_m": None,
                },
            },
            "obstacles": {
                "count": 0,
            },
            "accessibility": {
                "perimeter_complete": status == "SUCCESS",
                "traversable_area_sqm": round(length_m * width_m, 1) if length_m is not None else None,
            },
            "navigation": {
                "source": "map_court_sensor_fsm",
                "sensor_only": True,
                "mapping_pose_source": "platform_localization_estimate",
                "final_state": self.state.value,
                "last_event": self._last_event,
                "loop_closed": self._loop_closed,
                "distance_traveled_m": round(self._distance_traveled_m, 2),
                "elapsed_s": round(elapsed, 1),
            },
            "surveyed_at": now,
            "survey_complete": status == "SUCCESS",
            "sample_count": self.sample_count,
            "point_count": n,
        }
        self._attach_active_session(bounds)
        self._court_bounds = bounds
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.output_path.with_suffix(".tmp.json")
        with tmp.open("w", encoding="utf-8") as f:
            json.dump(bounds, f, indent=2)
            f.write("\n")
        tmp.replace(self.output_path)
        if status == "SUCCESS":
            print(f"map court: complete in {elapsed:.1f}s {self.sample_count} frames, {n} pts -> {self.output_path}")
        else:
            print(f"map court: FAILED in {elapsed:.1f}s {n} pts {failure_reason} -> {self.output_path}")

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
