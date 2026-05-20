"""Court survey pattern for measuring court edges and obstacle distances."""

from __future__ import annotations

import csv
import math
import os
import time
from dataclasses import dataclass
from enum import Enum
from pathlib import Path

from collector import BaseCommand


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SURVEY_FILE = PROJECT_ROOT / "runtime" / "court_survey.csv"


class SurveyState(str, Enum):
    GOTO = "goto"
    SAMPLE = "sample"
    DONE = "done"


@dataclass(frozen=True)
class SurveyConfig:
    x_min_m: float = -11.3
    x_max_m: float = 11.3
    y_min_m: float = -5.0
    y_max_m: float = 5.0
    row_step_m: float = 2.5
    waypoint_tolerance_m: float = 0.22
    heading_tolerance_rad: float = math.radians(5.0)
    drive_speed_m_s: float = 0.24
    turn_speed_rad_s: float = 0.75
    heading_gain: float = 2.2
    sample_hold_s: float = 0.35
    court_half_length_m: float = 13.0
    court_half_width_m: float = 6.5
    singles_half_width_m: float = 4.115
    doubles_half_width_m: float = 5.485
    baseline_x_m: float = 11.885
    service_line_x_m: float = 6.4

    @classmethod
    def from_env(cls) -> "SurveyConfig":
        defaults = cls()
        return cls(
            x_min_m=_env_float("SURVEY_X_MIN_M", defaults.x_min_m),
            x_max_m=_env_float("SURVEY_X_MAX_M", defaults.x_max_m),
            y_min_m=_env_float("SURVEY_Y_MIN_M", defaults.y_min_m),
            y_max_m=_env_float("SURVEY_Y_MAX_M", defaults.y_max_m),
            row_step_m=_env_float("SURVEY_ROW_STEP_M", defaults.row_step_m),
            waypoint_tolerance_m=_env_float("SURVEY_WAYPOINT_TOLERANCE_M", defaults.waypoint_tolerance_m),
            heading_tolerance_rad=math.radians(
                _env_float("SURVEY_HEADING_TOLERANCE_DEG", math.degrees(defaults.heading_tolerance_rad))
            ),
            drive_speed_m_s=_env_float("SURVEY_DRIVE_SPEED_M_S", defaults.drive_speed_m_s),
            turn_speed_rad_s=_env_float("SURVEY_TURN_SPEED_RAD_S", defaults.turn_speed_rad_s),
            heading_gain=_env_float("SURVEY_HEADING_GAIN", defaults.heading_gain),
            sample_hold_s=_env_float("SURVEY_SAMPLE_HOLD_S", defaults.sample_hold_s),
            court_half_length_m=_env_float("SURVEY_COURT_HALF_LENGTH_M", defaults.court_half_length_m),
            court_half_width_m=_env_float("SURVEY_COURT_HALF_WIDTH_M", defaults.court_half_width_m),
            singles_half_width_m=_env_float("SURVEY_SINGLES_HALF_WIDTH_M", defaults.singles_half_width_m),
            doubles_half_width_m=_env_float("SURVEY_DOUBLES_HALF_WIDTH_M", defaults.doubles_half_width_m),
            baseline_x_m=_env_float("SURVEY_BASELINE_X_M", defaults.baseline_x_m),
            service_line_x_m=_env_float("SURVEY_SERVICE_LINE_X_M", defaults.service_line_x_m),
        )


@dataclass(frozen=True)
class SurveyCommand:
    state: SurveyState
    base: BaseCommand
    waypoint_index: int
    sample_count: int


class CourtSurveyBehavior:
    """Drive a boustrophedon pattern and log distance measurements."""

    def __init__(self, config: SurveyConfig | None = None, output_path: Path = DEFAULT_SURVEY_FILE) -> None:
        self.config = config or SurveyConfig()
        self.output_path = output_path
        self.waypoints = self._build_waypoints()
        self.state = SurveyState.GOTO
        self.waypoint_index = 0
        self._sample_elapsed_s = 0.0
        self.sample_count = 0
        self._file_initialized = False
        self._start_waypoint_pending = True

    @classmethod
    def from_env(cls) -> "CourtSurveyBehavior":
        path = Path(os.getenv("SURVEY_OUTPUT_FILE", str(DEFAULT_SURVEY_FILE)))
        return cls(SurveyConfig.from_env(), path)

    def reset(self) -> None:
        self.state = SurveyState.GOTO
        self.waypoint_index = 0
        self._sample_elapsed_s = 0.0
        self.sample_count = 0
        self._file_initialized = False
        self._start_waypoint_pending = True

    def current_target(self) -> tuple[float, float] | None:
        if self.waypoint_index >= len(self.waypoints):
            return None
        return self.waypoints[self.waypoint_index]

    def update(
        self,
        x_m: float,
        y_m: float,
        yaw_rad: float,
        front_range_m: float | None,
        dt_s: float,
    ) -> SurveyCommand:
        if self._start_waypoint_pending:
            if not self.waypoints or math.hypot(self.waypoints[0][0] - x_m, self.waypoints[0][1] - y_m) > self.config.waypoint_tolerance_m:
                self.waypoints.insert(0, (x_m, y_m))
            self._start_waypoint_pending = False

        if self.waypoint_index >= len(self.waypoints):
            self.state = SurveyState.DONE
            return self._command(BaseCommand(0.0, 0.0))

        if self.state == SurveyState.GOTO:
            return self._update_goto(x_m, y_m, yaw_rad)
        if self.state == SurveyState.SAMPLE:
            return self._update_sample(x_m, y_m, yaw_rad, front_range_m, dt_s)
        return self._command(BaseCommand(0.0, 0.0))

    def _update_goto(self, x_m: float, y_m: float, yaw_rad: float) -> SurveyCommand:
        target_x, target_y = self.waypoints[self.waypoint_index]
        dx = target_x - x_m
        dy = target_y - y_m
        distance_m = math.hypot(dx, dy)
        if distance_m <= self.config.waypoint_tolerance_m:
            self.state = SurveyState.SAMPLE
            self._sample_elapsed_s = 0.0
            print(f"survey waypoint {self.waypoint_index + 1}/{len(self.waypoints)} reached at x={x_m:.2f} y={y_m:.2f}")
            return self._command(BaseCommand(0.0, 0.0))

        target_heading = math.atan2(dy, dx)
        heading_error = _wrap_angle(target_heading - yaw_rad)
        turn = self._clamp(heading_error * self.config.heading_gain, self.config.turn_speed_rad_s)
        linear = self.config.drive_speed_m_s * max(0.25, 1.0 - min(1.0, abs(heading_error) / math.pi))
        return self._command(BaseCommand(linear, turn))

    def _update_sample(
        self,
        x_m: float,
        y_m: float,
        yaw_rad: float,
        front_range_m: float | None,
        dt_s: float,
    ) -> SurveyCommand:
        self._sample_elapsed_s += max(0.0, dt_s)
        if self._sample_elapsed_s >= self.config.sample_hold_s:
            self._write_sample(x_m, y_m, yaw_rad, front_range_m)
            self.waypoint_index += 1
            self._sample_elapsed_s = 0.0
            self.state = SurveyState.GOTO
            if self.waypoint_index >= len(self.waypoints):
                self.state = SurveyState.DONE
                print(f"survey complete: {self.sample_count} samples written to {self.output_path}")
        return self._command(BaseCommand(0.0, 0.0))

    def _write_sample(self, x_m: float, y_m: float, yaw_rad: float, front_range_m: float | None) -> None:
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        is_new = not self._file_initialized or not self.output_path.exists()
        with self.output_path.open("a", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=self._fieldnames())
            if is_new:
                writer.writeheader()
                self._file_initialized = True
            writer.writerow(self._sample_row(x_m, y_m, yaw_rad, front_range_m))
        self.sample_count += 1

    def _sample_row(self, x_m: float, y_m: float, yaw_rad: float, front_range_m: float | None) -> dict[str, object]:
        cfg = self.config
        return {
            "timestamp_s": f"{time.time():.3f}",
            "waypoint_index": self.waypoint_index,
            "x_m": f"{x_m:.3f}",
            "y_m": f"{y_m:.3f}",
            "heading_deg": f"{math.degrees(yaw_rad):.1f}",
            "front_range_m": "" if front_range_m is None else f"{front_range_m:.3f}",
            "distance_to_east_fence_m": f"{cfg.court_half_length_m - x_m:.3f}",
            "distance_to_west_fence_m": f"{x_m + cfg.court_half_length_m:.3f}",
            "distance_to_north_fence_m": f"{cfg.court_half_width_m - y_m:.3f}",
            "distance_to_south_fence_m": f"{y_m + cfg.court_half_width_m:.3f}",
            "distance_to_nearest_singles_sideline_m": f"{abs(abs(y_m) - cfg.singles_half_width_m):.3f}",
            "distance_to_nearest_doubles_sideline_m": f"{abs(abs(y_m) - cfg.doubles_half_width_m):.3f}",
            "distance_to_nearest_baseline_m": f"{abs(abs(x_m) - cfg.baseline_x_m):.3f}",
            "distance_to_nearest_service_line_m": f"{abs(abs(x_m) - cfg.service_line_x_m):.3f}",
        }

    def _fieldnames(self) -> list[str]:
        return [
            "timestamp_s",
            "waypoint_index",
            "x_m",
            "y_m",
            "heading_deg",
            "front_range_m",
            "distance_to_east_fence_m",
            "distance_to_west_fence_m",
            "distance_to_north_fence_m",
            "distance_to_south_fence_m",
            "distance_to_nearest_singles_sideline_m",
            "distance_to_nearest_doubles_sideline_m",
            "distance_to_nearest_baseline_m",
            "distance_to_nearest_service_line_m",
        ]

    def _build_waypoints(self) -> list[tuple[float, float]]:
        waypoints: list[tuple[float, float]] = []
        y = self.config.y_min_m
        row = 0
        while y <= self.config.y_max_m + 1e-6:
            if math.isclose(self.config.x_min_m, self.config.x_max_m):
                row_points = [(self.config.x_min_m, y)]
            else:
                row_points = [(self.config.x_min_m, y), (self.config.x_max_m, y)]
            if row % 2:
                row_points.reverse()
            waypoints.extend(row_points)
            y += self.config.row_step_m
            row += 1
        return waypoints

    def _command(self, base: BaseCommand) -> SurveyCommand:
        return SurveyCommand(self.state, base, self.waypoint_index, self.sample_count)

    def _clamp(self, value: float, limit: float) -> float:
        return max(-limit, min(limit, value))


def _env_float(name: str, default: float) -> float:
    value = os.getenv(name)
    if value is None:
        return default
    try:
        return float(value)
    except ValueError:
        print(f"invalid {name}={value!r}; using {default}")
        return default


def _wrap_angle(angle_rad: float) -> float:
    while angle_rad > math.pi:
        angle_rad -= 2 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2 * math.pi
    return angle_rad
