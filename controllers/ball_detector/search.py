"""Coarse-to-fine half-court search: survey viewpoints → zone heatmap → prioritized local scan."""

from __future__ import annotations

import math
import os
from dataclasses import dataclass
from enum import Enum

from collector import BallObservationInput, BaseCommand
from config_utils import _env_float


class SearchState(str, Enum):
    SURVEY_VIEWPOINT = "survey_viewpoint"  # stationary rotation to build zone heatmap
    TRANSIT_TO_ZONE  = "transit_to_zone"   # driving to highest-score zone
    LOCAL_SCAN       = "local_scan"        # boustrophedon sweep within selected zone
    BALL_DETECTED    = "ball_detected"     # ball queued; waiting for collector handoff
    COMPLETE         = "complete"


@dataclass
class ZoneScore:
    zone_id: str
    center_x: float
    center_y: float
    min_x: float
    max_x: float
    min_y: float
    max_y: float
    estimated_count: float = 0.0  # weighted ball density from survey observations
    visit_count: int = 0


@dataclass(frozen=True)
class SearchConfig:
    side: str = "left"
    court_half_length_m: float = 11.885
    court_half_width_m: float = 5.485
    net_clearance_m: float = 0.55
    wall_clearance_m: float = 0.85
    zone_cols: int = 3               # grid columns along court length
    zone_rows: int = 2               # grid rows along court width
    survey_rotate_speed_rad_s: float = 0.45
    survey_viewpoint_dwell_s: float = 6.0
    lane_width_m: float = 1.5        # local scan lane spacing
    waypoint_tolerance_m: float = 0.24
    drive_speed_m_s: float = 0.24
    turn_speed_rad_s: float = 0.75
    heading_gain: float = 2.1
    detection_confidence_threshold: float = 0.03
    target_hold_s: float = 1.25
    obstacle_stop_range_m: float = 0.55
    detected_target_cooldown_s: float = 8.0
    target_merge_distance_m: float = 0.75
    zone_proximity_weight: float = 3.0  # meters; scales distance penalty in zone scoring

    @classmethod
    def from_env(cls) -> "SearchConfig":
        defaults = cls()
        return cls(
            side=os.getenv("SEARCH_SIDE", defaults.side).strip().lower() or defaults.side,
            court_half_length_m=_env_float("SEARCH_COURT_HALF_LENGTH_M", defaults.court_half_length_m),
            court_half_width_m=_env_float("SEARCH_COURT_HALF_WIDTH_M", defaults.court_half_width_m),
            net_clearance_m=_env_float("SEARCH_NET_CLEARANCE_M", defaults.net_clearance_m),
            wall_clearance_m=_env_float("SEARCH_WALL_CLEARANCE_M", defaults.wall_clearance_m),
            zone_cols=int(round(_env_float("SEARCH_ZONE_COLS", defaults.zone_cols))),
            zone_rows=int(round(_env_float("SEARCH_ZONE_ROWS", defaults.zone_rows))),
            survey_rotate_speed_rad_s=_env_float("SEARCH_SURVEY_ROTATE_SPEED_RAD_S", defaults.survey_rotate_speed_rad_s),
            survey_viewpoint_dwell_s=_env_float("SEARCH_SURVEY_VIEWPOINT_DWELL_S", defaults.survey_viewpoint_dwell_s),
            lane_width_m=_env_float("SEARCH_LANE_WIDTH_M", defaults.lane_width_m),
            waypoint_tolerance_m=_env_float("SEARCH_WAYPOINT_TOLERANCE_M", defaults.waypoint_tolerance_m),
            drive_speed_m_s=_env_float("SEARCH_DRIVE_SPEED_M_S", defaults.drive_speed_m_s),
            turn_speed_rad_s=_env_float("SEARCH_TURN_SPEED_RAD_S", defaults.turn_speed_rad_s),
            heading_gain=_env_float("SEARCH_HEADING_GAIN", defaults.heading_gain),
            detection_confidence_threshold=_env_float(
                "SEARCH_DETECTION_CONFIDENCE_THRESHOLD", defaults.detection_confidence_threshold
            ),
            target_hold_s=_env_float("SEARCH_TARGET_HOLD_S", defaults.target_hold_s),
            obstacle_stop_range_m=_env_float("SEARCH_OBSTACLE_STOP_RANGE_M", defaults.obstacle_stop_range_m),
            detected_target_cooldown_s=_env_float(
                "SEARCH_DETECTED_TARGET_COOLDOWN_S", defaults.detected_target_cooldown_s
            ),
            target_merge_distance_m=_env_float("SEARCH_TARGET_MERGE_DISTANCE_M", defaults.target_merge_distance_m),
            zone_proximity_weight=_env_float("SEARCH_ZONE_PROXIMITY_WEIGHT", defaults.zone_proximity_weight),
        )


@dataclass(frozen=True)
class SearchCommand:
    state: SearchState
    base: BaseCommand
    phase: str
    waypoint_index: int
    waypoint_count: int
    zone_id: str
    coverage_pct: float
    target_status: str
    path_status: str
    resume_marker: str


class HalfCourtSearchBehavior:
    """Coarse-to-fine search: survey viewpoints build a zone heatmap, then zones are
    visited in descending density order with a local boustrophedon sweep each."""

    def __init__(self, config: SearchConfig | None = None) -> None:
        self.config = config or SearchConfig()
        self._zones: list[ZoneScore] = self._build_zones()
        self._survey_viewpoints: list[tuple[float, float]] = self._build_survey_viewpoints()
        self._reset_state()

    @classmethod
    def from_env(cls) -> "HalfCourtSearchBehavior":
        return cls(SearchConfig.from_env())

    def reset(self) -> None:
        self._zones = self._build_zones()
        self._reset_state()

    def update(
        self,
        x_m: float,
        y_m: float,
        yaw_rad: float,
        observation: BallObservationInput,
        front_range_m: float | None,
        dt_s: float,
        target_id: int | None = None,
    ) -> SearchCommand:
        self._target_cooldown_s = max(0.0, self._target_cooldown_s - max(0.0, dt_s))

        if self.state == SearchState.BALL_DETECTED:
            self._target_lost_s += max(0.0, dt_s)
            if self._target_lost_s < self.config.target_hold_s:
                self._last_target_status = "queued"
                self._last_path_status = "waiting"
                return self._command(BaseCommand(0.0, 0.0), x_m, y_m)
            self.state = self._resume_state
            self._target_cooldown_s = self.config.detected_target_cooldown_s
            self._last_target_status = "queued"

        if self._has_interrupt_target(observation):
            self._resume_state = self.state
            self._save_resume_marker()
            self.state = SearchState.BALL_DETECTED
            self._target_lost_s = 0.0
            self._last_target_id = target_id
            if observation.world_x_m is not None and observation.world_y_m is not None:
                self._last_target_xy = (observation.world_x_m, observation.world_y_m)
            self._record_zone_observation(observation)
            self._last_target_status = "detected"
            self._last_path_status = "pending_validation"
            return self._command(BaseCommand(0.0, 0.0), x_m, y_m)

        if observation.visible:
            self._record_zone_observation(observation)

        if self.state == SearchState.SURVEY_VIEWPOINT:
            return self._update_survey(x_m, y_m, yaw_rad, front_range_m, dt_s)
        if self.state == SearchState.TRANSIT_TO_ZONE:
            return self._update_transit(x_m, y_m, yaw_rad, front_range_m)
        if self.state == SearchState.LOCAL_SCAN:
            return self._update_local_scan(x_m, y_m, yaw_rad, front_range_m)

        return self._command(BaseCommand(0.0, 0.0), x_m, y_m)

    def snapshot(self, x_m: float, y_m: float) -> dict[str, object]:
        cmd = self._command(BaseCommand(0.0, 0.0), x_m, y_m)
        return {
            "search_state": cmd.state.value,
            "phase": cmd.phase,
            "zone_id": cmd.zone_id,
            "coverage_pct": cmd.coverage_pct,
            "waypoint_index": cmd.waypoint_index,
            "waypoint_count": cmd.waypoint_count,
            "target_id": self._last_target_id,
            "target_x_m": None if self._transit_target is None else self._transit_target[0],
            "target_y_m": None if self._transit_target is None else self._transit_target[1],
            "target_status": cmd.target_status,
            "path_status": cmd.path_status,
            "resume_marker": cmd.resume_marker,
            "target_cooldown_s": round(self._target_cooldown_s, 2),
            "zone_heatmap": [
                {
                    "zone_id": z.zone_id,
                    "estimated_count": round(z.estimated_count, 2),
                    "visit_count": z.visit_count,
                }
                for z in self._zones
            ],
        }

    # ---- Survey phase ----

    def _update_survey(
        self,
        x_m: float,
        y_m: float,
        yaw_rad: float,
        front_range_m: float | None,
        dt_s: float,
    ) -> SearchCommand:
        if self._survey_viewpoint_index >= len(self._survey_viewpoints):
            return self._end_survey(x_m, y_m)

        target_vp = self._survey_viewpoints[self._survey_viewpoint_index]
        dist = math.hypot(target_vp[0] - x_m, target_vp[1] - y_m)

        if dist > self.config.waypoint_tolerance_m:
            base = self._goto_command(x_m, y_m, yaw_rad, target_vp)
            if base.linear_speed_m_s > 0.0 and front_range_m is not None and front_range_m < self.config.obstacle_stop_range_m:
                self._last_path_status = "blocked"
                return self._command(BaseCommand(0.0, 0.0), x_m, y_m)
            self._last_path_status = "clear"
            self._survey_dwell_s = 0.0
            return self._command(base, x_m, y_m)

        self._survey_dwell_s += max(0.0, dt_s)
        self._last_path_status = "clear"
        if self._survey_dwell_s >= self.config.survey_viewpoint_dwell_s:
            self._survey_viewpoint_index += 1
            self._survey_dwell_s = 0.0
            self._save_resume_marker()
            if self._survey_viewpoint_index >= len(self._survey_viewpoints):
                return self._end_survey(x_m, y_m)

        return self._command(BaseCommand(0.0, self.config.survey_rotate_speed_rad_s), x_m, y_m)

    def _end_survey(self, x_m: float, y_m: float) -> SearchCommand:
        zone = self._select_next_zone(x_m, y_m)
        if zone is None:
            self.state = SearchState.COMPLETE
        else:
            self._start_zone(zone)
        return self._command(BaseCommand(0.0, 0.0), x_m, y_m)

    # ---- Transit phase ----

    def _update_transit(
        self,
        x_m: float,
        y_m: float,
        yaw_rad: float,
        front_range_m: float | None,
    ) -> SearchCommand:
        target = self._transit_target
        if target is None or math.hypot(target[0] - x_m, target[1] - y_m) <= self.config.waypoint_tolerance_m:
            return self._enter_local_scan(x_m, y_m)

        base = self._goto_command(x_m, y_m, yaw_rad, target)
        if base.linear_speed_m_s > 0.0 and front_range_m is not None and front_range_m < self.config.obstacle_stop_range_m:
            self._last_path_status = "blocked"
            return self._command(BaseCommand(0.0, 0.0), x_m, y_m)
        self._last_path_status = "clear"
        return self._command(base, x_m, y_m)

    def _enter_local_scan(self, x_m: float, y_m: float) -> SearchCommand:
        if self._active_zone is not None:
            self.state = SearchState.LOCAL_SCAN
            self._local_waypoints = self._build_local_scan_waypoints(self._active_zone)
            self._local_waypoint_index = 0
            self._save_resume_marker()
        return self._command(BaseCommand(0.0, 0.0), x_m, y_m)

    # ---- Local scan phase ----

    def _update_local_scan(
        self,
        x_m: float,
        y_m: float,
        yaw_rad: float,
        front_range_m: float | None,
    ) -> SearchCommand:
        while self._local_waypoint_index < len(self._local_waypoints):
            wp = self._local_waypoints[self._local_waypoint_index]
            if math.hypot(wp[0] - x_m, wp[1] - y_m) > self.config.waypoint_tolerance_m:
                break
            self._local_waypoint_index += 1

        if self._local_waypoint_index >= len(self._local_waypoints):
            if self._active_zone is not None:
                self._active_zone.visit_count += 1
            zone = self._select_next_zone(x_m, y_m)
            if zone is None:
                self.state = SearchState.COMPLETE
            else:
                self._start_zone(zone)
            return self._command(BaseCommand(0.0, 0.0), x_m, y_m)

        wp = self._local_waypoints[self._local_waypoint_index]
        base = self._goto_command(x_m, y_m, yaw_rad, wp)
        if base.linear_speed_m_s > 0.0 and front_range_m is not None and front_range_m < self.config.obstacle_stop_range_m:
            self._last_path_status = "blocked"
            return self._command(BaseCommand(0.0, 0.0), x_m, y_m)
        self._last_path_status = "clear"
        return self._command(base, x_m, y_m)

    # ---- Zone management ----

    def _start_zone(self, zone: ZoneScore) -> None:
        self._active_zone = zone
        self._transit_target = (zone.center_x, zone.center_y)
        self.state = SearchState.TRANSIT_TO_ZONE
        self._save_resume_marker()

    def _select_next_zone(self, x_m: float, y_m: float) -> ZoneScore | None:
        candidates = [z for z in self._zones if z.visit_count == 0]
        if not candidates:
            # Re-visit zones that had balls but haven't been cleared
            candidates = [z for z in self._zones if z.estimated_count > 0.5 and z.visit_count < 2]
        if not candidates:
            return None

        def score(zone: ZoneScore) -> float:
            dist = math.hypot(zone.center_x - x_m, zone.center_y - y_m)
            density = zone.estimated_count / (1.0 + dist / max(0.1, self.config.zone_proximity_weight))
            return density + (0.1 if zone.visit_count == 0 else 0.0)

        return max(candidates, key=score)

    def _record_zone_observation(self, observation: BallObservationInput) -> None:
        if observation.world_x_m is None or observation.world_y_m is None:
            return
        for zone in self._zones:
            if zone.min_x <= observation.world_x_m <= zone.max_x and zone.min_y <= observation.world_y_m <= zone.max_y:
                zone.estimated_count += max(0.1, observation.confidence)
                break

    # ---- Interrupt / target helpers ----

    def _has_interrupt_target(self, observation: BallObservationInput) -> bool:
        if self._target_cooldown_s > 0.0:
            if self._same_recent_target(observation) or observation.world_x_m is None:
                return False
        return (
            observation.visible
            and observation.confidence >= self.config.detection_confidence_threshold
            and math.isfinite(observation.distance_m)
            and observation.distance_m <= self.max_interrupt_distance_m
        )

    def _same_recent_target(self, observation: BallObservationInput) -> bool:
        if self._last_target_xy is None or observation.world_x_m is None or observation.world_y_m is None:
            return False
        return (
            math.hypot(
                self._last_target_xy[0] - observation.world_x_m,
                self._last_target_xy[1] - observation.world_y_m,
            )
            <= self.config.target_merge_distance_m
        )

    # ---- Navigation ----

    def _goto_command(self, x_m: float, y_m: float, yaw_rad: float, target: tuple[float, float]) -> BaseCommand:
        dx = target[0] - x_m
        dy = target[1] - y_m
        heading_error = _wrap_angle(math.atan2(dy, dx) - yaw_rad)
        angular = _clamp(heading_error * self.config.heading_gain, self.config.turn_speed_rad_s)
        linear = self.config.drive_speed_m_s * max(0.20, 1.0 - min(1.0, abs(heading_error) / math.pi))
        if abs(heading_error) > math.radians(55.0):
            linear = 0.0
        return BaseCommand(linear, angular)

    # ---- Output / bookkeeping ----

    def _save_resume_marker(self) -> None:
        if self.state == SearchState.SURVEY_VIEWPOINT:
            self._resume_marker_str = f"survey_viewpoint:{self._survey_viewpoint_index}"
        elif self.state == SearchState.TRANSIT_TO_ZONE:
            zone_id = self._active_zone.zone_id if self._active_zone else "?"
            self._resume_marker_str = f"transit_to_zone:{zone_id}"
        elif self.state == SearchState.LOCAL_SCAN:
            zone_id = self._active_zone.zone_id if self._active_zone else "?"
            self._resume_marker_str = f"local_scan:{zone_id}:{self._local_waypoint_index}"
        else:
            self._resume_marker_str = self.state.value

    def _command(self, base: BaseCommand, x_m: float, y_m: float) -> SearchCommand:
        if self.state == SearchState.LOCAL_SCAN:
            wp_index = self._local_waypoint_index
            wp_count = len(self._local_waypoints)
        else:
            wp_index = self._survey_viewpoint_index
            wp_count = len(self._survey_viewpoints)

        return SearchCommand(
            state=self.state,
            base=base,
            phase=self.state.value,
            waypoint_index=wp_index,
            waypoint_count=wp_count,
            zone_id=self._active_zone.zone_id if self._active_zone else self._zone_id_at(x_m, y_m),
            coverage_pct=self._coverage_pct(),
            target_status=self._last_target_status,
            path_status=self._last_path_status,
            resume_marker=self._resume_marker_str,
        )

    def _coverage_pct(self) -> float:
        if self.state == SearchState.COMPLETE:
            return 100.0
        n_vps = max(1, len(self._survey_viewpoints))
        n_zones = max(1, len(self._zones))
        survey_frac = min(1.0, self._survey_viewpoint_index / n_vps)
        zones_visited = sum(1 for z in self._zones if z.visit_count > 0)
        zone_frac = zones_visited / n_zones
        return round((survey_frac * 0.25 + zone_frac * 0.75) * 100.0, 1)

    def _zone_id_at(self, x_m: float, y_m: float) -> str:
        for zone in self._zones:
            if zone.min_x <= x_m <= zone.max_x and zone.min_y <= y_m <= zone.max_y:
                return zone.zone_id
        return "F"

    # ---- Court geometry ----

    def _bounds(self) -> tuple[float, float, float, float]:
        cfg = self.config
        if cfg.side == "right":
            return (
                cfg.net_clearance_m,
                cfg.court_half_length_m - cfg.wall_clearance_m,
                -cfg.court_half_width_m + cfg.wall_clearance_m,
                cfg.court_half_width_m - cfg.wall_clearance_m,
            )
        return (
            -cfg.court_half_length_m + cfg.wall_clearance_m,
            -cfg.net_clearance_m,
            -cfg.court_half_width_m + cfg.wall_clearance_m,
            cfg.court_half_width_m - cfg.wall_clearance_m,
        )

    def _build_zones(self) -> list[ZoneScore]:
        min_x, max_x, min_y, max_y = self._bounds()
        cols = max(1, self.config.zone_cols)
        rows = max(1, self.config.zone_rows)
        x_step = (max_x - min_x) / cols
        y_step = (max_y - min_y) / rows
        zones = []
        for col in range(cols):
            for row in range(rows):
                zmin_x = min_x + col * x_step
                zmax_x = min_x + (col + 1) * x_step
                zmin_y = min_y + row * y_step
                zmax_y = min_y + (row + 1) * y_step
                zone_id = chr(ord("A") + col * rows + row)
                zones.append(
                    ZoneScore(
                        zone_id=zone_id,
                        center_x=(zmin_x + zmax_x) / 2,
                        center_y=(zmin_y + zmax_y) / 2,
                        min_x=zmin_x,
                        max_x=zmax_x,
                        min_y=zmin_y,
                        max_y=zmax_y,
                    )
                )
        return zones

    def _build_survey_viewpoints(self) -> list[tuple[float, float]]:
        min_x, max_x, min_y, max_y = self._bounds()
        mid_y = (min_y + max_y) / 2
        n = max(2, self.config.zone_cols)
        step = (max_x - min_x) / n
        return [(min_x + (i + 0.5) * step, mid_y) for i in range(n)]

    def _build_local_scan_waypoints(self, zone: ZoneScore) -> list[tuple[float, float]]:
        waypoints: list[tuple[float, float]] = []
        y = zone.min_y
        row = 0
        while y <= zone.max_y + 1e-6:
            row_points = [(zone.min_x, y), (zone.max_x, y)]
            if row % 2:
                row_points.reverse()
            waypoints.extend(row_points)
            y += max(0.5, self.config.lane_width_m)
            row += 1
        if waypoints and not math.isclose(waypoints[-1][1], zone.max_y, abs_tol=1e-6):
            row_points = [(zone.min_x, zone.max_y), (zone.max_x, zone.max_y)]
            if row % 2:
                row_points.reverse()
            waypoints.extend(row_points)
        return waypoints

    def _reset_state(self) -> None:
        self._survey_viewpoint_index: int = 0
        self._survey_dwell_s: float = 0.0
        self.state: SearchState = SearchState.SURVEY_VIEWPOINT
        self._active_zone: ZoneScore | None = None
        self._local_waypoints: list[tuple[float, float]] = []
        self._local_waypoint_index: int = 0
        self._transit_target: tuple[float, float] | None = None
        self._target_lost_s: float = 0.0
        self._resume_state: SearchState = SearchState.SURVEY_VIEWPOINT
        self._resume_marker_str: str = "survey_viewpoint:0"
        self._last_target_id: int | None = None
        self._last_target_xy: tuple[float, float] | None = None
        self._target_cooldown_s: float = 0.0
        self._last_target_status: str = "none"
        self._last_path_status: str = "clear"
        self.max_interrupt_distance_m: float = math.inf


def _wrap_angle(angle_rad: float) -> float:
    while angle_rad > math.pi:
        angle_rad -= 2 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2 * math.pi
    return angle_rad


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))
