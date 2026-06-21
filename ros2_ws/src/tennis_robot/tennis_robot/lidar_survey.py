"""ROS 2 LiDAR-guided perimeter survey — event-driven waypoint navigation.

Sensor readings are converted to semantic events (NEAR_NET, NEAR_FENCE,
CORNER_DETECTED). Events propose an active waypoint; the robot drives to that
waypoint. Only on physical arrival is the route point committed and the section
advanced. This decouples raw sensor noise from state transitions.

See docs/survey-design-el.md and research/survey_logic.ipynb for the design.
"""

from __future__ import annotations

import json
import math
import os
import time
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path

try:
    from tennis_robot.config_utils import _env_float
except ModuleNotFoundError:
    from config_utils import _env_float
try:
    from tennis_robot.motion import TurnTracker
except ModuleNotFoundError:
    from motion import TurnTracker
try:
    from tennis_robot.survey import SurveyVision
except ModuleNotFoundError:
    from survey import SurveyVision


PROJECT_ROOT = Path(os.getenv("TENNIS_ROBOT_ROOT", "/workspace"))
DEFAULT_BOUNDARY_FILE = PROJECT_ROOT / "runtime" / "court_boundary.json"


@dataclass(frozen=True)
class BaseCommand:
    linear_speed_m_s: float
    angular_speed_rad_s: float


class LidarSurveyState(str, Enum):
    NET_STANDOFF      = "net_standoff"
    TURN_180          = "turn_180"
    BASELINE_APPROACH = "baseline_approach"
    TURN_TO_SIDELINE  = "turn_to_left_sideline"
    DRIVE_SIDELINE    = "drive_left_sideline"
    TURN_TO_LONG_SIDE = "turn_to_long_side"
    DRIVE_LONG_SIDE   = "drive_long_side"
    TURN_TO_FAR_SHORT = "turn_to_far_short_side"
    DRIVE_FAR_SHORT   = "drive_far_short_side"
    TURN_TO_RETURN    = "turn_to_return_long"
    DRIVE_RETURN      = "drive_return_long"
    DONE              = "done"


@dataclass(frozen=True)
class _ActiveWaypoint:
    """A route waypoint proposed by a sensor event. Committed on physical arrival."""
    label: str
    x_m: float
    y_m: float
    note: str
    next_state: LidarSurveyState
    event_name: str


@dataclass(frozen=True)
class LidarSurveyConfig:
    drive_speed_m_s: float = 0.60
    turn_speed_rad_s: float = 0.80
    safety_stop_range_m: float = 0.35
    safety_slow_range_m: float = 0.75
    lidar_min_range_m: float = 0.35
    lidar_max_range_m: float = 12.0
    net_standoff_m: float = 4.00
    heading_tolerance_rad: float = math.radians(2.0)
    turn_completion_tolerance_rad: float = math.radians(1.0)
    long_side_stop_range_m: float = 2.50
    short_side_stop_range_m: float = 1.20
    sideline_drive_timeout_s: float = 300.0
    sideline_sector_half_deg: float = 25.0
    waypoint_reach_m: float = 0.20
    # Parallelism guard: max heading error (rad) before a waypoint proposal is rejected
    waypoint_boundary_tolerance_rad: float = math.radians(20.0)
    # Corner geometry guard: a corner event is accepted only when one detected
    # side remains parallel to the boundary currently being followed.
    corner_parallel_tolerance_rad: float = math.radians(15.0)
    corner_parallel_min_samples: int = 6
    corner_parallel_range_std_m: float = 0.18
    # WaypointGenerator
    waypoint_lookahead_m: float = 1.50
    target_fence_m: float = 0.70
    k_lidar: float = 0.35
    k_camera: float = 1.20
    k_camera_max_m: float = 0.30
    # Minimum turn speed — prevents stall but keeps movement slow enough for
    # TurnTracker to register completion within heading_tolerance.
    turn_min_speed_rad_s: float = 0.12
    turn_parallel_search_speed_rad_s: float = 0.10
    # Minimum distance (m) the robot must travel inside a DRIVE state before a
    # front-range fence stop is accepted.  Guards against stale-scan false positives
    # from the fence that was directly ahead in the previous state (e.g. the
    # near-baseline fence that was 1.2 m ahead during BASELINE_APPROACH is still
    # visible in the front sector for the first few ticks after the 90° turn).
    min_drive_before_corner_m: float = 1.5
    # Turn timeout — prevents infinite spinning if turn completion never triggers
    turn_timeout_s: float = 30.0
    # Consecutive ticks within heading_tolerance_rad before a turn is declared complete
    turn_settle_ticks: int = 3
    # Max consecutive guard rejections before the section is finalized as failed
    guard_reject_limit: int = 60
    # Pattern validation
    expected_court_length_m: float = 23.77
    expected_court_width_m: float = 10.97
    pattern_length_tolerance_m: float = 3.0
    # Doubles detection
    doubles_alley_m: float = 1.37
    # ── Live occupancy map (real LiDAR points, sim + real world) ──────────────
    # These only feed the Sensor Views overlay; they never affect the FSM.
    map_enabled: bool = True
    map_voxel_m: float = 0.10          # downsample grid; one point kept per cell
    map_max_voxels: int = 6000         # hard cap on accumulated cells (memory bound)
    map_sample_max: int = 1500         # max points serialized into telemetry JSON
    map_publish_every: int = 5         # rebuild serialized sample every N ticks
    map_lidar_offset_x_m: float = 0.0  # LiDAR mount offset in base frame (forward+)
    map_lidar_offset_y_m: float = 0.0  # LiDAR mount offset in base frame (left+)
    map_sensor_frame: str = "laser"    # label only, shown in the panel
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
            net_standoff_m=_env_float("ROS2_SURVEY_NET_CONFIRM_STANDOFF_M", d.net_standoff_m),
            heading_tolerance_rad=_env_float("ROS2_SURVEY_HEADING_TOL_RAD", d.heading_tolerance_rad),
            turn_completion_tolerance_rad=_env_float("ROS2_SURVEY_TURN_COMPLETION_TOL_RAD", d.turn_completion_tolerance_rad),
            long_side_stop_range_m=_env_float("SURVEY_SIDELINE_DRIVE_STOP_M", d.long_side_stop_range_m),
            short_side_stop_range_m=_env_float("SURVEY_SIDELINE_SHORT_STOP_M", d.short_side_stop_range_m),
            sideline_drive_timeout_s=_env_float("SURVEY_SIDELINE_DRIVE_TIMEOUT_S", d.sideline_drive_timeout_s),
            sideline_sector_half_deg=_env_float("SURVEY_SIDELINE_SECTOR_HALF_DEG", d.sideline_sector_half_deg),
            waypoint_reach_m=_env_float("SURVEY_WAYPOINT_REACH_M", d.waypoint_reach_m),
            waypoint_boundary_tolerance_rad=_env_float("SURVEY_WAYPOINT_BOUNDARY_TOL_RAD", d.waypoint_boundary_tolerance_rad),
            corner_parallel_tolerance_rad=_env_float("SURVEY_CORNER_PARALLEL_TOL_RAD", d.corner_parallel_tolerance_rad),
            corner_parallel_min_samples=int(os.getenv("SURVEY_CORNER_PARALLEL_MIN_SAMPLES", str(d.corner_parallel_min_samples))),
            corner_parallel_range_std_m=_env_float("SURVEY_CORNER_PARALLEL_RANGE_STD_M", d.corner_parallel_range_std_m),
            waypoint_lookahead_m=_env_float("SURVEY_WAYPOINT_LOOKAHEAD_M", d.waypoint_lookahead_m),
            target_fence_m=_env_float("SURVEY_TARGET_FENCE_M", d.target_fence_m),
            k_lidar=_env_float("SURVEY_K_LIDAR", d.k_lidar),
            k_camera=_env_float("SURVEY_K_CAMERA", d.k_camera),
            k_camera_max_m=_env_float("SURVEY_K_CAMERA_MAX_M", d.k_camera_max_m),
            expected_court_length_m=_env_float("SURVEY_EXPECTED_COURT_LENGTH_M", d.expected_court_length_m),
            expected_court_width_m=_env_float("SURVEY_EXPECTED_COURT_WIDTH_M", d.expected_court_width_m),
            pattern_length_tolerance_m=_env_float("SURVEY_PATTERN_LENGTH_TOLERANCE_M", d.pattern_length_tolerance_m),
            doubles_alley_m=_env_float("SURVEY_DOUBLES_ALLEY_M", d.doubles_alley_m),
            turn_min_speed_rad_s=_env_float("SURVEY_TURN_MIN_SPEED_RAD_S", d.turn_min_speed_rad_s),
            turn_parallel_search_speed_rad_s=_env_float("SURVEY_TURN_PARALLEL_SEARCH_SPEED_RAD_S", d.turn_parallel_search_speed_rad_s),
            min_drive_before_corner_m=_env_float("SURVEY_MIN_DRIVE_BEFORE_CORNER_M", d.min_drive_before_corner_m),
            turn_timeout_s=_env_float("SURVEY_TURN_TIMEOUT_S", d.turn_timeout_s),
            turn_settle_ticks=int(os.getenv("SURVEY_TURN_SETTLE_TICKS", str(d.turn_settle_ticks))),
            guard_reject_limit=int(os.getenv("SURVEY_GUARD_REJECT_LIMIT", str(d.guard_reject_limit))),
            map_enabled=os.getenv("SURVEY_MAP_ENABLED", "1" if d.map_enabled else "0") not in ("0", "false", "False"),
            map_voxel_m=_env_float("SURVEY_MAP_VOXEL_M", d.map_voxel_m),
            map_max_voxels=int(os.getenv("SURVEY_MAP_MAX_VOXELS", str(d.map_max_voxels))),
            map_sample_max=int(os.getenv("SURVEY_MAP_SAMPLE_MAX", str(d.map_sample_max))),
            map_publish_every=int(os.getenv("SURVEY_MAP_PUBLISH_EVERY", str(d.map_publish_every))),
            map_lidar_offset_x_m=_env_float("SURVEY_MAP_LIDAR_OFFSET_X_M", d.map_lidar_offset_x_m),
            map_lidar_offset_y_m=_env_float("SURVEY_MAP_LIDAR_OFFSET_Y_M", d.map_lidar_offset_y_m),
            map_sensor_frame=os.getenv("SURVEY_MAP_SENSOR_FRAME", d.map_sensor_frame),
            output_file=Path(os.getenv("SURVEY_OUTPUT_FILE", str(d.output_file))),
        )


@dataclass(frozen=True)
class LidarSurveyCommand:
    state: LidarSurveyState
    base: BaseCommand
    sample_count: int


# ─── Waypoint generator ────────────────────────────────────────────────────────

class WaypointGenerator:
    """Compute the next navigation waypoint from sensor readings.

    Projects LOOKAHEAD_M ahead of the robot in its current heading, then
    applies a lateral offset derived from:
      - LiDAR: distance from the side fence vs. TARGET_FENCE_M
      - Camera: court line offset (when available)

    The result is a (x, y) point in world frame that the robot steers toward.
    """

    def __init__(self, config: LidarSurveyConfig) -> None:
        self._cfg = config

    def compute(
        self,
        x_m: float,
        y_m: float,
        yaw_rad: float,
        lidar_ranges: list[float],
        lidar_angle_min: float,
        lidar_angle_increment: float,
        vision: SurveyVision | None,
        side_center_rad: float,
        use_camera: bool = True,
    ) -> tuple[float, float]:
        fwd_x = x_m + math.cos(yaw_rad) * self._cfg.waypoint_lookahead_m
        fwd_y = y_m + math.sin(yaw_rad) * self._cfg.waypoint_lookahead_m

        lateral = 0.0
        side_range = self._sector_median(
            lidar_ranges, lidar_angle_min, lidar_angle_increment,
            side_center_rad, math.radians(self._cfg.sideline_sector_half_deg),
        )
        if math.isfinite(side_range) and self._cfg.lidar_min_range_m <= side_range <= self._cfg.lidar_max_range_m:
            side_sign = -1.0 if side_center_rad < 0.0 else 1.0
            lateral += side_sign * (side_range - self._cfg.target_fence_m) * self._cfg.k_lidar

        if use_camera and vision is not None and vision.line_detected and vision.line_offset_m is not None:
            cam = vision.line_offset_m * self._cfg.k_camera
            cam = max(-self._cfg.k_camera_max_m, min(self._cfg.k_camera_max_m, cam))
            lateral += cam

        left_x = -math.sin(yaw_rad)
        left_y = math.cos(yaw_rad)
        return (fwd_x + left_x * lateral, fwd_y + left_y * lateral)

    def _sector_median(
        self,
        ranges: list[float],
        angle_min: float,
        angle_increment: float,
        center_rad: float,
        half_rad: float,
    ) -> float:
        vals: list[float] = []
        for i, r in enumerate(ranges):
            if not math.isfinite(r) or r < self._cfg.lidar_min_range_m or r > self._cfg.lidar_max_range_m:
                continue
            angle = angle_min + i * angle_increment
            angle = (angle + math.pi) % (2.0 * math.pi) - math.pi
            if center_rad - half_rad <= angle <= center_rad + half_rad:
                vals.append(r)
        if not vals:
            return math.inf
        vals.sort()
        return vals[len(vals) // 2]


# ─── Main survey class ─────────────────────────────────────────────────────────

class Ros2LidarCourtSurvey:
    def __init__(self, config: LidarSurveyConfig | None = None) -> None:
        self.config = config or LidarSurveyConfig()
        self.state = LidarSurveyState.NET_STANDOFF
        self.sample_count = 0
        self._waypoint_gen = WaypointGenerator(self.config)

        # Initial heading recorded on the very first NET_STANDOFF tick — used as the
        # side-wall reference for the 180° turn so that any last-moment angular
        # correction near the net does not corrupt the baseline heading.
        self._net_approach_yaw: float | None = None

        # Headings set when committing a waypoint before each TURN state
        self._turn_180_target: float | None = None
        self._baseline_heading: float | None = None
        self._sideline_heading: float | None = None
        self._long_side_heading: float | None = None
        self._far_short_heading: float | None = None
        self._return_heading: float | None = None

        # Side-fence LiDAR samples
        self._left_range_samples: list[float] = []
        self._right_range_samples: list[float] = []
        self._long_left_range_samples: list[float] = []
        self._far_short_range_samples: list[float] = []
        self._return_range_samples: list[float] = []

        # Camera line crossings → boundary-to-fence distances
        self._near_baseline_to_fence_m: float | None = None
        self._left_sideline_to_fence_m: float | None = None
        self._left_sideline_line_crossed: bool = False
        self._far_baseline_to_fence_m: float | None = None
        self._far_baseline_crossed: bool = False
        self._right_sideline_to_fence_m: float | None = None
        self._right_sideline_line_crossed: bool = False

        # Corner waypoints — the route signature validated at the end
        self._survey_navigation_points: list[dict] = []

        # Active waypoint: proposed by a sensor event, committed on physical arrival.
        # While set, the robot drives toward it; new events for the same section are ignored.
        self._active_wp: _ActiveWaypoint | None = None

        # Runtime state
        self._state_elapsed_s = 0.0
        self._started_at: float | None = None
        self._last_event = "none"
        self._failure_reason: str | None = None
        self._court_bounds: dict | None = None
        self._last_front_range_m = math.inf
        self._last_pose: tuple[float, float] | None = None
        self._distance_traveled_m = 0.0
        # Distance at state entry — used to gate corner detection until the robot has
        # moved far enough to guarantee the stop isn't a stale-scan false positive from
        # the fence that was directly ahead in the previous state (e.g. after a 90° turn,
        # the previous fence is 1.2m to the side but may still appear in the front sector).
        self._state_entry_distance_m = 0.0
        self._lidar_angle_min: float = -math.pi
        self._lidar_angle_increment: float = 2.0 * math.pi / 360
        self._last_scan_ranges: list[float] = []
        self._turn_tracker = TurnTracker()
        self._pending_turn_heading_reached: bool = False
        self._guard_reject_count: int = 0
        self._last_corner_reject_reason: str | None = None

        # ── Live occupancy map (isolated from the FSM) ───────────────────────────
        # Voxel grid of world-frame LiDAR hits. key=(ix,iy) -> (x_m, y_m).
        self._map_voxels: dict[tuple[int, int], tuple[float, float]] = {}
        self._map_sample_cache: list[dict] = []
        self._map_extents_cache: dict | None = None
        self._map_full_warned: bool = False
        self._map_tick: int = 0

    @classmethod
    def from_env(cls) -> "Ros2LidarCourtSurvey":
        return cls(LidarSurveyConfig.from_env())

    def reset(self) -> None:
        self.__init__(self.config)

    @property
    def court_bounds(self) -> dict | None:
        return self._court_bounds

    # ── Public update ──────────────────────────────────────────────────────────

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
        if self.state == LidarSurveyState.DONE:
            return self._cmd(BaseCommand(0.0, 0.0))

        self._dt_s = max(0.0, dt_s)
        self._state_elapsed_s += self._dt_s
        self._update_distance(x_m, y_m)
        self.sample_count += 1

        if lidar_ranges:
            n = len(lidar_ranges)
            self._lidar_angle_min = lidar_angle_min
            self._lidar_angle_increment = (
                lidar_angle_increment if lidar_angle_increment is not None
                else 2.0 * math.pi / max(1, n)
            )
            self._last_scan_ranges = lidar_ranges
            # Isolated occupancy mapping — uses the same scan but never the FSM.
            if self.config.map_enabled:
                self._accumulate_map_points(
                    x_m, y_m, yaw_rad, lidar_ranges,
                    self._lidar_angle_min, self._lidar_angle_increment,
                )

        self._last_front_range_m = self._front_range(self._last_scan_ranges or None)

        command = self._step(x_m, y_m, yaw_rad, vision)
        return self._cmd(self._apply_safety(command))

    # ── Telemetry ──────────────────────────────────────────────────────────────

    # ── Live occupancy map (isolated from the FSM) ──────────────────────────────

    def _accumulate_map_points(
        self,
        x_m: float,
        y_m: float,
        yaw_rad: float,
        ranges: list[float],
        angle_min: float,
        angle_increment: float,
    ) -> None:
        """Project the current scan into world frame and voxel-accumulate it.

        Read-only with respect to survey state: it touches only the map buffers.
        Identical math to the FSM's scan interpretation (sensor +x = forward),
        so it behaves the same in simulation and on the physical RPLIDAR — the
        live angle_min/increment from the actual scan drive the projection.
        """
        cfg = self.config
        if not ranges or angle_increment in (None, 0.0):
            return
        voxels = self._map_voxels
        voxel = cfg.map_voxel_m if cfg.map_voxel_m > 1e-3 else 0.10
        cap = cfg.map_max_voxels
        cos_y = math.cos(yaw_rad)
        sin_y = math.sin(yaw_rad)
        off_x = cfg.map_lidar_offset_x_m
        off_y = cfg.map_lidar_offset_y_m
        rmin = cfg.lidar_min_range_m
        rmax = cfg.lidar_max_range_m
        full = len(voxels) >= cap

        for i, r in enumerate(ranges):
            if not (isinstance(r, (int, float)) and math.isfinite(r)):
                continue
            if r < rmin or r > rmax:
                continue
            angle = angle_min + i * angle_increment
            # Point in sensor frame, then mount offset, both in base frame.
            sx = r * math.cos(angle) + off_x
            sy = r * math.sin(angle) + off_y
            # Rotate by robot yaw and translate to world.
            wx = x_m + sx * cos_y - sy * sin_y
            wy = y_m + sx * sin_y + sy * cos_y
            key = (int(math.floor(wx / voxel)), int(math.floor(wy / voxel)))
            if key in voxels:
                continue
            if full:
                if not self._map_full_warned:
                    self._map_full_warned = True
                continue
            voxels[key] = (wx, wy)
            if len(voxels) >= cap:
                full = True

        self._map_tick += 1
        if self._map_tick % max(1, cfg.map_publish_every) == 0:
            self._rebuild_map_cache()

    def _rebuild_map_cache(self) -> None:
        """Rebuild the serialized point sample + world extents (throttled)."""
        pts = list(self._map_voxels.values())
        if not pts:
            self._map_sample_cache = []
            self._map_extents_cache = None
            return
        sample_max = max(1, self.config.map_sample_max)
        if len(pts) > sample_max:
            step = len(pts) / sample_max
            pts_sample = [pts[int(k * step)] for k in range(sample_max)]
        else:
            pts_sample = pts
        self._map_sample_cache = [
            {"x_m": round(px, 3), "y_m": round(py, 3)} for px, py in pts_sample
        ]
        xs = [pt[0] for pt in pts]
        ys = [pt[1] for pt in pts]
        self._map_extents_cache = {
            "min_x_m": round(min(xs), 3),
            "max_x_m": round(max(xs), 3),
            "min_y_m": round(min(ys), 3),
            "max_y_m": round(max(ys), 3),
        }

    def _scan_clearance(self, center_rad: float, half_rad: float = math.radians(15.0)) -> float | None:
        """Nearest finite range within a sector of the latest scan (display only)."""
        ranges = self._last_scan_ranges
        if not ranges:
            return None
        amin = self._lidar_angle_min
        ainc = self._lidar_angle_increment
        best = math.inf
        for i, r in enumerate(ranges):
            if not (isinstance(r, (int, float)) and math.isfinite(r)):
                continue
            if r < self.config.lidar_min_range_m or r > self.config.lidar_max_range_m:
                continue
            angle = amin + i * ainc
            # Circular angular distance — correct even across the ±pi seam (rear).
            delta = abs((angle - center_rad + math.pi) % (2.0 * math.pi) - math.pi)
            if delta <= half_rad and r < best:
                best = r
        return None if math.isinf(best) else round(best, 3)

    def _scan_coverage(self) -> dict:
        return {
            "world_extents": self._map_extents_cache,
            "front_m": self._scan_clearance(0.0),
            "rear_m": self._scan_clearance(math.pi),
            "left_m": self._scan_clearance(math.pi / 2),
            "right_m": self._scan_clearance(-math.pi / 2),
        }

    def telemetry(self) -> dict:
        elapsed = 0.0 if self._started_at is None else time.time() - self._started_at
        return {
            "state": self.state.value,
            "navigation_source": "waypoint_driven_perimeter_survey",
            "last_event": self._last_event,
            "failure_reason": self._failure_reason,
            "sample_count": self.sample_count,
            "front_lidar_range_m": (
                None if math.isinf(self._last_front_range_m)
                else round(self._last_front_range_m, 3)
            ),
            "distance_traveled_m": round(self._distance_traveled_m, 2),
            "elapsed_s": round(elapsed, 1),
            "state_elapsed_s": round(self._state_elapsed_s, 1),
            "active_waypoint": (
                {"label": self._active_wp.label,
                 "x_m": self._active_wp.x_m,
                 "y_m": self._active_wp.y_m,
                 "next_state": self._active_wp.next_state.value}
                if self._active_wp else None
            ),
            "sideline_heading_deg": (
                None if self._sideline_heading is None
                else round(math.degrees(self._sideline_heading) % 360, 1)
            ),
            "long_side_heading_deg": (
                None if self._long_side_heading is None
                else round(math.degrees(self._long_side_heading) % 360, 1)
            ),
            "turn_progress_deg": round(math.degrees(self._turn_tracker.progress_rad), 1),
            "guard_reject_count": self._guard_reject_count,
            "last_corner_reject_reason": self._last_corner_reject_reason,
            "near_baseline_to_fence_m": self._near_baseline_to_fence_m,
            "left_sideline_to_fence_m": self._left_sideline_to_fence_m,
            "far_baseline_to_fence_m": self._far_baseline_to_fence_m,
            "right_sideline_to_fence_m": self._right_sideline_to_fence_m,
            "left_fence_sample_count": len(self._left_range_samples),
            "long_left_fence_sample_count": len(self._long_left_range_samples),
            "survey_navigation_points": self._survey_navigation_points_sample(),
            "survey_route": self._survey_route_points(),
            "survey_navigation_point_count": len(self._survey_navigation_points),
            "survey_pattern": self._survey_pattern_status(),
            # Live occupancy map for the Sensor Views overlay (real LiDAR hits).
            "map_points": self._map_sample_cache,
            "map_point_count": len(self._map_voxels),
            "scan_coverage": self._scan_coverage(),
            "sensor_frame": self.config.map_sensor_frame,
        }

    # ── State machine ──────────────────────────────────────────────────────────

    def _step(self, x_m: float, y_m: float, yaw_rad: float, vision: SurveyVision | None) -> BaseCommand:
        # Universal: if an active waypoint is pending, navigate to it.
        # New sensor events are ignored while a waypoint is in-flight.
        if self._active_wp is not None:
            dist = math.hypot(x_m - self._active_wp.x_m, y_m - self._active_wp.y_m)
            if dist <= self.config.waypoint_reach_m:
                self._commit_active_waypoint(yaw_rad)
            else:
                # Small course correction only — angular gain halved vs. normal driving
                # so the robot does not oversteer toward the committed waypoint.
                return self._drive_to_waypoint(
                    x_m, y_m, yaw_rad,
                    (self._active_wp.x_m, self._active_wp.y_m),
                    angular_gain=0.9,
                )
            return BaseCommand(0.0, 0.0)

        # ① net standoff — drive forward until LiDAR detects net
        if self.state == LidarSurveyState.NET_STANDOFF:
            # Capture the initial heading on the very first tick as the side-wall
            # parallel reference — before any proximity corrections near the net.
            if self._net_approach_yaw is None:
                self._net_approach_yaw = yaw_rad
            front = self._last_front_range_m
            if not math.isinf(front) and front <= self.config.net_standoff_m:
                self._propose_waypoint(
                    "near_net_standoff", x_m, y_m, "net standoff reached",
                    LidarSurveyState.TURN_180, "net_standoff_reached",
                )
                return BaseCommand(0.0, 0.0)
            return BaseCommand(self.config.drive_speed_m_s, 0.0)

        # 180° rotation toward baseline
        if self.state == LidarSurveyState.TURN_180:
            if self._state_elapsed_s >= self.config.turn_timeout_s:
                self._finalize_full_survey("turn_180_timeout")
                return BaseCommand(0.0, 0.0)
            if self._turn_180_target is None:
                self._fail("TURN_180_TARGET_NOT_SET")
                return BaseCommand(0.0, 0.0)
            if self._turn_complete(yaw_rad, self._turn_180_target, math.pi):
                self._enter(LidarSurveyState.BASELINE_APPROACH, "turn_180_complete")
                return BaseCommand(0.0, 0.0)
            err = self._angle_delta(self._turn_180_target, yaw_rad)
            # Near the antipode (err ≈ ±π) the sign is ambiguous — force the tracker's direction
            if abs(err) > math.pi - math.radians(20.0) and self._turn_tracker.direction != 0.0:
                err = abs(err) * self._turn_tracker.direction
            return BaseCommand(0.0, self._proportional_turn(err))

        # ② drive to near baseline fence
        if self.state == LidarSurveyState.BASELINE_APPROACH:
            if self._state_elapsed_s >= self.config.sideline_drive_timeout_s:
                self._finalize_full_survey("baseline_approach_timeout")
                return BaseCommand(0.0, 0.0)
            if (
                vision is not None
                and vision.line_detected
                and self._near_baseline_to_fence_m is None
                and not math.isinf(self._last_front_range_m)
            ):
                self._near_baseline_to_fence_m = round(self._last_front_range_m, 3)
            front = self._last_front_range_m
            if not math.isinf(front) and front <= self.config.short_side_stop_range_m:
                if self._propose_waypoint(
                    "near_baseline_fence_standoff", x_m, y_m, "near baseline fence reached",
                    LidarSurveyState.TURN_TO_SIDELINE, "baseline_fence_reached",
                    yaw_rad=yaw_rad, expected_heading_rad=self._baseline_heading,
                ):
                    return BaseCommand(0.0, 0.0)
                return self._guard_recovery(yaw_rad, self._baseline_heading)
            err = self._angle_delta(self._baseline_heading if self._baseline_heading is not None else yaw_rad, yaw_rad)
            turn = max(-self.config.turn_speed_rad_s * 0.5, min(self.config.turn_speed_rad_s * 0.5, err * 1.8))
            return BaseCommand(self.config.drive_speed_m_s, turn)

        # 90° left toward sideline
        if self.state == LidarSurveyState.TURN_TO_SIDELINE:
            print(f"TURN_TO_SIDELINE: yaw={math.degrees(yaw_rad):.1f}°, target={math.degrees(self._sideline_heading or 0):.1f}°")

            if self._state_elapsed_s >= self.config.turn_timeout_s:
                print("TURN_TO_SIDELINE: turn timeout")
                self._finalize_full_survey("turn_to_sideline_timeout")
                return BaseCommand(0.0, 0.0)
            if self._sideline_heading is None:
                print("TURN_TO_SIDELINE: sideline heading not set")
                self._fail("SIDELINE_HEADING_NOT_SET")
                return BaseCommand(0.0, 0.0)
            if self._turn_complete_with_parallel_boundary(yaw_rad, self._sideline_heading, math.radians(90.0), vision):
                print("TURN_TO_SIDELINE: turn complete")
                self._enter(LidarSurveyState.DRIVE_SIDELINE, "aligned_for_sideline_drive")
                return BaseCommand(0.0, 0.0)
            if self._pending_turn_heading_reached:
                return self._turn_parallel_search_command()
            err = self._angle_delta(self._sideline_heading, yaw_rad)
            print(f"TURN_TO_SIDELINE: angle error: {math.degrees(err):.1f}°")
            return BaseCommand(0.0, self._proportional_turn(err))

        # ③ drive short side (near-baseline fence leg)
        if self.state == LidarSurveyState.DRIVE_SIDELINE:
            if self._state_elapsed_s >= self.config.sideline_drive_timeout_s:
                self._finalize_full_survey("sideline_drive_timeout")
                return BaseCommand(0.0, 0.0)
            self._sample_side_ranges(
                self._left_range_samples, self._right_range_samples,
                left_cap=300, right_cap=300,
            )
            if (
                vision is not None and vision.line_detected
                and not self._left_sideline_line_crossed
                and not math.isinf(self._last_front_range_m)
            ):
                self._left_sideline_to_fence_m = round(self._last_front_range_m, 3)
                self._left_sideline_line_crossed = True
                self._add_survey_navigation_point(
                    "left_sideline_confirmed", x_m, y_m, "inner sideline confirmed"
                )
            front = self._last_front_range_m
            if self._min_drive_cleared() and not math.isinf(front) and front <= self.config.short_side_stop_range_m:
                if self._propose_waypoint(
                    "near_left_fence_corner", x_m, y_m, "near-left fence corner reached",
                    LidarSurveyState.TURN_TO_LONG_SIDE, "side_fence_reached",
                    yaw_rad=yaw_rad, expected_heading_rad=self._sideline_heading,
                    vision=vision, require_boundary_parallel=True,
                    boundary_samples=self._left_range_samples,
                ):
                    return BaseCommand(0.0, 0.0)
                return self._guard_recovery(yaw_rad, self._sideline_heading)
            wp = self._waypoint_gen.compute(
                x_m, y_m, yaw_rad,
                self._last_scan_ranges, self._lidar_angle_min, self._lidar_angle_increment,
                vision, side_center_rad=-math.pi / 2, use_camera=True,
            )
            return self._drive_to_waypoint(x_m, y_m, yaw_rad, wp)

        # 90° left toward long side
        if self.state == LidarSurveyState.TURN_TO_LONG_SIDE:
            if self._state_elapsed_s >= self.config.turn_timeout_s:
                self._finalize_full_survey("turn_to_long_side_timeout")
                return BaseCommand(0.0, 0.0)
            if self._long_side_heading is None:
                self._fail("LONG_SIDE_HEADING_NOT_SET")
                return BaseCommand(0.0, 0.0)
            if self._turn_complete_with_parallel_boundary(yaw_rad, self._long_side_heading, math.radians(90.0), vision):
                self._enter(LidarSurveyState.DRIVE_LONG_SIDE, "aligned_for_long_side_drive")
                return BaseCommand(0.0, 0.0)
            if self._pending_turn_heading_reached:
                return self._turn_parallel_search_command()
            err = self._angle_delta(self._long_side_heading, yaw_rad)
            return BaseCommand(0.0, self._proportional_turn(err))

        # ④ drive long side — 80th-pct front range sees through net
        if self.state == LidarSurveyState.DRIVE_LONG_SIDE:
            if self._state_elapsed_s >= self.config.sideline_drive_timeout_s:
                self._finalize_full_survey("long_side_drive_timeout")
                return BaseCommand(0.0, 0.0)
            self._sample_side_ranges(self._long_left_range_samples, cap_left=400)
            if (
                vision is not None and vision.line_detected
                and not self._far_baseline_crossed
                and not math.isinf(self._last_front_range_m)
            ):
                self._far_baseline_to_fence_m = round(self._last_front_range_m, 3)
                self._far_baseline_crossed = True
                self._add_survey_navigation_point(
                    "far_baseline_confirmed", x_m, y_m, "far baseline confirmed"
                )
            front = self._through_front_range()
            if self._min_drive_cleared() and not math.isinf(front) and front <= self.config.long_side_stop_range_m:
                obs = (vision.obstacle_class or "").strip().lower() if vision else ""
                if obs not in {"net", "net_post", "post", "posts"}:
                    if self._propose_waypoint(
                        "far_left_fence_corner", x_m, y_m, "far-left fence corner reached",
                        LidarSurveyState.TURN_TO_FAR_SHORT, "far_baseline_fence_reached",
                        yaw_rad=yaw_rad, expected_heading_rad=self._long_side_heading,
                        vision=vision, require_boundary_parallel=True,
                        boundary_samples=self._long_left_range_samples,
                    ):
                        return BaseCommand(0.0, 0.0)
                    return self._guard_recovery(yaw_rad, self._long_side_heading)
            wp = self._waypoint_gen.compute(
                x_m, y_m, yaw_rad,
                self._last_scan_ranges, self._lidar_angle_min, self._lidar_angle_increment,
                vision, side_center_rad=-math.pi / 2, use_camera=False,
            )
            return self._drive_to_waypoint(x_m, y_m, yaw_rad, wp)

        # 90° left toward far short side
        if self.state == LidarSurveyState.TURN_TO_FAR_SHORT:
            if self._state_elapsed_s >= self.config.turn_timeout_s:
                self._finalize_full_survey("turn_to_far_short_timeout")
                return BaseCommand(0.0, 0.0)
            if self._far_short_heading is None:
                self._fail("FAR_SHORT_HEADING_NOT_SET")
                return BaseCommand(0.0, 0.0)
            if self._turn_complete_with_parallel_boundary(yaw_rad, self._far_short_heading, math.radians(90.0), vision):
                self._enter(LidarSurveyState.DRIVE_FAR_SHORT, "aligned_for_far_short_drive")
                return BaseCommand(0.0, 0.0)
            if self._pending_turn_heading_reached:
                return self._turn_parallel_search_command()
            err = self._angle_delta(self._far_short_heading, yaw_rad)
            return BaseCommand(0.0, self._proportional_turn(err))

        # ⑤ drive far short side
        if self.state == LidarSurveyState.DRIVE_FAR_SHORT:
            if self._state_elapsed_s >= self.config.sideline_drive_timeout_s:
                self._finalize_full_survey("far_short_drive_timeout")
                return BaseCommand(0.0, 0.0)
            self._sample_side_ranges(self._far_short_range_samples, cap_left=300)
            if (
                vision is not None and vision.line_detected
                and not self._right_sideline_line_crossed
                and not math.isinf(self._last_front_range_m)
            ):
                self._right_sideline_to_fence_m = round(self._last_front_range_m, 3)
                self._right_sideline_line_crossed = True
                self._add_survey_navigation_point(
                    "right_sideline_confirmed", x_m, y_m, "far sideline confirmed"
                )
            front = self._last_front_range_m
            if self._min_drive_cleared() and not math.isinf(front) and front <= self.config.short_side_stop_range_m:
                if self._propose_waypoint(
                    "far_right_fence_corner", x_m, y_m, "far-right fence corner reached",
                    LidarSurveyState.TURN_TO_RETURN, "far_side_fence_reached",
                    yaw_rad=yaw_rad, expected_heading_rad=self._far_short_heading,
                    vision=vision, require_boundary_parallel=True,
                    boundary_samples=self._far_short_range_samples,
                ):
                    return BaseCommand(0.0, 0.0)
                return self._guard_recovery(yaw_rad, self._far_short_heading)
            wp = self._waypoint_gen.compute(
                x_m, y_m, yaw_rad,
                self._last_scan_ranges, self._lidar_angle_min, self._lidar_angle_increment,
                vision, side_center_rad=-math.pi / 2, use_camera=True,
            )
            return self._drive_to_waypoint(x_m, y_m, yaw_rad, wp)

        # 90° left toward return long side
        if self.state == LidarSurveyState.TURN_TO_RETURN:
            if self._state_elapsed_s >= self.config.turn_timeout_s:
                self._finalize_full_survey("turn_to_return_timeout")
                return BaseCommand(0.0, 0.0)
            if self._return_heading is None:
                self._fail("RETURN_HEADING_NOT_SET")
                return BaseCommand(0.0, 0.0)
            if self._turn_complete_with_parallel_boundary(yaw_rad, self._return_heading, math.radians(90.0), vision):
                self._enter(LidarSurveyState.DRIVE_RETURN, "aligned_for_return_drive")
                return BaseCommand(0.0, 0.0)
            if self._pending_turn_heading_reached:
                return self._turn_parallel_search_command()
            err = self._angle_delta(self._return_heading, yaw_rad)
            return BaseCommand(0.0, self._proportional_turn(err))

        # ⑥ return long side — 80th-pct front range sees through net
        if self.state == LidarSurveyState.DRIVE_RETURN:
            if self._state_elapsed_s >= self.config.sideline_drive_timeout_s:
                self._finalize_full_survey("return_drive_timeout")
                return BaseCommand(0.0, 0.0)
            self._sample_side_ranges(self._return_range_samples, cap_left=400)
            front = self._through_front_range()
            if self._min_drive_cleared() and not math.isinf(front) and front <= self.config.long_side_stop_range_m:
                obs = (vision.obstacle_class or "").strip().lower() if vision else ""
                if obs not in {"net", "net_post", "post", "posts"}:
                    if self._propose_waypoint(
                        "near_right_fence_corner", x_m, y_m, "near-right fence corner reached",
                        LidarSurveyState.DONE, "return_baseline_fence_reached",
                        yaw_rad=yaw_rad, expected_heading_rad=self._return_heading,
                        vision=vision, require_boundary_parallel=True,
                        boundary_samples=self._return_range_samples,
                    ):
                        return BaseCommand(0.0, 0.0)
                    return self._guard_recovery(yaw_rad, self._return_heading)
            wp = self._waypoint_gen.compute(
                x_m, y_m, yaw_rad,
                self._last_scan_ranges, self._lidar_angle_min, self._lidar_angle_increment,
                vision, side_center_rad=-math.pi / 2, use_camera=False,
            )
            return self._drive_to_waypoint(x_m, y_m, yaw_rad, wp)

        return BaseCommand(0.0, 0.0)

    # ── Active waypoint helpers ────────────────────────────────────────────────

    def _propose_waypoint(
        self,
        label: str,
        x_m: float,
        y_m: float,
        note: str,
        next_state: LidarSurveyState,
        event_name: str,
        yaw_rad: float = 0.0,
        expected_heading_rad: float | None = None,
        vision: SurveyVision | None = None,
        require_boundary_parallel: bool = False,
        boundary_samples: list[float] | None = None,
    ) -> bool:
        """Propose a route waypoint from a sensor event.

        Returns False (and ignores the proposal) if:
        - a waypoint is already pending (prevents sensor noise re-triggering), or
        - the robot's heading deviates more than waypoint_boundary_tolerance_rad from
          the expected section heading (parallelism guard: a non-parallel approach
          means the detection is unreliable).
        """
        if self._active_wp is not None:
            return False
        if expected_heading_rad is not None:
            heading_err = abs(self._angle_delta(expected_heading_rad, yaw_rad))
            if heading_err > self.config.waypoint_boundary_tolerance_rad:
                self._last_corner_reject_reason = f"robot_heading_not_parallel:{math.degrees(heading_err):.1f}deg"
                return False
        if require_boundary_parallel and not self._corner_has_parallel_boundary_side(vision, boundary_samples):
            return False
        self._active_wp = _ActiveWaypoint(label, x_m, y_m, note, next_state, event_name)
        self._guard_reject_count = 0
        self._last_corner_reject_reason = None
        return True

    def _corner_has_parallel_boundary_side(
        self,
        vision: SurveyVision | None,
        boundary_samples: list[float] | None,
    ) -> bool:
        """Validate that a corner candidate includes one boundary-parallel side."""
        if vision is not None and vision.line_detected and vision.line_heading_error_rad is not None:
            line_err = abs(self._angle_delta(0.0, vision.line_heading_error_rad))
            if line_err <= self.config.corner_parallel_tolerance_rad:
                return True
            self._last_corner_reject_reason = f"boundary_line_not_parallel:{math.degrees(line_err):.1f}deg"
            return False

        samples = [r for r in (boundary_samples or []) if math.isfinite(r)]
        min_samples = max(2, self.config.corner_parallel_min_samples)
        if len(samples) >= min_samples:
            recent = samples[-min_samples:]
            mean = sum(recent) / len(recent)
            variance = sum((r - mean) ** 2 for r in recent) / len(recent)
            if math.sqrt(variance) <= self.config.corner_parallel_range_std_m:
                return True
            self._last_corner_reject_reason = "side_lidar_boundary_not_stable"
            return False

        self._last_corner_reject_reason = "missing_parallel_boundary_evidence"
        return False

    def _commit_active_waypoint(self, yaw_rad: float) -> None:
        """Called when the robot physically arrives at the active waypoint.

        Records the route point, configures the heading for the upcoming turn,
        then advances to the next section.
        """
        wp = self._active_wp
        if wp is None:
            return
        self._active_wp = None

        self._add_survey_navigation_point(wp.label, wp.x_m, wp.y_m, wp.note)

        # Configure heading for the next turn section.
        # For TURN_180 specifically, use the heading recorded at the very start of the
        # net approach (not the instantaneous commit yaw) so that any angular drift or
        # last-moment correction near the net does not corrupt the baseline reference.
        if wp.next_state == LidarSurveyState.TURN_180:
            self._turn_180_target = yaw_rad + math.pi
            self._baseline_heading = (self._net_approach_yaw if self._net_approach_yaw is not None else yaw_rad) + math.pi
        elif wp.next_state == LidarSurveyState.TURN_TO_SIDELINE:
            self._sideline_heading = (self._baseline_heading if self._baseline_heading is not None else yaw_rad) + math.pi / 2
        elif wp.next_state == LidarSurveyState.TURN_TO_LONG_SIDE:
            self._long_side_heading = (self._sideline_heading if self._sideline_heading is not None else 0.0) + math.pi / 2
        elif wp.next_state == LidarSurveyState.TURN_TO_FAR_SHORT:
            self._far_short_heading = (self._long_side_heading if self._long_side_heading is not None else 0.0) + math.pi / 2
        elif wp.next_state == LidarSurveyState.TURN_TO_RETURN:
            self._return_heading = (self._far_short_heading if self._far_short_heading is not None else 0.0) + math.pi / 2

        if wp.next_state == LidarSurveyState.DONE:
            self._finalize_full_survey(None)
        else:
            self._enter(wp.next_state, wp.event_name)

    # ── Navigation helpers ─────────────────────────────────────────────────────

    def _drive_to_waypoint(
        self,
        x_m: float,
        y_m: float,
        yaw_rad: float,
        wp: tuple[float, float],
        angular_gain: float = 1.8,
    ) -> BaseCommand:
        dx = wp[0] - x_m
        dy = wp[1] - y_m
        desired = math.atan2(dy, dx)
        err = self._angle_delta(desired, yaw_rad)
        cap = self.config.turn_speed_rad_s * 0.5
        turn = max(-cap, min(cap, err * angular_gain))
        return BaseCommand(self.config.drive_speed_m_s, turn)

    def _min_drive_cleared(self) -> bool:
        """True once the robot has moved min_drive_before_corner_m since entering this state."""
        return (self._distance_traveled_m - self._state_entry_distance_m) >= self.config.min_drive_before_corner_m

    def _proportional_turn(self, err: float) -> float:
        """Angular speed with proportional slowdown near target.

        Full speed until ~10° from target, then ramps down to turn_min_speed_rad_s.
        Prevents overshoot oscillation with bang-bang control at ~2.7°/tick.
        """
        gain = self.config.turn_speed_rad_s / math.radians(10.0)
        speed = max(self.config.turn_min_speed_rad_s, min(self.config.turn_speed_rad_s, abs(err) * gain))
        return math.copysign(speed, err)

    def _through_front_range(self) -> float:
        """80th-pct front range — sees through sparse net, stops at solid fence."""
        if not self._last_scan_ranges:
            return math.inf
        return self._sector_median_range(
            self._last_scan_ranges, 0.0, math.radians(20.0), pct=0.80
        )

    def _sector_median_range(
        self,
        ranges: list[float],
        center_rad: float,
        half_rad: float,
        pct: float = 0.50,
    ) -> float:
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
        idx = min(len(vals) - 1, int(round((len(vals) - 1) * pct)))
        return vals[idx]

    def _sample_side_ranges(
        self,
        left_buf: list[float],
        right_buf: list[float] | None = None,
        cap_left: int = 300,
        left_cap: int | None = None,
        right_cap: int = 300,
    ) -> None:
        cap = left_cap if left_cap is not None else cap_left
        if not self._last_scan_ranges:
            return
        half = math.radians(self.config.sideline_sector_half_deg)
        left_r = self._sector_median_range(self._last_scan_ranges, -math.pi / 2, half)
        if math.isfinite(left_r):
            left_buf.append(left_r)
            if len(left_buf) > cap:
                del left_buf[: cap // 2]
        if right_buf is not None:
            right_r = self._sector_median_range(self._last_scan_ranges, math.pi / 2, half)
            if math.isfinite(right_r):
                right_buf.append(right_r)
                if len(right_buf) > right_cap:
                    del right_buf[: right_cap // 2]

    def _guard_recovery(self, yaw_rad: float, section_heading: float | None) -> BaseCommand:
        """Command issued when the parallelism guard rejects a waypoint proposal.

        Rotates toward section_heading when close to the fence; otherwise drives
        slowly with heading correction. Finalizes the section as failed after
        guard_reject_limit consecutive rejections without improvement.
        """
        self._guard_reject_count += 1
        if self._guard_reject_count > self.config.guard_reject_limit:
            self._finalize_full_survey(f"{self.state.value}_unrecoverable_heading")
            return BaseCommand(0.0, 0.0)
        if section_heading is None:
            return BaseCommand(self.config.drive_speed_m_s * 0.5, 0.0)
        err = self._angle_delta(section_heading, yaw_rad)
        if self._last_front_range_m <= self.config.safety_slow_range_m:
            return BaseCommand(0.0, self._proportional_turn(err))
        cap = self.config.turn_speed_rad_s * 0.5
        turn = max(-cap, min(cap, err * 1.8))
        return BaseCommand(self.config.drive_speed_m_s * 0.5, turn)

    def _turn_complete(self, yaw_rad: float, target_heading: float, target_delta_rad: float) -> bool:
        if self._turn_tracker.target_heading_rad is None:
            # Compute required angle from actual entry yaw for correct direction/magnitude.
            required = self._angle_delta(target_heading, yaw_rad)
            # Near-180° turns: ±π is the antipodal singularity where sign is ambiguous.
            # Lock direction deterministically (CCW = +1) so a yaw perturbation can't
            # flip the controller mid-turn.
            if abs(required) > math.pi - math.radians(10.0):
                direction = 1.0
            else:
                direction = math.copysign(1.0, required) if abs(required) > 1e-6 else 0.0
            self._turn_tracker.reset(target_heading, abs(required), direction)
        self._turn_tracker.update(yaw_rad)
        return self._turn_tracker.complete(
            yaw_rad,
            self.config.heading_tolerance_rad,
            self.config.turn_settle_ticks,
        )

    def _turn_complete_with_parallel_boundary(
        self,
        yaw_rad: float,
        target_heading: float,
        target_delta_rad: float,
        vision: SurveyVision | None,
    ) -> bool:
        heading_reached = self._pending_turn_heading_reached or self._turn_complete(
            yaw_rad, target_heading, target_delta_rad
        )
        if not heading_reached:
            return False
        self._pending_turn_heading_reached = True
        if self._boundary_line_parallel(vision):
            self._last_corner_reject_reason = None
            return True
        if self._last_corner_reject_reason is None:
            self._last_corner_reject_reason = "turn_waiting_for_parallel_boundary"
        return False

    def _boundary_line_parallel(self, vision: SurveyVision | None) -> bool:
        if vision is None or not vision.line_detected or vision.line_heading_error_rad is None:
            return False
        line_err = abs(self._angle_delta(0.0, vision.line_heading_error_rad))
        if line_err <= self.config.corner_parallel_tolerance_rad:
            return True
        self._last_corner_reject_reason = f"turn_boundary_line_not_parallel:{math.degrees(line_err):.1f}deg"
        return False

    def _turn_parallel_search_command(self) -> BaseCommand:
        direction = self._turn_tracker.direction
        if direction == 0.0:
            direction = 1.0
        return BaseCommand(0.0, direction * self.config.turn_parallel_search_speed_rad_s)

    # ── Output / pattern validation ────────────────────────────────────────────

    def _add_survey_navigation_point(self, label: str, x_m: float, y_m: float, note: str) -> None:
        if not math.isfinite(x_m) or not math.isfinite(y_m):
            return
        if any(p.get("label") == label for p in self._survey_navigation_points):
            return
        self._survey_navigation_points.append({
            "label": label,
            "x_m": round(float(x_m), 3),
            "y_m": round(float(y_m), 3),
            "state": self.state.value,
            "sample_count": self.sample_count,
            "distance_traveled_m": round(self._distance_traveled_m, 3),
            "note": note,
        })

    def _survey_navigation_points_sample(self) -> list[dict]:
        return list(self._survey_navigation_points)

    def _survey_route_points(self) -> list[dict]:
        return [
            {"x_m": p["x_m"], "y_m": p["y_m"], "label": p["label"]}
            for p in self._survey_navigation_points
        ]

    def _survey_pattern_status(self) -> dict:
        expected = [
            "near_net_standoff",
            "near_baseline_fence_standoff",
            "near_left_fence_corner",
            "far_left_fence_corner",
            "far_right_fence_corner",
            "near_right_fence_corner",
        ]
        by_label = {p["label"]: p for p in self._survey_navigation_points}
        labels = [p["label"] for p in self._survey_navigation_points]

        matched: list[str] = []
        pos = 0
        for label in labels:
            if pos < len(expected) and label == expected[pos]:
                matched.append(label)
                pos += 1
        missing = expected[pos:]

        geometry_valid: bool | None = None
        geometry_checks: dict = {}
        def pt(lbl: str) -> tuple[float, float] | None:
            p = by_label.get(lbl)
            return (p["x_m"], p["y_m"]) if p else None

        tol = self.config.pattern_length_tolerance_m
        cl = self.config.expected_court_length_m
        cw = self.config.expected_court_width_m

        checks: dict[str, dict] = {}
        p2, p3 = pt("near_baseline_fence_standoff"), pt("near_left_fence_corner")
        p3b, p4 = pt("near_left_fence_corner"), pt("far_left_fence_corner")
        p4b, p5 = pt("far_left_fence_corner"), pt("far_right_fence_corner")
        p5b, p6 = pt("far_right_fence_corner"), pt("near_right_fence_corner")

        if p2 and p3:
            dx = abs(p3[0] - p2[0])
            lateral_y = p3[1] - p2[1]
            dy = abs(lateral_y)
            d = self._distance(p2, p3)
            checks["near_baseline_to_near_left"] = {
                "dist_m": round(d, 2),
                "dx_m": round(dx, 2),
                "dy_m": round(dy, 2),
                "lateral_y_m": round(lateral_y, 2),
                "ok": d >= (cw / 2) - tol and lateral_y < 0.0 and dy >= dx,
            }
        if p3b and p4:
            d = self._distance(p3b, p4)
            checks["left_side"] = {"dist_m": round(d, 2), "ok": abs(d - cl) < tol * 2}
        if p4b and p5:
            d = self._distance(p4b, p5)
            checks["far_baseline"] = {"dist_m": round(d, 2), "ok": abs(d - cw) < tol * 2}
        if p5b and p6:
            d = self._distance(p5b, p6)
            checks["right_side"] = {"dist_m": round(d, 2), "ok": abs(d - cl) < tol * 2}

        geometry_checks = checks
        if checks:
            geometry_valid = all(v["ok"] for v in checks.values())

        return {
            "pattern": "sensor_discovered_perimeter_loop",
            "complete": len(missing) == 0,
            "geometry_valid": geometry_valid,
            "matched": matched,
            "missing": missing,
            "expected_next": missing[0] if missing else None,
            "point_count": len(labels),
            "geometry_checks": geometry_checks,
        }

    def _canonical_fence_model(self, pattern: dict) -> dict:
        by_label = {p["label"]: p for p in self._survey_navigation_points}
        label_to_corner = {
            "near_left_fence_corner": "near_left",
            "far_left_fence_corner": "far_left",
            "far_right_fence_corner": "far_right",
            "near_right_fence_corner": "near_right",
        }

        corners: dict[str, dict] = {}
        for source_label, corner_label in label_to_corner.items():
            point = by_label.get(source_label)
            if point is not None:
                corners[corner_label] = {
                    "x_m": point["x_m"],
                    "y_m": point["y_m"],
                    "source_label": source_label,
                }

        reference_points: dict[str, dict] = {}
        for label in ("near_net_standoff", "near_baseline_fence_standoff"):
            point = by_label.get(label)
            if point is not None:
                reference_points[label] = {"x_m": point["x_m"], "y_m": point["y_m"]}

        errors: list[str] = []
        missing_route_points = pattern.get("missing") or []
        if missing_route_points:
            errors.append("missing_route_points:" + ",".join(missing_route_points))
        missing_corners = sorted(set(label_to_corner.values()) - set(corners))
        if missing_corners:
            errors.append("missing_canonical_corners:" + ",".join(missing_corners))
        if pattern.get("geometry_valid") is not True:
            errors.append("navigation_pattern_geometry_invalid")

        def _fence(label: str, start_corner: str, end_corner: str) -> dict | None:
            start = corners.get(start_corner)
            end = corners.get(end_corner)
            if start is None or end is None:
                return None
            return {
                "start_corner": start_corner,
                "end_corner": end_corner,
                "length_m": round(
                    self._distance((start["x_m"], start["y_m"]), (end["x_m"], end["y_m"])),
                    3,
                ),
            }

        fence_specs = {
            "near_baseline_fence": ("near_left", "near_right"),
            "left_side_fence": ("near_left", "far_left"),
            "far_baseline_fence": ("far_left", "far_right"),
            "right_side_fence": ("far_right", "near_right"),
        }
        fences = {
            label: fence
            for label, (start, end) in fence_specs.items()
            if (fence := _fence(label, start, end)) is not None
        }

        return {
            "status": "VALID" if not errors else "PARTIAL",
            "corners": corners,
            "fences": fences,
            "reference_points": reference_points,
            "validation_errors": errors,
        }

    def _finalize_full_survey(self, failure: str | None) -> None:
        now = time.time()
        elapsed = 0.0 if self._started_at is None else now - self._started_at

        def _median(samples: list[float]) -> float | None:
            if not samples:
                return None
            s = sorted(samples)
            return round(s[len(s) // 2], 3)

        left_sl = self._left_sideline_to_fence_m
        right_sl = self._right_sideline_to_fence_m
        is_doubles: bool | None = None
        doubles_threshold = self.config.doubles_alley_m + 0.4
        if left_sl is not None and right_sl is not None:
            is_doubles = (left_sl > doubles_threshold) and (right_sl > doubles_threshold)
        elif left_sl is not None:
            is_doubles = left_sl > doubles_threshold
        elif right_sl is not None:
            is_doubles = right_sl > doubles_threshold

        pattern = self._survey_pattern_status()
        canonical = self._canonical_fence_model(pattern)
        overall_status = "SUCCESS" if (canonical["status"] == "VALID" and not failure) else "PARTIAL"
        failure_reason = failure
        expected_next = pattern.get("expected_next")
        if failure_reason and expected_next:
            failure_reason = f"{failure_reason}_waiting_for_{expected_next}"

        bounds = {
            "surveyed_at": now,
            "status": overall_status,
            "survey_complete": overall_status == "SUCCESS",
            "survey_type": "full_perimeter",
            "failure_reason": failure_reason,
            "canonical_fence_model": canonical,
            "boundary_distances": {
                "near_baseline_to_fence_m": self._near_baseline_to_fence_m,
                "far_baseline_to_fence_m": self._far_baseline_to_fence_m,
                "left_sideline_to_fence_m": self._left_sideline_to_fence_m,
                "right_sideline_to_fence_m": self._right_sideline_to_fence_m,
                "left_side_lidar_median_m": _median(self._left_range_samples),
                "right_side_lidar_median_m": _median(self._right_range_samples),
                "long_side_left_lidar_median_m": _median(self._long_left_range_samples),
                "far_short_side_lidar_median_m": _median(self._far_short_range_samples),
                "return_side_lidar_median_m": _median(self._return_range_samples),
                "far_baseline_crossed": self._far_baseline_crossed,
            },
            "navigation_points": self._survey_navigation_points_sample(),
            "navigation_route": self._survey_route_points(),
            "navigation_pattern": pattern,
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

    def _write_bounds(self, bounds: dict) -> None:
        self._court_bounds = bounds
        self.config.output_file.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.config.output_file.with_suffix(".tmp.json")
        tmp.write_text(json.dumps(bounds, indent=2) + "\n", encoding="utf-8")
        tmp.replace(self.config.output_file)

    # ── Internal plumbing ──────────────────────────────────────────────────────

    def _fail(self, reason: str) -> None:
        self._failure_reason = reason
        self._finalize_full_survey(reason)

    def _enter(self, state: LidarSurveyState, event: str) -> None:
        self.state = state
        self._last_event = event
        self._state_elapsed_s = 0.0
        self._state_entry_distance_m = self._distance_traveled_m
        if state in {
            LidarSurveyState.TURN_180,
            LidarSurveyState.TURN_TO_SIDELINE,
            LidarSurveyState.TURN_TO_LONG_SIDE,
            LidarSurveyState.TURN_TO_FAR_SHORT,
            LidarSurveyState.TURN_TO_RETURN,
        }:
            self._turn_tracker.reset()
            self._pending_turn_heading_reached = False
        if state in {
            LidarSurveyState.BASELINE_APPROACH,
            LidarSurveyState.DRIVE_SIDELINE,
            LidarSurveyState.DRIVE_LONG_SIDE,
            LidarSurveyState.DRIVE_FAR_SHORT,
            LidarSurveyState.DRIVE_RETURN,
        }:
            self._guard_reject_count = 0

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

    @staticmethod
    def _angle_delta(a: float, b: float) -> float:
        return (a - b + math.pi) % (2.0 * math.pi) - math.pi

    @staticmethod
    def _distance(a: tuple[float, float], b: tuple[float, float]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    @staticmethod
    def _quantile(values: list[float], q: float) -> float:
        idx = min(len(values) - 1, max(0, int(round((len(values) - 1) * q))))
        return values[idx]

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
        pts = list(buckets.values())
        stride = max(1, len(pts) // limit)
        return [{"x_m": round(x, 3), "y_m": round(y, 3)} for x, y in pts[::stride]][-limit:]
