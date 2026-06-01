"""Court survey — drives the full court perimeter to measure all boundary fences with LiDAR.

Route: nearest perimeter corner → west side → north gap (between N net post and N fence)
       → east side → south gap (between S net post and S fence) → back to start.

The robot drives fast (~0.55 m/s) and samples the full 360° LiDAR scan on every timestep.
Each LiDAR return is projected into world coordinates.  When the loop is complete the
5th/95th percentiles of the accumulated point cloud give reliable fence positions, which
are saved to runtime/court_boundary.json for use by the Collect Left/Right Side missions.

Net-crossing strategy
─────────────────────
Net posts are at (x=0, y=±5.485).  The crossing gap between post and outer fence is
only ~1 m wide.  Two "funnel" waypoints (at x=±0.4, y=±5.7) steer the robot above the
post level before it reaches x=0, then the crossing waypoints at y=±6.2 thread the
needle through the centre of the gap.
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

# LiDAR mount offset in robot frame (must match ball_detector.py)
_LIDAR_LOCAL_X = -0.20
_LIDAR_LOCAL_Y = 0.0

# ── Full-court perimeter waypoints ────────────────────────────────────────────
# The net mesh ends around y=±5.6 and the fence inner wall is at y=±6.46.
# With a ~0.58 m wide chassis the safe centerline is only roughly 5.9..6.15,
# so the route keeps several waypoints outside the doubles sideline before
# returning to the inner survey lane.  LiDAR lateral sectors guide the robot
# through the narrow net-gap corridor.
_PERIMETER: list[tuple[float, float]] = [
    (-10.5, -4.0),   # 0  SW  — close to west + south fences
    (-10.5, +4.0),   # 1  NW  — close to west + north fences
    (-1.4,  +4.0),   # 2  north approach, left side
    (-1.4,  +6.05),  # 3  ▶ move outside doubles before reaching the net
    (-0.35, +6.05),  # 4  north gap entry
    (+0.35, +6.05),  # 5  north gap exit
    (+1.4,  +6.05),  # 6  stay outside until fully clear of the net
    (+1.4,  +4.0),   # 7  return to inner survey lane
    (+10.5, +4.0),   # 8  NE  — close to east + north fences
    (+10.5, -4.0),   # 9  SE  — close to east + south fences
    (+1.4,  -4.0),   # 10 south approach, right side
    (+1.4,  -6.05),  # 11 ▶ move outside doubles before reaching the net
    (+0.35, -6.05),  # 12 south gap entry
    (-0.35, -6.05),  # 13 south gap exit
    (-1.4,  -6.05),  # 14 stay outside until fully clear of the net
    (-1.4,  -4.0),   # 15 return to inner survey lane
]

_SUBSAMPLE = 8          # keep every Nth LiDAR index to limit memory
_MIN_POINTS = 500       # minimum accumulated points for a trusted result
_TIMEOUT_S = 480.0      # hard timeout — finalize with whatever we have


class SurveyState(str, Enum):
    GOTO = "goto"
    DONE = "done"


@dataclass(frozen=True)
class SurveyConfig:
    waypoint_tolerance_m: float = 0.45
    crossing_tolerance_m: float = 0.22
    drive_speed_m_s: float = 0.9
    turn_speed_rad_s: float = 1.8
    heading_gain: float = 2.5
    min_fence_range_m: float = 0.4
    max_fence_range_m: float = 11.5

    @classmethod
    def from_env(cls) -> "SurveyConfig":
        d = cls()
        return cls(
            waypoint_tolerance_m=_env_float("SURVEY_WAYPOINT_TOL_M", d.waypoint_tolerance_m),
            crossing_tolerance_m=_env_float("SURVEY_CROSSING_TOL_M", d.crossing_tolerance_m),
            drive_speed_m_s=_env_float("SURVEY_DRIVE_SPEED_M_S", d.drive_speed_m_s),
            turn_speed_rad_s=_env_float("SURVEY_TURN_SPEED_RAD_S", d.turn_speed_rad_s),
            heading_gain=_env_float("SURVEY_HEADING_GAIN", d.heading_gain),
        )


@dataclass(frozen=True)
class SurveyVision:
    """Forward OAK-D depth clearance summary for narrow survey corridors."""

    center_m: float | None = None
    left_m: float | None = None
    right_m: float | None = None
    valid_count: int = 0


@dataclass(frozen=True)
class SurveyCommand:
    state: SurveyState
    base: BaseCommand
    waypoint_index: int
    sample_count: int
    vision: SurveyVision | None = None


class CourtSurveyBehavior:
    """Drive the full court perimeter and accumulate LiDAR data to measure all boundaries.

    The robot navigates a 10-waypoint closed loop that hugs each fence and crosses
    between the two court halves via the gaps at the ends of the net.  Every timestep
    all in-range LiDAR returns are projected into world coordinates.  At loop completion
    the point cloud is summarised via percentiles and saved to court_boundary.json.
    """

    def __init__(
        self,
        config: SurveyConfig | None = None,
        output_path: Path = DEFAULT_BOUNDARY_FILE,
    ) -> None:
        self.config = config or SurveyConfig()
        self.output_path = output_path
        self.state = SurveyState.GOTO
        self.waypoint_index = 0
        self.waypoints: list[tuple[float, float]] = []
        self.sample_count = 0
        self._initialized = False
        self._started_at: float | None = None
        self._world_xs: list[float] = []
        self._world_ys: list[float] = []
        self._court_bounds: dict | None = None
        self._min_front_range: float = float("inf")
        self._obstacle_bias: float = 0.0   # +1 = obstacle left → steer right, -1 = right → steer left
        self._current_ranges: list[float] = []

    @classmethod
    def from_env(cls) -> "CourtSurveyBehavior":
        path = Path(os.getenv("SURVEY_OUTPUT_FILE", str(DEFAULT_BOUNDARY_FILE)))
        return cls(SurveyConfig.from_env(), path)

    def reset(self) -> None:
        self.state = SurveyState.GOTO
        self.waypoint_index = 0
        self.waypoints = []
        self.sample_count = 0
        self._initialized = False
        self._started_at = None
        self._world_xs = []
        self._world_ys = []
        self._court_bounds = None
        self._min_front_range = float("inf")
        self._obstacle_bias = 0.0
        self._current_ranges = []

    def current_target(self) -> tuple[float, float] | None:
        if self.waypoint_index >= len(self.waypoints):
            return None
        return self.waypoints[self.waypoint_index]

    @property
    def court_bounds(self) -> dict | None:
        """Measured court boundaries, populated after the survey completes."""
        return self._court_bounds

    def update(
        self,
        x_m: float,
        y_m: float,
        yaw_rad: float,
        lidar_ranges: list[float] | None,
        dt_s: float,  # noqa: ARG002 — kept for API compatibility with other behaviors
        vision: SurveyVision | None = None,
    ) -> SurveyCommand:
        if not self._initialized:
            self._setup_route(x_m, y_m)
            self._initialized = True
            self._started_at = time.time()

        if self.state == SurveyState.DONE:
            return self._cmd(BaseCommand(0.0, 0.0))

        # Accumulate LiDAR data on every active timestep
        if lidar_ranges:
            self._current_ranges = lidar_ranges
            self._accumulate(lidar_ranges, x_m, y_m, yaw_rad)
            self._min_front_range, self._obstacle_bias = self._front_scan(lidar_ranges)

        # Hard timeout — avoids getting stuck if the robot can't complete the loop
        if self._started_at is not None and time.time() - self._started_at > _TIMEOUT_S:
            print(f"survey: timeout after {_TIMEOUT_S:.0f}s — finalizing with {len(self._world_xs)} pts")
            self._finalize()
            self.state = SurveyState.DONE
            return self._cmd(BaseCommand(0.0, 0.0))

        return self._step_goto(x_m, y_m, yaw_rad, vision)

    # ── Private ────────────────────────────────────────────────────────────────

    def _setup_route(self, x_m: float, y_m: float) -> None:
        """Rotate the fixed perimeter so the nearest waypoint comes first, then return to start."""
        start_idx = min(
            range(len(_PERIMETER)),
            key=lambda i: math.hypot(_PERIMETER[i][0] - x_m, _PERIMETER[i][1] - y_m),
        )
        loop = _PERIMETER[start_idx:] + _PERIMETER[:start_idx]
        # Append start position so the robot returns home after the loop
        self.waypoints = list(loop) + [(x_m, y_m)]
        print(
            f"survey: start=({x_m:.1f},{y_m:.1f})  "
            f"entry={_PERIMETER[start_idx]}  "
            f"{len(self.waypoints)} wps  "
            f"speed={self.config.drive_speed_m_s} m/s"
        )

    def _step_goto(
        self,
        x_m: float,
        y_m: float,
        yaw_rad: float,
        vision: SurveyVision | None,
    ) -> SurveyCommand:
        if self.waypoint_index >= len(self.waypoints):
            self._finalize()
            self.state = SurveyState.DONE
            return self._cmd(BaseCommand(0.0, 0.0))

        target_x, target_y = self.waypoints[self.waypoint_index]
        dx = target_x - x_m
        dy = target_y - y_m
        dist = math.hypot(dx, dy)

        tolerance = self._target_tolerance(target_x, target_y)
        if dist <= tolerance:
            self.waypoint_index += 1
            print(
                f"survey: reached wp {self.waypoint_index}/{len(self.waypoints)}  "
                f"reached ({target_x:.1f},{target_y:.1f})  "
                f"pts={len(self._world_xs)}"
            )
            if self.waypoint_index >= len(self.waypoints):
                self._finalize()
                self.state = SurveyState.DONE
            return self._cmd(BaseCommand(0.0, 0.0))

        heading_error = _wrap(math.atan2(dy, dx) - yaw_rad)
        turn = max(
            -self.config.turn_speed_rad_s,
            min(self.config.turn_speed_rad_s, heading_error * self.config.heading_gain),
        )

        # LiDAR lateral avoidance: steer away from the side where the obstacle is.
        # bias > 0 → obstacle on LEFT → add rightward (negative) correction.
        # Only active when something is within the avoidance horizon (~1.2 m).
        if self._min_front_range < 1.2:
            avoid_strength = min(1.0, (1.2 - self._min_front_range) / 0.9)
            avoidance = -self._obstacle_bias * avoid_strength * self.config.turn_speed_rad_s
            turn = max(
                -self.config.turn_speed_rad_s,
                min(self.config.turn_speed_rad_s, turn + avoidance),
            )

        # Slow down near broad obstacles; use min() not × to avoid compounding with
        # heading_scale (turning near any object would otherwise stall to ~0.03 m/s)
        heading_scale = max(0.15, 1.0 - abs(heading_error) / math.pi)
        proximity_scale = min(1.0, max(0.2, (self._min_front_range - 0.3) / 0.6))
        linear = self.config.drive_speed_m_s * min(heading_scale, proximity_scale)
        linear, turn = self._lidar_crossing_guard(target_x, target_y, linear, turn)
        linear = self._oak_forward_brake(linear, vision)
        return self._cmd(BaseCommand(linear, turn), vision)

    def _target_tolerance(self, target_x: float, target_y: float) -> float:
        if abs(target_y) > 5.6 and abs(target_x) < 1.6:
            return self.config.crossing_tolerance_m
        return self.config.waypoint_tolerance_m

    def _oak_forward_brake(self, linear: float, vision: SurveyVision | None) -> float:
        """Decelerate based on OAK-D center depth to stop before chain-link fences.

        LiDAR passes through mesh fences; OAK-D fills that blind spot.
        Hard stop at 0.25 m (camera reading), gradual ramp from 1.2 m.
        Preserves a 0.04 m/s creep so the robot can still satisfy waypoint tolerances
        when the target is close to the fence.
        """
        if vision is None or vision.center_m is None:
            return linear
        d = vision.center_m
        if d >= 1.2:
            return linear
        if d <= 0.25:
            return 0.0
        scale = (d - 0.25) / (1.2 - 0.25)
        braked = min(linear, self.config.drive_speed_m_s * scale)
        return max(braked, 0.04)

    def _lidar_crossing_guard(
        self,
        target_x: float,
        target_y: float,
        linear: float,
        turn: float,
    ) -> tuple[float, float]:
        """Center the robot in the net-gap corridor using perpendicular LiDAR sectors.

        Active only during the actual east-west crossing (|target_y| > 5.6 AND
        |target_x| < 1.6).  This excludes the northward/southward approach legs
        (WP3/WP11) where the lateral sectors would point east/west and be useless.
        LiDAR index n//4 points in the local −y direction (toward the net post on both
        crossings), index 3n//4 points in local +y (toward the outer fence).
        """
        if not self._current_ranges or abs(target_y) < 5.6 or abs(target_x) >= 1.6:
            return linear, turn
        ranges = self._current_ranges
        n = len(ranges)
        if n < 10:
            return linear, turn

        sector_w = n // 8          # ±45° half-width around the perpendicular
        right_idx = n // 4         # local −y → net post side
        left_idx  = 3 * n // 4    # local +y → outer fence side
        min_r = self.config.min_fence_range_m
        max_r = 2.5

        def side_p25(center: int) -> float | None:
            lo = max(0, center - sector_w)
            hi = min(n, center + sector_w)
            vals = [
                ranges[i] for i in range(lo, hi)
                if math.isfinite(ranges[i]) and min_r < ranges[i] < max_r
            ]
            if not vals:
                return None
            vals.sort()
            return vals[len(vals) // 4]

        right_r = side_p25(right_idx)   # clearance to net post
        left_r  = side_p25(left_idx)    # clearance to outer fence

        linear = min(linear, 0.25)

        if right_r is not None and left_r is not None:
            # Steer toward the side with more space: positive = left = toward fence
            correction = 0.6 * self.config.turn_speed_rad_s * math.tanh((left_r - right_r) / 0.4)
            turn = max(
                -self.config.turn_speed_rad_s,
                min(self.config.turn_speed_rad_s, turn + correction),
            )
        elif right_r is not None:
            # Only net post visible — steer left (toward fence)
            turn = min(self.config.turn_speed_rad_s, turn + 0.3 * self.config.turn_speed_rad_s)
        elif left_r is not None:
            # Only fence visible — steer right (toward net post)
            turn = max(-self.config.turn_speed_rad_s, turn - 0.3 * self.config.turn_speed_rad_s)

        return linear, turn

    def _front_scan(self, ranges: list[float]) -> tuple[float, float]:
        """Scan the ±60° forward sector.

        Returns (proximity_range, lateral_bias):
        - proximity_range: 30th-percentile of valid returns (for speed scaling).
          Uses min_fence_range_m floor so chassis self-returns are excluded.
        - lateral_bias: weighted centre of close returns, normalised to [-1, +1].
          +1 = obstacle concentrated on the LEFT  → caller should steer right.
          -1 = obstacle concentrated on the RIGHT → caller should steer left.
          Only returns in the avoidance window (min_r … 1.2 m) contribute.
        """
        n = len(ranges)
        if n < 10:
            return float("inf"), 0.0

        sector = n // 6   # ±60°
        mid = n // 2
        lo, hi = max(0, mid - sector), min(n, mid + sector)
        min_r = self.config.min_fence_range_m
        avoid_max = 1.2   # avoidance horizon (m)

        valid = []
        weight_sum = 0.0
        lateral_sum = 0.0

        for i in range(lo, hi):
            r = ranges[i]
            if not math.isfinite(r) or r <= min_r:
                continue
            valid.append(r)
            if r < avoid_max:
                # weight by proximity: 1.0 at min_r, 0.0 at avoid_max
                w = (avoid_max - r) / (avoid_max - min_r)
                # lateral position: (i - mid) / sector → +1 = left, -1 = right
                lateral_sum += ((i - mid) / sector) * w
                weight_sum += w

        if not valid:
            return float("inf"), 0.0

        valid.sort()
        idx = max(0, int(len(valid) * 0.30) - 1)
        prox_range = valid[idx]
        bias = lateral_sum / weight_sum if weight_sum > 0 else 0.0
        return prox_range, bias

    def _accumulate(
        self,
        ranges: list[float],
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
    ) -> None:
        """Project subsampled LiDAR returns into world coordinates and append to running lists."""
        n = len(ranges)
        if n < 10:
            return

        cos_y = math.cos(robot_yaw)
        sin_y = math.sin(robot_yaw)
        r_min = self.config.min_fence_range_m
        r_max = self.config.max_fence_range_m
        added = 0

        for i in range(0, n, _SUBSAMPLE):
            r = ranges[i]
            if not math.isfinite(r) or r < r_min or r > r_max:
                continue
            # Index 0 = backward (−x in robot frame), index n/2 = forward (+x)
            angle = (i / n) * 2.0 * math.pi - math.pi
            lx = _LIDAR_LOCAL_X + r * math.cos(angle)
            ly = _LIDAR_LOCAL_Y + r * math.sin(angle)
            wx = robot_x + cos_y * lx - sin_y * ly
            wy = robot_y + sin_y * lx + cos_y * ly
            # Reject points outside the court+margin (grandstand, chairs, floodlights)
            if abs(wx) > 13.5 or abs(wy) > 7.0:
                continue
            self._world_xs.append(wx)
            self._world_ys.append(wy)
            added += 1

        if added:
            self.sample_count += 1

    def _finalize(self) -> None:
        """Compute fence positions from the accumulated world-coordinate cloud and save."""
        n = len(self._world_xs)
        elapsed = 0.0 if self._started_at is None else time.time() - self._started_at
        now = time.time()

        if n >= _MIN_POINTS:
            xs = sorted(self._world_xs)
            ys = sorted(self._world_ys)
            west_x  = round(xs[int(n * 0.05)], 3)
            east_x  = round(xs[int(n * 0.95)], 3)
            south_y = round(ys[int(n * 0.05)], 3)
            north_y = round(ys[int(n * 0.95)], 3)
            length_m = round(east_x - west_x, 3)
            width_m  = round(north_y - south_y, 3)
            status = "SUCCESS"
            failure_reason = None
        else:
            print(f"map court: FAILED — only {n} pts accumulated (minimum {_MIN_POINTS} required)")
            west_x = east_x = south_y = north_y = None
            length_m = width_m = None
            status = "FAILED"
            failure_reason = f"Insufficient LiDAR coverage: {n} points accumulated (minimum {_MIN_POINTS} required)"

        bounds: dict = {
            # Court Knowledge Model status
            "mapped_at": now,
            "status": status,
            "failure_reason": failure_reason,

            # Court geometry
            "court_geometry": {
                "length_m": length_m,
                "width_m": width_m,
                "orientation_deg": 0.0,
            },

            # Fence geometry with clearance from standard court lines
            # (court baselines ±11.885 m, doubles sidelines ±5.485 m)
            "fence_geometry": {
                "west_x":  west_x,
                "east_x":  east_x,
                "south_y": south_y,
                "north_y": north_y,
                "clearance": {
                    "west_m":  round(abs(west_x)  - 11.885, 2) if west_x  is not None else None,
                    "east_m":  round(east_x        - 11.885, 2) if east_x  is not None else None,
                    "south_m": round(abs(south_y)  - 5.485,  2) if south_y is not None else None,
                    "north_m": round(north_y        - 5.485,  2) if north_y is not None else None,
                },
            },

            # Obstacles — full per-object detection not yet implemented
            "obstacles": {
                "count": 0,
            },

            # Accessibility derived from traversal completeness
            "accessibility": {
                "perimeter_complete": status == "SUCCESS",
                "traversable_area_sqm": round(length_m * width_m, 1) if length_m is not None else None,
            },

            # Navigation summary
            "navigation": {
                "perimeter_waypoint_count": len(self.waypoints),
                "elapsed_s": round(elapsed, 1),
            },

            # Legacy keys — kept for backward compatibility with mapping.py and DB import
            "surveyed_at": now,
            "survey_complete": status == "SUCCESS",
            "west_fence_x":  west_x,
            "east_fence_x":  east_x,
            "south_fence_y": south_y,
            "north_fence_y": north_y,
            "sample_count": self.sample_count,
            "point_count": n,
        }

        # Embed active vendor/court session if set via the control panel
        try:
            if _VENDORS_FILE.exists():
                vdata = json.loads(_VENDORS_FILE.read_text(encoding="utf-8"))
                active = vdata.get("active") or {}
                if active.get("vendor_id"):
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
            pass

        self._court_bounds = bounds
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.output_path.with_suffix(".tmp.json")
        with tmp.open("w", encoding="utf-8") as f:
            json.dump(bounds, f, indent=2)
            f.write("\n")
        tmp.replace(self.output_path)

        if status == "SUCCESS":
            print(
                f"map court: complete in {elapsed:.1f}s  "
                f"{self.sample_count} frames, {n} pts  "
                f"W={west_x:.2f}  E={east_x:.2f}  S={south_y:.2f}  N={north_y:.2f}  "
                f"-> {self.output_path}"
            )
        else:
            print(
                f"map court: FAILED in {elapsed:.1f}s  {n} pts  "
                f"{failure_reason}  -> {self.output_path}"
            )

    def _cmd(self, base: BaseCommand, vision: SurveyVision | None = None) -> SurveyCommand:
        return SurveyCommand(self.state, base, self.waypoint_index, self.sample_count, vision)


def _wrap(a: float) -> float:
    """Wrap angle to (−π, +π]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi
