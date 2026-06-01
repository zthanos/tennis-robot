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

# Fallback fence positions — inner walls of the perimeter fencing in tennis_court.wbt.
# Fence centers: east/west at x=±13.0, north/south at y=±6.5 (thickness 0.08 m each).
# Inner wall = center ∓ 0.04 m.  Used only when LiDAR accumulates < _MIN_POINTS.
_FB_EAST_X = 12.96
_FB_WEST_X = -12.96
_FB_NORTH_Y = 6.46
_FB_SOUTH_Y = -6.46

# ── Full-court perimeter waypoints ────────────────────────────────────────────
# The net mesh ends around y=±5.6 and the fence inner wall is at y=±6.46.
# With a ~0.58 m wide chassis the safe centerline is only roughly 5.9..6.15,
# so the route keeps several waypoints outside the doubles sideline before
# returning to the inner survey lane.  OAK-D depth acts as the forward guard
# for this narrow visual corridor; LiDAR still records the fence/boundary cloud.
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
_TIMEOUT_S = 300.0      # hard timeout — finalize with whatever we have


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
    oak_min_clearance_m: float = 0.85

    @classmethod
    def from_env(cls) -> "SurveyConfig":
        d = cls()
        return cls(
            waypoint_tolerance_m=_env_float("SURVEY_WAYPOINT_TOL_M", d.waypoint_tolerance_m),
            crossing_tolerance_m=_env_float("SURVEY_CROSSING_TOL_M", d.crossing_tolerance_m),
            drive_speed_m_s=_env_float("SURVEY_DRIVE_SPEED_M_S", d.drive_speed_m_s),
            turn_speed_rad_s=_env_float("SURVEY_TURN_SPEED_RAD_S", d.turn_speed_rad_s),
            heading_gain=_env_float("SURVEY_HEADING_GAIN", d.heading_gain),
            oak_min_clearance_m=_env_float("SURVEY_OAK_MIN_CLEARANCE_M", d.oak_min_clearance_m),
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
        linear, turn = self._apply_oak_corridor_guard(target_y, linear, turn, vision)
        return self._cmd(BaseCommand(linear, turn), vision)

    def _target_tolerance(self, target_x: float, target_y: float) -> float:
        if abs(target_y) > 5.6 and abs(target_x) < 1.6:
            return self.config.crossing_tolerance_m
        return self.config.waypoint_tolerance_m

    def _apply_oak_corridor_guard(
        self,
        target_y: float,
        linear: float,
        turn: float,
        vision: SurveyVision | None,
    ) -> tuple[float, float]:
        """Use OAK-D depth to slow and bias the robot in the narrow outer gap."""
        if vision is None or vision.center_m is None or abs(target_y) < 5.6:
            return linear, turn
        if vision.center_m >= self.config.oak_min_clearance_m:
            return linear, turn

        linear = min(linear, 0.12)
        left = vision.left_m if vision.left_m is not None else 0.0
        right = vision.right_m if vision.right_m is not None else 0.0
        if left or right:
            # Positive turn steers toward the left side of the camera image.
            turn += 0.45 * self.config.turn_speed_rad_s * (1.0 if left > right else -1.0)
        else:
            # When depth is sparse, bias outward from the net side.
            turn += 0.25 * self.config.turn_speed_rad_s * (1.0 if target_y > 0 else -1.0)
        return (
            linear,
            max(-self.config.turn_speed_rad_s, min(self.config.turn_speed_rad_s, turn)),
        )

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
        fb = {
            "east_fence_x":  _FB_EAST_X,
            "west_fence_x":  _FB_WEST_X,
            "north_fence_y": _FB_NORTH_Y,
            "south_fence_y": _FB_SOUTH_Y,
        }
        bounds: dict = {
            "surveyed_at": time.time(),
            "survey_complete": True,
            "sample_count": self.sample_count,
            "point_count": n,
        }

        if n >= _MIN_POINTS:
            xs = sorted(self._world_xs)
            ys = sorted(self._world_ys)
            bounds["west_fence_x"]  = round(xs[int(n * 0.05)], 3)
            bounds["east_fence_x"]  = round(xs[int(n * 0.95)], 3)
            bounds["south_fence_y"] = round(ys[int(n * 0.05)], 3)
            bounds["north_fence_y"] = round(ys[int(n * 0.95)], 3)
        else:
            print(f"survey: only {n} pts accumulated — using fallback dimensions")
            bounds.update(fb)

        for k, v in fb.items():
            if bounds.get(k) is None:
                bounds[k] = v

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

        elapsed = 0.0 if self._started_at is None else time.time() - self._started_at
        print(
            f"survey: complete in {elapsed:.1f}s — "
            f"{self.sample_count} frames, {n} pts  "
            f"W={bounds['west_fence_x']:.2f}  E={bounds['east_fence_x']:.2f}  "
            f"S={bounds['south_fence_y']:.2f}  N={bounds['north_fence_y']:.2f}  "
            f"-> {self.output_path}"
        )

    def _cmd(self, base: BaseCommand, vision: SurveyVision | None = None) -> SurveyCommand:
        return SurveyCommand(self.state, base, self.waypoint_index, self.sample_count, vision)


def _wrap(a: float) -> float:
    """Wrap angle to (−π, +π]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi
