"""Half-court mapping mission: navigate scan poses and build a 3×3 ball-count grid.

No collection — pure mapping. Ball source is injected so Phase 1 uses Supervisor
ground-truth positions; Phase 2 swaps in mapped_balls from camera detection.
"""

from __future__ import annotations

import json
import math
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional, Protocol, Tuple

from tennis_robot.config_utils import _env_float
from tennis_robot.collector import BaseCommand, CollectorCommand, CollectorState, ConceptACommand

_SOURCE_ROOT = Path(__file__).resolve().parents[4]
PROJECT_ROOT = Path(os.getenv("TENNIS_ROBOT_ROOT", os.getenv("WORKSPACE", str(_SOURCE_ROOT))))
RUNTIME_DIR = Path(
    os.getenv("ROBOT_STATUS_FILE", str(PROJECT_ROOT / "runtime" / "robot_status.json"))
).parent
DEFAULT_BOUNDARY_FILE = RUNTIME_DIR / "court_boundary.json"


# ── Config ─────────────────────────────────────────────────────────────────────

SCAN_DURATION_S = _env_float("MAP_SCAN_DURATION_S", 0.75)
NAV_POSITION_TOL_M = _env_float("MAP_NAV_POSITION_TOL_M", 0.45)
NAV_LINEAR_GAIN = _env_float("MAP_NAV_LINEAR_GAIN", 0.95)
NAV_ANGULAR_GAIN = _env_float("MAP_NAV_ANGULAR_GAIN", 2.4)
NAV_MAX_SPEED_M_S = _env_float("MAP_NAV_MAX_SPEED_M_S", 0.62)
NAV_MAX_TURN_RAD_S = _env_float("MAP_NAV_MAX_TURN_RAD_S", 1.7)
SCAN_OFFSET_M = _env_float("MAP_SCAN_OFFSET_M", 3.0)
DETECTION_RANGE_M = _env_float("MAP_DETECTION_RANGE_M", 8.0)   # simulated OAK-D range per scan pose
CLUSTER_RADIUS_M = _env_float("MAP_CLUSTER_RADIUS_M", 0.35)   # deduplicate detections within this radius
COLLECTION_SCAN_CLUSTER_RADIUS_M = _env_float("COLLECTION_SCAN_CLUSTER_RADIUS_M", 0.75)
RETURN_TO_START = os.getenv("MAP_RETURN_TO_START", "true").strip().lower() in {"1", "true", "yes", "on"}

# Each entry: (pose_name, should_scan)
# Pattern: scan at center first, then each offset, returning through center each time.
_SEQUENCE: list[tuple[str, bool]] = [
    ("center", True),
    ("left",   True),
    ("center", False),  # transit back
    ("right",  True),
    ("center", False),  # transit back
    ("front",  True),
    ("center", False),  # transit back
    ("back",   True),
    # after back scan, return to start position
]
_SCAN_POSE_COUNT = sum(1 for _, s in _SEQUENCE if s)


# ── Court model ────────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class HalfCourtBounds:
    """Operational half-court boundaries in Webots world coordinates."""

    side: str       # "left" or "right"
    x_min: float   # fence-side x (negative for left half)
    x_max: float   # net-side x
    y_min: float   # "right" edge in map convention (lower y)
    y_max: float   # "left" edge in map convention (higher y)

    @property
    def center_x(self) -> float:
        return (self.x_min + self.x_max) / 2.0

    @property
    def span_x(self) -> float:
        return self.x_max - self.x_min

    @property
    def span_y(self) -> float:
        return self.y_max - self.y_min


@dataclass(frozen=True)
class CourtFrame:
    center_x_m: float
    center_y_m: float
    axis_length_x: float
    axis_length_y: float
    axis_width_x: float
    axis_width_y: float

    def court_to_map(self, x_m: float, y_m: float) -> tuple[float, float]:
        return (
            self.center_x_m + x_m * self.axis_length_x + y_m * self.axis_width_x,
            self.center_y_m + x_m * self.axis_length_y + y_m * self.axis_width_y,
        )

    def map_to_court(self, x_m: float, y_m: float) -> tuple[float, float]:
        dx = x_m - self.center_x_m
        dy = y_m - self.center_y_m
        return (
            dx * self.axis_length_x + dy * self.axis_length_y,
            dx * self.axis_width_x + dy * self.axis_width_y,
        )


class BoundaryProvider(Protocol):
    """Swap this out to replace Supervisor ground-truth with LiDAR detection."""

    def get_bounds(self, side: str) -> HalfCourtBounds: ...


class SupervisorBoundaryProvider:
    """Returns fixed bounds from Webots court constants (fallback / testing)."""

    def __init__(
        self,
        court_half_x_m: float = 15.0,
        court_half_y_m: float = 5.485,
        y_buffer_m: float = 2.0,
    ) -> None:
        self._half_x = court_half_x_m
        self._y_max = court_half_y_m + y_buffer_m

    def get_bounds(self, side: str) -> HalfCourtBounds:
        y = self._y_max
        if side == "left":
            return HalfCourtBounds("left", -self._half_x, 0.0, -y, y)
        return HalfCourtBounds("right", 0.0, self._half_x, -y, y)


class LidarSurveyBoundaryProvider:
    """Reads court boundaries from the survey output file written by CourtSurveyBehavior.

    Raises RuntimeError if the file is missing, the survey failed, or the data is malformed.
    Run Map Court before any mission that depends on this provider.
    """

    def __init__(self, boundary_path: Path | None = None) -> None:
        self._path = boundary_path or DEFAULT_BOUNDARY_FILE

    def get_bounds(self, side: str) -> HalfCourtBounds:
        try:
            with self._path.open("r", encoding="utf-8") as f:
                data = json.load(f)
        except FileNotFoundError as exc:
            raise RuntimeError(
                f"Court survey file not found: {self._path} — run Map Court first."
            ) from exc
        except OSError as exc:
            raise RuntimeError(f"Cannot read court survey file: {exc}") from exc

        if not data.get("survey_complete"):
            reason = data.get("failure_reason") or "survey_complete=false"
            raise RuntimeError(f"Court survey incomplete — run Map Court first. Reason: {reason}")

        try:
            fence_geometry = _canonical_fence_bounds(data)
            east_x  = float(fence_geometry["east_x"])
            west_x  = float(fence_geometry["west_x"])
            north_y = float(fence_geometry["north_y"])
            south_y = float(fence_geometry["south_y"])
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError(f"Court survey data malformed: {exc}") from exc

        if side == "left":
            return HalfCourtBounds("left", west_x, 0.0, south_y, north_y)
        return HalfCourtBounds("right", 0.0, east_x, south_y, north_y)


# ── 3×3 grid ───────────────────────────────────────────────────────────────────

def _canonical_fence_bounds(data: dict) -> dict:
    canonical = data["canonical_fence_model"]
    corners = canonical["corners"]
    xs = [float(corner["x_m"]) for corner in corners.values()]
    ys = [float(corner["y_m"]) for corner in corners.values()]
    if not xs or not ys:
        raise KeyError("canonical_fence_model.corners")
    return {
        "west_x": min(xs),
        "east_x": max(xs),
        "south_y": min(ys),
        "north_y": max(ys),
    }


def _load_v2_court_frame(data: dict) -> CourtFrame:
    frame = (data.get("map_artifact") or {}).get("court_frame") or {}
    center = frame.get("center") or data.get("net", {}).get("center") or {}
    axis_length = frame.get("axis_length") or data.get("net", {}).get("axis_length") or {}
    axis_width = frame.get("axis_width") or data.get("net", {}).get("axis_width") or {}
    try:
        return CourtFrame(
            center_x_m=float(center["x_m"]),
            center_y_m=float(center["y_m"]),
            axis_length_x=float(axis_length["x_m"]),
            axis_length_y=float(axis_length["y_m"]),
            axis_width_x=float(axis_width["x_m"]),
            axis_width_y=float(axis_width["y_m"]),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError(f"Court survey v2 frame malformed: {exc}") from exc


def _load_v2_lines(data: dict) -> dict:
    try:
        lines = data["court"]["lines_court_frame"]
        return {
            "baselines_x": [float(v) for v in lines["baselines_x"]],
            "service_x": [float(v) for v in lines["service_x"]],
            "sidelines_y": [float(v) for v in lines["sidelines_y"]],
        }
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError(f"Court survey v2 lines malformed: {exc}") from exc


def _angle_delta(a: float, b: float) -> float:
    return (a - b + math.pi) % (2 * math.pi) - math.pi


def _classify(x_m: float, y_m: float, b: HalfCourtBounds) -> Optional[Tuple[int, int]]:
    """Map world (x, y) → (row, col), or None if outside operational area.

    Row 0 = Φράχτης (fence)    Row 2 = Φιλέ (net)
    Col 0 = Αριστερά (y_max)   Col 2 = Δεξιά (y_min)

    Convention: map-relative, looking from fence toward net.
    y > 0 is "left" (col 0), y < 0 is "right" (col 2).
    """
    if not (b.x_min <= x_m <= b.x_max and b.y_min <= y_m <= b.y_max):
        return None
    if b.span_x < 1e-6 or b.span_y < 1e-6:
        return None

    # x_frac: 0 at fence → row 0, 1 at net → row 2
    if b.side in {"left", "side_neg_x"}:
        xf = (x_m - b.x_min) / b.span_x
    else:
        xf = (b.x_max - x_m) / b.span_x

    # y_frac: 0 at y_max (left) → col 0, 1 at y_min (right) → col 2
    yf = (b.y_max - y_m) / b.span_y

    return (min(2, int(xf * 3)), min(2, int(yf * 3)))


# ── Candidate store ────────────────────────────────────────────────────────────

@dataclass
class _Cand:
    x_m: float
    y_m: float


def _add_candidate(cands: list[_Cand], x_m: float, y_m: float, radius_m: float = CLUSTER_RADIUS_M) -> None:
    """Add (x, y) only if no existing candidate is within CLUSTER_RADIUS_M."""
    for c in cands:
        if math.hypot(c.x_m - x_m, c.y_m - y_m) < radius_m:
            return
    cands.append(_Cand(x_m, y_m))


# ── Navigation ─────────────────────────────────────────────────────────────────

def _nav_to(
    rx: float, ry: float, ryaw: float, tx: float, ty: float
) -> Optional[ConceptACommand]:
    """P-controller drive command toward (tx, ty). Returns None when arrived."""
    dx, dy = tx - rx, ty - ry
    dist = math.hypot(dx, dy)
    if dist < NAV_POSITION_TOL_M:
        return None
    yaw_err = (math.atan2(dy, dx) - ryaw + math.pi) % (2 * math.pi) - math.pi
    linear = min(NAV_MAX_SPEED_M_S, dist * NAV_LINEAR_GAIN)
    if abs(yaw_err) > math.radians(35.0):
        linear = 0.0
    angular = max(-NAV_MAX_TURN_RAD_S, min(NAV_MAX_TURN_RAD_S, yaw_err * NAV_ANGULAR_GAIN))
    return ConceptACommand(
        state=CollectorState.SURVEY,
        base=BaseCommand(linear, angular),
        collector=CollectorCommand(0.0, False),
    )


def _idle_cmd(state: CollectorState = CollectorState.IDLE) -> ConceptACommand:
    return ConceptACommand(state=state, base=BaseCommand(0.0, 0.0), collector=CollectorCommand(0.0, False))


def _scan_cmd() -> ConceptACommand:
    return _idle_cmd(CollectorState.SCAN)


# ── Mission ────────────────────────────────────────────────────────────────────

BallSourceFn = Callable[[], list[tuple[float, float]]]


class MapLeftSideMission:
    """Navigate 5 scan poses on one half-court and build a 3×3 ball-count grid.

    Phase 1 (Supervisor): ball_source_fn returns exact Webots node positions.
    Phase 2 (camera): swap ball_source_fn to read from the controller's mapped_balls.
    """

    def __init__(self, boundary_provider: BoundaryProvider, ball_source_fn: BallSourceFn) -> None:
        self._boundary = boundary_provider
        self._ball_source = ball_source_fn

        self.active: bool = False
        self.complete: bool = False
        self.bounds: HalfCourtBounds | None = None
        self.candidates: list[_Cand] = []
        self.grid: list[list[int]] = [[0] * 3 for _ in range(3)]

        self._state: str = "idle"
        self._wp_index: int = 0
        self._scan_started_at: float | None = None
        self._scan_poses_done: int = 0
        self._start_pose: tuple[float, float, float] | None = None
        self._elapsed_start: float | None = None
        self._poses: dict[str, tuple[float, float]] = {}

    # ── Public API ─────────────────────────────────────────────────────────────

    def start(self, rx: float, ry: float, ryaw: float) -> None:
        side = "left" if rx < 0.0 else "right"
        self.bounds = self._boundary.get_bounds(side)
        cx = self.bounds.center_x

        # "front" = toward net (x → 0), "back" = toward fence (x away from 0)
        # Left side: net is at x=0, fence at x=-11.885 → toward_net is +x
        toward_net = SCAN_OFFSET_M if side == "left" else -SCAN_OFFSET_M

        self._poses = {
            "center": (cx,                 0.0),
            "left":   (cx,                 SCAN_OFFSET_M),
            "right":  (cx,                -SCAN_OFFSET_M),
            "front":  (cx + toward_net,    0.0),
            "back":   (cx - toward_net,    0.0),
        }

        self.active = True
        self.complete = False
        self.candidates = []
        self.grid = [[0] * 3 for _ in range(3)]
        self._state = "transit"
        self._wp_index = 0
        self._scan_started_at = None
        self._scan_poses_done = 0
        self._start_pose = (rx, ry, ryaw)
        self._elapsed_start = time.time()

    def reset(self) -> None:
        self.active = False
        self.complete = False
        self._state = "idle"

    def update(self, rx: float, ry: float, ryaw: float, _dt_s: float) -> ConceptACommand:
        if self._state == "transit":
            return self._step_transit(rx, ry, ryaw)
        if self._state == "scanning":
            return self._step_scan(rx, ry)
        if self._state == "returning":
            return self._step_return(rx, ry, ryaw)
        return _idle_cmd()

    def telemetry(self) -> dict:
        elapsed = 0.0 if self._elapsed_start is None else time.time() - self._elapsed_start
        scan_elapsed = 0.0 if self._scan_started_at is None else time.time() - self._scan_started_at
        return {
            "active": self.active and not self.complete,
            "side": self.bounds.side if self.bounds else "left",
            "phase": self._state,
            "phase_label": self._phase_label(),
            "scan_poses_done": self._scan_poses_done,
            "scan_poses_total": _SCAN_POSE_COUNT,
            "scan_elapsed_s": scan_elapsed,
            "scan_duration_s": SCAN_DURATION_S,
            "total_candidates": len(self.candidates),
            "grid": [row[:] for row in self.grid],
            "elapsed_s": elapsed,
            "complete": self.complete,
            "return_to_start": RETURN_TO_START,
        }

    # ── Internal steps ─────────────────────────────────────────────────────────

    def _step_transit(self, rx: float, ry: float, ryaw: float) -> ConceptACommand:
        if self._wp_index >= len(_SEQUENCE):
            self._state = "returning"
            return _scan_cmd()

        pose_name, should_scan = _SEQUENCE[self._wp_index]
        tx, ty = self._poses[pose_name]
        cmd = _nav_to(rx, ry, ryaw, tx, ty)
        if cmd is not None:
            return cmd

        # Arrived at waypoint
        if should_scan:
            self._scan_started_at = time.time()
            self._state = "scanning"
        else:
            self._wp_index += 1
            if self._wp_index >= len(_SEQUENCE):
                self._state = "returning"
        return _scan_cmd()

    def _step_scan(self, rx: float, ry: float) -> ConceptACommand:
        for bx, by in self._ball_source():
            if math.hypot(bx - rx, by - ry) <= DETECTION_RANGE_M:
                _add_candidate(self.candidates, bx, by)
        self._rebuild_grid()

        assert self._scan_started_at is not None
        if time.time() - self._scan_started_at >= SCAN_DURATION_S:
            self._scan_poses_done += 1
            self._wp_index += 1
            self._scan_started_at = None
            self._state = "transit" if self._wp_index < len(_SEQUENCE) else "returning"
        return _scan_cmd()

    def _step_return(self, rx: float, ry: float, ryaw: float) -> ConceptACommand:
        if not RETURN_TO_START:
            self._finish()
            return _idle_cmd()
        if self._start_pose is None:
            self._finish()
            return _idle_cmd()
        sx, sy, _ = self._start_pose
        cmd = _nav_to(rx, ry, ryaw, sx, sy)
        if cmd is not None:
            return cmd
        self._finish()
        return _idle_cmd()

    def _finish(self) -> None:
        self._state = "complete"
        self.complete = True

    def _rebuild_grid(self) -> None:
        if self.bounds is None:
            return
        g: list[list[int]] = [[0] * 3 for _ in range(3)]
        for c in self.candidates:
            cell = _classify(c.x_m, c.y_m, self.bounds)
            if cell is not None:
                g[cell[0]][cell[1]] += 1
        self.grid = g

    def _phase_label(self) -> str:
        if self._state == "scanning" and self._wp_index < len(_SEQUENCE):
            pose, _ = _SEQUENCE[self._wp_index]
            elapsed = 0.0 if self._scan_started_at is None else time.time() - self._scan_started_at
            return f"Scanning: {pose} ({elapsed:.1f}s / {SCAN_DURATION_S:.0f}s)"
        if self._state == "transit" and self._wp_index < len(_SEQUENCE):
            pose, should_scan = _SEQUENCE[self._wp_index]
            return f"{'→ Scan' if should_scan else '→ Transit'}: {pose}"
        if self._state == "returning":
            return "Returning to start"
        if self._state == "complete":
            return "Complete"
        return self._state


class ServiceLineDistributionScanMission:
    """Move to the selected side service line, spin once, and build a 3x3 grid.

    This is the first collection smoke test: it produces matrix distribution
    telemetry without starting the intake or target-by-target collection.
    """

    def __init__(
        self,
        ball_source_fn: BallSourceFn,
        boundary_path: Path | None = None,
    ) -> None:
        self._ball_source = ball_source_fn
        self._path = boundary_path or DEFAULT_BOUNDARY_FILE

        self.active: bool = False
        self.complete: bool = False
        self.side_id: str = "unknown"
        self.side_sign: int = -1
        self.candidates: list[_Cand] = []
        self.grid: list[list[int]] = [[0] * 3 for _ in range(3)]
        self.unassigned_candidates: int = 0
        self.service_pose_map: tuple[float, float] | None = None
        self.target_grid_cell: tuple[int, int] | None = None
        self.target_pose_map: tuple[float, float] | None = None

        self._state: str = "idle"
        self._elapsed_start: float | None = None
        self._scan_started_at: float | None = None
        self._last_scan_yaw: float | None = None
        self._scan_accumulated_rad: float = 0.0
        self._frame: CourtFrame | None = None
        self._bounds: HalfCourtBounds | None = None

    def start(self, rx: float, ry: float, _ryaw: float) -> None:
        frame, lines = self._load_v2_model()
        court_x, _court_y = frame.map_to_court(rx, ry)
        sign = -1 if court_x < 0.0 else 1
        service_candidates = sorted(float(v) for v in lines["service_x"])
        baseline_candidates = sorted(float(v) for v in lines["baselines_x"])
        sidelines = sorted(float(v) for v in lines["sidelines_y"])

        service_x = service_candidates[0] if sign < 0 else service_candidates[-1]
        baseline_x = baseline_candidates[0] if sign < 0 else baseline_candidates[-1]
        target_x, target_y = frame.court_to_map(service_x, 0.0)

        self.active = True
        self.complete = False
        self.side_sign = sign
        self.side_id = "side_neg_x" if sign < 0 else "side_pos_x"
        self.candidates = []
        self.grid = [[0] * 3 for _ in range(3)]
        self.unassigned_candidates = 0
        self.service_pose_map = (target_x, target_y)
        self.target_grid_cell = None
        self.target_pose_map = None
        self._state = "transit"
        self._elapsed_start = time.time()
        self._scan_started_at = None
        self._last_scan_yaw = None
        self._scan_accumulated_rad = 0.0
        self._frame = frame
        self._bounds = HalfCourtBounds(
            self.side_id,
            min(0.0, baseline_x),
            max(0.0, baseline_x),
            sidelines[0],
            sidelines[-1],
        )

    def reset(self) -> None:
        self.active = False
        self.complete = False
        self._state = "idle"
        self._scan_started_at = None
        self._last_scan_yaw = None
        self._scan_accumulated_rad = 0.0
        self.target_grid_cell = None
        self.target_pose_map = None

    def update(self, rx: float, ry: float, ryaw: float, _dt_s: float) -> ConceptACommand:
        if self._state == "transit":
            return self._step_transit(rx, ry, ryaw)
        if self._state == "scanning":
            return self._step_scan(rx, ry, ryaw)
        if self._state == "transit_to_grid_cell":
            return self._step_transit_to_grid_cell(rx, ry, ryaw)
        return _idle_cmd()

    def telemetry(self) -> dict:
        elapsed = 0.0 if self._elapsed_start is None else time.time() - self._elapsed_start
        scan_elapsed = 0.0 if self._scan_started_at is None else time.time() - self._scan_started_at
        progress = min(1.0, self._scan_accumulated_rad / (2 * math.pi))
        return {
            "active": self.active and not self.complete,
            "complete": self.complete,
            "phase": self._state,
            "phase_label": self._phase_label(),
            "side": self.side_id,
            "service_pose_map": (
                {"x_m": round(self.service_pose_map[0], 3), "y_m": round(self.service_pose_map[1], 3)}
                if self.service_pose_map is not None
                else None
            ),
            "target_grid_cell": (
                {"row": self.target_grid_cell[0], "col": self.target_grid_cell[1]}
                if self.target_grid_cell is not None
                else None
            ),
            "target_pose_map": (
                {"x_m": round(self.target_pose_map[0], 3), "y_m": round(self.target_pose_map[1], 3)}
                if self.target_pose_map is not None
                else None
            ),
            "scan_progress_pct": round(progress * 100.0, 1),
            "scan_accumulated_rad": round(self._scan_accumulated_rad, 3),
            "scan_elapsed_s": round(scan_elapsed, 1),
            "total_candidates": len(self.candidates),
            "assigned_candidates": sum(sum(row) for row in self.grid),
            "unassigned_candidates": self.unassigned_candidates,
            "candidates": self._candidate_telemetry(),
            "grid": [row[:] for row in self.grid],
            "elapsed_s": round(elapsed, 1),
        }

    def _candidate_telemetry(self) -> list[dict]:
        rows: list[dict] = []
        for idx, c in enumerate(self.candidates, start=1):
            item: dict = {
                "id": idx,
                "x_m": round(c.x_m, 3),
                "y_m": round(c.y_m, 3),
                "source": "collection_scan",
            }
            if self._frame is not None and self._bounds is not None:
                cx, cy = self._frame.map_to_court(c.x_m, c.y_m)
                cell = _classify(cx, cy, self._bounds)
                if cell is not None:
                    item["grid_cell"] = {"row": cell[0], "col": cell[1]}
            rows.append(item)
        return rows

    def _load_v2_model(self) -> tuple[CourtFrame, dict]:
        try:
            with self._path.open("r", encoding="utf-8") as f:
                data = json.load(f)
        except FileNotFoundError as exc:
            raise RuntimeError(
                f"Court survey file not found: {self._path} - run Map Court first."
            ) from exc
        except OSError as exc:
            raise RuntimeError(f"Cannot read court survey file: {exc}") from exc

        if data.get("schema") != "court_knowledge_model/v2" or data.get("status") != "OK":
            reason = data.get("failure_reason") or data.get("status") or data.get("schema")
            raise RuntimeError(f"Court survey v2 not ready - run Map Court first. Reason: {reason}")
        return _load_v2_court_frame(data), _load_v2_lines(data)

    def _step_transit(self, rx: float, ry: float, ryaw: float) -> ConceptACommand:
        if self.service_pose_map is None:
            self._finish()
            return _idle_cmd()
        tx, ty = self.service_pose_map
        cmd = _nav_to(rx, ry, ryaw, tx, ty)
        if cmd is not None:
            return cmd
        self._state = "scanning"
        self._scan_started_at = time.time()
        self._last_scan_yaw = ryaw
        self._scan_accumulated_rad = 0.0
        return _scan_cmd()

    def _step_scan(self, rx: float, ry: float, ryaw: float) -> ConceptACommand:
        self._sample_balls(rx, ry)
        if self._last_scan_yaw is not None:
            self._scan_accumulated_rad += abs(_angle_delta(ryaw, self._last_scan_yaw))
        self._last_scan_yaw = ryaw

        if self._scan_accumulated_rad >= 2 * math.pi:
            if self._select_best_grid_target():
                self._state = "transit_to_grid_cell"
            else:
                self._finish()
            return _scan_cmd()
        return ConceptACommand(
            state=CollectorState.SCAN,
            base=BaseCommand(0.0, NAV_MAX_TURN_RAD_S * 0.45),
            collector=CollectorCommand(0.0, False),
        )

    def _step_transit_to_grid_cell(self, rx: float, ry: float, ryaw: float) -> ConceptACommand:
        if self.target_pose_map is None:
            self._finish()
            return _idle_cmd()
        tx, ty = self.target_pose_map
        cmd = _nav_to(rx, ry, ryaw, tx, ty)
        if cmd is not None:
            return cmd
        self._finish()
        return _idle_cmd()

    def _sample_balls(self, rx: float, ry: float) -> None:
        for bx, by in self._ball_source():
            if math.hypot(bx - rx, by - ry) <= DETECTION_RANGE_M:
                _add_candidate(self.candidates, bx, by, COLLECTION_SCAN_CLUSTER_RADIUS_M)
        self._rebuild_grid()

    def _rebuild_grid(self) -> None:
        if self._frame is None or self._bounds is None:
            return
        g: list[list[int]] = [[0] * 3 for _ in range(3)]
        for c in self.candidates:
            cx, cy = self._frame.map_to_court(c.x_m, c.y_m)
            cell = _classify(cx, cy, self._bounds)
            if cell is not None:
                g[cell[0]][cell[1]] += 1
        self.grid = g
        self.unassigned_candidates = len(self.candidates) - sum(sum(row) for row in g)

    def _select_best_grid_target(self) -> bool:
        if self._frame is None or self._bounds is None:
            return False
        cell = self._best_grid_cell()
        if cell is None:
            return False
        court_x, court_y = self._cell_center_court(*cell)
        self.target_grid_cell = cell
        self.target_pose_map = self._frame.court_to_map(court_x, court_y)
        return True

    def _best_grid_cell(self) -> tuple[int, int] | None:
        best_cell: tuple[int, int] | None = None
        best_count = 0
        for row, values in enumerate(self.grid):
            for col, count in enumerate(values):
                if count > best_count:
                    best_count = count
                    best_cell = (row, col)
        return best_cell

    def _cell_center_court(self, row: int, col: int) -> tuple[float, float]:
        assert self._bounds is not None
        b = self._bounds
        row_offset = (row + 0.5) * b.span_x / 3.0
        if b.side in {"left", "side_neg_x"}:
            x_m = b.x_min + row_offset
        else:
            x_m = b.x_max - row_offset
        y_m = b.y_max - (col + 0.5) * b.span_y / 3.0
        return x_m, y_m

    def _finish(self) -> None:
        self._state = "complete"
        self.active = False
        self.complete = True

    def _phase_label(self) -> str:
        if self._state == "transit":
            return f"Navigating to {self.side_id} service line"
        if self._state == "scanning":
            progress = min(100.0, self._scan_accumulated_rad / (2 * math.pi) * 100.0)
            return f"360 distribution scan ({progress:.0f}%)"
        if self._state == "transit_to_grid_cell":
            if self.target_grid_cell is None:
                return "Navigating to estimated collection cell"
            row, col = self.target_grid_cell
            return f"Navigating to estimated collection cell r{row + 1}c{col + 1}"
        if self._state == "complete":
            return "Distribution scan complete"
        return self._state
