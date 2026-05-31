"""Half-court mapping mission: navigate scan poses and build a 3×3 ball-count grid.

No collection — pure mapping. Ball source is injected so Phase 1 uses Supervisor
ground-truth positions; Phase 2 swaps in mapped_balls from camera detection.
"""

from __future__ import annotations

import math
import os
import time
from dataclasses import dataclass
from typing import Callable, Optional, Protocol, Tuple

from config_utils import _env_float
from collector import BaseCommand, CollectorCommand, CollectorState, ConceptACommand


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


class BoundaryProvider(Protocol):
    """Swap this out to replace Supervisor ground-truth with LiDAR detection."""

    def get_bounds(self, side: str) -> HalfCourtBounds: ...


class SupervisorBoundaryProvider:
    """Returns fixed bounds from Webots court constants.

    Replace with a LiDAR implementation for physical deployment.
    """

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


# ── 3×3 grid ───────────────────────────────────────────────────────────────────

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
    if b.side == "left":
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


def _add_candidate(cands: list[_Cand], x_m: float, y_m: float) -> None:
    """Add (x, y) only if no existing candidate is within CLUSTER_RADIUS_M."""
    for c in cands:
        if math.hypot(c.x_m - x_m, c.y_m - y_m) < CLUSTER_RADIUS_M:
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
