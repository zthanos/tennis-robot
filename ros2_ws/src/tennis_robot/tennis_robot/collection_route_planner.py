"""Route planner for the collect_route mission.

Pure geometry/ordering library — no rclpy. Ported and adapted from the offline
route benchmark primitives (scripts/route_benchmark.py: ball_risk,
pickup_targets, obstacle_clearance). Obstacle-aware pathing between stops is
Nav2's job; ordering here uses euclidean cost over the court knowledge model
(runtime/court_boundary.json, schema court_knowledge_model/v2).
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, field
from pathlib import Path

from tennis_robot.config_utils import _env_float

# Robot footprint radius, matches scripts/route_benchmark.py ROBOT_RADIUS_M.
ROBOT_RADIUS_M = 0.36

# Funnel mouth 260-340 mm (docs/concept-a-funnel-lift-wheel-plan.md): the
# capture corridor swept while driving standoff -> ball must stay clear of
# fence/net by half the mouth width.
FUNNEL_CORRIDOR_HALF_WIDTH_M = 0.17
CAPTURE_OVERRUN_M = 0.30

_LATERAL_CANDIDATE_HEADINGS = 16


def _env_bool(name: str, default: bool) -> bool:
    val = os.getenv(name)
    if val is None:
        return default
    return val.strip().lower() in ("1", "true", "yes", "on")


@dataclass(frozen=True)
class RoutePlannerConfig:
    scan_range_m: float = 9.0
    standoff_m: float = 1.3
    boundary_margin_m: float = 0.9
    robot_radius_m: float = ROBOT_RADIUS_M
    # Extra clearance for Nav2 GOAL poses beyond the physical radius: goals
    # inside the costmap inflation band get rejected outright (run-5: the
    # 0.36 m-from-net lateral standoff was permanently unreachable).
    goal_clearance_margin_m: float = 0.15
    insertion_max_detour_m: float = 3.0
    two_opt: bool = True
    two_opt_max_passes: int = 4

    @classmethod
    def from_env(cls) -> "RoutePlannerConfig":
        return cls(
            scan_range_m=_env_float("COLLECT_ROUTE_SCAN_RANGE_M", 9.0),
            standoff_m=_env_float("COLLECT_ROUTE_STANDOFF_M", 1.3),
            boundary_margin_m=_env_float("COLLECT_ROUTE_BOUNDARY_MARGIN_M", 0.9),
            goal_clearance_margin_m=_env_float("COLLECT_ROUTE_GOAL_CLEARANCE_M", 0.15),
            insertion_max_detour_m=_env_float("COLLECT_ROUTE_INSERTION_MAX_DETOUR_M", 3.0),
            two_opt=_env_bool("COLLECT_ROUTE_TWO_OPT", True),
        )


@dataclass(frozen=True)
class ApproachPose:
    x_m: float
    y_m: float
    yaw_rad: float
    mode: str  # "direct" | "lateral"
    risk: str  # "normal" | "net_wall" | "obstacle"


@dataclass
class RouteStop:
    ball_id: int
    ball_x_m: float
    ball_y_m: float
    approach: ApproachPose
    order: int
    attempts: int = 0
    status: str = "pending"  # pending|active|collected|skipped|missing
    # Plan-time position: goal refreshes follow map drift only up to a sanity
    # cap around this point (chain-merged entries can wander metres away).
    planned_x_m: float = 0.0
    planned_y_m: float = 0.0
    # Sweep-only exact run-in.  It can be shorter than SWEEP_RUN_IN_M for
    # closely-spaced balls so the route never needs to turn back.
    sweep_entry_x_m: float | None = None
    sweep_entry_y_m: float | None = None

    def __post_init__(self) -> None:
        if self.planned_x_m == 0.0 and self.planned_y_m == 0.0:
            self.planned_x_m = self.ball_x_m
            self.planned_y_m = self.ball_y_m


# ── Geometry helpers ───────────────────────────────────────────────────────────


def _dist(ax: float, ay: float, bx: float, by: float) -> float:
    return math.hypot(ax - bx, ay - by)


def _point_segment_distance(
    px: float, py: float, ax: float, ay: float, bx: float, by: float
) -> float:
    abx, aby = bx - ax, by - ay
    denom = abx * abx + aby * aby
    if denom <= 1e-12:
        return _dist(px, py, ax, ay)
    t = max(0.0, min(1.0, ((px - ax) * abx + (py - ay) * aby) / denom))
    return _dist(px, py, ax + t * abx, ay + t * aby)


def _segment_tangent(ax: float, ay: float, bx: float, by: float) -> tuple[float, float]:
    dx, dy = bx - ax, by - ay
    norm = math.hypot(dx, dy)
    if norm <= 1e-9:
        return (1.0, 0.0)
    return (dx / norm, dy / norm)


def _point_in_polygon(px: float, py: float, corners: list[tuple[float, float]]) -> bool:
    inside = False
    n = len(corners)
    for i in range(n):
        ax, ay = corners[i]
        bx, by = corners[(i + 1) % n]
        if (ay > py) != (by > py):
            x_cross = ax + (py - ay) * (bx - ax) / (by - ay)
            if px < x_cross:
                inside = not inside
    return inside


# ── Court knowledge model ──────────────────────────────────────────────────────


@dataclass(frozen=True)
class _ObstacleCircle:
    x_m: float
    y_m: float
    radius_m: float


class CourtModel:
    """Boundary/obstacle queries in the map frame, from court_boundary.json v2."""

    def __init__(
        self,
        fence_corners: list[tuple[float, float]],
        net_segment: tuple[tuple[float, float], tuple[float, float]],
        obstacles: list[_ObstacleCircle] = (),
    ) -> None:
        self.fence_corners = list(fence_corners)
        self.net_segment = net_segment
        self.obstacles = list(obstacles)

    @classmethod
    def from_boundary_file(cls, path: Path) -> "CourtModel | None":
        try:
            with Path(path).open("r", encoding="utf-8") as f:
                data = json.load(f)
        except (FileNotFoundError, OSError, json.JSONDecodeError):
            return None
        try:
            return cls.from_boundary_data(data)
        except (KeyError, TypeError, ValueError, IndexError):
            return None

    @classmethod
    def from_boundary_data(cls, data: dict) -> "CourtModel":
        corners = [
            (float(c["x_m"]), float(c["y_m"])) for c in data["fence"]["corners"]
        ]
        if len(corners) < 3:
            raise ValueError("fence.corners needs at least 3 points")
        net = data["net"]
        posts = net.get("posts") or []
        if len(posts) >= 2:
            net_a = (float(posts[0]["x_m"]), float(posts[0]["y_m"]))
            net_b = (float(posts[1]["x_m"]), float(posts[1]["y_m"]))
        else:
            center = net["center"]
            axis_w = net["axis_width"]
            half_span = float(net.get("span_m", 11.0)) / 2.0
            net_a = (
                float(center["x_m"]) - half_span * float(axis_w["x_m"]),
                float(center["y_m"]) - half_span * float(axis_w["y_m"]),
            )
            net_b = (
                float(center["x_m"]) + half_span * float(axis_w["x_m"]),
                float(center["y_m"]) + half_span * float(axis_w["y_m"]),
            )
        obstacles: list[_ObstacleCircle] = []
        for entry in data.get("obstacles") or []:
            center = entry.get("center") or {}
            size = entry.get("size_m") or {}
            try:
                x = float(center["x_m"])
                y = float(center["y_m"])
            except (KeyError, TypeError, ValueError):
                continue
            radius = 0.5 * math.hypot(float(size.get("w", 0.3)), float(size.get("h", 0.3)))
            obstacles.append(_ObstacleCircle(x, y, radius))
        return cls(corners, (net_a, net_b), obstacles)

    # -- queries --

    def _fence_edges(self) -> list[tuple[float, float, float, float]]:
        n = len(self.fence_corners)
        return [
            (*self.fence_corners[i], *self.fence_corners[(i + 1) % n])
            for i in range(n)
        ]

    def fence_distance(self, x: float, y: float) -> float:
        return min(
            _point_segment_distance(x, y, ax, ay, bx, by)
            for ax, ay, bx, by in self._fence_edges()
        )

    def net_distance(self, x: float, y: float) -> float:
        (ax, ay), (bx, by) = self.net_segment
        return _point_segment_distance(x, y, ax, ay, bx, by)

    def obstacle_clearance(self, x: float, y: float) -> float:
        if not self.obstacles:
            return math.inf
        return min(
            _dist(x, y, o.x_m, o.y_m) - o.radius_m for o in self.obstacles
        )

    def nearest_boundary(self, x: float, y: float) -> tuple[float, tuple[float, float]]:
        """Distance and unit tangent of the closest fence edge or the net."""
        best_dist = math.inf
        best_tangent = (1.0, 0.0)
        for ax, ay, bx, by in self._fence_edges():
            d = _point_segment_distance(x, y, ax, ay, bx, by)
            if d < best_dist:
                best_dist = d
                best_tangent = _segment_tangent(ax, ay, bx, by)
        (nax, nay), (nbx, nby) = self.net_segment
        d = _point_segment_distance(x, y, nax, nay, nbx, nby)
        if d < best_dist:
            best_dist = d
            best_tangent = _segment_tangent(nax, nay, nbx, nby)
        return best_dist, best_tangent

    def contains(self, x: float, y: float) -> bool:
        return _point_in_polygon(x, y, self.fence_corners)

    def same_side(
        self, ax: float, ay: float, bx: float, by: float, clearance_m: float = 0.25
    ) -> bool:
        """True when both points sit on the same side of the REAL net line.

        Replaces the legacy across_net(net_x=0) convention: in the SLAM map
        frame the net is wherever the survey found it (e.g. x≈8.08), so side
        classification must use the surveyed net segment, not world x=0.
        Points within clearance of the line count as same-side (matching
        across_net's behavior at the net).
        """
        (nax, nay), (nbx, nby) = self.net_segment
        tx, ty = _segment_tangent(nax, nay, nbx, nby)
        sa = tx * (ay - nay) - ty * (ax - nax)
        sb = tx * (by - nay) - ty * (bx - nax)
        if abs(sa) < clearance_m or abs(sb) < clearance_m:
            return True
        return sa * sb > 0

    def ball_risk(self, x: float, y: float, margin_m: float) -> str:
        if self.obstacle_clearance(x, y) <= ROBOT_RADIUS_M + margin_m:
            return "obstacle"
        if self.net_distance(x, y) <= margin_m or self.fence_distance(x, y) <= margin_m:
            return "net_wall"
        return "normal"

    def pose_is_free(self, x: float, y: float, robot_radius_m: float) -> bool:
        if not _point_in_polygon(x, y, self.fence_corners):
            return False
        if self.fence_distance(x, y) < robot_radius_m:
            return False
        if self.net_distance(x, y) < robot_radius_m:
            return False
        return self.obstacle_clearance(x, y) >= 0.0


# ── Route ordering ─────────────────────────────────────────────────────────────


def _path_length(points: list[tuple[float, float]]) -> float:
    return sum(
        _dist(*points[i], *points[i + 1]) for i in range(len(points) - 1)
    )


def order_route(
    start_xy: tuple[float, float],
    balls: list[tuple[int, float, float]],
    cfg: RoutePlannerConfig,
) -> list[int]:
    """Order ball ids by greedy nearest-neighbor from start, then 2-opt polish.

    Open path (no return to start). Runs once at plan time — never in the
    controller tick.
    """
    if not balls:
        return []
    remaining = list(balls)
    ordered: list[tuple[int, float, float]] = []
    cx, cy = start_xy
    while remaining:
        best = min(remaining, key=lambda b: _dist(cx, cy, b[1], b[2]))
        remaining.remove(best)
        ordered.append(best)
        cx, cy = best[1], best[2]

    if cfg.two_opt and len(ordered) >= 3:
        ordered = _two_opt(start_xy, ordered, cfg.two_opt_max_passes)
    return [b[0] for b in ordered]


def _two_opt(
    start_xy: tuple[float, float],
    ordered: list[tuple[int, float, float]],
    max_passes: int,
) -> list[tuple[int, float, float]]:
    def leg(a: tuple[float, float], b: tuple[int, float, float]) -> float:
        return _dist(a[0], a[1], b[1], b[2])

    for _ in range(max_passes):
        improved = False
        n = len(ordered)
        for i in range(n - 1):
            prev = start_xy if i == 0 else (ordered[i - 1][1], ordered[i - 1][2])
            for j in range(i + 1, n):
                # Reversing ordered[i..j] replaces edges (prev,i) and (j,j+1)
                # with (prev,j) and (i,j+1); the tail edge vanishes at j==n-1.
                old = leg(prev, ordered[i])
                new = leg(prev, ordered[j])
                if j + 1 < n:
                    nxt = (ordered[j + 1][1], ordered[j + 1][2])
                    old += _dist(ordered[j][1], ordered[j][2], *nxt)
                    new += _dist(ordered[i][1], ordered[i][2], *nxt)
                if new + 1e-9 < old:
                    ordered[i : j + 1] = reversed(ordered[i : j + 1])
                    improved = True
        if not improved:
            break
    return ordered


# ── Approach poses ─────────────────────────────────────────────────────────────


def _corridor_clear(
    court: CourtModel,
    start: tuple[float, float],
    heading: tuple[float, float],
    length_m: float,
    half_width_m: float,
) -> bool:
    """The funnel corridor swept from standoff through the ball must stay
    inside the fence and off the net/obstacles."""
    hx, hy = heading
    px, py = -hy, hx  # corridor lateral axis
    steps = max(2, math.ceil(length_m / 0.15))
    for i in range(steps + 1):
        t = (i / steps) * length_m
        cx, cy = start[0] + hx * t, start[1] + hy * t
        for offset in (-half_width_m, 0.0, half_width_m):
            x, y = cx + px * offset, cy + py * offset
            if not _point_in_polygon(x, y, court.fence_corners):
                return False
            if court.net_distance(x, y) < half_width_m * 0.5:
                return False
            if court.obstacle_clearance(x, y) < 0.0:
                return False
    return True


def approach_pose_for_ball(
    ball_xy: tuple[float, float],
    from_xy: tuple[float, float],
    court: "CourtModel | None",
    cfg: RoutePlannerConfig,
) -> ApproachPose:
    bx, by = ball_xy
    fx, fy = from_xy
    dx, dy = bx - fx, by - fy
    norm = math.hypot(dx, dy)
    incoming = (dx / norm, dy / norm) if norm > 1e-6 else (1.0, 0.0)

    risk = court.ball_risk(bx, by, cfg.boundary_margin_m) if court else "normal"
    if risk == "normal" or court is None:
        hx, hy = incoming
        return ApproachPose(
            x_m=bx - hx * cfg.standoff_m,
            y_m=by - hy * cfg.standoff_m,
            yaw_rad=math.atan2(hy, hx),
            mode="direct",
            risk=risk,
        )

    _, tangent = court.nearest_boundary(bx, by)
    corridor_len = cfg.standoff_m + CAPTURE_OVERRUN_M
    viable: list[tuple[float, float, tuple[float, float]]] = []
    fallback: list[tuple[float, tuple[float, float]]] = []
    for k in range(_LATERAL_CANDIDATE_HEADINGS):
        angle = (2.0 * math.pi * k) / _LATERAL_CANDIDATE_HEADINGS
        hx, hy = math.cos(angle), math.sin(angle)
        sx, sy = bx - hx * cfg.standoff_m, by - hy * cfg.standoff_m
        clearance = min(
            court.fence_distance(sx, sy),
            court.net_distance(sx, sy),
            court.obstacle_clearance(sx, sy),
        )
        fallback.append((clearance, (hx, hy)))
        if not court.pose_is_free(
            sx, sy, cfg.robot_radius_m + cfg.goal_clearance_margin_m
        ):
            continue
        if not _corridor_clear(
            court, (sx, sy), (hx, hy), corridor_len, FUNNEL_CORRIDOR_HALF_WIDTH_M
        ):
            continue
        parallelism = abs(hx * tangent[0] + hy * tangent[1])
        travel = _dist(fx, fy, sx, sy)
        viable.append((parallelism, -travel, (hx, hy)))

    if viable:
        _, _, (hx, hy) = max(viable)
    else:
        # No candidate satisfied both checks; take the best-clearance heading
        # and keep the risk flag so the mission caps attempts.
        _, (hx, hy) = max(fallback)
    return ApproachPose(
        x_m=bx - hx * cfg.standoff_m,
        y_m=by - hy * cfg.standoff_m,
        yaw_rad=math.atan2(hy, hx),
        mode="lateral",
        risk=risk,
    )


# ── Dynamic insertion & console export ─────────────────────────────────────────


def cheapest_insertion(
    route_points: list[tuple[float, float]],
    new_point: tuple[float, float],
    start_index: int,
) -> tuple[int, float]:
    """Cheapest position to insert new_point into route_points.

    Returns (insert_index, detour_m); insert_index is where the new point goes
    in route_points, never below start_index (the leg in progress and completed
    legs stay untouched). Appending at the end is always a candidate.
    """
    n = len(route_points)
    first = max(1, start_index)
    if n == 0:
        return 0, 0.0
    best_index = n
    best_delta = _dist(*route_points[-1], *new_point)
    for i in range(first, n):
        ax, ay = route_points[i - 1]
        bx, by = route_points[i]
        delta = (
            _dist(ax, ay, *new_point)
            + _dist(*new_point, bx, by)
            - _dist(ax, ay, bx, by)
        )
        if delta < best_delta - 1e-9:
            best_delta = delta
            best_index = i
    return best_index, best_delta


def route_polyline(
    robot_xy: tuple[float, float], stops: list[RouteStop]
) -> list[dict]:
    """Polyline for the console Collection Map (map.route contract)."""
    points = [{"x_m": round(robot_xy[0], 3), "y_m": round(robot_xy[1], 3)}]
    for stop in stops:
        if stop.status in ("collected", "skipped", "missing"):
            continue
        points.append(
            {"x_m": round(stop.approach.x_m, 3), "y_m": round(stop.approach.y_m, 3)}
        )
        points.append(
            {"x_m": round(stop.ball_x_m, 3), "y_m": round(stop.ball_y_m, 3)}
        )
    return points


def remaining_route_length_m(
    robot_xy: tuple[float, float], stops: list[RouteStop]
) -> float:
    points = [(p["x_m"], p["y_m"]) for p in route_polyline(robot_xy, stops)]
    return round(_path_length(points), 2)


# ── Sweep route: collection decoupled from the route (debug log #21) ───────────

SWEEP_RUN_IN_M = 1.0
SWEEP_OVERRUN_M = 0.35
# Preserve a small moving link before the next crossing.  Without this cap,
# closely-spaced balls put the next 1 m run-in BEHIND the previous exit,
# forcing a rotate/reverse despite a drive-through route.
SWEEP_MIN_LINK_M = 0.08


@dataclass(frozen=True)
class SweepLeg:
    """One drive-through pass: straight run-in, ball centred in the funnel,
    exit past it. No stop — the route continues regardless of capture."""

    ball_id: int
    ball_x_m: float
    ball_y_m: float
    entry_x_m: float
    entry_y_m: float
    exit_x_m: float
    exit_y_m: float
    yaw_rad: float
    mode: str  # "direct" | "lateral"
    risk: str


def sweep_route(
    start_xy: tuple[float, float],
    balls: list[tuple[int, float, float]],
    court: "CourtModel | None",
    cfg: RoutePlannerConfig,
) -> list[SweepLeg]:
    """Ordered drive-through legs over every ball.

    Heading per ball comes from approach_pose_for_ball (incoming direction,
    or obstacle-parallel lateral with corridor/pose checks — rule R1); the
    run-in starts up to SWEEP_RUN_IN_M before the ball so the funnel receives
    it centred. For a following, closely-spaced ball the run-in is shortened
    instead of being placed behind the previous exit; the exit lies
    SWEEP_OVERRUN_M past the ball (drive-through, no in-place rotation)."""
    order = order_route(start_xy, balls, cfg)
    by_id = {b[0]: (b[1], b[2]) for b in balls}
    legs: list[SweepLeg] = []
    cx, cy = start_xy
    for ball_id in order:
        bx, by = by_id[ball_id]
        pose = approach_pose_for_ball((bx, by), (cx, cy), court, cfg)
        hx, hy = math.cos(pose.yaw_rad), math.sin(pose.yaw_rad)
        # The next entry must be ahead of the prior exit along the crossing
        # heading.  A fixed 1 m run-in fails this whenever two balls are
        # closer than run-in + overrun (seen in live route #1 → #2).
        forward_to_ball = (bx - cx) * hx + (by - cy) * hy
        run_in_m = min(
            SWEEP_RUN_IN_M,
            max(0.0, forward_to_ball - SWEEP_MIN_LINK_M),
        )
        legs.append(
            SweepLeg(
                ball_id=ball_id,
                ball_x_m=bx,
                ball_y_m=by,
                entry_x_m=bx - hx * run_in_m,
                entry_y_m=by - hy * run_in_m,
                exit_x_m=bx + hx * SWEEP_OVERRUN_M,
                exit_y_m=by + hy * SWEEP_OVERRUN_M,
                yaw_rad=pose.yaw_rad,
                mode=pose.mode,
                risk=pose.risk,
            )
        )
        cx, cy = legs[-1].exit_x_m, legs[-1].exit_y_m
    return legs
