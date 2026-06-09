"""Probabilistic ball registry — pure Python, no Webots dependency.

Tracks, merges, and prunes ball observations in world coordinates.
The controller manages active_target_id; this module owns everything
about what balls exist and where they are.
"""

from __future__ import annotations

import math
import time as _time
from dataclasses import dataclass, field
from typing import Any


# ── Court geometry helpers ─────────────────────────────────────────────────────

def across_net(
    robot_x_m: float,
    ball_x_m: float,
    net_x_m: float = 0.0,
    net_side_clearance_m: float = 0.25,
) -> bool:
    """True if robot and ball are on opposite sides of the net."""
    if abs(robot_x_m - net_x_m) < net_side_clearance_m:
        return False
    if abs(ball_x_m - net_x_m) < net_side_clearance_m:
        return False
    return (robot_x_m - net_x_m) * (ball_x_m - net_x_m) < 0


# ── Data ───────────────────────────────────────────────────────────────────────

@dataclass
class MappedBall:
    id: int
    x_m: float
    y_m: float
    confidence: float
    first_seen_s: float
    last_seen_s: float
    source: str = "unknown"
    seen_count: int = 1
    state: str = "detected"


@dataclass(frozen=True)
class BallMapConfig:
    merge_distance_m: float = 0.65
    max_merge_distance_m: float = 1.6
    min_seen_count: int = 5
    max_create_distance_m: float = 3.0
    stale_after_s: float = 45.0
    court_max_x_m: float = 11.885
    court_max_y_m: float = 5.485
    court_ball_margin_m: float = 3.2
    net_x_m: float = 0.0
    net_side_clearance_m: float = 0.25
    supervised_fov_rad: float = 1.05
    supervised_max_range_m: float = 8.0


# ── BallObservationInput stub for type-checking only ──────────────────────────

try:
    from tennis_robot.collector import BallObservationInput
except ImportError:  # pragma: no cover
    BallObservationInput = Any  # type: ignore[assignment,misc]


# ── BallMap ────────────────────────────────────────────────────────────────────

class BallMap:
    """Probabilistic registry of detected ball positions on the court.

    All methods that query robot pose accept explicit (robot_x, robot_y,
    robot_yaw) so this class has no Webots dependency.
    """

    def __init__(self, config: BallMapConfig | None = None) -> None:
        self.config = config or BallMapConfig()
        self.balls: dict[int, MappedBall] = {}
        self._next_id = 1
        self._seeded_signature: tuple[tuple[float, float], ...] = ()

    def __len__(self) -> int:
        return len(self.balls)

    def __bool__(self) -> bool:
        return bool(self.balls)

    # ── Mutation ───────────────────────────────────────────────────────────────

    def reset(self) -> None:
        self.balls.clear()
        self._next_id = 1
        self._seeded_signature = ()

    def update(self, observation: BallObservationInput, now: float) -> tuple[int | None, bool]:
        """Merge observation into the map.

        Returns (ball_id, is_new).  is_new=True means a new entry was created.
        Returns (None, False) when the observation is rejected.
        """
        cfg = self.config
        if not observation.visible or observation.world_x_m is None or observation.world_y_m is None:
            return None, False
        if (
            abs(observation.world_x_m) > cfg.court_max_x_m + cfg.court_ball_margin_m
            or abs(observation.world_y_m) > cfg.court_max_y_m + cfg.court_ball_margin_m
        ):
            return None, False

        merge_dist = self._merge_distance(observation)
        best_id: int | None = None
        best_dist = merge_dist
        for ball_id, ball in self.balls.items():
            if ball.state == "collected":
                continue
            dist = math.hypot(ball.x_m - observation.world_x_m, ball.y_m - observation.world_y_m)
            if dist < best_dist:
                best_id = ball_id
                best_dist = dist

        if best_id is None:
            if observation.distance_m > cfg.max_create_distance_m:
                return None, False
            ball_id = self._next_id
            self._next_id += 1
            self.balls[ball_id] = MappedBall(
                id=ball_id,
                x_m=observation.world_x_m,
                y_m=observation.world_y_m,
                confidence=observation.confidence,
                first_seen_s=now,
                last_seen_s=now,
                source=observation.source,
            )
            return ball_id, True

        ball = self.balls[best_id]
        # Weight grows as robot approaches: far observations barely move stored
        # position; close ones snap it toward the more accurate estimate.
        close_factor = max(0.0, 1.0 - observation.distance_m / cfg.max_create_distance_m)
        weight = min(0.75, max(0.12, observation.confidence * 0.35 + close_factor * 0.5))
        ball.x_m = ball.x_m * (1.0 - weight) + observation.world_x_m * weight
        ball.y_m = ball.y_m * (1.0 - weight) + observation.world_y_m * weight
        ball.confidence = max(ball.confidence, observation.confidence)
        ball.last_seen_s = now
        ball.seen_count += 1
        ball.source = observation.source
        ball.state = "detected"
        return best_id, False

    def set_state(self, ball_id: int, state: str) -> None:
        if ball_id in self.balls:
            self.balls[ball_id].state = state

    def mark_nearest_collected(
        self,
        robot_x: float,
        robot_y: float,
        now: float,
    ) -> int | None:
        """Mark the nearest active ball as collected. Returns its id, or None."""
        if not self.balls:
            return None
        active = [b for b in self.balls.values() if b.state != "collected"]
        if not active:
            return None
        nearest = min(active, key=lambda b: math.hypot(b.x_m - robot_x, b.y_m - robot_y))
        nearest.state = "collected"
        nearest.last_seen_s = now
        return nearest.id

    def seed_from_candidates(self, candidates: list[Any], now: float) -> int:
        """Populate map from mission candidate list. Idempotent; returns count seeded."""
        if not candidates:
            return 0
        signature: tuple[tuple[float, float], ...] = tuple(
            sorted((round(c.x_m, 2), round(c.y_m, 2)) for c in candidates)
        )
        if signature == self._seeded_signature and self.balls:
            return 0
        self.reset()
        for candidate in candidates:
            ball_id = self._next_id
            self._next_id += 1
            self.balls[ball_id] = MappedBall(
                id=ball_id,
                x_m=candidate.x_m,
                y_m=candidate.y_m,
                confidence=0.95,
                first_seen_s=now,
                last_seen_s=now,
                source="map_mission",
                seen_count=self.config.min_seen_count,
            )
        self._seeded_signature = signature
        return len(self.balls)

    def prune_phantoms(self, now: float) -> None:
        """Remove/merge pending entries that duplicate confirmed entries."""
        cfg = self.config
        confirmed: list[MappedBall] = []
        pending: list[MappedBall] = []
        for ball in self.balls.values():
            if ball.state == "collected":
                continue
            if now - ball.last_seen_s > cfg.stale_after_s:
                continue
            (confirmed if ball.seen_count >= cfg.min_seen_count else pending).append(ball)

        to_remove: set[int] = set()
        for p in pending:
            for c in confirmed:
                if math.hypot(p.x_m - c.x_m, p.y_m - c.y_m) < cfg.merge_distance_m:
                    to_remove.add(p.id)
                    break

        survivors = [p for p in pending if p.id not in to_remove]
        survivors.sort(key=lambda b: b.seen_count, reverse=True)
        absorbed: set[int] = set()
        for i, dominant in enumerate(survivors):
            if dominant.id in absorbed:
                continue
            for weak in survivors[i + 1:]:
                if weak.id in absorbed:
                    continue
                if math.hypot(dominant.x_m - weak.x_m, dominant.y_m - weak.y_m) < cfg.merge_distance_m:
                    dominant.seen_count += weak.seen_count
                    dominant.confidence = max(dominant.confidence, weak.confidence)
                    if dominant.source != "oak_depth" and weak.source == "oak_depth":
                        dominant.source = weak.source
                    dominant.last_seen_s = max(dominant.last_seen_s, weak.last_seen_s)
                    absorbed.add(weak.id)
        to_remove.update(absorbed)
        for ball_id in to_remove:
            del self.balls[ball_id]

    # ── Queries ────────────────────────────────────────────────────────────────

    def nearest_target_id(
        self,
        robot_x: float,
        robot_y: float,
        now: float,
    ) -> int | None:
        """Return id of nearest confirmed same-side uncollected ball."""
        cfg = self.config
        candidates = [
            b for b in self.balls.values()
            if b.state not in {"collected", "collection_failed"}
            and b.seen_count >= cfg.min_seen_count
            and now - b.last_seen_s <= cfg.stale_after_s
            and not across_net(robot_x, b.x_m, cfg.net_x_m, cfg.net_side_clearance_m)
        ]
        if not candidates:
            return None
        return min(candidates, key=lambda b: math.hypot(b.x_m - robot_x, b.y_m - robot_y)).id

    def observation_from_target(
        self,
        ball_id: int | None,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        now: float,
    ) -> BallObservationInput | None:
        """Convert stored ball position to a live observation in robot frame."""
        if ball_id is None:
            return None
        ball = self.balls.get(ball_id)
        if ball is None or ball.state in {"collected", "collection_failed"}:
            return None
        if now - ball.last_seen_s > self.config.stale_after_s:
            return None
        dx = ball.x_m - robot_x
        dy = ball.y_m - robot_y
        local_x = math.cos(-robot_yaw) * dx - math.sin(-robot_yaw) * dy
        local_y = math.sin(-robot_yaw) * dx + math.cos(-robot_yaw) * dy
        if local_x <= -0.1:
            return None
        return BallObservationInput(
            visible=True,
            bearing_rad=math.atan2(local_y, local_x),
            distance_m=math.hypot(local_x, local_y),
            confidence=ball.confidence,
            source=ball.source,
            robot_x_m=local_x,
            robot_y_m=local_y,
            world_x_m=ball.x_m,
            world_y_m=ball.y_m,
        )

    def all_collected(self, now: float) -> bool:
        """True when all confirmed balls have been collected and map is non-empty."""
        if not self.balls:
            return False
        cfg = self.config
        ever_confirmed = any(b.seen_count >= cfg.min_seen_count for b in self.balls.values())
        if not ever_confirmed:
            return False
        active = [
            b for b in self.balls.values()
            if b.state not in {"collected", "collection_failed"}
            and b.seen_count >= cfg.min_seen_count
            and now - b.last_seen_s <= cfg.stale_after_s
        ]
        return len(active) == 0

    def rows_for_route(
        self,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        now: float,
    ) -> tuple[list[dict], list[MappedBall]]:
        """Return (display_rows, confirmed_same_side_balls) for status/route planning."""
        cfg = self.config
        rows: list[dict] = []
        same_side: list[MappedBall] = []
        for ball in sorted(self.balls.values(), key=lambda b: b.id):
            if ball.state == "collected":
                continue
            age_s = now - ball.last_seen_s
            if age_s > cfg.stale_after_s:
                continue
            confirmed = ball.seen_count >= cfg.min_seen_count
            is_across = across_net(robot_x, ball.x_m, cfg.net_x_m, cfg.net_side_clearance_m)
            local_x = math.cos(-robot_yaw) * (ball.x_m - robot_x) - math.sin(-robot_yaw) * (ball.y_m - robot_y)
            local_y = math.sin(-robot_yaw) * (ball.x_m - robot_x) + math.cos(-robot_yaw) * (ball.y_m - robot_y)
            distance_m = math.hypot(local_x, local_y)
            bearing_rad = math.atan2(local_y, local_x)
            visible_candidate = (
                not is_across
                and local_x > 0
                and abs(bearing_rad) <= cfg.supervised_fov_rad / 2
                and distance_m <= cfg.supervised_max_range_m
            )
            rows.append({
                "id": ball.id,
                "x_m": ball.x_m,
                "y_m": ball.y_m,
                "side": "across_net" if is_across else "same_side",
                "visible_candidate": visible_candidate,
                "confirmed": confirmed,
                "planned": False,
                "order": None,
                "risk": None,
                "source": ball.source,
                "confidence": ball.confidence,
                "seen_count": ball.seen_count,
                "age_s": age_s,
            })
            if confirmed and not is_across:
                same_side.append(ball)
        return rows, same_side

    # ── Internal ───────────────────────────────────────────────────────────────

    def _merge_distance(self, observation: BallObservationInput) -> float:
        cfg = self.config
        if not math.isfinite(observation.distance_m):
            return cfg.merge_distance_m
        distance_margin = observation.distance_m * 0.28
        confidence_margin = (1.0 - max(0.0, min(1.0, observation.confidence))) * 0.35
        return max(
            cfg.merge_distance_m,
            min(cfg.max_merge_distance_m, distance_margin + confidence_margin),
        )
