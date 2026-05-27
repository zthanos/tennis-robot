#!/usr/bin/env python3
"""Monte Carlo benchmark for tennis robot half-court route planning."""

from __future__ import annotations

import argparse
import csv
import heapq
import json
import math
import random
from dataclasses import asdict, dataclass
from pathlib import Path
from statistics import mean


COURT_LENGTH_M = 23.77
COURT_WIDTH_M = 10.97
COURT_SERVICE_M = 6.40
GRID_M = 0.35
ROBOT_RADIUS_M = 0.36
NET_CLEARANCE_X_M = 0.35


@dataclass(frozen=True)
class Bounds:
    min_x: float
    max_x: float
    min_y: float
    max_y: float


@dataclass
class Point:
    x: float
    y: float


@dataclass
class Obstacle:
    kind: str
    label: str
    x: float
    y: float
    radius: float = 0.0
    width: float = 0.0
    height: float = 0.0
    human: bool = False


@dataclass
class Ball(Point):
    id: int
    blocked: bool = False
    collected: bool = False


@dataclass
class Scenario:
    seed: int
    bounds: Bounds
    robot_start: Point
    obstacles: list[Obstacle]
    balls: list[Ball]


@dataclass
class Leg:
    phase: int
    ball_id: int
    distance_m: float
    travel_s: float
    mode: str
    path: list[Point]
    risk: str


@dataclass
class RunMetrics:
    seed: int
    balls_detected: int
    balls_collectable: int
    balls_blocked: int
    collected_rate: float
    total_distance_m: float
    total_time_s: float
    travel_time_s: float
    scan_time_s: float
    pickup_time_s: float
    estimated_avg_speed_m_s: float
    planned_replans: int
    scan_events: int
    risky_balls: int
    expected_misses: float
    net_wall_risks: int
    obstacle_risks: int
    avoid_legs: int


@dataclass
class TrainingRow:
    seed: int
    decision_id: int
    phase: int
    step_in_phase: int
    remaining_balls: int
    candidate_rank_by_distance: int
    selected: int
    robot_x_m: float
    robot_y_m: float
    ball_id: int
    ball_x_m: float
    ball_y_m: float
    direct_distance_m: float
    estimated_route_distance_m: float
    estimated_travel_s: float
    pickup_pose_distance_m: float
    approach_clear: int
    risk_type: str
    miss_probability: float
    distance_to_net_m: float
    distance_to_nearest_wall_m: float
    distance_to_nearest_obstacle_m: float
    phase_min_x_m: float
    phase_max_x_m: float
    phase_min_y_m: float
    phase_max_y_m: float


def half_bounds(side: str) -> Bounds:
    half = COURT_LENGTH_M / 2
    if side == "left":
        return Bounds(-half, -NET_CLEARANCE_X_M, -COURT_WIDTH_M / 2, COURT_WIDTH_M / 2)
    return Bounds(NET_CLEARANCE_X_M, half, -COURT_WIDTH_M / 2, COURT_WIDTH_M / 2)


def full_bounds() -> Bounds:
    half = COURT_LENGTH_M / 2
    return Bounds(-half, half, -COURT_WIDTH_M / 2, COURT_WIDTH_M / 2)


def phase_start(bounds: Bounds) -> Point:
    from_left = abs(bounds.min_x) >= abs(bounds.max_x)
    return Point(bounds.min_x + 1.15 if from_left else bounds.max_x - 1.15, 0.0)


def dist(a: Point, b: Point) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def in_bounds(point: Point, bounds: Bounds, margin: float = 0.0) -> bool:
    return (
        bounds.min_x + margin <= point.x <= bounds.max_x - margin
        and bounds.min_y + margin <= point.y <= bounds.max_y - margin
    )


def obstacle_clearance(point: Point, obstacle: Obstacle) -> float:
    if obstacle.kind == "circle":
        human_pad = 0.35 if obstacle.human else 0.0
        return math.hypot(point.x - obstacle.x, point.y - obstacle.y) - obstacle.radius - human_pad
    dx = max(abs(point.x - obstacle.x) - obstacle.width / 2, 0.0)
    dy = max(abs(point.y - obstacle.y) - obstacle.height / 2, 0.0)
    return math.hypot(dx, dy)


def collides(x: float, y: float, obstacle: Obstacle, pad: float) -> bool:
    return obstacle_clearance(Point(x, y), obstacle) <= pad


def ball_position_clear(point: Point, obstacles: list[Obstacle]) -> bool:
    return all(obstacle_clearance(point, obstacle) > 0.08 for obstacle in obstacles)


def is_free(
    x: float,
    y: float,
    obstacles: list[Obstacle],
    bounds: Bounds,
    safety_buffer_m: float,
    extra: float = 0.0,
) -> bool:
    pad = ROBOT_RADIUS_M + safety_buffer_m + extra
    if x < bounds.min_x + pad or x > bounds.max_x - pad:
        return False
    if y < bounds.min_y + pad or y > bounds.max_y - pad:
        return False
    return not any(collides(x, y, obstacle, pad) for obstacle in obstacles)


def segment_clear(
    start: Point,
    goal: Point,
    obstacles: list[Obstacle],
    bounds: Bounds,
    safety_buffer_m: float,
) -> bool:
    distance = dist(start, goal)
    steps = max(2, math.ceil(distance / 0.18))
    for index in range(steps + 1):
        t = index / steps
        x = start.x + (goal.x - start.x) * t
        y = start.y + (goal.y - start.y) * t
        if not is_free(x, y, obstacles, bounds, safety_buffer_m):
            return False
    return True


def world_to_grid(point: Point, bounds: Bounds) -> tuple[int, int]:
    cols = round((bounds.max_x - bounds.min_x) / GRID_M)
    rows = round((bounds.max_y - bounds.min_y) / GRID_M)
    return (
        max(0, min(cols, round((point.x - bounds.min_x) / GRID_M))),
        max(0, min(rows, round((point.y - bounds.min_y) / GRID_M))),
    )


def grid_to_world(grid: tuple[int, int], bounds: Bounds) -> Point:
    return Point(bounds.min_x + grid[0] * GRID_M, bounds.min_y + grid[1] * GRID_M)


def path_distance(path: list[Point]) -> float:
    return sum(dist(path[index], point) for index, point in enumerate(path[1:]))


def simplify_path(
    nodes: list[Point],
    obstacles: list[Obstacle],
    bounds: Bounds,
    safety_buffer_m: float,
) -> list[Point]:
    if len(nodes) <= 2:
        return nodes
    out = [nodes[0]]
    anchor = 0
    while anchor < len(nodes) - 1:
        next_index = len(nodes) - 1
        while next_index > anchor + 1:
            if segment_clear(nodes[anchor], nodes[next_index], obstacles, bounds, safety_buffer_m):
                break
            next_index -= 1
        out.append(nodes[next_index])
        anchor = next_index
    return out


def pathfind(
    start: Point,
    goal: Point,
    obstacles: list[Obstacle],
    bounds: Bounds,
    safety_buffer_m: float,
) -> tuple[float, list[Point], str]:
    if segment_clear(start, goal, obstacles, bounds, safety_buffer_m):
        return dist(start, goal), [start, goal], "direct"

    cols = round((bounds.max_x - bounds.min_x) / GRID_M) + 1
    rows = round((bounds.max_y - bounds.min_y) / GRID_M) + 1
    start_grid = world_to_grid(start, bounds)
    goal_grid = world_to_grid(goal, bounds)
    open_set: list[tuple[float, float, tuple[int, int]]] = [(0.0, 0.0, start_grid)]
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    costs = {start_grid: 0.0}
    dirs = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    found: tuple[int, int] | None = None

    while open_set:
        _, cost, current = heapq.heappop(open_set)
        if current == goal_grid:
            found = current
            break
        for dx, dy in dirs:
            nxt = (current[0] + dx, current[1] + dy)
            if nxt[0] < 0 or nxt[1] < 0 or nxt[0] >= cols or nxt[1] >= rows:
                continue
            point = grid_to_world(nxt, bounds)
            if not is_free(point.x, point.y, obstacles, bounds, safety_buffer_m):
                continue
            next_cost = cost + math.hypot(dx, dy) * GRID_M
            if nxt not in costs or next_cost < costs[nxt]:
                costs[nxt] = next_cost
                came_from[nxt] = current
                heuristic = math.hypot(nxt[0] - goal_grid[0], nxt[1] - goal_grid[1]) * GRID_M
                heapq.heappush(open_set, (next_cost + heuristic, next_cost, nxt))

    if found is None:
        return math.inf, [], "blocked"

    grids = [found]
    while grids[-1] in came_from:
        grids.append(came_from[grids[-1]])
    nodes = [grid_to_world(grid, bounds) for grid in reversed(grids)]
    nodes[0] = start
    nodes[-1] = goal
    simplified = simplify_path(nodes, obstacles, bounds, safety_buffer_m)
    return path_distance(simplified), simplified, "avoid"


def random_playable_area(rng: random.Random, area_mode: str, distribution: str, bounds: Bounds) -> Bounds:
    if area_mode == "two-phase" or (area_mode == "full" and distribution == "realistic"):
        return half_bounds("left" if rng.random() < 0.5 else "right")
    return bounds


def sample_ball_position(rng: random.Random, bounds: Bounds, area_mode: str, distribution: str) -> Point:
    area = random_playable_area(rng, area_mode, distribution, bounds)
    margin = 0.55
    if distribution == "uniform":
        return Point(
            rng.uniform(area.min_x + margin, area.max_x - margin),
            rng.uniform(area.min_y + margin, area.max_y - margin),
        )

    zone = rng.random()
    left_side = abs(area.min_x) > abs(area.max_x)
    net_x = area.max_x if left_side else area.min_x
    back_x = area.min_x if left_side else area.max_x
    if zone < 0.46:
        x = net_x + (-1 if left_side else 1) * abs(rng.gauss(0, 1.05))
        y = rng.gauss(0, COURT_WIDTH_M * 0.26)
    elif zone < 0.82:
        x = back_x + (1 if left_side else -1) * abs(rng.gauss(0, 1.20))
        y = rng.gauss(0, COURT_WIDTH_M * 0.30)
    elif zone < 0.94:
        x = (-COURT_SERVICE_M if left_side else COURT_SERVICE_M) + rng.gauss(0, 1.65)
        y = rng.gauss(0, COURT_WIDTH_M * 0.24)
    else:
        x = rng.uniform(area.min_x + margin, area.max_x - margin)
        y = rng.uniform(area.min_y + margin, area.max_y - margin)
    return Point(clamp(x, area.min_x + margin, area.max_x - margin), clamp(y, area.min_y + margin, area.max_y - margin))


def make_scenario(
    seed: int,
    ball_count: int,
    area_mode: str,
    distribution: str,
    people_count: int,
    fixed_count: int,
    safety_buffer_m: float,
) -> Scenario:
    rng = random.Random(seed)
    bounds = half_bounds("left") if area_mode == "half" else full_bounds()
    robot_start = Point(bounds.min_x + 1.15, 0.0)
    obstacles = [Obstacle("rect", "net", 0.0, 0.0, width=0.18, height=COURT_WIDTH_M + 0.8)]

    for index in range(people_count):
        angle = rng.random() * math.pi * 2
        speed = 0.10 + rng.random() * 0.22
        _ = speed
        for _attempt in range(60):
            candidate = Obstacle(
                "circle",
                f"person-{index + 1}",
                rng.uniform(bounds.min_x + 1.5, bounds.max_x - 1.5),
                rng.uniform(bounds.min_y + 1.0, bounds.max_y - 1.0),
                radius=0.42,
                human=True,
            )
            if dist(Point(candidate.x, candidate.y), robot_start) > 1.8 and is_free(
                candidate.x, candidate.y, obstacles, bounds, safety_buffer_m, 0.4
            ):
                obstacles.append(candidate)
                break

    for index in range(fixed_count):
        for _attempt in range(60):
            candidate = Obstacle(
                "circle",
                f"bag-{index + 1}",
                rng.uniform(bounds.min_x + 1.2, bounds.max_x - 1.2),
                rng.uniform(bounds.min_y + 0.8, bounds.max_y - 0.8),
                radius=0.32 + rng.random() * 0.25,
            )
            if dist(Point(candidate.x, candidate.y), robot_start) > 1.5 and is_free(
                candidate.x, candidate.y, obstacles, bounds, safety_buffer_m, 0.25
            ):
                obstacles.append(candidate)
                break

    balls: list[Ball] = []
    attempts = 0
    while len(balls) < ball_count and attempts < ball_count * 90:
        attempts += 1
        point = sample_ball_position(rng, bounds, area_mode, distribution)
        if ball_position_clear(point, obstacles):
            balls.append(Ball(x=point.x, y=point.y, id=len(balls) + 1))

    return Scenario(seed, bounds, robot_start, obstacles, balls)


def planning_phases(area_mode: str, scenario: Scenario) -> list[tuple[int, Bounds, Point]]:
    if area_mode != "two-phase":
        return [(1, scenario.bounds, scenario.robot_start)]
    left = half_bounds("left")
    right = half_bounds("right")
    return [(1, left, phase_start(left)), (2, right, phase_start(right))]


def ball_risk(ball: Ball, obstacles: list[Obstacle], phase_bounds: Bounds, collection_margin_m: float) -> str:
    near_net = abs(ball.x) <= NET_CLEARANCE_X_M + collection_margin_m
    near_wall = (
        abs(ball.x - phase_bounds.min_x) <= collection_margin_m
        or abs(ball.x - phase_bounds.max_x) <= collection_margin_m
        or abs(ball.y - phase_bounds.min_y) <= collection_margin_m
        or abs(ball.y - phase_bounds.max_y) <= collection_margin_m
    )
    near_obstacle = any(
        obstacle.label != "net" and obstacle_clearance(ball, obstacle) <= ROBOT_RADIUS_M + collection_margin_m
        for obstacle in obstacles
    )
    if near_obstacle:
        return "obstacle"
    if near_net or near_wall:
        return "net_wall"
    return "normal"


def nearest_wall_distance(point: Point, bounds: Bounds) -> float:
    return min(
        abs(point.x - bounds.min_x),
        abs(point.x - bounds.max_x),
        abs(point.y - bounds.min_y),
        abs(point.y - bounds.max_y),
    )


def nearest_obstacle_distance(point: Point, obstacles: list[Obstacle]) -> float:
    distances = [obstacle_clearance(point, obstacle) for obstacle in obstacles if obstacle.label != "net"]
    return min(distances) if distances else 999.0


def lidar_clearance_penalty(
    point: Point,
    obstacles: list[Obstacle],
    phase_bounds: Bounds,
    enabled: bool,
) -> float:
    if not enabled:
        return 0.0

    wall_clearance = nearest_wall_distance(point, phase_bounds)
    obstacle_clear = nearest_obstacle_distance(point, obstacles)
    net_clearance = abs(point.x)
    penalty = 0.0
    for clearance, preferred, weight in (
        (wall_clearance, 0.95, 1.2),
        (obstacle_clear, 1.15, 1.5),
        (net_clearance, 0.85, 1.0),
    ):
        if clearance < preferred:
            penalty += (preferred - clearance) * weight
    return penalty


def pickup_targets(
    ball: Ball,
    obstacles: list[Obstacle],
    phase_bounds: Bounds,
    safety_buffer_m: float,
) -> list[Point]:
    """Return reachable robot-center poses that could present the front intake to a ball."""

    targets: list[Point] = []
    if is_free(ball.x, ball.y, obstacles, phase_bounds, safety_buffer_m, 0.08):
        targets.append(Point(ball.x, ball.y))

    for radius in (0.40, 0.65):
        for index in range(8):
            angle = (math.pi * 2 * index) / 8
            target = Point(ball.x + math.cos(angle) * radius, ball.y + math.sin(angle) * radius)
            if is_free(target.x, target.y, obstacles, phase_bounds, safety_buffer_m, 0.0):
                targets.append(target)

    # Prefer close pickup poses; path planning will still choose by route cost.
    return sorted(targets, key=lambda target: dist(target, ball))


def candidate_features(
    current: Point,
    ball: Ball,
    obstacles: list[Obstacle],
    phase_bounds: Bounds,
    safety_buffer_m: float,
    collection_margin_m: float,
    travel_speed_m_s: float,
    lidar_costmap: bool = False,
) -> dict[str, float | int | str] | None:
    best: tuple[float, Point, bool] | None = None
    for target in pickup_targets(ball, obstacles, phase_bounds, safety_buffer_m):
        direct_distance = dist(current, target)
        pickup_distance = dist(target, ball)
        clear = segment_clear(current, target, obstacles, phase_bounds, safety_buffer_m)
        estimated_distance = direct_distance + pickup_distance * 0.35
        if not clear:
            estimated_distance = estimated_distance * 1.35 + 2.0
        estimated_distance += lidar_clearance_penalty(target, obstacles, phase_bounds, lidar_costmap)
        if best is None or estimated_distance < best[0]:
            best = (estimated_distance, target, clear)
    if best is None:
        return None

    estimated_distance, target, clear = best
    risk = ball_risk(ball, obstacles, phase_bounds, collection_margin_m)
    return {
        "direct_distance_m": dist(current, ball),
        "estimated_route_distance_m": estimated_distance,
        "estimated_travel_s": estimated_distance / travel_speed_m_s,
        "pickup_pose_distance_m": dist(target, ball),
        "approach_clear": int(clear),
        "risk_type": risk,
        "miss_probability": miss_probability(risk, "direct" if clear else "avoid"),
        "distance_to_net_m": abs(ball.x),
        "distance_to_nearest_wall_m": nearest_wall_distance(ball, phase_bounds),
        "distance_to_nearest_obstacle_m": nearest_obstacle_distance(ball, obstacles),
        "target": target,
    }


def miss_probability(risk: str, mode: str) -> float:
    base = {"normal": 0.03, "net_wall": 0.18, "obstacle": 0.32}[risk]
    if mode == "avoid":
        base += 0.04
    return min(base, 0.60)


def plan_route(
    scenario: Scenario,
    area_mode: str,
    travel_speed_m_s: float,
    pickup_time_s: float,
    scan_time_s: float,
    rescan_every: int,
    safety_buffer_m: float,
    collection_margin_m: float,
    candidate_window: int,
    training_rows: list[TrainingRow] | None = None,
    lidar_costmap: bool = False,
) -> tuple[list[Leg], RunMetrics]:
    legs: list[Leg] = []
    planned_balls: set[int] = set()
    planned_replans = 0
    scan_events = 0
    decision_id = 0

    for phase_index, phase_bounds, phase_start_point in planning_phases(area_mode, scenario):
        scan_events += 1
        planned_replans += 1
        phase_balls = [ball for ball in scenario.balls if in_bounds(ball, phase_bounds, 0.08)]
        current = phase_start_point
        remaining = phase_balls[:]
        step_in_phase = 0
        while remaining:
            best_index = -1
            best: tuple[Point, Ball] | None = None
            best_score = math.inf
            shortlist = sorted(enumerate(remaining), key=lambda item: dist(current, item[1]))[:candidate_window]
            candidate_rows: list[tuple[Ball, dict[str, float | int | str], int]] = []
            for index, candidate in shortlist:
                features = candidate_features(
                    current,
                    candidate,
                    scenario.obstacles,
                    phase_bounds,
                    safety_buffer_m,
                    collection_margin_m,
                    travel_speed_m_s,
                    lidar_costmap,
                )
                if features is None:
                    continue
                candidate_rows.append((candidate, features, index))
                weighted_distance = float(features["estimated_route_distance_m"])
                if weighted_distance < best_score:
                    best = (features["target"], candidate)  # type: ignore[arg-type]
                    best_score = weighted_distance
                    best_index = index
            if best is None:
                break
            target, ball = best
            distance, path, mode = pathfind(current, target, scenario.obstacles, phase_bounds, safety_buffer_m)
            if distance == math.inf:
                remaining.pop(best_index)
                continue
            if training_rows is not None:
                decision_id += 1
                step_in_phase += 1
                ranked_rows = sorted(candidate_rows, key=lambda item: float(item[1]["estimated_route_distance_m"]))
                for rank, (candidate, features, _index) in enumerate(ranked_rows, start=1):
                    training_rows.append(
                        TrainingRow(
                            seed=scenario.seed,
                            decision_id=decision_id,
                            phase=phase_index,
                            step_in_phase=step_in_phase,
                            remaining_balls=len(remaining),
                            candidate_rank_by_distance=rank,
                            selected=int(candidate.id == ball.id),
                            robot_x_m=current.x,
                            robot_y_m=current.y,
                            ball_id=candidate.id,
                            ball_x_m=candidate.x,
                            ball_y_m=candidate.y,
                            direct_distance_m=float(features["direct_distance_m"]),
                            estimated_route_distance_m=float(features["estimated_route_distance_m"]),
                            estimated_travel_s=float(features["estimated_travel_s"]),
                            pickup_pose_distance_m=float(features["pickup_pose_distance_m"]),
                            approach_clear=int(features["approach_clear"]),
                            risk_type=str(features["risk_type"]),
                            miss_probability=float(features["miss_probability"]),
                            distance_to_net_m=float(features["distance_to_net_m"]),
                            distance_to_nearest_wall_m=float(features["distance_to_nearest_wall_m"]),
                            distance_to_nearest_obstacle_m=float(features["distance_to_nearest_obstacle_m"]),
                            phase_min_x_m=phase_bounds.min_x,
                            phase_max_x_m=phase_bounds.max_x,
                            phase_min_y_m=phase_bounds.min_y,
                            phase_max_y_m=phase_bounds.max_y,
                        )
                    )
            risk = ball_risk(ball, scenario.obstacles, phase_bounds, collection_margin_m)
            legs.append(Leg(phase_index, ball.id, distance, distance / travel_speed_m_s, mode, path, risk))
            planned_balls.add(ball.id)
            current = path[-1]
            remaining.pop(best_index)
            if rescan_every > 0 and len(remaining) > 0 and len([leg for leg in legs if leg.phase == phase_index]) % rescan_every == 0:
                scan_events += 1
                planned_replans += 1

    for ball in scenario.balls:
        ball.blocked = ball.id not in planned_balls

    travel_time = sum(leg.travel_s for leg in legs)
    pickup_time = len(legs) * pickup_time_s
    scan_time = scan_events * scan_time_s
    expected_misses = sum(miss_probability(leg.risk, leg.mode) for leg in legs)
    total_distance = sum(leg.distance_m for leg in legs)
    total_time = travel_time + pickup_time + scan_time
    avoid_legs = sum(1 for leg in legs if leg.mode == "avoid")
    metrics = RunMetrics(
        seed=scenario.seed,
        balls_detected=len(scenario.balls),
        balls_collectable=len(legs),
        balls_blocked=sum(1 for ball in scenario.balls if ball.blocked),
        collected_rate=len(legs) / max(1, len(scenario.balls)),
        total_distance_m=total_distance,
        total_time_s=total_time,
        travel_time_s=travel_time,
        scan_time_s=scan_time,
        pickup_time_s=pickup_time,
        estimated_avg_speed_m_s=total_distance / max(1.0, travel_time),
        planned_replans=planned_replans + avoid_legs,
        scan_events=scan_events,
        risky_balls=sum(1 for leg in legs if leg.risk != "normal"),
        expected_misses=expected_misses,
        net_wall_risks=sum(1 for leg in legs if leg.risk == "net_wall"),
        obstacle_risks=sum(1 for leg in legs if leg.risk == "obstacle"),
        avoid_legs=avoid_legs,
    )
    return legs, metrics


def percentile(values: list[float], pct: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, round((pct / 100) * (len(ordered) - 1))))
    return ordered[index]


def summarize(runs: list[RunMetrics]) -> dict[str, float | int]:
    return {
        "runs": len(runs),
        "avg_balls_detected": mean(run.balls_detected for run in runs),
        "avg_balls_collectable": mean(run.balls_collectable for run in runs),
        "avg_collected_rate": mean(run.collected_rate for run in runs),
        "avg_total_time_s": mean(run.total_time_s for run in runs),
        "p50_total_time_s": percentile([run.total_time_s for run in runs], 50),
        "p90_total_time_s": percentile([run.total_time_s for run in runs], 90),
        "avg_total_distance_m": mean(run.total_distance_m for run in runs),
        "avg_estimated_speed_m_s": mean(run.estimated_avg_speed_m_s for run in runs),
        "avg_expected_misses": mean(run.expected_misses for run in runs),
        "avg_risky_balls": mean(run.risky_balls for run in runs),
        "avg_net_wall_risks": mean(run.net_wall_risks for run in runs),
        "avg_obstacle_risks": mean(run.obstacle_risks for run in runs),
        "avg_blocked_balls": mean(run.balls_blocked for run in runs),
        "avg_replans": mean(run.planned_replans for run in runs),
        "avg_scan_events": mean(run.scan_events for run in runs),
    }


def write_csv(path: Path, runs: list[RunMetrics]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(asdict(runs[0]).keys()))
        writer.writeheader()
        for run in runs:
            writer.writerow(asdict(run))


def write_training_csv(path: Path, rows: list[TrainingRow]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(TrainingRow.__dataclass_fields__.keys())
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(asdict(row))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Benchmark tennis robot route planning over random scenarios.")
    parser.add_argument("--runs", type=int, default=100)
    parser.add_argument("--balls", type=int, default=40)
    parser.add_argument("--seed", type=int, default=37)
    parser.add_argument("--area-mode", choices=("half", "two-phase", "full"), default="two-phase")
    parser.add_argument("--distribution", choices=("realistic", "uniform"), default="realistic")
    parser.add_argument("--people", type=int, default=3)
    parser.add_argument("--fixed-obstacles", type=int, default=3)
    parser.add_argument("--travel-speed", type=float, default=0.85)
    parser.add_argument("--pickup-time", type=float, default=1.2)
    parser.add_argument("--scan-time", type=float, default=7.0)
    parser.add_argument("--rescan-every", type=int, default=5)
    parser.add_argument("--safety-buffer", type=float, default=0.55)
    parser.add_argument("--collection-margin", type=float, default=0.55)
    parser.add_argument("--candidate-window", type=int, default=12)
    parser.add_argument("--lidar-costmap", action="store_true")
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--csv-out", type=Path)
    parser.add_argument("--training-out", type=Path)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    runs: list[RunMetrics] = []
    training_rows: list[TrainingRow] = []
    for index in range(args.runs):
        seed = args.seed + index
        scenario = make_scenario(
            seed,
            args.balls,
            args.area_mode,
            args.distribution,
            args.people,
            args.fixed_obstacles,
            args.safety_buffer,
        )
        _legs, metrics = plan_route(
            scenario,
            args.area_mode,
            args.travel_speed,
            args.pickup_time,
            args.scan_time,
            args.rescan_every,
            args.safety_buffer,
            args.collection_margin,
            args.candidate_window,
            training_rows if args.training_out else None,
            args.lidar_costmap,
        )
        runs.append(metrics)

    summary = summarize(runs)
    payload = {
        "config": {
            "runs": args.runs,
            "balls": args.balls,
            "seed_start": args.seed,
            "area_mode": args.area_mode,
            "distribution": args.distribution,
            "people": args.people,
            "fixed_obstacles": args.fixed_obstacles,
            "travel_speed_m_s": args.travel_speed,
            "pickup_time_s": args.pickup_time,
            "scan_time_s": args.scan_time,
            "rescan_every": args.rescan_every,
            "safety_buffer_m": args.safety_buffer,
            "collection_margin_m": args.collection_margin,
            "candidate_window": args.candidate_window,
            "lidar_costmap": args.lidar_costmap,
        },
        "summary": summary,
        "runs": [asdict(run) for run in runs],
    }
    if args.training_out:
        payload["training_rows"] = len(training_rows)

    print(json.dumps({"config": payload["config"], "summary": summary}, indent=2))
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    if args.csv_out:
        write_csv(args.csv_out, runs)
    if args.training_out:
        write_training_csv(args.training_out, training_rows)


if __name__ == "__main__":
    main()
