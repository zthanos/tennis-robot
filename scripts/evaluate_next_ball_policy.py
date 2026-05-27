#!/usr/bin/env python3
"""Compare the trained next-ball policy against the benchmark planner."""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

from route_benchmark import (
    Ball,
    Leg,
    Point,
    RunMetrics,
    ball_risk,
    candidate_features,
    half_bounds,
    in_bounds,
    make_scenario,
    miss_probability,
    pathfind,
    plan_route,
    planning_phases,
)


@dataclass
class ComparisonRow:
    seed: int
    planner_collectable: int
    model_collectable: int
    planner_time_s: float
    model_time_s: float
    delta_time_s: float
    planner_distance_m: float
    model_distance_m: float
    delta_distance_m: float
    planner_expected_misses: float
    model_expected_misses: float
    delta_expected_misses: float
    planner_risky_balls: int
    model_risky_balls: int
    planner_blocked_balls: int
    model_blocked_balls: int
    model_choice_count: int


def sigmoid(value: float) -> float:
    value = max(-40.0, min(40.0, value))
    return 1.0 / (1.0 + math.exp(-value))


def load_model(path: Path) -> dict[str, object]:
    return json.loads(path.read_text(encoding="utf-8"))


def model_score(model: dict[str, object], row: dict[str, float | int | str]) -> float:
    feature_names = model["feature_names"]
    mean = np.array(model["mean"], dtype=np.float64)
    std = np.array(model["std"], dtype=np.float64)
    weights = np.array(model["weights"], dtype=np.float64)
    values = []
    for feature in feature_names:  # type: ignore[assignment]
        name = str(feature)
        if name.startswith("risk_type_"):
            values.append(1.0 if row["risk_type"] == name.removeprefix("risk_type_") else 0.0)
        else:
            values.append(float(row[name]))
    x = (np.array(values, dtype=np.float64) - mean) / std
    return sigmoid(float(x @ weights + float(model["bias"])))


def run_metrics_from_legs(
    seed: int,
    balls: list[Ball],
    legs: list[Leg],
    travel_speed_m_s: float,
    pickup_time_s: float,
    scan_time_s: float,
    scan_events: int,
    planned_replans: int,
) -> RunMetrics:
    travel_time = sum(leg.travel_s for leg in legs)
    pickup_time = len(legs) * pickup_time_s
    scan_time = scan_events * scan_time_s
    total_distance = sum(leg.distance_m for leg in legs)
    avoid_legs = sum(1 for leg in legs if leg.mode == "avoid")
    expected_misses = sum(miss_probability(leg.risk, leg.mode) for leg in legs)
    return RunMetrics(
        seed=seed,
        balls_detected=len(balls),
        balls_collectable=len(legs),
        balls_blocked=sum(1 for ball in balls if ball.blocked),
        collected_rate=len(legs) / max(1, len(balls)),
        total_distance_m=total_distance,
        total_time_s=travel_time + pickup_time + scan_time,
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


def plan_model_policy(
    model: dict[str, object],
    scenario,
    area_mode: str,
    travel_speed_m_s: float,
    pickup_time_s: float,
    scan_time_s: float,
    rescan_every: int,
    safety_buffer_m: float,
    collection_margin_m: float,
    candidate_window: int,
    lidar_costmap: bool,
) -> tuple[list[Leg], RunMetrics, int]:
    legs: list[Leg] = []
    planned_balls: set[int] = set()
    scan_events = 0
    planned_replans = 0
    model_choice_count = 0

    for phase_index, phase_bounds, phase_start_point in planning_phases(area_mode, scenario):
        scan_events += 1
        planned_replans += 1
        current = phase_start_point
        remaining = [ball for ball in scenario.balls if in_bounds(ball, phase_bounds, 0.08)]
        step_in_phase = 0

        while remaining:
            step_in_phase += 1
            shortlist = sorted(enumerate(remaining), key=lambda item: math.hypot(item[1].x - current.x, item[1].y - current.y))[
                :candidate_window
            ]
            scored: list[tuple[float, int, Ball, Point, dict[str, float | int | str]]] = []
            for rank, (index, candidate) in enumerate(shortlist, start=1):
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
                row = {
                    **features,
                    "phase": phase_index,
                    "step_in_phase": step_in_phase,
                    "remaining_balls": len(remaining),
                    "candidate_rank_by_distance": rank,
                    "robot_x_m": current.x,
                    "robot_y_m": current.y,
                    "ball_x_m": candidate.x,
                    "ball_y_m": candidate.y,
                    "phase_min_x_m": phase_bounds.min_x,
                    "phase_max_x_m": phase_bounds.max_x,
                    "phase_min_y_m": phase_bounds.min_y,
                    "phase_max_y_m": phase_bounds.max_y,
                }
                scored.append((model_score(model, row), index, candidate, features["target"], features))  # type: ignore[arg-type]

            if not scored:
                break
            _score, best_index, ball, target, _features = max(scored, key=lambda item: item[0])
            distance, path, mode = pathfind(current, target, scenario.obstacles, phase_bounds, safety_buffer_m)
            if distance == math.inf:
                remaining.pop(best_index)
                continue

            risk = ball_risk(ball, scenario.obstacles, phase_bounds, collection_margin_m)
            legs.append(Leg(phase_index, ball.id, distance, distance / travel_speed_m_s, mode, path, risk))
            planned_balls.add(ball.id)
            model_choice_count += 1
            current = path[-1]
            remaining.pop(best_index)
            phase_leg_count = sum(1 for leg in legs if leg.phase == phase_index)
            if rescan_every > 0 and remaining and phase_leg_count % rescan_every == 0:
                scan_events += 1
                planned_replans += 1

    for ball in scenario.balls:
        ball.blocked = ball.id not in planned_balls

    metrics = run_metrics_from_legs(
        scenario.seed,
        scenario.balls,
        legs,
        travel_speed_m_s,
        pickup_time_s,
        scan_time_s,
        scan_events,
        planned_replans,
    )
    return legs, metrics, model_choice_count


def summarize(rows: list[ComparisonRow]) -> dict[str, float | int]:
    def avg(name: str) -> float:
        return sum(float(getattr(row, name)) for row in rows) / max(1, len(rows))

    model_wins_time = sum(1 for row in rows if row.delta_time_s < 0)
    model_wins_distance = sum(1 for row in rows if row.delta_distance_m < 0)
    return {
        "runs": len(rows),
        "avg_planner_time_s": avg("planner_time_s"),
        "avg_model_time_s": avg("model_time_s"),
        "avg_delta_time_s": avg("delta_time_s"),
        "avg_planner_distance_m": avg("planner_distance_m"),
        "avg_model_distance_m": avg("model_distance_m"),
        "avg_delta_distance_m": avg("delta_distance_m"),
        "avg_planner_expected_misses": avg("planner_expected_misses"),
        "avg_model_expected_misses": avg("model_expected_misses"),
        "avg_delta_expected_misses": avg("delta_expected_misses"),
        "avg_planner_collectable": avg("planner_collectable"),
        "avg_model_collectable": avg("model_collectable"),
        "model_time_win_rate": model_wins_time / max(1, len(rows)),
        "model_distance_win_rate": model_wins_distance / max(1, len(rows)),
    }


def write_comparison_csv(path: Path, rows: list[ComparisonRow]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(ComparisonRow.__dataclass_fields__.keys()))
        writer.writeheader()
        for row in rows:
            writer.writerow(asdict(row))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Evaluate trained next-ball policy against the benchmark planner.")
    parser.add_argument("--model", type=Path, default=Path("runtime/next-ball-policy-model.json"))
    parser.add_argument("--runs", type=int, default=100)
    parser.add_argument("--balls", type=int, default=40)
    parser.add_argument("--seed", type=int, default=10000)
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
    parser.add_argument("--json-out", type=Path, default=Path("runtime/next-ball-policy-eval.json"))
    parser.add_argument("--csv-out", type=Path, default=Path("runtime/next-ball-policy-eval.csv"))
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    model = load_model(args.model)
    rows: list[ComparisonRow] = []

    for index in range(args.runs):
        seed = args.seed + index
        planner_scenario = make_scenario(
            seed,
            args.balls,
            args.area_mode,
            args.distribution,
            args.people,
            args.fixed_obstacles,
            args.safety_buffer,
        )
        model_scenario = make_scenario(
            seed,
            args.balls,
            args.area_mode,
            args.distribution,
            args.people,
            args.fixed_obstacles,
            args.safety_buffer,
        )
        _planner_legs, planner_metrics = plan_route(
            planner_scenario,
            args.area_mode,
            args.travel_speed,
            args.pickup_time,
            args.scan_time,
            args.rescan_every,
            args.safety_buffer,
            args.collection_margin,
            args.candidate_window,
            args.lidar_costmap,
        )
        _model_legs, model_metrics, model_choice_count = plan_model_policy(
            model,
            model_scenario,
            args.area_mode,
            args.travel_speed,
            args.pickup_time,
            args.scan_time,
            args.rescan_every,
            args.safety_buffer,
            args.collection_margin,
            args.candidate_window,
            args.lidar_costmap,
        )
        rows.append(
            ComparisonRow(
                seed=seed,
                planner_collectable=planner_metrics.balls_collectable,
                model_collectable=model_metrics.balls_collectable,
                planner_time_s=planner_metrics.total_time_s,
                model_time_s=model_metrics.total_time_s,
                delta_time_s=model_metrics.total_time_s - planner_metrics.total_time_s,
                planner_distance_m=planner_metrics.total_distance_m,
                model_distance_m=model_metrics.total_distance_m,
                delta_distance_m=model_metrics.total_distance_m - planner_metrics.total_distance_m,
                planner_expected_misses=planner_metrics.expected_misses,
                model_expected_misses=model_metrics.expected_misses,
                delta_expected_misses=model_metrics.expected_misses - planner_metrics.expected_misses,
                planner_risky_balls=planner_metrics.risky_balls,
                model_risky_balls=model_metrics.risky_balls,
                planner_blocked_balls=planner_metrics.balls_blocked,
                model_blocked_balls=model_metrics.balls_blocked,
                model_choice_count=model_choice_count,
            )
        )

    summary = summarize(rows)
    payload = {
        "config": vars(args) | {"model": str(args.model)},
        "summary": summary,
        "runs": [asdict(row) for row in rows],
    }
    print(json.dumps({"config": payload["config"], "summary": summary}, indent=2, default=str))
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(payload, indent=2, default=str), encoding="utf-8")
    if args.csv_out:
        write_comparison_csv(args.csv_out, rows)


if __name__ == "__main__":
    main()
