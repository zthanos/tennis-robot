#!/usr/bin/env python3
"""Generate outcome-labeled next-ball training data.

Unlike route_benchmark.py --training-out, this labels the candidate with the
lowest estimated future outcome cost instead of the candidate selected by the
current greedy planner.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from route_benchmark import (
    TrainingRow,
    candidate_features,
    in_bounds,
    make_scenario,
    pathfind,
    planning_phases,
    write_training_csv,
)


def local_candidate_cost(features: dict[str, float | int | str], pickup_time_s: float, miss_penalty_s: float) -> float:
    return float(features["estimated_travel_s"]) + pickup_time_s + float(features["miss_probability"]) * miss_penalty_s


def greedy_rollout_cost(
    start,
    remaining,
    obstacles,
    phase_bounds,
    safety_buffer_m: float,
    collection_margin_m: float,
    travel_speed_m_s: float,
    pickup_time_s: float,
    miss_penalty_s: float,
    candidate_window: int,
    max_depth: int,
    lidar_costmap: bool,
) -> float:
    current = start
    balls = list(remaining)
    total = 0.0
    for _step in range(max_depth):
        if not balls:
            break
        best_index = -1
        best_features = None
        best_cost = math.inf
        shortlist = sorted(enumerate(balls), key=lambda item: math.hypot(item[1].x - current.x, item[1].y - current.y))[
            :candidate_window
        ]
        for index, ball in shortlist:
            features = candidate_features(
                current,
                ball,
                obstacles,
                phase_bounds,
                safety_buffer_m,
                collection_margin_m,
                travel_speed_m_s,
                lidar_costmap,
            )
            if features is None:
                continue
            cost = local_candidate_cost(features, pickup_time_s, miss_penalty_s)
            if cost < best_cost:
                best_index = index
                best_features = features
                best_cost = cost
        if best_features is None:
            break
        total += best_cost
        current = best_features["target"]
        balls.pop(best_index)
    return total


def append_training_row(
    rows: list[TrainingRow],
    seed: int,
    decision_id: int,
    phase_index: int,
    step_in_phase: int,
    remaining_count: int,
    rank: int,
    selected: bool,
    current,
    ball,
    features: dict[str, float | int | str],
    phase_bounds,
) -> None:
    rows.append(
        TrainingRow(
            seed=seed,
            decision_id=decision_id,
            phase=phase_index,
            step_in_phase=step_in_phase,
            remaining_balls=remaining_count,
            candidate_rank_by_distance=rank,
            selected=int(selected),
            robot_x_m=current.x,
            robot_y_m=current.y,
            ball_id=ball.id,
            ball_x_m=ball.x,
            ball_y_m=ball.y,
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


def generate_rows_for_scenario(
    scenario,
    area_mode: str,
    travel_speed_m_s: float,
    pickup_time_s: float,
    safety_buffer_m: float,
    collection_margin_m: float,
    candidate_window: int,
    rollout_depth: int,
    miss_penalty_s: float,
    start_decision_id: int,
    lidar_costmap: bool,
) -> tuple[list[TrainingRow], int, int]:
    rows: list[TrainingRow] = []
    decision_id = start_decision_id
    decisions = 0

    for phase_index, phase_bounds, phase_start_point in planning_phases(area_mode, scenario):
        current = phase_start_point
        remaining = [ball for ball in scenario.balls if in_bounds(ball, phase_bounds, 0.08)]
        step_in_phase = 0
        while remaining:
            step_in_phase += 1
            candidate_rows = []
            shortlist = sorted(enumerate(remaining), key=lambda item: math.hypot(item[1].x - current.x, item[1].y - current.y))[
                :candidate_window
            ]
            for index, ball in shortlist:
                features = candidate_features(
                    current,
                    ball,
                    scenario.obstacles,
                    phase_bounds,
                    safety_buffer_m,
                    collection_margin_m,
                    travel_speed_m_s,
                    lidar_costmap,
                )
                if features is None:
                    continue
                future_remaining = [candidate for candidate in remaining if candidate.id != ball.id]
                immediate = local_candidate_cost(features, pickup_time_s, miss_penalty_s)
                rollout = greedy_rollout_cost(
                    features["target"],
                    future_remaining,
                    scenario.obstacles,
                    phase_bounds,
                    safety_buffer_m,
                    collection_margin_m,
                    travel_speed_m_s,
                    pickup_time_s,
                    miss_penalty_s,
                    candidate_window,
                    rollout_depth,
                    lidar_costmap,
                )
                candidate_rows.append((immediate + rollout, index, ball, features))
            if not candidate_rows:
                break

            decision_id += 1
            decisions += 1
            best = min(candidate_rows, key=lambda item: item[0])
            ranked = sorted(candidate_rows, key=lambda item: float(item[3]["estimated_route_distance_m"]))
            for rank, (_outcome_cost, _index, ball, features) in enumerate(ranked, start=1):
                append_training_row(
                    rows,
                    scenario.seed,
                    decision_id,
                    phase_index,
                    step_in_phase,
                    len(remaining),
                    rank,
                    selected=ball.id == best[2].id,
                    current=current,
                    ball=ball,
                    features=features,
                    phase_bounds=phase_bounds,
                )

            _cost, best_index, _ball, best_features = best
            distance, path, _mode = pathfind(
                current,
                best_features["target"],
                scenario.obstacles,
                phase_bounds,
                safety_buffer_m,
            )
            if distance == math.inf:
                remaining.pop(best_index)
                continue
            current = path[-1]
            remaining.pop(best_index)

    return rows, decision_id, decisions


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate outcome-labeled next-ball policy training data.")
    parser.add_argument("--runs", type=int, default=300)
    parser.add_argument("--balls", type=int, default=40)
    parser.add_argument("--seed", type=int, default=37)
    parser.add_argument("--area-mode", choices=("half", "two-phase", "full"), default="two-phase")
    parser.add_argument("--distribution", choices=("realistic", "uniform"), default="realistic")
    parser.add_argument("--people", type=int, default=3)
    parser.add_argument("--fixed-obstacles", type=int, default=3)
    parser.add_argument("--travel-speed", type=float, default=0.85)
    parser.add_argument("--pickup-time", type=float, default=1.2)
    parser.add_argument("--safety-buffer", type=float, default=0.55)
    parser.add_argument("--collection-margin", type=float, default=0.55)
    parser.add_argument("--candidate-window", type=int, default=12)
    parser.add_argument("--rollout-depth", type=int, default=8)
    parser.add_argument("--miss-penalty", type=float, default=12.0)
    parser.add_argument("--lidar-costmap", action="store_true")
    parser.add_argument("--training-out", type=Path, default=Path("runtime/route-outcome-training.csv"))
    parser.add_argument("--summary-out", type=Path, default=Path("runtime/route-outcome-training-summary.json"))
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rows: list[TrainingRow] = []
    decision_id = 0
    decisions = 0
    for index in range(args.runs):
        scenario = make_scenario(
            args.seed + index,
            args.balls,
            args.area_mode,
            args.distribution,
            args.people,
            args.fixed_obstacles,
            args.safety_buffer,
        )
        scenario_rows, decision_id, scenario_decisions = generate_rows_for_scenario(
            scenario,
            args.area_mode,
            args.travel_speed,
            args.pickup_time,
            args.safety_buffer,
            args.collection_margin,
            args.candidate_window,
            args.rollout_depth,
            args.miss_penalty,
            decision_id,
            args.lidar_costmap,
        )
        rows.extend(scenario_rows)
        decisions += scenario_decisions

    write_training_csv(args.training_out, rows)
    summary = {
        "runs": args.runs,
        "balls": args.balls,
        "rows": len(rows),
        "decisions": decisions,
        "seed_start": args.seed,
        "candidate_window": args.candidate_window,
        "rollout_depth": args.rollout_depth,
        "miss_penalty_s": args.miss_penalty,
        "lidar_costmap": args.lidar_costmap,
        "training_out": str(args.training_out),
    }
    args.summary_out.parent.mkdir(parents=True, exist_ok=True)
    args.summary_out.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
