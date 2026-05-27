#!/usr/bin/env python3
"""Evaluate a defer/edge-pass collection policy against the baseline planner."""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path

from evaluate_next_ball_policy import run_metrics_from_legs
from route_benchmark import (
    Ball,
    Leg,
    RunMetrics,
    ball_risk,
    candidate_features,
    in_bounds,
    make_scenario,
    pathfind,
    plan_route,
    planning_phases,
)


@dataclass
class DeferComparisonRow:
    seed: int
    planner_collectable: int
    defer_collectable: int
    deferred_balls: int
    skipped_balls: int
    planner_time_s: float
    defer_time_s: float
    delta_time_s: float
    planner_distance_m: float
    defer_distance_m: float
    delta_distance_m: float
    planner_expected_misses: float
    defer_expected_misses: float
    delta_expected_misses: float
    planner_risky_balls: int
    defer_risky_balls: int


def adjusted_risk(risk: str, edge_pass: bool, skip_risky: bool) -> str | None:
    if risk == "normal":
        return risk
    if skip_risky:
        return None
    if edge_pass:
        return "normal" if risk == "net_wall" else "net_wall"
    return risk


def risk_score(risk: str, edge_pass: bool, edge_pass_miss_multiplier: float) -> float:
    score = {"normal": 0.03, "net_wall": 0.18, "obstacle": 0.32}[risk]
    return score * edge_pass_miss_multiplier if edge_pass and risk != "normal" else score


def choose_next(
    current,
    candidates: list[Ball],
    obstacles,
    phase_bounds,
    travel_speed_m_s: float,
    safety_buffer_m: float,
    collection_margin_m: float,
    miss_penalty_s: float,
    edge_pass_miss_multiplier: float,
    candidate_window: int,
    edge_pass: bool,
    skip_risky: bool,
    lidar_costmap: bool,
):
    best = None
    best_score = math.inf
    shortlist = sorted(enumerate(candidates), key=lambda item: math.hypot(item[1].x - current.x, item[1].y - current.y))[
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
        risk = adjusted_risk(str(features["risk_type"]), edge_pass, skip_risky)
        if risk is None:
            continue
        score = float(features["estimated_travel_s"]) + risk_score(risk, edge_pass, edge_pass_miss_multiplier) * miss_penalty_s
        if score < best_score:
            best_score = score
            best = (index, ball, features["target"], risk)
    return best


def plan_defer_policy(
    scenario,
    area_mode: str,
    travel_speed_m_s: float,
    pickup_time_s: float,
    scan_time_s: float,
    rescan_every: int,
    safety_buffer_m: float,
    collection_margin_m: float,
    candidate_window: int,
    defer_risk_threshold: float,
    miss_penalty_s: float,
    edge_pass_miss_multiplier: float,
    skip_risky: bool,
    lidar_costmap: bool,
) -> tuple[list[Leg], RunMetrics, int, int]:
    legs: list[Leg] = []
    planned_balls: set[int] = set()
    deferred_total = 0
    skipped_total = 0
    scan_events = 0
    planned_replans = 0

    for phase_index, phase_bounds, phase_start_point in planning_phases(area_mode, scenario):
        scan_events += 1
        planned_replans += 1
        current = phase_start_point
        phase_balls = [ball for ball in scenario.balls if in_bounds(ball, phase_bounds, 0.08)]
        primary: list[Ball] = []
        deferred: list[Ball] = []
        for ball in phase_balls:
            risk = ball_risk(ball, scenario.obstacles, phase_bounds, collection_margin_m)
            if risk_score(risk, False, edge_pass_miss_multiplier) > defer_risk_threshold:
                deferred.append(ball)
            else:
                primary.append(ball)
        deferred_total += len(deferred)

        for pass_name, pass_balls in (("primary", primary), ("edge", deferred)):
            if not pass_balls:
                continue
            if pass_name == "edge":
                scan_events += 1
                planned_replans += 1
            remaining = pass_balls[:]
            while remaining:
                choice = choose_next(
                    current,
                    remaining,
                    scenario.obstacles,
                    phase_bounds,
                    travel_speed_m_s,
                    safety_buffer_m,
                    collection_margin_m,
                    miss_penalty_s,
                    edge_pass_miss_multiplier,
                    candidate_window,
                    edge_pass=pass_name == "edge",
                    skip_risky=skip_risky and pass_name == "edge",
                    lidar_costmap=lidar_costmap,
                )
                if choice is None:
                    skipped_total += len(remaining)
                    break
                best_index, ball, target, adjusted = choice
                distance, path, mode = pathfind(current, target, scenario.obstacles, phase_bounds, safety_buffer_m)
                if distance == math.inf:
                    remaining.pop(best_index)
                    skipped_total += 1
                    continue
                legs.append(Leg(phase_index, ball.id, distance, distance / travel_speed_m_s, mode, path, adjusted))
                planned_balls.add(ball.id)
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
    return legs, metrics, deferred_total, skipped_total


def summarize(rows: list[DeferComparisonRow]) -> dict[str, float | int]:
    def avg(name: str) -> float:
        return sum(float(getattr(row, name)) for row in rows) / max(1, len(rows))

    return {
        "runs": len(rows),
        "avg_planner_time_s": avg("planner_time_s"),
        "avg_defer_time_s": avg("defer_time_s"),
        "avg_delta_time_s": avg("delta_time_s"),
        "avg_planner_distance_m": avg("planner_distance_m"),
        "avg_defer_distance_m": avg("defer_distance_m"),
        "avg_delta_distance_m": avg("delta_distance_m"),
        "avg_planner_expected_misses": avg("planner_expected_misses"),
        "avg_defer_expected_misses": avg("defer_expected_misses"),
        "avg_delta_expected_misses": avg("delta_expected_misses"),
        "avg_planner_collectable": avg("planner_collectable"),
        "avg_defer_collectable": avg("defer_collectable"),
        "avg_deferred_balls": avg("deferred_balls"),
        "avg_skipped_balls": avg("skipped_balls"),
    }


def write_csv(path: Path, rows: list[DeferComparisonRow]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(DeferComparisonRow.__dataclass_fields__.keys()))
        writer.writeheader()
        for row in rows:
            writer.writerow(asdict(row))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Evaluate defer/edge-pass policy against baseline planner.")
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
    parser.add_argument("--defer-risk-threshold", type=float, default=0.10)
    parser.add_argument("--miss-penalty", type=float, default=12.0)
    parser.add_argument("--edge-pass-miss-multiplier", type=float, default=0.55)
    parser.add_argument("--skip-risky", action="store_true")
    parser.add_argument("--lidar-costmap", action="store_true")
    parser.add_argument("--json-out", type=Path, default=Path("runtime/defer-policy-eval.json"))
    parser.add_argument("--csv-out", type=Path, default=Path("runtime/defer-policy-eval.csv"))
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rows: list[DeferComparisonRow] = []
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
        defer_scenario = make_scenario(
            seed,
            args.balls,
            args.area_mode,
            args.distribution,
            args.people,
            args.fixed_obstacles,
            args.safety_buffer,
        )
        _planner_legs, planner = plan_route(
            planner_scenario,
            args.area_mode,
            args.travel_speed,
            args.pickup_time,
            args.scan_time,
            args.rescan_every,
            args.safety_buffer,
            args.collection_margin,
            args.candidate_window,
        )
        _defer_legs, defer, deferred, skipped = plan_defer_policy(
            defer_scenario,
            args.area_mode,
            args.travel_speed,
            args.pickup_time,
            args.scan_time,
            args.rescan_every,
            args.safety_buffer,
            args.collection_margin,
            args.candidate_window,
            args.defer_risk_threshold,
            args.miss_penalty,
            args.edge_pass_miss_multiplier,
            args.skip_risky,
            args.lidar_costmap,
        )
        rows.append(
            DeferComparisonRow(
                seed=seed,
                planner_collectable=planner.balls_collectable,
                defer_collectable=defer.balls_collectable,
                deferred_balls=deferred,
                skipped_balls=skipped,
                planner_time_s=planner.total_time_s,
                defer_time_s=defer.total_time_s,
                delta_time_s=defer.total_time_s - planner.total_time_s,
                planner_distance_m=planner.total_distance_m,
                defer_distance_m=defer.total_distance_m,
                delta_distance_m=defer.total_distance_m - planner.total_distance_m,
                planner_expected_misses=planner.expected_misses,
                defer_expected_misses=defer.expected_misses,
                delta_expected_misses=defer.expected_misses - planner.expected_misses,
                planner_risky_balls=planner.risky_balls,
                defer_risky_balls=defer.risky_balls,
            )
        )

    summary = summarize(rows)
    payload = {"config": vars(args), "summary": summary, "runs": [asdict(row) for row in rows]}
    print(json.dumps({"config": vars(args), "summary": summary}, indent=2, default=str))
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(payload, indent=2, default=str), encoding="utf-8")
    if args.csv_out:
        write_csv(args.csv_out, rows)


if __name__ == "__main__":
    main()
