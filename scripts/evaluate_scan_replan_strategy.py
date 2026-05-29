#!/usr/bin/env python3
"""Evaluate scan-first route planning with event-based replans."""

from __future__ import annotations

import argparse
import copy
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path

from eval_utils import avg_field, write_dataclass_csv
from evaluate_defer_policy import STRATEGY_PRESETS, adjusted_risk, choose_next, risk_score
from route_benchmark import (
    BALL_PHASE_MARGIN_M,
    Ball,
    Leg,
    RunMetrics,
    ball_risk,
    candidate_features,
    in_bounds,
    make_scenario,
    nearest_shortlist,
    pathfind,
    plan_route,
    planning_phases,
    run_metrics_from_legs,
)


@dataclass
class ScanReplanRow:
    seed: int
    planner_collectable: int
    scan_collectable: int
    skipped_balls: int
    planner_time_s: float
    scan_time_s: float
    delta_time_s: float
    planner_distance_m: float
    scan_distance_m: float
    delta_distance_m: float
    planner_expected_misses: float
    scan_expected_misses: float
    delta_expected_misses: float
    scan_events: int
    event_replans: int


def build_scan_plan(
    start,
    remaining: list[Ball],
    obstacles,
    phase_bounds,
    travel_speed_m_s: float,
    safety_buffer_m: float,
    collection_margin_m: float,
    miss_penalty_s: float,
    edge_pass_miss_multiplier: float,
    candidate_window: int,
    skip_risky: bool,
    lidar_costmap: bool,
) -> list[tuple[Ball, object, str]]:
    """Plan an ordered batch from the current scan snapshot."""

    current = start
    planned: list[tuple[Ball, object, str]] = []
    candidates = remaining[:]
    while candidates:
        choice = choose_next(
            current,
            candidates,
            obstacles,
            phase_bounds,
            travel_speed_m_s,
            safety_buffer_m,
            collection_margin_m,
            miss_penalty_s,
            edge_pass_miss_multiplier,
            candidate_window,
            edge_pass=False,
            skip_risky=skip_risky,
            lidar_costmap=lidar_costmap,
        )
        if choice is None:
            break
        index, ball, target, risk = choice
        distance, path, _mode = pathfind(current, target, obstacles, phase_bounds, safety_buffer_m)
        if distance == math.inf:
            candidates.pop(index)
            continue
        planned.append((ball, target, risk))
        current = path[-1]
        candidates.pop(index)
    return planned


def plan_scan_replan_policy(
    scenario,
    area_mode: str,
    travel_speed_m_s: float,
    pickup_time_s: float,
    scan_time_s: float,
    replan_every: int,
    safety_buffer_m: float,
    collection_margin_m: float,
    candidate_window: int,
    miss_penalty_s: float,
    edge_pass_miss_multiplier: float,
    skip_risky: bool,
    lidar_costmap: bool,
) -> tuple[list[Leg], RunMetrics, int, int, int]:
    legs: list[Leg] = []
    planned_balls: set[int] = set()
    scan_events = 0
    event_replans = 0

    for phase_index, phase_bounds, phase_start_point in planning_phases(area_mode, scenario):
        current = phase_start_point
        remaining = [ball for ball in scenario.balls if in_bounds(ball, phase_bounds, BALL_PHASE_MARGIN_M)]

        while remaining:
            scan_events += 1
            plan = build_scan_plan(
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
                skip_risky,
                lidar_costmap,
            )
            if not plan:
                break

            pickups_since_scan = 0
            replan_requested = False
            for ball, target, risk in plan:
                if ball not in remaining:
                    continue
                distance, path, mode = pathfind(current, target, scenario.obstacles, phase_bounds, safety_buffer_m)
                if distance == math.inf:
                    event_replans += 1
                    replan_requested = True
                    break

                legs.append(Leg(phase_index, ball.id, distance, distance / travel_speed_m_s, mode, path, risk))
                planned_balls.add(ball.id)
                current = path[-1]
                remaining.remove(ball)
                pickups_since_scan += 1

                if replan_every > 0 and remaining and pickups_since_scan >= replan_every:
                    event_replans += 1
                    replan_requested = True
                    break

            if not replan_requested:
                break

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
        scan_events + event_replans,
    )
    return legs, metrics, metrics.balls_blocked, scan_events, event_replans


def summarize(rows: list[ScanReplanRow]) -> dict[str, float | int]:
    avg = lambda name: avg_field(rows, name)
    return {
        "runs": len(rows),
        "avg_planner_time_s": avg("planner_time_s"),
        "avg_scan_time_s": avg("scan_time_s"),
        "avg_delta_time_s": avg("delta_time_s"),
        "avg_planner_distance_m": avg("planner_distance_m"),
        "avg_scan_distance_m": avg("scan_distance_m"),
        "avg_delta_distance_m": avg("delta_distance_m"),
        "avg_planner_expected_misses": avg("planner_expected_misses"),
        "avg_scan_expected_misses": avg("scan_expected_misses"),
        "avg_delta_expected_misses": avg("delta_expected_misses"),
        "avg_planner_collectable": avg("planner_collectable"),
        "avg_scan_collectable": avg("scan_collectable"),
        "avg_skipped_balls": avg("skipped_balls"),
        "avg_scan_events": avg("scan_events"),
        "avg_event_replans": avg("event_replans"),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Evaluate scan-first route planning with event-based replans.")
    parser.add_argument("--strategy-preset", choices=tuple(STRATEGY_PRESETS), default="thorough")
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
    parser.add_argument("--replan-every", type=int, default=5)
    parser.add_argument("--safety-buffer", type=float, default=0.55)
    parser.add_argument("--collection-margin", type=float, default=0.55)
    parser.add_argument("--candidate-window", type=int, default=12)
    parser.add_argument("--lidar-costmap", action="store_true")
    parser.add_argument("--json-out", type=Path, default=Path("runtime/scan-replan-strategy-eval.json"))
    parser.add_argument("--csv-out", type=Path, default=Path("runtime/scan-replan-strategy-eval.csv"))
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    preset = STRATEGY_PRESETS[args.strategy_preset]
    rows: list[ScanReplanRow] = []
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
        scan_scenario = copy.deepcopy(planner_scenario)
        _planner_legs, planner = plan_route(
            planner_scenario,
            area_mode=args.area_mode,
            travel_speed_m_s=args.travel_speed,
            pickup_time_s=args.pickup_time,
            scan_time_s=args.scan_time,
            rescan_every=args.replan_every,
            safety_buffer_m=args.safety_buffer,
            collection_margin_m=args.collection_margin,
            candidate_window=args.candidate_window,
            lidar_costmap=args.lidar_costmap,
        )
        _scan_legs, scan, skipped, scan_events, event_replans = plan_scan_replan_policy(
            scan_scenario,
            args.area_mode,
            args.travel_speed,
            args.pickup_time,
            args.scan_time,
            args.replan_every,
            args.safety_buffer,
            args.collection_margin,
            args.candidate_window,
            float(preset["miss_penalty"]),
            float(preset["edge_pass_miss_multiplier"]),
            bool(preset["skip_risky"]),
            args.lidar_costmap,
        )
        rows.append(
            ScanReplanRow(
                seed=seed,
                planner_collectable=planner.balls_collectable,
                scan_collectable=scan.balls_collectable,
                skipped_balls=skipped,
                planner_time_s=planner.total_time_s,
                scan_time_s=scan.total_time_s,
                delta_time_s=scan.total_time_s - planner.total_time_s,
                planner_distance_m=planner.total_distance_m,
                scan_distance_m=scan.total_distance_m,
                delta_distance_m=scan.total_distance_m - planner.total_distance_m,
                planner_expected_misses=planner.expected_misses,
                scan_expected_misses=scan.expected_misses,
                delta_expected_misses=scan.expected_misses - planner.expected_misses,
                scan_events=scan_events,
                event_replans=event_replans,
            )
        )

    summary = summarize(rows)
    payload = {"config": vars(args), "summary": summary, "runs": [asdict(row) for row in rows]}
    print(json.dumps({"config": vars(args), "summary": summary}, indent=2, default=str))
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(payload, indent=2, default=str), encoding="utf-8")
    if args.csv_out:
        write_dataclass_csv(args.csv_out, rows)


if __name__ == "__main__":
    main()
