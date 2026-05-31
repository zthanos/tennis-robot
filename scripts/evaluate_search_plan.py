#!/usr/bin/env python3
"""Evaluate the half-court search pattern before running it in Webots."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import asdict, dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "ball_detector"))

from collector import BallObservationInput  # noqa: E402
from route_benchmark import (  # noqa: E402
    BALL_PHASE_MARGIN_M,
    Ball,
    Bounds,
    Point,
    ball_risk,
    half_bounds,
    in_bounds,
    make_scenario,
    phase_start,
)
from search import HalfCourtSearchBehavior, SearchConfig, SearchState  # noqa: E402


SAMPLE_STEP_M = 0.25


@dataclass
class SearchEvalRow:
    seed: int
    balls_in_phase: int
    boundary_priority_balls: int
    boundary_first_detected: int
    lane_only_detected: int
    boundary_first_detected_rate: float
    lane_only_detected_rate: float
    boundary_first_coverage_pct: float
    lane_only_coverage_pct: float
    boundary_first_time_to_first_s: float
    lane_only_time_to_first_s: float
    boundary_first_time_to_first_priority_s: float
    lane_only_time_to_first_priority_s: float
    boundary_first_path_m: float
    lane_only_path_m: float
    boundary_first_waypoints: int
    lane_only_waypoints: int
    resume_check_passed: int


@dataclass(frozen=True)
class PathSample:
    point: Point
    heading_rad: float
    distance_along_m: float
    elapsed_s: float


def angle_delta_rad(a: float, b: float) -> float:
    return (a - b + math.pi) % (2 * math.pi) - math.pi


def path_distance(points: list[Point]) -> float:
    return sum(math.hypot(point.x - points[index].x, point.y - points[index].y) for index, point in enumerate(points[1:]))


def sampled_path(
    points: list[Point],
    travel_speed_m_s: float,
    step_m: float = SAMPLE_STEP_M,
    waypoint_scan_s: float = 0.0,
    scan_headings: int = 12,
) -> list[PathSample]:
    samples: list[PathSample] = []
    distance_along = 0.0
    elapsed_s = 0.0
    for waypoint in points:
        if waypoint_scan_s <= 0.0:
            continue
        for index in range(max(1, scan_headings)):
            heading = (math.pi * 2 * index) / max(1, scan_headings)
            samples.append(
                PathSample(
                    waypoint,
                    heading,
                    distance_along,
                    elapsed_s + waypoint_scan_s * index / max(1, scan_headings),
                )
            )
        elapsed_s += waypoint_scan_s

    for start, end in zip(points, points[1:]):
        dx = end.x - start.x
        dy = end.y - start.y
        segment_m = math.hypot(dx, dy)
        if segment_m <= 1e-9:
            continue
        heading = math.atan2(dy, dx)
        steps = max(1, math.ceil(segment_m / step_m))
        for index in range(steps):
            t = index / steps
            samples.append(
                PathSample(
                    Point(start.x + dx * t, start.y + dy * t),
                    heading,
                    distance_along + segment_m * t,
                    elapsed_s + segment_m * t / travel_speed_m_s,
                )
            )
        elapsed_s += segment_m / travel_speed_m_s
        distance_along += segment_m
    if points:
        heading = samples[-1].heading_rad if samples else 0.0
        samples.append(PathSample(points[-1], heading, distance_along, elapsed_s))
    return samples


def visible_from_sample(
    sample: PathSample,
    target: Point,
    detection_range_m: float,
    camera_fov_rad: float,
) -> bool:
    dx = target.x - sample.point.x
    dy = target.y - sample.point.y
    distance_m = math.hypot(dx, dy)
    if distance_m > detection_range_m:
        return False
    bearing = math.atan2(dy, dx)
    return abs(angle_delta_rad(bearing, sample.heading_rad)) <= camera_fov_rad / 2


def first_seen_sample(
    samples: list[PathSample],
    ball: Ball,
    detection_range_m: float,
    camera_fov_rad: float,
) -> PathSample | None:
    target = Point(ball.x, ball.y)
    for sample in samples:
        if visible_from_sample(sample, target, detection_range_m, camera_fov_rad):
            return sample
    return None


def coverage_pct(
    samples: list[PathSample],
    bounds: Bounds,
    detection_range_m: float,
    camera_fov_rad: float,
    grid_m: float,
) -> float:
    total = 0
    visible = 0
    x = bounds.min_x + BALL_PHASE_MARGIN_M
    while x <= bounds.max_x - BALL_PHASE_MARGIN_M + 1e-9:
        y = bounds.min_y + BALL_PHASE_MARGIN_M
        while y <= bounds.max_y - BALL_PHASE_MARGIN_M + 1e-9:
            total += 1
            point = Point(x, y)
            if any(visible_from_sample(sample, point, detection_range_m, camera_fov_rad) for sample in samples):
                visible += 1
            y += grid_m
        x += grid_m
    return 100.0 * visible / max(1, total)


def build_paths(side: str, config: SearchConfig) -> tuple[list[Point], list[Point], HalfCourtSearchBehavior]:
    behavior = HalfCourtSearchBehavior(config)
    bounds = half_bounds(side)
    start = phase_start(bounds)
    boundary_first = [start]
    boundary_first.extend(Point(x, y) for x, y in behavior.boundary_waypoints)
    boundary_first.extend(Point(x, y) for x, y in behavior.lane_waypoints)
    lane_only = [start]
    lane_only.extend(Point(x, y) for x, y in behavior.lane_waypoints)
    return boundary_first, lane_only, behavior


def time_or_inf(sample: PathSample | None) -> float:
    if sample is None:
        return math.inf
    return sample.elapsed_s


def resume_check(config: SearchConfig) -> bool:
    behavior = HalfCourtSearchBehavior(config)
    first_x, first_y = behavior.boundary_waypoints[0]
    command = behavior.update(
        first_x,
        first_y,
        0.0,
        BallObservationInput(False),
        front_range_m=2.0,
        dt_s=0.032,
    )
    if command.state != SearchState.BOUNDARY_FIRST or command.waypoint_index < 1:
        return False
    interrupted = behavior.update(
        first_x,
        first_y,
        0.0,
        BallObservationInput(True, distance_m=2.0, confidence=0.9, source="oak_depth"),
        front_range_m=2.0,
        dt_s=0.032,
        target_id=99,
    )
    if interrupted.state != SearchState.BALL_DETECTED:
        return False
    resumed = behavior.update(
        first_x,
        first_y,
        0.0,
        BallObservationInput(False),
        front_range_m=2.0,
        dt_s=config.target_hold_s + 0.01,
    )
    return resumed.state == SearchState.BOUNDARY_FIRST and resumed.waypoint_index >= 1


def evaluate_seed(args: argparse.Namespace, seed: int) -> SearchEvalRow:
    side = args.side
    config = SearchConfig(
        side=side,
        lane_width_m=args.lane_width,
        detection_confidence_threshold=args.detection_confidence,
    )
    boundary_path, lane_path, behavior = build_paths(side, config)
    boundary_samples = sampled_path(
        boundary_path,
        args.travel_speed,
        waypoint_scan_s=args.waypoint_scan_time,
        scan_headings=args.scan_headings,
    )
    lane_samples = sampled_path(
        lane_path,
        args.travel_speed,
        waypoint_scan_s=args.waypoint_scan_time,
        scan_headings=args.scan_headings,
    )
    phase_bounds = half_bounds(side)
    scenario = make_scenario(
        seed,
        args.balls,
        "two-phase",
        args.distribution,
        args.people,
        args.fixed_obstacles,
        args.safety_buffer,
    )
    phase_balls = [ball for ball in scenario.balls if in_bounds(ball, phase_bounds, BALL_PHASE_MARGIN_M)]
    priority_balls = [
        ball for ball in phase_balls
        if ball_risk(ball, scenario.obstacles, phase_bounds, args.collection_margin) != "normal"
    ]

    boundary_seen = [
        first_seen_sample(boundary_samples, ball, args.detection_range, args.camera_fov_rad)
        for ball in phase_balls
    ]
    lane_seen = [
        first_seen_sample(lane_samples, ball, args.detection_range, args.camera_fov_rad)
        for ball in phase_balls
    ]
    boundary_priority_seen = [
        first_seen_sample(boundary_samples, ball, args.detection_range, args.camera_fov_rad)
        for ball in priority_balls
    ]
    lane_priority_seen = [
        first_seen_sample(lane_samples, ball, args.detection_range, args.camera_fov_rad)
        for ball in priority_balls
    ]
    boundary_first_seen = min((value for value in boundary_seen if value is not None), key=lambda s: s.elapsed_s, default=None)
    lane_first_seen = min((value for value in lane_seen if value is not None), key=lambda s: s.elapsed_s, default=None)
    boundary_first_priority = min(
        (value for value in boundary_priority_seen if value is not None),
        key=lambda s: s.elapsed_s,
        default=None,
    )
    lane_first_priority = min(
        (value for value in lane_priority_seen if value is not None),
        key=lambda s: s.elapsed_s,
        default=None,
    )

    return SearchEvalRow(
        seed=seed,
        balls_in_phase=len(phase_balls),
        boundary_priority_balls=len(priority_balls),
        boundary_first_detected=sum(1 for value in boundary_seen if value is not None),
        lane_only_detected=sum(1 for value in lane_seen if value is not None),
        boundary_first_detected_rate=sum(1 for value in boundary_seen if value is not None) / max(1, len(phase_balls)),
        lane_only_detected_rate=sum(1 for value in lane_seen if value is not None) / max(1, len(phase_balls)),
        boundary_first_coverage_pct=coverage_pct(
            boundary_samples,
            phase_bounds,
            args.detection_range,
            args.camera_fov_rad,
            args.coverage_grid,
        ),
        lane_only_coverage_pct=coverage_pct(
            lane_samples,
            phase_bounds,
            args.detection_range,
            args.camera_fov_rad,
            args.coverage_grid,
        ),
        boundary_first_time_to_first_s=time_or_inf(boundary_first_seen),
        lane_only_time_to_first_s=time_or_inf(lane_first_seen),
        boundary_first_time_to_first_priority_s=time_or_inf(boundary_first_priority),
        lane_only_time_to_first_priority_s=time_or_inf(lane_first_priority),
        boundary_first_path_m=path_distance(boundary_path),
        lane_only_path_m=path_distance(lane_path),
        boundary_first_waypoints=len(behavior.boundary_waypoints) + len(behavior.lane_waypoints),
        lane_only_waypoints=len(behavior.lane_waypoints),
        resume_check_passed=int(resume_check(config)),
    )


def finite_mean(values: list[float]) -> float:
    finite = [value for value in values if math.isfinite(value)]
    return sum(finite) / max(1, len(finite))


def avg(rows: list[SearchEvalRow], name: str) -> float:
    return sum(float(getattr(row, name)) for row in rows) / max(1, len(rows))


def summarize(rows: list[SearchEvalRow]) -> dict[str, float | int]:
    return {
        "runs": len(rows),
        "avg_balls_in_phase": avg(rows, "balls_in_phase"),
        "avg_boundary_priority_balls": avg(rows, "boundary_priority_balls"),
        "avg_boundary_first_detected_rate": avg(rows, "boundary_first_detected_rate"),
        "avg_lane_only_detected_rate": avg(rows, "lane_only_detected_rate"),
        "avg_boundary_first_coverage_pct": avg(rows, "boundary_first_coverage_pct"),
        "avg_lane_only_coverage_pct": avg(rows, "lane_only_coverage_pct"),
        "avg_boundary_first_time_to_first_s": finite_mean([row.boundary_first_time_to_first_s for row in rows]),
        "avg_lane_only_time_to_first_s": finite_mean([row.lane_only_time_to_first_s for row in rows]),
        "avg_boundary_first_time_to_first_priority_s": finite_mean(
            [row.boundary_first_time_to_first_priority_s for row in rows]
        ),
        "avg_lane_only_time_to_first_priority_s": finite_mean([row.lane_only_time_to_first_priority_s for row in rows]),
        "avg_boundary_first_path_m": avg(rows, "boundary_first_path_m"),
        "avg_lane_only_path_m": avg(rows, "lane_only_path_m"),
        "resume_checks_passed": sum(row.resume_check_passed for row in rows),
    }


def write_csv(path: Path, rows: list[SearchEvalRow]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(SearchEvalRow.__dataclass_fields__.keys()))
        writer.writeheader()
        for row in rows:
            writer.writerow(asdict(row))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Evaluate half-court search plan coverage and detection timing.")
    parser.add_argument("--runs", type=int, default=100)
    parser.add_argument("--balls", type=int, default=40)
    parser.add_argument("--seed", type=int, default=23000)
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument("--distribution", choices=("realistic", "uniform"), default="realistic")
    parser.add_argument("--people", type=int, default=3)
    parser.add_argument("--fixed-obstacles", type=int, default=3)
    parser.add_argument("--travel-speed", type=float, default=0.24)
    parser.add_argument("--lane-width", type=float, default=2.2)
    parser.add_argument("--detection-range", type=float, default=5.5)
    parser.add_argument("--camera-fov-deg", type=float, default=69.0)
    parser.add_argument("--waypoint-scan-time", type=float, default=0.8)
    parser.add_argument("--scan-headings", type=int, default=12)
    parser.add_argument("--detection-confidence", type=float, default=0.35)
    parser.add_argument("--coverage-grid", type=float, default=0.35)
    parser.add_argument("--safety-buffer", type=float, default=0.55)
    parser.add_argument("--collection-margin", type=float, default=0.55)
    parser.add_argument("--json-out", type=Path, default=Path("runtime/search-plan-eval.json"))
    parser.add_argument("--csv-out", type=Path, default=Path("runtime/search-plan-eval.csv"))
    args = parser.parse_args()
    args.camera_fov_rad = math.radians(args.camera_fov_deg)
    return args


def main() -> None:
    args = parse_args()
    rows = [evaluate_seed(args, args.seed + index) for index in range(args.runs)]
    summary = summarize(rows)
    payload = {
        "config": {
            "runs": args.runs,
            "balls": args.balls,
            "seed_start": args.seed,
            "side": args.side,
            "distribution": args.distribution,
            "people": args.people,
            "fixed_obstacles": args.fixed_obstacles,
            "travel_speed_m_s": args.travel_speed,
            "lane_width_m": args.lane_width,
            "detection_range_m": args.detection_range,
            "camera_fov_deg": args.camera_fov_deg,
            "waypoint_scan_time_s": args.waypoint_scan_time,
            "scan_headings": args.scan_headings,
            "coverage_grid_m": args.coverage_grid,
            "safety_buffer_m": args.safety_buffer,
            "collection_margin_m": args.collection_margin,
        },
        "summary": summary,
        "runs": [asdict(row) for row in rows],
    }
    print(json.dumps({"config": payload["config"], "summary": summary}, indent=2))
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    if args.csv_out:
        write_csv(args.csv_out, rows)


if __name__ == "__main__":
    main()
