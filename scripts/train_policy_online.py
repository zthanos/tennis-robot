#!/usr/bin/env python3
"""DAgger-style self-play loop for the next-ball MLP policy.

Each iteration:
  1. Run current model as policy on N scenarios (epsilon-greedy exploration)
  2. At every decision point, compute counterfactual rollout quality for ALL candidates
  3. Combine self-play data with baseline + all previous iterations → retrain from scratch
  4. Evaluate vs greedy planner on held-out scenarios
  5. Save model version; update best model if improved
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import random
from pathlib import Path

import numpy as np

from route_benchmark import (
    BALL_PHASE_MARGIN_M,
    TrainingRow,
    ball_centroid,
    candidate_features,
    dist,
    heading_bearing_cost,
    in_bounds,
    make_scenario,
    nearest_shortlist,
    pathfind,
    plan_route,
    planning_phases,
    write_training_csv,
)
from generate_outcome_training import greedy_rollout_cost, local_candidate_cost
from train_next_ball_policy import (
    MLP,
    NUMERIC_FEATURES,
    RISK_TYPES,
    build_groups,
    grouped_top1_accuracy,
    load_rows,
    make_matrix,
    split_by_seed,
    standardize,
    train_mlp,
)
from evaluate_next_ball_policy import model_score, plan_model_policy


SELF_PLAY_SEED_BASE = 10000
EVAL_SEED_BASE = 20000


# ── Self-play data collection ─────────────────────────────────────────────────

def collect_self_play_rows(
    model: dict | None,
    seed_start: int,
    n_scenarios: int,
    balls: int,
    area_mode: str,
    distribution: str,
    people: int,
    fixed_obstacles: int,
    travel_speed_m_s: float,
    pickup_time_s: float,
    safety_buffer_m: float,
    collection_margin_m: float,
    candidate_window: int,
    rollout_depth: int,
    miss_penalty_s: float,
    epsilon: float,
    lidar_costmap: bool,
    rng: random.Random,
) -> list[TrainingRow]:
    rows: list[TrainingRow] = []

    for sc_idx in range(n_scenarios):
        seed = seed_start + sc_idx
        scenario = make_scenario(seed, balls, area_mode, distribution, people, fixed_obstacles, safety_buffer_m)

        for phase_index, phase_bounds, phase_start_point in planning_phases(area_mode, scenario):
            phase_balls = [b for b in scenario.balls if in_bounds(b, phase_bounds, BALL_PHASE_MARGIN_M)]
            total_phase_balls = max(1, len(phase_balls))
            current = phase_start_point
            prev = phase_start_point
            remaining = list(phase_balls)
            decision_id = 0
            step_in_phase = 0

            while remaining:
                step_in_phase += 1
                shortlist = list(nearest_shortlist(current, remaining, candidate_window))
                centroid_pt = ball_centroid(remaining)
                phase_span_x = max(1e-6, phase_bounds.max_x - phase_bounds.min_x)

                # Evaluate every candidate: compute model score + rollout quality
                evaluated: list[tuple[float, float, int, object, dict]] = []
                # items: (outcome_cost, model_score, remaining_index, ball, features)

                for rank, (r_idx, candidate) in enumerate(shortlist, start=1):
                    feats = candidate_features(
                        current, candidate, scenario.obstacles, phase_bounds,
                        safety_buffer_m, collection_margin_m, travel_speed_m_s, lidar_costmap,
                    )
                    if feats is None:
                        continue

                    future = [b for b in remaining if b.id != candidate.id]
                    immediate = local_candidate_cost(feats, pickup_time_s, miss_penalty_s)
                    rollout = greedy_rollout_cost(
                        feats["target"], future, scenario.obstacles, phase_bounds,
                        safety_buffer_m, collection_margin_m, travel_speed_m_s,
                        pickup_time_s, miss_penalty_s, candidate_window, rollout_depth, lidar_costmap,
                    )

                    if model is not None:
                        row_dict = {
                            **feats,
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
                            "balls_within_2m": sum(1 for b in remaining if b.id != candidate.id and dist(b, candidate) <= 2.0),
                            "phase_progress": step_in_phase / total_phase_balls,
                            "bearing_cost_rad": heading_bearing_cost(prev, current, candidate),
                            "cluster_distance_m": dist(candidate, centroid_pt),
                            "normalized_x": (candidate.x - phase_bounds.min_x) / phase_span_x,
                        }
                        sc = model_score(model, row_dict)
                    else:
                        sc = -float(feats["estimated_route_distance_m"])

                    evaluated.append((immediate + rollout, sc, r_idx, candidate, feats))

                if not evaluated:
                    break

                costs = [e[0] for e in evaluated]
                mean_cost = sum(costs) / len(costs)
                denom = max(1e-6, mean_cost)
                decision_id += 1

                # Epsilon-greedy: choose which ball to move to
                by_score = sorted(evaluated, key=lambda e: e[1], reverse=True)
                if epsilon > 0.0 and rng.random() < epsilon and len(by_score) >= 2:
                    top_k = min(3, len(by_score))
                    chosen = rng.choice(by_score[:top_k])
                else:
                    chosen = by_score[0]

                ranked_by_dist = sorted(evaluated, key=lambda e: float(e[4]["estimated_route_distance_m"]))
                rank_lookup = {e[3].id: r + 1 for r, e in enumerate(ranked_by_dist)}
                best_ball_id = min(evaluated, key=lambda e: e[0])[3].id

                for (outcome_cost, _sc, _r_idx, candidate, feats) in evaluated:
                    adv = (mean_cost - outcome_cost) / denom
                    rows.append(TrainingRow(
                        seed=seed,
                        decision_id=decision_id,
                        phase=phase_index,
                        step_in_phase=step_in_phase,
                        remaining_balls=len(remaining),
                        candidate_rank_by_distance=rank_lookup[candidate.id],
                        selected=int(candidate.id == best_ball_id),
                        robot_x_m=current.x,
                        robot_y_m=current.y,
                        ball_id=candidate.id,
                        ball_x_m=candidate.x,
                        ball_y_m=candidate.y,
                        direct_distance_m=float(feats["direct_distance_m"]),
                        estimated_route_distance_m=float(feats["estimated_route_distance_m"]),
                        estimated_travel_s=float(feats["estimated_travel_s"]),
                        pickup_pose_distance_m=float(feats["pickup_pose_distance_m"]),
                        approach_clear=int(feats["approach_clear"]),
                        risk_type=str(feats["risk_type"]),
                        miss_probability=float(feats["miss_probability"]),
                        distance_to_net_m=float(feats["distance_to_net_m"]),
                        distance_to_nearest_wall_m=float(feats["distance_to_nearest_wall_m"]),
                        distance_to_nearest_obstacle_m=float(feats["distance_to_nearest_obstacle_m"]),
                        phase_min_x_m=phase_bounds.min_x,
                        phase_max_x_m=phase_bounds.max_x,
                        phase_min_y_m=phase_bounds.min_y,
                        phase_max_y_m=phase_bounds.max_y,
                        balls_within_2m=sum(1 for b in remaining if b.id != candidate.id and dist(b, candidate) <= 2.0),
                        phase_progress=step_in_phase / total_phase_balls,
                        bearing_cost_rad=heading_bearing_cost(prev, current, candidate),
                        cluster_distance_m=dist(candidate, centroid_pt),
                        normalized_x=(candidate.x - phase_bounds.min_x) / phase_span_x,
                        quality_advantage=adv,
                    ))

                _cost, _sc, best_r_idx, _ball, best_feats = chosen
                distance, path, _mode = pathfind(
                    current, best_feats["target"], scenario.obstacles, phase_bounds, safety_buffer_m,
                )
                if distance == math.inf:
                    remaining.pop(best_r_idx)
                    continue
                prev = current
                current = path[-1]
                remaining.pop(best_r_idx)

    return rows


# ── Training helpers ──────────────────────────────────────────────────────────

def load_combined_rows(csv_paths: list[Path]) -> list[dict[str, str]]:
    combined = []
    for p in csv_paths:
        if p.exists():
            combined.extend(load_rows(p))
    return combined


def train_on_rows(
    rows: list[dict[str, str]],
    epochs: int,
    lr: float,
    temperature: float,
    l2: float,
    val_fraction: float,
    batch_size: int = 0,
) -> tuple[MLP, np.ndarray, np.ndarray, list[str], dict]:
    train_rows, val_rows = split_by_seed(rows, val_fraction)
    train_x, train_y, _sel, feature_names = make_matrix(train_rows)
    val_x, val_y, _sel, _ = make_matrix(val_rows)
    train_x, val_x, feat_mean, feat_std = standardize(train_x, val_x)
    groups = build_groups(train_rows)
    mlp, losses = train_mlp(train_x, train_y, groups, epochs, lr, l2, temperature, batch_size)
    metrics = {
        "rows": len(rows),
        "train_rows": len(train_rows),
        "validation_rows": len(val_rows),
        "train_ranking": grouped_top1_accuracy(train_rows, mlp.score(train_x)),
        "validation_ranking": grouped_top1_accuracy(val_rows, mlp.score(val_x)),
        "initial_loss": losses[0] if losses else math.nan,
        "final_loss": losses[-1] if losses else math.nan,
    }
    return mlp, feat_mean, feat_std, feature_names, metrics


def model_to_dict(mlp: MLP, mean: np.ndarray, std: np.ndarray, feature_names: list[str], metrics: dict) -> dict:
    return {
        "model_type": "mlp",
        "feature_names": feature_names,
        "risk_types": RISK_TYPES,
        "mean": mean.tolist(),
        "std": std.tolist(),
        "decision_rule": "score every candidate row and choose the highest score within a decision group",
        "metrics": metrics,
        **mlp.to_dict(),
    }


# ── Evaluation ────────────────────────────────────────────────────────────────

def evaluate_model(
    model: dict,
    seed_start: int,
    n_runs: int,
    args: argparse.Namespace,
) -> dict:
    planner_times, model_times, wins = [], [], 0

    for i in range(n_runs):
        seed = seed_start + i
        planner_sc = make_scenario(seed, args.balls, args.area_mode, args.distribution,
                                   args.people, args.fixed_obstacles, args.safety_buffer)
        model_sc = copy.deepcopy(planner_sc)

        _legs, pm = plan_route(
            planner_sc, args.area_mode, args.travel_speed, args.pickup_time,
            args.scan_time, args.rescan_every, args.safety_buffer,
            args.collection_margin, args.candidate_window,
            lidar_costmap=args.lidar_costmap,
        )
        _legs, mm, _n = plan_model_policy(
            model, model_sc, args.area_mode, args.travel_speed, args.pickup_time,
            args.scan_time, args.rescan_every, args.safety_buffer,
            args.collection_margin, args.candidate_window, args.lidar_costmap,
        )
        planner_times.append(pm.total_time_s)
        model_times.append(mm.total_time_s)
        if mm.total_time_s < pm.total_time_s:
            wins += 1

    n = len(planner_times)
    avg_p = sum(planner_times) / n
    avg_m = sum(model_times) / n
    return {
        "runs": n,
        "avg_planner_time_s": avg_p,
        "avg_model_time_s": avg_m,
        "avg_delta_time_s": avg_m - avg_p,
        "win_rate": wins / n,
    }


# ── CLI ───────────────────────────────────────────────────────────────────────

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="DAgger self-play training loop for next-ball MLP policy.")
    p.add_argument("--baseline-csv", type=Path, default=Path("runtime/route-outcome-training.csv"))
    p.add_argument("--baseline-model", type=Path, default=Path("runtime/next-ball-policy-model.json"))
    p.add_argument("--output-dir", type=Path, default=Path("runtime/online"))
    p.add_argument("--iterations", type=int, default=5)
    p.add_argument("--scenarios-per-iter", type=int, default=300)
    p.add_argument("--eval-runs", type=int, default=100)
    p.add_argument("--balls", type=int, default=40)
    p.add_argument("--area-mode", choices=("half", "two-phase", "full"), default="two-phase")
    p.add_argument("--distribution", choices=("realistic", "uniform"), default="realistic")
    p.add_argument("--people", type=int, default=3)
    p.add_argument("--fixed-obstacles", type=int, default=3)
    p.add_argument("--travel-speed", type=float, default=0.85)
    p.add_argument("--pickup-time", type=float, default=1.2)
    p.add_argument("--scan-time", type=float, default=7.0)
    p.add_argument("--rescan-every", type=int, default=5)
    p.add_argument("--safety-buffer", type=float, default=0.55)
    p.add_argument("--collection-margin", type=float, default=0.55)
    p.add_argument("--candidate-window", type=int, default=12)
    p.add_argument("--rollout-depth", type=int, default=8)
    p.add_argument("--miss-penalty", type=float, default=12.0)
    p.add_argument("--lidar-costmap", action="store_true")
    p.add_argument("--epochs", type=int, default=300)
    p.add_argument("--learning-rate", type=float, default=0.005)
    p.add_argument("--temperature", type=float, default=5.0)
    p.add_argument("--l2", type=float, default=0.001)
    p.add_argument("--validation-fraction", type=float, default=0.2)
    p.add_argument("--epsilon-start", type=float, default=0.05)
    p.add_argument("--epsilon-decay", type=float, default=0.80)
    p.add_argument("--batch-size", type=int, default=256,
                   help="Mini-batch size in groups (0=full-batch)")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    rng = random.Random(999)

    current_model: dict | None = None
    if args.baseline_model.exists():
        current_model = json.loads(args.baseline_model.read_text(encoding="utf-8"))

    best_model_time = math.inf
    history: list[dict] = []

    # Evaluate baseline before any self-play
    if current_model is not None:
        print("=== Baseline model evaluation ===")
        result = evaluate_model(current_model, EVAL_SEED_BASE, args.eval_runs, args)
        entry = {"iteration": 0, **result}
        history.append(entry)
        best_model_time = result["avg_model_time_s"]
        print(f"  planner={result['avg_planner_time_s']:.1f}s  model={result['avg_model_time_s']:.1f}s  "
              f"dt={result['avg_delta_time_s']:+.1f}s  win={result['win_rate']:.1%}")

    for iteration in range(1, args.iterations + 1):
        epsilon = args.epsilon_start * (args.epsilon_decay ** (iteration - 1))
        seed_start = SELF_PLAY_SEED_BASE + (iteration - 1) * args.scenarios_per_iter

        print(f"\n=== Iteration {iteration}/{args.iterations}  eps={epsilon:.3f} ===")

        # 1. Collect self-play data
        sp_csv = args.output_dir / f"self-play-iter{iteration}.csv"
        print(f"  collecting {args.scenarios_per_iter} scenarios (seeds {seed_start}-{seed_start + args.scenarios_per_iter - 1})...")
        sp_rows = collect_self_play_rows(
            current_model, seed_start, args.scenarios_per_iter, args.balls,
            args.area_mode, args.distribution, args.people, args.fixed_obstacles,
            args.travel_speed, args.pickup_time, args.safety_buffer, args.collection_margin,
            args.candidate_window, args.rollout_depth, args.miss_penalty,
            epsilon, args.lidar_costmap, rng,
        )
        write_training_csv(sp_csv, sp_rows)
        print(f"  saved {len(sp_rows)} rows to {sp_csv}")

        # 2. Combine all data
        all_csvs = [args.baseline_csv] + [
            args.output_dir / f"self-play-iter{i}.csv" for i in range(1, iteration + 1)
        ]
        all_rows = load_combined_rows(all_csvs)
        print(f"  combined rows: {len(all_rows)}")

        # 3. Train from scratch
        print(f"  training {args.epochs} epochs...")
        mlp, feat_mean, feat_std, feature_names, metrics = train_on_rows(
            all_rows, args.epochs, args.learning_rate, args.temperature, args.l2,
            args.validation_fraction, args.batch_size,
        )
        print(f"  loss {metrics['initial_loss']:.4f}->{metrics['final_loss']:.4f}  "
              f"val_top1={metrics['validation_ranking']['top1_accuracy']:.3f}  "
              f"val_mrr={metrics['validation_ranking']['mean_reciprocal_rank']:.3f}")

        # 4. Save version
        model_dict = model_to_dict(mlp, feat_mean, feat_std, feature_names, metrics)
        version_path = args.output_dir / f"model-v{iteration}.json"
        version_path.write_text(json.dumps(model_dict, indent=2), encoding="utf-8")

        # 5. Evaluate
        print(f"  evaluating {args.eval_runs} scenarios vs planner...")
        result = evaluate_model(model_dict, EVAL_SEED_BASE, args.eval_runs, args)
        entry = {"iteration": iteration, "epsilon": epsilon, "rows": len(all_rows), **result}
        history.append(entry)
        print(f"  planner={result['avg_planner_time_s']:.1f}s  model={result['avg_model_time_s']:.1f}s  "
              f"dt={result['avg_delta_time_s']:+.1f}s  win={result['win_rate']:.1%}")

        # 6. Update best
        if result["avg_model_time_s"] < best_model_time:
            best_model_time = result["avg_model_time_s"]
            best_path = Path("runtime/next-ball-policy-model.json")
            best_path.write_text(json.dumps(model_dict, indent=2), encoding="utf-8")
            print(f"  *** new best ({best_model_time:.1f}s) saved to {best_path} ***")

        current_model = model_dict

    # Summary table
    print("\n=== Self-play complete ===")
    print(f"{'iter':>4}  {'model_time':>11}  {'dt_s':>8}  {'win_rate':>9}  {'rows':>8}")
    for h in history:
        print(f"{h['iteration']:>4}  {h['avg_model_time_s']:>10.2f}s  "
              f"{h['avg_delta_time_s']:>+8.2f}s  {h['win_rate']:>9.1%}  "
              f"{h.get('rows', 0):>8}")

    (args.output_dir / "self-play-history.json").write_text(
        json.dumps(history, indent=2), encoding="utf-8"
    )


if __name__ == "__main__":
    main()
