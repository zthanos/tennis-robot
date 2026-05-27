#!/usr/bin/env python3
"""Train a baseline next-ball selection model from route benchmark data."""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path

import numpy as np


NUMERIC_FEATURES = [
    "phase",
    "step_in_phase",
    "remaining_balls",
    "candidate_rank_by_distance",
    "robot_x_m",
    "robot_y_m",
    "ball_x_m",
    "ball_y_m",
    "direct_distance_m",
    "estimated_route_distance_m",
    "estimated_travel_s",
    "pickup_pose_distance_m",
    "approach_clear",
    "miss_probability",
    "distance_to_net_m",
    "distance_to_nearest_wall_m",
    "distance_to_nearest_obstacle_m",
    "phase_min_x_m",
    "phase_max_x_m",
    "phase_min_y_m",
    "phase_max_y_m",
]
RISK_TYPES = ["normal", "net_wall", "obstacle"]


def load_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def make_matrix(rows: list[dict[str, str]]) -> tuple[np.ndarray, np.ndarray, list[str]]:
    feature_names = NUMERIC_FEATURES + [f"risk_type_{risk}" for risk in RISK_TYPES]
    x = np.zeros((len(rows), len(feature_names)), dtype=np.float64)
    y = np.zeros(len(rows), dtype=np.float64)

    for row_index, row in enumerate(rows):
        for col_index, feature in enumerate(NUMERIC_FEATURES):
            x[row_index, col_index] = float(row[feature])
        risk = row["risk_type"]
        for risk_index, risk_type in enumerate(RISK_TYPES):
            x[row_index, len(NUMERIC_FEATURES) + risk_index] = 1.0 if risk == risk_type else 0.0
        y[row_index] = float(row["selected"])

    return x, y, feature_names


def split_by_seed(rows: list[dict[str, str]], validation_fraction: float) -> tuple[list[dict[str, str]], list[dict[str, str]]]:
    seeds = sorted({int(row["seed"]) for row in rows})
    validation_count = max(1, round(len(seeds) * validation_fraction))
    validation_seeds = set(seeds[-validation_count:])
    train_rows = [row for row in rows if int(row["seed"]) not in validation_seeds]
    validation_rows = [row for row in rows if int(row["seed"]) in validation_seeds]
    return train_rows, validation_rows


def standardize(train_x: np.ndarray, validation_x: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    mean = train_x.mean(axis=0)
    std = train_x.std(axis=0)
    std[std < 1e-9] = 1.0
    return (train_x - mean) / std, (validation_x - mean) / std, mean, std


def sigmoid(z: np.ndarray) -> np.ndarray:
    z = np.clip(z, -40, 40)
    return 1.0 / (1.0 + np.exp(-z))


def train_logistic_regression(
    x: np.ndarray,
    y: np.ndarray,
    learning_rate: float,
    epochs: int,
    l2: float,
) -> tuple[np.ndarray, float, list[float]]:
    weights = np.zeros(x.shape[1], dtype=np.float64)
    bias = 0.0
    positives = max(1.0, float(y.sum()))
    negatives = max(1.0, float(len(y) - y.sum()))
    sample_weights = np.where(y > 0.5, len(y) / (2 * positives), len(y) / (2 * negatives))
    losses: list[float] = []

    for _epoch in range(epochs):
        prediction = sigmoid(x @ weights + bias)
        error = (prediction - y) * sample_weights
        grad_w = (x.T @ error) / len(y) + l2 * weights
        grad_b = float(error.mean())
        weights -= learning_rate * grad_w
        bias -= learning_rate * grad_b

        eps = 1e-9
        loss = -np.mean(sample_weights * (y * np.log(prediction + eps) + (1 - y) * np.log(1 - prediction + eps)))
        loss += 0.5 * l2 * float(weights @ weights)
        losses.append(float(loss))

    return weights, bias, losses


def binary_metrics(y: np.ndarray, scores: np.ndarray) -> dict[str, float]:
    prediction = scores >= 0.5
    actual = y >= 0.5
    tp = int(np.sum(prediction & actual))
    fp = int(np.sum(prediction & ~actual))
    tn = int(np.sum(~prediction & ~actual))
    fn = int(np.sum(~prediction & actual))
    return {
        "accuracy": (tp + tn) / max(1, len(y)),
        "precision": tp / max(1, tp + fp),
        "recall": tp / max(1, tp + fn),
        "positive_rate": float(np.mean(prediction)),
    }


def grouped_top1_accuracy(rows: list[dict[str, str]], scores: np.ndarray) -> dict[str, float]:
    grouped: dict[tuple[int, int], list[tuple[int, float]]] = defaultdict(list)
    for index, row in enumerate(rows):
        grouped[(int(row["seed"]), int(row["decision_id"]))].append((index, float(scores[index])))

    correct = 0
    total = 0
    reciprocal_ranks = []
    for group in grouped.values():
        selected_indices = {index for index, _score in group if rows[index]["selected"] == "1"}
        if not selected_indices:
            continue
        ranked = sorted(group, key=lambda item: item[1], reverse=True)
        total += 1
        if ranked[0][0] in selected_indices:
            correct += 1
        for rank, (index, _score) in enumerate(ranked, start=1):
            if index in selected_indices:
                reciprocal_ranks.append(1.0 / rank)
                break

    return {
        "decisions": total,
        "top1_accuracy": correct / max(1, total),
        "mean_reciprocal_rank": sum(reciprocal_ranks) / max(1, len(reciprocal_ranks)),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Train a baseline next-ball ranking policy.")
    parser.add_argument("--training-csv", type=Path, default=Path("runtime/route-training.csv"))
    parser.add_argument("--model-out", type=Path, default=Path("runtime/next-ball-policy-model.json"))
    parser.add_argument("--metrics-out", type=Path, default=Path("runtime/next-ball-policy-metrics.json"))
    parser.add_argument("--validation-fraction", type=float, default=0.2)
    parser.add_argument("--epochs", type=int, default=450)
    parser.add_argument("--learning-rate", type=float, default=0.08)
    parser.add_argument("--l2", type=float, default=0.001)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rows = load_rows(args.training_csv)
    if not rows:
        raise SystemExit(f"no training rows found in {args.training_csv}")

    train_rows, validation_rows = split_by_seed(rows, args.validation_fraction)
    train_x, train_y, feature_names = make_matrix(train_rows)
    validation_x, validation_y, _ = make_matrix(validation_rows)
    train_x, validation_x, mean, std = standardize(train_x, validation_x)
    weights, bias, losses = train_logistic_regression(train_x, train_y, args.learning_rate, args.epochs, args.l2)

    train_scores = sigmoid(train_x @ weights + bias)
    validation_scores = sigmoid(validation_x @ weights + bias)
    metrics = {
        "rows": len(rows),
        "train_rows": len(train_rows),
        "validation_rows": len(validation_rows),
        "train_binary": binary_metrics(train_y, train_scores),
        "validation_binary": binary_metrics(validation_y, validation_scores),
        "train_ranking": grouped_top1_accuracy(train_rows, train_scores),
        "validation_ranking": grouped_top1_accuracy(validation_rows, validation_scores),
        "initial_loss": losses[0] if losses else math.nan,
        "final_loss": losses[-1] if losses else math.nan,
    }

    model = {
        "model_type": "standardized_logistic_regression",
        "feature_names": feature_names,
        "risk_types": RISK_TYPES,
        "mean": mean.tolist(),
        "std": std.tolist(),
        "weights": weights.tolist(),
        "bias": bias,
        "decision_rule": "score every candidate row and choose the highest score within a decision group",
        "metrics": metrics,
    }

    args.model_out.parent.mkdir(parents=True, exist_ok=True)
    args.metrics_out.parent.mkdir(parents=True, exist_ok=True)
    args.model_out.write_text(json.dumps(model, indent=2), encoding="utf-8")
    args.metrics_out.write_text(json.dumps(metrics, indent=2), encoding="utf-8")
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
