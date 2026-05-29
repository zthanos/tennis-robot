#!/usr/bin/env python3
"""Train a next-ball selection MLP from outcome-labeled route data.

Uses ListNet ranking loss over quality_advantage scores (counterfactual rollout
outcomes) rather than binary cross-entropy over planner selections.
Falls back to selected=0/1 binary labels if quality_advantage is all zeros.
"""

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
    "balls_within_2m",
    "phase_progress",
    "bearing_cost_rad",
    "cluster_distance_m",
    "normalized_x",
]
RISK_TYPES = ["normal", "net_wall", "obstacle"]


def load_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def make_matrix(rows: list[dict[str, str]]) -> tuple[np.ndarray, np.ndarray, np.ndarray, list[str]]:
    feature_names = NUMERIC_FEATURES + [f"risk_type_{r}" for r in RISK_TYPES]
    x = np.zeros((len(rows), len(feature_names)), dtype=np.float64)
    y_selected = np.zeros(len(rows), dtype=np.float64)
    y_advantage = np.zeros(len(rows), dtype=np.float64)

    for i, row in enumerate(rows):
        for j, feat in enumerate(NUMERIC_FEATURES):
            x[i, j] = float(row.get(feat, 0.0) or 0.0)
        risk = row.get("risk_type", "normal")
        for k, rt in enumerate(RISK_TYPES):
            x[i, len(NUMERIC_FEATURES) + k] = 1.0 if risk == rt else 0.0
        y_selected[i] = float(row.get("selected", 0))
        y_advantage[i] = float(row.get("quality_advantage", 0.0) or 0.0)

    # Use quality_advantage if present; fall back to binary selected
    use_advantage = float(np.abs(y_advantage).max()) > 1e-6
    y = y_advantage if use_advantage else y_selected
    return x, y, y_selected, feature_names


def split_by_seed(rows: list[dict[str, str]], val_frac: float) -> tuple[list[dict[str, str]], list[dict[str, str]]]:
    seeds = sorted({int(row["seed"]) for row in rows})
    n_val = max(1, round(len(seeds) * val_frac))
    val_seeds = set(seeds[-n_val:])
    return [r for r in rows if int(r["seed"]) not in val_seeds], [r for r in rows if int(r["seed"]) in val_seeds]


def standardize(train_x: np.ndarray, val_x: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    mean = train_x.mean(axis=0)
    std = train_x.std(axis=0)
    std[std < 1e-9] = 1.0
    return (train_x - mean) / std, (val_x - mean) / std, mean, std


# ── MLP ───────────────────────────────────────────────────────────────────────

def leaky_relu(z: np.ndarray) -> np.ndarray:
    return np.where(z > 0, z, 0.01 * z)


def leaky_relu_grad(z: np.ndarray) -> np.ndarray:
    return np.where(z > 0, 1.0, 0.01)


class MLP:
    def __init__(self, n_features: int, h1: int = 64, h2: int = 32, seed: int = 42) -> None:
        rng = np.random.RandomState(seed)
        self.W1 = rng.randn(n_features, h1) * math.sqrt(2.0 / n_features)
        self.b1 = np.zeros(h1)
        self.W2 = rng.randn(h1, h2) * math.sqrt(2.0 / h1)
        self.b2 = np.zeros(h2)
        self.W3 = rng.randn(h2, 1) * math.sqrt(2.0 / h2)
        self.b3 = np.zeros(1)

    def forward(self, x: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        z1 = x @ self.W1 + self.b1
        h1 = leaky_relu(z1)
        z2 = h1 @ self.W2 + self.b2
        h2 = leaky_relu(z2)
        logit = (h2 @ self.W3 + self.b3).ravel()
        return z1, h1, z2, h2, logit

    def score(self, x: np.ndarray) -> np.ndarray:
        _, _, _, _, logit = self.forward(x)
        return 1.0 / (1.0 + np.exp(-np.clip(logit, -40, 40)))

    def backward(
        self,
        x: np.ndarray,
        z1: np.ndarray,
        h1: np.ndarray,
        z2: np.ndarray,
        h2: np.ndarray,
        d_logit: np.ndarray,
        l2: float,
    ) -> tuple:
        n = len(x)
        d = d_logit.reshape(-1, 1)
        dW3 = h2.T @ d / n + l2 * self.W3
        db3 = d.mean(axis=0)
        d_h2 = d @ self.W3.T
        d_z2 = d_h2 * leaky_relu_grad(z2)
        dW2 = h1.T @ d_z2 / n + l2 * self.W2
        db2 = d_z2.mean(axis=0)
        d_h1 = d_z2 @ self.W2.T
        d_z1 = d_h1 * leaky_relu_grad(z1)
        dW1 = x.T @ d_z1 / n + l2 * self.W1
        db1 = d_z1.mean(axis=0)
        return dW1, db1, dW2, db2, dW3, db3

    def update(self, grads: tuple, lr: float) -> None:
        dW1, db1, dW2, db2, dW3, db3 = grads
        self.W1 -= lr * dW1
        self.b1 -= lr * db1
        self.W2 -= lr * dW2
        self.b2 -= lr * db2
        self.W3 -= lr * dW3
        self.b3 -= lr * db3

    def to_dict(self) -> dict:
        return {
            "layers": [
                {"weights": self.W1.tolist(), "bias": self.b1.tolist(), "activation": "leaky_relu"},
                {"weights": self.W2.tolist(), "bias": self.b2.tolist(), "activation": "leaky_relu"},
                {"weights": self.W3.tolist(), "bias": self.b3.tolist(), "activation": "sigmoid"},
            ]
        }


# ── ListNet loss ──────────────────────────────────────────────────────────────

def build_groups(rows: list[dict[str, str]]) -> dict[tuple[int, int], list[int]]:
    groups: dict[tuple[int, int], list[int]] = defaultdict(list)
    for i, row in enumerate(rows):
        groups[(int(row["seed"]), int(row["decision_id"]))].append(i)
    return dict(groups)


def listnet_loss_and_grad(
    logits: np.ndarray,
    y: np.ndarray,
    groups: dict[tuple[int, int], list[int]],
    temperature: float = 2.0,
) -> tuple[float, np.ndarray]:
    d_logit = np.zeros_like(logits)
    total_loss = 0.0
    n_groups = len(groups)

    for indices in groups.values():
        idx = np.array(indices)
        scores = logits[idx]
        targets = y[idx] * temperature

        t_exp = np.exp(targets - targets.max())
        p_target = t_exp / t_exp.sum()

        s_exp = np.exp(scores - scores.max())
        p_pred = s_exp / s_exp.sum()

        total_loss += -float(np.sum(p_target * np.log(p_pred + 1e-9)))
        # No /n_groups here — backward() already divides by batch size n
        d_logit[idx] = p_pred - p_target

    return total_loss / n_groups, d_logit


# ── Metrics ───────────────────────────────────────────────────────────────────

def grouped_top1_accuracy(rows: list[dict[str, str]], scores: np.ndarray) -> dict[str, float]:
    grouped: dict[tuple[int, int], list[tuple[int, float]]] = defaultdict(list)
    for i, row in enumerate(rows):
        grouped[(int(row["seed"]), int(row["decision_id"]))].append((i, float(scores[i])))

    correct = total = 0
    reciprocal_ranks: list[float] = []
    for group in grouped.values():
        selected = {i for i, _ in group if rows[i].get("selected") == "1"}
        if not selected:
            continue
        ranked = sorted(group, key=lambda item: item[1], reverse=True)
        total += 1
        if ranked[0][0] in selected:
            correct += 1
        for rank, (i, _) in enumerate(ranked, start=1):
            if i in selected:
                reciprocal_ranks.append(1.0 / rank)
                break

    return {
        "decisions": total,
        "top1_accuracy": correct / max(1, total),
        "mean_reciprocal_rank": sum(reciprocal_ranks) / max(1, len(reciprocal_ranks)),
    }


# ── Training loop ─────────────────────────────────────────────────────────────

def train_mlp(
    x: np.ndarray,
    y: np.ndarray,
    groups: dict[tuple[int, int], list[int]],
    epochs: int,
    lr: float,
    l2: float,
    temperature: float,
    batch_size: int = 0,  # 0 = full-batch; >0 = mini-batch (groups per batch)
) -> tuple[MLP, list[float]]:
    model = MLP(x.shape[1])
    losses: list[float] = []
    group_list = list(groups.items())
    n_groups = len(group_list)
    use_mini = 0 < batch_size < n_groups
    rng_mb = np.random.RandomState(42)

    for epoch in range(epochs):
        if use_mini:
            perm = rng_mb.permutation(n_groups)
            shuffled = [group_list[i] for i in perm]
            epoch_loss = 0.0
            n_batches = 0
            for start in range(0, n_groups, batch_size):
                batch_items = shuffled[start : start + batch_size]
                b_idx: list[int] = []
                b_groups: dict[tuple[int, int], list[int]] = {}
                offset = 0
                for key, g_idx in batch_items:
                    b_groups[key] = list(range(offset, offset + len(g_idx)))
                    b_idx.extend(g_idx)
                    offset += len(g_idx)
                bx, by = x[b_idx], y[b_idx]
                z1, h1, z2, h2, logit = model.forward(bx)
                loss, d_logit = listnet_loss_and_grad(logit, by, b_groups, temperature)
                grads = model.backward(bx, z1, h1, z2, h2, d_logit, l2)
                model.update(grads, lr)
                epoch_loss += loss
                n_batches += 1
            avg_loss = epoch_loss / max(1, n_batches)
        else:
            z1, h1, z2, h2, logit = model.forward(x)
            avg_loss, d_logit = listnet_loss_and_grad(logit, y, groups, temperature)
            grads = model.backward(x, z1, h1, z2, h2, d_logit, l2)
            model.update(grads, lr)

        losses.append(avg_loss)
        if (epoch + 1) % 50 == 0:
            print(f"  epoch {epoch + 1}/{epochs}  loss={avg_loss:.4f}")

    return model, losses


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Train MLP next-ball ranking policy with ListNet loss.")
    parser.add_argument("--training-csv", type=Path, default=Path("runtime/route-outcome-training.csv"))
    parser.add_argument("--model-out", type=Path, default=Path("runtime/next-ball-policy-model.json"))
    parser.add_argument("--metrics-out", type=Path, default=Path("runtime/next-ball-policy-metrics.json"))
    parser.add_argument("--validation-fraction", type=float, default=0.2)
    parser.add_argument("--epochs", type=int, default=300)
    parser.add_argument("--learning-rate", type=float, default=0.005)
    parser.add_argument("--l2", type=float, default=0.001)
    parser.add_argument("--temperature", type=float, default=2.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rows = load_rows(args.training_csv)
    if not rows:
        raise SystemExit(f"no training rows found in {args.training_csv}")

    train_rows, val_rows = split_by_seed(rows, args.validation_fraction)
    train_x, train_y, train_y_sel, feature_names = make_matrix(train_rows)
    val_x, val_y, val_y_sel, _ = make_matrix(val_rows)
    train_x, val_x, feat_mean, feat_std = standardize(train_x, val_x)

    train_groups = build_groups(train_rows)
    print(f"Training: {len(train_rows)} rows, {len(train_groups)} decision groups")
    print(f"Validation: {len(val_rows)} rows")

    model, losses = train_mlp(train_x, train_y, train_groups, args.epochs, args.learning_rate, args.l2, args.temperature)

    train_scores = model.score(train_x)
    val_scores = model.score(val_x)

    metrics = {
        "rows": len(rows),
        "train_rows": len(train_rows),
        "validation_rows": len(val_rows),
        "train_ranking": grouped_top1_accuracy(train_rows, train_scores),
        "validation_ranking": grouped_top1_accuracy(val_rows, val_scores),
        "initial_loss": losses[0] if losses else math.nan,
        "final_loss": losses[-1] if losses else math.nan,
        "label_type": "quality_advantage" if float(np.abs(train_y).max()) > 1e-6 else "selected_binary",
    }

    model_payload = {
        "model_type": "mlp",
        "feature_names": feature_names,
        "risk_types": RISK_TYPES,
        "mean": feat_mean.tolist(),
        "std": feat_std.tolist(),
        "decision_rule": "score every candidate row and choose the highest score within a decision group",
        "metrics": metrics,
        **model.to_dict(),
    }

    args.model_out.parent.mkdir(parents=True, exist_ok=True)
    args.metrics_out.parent.mkdir(parents=True, exist_ok=True)
    args.model_out.write_text(json.dumps(model_payload, indent=2), encoding="utf-8")
    args.metrics_out.write_text(json.dumps(metrics, indent=2), encoding="utf-8")
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
