"""Pure target-association and measured-range rules for C2 trials."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class AssociationResult:
    kind: str  # target | non_target | ambiguous | unmatched
    ball_id: str | None
    distance_m: float | None


def associate_target_candidate(
    candidate_xyz_m: tuple[float, float, float],
    ground_truth_xyz_m: dict[str, tuple[float, float, float]],
    *,
    target_ball_id: str,
    association_gate_m: float,
    ambiguity_margin_m: float,
) -> AssociationResult:
    if not math.isfinite(association_gate_m) or association_gate_m <= 0.0:
        raise ValueError("association gate must be positive")
    distances = sorted(
        (math.dist(candidate_xyz_m, truth), ball_id)
        for ball_id, truth in ground_truth_xyz_m.items()
    )
    if not distances or distances[0][0] > association_gate_m:
        return AssociationResult("unmatched", None, None)
    nearest_distance, nearest_id = distances[0]
    if len(distances) > 1 and distances[1][0] <= association_gate_m and distances[1][0] - nearest_distance <= ambiguity_margin_m:
        return AssociationResult("ambiguous", None, nearest_distance)
    return AssociationResult("target" if nearest_id == target_ball_id else "non_target", nearest_id, nearest_distance)


def measured_range_in_bin(range_m: float, requested_bin: str) -> bool:
    low, high = (float(value) for value in requested_bin.split("-"))
    return math.isfinite(range_m) and low <= range_m <= high
