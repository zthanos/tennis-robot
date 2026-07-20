"""Pure Phase 3C shared-pass candidate generation."""

from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations
import math

from tennis_robot.collection_route_planner_v2 import CourtModel, FunnelPassCandidate, _segment_is_collision_free
from tennis_robot.collection_route_types import CollectionRouteConfiguration, Point2D, Pose2D


@dataclass(frozen=True)
class SharedPassGenerationResult:
    candidates: tuple[FunnelPassCandidate, ...]
    candidate_budget_exhausted: bool


def generate_shared_passes(*, single_ball_candidates: tuple[FunnelPassCandidate, ...], court: CourtModel, configuration: CollectionRouteConfiguration) -> SharedPassGenerationResult:
    if not isinstance(single_ball_candidates, tuple) or any(not isinstance(candidate, FunnelPassCandidate) for candidate in single_ball_candidates):
        raise ValueError("single_ball_candidates must be FunnelPassCandidate tuple")
    if not isinstance(court, CourtModel) or not isinstance(configuration, CollectionRouteConfiguration):
        raise ValueError("court and configuration are required")
    single = tuple(candidate for candidate in single_ball_candidates if len(candidate.covered_ball_ids) == 1)
    by_heading: dict[float, list[FunnelPassCandidate]] = {}
    for candidate in single:
        by_heading.setdefault(candidate.heading_rad, []).append(candidate)
    generated: list[FunnelPassCandidate] = []
    exhausted = False
    shared = configuration.shared_pass
    for heading in sorted(by_heading):
        group = tuple(sorted(by_heading[heading], key=lambda item: item.ball_id))
        for size in range(2, min(shared.max_shared_pass_balls, len(group)) + 1):
            for members in combinations(group, size):
                candidate = _build_shared_candidate(members, court, configuration)
                if candidate is None:
                    continue
                if len(generated) >= shared.max_shared_pass_candidates:
                    exhausted = True
                    continue
                generated.append(candidate)
    return SharedPassGenerationResult(tuple(generated), exhausted)


def _build_shared_candidate(members, court, configuration):
    if len({member.covered_ball_ids[0] for member in members}) != len(members):
        return None
    heading = members[0].heading_rad
    if any(member.heading_rad != heading for member in members[1:]):
        return None
    direction = (math.cos(heading), math.sin(heading))
    normal = (-direction[1], direction[0])
    ordered = tuple(sorted(members, key=lambda item: _dot(item.crossing, direction)))
    longitudinal = tuple(_dot(member.crossing, direction) for member in ordered)
    if any(next_value - value < configuration.shared_pass.minimum_mechanical_ball_spacing_m for value, next_value in zip(longitudinal, longitudinal[1:])):
        return None
    lateral = tuple(_dot(member.crossing, normal) for member in ordered)
    center_lateral = (min(lateral) + max(lateral)) / 2.0
    if any(abs(value - center_lateral) > member.effective_capture_half_width_m for member, value in zip(ordered, lateral)):
        return None
    first_longitudinal, last_longitudinal = longitudinal[0], longitudinal[-1]
    entry_longitudinal = first_longitudinal - configuration.mechanical.minimum_run_in_m
    exit_longitudinal = last_longitudinal + configuration.mechanical.minimum_run_out_m
    entry = _point(entry_longitudinal, center_lateral, direction, normal)
    crossing = _point(first_longitudinal, center_lateral, direction, normal)
    exit_point = _point(exit_longitudinal, center_lateral, direction, normal)
    if not _segment_is_collision_free(entry, exit_point, court, configuration.feasibility.footprint_clearance_radius_m):
        return None
    covered = tuple(member.covered_ball_ids[0] for member in ordered)
    return FunnelPassCandidate(
        "shared:" + "+".join(covered),
        covered,
        heading,
        Pose2D(entry.x_m, entry.y_m, heading),
        crossing,
        Pose2D(exit_point.x_m, exit_point.y_m, heading),
        min(member.effective_capture_half_width_m for member in ordered),
        tuple(_point(value, center_lateral, direction, normal) for value in longitudinal),
    )


def _dot(point, vector):
    return point.x_m * vector[0] + point.y_m * vector[1]


def _point(longitudinal, lateral, direction, normal):
    return Point2D(longitudinal * direction[0] + lateral * normal[0], longitudinal * direction[1] + lateral * normal[1])
