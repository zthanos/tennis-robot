"""The one route cost formula, shared by every router.

Cost is linear in the four accumulators (length, duration, curvature, pass
count), which is what lets a search add them incrementally and still compare
prefixes against whole routes: every extension only adds, so a prefix's cost is
a lower bound on any route extending it.
"""

from __future__ import annotations

from dataclasses import dataclass

from tennis_robot.collection_route_types import CollectionRouteConfiguration


@dataclass(frozen=True)
class RouteAccumulators:
    length_m: float = 0.0
    duration_s: float = 0.0
    curvature_rad: float = 0.0
    pass_count: int = 0

    def plus(self, length_m=0.0, duration_s=0.0, curvature_rad=0.0, pass_count=0) -> "RouteAccumulators":
        return RouteAccumulators(
            self.length_m + length_m,
            self.duration_s + duration_s,
            self.curvature_rad + curvature_rad,
            self.pass_count + pass_count,
        )


def cost_value(length_m: float, duration_s: float, curvature_rad: float, pass_count: int, configuration: CollectionRouteConfiguration) -> float:
    search = configuration.global_route_search
    energy = length_m + search.turn_energy_equivalent_m_per_rad * curvature_rad
    return (
        search.weight_length * length_m
        + search.weight_time * duration_s
        + search.weight_curvature * curvature_rad
        + search.weight_energy * energy
        + search.weight_pass_count * pass_count
    )


def accumulated_cost(totals: RouteAccumulators, configuration: CollectionRouteConfiguration) -> float:
    return cost_value(
        totals.length_m, totals.duration_s, totals.curvature_rad, totals.pass_count, configuration
    )


def plan_objective_cost(plan, configuration: CollectionRouteConfiguration) -> float:
    """Score a finished plan with the same objective the search optimises.

    Route length is *not* the objective -- a route can be centimetres longer and
    genuinely cheaper because it turns less or needs one pass fewer.  Anything
    checking "more search never made it worse" has to compare this, not length.
    """
    import math

    from tennis_robot.collection_route_types import RouteSegmentType

    search = configuration.global_route_search
    crossing = sum(
        segment.progress_end_m - segment.progress_start_m
        for segment in plan.segments
        if segment.type is RouteSegmentType.FUNNEL_PASS
    )
    passes = sum(
        1 for segment in plan.segments if segment.type is RouteSegmentType.FUNNEL_PASS
    )
    turn = 0.0
    for segment in plan.segments:
        points = segment.path.points
        for first, second in zip(points, points[1:]):
            turn += abs(
                math.atan2(
                    math.sin(second.pose.yaw_rad - first.pose.yaw_rad),
                    math.cos(second.pose.yaw_rad - first.pose.yaw_rad),
                )
            )
    length = plan.total_length_m
    duration = (
        crossing / search.crossing_nominal_speed_m_s
        + (length - crossing) / search.connector_nominal_speed_m_s
    )
    return cost_value(length, duration, turn, passes, configuration)
