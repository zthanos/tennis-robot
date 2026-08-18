"""When the router reports COMPLETE, it must have found the actual optimum.

The router's optimality does not come from its priority function -- that only
decides what is found first.  It comes from exhausting the frontier under an
exact dominance rule, so a run that empties its queue has enumerated every route
the candidate set admits.  These tests check that claim the only way it can be
checked: against brute force on instances small enough to enumerate.
"""

from dataclasses import replace
from itertools import permutations
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"))

from collection_route_fixtures import FAKE_TIME_S, default_configuration  # noqa: E402
from tennis_robot.collection_route_connector_graph import link_poses  # noqa: E402
from tennis_robot.collection_route_cost import RouteAccumulators, accumulated_cost  # noqa: E402
from tennis_robot.collection_route_plan_builder import pass_length  # noqa: E402
from tennis_robot.collection_route_planner_v2 import (  # noqa: E402
    CourtModel,
    _bounded_candidates,
    _merge_candidates,
    analyze_snapshot,
    plan_collection_route,
)
from tennis_robot.collection_route_router import solve_route  # noqa: E402
from tennis_robot.collection_route_shared_pass import generate_shared_passes  # noqa: E402
from tennis_robot.collection_route_types import (  # noqa: E402
    BallStatus,
    PlanningSearchStatus,
    Point2D,
    Pose2D,
    PositionCovariance2D,
    ScanSnapshot,
    SnapshotBall,
)

COURT = CourtModel(
    tuple(Point2D(x, y) for x, y in ((-20.0, -20.0), (20.0, -20.0), (20.0, 20.0), (-20.0, 20.0))),
    (),
)


def snapshot(configuration, *entries):
    return ScanSnapshot(
        "scan-optimal", FAKE_TIME_S, "map", Pose2D(0.0, 0.0, 0.0),
        tuple(
            SnapshotBall(ball_id, Point2D(x, y), 0.95, PositionCovariance2D(1e-6, 0.0, 1e-6))
            for ball_id, x, y in entries
        ),
        configuration,
    )


def candidate_set(scan, configuration):
    feasibility = analyze_snapshot(scan, COURT, configuration)
    singles = tuple(
        candidate for item in feasibility for candidate in item.candidates
        if len(candidate.covered_ball_ids) == 1
    )
    shared = generate_shared_passes(
        snapshot=scan, single_ball_candidates=singles, court=COURT, configuration=configuration
    )
    merged = _merge_candidates(singles + shared.candidates)
    bounded, _ = _bounded_candidates(scan, merged, configuration.planning.maximum_candidate_count)
    return feasibility, bounded


def brute_force_best(scan, candidates, configuration):
    """Every route of every length over the candidate set, scored identically.

    Deliberately naive: enumerate ordered selections, link them, close them with
    the terminal run-out, and keep the best by (coverage, cost).  This is the
    definition the router must agree with.
    """
    search = configuration.global_route_search
    eligible = tuple(
        ball for ball in scan.balls
        if any(ball.ball_id in item.covered_ball_ids for item in candidates)
    )

    def link(source, target):
        return link_poses(
            source_id="s", source_pose=source, target_id="t",
            target_pose=candidates[target].entry_pose, court=COURT,
            configuration=configuration, balls=eligible,
        ).edge

    def terminal(pose):
        from tennis_robot.collection_route_planner_v2 import _segment_is_collision_free
        import math

        length = search.terminal_run_out_m
        end = Point2D(
            pose.x_m + length * math.cos(pose.yaw_rad), pose.y_m + length * math.sin(pose.yaw_rad)
        )
        if not _segment_is_collision_free(
            Point2D(pose.x_m, pose.y_m), end, COURT,
            configuration.feasibility.footprint_clearance_radius_m,
        ):
            return None
        return length

    best = None
    indices = range(len(candidates))
    for size in range(1, min(len(candidates), 3) + 1):
        for order in permutations(indices, size):
            pose = scan.robot_pose_at_scan
            covered: set[str] = set()
            totals = RouteAccumulators()
            feasible = True
            for index in order:
                edge = link(pose, index)
                if edge is None:
                    feasible = False
                    break
                candidate = candidates[index]
                gained = (set(edge.swept_ball_ids) | set(candidate.covered_ball_ids)) - covered
                if not gained:
                    feasible = False
                    break
                length = pass_length(candidate)
                declares_pass = bool(
                    {
                        ball_id for ball_id in candidate.covered_ball_ids
                        if ball_id not in covered and ball_id not in edge.swept_ball_ids
                    }
                )
                totals = totals.plus(
                    length_m=edge.path.length_m + length,
                    duration_s=(
                        edge.path.length_m / search.connector_nominal_speed_m_s
                        + length / search.crossing_nominal_speed_m_s
                    ),
                    curvature_rad=edge.path.total_turn_rad,
                    pass_count=1 if declares_pass else 0,
                )
                covered |= gained
                pose = candidate.exit_pose
            if not feasible:
                continue
            run_out = terminal(pose)
            if run_out is None:
                continue
            closed = totals.plus(
                length_m=run_out, duration_s=run_out / search.connector_nominal_speed_m_s
            )
            key = (-len(covered), accumulated_cost(closed, configuration))
            if best is None or key < best:
                best = key
    return best


LAYOUTS = {
    "pair": (("a", 3.0, 0.0), ("b", 6.0, 1.0)),
    "triple": (("a", 3.0, 0.0), ("b", 4.0, 0.0), ("c", 7.0, 2.0)),
    "spread": (("a", 2.5, 1.0), ("b", 6.0, -1.5), ("c", 9.0, 1.0)),
    "with transit ball": (("a", 3.0, 0.0), ("mid", 6.0, 0.0), ("c", 9.0, 0.0)),
}


@pytest.mark.parametrize("name", sorted(LAYOUTS))
def test_complete_search_matches_brute_force(name):
    # A small candidate cap keeps brute force tractable; the router gets exactly
    # the same candidate set, so the comparison is like for like.
    configuration = default_configuration(maximum_candidate_count=6)
    scan = snapshot(configuration, *LAYOUTS[name])
    feasibility, candidates = candidate_set(scan, configuration)
    result = solve_route(
        snapshot=scan, feasibility=feasibility, candidates=candidates,
        court=COURT, configuration=configuration,
    )
    assert result.search_complete, "test instance must exhaust its frontier"

    expected = brute_force_best(scan, candidates, configuration)
    assert expected is not None
    covered = sum(
        1 for item in result.plan.ball_results if item.status is BallStatus.COVERED
    )
    assert -covered == expected[0], "router covered fewer balls than brute force"
    # Cost is compared only at equal coverage, which is what the objective says.
    from tennis_robot.collection_route_types import RouteSegmentType

    length = result.plan.total_length_m
    crossing = sum(
        segment.progress_end_m - segment.progress_start_m
        for segment in result.plan.segments
        if segment.type is RouteSegmentType.FUNNEL_PASS
    )
    search = configuration.global_route_search
    turn = 0.0
    import math

    for segment in result.plan.segments:
        points = segment.path.points
        for first, second in zip(points, points[1:]):
            turn += abs(
                math.atan2(
                    math.sin(second.pose.yaw_rad - first.pose.yaw_rad),
                    math.cos(second.pose.yaw_rad - first.pose.yaw_rad),
                )
            )
    passes = sum(
        1 for segment in result.plan.segments if segment.type is RouteSegmentType.FUNNEL_PASS
    )
    actual = accumulated_cost(
        RouteAccumulators(
            length,
            crossing / search.crossing_nominal_speed_m_s
            + (length - crossing) / search.connector_nominal_speed_m_s,
            turn,
            passes,
        ),
        configuration,
    )
    # The router's own accounting uses arc lengths where the plan stores chord
    # polylines, so allow a small geometric slack rather than exact equality.
    assert actual <= expected[1] * 1.05 + 1e-6


def test_completeness_is_reported_honestly():
    configuration = default_configuration(maximum_candidate_count=6)
    scan = snapshot(configuration, *LAYOUTS["triple"])
    feasibility, candidates = candidate_set(scan, configuration)
    complete = solve_route(
        snapshot=scan, feasibility=feasibility, candidates=candidates,
        court=COURT, configuration=configuration,
    )
    assert complete.search_complete
    assert complete.plan.planning_search_status is PlanningSearchStatus.COMPLETE

    starved = replace(
        configuration,
        global_route_search=replace(configuration.global_route_search, max_search_expansions=1),
    )
    scan = snapshot(starved, *LAYOUTS["triple"])
    feasibility, candidates = candidate_set(scan, starved)
    truncated = solve_route(
        snapshot=scan, feasibility=feasibility, candidates=candidates,
        court=COURT, configuration=starved,
    )
    assert not truncated.search_complete
    assert truncated.plan.planning_search_status is PlanningSearchStatus.BUDGET_EXHAUSTED


def test_a_plan_claims_completeness_only_when_nothing_was_left_unexamined():
    # Two budgets can truncate a run: the frontier and the candidate cap.  The
    # plan may report COMPLETE only when neither did.
    tight = default_configuration(maximum_candidate_count=6)
    capped = plan_collection_route(
        snapshot=snapshot(tight, *LAYOUTS["pair"]), court=COURT, configuration=tight
    )
    assert capped.search_complete  # the frontier did empty
    assert capped.plan.planning_search_status is PlanningSearchStatus.BUDGET_EXHAUSTED
    assert capped.search_expansions > 0

    generous = default_configuration(maximum_candidate_count=400)
    full = plan_collection_route(
        snapshot=snapshot(generous, *LAYOUTS["pair"]), court=COURT, configuration=generous
    )
    assert full.search_complete
    assert full.plan.planning_search_status is PlanningSearchStatus.COMPLETE


@pytest.mark.parametrize("name", sorted(LAYOUTS))
@pytest.mark.parametrize("batch", [1, 3, 10 ** 6])
def test_complete_optimum_is_identical_at_every_batch_size(name, batch):
    # Pacing may change which route is found first; it may not change which
    # route an exhausted search concludes with.
    configuration = default_configuration(maximum_candidate_count=6)
    configuration = replace(
        configuration,
        global_route_search=replace(
            configuration.global_route_search,
            successor_batch_size=batch,
            max_search_expansions=10 ** 6,
        ),
    )
    scan = snapshot(configuration, *LAYOUTS[name])
    feasibility, candidates = candidate_set(scan, configuration)
    result = solve_route(
        snapshot=scan, feasibility=feasibility, candidates=candidates,
        court=COURT, configuration=configuration,
    )
    assert result.search_complete
    expected = brute_force_best(scan, candidates, configuration)
    covered = sum(
        1 for item in result.plan.ball_results if item.status is BallStatus.COVERED
    )
    assert -covered == expected[0]
    assert result.plan.plan_id == _reference_plan_id(name)


_REFERENCE_PLANS: dict[str, str] = {}


def _reference_plan_id(name):
    """First COMPLETE plan seen for a layout; every batch size must match it."""
    if name not in _REFERENCE_PLANS:
        configuration = default_configuration(maximum_candidate_count=6)
        configuration = replace(
            configuration,
            global_route_search=replace(
                configuration.global_route_search, max_search_expansions=10 ** 6
            ),
        )
        scan = snapshot(configuration, *LAYOUTS[name])
        feasibility, candidates = candidate_set(scan, configuration)
        _REFERENCE_PLANS[name] = solve_route(
            snapshot=scan, feasibility=feasibility, candidates=candidates,
            court=COURT, configuration=configuration,
        ).plan.plan_id
    return _REFERENCE_PLANS[name]


def test_adaptive_batching_matches_every_fixed_batch_at_complete():
    # The adaptive policy is pacing: at COMPLETE it must land on exactly the
    # route every fixed pacing lands on, or it is quietly a search policy.
    from tennis_robot.collection_route_router import successor_batch_size
    from tennis_robot.collection_route_types import SuccessorBatchPolicy

    for name in sorted(LAYOUTS):
        base = default_configuration(maximum_candidate_count=6)
        base = replace(
            base,
            global_route_search=replace(
                base.global_route_search, max_search_expansions=10 ** 6
            ),
        )
        plans = {}
        for label, search in (
            ("adaptive", replace(
                base.global_route_search,
                successor_batch_policy=SuccessorBatchPolicy.ADAPTIVE,
            )),
            *(
                (f"fixed-{size}", replace(
                    base.global_route_search,
                    successor_batch_policy=SuccessorBatchPolicy.FIXED,
                    successor_batch_size=size,
                ))
                for size in (1, 4, 16, 64, 10 ** 6)
            ),
        ):
            configuration = replace(base, global_route_search=search)
            scan = snapshot(configuration, *LAYOUTS[name])
            feasibility, candidates = candidate_set(scan, configuration)
            result = solve_route(
                snapshot=scan, feasibility=feasibility, candidates=candidates,
                court=COURT, configuration=configuration,
            )
            assert result.search_complete, f"{name}/{label} did not exhaust its frontier"
            plans[label] = result.plan.plan_id
        assert len(set(plans.values())) == 1, (name, plans)


def test_the_adaptive_rule_depends_only_on_candidate_count():
    from tennis_robot.collection_route_router import successor_batch_size

    configuration = default_configuration()
    sizes = [successor_batch_size(configuration, count) for count in range(0, 400, 10)]
    # Deterministic, bounded and never decreasing as the problem grows.
    assert sizes == sorted(sizes)
    assert min(sizes) >= 1
    assert successor_batch_size(configuration, 10) == successor_batch_size(configuration, 10)


def test_fixed_policy_is_honoured_verbatim():
    from tennis_robot.collection_route_router import successor_batch_size
    from tennis_robot.collection_route_types import SuccessorBatchPolicy

    base = default_configuration()
    pinned = replace(
        base,
        global_route_search=replace(
            base.global_route_search,
            successor_batch_policy=SuccessorBatchPolicy.FIXED,
            successor_batch_size=7,
        ),
    )
    assert successor_batch_size(pinned, 5) == 7
    assert successor_batch_size(pinned, 5000) == 7
