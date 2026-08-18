"""Adversarial tests for the bounded anytime pass router.

These geometries are chosen to break the *abstraction*, not to match the
implementation: each one is a case where a cluster-first block router deleted
executable routes (debug log #57).  The invariants they defend are the reason
the router exists, so they are written against behaviour a correct planner must
have rather than against how this one happens to work.
"""

from dataclasses import replace
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"))

from collection_route_fixtures import FAKE_TIME_S, default_configuration  # noqa: E402
from tennis_robot.collection_route_planner_v2 import (  # noqa: E402
    CourtModel,
    PolygonObstacle,
    plan_collection_route,
)
from tennis_robot.collection_route_cost import plan_objective_cost  # noqa: E402
from tennis_robot.collection_route_router import cluster_balls  # noqa: E402
from tennis_robot.collection_route_types import (  # noqa: E402
    BallReasonCode,
    BallStatus,
    PlanningSearchStatus,
    PlanningStatus,
    Point2D,
    Pose2D,
    PositionCovariance2D,
    RouteSegmentType,
    ScanSnapshot,
    SnapshotBall,
    SuccessorBatchPolicy,
)


def polygon(*points):
    return tuple(Point2D(float(x), float(y)) for x, y in points)


def court(*obstacles, extent=20.0):
    return CourtModel(
        polygon((-extent, -extent), (extent, -extent), (extent, extent), (-extent, extent)),
        tuple(obstacles),
    )


def snapshot(configuration, *entries, pose=Pose2D(0.0, 0.0, 0.0)):
    return ScanSnapshot(
        "scan-router", FAKE_TIME_S, "map", pose,
        tuple(
            SnapshotBall(ball_id, Point2D(x, y), 0.95, PositionCovariance2D(1e-6, 0.0, 1e-6))
            for ball_id, x, y in entries
        ),
        configuration,
    )


def plan_for(configuration, *entries, obstacles=(), pose=Pose2D(0.0, 0.0, 0.0)):
    return plan_collection_route(
        snapshot=snapshot(configuration, *entries, pose=pose),
        court=court(*obstacles), configuration=configuration,
    )


def covered_ids(plan):
    return {result.ball_id for result in plan.ball_results if result.status is BallStatus.COVERED}


def reasons(plan):
    return {result.ball_id: result.reason_code for result in plan.ball_results}


def statuses(plan):
    return {result.ball_id: result.status for result in plan.ball_results}


def collecting_segments(plan):
    return [segment for segment in plan.segments if segment.covered_ball_ids]


def total_turn(plan):
    total = 0.0
    for segment in plan.segments:
        points = segment.path.points
        for first, second in zip(points, points[1:]):
            total += abs(
                math.atan2(
                    math.sin(second.pose.yaw_rad - first.pose.yaw_rad),
                    math.cos(second.pose.yaw_rad - first.pose.yaw_rad),
                )
            )
    return total


def with_budget(configuration, expansions):
    return replace(
        configuration,
        global_route_search=replace(
            configuration.global_route_search, max_search_expansions=expansions
        ),
    )


def with_shipped_turn_limits(configuration):
    """The connector arc limits the robot actually runs with.

    The fixture caps each CSC arc at 1.5 rad, which forbids the forward U-turn
    outright; the shipped configuration allows 3.0 (debug log #46).  Tests about
    reaching a ball behind the robot need the real limits or they are testing
    the fixture instead of the planner.
    """
    return replace(
        configuration,
        connector=replace(
            configuration.connector,
            max_connector_arc_angle_rad=3.0,
            max_connector_total_turn_rad=6.0,
        ),
    )


# --- heuristics may not redefine feasibility ------------------------------

def test_collinear_balls_far_apart_are_collected_by_one_straight_sweep():
    # Spread 8 m, far beyond cluster_threshold_m: a block router put each ball in
    # its own block and drove three separate approaches.  What matters is the
    # shape of the drive, not how many segments it is cut into: the collecting
    # part must be one straight run down the line the balls define.
    configuration = default_configuration(maximum_candidate_count=60)
    plan = plan_for(configuration, ("a", 4.0, 6.0), ("b", 8.0, 6.0), ("c", 12.0, 6.0)).plan
    assert covered_ids(plan) == {"a", "b", "c"}
    # Floor: reach the line (~6.7 m), sweep it (1.0 run-in + 8.0 + 0.3 run-out),
    # then the 0.5 m terminal run-out.  Anything approaching each ball on its own
    # terms costs far more than this.
    assert plan.total_length_m <= 17.5
    assert len(collecting_segments(plan)) <= 3
    # No per-ball loops: turning at all is only needed once, to line the robot up
    # with the row.  A route that circles back for each ball spends 2*pi per loop.
    assert total_turn(plan) <= 3.5


def test_a_line_the_balls_share_survives_the_clustering_threshold():
    # The candidate must exist regardless of how far apart the balls are: this is
    # the generation-side half of "clustering is not geometry".
    from tennis_robot.collection_route_planner_v2 import analyze_snapshot
    from tennis_robot.collection_route_shared_pass import generate_shared_passes

    configuration = default_configuration(maximum_candidate_count=60)
    scan = snapshot(configuration, ("a", 4.0, 6.0), ("b", 8.0, 6.0), ("c", 12.0, 6.0))
    feasibility = analyze_snapshot(scan, court(), configuration)
    singles = tuple(
        candidate for item in feasibility for candidate in item.candidates
        if len(candidate.covered_ball_ids) == 1
    )
    shared = generate_shared_passes(
        snapshot=scan, single_ball_candidates=singles, court=court(), configuration=configuration
    )
    spans = {frozenset(candidate.covered_ball_ids) for candidate in shared.candidates}
    assert frozenset({"a", "b", "c"}) in spans


def test_cluster_threshold_cannot_change_which_balls_are_collected():
    base = default_configuration(maximum_candidate_count=60)
    layout = (("a", 4.0, 6.0), ("b", 8.0, 6.0), ("c", 12.0, 6.0), ("d", 5.0, -3.0))
    outcomes = []
    for threshold in (0.5, 2.5, 10.0):
        configuration = replace(
            base,
            cluster_heuristics=replace(base.cluster_heuristics, cluster_threshold_m=threshold),
        )
        outcomes.append(covered_ids(plan_for(configuration, *layout).plan))
    assert outcomes[0] == outcomes[1] == outcomes[2] == {"a", "b", "c", "d"}


def test_macros_are_optional_and_never_reduce_coverage():
    base = default_configuration(maximum_candidate_count=60)
    layout = (("a", 3.0, 2.0), ("b", 4.0, 2.0), ("c", 5.0, 2.0), ("d", 9.0, -2.0))
    without = replace(
        base, cluster_heuristics=replace(base.cluster_heuristics, maximum_macro_chains=0)
    )
    with_macros = replace(
        base, cluster_heuristics=replace(base.cluster_heuristics, maximum_macro_chains=24)
    )
    plain = plan_for(without, *layout)
    accelerated = plan_for(with_macros, *layout)
    assert covered_ids(accelerated.plan) >= covered_ids(plain.plan)
    assert covered_ids(plain.plan) == {"a", "b", "c", "d"}


def test_balls_too_close_to_share_a_pass_get_their_own():
    # 0.3 m apart along the travel direction is below the mechanical spacing, so
    # one pass cannot take both; both must still be collected, which here needs
    # the forward U-turn the shipped turn limits allow.
    configuration = with_shipped_turn_limits(default_configuration(maximum_candidate_count=60))
    plan = plan_for(configuration, ("a", 4.0, 0.0), ("b", 4.3, 0.0)).plan
    assert covered_ids(plan) == {"a", "b"}
    assert len(collecting_segments(plan)) >= 2


def test_interleaving_two_clusters_is_legal():
    # a1 and a2 straddle the b pair: any decomposition that keeps clusters
    # contiguous cannot collect all four.
    configuration = default_configuration(maximum_candidate_count=60)
    plan = plan_for(
        configuration, ("a1", 2.0, 5.0), ("b1", 6.0, 5.0), ("b2", 6.6, 5.0), ("a2", 10.0, 5.0)
    ).plan
    assert covered_ids(plan) == {"a1", "a2", "b1", "b2"}


# --- overlap semantics -----------------------------------------------------

def test_a_ball_is_declared_by_exactly_one_segment():
    configuration = default_configuration(maximum_candidate_count=60)
    plan = plan_for(
        configuration, ("a", 3.0, 0.0), ("b", 4.0, 0.0), ("c", 8.0, 3.0), ("d", 9.0, 3.0)
    ).plan
    declared = [ball for segment in plan.segments for ball in segment.covered_ball_ids]
    assert len(declared) == len(set(declared))
    assert set(declared) == covered_ids(plan)


def test_every_covered_ball_points_at_the_segment_that_took_it():
    configuration = default_configuration(maximum_candidate_count=60)
    plan = plan_for(configuration, ("a", 3.0, 0.0), ("b", 4.0, 0.0), ("c", 7.5, 1.0)).plan
    by_id = {segment.id: segment for segment in plan.segments}
    for result in plan.ball_results:
        if result.status is BallStatus.COVERED:
            assert result.ball_id in by_id[result.pass_id].covered_ball_ids


# --- connector collection --------------------------------------------------

def test_a_ball_on_the_transit_line_is_collected_by_the_connector():
    # 'mid' sits between two groups; the drive between them sweeps it, so no
    # dedicated pass should be spent on it and it must not be left untouched.
    configuration = default_configuration(maximum_candidate_count=60)
    plan = plan_for(
        configuration, ("a", 3.0, 0.0), ("b", 4.0, 0.0), ("mid", 8.0, 0.0),
        ("c", 12.0, 0.0), ("d", 13.0, 0.0),
    ).plan
    assert covered_ids(plan) == {"a", "b", "mid", "c", "d"}
    connectors = [
        segment for segment in plan.segments
        if segment.type is RouteSegmentType.CONNECTOR and segment.covered_ball_ids
    ]
    for segment in connectors:
        # A collecting connector is capture motion and holds the tight gate.
        assert segment.execution_profile.max_heading_error_rad == (
            configuration.planning.default_execution_profile.max_heading_error_rad
        )
        assert tuple(item.ball_id for item in segment.planned_crossings) == segment.covered_ball_ids


def test_connectors_never_sweep_a_ball_that_has_no_pass_of_its_own():
    configuration = default_configuration(maximum_candidate_count=60)
    # 'walled' is inside an obstacle: unreachable, and driving through it is
    # exactly what must not be planned.
    bench = PolygonObstacle("bench", "bench", polygon((7.6, -0.4), (8.4, -0.4), (8.4, 0.4), (7.6, 0.4)))
    plan = plan_for(
        configuration, ("a", 3.0, 0.0), ("b", 4.0, 0.0), ("walled", 8.0, 0.0),
        obstacles=(bench,),
    ).plan
    assert "walled" not in covered_ids(plan)
    assert statuses(plan)["walled"] is BallStatus.UNREACHABLE


# --- the partial-route invariant -------------------------------------------

def test_an_unreachable_ball_does_not_erase_the_reachable_ones():
    configuration = default_configuration(maximum_candidate_count=60)
    pocket = PolygonObstacle("pocket", "bench", polygon((6.0, 6.0), (7.0, 6.0), (7.0, 7.0), (6.0, 7.0)))
    plan = plan_for(
        configuration, ("hard", 6.5, 6.5), ("x", 3.0, 0.0), ("y", 4.0, 0.0), obstacles=(pocket,)
    ).plan
    assert plan.planning_status is PlanningStatus.PARTIAL
    assert covered_ids(plan) == {"x", "y"}
    assert reasons(plan)["hard"] is BallReasonCode.KEEPOUT


def test_a_tight_ring_returns_the_best_partial_never_nothing():
    # Six balls on a 1 m circle: the intra-cluster connectors mostly fail, which
    # is exactly the case where making the cluster atomic lost all six.
    configuration = default_configuration(maximum_candidate_count=60)
    ring = tuple(
        ("b%d" % index, 4.0 + math.cos(index * 1.04), 6.0 + math.sin(index * 1.04))
        for index in range(6)
    )
    plan = plan_for(configuration, *ring).plan
    assert plan.is_executable
    assert plan.segments
    assert covered_ids(plan)


@pytest.mark.parametrize("cap", [1, 2, 3, 5, 10])
def test_a_starved_candidate_cap_never_produces_a_zero_route(cap):
    configuration = default_configuration(maximum_candidate_count=cap)
    plan = plan_for(configuration, ("a", 3.0, 0.0), ("b", 8.0, 3.0), ("c", -1.0, -5.0)).plan
    assert plan.is_executable
    assert plan.segments
    assert covered_ids(plan)
    # A ball the cap trimmed is unresolved search, never a geometric verdict.
    for result in plan.ball_results:
        if result.status is not BallStatus.COVERED:
            assert result.reason_code is BallReasonCode.PLANNING_BUDGET


def test_a_candidate_less_ball_does_not_poison_its_neighbours():
    configuration = with_shipped_turn_limits(default_configuration(maximum_candidate_count=60))
    bench = PolygonObstacle("bench", "bench", polygon((3.6, 1.6), (4.4, 1.6), (4.4, 2.4), (3.6, 2.4)))
    plan = plan_for(
        configuration, ("blocked", 4.0, 2.0), ("n1", 9.0, 2.0), ("n2", 10.0, 2.0),
        obstacles=(bench,),
    ).plan
    assert {"n1", "n2"} <= covered_ids(plan)
    assert statuses(plan)["blocked"] is BallStatus.UNREACHABLE


# --- budget monotonicity ---------------------------------------------------

_LADDER = (1, 2, 5, 10, 50, 1000000)

_LAYOUTS = {
    "tight cluster": (("a", 3.0, 1.0), ("b", 4.0, 1.0), ("c", 5.0, 1.2), ("d", 4.5, 2.5)),
    "scattered": (("a", 3.0, 0.0), ("b", 8.0, 3.0), ("c", -1.0, -5.0), ("d", 6.0, -6.0)),
    "mixed": (("a", 3.0, 0.0), ("b", 4.0, 0.0), ("mid", 8.0, 0.0), ("c", 12.0, 0.0), ("d", 13.0, 0.4)),
}

_LAYOUTS_FOR_ATTRIBUTION = dict(
    _LAYOUTS,
    **{"behind the robot": (("a", 3.0, 0.0), ("back", -2.0, 0.0))},
)


@pytest.mark.parametrize("name", sorted(_LAYOUTS))
def test_more_budget_is_never_worse(name):
    # The guarantee is over the objective, not over route length: a longer route
    # that turns less can be genuinely cheaper, and the search is entitled to
    # prefer it.  Comparing length here would assert something the planner never
    # promised.
    base = default_configuration(maximum_candidate_count=40)
    previous = None
    for expansions in _LADDER:
        configuration = with_budget(base, expansions)
        plan = plan_for(configuration, *_LAYOUTS[name]).plan
        current = (len(covered_ids(plan)), plan_objective_cost(plan, configuration))
        if previous is not None:
            assert current[0] >= previous[0], f"coverage fell at {expansions} expansions"
            if current[0] == previous[0]:
                assert current[1] <= previous[1] + 1e-6, f"cost rose at {expansions} expansions"
        previous = current


def test_a_budgeted_run_is_a_prefix_of_the_unbounded_one():
    # Expansion order may not depend on the budget: whatever a large budget does
    # first, a small budget must do identically.
    base = default_configuration(maximum_candidate_count=40)
    layout = _LAYOUTS["mixed"]
    small = plan_for(with_budget(base, 3), *layout)
    assert small.search_expansions == 3
    assert not small.search_complete
    large = plan_for(with_budget(base, 1000000), *layout)
    assert large.search_complete
    assert len(covered_ids(large.plan)) >= len(covered_ids(small.plan))


# --- reason attribution is epistemic ---------------------------------------

def test_budget_exhaustion_never_claims_a_geometric_verdict():
    base = default_configuration(maximum_candidate_count=40)
    plan = plan_for(with_budget(base, 1), *_LAYOUTS["scattered"]).plan
    assert plan.planning_search_status is PlanningSearchStatus.BUDGET_EXHAUSTED
    for result in plan.ball_results:
        if result.status is not BallStatus.COVERED:
            assert result.reason_code is BallReasonCode.PLANNING_BUDGET
            assert result.status is BallStatus.DEFERRED


def test_generation_failures_are_reported_even_under_a_starved_budget():
    # A keepout ball is established without any search, so the budget must not
    # downgrade that verdict to uncertainty.
    base = default_configuration(maximum_candidate_count=40)
    bench = PolygonObstacle("bench", "bench", polygon((5.6, -0.4), (6.4, -0.4), (6.4, 0.4), (5.6, 0.4)))
    plan = plan_for(
        with_budget(base, 1), ("a", 2.0, 0.0), ("walled", 6.0, 0.0), obstacles=(bench,)
    ).plan
    assert reasons(plan)["walled"] is BallReasonCode.KEEPOUT
    assert statuses(plan)["walled"] is BallStatus.UNREACHABLE


def test_no_terminal_anywhere_is_reported_as_such_not_as_budget():
    base = default_configuration(maximum_candidate_count=40)
    configuration = replace(
        base, global_route_search=replace(base.global_route_search, terminal_run_out_m=10.0)
    )
    plan = plan_collection_route(
        snapshot=snapshot(configuration, ("a", 3.0, 0.0)),
        court=court(extent=5.0), configuration=configuration,
    ).plan
    assert not plan.is_executable
    assert plan.planning_status is PlanningStatus.PLANNING_TIMEOUT
    assert reasons(plan)["a"] is BallReasonCode.NO_TERMINAL


def test_a_ball_only_a_u_turn_could_reach_is_a_turn_radius_verdict():
    # Directly behind the robot, closer than the turning circle allows: every
    # connector into every one of its passes fails on turning geometry alone,
    # and an exhaustive search may say so.
    configuration = default_configuration(maximum_candidate_count=40)
    plan = plan_for(configuration, ("back", -2.0, 0.0)).plan
    assert statuses(plan)["back"] is BallStatus.UNREACHABLE
    assert reasons(plan)["back"] is BallReasonCode.TURN_RADIUS
    # The shipped turn limits do allow the manoeuvre, which is what makes the
    # verdict a statement about the configured geometry rather than a guess.
    relaxed = plan_for(with_shipped_turn_limits(configuration), ("back", -2.0, 0.0)).plan
    assert covered_ids(relaxed) == {"back"}


@pytest.mark.parametrize("name", sorted(_LAYOUTS_FOR_ATTRIBUTION))
def test_an_exhaustive_search_never_blames_the_budget(name):
    # The other half of the epistemic rule: once the frontier empties, nothing
    # is unresolved, so no ball may be filed under "we stopped looking".
    configuration = default_configuration(maximum_candidate_count=200)
    result = plan_for(configuration, *_LAYOUTS_FOR_ATTRIBUTION[name])
    if not result.search_complete:
        pytest.skip("layout did not exhaust its frontier within the budget")
    assert result.plan.planning_search_status is PlanningSearchStatus.COMPLETE
    for item in result.plan.ball_results:
        if item.status is not BallStatus.COVERED:
            assert item.reason_code is not BallReasonCode.PLANNING_BUDGET


# --- quality and determinism -----------------------------------------------

def test_clustered_layout_beats_collecting_the_same_balls_one_by_one():
    configuration = default_configuration(maximum_candidate_count=80)
    grouped = plan_for(
        configuration, ("a", 3.0, 2.0), ("b", 4.0, 2.0), ("c", 5.0, 2.0),
        ("d", 10.0, -2.0), ("e", 11.0, -2.0), ("f", 12.0, -2.0),
    ).plan
    assert covered_ids(grouped) == {"a", "b", "c", "d", "e", "f"}
    # Two straight sweeps and one transition, not six individual approaches.
    assert len(collecting_segments(grouped)) <= 3
    assert total_turn(grouped) < 8.0


def test_planning_is_deterministic():
    configuration = default_configuration(maximum_candidate_count=40)
    first = plan_for(configuration, *_LAYOUTS["mixed"])
    second = plan_for(configuration, *_LAYOUTS["mixed"])
    assert first.plan.to_dict() == second.plan.to_dict()
    assert first.search_expansions == second.search_expansions


def test_clustering_is_a_hint_with_a_bounded_group_count():
    configuration = default_configuration()
    balls = snapshot(
        configuration, *[("b%d" % index, float(index) * 3.0, 0.0) for index in range(12)]
    ).balls
    clusters = cluster_balls(balls, configuration)
    assert len(clusters) <= configuration.cluster_heuristics.maximum_clusters
    assert {ball_id for cluster in clusters for ball_id in cluster} == {ball.ball_id for ball in balls}


# --- lazy successor expansion ----------------------------------------------

def with_batch(configuration, size):
    """Pin the pacing, so a test measures the router rather than the policy."""
    return replace(
        configuration,
        global_route_search=replace(
            configuration.global_route_search,
            successor_batch_size=size,
            successor_batch_policy=SuccessorBatchPolicy.FIXED,
        ),
    )


@pytest.mark.parametrize("name", sorted(_LAYOUTS))
def test_batch_size_changes_pacing_not_the_exhausted_result(name):
    # Expanding a state a few successors at a time must not change what an
    # exhaustive search concludes: a visited state keeps the successors it has
    # not reached yet, so the enumeration is the same either way.
    # A small batch needs many more visits to exhaust the same frontier, so the
    # expansion budget must not be what ends the run here.
    base = with_budget(default_configuration(maximum_candidate_count=40), 10 ** 6)
    outcomes = []
    for size in (1, 3, 8, 10 ** 6):
        result = plan_for(with_batch(base, size), *_LAYOUTS[name])
        if not result.search_complete:
            pytest.skip("layout did not exhaust its frontier")
        outcomes.append((
            frozenset(covered_ids(result.plan)), round(result.plan.total_length_m, 6)
        ))
    assert len(set(outcomes)) == 1, outcomes


def test_a_small_batch_still_returns_a_route_on_the_first_expansion():
    base = default_configuration(maximum_candidate_count=40)
    configuration = with_budget(with_batch(base, 1), 1)
    plan = plan_for(configuration, *_LAYOUTS["mixed"]).plan
    assert plan.is_executable
    assert covered_ids(plan)


@pytest.mark.parametrize("size", [1, 3, 8])
def test_monotonic_budget_holds_for_every_batch_size(size):
    base = with_batch(default_configuration(maximum_candidate_count=40), size)
    previous = None
    for expansions in (1, 2, 5, 10, 50, 1000000):
        configuration = with_budget(base, expansions)
        plan = plan_for(configuration, *_LAYOUTS["mixed"]).plan
        current = (len(covered_ids(plan)), plan_objective_cost(plan, configuration))
        if previous is not None:
            assert current[0] >= previous[0]
            if current[0] == previous[0]:
                assert current[1] <= previous[1] + 1e-6
        previous = current


def test_an_unfinished_state_is_reoffered_until_its_successors_run_out():
    # With a batch of one, reaching a four-ball layout needs many more visits
    # than there are states, which only works if states come back.
    base = with_batch(with_budget(default_configuration(maximum_candidate_count=40), 10 ** 6), 1)
    result = plan_for(base, *_LAYOUTS["tight cluster"])
    assert result.search_complete
    assert covered_ids(result.plan)
    # Exhausting the same frontier costs far more *visits* with a batch of one,
    # while costing the same number of successor evaluations -- which is exactly
    # what resumable expansion means, and why the budget counts evaluations.
    wide = plan_for(with_batch(base, 10 ** 6), *_LAYOUTS["tight cluster"])
    assert result.state_pops > wide.state_pops
    assert result.state_resumptions > wide.state_resumptions
    assert result.search_expansions == wide.search_expansions
    assert covered_ids(result.plan) == covered_ids(wide.plan)
