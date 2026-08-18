"""Tests for the pure, bounded adaptive-approach generator (shadow / offline)."""

from dataclasses import replace
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"))

from collection_route_fixtures import default_configuration
from tennis_robot.collection_adaptive_approach import (
    AdaptiveApproachCandidate,
    AdaptiveApproachConfiguration,
    AdaptiveApproachError,
    AdaptiveApproachResult,
    baseline_shadow_solve,
    generate_adaptive_candidates,
    generate_adaptive_shared_passes,
    pareto_filter,
    shadow_global_solve,
)
from tennis_robot.collection_capture_geometry import (
    PlaneProvenance,
    repo_base_footprint_capture_geometry,
)
from tennis_robot.collection_route_planner_v2 import (
    CourtModel,
    FunnelPassCandidate,
    PolygonObstacle,
    plan_collection_route,
)
from tennis_robot.collection_route_types import (
    BallReasonCode,
    BallStatus,
    Point2D,
    Pose2D,
    PositionCovariance2D,
    ScanSnapshot,
    SnapshotBall,
)

LOW_COV = PositionCovariance2D(1e-6, 0.0, 1e-6)


def polygon(*points):
    return tuple(Point2D(float(x), float(y)) for x, y in points)


def court(*obstacles, extent=20.0):
    return CourtModel(
        polygon((-extent, -extent), (extent, -extent), (extent, extent), (-extent, extent)),
        tuple(obstacles),
    )


def snapshot(configuration, *entries, cov=LOW_COV, start=Pose2D(0.0, 0.0, 0.0)):
    return ScanSnapshot(
        "scan-adaptive", 1000.0, "map", start,
        tuple(SnapshotBall(ball_id, Point2D(x, y), 0.95, cov) for ball_id, x, y in entries),
        configuration,
    )


def capture_geometry(pre_contact=0.0, provenance=PlaneProvenance.CONFIGURED):
    return repo_base_footprint_capture_geometry(
        required_pre_contact_straight_m=pre_contact,
        required_pre_contact_provenance=provenance,
    )


def uncalibrated_geometry():
    return repo_base_footprint_capture_geometry(
        required_pre_contact_straight_m=0.0,
        required_pre_contact_provenance=PlaneProvenance.UNCALIBRATED,
    )


def generate(config, snap, crt, adaptive, geom=None):
    return generate_adaptive_candidates(
        snapshot=snap, court=crt, configuration=config,
        capture_geometry=geom or capture_geometry(), adaptive=adaptive,
    )


# ── Test 3: baseline candidate is always present ─────────────────────────────
def test_every_reachable_ball_retains_its_baseline():
    config = default_configuration(maximum_candidate_count=200)
    snap = snapshot(config, ("a", 3.0, 0.0), ("b", 5.0, 1.0))
    adaptive = AdaptiveApproachConfiguration((1.4, 1.9), (0.0, 0.02, -0.02), 8, 64)
    result = generate(config, snap, court(), adaptive)
    for per_ball in result.per_ball:
        assert per_ball.candidates, per_ball.ball_id
        assert any(c.is_baseline for c in per_ball.candidates)
        # There is exactly one baseline per valid heading and no duplicate.
        baseline_headings = [c.heading_rad for c in per_ball.candidates if c.is_baseline]
        assert len(baseline_headings) == len(set(baseline_headings))


def test_no_adaptive_extras_yields_only_baselines():
    config = default_configuration(maximum_candidate_count=200)
    snap = snapshot(config, ("a", 3.0, 0.0))
    # Only the zero lateral offset and a gate that is culled below corridor is
    # impossible; use a single gate but zero offsets so nothing new is created.
    adaptive = AdaptiveApproachConfiguration((1.9,), (0.0,), 8, 64)
    geom = capture_geometry(pre_contact=5.0)  # corridor 5.876 > every gate -> culls extras
    result = generate(config, snap, court(), adaptive, geom)
    for per_ball in result.per_ball:
        assert all(c.is_baseline for c in per_ball.candidates)


# ── Test 5: lateral offset never exceeds the effective capture corridor ───────
def test_lateral_offset_bounded_by_effective_capture_half_width():
    config = default_configuration(maximum_candidate_count=200)
    snap = snapshot(config, ("a", 3.0, 0.0))
    # Effective half-width is ~0.045 m here; 0.02 fits, 0.20 must never appear.
    adaptive = AdaptiveApproachConfiguration((1.4,), (0.0, 0.02, -0.02, 0.20, -0.20), 20, 200)
    result = generate(config, snap, court(), adaptive)
    for per_ball in result.per_ball:
        for candidate in per_ball.candidates:
            width = candidate.candidate.effective_capture_half_width_m
            assert abs(candidate.lateral_offset_m) <= width + 1e-9
            assert abs(candidate.lateral_offset_m) != 0.20


# ── Test 6: the final approach is straight and tangent/aligned ────────────────
def test_final_approach_is_straight_and_heading_aligned():
    config = default_configuration(maximum_candidate_count=200)
    snap = snapshot(config, ("a", 3.0, 0.0))
    adaptive = AdaptiveApproachConfiguration((1.4, 1.9), (0.0, 0.02), 20, 200)
    result = generate(config, snap, court(), adaptive)
    for per_ball in result.per_ball:
        for wrapped in per_ball.candidates:
            candidate = wrapped.candidate
            heading = candidate.heading_rad
            direction = (math.cos(heading), math.sin(heading))
            entry, crossing, exit_pose = candidate.entry_pose, candidate.crossing, candidate.exit_pose
            # entry -> crossing -> exit are collinear along the heading.
            for a, b in ((entry, crossing), (crossing, exit_pose)):
                dx, dy = b.x_m - a.x_m, b.y_m - a.y_m
                length = math.hypot(dx, dy)
                assert length > 0
                assert abs(dx / length - direction[0]) < 1e-9
                assert abs(dy / length - direction[1]) < 1e-9
            # Entry and exit yaw are exactly the crossing heading (aligned).
            assert entry.yaw_rad == pytest.approx(candidate.exit_pose.yaw_rad)
            # The straight run-in length equals the reported alignment corridor.
            run_in = math.hypot(crossing.x_m - entry.x_m, crossing.y_m - entry.y_m)
            assert run_in == pytest.approx(wrapped.alignment_corridor_m)


def test_adaptive_gate_below_corridor_is_rejected_baseline_kept():
    config = default_configuration(maximum_candidate_count=200)
    snap = snapshot(config, ("a", 3.0, 0.0))
    # Corridor 2.0 m: gate 1.4 is inside it and must be culled; gate 2.4 is fine.
    geom = capture_geometry(pre_contact=2.0 - 0.876)  # corridor = 2.0
    adaptive = AdaptiveApproachConfiguration((1.4, 2.4), (0.0,), 20, 200)
    result = generate(config, snap, court(), adaptive, geom)
    gates = {c.approach_gate_distance_m for pb in result.per_ball for c in pb.candidates}
    assert config.mechanical.minimum_run_in_m in gates  # baseline kept
    assert 1.4 not in gates  # below corridor, culled
    assert 2.4 in gates


# ── Test 7: a collision in the approach corridor rejects only that candidate ──
def test_corridor_collision_rejects_only_the_blocked_candidate():
    config = default_configuration(maximum_candidate_count=200)
    snap = snapshot(config, ("a", 5.0, 0.0))
    # Obstacle inflated to x in [2.15, 3.45]: blocks the heading-0 gate-1.9 entry
    # (x=3.1) but neither the baseline (x=4.0) nor gate-1.4 (x=3.6).
    blocker = PolygonObstacle("box", "bench", polygon((2.65, -0.15), (2.95, -0.15), (2.95, 0.15), (2.65, 0.15)))
    adaptive = AdaptiveApproachConfiguration((1.4, 1.9), (0.0,), 30, 200)
    result = generate(config, snap, court(blocker), adaptive)
    ball = result.per_ball[0]
    assert ball.candidates  # still reachable
    heading0 = [c for c in ball.candidates if abs(c.heading_rad) < 1e-9]
    gates0 = {round(c.approach_gate_distance_m, 6) for c in heading0}
    assert config.mechanical.minimum_run_in_m in gates0  # baseline survives
    assert 1.4 in gates0  # unblocked variant survives
    assert 1.9 not in gates0  # only the blocked variant is dropped
    # gate 1.9 is not globally rejected: other headings still offer it.
    all_gate_19 = [c for c in ball.candidates if round(c.approach_gate_distance_m, 6) == 1.9]
    assert all_gate_19


# ── Test 8: Pareto pruning is deterministic and correct ──────────────────────
def _fake_candidate(cid, *, baseline, conn, plength, gate, clearance, margin, lateral=0.0, heading=0.0):
    dummy = FunnelPassCandidate(
        cid, (cid,), heading, Pose2D(0.0, 0.0, heading), Point2D(1.0, 0.0),
        Pose2D(2.0, 0.0, heading), 0.1, (Point2D(1.0, 0.0),),
    )
    return AdaptiveApproachCandidate(
        candidate=dummy, ball_id=cid, heading_rad=heading, is_baseline=baseline,
        approach_gate_distance_m=gate, lateral_offset_m=lateral, alignment_corridor_m=gate,
        minimum_alignment_corridor_m=0.5, capture_margin_m=margin, minimum_clearance_m=clearance,
        connector_lower_bound_m=conn, pass_length_m=plength,
    )


def test_pareto_filter_drops_dominated_keeps_baseline_and_is_deterministic():
    baseline = _fake_candidate("a", baseline=True, conn=2.0, plength=1.3, gate=1.0, clearance=1.0, margin=0.1)
    better = _fake_candidate("a", baseline=False, conn=1.0, plength=1.3, gate=1.0, clearance=1.0, margin=0.1, lateral=0.01)
    dominated = _fake_candidate("a", baseline=False, conn=3.0, plength=2.0, gate=1.9, clearance=0.5, margin=0.05, lateral=0.02)
    kept, exhausted = pareto_filter((baseline, better, dominated), cap=10)
    kept_ids = [(c.is_baseline, round(c.connector_lower_bound_m, 3)) for c in kept]
    assert (True, 2.0) in kept_ids  # baseline always kept
    assert (False, 1.0) in kept_ids  # non-dominated kept
    assert (False, 3.0) not in kept_ids  # strictly dominated dropped
    assert exhausted is False
    # Deterministic: same input, same output order.
    again, _ = pareto_filter((dominated, better, baseline), cap=10)
    assert [c.sort_key() for c in kept] == [c.sort_key() for c in again]


# ── Test 9: budget exhaustion is never turned into unreachable ────────────────
def test_budget_exhaustion_keeps_ball_reachable():
    config = default_configuration(maximum_candidate_count=200)
    snap = snapshot(config, ("a", 3.0, 0.0))
    # Tight per-heading cap forces trimming of surviving variants.
    adaptive = AdaptiveApproachConfiguration((1.4, 1.9), (0.0, 0.02, -0.02), 1, 200)
    result = generate(config, snap, court(), adaptive)
    assert result.budget_exhausted is True
    for per_ball in result.per_ball:
        assert per_ball.candidates  # still reachable
        assert per_ball.unreachable_reason is None
        assert any(c.is_baseline for c in per_ball.candidates)  # baseline never trimmed


def test_pareto_cap_never_drops_baseline_even_when_baselines_exceed_cap():
    baselines = tuple(
        _fake_candidate(f"b{i}", baseline=True, conn=float(i), plength=1.3, gate=1.0, clearance=1.0, margin=0.1, heading=float(i))
        for i in range(5)
    )
    kept, exhausted = pareto_filter(baselines, cap=2)
    assert len(kept) == 5  # all baselines survive
    assert exhausted is True


# ── Test 10: identical input yields identical output ordering ────────────────
def test_generation_is_deterministic():
    config = default_configuration(maximum_candidate_count=200)
    snap = snapshot(config, ("a", 3.0, 0.0), ("b", 5.0, -1.0))
    adaptive = AdaptiveApproachConfiguration((1.4, 1.9), (0.0, 0.02, -0.02), 6, 40)
    first = generate(config, snap, court(), adaptive)
    second = generate(config, snap, court(), adaptive)
    assert first == second


# ── Test 11: fence / net / obstacle and a ball near the boundary ──────────────
def _net_wall(x=0.0, y_half=6.0):
    return PolygonObstacle("net", "net", polygon((x - 0.02, -y_half), (x + 0.02, -y_half), (x + 0.02, y_half), (x - 0.02, y_half)))


def test_ball_near_net_only_gets_net_parallel_headings():
    config = default_configuration(maximum_candidate_count=200)
    net = _net_wall(x=0.0)
    # Ball on the +x side of the net wall: far enough to clear the footprint
    # keepout (>0.5 m from the wall) but within tangent activation (<=0.75 m).
    snap = snapshot(config, ("a", 0.6, 0.0), start=Pose2D(3.0, -3.0, 0.0))
    result = generate(config, snap, court(net), _adaptive_default())
    ball = result.per_ball[0]
    assert ball.candidates  # reachable, net-parallel
    for candidate in ball.candidates:
        # Net runs along +/-y (heading ~ pi/2); every candidate is parallel.
        error = min(
            abs(_wrap(candidate.heading_rad - math.pi / 2.0)),
            abs(_wrap(candidate.heading_rad + math.pi / 2.0)),
        )
        assert error <= config.feasibility.max_parallel_heading_error_rad + 1e-9


def test_ball_inside_obstacle_is_unreachable_keepout():
    config = default_configuration(maximum_candidate_count=200)
    blocker = PolygonObstacle("bench", "bench", polygon((2.8, -0.2), (3.2, -0.2), (3.2, 0.2), (2.8, 0.2)))
    snap = snapshot(config, ("a", 3.0, 0.0))
    result = generate(config, snap, court(blocker), _adaptive_default())
    ball = result.per_ball[0]
    assert ball.candidates == ()
    assert ball.unreachable_reason is BallReasonCode.KEEPOUT


def _adaptive_default():
    return AdaptiveApproachConfiguration((1.4, 1.9), (0.0, 0.02, -0.02), 8, 64)


def _wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


# ── Test 2 (part): the live planner is byte-identical with the module present ─
def test_baseline_shadow_solve_matches_live_planner_bytes():
    config = default_configuration(maximum_candidate_count=200)
    snap = snapshot(config, ("a", 3.0, 0.0), ("b", 5.0, 0.4))
    crt = court()
    live = plan_collection_route(snapshot=snap, court=crt, configuration=config).plan
    shadow = baseline_shadow_solve(snapshot=snap, court=crt, configuration=config)
    assert shadow.plan.to_dict() == live.to_dict()


def test_shadow_adaptive_never_reduces_coverage_below_baseline():
    config = default_configuration(maximum_candidate_count=200)
    snap = snapshot(config, ("a", 3.0, 0.0), ("b", 5.0, 0.4))
    crt = court()
    baseline = baseline_shadow_solve(snapshot=snap, court=crt, configuration=config)
    adaptive = generate(config, snap, crt, _adaptive_default())
    shadow = shadow_global_solve(snapshot=snap, court=crt, configuration=config, adaptive_result=adaptive)

    def covered(plan):
        return {r.ball_id for r in plan.ball_results if r.status is BallStatus.COVERED}

    assert covered(baseline.plan) <= covered(shadow.plan)


# ── Configuration validation ─────────────────────────────────────────────────
def test_gate_must_exceed_production_baseline():
    config = default_configuration(maximum_candidate_count=200)
    snap = snapshot(config, ("a", 3.0, 0.0))
    # Baseline run-in is 1.0; a gate of 0.9 is not strictly greater.
    adaptive = AdaptiveApproachConfiguration((0.9,), (0.0,), 8, 64)
    with pytest.raises(AdaptiveApproachError):
        generate(config, snap, court(), adaptive)


def test_invalid_adaptive_configuration_rejected():
    with pytest.raises(AdaptiveApproachError):
        AdaptiveApproachConfiguration((1.4, 1.4), (0.0,), 8, 64)  # duplicate gate
    with pytest.raises(AdaptiveApproachError):
        AdaptiveApproachConfiguration((-1.0,), (0.0,), 8, 64)  # negative gate
    with pytest.raises(AdaptiveApproachError):
        AdaptiveApproachConfiguration((1.4,), (0.0,), 0, 64)  # non-positive cap


# ── Fix B: uncalibrated required geometry fails explicitly ────────────────────
def test_generate_raises_on_uncalibrated_geometry_listing_fields():
    config = default_configuration(maximum_candidate_count=200)
    snap = snapshot(config, ("a", 3.0, 0.0))
    with pytest.raises(AdaptiveApproachError) as excinfo:
        generate(config, snap, court(), _adaptive_default(), geom=uncalibrated_geometry())
    assert "required_pre_contact_straight_m" in str(excinfo.value)


def test_shadow_global_solve_rejects_uncalibrated_result():
    config = default_configuration(maximum_candidate_count=200)
    snap = snapshot(config, ("a", 3.0, 0.0))
    crt = court()
    result = generate(config, snap, crt, _adaptive_default())  # calibrated
    # Swap in uncalibrated geometry to prove the belt-and-braces guard fires.
    tainted = replace(result, capture_geometry=uncalibrated_geometry())
    with pytest.raises(AdaptiveApproachError):
        shadow_global_solve(snapshot=snap, court=crt, configuration=config, adaptive_result=tainted)


# ── Fix D: exact candidate-count arithmetic invariants ────────────────────────
def test_candidate_counters_satisfy_exact_invariants():
    config = default_configuration(maximum_candidate_count=200)
    snap = snapshot(config, ("a", 3.0, 0.0), ("b", 5.0, 1.0))
    result = generate(config, snap, court(), AdaptiveApproachConfiguration((1.4, 1.9), (0.0, 0.02, -0.02), 8, 64))
    assert result.raw_candidate_count == result.raw_baseline_candidate_count + result.raw_adaptive_extra_candidate_count
    assert result.pareto_kept_total == result.pareto_kept_baseline_count + result.pareto_kept_adaptive_count
    assert result.pareto_pruned_count == result.raw_candidate_count - result.pareto_kept_total
    # The counters agree with the actual per-ball candidates.
    kept_baseline = sum(1 for pb in result.per_ball for c in pb.candidates if c.is_baseline)
    kept_adaptive = sum(1 for pb in result.per_ball for c in pb.candidates if not c.is_baseline)
    assert result.pareto_kept_baseline_count == kept_baseline
    assert result.pareto_kept_adaptive_count == kept_adaptive


def test_single_free_ball_exact_counts():
    config = default_configuration(maximum_candidate_count=200)
    snap = snapshot(config, ("a", 3.0, 0.0))
    # One free ball has all 16 headings valid, so exactly 16 baselines.
    result = generate(config, snap, court(), AdaptiveApproachConfiguration((1.4,), (0.0,), 20, 200))
    assert result.raw_baseline_candidate_count == 16
    # Every kept baseline is preserved.
    assert result.pareto_kept_baseline_count == 16


# ── Fix A: adaptive shared passes preserve physical ball positions ────────────
def _aligned_snapshot(config, cov=LOW_COV):
    return snapshot(config, ("a", 3.0, 0.0), ("b", 4.0, 0.0), cov=cov, start=Pose2D(0.0, -3.0, 0.0))


def _adaptive_singles(config, snap, crt, adaptive):
    result = generate(config, snap, crt, adaptive)
    return tuple(c.candidate for pb in result.per_ball for c in pb.candidates)


def test_adaptive_shared_pass_keeps_physical_positions_with_offset_centreline():
    config = default_configuration(maximum_candidate_count=200)
    snap = _aligned_snapshot(config)
    crt = court()
    singles = _adaptive_singles(config, snap, crt, AdaptiveApproachConfiguration((1.4,), (0.0, 0.02, -0.02), 20, 200))
    shared = generate_adaptive_shared_passes(single_ball_candidates=singles, court=crt, configuration=config)
    ab = [c for c in shared.candidates if set(c.covered_ball_ids) == {"a", "b"}]
    assert ab
    for candidate in ab:
        # crossing_positions are the PHYSICAL balls (y=0), never the shifted centreline.
        for position in candidate.crossing_positions:
            assert position.y_m == pytest.approx(0.0, abs=1e-9)


def _single_candidate(ball_id, x, y, width, *, heading=0.0):
    """A single-ball FunnelPassCandidate with an explicit effective capture
    half-width (so genuinely asymmetric widths can be exercised — the default
    mechanical config caps the derived width near 0.047 m)."""
    direction = (math.cos(heading), math.sin(heading))
    physical = Point2D(x, y)
    entry = Point2D(x - direction[0], y - direction[1])
    exit_point = Point2D(x + 0.3 * direction[0], y + 0.3 * direction[1])
    return FunnelPassCandidate(
        ball_id, (ball_id,), heading,
        Pose2D(entry.x_m, entry.y_m, heading), physical,
        Pose2D(exit_point.x_m, exit_point.y_m, heading), width, (physical,),
    )


def test_adaptive_shared_pass_asymmetric_widths_feasible():
    # Widths 0.02 and 0.05; physical lateral separation 0.06 m.  The margin-
    # maximising centreline (feasibility-interval centre) lies in [0.01, 0.02],
    # away from the physical midpoint (0.03), so a feasible shared pass exists.
    config = default_configuration(maximum_candidate_count=200)
    crt = court()
    singles = (
        _single_candidate("a", 3.0, 0.00, 0.02),
        _single_candidate("b", 4.0, 0.06, 0.05),  # distinct longitudinal, +0.06 lateral
    )
    shared = generate_adaptive_shared_passes(single_ball_candidates=singles, court=crt, configuration=config)
    ab = [c for c in shared.candidates if set(c.covered_ball_ids) == {"a", "b"}]
    assert ab, "a feasible asymmetric-width shared pass must be accepted"
    for candidate in ab:
        phys = {(round(p.x_m, 6), round(p.y_m, 6)) for p in candidate.crossing_positions}
        assert phys == {(3.0, 0.0), (4.0, 0.06)}  # both physical positions preserved
        # Each ball is within its OWN effective half-width of the centreline.
        heading = candidate.heading_rad
        normal = (-math.sin(heading), math.cos(heading))
        centre = candidate.crossing.x_m * normal[0] + candidate.crossing.y_m * normal[1]
        widths = {"a": 0.02, "b": 0.05}
        for ball_id, position in zip(candidate.covered_ball_ids, candidate.crossing_positions):
            lateral = position.x_m * normal[0] + position.y_m * normal[1]
            assert abs(lateral - centre) <= widths[ball_id] + 1e-9


def test_adaptive_shared_pass_asymmetric_widths_infeasible():
    # Same widths, physical lateral separation 0.08 m: the feasibility interval
    # is empty (max(low)=0.03 > min(high)=0.02), so no common centreline keeps
    # both within their own capture width.  The pass must be rejected explicitly.
    config = default_configuration(maximum_candidate_count=200)
    crt = court()
    singles = (
        _single_candidate("a", 3.0, 0.00, 0.02),
        _single_candidate("b", 4.0, 0.08, 0.05),
    )
    shared = generate_adaptive_shared_passes(single_ball_candidates=singles, court=crt, configuration=config)
    assert not any(set(c.covered_ball_ids) == {"a", "b"} for c in shared.candidates)
    reasons = {reason for ids, reason in shared.rejections if set(ids) == {"a", "b"}}
    assert reasons == {"lateral_exceeds_capture"}


def test_adaptive_shared_pass_rejects_when_physical_positions_infeasible():
    config = default_configuration(maximum_candidate_count=200)
    # Two balls 0.30 m apart laterally: no single centreline keeps both within
    # their ~0.045 m effective capture half-width, even though each shifted
    # centreline alone would look fine.
    snap = snapshot(
        config, ("a", 3.0, 0.0), ("b", 3.0, 0.30),
        cov=LOW_COV, start=Pose2D(0.0, -3.0, 0.0),
    )
    crt = court()
    singles = _adaptive_singles(config, snap, crt, AdaptiveApproachConfiguration((1.4,), (0.0, 0.02, -0.02), 20, 200))
    shared = generate_adaptive_shared_passes(single_ball_candidates=singles, court=crt, configuration=config)
    # For the heading along which the two balls are laterally 0.30 m apart there
    # is no feasible shared pass; a rejection is recorded, not fabricated geometry.
    assert not any(set(c.covered_ball_ids) == {"a", "b"} and abs(_lateral_span(c)) > 0.2 for c in shared.candidates)
    assert shared.rejections


def _lateral_span(candidate):
    heading = candidate.heading_rad
    normal = (-math.sin(heading), math.cos(heading))
    laterals = [p.x_m * normal[0] + p.y_m * normal[1] for p in candidate.crossing_positions]
    return max(laterals) - min(laterals)


def test_adaptive_shared_pass_is_deterministic():
    config = default_configuration(maximum_candidate_count=200)
    snap = _aligned_snapshot(config)
    crt = court()
    singles = _adaptive_singles(config, snap, crt, AdaptiveApproachConfiguration((1.4,), (0.0, 0.02, -0.02), 20, 200))
    first = generate_adaptive_shared_passes(single_ball_candidates=singles, court=crt, configuration=config)
    second = generate_adaptive_shared_passes(single_ball_candidates=singles, court=crt, configuration=config)
    assert [c.to_dict() if hasattr(c, "to_dict") else c for c in first.candidates] == [
        c.to_dict() if hasattr(c, "to_dict") else c for c in second.candidates
    ]
    assert first.rejections == second.rejections


def test_production_shared_pass_output_is_unchanged():
    # The production generator must be byte-identical to itself (we never touch
    # it), and baseline_shadow_solve keeps using it, so the baseline plan stays
    # byte-identical to the live planner.
    config = default_configuration(maximum_candidate_count=200)
    snap = snapshot(config, ("a", 3.0, 0.0), ("b", 4.0, 0.0))
    crt = court()
    live = plan_collection_route(snapshot=snap, court=crt, configuration=config).plan
    shadow = baseline_shadow_solve(snapshot=snap, court=crt, configuration=config)
    assert shadow.plan.to_dict() == live.to_dict()
    # Exactly one funnel pass covering both balls (the production shared pass).
    passes = [s for s in live.segments if s.type.value == "funnel_pass"]
    assert len(passes) == 1 and set(passes[0].covered_ball_ids) == {"a", "b"}
