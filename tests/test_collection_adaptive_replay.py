"""Tests for the offline adaptive-replay CLI (scripts/sim_debug)."""

import importlib.util
import json
import os
import sys
from types import SimpleNamespace

import pytest

_ROOT = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, os.path.join(_ROOT, "ros2_ws", "src", "tennis_robot"))

from collection_route_fixtures import default_configuration
from tennis_robot.collection_route_planner_v2 import CourtModel, PolygonObstacle, plan_collection_route
from tennis_robot.collection_route_types import (
    Point2D, Pose2D, PositionCovariance2D, ScanSnapshot, SnapshotBall,
)


def _load_cli():
    path = os.path.join(_ROOT, "scripts", "sim_debug", "collection_route_adaptive_replay.py")
    spec = importlib.util.spec_from_file_location("collection_route_adaptive_replay", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


CLI = _load_cli()


def _snapshot():
    config = default_configuration(maximum_candidate_count=200)
    # Start on the +x side of the net (not on the net line at x=0).
    return ScanSnapshot(
        "scan-replay", 1000.0, "map", Pose2D(2.0, -2.0, 0.0),
        (
            SnapshotBall("a", Point2D(3.0, 0.0), 0.95, PositionCovariance2D(1e-6, 0.0, 1e-6)),
            SnapshotBall("b", Point2D(5.0, 0.6), 0.95, PositionCovariance2D(1e-6, 0.0, 1e-6)),
        ),
        config,
    )


def _court_boundary():
    return {
        "schema": "court_knowledge_model/v2",
        "frame": "map",
        "status": "OK",
        "completed": True,
        "fence": {"corners": [
            {"x_m": -20.0, "y_m": -20.0}, {"x_m": 20.0, "y_m": -20.0},
            {"x_m": 20.0, "y_m": 20.0}, {"x_m": -20.0, "y_m": 20.0},
        ]},
        "net": {
            "center": {"x_m": 0.0, "y_m": 0.0},
            "posts": [{"x_m": 0.0, "y_m": -6.0}, {"x_m": 0.0, "y_m": 6.0}],
        },
        "obstacles": [],
    }


def _artifact(snapshot):
    boundary = _court_boundary()
    from tennis_robot.collection_court_model_builder import build_court_model
    court = build_court_model(boundary)
    plan = plan_collection_route(snapshot=snapshot, court=court, configuration=snapshot.configuration_snapshot).plan
    return {"schema_version": 1, "snapshot": snapshot.to_dict(), "plan": plan.to_dict()}, boundary


def _adaptive():
    from tennis_robot.collection_adaptive_approach import AdaptiveApproachConfiguration
    return AdaptiveApproachConfiguration((1.4, 1.9), (0.0, 0.02, -0.02), 6, 64)


CONFIGURED = None  # set below once CLI is imported


def _configured():
    return CLI.PlaneProvenance.CONFIGURED


def _comparison_inputs(
    *,
    baseline_covered=1,
    adaptive_covered=1,
    baseline_length_m=10.0,
    adaptive_length_m=10.0,
    candidate_budget_exhausted=False,
    baseline_global_cap_exhausted=False,
    adaptive_global_cap_exhausted=False,
    shared_pass_budget_exhausted=False,
    baseline_combined_status="complete",
    adaptive_combined_status="complete",
):
    def plan(covered, length_m):
        return SimpleNamespace(
            ball_results=[
                SimpleNamespace(status=CLI.BallStatus.COVERED) for _ in range(covered)
            ],
            total_length_m=length_m,
        )

    baseline = SimpleNamespace(
        plan=plan(baseline_covered, baseline_length_m),
        global_candidate_cap_exhausted=baseline_global_cap_exhausted,
        combined_planner_budget_status=baseline_combined_status,
    )
    adaptive_result = SimpleNamespace(budget_exhausted=candidate_budget_exhausted)
    shadow = SimpleNamespace(
        plan=plan(adaptive_covered, adaptive_length_m),
        shared_pass_budget_exhausted=shared_pass_budget_exhausted,
        global_candidate_cap_exhausted=adaptive_global_cap_exhausted,
        combined_planner_budget_status=adaptive_combined_status,
    )
    return baseline, adaptive_result, shadow


def _comparison(**kwargs):
    return CLI._build_comparison(*_comparison_inputs(**kwargs))


def _build(artifact, boundary, *, run_shadow_solve=True, baseline_only=False, provenance=None, pre_contact=0.0):
    return CLI.build_report(
        artifact=artifact, boundary=boundary, adaptive=_adaptive(),
        capture_reference_plane_id="intake_mouth_contact",
        required_pre_contact_straight_m=pre_contact,
        required_pre_contact_provenance=provenance or _configured(),
        run_shadow_solve=run_shadow_solve, baseline_only=baseline_only,
    )


def test_report_baseline_matches_embedded_plan_bytes():
    snapshot = _snapshot()
    artifact, boundary = _artifact(snapshot)
    report = _build(artifact, boundary)
    assert report["baseline"]["matches_embedded_plan"] is True


def test_build_report_raises_on_uncalibrated_adaptive():
    snapshot = _snapshot()
    artifact, boundary = _artifact(snapshot)
    from tennis_robot.collection_adaptive_approach import AdaptiveApproachError
    with pytest.raises(AdaptiveApproachError):
        _build(artifact, boundary, provenance=CLI.PlaneProvenance.UNCALIBRATED)


def test_report_contains_all_required_fields():
    snapshot = _snapshot()
    artifact, boundary = _artifact(snapshot)
    report = _build(artifact, boundary)
    assert report["mode"] == "adaptive"
    assert report["snapshot"]["ball_count"] == 2
    baseline = report["baseline"]
    assert set(baseline["coverage"]) == {"covered", "deferred", "unreachable", "snapshot_balls"}
    assert "connector_edge_rejection_histogram" in baseline
    assert baseline["pass_count"] >= 1
    assert {"balls_per_shared_pass", "expected_duration_s", "combined_planner_budget_status"} <= set(baseline)
    assert {"global_candidate_cap_exhausted", "shared_pass_budget_exhausted"} <= set(baseline)
    assert baseline["search_exhaustion_independently_known"] is False
    assert "search_status" not in baseline  # honest rename; not an independent signal
    assert {"total_length_m", "connector_length_m", "pass_length_m", "terminal_length_m"} <= set(baseline)

    adaptive = report["adaptive"]
    # Exact unambiguous candidate-count invariants.
    assert adaptive["raw_candidate_count"] == (
        adaptive["raw_baseline_candidate_count"] + adaptive["raw_adaptive_extra_candidate_count"]
    )
    assert adaptive["pareto_kept_total"] == (
        adaptive["pareto_kept_baseline_count"] + adaptive["pareto_kept_adaptive_count"]
    )
    assert adaptive["pareto_pruned_count"] == adaptive["raw_candidate_count"] - adaptive["pareto_kept_total"]
    assert "candidate_generation_budget_exhausted" in adaptive
    assert len(adaptive["per_ball"]) == 2
    for entry in adaptive["per_ball"]:
        assert {"ball_id", "pareto_kept_candidates", "pareto_kept_baseline", "pareto_kept_adaptive"} <= set(entry)

    shadow = report["shadow_global_solve"]
    assert "planning_wall_ms" in shadow
    assert "connector_edge_rejection_histogram" in shadow
    assert shadow["search_expansions"] is None  # honest: not surfaced by the solver
    assert shadow["search_exhaustion_independently_known"] is False
    assert "search_status" not in shadow  # renamed to the honest combined status
    assert {"global_candidate_cap_exhausted", "shared_pass_budget_exhausted",
            "shared_pass_rejection_count", "combined_planner_budget_status"} <= set(shadow)

    comparison = report["comparison"]
    assert isinstance(comparison["comparison_conclusive"], bool)
    assert comparison["search_exhaustion_independently_known"] is False
    assert "comparison_limitations" in comparison
    assert "comparison_conclusion" in comparison
    # Capture geometry provenance is surfaced (configured, so calibrated here).
    assert report["capture_geometry"]["is_calibrated"] is True


def _cap1_snapshot():
    config = default_configuration(maximum_candidate_count=1)
    return ScanSnapshot(
        "scan-cap1", 1000.0, "map", Pose2D(2.0, -2.0, 0.0),
        (
            SnapshotBall("a", Point2D(3.0, 0.0), 0.95, PositionCovariance2D(1e-6, 0.0, 1e-6)),
            SnapshotBall("b", Point2D(5.0, 0.6), 0.95, PositionCovariance2D(1e-6, 0.0, 1e-6)),
        ),
        config,
    )


def test_candidate_cap_exhaustion_is_not_reported_as_independent_search_exhaustion():
    # A candidate cap of 1 forces global candidate-cap exhaustion.  The combined
    # production planner status will be budget_exhausted, but that must NEVER be
    # surfaced as an independent DFS/search exhaustion.
    artifact, boundary = _artifact(_cap1_snapshot())
    report = _build(artifact, boundary)
    comparison = report["comparison"]
    limitations = comparison["comparison_limitations"]

    assert comparison["comparison_conclusive"] is False
    assert comparison["search_exhaustion_independently_known"] is False
    # The candidate-cap event is attributed to the cap, not to search exhaustion.
    assert "adaptive_global_candidate_cap_exhausted" in limitations
    assert "adaptive_search_budget_exhausted" not in limitations
    assert "baseline_search_budget_exhausted" not in limitations
    # A non-complete combined status is recorded as a combined (non-decomposable)
    # limitation, never as independent DFS search.
    assert any(l.startswith("adaptive_combined_planner_budget_status_") for l in limitations)
    assert "inconclusive" in comparison["comparison_conclusion"].lower()
    assert "comparison_limitations" in comparison["comparison_conclusion"]
    # The shadow report honestly states independent search exhaustion is unknown.
    assert report["shadow_global_solve"]["search_exhaustion_independently_known"] is False


def test_unchanged_observation_with_candidate_and_global_cap_exhaustion():
    comparison = _comparison(
        candidate_budget_exhausted=True,
        adaptive_global_cap_exhausted=True,
    )

    assert comparison["observation"]["observed_result"] == "unchanged"
    assert comparison["comparison_conclusive"] is False
    assert "no improvement was observed" in comparison["comparison_conclusion"].lower()


def test_improved_coverage_with_exhausted_budget_never_says_no_improvement():
    comparison = _comparison(
        adaptive_covered=2,
        candidate_budget_exhausted=True,
    )

    assert comparison["observation"]["coverage_delta"] == 1
    assert comparison["observation"]["observed_result"] == "improved"
    assert comparison["comparison_conclusive"] is False
    assert "improvement was observed" in comparison["comparison_conclusion"].lower()
    assert "no improvement" not in comparison["comparison_conclusion"].lower()


def test_equal_coverage_shorter_route_with_exhausted_budget_is_improved():
    comparison = _comparison(
        adaptive_length_m=9.0,
        candidate_budget_exhausted=True,
    )

    assert comparison["observation"]["coverage_delta"] == 0
    assert comparison["observation"]["length_delta_m"] == -1.0
    assert comparison["observation"]["observed_result"] == "improved"
    assert comparison["comparison_conclusive"] is False


def test_shared_pass_only_limitation_does_not_invent_other_exhaustions():
    comparison = _comparison(shared_pass_budget_exhausted=True)

    assert comparison["comparison_limitations"] == [
        "adaptive_shared_pass_budget_exhausted"
    ]
    assert comparison["comparison_conclusive"] is False
    conclusion = comparison["comparison_conclusion"].lower()
    assert "candidate-generation" not in conclusion
    assert "global-cap" not in conclusion


def test_combined_status_only_limitation_makes_no_independent_search_claim():
    comparison = _comparison(adaptive_combined_status="budget_exhausted")

    assert comparison["comparison_limitations"] == [
        "adaptive_combined_planner_budget_status_budget_exhausted"
    ]
    assert comparison["comparison_conclusive"] is False
    assert comparison["search_exhaustion_independently_known"] is False
    assert "dfs" not in comparison["comparison_conclusion"].lower()
    assert "search exhaustion" not in comparison["comparison_conclusion"].lower()


def test_conclusive_when_no_budget_exhausted():
    # A single free ball with a generous cap exhausts no budget, so the
    # comparison is conclusive.
    config = default_configuration(maximum_candidate_count=200)
    snapshot = ScanSnapshot(
        "scan-free", 1000.0, "map", Pose2D(0.0, 0.0, 0.0),
        (SnapshotBall("a", Point2D(3.0, 0.0), 0.95, PositionCovariance2D(1e-6, 0.0, 1e-6)),),
        config,
    )
    artifact, boundary = _artifact(snapshot)
    report = CLI.build_report(
        artifact=artifact, boundary=boundary,
        adaptive=CLI.AdaptiveApproachConfiguration((1.4,), (0.0,), 20, 200),
        capture_reference_plane_id="intake_mouth_contact",
        required_pre_contact_straight_m=0.0,
        required_pre_contact_provenance=_configured(),
        run_shadow_solve=True,
    )
    comparison = report["comparison"]
    assert comparison["comparison_conclusive"] is True
    assert comparison["comparison_limitations"] == []
    expected_result = CLI._classify_observation(
        baseline_covered=comparison["observation"]["baseline_covered"],
        adaptive_covered=comparison["observation"]["adaptive_covered"],
        baseline_total_length_m=comparison["observation"]["baseline_total_length_m"],
        adaptive_total_length_m=comparison["observation"]["adaptive_total_length_m"],
    )[2]
    assert comparison["observation"]["observed_result"] == expected_result
    assert report["baseline"]["combined_planner_budget_status"] == "complete"
    assert report["shadow_global_solve"]["combined_planner_budget_status"] == "complete"


def test_length_comparison_epsilon_boundary_is_deterministic():
    epsilon = CLI._LENGTH_COMPARISON_EPSILON_M

    at_short_boundary = _comparison(baseline_length_m=0.0, adaptive_length_m=-epsilon)
    at_long_boundary = _comparison(baseline_length_m=0.0, adaptive_length_m=epsilon)
    beyond_short_boundary = _comparison(
        baseline_length_m=0.0, adaptive_length_m=-2.0 * epsilon
    )
    beyond_long_boundary = _comparison(
        baseline_length_m=0.0, adaptive_length_m=2.0 * epsilon
    )

    assert at_short_boundary["observation"]["observed_result"] == "unchanged"
    assert at_long_boundary["observation"]["observed_result"] == "unchanged"
    assert beyond_short_boundary["observation"]["observed_result"] == "improved"
    assert beyond_long_boundary["observation"]["observed_result"] == "worsened"
    assert at_short_boundary["observation"]["length_comparison_epsilon_m"] == epsilon


def test_analysis_inputs_echo_all_algorithm_affecting_parameters():
    artifact, boundary = _artifact(_snapshot())
    report = CLI.build_report(
        artifact=artifact, boundary=boundary,
        adaptive=CLI.AdaptiveApproachConfiguration((1.4, 1.9), (0.0, 0.02, -0.02), 6, 64),
        capture_reference_plane_id="intake_mouth_contact",
        required_pre_contact_straight_m=0.0,
        required_pre_contact_provenance=_configured(),
        run_shadow_solve=True,
    )
    inputs = report["analysis_inputs"]
    assert inputs["additional_gate_distances_m"] == [1.4, 1.9]
    assert inputs["lateral_offsets_m"] == [0.0, 0.02, -0.02]
    assert inputs["max_candidates_per_heading"] == 6
    assert inputs["max_candidates_per_ball"] == 64
    assert inputs["capture_reference_plane_id"] == "intake_mouth_contact"
    assert inputs["required_pre_contact_straight_m"] == 0.0
    assert inputs["required_pre_contact_provenance"] == "configured"
    assert inputs["run_shadow_solve"] is True
    assert inputs["maximum_candidate_count"] == 200
    # The pre-contact assumption is flagged as an offline analysis assumption.
    assert inputs["assumptions"]["required_pre_contact_is_offline_analysis_assumption"] is True
    assert "intake trials" in inputs["assumptions"]["note"]


def test_report_is_deterministic():
    snapshot = _snapshot()
    artifact, boundary = _artifact(snapshot)

    def build():
        report = _build(artifact, boundary)
        for section in ("baseline", "adaptive", "shadow_global_solve"):
            for key in list(report[section]):
                if key.endswith("_wall_ms"):
                    report[section].pop(key)
        return report

    assert build() == build()


def test_cli_main_writes_only_to_requested_output(tmp_path):
    snapshot = _snapshot()
    artifact, boundary = _artifact(snapshot)
    artifact_path = tmp_path / "audit.json"
    boundary_path = tmp_path / "court_boundary.json"
    out_path = tmp_path / "report.json"
    artifact_path.write_text(json.dumps(artifact))
    boundary_path.write_text(json.dumps(boundary))

    before = set(os.listdir(tmp_path))
    rc = CLI.main([
        "--audit-artifact", str(artifact_path),
        "--court-boundary", str(boundary_path),
        "--required-pre-contact-m", "0.0",
        "--pre-contact-provenance", "configured",
        "--shadow-solve",
        "--output", str(out_path),
    ])
    assert rc == 0
    written = json.loads(out_path.read_text())
    assert written["snapshot"]["ball_count"] == 2
    assert "comparison" in written
    # No stray runtime files created beyond the requested output.
    after = set(os.listdir(tmp_path))
    assert after - before == {"report.json"}


def test_cli_uncalibrated_adaptive_exits_non_zero(tmp_path):
    snapshot = _snapshot()
    artifact, boundary = _artifact(snapshot)
    artifact_path = tmp_path / "audit.json"
    boundary_path = tmp_path / "court_boundary.json"
    artifact_path.write_text(json.dumps(artifact))
    boundary_path.write_text(json.dumps(boundary))
    # No provenance given -> the silent uncalibrated combination is refused.
    rc = CLI.main([
        "--audit-artifact", str(artifact_path),
        "--court-boundary", str(boundary_path),
        "--shadow-solve",
    ])
    assert rc == 2


def test_cli_baseline_only_allows_uncalibrated(tmp_path):
    snapshot = _snapshot()
    artifact, boundary = _artifact(snapshot)
    artifact_path = tmp_path / "audit.json"
    boundary_path = tmp_path / "court_boundary.json"
    out_path = tmp_path / "report.json"
    artifact_path.write_text(json.dumps(artifact))
    boundary_path.write_text(json.dumps(boundary))
    rc = CLI.main([
        "--audit-artifact", str(artifact_path),
        "--court-boundary", str(boundary_path),
        "--baseline-only",
        "--output", str(out_path),
    ])
    assert rc == 0
    written = json.loads(out_path.read_text())
    assert written["mode"] == "baseline_only"
    assert written["adaptive"] is None
    assert written["capture_geometry"]["is_calibrated"] is False


def test_cli_rejects_gate_not_greater_than_baseline(tmp_path):
    snapshot = _snapshot()
    artifact, boundary = _artifact(snapshot)
    artifact_path = tmp_path / "audit.json"
    boundary_path = tmp_path / "court_boundary.json"
    artifact_path.write_text(json.dumps(artifact))
    boundary_path.write_text(json.dumps(boundary))
    # Baseline run-in is 1.0; a gate of 0.5 must fail loudly (non-zero exit).
    rc = CLI.main([
        "--audit-artifact", str(artifact_path),
        "--court-boundary", str(boundary_path),
        "--required-pre-contact-m", "0.0",
        "--pre-contact-provenance", "configured",
        "--additional-gates", "0.5",
    ])
    assert rc == 2
