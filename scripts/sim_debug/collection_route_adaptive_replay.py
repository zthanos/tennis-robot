#!/usr/bin/env python3
"""Offline replay/analysis of the SHADOW adaptive collection-route approach.

Reads a frozen planner audit artifact (``{snapshot, plan, ...}``) and a
``court_boundary.json`` survey artifact, then reports, side by side:

1. the current live baseline planner outcome (byte-compared to the embedded
   plan when present),
2. the shadow adaptive candidate / connector-graph analysis,
3. optionally, a shadow global solve — **only** through the offline
   ``collection_adaptive_approach`` API, never the live planner.

The tool is strictly read-only: it writes NO runtime state and moves NO robot.
It prints machine-readable JSON to stdout (or ``--output``).

It imports the collection-route modules from the package source tree; no
absolute paths are baked into any production module.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from typing import Any, Mapping

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_PACKAGE_ROOT = os.path.join(_REPO_ROOT, "ros2_ws", "src", "tennis_robot")
if _PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, _PACKAGE_ROOT)

from tennis_robot.collection_adaptive_approach import (  # noqa: E402
    AdaptiveApproachConfiguration,
    AdaptiveApproachError,
    baseline_shadow_solve,
    generate_adaptive_candidates,
    shadow_global_solve,
)
from tennis_robot.collection_capture_geometry import (  # noqa: E402
    PlaneProvenance,
    repo_base_footprint_capture_geometry,
)
from tennis_robot.collection_court_model_builder import build_court_model  # noqa: E402
from tennis_robot.collection_route_planner_v2 import plan_collection_route  # noqa: E402
from tennis_robot.collection_route_types import (  # noqa: E402
    BallStatus,
    RouteSegmentType,
    ScanSnapshot,
)


def _plan_length_breakdown(plan) -> dict[str, float]:
    connector = 0.0
    pass_length = 0.0
    terminal = 0.0
    for segment in plan.segments:
        span = segment.progress_end_m - segment.progress_start_m
        if segment.type is RouteSegmentType.CONNECTOR:
            connector += span
        elif segment.type is RouteSegmentType.FUNNEL_PASS:
            pass_length += span
        elif segment.type is RouteSegmentType.TERMINAL_CONNECTOR:
            terminal += span
    return {
        "total_length_m": plan.total_length_m,
        "connector_length_m": connector,
        "pass_length_m": pass_length,
        "terminal_length_m": terminal,
    }


def _plan_summary(plan) -> dict[str, Any]:
    funnel_passes = [s for s in plan.segments if s.type is RouteSegmentType.FUNNEL_PASS]
    covered = [r for r in plan.ball_results if r.status is BallStatus.COVERED]
    deferred = [r for r in plan.ball_results if r.status is BallStatus.DEFERRED]
    unreachable = [r for r in plan.ball_results if r.status is BallStatus.UNREACHABLE]
    balls_per_pass = sorted(len(s.covered_ball_ids) for s in funnel_passes)
    breakdown = _plan_length_breakdown(plan)
    return {
        "planning_status": plan.planning_status.value,
        "planning_search_status": plan.planning_search_status.value,
        "coverage": {
            "covered": len(covered),
            "deferred": len(deferred),
            "unreachable": len(unreachable),
            "snapshot_balls": len(plan.snapshot_ball_ids),
        },
        "expected_duration_s": plan.expected_duration_s,
        "pass_count": len(funnel_passes),
        "balls_per_shared_pass": balls_per_pass,
        "deferred_reasons": sorted(r.reason_code.value for r in deferred),
        "unreachable_reasons": sorted(r.reason_code.value for r in unreachable),
        **breakdown,
    }


_SEARCH_EXPANSIONS_NOTE = (
    "The production solver combines candidate-cap and DFS-search exhaustion "
    "(planning_search_status = search_exhausted OR candidate_budget_exhausted); "
    "independent DFS exhaustion is not surfaced."
)


def _shadow_solve_report(result) -> dict[str, Any]:
    report = _plan_summary(result.plan)
    report.update(
        {
            "graph_node_count": result.graph_node_count,
            "graph_edge_count": result.graph_edge_count,
            "connector_edge_rejection_histogram": dict(sorted(result.connector_rejection_histogram.items())),
            "bounded_candidate_count": result.bounded_candidate_count,
            # Independently known budget signals — attributable to a single cause.
            "global_candidate_cap_exhausted": result.global_candidate_cap_exhausted,
            "shared_pass_budget_exhausted": result.shared_pass_budget_exhausted,
            "shared_pass_rejection_count": len(result.shared_pass_rejections),
            # COMBINED production status: cannot be decomposed into DFS-search vs
            # candidate-cap exhaustion — reported honestly, never as independent
            # DFS/search exhaustion.
            "combined_planner_budget_status": result.combined_planner_budget_status,
            "search_exhaustion_independently_known": False,
            "search_expansions": None,
            "search_expansions_note": _SEARCH_EXPANSIONS_NOTE,
        }
    )
    return report


_LENGTH_COMPARISON_EPSILON_M = 1e-6


def _classify_observation(
    *,
    baseline_covered: int,
    adaptive_covered: int,
    baseline_total_length_m: float,
    adaptive_total_length_m: float,
) -> tuple[int, float, str]:
    """Classify raw A/B observations deterministically.

    Coverage is primary.  Route length is compared only when coverage is equal,
    and differences within one micrometre are treated as equal to avoid
    classifying insignificant floating-point noise as a route change.
    """
    coverage_delta = adaptive_covered - baseline_covered
    length_delta_m = adaptive_total_length_m - baseline_total_length_m

    if coverage_delta > 0:
        observed_result = "improved"
    elif coverage_delta < 0:
        observed_result = "worsened"
    elif not math.isfinite(length_delta_m):
        observed_result = "mixed"
    elif length_delta_m < -_LENGTH_COMPARISON_EPSILON_M:
        observed_result = "improved"
    elif length_delta_m > _LENGTH_COMPARISON_EPSILON_M:
        observed_result = "worsened"
    else:
        observed_result = "unchanged"

    return coverage_delta, length_delta_m, observed_result


def _comparison_conclusion(*, conclusive: bool, observed_result: str) -> str:
    if conclusive:
        return f"Conclusive comparison under the current bounded heuristic; observed result: {observed_result}."

    observation_text = {
        "improved": "An improvement was observed",
        "unchanged": "No improvement was observed",
        "worsened": "A worse result was observed",
        "mixed": "A mixed result was observed",
    }[observed_result]
    return (
        f"{observation_text}, but the comparison is inconclusive; see comparison_limitations. "
        "Raw observations are reported separately and no causal conclusion may be drawn."
    )


def _build_comparison(baseline_graph, adaptive_result, shadow) -> dict[str, Any]:
    """Separate raw observation from causal conclusion.

    Only *independently known* budget exhaustions are asserted as such
    (candidate generation, shared-pass, global candidate cap).  The production
    planner's ``planning_search_status`` is a COMBINED signal — it is
    ``budget_exhausted`` when either DFS search OR the candidate cap ran out — so
    we never claim independent DFS/search exhaustion.  When that combined status
    is non-complete but its cause cannot be separated, it is recorded as a
    distinct ``*_combined_planner_budget_status_*`` limitation, not as a proven
    search-budget exhaustion.
    """
    limitations: list[str] = []
    # Independently attributable exhaustions.
    if adaptive_result.budget_exhausted:
        limitations.append("adaptive_candidate_generation_budget_exhausted")
    if shadow.shared_pass_budget_exhausted:
        limitations.append("adaptive_shared_pass_budget_exhausted")
    if baseline_graph.global_candidate_cap_exhausted:
        limitations.append("baseline_global_candidate_cap_exhausted")
    if shadow.global_candidate_cap_exhausted:
        limitations.append("adaptive_global_candidate_cap_exhausted")
    # Combined (non-decomposable) planner status — never asserted as DFS search.
    if baseline_graph.combined_planner_budget_status != "complete":
        limitations.append(
            f"baseline_combined_planner_budget_status_{baseline_graph.combined_planner_budget_status}"
        )
    if shadow.combined_planner_budget_status != "complete":
        limitations.append(
            f"adaptive_combined_planner_budget_status_{shadow.combined_planner_budget_status}"
        )

    baseline_covered = sum(
        1 for r in baseline_graph.plan.ball_results if r.status is BallStatus.COVERED
    )
    adaptive_covered = sum(1 for r in shadow.plan.ball_results if r.status is BallStatus.COVERED)
    baseline_total_length_m = baseline_graph.plan.total_length_m
    adaptive_total_length_m = shadow.plan.total_length_m
    coverage_delta, length_delta_m, observed_result = _classify_observation(
        baseline_covered=baseline_covered,
        adaptive_covered=adaptive_covered,
        baseline_total_length_m=baseline_total_length_m,
        adaptive_total_length_m=adaptive_total_length_m,
    )

    conclusive = not limitations
    return {
        "comparison_conclusive": conclusive,
        "comparison_limitations": limitations,
        # Never claim independent DFS/search exhaustion.
        "search_exhaustion_independently_known": False,
        "comparison_conclusion": _comparison_conclusion(
            conclusive=conclusive,
            observed_result=observed_result,
        ),
        "observation": {
            "baseline_covered": baseline_covered,
            "adaptive_covered": adaptive_covered,
            "coverage_delta": coverage_delta,
            "baseline_total_length_m": baseline_total_length_m,
            "adaptive_total_length_m": adaptive_total_length_m,
            "length_delta_m": length_delta_m,
            "length_comparison_epsilon_m": _LENGTH_COMPARISON_EPSILON_M,
            "observed_result": observed_result,
            "note": "Raw A/B numbers only; see comparison_conclusive for whether a conclusion may be drawn.",
        },
    }


def build_report(
    *,
    artifact: Mapping[str, Any],
    boundary: Mapping[str, Any],
    adaptive: AdaptiveApproachConfiguration,
    capture_reference_plane_id: str,
    required_pre_contact_straight_m: float,
    required_pre_contact_provenance: PlaneProvenance,
    run_shadow_solve: bool,
    baseline_only: bool = False,
    input_paths: Mapping[str, str] | None = None,
) -> dict[str, Any]:
    """Pure report builder — no file I/O, no argparse, deterministic.

    ``baseline_only`` runs the live baseline planner and displays the (possibly
    uncalibrated) capture geometry, but does NOT generate adaptive candidates or
    run the shadow solve.  With ``baseline_only`` False, the capture geometry
    must be calibrated/configured: ``generate_adaptive_candidates`` raises
    ``AdaptiveApproachError`` on uncalibrated geometry, which the CLI surfaces as
    a non-zero exit.

    Every algorithm-affecting parameter is echoed back under ``analysis_inputs``
    so a frozen replay is fully reproducible from its own report.  ``input_paths``
    is optional (filesystem paths are recorded for provenance but excluded from
    deterministic report comparison by callers that omit them).
    """
    snapshot = ScanSnapshot.from_dict(artifact["snapshot"])
    court = build_court_model(boundary)
    configuration = snapshot.configuration_snapshot

    capture_geometry = repo_base_footprint_capture_geometry(
        capture_reference_plane_id=capture_reference_plane_id,
        required_pre_contact_straight_m=required_pre_contact_straight_m,
        required_pre_contact_provenance=required_pre_contact_provenance,
    )

    analysis_inputs: dict[str, Any] = {
        "additional_gate_distances_m": list(adaptive.additional_gate_distances_m),
        "lateral_offsets_m": list(adaptive.lateral_offsets_m),
        "max_candidates_per_heading": adaptive.max_candidates_per_heading,
        "max_candidates_per_ball": adaptive.max_candidates_per_ball,
        "capture_reference_plane_id": capture_reference_plane_id,
        "required_pre_contact_straight_m": required_pre_contact_straight_m,
        "required_pre_contact_provenance": required_pre_contact_provenance.value,
        "run_shadow_solve": run_shadow_solve,
        "baseline_only": baseline_only,
        # The global candidate cap comes from the frozen snapshot configuration
        # and is algorithm-affecting for the shadow solve.
        "maximum_candidate_count": configuration.planning.maximum_candidate_count,
        "minimum_run_in_m": configuration.mechanical.minimum_run_in_m,
        "assumptions": {
            "required_pre_contact_is_offline_analysis_assumption": (
                required_pre_contact_provenance is not PlaneProvenance.MEASURED
            ),
            "note": (
                "required_pre_contact_straight_m with provenance!=measured is an "
                "OFFLINE ANALYSIS ASSUMPTION for this frozen comparison — not an "
                "intake measurement and not a production calibration. The real "
                "required pre-contact distance still requires intake trials."
            ),
        },
    }
    if input_paths is not None:
        analysis_inputs["paths"] = dict(input_paths)

    # ── baseline ────────────────────────────────────────────────────────────
    t0 = time.perf_counter()
    baseline_result = plan_collection_route(
        snapshot=snapshot, court=court, configuration=configuration
    )
    baseline_wall_ms = (time.perf_counter() - t0) * 1000.0
    baseline_plan = baseline_result.plan

    baseline_matches_embedded = None
    if "plan" in artifact:
        baseline_matches_embedded = json.dumps(baseline_plan.to_dict(), sort_keys=True) == json.dumps(
            artifact["plan"], sort_keys=True
        )

    t0 = time.perf_counter()
    baseline_graph = baseline_shadow_solve(
        snapshot=snapshot, court=court, configuration=configuration
    )
    baseline_graph_wall_ms = (time.perf_counter() - t0) * 1000.0

    report: dict[str, Any] = {
        "mode": "baseline_only" if baseline_only else "adaptive",
        "analysis_inputs": analysis_inputs,
        "snapshot": {
            "scan_id": snapshot.scan_id,
            "map_frame": snapshot.map_frame,
            "ball_count": len(snapshot.balls),
        },
        "capture_geometry": {
            **capture_geometry.to_dict(),
            "minimum_alignment_corridor_m": capture_geometry.minimum_alignment_corridor_m,
            "is_calibrated": capture_geometry.is_calibrated,
            "uncalibrated_fields": list(capture_geometry.uncalibrated_fields()),
        },
        "baseline": {
            "matches_embedded_plan": baseline_matches_embedded,
            "shared_pass_candidate_budget_exhausted": baseline_result.shared_pass_candidate_budget_exhausted,
            "planning_wall_ms": baseline_wall_ms,
            "graph_wall_ms": baseline_graph_wall_ms,
            **_shadow_solve_report(baseline_graph),
        },
    }

    if baseline_only:
        report["adaptive"] = None
        report["comparison"] = {
            "comparison_conclusive": None,
            "comparison_limitations": ["baseline_only_mode_no_adaptive_generation"],
            "comparison_conclusion": "Baseline-only mode: no adaptive candidates generated.",
        }
        return report

    # ── adaptive candidate generation (requires calibrated geometry) ──────────
    t0 = time.perf_counter()
    adaptive_result = generate_adaptive_candidates(
        snapshot=snapshot,
        court=court,
        configuration=configuration,
        capture_geometry=capture_geometry,
        adaptive=adaptive,
    )
    adaptive_wall_ms = (time.perf_counter() - t0) * 1000.0

    per_ball = []
    for entry in adaptive_result.per_ball:
        baselines = sum(1 for c in entry.candidates if c.is_baseline)
        per_ball.append(
            {
                "ball_id": entry.ball_id,
                "reachable": bool(entry.candidates),
                "unreachable_reason": entry.unreachable_reason.value if entry.unreachable_reason else None,
                "pareto_kept_candidates": len(entry.candidates),
                "pareto_kept_baseline": baselines,
                "pareto_kept_adaptive": len(entry.candidates) - baselines,
                "per_heading_budget_exhausted": entry.per_heading_budget_exhausted,
                "per_ball_budget_exhausted": entry.per_ball_budget_exhausted,
            }
        )

    report["adaptive"] = {
        "generation_wall_ms": adaptive_wall_ms,
        # Unambiguous candidate counters (exact arithmetic invariants).
        "raw_baseline_candidate_count": adaptive_result.raw_baseline_candidate_count,
        "raw_adaptive_extra_candidate_count": adaptive_result.raw_adaptive_extra_candidate_count,
        "raw_candidate_count": adaptive_result.raw_candidate_count,
        "pareto_kept_baseline_count": adaptive_result.pareto_kept_baseline_count,
        "pareto_kept_adaptive_count": adaptive_result.pareto_kept_adaptive_count,
        "pareto_kept_total": adaptive_result.pareto_kept_total,
        "pareto_pruned_count": adaptive_result.pareto_pruned_count,
        "candidate_generation_budget_exhausted": adaptive_result.budget_exhausted,
        "configuration": {
            "additional_gate_distances_m": list(adaptive.additional_gate_distances_m),
            "lateral_offsets_m": list(adaptive.lateral_offsets_m),
            "max_candidates_per_heading": adaptive.max_candidates_per_heading,
            "max_candidates_per_ball": adaptive.max_candidates_per_ball,
        },
        "per_ball": per_ball,
    }

    if run_shadow_solve:
        t0 = time.perf_counter()
        shadow = shadow_global_solve(
            snapshot=snapshot,
            court=court,
            configuration=configuration,
            adaptive_result=adaptive_result,
        )
        shadow_wall_ms = (time.perf_counter() - t0) * 1000.0
        report["shadow_global_solve"] = {
            "planning_wall_ms": shadow_wall_ms,
            **_shadow_solve_report(shadow),
        }
        report["comparison"] = _build_comparison(baseline_graph, adaptive_result, shadow)
    else:
        report["comparison"] = {
            "comparison_conclusive": None,
            "comparison_limitations": ["shadow_global_solve_not_run"],
            "comparison_conclusion": "Shadow global solve not run; no A/B comparison performed.",
        }

    return report


def _parse_floats(text: str, name: str) -> tuple[float, ...]:
    if not text.strip():
        return ()
    try:
        return tuple(float(item) for item in text.split(","))
    except ValueError as exc:
        raise SystemExit(f"invalid {name}: {text!r} ({exc})")


def _load_json(path: str, name: str) -> Any:
    if not os.path.isfile(path):
        raise SystemExit(f"{name} not found: {path}")
    with open(path, encoding="utf-8") as handle:
        return json.load(handle)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--audit-artifact", required=True, help="planner audit JSON ({snapshot, plan, ...})")
    parser.add_argument("--court-boundary", required=True, help="court_boundary.json (schema v2)")
    parser.add_argument("--additional-gates", default="1.4,1.9", help="comma-separated extra run-in distances (m)")
    parser.add_argument("--lateral-offsets", default="0.0,0.02,-0.02", help="comma-separated signed lateral offsets (m)")
    parser.add_argument("--max-per-heading", type=int, default=6)
    parser.add_argument("--max-per-ball", type=int, default=64)
    parser.add_argument(
        "--capture-reference-plane",
        default="intake_mouth_contact",
        help="plane id the run-in aligns to",
    )
    parser.add_argument(
        "--required-pre-contact-m",
        type=float,
        default=None,
        help="required straight distance before first funnel contact (m); required for adaptive mode",
    )
    parser.add_argument(
        "--pre-contact-provenance",
        default=None,
        choices=[p.value for p in PlaneProvenance],
        help="provenance of the pre-contact straight distance; must be measured/configured for adaptive mode",
    )
    parser.add_argument("--shadow-solve", action="store_true", help="run the optional shadow global solve")
    parser.add_argument(
        "--baseline-only",
        action="store_true",
        help="report the live baseline + (possibly uncalibrated) capture geometry only; no adaptive generation",
    )
    parser.add_argument("--output", default=None, help="write JSON here instead of stdout")
    args = parser.parse_args(argv)

    artifact = _load_json(args.audit_artifact, "audit artifact")
    if not isinstance(artifact, Mapping) or "snapshot" not in artifact:
        raise SystemExit("audit artifact must contain a 'snapshot' object")
    boundary = _load_json(args.court_boundary, "court boundary")

    # Resolve capture-geometry provenance without a silent uncalibrated default
    # on the adaptive path.
    provenance_value = args.pre_contact_provenance
    pre_contact_m = args.required_pre_contact_m
    if args.baseline_only:
        # Report-only: uncalibrated is allowed and displayed, never used to solve.
        provenance = PlaneProvenance(provenance_value or "uncalibrated")
        pre_contact_m = 0.0 if pre_contact_m is None else pre_contact_m
    else:
        if provenance_value is None or provenance_value == PlaneProvenance.UNCALIBRATED.value or pre_contact_m is None:
            error = {
                "error": "adaptive mode requires explicit calibrated/configured capture geometry",
                "detail": (
                    "pass --required-pre-contact-m and --pre-contact-provenance "
                    "(measured|configured), or use --baseline-only"
                ),
            }
            print(json.dumps(error), file=sys.stderr)
            return 2
        provenance = PlaneProvenance(provenance_value)

    adaptive = AdaptiveApproachConfiguration(
        _parse_floats(args.additional_gates, "additional-gates"),
        _parse_floats(args.lateral_offsets, "lateral-offsets"),
        args.max_per_heading,
        args.max_per_ball,
    )

    try:
        report = build_report(
            artifact=artifact,
            boundary=boundary,
            adaptive=adaptive,
            capture_reference_plane_id=args.capture_reference_plane,
            required_pre_contact_straight_m=pre_contact_m,
            required_pre_contact_provenance=provenance,
            run_shadow_solve=args.shadow_solve,
            baseline_only=args.baseline_only,
            input_paths={
                "audit_artifact": args.audit_artifact,
                "court_boundary": args.court_boundary,
            },
        )
    except AdaptiveApproachError as exc:
        print(json.dumps({"error": "adaptive_approach_error", "detail": str(exc)}), file=sys.stderr)
        return 2

    text = json.dumps(report, indent=2, sort_keys=True)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as handle:
            handle.write(text + "\n")
    else:
        print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
