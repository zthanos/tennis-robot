#!/usr/bin/env python3
"""Report what a recorded collection run actually did, ball by ball.

Reads a planner audit artifact ({snapshot, plan}) and the execution trace
recorded alongside it, and prints the Phase 9 answers: per-segment tracking,
the planned/executed/confirmed matrix, pass-versus-connector success, outcome
classification per ball, and disturbance measurements.

Strictly read-only: it writes nothing and moves no robot.
"""

from __future__ import annotations

import argparse
import json
import os
import sys

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_PACKAGE_ROOT = os.path.join(_REPO_ROOT, "ros2_ws", "src", "tennis_robot")
if _PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, _PACKAGE_ROOT)

from tennis_robot.collection_capture_geometry import (  # noqa: E402
    PlaneProvenance,
    repo_base_footprint_capture_geometry,
)
from tennis_robot.collection_execution_evaluator import evaluate_execution  # noqa: E402
from tennis_robot.collection_execution_trace import ExecutionTrace  # noqa: E402
from tennis_robot.collection_route_schema_migration import (  # noqa: E402
    migrate_configuration_dict,
    migrate_snapshot_dict,
)
from tennis_robot.collection_route_types import (  # noqa: E402
    CollectionRoutePlan,
    ScanSnapshot,
)


def load_audit(path):
    with open(path, encoding="utf-8") as handle:
        artifact = json.load(handle)
    snapshot = ScanSnapshot.from_dict(migrate_snapshot_dict(artifact["snapshot"]).data)
    plan = dict(artifact["plan"])
    plan["configuration_snapshot"] = migrate_configuration_dict(
        plan["configuration_snapshot"]
    ).data
    return snapshot, CollectionRoutePlan.from_dict(plan)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--audit-artifact", required=True, help="planner audit JSON")
    parser.add_argument("--trace", required=True, help="execution trace JSON")
    parser.add_argument(
        "--displacement-threshold-m", type=float, default=0.10,
        help="diagnostic label only; the measurement is always reported in metres",
    )
    parser.add_argument("--disturbance-radius-m", type=float, default=1.5)
    parser.add_argument("--crossing-window-m", type=float, default=0.5)
    parser.add_argument("--required-pre-contact-m", type=float, default=0.3)
    parser.add_argument("--json", action="store_true", help="machine-readable output")
    arguments = parser.parse_args()

    snapshot, plan = load_audit(arguments.audit_artifact)
    with open(arguments.trace, encoding="utf-8") as handle:
        trace = ExecutionTrace.from_dict(json.load(handle))
    geometry = repo_base_footprint_capture_geometry(
        required_pre_contact_straight_m=arguments.required_pre_contact_m,
        required_pre_contact_provenance=PlaneProvenance.CONFIGURED,
    )
    evaluation = evaluate_execution(
        snapshot=snapshot, plan=plan, trace=trace, capture_geometry=geometry,
        displacement_threshold_m=arguments.displacement_threshold_m,
        disturbance_reporting_radius_m=arguments.disturbance_radius_m,
        crossing_window_m=arguments.crossing_window_m,
    )

    if arguments.json:
        print(json.dumps(_as_dict(evaluation), indent=1, sort_keys=True))
        return 0

    print(f"run {evaluation.run_id}  plan {evaluation.plan_id}  "
          f"telemetry rows {evaluation.telemetry_rows}\n")

    print("per-ball outcome")
    print(f"  {'ball':>16s} {'outcome':>34s} {'swept':>6s} {'clear m':>8s} "
          f"{'lat m':>7s} {'conf':>5s} {'moved m':>8s} {'segment':>18s}")
    for item in evaluation.outcomes:
        print(
            f"  {item.ball_id[-16:]:>16s} {item.outcome.value:>34s} "
            f"{str(item.executed.crossed):>6s} {item.executed.minimum_clearance_m:>8.3f} "
            f"{_optional(item.executed.lateral_offset_m):>7s} {str(item.confirmed):>5s} "
            f"{_optional(item.displacement_m):>8s} {str(item.intended_segment_id)[-18:]:>18s}"
        )

    print("\nplanned / executed / confirmed matrix")
    for key, count in sorted(evaluation.matrix().items()):
        print(f"  planned={key[0]:>9s} executed={key[1]:>9s} confirmed={key[2]:>9s} : {count}")

    print("\nby segment type")
    for kind, row in sorted(evaluation.by_segment_type().items()):
        print(f"  {kind:>16s} planned {row['planned']:>3d} executed {row['executed']:>3d} "
              f"confirmed {row['confirmed']:>3d}")

    print("\nsegment tracking")
    print(f"  {'segment':>20s} {'type':>16s} {'n':>5s} {'maxXT':>7s} {'rmsXT':>7s} "
          f"{'maxHE':>7s} {'endPos':>7s} {'plan m':>7s} {'exec m':>7s} {'s':>6s}")
    for item in evaluation.tracking:
        print(
            f"  {item.segment_id[-20:]:>20s} {item.segment_type:>16s} {item.samples:>5d} "
            f"{item.max_cross_track_m:>7.3f} {item.rms_cross_track_m:>7.3f} "
            f"{item.max_heading_error_rad:>7.3f} {item.endpoint_position_error_m:>7.3f} "
            f"{item.planned_length_m:>7.2f} {item.executed_length_m:>7.2f} {item.duration_s:>6.1f}"
        )

    if evaluation.disturbances:
        print("\nclosest approach to balls that were not the current target")
        print(f"  {'ball':>16s} {'body m':>8s} {'mouth m':>8s} {'speed':>6s} "
              f"{'moved m':>8s} {'t to attempt':>13s} {'segment':>18s}")
        for item in evaluation.disturbances:
            print(
                f"  {item.ball_id[-16:]:>16s} {item.body_clearance_m:>8.3f} "
                f"{item.mouth_clearance_m:>8.3f} {item.speed_mps:>6.2f} "
                f"{_optional(item.displacement_m):>8s} "
                f"{_optional(item.time_to_intended_attempt_s):>13s} "
                f"{str(item.segment_id)[-18:]:>18s}"
            )
    return 0


def _optional(value):
    return "-" if value is None else f"{value:.3f}"


def _as_dict(evaluation):
    return {
        "run_id": evaluation.run_id,
        "plan_id": evaluation.plan_id,
        "telemetry_rows": evaluation.telemetry_rows,
        "matrix": {"/".join(key): count for key, count in evaluation.matrix().items()},
        "by_segment_type": evaluation.by_segment_type(),
        "outcomes": [
            {
                "ball_id": item.ball_id,
                "outcome": item.outcome.value,
                "planned_crossing": item.planned_crossing,
                "executed_crossing": item.executed.crossed,
                "minimum_clearance_m": item.executed.minimum_clearance_m,
                "lateral_offset_m": item.executed.lateral_offset_m,
                "confirmed": item.confirmed,
                "confirmation_latency_s": item.confirmation_latency_s,
                "displacement_m": item.displacement_m,
                "intended_segment_id": item.intended_segment_id,
                "intended_segment_type": item.intended_segment_type,
                "detail": item.detail,
            }
            for item in evaluation.outcomes
        ],
        "tracking": [
            {
                "segment_id": item.segment_id, "segment_type": item.segment_type,
                "samples": item.samples,
                "max_cross_track_m": item.max_cross_track_m,
                "rms_cross_track_m": item.rms_cross_track_m,
                "max_heading_error_rad": item.max_heading_error_rad,
                "endpoint_position_error_m": item.endpoint_position_error_m,
                "planned_length_m": item.planned_length_m,
                "executed_length_m": item.executed_length_m,
                "duration_s": item.duration_s,
            }
            for item in evaluation.tracking
        ],
        "disturbances": [
            {
                "ball_id": item.ball_id, "segment_id": item.segment_id,
                "body_clearance_m": item.body_clearance_m,
                "mouth_clearance_m": item.mouth_clearance_m,
                "speed_mps": item.speed_mps,
                "displacement_m": item.displacement_m,
                "time_to_intended_attempt_s": item.time_to_intended_attempt_s,
            }
            for item in evaluation.disturbances
        ],
    }


if __name__ == "__main__":
    raise SystemExit(main())
