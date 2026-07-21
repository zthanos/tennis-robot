#!/usr/bin/env python3
"""Emit a cross-language parity fixture for the Phase 6B container test.

Builds a REAL executable ``CollectionRoutePlan`` (single free ball, via
``plan_collection_route`` + the Phase 6A ``CourtModel`` builder), serializes it
with the pure Phase 6B modules, and writes every field value the C++ parity
gtest needs to reconstruct the ``CollectionExecutionContext`` message and the
``nav_msgs/Path``.

This script is PURE Python: no ROS import, only the offline collection modules.
The C++ gtest then proves the real controller accepts what this produced.

Usage:  python3 scripts/emit_collection_parity_fixture.py <output.json>
"""

from __future__ import annotations

import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"))

from tennis_robot.collection_court_model_builder import build_court_model
from tennis_robot.collection_execution_context_builder import (
    CONTEXT_SCHEMA_VERSION,
    ControllerTuning,
    build_execution_context,
)
from tennis_robot.collection_route_planner_v2 import plan_collection_route
from tennis_robot.collection_route_types import (
    CollectionRouteConfiguration,
    Point2D,
    Pose2D,
    PositionCovariance2D,
    ScanSnapshot,
    SnapshotBall,
)

# Minimal deterministic v2 survey artifact (clean axis-aligned court), fed
# through the real Phase 6A builder so the CourtModel is production geometry.
# The net wall is placed at x=8 (far from the route) so the collection route is
# a single straight line along y=0: the flattened polyline length then equals
# the plan's total_length exactly, which keeps make_tracking_plan's
# chord-vs-arc-length check exact rather than approximate.
_BOUNDARY = {
    "schema": "court_knowledge_model/v2",
    "status": "OK",
    "failure_reason": None,
    "frame": "map",
    "completed": True,
    "net": {
        "center": {"x_m": 8.0, "y_m": 0.0},
        "posts": [{"x_m": 8.0, "y_m": 6.0}, {"x_m": 8.0, "y_m": -6.0}],
        "span_m": 12.0,
    },
    "fence": {
        "corners": [
            {"x_m": -9.0, "y_m": -8.0},
            {"x_m": 9.0, "y_m": -8.0},
            {"x_m": 9.0, "y_m": 8.0},
            {"x_m": -9.0, "y_m": 8.0},
        ],
    },
    "obstacles": [],
}


def _default_configuration() -> CollectionRouteConfiguration:
    # Reuse the test fixture's approved configuration builder (pure).
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tests"))
    from collection_route_fixtures import default_configuration

    return default_configuration()


def _pose_list(poses):
    return [[p.x, p.y, p.z, p.qx, p.qy, p.qz, p.qw] for p in poses]


def build_fixture() -> dict:
    config = _default_configuration()
    court = build_court_model(_BOUNDARY)
    # A single free ball straight ahead of the robot (heading 0, same y) so the
    # whole route is one straight line, clear of net/fence walls.
    snapshot = ScanSnapshot(
        "scan-parity", 1000.0, "map", Pose2D(0.0, 0.0, 0.0),
        (SnapshotBall("ball-parity", Point2D(3.0, 0.0), 0.95, PositionCovariance2D(1e-6, 0.0, 1e-6)),),
        config,
    )
    plan = plan_collection_route(snapshot=snapshot, court=court, configuration=config).plan
    if not plan.is_executable or not plan.segments:
        raise SystemExit(f"parity fixture requires an executable plan, got {plan.planning_status}")

    tuning = ControllerTuning(
        lookahead_distance_m=1.0,
        max_angular_velocity_rad_s=2.0,
        progress_projection_window_m=5.0,
        crossing_speed_window_m=0.25,
        terminal_progress_tolerance_m=0.05,
    )
    context = build_execution_context(
        plan,
        controller_tuning=tuning,
        context_schema_version=CONTEXT_SCHEMA_VERSION,
        context_activation_timeout_s=5.0,
    )

    return {
        "map_frame": context.map_frame,
        "path_sha256": context.path_sha256,
        "poses": _pose_list(context.follow_path_poses),
        "context": {
            "context_schema_version": context.context_schema_version,
            "plan_id": context.plan_id,
            "path_sha256": context.path_sha256,
            "context_activation_timeout_s": context.context_activation_timeout_s,
            "terminal_progress_s": context.terminal_progress_s,
            "terminal_pose": [
                context.terminal_pose.x, context.terminal_pose.y, context.terminal_pose.z,
                context.terminal_pose.qx, context.terminal_pose.qy, context.terminal_pose.qz,
                context.terminal_pose.qw,
            ],
            "configuration_snapshot_json": context.configuration_snapshot_json,
            "controller_tuning": {
                "lookahead_distance_m": tuning.lookahead_distance_m,
                "max_angular_velocity_rad_s": tuning.max_angular_velocity_rad_s,
                "progress_projection_window_m": tuning.progress_projection_window_m,
                "crossing_speed_window_m": tuning.crossing_speed_window_m,
                "terminal_progress_tolerance_m": tuning.terminal_progress_tolerance_m,
            },
            "segments": [
                {
                    "segment_id": segment.segment_id,
                    "segment_type": segment.segment_type,
                    "progress_start_s": segment.progress_start_s,
                    "progress_end_s": segment.progress_end_s,
                    "execution_profile": {
                        "nominal_speed_mps": segment.execution_profile.nominal_speed_mps,
                        "min_speed_mps": segment.execution_profile.min_speed_mps,
                        "max_speed_mps": segment.execution_profile.max_speed_mps,
                        "nominal_speed_warning_tolerance_mps": segment.execution_profile.nominal_speed_warning_tolerance_mps,
                        "max_acceleration_mps2": segment.execution_profile.max_acceleration_mps2,
                        "max_deceleration_mps2": segment.execution_profile.max_deceleration_mps2,
                        "required_entry_m": segment.execution_profile.required_entry_m,
                        "required_run_in_m": segment.execution_profile.required_run_in_m,
                        "required_run_out_m": segment.execution_profile.required_run_out_m,
                        "max_curvature_per_m": segment.execution_profile.max_curvature_per_m,
                        "max_lateral_error_m": segment.execution_profile.max_lateral_error_m,
                        "max_heading_error_rad": segment.execution_profile.max_heading_error_rad,
                        "allow_reversing": segment.execution_profile.allow_reversing,
                        "allow_standalone_rotate": segment.execution_profile.allow_standalone_rotate,
                    },
                    "planned_crossings": [
                        {
                            "ball_id": crossing.ball_id,
                            "position_x_m": crossing.position_x_m,
                            "position_y_m": crossing.position_y_m,
                            "progress_s": crossing.progress_s,
                            "heading_rad": crossing.heading_rad,
                            "predicted_lateral_error": crossing.predicted_lateral_error,
                        }
                        for crossing in segment.planned_crossings
                    ],
                }
                for segment in context.segments
            ],
        },
    }


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit("usage: emit_collection_parity_fixture.py <output.json>")
    fixture = build_fixture()
    with open(sys.argv[1], "w", encoding="utf-8") as handle:
        json.dump(fixture, handle, indent=2)
    print(f"wrote parity fixture -> {sys.argv[1]}")
    print(f"  plan_id={fixture['context']['plan_id']} poses={len(fixture['poses'])} "
          f"segments={len(fixture['context']['segments'])} sha256={fixture['path_sha256']}")


if __name__ == "__main__":
    main()
