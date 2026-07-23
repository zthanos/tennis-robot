"""Offline contract tests for the strict runtime configuration builder."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import replace
import os
from pathlib import Path
import sys

import pytest

sys.path.insert(
    0,
    os.path.join(
        os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"
    ),
)

from collection_route_fixtures import (  # noqa: E402
    SCAN_POSE,
    default_configuration,
    default_court_half_boundary,
)
from tennis_robot.collection_route_config_builder import (  # noqa: E402
    CollectionRouteConfigurationBuildError,
    build_collection_route_configuration,
)
from tennis_robot.collection_route_planner_v2 import (  # noqa: E402
    CourtModel,
    plan_collection_route,
)
from tennis_robot.collection_route_types import (  # noqa: E402
    AcceptedSpatialObservation,
    CollectionRouteConfiguration,
    Point2D,
    PositionCovariance2D,
)
from tennis_robot.collection_scan_snapshot import ScanSnapshotBuilder  # noqa: E402
from tennis_robot.perception_covariance_calibration import load_artifact  # noqa: E402


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
CALIBRATION_ARTIFACT_PATH = (
    REPOSITORY_ROOT
    / "calibration_artifacts/gazebo/range_depth_quality_diagonal_v1-gazebo-v2.json"
)


def complete_source() -> dict:
    source = default_configuration().to_dict()
    del source["calibration_artifact"]
    return source


def build(source: dict | None = None) -> CollectionRouteConfiguration:
    return build_collection_route_configuration(
        complete_source() if source is None else source,
        calibration_artifact_path=CALIBRATION_ARTIFACT_PATH,
    )


def observation(step: str, x_m: float) -> AcceptedSpatialObservation:
    return AcceptedSpatialObservation(
        "runtime-config-scan",
        0,
        1.0,
        1.0,
        Point2D(x_m, 2.0),
        PositionCovariance2D(0.01, 0.0, 0.01),
        0.9,
        step,
        "gazebo-range-depth-quality-diagonal-v1-20260719-v2",
        "gazebo-mvp-provisional-planning-safety-v1",
    )


def test_complete_source_and_real_artifact_build_expected_configuration():
    configuration = build()
    expected = replace(
        default_configuration(),
        calibration_artifact=load_artifact(CALIBRATION_ARTIFACT_PATH),
    )

    assert configuration == expected
    assert configuration.calibration_artifact == load_artifact(
        CALIBRATION_ARTIFACT_PATH
    )


def test_configuration_passes_unchanged_through_snapshot_builder_and_planner():
    configuration = build()
    snapshot_builder = ScanSnapshotBuilder(
        scan_id="runtime-config-scan",
        scan_timestamp_s=1.0,
        robot_pose_at_scan=SCAN_POSE,
        configuration_snapshot=configuration,
        expected_scan_step_ids=("step-a", "step-b"),
        court_half_boundary=default_court_half_boundary(),
    )
    # Both steps count toward coverage, but separate one-observation tracks do
    # not meet the explicit two-confirmation threshold: the snapshot is empty.
    snapshot_builder.add(observation("step-a", 1.0))
    snapshot_builder.add(observation("step-b", 10.0))
    snapshot = snapshot_builder.finalize(2.0)
    court = CourtModel(
        (
            Point2D(-20.0, -20.0),
            Point2D(20.0, -20.0),
            Point2D(20.0, 20.0),
            Point2D(-20.0, 20.0),
        ),
        (),
    )
    plan = plan_collection_route(
        snapshot=snapshot, court=court, configuration=configuration
    ).plan

    assert snapshot.configuration_snapshot is configuration
    assert plan.configuration_snapshot is configuration


@pytest.mark.parametrize(
    ("mutation", "message"),
    (
        (lambda source: source.pop("mechanical"), "source.mechanical"),
        (
            lambda source: source["planning"].pop("maximum_planning_time_s"),
            "planning.maximum_planning_time_s",
        ),
    ),
)
def test_missing_group_or_field_fails_loud_with_exact_location(mutation, message):
    source = deepcopy(complete_source())
    mutation(source)

    with pytest.raises(CollectionRouteConfigurationBuildError, match=message):
        build(source)


def test_invalid_group_value_is_wrapped_with_group_context():
    source = deepcopy(complete_source())
    source["scan"]["scan_timeout_s"] = 0.0

    with pytest.raises(
        CollectionRouteConfigurationBuildError,
        match="invalid group 'scan'.*scan_timeout_s",
    ):
        build(source)


def test_invalid_calibration_path_fails_loud_with_typed_error(tmp_path):
    missing = tmp_path / "missing-artifact.json"

    with pytest.raises(
        CollectionRouteConfigurationBuildError, match="calibration_artifact"
    ):
        build_collection_route_configuration(
            complete_source(), calibration_artifact_path=missing
        )


def test_built_configuration_serialization_round_trip():
    configuration = build()

    assert CollectionRouteConfiguration.from_dict(
        configuration.to_dict()
    ) == configuration


def test_runtime_yaml_reserves_tracking_curvature_margin():
    import yaml

    source = yaml.safe_load(
        (REPOSITORY_ROOT / "ros2_ws/src/tennis_robot/config/collection_route.yaml").read_text()
    )
    mechanical = source["mechanical"]
    assert (
        mechanical["minimum_turning_radius_m"]
        * mechanical["maximum_curvature_per_m"]
    ) >= 1.5
    profile = source["planning"]["default_execution_profile"]
    assert profile["min_speed_mps"] <= profile["nominal_speed_mps"] <= 0.6
    assert profile["max_heading_error_rad"] <= 0.15

    nav2 = yaml.safe_load(
        (REPOSITORY_ROOT / "ros2_ws/src/tennis_robot/config/nav2_params.yaml").read_text()
    )
    controller = nav2["controller_server"]["ros__parameters"]
    assert controller["odom_topic"] == "/odometry/filtered"
    assert "collection_goal_checker" in controller["goal_checker_plugins"]
    assert controller["collection_goal_checker"]["xy_goal_tolerance"] >= 0.25
    executor = nav2["collection_route_executor"]["ros__parameters"]
    assert executor["collection_controller_tuning.lookahead_distance_m"] >= 0.6
    assert executor["collection_route.goal_checker_id"] == "collection_goal_checker"


def test_nav2_behavior_trees_select_general_goal_checker_explicitly():
    import xml.etree.ElementTree as ET

    config_dir = REPOSITORY_ROOT / "ros2_ws/src/tennis_robot/config"
    for filename in ("navigate_to_pose.xml", "court_survey.xml"):
        root = ET.parse(config_dir / filename).getroot()
        follow_path_nodes = root.findall(".//FollowPath")
        assert follow_path_nodes
        assert all(
            node.attrib.get("goal_checker_id") == "general_goal_checker"
            for node in follow_path_nodes
        )
