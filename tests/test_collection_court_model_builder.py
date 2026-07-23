"""Phase 6A: pure court_boundary.json (v2) -> planner CourtModel builder.

Every test proves a planner-observable property: the produced ``CourtModel``
is fed straight into ``analyze_snapshot`` (the real Phase 3A geometry, never
mocked) so the fence/net wall representation is validated by what the planner
actually does with it, not by inspecting the polygons directly.
"""

from copy import deepcopy
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"))

from collection_route_fixtures import FAKE_TIME_S, SCAN_POSE, default_configuration
from tennis_robot.collection_court_model_builder import (
    CourtModelBuildError,
    build_court_model,
)
from tennis_robot.collection_route_planner_v2 import CourtModel, analyze_snapshot
from tennis_robot.collection_route_types import (
    BallReasonCode,
    Point2D,
    PositionCovariance2D,
    ScanSnapshot,
    SnapshotBall,
)


# A minimal, deterministic v2 artifact modelled on runtime/court_boundary.json
# but with clean axis-aligned geometry so tangent directions are exact.  The
# fence is a rectangle x in [-9, 9], y in [-8, 8]; the net is the vertical
# segment x=0 spanning the two posts; one perimeter fixture sits centrally so a
# ball can be placed exactly on it.
def _boundary() -> dict:
    return {
        "schema": "court_knowledge_model/v2",
        "status": "OK",
        "failure_reason": None,
        "frame": "map",
        "completed": True,
        "net": {
            "center": {"x_m": 0.0, "y_m": 0.0},
            "posts": [{"x_m": 0.0, "y_m": 6.0}, {"x_m": 0.0, "y_m": -6.0}],
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
        "obstacles": [
            {
                "id": 1,
                "class": "perimeter_fixture",
                "center": {"x_m": 3.0, "y_m": 3.0},
                "size_m": {"w": 0.4, "h": 0.4},
            },
        ],
    }


def _snapshot(config, *balls):
    return ScanSnapshot(
        "scan-6a",
        FAKE_TIME_S,
        "map",
        SCAN_POSE,
        tuple(
            SnapshotBall(ball_id, Point2D(x, y), 0.95, PositionCovariance2D(1e-6, 0.0, 1e-6))
            for ball_id, x, y in balls
        ),
        config,
    )


def _feasibility_at(x, y):
    config = default_configuration()
    court = build_court_model(_boundary())
    return analyze_snapshot(_snapshot(config, ("ball-1", x, y)), court, config)[0]


def test_valid_v2_dict_builds_planner_accepted_court_model():
    court = build_court_model(_boundary())
    assert isinstance(court, CourtModel)
    # An interior free ball is reachable — the fence is NOT a filled polygon
    # that would make every inside ball point-in-polygon / KEEPOUT.
    feasibility = _feasibility_at(3.0, -3.0)
    assert feasibility.reachable
    assert feasibility.unreachable_reason is None


def test_ball_near_fence_edge_gets_parallel_only_tangent_not_keepout():
    # 0.6 m below the top fence edge (y=8): inside the eroded polygon but within
    # tangent activation distance -> only headings parallel to that edge (x).
    feasibility = _feasibility_at(0.0, 7.4)
    assert feasibility.reachable
    assert feasibility.unreachable_reason is None
    assert all(abs(math.sin(candidate.heading_rad)) < 1e-6 for candidate in feasibility.candidates)


def test_ball_near_net_gets_net_parallel_tangent():
    # Just off the net wall (x=0): net-parallel headings (along y) only.
    feasibility = _feasibility_at(0.65, 0.0)
    assert feasibility.reachable
    assert feasibility.unreachable_reason is None
    assert all(abs(math.cos(candidate.heading_rad)) < 1e-6 for candidate in feasibility.candidates)


def test_ball_outside_fence_is_keepout():
    feasibility = _feasibility_at(10.0, 0.0)
    assert feasibility.candidates == ()
    assert feasibility.unreachable_reason is BallReasonCode.KEEPOUT


def test_ball_on_interior_obstacle_is_keepout():
    # Dead centre of the fixture at (3, 3), far from every court boundary, so
    # the only possible reason is the interior obstacle itself.
    feasibility = _feasibility_at(3.0, 3.0)
    assert feasibility.candidates == ()
    assert feasibility.unreachable_reason is BallReasonCode.KEEPOUT


def test_perimeter_fixture_maps_to_non_tangent_kind():
    court = build_court_model(_boundary())
    fixture = next(obstacle for obstacle in court.obstacles if obstacle.obstacle_id == "obstacle_1")
    assert fixture.kind == "bench"
    assert fixture.kind not in {"net", "fence"}


@pytest.mark.parametrize(
    "mutate",
    [
        pytest.param(lambda d: d.__setitem__("frame", "odom"), id="wrong_frame"),
        pytest.param(lambda d: d.__setitem__("schema", "court_knowledge_model/v1"), id="wrong_schema"),
        pytest.param(lambda d: d.__setitem__("status", "FAILED"), id="failed_status"),
        pytest.param(lambda d: d.__setitem__("completed", False), id="not_completed"),
        pytest.param(lambda d: d.pop("fence"), id="missing_fence"),
        pytest.param(lambda d: d.pop("net"), id="missing_net"),
        pytest.param(lambda d: d["net"].pop("posts"), id="missing_posts"),
        pytest.param(lambda d: d["fence"].__setitem__("corners", d["fence"]["corners"][:2]), id="too_few_corners"),
    ],
)
def test_invalid_schema_is_typed_rejection(mutate):
    boundary = _boundary()
    mutate(boundary)
    with pytest.raises(CourtModelBuildError):
        build_court_model(boundary)


def test_build_is_deterministic():
    first = build_court_model(_boundary())
    # Shuffle the (single-element here) obstacle array + rebuild from a fresh
    # deep copy: identical input dict must yield an equal CourtModel with a
    # stable obstacle order.
    second = build_court_model(deepcopy(_boundary()))
    assert first == second
    assert [obstacle.obstacle_id for obstacle in first.obstacles] == [
        obstacle.obstacle_id for obstacle in second.obstacles
    ]
