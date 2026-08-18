"""The accelerated collision check must agree with the plain one, always.

`_segment_is_collision_free` gained bounding-box and half-plane short cuts
because it was 83% of planning time.  Those short cuts are only legitimate if
they never change a verdict, so the original formulation is kept here as the
definition and the two are compared over randomised geometry, including cases
placed deliberately on the boundary where an epsilon disagreement would show.
"""

import math
import os
import random
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"))

from tennis_robot.collection_route_planner_v2 import (  # noqa: E402
    _EPSILON,
    CourtModel,
    PolygonObstacle,
    _point_in_eroded_polygon,
    _segment_crosses_polygon_boundary,
    _segment_hits_inflated_polygon,
    _segment_is_collision_free,
    _segment_polygon_distance,
)
from tennis_robot.collection_route_types import Point2D  # noqa: E402


def reference_is_collision_free(start, end, court, clearance):
    """The frozen definition: what the check meant before it was made fast."""
    if not _point_in_eroded_polygon(start, court.navigable_polygon, clearance):
        return False
    if not _point_in_eroded_polygon(end, court.navigable_polygon, clearance):
        return False
    if _segment_crosses_polygon_boundary(start, end, court.navigable_polygon):
        return False
    if _segment_polygon_distance(start, end, court.navigable_polygon) <= clearance + _EPSILON:
        return False
    for obstacle in court.obstacles:
        if _segment_hits_inflated_polygon(start, end, obstacle.polygon, clearance):
            return False
    return True


def polygon(*points):
    return tuple(Point2D(float(x), float(y)) for x, y in points)


RECTANGLE = polygon((-12.0, -8.0), (12.0, -8.0), (12.0, 8.0), (-12.0, 8.0))
DIAMOND = polygon((0.0, -9.0), (13.0, 0.0), (0.0, 9.0), (-13.0, 0.0))
# Deliberately non-convex, so the fast path must fall back to the definition.
L_SHAPE = polygon(
    (-12.0, -8.0), (12.0, -8.0), (12.0, 0.0), (2.0, 0.0), (2.0, 8.0), (-12.0, 8.0)
)

OBSTACLES = (
    PolygonObstacle("net", "net", polygon((-0.05, -6.0), (0.05, -6.0), (0.05, 6.0), (-0.05, 6.0))),
    PolygonObstacle("bench", "bench", polygon((5.0, 3.0), (7.0, 3.0), (7.0, 4.5), (5.0, 4.5))),
    PolygonObstacle("post", "post", polygon((-8.0, -5.0), (-7.4, -5.0), (-7.4, -4.4), (-8.0, -4.4))),
)

COURTS = {
    "rectangle": CourtModel(RECTANGLE, OBSTACLES),
    "rectangle without obstacles": CourtModel(RECTANGLE, ()),
    "diamond": CourtModel(DIAMOND, OBSTACLES[:1]),
    "l shape": CourtModel(L_SHAPE, OBSTACLES),
}


@pytest.mark.parametrize("name", sorted(COURTS))
def test_random_segments_agree_with_the_definition(name):
    court = COURTS[name]
    generator = random.Random(20260815)
    disagreements = []
    for _ in range(4000):
        start = Point2D(generator.uniform(-15.0, 15.0), generator.uniform(-11.0, 11.0))
        end = Point2D(generator.uniform(-15.0, 15.0), generator.uniform(-11.0, 11.0))
        clearance = generator.choice((0.0, 0.05, 0.2, 0.5, 1.0))
        fast = _segment_is_collision_free(start, end, court, clearance)
        slow = reference_is_collision_free(start, end, court, clearance)
        if fast != slow:
            disagreements.append((start, end, clearance, fast, slow))
    assert not disagreements, disagreements[:5]


@pytest.mark.parametrize("name", sorted(COURTS))
def test_short_segments_near_the_boundary_agree(name):
    # Long random chords rarely land near a boundary; these are placed on it.
    court = COURTS[name]
    generator = random.Random(4242)
    clearance = 0.5
    disagreements = []
    for _ in range(4000):
        for edge_start, edge_end in zip(
            court.navigable_polygon, court.navigable_polygon[1:] + court.navigable_polygon[:1]
        ):
            ratio = generator.random()
            base = Point2D(
                edge_start.x_m + ratio * (edge_end.x_m - edge_start.x_m),
                edge_start.y_m + ratio * (edge_end.y_m - edge_start.y_m),
            )
            offset = generator.uniform(-1.2, 1.2)
            angle = generator.uniform(0.0, 2.0 * math.pi)
            start = Point2D(base.x_m + offset * math.cos(angle), base.y_m + offset * math.sin(angle))
            end = Point2D(
                start.x_m + generator.uniform(-1.0, 1.0), start.y_m + generator.uniform(-1.0, 1.0)
            )
            if _segment_is_collision_free(start, end, court, clearance) != (
                reference_is_collision_free(start, end, court, clearance)
            ):
                disagreements.append((start, end))
            break
    assert not disagreements, disagreements[:5]


def test_segments_hugging_an_obstacle_agree():
    court = COURTS["rectangle"]
    generator = random.Random(99)
    disagreements = []
    for obstacle in court.obstacles:
        for _ in range(2000):
            vertex = generator.choice(obstacle.polygon)
            clearance = generator.choice((0.05, 0.2, 0.5))
            start = Point2D(
                vertex.x_m + generator.uniform(-1.0, 1.0), vertex.y_m + generator.uniform(-1.0, 1.0)
            )
            end = Point2D(
                vertex.x_m + generator.uniform(-1.0, 1.0), vertex.y_m + generator.uniform(-1.0, 1.0)
            )
            if _segment_is_collision_free(start, end, court, clearance) != (
                reference_is_collision_free(start, end, court, clearance)
            ):
                disagreements.append((start, end, clearance))
    assert not disagreements, disagreements[:5]


def test_derived_geometry_does_not_change_court_identity():
    first = CourtModel(RECTANGLE, OBSTACLES)
    second = CourtModel(RECTANGLE, OBSTACLES)
    _segment_is_collision_free(Point2D(0.0, 0.0), Point2D(1.0, 1.0), first, 0.1)
    assert first == second
    assert hash(first) == hash(second)
    assert first.navigable_polygon == RECTANGLE


def test_convex_and_non_convex_courts_take_different_paths_but_agree():
    from tennis_robot.collection_route_planner_v2 import _court_geometry

    assert _court_geometry(COURTS["rectangle"]).navigable.half_planes is not None
    assert _court_geometry(COURTS["diamond"]).navigable.half_planes is not None
    assert _court_geometry(COURTS["l shape"]).navigable.half_planes is None
