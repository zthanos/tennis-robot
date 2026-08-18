"""The frame-diagnosis checks must fail on the defects they exist to catch.

Phase 11G.  Each test builds a recording with exactly one property broken and
asserts the corresponding check rejects it; the clean recording passes all four.
Without these, a check that silently always passes would look like evidence.
"""

import importlib.util
import math
import os
import sys

import pytest

_MODULE_PATH = os.path.join(
    os.path.dirname(__file__), "..", "scripts", "sim_debug", "analyze_frame_diagnosis.py"
)
_spec = importlib.util.spec_from_file_location("analyze_frame_diagnosis", _MODULE_PATH)
analysis = importlib.util.module_from_spec(_spec)
sys.modules["analyze_frame_diagnosis"] = analysis
_spec.loader.exec_module(analysis)


def transform(x=0.0, y=0.0, yaw=0.0, stamp=100.0, target="map", source="odom"):
    return {"target_frame": target, "source_frame": source,
            "x_m": x, "y_m": y, "yaw_rad": yaw, "stamp_s": stamp}


def row(*, pose=(1.0, 0.0, 0.0), reference=(1.0, 0.05, 0.0), pose_frame="odom",
        path_frame="odom", lateral=None, heading=None, map_odom=(0.2, 0.1, 0.0),
        transform_stamp=100.0, pose_stamp=100.0, update_stamp=100.01, path_stamp=60.0):
    """One tracking update, consistent by default."""
    odom_base = (5.0, 2.0, 0.3)
    cos, sin = math.cos(map_odom[2]), math.sin(map_odom[2])
    map_base = (
        map_odom[0] + cos * odom_base[0] - sin * odom_base[1],
        map_odom[1] + sin * odom_base[0] + cos * odom_base[1],
        odom_base[2] + map_odom[2],
    )
    if lateral is None:
        lateral = math.dist(pose[:2], reference[:2])
    if heading is None:
        heading = (reference[2] - pose[2] + math.pi) % (2 * math.pi) - math.pi
    return {
        "progress_s": 1.0,
        "reported_lateral_error_m": lateral,
        "reported_heading_error_rad": heading,
        "tracker": {"pose_frame_id": pose_frame, "pose_stamp_s": pose_stamp,
                    "x_m": pose[0], "y_m": pose[1], "yaw_rad": pose[2],
                    "update_stamp_s": update_stamp},
        "reference": {"path_frame_id": path_frame, "path_stamp_s": path_stamp,
                      "x_m": reference[0], "y_m": reference[1], "yaw_rad": reference[2]},
        "map_odom": transform(*map_odom, stamp=transform_stamp),
        "odom_base": transform(*odom_base, stamp=pose_stamp, target="odom",
                               source="base_footprint"),
        "map_base": transform(*map_base, stamp=pose_stamp, target="map",
                              source="base_footprint"),
    }


def test_a_consistent_recording_passes_every_check():
    report = analysis.analyze([row(), row(pose=(2.0, 0.1, 0.05))])
    assert report["tracker_self_consistency"]["passed"]
    assert report["frame_transform_consistency"]["passed"]
    assert report["path_frame_consistency"]["passed"]
    assert report["timestamp_consistency"]["stale_transform_rows"] == 0


def test_a_tracker_that_reports_an_error_it_did_not_compute_is_caught():
    # The defect this whole phase exists to make visible: a reported number that
    # cannot be reproduced from the two objects it claims to come from.
    report = analysis.analyze([row(lateral=0.001)])
    assert not report["tracker_self_consistency"]["passed"]
    assert report["tracker_self_consistency"]["max_lateral_residual_m"] == pytest.approx(0.049)


def test_a_heading_error_inconsistent_with_the_reference_tangent_is_caught():
    report = analysis.analyze([row(heading=0.5)])
    assert not report["tracker_self_consistency"]["passed"]
    assert report["tracker_self_consistency"]["max_heading_residual_rad"] > 0.4


def test_a_broken_transform_chain_is_caught():
    broken = row()
    broken["map_base"]["x_m"] += 0.3
    report = analysis.analyze([broken])
    assert not report["frame_transform_consistency"]["passed"]
    assert report["frame_transform_consistency"]["max_residual_m"] == pytest.approx(0.3)


def test_comparing_a_pose_and_a_path_point_from_different_frames_is_caught():
    report = analysis.analyze([row(pose_frame="odom", path_frame="map")])
    assert not report["path_frame_consistency"]["passed"]
    assert report["path_frame_consistency"]["mismatched"] == [("odom", "map")]


def test_an_unnamed_frame_is_not_accepted_as_agreement():
    # Two empty frame ids are equal but say nothing; that must not read as a pass.
    report = analysis.analyze([row(pose_frame="", path_frame="")])
    assert not report["path_frame_consistency"]["passed"]


def test_a_stale_transform_is_flagged_rather_than_used_silently():
    report = analysis.analyze([row(transform_stamp=99.0)])
    assert report["timestamp_consistency"]["stale_transform_rows"] == 1
    assert report["timestamp_consistency"]["tolerance_s"] == analysis.TRANSFORM_AGE_TOLERANCE_S


def test_the_age_of_a_frozen_path_is_reported():
    report = analysis.analyze([row(update_stamp=140.0, path_stamp=60.0)])
    assert report["timestamp_consistency"]["max_path_age_s"] == pytest.approx(80.0)
