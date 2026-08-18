"""Tests for the pure capture-plane geometry model (shadow / offline only)."""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"))

from tennis_robot.collection_capture_geometry import (
    CAPTURE_FRAME,
    CaptureGeometry,
    CaptureGeometryError,
    CapturePlane,
    INTAKE_MOUTH_PLANE_ID,
    PlaneProvenance,
    repo_base_footprint_capture_geometry,
    repo_capture_planes,
)


def _calibrated_geometry() -> CaptureGeometry:
    return repo_base_footprint_capture_geometry(
        required_pre_contact_straight_m=0.1,
        required_pre_contact_provenance=PlaneProvenance.MEASURED,
    )


# ── Test 1: validation + deterministic round-trip ────────────────────────────
def test_repo_geometry_round_trips_deterministically():
    geometry = _calibrated_geometry()
    first = geometry.to_dict()
    restored = CaptureGeometry.from_dict(first)
    assert restored == geometry
    # Serialization is deterministic and idempotent.
    assert restored.to_dict() == first
    assert CaptureGeometry.from_dict(restored.to_dict()) == geometry


def test_plane_round_trips():
    plane = repo_capture_planes()[0]
    assert CapturePlane.from_dict(plane.to_dict()) == plane


def test_repo_planes_carry_cited_values_and_provenance():
    planes = {plane.plane_id: plane for plane in repo_capture_planes()}
    assert planes[INTAKE_MOUTH_PLANE_ID].longitudinal_offset_m == 0.876
    assert planes["roller_nip"].longitudinal_offset_m == 0.540
    for plane in planes.values():
        assert plane.source  # every value cites a source


def test_repo_configuration_values_are_not_labelled_measured():
    # URDF/xacro/env configuration is a design intent, not a physical
    # measurement: none of the packaged planes may claim MEASURED provenance.
    for plane in repo_capture_planes():
        assert plane.provenance is PlaneProvenance.CONFIGURED, plane.plane_id


def test_minimum_alignment_corridor_is_reference_offset_plus_pre_contact():
    geometry = repo_base_footprint_capture_geometry(
        required_pre_contact_straight_m=0.2,
        required_pre_contact_provenance=PlaneProvenance.CONFIGURED,
    )
    assert geometry.minimum_alignment_corridor_m == pytest.approx(0.876 + 0.2)


def test_frame_must_be_base_footprint():
    with pytest.raises(CaptureGeometryError):
        CaptureGeometry(
            "map",
            repo_capture_planes(),
            INTAKE_MOUTH_PLANE_ID,
            0.0,
            PlaneProvenance.MEASURED,
        )
    assert _calibrated_geometry().frame == CAPTURE_FRAME


# ── Test 4: uncalibrated required geometry is surfaced (never hidden) ─────────
# The fail-loud enforcement (adaptive generation refuses to run on uncalibrated
# geometry) is tested in test_collection_adaptive_approach.py; here we only
# assert the geometry model surfaces the uncalibrated fields honestly.
def test_uncalibrated_pre_contact_is_surfaced_not_hidden():
    geometry = repo_base_footprint_capture_geometry(
        required_pre_contact_straight_m=0.0,
        required_pre_contact_provenance=PlaneProvenance.UNCALIBRATED,
    )
    assert geometry.is_calibrated is False
    assert "required_pre_contact_straight_m" in geometry.uncalibrated_fields()


def test_uncalibrated_plane_is_surfaced():
    planes = repo_capture_planes() + (
        CapturePlane("trial_reference", 0.9, 0.1, PlaneProvenance.UNCALIBRATED, "pending intake trial"),
    )
    geometry = CaptureGeometry(
        CAPTURE_FRAME, planes, INTAKE_MOUTH_PLANE_ID, 0.1, PlaneProvenance.MEASURED
    )
    assert geometry.is_calibrated is False
    assert "plane:trial_reference" in geometry.uncalibrated_fields()


def test_fully_calibrated_geometry_reports_calibrated():
    assert _calibrated_geometry().is_calibrated is True
    assert _calibrated_geometry().uncalibrated_fields() == ()


def test_capture_reference_plane_id_must_exist():
    with pytest.raises(CaptureGeometryError):
        CaptureGeometry(
            CAPTURE_FRAME, repo_capture_planes(), "no_such_plane", 0.0, PlaneProvenance.MEASURED
        )


def test_negative_offset_and_bad_provenance_are_rejected():
    with pytest.raises(CaptureGeometryError):
        CapturePlane("p", -0.1, 0.1, PlaneProvenance.MEASURED, "src")
    with pytest.raises(CaptureGeometryError):
        CapturePlane.from_dict(
            {"plane_id": "p", "longitudinal_offset_m": 0.5, "half_width_m": 0.1, "provenance": "guessed", "source": "s"}
        )


def test_duplicate_plane_ids_rejected():
    plane = repo_capture_planes()[0]
    with pytest.raises(CaptureGeometryError):
        CaptureGeometry(
            CAPTURE_FRAME, (plane, plane), plane.plane_id, 0.0, PlaneProvenance.MEASURED
        )


def test_from_dict_rejects_extra_or_missing_fields():
    data = _calibrated_geometry().to_dict()
    data["surprise"] = 1
    with pytest.raises(CaptureGeometryError):
        CaptureGeometry.from_dict(data)
    del data["surprise"]
    del data["frame"]
    with pytest.raises(CaptureGeometryError):
        CaptureGeometry.from_dict(data)
