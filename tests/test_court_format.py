from tennis_robot.court_format import (
    CameraCornerEvidence,
    add_distinct_corner,
    camera_corner_to_map,
    estimate_court_format,
)


FRAME = {
    "net_center_x_m": 0.0,
    "net_center_y_m": 0.0,
    "length_axis_x": 1.0,
    "length_axis_y": 0.0,
    "width_axis_x": 0.0,
    "width_axis_y": 1.0,
}


def test_doubles_corners_are_classified_from_camera_evidence():
    points = [
        CameraCornerEvidence(11.9, 5.48, 0.9),
        CameraCornerEvidence(11.8, -5.50, 0.8),
        CameraCornerEvidence(-11.9, 5.46, 0.9),
    ]
    result = estimate_court_format(points, **FRAME)
    assert result["label"] == "doubles"
    assert result["confidence"] >= 0.55
    assert result["affects_navigation"] is False


def test_singles_corners_are_classified_from_camera_evidence():
    points = [
        CameraCornerEvidence(11.9, 4.12, 0.9),
        CameraCornerEvidence(11.8, -4.10, 0.8),
        CameraCornerEvidence(-11.9, 4.08, 0.9),
    ]
    result = estimate_court_format(points, **FRAME)
    assert result["label"] == "singles"
    assert result["confidence"] >= 0.55


def test_sparse_or_non_baseline_evidence_remains_unknown():
    sparse = estimate_court_format(
        [CameraCornerEvidence(11.9, 4.12, 1.0)], **FRAME
    )
    middle = estimate_court_format(
        [
            CameraCornerEvidence(4.0, 4.12, 1.0),
            CameraCornerEvidence(5.0, -4.12, 1.0),
        ],
        **FRAME,
    )
    assert sparse["label"] == "unknown"
    assert middle["label"] == "unknown"


def test_camera_projection_and_deduplication():
    projected = camera_corner_to_map(
        robot_x_m=1.0,
        robot_y_m=2.0,
        robot_yaw_rad=0.0,
        bearing_rad=0.0,
        distance_m=3.0,
        camera_x_m=0.5,
    )
    assert projected is not None
    assert projected.map_x_m == 4.5
    assert projected.map_y_m == 2.0

    evidence = []
    add_distinct_corner(evidence, CameraCornerEvidence(1.0, 1.0, 0.4))
    add_distinct_corner(evidence, CameraCornerEvidence(1.1, 1.1, 0.9))
    assert len(evidence) == 1
    assert evidence[0].confidence == 0.9
