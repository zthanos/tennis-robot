"""ROS-independent tests for neural net/fence detection and depth fusion."""

import os
import importlib.util
from pathlib import Path
import sys
import types

import numpy as np
import pytest

sys.path.insert(
    0,
    os.path.join(
        os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"
    ),
)

from tennis_robot import court_scene_detector as csd

_CAPTURE_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "capture_court_scene_dataset.py"
)
_CAPTURE_SPEC = importlib.util.spec_from_file_location(
    "capture_court_scene_dataset", _CAPTURE_PATH
)
capture = importlib.util.module_from_spec(_CAPTURE_SPEC)
assert _CAPTURE_SPEC.loader is not None
_CAPTURE_SPEC.loader.exec_module(capture)

_TRAIN_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "train_court_scene_yolo.py"
)
_TRAIN_SPEC = importlib.util.spec_from_file_location(
    "train_court_scene_yolo", _TRAIN_PATH
)
train = importlib.util.module_from_spec(_TRAIN_SPEC)
assert _TRAIN_SPEC.loader is not None
_TRAIN_SPEC.loader.exec_module(train)


def _column(cx, cy, width, height, net_score=0.0, fence_score=0.0):
    return np.asarray(
        [cx, cy, width, height, net_score, fence_score], dtype=np.float32
    )


class _FakeInput:
    name = "images"
    shape = [1, 3, 640, 640]


class _FakeSession:
    def __init__(self, output):
        self.output = output

    def get_inputs(self):
        return [_FakeInput()]

    def run(self, _outputs, _feed):
        return [self.output]


def _detector(monkeypatch, columns, **kwargs):
    fillers = [
        np.zeros(6, dtype=np.float32)
        for _ in range(max(0, 100 - len(columns)))
    ]
    output = np.stack(list(columns) + fillers, axis=1)[None]
    fake_ort = types.SimpleNamespace(
        InferenceSession=lambda *args, **kw: _FakeSession(output)
    )
    monkeypatch.setitem(sys.modules, "onnxruntime", fake_ort)
    return csd.YoloOnnxCourtSceneDetector("dummy.onnx", **kwargs)


def test_detect_keeps_overlapping_net_and_fence_hypotheses(monkeypatch):
    detector = _detector(
        monkeypatch,
        [
            _column(320, 320, 400, 260, net_score=0.92),
            _column(320, 300, 420, 280, fence_score=0.88),
            _column(325, 322, 400, 260, net_score=0.70),
        ],
    )
    detections = detector.detect(np.zeros((640, 640, 3), dtype=np.uint8))
    assert [d.label for d in detections] == ["net", "fence"]
    assert detections[0].confidence == pytest.approx(0.92)
    assert detections[1].confidence == pytest.approx(0.88)


def test_detect_maps_letterboxed_box_to_source_frame(monkeypatch):
    detector = _detector(
        monkeypatch, [_column(320, 320, 320, 240, net_score=0.9)]
    )
    detection = detector.detect(np.zeros((480, 640, 3), dtype=np.uint8))[0]
    assert detection.center_x == pytest.approx(320, abs=2)
    assert detection.center_y == pytest.approx(240, abs=2)


def test_depth_fusion_uses_timestamp_matched_foreground_percentile():
    detection = csd.CourtSceneDetection(
        x=120,
        y=100,
        width=400,
        height=240,
        confidence=0.9,
        label="net",
        class_id=0,
    )
    depth = np.full((480, 640), 8.0, dtype=np.float32)
    # Sparse foreground net depths inside a farther fence/background ROI.
    depth[180:300:2, 200:440:2] = 2.0
    observations = csd.fuse_court_scene_detections(
        [detection], depth, 640, 480, 1.204
    )
    assert len(observations) == 1
    assert observations[0].distance_m == pytest.approx(2.0)
    assert observations[0].bearing_rad == pytest.approx(0.0, abs=1e-6)


def test_primary_observation_prefers_forward_corridor_and_nearest_depth():
    net = csd.CourtSceneObservation(
        csd.CourtSceneDetection(120, 100, 400, 250, 0.8, "net", 0),
        distance_m=2.0,
        bearing_rad=0.0,
        valid_depth_count=100,
    )
    side_fence = csd.CourtSceneObservation(
        csd.CourtSceneDetection(0, 80, 100, 300, 0.99, "fence", 1),
        distance_m=0.5,
        bearing_rad=0.5,
        valid_depth_count=100,
    )
    assert csd.select_primary_observation([side_fence, net], 640) == net


def test_primary_observation_prefers_overlapping_codepth_net():
    fence = csd.CourtSceneObservation(
        csd.CourtSceneDetection(128, 21, 374, 97, 0.83, "fence", 1),
        distance_m=7.06,
        bearing_rad=-0.01,
        valid_depth_count=2100,
    )
    net = csd.CourtSceneObservation(
        csd.CourtSceneDetection(0, 75, 639, 70, 0.89, "net", 0),
        distance_m=7.12,
        bearing_rad=0.0,
        valid_depth_count=4263,
    )
    assert csd.select_primary_observation([fence, net], 640) == net


def test_primary_observation_keeps_genuinely_closer_fence():
    fence = csd.CourtSceneObservation(
        csd.CourtSceneDetection(128, 21, 374, 160, 0.83, "fence", 1),
        distance_m=2.0,
        bearing_rad=0.0,
        valid_depth_count=2100,
    )
    net = csd.CourtSceneObservation(
        csd.CourtSceneDetection(0, 75, 639, 100, 0.89, "net", 0),
        distance_m=3.0,
        bearing_rad=0.0,
        valid_depth_count=4263,
    )
    assert csd.select_primary_observation([fence, net], 640) == fence


def test_semantic_confirmation_requires_consecutive_frames_and_expires():
    confirmation = csd.SemanticConfirmation(required_frames=3)
    assert confirmation.update("net") is None
    assert confirmation.update("net") is None
    assert confirmation.update("net") == "net"
    assert confirmation.update(None) is None
    assert confirmation.update("fence") is None


def test_factory_rejects_classical_backend():
    with pytest.raises(ValueError, match="only 'yolo_onnx' is allowed"):
        csd.load_court_scene_detector(backend="opencv")


def test_factory_missing_model_fails_loudly():
    with pytest.raises(FileNotFoundError, match="court-scene YOLO model not found"):
        csd.load_court_scene_detector(model_path="/no/such/court_scene.onnx")


def test_class_map_requires_exact_net_and_fence_classes():
    assert csd._parse_class_labels("4:net,7:fence") == {4: "net", 7: "fence"}
    with pytest.raises(ValueError, match="exactly one net and one fence"):
        csd._parse_class_labels("0:net")


def test_dataset_projection_produces_normalized_visible_box():
    points = [
        np.asarray([-1.0, -0.5, 5.0]),
        np.asarray([1.0, -0.5, 5.0]),
        np.asarray([1.0, 0.5, 5.0]),
        np.asarray([-1.0, 0.5, 5.0]),
    ]
    box = capture.project_surface_box(
        points, np.eye(3), np.zeros(3), 640, 480, 1.204
    )
    assert box is not None
    center_x, center_y, width, height = box
    assert center_x == pytest.approx(0.5)
    assert center_y == pytest.approx(0.5)
    assert 0.0 < width < 1.0
    assert 0.0 < height < 1.0


def test_dataset_projection_clips_plane_crossing_behind_camera():
    points = [
        np.asarray([-1.0, -0.5, -1.0]),
        np.asarray([1.0, -0.5, 5.0]),
        np.asarray([1.0, 0.5, 5.0]),
        np.asarray([-1.0, 0.5, -1.0]),
    ]
    box = capture.project_surface_box(
        points, np.eye(3), np.zeros(3), 640, 480, 1.204
    )
    assert box is not None
    center_x, center_y, width, height = box
    assert 0.0 <= center_x <= 1.0
    assert 0.0 <= center_y <= 1.0
    assert 0.0 < width <= 1.0
    assert 0.0 < height <= 1.0


def test_dataset_projection_rejects_plane_fully_behind_camera():
    points = [
        np.asarray([-1.0, -0.5, -2.0]),
        np.asarray([1.0, -0.5, -1.0]),
        np.asarray([1.0, 0.5, -1.0]),
        np.asarray([-1.0, 0.5, -2.0]),
    ]
    assert (
        capture.project_surface_box(
            points, np.eye(3), np.zeros(3), 640, 480, 1.204
        )
        is None
    )


def test_training_dataset_root_is_relative_to_yaml(tmp_path):
    config_path = tmp_path / "court_scene" / "court_scene.yaml"
    config_path.parent.mkdir()
    assert train._dataset_root(config_path, {"path": "."}) == config_path.parent
