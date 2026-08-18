"""Offline tests for the simulated OAK-D neural detector + depth fusion.

These run without ROS, without a real model file, and without onnxruntime by
injecting a fake InferenceSession — so they validate the *pipeline logic*
(letterbox, YOLOv8 decode, class filter, NMS, bearing/depth fusion)
deterministically in CI.
"""

import math
import os
import sys
import types

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"))

from tennis_robot import ball_detector as bd
from tennis_robot.perception import (
    BallDetection,
    camera_frame_position,
    estimate_depth_ball_observation,
    pixel_elevation_rad,
)

NUM_CLASSES = 80
SPORTS_BALL = bd.DEFAULT_SPORTS_BALL_CLASS_ID  # 32


def _yolo_column(cx, cy, w, h, class_id, score):
    col = np.zeros(4 + NUM_CLASSES, dtype=np.float32)
    col[:4] = (cx, cy, w, h)
    col[4 + class_id] = score
    return col


def _pad(columns, n_anchors=100):
    """Pad to a realistic anchor count (N >> num_classes), like a real YOLOv8
    head which always emits a fixed grid (e.g. 8400) regardless of detections.
    Filler columns are all-zero => below threshold => dropped."""
    fillers = [np.zeros(4 + NUM_CLASSES, dtype=np.float32)
               for _ in range(max(0, n_anchors - len(columns)))]
    return list(columns) + fillers


class _FakeInput:
    name = "images"
    shape = [1, 3, 640, 640]


class _FakeSession:
    """Returns a fixed YOLOv8-style (1, 84, N) output regardless of input."""

    def __init__(self, output):
        self._output = output

    def get_inputs(self):
        return [_FakeInput()]

    def run(self, _outputs, _feed):
        return [self._output]


class _FakeSessionOptions:
    def add_session_config_entry(self, _name, _value):
        pass


def _fake_ort(output):
    return types.SimpleNamespace(
        SessionOptions=_FakeSessionOptions,
        ExecutionMode=types.SimpleNamespace(ORT_SEQUENTIAL="sequential"),
        GraphOptimizationLevel=types.SimpleNamespace(ORT_ENABLE_ALL="all"),
        InferenceSession=lambda *a, **k: _FakeSession(output),
    )


def _make_detector(monkeypatch, columns, **kwargs):
    """Build a YoloOnnxBallDetector backed by a fake onnxruntime session."""
    output = np.stack(_pad(columns), axis=1)[None]  # (1, 84, N)
    monkeypatch.setitem(sys.modules, "onnxruntime", _fake_ort(output))
    return bd.YoloOnnxBallDetector("dummy.onnx", **kwargs)


# ---------------------------------------------------------------------------
# NMS
# ---------------------------------------------------------------------------
def test_nms_suppresses_overlap_keeps_best():
    boxes = np.array([
        [100, 100, 140, 140],   # A
        [104, 102, 144, 142],   # B (≈A, lower score) -> suppressed
        [300, 300, 320, 320],   # C (disjoint) -> kept
    ], dtype=np.float32)
    scores = np.array([0.9, 0.8, 0.7], dtype=np.float32)
    keep = bd._nms(boxes, scores, iou_threshold=0.45)
    assert keep == [0, 2]


# ---------------------------------------------------------------------------
# YOLO decode end-to-end through detect()
# ---------------------------------------------------------------------------
def test_detect_filters_class_threshold_and_nms(monkeypatch):
    cols = [
        _yolo_column(320, 320, 40, 40, SPORTS_BALL, 0.90),  # keep
        _yolo_column(326, 322, 40, 40, SPORTS_BALL, 0.80),  # overlaps -> NMS drop
        _yolo_column(100, 100, 20, 20, 0, 0.95),            # person -> class filtered
        _yolo_column(500, 400, 30, 30, SPORTS_BALL, 0.10),  # below conf -> drop
    ]
    det = _make_detector(monkeypatch, cols, conf_threshold=0.35, iou_threshold=0.45)
    # 640x640 frame => letterbox scale=1, pad=0 => model coords == image coords.
    frame = np.zeros((640, 640, 3), dtype=np.uint8)
    out = det.detect(frame)

    assert len(out) == 1
    d = out[0]
    assert isinstance(d, BallDetection)
    assert d.label == "tennis_ball"
    assert d.confidence == pytest.approx(0.90, abs=1e-5)
    assert d.center_x == pytest.approx(320, abs=1.0)
    assert d.center_y == pytest.approx(320, abs=1.0)
    assert d.width == pytest.approx(40, abs=1.0)


def test_detect_handles_transposed_output(monkeypatch):
    # Some exports give (1, N, 84) instead of (1, 84, N); detect must cope.
    cols = _pad([_yolo_column(320, 240, 30, 30, SPORTS_BALL, 0.8)])
    output = np.stack(cols, axis=0)[None]  # (1, N, 84)
    monkeypatch.setitem(sys.modules, "onnxruntime", _fake_ort(output))
    det = bd.YoloOnnxBallDetector("dummy.onnx", conf_threshold=0.35)
    out = det.detect(np.zeros((640, 640, 3), np.uint8))
    assert len(out) == 1
    assert out[0].center_x == pytest.approx(320, abs=1.0)


def test_detect_maps_boxes_back_through_letterbox(monkeypatch):
    # Non-square frame: a centred model-space box must land at the image centre.
    cols = [_yolo_column(320, 320, 64, 64, SPORTS_BALL, 0.9)]
    det = _make_detector(monkeypatch, cols, conf_threshold=0.35)
    frame = np.zeros((360, 640, 3), dtype=np.uint8)  # 16:9
    out = det.detect(frame)
    assert len(out) == 1
    assert out[0].center_x == pytest.approx(320, abs=2.0)
    assert out[0].center_y == pytest.approx(180, abs=2.0)


def test_center_zoom_maps_detection_back_and_merges_full_frame_duplicate(monkeypatch):
    # The fake model returns a centered 40x40 box for both passes.  On a
    # 640x480 source, the zoom crop is 320x240 and its mapped box is therefore
    # smaller in source pixels but remains centered.  Global NMS keeps one.
    cols = [_yolo_column(320, 320, 40, 40, SPORTS_BALL, 0.9)]
    det = _make_detector(
        monkeypatch, cols, conf_threshold=0.35, center_zoom_factor=2.0
    )
    out = det.detect(np.zeros((480, 640, 3), np.uint8))
    assert len(out) == 1
    assert out[0].center_x == pytest.approx(320, abs=2.0)
    assert out[0].center_y == pytest.approx(240, abs=2.0)


def test_invalid_center_zoom_is_rejected(monkeypatch):
    with pytest.raises(ValueError, match="center_zoom_factor"):
        _make_detector(monkeypatch, [], center_zoom_factor=0.5)


def test_zoom_tile_maps_detection_to_configured_centre(monkeypatch):
    cols = [_yolo_column(320, 320, 40, 40, SPORTS_BALL, 0.9)]
    det = _make_detector(
        monkeypatch,
        cols,
        conf_threshold=0.35,
        center_zoom_factor=3.0,
        zoom_tiles=((0.3, 1.0 / 3.0),),
    )
    out = det.detect(np.zeros((480, 640, 3), np.uint8))
    assert any(d.center_x == pytest.approx(192, abs=2.0) for d in out)
    assert any(d.center_y == pytest.approx(160, abs=2.0) for d in out)


# ---------------------------------------------------------------------------
# Factory startup contract
# ---------------------------------------------------------------------------
def test_factory_rejects_non_neural_backend():
    with pytest.raises(ValueError, match="only 'yolo_onnx' is allowed"):
        bd.load_ball_detector(backend="hsv")


def test_factory_missing_model_fails_loudly():
    with pytest.raises(FileNotFoundError, match="YOLO model not found"):
        bd.load_ball_detector(backend="yolo_onnx", model_path="/no/such/model.onnx")


# ---------------------------------------------------------------------------
# Depth fusion / spatial geometry
# ---------------------------------------------------------------------------
def test_depth_fusion_centre_ball_zero_bearing():
    det = BallDetection(310, 230, 20, 20, confidence=0.8)  # centred in 640x480
    depth = np.full((480, 640), 3.0, dtype=np.float32)
    obs = estimate_depth_ball_observation(det, depth, 640, 480, math.radians(69))
    assert obs is not None
    assert obs.bearing_rad == pytest.approx(0.0, abs=1e-3)
    assert obs.distance_m == pytest.approx(3.0, abs=1e-3)
    assert obs.distance_source == "oak_depth"


def test_depth_fusion_right_ball_negative_bearing():
    det = BallDetection(560, 230, 20, 20, confidence=0.8)  # right of centre
    depth = np.full((480, 640), 2.0, dtype=np.float32)
    obs = estimate_depth_ball_observation(det, depth, 640, 480, math.radians(69))
    assert obs is not None
    assert obs.bearing_rad < -0.05  # canonical bearing is +left / CCW


def test_depth_fusion_converts_optical_z_to_slant_range():
    detection = BallDetection(550, 90, 20, 20, confidence=0.8)
    depth = np.full((480, 640), 5.0, dtype=np.float32)
    obs = estimate_depth_ball_observation(
        detection, depth, 640, 480, math.radians(69)
    )
    assert obs is not None
    elevation = pixel_elevation_rad(
        detection.center_y, 480, math.radians(69) * 480 / 640
    )
    expected = 5.0 * math.sqrt(
        1.0 + math.tan(obs.bearing_rad) ** 2 + math.tan(elevation) ** 2
    )
    assert obs.distance_m == pytest.approx(expected)
    right, down, forward = camera_frame_position(
        obs.bearing_rad, obs.distance_m, elevation
    )
    assert forward == pytest.approx(5.0)
    assert right == pytest.approx(-5.0 * math.tan(obs.bearing_rad))
    assert down == pytest.approx(-5.0 * math.tan(elevation))


def test_depth_fusion_rejects_minority_foreground_occluder():
    detection = BallDetection(318, 238, 4, 4, confidence=0.8)
    depth = np.full((480, 640), 8.0, dtype=np.float32)
    # The 3x3 fusion ROI contains one foreground net strand (three pixels) and
    # six ball pixels.  The old 20th-percentile-only estimator returned ~4.7 m
    # and incorrectly moved the far-side ball in front of the net.
    depth[239:242, 319] = 4.7
    assert (
        estimate_depth_ball_observation(
            detection, depth, 640, 480, math.radians(69)
        )
        is None
    )


def test_depth_fusion_keeps_dominant_near_ball_against_far_background():
    detection = BallDetection(318, 238, 4, 4, confidence=0.8)
    depth = np.full((480, 640), 8.0, dtype=np.float32)
    # Six of the nine central ROI pixels belong to the ball.  The median and
    # foreground estimate agree, so farther background must not suppress it.
    depth[239:242, 319:321] = 4.7
    observation = estimate_depth_ball_observation(
        detection, depth, 640, 480, math.radians(69)
    )
    assert observation is not None
    assert observation.distance_m == pytest.approx(4.7)


def test_camera_frame_position_conventions():
    # Ball 2 m away, 30 degrees to the left, level.
    right, down, forward = camera_frame_position(math.radians(30), 2.0, 0.0)
    assert right < 0  # optical +X points right
    assert down == pytest.approx(0.0, abs=1e-6)
    assert forward == pytest.approx(2.0 * math.cos(math.radians(30)), abs=1e-6)


def test_pixel_elevation_sign():
    # A row above centre => looking up => positive elevation.
    assert pixel_elevation_rad(100, 480, math.radians(50)) > 0
    assert pixel_elevation_rad(380, 480, math.radians(50)) < 0
