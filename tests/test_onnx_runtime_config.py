"""ROS-independent resource-bound tests for neural perception sessions."""

import os
import sys
import types

import pytest

sys.path.insert(
    0,
    os.path.join(
        os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"
    ),
)

from tennis_robot.onnx_runtime_config import create_cpu_inference_session


class _Options:
    def __init__(self):
        self.entries = {}

    def add_session_config_entry(self, name, value):
        self.entries[name] = value


class _Ort:
    SessionOptions = _Options
    ExecutionMode = types.SimpleNamespace(ORT_SEQUENTIAL="sequential")
    GraphOptimizationLevel = types.SimpleNamespace(ORT_ENABLE_ALL="all")

    def __init__(self):
        self.call = None

    def InferenceSession(self, path, **kwargs):
        self.call = (path, kwargs)
        return "session"


def test_session_has_bounded_sequential_cpu_pools(monkeypatch):
    monkeypatch.setenv("PERCEPTION_ONNX_INTRA_OP_THREADS", "3")
    monkeypatch.setenv("PERCEPTION_ONNX_INTER_OP_THREADS", "1")
    ort = _Ort()

    assert create_cpu_inference_session(ort, "model.onnx") == "session"

    path, kwargs = ort.call
    options = kwargs["sess_options"]
    assert path == "model.onnx"
    assert kwargs["providers"] == ["CPUExecutionProvider"]
    assert options.intra_op_num_threads == 3
    assert options.inter_op_num_threads == 1
    assert options.execution_mode == "sequential"
    assert options.graph_optimization_level == "all"
    assert options.entries == {
        "session.intra_op.allow_spinning": "0",
        "session.inter_op.allow_spinning": "0",
    }


@pytest.mark.parametrize("value", ["0", "-1", "not-an-int"])
def test_invalid_thread_bound_fails_loudly(monkeypatch, value):
    monkeypatch.setenv("PERCEPTION_ONNX_INTRA_OP_THREADS", value)
    with pytest.raises(ValueError, match="PERCEPTION_ONNX_INTRA_OP_THREADS"):
        create_cpu_inference_session(_Ort(), "model.onnx")
