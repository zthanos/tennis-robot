"""Bounded ONNX Runtime CPU configuration shared by neural perception.

ONNX Runtime otherwise sizes a thread pool from the host CPU topology for
every session.  Simulation loads two models in one process, so leaving both
sessions at that default can consume the whole workstation and starve Gazebo,
the UI, and ROS networking.
"""

from __future__ import annotations

import os


DEFAULT_INTRA_OP_THREADS = 4
DEFAULT_INTER_OP_THREADS = 1


def _positive_thread_count(name: str, default: int) -> int:
    raw = os.getenv(name, str(default))
    try:
        value = int(raw)
    except ValueError as exc:
        raise ValueError(f"{name} must be a positive integer, got {raw!r}") from exc
    if value <= 0:
        raise ValueError(f"{name} must be a positive integer, got {raw!r}")
    return value


def create_cpu_inference_session(ort, model_path: str, *, providers=None):
    """Create a deterministic, explicitly bounded CPU inference session."""

    intra_op_threads = _positive_thread_count(
        "PERCEPTION_ONNX_INTRA_OP_THREADS", DEFAULT_INTRA_OP_THREADS
    )
    inter_op_threads = _positive_thread_count(
        "PERCEPTION_ONNX_INTER_OP_THREADS", DEFAULT_INTER_OP_THREADS
    )

    options = ort.SessionOptions()
    options.intra_op_num_threads = intra_op_threads
    options.inter_op_num_threads = inter_op_threads
    # The two detectors are called serially from one synchronized ROS callback.
    # A parallel graph executor adds another pool without useful concurrency.
    options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
    options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    # Avoid busy-spinning worker threads between camera frames.  Older ORT
    # versions may not expose this API, while the thread bounds above remain.
    add_entry = getattr(options, "add_session_config_entry", None)
    if add_entry is not None:
        add_entry("session.intra_op.allow_spinning", "0")
        add_entry("session.inter_op.allow_spinning", "0")

    return ort.InferenceSession(
        model_path,
        sess_options=options,
        providers=providers or ["CPUExecutionProvider"],
    )


def configured_thread_counts() -> tuple[int, int]:
    """Return effective intra/inter settings for startup diagnostics."""

    return (
        _positive_thread_count(
            "PERCEPTION_ONNX_INTRA_OP_THREADS", DEFAULT_INTRA_OP_THREADS
        ),
        _positive_thread_count(
            "PERCEPTION_ONNX_INTER_OP_THREADS", DEFAULT_INTER_OP_THREADS
        ),
    )
