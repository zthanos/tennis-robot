"""UI snapshot work must remain bounded independently of sensor rates."""

import os
import sys
from types import SimpleNamespace

sys.path.insert(
    0,
    os.path.join(
        os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"
    ),
)

from tennis_robot import sensor_snapshot_node as snapshot


def test_preview_processing_is_rate_limited_per_stream(monkeypatch):
    state = SimpleNamespace(
        _last_preview_process_s={
            "image": float("-inf"),
            "depth": float("-inf"),
            "scan": float("-inf"),
        }
    )
    now = iter([10.0, 10.1, 10.2, 10.3, 10.0 + snapshot.WRITE_INTERVAL_S])
    monkeypatch.setattr(snapshot.time, "monotonic", lambda: next(now))

    assert snapshot.SensorSnapshotNode._preview_processing_due(state, "image")
    assert not snapshot.SensorSnapshotNode._preview_processing_due(state, "image")
    assert snapshot.SensorSnapshotNode._preview_processing_due(state, "depth")
    assert snapshot.SensorSnapshotNode._preview_processing_due(state, "scan")
    assert snapshot.SensorSnapshotNode._preview_processing_due(state, "image")
