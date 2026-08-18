"""The recorder must be cheap, bounded, and invisible when switched off."""

import json
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"))

from tennis_robot.collection_execution_recorder import (  # noqa: E402
    ExecutionTraceCapture,
    capture_from_env,
)
from tennis_robot.collection_execution_trace import (  # noqa: E402
    ExecutionTrace,
    ExecutionTraceError,
    ExecutionTraceRecorder,
    TrajectorySample,
)
from tennis_robot.collection_route_types import Pose2D  # noqa: E402


class Plan:
    plan_id = "route-abc"
    scan_id = "scan-1"


class State:
    def __init__(self, **values):
        self.__dict__.update(values)


class Clock:
    def __init__(self):
        self.value = 0.0

    def __call__(self):
        self.value += 0.1
        return round(self.value, 6)


def test_absent_environment_means_no_recorder_at_all(monkeypatch):
    monkeypatch.delenv("COLLECTION_EXECUTION_TRACE_DIR", raising=False)
    assert capture_from_env(run_id="r", clock_fn=lambda: 0.0) is None


def test_capture_writes_a_readable_trace(tmp_path):
    capture = ExecutionTraceCapture(
        directory=tmp_path, run_id="run-7", clock_fn=Clock(), spacing_m=0.0, interval_s=0.0
    )
    capture.start(Plan())
    for index in range(5):
        capture.record_state(
            pose=Pose2D(float(index), 0.0, 0.0),
            state=State(
                active_segment_id="pass-0", progress_s=float(index),
                measured_speed_mps=0.35, has_active_crossing=index == 2,
                active_ball_id="b1", active_crossing_progress_s=2.0,
                lateral_error_m=0.01, heading_error_rad=0.02,
            ),
        )
    capture.record_beams(entry=True, confirmed=False)
    capture.record_beams(entry=True, confirmed=True)
    capture.record_observation(ball_id="b2", x_m=3.0, y_m=1.0)
    target = capture.finish()
    assert target is not None and target.exists()

    trace = ExecutionTrace.from_dict(json.loads(target.read_text()))
    assert trace.run_id == "run-7" and trace.plan_id == "route-abc"
    assert len(trace.samples) == 5
    assert [item.ball_id for item in trace.crossings] == ["b1"]
    assert [(item.beam, item.rising) for item in trace.beams] == [
        ("entry", True), ("confirmed", True)
    ]
    assert [item.ball_id for item in trace.observations] == ["b2"]


def test_a_stopped_capture_ignores_everything(tmp_path):
    capture = ExecutionTraceCapture(directory=tmp_path, run_id="r", clock_fn=Clock())
    # Never started: every call is a no-op and nothing is written.
    capture.record_state(pose=Pose2D(0.0, 0.0, 0.0), state=State())
    capture.record_beams(entry=True, confirmed=True)
    capture.record_observation(ball_id="b", x_m=0.0, y_m=0.0)
    assert capture.finish() is None
    assert not list(tmp_path.iterdir())


def test_decimation_keeps_the_row_count_proportional_to_distance():
    recorder = ExecutionTraceRecorder(
        run_id="r", plan_id="p", scan_id="s",
        minimum_spacing_m=0.10, minimum_interval_s=0.5, maximum_samples=10000,
    )
    # 40 m driven at 0.35 m/s, sampled at 20 Hz: 2286 raw ticks.
    kept = 0
    ticks = 2286
    for index in range(ticks):
        moment = index * 0.05
        if recorder.record_pose(
            TrajectorySample(moment, 0.35 * moment, 0.0, 0.0, 0.35, 0.0, "pass-0", 0.35 * moment)
        ):
            kept += 1
    assert kept == pytest.approx(40.0 / 0.10, rel=0.05)
    assert recorder.dropped_samples == ticks - kept
    # Roughly 400 rows for a 40 m route: about 1.4% of the raw tick rate.
    assert kept < ticks * 0.20


def test_a_slow_turn_is_still_sampled_in_time():
    recorder = ExecutionTraceRecorder(
        run_id="r", plan_id="p", scan_id="s",
        minimum_spacing_m=0.10, minimum_interval_s=0.5, maximum_samples=100,
    )
    # Rotating in place: no distance at all, so only the time bound applies.
    kept = sum(
        1 for index in range(100)
        if recorder.record_pose(
            TrajectorySample(index * 0.05, 0.0, 0.0, index * 0.01, 0.0, 0.2)
        )
    )
    assert kept == pytest.approx(5.0 / 0.5, abs=1)


def test_the_sample_cap_is_honoured_and_counted():
    recorder = ExecutionTraceRecorder(
        run_id="r", plan_id="p", scan_id="s",
        minimum_spacing_m=0.0, minimum_interval_s=0.0, maximum_samples=10,
    )
    for index in range(25):
        recorder.record_pose(TrajectorySample(float(index), float(index), 0.0, 0.0, 0.3, 0.0))
    trace = recorder.build()
    assert len(trace.samples) == 10
    assert recorder.dropped_samples == 15


def test_beam_edges_only():
    recorder = ExecutionTraceRecorder(
        run_id="r", plan_id="p", scan_id="s",
        minimum_spacing_m=0.0, minimum_interval_s=0.0, maximum_samples=100,
    )
    for level in (False, False, True, True, True, False, False, True):
        recorder.record_beam(t_s=0.0, beam="entry", level=level)
    trace = recorder.build()
    assert [item.rising for item in trace.beams] == [True, False, True]


def test_trace_round_trips_and_rejects_nonsense():
    recorder = ExecutionTraceRecorder(
        run_id="r", plan_id="p", scan_id="s",
        minimum_spacing_m=0.0, minimum_interval_s=0.0, maximum_samples=10,
    )
    recorder.record_pose(TrajectorySample(0.0, 1.0, 2.0, 0.3, 0.35, 0.0, "pass-0", 1.0))
    trace = recorder.build()
    assert ExecutionTrace.from_dict(trace.to_dict()) == trace
    with pytest.raises(ExecutionTraceError):
        TrajectorySample(float("nan"), 0.0, 0.0, 0.0, 0.0, 0.0)
    with pytest.raises(ExecutionTraceError):
        ExecutionTrace("v", "r", "p", "s", (
            TrajectorySample(1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            TrajectorySample(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        ))
