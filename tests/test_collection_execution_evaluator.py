"""Every outcome class must be reachable, and reached for the stated reason."""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"))

from collection_execution_fixtures import (  # noqa: E402
    add_planned_crossing_rows,
    capture_geometry,
    confirm,
    crossing_times,
    execute,
    observe,
    scenario_connector_collects,
    scenario_near_miss,
    scenario_straight_sweep,
    scenario_two_passes_with_connector,
)
from collection_route_fixtures import default_configuration  # noqa: E402
from tennis_robot.collection_execution_evaluator import (  # noqa: E402
    CrossingOutcome,
    evaluate_execution,
)
from tennis_robot.collection_route_types import RouteSegmentType  # noqa: E402


def evaluate(snapshot, plan, recorder, *, displacement_threshold_m=0.10):
    return evaluate_execution(
        snapshot=snapshot, plan=plan, trace=recorder.build(),
        capture_geometry=capture_geometry(),
        displacement_threshold_m=displacement_threshold_m,
        disturbance_reporting_radius_m=1.5,
        crossing_window_m=0.5,
    )


def outcomes_by_ball(evaluation):
    return {item.ball_id: item.outcome for item in evaluation.outcomes}


def test_a_clean_run_collects_everything_it_planned():
    snapshot, plan = scenario_straight_sweep()
    recorder, _ = execute(plan)
    times = crossing_times(plan, recorder.build())
    for ball_id, moment in times.items():
        confirm(recorder, t_s=moment, ball_id=ball_id)
    add_planned_crossing_rows(recorder, plan)
    evaluation = evaluate(snapshot, plan, recorder)
    assert set(outcomes_by_ball(evaluation).values()) == {
        CrossingOutcome.PLANNED_AND_EXECUTED_COLLECTED
    }
    for item in evaluation.outcomes:
        assert item.executed.crossed
        assert item.confirmed
        assert abs(item.executed.lateral_offset_m) < 0.05


def test_a_lateral_tracking_error_shows_up_as_a_tracking_miss():
    # 0.30 m of cross-track error puts the ball outside the 0.205 m mouth.
    snapshot, plan = scenario_straight_sweep()
    recorder, _ = execute(plan, lateral_bias_m=0.30)
    evaluation = evaluate(snapshot, plan, recorder)
    assert set(outcomes_by_ball(evaluation).values()) == {
        CrossingOutcome.PLANNED_BUT_TRACKING_MISSED
    }
    for item in evaluation.outcomes:
        assert item.planned_crossing
        assert not item.executed.crossed
        assert item.executed.minimum_clearance_m > 0.09
    # And the tracking metrics say the same thing independently.
    for segment in evaluation.tracking:
        assert segment.max_cross_track_m > 0.25


def test_a_sweep_without_confirmation_is_not_called_a_collection():
    snapshot, plan = scenario_straight_sweep()
    recorder, _ = execute(plan)
    # The balls are seen exactly where planning left them, and no beam fires.
    for ball in snapshot.balls:
        observe(recorder, t_s=99.0, ball_id=ball.ball_id,
                x_m=ball.position.x_m, y_m=ball.position.y_m)
    evaluation = evaluate(snapshot, plan, recorder)
    assert set(outcomes_by_ball(evaluation).values()) == {
        CrossingOutcome.EXECUTED_CROSSING_NOT_COLLECTED
    }
    for item in evaluation.outcomes:
        assert item.executed.crossed and not item.confirmed


def test_a_swept_but_unobserved_ball_is_uncertain_not_a_mechanical_failure():
    # Nothing was seen and nothing was confirmed: the evaluator must decline to
    # blame the collector.
    snapshot, plan = scenario_straight_sweep()
    recorder, _ = execute(plan)
    evaluation = evaluate(snapshot, plan, recorder)
    assert set(outcomes_by_ball(evaluation).values()) == {
        CrossingOutcome.OBSERVATION_UNCERTAIN
    }


def test_a_displaced_ball_is_reported_as_displaced_not_missed():
    snapshot, plan = scenario_straight_sweep()
    recorder, _ = execute(plan, lateral_bias_m=0.30)
    for ball in snapshot.balls:
        # Seen well away from where planning believed it was.
        observe(recorder, t_s=50.0, ball_id=ball.ball_id,
                x_m=ball.position.x_m + 0.4, y_m=ball.position.y_m - 0.3)
    evaluation = evaluate(snapshot, plan, recorder)
    assert set(outcomes_by_ball(evaluation).values()) == {
        CrossingOutcome.BALL_DISPLACED_BEFORE_ATTEMPT
    }
    for item in evaluation.outcomes:
        assert item.displacement_m == pytest.approx(0.5, abs=1e-6)
        assert item.latest_observed_position is not None
        # The planning belief is preserved alongside the observation.
        assert item.planning_position != item.latest_observed_position


def test_collection_by_another_segment_is_valid_behaviour():
    snapshot, plan = scenario_connector_collects()
    recorder, _ = execute(plan)
    trace = recorder.build()
    times = crossing_times(plan, trace)
    for ball_id, moment in times.items():
        confirm(recorder, t_s=moment, ball_id=ball_id)
    evaluation = evaluate(snapshot, plan, recorder)
    assert all(item.confirmed for item in evaluation.outcomes)
    # Whatever the router chose, no ball is reported as a failure.
    assert not {
        CrossingOutcome.PLANNED_BUT_TRACKING_MISSED,
        CrossingOutcome.EXECUTED_CROSSING_NOT_COLLECTED,
    } & set(outcomes_by_ball(evaluation).values())


def test_the_matrix_and_segment_split_are_reported():
    snapshot, plan = scenario_two_passes_with_connector()
    recorder, _ = execute(plan)
    times = crossing_times(plan, recorder.build())
    for ball_id, moment in times.items():
        confirm(recorder, t_s=moment, ball_id=ball_id)
    evaluation = evaluate(snapshot, plan, recorder)
    matrix = evaluation.matrix()
    assert sum(matrix.values()) == len(snapshot.balls)
    assert matrix[("yes", "yes", "yes")] == len(snapshot.balls)
    split = evaluation.by_segment_type()
    assert set(split) <= {
        RouteSegmentType.FUNNEL_PASS.value, RouteSegmentType.CONNECTOR.value, "unassigned"
    }
    assert sum(row["confirmed"] for row in split.values()) == len(snapshot.balls)


def test_a_bystander_ball_is_measured_for_disturbance():
    snapshot, plan = scenario_near_miss()
    recorder, _ = execute(plan)
    evaluation = evaluate(snapshot, plan, recorder)
    events = {item.ball_id: item for item in evaluation.disturbances}
    # The bystander is approached by a segment that is not its own attempt.
    assert "bystander" in events
    event = events["bystander"]
    assert event.body_clearance_m >= 0.0
    assert event.mouth_clearance_m >= 0.0
    assert event.segment_id is not None


def test_disturbance_reports_distance_continuously_not_a_verdict():
    snapshot, plan = scenario_near_miss()
    recorder, _ = execute(plan)
    observe(recorder, t_s=80.0, ball_id="bystander", x_m=6.05, y_m=0.60)
    evaluation = evaluate(snapshot, plan, recorder)
    event = next(item for item in evaluation.disturbances if item.ball_id == "bystander")
    assert event.displacement_m == pytest.approx(math.hypot(0.05, 0.05), abs=1e-6)
    # Reported in metres; no threshold is applied to the measurement itself.
    assert isinstance(event.body_clearance_m, float)


def test_tracking_metrics_are_computed_per_segment():
    snapshot, plan = scenario_two_passes_with_connector()
    recorder, _ = execute(plan, lateral_bias_m=0.05)
    evaluation = evaluate(snapshot, plan, recorder)
    assert evaluation.tracking
    for segment in evaluation.tracking:
        assert segment.samples > 0
        assert segment.max_cross_track_m >= segment.rms_cross_track_m >= 0.0
        assert segment.rms_cross_track_m >= segment.mean_cross_track_m - 1e-9
        assert segment.executed_length_m > 0.0
        assert segment.duration_s >= 0.0
    # A uniform 5 cm offset shows up as 5 cm on the straight passes.  On a
    # curved connector the offset curve is genuinely nearer to other parts of
    # the same arc, so its cross-track is legitimately smaller -- the metric is
    # distance to the planned path, not the size of the injected offset.
    passes = [
        item for item in evaluation.tracking
        if item.segment_type == RouteSegmentType.FUNNEL_PASS.value
    ]
    assert passes
    for item in passes:
        assert item.mean_cross_track_m == pytest.approx(0.05, abs=0.01)


def test_a_segment_specific_error_is_isolated_to_that_segment():
    snapshot, plan = scenario_two_passes_with_connector()
    passes = [
        segment.id for segment in plan.segments
        if segment.type is RouteSegmentType.FUNNEL_PASS
    ]
    recorder, _ = execute(plan, segment_bias={passes[0]: 0.35})
    evaluation = evaluate(snapshot, plan, recorder)
    by_id = {item.segment_id: item for item in evaluation.tracking}
    assert by_id[passes[0]].max_cross_track_m == pytest.approx(0.35, abs=0.01)
    for segment_id, item in by_id.items():
        if segment_id != passes[0]:
            assert item.max_cross_track_m < 0.01


def test_the_trace_must_belong_to_the_plan_it_is_evaluated_against():
    snapshot, plan = scenario_straight_sweep()
    other_snapshot, other_plan = scenario_near_miss()
    recorder, _ = execute(other_plan)
    with pytest.raises(ValueError):
        evaluate(snapshot, plan, recorder)


def test_evaluation_counts_the_telemetry_it_consumed():
    snapshot, plan = scenario_straight_sweep()
    recorder, _ = execute(plan)
    evaluation = evaluate(snapshot, plan, recorder)
    assert evaluation.telemetry_rows == len(recorder.build().samples)


# ── authoritative confirmations and the geometry they can disagree with ─────

def confirmation(ball_id, **overrides):
    from tennis_robot.collection_execution_trace import ConfirmationEvent

    values = dict(
        t_s=10.0, confirmation_id=1, ball_id=ball_id, association="active_crossing",
        segment_id=None, progress_s=5.0, crossing_progress_s=5.0,
        lateral_error_m=0.005, heading_error_rad=0.0, measured_speed_mps=0.35,
    )
    values.update(overrides)
    return ConfirmationEvent(**values)


def with_confirmations(recorder, *events):
    for event in events:
        recorder.record_confirmation(event)
    return recorder


def test_an_attributed_confirmation_is_believed_without_any_timing_heuristic():
    # No beam edges at all: the attributed confirmation alone must count.
    snapshot, plan = scenario_straight_sweep()
    recorder, _ = execute(plan)
    with_confirmations(
        recorder, *(confirmation(ball.ball_id, confirmation_id=index + 1)
                    for index, ball in enumerate(snapshot.balls))
    )
    evaluation = evaluate(snapshot, plan, recorder)
    assert all(item.confirmed for item in evaluation.outcomes)
    assert set(outcomes_by_ball(evaluation).values()) == {
        CrossingOutcome.PLANNED_AND_EXECUTED_COLLECTED
    }


def test_a_collected_ball_is_never_uncertain_just_because_beams_were_omitted():
    """The exact Phase 9B failure: 4/4 collected, reported as uncertain."""
    snapshot, plan = scenario_straight_sweep()
    recorder, _ = execute(plan)
    with_confirmations(
        recorder, *(confirmation(ball.ball_id, confirmation_id=index + 1)
                    for index, ball in enumerate(snapshot.balls))
    )
    evaluation = evaluate(snapshot, plan, recorder)
    assert CrossingOutcome.OBSERVATION_UNCERTAIN not in set(
        outcomes_by_ball(evaluation).values()
    )


def test_confirmation_without_a_reconstructed_crossing_is_surfaced_not_hidden():
    # The target-3 shape: the machine confirms, the reconstruction says the
    # mouth never swept the believed position.  Neither fact may be rewritten.
    snapshot, plan = scenario_straight_sweep()
    recorder, _ = execute(plan, lateral_bias_m=0.30)   # mouth misses every ball
    with_confirmations(
        recorder, *(confirmation(ball.ball_id, confirmation_id=index + 1,
                                 association="intake_lead_crossing")
                    for index, ball in enumerate(snapshot.balls))
    )
    evaluation = evaluate(snapshot, plan, recorder)
    assert set(outcomes_by_ball(evaluation).values()) == {
        CrossingOutcome.CONFIRMED_WITHOUT_RECONSTRUCTED_CROSSING
    }
    assert len(evaluation.inconsistencies) == len(snapshot.balls)
    for item in evaluation.outcomes:
        # Both halves of the disagreement survive.
        assert item.confirmed is True
        assert item.executed.crossed is False
        assert "intake_lead_crossing" in item.detail
    # And the matrix still reports the physical reconstruction honestly.
    assert evaluation.matrix()[("yes", "no", "yes")] == len(snapshot.balls)


def test_a_trace_without_confirmations_still_reads_and_evaluates_conservatively():
    """Phase 9/9B traces predate attributed confirmations."""
    from tennis_robot.collection_execution_trace import ExecutionTrace

    snapshot, plan = scenario_straight_sweep()
    recorder, _ = execute(plan)
    payload = recorder.build().to_dict()
    payload["schema_version"] = "collection-execution-trace/v1"
    del payload["confirmations"]
    trace = ExecutionTrace.from_dict(payload)
    assert trace.confirmations == ()
    evaluation = evaluate_execution(
        snapshot=snapshot, plan=plan, trace=trace, capture_geometry=capture_geometry(),
        displacement_threshold_m=0.1, disturbance_reporting_radius_m=1.5,
        crossing_window_m=0.5,
    )
    # Conservative: swept but unconfirmed and unobserved stays uncertain.
    assert set(item.outcome for item in evaluation.outcomes) == {
        CrossingOutcome.OBSERVATION_UNCERTAIN
    }
