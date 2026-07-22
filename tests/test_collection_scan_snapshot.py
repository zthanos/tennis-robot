import os, sys
from dataclasses import FrozenInstanceError
import pytest
sys.path.insert(0, os.path.join(os.path.dirname(__file__),"..","ros2_ws","src","tennis_robot"))
from tennis_robot.collection_scan_snapshot import CourtHalfBoundary, ScanSnapshotBuilder, ScanSnapshotFailure, ScanSnapshotFailureCode
from tennis_robot.collection_route_types import *
from collection_route_fixtures import default_configuration, default_court_half_boundary, SCAN_POSE

def obs(step,x=1.,y=2.,c=.9,cov=None): return AcceptedSpatialObservation("scan",0,1.,1.,Point2D(x,y),cov or PositionCovariance2D(.1,0.,.1),c,step,"gazebo-v2","cfg")
def builder(**kw):
    kw.setdefault("configuration_snapshot",default_configuration())
    kw.setdefault("expected_scan_step_ids",("a","b"))
    kw.setdefault("court_half_boundary",default_court_half_boundary())
    return ScanSnapshotBuilder(scan_id="scan",scan_timestamp_s=1.,robot_pose_at_scan=SCAN_POSE,**kw)

def test_coverage_and_timeout_have_single_source_in_configuration_snapshot():
    b=builder(); scan=default_configuration().scan
    assert b.required_coverage_fraction == scan.required_coverage_fraction
    assert b.scan_timeout_s == scan.scan_timeout_s
    with pytest.raises(ScanSnapshotFailure): ScanSnapshotBuilder(scan_id="scan",scan_timestamp_s=1.,robot_pose_at_scan=SCAN_POSE,configuration_snapshot=None,expected_scan_step_ids=("a",),court_half_boundary=default_court_half_boundary())
    with pytest.raises(ScanSnapshotFailure): ScanSnapshotBuilder(scan_id="scan",scan_timestamp_s=1.,robot_pose_at_scan=SCAN_POSE,configuration_snapshot=default_configuration(),expected_scan_step_ids=("a",),court_half_boundary=None)

def test_empty_snapshot_requires_complete_coverage_and_is_immutable():
    b=builder(); b.add(obs("a",1.,2.)); b.add(obs("b",10.,2.)); s=b.finalize(2.)
    assert s.balls == ()
    with pytest.raises(FrozenInstanceError): s.scan_id="x"
    with pytest.raises(ScanSnapshotFailure): b.add(obs("a"))

def test_valid_empty_heartbeat_steps_produce_empty_snapshot():
    b=builder(); b.record_visited_step("a"); b.record_visited_step("b"); s=b.finalize(2.)
    assert s.balls == ()

def test_confirmed_snapshot_duplicate_fusion_and_stable_id():
    b=builder(); b.add(obs("a",1,2)); b.add(obs("b",1.2,2)); s=b.finalize(2.)
    assert len(s.balls)==1 and s.balls[0].ball_id=="scan/target-1" and s.balls[0].position.x_m==pytest.approx(1.1)

def test_timeout_and_failed_lifecycle():
    b=builder(); b.add(obs("a")); b.add(obs("b"))
    with pytest.raises(ScanSnapshotFailure, match="steps") as error: b.finalize(1.+default_configuration().scan.scan_timeout_s+1.)
    assert error.value.code is ScanSnapshotFailureCode.TIMEOUT
    with pytest.raises(ScanSnapshotFailure): b.add(obs("a"))

def test_insufficient_coverage_unknown_duplicate_and_rejections_do_not_count():
    b=builder(); b.add(obs("a")); b.add(obs("a")); b.add(obs("unknown")); b.add(SpatialObservationRejection(SpatialObservationRejectionCode.NON_SPATIAL_DETECTION,0,1.,"x"))
    with pytest.raises(ScanSnapshotFailure, match="covered") as error: b.finalize(2.)
    assert error.value.code is ScanSnapshotFailureCode.INSUFFICIENT_COVERAGE
    assert any(item.detail=="unknown_scan_step" for item in b.rejections)
    with pytest.raises(ScanSnapshotFailure): b.finalize(2.)

def test_fused_covariance_never_falls_below_localization_budget():
    # Many precise observations: information fusion would otherwise shrink the
    # shared localization error by the observation count.
    b=builder(expected_scan_step_ids=("a","b","c","d"))
    for step in ("a","b","c","d"): b.add(obs(step,1.,2.,cov=PositionCovariance2D(1e-4,0.,1e-4)))
    s=b.finalize(2.)
    budget=default_configuration().gazebo_snapshot.localization_xy_covariance.covariance
    assert len(s.balls)==1
    assert s.balls[0].position_covariance.xx >= budget.xx
    assert s.balls[0].position_covariance.yy >= budget.yy
    assert s.balls[0].position_covariance.xx == pytest.approx(budget.xx + 1e-4/4)

def test_cross_half_ball_directly_behind_the_net_is_rejected():
    b=builder()
    b.add(obs("a",-0.05,0.))   # just behind the net (net at x=0, robot at x>0)
    b.add(obs("a",0.,0.))      # exactly on the net line: not the robot's half
    b.add(obs("a",1.,2.)); b.add(obs("b",1.,2.))
    s=b.finalize(2.)
    assert [item.detail for item in b.rejections] == ["opposite_court_half","opposite_court_half"]
    assert len(s.balls)==1 and s.balls[0].position.x_m > 0.

def test_singular_observation_covariance_is_typed_rejection_not_crash():
    b=builder(); b.add(obs("a",cov=PositionCovariance2D(0.,0.,0.)))
    assert b.rejections[-1].detail=="singular_covariance"
    b.add(obs("a")); b.add(obs("b")); assert len(b.finalize(2.).balls)==1

def test_ambiguous_association_rejection_does_not_count_scan_step_coverage():
    b=builder(expected_scan_step_ids=("a","b","c"))
    b.add(obs("a",1.,-3.)); b.add(obs("b",4.,-3.))          # two separated tracks
    b.add(obs("c",2.5,-3.,cov=PositionCovariance2D(.4,0.,.4)))  # gates to both
    assert b.rejections[-1].detail=="ambiguous_scan_local_association"
    with pytest.raises(ScanSnapshotFailure, match="covered"): b.finalize(2.)

def test_same_step_duplicate_in_track_is_recorded_in_telemetry():
    b=builder()
    b.add(obs("a",1.,2.)); b.add(obs("a",1.05,2.)); b.add(obs("b",1.,2.))
    s=b.finalize(2.)
    assert len(b.duplicate_step_observations)==1
    assert b.duplicate_step_observations[0].scan_step_id=="a"
    assert len(s.balls)==1 and b.rejections==[]
