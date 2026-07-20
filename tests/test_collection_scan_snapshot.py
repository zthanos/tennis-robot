import os, sys
from dataclasses import FrozenInstanceError
import pytest
sys.path.insert(0, os.path.join(os.path.dirname(__file__),"..","ros2_ws","src","tennis_robot"))
from tennis_robot.collection_scan_snapshot import ScanSnapshotBuilder, ScanSnapshotFailure, ScanSnapshotFailureCode
from tennis_robot.collection_route_types import *
from collection_route_fixtures import default_configuration, SCAN_POSE

def obs(step,x=1.,y=2.,c=.9): return AcceptedSpatialObservation("scan",0,1.,1.,Point2D(x,y),PositionCovariance2D(.1,0.,.1),c,step,"gazebo-v2","cfg")
def builder(**kw): return ScanSnapshotBuilder(scan_id="scan",scan_timestamp_s=1.,robot_pose_at_scan=SCAN_POSE,configuration_snapshot=default_configuration(),expected_scan_step_ids=("a","b"),required_coverage_fraction=1.,scan_timeout_s=5.,**kw)

def test_empty_snapshot_requires_complete_coverage_and_is_immutable():
    b=builder(); b.add(obs("a",1.,2.)); b.add(obs("b",10.,2.)); s=b.finalize(2.)
    assert s.balls == ()
    with pytest.raises(FrozenInstanceError): s.scan_id="x"
    with pytest.raises(ScanSnapshotFailure): b.add(obs("a"))

def test_confirmed_snapshot_duplicate_fusion_and_stable_id():
    b=builder(); b.add(obs("a",1,2)); b.add(obs("b",1.2,2)); s=b.finalize(2.)
    assert len(s.balls)==1 and s.balls[0].ball_id=="scan/target-1" and s.balls[0].position.x_m==pytest.approx(1.1)

def test_timeout_and_failed_lifecycle():
    b=builder(); b.add(obs("a")); b.add(obs("b"))
    with pytest.raises(ScanSnapshotFailure, match="scan_timeout") as error: b.finalize(7.)
    assert error.value.code is ScanSnapshotFailureCode.TIMEOUT
    with pytest.raises(ScanSnapshotFailure): b.add(obs("a"))

def test_insufficient_coverage_unknown_duplicate_and_rejections_do_not_count():
    b=builder(); b.add(obs("a")); b.add(obs("a")); b.add(obs("unknown")); b.add(SpatialObservationRejection(SpatialObservationRejectionCode.NON_SPATIAL_DETECTION,0,1.,"x"))
    with pytest.raises(ScanSnapshotFailure, match="insufficient_coverage") as error: b.finalize(2.)
    assert error.value.code is ScanSnapshotFailureCode.INSUFFICIENT_COVERAGE
    assert any(item.detail=="unknown_scan_step" for item in b.rejections)
    with pytest.raises(ScanSnapshotFailure): b.finalize(2.)
