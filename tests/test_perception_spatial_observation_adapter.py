import os, sys
from dataclasses import FrozenInstanceError, replace
import math
import pytest
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"))
from tennis_robot.collection_route_types import GazeboSnapshotConfiguration, LocalizationXYCovariance, PositionCovariance2D, SnapshotAssociationConfiguration, SpatialObservationRejection, SpatialObservationRejectionCode
from tennis_robot.perception_spatial_observation_adapter import C1ValidatedSpatialDetection, PerceptionSpatialObservationAdapter, TimestampedCameraToMapTransform

def _d(): return C1ValidatedSpatialDetection((1.,0.,2.), (1.,0.,0.,0.,4.,0.,0.,0.,9.), .9, 10.,10., "camera_link_optical_frame", "gazebo-v2", "config-1")
def _t(): return TimestampedCameraToMapTransform(10., "map", "camera_link_optical_frame", (2.,3.,0.), (0.,0.,0.7071067811865475,0.7071067811865476))
def test_rotation_keeps_measurement_only_covariance_without_localization_addition():
    # The localization budget is applied once per fused ball by the builder,
    # never per observation; the adapter publishes the rotated camera XY
    # measurement covariance unchanged.
    result=PerceptionSpatialObservationAdapter().accept(scan_id="s", detection_index=0, detection=_d(), transform=_t(), localization_xy_covariance=LocalizationXYCovariance(PositionCovariance2D(.5,0.,.25)), scan_step_id="step-1")
    assert result.position_map_xy.x_m == 2.0 and result.position_map_xy.y_m == 4.0
    assert result.position_covariance_map_xy.xx == pytest.approx(4.0) and result.position_covariance_map_xy.yy == pytest.approx(1.0)
def test_missing_step_or_covariance_rejects():
    a=PerceptionSpatialObservationAdapter(); assert isinstance(a.accept(scan_id="s", detection_index=0, detection=_d(), transform=_t(), localization_xy_covariance=None, scan_step_id="x"), SpatialObservationRejection)
    assert isinstance(a.accept(scan_id="s", detection_index=0, detection=_d(), transform=_t(), localization_xy_covariance=LocalizationXYCovariance(PositionCovariance2D(1.,0.,1.)), scan_step_id=None), SpatialObservationRejection)

def test_invalid_covariance_contract_and_configuration_are_immutable():
    with pytest.raises(Exception): PositionCovariance2D(1., 2., 1.)
    with pytest.raises(Exception): PositionCovariance2D(math.nan, 0., 1.)
    config=GazeboSnapshotConfiguration("cfg", LocalizationXYCovariance(PositionCovariance2D(1.,0.,1.)), SnapshotAssociationConfiguration(5.99,2,2))
    with pytest.raises(FrozenInstanceError): config.association = None
    with pytest.raises(FrozenInstanceError): config.localization_xy_covariance.covariance = None

def test_typed_rejections_never_return_geometry():
    a=PerceptionSpatialObservationAdapter()
    unhealthy=replace(_d(), spatial_targets_healthy=False)
    nonspatial=replace(_d(), has_spatial=False)
    bad_frame=TimestampedCameraToMapTransform(10., "map", "other", (0.,0.,0.), (0.,0.,0.,1.))
    bad_time=TimestampedCameraToMapTransform(11., "map", "camera_link_optical_frame", (0.,0.,0.), (0.,0.,0.,1.))
    cases=[(unhealthy,_t(),"x",SpatialObservationRejectionCode.SPATIAL_TARGETS_UNHEALTHY),(nonspatial,_t(),"x",SpatialObservationRejectionCode.NON_SPATIAL_DETECTION),(_d(),bad_frame,"x",SpatialObservationRejectionCode.FRAME_MISMATCH),(_d(),bad_time,"x",SpatialObservationRejectionCode.PERCEPTION_TF_REJECTED),(_d(),_t(),"",SpatialObservationRejectionCode.PERCEPTION_METADATA_REJECTED)]
    for detection,transform,step,code in cases:
        result=a.accept(scan_id="s", detection_index=0, detection=detection, transform=transform, localization_xy_covariance=LocalizationXYCovariance(PositionCovariance2D(1.,0.,1.)), scan_step_id=step)
        assert isinstance(result, SpatialObservationRejection) and result.code is code
        assert not hasattr(result, "position_map_xy") and not hasattr(result, "position_covariance_map_xy")
