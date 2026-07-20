"""Pure Gazebo-MVP C1-to-snapshot spatial observation boundary."""
from __future__ import annotations

from dataclasses import dataclass
import math

from tennis_robot.collection_route_types import (
    AcceptedSpatialObservation, DomainValidationError, LocalizationXYCovariance,
    Point2D, PositionCovariance2D, SpatialObservationRejection,
    SpatialObservationRejectionCode,
)


@dataclass(frozen=True)
class C1ValidatedSpatialDetection:
    position_camera_xyz: tuple[float, float, float]
    covariance_camera_xyz: tuple[float, ...]
    confidence: float
    rgb_timestamp_s: float
    matched_depth_timestamp_s: float
    frame_id: str
    calibration_id: str
    configuration_id: str
    spatial_targets_healthy: bool = True
    has_spatial: bool = True


@dataclass(frozen=True)
class TimestampedCameraToMapTransform:
    timestamp_s: float
    map_frame: str
    camera_frame: str
    translation_xyz: tuple[float, float, float]
    rotation_xyzw: tuple[float, float, float, float]


class PerceptionSpatialObservationAdapter:
    """Projects already validated metadata; it performs no ROS/TF lookup.

    The accepted observation carries ONLY the rotated camera-XY measurement
    covariance. The configured ``localization_xy_covariance`` is still a
    required input (its absence is a typed rejection) but it is applied exactly
    once per fused ball by the snapshot builder at finalize — adding it to
    every observation would let information fusion divide the shared
    localization error by the observation count.
    """
    def accept(self, *, scan_id: str, detection_index: int, detection: C1ValidatedSpatialDetection,
               transform: TimestampedCameraToMapTransform,
               localization_xy_covariance: LocalizationXYCovariance | None,
               scan_step_id: str | None) -> AcceptedSpatialObservation | SpatialObservationRejection:
        if not detection.spatial_targets_healthy:
            return self._reject(SpatialObservationRejectionCode.SPATIAL_TARGETS_UNHEALTHY, detection_index, detection.rgb_timestamp_s, "spatial_targets_unhealthy")
        if not detection.has_spatial:
            return self._reject(SpatialObservationRejectionCode.NON_SPATIAL_DETECTION, detection_index, detection.rgb_timestamp_s, "non_spatial_detection")
        if not scan_step_id or not isinstance(scan_step_id, str):
            return self._reject(SpatialObservationRejectionCode.PERCEPTION_METADATA_REJECTED, detection_index, detection.rgb_timestamp_s, "scan_step_id_missing")
        if localization_xy_covariance is None:
            return self._reject(SpatialObservationRejectionCode.PERCEPTION_METADATA_REJECTED, detection_index, detection.rgb_timestamp_s, "localization_xy_covariance_missing")
        if detection.frame_id != transform.camera_frame or transform.map_frame != "map":
            return self._reject(SpatialObservationRejectionCode.FRAME_MISMATCH, detection_index, detection.rgb_timestamp_s, "transform frame mismatch")
        if not math.isclose(transform.timestamp_s, detection.rgb_timestamp_s, abs_tol=1e-9):
            return self._reject(SpatialObservationRejectionCode.PERCEPTION_TF_REJECTED, detection_index, detection.rgb_timestamp_s, "transform timestamp mismatch")
        try:
            x, y, _ = detection.position_camera_xyz
            qx, qy, qz, qw = transform.rotation_xyzw
            tx, ty, _ = transform.translation_xyz
            if not all(math.isfinite(v) for v in (*detection.position_camera_xyz, *detection.covariance_camera_xyz, qx, qy, qz, qw, tx, ty)) or len(detection.covariance_camera_xyz) != 9:
                raise ValueError
            r00=1-2*(qy*qy+qz*qz); r01=2*(qx*qy-qw*qz); r10=2*(qx*qy+qw*qz); r11=1-2*(qx*qx+qz*qz)
            a,b,d = detection.covariance_camera_xyz[0], detection.covariance_camera_xyz[1], detection.covariance_camera_xyz[4]
            cam = PositionCovariance2D(a, b, d)
            xx=r00*r00*cam.xx+2*r00*r01*cam.xy+r01*r01*cam.yy
            xy=r00*r10*cam.xx+(r00*r11+r01*r10)*cam.xy+r01*r11*cam.yy
            yy=r10*r10*cam.xx+2*r10*r11*cam.xy+r11*r11*cam.yy
            return AcceptedSpatialObservation(scan_id, detection_index, detection.rgb_timestamp_s, detection.matched_depth_timestamp_s, Point2D(tx+r00*x+r01*y, ty+r10*x+r11*y), PositionCovariance2D(xx, xy, yy), detection.confidence, scan_step_id, detection.calibration_id, detection.configuration_id)
        except (ValueError, TypeError, DomainValidationError):
            return self._reject(SpatialObservationRejectionCode.PERCEPTION_METADATA_REJECTED, detection_index, detection.rgb_timestamp_s, "invalid_c1_spatial_detection")

    @staticmethod
    def _reject(code, index, stamp, detail):
        return SpatialObservationRejection(code, index, stamp, detail)
