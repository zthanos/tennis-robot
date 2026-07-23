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
            x, y, z = detection.position_camera_xyz
            qx, qy, qz, qw = transform.rotation_xyzw
            tx, ty, _ = transform.translation_xyz
            if not all(math.isfinite(v) for v in (*detection.position_camera_xyz, *detection.covariance_camera_xyz, qx, qy, qz, qw, tx, ty)) or len(detection.covariance_camera_xyz) != 9:
                raise ValueError
            rotation = (
                (1-2*(qy*qy+qz*qz), 2*(qx*qy-qw*qz), 2*(qx*qz+qw*qy)),
                (2*(qx*qy+qw*qz), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qw*qx)),
                (2*(qx*qz-qw*qy), 2*(qy*qz+qw*qx), 1-2*(qx*qx+qy*qy)),
            )
            covariance = (
                detection.covariance_camera_xyz[0:3],
                detection.covariance_camera_xyz[3:6],
                detection.covariance_camera_xyz[6:9],
            )

            def rotated_covariance(row_a: int, row_b: int) -> float:
                return sum(
                    rotation[row_a][i] * covariance[i][j] * rotation[row_b][j]
                    for i in range(3) for j in range(3)
                )

            map_x = tx + rotation[0][0]*x + rotation[0][1]*y + rotation[0][2]*z
            map_y = ty + rotation[1][0]*x + rotation[1][1]*y + rotation[1][2]*z
            xx = rotated_covariance(0, 0)
            xy = rotated_covariance(0, 1)
            yy = rotated_covariance(1, 1)
            return AcceptedSpatialObservation(scan_id, detection_index, detection.rgb_timestamp_s, detection.matched_depth_timestamp_s, Point2D(map_x, map_y), PositionCovariance2D(xx, xy, yy), detection.confidence, scan_step_id, detection.calibration_id, detection.configuration_id)
        except (ValueError, TypeError, DomainValidationError):
            return self._reject(SpatialObservationRejectionCode.PERCEPTION_METADATA_REJECTED, detection_index, detection.rgb_timestamp_s, "invalid_c1_spatial_detection")

    @staticmethod
    def _reject(code, index, stamp, detail):
        return SpatialObservationRejection(code, index, stamp, detail)
