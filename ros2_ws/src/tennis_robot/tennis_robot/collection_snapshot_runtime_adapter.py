"""Thin scan-runtime bridge; it owns neither ROS subscriptions nor scan policy."""

from __future__ import annotations

from tennis_robot.collection_route_types import (
    CollectionRouteConfiguration,
    LocalizationXYCovariance,
    Pose2D,
    SpatialObservationRejection,
    SpatialObservationRejectionCode,
)
from tennis_robot.collection_scan_snapshot import ScanSnapshotBuilder
from tennis_robot.perception_covariance_calibration import (
    PerceptionSpatialValidationConfig,
    validate_spatial_metadata,
)
from tennis_robot.perception_spatial_observation_adapter import (
    C1ValidatedSpatialDetection,
    PerceptionSpatialObservationAdapter,
)


def _stamp(value) -> float:
    return float(value.sec) + float(value.nanosec) * 1e-9


class CollectionSnapshotRuntimeAdapter:
    """Validate a canonical frame and forward only typed adapter outputs."""

    def __init__(
        self,
        *,
        tf_provider,
        validation_config: PerceptionSpatialValidationConfig,
        localization_xy_covariance: LocalizationXYCovariance | None,
    ) -> None:
        self.tf_provider = tf_provider
        self.validation_config = validation_config
        self.localization_xy_covariance = localization_xy_covariance
        self.pure = PerceptionSpatialObservationAdapter()

    def forward(self, *, scan_id: str, frame, scan_step_id: str | None, builder) -> None:
        rgb_timestamp_s = _stamp(frame.header.stamp)
        for index, detection_message in enumerate(frame.detections):
            result = self._adapt_one(
                scan_id=scan_id,
                frame=frame,
                detection_index=index,
                detection_message=detection_message,
                rgb_timestamp_s=rgb_timestamp_s,
                scan_step_id=scan_step_id,
            )
            builder.add(result)

    def _adapt_one(
        self,
        *,
        scan_id: str,
        frame,
        detection_index: int,
        detection_message,
        rgb_timestamp_s: float,
        scan_step_id: str | None,
    ):
        if not frame.spatial_targets_healthy:
            return self._reject(
                SpatialObservationRejectionCode.SPATIAL_TARGETS_UNHEALTHY,
                detection_index,
                rgb_timestamp_s,
                "spatial_targets_unhealthy",
            )
        if not detection_message.has_spatial:
            return self._reject(
                SpatialObservationRejectionCode.NON_SPATIAL_DETECTION,
                detection_index,
                rgb_timestamp_s,
                "non_spatial_detection",
            )

        metadata_reason = validate_spatial_metadata(
            rgb_timestamp_s,
            _stamp(detection_message.matched_depth_stamp),
            tuple(detection_message.position_covariance),
            self.validation_config,
        )
        if metadata_reason:
            return self._reject(
                SpatialObservationRejectionCode.PERCEPTION_METADATA_REJECTED,
                detection_index,
                rgb_timestamp_s,
                metadata_reason,
            )
        try:
            transform = self.tf_provider.at(rgb_timestamp_s, frame.header.frame_id)
        except Exception:
            return self._reject(
                SpatialObservationRejectionCode.PERCEPTION_TF_REJECTED,
                detection_index,
                rgb_timestamp_s,
                "tf_at_rgb_timestamp_unavailable",
            )

        detection = C1ValidatedSpatialDetection(
            (detection_message.position_x, detection_message.position_y, detection_message.position_z),
            tuple(detection_message.position_covariance),
            detection_message.confidence,
            rgb_timestamp_s,
            _stamp(detection_message.matched_depth_stamp),
            frame.header.frame_id,
            getattr(frame, "calibration_id", "runtime"),
            getattr(frame, "configuration_id", "runtime"),
        )
        return self.pure.accept(
            scan_id=scan_id,
            detection_index=detection_index,
            detection=detection,
            transform=transform,
            localization_xy_covariance=self.localization_xy_covariance,
            scan_step_id=scan_step_id,
        )

    @staticmethod
    def _reject(code, detection_index: int, rgb_timestamp_s: float, detail: str):
        return SpatialObservationRejection(code, detection_index, rgb_timestamp_s, detail)


class CollectionSnapshotRuntimeSession:
    """Compose the runtime adapter and pure builder for one externally-owned scan.

    The caller supplies each scan-step ID; this class does not rotate the robot,
    subscribe to ROS topics, or make any planner/executor decision.
    """

    def __init__(
        self,
        *,
        scan_id: str,
        scan_timestamp_s: float,
        robot_pose_at_scan: Pose2D,
        configuration_snapshot: CollectionRouteConfiguration,
        validation_config: PerceptionSpatialValidationConfig,
        expected_scan_step_ids: tuple[str, ...],
        tf_provider,
        map_frame: str = "map",
    ) -> None:
        self.builder = ScanSnapshotBuilder(
            scan_id=scan_id,
            scan_timestamp_s=scan_timestamp_s,
            robot_pose_at_scan=robot_pose_at_scan,
            configuration_snapshot=configuration_snapshot,
            expected_scan_step_ids=expected_scan_step_ids,
            map_frame=map_frame,
        )
        self.adapter = CollectionSnapshotRuntimeAdapter(
            tf_provider=tf_provider,
            validation_config=validation_config,
            localization_xy_covariance=configuration_snapshot.gazebo_snapshot.localization_xy_covariance,
        )

    def forward_frame(self, frame, *, scan_step_id: str | None) -> None:
        self.adapter.forward(
            scan_id=self.builder.scan_id,
            frame=frame,
            scan_step_id=scan_step_id,
            builder=self.builder,
        )

    def finalize(self, now_s: float):
        return self.builder.finalize(now_s)
