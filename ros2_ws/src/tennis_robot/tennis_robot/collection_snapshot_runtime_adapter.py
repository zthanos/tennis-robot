"""Thin scan-runtime bridge; it owns neither ROS subscriptions nor scan policy."""

from __future__ import annotations

from tennis_robot.collection_route_types import (
    CollectionRouteConfiguration,
    LocalizationXYCovariance,
    Pose2D,
    SpatialObservationRejection,
    SpatialObservationRejectionCode,
)
from tennis_robot.collection_scan_snapshot import CourtHalfBoundary, ScanSnapshotBuilder
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
        calibration_id = getattr(frame, "calibration_id", "")
        configuration_id = getattr(frame, "configuration_id", "")
        if (
            frame.spatial_targets_healthy
            and isinstance(calibration_id, str) and calibration_id
            and isinstance(configuration_id, str) and configuration_id
            and scan_step_id
        ):
            # Any valid perception frame from this heading marks the sector
            # covered — even a heartbeat, or a frame whose detections are all
            # rejected (e.g. cross-half balls). Coverage is sector observation,
            # not ball acceptance; gating it on accepted balls made a scan of an
            # empty own-half fail whenever the far half was visible.
            builder.record_visited_step(scan_step_id)
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
        calibration_id = getattr(frame, "calibration_id", "")
        configuration_id = getattr(frame, "configuration_id", "")
        if not isinstance(calibration_id, str) or not calibration_id or not isinstance(configuration_id, str) or not configuration_id:
            return self._reject(
                SpatialObservationRejectionCode.PERCEPTION_METADATA_REJECTED,
                detection_index,
                rgb_timestamp_s,
                "calibration_identity_missing",
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
        if abs(transform.timestamp_s - rgb_timestamp_s) > self.validation_config.max_detection_to_tf_age_s:
            return self._reject(
                SpatialObservationRejectionCode.PERCEPTION_TF_REJECTED,
                detection_index,
                rgb_timestamp_s,
                "detection_to_tf_age_exceeded",
            )

        detection = C1ValidatedSpatialDetection(
            (detection_message.position_x, detection_message.position_y, detection_message.position_z),
            tuple(detection_message.position_covariance),
            detection_message.confidence,
            rgb_timestamp_s,
            _stamp(detection_message.matched_depth_stamp),
            frame.header.frame_id,
            calibration_id,
            configuration_id,
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
        expected_scan_step_ids: tuple[str, ...],
        court_half_boundary: CourtHalfBoundary,
        tf_provider,
        robot_pose_provider=None,
        map_frame: str = "map",
    ) -> None:
        self.builder = ScanSnapshotBuilder(
            scan_id=scan_id,
            scan_timestamp_s=scan_timestamp_s,
            robot_pose_at_scan=robot_pose_at_scan,
            configuration_snapshot=configuration_snapshot,
            expected_scan_step_ids=expected_scan_step_ids,
            court_half_boundary=court_half_boundary,
            map_frame=map_frame,
        )
        # Validation thresholds and localization covariance have a single
        # source of truth: the immutable configuration snapshot itself.
        self.adapter = CollectionSnapshotRuntimeAdapter(
            tf_provider=tf_provider,
            validation_config=configuration_snapshot.perception_spatial_validation,
            localization_xy_covariance=configuration_snapshot.gazebo_snapshot.localization_xy_covariance,
        )
        if robot_pose_provider is not None and not callable(robot_pose_provider):
            raise TypeError("robot_pose_provider must be callable")
        self._robot_pose_provider = robot_pose_provider

    def start(self, scan_timestamp_s: float) -> None:
        self.builder.start(scan_timestamp_s)

    def forward_frame(self, frame, *, scan_step_id: str | None) -> None:
        self.adapter.forward(
            scan_id=self.builder.scan_id,
            frame=frame,
            scan_step_id=scan_step_id,
            builder=self.builder,
        )

    def finalize(self, now_s: float):
        robot_pose = self._robot_pose_provider() if self._robot_pose_provider is not None else None
        return self.builder.finalize(now_s, robot_pose_at_scan=robot_pose)

    @property
    def diagnostics(self) -> dict:
        return self.builder.diagnostics
