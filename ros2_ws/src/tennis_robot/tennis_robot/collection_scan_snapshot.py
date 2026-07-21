"""Pure Gazebo-MVP scan aggregation and scan-local duplicate fusion."""
from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
import math
from tennis_robot.collection_route_types import (AcceptedSpatialObservation, CollectionRouteConfiguration, DomainValidationError, Point2D, PositionCovariance2D, Pose2D, ScanSnapshot, SnapshotBall, SpatialObservationRejection, SpatialObservationRejectionCode)

_SINGULAR_RELATIVE_TOLERANCE = 1e-12

class ScanSnapshotFailureCode(str, Enum):
    TIMEOUT = "scan_timeout"
    INSUFFICIENT_COVERAGE = "insufficient_coverage"
    UNKNOWN_SCAN_STEP = "unknown_scan_step"
    LIFECYCLE = "lifecycle"

class ScanSnapshotFailure(ValueError):
    def __init__(self, code: ScanSnapshotFailureCode, detail: str = ""):
        self.code = code; super().__init__(detail or code.value)

@dataclass(frozen=True)
class CourtHalfBoundary:
    """The robot-side half plane of the court, injected from the court model.

    ``robot_side_sign`` declares which sign of the cross product
    ``(net_b - net_a) x (point - net_a)`` is the robot's half. A point exactly
    on the net line is not on the robot's side.
    """
    net_point_a: Point2D
    net_point_b: Point2D
    robot_side_sign: int

    def __post_init__(self) -> None:
        if not isinstance(self.net_point_a, Point2D) or not isinstance(self.net_point_b, Point2D):
            raise DomainValidationError("court half boundary needs two net Point2D endpoints")
        if self.robot_side_sign not in (-1, 1) or isinstance(self.robot_side_sign, bool):
            raise DomainValidationError("robot_side_sign must be -1 or +1")
        if math.hypot(self.net_point_b.x_m - self.net_point_a.x_m, self.net_point_b.y_m - self.net_point_a.y_m) <= 1e-9:
            raise DomainValidationError("net endpoints must be distinct")

    def contains(self, point: Point2D) -> bool:
        cross = ((self.net_point_b.x_m - self.net_point_a.x_m) * (point.y_m - self.net_point_a.y_m)
                 - (self.net_point_b.y_m - self.net_point_a.y_m) * (point.x_m - self.net_point_a.x_m))
        return cross * self.robot_side_sign > 0.0

@dataclass
class _Track:
    observations: list[AcceptedSpatialObservation]

    @property
    def steps(self): return {obs.scan_step_id for obs in self.observations}

    def fused(self):
        # Information-form fusion of the (measurement-only) 2D covariances.
        a=b=c=ux=uy=0.0
        for obs in self.observations:
            cov=obs.position_covariance_map_xy; det=cov.xx*cov.yy-cov.xy*cov.xy
            ixx,ixy,iyy=cov.yy/det,-cov.xy/det,cov.xx/det
            a+=ixx; b+=ixy; c+=iyy; ux+=ixx*obs.position_map_xy.x_m+ixy*obs.position_map_xy.y_m; uy+=ixy*obs.position_map_xy.x_m+iyy*obs.position_map_xy.y_m
        det=a*c-b*b
        cov=PositionCovariance2D(c/det,-b/det,a/det)
        return Point2D((c*ux-b*uy)/det,(-b*ux+a*uy)/det),cov

class ScanSnapshotBuilder:
    """Aggregates adapter outputs for one scan.

    Constructor arguments are scan-instance data only; the coverage fraction and
    scan timeout are read exclusively from ``configuration_snapshot.scan`` so a
    snapshot can never disagree with its own serialized configuration.

    Confirmation semantics: a ball is confirmed only by observations from the
    required minimum number of DISTINCT scan steps — repeated observations of a
    track within the same step never add confirmations (they are dropped
    silently and recorded in ``duplicate_step_observations`` telemetry). The
    published ball confidence is the arithmetic mean of the contributing
    observation confidences.

    Covariance semantics: observations carry measurement-only covariance; the
    configured ``localization_xy_covariance`` is added exactly once per ball at
    ``finalize``, so information fusion can never drive the published
    covariance below the shared localization budget.
    """
    def __init__(self, *, scan_id:str, scan_timestamp_s:float, robot_pose_at_scan:Pose2D, configuration_snapshot:CollectionRouteConfiguration, expected_scan_step_ids:tuple[str,...], court_half_boundary:CourtHalfBoundary, map_frame:str="map"):
        if not isinstance(configuration_snapshot, CollectionRouteConfiguration): raise ScanSnapshotFailure(ScanSnapshotFailureCode.LIFECYCLE,"configuration_snapshot must be CollectionRouteConfiguration")
        if not isinstance(court_half_boundary, CourtHalfBoundary): raise ScanSnapshotFailure(ScanSnapshotFailureCode.LIFECYCLE,"court_half_boundary must be CourtHalfBoundary")
        if not scan_id or map_frame != "map" or not math.isfinite(scan_timestamp_s) or not isinstance(expected_scan_step_ids, tuple) or not expected_scan_step_ids or any(not isinstance(x,str) or not x for x in expected_scan_step_ids) or len(set(expected_scan_step_ids)) != len(expected_scan_step_ids): raise ScanSnapshotFailure(ScanSnapshotFailureCode.LIFECYCLE,"invalid scan lifecycle configuration")
        self.scan_id,self.scan_timestamp_s,self.robot_pose_at_scan=scan_id,scan_timestamp_s,robot_pose_at_scan
        self.configuration_snapshot,self.map_frame=configuration_snapshot,map_frame
        self.expected_scan_step_ids=expected_scan_step_ids
        self.court_half_boundary=court_half_boundary
        self._accepted_steps=set(); self._tracks=[]; self._state="collecting"; self.rejections=[]
        self.duplicate_step_observations=[]

    @property
    def required_coverage_fraction(self) -> float: return self.configuration_snapshot.scan.required_coverage_fraction

    @property
    def scan_timeout_s(self) -> float: return self.configuration_snapshot.scan.scan_timeout_s

    def add(self, item):
        if self._state!="collecting": raise ScanSnapshotFailure(ScanSnapshotFailureCode.LIFECYCLE,"scan is not collecting")
        if isinstance(item, SpatialObservationRejection): self.rejections.append(item); return
        if not isinstance(item, AcceptedSpatialObservation) or item.scan_id!=self.scan_id: raise ScanSnapshotFailure(ScanSnapshotFailureCode.LIFECYCLE,"observation scan mismatch")
        if item.scan_step_id not in self.expected_scan_step_ids:
            self.rejections.append(SpatialObservationRejection(SpatialObservationRejectionCode.PERCEPTION_METADATA_REJECTED,item.detection_index,item.rgb_timestamp_s,"unknown_scan_step")); return
        if not self.court_half_boundary.contains(item.position_map_xy):
            self.rejections.append(SpatialObservationRejection(SpatialObservationRejectionCode.PERCEPTION_METADATA_REJECTED,item.detection_index,item.rgb_timestamp_s,"opposite_court_half")); return
        cov=item.position_covariance_map_xy; det=cov.xx*cov.yy-cov.xy*cov.xy
        if det <= _SINGULAR_RELATIVE_TOLERANCE*max(cov.xx*cov.yy, cov.xy*cov.xy):
            self.rejections.append(SpatialObservationRejection(SpatialObservationRejectionCode.PERCEPTION_METADATA_REJECTED,item.detection_index,item.rgb_timestamp_s,"singular_covariance")); return
        candidates=[]
        for track in self._tracks:
            point,fused_cov=track.fused(); dx=item.position_map_xy.x_m-point.x_m; dy=item.position_map_xy.y_m-point.y_m
            cc=PositionCovariance2D(fused_cov.xx+cov.xx,fused_cov.xy+cov.xy,fused_cov.yy+cov.yy); cdet=cc.xx*cc.yy-cc.xy*cc.xy
            d2=(cc.yy*dx*dx-2*cc.xy*dx*dy+cc.xx*dy*dy)/cdet
            if d2<=self.configuration_snapshot.gazebo_snapshot.association.association_mahalanobis_gate_chi2: candidates.append(track)
        if len(candidates)>1:
            self.rejections.append(SpatialObservationRejection(SpatialObservationRejectionCode.PERCEPTION_METADATA_REJECTED,item.detection_index,item.rgb_timestamp_s,"ambiguous_scan_local_association")); return
        # The observation is now finally accepted; only such observations may
        # count their scan step toward coverage.
        self._accepted_steps.add(item.scan_step_id)
        if not candidates: self._tracks.append(_Track([item])); return
        track=candidates[0]
        if item.scan_step_id in track.steps:
            self.duplicate_step_observations.append(item); return
        track.observations.append(item)

    def record_empty_step(self, scan_step_id: str) -> None:
        """Count one valid empty perception heartbeat toward scan coverage."""
        if self._state != "collecting":
            raise ScanSnapshotFailure(ScanSnapshotFailureCode.LIFECYCLE, "scan is not collecting")
        if scan_step_id not in self.expected_scan_step_ids:
            raise ScanSnapshotFailure(ScanSnapshotFailureCode.UNKNOWN_SCAN_STEP)
        self._accepted_steps.add(scan_step_id)

    def finalize(self, now_s:float):
        if self._state!="collecting": raise ScanSnapshotFailure(ScanSnapshotFailureCode.LIFECYCLE,"scan already finalized")
        if not math.isfinite(now_s) or now_s-self.scan_timestamp_s > self.scan_timeout_s:
            self._state="failed"; raise ScanSnapshotFailure(ScanSnapshotFailureCode.TIMEOUT)
        if len(self._accepted_steps)/len(self.expected_scan_step_ids) < self.required_coverage_fraction:
            self._state="failed"; raise ScanSnapshotFailure(ScanSnapshotFailureCode.INSUFFICIENT_COVERAGE)
        balls=[]
        association = self.configuration_snapshot.gazebo_snapshot.association
        localization = self.configuration_snapshot.gazebo_snapshot.localization_xy_covariance.covariance
        minimum=max(self.configuration_snapshot.scan.minimum_confirmation_count,association.min_confirmations,association.min_distinct_scan_steps)
        for track in self._tracks:
            if len(track.steps)<minimum: continue
            point,measurement_cov=track.fused(); confidence=sum(o.confidence for o in track.observations)/len(track.observations)
            cov=PositionCovariance2D(measurement_cov.xx+localization.xx,measurement_cov.xy+localization.xy,measurement_cov.yy+localization.yy)
            balls.append(SnapshotBall(f"{self.scan_id}/target-{len(balls)+1}",point,confidence,cov))
        self._state="finalized"
        return ScanSnapshot(self.scan_id,self.scan_timestamp_s,self.map_frame,self.robot_pose_at_scan,tuple(balls),self.configuration_snapshot)
