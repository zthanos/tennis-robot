"""Pure Gazebo-MVP scan aggregation and scan-local duplicate fusion."""
from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
import math
from tennis_robot.collection_route_types import (AcceptedSpatialObservation, CollectionRouteConfiguration, DomainValidationError, Point2D, PositionCovariance2D, Pose2D, ScanSnapshot, SnapshotBall, SpatialObservationRejection, SpatialObservationRejectionCode)

class ScanSnapshotFailureCode(str, Enum):
    TIMEOUT = "scan_timeout"
    INSUFFICIENT_COVERAGE = "insufficient_coverage"
    UNKNOWN_SCAN_STEP = "unknown_scan_step"
    LIFECYCLE = "lifecycle"

class ScanSnapshotFailure(ValueError):
    def __init__(self, code: ScanSnapshotFailureCode, detail: str = ""):
        self.code = code; super().__init__(detail or code.value)

@dataclass
class _Track:
    observations: list[AcceptedSpatialObservation]

    @property
    def steps(self): return {obs.scan_step_id for obs in self.observations}

    def fused(self):
        # Information-form fusion for independent 2D observations.
        a=b=c=ux=uy=0.0
        for obs in self.observations:
            cov=obs.position_covariance_map_xy; det=cov.xx*cov.yy-cov.xy*cov.xy
            ixx,ixy,iyy=cov.yy/det,-cov.xy/det,cov.xx/det
            a+=ixx; b+=ixy; c+=iyy; ux+=ixx*obs.position_map_xy.x_m+ixy*obs.position_map_xy.y_m; uy+=ixy*obs.position_map_xy.x_m+iyy*obs.position_map_xy.y_m
        det=a*c-b*b
        cov=PositionCovariance2D(c/det,-b/det,a/det)
        return Point2D((c*ux-b*uy)/det,(-b*ux+a*uy)/det),cov

class ScanSnapshotBuilder:
    def __init__(self, *, scan_id:str, scan_timestamp_s:float, robot_pose_at_scan:Pose2D, configuration_snapshot:CollectionRouteConfiguration, expected_scan_step_ids:tuple[str,...], required_coverage_fraction:float, scan_timeout_s:float, map_frame:str="map"):
        if not scan_id or map_frame != "map" or not math.isfinite(scan_timestamp_s) or not isinstance(expected_scan_step_ids, tuple) or not expected_scan_step_ids or any(not isinstance(x,str) or not x for x in expected_scan_step_ids) or len(set(expected_scan_step_ids)) != len(expected_scan_step_ids) or not math.isfinite(required_coverage_fraction) or not 0 < required_coverage_fraction <= 1 or not math.isfinite(scan_timeout_s) or scan_timeout_s <= 0: raise ScanSnapshotFailure(ScanSnapshotFailureCode.LIFECYCLE,"invalid scan lifecycle configuration")
        self.scan_id,self.scan_timestamp_s,self.robot_pose_at_scan=scan_id,scan_timestamp_s,robot_pose_at_scan
        self.configuration_snapshot,self.map_frame=configuration_snapshot,map_frame
        self.expected_scan_step_ids=expected_scan_step_ids; self.required_coverage_fraction=required_coverage_fraction; self.scan_timeout_s=scan_timeout_s
        self._accepted_steps=set(); self._tracks=[]; self._state="collecting"; self.rejections=[]
    def add(self, item):
        if self._state!="collecting": raise ScanSnapshotFailure(ScanSnapshotFailureCode.LIFECYCLE,"scan is not collecting")
        if isinstance(item, SpatialObservationRejection): self.rejections.append(item); return
        if not isinstance(item, AcceptedSpatialObservation) or item.scan_id!=self.scan_id: raise ScanSnapshotFailure(ScanSnapshotFailureCode.LIFECYCLE,"observation scan mismatch")
        if item.scan_step_id not in self.expected_scan_step_ids:
            self.rejections.append(SpatialObservationRejection(SpatialObservationRejectionCode.PERCEPTION_METADATA_REJECTED,item.detection_index,item.rgb_timestamp_s,"unknown_scan_step")); return
        self._accepted_steps.add(item.scan_step_id)
        candidates=[]
        for track in self._tracks:
            point,cov=track.fused(); dx=item.position_map_xy.x_m-point.x_m; dy=item.position_map_xy.y_m-point.y_m
            cc=PositionCovariance2D(cov.xx+item.position_covariance_map_xy.xx,cov.xy+item.position_covariance_map_xy.xy,cov.yy+item.position_covariance_map_xy.yy); det=cc.xx*cc.yy-cc.xy*cc.xy
            d2=(cc.yy*dx*dx-2*cc.xy*dx*dy+cc.xx*dy*dy)/det
            if d2<=self.configuration_snapshot.gazebo_snapshot.association.association_mahalanobis_gate_chi2: candidates.append(track)
        if len(candidates)>1:
            self.rejections.append(SpatialObservationRejection(SpatialObservationRejectionCode.PERCEPTION_METADATA_REJECTED,item.detection_index,item.rgb_timestamp_s,"ambiguous_scan_local_association")); return
        if not candidates: self._tracks.append(_Track([item])); return
        track=candidates[0]
        if item.scan_step_id not in track.steps: track.observations.append(item)
    def finalize(self, now_s:float):
        if self._state!="collecting": raise ScanSnapshotFailure(ScanSnapshotFailureCode.LIFECYCLE,"scan already finalized")
        if not math.isfinite(now_s) or now_s-self.scan_timestamp_s > self.scan_timeout_s:
            self._state="failed"; raise ScanSnapshotFailure(ScanSnapshotFailureCode.TIMEOUT)
        if len(self._accepted_steps)/len(self.expected_scan_step_ids) < self.required_coverage_fraction:
            self._state="failed"; raise ScanSnapshotFailure(ScanSnapshotFailureCode.INSUFFICIENT_COVERAGE)
        balls=[]
        association = self.configuration_snapshot.gazebo_snapshot.association
        minimum=max(self.configuration_snapshot.scan.minimum_confirmation_count,association.min_confirmations,association.min_distinct_scan_steps)
        for track in self._tracks:
            if len(track.steps)<minimum: continue
            point,cov=track.fused(); confidence=sum(o.confidence for o in track.observations)/len(track.observations)
            balls.append(SnapshotBall(f"{self.scan_id}/target-{len(balls)+1}",point,confidence,cov))
        self._state="finalized"
        return ScanSnapshot(self.scan_id,self.scan_timestamp_s,self.map_frame,self.robot_pose_at_scan,tuple(balls),self.configuration_snapshot)
