"""Pure, immutable contracts for continuous collection-route planning.

This module deliberately has no ROS dependencies.  It defines the persisted
scan and plan artifacts shared by the future snapshot builder, planner,
executor, and telemetry layers.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Any, Mapping, TypeVar

POSITION_TOLERANCE_M = 1e-6
ANGLE_TOLERANCE_RAD = 1e-6
PROGRESS_TOLERANCE_M = 1e-6

_T = TypeVar("_T")


class DomainValidationError(ValueError):
    """Raised when a serialized or constructed domain artifact is invalid."""


class _StringEnum(str, Enum):
    def __str__(self) -> str:
        return self.value


class BallStatus(_StringEnum):
    COVERED = "covered"
    DEFERRED = "deferred"
    UNREACHABLE = "unreachable"


class BallReasonCode(_StringEnum):
    SELECTED = "selected"
    ROUTE_CONFLICT = "route_conflict"
    PLANNING_BUDGET = "planning_budget"
    NO_CANDIDATE_FOUND = "no_candidate_found"
    KEEPOUT = "keepout"
    TURN_RADIUS = "turn_radius"
    NO_ENTRY = "no_entry"
    NO_EXIT = "no_exit"
    MECHANICAL_SPACING = "mechanical_spacing"


class PlanningStatus(_StringEnum):
    FEASIBLE = "feasible"
    PARTIAL = "partial"
    EMPTY_NO_BALLS = "empty_no_balls"
    EMPTY_NO_FEASIBLE_TARGETS = "empty_no_feasible_targets"
    PLANNING_TIMEOUT = "planning_timeout"
    PLANNING_FAILED = "planning_failed"


class PlanningSearchStatus(_StringEnum):
    COMPLETE = "complete"
    BUDGET_EXHAUSTED = "budget_exhausted"
    FAILED = "failed"


class RouteSegmentType(_StringEnum):
    CONNECTOR = "connector"
    FUNNEL_PASS = "funnel_pass"
    TERMINAL_CONNECTOR = "terminal_connector"


class ObstacleConstraintKind(_StringEnum):
    NONE = "none"
    COURT_BOUNDARY = "court_boundary"
    NET = "net"
    FENCE = "fence"
    STATIC_KEEPOUT = "static_keepout"
    COMPOSITE = "composite"


class SpatialObservationRejectionCode(_StringEnum):
    SPATIAL_TARGETS_UNHEALTHY = "spatial_targets_unhealthy"
    NON_SPATIAL_DETECTION = "non_spatial_detection"
    PERCEPTION_METADATA_REJECTED = "perception_metadata_rejected"
    PERCEPTION_TF_REJECTED = "perception_tf_rejected"
    FRAME_MISMATCH = "frame_mismatch"


def _finite(value: float, name: str, *, minimum: float | None = None) -> None:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise DomainValidationError(f"{name} must be a finite number")
    if minimum is not None and value < minimum:
        raise DomainValidationError(f"{name} must be >= {minimum}")


def _non_empty(value: str, name: str) -> None:
    if not isinstance(value, str) or not value:
        raise DomainValidationError(f"{name} must be a non-empty string")


def _tuple_of_strings(value: tuple[str, ...], name: str) -> None:
    if not isinstance(value, tuple) or any(not isinstance(item, str) or not item for item in value):
        raise DomainValidationError(f"{name} must be a tuple of non-empty strings")


def _unique(value: tuple[str, ...], name: str) -> None:
    if len(set(value)) != len(value):
        raise DomainValidationError(f"{name} must contain unique values")


def _fields(data: Mapping[str, Any], expected: set[str], name: str) -> None:
    if not isinstance(data, Mapping):
        raise DomainValidationError(f"{name} must be a mapping")
    actual = set(data)
    if actual != expected:
        raise DomainValidationError(f"{name} fields must be exactly {sorted(expected)}")


def _enum(value: Any, enum_type: type[_StringEnum], name: str) -> _StringEnum:
    try:
        return enum_type(value)
    except (TypeError, ValueError) as exc:
        raise DomainValidationError(f"invalid {name}: {value!r}") from exc


def _angular_distance(first: float, second: float) -> float:
    """Return the smallest absolute angular difference in radians."""
    return abs((first - second + math.pi) % (2 * math.pi) - math.pi)


@dataclass(frozen=True)
class Point2D:
    x_m: float
    y_m: float

    def __post_init__(self) -> None:
        _finite(self.x_m, "x_m")
        _finite(self.y_m, "y_m")

    def to_dict(self) -> dict[str, float]: return {"x_m": self.x_m, "y_m": self.y_m}
    @classmethod
    def from_dict(cls: type[_T], data: Mapping[str, Any]) -> _T:
        _fields(data, {"x_m", "y_m"}, "Point2D")
        return cls(**data)


@dataclass(frozen=True)
class Pose2D:
    x_m: float
    y_m: float
    yaw_rad: float

    def __post_init__(self) -> None:
        _finite(self.x_m, "x_m")
        _finite(self.y_m, "y_m")
        _finite(self.yaw_rad, "yaw_rad")
        normalized_yaw = (self.yaw_rad + math.pi) % (2.0 * math.pi) - math.pi
        object.__setattr__(self, "yaw_rad", normalized_yaw)

    def to_dict(self) -> dict[str, float]:
        return {"x_m": self.x_m, "y_m": self.y_m, "yaw_rad": self.yaw_rad}
    @classmethod
    def from_dict(cls: type[_T], data: Mapping[str, Any]) -> _T:
        _fields(data, {"x_m", "y_m", "yaw_rad"}, "Pose2D")
        return cls(**data)


@dataclass(frozen=True)
class PathPoint:
    pose: Pose2D

    def __post_init__(self) -> None:
        if not isinstance(self.pose, Pose2D): raise DomainValidationError("pose must be Pose2D")
    def to_dict(self) -> dict[str, Any]: return {"pose": self.pose.to_dict()}
    @classmethod
    def from_dict(cls: type[_T], data: Mapping[str, Any]) -> _T:
        _fields(data, {"pose"}, "PathPoint")
        return cls(Pose2D.from_dict(data["pose"]))


@dataclass(frozen=True)
class Path2D:
    points: tuple[PathPoint, ...]

    def __post_init__(self) -> None:
        if not isinstance(self.points, tuple) or any(not isinstance(point, PathPoint) for point in self.points):
            raise DomainValidationError("points must be a tuple of PathPoint")
    def validate_executable(self) -> None:
        if len(self.points) < 2: raise DomainValidationError("executable path needs at least two points")
    def to_dict(self) -> dict[str, Any]: return {"points": [point.to_dict() for point in self.points]}
    @classmethod
    def from_dict(cls: type[_T], data: Mapping[str, Any]) -> _T:
        _fields(data, {"points"}, "Path2D")
        if not isinstance(data["points"], list): raise DomainValidationError("points must be a JSON array")
        return cls(tuple(PathPoint.from_dict(point) for point in data["points"]))


@dataclass(frozen=True)
class PositionCovariance2D:
    xx: float
    xy: float
    yy: float

    def __post_init__(self) -> None:
        _finite(self.xx, "xx", minimum=0.0); _finite(self.xy, "xy"); _finite(self.yy, "yy", minimum=0.0)
        determinant = self.xx * self.yy - self.xy * self.xy
        scale = max(self.xx * self.yy, self.xy * self.xy)
        if determinant < -1e-12 * scale:
            raise DomainValidationError("covariance must be positive-semidefinite")
    def to_dict(self) -> dict[str, float]: return {"xx": self.xx, "xy": self.xy, "yy": self.yy}
    @classmethod
    def from_dict(cls: type[_T], data: Mapping[str, Any]) -> _T:
        _fields(data, {"xx", "xy", "yy"}, "PositionCovariance2D")
        return cls(**data)


@dataclass(frozen=True)
class LocalizationXYCovariance:
    covariance: PositionCovariance2D

    def __post_init__(self) -> None:
        if not isinstance(self.covariance, PositionCovariance2D):
            raise DomainValidationError("localization covariance must be PositionCovariance2D")

    def to_dict(self) -> dict[str, Any]:
        return {"covariance": self.covariance.to_dict()}

    @classmethod
    def from_dict(cls: type[_T], data: Mapping[str, Any]) -> _T:
        _fields(data, {"covariance"}, "LocalizationXYCovariance")
        return cls(PositionCovariance2D.from_dict(data["covariance"]))


@dataclass(frozen=True)
class SnapshotAssociationConfiguration:
    association_mahalanobis_gate_chi2: float
    min_confirmations: int
    min_distinct_scan_steps: int

    def __post_init__(self) -> None:
        _finite(self.association_mahalanobis_gate_chi2, "association_mahalanobis_gate_chi2", minimum=0.0)
        if self.association_mahalanobis_gate_chi2 <= 0.0:
            raise DomainValidationError("association_mahalanobis_gate_chi2 must be positive")
        for name in ("min_confirmations", "min_distinct_scan_steps"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise DomainValidationError(f"{name} must be positive int")

    def to_dict(self) -> dict[str, Any]:
        return {
            "association_mahalanobis_gate_chi2": self.association_mahalanobis_gate_chi2,
            "min_confirmations": self.min_confirmations,
            "min_distinct_scan_steps": self.min_distinct_scan_steps,
        }

    @classmethod
    def from_dict(cls: type[_T], data: Mapping[str, Any]) -> _T:
        _fields(data, set(cls.__dataclass_fields__), "SnapshotAssociationConfiguration")
        return cls(**data)


@dataclass(frozen=True)
class GazeboSnapshotConfiguration:
    configuration_id: str
    localization_xy_covariance: LocalizationXYCovariance
    association: SnapshotAssociationConfiguration

    def __post_init__(self) -> None:
        _non_empty(self.configuration_id, "configuration_id")
        if not isinstance(self.localization_xy_covariance, LocalizationXYCovariance) or not isinstance(self.association, SnapshotAssociationConfiguration):
            raise DomainValidationError("invalid Gazebo snapshot configuration")

    def to_dict(self) -> dict[str, Any]:
        return {
            "configuration_id": self.configuration_id,
            "localization_xy_covariance": self.localization_xy_covariance.to_dict(),
            "association": self.association.to_dict(),
        }

    @classmethod
    def from_dict(cls: type[_T], data: Mapping[str, Any]) -> _T:
        _fields(data, set(cls.__dataclass_fields__), "GazeboSnapshotConfiguration")
        return cls(
            configuration_id=data["configuration_id"],
            localization_xy_covariance=LocalizationXYCovariance.from_dict(
                data["localization_xy_covariance"]
            ),
            association=SnapshotAssociationConfiguration.from_dict(data["association"]),
        )


@dataclass(frozen=True)
class AcceptedSpatialObservation:
    scan_id: str
    detection_index: int
    rgb_timestamp_s: float
    matched_depth_timestamp_s: float
    position_map_xy: Point2D
    position_covariance_map_xy: PositionCovariance2D
    confidence: float
    scan_step_id: str
    calibration_id: str
    configuration_id: str

    def __post_init__(self) -> None:
        _non_empty(self.scan_id, "scan_id"); _non_empty(self.scan_step_id, "scan_step_id")
        _non_empty(self.calibration_id, "calibration_id"); _non_empty(self.configuration_id, "configuration_id")
        for name in ("rgb_timestamp_s", "matched_depth_timestamp_s", "confidence"):
            _finite(getattr(self, name), name)
        if isinstance(self.detection_index, bool) or not isinstance(self.detection_index, int) or self.detection_index < 0:
            raise DomainValidationError("detection_index must be non-negative int")
        if not 0.0 <= self.confidence <= 1.0:
            raise DomainValidationError("confidence must be in [0, 1]")


@dataclass(frozen=True)
class SpatialObservationRejection:
    code: SpatialObservationRejectionCode
    detection_index: int
    rgb_timestamp_s: float
    detail: str

    def __post_init__(self) -> None:
        if not isinstance(self.code, SpatialObservationRejectionCode):
            raise DomainValidationError("invalid spatial observation rejection code")
        if isinstance(self.detection_index, bool) or not isinstance(self.detection_index, int) or self.detection_index < 0:
            raise DomainValidationError("detection_index must be non-negative int")
        _finite(self.rgb_timestamp_s, "rgb_timestamp_s"); _non_empty(self.detail, "detail")


@dataclass(frozen=True)
class ExecutionProfile:
    nominal_speed_mps: float; min_speed_mps: float; max_speed_mps: float; nominal_speed_warning_tolerance_mps: float
    max_acceleration_mps2: float; max_deceleration_mps2: float
    required_entry_m: float; required_run_in_m: float; required_run_out_m: float
    max_curvature_per_m: float; max_lateral_error_m: float; max_heading_error_rad: float
    allow_reversing: bool; allow_standalone_rotate: bool

    def __post_init__(self) -> None:
        for name in ("nominal_speed_mps", "min_speed_mps", "max_speed_mps", "nominal_speed_warning_tolerance_mps", "max_acceleration_mps2", "max_deceleration_mps2", "required_entry_m", "required_run_in_m", "required_run_out_m", "max_curvature_per_m", "max_lateral_error_m", "max_heading_error_rad"):
            _finite(getattr(self, name), name, minimum=0.0)
        if not self.min_speed_mps <= self.nominal_speed_mps <= self.max_speed_mps:
            raise DomainValidationError("speed limits must satisfy min <= nominal <= max")
        if not isinstance(self.allow_reversing, bool) or not isinstance(self.allow_standalone_rotate, bool):
            raise DomainValidationError("route permissions must be bool")

    def to_dict(self) -> dict[str, Any]: return {name: getattr(self, name) for name in self.__dataclass_fields__}
    @classmethod
    def from_dict(cls: type[_T], data: Mapping[str, Any]) -> _T:
        _fields(data, set(cls.__dataclass_fields__), "ExecutionProfile")
        return cls(**data)


@dataclass(frozen=True)
class ObstacleConstraint:
    kind: ObstacleConstraintKind
    source_ids: tuple[str, ...]
    required_clearance_m: float

    def __post_init__(self) -> None:
        if not isinstance(self.kind, ObstacleConstraintKind): raise DomainValidationError("kind must be ObstacleConstraintKind")
        _tuple_of_strings(self.source_ids, "source_ids"); _unique(self.source_ids, "source_ids")
        _finite(self.required_clearance_m, "required_clearance_m", minimum=0.0)
        if self.kind is ObstacleConstraintKind.NONE and (self.source_ids or self.required_clearance_m != 0):
            raise DomainValidationError("none constraint requires empty ids and zero clearance")
        if self.kind is not ObstacleConstraintKind.NONE and not self.source_ids:
            raise DomainValidationError("non-none constraint requires source_ids")
    def to_dict(self) -> dict[str, Any]: return {"kind": self.kind.value, "source_ids": list(self.source_ids), "required_clearance_m": self.required_clearance_m}
    @classmethod
    def from_dict(cls: type[_T], data: Mapping[str, Any]) -> _T:
        _fields(data, {"kind", "source_ids", "required_clearance_m"}, "ObstacleConstraint")
        if not isinstance(data["source_ids"], list): raise DomainValidationError("source_ids must be a JSON array")
        return cls(_enum(data["kind"], ObstacleConstraintKind, "kind"), tuple(data["source_ids"]), data["required_clearance_m"])


@dataclass(frozen=True)
class MechanicalConfiguration:
    capture_half_width_m: float; ball_radius_m: float; robot_length_m: float; robot_width_m: float
    funnel_length_m: float; funnel_width_m: float; safety_margin_m: float
    minimum_turning_radius_m: float; maximum_curvature_per_m: float
    minimum_entry_m: float; minimum_run_in_m: float; minimum_run_out_m: float; minimum_mechanical_ball_spacing_m: float
    def __post_init__(self) -> None:
        for name in self.__dataclass_fields__: _finite(getattr(self, name), name, minimum=0.0)
        for name in ("capture_half_width_m", "ball_radius_m", "robot_length_m", "robot_width_m", "funnel_length_m", "funnel_width_m", "minimum_turning_radius_m"):
            if getattr(self, name) <= 0: raise DomainValidationError(f"{name} must be positive")
    def to_dict(self) -> dict[str, Any]: return {name: getattr(self, name) for name in self.__dataclass_fields__}
    @classmethod
    def from_dict(cls: type[_T], data: Mapping[str, Any]) -> _T: _fields(data, set(cls.__dataclass_fields__), "MechanicalConfiguration"); return cls(**data)


@dataclass(frozen=True)
class UncertaintyConfiguration:
    ball_position_uncertainty_m: float; localization_uncertainty_m: float; tracking_uncertainty_m: float
    def __post_init__(self) -> None:
        for name in self.__dataclass_fields__: _finite(getattr(self, name), name, minimum=0.0)
    def to_dict(self) -> dict[str, Any]: return {name: getattr(self, name) for name in self.__dataclass_fields__}
    @classmethod
    def from_dict(cls: type[_T], data: Mapping[str, Any]) -> _T: _fields(data, set(cls.__dataclass_fields__), "UncertaintyConfiguration"); return cls(**data)


@dataclass(frozen=True)
class SafetyConfiguration:
    static_obstacle_clearance_m: float; costmap_inflation_m: float; trajectory_tube_radius_m: float; progress_window_m: float; max_parallel_heading_error_rad: float
    collector_start_timeout_s: float; collector_stop_timeout_s: float; safety_pause_timeout_s: float
    def __post_init__(self) -> None:
        for name in self.__dataclass_fields__: _finite(getattr(self, name), name, minimum=0.0)
        for name in ("collector_start_timeout_s", "collector_stop_timeout_s", "safety_pause_timeout_s"):
            if getattr(self, name) <= 0: raise DomainValidationError(f"{name} must be positive")
    def to_dict(self) -> dict[str, Any]: return {name: getattr(self, name) for name in self.__dataclass_fields__}
    @classmethod
    def from_dict(cls: type[_T], data: Mapping[str, Any]) -> _T: _fields(data, set(cls.__dataclass_fields__), "SafetyConfiguration"); return cls(**data)


@dataclass(frozen=True)
class ScanConfiguration:
    required_coverage_fraction: float; scan_timeout_s: float; minimum_confirmation_count: int
    def __post_init__(self) -> None:
        _finite(self.required_coverage_fraction, "required_coverage_fraction")
        _finite(self.scan_timeout_s, "scan_timeout_s")
        if not 0 < self.required_coverage_fraction <= 1: raise DomainValidationError("coverage must be in (0, 1]")
        if self.scan_timeout_s <= 0: raise DomainValidationError("scan_timeout_s must be positive")
        if isinstance(self.minimum_confirmation_count, bool) or not isinstance(self.minimum_confirmation_count, int) or self.minimum_confirmation_count <= 0: raise DomainValidationError("minimum_confirmation_count must be positive int")
    def to_dict(self) -> dict[str, Any]: return {name: getattr(self, name) for name in self.__dataclass_fields__}
    @classmethod
    def from_dict(cls: type[_T], data: Mapping[str, Any]) -> _T: _fields(data, set(cls.__dataclass_fields__), "ScanConfiguration"); return cls(**data)


@dataclass(frozen=True)
class FeasibilityConfiguration:
    heading_sample_count: int
    confidence_multiplier: float
    tracking_lateral_error_bound_m: float
    capture_safety_margin_m: float
    footprint_clearance_radius_m: float
    tangent_activation_distance_m: float
    max_parallel_heading_error_rad: float

    def __post_init__(self) -> None:
        if isinstance(self.heading_sample_count, bool) or not isinstance(self.heading_sample_count, int) or self.heading_sample_count <= 0:
            raise DomainValidationError("heading_sample_count must be positive int")
        for name in (
            "confidence_multiplier",
            "tracking_lateral_error_bound_m",
            "capture_safety_margin_m",
            "footprint_clearance_radius_m",
            "tangent_activation_distance_m",
            "max_parallel_heading_error_rad",
        ):
            _finite(getattr(self, name), name, minimum=0.0)
        if self.footprint_clearance_radius_m <= 0.0:
            raise DomainValidationError("footprint_clearance_radius_m must be positive")

    def to_dict(self) -> dict[str, Any]:
        return {name: getattr(self, name) for name in self.__dataclass_fields__}

    @classmethod
    def from_dict(cls: type[_T], data: Mapping[str, Any]) -> _T:
        _fields(data, set(cls.__dataclass_fields__), "FeasibilityConfiguration")
        return cls(**data)


@dataclass(frozen=True)
class ConnectorConfiguration:
    max_connector_length_m: float
    max_connector_arc_angle_rad: float
    max_connector_total_turn_rad: float

    def __post_init__(self) -> None:
        for name in self.__dataclass_fields__:
            _finite(getattr(self, name), name, minimum=0.0)
            if getattr(self, name) <= 0.0:
                raise DomainValidationError(f"{name} must be positive")

    def to_dict(self) -> dict[str, Any]:
        return {name: getattr(self, name) for name in self.__dataclass_fields__}

    @classmethod
    def from_dict(cls: type[_T], data: Mapping[str, Any]) -> _T:
        _fields(data, set(cls.__dataclass_fields__), "ConnectorConfiguration")
        return cls(**data)


@dataclass(frozen=True)
class GlobalRouteSearchConfiguration:
    max_search_expansions: int
    terminal_run_out_m: float
    connector_nominal_speed_m_s: float
    crossing_nominal_speed_m_s: float
    turn_energy_equivalent_m_per_rad: float
    weight_length: float
    weight_time: float
    weight_curvature: float
    weight_energy: float
    weight_pass_count: float

    def __post_init__(self) -> None:
        if isinstance(self.max_search_expansions, bool) or not isinstance(self.max_search_expansions, int) or self.max_search_expansions <= 0:
            raise DomainValidationError("max_search_expansions must be positive int")
        for name in ("terminal_run_out_m", "connector_nominal_speed_m_s", "crossing_nominal_speed_m_s"):
            _finite(getattr(self, name), name, minimum=0.0)
            if getattr(self, name) <= 0.0:
                raise DomainValidationError(f"{name} must be positive")
        for name in ("turn_energy_equivalent_m_per_rad", "weight_length", "weight_time", "weight_curvature", "weight_energy", "weight_pass_count"):
            _finite(getattr(self, name), name, minimum=0.0)
        if not any(getattr(self, name) > 0.0 for name in ("weight_length", "weight_time", "weight_curvature", "weight_energy", "weight_pass_count")):
            raise DomainValidationError("at least one global route search weight must be positive")

    def to_dict(self) -> dict[str, Any]:
        return {name: getattr(self, name) for name in self.__dataclass_fields__}

    @classmethod
    def from_dict(cls: type[_T], data: Mapping[str, Any]) -> _T:
        _fields(data, set(cls.__dataclass_fields__), "GlobalRouteSearchConfiguration")
        return cls(**data)


@dataclass(frozen=True)
class SharedPassConfiguration:
    max_shared_pass_balls: int
    max_shared_pass_candidates: int
    minimum_mechanical_ball_spacing_m: float

    def __post_init__(self) -> None:
        for name, minimum in (("max_shared_pass_balls", 2), ("max_shared_pass_candidates", 1)):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
                raise DomainValidationError(f"{name} must be int >= {minimum}")
        _finite(self.minimum_mechanical_ball_spacing_m, "minimum_mechanical_ball_spacing_m", minimum=0.0)
        if self.minimum_mechanical_ball_spacing_m <= 0.0:
            raise DomainValidationError("minimum_mechanical_ball_spacing_m must be positive")

    def to_dict(self) -> dict[str, Any]:
        return {name: getattr(self, name) for name in self.__dataclass_fields__}

    @classmethod
    def from_dict(cls: type[_T], data: Mapping[str, Any]) -> _T:
        _fields(data, set(cls.__dataclass_fields__), "SharedPassConfiguration")
        return cls(**data)


@dataclass(frozen=True)
class FollowUpConfiguration:
    """Explicit bounded lifecycle policy for the pure route executor."""

    enabled: bool
    max_total_runs: int

    def __post_init__(self) -> None:
        if not isinstance(self.enabled, bool):
            raise DomainValidationError("follow-up enabled must be bool")
        if isinstance(self.max_total_runs, bool) or not isinstance(self.max_total_runs, int) or self.max_total_runs <= 0:
            raise DomainValidationError("max_total_runs must be positive int")
        if not self.enabled and self.max_total_runs != 1:
            raise DomainValidationError("disabled follow-up requires max_total_runs == 1")

    def to_dict(self) -> dict[str, Any]:
        return {name: getattr(self, name) for name in self.__dataclass_fields__}

    @classmethod
    def from_dict(cls: type[_T], data: Mapping[str, Any]) -> _T:
        _fields(data, set(cls.__dataclass_fields__), "FollowUpConfiguration")
        return cls(**data)


@dataclass(frozen=True)
class PlanningConfiguration:
    default_execution_profile: ExecutionProfile; maximum_planning_time_s: float; maximum_candidate_count: int
    cost_weight_length: float; cost_weight_time: float; cost_weight_curvature: float; cost_weight_energy: float; cost_weight_passes: float
    def __post_init__(self) -> None:
        if not isinstance(self.default_execution_profile, ExecutionProfile): raise DomainValidationError("default_execution_profile must be ExecutionProfile")
        for name in ("maximum_planning_time_s", "cost_weight_length", "cost_weight_time", "cost_weight_curvature", "cost_weight_energy", "cost_weight_passes"): _finite(getattr(self, name), name, minimum=0.0)
        if self.maximum_planning_time_s <= 0: raise DomainValidationError("maximum_planning_time_s must be positive")
        if isinstance(self.maximum_candidate_count, bool) or not isinstance(self.maximum_candidate_count, int) or self.maximum_candidate_count <= 0: raise DomainValidationError("maximum_candidate_count must be positive int")
    def to_dict(self) -> dict[str, Any]:
        data = {name: getattr(self, name) for name in self.__dataclass_fields__}; data["default_execution_profile"] = self.default_execution_profile.to_dict(); return data
    @classmethod
    def from_dict(cls: type[_T], data: Mapping[str, Any]) -> _T:
        _fields(data, set(cls.__dataclass_fields__), "PlanningConfiguration"); values = dict(data); values["default_execution_profile"] = ExecutionProfile.from_dict(values["default_execution_profile"]); return cls(**values)


@dataclass(frozen=True)
class CollectionRouteConfiguration:
    schema_version: str; mechanical: MechanicalConfiguration; uncertainty: UncertaintyConfiguration; safety: SafetyConfiguration; scan: ScanConfiguration; feasibility: FeasibilityConfiguration; connector: ConnectorConfiguration; global_route_search: GlobalRouteSearchConfiguration; shared_pass: SharedPassConfiguration; follow_up: FollowUpConfiguration; planning: PlanningConfiguration; gazebo_snapshot: GazeboSnapshotConfiguration
    def __post_init__(self) -> None:
        _non_empty(self.schema_version, "schema_version")
        for name, typ in (("mechanical", MechanicalConfiguration), ("uncertainty", UncertaintyConfiguration), ("safety", SafetyConfiguration), ("scan", ScanConfiguration), ("feasibility", FeasibilityConfiguration), ("connector", ConnectorConfiguration), ("global_route_search", GlobalRouteSearchConfiguration), ("shared_pass", SharedPassConfiguration), ("follow_up", FollowUpConfiguration), ("planning", PlanningConfiguration), ("gazebo_snapshot", GazeboSnapshotConfiguration)):
            if not isinstance(getattr(self, name), typ): raise DomainValidationError(f"{name} must be {typ.__name__}")
    def to_dict(self) -> dict[str, Any]: return {"schema_version": self.schema_version, "mechanical": self.mechanical.to_dict(), "uncertainty": self.uncertainty.to_dict(), "safety": self.safety.to_dict(), "scan": self.scan.to_dict(), "feasibility": self.feasibility.to_dict(), "connector": self.connector.to_dict(), "global_route_search": self.global_route_search.to_dict(), "shared_pass": self.shared_pass.to_dict(), "follow_up": self.follow_up.to_dict(), "planning": self.planning.to_dict(), "gazebo_snapshot": self.gazebo_snapshot.to_dict()}
    @classmethod
    def from_dict(cls: type[_T], data: Mapping[str, Any]) -> _T:
        _fields(data, set(cls.__dataclass_fields__), "CollectionRouteConfiguration")
        return cls(data["schema_version"], MechanicalConfiguration.from_dict(data["mechanical"]), UncertaintyConfiguration.from_dict(data["uncertainty"]), SafetyConfiguration.from_dict(data["safety"]), ScanConfiguration.from_dict(data["scan"]), FeasibilityConfiguration.from_dict(data["feasibility"]), ConnectorConfiguration.from_dict(data["connector"]), GlobalRouteSearchConfiguration.from_dict(data["global_route_search"]), SharedPassConfiguration.from_dict(data["shared_pass"]), FollowUpConfiguration.from_dict(data["follow_up"]), PlanningConfiguration.from_dict(data["planning"]), GazeboSnapshotConfiguration.from_dict(data["gazebo_snapshot"]))


@dataclass(frozen=True)
class SnapshotBall:
    ball_id: str; position: Point2D; confidence: float; position_covariance: PositionCovariance2D
    def __post_init__(self) -> None:
        _non_empty(self.ball_id, "ball_id"); _finite(self.confidence, "confidence")
        if not 0 <= self.confidence <= 1: raise DomainValidationError("confidence must be in [0, 1]")
        if not isinstance(self.position, Point2D) or not isinstance(self.position_covariance, PositionCovariance2D): raise DomainValidationError("invalid snapshot ball geometry")
    def to_dict(self) -> dict[str, Any]: return {"ball_id": self.ball_id, "position": self.position.to_dict(), "confidence": self.confidence, "position_covariance": self.position_covariance.to_dict()}
    @classmethod
    def from_dict(cls: type[_T], data: Mapping[str, Any]) -> _T:
        _fields(data, set(cls.__dataclass_fields__), "SnapshotBall"); return cls(data["ball_id"], Point2D.from_dict(data["position"]), data["confidence"], PositionCovariance2D.from_dict(data["position_covariance"]))


@dataclass(frozen=True)
class ScanSnapshot:
    scan_id: str; scan_timestamp: float; map_frame: str; robot_pose_at_scan: Pose2D; balls: tuple[SnapshotBall, ...]; configuration_snapshot: CollectionRouteConfiguration
    def __post_init__(self) -> None:
        _non_empty(self.scan_id, "scan_id"); _finite(self.scan_timestamp, "scan_timestamp"); _non_empty(self.map_frame, "map_frame")
        if not isinstance(self.robot_pose_at_scan, Pose2D) or not isinstance(self.configuration_snapshot, CollectionRouteConfiguration): raise DomainValidationError("invalid snapshot fields")
        if not isinstance(self.balls, tuple) or any(not isinstance(ball, SnapshotBall) for ball in self.balls): raise DomainValidationError("balls must be tuple of SnapshotBall")
        _unique(tuple(ball.ball_id for ball in self.balls), "snapshot ball ids")
    def to_dict(self) -> dict[str, Any]: return {"scan_id": self.scan_id, "scan_timestamp": self.scan_timestamp, "map_frame": self.map_frame, "robot_pose_at_scan": self.robot_pose_at_scan.to_dict(), "balls": [ball.to_dict() for ball in self.balls], "configuration_snapshot": self.configuration_snapshot.to_dict()}
    @classmethod
    def from_dict(cls: type[_T], data: Mapping[str, Any]) -> _T:
        _fields(data, set(cls.__dataclass_fields__), "ScanSnapshot")
        if not isinstance(data["balls"], list): raise DomainValidationError("balls must be a JSON array")
        return cls(data["scan_id"], data["scan_timestamp"], data["map_frame"], Pose2D.from_dict(data["robot_pose_at_scan"]), tuple(SnapshotBall.from_dict(ball) for ball in data["balls"]), CollectionRouteConfiguration.from_dict(data["configuration_snapshot"]))


@dataclass(frozen=True)
class BallResult:
    ball_id: str; status: BallStatus; reason_code: BallReasonCode; pass_id: str | None = None; predicted_lateral_error_m: float | None = None
    def __post_init__(self) -> None:
        _non_empty(self.ball_id, "ball_id")
        if not isinstance(self.status, BallStatus) or not isinstance(self.reason_code, BallReasonCode): raise DomainValidationError("status and reason_code must be documented enums")
        if self.pass_id is not None: _non_empty(self.pass_id, "pass_id")
        if self.predicted_lateral_error_m is not None: _finite(self.predicted_lateral_error_m, "predicted_lateral_error_m")
    def to_dict(self) -> dict[str, Any]: return {"ball_id": self.ball_id, "status": self.status.value, "reason_code": self.reason_code.value, "pass_id": self.pass_id, "predicted_lateral_error_m": self.predicted_lateral_error_m}
    @classmethod
    def from_dict(cls: type[_T], data: Mapping[str, Any]) -> _T:
        _fields(data, set(cls.__dataclass_fields__), "BallResult")
        return cls(data["ball_id"], _enum(data["status"], BallStatus, "status"), _enum(data["reason_code"], BallReasonCode, "reason_code"), data["pass_id"], data["predicted_lateral_error_m"])


@dataclass(frozen=True)
class PlannedCrossing:
    ball_id: str
    position_xy: Point2D
    progress_s: float
    heading_rad: float
    predicted_lateral_error: float

    def __post_init__(self) -> None:
        _non_empty(self.ball_id, "planned crossing ball_id")
        if not isinstance(self.position_xy, Point2D):
            raise DomainValidationError("planned crossing position_xy must be Point2D")
        _finite(self.progress_s, "planned crossing progress_s", minimum=0.0)
        _finite(self.heading_rad, "planned crossing heading_rad")
        _finite(self.predicted_lateral_error, "planned crossing predicted_lateral_error")

    def to_dict(self) -> dict[str, Any]:
        return {
            "ball_id": self.ball_id,
            "position_xy": self.position_xy.to_dict(),
            "progress_s": self.progress_s,
            "heading_rad": self.heading_rad,
            "predicted_lateral_error": self.predicted_lateral_error,
        }

    @classmethod
    def from_dict(cls: type[_T], data: Mapping[str, Any]) -> _T:
        _fields(data, set(cls.__dataclass_fields__), "PlannedCrossing")
        return cls(
            data["ball_id"], Point2D.from_dict(data["position_xy"]),
            data["progress_s"], data["heading_rad"], data["predicted_lateral_error"],
        )


@dataclass(frozen=True)
class RouteSegment:
    id: str; type: RouteSegmentType; path: Path2D; progress_start_m: float; progress_end_m: float; execution_profile: ExecutionProfile; covered_ball_ids: tuple[str, ...]; obstacle_constraint: ObstacleConstraint; planned_crossings: tuple[PlannedCrossing, ...] = ()
    def __post_init__(self) -> None:
        _non_empty(self.id, "segment id")
        if not isinstance(self.type, RouteSegmentType) or not isinstance(self.path, Path2D) or not isinstance(self.execution_profile, ExecutionProfile) or not isinstance(self.obstacle_constraint, ObstacleConstraint): raise DomainValidationError("invalid route segment fields")
        self.path.validate_executable(); _finite(self.progress_start_m, "progress_start_m", minimum=0.0); _finite(self.progress_end_m, "progress_end_m")
        if self.progress_end_m <= self.progress_start_m: raise DomainValidationError("segment progress must increase")
        _tuple_of_strings(self.covered_ball_ids, "covered_ball_ids"); _unique(self.covered_ball_ids, "covered_ball_ids")
        if not isinstance(self.planned_crossings, tuple) or any(not isinstance(item, PlannedCrossing) for item in self.planned_crossings):
            raise DomainValidationError("planned_crossings must be tuple of PlannedCrossing")
        if self.type is RouteSegmentType.FUNNEL_PASS:
            if not self.covered_ball_ids or self.execution_profile.min_speed_mps <= 0: raise DomainValidationError("funnel pass requires covered balls and positive min speed")
            crossing_ids = tuple(item.ball_id for item in self.planned_crossings)
            if crossing_ids != self.covered_ball_ids:
                raise DomainValidationError("planned crossings must exactly match covered ball IDs in order")
            previous_progress = self.progress_start_m
            for crossing in self.planned_crossings:
                if not self.progress_start_m < crossing.progress_s < self.progress_end_m:
                    raise DomainValidationError("planned crossing must be inside funnel-pass progress interval")
                if crossing.progress_s <= previous_progress:
                    raise DomainValidationError("planned crossings must have strictly increasing progress")
                previous_progress = crossing.progress_s
        elif self.covered_ball_ids or self.planned_crossings: raise DomainValidationError("only funnel passes may have covered balls or planned crossings")
        if self.execution_profile.allow_reversing or self.execution_profile.allow_standalone_rotate: raise DomainValidationError("collection segments cannot reverse or rotate standalone")
    def to_dict(self) -> dict[str, Any]: return {"id": self.id, "type": self.type.value, "path": self.path.to_dict(), "progress_start_m": self.progress_start_m, "progress_end_m": self.progress_end_m, "execution_profile": self.execution_profile.to_dict(), "covered_ball_ids": list(self.covered_ball_ids), "obstacle_constraint": self.obstacle_constraint.to_dict(), "planned_crossings": [item.to_dict() for item in self.planned_crossings]}
    @classmethod
    def from_dict(cls: type[_T], data: Mapping[str, Any]) -> _T:
        _fields(data, set(cls.__dataclass_fields__), "RouteSegment")
        if not isinstance(data["covered_ball_ids"], list) or not isinstance(data["planned_crossings"], list): raise DomainValidationError("route segment arrays must be JSON arrays")
        return cls(data["id"], _enum(data["type"], RouteSegmentType, "type"), Path2D.from_dict(data["path"]), data["progress_start_m"], data["progress_end_m"], ExecutionProfile.from_dict(data["execution_profile"]), tuple(data["covered_ball_ids"]), ObstacleConstraint.from_dict(data["obstacle_constraint"]), tuple(PlannedCrossing.from_dict(item) for item in data["planned_crossings"]))


@dataclass(frozen=True)
class CollectionRoutePlan:
    plan_id: str; scan_id: str; map_frame: str; start_pose: Pose2D; terminal_pose: Pose2D; total_length_m: float; expected_duration_s: float; planning_status: PlanningStatus; planning_search_status: PlanningSearchStatus; segments: tuple[RouteSegment, ...]; snapshot_ball_ids: tuple[str, ...]; ball_results: tuple[BallResult, ...]; configuration_snapshot: CollectionRouteConfiguration

    def __post_init__(self) -> None:
        _non_empty(self.plan_id, "plan_id"); _non_empty(self.scan_id, "scan_id"); _non_empty(self.map_frame, "map_frame")
        _finite(self.total_length_m, "total_length_m", minimum=0.0); _finite(self.expected_duration_s, "expected_duration_s", minimum=0.0)
        if not isinstance(self.start_pose, Pose2D) or not isinstance(self.terminal_pose, Pose2D) or not isinstance(self.planning_status, PlanningStatus) or not isinstance(self.planning_search_status, PlanningSearchStatus) or not isinstance(self.configuration_snapshot, CollectionRouteConfiguration): raise DomainValidationError("invalid plan fields")
        if not isinstance(self.segments, tuple) or any(not isinstance(segment, RouteSegment) for segment in self.segments): raise DomainValidationError("segments must be tuple of RouteSegment")
        _tuple_of_strings(self.snapshot_ball_ids, "snapshot_ball_ids"); _unique(self.snapshot_ball_ids, "snapshot_ball_ids")
        if not isinstance(self.ball_results, tuple) or any(not isinstance(result, BallResult) for result in self.ball_results): raise DomainValidationError("ball_results must be tuple of BallResult")
        result_ids = tuple(result.ball_id for result in self.ball_results); _unique(result_ids, "ball result ids")
        if set(result_ids) != set(self.snapshot_ball_ids): raise DomainValidationError("ball results must cover snapshot ball ids exactly")
        self._validate_segments(); self._validate_status()

    def _validate_segments(self) -> None:
        if not self.segments: return
        if self.segments[0].progress_start_m > PROGRESS_TOLERANCE_M: raise DomainValidationError("first segment must start at progress zero")
        ids = tuple(segment.id for segment in self.segments); _unique(ids, "segment ids")
        covered: list[str] = []
        for previous, current in zip(self.segments, self.segments[1:]):
            if abs(current.progress_start_m - previous.progress_end_m) > PROGRESS_TOLERANCE_M: raise DomainValidationError("segments must have contiguous progress")
            if current.progress_end_m <= previous.progress_end_m: raise DomainValidationError("segments must be strictly monotonic")
        for segment in self.segments: covered.extend(segment.covered_ball_ids)
        if len(set(covered)) != len(covered): raise DomainValidationError("a covered ball may appear in one funnel pass only")
        if abs(self.total_length_m - self.segments[-1].progress_end_m) > PROGRESS_TOLERANCE_M: raise DomainValidationError("total_length_m must equal final progress")

    def _validate_status(self) -> None:
        if self.planning_status is PlanningStatus.PLANNING_FAILED and self.planning_search_status is not PlanningSearchStatus.FAILED:
            raise DomainValidationError("planning_failed requires failed search status")
        covered = {result.ball_id for result in self.ball_results if result.status is BallStatus.COVERED}
        pass_coverage = {
            ball_id: segment.id
            for segment in self.segments
            for ball_id in segment.covered_ball_ids
        }
        segment_covered = set(pass_coverage)
        if covered != segment_covered: raise DomainValidationError("covered results must match funnel-pass coverage")
        for result in self.ball_results:
            if result.status is BallStatus.COVERED and (
                result.pass_id is None or pass_coverage.get(result.ball_id) != result.pass_id
            ):
                raise DomainValidationError("covered result must reference its funnel pass")
        executable = self.planning_status in {PlanningStatus.FEASIBLE, PlanningStatus.PARTIAL}
        if executable:
            if not self.segments or self.segments[-1].type is not RouteSegmentType.TERMINAL_CONNECTOR: raise DomainValidationError("executable plan needs terminal connector")
            end = self.segments[-1].path.points[-1].pose
            if math.hypot(end.x_m - self.terminal_pose.x_m, end.y_m - self.terminal_pose.y_m) > POSITION_TOLERANCE_M or _angular_distance(end.yaw_rad, self.terminal_pose.yaw_rad) > ANGLE_TOLERANCE_RAD: raise DomainValidationError("terminal pose must match terminal path point")
            if self.planning_status is PlanningStatus.FEASIBLE and (not self.snapshot_ball_ids or covered != set(self.snapshot_ball_ids)): raise DomainValidationError("feasible plan must cover every snapshot ball")
            if self.planning_status is PlanningStatus.PARTIAL and (not covered or covered == set(self.snapshot_ball_ids)): raise DomainValidationError("partial plan needs covered and non-covered balls")
        elif self.planning_status is PlanningStatus.EMPTY_NO_BALLS:
            if self.snapshot_ball_ids or self.ball_results or self.segments: raise DomainValidationError("empty_no_balls must be empty")
            self._validate_empty_geometry()
            self._validate_empty_terminal()
        elif self.planning_status is PlanningStatus.EMPTY_NO_FEASIBLE_TARGETS:
            if not self.snapshot_ball_ids or covered or self.segments: raise DomainValidationError("empty_no_feasible_targets must have only non-covered results")
            self._validate_empty_geometry()
            self._validate_empty_terminal()
        else:
            if self.segments:
                raise DomainValidationError("non-executable plan artifacts cannot have segments")
            self._validate_empty_geometry()
            self._validate_empty_terminal()

    def _validate_empty_geometry(self) -> None:
        if self.total_length_m != 0 or self.expected_duration_s != 0:
            raise DomainValidationError("non-executable plan artifacts cannot have route geometry")

    def _validate_empty_terminal(self) -> None:
        if math.hypot(self.start_pose.x_m - self.terminal_pose.x_m, self.start_pose.y_m - self.terminal_pose.y_m) > POSITION_TOLERANCE_M or _angular_distance(self.start_pose.yaw_rad, self.terminal_pose.yaw_rad) > ANGLE_TOLERANCE_RAD: raise DomainValidationError("empty plan terminal pose must equal start pose")

    @property
    def is_executable(self) -> bool: return self.planning_status in {PlanningStatus.FEASIBLE, PlanningStatus.PARTIAL}
    def validate_against(self, snapshot: ScanSnapshot) -> None:
        if not isinstance(snapshot, ScanSnapshot): raise DomainValidationError("snapshot must be ScanSnapshot")
        if self.scan_id != snapshot.scan_id or self.map_frame != snapshot.map_frame: raise DomainValidationError("plan and snapshot identity mismatch")
        if set(self.snapshot_ball_ids) != {ball.ball_id for ball in snapshot.balls}: raise DomainValidationError("plan and snapshot ball ids mismatch")
        if self.configuration_snapshot != snapshot.configuration_snapshot: raise DomainValidationError("plan and snapshot configuration mismatch")
    def to_dict(self) -> dict[str, Any]:
        return {"plan_id": self.plan_id, "scan_id": self.scan_id, "map_frame": self.map_frame, "start_pose": self.start_pose.to_dict(), "terminal_pose": self.terminal_pose.to_dict(), "total_length_m": self.total_length_m, "expected_duration_s": self.expected_duration_s, "planning_status": self.planning_status.value, "planning_search_status": self.planning_search_status.value, "segments": [segment.to_dict() for segment in self.segments], "snapshot_ball_ids": list(self.snapshot_ball_ids), "ball_results": [result.to_dict() for result in self.ball_results], "configuration_snapshot": self.configuration_snapshot.to_dict()}
    @classmethod
    def from_dict(cls: type[_T], data: Mapping[str, Any]) -> _T:
        _fields(data, set(cls.__dataclass_fields__), "CollectionRoutePlan")
        for name in ("segments", "snapshot_ball_ids", "ball_results"):
            if not isinstance(data[name], list): raise DomainValidationError(f"{name} must be a JSON array")
        return cls(data["plan_id"], data["scan_id"], data["map_frame"], Pose2D.from_dict(data["start_pose"]), Pose2D.from_dict(data["terminal_pose"]), data["total_length_m"], data["expected_duration_s"], _enum(data["planning_status"], PlanningStatus, "planning_status"), _enum(data["planning_search_status"], PlanningSearchStatus, "planning_search_status"), tuple(RouteSegment.from_dict(segment) for segment in data["segments"]), tuple(data["snapshot_ball_ids"]), tuple(BallResult.from_dict(result) for result in data["ball_results"]), CollectionRouteConfiguration.from_dict(data["configuration_snapshot"]))
