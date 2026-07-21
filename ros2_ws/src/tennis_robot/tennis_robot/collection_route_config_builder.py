"""Strict runtime builder for the immutable collection-route configuration.

The caller owns configuration-source and path resolution.  This module reads
no ROS parameters or environment variables and supplies no fallback values.
"""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
from typing import Any

from tennis_robot.collection_route_types import (
    CollectionRouteConfiguration,
    ConnectorConfiguration,
    DomainValidationError,
    FeasibilityConfiguration,
    FollowUpConfiguration,
    GazeboSnapshotConfiguration,
    GlobalRouteSearchConfiguration,
    MechanicalConfiguration,
    PlanningConfiguration,
    SafetyConfiguration,
    ScanConfiguration,
    SharedPassConfiguration,
)
from tennis_robot.perception_covariance_calibration import (
    CalibrationError,
    PerceptionSpatialValidationConfig,
    load_artifact,
)


class CollectionRouteConfigurationBuildError(ValueError):
    """An explicit source or calibration artifact cannot produce the contract."""


_GROUP_TYPES = {
    "mechanical": MechanicalConfiguration,
    "safety": SafetyConfiguration,
    "scan": ScanConfiguration,
    "feasibility": FeasibilityConfiguration,
    "connector": ConnectorConfiguration,
    "global_route_search": GlobalRouteSearchConfiguration,
    "shared_pass": SharedPassConfiguration,
    "follow_up": FollowUpConfiguration,
    "planning": PlanningConfiguration,
    "gazebo_snapshot": GazeboSnapshotConfiguration,
}

_SOURCE_FIELDS = {"schema_version", "perception_spatial_validation", *_GROUP_TYPES}

# Nested shapes that cannot be inferred from the top-level group dataclass.
_NESTED_FIELDS = {
    "planning.default_execution_profile": {
        "nominal_speed_mps",
        "min_speed_mps",
        "max_speed_mps",
        "nominal_speed_warning_tolerance_mps",
        "max_acceleration_mps2",
        "max_deceleration_mps2",
        "required_entry_m",
        "required_run_in_m",
        "required_run_out_m",
        "max_curvature_per_m",
        "max_lateral_error_m",
        "max_heading_error_rad",
        "allow_reversing",
        "allow_standalone_rotate",
    },
    "gazebo_snapshot.localization_xy_covariance": {"covariance"},
    "gazebo_snapshot.localization_xy_covariance.covariance": {"xx", "xy", "yy"},
    "gazebo_snapshot.association": {
        "association_mahalanobis_gate_chi2",
        "min_confirmations",
        "min_distinct_scan_steps",
    },
}


def build_collection_route_configuration(
    source: Mapping[str, Any],
    *,
    calibration_artifact_path: str | Path,
) -> CollectionRouteConfiguration:
    """Build the frozen configuration from one complete, explicit mapping.

    ``source`` uses the same shape as ``CollectionRouteConfiguration.to_dict``
    except that ``calibration_artifact`` is deliberately absent: its sole
    source is ``calibration_artifact_path``.
    """
    root = _require_mapping(source, "source")
    _require_exact_fields(root, _SOURCE_FIELDS, "source")

    groups: dict[str, Any] = {}
    for name, group_type in _GROUP_TYPES.items():
        data = _require_mapping(root[name], name)
        _require_exact_fields(data, set(group_type.__dataclass_fields__), name)
        _validate_nested_shape(name, data)
        try:
            groups[name] = group_type.from_dict(dict(data))
        except (DomainValidationError, TypeError, ValueError) as exc:
            raise CollectionRouteConfigurationBuildError(
                f"invalid group {name!r}: {exc}"
            ) from exc

    validation_data = _require_mapping(
        root["perception_spatial_validation"], "perception_spatial_validation"
    )
    _require_exact_fields(
        validation_data,
        set(PerceptionSpatialValidationConfig.__dataclass_fields__),
        "perception_spatial_validation",
    )
    try:
        spatial_validation = PerceptionSpatialValidationConfig.from_dict(
            dict(validation_data)
        )
    except (CalibrationError, TypeError, ValueError) as exc:
        raise CollectionRouteConfigurationBuildError(
            f"invalid group 'perception_spatial_validation': {exc}"
        ) from exc

    try:
        calibration_artifact = load_artifact(calibration_artifact_path)
    except (CalibrationError, OSError, TypeError, KeyError, AttributeError) as exc:
        raise CollectionRouteConfigurationBuildError(
            f"invalid group 'calibration_artifact' from "
            f"{str(calibration_artifact_path)!r}: {exc}"
        ) from exc

    try:
        return CollectionRouteConfiguration(
            schema_version=root["schema_version"],
            perception_spatial_validation=spatial_validation,
            calibration_artifact=calibration_artifact,
            **groups,
        )
    except (DomainValidationError, TypeError, ValueError) as exc:
        raise CollectionRouteConfigurationBuildError(
            f"invalid field 'schema_version' or assembled configuration: {exc}"
        ) from exc


def _validate_nested_shape(group: str, data: Mapping[str, Any]) -> None:
    for path, expected in _NESTED_FIELDS.items():
        parts = path.split(".")
        if parts[0] != group:
            continue
        value: Any = data
        traversed = [group]
        for part in parts[1:]:
            value = _require_mapping(value, ".".join(traversed))
            if part not in value:
                raise CollectionRouteConfigurationBuildError(
                    f"missing field {'.'.join((*traversed, part))!r}"
                )
            value = value[part]
            traversed.append(part)
        nested = _require_mapping(value, path)
        _require_exact_fields(nested, expected, path)


def _require_mapping(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise CollectionRouteConfigurationBuildError(
            f"field {path!r} must be a mapping"
        )
    return value


def _require_exact_fields(
    data: Mapping[str, Any], expected: set[str], path: str
) -> None:
    actual = set(data)
    missing = sorted(expected - actual, key=repr)
    if missing:
        raise CollectionRouteConfigurationBuildError(
            f"missing field {path + '.' + missing[0]!r}"
        )
    extra = sorted(actual - expected, key=repr)
    if extra:
        raise CollectionRouteConfigurationBuildError(
            f"unexpected field {path + '.' + extra[0]!r}"
        )
