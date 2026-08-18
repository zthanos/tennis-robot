"""Explicit migration of archived collection-route configuration payloads.

``CollectionRouteConfiguration.from_dict`` is deliberately strict: every field
must be present, because a planning configuration that silently defaults is a
configuration nobody can reason about afterwards.  That strictness is right for
the runtime and wrong for the archive -- a recorded snapshot or planner audit is
frozen at the schema of the day it was written, and the moment a group is added
the whole artifact becomes unreadable (debug log #57, finding 10).

This module is the one place allowed to fill in what a newer schema expects.  It
is offline tooling, never imported by the planner or any node: replay scripts and
tests call it explicitly, and it reports exactly which paths it filled so the
substitution is visible rather than assumed.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

from tennis_robot.collection_route_types import (
    CONFIGURATION_SCHEMA_VERSION,
    DomainValidationError,
)


_LEGACY_SCHEMA_VERSION = "collection-route/v1"

# Values added to the configuration after v1, with the version that introduced
# them.  Each is the value that shipped with that change, so a migrated artifact
# behaves the way the artifact's own era behaved wherever that is knowable, and
# the current default where the field had no v1 equivalent at all.
_V1_ADDITIONS: tuple[tuple[tuple[str, ...], Any], ...] = (
    # Boundary-contact recovery (#46): half the funnel-mouth width, so a ball
    # against the net or fence is reached on the outer cheek.
    (("feasibility", "boundary_recovery_contact_offset_m"), 0.205),
    # Gentler connector alternatives (#52).  A single 1.0 multiplier reproduces
    # the pre-#52 graph exactly, which is what a v1 artifact was planned with.
    (("connector", "sweep_radius_multipliers"), [1.0]),
    (("connector", "capture_minimum_turn_radius_m"), 2.5),
    # Lazy successor expansion (v2).  A v1 artifact was planned with the
    # all-at-once behaviour, which a batch of this size reproduces.
    (("global_route_search", "successor_batch_size"), 1000000),
    (("global_route_search", "successor_batch_policy"), "fixed"),
    # Cluster heuristics (v2).  maximum_macro_chains 0 disables macros, which is
    # the only setting that reproduces v1 routing behaviour.
    (
        ("cluster_heuristics",),
        {
            "cluster_threshold_m": 2.5,
            "maximum_clusters": 6,
            "maximum_macro_passes": 4,
            "maximum_macro_chains": 0,
        },
    ),
)


@dataclass(frozen=True)
class ConfigurationMigrationResult:
    """A migrated payload plus the dotted paths that were filled in."""

    data: dict[str, Any]
    filled: tuple[str, ...]


def migrate_configuration_dict(data: Mapping[str, Any]) -> ConfigurationMigrationResult:
    """Bring an archived configuration payload up to the current schema.

    Only absent paths are filled; anything the artifact states is preserved.
    An unknown schema version is an error rather than a best effort.
    """
    if not isinstance(data, Mapping):
        raise DomainValidationError("configuration payload must be a mapping")
    version = data.get("schema_version")
    if version == CONFIGURATION_SCHEMA_VERSION:
        return ConfigurationMigrationResult(dict(data), ())
    if version != _LEGACY_SCHEMA_VERSION:
        raise DomainValidationError(
            f"cannot migrate unknown schema version {version!r}; "
            f"known versions are {_LEGACY_SCHEMA_VERSION!r} and "
            f"{CONFIGURATION_SCHEMA_VERSION!r}"
        )

    migrated = _deep_copy(data)
    filled: list[str] = []
    for path, value in _V1_ADDITIONS:
        if _insert_missing(migrated, path, value):
            filled.append(".".join(path))
    migrated["schema_version"] = CONFIGURATION_SCHEMA_VERSION
    return ConfigurationMigrationResult(migrated, tuple(filled))


def migrate_snapshot_dict(data: Mapping[str, Any]) -> ConfigurationMigrationResult:
    """Same, for a recorded ``ScanSnapshot`` payload."""
    if not isinstance(data, Mapping) or "configuration_snapshot" not in data:
        raise DomainValidationError("snapshot payload must carry configuration_snapshot")
    result = migrate_configuration_dict(data["configuration_snapshot"])
    migrated = _deep_copy(data)
    migrated["configuration_snapshot"] = result.data
    return ConfigurationMigrationResult(migrated, result.filled)


def _insert_missing(target: dict[str, Any], path: tuple[str, ...], value: Any) -> bool:
    cursor: Any = target
    for key in path[:-1]:
        if not isinstance(cursor, dict) or key not in cursor:
            raise DomainValidationError(f"archived payload has no group {key!r} to extend")
        cursor = cursor[key]
    leaf = path[-1]
    if not isinstance(cursor, dict):
        raise DomainValidationError(f"archived payload group {path[0]!r} is not a mapping")
    if leaf in cursor:
        return False
    cursor[leaf] = _deep_copy(value) if isinstance(value, (dict, list)) else value
    return True


def _deep_copy(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {key: _deep_copy(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_deep_copy(item) for item in value]
    return value
