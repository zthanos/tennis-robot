"""Archived collection-route payloads must stay readable across schema bumps."""

import json
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"))

from collection_route_fixtures import default_configuration  # noqa: E402
from tennis_robot.collection_route_schema_migration import (  # noqa: E402
    migrate_configuration_dict,
    migrate_snapshot_dict,
)
from tennis_robot.collection_route_types import (  # noqa: E402
    CONFIGURATION_SCHEMA_VERSION,
    CollectionRouteConfiguration,
    DomainValidationError,
    ScanSnapshot,
)

_REPOSITORY_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
_ARCHIVED_AUDIT = os.path.join(
    _REPOSITORY_ROOT, "runtime", "route_audit", "clean_current_20260728_1315",
    "collection-scan-39264000000.json",
)


def legacy_configuration_dict():
    """The current configuration reduced to what schema v1 actually carried."""
    data = default_configuration().to_dict()
    data["schema_version"] = "collection-route/v1"
    data.pop("cluster_heuristics")
    data["feasibility"].pop("boundary_recovery_contact_offset_m")
    data["connector"].pop("sweep_radius_multipliers")
    data["connector"].pop("capture_minimum_turn_radius_m")
    return data


def test_v1_configuration_migrates_and_loads():
    result = migrate_configuration_dict(legacy_configuration_dict())
    assert result.data["schema_version"] == CONFIGURATION_SCHEMA_VERSION
    assert set(result.filled) == {
        "cluster_heuristics",
        "feasibility.boundary_recovery_contact_offset_m",
        "connector.sweep_radius_multipliers",
        "connector.capture_minimum_turn_radius_m",
    }
    configuration = CollectionRouteConfiguration.from_dict(result.data)
    # v1 knew only the tight turning radius, and macros did not exist: the
    # migrated artifact must reproduce that era rather than today's defaults.
    assert configuration.connector.sweep_radius_multipliers == (1.0,)
    assert configuration.cluster_heuristics.maximum_macro_chains == 0


def test_migration_never_overwrites_what_the_artifact_states():
    data = legacy_configuration_dict()
    data["connector"]["capture_minimum_turn_radius_m"] = 4.0
    result = migrate_configuration_dict(data)
    assert result.data["connector"]["capture_minimum_turn_radius_m"] == 4.0
    assert "connector.capture_minimum_turn_radius_m" not in result.filled


def test_current_payload_is_returned_untouched():
    data = default_configuration().to_dict()
    result = migrate_configuration_dict(data)
    assert result.filled == ()
    assert result.data == data


def test_unknown_schema_version_is_an_error_not_a_best_effort():
    data = legacy_configuration_dict()
    data["schema_version"] = "collection-route/v99"
    with pytest.raises(DomainValidationError):
        migrate_configuration_dict(data)


def test_runtime_loader_stays_strict():
    # The migration is offline tooling; the domain loader must keep refusing an
    # incomplete payload rather than defaulting silently.
    with pytest.raises(DomainValidationError):
        CollectionRouteConfiguration.from_dict(legacy_configuration_dict())


@pytest.mark.skipif(not os.path.exists(_ARCHIVED_AUDIT), reason="archived audit artifact absent")
def test_recorded_planner_audit_artifact_still_deserializes():
    with open(_ARCHIVED_AUDIT, encoding="utf-8") as handle:
        artifact = json.load(handle)
    result = migrate_snapshot_dict(artifact["snapshot"])
    snapshot = ScanSnapshot.from_dict(result.data)
    assert len(snapshot.balls) == 10
    assert snapshot.configuration_snapshot.schema_version == CONFIGURATION_SCHEMA_VERSION
