import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CHECKPOINT = ROOT / "config/flywheel_launcher_checkpoint.json"
AUTHORITATIVE = ROOT / "docs/mechanism/standalone-flywheel-launcher.md"
BOM = ROOT / "docs/hardware/prototype-purchase-list-el.md"
ARCHIVE = ROOT / "docs/archive/mechanism/flywheel-launcher"


def checkpoint():
    return json.loads(CHECKPOINT.read_text(encoding="utf-8"))


def test_provisional_freeze_has_no_physical_capability_claim():
    data = checkpoint()
    classes = data["classifications"]
    assert classes["FLYWHEEL_ARCHITECTURE_FROZEN_PROVISIONALLY"]
    assert classes["FLYWHEEL_STANDALONE_MODEL_DEFINED"]
    assert classes["FLYWHEEL_POST_NIP_PATH_VALIDATED_IN_SIM"]
    assert classes["FLYWHEEL_KINEMATIC_14MPS_POTENTIAL_DEMONSTRATED"]
    assert [classes["FLYWHEEL_TARGET_EXIT_SPEED_MIN_M_S"],
            classes["FLYWHEEL_TARGET_EXIT_SPEED_MAX_M_S"]] == [12, 14]
    assert not classes["FLYWHEEL_PHYSICAL_EXIT_SPEED_VALIDATED"]
    assert not classes["FLYWHEEL_PHYSICAL_RANGE_VALIDATED"]
    assert not classes["FLYWHEEL_PHYSICAL_SPIN_VALIDATED"]
    assert classes["FLYWHEEL_PHYSICAL_HARDWARE_PENDING"]
    assert not data["decision"]["physical_capability_claimed"]


def test_mechanical_release_holds_are_explicit():
    data = checkpoint()
    wheel = data["candidate_wheel"]
    hub = data["provisional_hub_arbor"]
    assert wheel["nominal_axle_or_bore_m"] == 0.01
    assert "bearing inner race" in wheel["critical_reopen_condition"]
    assert hub["shaft_engagement_m"] == 0.0215
    assert hub["positive_clamping_required"]
    assert hub["removable_axial_retention_required"]
    assert not hub["printed_torque_transmitting_hub_allowed"]
    assert hub["final_dimensions_depend_on_received_wheel_measurement"]
    holds = data["manufacturing_release"]["hold_until_hardware_measurement"]
    assert "final_wheel_hub_or_arbor" in holds
    assert "final_upper_panel_service_cutout" in holds
    assert not data["classifications"]["FLYWHEEL_FINAL_HUB_RELEASED_FOR_MANUFACTURE"]


def test_authoritative_document_and_bom_are_discoverable_and_consistent():
    document = AUTHORITATIVE.read_text(encoding="utf-8")
    bom = BOM.read_text(encoding="utf-8")
    assert "AUTHORITATIVE" in document
    assert "SIMULATION_VALIDATED" in document
    assert "PHYSICAL_VALIDATION_PENDING" in document
    assert "CAD cylinder" in document and "not a barrel" in document
    assert "12–14 m/s" in document
    assert "### ALREADY OWNED" in bom
    assert "### TO BUY" in bom
    assert "### PROVISIONAL / VERIFY BEFORE ORDERING OR MANUFACTURING" in bom
    for text in ("D5065-270KV", "12N14P", "21.5 mm", "Urethane",
                 "nitrile/NBR", "butyl rubber", "bearing inner race"):
        assert text in bom


def test_superseded_reports_are_archived_and_traceable():
    archived = {
        "flywheel-launcher-exploration-el.md",
        "flywheel-launcher-physics-bench-stop-report.md",
        "flywheel-launcher-capability-bench-mechanical-stop-report.md",
    }
    assert archived.issubset({path.name for path in ARCHIVE.iterdir()})
    manifest = (ARCHIVE / "README.md").read_text(encoding="utf-8")
    assert "non-authoritative" in manifest
    assert "SUPERSEDED_MOTOR_MOUNT_CONCEPT" in manifest
    assert "FAILED_PRELIMINARY_CAPABILITY_MODEL" in manifest
    assert "PRE_EXIT_CORRIDOR_CORRECTION" in manifest
    for name in archived:
        assert not (ROOT / "docs/mechanism" / name).exists()
        assert name in manifest


def test_archive_and_active_tree_classifications_are_closed():
    classes = checkpoint()["classifications"]
    assert classes["FLYWHEEL_ACTIVE_DESIGN_UNAMBIGUOUS"]
    assert classes["FLYWHEEL_SUPERSEDED_ATTEMPTS_ARCHIVED"]
    assert classes["FLYWHEEL_HISTORICAL_TRACEABILITY_PRESERVED"]
    assert not classes["FLYWHEEL_STALE_ACTIVE_REFERENCES"]
