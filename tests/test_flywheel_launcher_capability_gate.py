import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from scripts.evaluate_flywheel_launcher_capability_gate import RESULTS, evaluate


def test_provisional_gate_supersedes_historical_physical_stop():
    result = evaluate()
    assert result == {
        "ball_reference_unchanged": True,
        "historical_stop_snapshot_preserved": True,
        "provisional_gate_supersedes_historical_stop": True,
        "motor_bodies_present_in_standalone_geometry": True,
        "provisional_hubs_present_in_standalone_geometry": True,
        "axial_retention_present_in_standalone_geometry": True,
        "d5065_effort_limit_enforced": True,
        "capability_phase_authorized": True,
        "physical_validation_remains_false": True,
    }


def test_historical_capability_claims_remain_false_until_trials_run():
    data = json.loads(Path(RESULTS).read_text(encoding="utf-8"))
    classifications = data["classifications"]
    required = {
        "STANDALONE_FLYWHEEL_LAUNCHER_MECHANICALLY_COMPLETE",
        "D5065_MOTOR_MODEL_EVIDENCE_SUPPORTED",
        "D5065_POWER_LIMITS_ENFORCED",
        "CALIBRATED_BALL_MODEL_UNCHANGED",
        "LAUNCHER_TYRE_FRICTION_CALIBRATED",
        "LAUNCHER_TYRE_NORMAL_COMPLIANCE_VALIDATED",
        "NORMAL_CONTACT_LAUNCH_VALIDATED",
        "TANGENTIAL_CONTACT_LAUNCH_VALIDATED",
        "LAUNCH_EXIT_VELOCITY_VALIDATED",
        "LAUNCH_EXIT_ANGLE_VALIDATED",
        "LAUNCH_SPIN_VALIDATED",
        "RPM_DROOP_AND_RECOVERY_VALIDATED",
        "LAUNCH_ENERGY_ACCOUNTING_VALIDATED",
        "LAUNCHER_CONTACT_TIMESTEP_CONVERGED",
        "LAUNCHER_12_M_S_CAPABILITY",
        "LAUNCHER_14_M_S_CAPABILITY",
        "LAUNCHER_16_M_S_CAPABILITY",
        "LAUNCHER_18_M_S_CAPABILITY",
        "COURT_TRAJECTORY_MODEL_VALIDATED",
        "OPPOSITE_BASELINE_REACH_CAPABILITY",
        "LEFT_DEEP_CORNER_REACH_CAPABILITY",
        "RIGHT_DEEP_CORNER_REACH_CAPABILITY",
        "LAUNCHER_CAPABILITY_MAP_GENERATED",
    }
    assert required == set(classifications)
    assert classifications["CALIBRATED_BALL_MODEL_UNCHANGED"] is True
    assert all(
        value is False
        for name, value in classifications.items()
        if name != "CALIBRATED_BALL_MODEL_UNCHANGED"
    )
