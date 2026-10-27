import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CHECKPOINT = ROOT / "config/standalone_intake_checkpoint.json"


def load_checkpoint():
    return json.loads(CHECKPOINT.read_text(encoding="utf-8"))


def test_checkpoint_records_case_c_stop_consistently():
    data = load_checkpoint()
    flags = data["classifications"]

    assert data["decision"]["case"] == "C_MECHANICAL_DESIGN_INCOMPLETE"
    assert data["decision"]["stop_condition_triggered"] is True
    assert data["decision"]["simulation_campaign_authorized"] is False
    assert flags["INTAKE_CURRENT_ARCHITECTURE_IDENTIFIED"] is True
    assert flags["INTAKE_GUIDE_ARCHITECTURE_DEFINED"] is False
    assert flags["INTAKE_GUIDE_ARCHITECTURE_UNRESOLVED"] is True
    assert flags["INTAKE_GUIDE_ARCHITECTURE_VALIDATED_IN_SIM"] is False
    assert flags["INTAKE_ARCHITECTURE_FROZEN_PROVISIONALLY"] is False
    assert flags["INTAKE_MECHANICAL_DESIGN_INCOMPLETE"] is True
    assert flags["INTAKE_READY_FOR_COMPLETE_ROBOT_INTEGRATION"] is False


def test_mirrored_axis_vectors_match_35_degree_side_convention():
    datums = load_checkpoint()["datums"]
    sine = math.sin(math.radians(35.0))
    cosine = math.cos(math.radians(35.0))

    assert datums["left_axis"] == [0.0, round(sine, 9), round(cosine, 9)]
    assert datums["right_axis"] == [0.0, round(-sine, 9), round(cosine, 9)]
    assert datums["motor_location"] == {"left": "OUTBOARD", "right": "OUTBOARD"}


def test_current_cad_and_xacro_use_yz_plane_mirrored_axes():
    cad = (ROOT / "cad/collector-intake-v1/option-a/option-a.scad").read_text(
        encoding="utf-8"
    )
    xacro = (
        ROOT / "ros2_ws/src/tennis_robot/urdf/components/drivetrain.urdf.xacro"
    ).read_text(encoding="utf-8")

    assert "rotate([-side*oa_wheel_tilt, 0, 0])" in cad
    assert 'rpy="${-tilt_rad if y >= 0 else tilt_rad} 0 0"' in xacro
    assert "rotate([0, oa_wheel_tilt, 0])" not in cad


def test_direct_drive_chain_excludes_obsolete_torque_hub():
    data = load_checkpoint()
    chain = " ".join(data["architecture"]["driveline"])
    flags = data["classifications"]

    assert "14-00012630" in chain
    assert "FIT0186" in chain
    assert "PRO117010" in chain
    assert flags["INTAKE_PRINTED_TORQUE_HUB_REQUIRED"] is False
    assert flags["INTAKE_DIRECT_DRIVE_INTERFACE_DEFINED"] is False


def test_bom_allocates_only_two_of_six_purchased_adapters():
    entries = load_checkpoint()["purchase_bom_audit"]
    adapter = next(item for item in entries if "14-00012630" in item["component"])

    assert adapter["intake_quantity"] == 2
    assert adapter["project_purchased_quantity"] == 6
