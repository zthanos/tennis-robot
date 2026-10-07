import json
import math
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
CHECKPOINT = ROOT / "config" / "standalone_intake_fixed_motor_checkpoint.json"
OLD_GUIDE = (
    ROOT
    / "docs"
    / "archive"
    / "mechanism"
    / "intake"
    / "config"
    / "standalone_intake_guide_definition.json"
)
CONTRACT = ROOT / "config" / "compact_mechanical_contract.json"
CAD = (
    ROOT
    / "cad"
    / "standalone-intake-fixed-motor"
    / "standalone-intake-fixed-motor-compliant-tyre.scad"
)
ARCHIVE_LEDGER = ROOT / "docs" / "archive" / "mechanism" / "intake" / "README.md"
PURCHASE_LIST = ROOT / "docs" / "hardware" / "prototype-purchase-list-el.md"
OPTION_A_README = ROOT / "cad" / "collector-intake-v1" / "option-a" / "README.md"
OPTION_A_CAD = ROOT / "cad" / "collector-intake-v1" / "option-a" / "option-a.scad"


def _load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def test_fixed_motor_architecture_and_stop_gate():
    result = _load(CHECKPOINT)
    c = result["classifications"]

    assert c["INTAKE_PHYSICAL_ARCHITECTURE"] == "FIXED_MOTOR_COMPLIANT_TYRE"
    assert c["INTAKE_MOTORS_FIXED_TO_BRIDGE"] is True
    assert c["INTAKE_TRANSLATING_MOTOR_PODS_REQUIRED"] is False
    assert c["INTAKE_LINEAR_GUIDES_REQUIRED"] is False
    assert c["INTAKE_RETURN_SPRINGS_REQUIRED"] is False
    assert c["INTAKE_POD_HARD_STOPS_REQUIRED"] is False
    assert c["INTAKE_TYRE_COMPLIANCE_REQUIRED"] is True
    assert c["INTAKE_TYRE_COMPLIANCE_MODEL_DEFINED"] is False
    assert c["INTAKE_STATIC_FIXED_GEOMETRY_PASSAGE_VALIDATED"] is False
    assert c["INTAKE_DYNAMIC_CAPTURE_VALIDATED_IN_SIM"] is False
    assert result["decision"]["dynamic_capture_authorized"] is False
    assert c["INTAKE_ACTIVE_DESIGN_UNAMBIGUOUS"] is True
    assert c["INTAKE_35_DEGREE_LONGITUDINAL_AXES_VALIDATED"] is True
    assert c["INTAKE_FRONT_VIEW_AXES_PARALLEL"] is True
    assert c["INTAKE_WHEEL_ADAPTER_MOTOR_COAXIAL"] is True
    assert c["INTAKE_ARCHITECTURE_FROZEN_PROVISIONALLY"] is False
    assert c["INTAKE_PHYSICAL_VALIDATION_PENDING"] is True


def test_fixed_axes_are_parallel_longitudinal_and_stack_is_coaxial():
    result = _load(CHECKPOINT)
    fixed = result["fixed_geometry"]
    left = fixed["left_axis"]
    right = fixed["right_axis"]

    assert math.sqrt(sum(value * value for value in left)) == pytest.approx(1.0)
    assert right == pytest.approx(left)
    assert left == pytest.approx(
        [math.sin(math.radians(35)), 0.0, math.cos(math.radians(35))]
    )
    wheels = fixed["option_a_local_mm"]["wheel_centres"]
    motors = fixed["option_a_local_mm"]["motor_centres"]
    for side in ("left", "right"):
        assert motors[side][1] == pytest.approx(wheels[side][1])
        assert motors[side][0] > wheels[side][0]
        assert motors[side][2] > wheels[side][2]
    assert fixed["motor_and_wheel_translation_allowed"] is False


def test_bridge_interface_is_derived_from_the_retained_axis():
    mount = _load(CHECKPOINT)["fixed_bridge_mount"]

    assert mount["axis_y_at_bridge_mm"] == {"left": 90.0, "right": -90.0}
    assert mount["bridge_z_mm"] == [190.0, 208.0]
    assert mount["axis_x_at_bridge_under_mm"] == pytest.approx(554.024904585)
    assert mount["axis_x_at_bridge_top_mm"] == pytest.approx(566.628640273)
    assert mount["invalid_previous_vertical_opening_removed"] == [22.0, 22.0]
    assert mount["invented_bridge_fastener_pattern_removed"] is True
    assert mount["printed_mounts_quantity"] == 2
    assert mount["printed_mount_grip_diameter_mm"] == 30.0
    assert mount["printed_mount_axial_depth_mm"] == 10.0
    assert mount["printed_mount_axial_position"] == (
        "TOPMOST_10_MM_OF_MOTOR_BODY_WITH_UPPER_EDGE_FLUSH_TO_MOTOR_TOP_FACE"
    )
    assert mount["bridge_motor_opening_required"] is False
    assert mount["motor_to_bridge_vertical_clearance_mm"] == pytest.approx(
        6.135315764
    )


def test_compliance_partitions_close_and_follow_calibrated_ball_law():
    result = _load(CHECKPOINT)
    compliance = result["compliance_partition"]
    stiffness = compliance["ball_loading_stiffness_n_m_pow"]

    for row in compliance["partitions"]:
        ball_mm = row["ball_deformation_mm"]
        tyre_mm = row["tyre_deformation_each_mm"]
        force_n = stiffness * (ball_mm / 1000.0) ** 1.5
        assert ball_mm + 2.0 * tyre_mm == pytest.approx(10.0, abs=2e-6)
        assert row["ball_force_n"] == pytest.approx(force_n, abs=0.01)

    assert compliance["unique_equilibrium_solved"] is False


def test_previous_guide_is_explicitly_superseded():
    old = _load(OLD_GUIDE)
    assert old["artifact_status"] == "SUPERSEDED_WRONG_COMPLIANCE_ARCHITECTURE"
    assert old["decision"]["guide_selected"] is False
    assert old["classifications"]["INTAKE_PREVIOUS_GUIDE_STUDY_SUPERSEDED"] is True


def test_active_contract_distinguishes_physical_architecture_from_surrogate():
    contract = _load(CONTRACT)
    orientation = contract["intake_direct_drive_orientation"]
    component = contract["components"]["intake_direct_drive_pods"]

    assert orientation["physical_architecture"] == "FIXED_MOTOR_COMPLIANT_TYRE"
    assert orientation["motors_fixed_to_bridge"] is True
    assert orientation["translating_motor_pods_required"] is False
    assert "SIMULATION_ONLY_TYRE_COMPLIANCE_SURROGATE" in orientation["simulation_surrogate_notice"]
    assert component["physical_role"] == "FIXED_DIRECT_DRIVE_STACKS_AT_REST"


def test_new_analysis_cad_has_no_moving_guide_implementation():
    source = CAD.read_text(encoding="utf-8")
    assert "module fixed_stack" in source
    assert "module moving_guide" not in source
    assert "module spring_symbol" not in source
    assert "carriage_joint" not in source
    assert "rotate([0, tilt_deg, 0])" in source
    assert "rotate([-side*tilt_deg" not in source
    assert "bridge_opening_placeholder" not in source
    assert "module printed_motor_mount" in source
    assert "mount_motor_d = 30" in source
    assert "mount_depth = 10" in source
    assert "bridge_under_z = 190" in source

    option_a = OPTION_A_CAD.read_text(encoding="utf-8")
    assert "rotate([0, oa_wheel_tilt, 0])" in option_a
    assert "rotate([-side*oa_wheel_tilt" not in option_a
    assert "cube([22, 22" not in option_a


def test_archive_and_active_bom_are_unambiguous():
    ledger = ARCHIVE_LEDGER.read_text(encoding="utf-8")
    purchase = PURCHASE_LIST.read_text(encoding="utf-8")
    option_a = OPTION_A_README.read_text(encoding="utf-8")

    assert "CURRENT_AUTHORITATIVE" in ledger
    assert "HISTORICAL_SUPERSEDED" in ledger
    assert "fixed FIT0186 motor mounts" in purchase
    assert "14-00012630" in purchase
    assert "fixed bearing-supported intake pods" not in purchase
    assert "hex_hub.stl" not in option_a
    assert "bearing_cartridge.stl" not in option_a


def test_simulation_carries_the_frozen_fixed_motor_architecture():
    """LEGACY_CARRIAGE_REMOVED — the surrogate is gone, not merely disabled.

    The simulation was corrected toward the frozen hardware, so the files that
    used to be labelled NOT_PHYSICAL_INTAKE_ARCHITECTURE must no longer contain
    the translating carriage at all: no prismatic joint, no travel parameter,
    no SDF spring patch, no carriage state interface.
    """

    paths = [
        ROOT / "ros2_ws/src/tennis_robot/urdf/tennis_robot.urdf.xacro",
        ROOT / "ros2_ws/src/tennis_robot/urdf/components/drivetrain.urdf.xacro",
        ROOT / "ros2_ws/src/tennis_robot/urdf/components/ros2_control.urdf.xacro",
        ROOT / "scripts/generate_robot_urdf.py",
        ROOT / "scripts/sim_debug/analyze_intake_release_criteria.py",
        ROOT / "ros2_ws/src/tennis_robot/tennis_robot/sim_physics_probe.py",
    ]
    for path in paths:
        source = path.read_text(encoding="utf-8")
        assert "intake_wheel_left_carriage_joint" not in source, path
        assert "intake_carriage_travel" not in source, path
        assert "INTAKE_WHEEL_SPRING_K" not in source, path
        assert "INTAKE_EXPOSE_CARRIAGE_STATE" not in source, path

    drivetrain = (
        ROOT / "ros2_ws/src/tennis_robot/urdf/components/drivetrain.urdf.xacro"
    ).read_text(encoding="utf-8")
    assert "FIXED-MOTOR, COMPLIANT-TYRE INTAKE" in drivetrain
    assert "intake_wheel_${side}_mount_joint" in drivetrain
    assert 'type="prismatic"' not in drivetrain
    # D3: no unphysical friction may be hardcoded on any intake surface.
    assert "<mu1>2.5</mu1>" not in drivetrain
