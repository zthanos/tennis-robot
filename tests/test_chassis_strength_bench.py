import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
BENCH = ROOT / "engineering" / "chassis-strength-bench"


def _module():
    spec = importlib.util.spec_from_file_location("chassis_strength", BENCH / "analyze.py")
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_all_three_material_profiles_are_screened():
    module = _module()
    config = json.loads((BENCH / "config" / "baseline.json").read_text())
    materials = json.loads((BENCH / "config" / "materials.json").read_text())
    report = module.analyze(config, materials)
    assert set(materials["materials"]) == {"PLA", "PETG", "PETG-CF"}
    for scenario in report["scenarios"].values():
        assert set(scenario["materials"]) == {"PLA", "PETG", "PETG-CF"}


def test_increased_mass_does_not_improve_minimum_safety_factor():
    module = _module()
    config = json.loads((BENCH / "config" / "baseline.json").read_text())
    materials = json.loads((BENCH / "config" / "materials.json").read_text())
    report = module.analyze(config, materials)
    light = report["scenarios"]["light_test_fixture"]["materials"]
    heavy = report["scenarios"]["conservative_turn"]["materials"]
    for name in materials["materials"]:
        assert heavy[name]["minimum_safety_factor"] <= light[name]["minimum_safety_factor"]


def test_report_remains_blocked_until_dynamic_and_coupon_inputs_exist():
    module = _module()
    config = json.loads((BENCH / "config" / "baseline.json").read_text())
    materials = json.loads((BENCH / "config" / "materials.json").read_text())
    report = module.analyze(config, materials)
    assert report["benchmark_status"] == "BENCHMARK_INCOMPLETE"
    assert report["decision"] == "REDESIGN_BEFORE_PRINT"
    assert report["missing_measurements"]


def test_gazebo_peaks_can_only_raise_conservative_design_loads():
    module = _module()
    config = json.loads((BENCH / "config" / "baseline.json").read_text())
    materials = json.loads((BENCH / "config" / "materials.json").read_text())
    plain = module.analyze(config, materials)
    summary = {
        "global": {
            "maximum_force_norm_n": 100.0,
            "maximum_torque_norm_nm": 9.0,
            "maximum_abs_force_z_n": 50.0,
        }
    }
    measured = module.analyze(config, materials, summary)
    plain_loads = plain["scenarios"]["conservative_turn"]["loads"]
    measured_loads = measured["scenarios"]["conservative_turn"]["loads"]
    assert measured_loads["design_turn_force_per_wheel_n"] >= plain_loads["design_turn_force_per_wheel_n"]
    assert measured_loads["design_motor_reaction_torque_nm"] >= plain_loads["design_motor_reaction_torque_nm"]


def test_passing_local_fea_releases_only_a_coupon():
    module = _module()
    config = json.loads((BENCH / "config" / "baseline.json").read_text())
    materials = json.loads((BENCH / "config" / "materials.json").read_text())
    fea = {
        "materials": {
            name: {"screen_safety_factor": 3.0} for name in materials["materials"]
        },
        "limitations": ["layer peel unresolved"],
    }
    report = module.analyze(config, materials, fea_summary=fea)
    assert report["decision"] == "PRINT_LOCAL_COUPON_NOT_FULL_CHASSIS"
    assert report["benchmark_status"] == "BASELINE_COMPLETE_AWAITING_PHYSICAL_COUPON"


def test_payload_sweep_increases_deflection_and_reaches_motor_limit():
    module = _module()
    config = json.loads((BENCH / "config" / "baseline.json").read_text())
    materials = json.loads((BENCH / "config" / "materials.json").read_text())
    report = module.analyze(config, materials)
    points = report["payload_sweep"]["points"]
    assert points[0]["effective_total_mass_kg"] == 10.0
    assert points[-1]["effective_total_mass_kg"] == 20.0
    for name in materials["materials"]:
        assert (
            points[-1]["materials"][name]["rail_deflection_mm"]
            > points[0]["materials"][name]["rail_deflection_mm"]
        )
    assert points[0]["turn_force_limit"] == "tyre_friction"
    assert points[-1]["turn_force_limit"] == "motor_torque"


def test_corner_distance_increases_root_moment_and_reduces_safety_factor():
    module = _module()
    config = json.loads((BENCH / "config" / "baseline.json").read_text())
    materials = json.loads((BENCH / "config" / "materials.json").read_text())
    report = module.analyze(config, materials)
    points = report["corner_proximity_sweep"]["points"]
    assert points[0]["motor_center_to_corner_mm"] == 50.0
    assert points[-1]["motor_center_to_corner_mm"] == 200.0
    assert points[-1]["vertical_root_moment_nm"] > points[0]["vertical_root_moment_nm"]
    for name in materials["materials"]:
        assert (
            points[-1]["materials"][name]["screen_safety_factor"]
            < points[0]["materials"][name]["screen_safety_factor"]
        )
