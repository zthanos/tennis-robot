#!/usr/bin/env python3
"""Conservative first-pass screen for the modular test chassis.

This is intentionally transparent beam / bearing mechanics, not a replacement
for a meshed FEA model. It identifies which load path deserves the first CAD
change and records every provisional input in a machine-readable report.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parent
G = 9.80665


def _load(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _mm(value: float) -> float:
    return value / 1000.0


def _section(geometry: dict[str, float]) -> dict[str, float]:
    b = _mm(geometry["rail_outer_width_mm"])
    h = _mm(geometry["rail_outer_height_mm"])
    t = _mm(geometry["rail_wall_mm"])
    web_t = _mm(geometry.get("internal_web_mm", 0.0))
    bi = b - 2.0 * t
    hi = h - 2.0 * t
    if min(bi, hi, t) <= 0:
        raise ValueError("Invalid hollow rail dimensions")
    shell_area = b * h - bi * hi
    shell_vertical_i = (b * h**3 - bi * hi**3) / 12.0
    # The printable W has three identical diagonal strips in cross-section.
    # Each spans one third of the inner width and the complete inner height.
    # Their centroids lie on the section neutral axis. Overlap at the shell
    # and the two W vertices is ignored, so this is still a screening model.
    web_segment_length = math.hypot(bi / 3.0, hi)
    web_area = 3.0 * web_t * web_segment_length
    web_vertical_i = 3.0 * web_t * web_segment_length * hi**2 / 12.0
    return {
        "area_m2": shell_area + web_area,
        "shell_area_m2": shell_area,
        "internal_web_area_m2": web_area,
        "vertical_bending_i_m4": shell_vertical_i + web_vertical_i,
        "shell_vertical_bending_i_m4": shell_vertical_i,
        "internal_web_vertical_bending_i_m4": web_vertical_i,
        "lateral_bending_i_m4": (h * b**3 - hi * bi**3) / 12.0,
        "outer_vertical_c_m": h / 2.0,
        "outer_lateral_c_m": b / 2.0,
        "median_enclosed_area_m2": (b - t) * (h - t),
        "wall_m": t,
    }


def _scenario_loads(config: dict[str, Any], scenario: dict[str, float]) -> dict[str, float]:
    geo = config["geometry"]
    drive = config["drivetrain"]
    if "base_mass_kg" in scenario:
        base_mass = scenario["base_mass_kg"]
        payload_mass = scenario.get("payload_mass_kg", 0.0)
        mass = base_mass + payload_mass
    else:
        # Backward compatibility for older benchmark files.
        base_mass = scenario["total_mass_kg"]
        payload_mass = 0.0
        mass = base_mass
    normal_static = mass * G / drive["wheel_count"]
    torque_force = drive["motor_stall_torque_nm"] / _mm(geo["wheel_radius_mm"])
    friction_force = scenario["tyre_ground_mu"] * normal_static
    contact_force = min(torque_force, friction_force)
    return {
        "base_mass_kg": base_mass,
        "payload_mass_kg": payload_mass,
        "effective_total_mass_kg": mass,
        "normal_static_per_wheel_n": normal_static,
        "motor_torque_limited_force_n": torque_force,
        "friction_limited_force_n": friction_force,
        "turn_contact_force_per_wheel_n": contact_force,
        "design_turn_force_per_wheel_n": contact_force * scenario["dynamic_factor"],
        "design_vertical_force_per_wheel_n": normal_static * scenario["vertical_dynamic_factor"],
        "design_motor_reaction_torque_nm": min(
            drive["motor_stall_torque_nm"],
            contact_force * _mm(geo["wheel_radius_mm"]),
        ) * scenario["dynamic_factor"],
    }


def _screen_material(
    config: dict[str, Any],
    scenario: dict[str, float],
    loads: dict[str, float],
    material: dict[str, Any],
) -> dict[str, Any]:
    geo = config["geometry"]
    sec = _section(geo)
    length = _mm(geo["wheelbase_mm"])
    modulus = material["reference_elastic_modulus_gpa"] * 1e9

    # One rail receives half of the vertical chassis load. A centre point load
    # is deliberately more conservative than a uniform payload distribution.
    rail_vertical_load = 2.0 * loads["design_vertical_force_per_wheel_n"]
    vertical_moment = rail_vertical_load * length / 4.0
    vertical_stress = (
        vertical_moment * sec["outer_vertical_c_m"] / sec["vertical_bending_i_m4"]
    )
    rail_deflection = (
        rail_vertical_load * length**3
        / (48.0 * modulus * sec["vertical_bending_i_m4"])
    )

    # Longitudinal skid-steer force enters each rail through two motor nodes.
    axial_force = 2.0 * loads["design_turn_force_per_wheel_n"]
    axial_stress = axial_force / sec["area_m2"]

    # Torsional screen from the vertical offset of the tyre contact patch. The
    # much larger lateral offset produces chassis yaw and is reacted mainly by
    # the crossbars; it is reported separately because this 1-D rail model
    # cannot certify that open-frame load path.
    rail_torque = loads["design_turn_force_per_wheel_n"] * _mm(
        geo["wheel_contact_to_rail_centroid_z_mm"]
    )
    torsion_shear = rail_torque / (
        2.0 * sec["median_enclosed_area_m2"] * sec["wall_m"]
    )
    combined_stress = math.sqrt(
        (vertical_stress + axial_stress) ** 2 + 3.0 * torsion_shear**2
    )

    # The steel L-foot reacts motor torque as a bolt-row couple, rather than as
    # an 8 mm cantilever plate. This checks bearing and net-section tension in
    # the PLA node. Pull-through / layer peel still requires local FEA/coupon.
    node_t = _mm(geo["drive_node_thickness_mm"])
    node_w = _mm(geo["drive_node_width_mm"])
    bolt_d = _mm(geo["mount_bolt_diameter_mm"])
    row_spacing = _mm(geo["mount_torque_row_spacing_mm"])
    row_force = loads["design_motor_reaction_torque_nm"] / row_spacing
    bolt_force = row_force / geo["mount_bolts_per_row"]
    bearing_stress = bolt_force / (bolt_d * node_t)
    net_width = node_w - 2.0 * bolt_d
    net_section_stress = row_force / (net_width * node_t)

    # The splice check assumes the two longitudinal wheel forces on one side
    # cross one sleeve and are shared by the two transverse bolts.
    splice_d = _mm(geo["splice_bolt_diameter_mm"])
    splice_wall = _mm(geo["splice_effective_wall_mm"])
    splice_bolt_force = axial_force / geo["splice_bolt_count_per_side"]
    splice_bearing_stress = splice_bolt_force / (splice_d * splice_wall)

    strength = material["screen_strength_mpa"] * 1e6
    shear_strength = material["screen_shear_strength_mpa"] * 1e6
    factors = {
        "rail_combined": strength / combined_stress,
        "drive_node_bearing": strength / bearing_stress,
        "drive_node_net_section": strength / net_section_stress,
        "splice_bearing": strength / splice_bearing_stress,
        "rail_wall_shear": shear_strength / torsion_shear,
    }
    minimum = min(factors.values())
    criteria = config["criteria"]
    numerical_pass = (
        minimum >= criteria["minimum_safety_factor"]
        and rail_deflection * 1000.0 <= criteria["maximum_rail_deflection_mm"]
    )
    return {
        "stress_mpa": {
            "rail_vertical_bending": vertical_stress / 1e6,
            "rail_axial": axial_stress / 1e6,
            "rail_torsion_shear": torsion_shear / 1e6,
            "rail_von_mises_screen": combined_stress / 1e6,
            "drive_node_bearing": bearing_stress / 1e6,
            "drive_node_net_section": net_section_stress / 1e6,
            "splice_bearing": splice_bearing_stress / 1e6,
        },
        "rail_deflection_mm": rail_deflection * 1000.0,
        "safety_factors": factors,
        "minimum_safety_factor": minimum,
        "numerical_pass": numerical_pass,
        "unresolved_local_modes": [
            "drive-node bolt pull-through and inter-layer peel",
            "rail-to-crossbar corner stress concentration during chassis yaw",
            "splice-end stress concentration, bolt preload and hole clearance",
            "fatigue and impact damage at layer seams",
        ],
    }


def _corner_proximity_sweep(
    config: dict[str, Any],
    loads: dict[str, float],
    materials_doc: dict[str, Any],
) -> dict[str, Any] | None:
    """Screen the nominal root stress where a motor rail meets a frame corner.

    This deliberately applies a configurable stress concentration factor to a
    beam-root model. It is useful for comparing motor-to-corner distances, but
    it cannot resolve printed corner fillets, layer peel or connector contact.
    """
    sweep = config.get("corner_proximity_sweep")
    if not sweep:
        return None

    geo = config["geometry"]
    sec = _section(geo)
    kt = sweep["stress_concentration_factor"]
    yaw_moment = loads["design_turn_force_per_wheel_n"] * _mm(
        geo["wheel_contact_to_rail_centroid_y_mm"]
    )
    rail_torque = loads["design_turn_force_per_wheel_n"] * _mm(
        geo["wheel_contact_to_rail_centroid_z_mm"]
    )
    points = []
    for distance_mm in sweep["motor_center_to_corner_values_mm"]:
        vertical_root_moment = (
            loads["design_vertical_force_per_wheel_n"] * _mm(distance_mm)
            + loads["design_motor_reaction_torque_nm"]
        )
        vertical_normal = (
            vertical_root_moment
            * sec["outer_vertical_c_m"]
            / sec["vertical_bending_i_m4"]
        )
        yaw_normal = (
            yaw_moment
            * sec["outer_lateral_c_m"]
            / sec["lateral_bending_i_m4"]
        )
        torsion_shear = rail_torque / (
            2.0 * sec["median_enclosed_area_m2"] * sec["wall_m"]
        )
        # Worst same-fibre addition of the two bending components, followed by
        # a global Kt for the unmeshed printed 90-degree connection.
        concentrated_normal = kt * (vertical_normal + yaw_normal)
        concentrated_shear = kt * torsion_shear
        von_mises = math.sqrt(
            concentrated_normal**2 + 3.0 * concentrated_shear**2
        )
        material_points = {}
        for name, material in materials_doc["materials"].items():
            material_points[name] = {
                "screen_safety_factor": (
                    material["screen_strength_mpa"] * 1e6 / von_mises
                )
            }
        points.append(
            {
                "motor_center_to_corner_mm": distance_mm,
                "vertical_root_moment_nm": vertical_root_moment,
                "yaw_root_moment_nm": yaw_moment,
                "rail_torsion_nm": rail_torque,
                "nominal_von_mises_mpa_before_kt": von_mises / kt / 1e6,
                "screen_von_mises_mpa_after_kt": von_mises / 1e6,
                "materials": material_points,
            }
        )
    return {
        "method": "cantilever root screen with combined vertical/yaw bending and rail torsion",
        "stress_concentration_factor": kt,
        "assumption": sweep["assumption"],
        "points": points,
        "limitations": [
            "does not model the actual 90-degree corner, sleeve contact or fastener preload",
            "does not resolve printed-layer peel or fillet notch stress",
            "the motor reaction torque is added to vertical root bending conservatively",
        ],
    }
def analyze(
    config: dict[str, Any],
    materials_doc: dict[str, Any],
    wrench_summary: dict[str, Any] | None = None,
    fea_summary: dict[str, Any] | None = None,
) -> dict[str, Any]:
    report: dict[str, Any] = {
        "schema_version": 1,
        "benchmark_status": "BENCHMARK_INCOMPLETE",
        "input_classification": "PROVISIONAL_NOT_MEASURED",
        "geometry_section": _section(config["geometry"]),
        "scenarios": {},
        "missing_measurements": config["missing_measurements"],
    }
    all_numerical_pass = True
    for scenario_name, scenario in config["scenarios"].items():
        loads = _scenario_loads(config, scenario)
        if wrench_summary is not None and scenario_name == "conservative_turn":
            measured = wrench_summary["global"]
            loads["analytical_design_turn_force_per_wheel_n"] = loads[
                "design_turn_force_per_wheel_n"
            ]
            loads["analytical_design_motor_reaction_torque_nm"] = loads[
                "design_motor_reaction_torque_nm"
            ]
            loads["design_turn_force_per_wheel_n"] = max(
                loads["design_turn_force_per_wheel_n"],
                measured["maximum_force_norm_n"],
            )
            loads["design_motor_reaction_torque_nm"] = max(
                loads["design_motor_reaction_torque_nm"],
                measured["maximum_torque_norm_nm"],
            )
            loads["gazebo_peak_force_norm_n"] = measured["maximum_force_norm_n"]
            loads["gazebo_peak_torque_norm_nm"] = measured["maximum_torque_norm_nm"]
        material_results = {}
        for material_name, material in materials_doc["materials"].items():
            result = _screen_material(config, scenario, loads, material)
            material_results[material_name] = result
            all_numerical_pass &= result["numerical_pass"]
        report["scenarios"][scenario_name] = {
            "description": scenario["description"],
            "inputs": scenario,
            "loads": loads,
            "materials": material_results,
        }

    sweep = config.get("payload_sweep")
    if sweep:
        points = []
        for payload_mass in sweep["payload_values_kg"]:
            scenario = {
                "base_mass_kg": sweep["base_fixture_mass_kg"],
                "payload_mass_kg": payload_mass,
                "tyre_ground_mu": sweep["tyre_ground_mu"],
                "dynamic_factor": sweep["dynamic_factor"],
                "vertical_dynamic_factor": sweep["vertical_dynamic_factor"],
            }
            loads = _scenario_loads(config, scenario)
            material_points = {}
            for material_name, material in materials_doc["materials"].items():
                result = _screen_material(config, scenario, loads, material)
                material_points[material_name] = {
                    "rail_deflection_mm": result["rail_deflection_mm"],
                    "minimum_safety_factor": result["minimum_safety_factor"],
                    "numerical_pass": result["numerical_pass"],
                }
            points.append(
                {
                    "payload_mass_kg": payload_mass,
                    "effective_total_mass_kg": loads["effective_total_mass_kg"],
                    "design_vertical_force_per_wheel_n": loads[
                        "design_vertical_force_per_wheel_n"
                    ],
                    "design_turn_force_per_wheel_n": loads[
                        "design_turn_force_per_wheel_n"
                    ],
                    "turn_force_limit": (
                        "motor_torque"
                        if loads["motor_torque_limited_force_n"]
                        <= loads["friction_limited_force_n"]
                        else "tyre_friction"
                    ),
                    "materials": material_points,
                }
            )
        report["payload_sweep"] = {
            "base_fixture_mass_kg": sweep["base_fixture_mass_kg"],
            "assumption": sweep["assumption"],
            "points": points,
        }

    corner_loads = _scenario_loads(config, config["scenarios"]["conservative_turn"])
    if wrench_summary is not None:
        measured = wrench_summary["global"]
        corner_loads["design_turn_force_per_wheel_n"] = max(
            corner_loads["design_turn_force_per_wheel_n"],
            measured["maximum_force_norm_n"],
        )
        corner_loads["design_motor_reaction_torque_nm"] = max(
            corner_loads["design_motor_reaction_torque_nm"],
            measured["maximum_torque_norm_nm"],
        )
    corner_sweep = _corner_proximity_sweep(config, corner_loads, materials_doc)
    if corner_sweep is not None:
        report["corner_proximity_sweep"] = corner_sweep

    report["analytical_screen"] = (
        "PASS_WITH_UNRESOLVED_LOCAL_MODES"
        if all_numerical_pass
        else "FAIL_STIFFNESS_OR_STRENGTH"
    )
    report["decision"] = "REDESIGN_BEFORE_PRINT"
    report["decision_reason"] = (
        "The numerical beam/bearing screens are only a first bound. Peak turn loads, "
        "crossbar corner stresses, bolt pull-through, print orientation and coupon "
        "strength remain unresolved; do not print the full chassis yet."
    )
    if wrench_summary is not None:
        report["dynamic_input"] = {
            "status": "GAZEBO_JAZZY_HARMONIC_CAPTURED",
            "summary": wrench_summary["global"],
            "use": "Conservative scenario takes the maximum of analytical and Gazebo vector-norm peaks.",
        }
    if fea_summary is not None:
        report["local_drive_node_fea"] = {
            "status": "SCREEN_PASS"
            if min(
                value["screen_safety_factor"]
                for value in fea_summary["materials"].values()
            ) >= config["criteria"]["minimum_safety_factor"]
            else "SCREEN_FAIL",
            "materials": fea_summary["materials"],
            "limitations": fea_summary["limitations"],
        }
        if report["local_drive_node_fea"]["status"] == "SCREEN_PASS":
            report["benchmark_status"] = "BASELINE_COMPLETE_AWAITING_PHYSICAL_COUPON"
            report["decision"] = "PRINT_LOCAL_COUPON_NOT_FULL_CHASSIS"
            report["decision_reason"] = (
                "The rail beam screen and local isotropic drive-node FEM pass, but "
                "printed-layer peel, washer pull-through, splice behaviour and frame-corner "
                "loads still need a physical coupon / later global frame model. Print one "
                "short rail plus drive-node coupon before committing to all chassis segments."
            )
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config", type=Path, default=ROOT / "config" / "baseline.json"
    )
    parser.add_argument(
        "--materials", type=Path, default=ROOT / "config" / "materials.json"
    )
    parser.add_argument("--output", type=Path)
    parser.add_argument("--wrench-summary", type=Path)
    parser.add_argument("--fea-summary", type=Path)
    args = parser.parse_args()

    wrench_summary = _load(args.wrench_summary) if args.wrench_summary else None
    fea_summary = _load(args.fea_summary) if args.fea_summary else None
    report = analyze(
        _load(args.config), _load(args.materials), wrench_summary, fea_summary
    )
    payload = json.dumps(report, indent=2, ensure_ascii=False) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload, encoding="utf-8")
    print(payload, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
