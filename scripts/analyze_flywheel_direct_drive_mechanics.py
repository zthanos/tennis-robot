#!/usr/bin/env python3
"""Evaluate the direct-panel D5065 launcher architecture without inventing a hub."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "config/flywheel_launcher_direct_drive_mechanical_gate.json"
BALL_RESULTS = ROOT / "config/tennis_ball_compliance_calibration_results.json"


def build_result() -> dict[str, object]:
    # Accepted launcher CAD datums, metres unless stated otherwise.
    wheel_radius = 0.100
    wheel_width = 0.050
    wheel_y = 0.129
    panel_x = 0.256
    panel_y = 0.314
    panel_t = 0.008
    panel_center_z = 0.043
    panel_inside_z = panel_center_z - panel_t / 2.0
    panel_outside_z = panel_center_z + panel_t / 2.0

    # Manufacturer-backed D5065 dimensions already captured by the repository.
    motor_mass = 0.49
    motor_diameter = 0.050
    motor_length = 0.065
    shaft_diameter = 0.008
    shaft_projection = 0.030
    shaft_flat_length = 0.024
    shaft_flat_depth = 0.0005
    mount_hole_diameter = 0.004
    mount_pcd = 0.030

    wheel_outer_face_z = wheel_width / 2.0
    wheel_inner_face_z = -wheel_width / 2.0
    shaft_tip_z = panel_outside_z - shaft_projection
    shaft_inside_panel = panel_inside_z - shaft_tip_z
    shaft_inside_wheel_envelope = wheel_outer_face_z - shaft_tip_z
    panel_to_wheel_gap = panel_inside_z - wheel_outer_face_z

    body_edge_clearance = panel_y / 2.0 - (wheel_y + motor_diameter / 2.0)
    mounting_hole_edge_ligament = panel_y / 2.0 - (
        wheel_y + mount_pcd / 2.0 + mount_hole_diameter / 2.0
    )
    opening_ligaments = {
        f"diameter_{diameter_mm}_mm": (
            mount_pcd / 2.0 - mount_hole_diameter / 2.0 - diameter_mm / 2000.0
        )
        for diameter_mm in (10, 16, 18, 20)
    }

    ball = json.loads(BALL_RESULTS.read_text(encoding="utf-8"))
    loading_stiffness = ball["calibrated_parameters"]["loading_stiffness_n_m_pow"]
    nominal_per_side_compression = (0.066 - 0.058) / 2.0
    quasi_static_ball_reaction = loading_stiffness * nominal_per_side_compression**1.5
    calibration_peak_force = max(item["peak_force_n"] for item in ball["rebound_results"])
    screen_radial_reaction = 1.25 * calibration_peak_force

    gravity = 9.80665
    provisional_wheel_mass = 0.40
    motor_static_moment = motor_mass * gravity * motor_length / 2.0
    provisional_wheel_static_moment = provisional_wheel_mass * gravity * panel_center_z
    static_moment_without_hub = motor_static_moment + provisional_wheel_static_moment
    radial_moment = screen_radial_reaction * panel_center_z

    motor_peak_torque = 0.031 * 85.0
    provisional_motor_torque = 0.031 * 20.0
    bolt_radius = mount_pcd / 2.0
    conservative_overturning_moment = radial_moment + static_moment_without_hub
    # Worst cardinal orientation: two bolts form the tension/compression couple.
    bolt_tension = conservative_overturning_moment / (2.0 * bolt_radius)
    radial_shear_per_bolt = screen_radial_reaction / 4.0
    torque_shear_per_bolt = motor_peak_torque / (4.0 * bolt_radius)
    resultant_shear_per_bolt = math.hypot(radial_shear_per_bolt, torque_shear_per_bolt)
    panel_bearing_stress = bolt_tension / (panel_t * mount_hole_diameter)
    strip_stress_30mm = 6.0 * conservative_overturning_moment / (0.030 * panel_t**2)
    strip_stress_50mm = 6.0 * conservative_overturning_moment / (0.050 * panel_t**2)

    classifications = {
        "D5065_DIRECT_PANEL_MOUNT_GEOMETRICALLY_VALID": True,
        "D5065_DIRECT_PANEL_MOUNT_STRUCTURALLY_SCREENED": True,
        "FLYWHEEL_PANEL_CUTOUT_DEFINED": False,
        "FLYWHEEL_DIRECT_DRIVE_HUB_DEFINED": False,
        "FLYWHEEL_SHAFT_ENGAGEMENT_VALIDATED": False,
        "FLYWHEEL_AXIAL_RETENTION_DEFINED": False,
        "FLYWHEEL_ROTATING_MASS_DEFINED": False,
        "FLYWHEEL_ROTATING_INERTIA_DEFINED": False,
        "FLYWHEEL_MECHANICAL_GATE_A_PASSED": False,
    }

    return {
        "schema_version": 1,
        "generated_on": "2026-08-26",
        "scope": "standalone flywheel launcher direct-drive mechanical definition",
        "decision": {
            "status": "PARTIAL_DEFINITION_STOP",
            "summary": "The accepted upper panel is a geometrically feasible direct D5065 mounting plate and has been structurally screened. Gate A remains stopped at the wheel/hub/retention/mass-property interface.",
            "separate_motor_bracket_required": False,
            "independent_motor_pitch_hardware_required": False,
            "launcher_trials_run": False,
        },
        "evidence_classes": [
            "MANUFACTURER_SPEC",
            "MEASURED_FROM_HARDWARE",
            "MEASURED_FROM_CAD",
            "DERIVED",
            "ASSUMED / PROVISIONAL",
            "MISSING",
        ],
        "authoritative_launcher_datums": {
            "flywheel_diameter_m": 0.200,
            "flywheel_width_m": wheel_width,
            "wheel_centre_spacing_m": 0.258,
            "nip_m": 0.058,
            "cradle_plate_size_m": [panel_x, panel_y, panel_t],
            "launcher_pitch_deg": 20.0,
        },
        "direct_panel_mount": {
            "motor_centres_launcher_local_m": [
                [0.0, wheel_y, panel_outside_z],
                [0.0, -wheel_y, panel_outside_z],
            ],
            "shaft_axes_launcher_local": [[0.0, 0.0, -1.0], [0.0, 0.0, -1.0]],
            "orientation": "motor body outside upper panel; shaft and hub toward launcher inside; shaft coaxial with flywheel",
            "motor_body_edge_clearance_m": body_edge_clearance,
            "mount_hole_edge_ligament_m": mounting_hole_edge_ligament,
            "mount_pattern": {
                "hole_count": 4,
                "hole_diameter_m": mount_hole_diameter,
                "pitch_circle_diameter_m": mount_pcd,
                "classification": "MANUFACTURER_SPEC",
                "thread_or_clearance": None,
                "usable_thread_engagement_m": None,
                "missing": "Confirm whether the four 4 mm drawing features are threaded, their thread specification, and usable depth.",
            },
            "motor_body": {
                "diameter_m": motor_diameter,
                "length_m": motor_length,
                "mass_kg": motor_mass,
                "classification": "MANUFACTURER_SPEC",
                "source": "https://shop.odriverobotics.com/products/odrive-custom-motor-d5065",
            },
            "wire_and_service_clearance": {
                "status": "MISSING",
                "reason": "Repository evidence does not dimension lead exit, 4 mm bullet connector envelope, thermistor lead exit, bend radius, or tool swing.",
            },
        },
        "axial_stack_launcher_local_z_m": {
            "motor_body_outside_end": panel_outside_z + motor_length,
            "motor_mounting_face": panel_outside_z,
            "panel_outside_face": panel_outside_z,
            "panel_inside_face": panel_inside_z,
            "shaft_tip": shaft_tip_z,
            "hub_start": None,
            "hub_end": None,
            "flywheel_outer_face": wheel_outer_face_z,
            "flywheel_centre_plane": 0.0,
            "flywheel_inner_face": wheel_inner_face_z,
            "axial_retention_hardware": None,
            "shaft_projection_beyond_panel_inside_m": shaft_inside_panel,
            "shaft_projection_inside_wheel_envelope_m": shaft_inside_wheel_envelope,
            "panel_inside_to_flywheel_outer_gap_m": panel_to_wheel_gap,
            "manufacturer_flat_length_m": shaft_flat_length,
            "manufacturer_flat_depth_m": shaft_flat_depth,
        },
        "panel_cutout_screen": {
            "shaft_diameter_m": shaft_diameter,
            "final_cutout_defined": False,
            "reason": "The required opening depends on the selected hub outside diameter, pinch-bolt/tool access, and axial-retention installation path.",
            "central_opening_to_mount_hole_edge_ligament_m": opening_ligaments,
            "note": "10/16/18/20 mm openings are geometric sensitivity cases only, not released manufacturing dimensions.",
        },
        "purchased_hub_audit": {
            "selected": None,
            "candidates": [
                {
                    "manufacturer": "goBILDA",
                    "part": "1309-0016-0008 Sonic Hub, 8 mm round bore",
                    "material": "aluminum",
                    "mass_kg": 0.014,
                    "wheel_interface": "M4 threaded holes on 16 mm pattern",
                    "shaft_retention": "dual pinch bolts",
                    "source": "https://www.gobilda.com/1309-series-sonic-hub-8mm-bore/",
                },
                {
                    "manufacturer": "goBILDA",
                    "part": "1310-0016-0008 Hyper Hub, 8 mm round bore",
                    "material": "aluminum",
                    "mass_kg": 0.024,
                    "wheel_interface": "16 mm goBILDA pattern; exact target-wheel compatibility not established",
                    "shaft_retention": "balanced dual-pinch clamping",
                    "source": "https://www.gobilda.com/1310-series-hyper-hub-8mm-bore/",
                },
            ],
            "stop_reason": "No repository evidence defines the actual 200 x 50 mm flywheel centre bore, recess, bolt pattern, material, or torque rating. Candidate clamp hubs also provide no quantified torque/axial capacity for this D-shaft application in the captured evidence.",
        },
        "structural_screen": {
            "status": "SCREEN_PASS_WITH_ASSUMPTIONS_NOT_PHYSICAL_VALIDATION",
            "physical_inputs": {
                "motor_mass_kg": motor_mass,
                "panel_thickness_m": panel_t,
                "calibrated_quasi_static_per_wheel_reaction_at_4mm_n": quasi_static_ball_reaction,
            },
            "provisional_inputs": {
                "flywheel_mass_kg": provisional_wheel_mass,
                "hub_mass_kg": None,
                "panel_alloy_for_stress_comparison": "6061-T6, assumed only; CAD identifies aluminium density but no alloy/temper",
                "screen_radial_reaction_n": screen_radial_reaction,
                "screen_radial_reaction_basis": "1.25 x maximum independent ball-rebound calibration peak; conservative screen case, not a measured launcher load",
            },
            "derived": {
                "static_gravity_moment_without_hub_nm": static_moment_without_hub,
                "radial_screen_moment_nm": radial_moment,
                "motor_torque_at_provisional_20a_nm": provisional_motor_torque,
                "motor_torque_at_manufacturer_85a_3s_nm": motor_peak_torque,
                "worst_case_bolt_tension_n": bolt_tension,
                "radial_shear_per_bolt_n": radial_shear_per_bolt,
                "torque_shear_per_bolt_n": torque_shear_per_bolt,
                "resultant_shear_per_bolt_n": resultant_shear_per_bolt,
                "panel_bearing_stress_pa": panel_bearing_stress,
                "strip_bending_stress_30mm_effective_width_pa": strip_stress_30mm,
                "strip_bending_stress_50mm_effective_width_pa": strip_stress_50mm,
            },
            "reinforcement_decision": "No reinforcement released. The provisional stress screen does not justify a separate cradle; final cutout, alloy, hub/flywheel mass, actual launcher reaction, fastener engagement, and vibration evidence must precede any local doubler decision.",
            "vibration_resonance": "UNRESOLVED: requires actual rotating mass/inertia, runout/balance, motor bearing stiffness, panel boundary conditions, and modal/bench measurement.",
        },
        "rotating_assembly": {
            "flywheel_mass_kg": None,
            "flywheel_inertia_kg_m2": None,
            "hub_mass_kg": None,
            "hub_inertia_kg_m2": None,
            "motor_rotor_inertia_kg_m2": None,
            "provisional_simulation_wheel_mass_kg": 0.40,
            "provisional_solid_cylinder_spin_inertia_kg_m2": 0.002,
            "provisional_values_accepted_for_capability_simulation": False,
        },
        "simulation_ownership": {
            "standalone_xacro": "ros2_ws/src/tennis_robot/urdf/components/flywheel_launcher_module.urdf.xacro",
            "standalone_xacro_updated": False,
            "reason": "The wheel, hub, retention and complete rotating inertia are not sufficiently defined; the existing 0.40 kg wheel and 0.002 kg m^2 solid-cylinder inertia remain explicit provisional placeholders.",
            "complete_robot_files_updated": False,
        },
        "remaining_physical_measurements": [
            "Select or provide the actual 200 x 50 mm flywheel and measure its mass, centre bore/hex, recesses, face thickness, any bolt-circle diameters/hole sizes, and axial datum.",
            "Confirm D5065 mounting-face hole thread/clearance type and usable engagement depth on delivered hardware; record lead/thermistor exit and connector service envelope.",
            "After matching a metal hub to the measured wheel interface, record hub drawing, engagement length, torque/axial rating, mass, axial stop and removable retention hardware.",
            "Obtain manufacturer motor-rotor inertia or measure it independently; then calculate the complete motor rotor + hub + flywheel inertia.",
            "Verify upper-panel alloy/temper, final cutout, flatness and mounting boundary; weigh the finished rotating assembly and perform balance/runout and vibration checks.",
        ],
        "classifications": classifications,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    result = build_result()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result["classifications"], indent=2, sort_keys=True))
    return 2 if not result["classifications"]["FLYWHEEL_MECHANICAL_GATE_A_PASSED"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
