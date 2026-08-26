#!/usr/bin/env python3
"""Audit the measured post-nip path and size a local cradle exit relief."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path


BALL_RADIUS_M = 0.033
PITCH_RAD = math.radians(20.0)
PLATE_X_HALF_M = 0.128
PLATE_Y_HALF_M = 0.157
PLATE_THICKNESS_M = 0.008
LOWER_PLATE_TOP_M = -0.039
UPPER_PLATE_BOTTOM_M = 0.039
CAD_CORRIDOR_X_M = (0.100, 0.320)
CAD_CORRIDOR_RADIUS_M = 0.045
MOTOR_AXIS_Y_M = 0.129
MOTOR_MOUNT_PCD_RADIUS_M = 0.015
MOTOR_HOLE_RADIUS_M = 0.002
ALUMINIUM_DENSITY_KG_M3 = 2700.0


def load_csv(path: Path) -> list[dict[str, float]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return [{key: float(value) for key, value in row.items()}
                for row in csv.DictReader(stream)]


def local_state(row: dict[str, float]) -> dict[str, list[float]]:
    c = math.cos(PITCH_RAD)
    s = math.sin(PITCH_RAD)
    x = row["ball_x_m"]
    y = row["ball_y_m"]
    z = row["ball_z_m"] - 0.350
    vx = row["ball_vx_m_s"]
    vy = row["ball_vy_m_s"]
    vz = row["ball_vz_m_s"]
    return {
        "position_m": [c * x + s * z, y, -s * x + c * z],
        "velocity_m_s": [c * vx + s * vz, vy, -s * vx + c * vz],
    }


def ballistic_points(release: dict[str, list[float]]) -> list[tuple[float, float]]:
    x0, _, z0 = release["position_m"]
    vx, _, vz = release["velocity_m_s"]
    ax = -9.8 * math.sin(PITCH_RAD)
    az = -9.8 * math.cos(PITCH_RAD)
    points = []
    for index in range(4001):
        time_s = index * 0.000025
        x = x0 + vx * time_s + 0.5 * ax * time_s * time_s
        z = z0 + vz * time_s + 0.5 * az * time_s * time_s
        points.append((x, z))
        if x > CAD_CORRIDOR_X_M[1] + 0.050:
            break
    return points


def footprint(points: list[tuple[float, float]], clearance_m: float,
              include_cad_corridor: bool) -> dict[str, object]:
    radius = BALL_RADIUS_M + clearance_m
    disks = []
    for x, z in points:
        vertical_distance = z - LOWER_PLATE_TOP_M
        if 0.0 <= vertical_distance < radius:
            disks.append((x, math.sqrt(radius * radius - vertical_distance * vertical_distance)))
    dx = 0.00005
    samples: list[tuple[float, float]] = []
    count = int(round(2.0 * PLATE_X_HALF_M / dx)) + 1
    cad_half_width = math.sqrt(CAD_CORRIDOR_RADIUS_M ** 2 - LOWER_PLATE_TOP_M ** 2)
    for index in range(count):
        x = -PLATE_X_HALF_M + index * dx
        half_width = max(
            (math.sqrt(max(0.0, disk_radius ** 2 - (x - centre_x) ** 2))
             for centre_x, disk_radius in disks if abs(x - centre_x) <= disk_radius),
            default=0.0,
        )
        if include_cad_corridor and CAD_CORRIDOR_X_M[0] <= x <= PLATE_X_HALF_M:
            half_width = max(half_width, cad_half_width)
        samples.append((x, half_width))
    active = [(x, half_width) for x, half_width in samples if half_width > 1e-12]
    area = sum(2.0 * half_width * dx for _, half_width in samples)
    return {
        "clearance_m": clearance_m,
        "includes_cad_nominal_corridor": include_cad_corridor,
        "bounding_box_xy_m": {
            "x": [min(x for x, _ in active), max(x for x, _ in active)],
            "y": [-max(width for _, width in active), max(width for _, width in active)],
        },
        "removed_area_m2": area,
        "removed_volume_m3": area * PLATE_THICKNESS_M,
        "removed_6061_mass_kg": area * PLATE_THICKNESS_M * ALUMINIUM_DENSITY_KG_M3,
        "remaining_side_ligament_each_m": PLATE_Y_HALF_M - max(width for _, width in active),
        "samples": samples,
    }


def distance_to_footprint(point: tuple[float, float], samples: list[tuple[float, float]]) -> float:
    px, py = point
    return min(math.hypot(px - x, max(0.0, abs(py) - half_width))
               for x, half_width in samples if half_width > 0.0)


def strip_samples(result: dict[str, object]) -> dict[str, object]:
    result = dict(result)
    result.pop("samples", None)
    return result


def retest_evidence(case_dir: Path) -> dict[str, object]:
    states = load_csv(case_dir / "state.csv")
    contacts = load_csv(case_dir / "contacts.csv")
    wheel_contacts = [row for row in contacts if row["contact_id"] < 1.5]
    contact_end = max(row["time_s"] for row in wheel_contacts)
    ground = [row for row in contacts if row["contact_id"] > 1.5 and row["time_s"] > contact_end]
    ground_time = min(row["time_s"] for row in ground) if ground else states[-1]["time_s"]
    post = [row for row in states if contact_end + 0.002 <= row["time_s"] < ground_time]
    maximum_non_gravity_delta_v = 0.0
    for previous, row in zip(post, post[1:]):
        dt = row["time_s"] - previous["time_s"]
        residual = math.sqrt(
            (row["ball_vx_m_s"] - previous["ball_vx_m_s"]) ** 2 +
            (row["ball_vy_m_s"] - previous["ball_vy_m_s"]) ** 2 +
            (row["ball_vz_m_s"] - previous["ball_vz_m_s"] + 9.8 * dt) ** 2
        )
        maximum_non_gravity_delta_v = max(maximum_non_gravity_delta_v, residual)
    result_path = case_dir / "result.json"
    reduced = json.loads(result_path.read_text(encoding="utf-8")) if result_path.exists() else {}
    return {
        "case_id": "same_low_energy_mu_0p3_dt_1ms_after_exit_relief",
        "wheel_target_rad_s": [80.0, -80.0],
        "friction_assumption": 0.3,
        "timestep_s": 0.001,
        "injection_speed_m_s": 3.0,
        "exit_speed_m_s": reduced.get("exit_speed_m_s"),
        "exit_elevation_deg": reduced.get("elevation_deg"),
        "post_release_geometry_clear": reduced.get("post_release_geometry_clear"),
        "maximum_non_gravity_delta_v_before_ground_m_s": maximum_non_gravity_delta_v,
        "first_ground_contact_time_s": ground_time if ground else None,
        "first_bounce_xyz_m": reduced.get("first_bounce_xyz_m"),
        "recovery_time_s": reduced.get("recovery_time_s"),
        "passed": bool(reduced.get("successful_launch") and
                       reduced.get("post_release_geometry_clear") and
                       maximum_non_gravity_delta_v < 0.001),
    }


def audit(case_dir: Path, retest_case_dir: Path | None = None) -> dict[str, object]:
    states = load_csv(case_dir / "state.csv")
    contacts = load_csv(case_dir / "contacts.csv")
    wheel_contacts = [row for row in contacts if row["contact_id"] < 1.5]
    dt = states[1]["time_s"] - states[0]["time_s"]
    final_contact_time = max(row["time_s"] for row in wheel_contacts)
    release_row = min(states, key=lambda row: abs(row["time_s"] - (final_contact_time + 2.0 * dt)))
    release = local_state(release_row)
    clearance_limit = LOWER_PLATE_TOP_M + BALL_RADIUS_M
    impact_row = next(
        row for row in states
        if row["time_s"] > final_contact_time and local_state(row)["position_m"][2] <= clearance_limit
    )
    impact = local_state(impact_row)
    release_world = [release_row[key] for key in ("ball_x_m", "ball_y_m", "ball_z_m")]
    impact_world = [impact_row[key] for key in ("ball_x_m", "ball_y_m", "ball_z_m")]
    distance = math.dist(release_world, impact_world)
    points = ballistic_points(release)

    cutouts = {}
    raw_results = {}
    for label, clearance in (("zero_contact", 0.0), ("plus_2mm", 0.002), ("plus_5mm", 0.005)):
        result = footprint(points, clearance, include_cad_corridor=False)
        raw_results[label] = result
        cutouts[label] = strip_samples(result)
    practical = footprint(points, 0.005, include_cad_corridor=True)
    practical_samples = practical["samples"]
    cutouts["practical_plus_5mm_union_cad_corridor"] = strip_samples(practical)

    mount_holes = []
    for sign in (-1.0, 1.0):
        axis_y = sign * MOTOR_AXIS_Y_M
        mount_holes.extend([
            (MOTOR_MOUNT_PCD_RADIUS_M, axis_y),
            (-MOTOR_MOUNT_PCD_RADIUS_M, axis_y),
            (0.0, axis_y + MOTOR_MOUNT_PCD_RADIUS_M),
            (0.0, axis_y - MOTOR_MOUNT_PCD_RADIUS_M),
        ])
    mount_edge_distance = min(
        distance_to_footprint(hole, practical_samples) - MOTOR_HOLE_RADIUS_M
        for hole in mount_holes
    )
    shaft_axis_distance = min(
        distance_to_footprint((0.0, sign * MOTOR_AXIS_Y_M), practical_samples)
        for sign in (-1.0, 1.0)
    )

    full_ball_exit = next(
        (x, z) for x, z in points
        if x >= CAD_CORRIDOR_X_M[0] and abs(z) + BALL_RADIUS_M > CAD_CORRIDOR_RADIUS_M
    )
    centre_inside = all(
        abs(z) <= CAD_CORRIDOR_RADIUS_M
        for x, z in points if CAD_CORRIDOR_X_M[0] <= x <= CAD_CORRIDOR_X_M[1]
    )
    corridor_plate_half_width = math.sqrt(
        CAD_CORRIDOR_RADIUS_M ** 2 - LOWER_PLATE_TOP_M ** 2)

    result = {
        "schema_version": 1,
        "scope": "standalone_flywheel_post_nip_exit_corridor_preimplementation_audit",
        "cad_cylinder": {
            "classification": "NOMINAL_LAUNCH_CORRIDOR",
            "physical_hardware": False,
            "exit_keep_out_or_reference": True,
            "source_file": "cad/flywheel-launcher-v0/launcher-envelope.scad",
            "source_module": "exit_guide_envelope",
            "parameter_source": "cad/flywheel-launcher-v0/params.scad",
            "diameter_m": 0.090,
            "length_m": 0.220,
            "launcher_local_origin_m": [0.210, 0.0, 0.0],
            "launcher_local_axis": [1.0, 0.0, 0.0],
            "launcher_local_x_extent_m": [0.100, 0.320],
            "collision_geometry": False,
            "intended_for_manufacture": False,
        },
        "measured_path": {
            "final_wheel_contact_time_s": final_contact_time,
            "wheel_release_time_s": release_row["time_s"],
            "wheel_release_world_position_m": release_world,
            "wheel_release_launcher_local_position_m": release["position_m"],
            "wheel_release_world_velocity_m_s": [release_row[key] for key in (
                "ball_vx_m_s", "ball_vy_m_s", "ball_vz_m_s")],
            "wheel_release_launcher_local_velocity_m_s": release["velocity_m_s"],
            "first_cradle_contact_time_s": impact_row["time_s"],
            "first_cradle_contact_world_position_m": impact_world,
            "first_cradle_contact_launcher_local_position_m": impact["position_m"],
            "collision_name": "flywheel_launcher_frame_link_fixed_joint_lump__flywheel_cradle_lower_plate_col_collision",
            "contact_normal_world_on_ball": [-math.sin(PITCH_RAD), 0.0, math.cos(PITCH_RAD)],
            "geometric_penetration_m": clearance_limit - impact["position_m"][2],
            "centre_distance_release_to_contact_m": distance,
        },
        "cad_corridor_comparison": {
            "measured_centre_path_inside_cylinder": centre_inside,
            "full_66mm_swept_ball_inside_cylinder": False,
            "full_ball_first_exits_cylinder_at_launcher_local_x_m": full_ball_exit[0],
            "full_ball_centre_z_at_exit_m": full_ball_exit[1],
            "cradle_plate_intersects_cylinder": True,
            "plate_overlap_launcher_local_x_m": [CAD_CORRIDOR_X_M[0], PLATE_X_HALF_M],
            "cylinder_half_width_at_lower_plate_inner_face_m": corridor_plate_half_width,
        },
        "root_cause": {
            "A_cradle_incorrectly_reconstructed": False,
            "B_cad_lacks_required_exit_opening": True,
            "C_actual_exit_departs_nominal_axis_enough_to_exhaust_6mm_clearance": True,
            "D_shaped_exit_cutout_required": True,
            "E_other_fixed_component_redirects_ball_before_impact": False,
        },
        "cutout_study": cutouts,
        "proposed_practical_cutout": {
            "definition": "union of measured ballistic swept sphere at radius 38 mm and explicit CAD diameter-90-mm nominal corridor, clipped to each plate",
            "lower_plate": strip_samples(practical),
            "upper_plate": {
                "definition": "CAD nominal corridor intersection only; no unsupported mirror of measured downward path",
                "bounding_box_xy_m": {
                    "x": [CAD_CORRIDOR_X_M[0], PLATE_X_HALF_M],
                    "y": [-corridor_plate_half_width, corridor_plate_half_width],
                },
                "removed_area_m2": (PLATE_X_HALF_M - CAD_CORRIDOR_X_M[0]) * 2.0 * corridor_plate_half_width,
                "removed_volume_m3": ((PLATE_X_HALF_M - CAD_CORRIDOR_X_M[0]) *
                                      2.0 * corridor_plate_half_width * PLATE_THICKNESS_M),
                "removed_6061_mass_kg": ((PLATE_X_HALF_M - CAD_CORRIDOR_X_M[0]) *
                                         2.0 * corridor_plate_half_width *
                                         PLATE_THICKNESS_M * ALUMINIUM_DENSITY_KG_M3),
                "remaining_side_ligament_each_m": PLATE_Y_HALF_M - corridor_plate_half_width,
            },
            "minimum_distance_to_motor_mount_hole_edge_m": mount_edge_distance,
            "minimum_distance_to_shaft_axis_m": shaft_axis_distance,
            "minimum_distance_to_panel_side_edge_m": practical["remaining_side_ligament_each_m"],
            "downstream_edge_is_intentionally_open": True,
            "structural_classification": "GEOMETRIC_PASS_STRUCTURAL_REVIEW_REQUIRED",
        },
        "future_capability_envelope": {
            "minimum_current_test_clearance": "defined by measured path plus 5 mm",
            "future_capability_keep_out": "provisional existing diameter-90-mm CAD corridor only",
            "final_12_to_18_m_s_angular_envelope_defined": False,
            "reason": "exit elevation across the stopped RPM and timestep matrices has not been measured",
        },
        "decisions_preimplementation": {
            "CAD_CYLINDER_IS_PHYSICAL_HARDWARE": False,
            "CAD_CYLINDER_IS_EXIT_KEEP_OUT_OR_REFERENCE": True,
            "CURRENT_CRADLE_VIOLATES_BALL_EXIT_ENVELOPE": True,
            "LOWER_PLATE_EXIT_CUTOUT_REQUIRED": True,
            "MINIMUM_EXIT_CUTOUT_DEFINED": True,
            "PRACTICAL_EXIT_CLEARANCE_DEFINED": True,
            "POST_FLYWHEEL_PATH_NONCONTACTING": False,
            "POST_FLYWHEEL_BARREL_CONTACT_REQUIRED": False,
            "CRADLE_EXIT_GEOMETRY_READY_FOR_CAPABILITY_RETEST": False,
            "STRUCTURAL_REVIEW_REQUIRED": True,
        },
    }
    if retest_case_dir is not None:
        retest = retest_evidence(retest_case_dir)
        result["postimplementation_retest"] = retest
        result["decisions_final"] = {
            "CAD_CYLINDER_IS_PHYSICAL_HARDWARE": False,
            "CAD_CYLINDER_IS_EXIT_KEEP_OUT_OR_REFERENCE": True,
            "CURRENT_CRADLE_VIOLATES_BALL_EXIT_ENVELOPE": not retest["passed"],
            "LOWER_PLATE_EXIT_CUTOUT_REQUIRED": True,
            "MINIMUM_EXIT_CUTOUT_DEFINED": True,
            "PRACTICAL_EXIT_CLEARANCE_DEFINED": True,
            "POST_FLYWHEEL_PATH_NONCONTACTING": retest["passed"],
            "POST_FLYWHEEL_BARREL_CONTACT_REQUIRED": False,
            "CRADLE_EXIT_GEOMETRY_READY_FOR_CAPABILITY_RETEST": retest["passed"],
            "STRUCTURAL_REVIEW_REQUIRED": True,
        }
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("case_dir", type=Path)
    parser.add_argument("--retest-case-dir", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.case_dir, args.retest_case_dir)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
