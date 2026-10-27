#!/usr/bin/env python3
"""P2-51 static ski-jump, ballistic envelope, capacity and option screen."""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path

G = 9.80665
BALL_R = 0.033
BALL_D_MM = 66.0
BALL_VOLUME_L = 4 * math.pi * (BALL_R ** 3) / 3 * 1000
SHELL_K = 2 / 3
RAMP_R = 0.075
TANGENT_LENGTH = 0.052
RAMP_START = (0.435, 0.010)
ARC_START = (RAMP_START[0] - TANGENT_LENGTH, RAMP_START[1])
ANGLES = (35, 45, 55)
RAMP_MUS = (0.15, 0.25, 0.40)
EXIT_SPEEDS = (3.0, 3.5, 4.0, 4.5)
OFFSETS_MM = (0, -50, 50, -100, 100)


def ramp_exit(angle_deg: float) -> tuple[float, float]:
    a = math.radians(angle_deg)
    return ARC_START[0] - RAMP_R * math.sin(a), ARC_START[1] + RAMP_R * (1 - math.cos(a))


def ramp_forward(v0: float, mu: float, angle_deg: float, ds: float = 0.0001) -> dict:
    """Sliding-to-rolling thin-shell integration; friction is a material sweep."""
    v, omega, loss_per_mass = v0, 0.0, 0.0
    rolling_at_exit = False
    phases = ((0.0, TANGENT_LENGTH), (math.radians(angle_deg), RAMP_R * math.radians(angle_deg)))
    for end_angle, length in phases:
        steps = max(1, round(length / ds))
        step = length / steps
        for i in range(steps):
            theta = 0.0 if end_angle == 0 else (i + 0.5) * step / RAMP_R
            normal_per_mass = G * math.cos(theta) + (v * v / RAMP_R if end_angle else 0.0)
            slip = v - BALL_R * omega
            dt = step / max(v, 1e-6)
            if slip > 1e-5:
                friction_acc = mu * normal_per_mass
                new_v = max(0.0, v + (-G * math.sin(theta) - friction_acc) * dt)
                new_omega = omega + friction_acc / (SHELL_K * BALL_R) * dt
                loss_per_mass += mu * normal_per_mass * slip * dt
                if BALL_R * new_omega >= new_v:
                    shared = (v + SHELL_K * BALL_R * omega) / (1 + SHELL_K)
                    new_v, new_omega = shared, shared / BALL_R
                v, omega = new_v, new_omega
            else:
                v = max(0.0, v - G * math.sin(theta) / (1 + SHELL_K) * dt)
                omega = v / BALL_R
            if v <= 1e-6:
                return {"exit_speed_m_s": 0.0, "exit_spin_rad_s": omega,
                        "specific_dissipation_j_kg": loss_per_mass,
                        "rolling_at_exit": False}
    rolling_at_exit = abs(v - BALL_R * omega) <= 0.02
    return {"exit_speed_m_s": v, "exit_spin_rad_s": omega,
            "specific_dissipation_j_kg": loss_per_mass,
            "rolling_at_exit": rolling_at_exit}


def required_entry_speed(target: float, mu: float, angle: float) -> dict:
    lo, hi = target, 10.0
    for _ in range(60):
        mid = (lo + hi) / 2
        if ramp_forward(mid, mu, angle)["exit_speed_m_s"] < target:
            lo = mid
        else:
            hi = mid
    result = ramp_forward(hi, mu, angle)
    return {"target_exit_speed_m_s": target, "required_ramp_entry_speed_m_s": hi,
            "speed_loss_m_s": hi - target, **result}


def transfer_efficiency(surface_speed: float, tread_mu: float, offset_mm: int) -> float:
    """Provisional map anchored to P2-50; Gazebo P2-51 replaces it."""
    # P2-50 central mu=.6: 0.672 at 2.8, 0.644 at 3.0, 0.623 at 3.2.
    central = 0.67 / (1 + 0.12 * max(0.0, surface_speed - 2.8))
    if tread_mu == 0.9:
        central = min(0.82, central + 0.12)
    penalty = {0: 1.0, 50: 0.82, 100: 0.68}[abs(offset_mm)]
    return central * penalty


def required_surface_speed(entry_target: float, tread_mu: float, offset_mm: int) -> dict:
    lo, hi = entry_target, 15.0
    for _ in range(80):
        mid = (lo + hi) / 2
        delivered = mid * transfer_efficiency(mid, tread_mu, offset_mm)
        if delivered < entry_target:
            lo = mid
        else:
            hi = mid
    eta = transfer_efficiency(hi, tread_mu, offset_mm)
    return {"tread_mu": tread_mu, "offset_mm": offset_mm,
            "required_surface_speed_m_s_estimate": hi,
            "estimated_transfer_ratio": eta,
            "rpm_d200": hi / (math.pi * 0.2) * 60,
            "model_status": "provisional P2-50 anchored; replace with P2-51 Gazebo/physical measurement"}


def trajectory(angle_deg: float, speed: float, lateral_mm: float) -> dict:
    x0, floor_z = ramp_exit(angle_deg)
    z0 = floor_z + BALL_R
    a = math.radians(angle_deg)
    vx, vz = -speed * math.cos(a), speed * math.sin(a)
    # Leaned backboard occupies about X=-64 mm at the relevant height.
    rear_contact_x = -0.064 + BALL_R
    floor_center_z = 0.075
    t_rear = (rear_contact_x - x0) / vx
    z_rear = z0 + vz * t_rear - 0.5 * G * t_rear * t_rear
    disc = vz * vz + 2 * G * (z0 - floor_center_z)
    t_floor = (vz + math.sqrt(max(0.0, disc))) / G
    x_floor = x0 + vx * t_floor
    hit_rear = t_rear <= t_floor and z_rear >= floor_center_z
    t_contact = t_rear if hit_rear else t_floor
    x_contact = rear_contact_x if hit_rear else x_floor
    z_contact = z_rear if hit_rear else floor_center_z
    # Frozen P2-40 X300 crossbar: its collision top is Z=42 mm.  This
    # explicit screen was added after the superseded R125 ski allowed the
    # ball to clip that crossbar before takeoff.
    crossbar_x = 0.300
    t_crossbar = (crossbar_x - x0) / vx
    crossbar_center_z = z0 + vz * t_crossbar - 0.5 * G * t_crossbar * t_crossbar
    crossbar_clearance = crossbar_center_z - BALL_R - 0.042
    t_apex = vz / G
    z_apex_free = z0 + vz * vz / (2 * G)
    z_high = z_apex_free if 0 <= t_apex <= t_contact else max(z0, z_contact)
    ball_top = z_high + BALL_R
    inside_plan = (-0.085 + BALL_R <= x_contact <= 0.278 - BALL_R
                   and abs(lateral_mm / 1000) <= 0.172 - BALL_R)
    return {
        "angle_deg": angle_deg, "exit_speed_m_s": speed, "lateral_mm": lateral_mm,
        "takeoff_center_xz_mm": [1000 * x0, 1000 * z0],
        "first_contact": "rear_backboard" if hit_rear else "floor",
        "first_contact_center_xyz_mm": [1000 * x_contact, lateral_mm, 1000 * z_contact],
        "highest_ball_center_z_mm_before_contact": 1000 * z_high,
        "highest_ball_top_z_mm_before_contact": 1000 * ball_top,
        "lidar_margin_to_ball_top_mm": 390 - 1000 * ball_top,
        "inside_plan_with_ball_radius": inside_plan,
        "x300_crossbar_ball_clearance_mm": 1000 * crossbar_clearance,
        "passes_x300_crossbar_8mm_margin": crossbar_clearance >= 0.008,
        "passes_lidar_10mm_margin": ball_top <= 0.380,
        "passes_static_envelope": inside_plan and ball_top <= 0.380 and crossbar_clearance >= 0.008,
    }


def collection_angle_screen() -> list[dict]:
    rows = []
    for angle in (-3, -5, -7):
        # P2-39 selected edge is locked at Z23; wheel clearance loses R*sin(a).
        raw = 23.0 - 100.0 * abs(math.sin(math.radians(angle)))
        rows.append({"collection_angle_deg": angle,
                     "raw_wheel_ground_clearance_mm": raw,
                     "after_5mm_motion_allowance_mm": raw - 5,
                     "ball_to_usable_edge_mm": 8.0,
                     "passes_moving_clearance_12mm": raw - 5 >= 12.0,
                     "estimated_incoming_angle_deg": 16 + (-angle - 3),
                     "ski_deflection_to_35_deg": 35 - (16 + (-angle - 3))})
    return rows


def capacity() -> dict:
    # Main 293x344 area plus 70x280 central rear extension; reserve hardware.
    area_mm2 = 293 * 344 + 70 * 280
    net_volume_l = area_mm2 * (320 - 55) / 1e6 - 1.7
    rows = {}
    for count in (40, 45):
        packing = 0.56
        mean = (count * BALL_VOLUME_L / 1000) / packing / (area_mm2 / 1e6) * 1000
        local_top = 70 + mean + BALL_D_MM
        rows[str(count)] = {"solid_ball_volume_l": count * BALL_VOLUME_L,
                            "mean_bulk_height_mm": mean,
                            "conservative_local_top_z_mm": local_top}
    return {"plan_area_mm2": area_mm2, "net_volume_l_estimate": net_volume_l,
            "volumetric_capacity_at_0p56": net_volume_l * 0.56 / BALL_VOLUME_L,
            "pile": rows, "mandatory_40_volume_pass": net_volume_l >= 40 * BALL_VOLUME_L / 0.56,
            "target_45_volume_pass": net_volume_l >= 45 * BALL_VOLUME_L / 0.56}


def main() -> None:
    ramp_rows = []
    requirement_rows = []
    for angle in ANGLES:
        for mu in RAMP_MUS:
            for target in EXIT_SPEEDS:
                item = {"angle_deg": angle, "ramp_mu": mu, **required_entry_speed(target, mu, angle)}
                ramp_rows.append(item)
                for tread_mu in (0.6, 0.9):
                    for offset in OFFSETS_MM:
                        requirement_rows.append({**item, **required_surface_speed(item["required_ramp_entry_speed_m_s"], tread_mu, offset)})

    trajectories = [trajectory(a, v, y) for a in ANGLES for v in EXIT_SPEEDS for y in OFFSETS_MM]
    candidates = [x for x in trajectories if x["passes_static_envelope"]]
    report = {
        "status": "P2_51_STATIC_BALLISTIC_SCREEN_COMPLETE_PENDING_GAZEBO",
        "passive_only": True,
        "powered_ramp_excluded_by_user": True,
        "ski_jump": {"tangent_length_mm": 52, "radius_mm": 75,
                     "arc_plus_tangent_length_mm_by_angle": {str(a): 1000 * (TANGENT_LENGTH + RAMP_R * math.radians(a)) for a in ANGLES},
                     "exit_surface_xz_mm_by_angle": {str(a): [1000*x for x in ramp_exit(a)] for a in ANGLES}},
        "collection_angle_screen": collection_angle_screen(),
        "ramp_loss_rows": ramp_rows,
        "provisional_nip_surface_requirements": requirement_rows,
        "trajectory_rows": trajectories,
        "static_pass_count": len(candidates),
        "static_total_count": len(trajectories),
        "recommended_static_point": trajectory(35, 4.0, 100),
        "basket": capacity(),
        "packaging": {
            "plan_x_mm": [-85, 278], "main_half_width_y_mm": 172,
            "rear_extension_half_width_y_mm": 140,
            "rear_fit0403_exclusion": "rear central extension stays inside |Y|<=140; FIT0403 envelopes begin near |Y|=160",
            "parkside_gap_mm": 12.5,
            "ugreen": "move beside Parkside/rear electronics carrier; interface only, no electronics change",
            "side_rim_z_mm": 320, "local_rear_backboard_top_z_mm": 375,
            "lidar_scan_z_mm": 390, "front_ballistic_aperture_clear_width_mm": 280,
            "floor_slope_deg": 4.5,
            "feed_channel": "floor drains toward unchanged low channel and common-throat diverter; mechanism remains 5d",
        },
        "interpretation": [
            "The R75 ski is screened explicitly against the frozen X300 crossbar with an 8 mm ball-clearance criterion.",
            "35 deg is the only angle with >=10 mm LiDAR margin for the full 3.0..4.5 m/s sweep.",
            "45 deg is statically viable through 3.5 m/s; 4.0 has only about 5 mm and 4.5 exceeds Z390.",
            "55 deg exceeds the LiDAR plane even at 3.0 m/s before rear interception.",
            "-5 and -7 deg reduce ski deflection but fail the frozen moving ground-clearance gate when the 8 mm usable-edge condition is retained.",
            "Offset transfer efficiencies are provisional; Gazebo and the TPU incline test replace them.",
        ],
    }
    outdir = Path("engineering/chassis-strength-bench/results/shared_flywheel_phase2")
    outdir.mkdir(parents=True, exist_ok=True)
    (outdir / "p251_ballistic_static.json").write_text(json.dumps(report, indent=2) + "\n")
    for filename, rows in (("p251_ramp_loss.csv", ramp_rows),
                           ("p251_surface_requirement.csv", requirement_rows),
                           ("p251_trajectories.csv", trajectories)):
        with (outdir / filename).open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
