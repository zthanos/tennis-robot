#!/usr/bin/env python3
"""Resolve the measured wheel/ball contact mechanism for one launcher trial."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scripts.analyze_flywheel_capability_case import analyze, closest, rows


BALL_RADIUS_M = 0.033
BALL_MASS_KG = 0.058
BALL_INERTIA_KG_M2 = 4.2108e-5
GRAVITY_M_S2 = 9.8


def add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def subtract(a, b):
    return tuple(x - y for x, y in zip(a, b))


def scale(value, vector):
    return tuple(value * x for x in vector)


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def magnitude(vector):
    return math.sqrt(dot(vector, vector))


def unit(vector):
    length = magnitude(vector)
    return scale(1.0 / length, vector) if length else (0.0, 0.0, 0.0)


def local_position(position):
    pitch = math.radians(20.0)
    x, y, z = position[0], position[1], position[2] - 0.350
    return (math.cos(pitch) * x + math.sin(pitch) * z,
            y,
            -math.sin(pitch) * x + math.cos(pitch) * z)


def integrate(samples, key):
    return sum(key(previous) * (current["time_s"] - previous["time_s"])
               for previous, current in zip(samples, samples[1:]))


def contact_row(contact, state, friction, start_time, wheel_inertia, precontact_speed):
    side = "left" if contact["contact_id"] < 0.5 else "right"
    prefix = "left" if side == "left" else "right"
    ball_position = tuple(state[f"ball_{axis}_m"] for axis in "xyz")
    ball_linear = tuple(state[f"ball_v{axis}_m_s"] for axis in "xyz")
    ball_angular = tuple(state[f"ball_w{axis}_rad_s"] for axis in "xyz")
    wheel_position = tuple(state[f"{prefix}_{axis}_m"] for axis in "xyz")
    wheel_axis = tuple(state[f"{prefix}_axis_{axis}"] for axis in "xyz")
    wheel_angular = scale(state[f"{prefix}_speed_rad_s"], wheel_axis)
    point = tuple(contact[f"contact_{axis}_m"] for axis in "xyz")
    normal = tuple(contact[f"normal_{axis}"] for axis in "xyz")
    tangential_force = tuple(contact[f"tangential_force_{axis}_n"] for axis in "xyz")
    ball_arm = subtract(point, ball_position)
    wheel_arm = subtract(point, wheel_position)
    ball_point_velocity = add(ball_linear, cross(ball_angular, ball_arm))
    wheel_point_velocity = cross(wheel_angular, wheel_arm)
    relative = subtract(ball_point_velocity, wheel_point_velocity)
    relative_tangential = subtract(relative, scale(dot(relative, normal), normal))
    wheel_tangential = subtract(wheel_point_velocity, scale(dot(wheel_point_velocity, normal), normal))
    ball_tangential = subtract(ball_point_velocity, scale(dot(ball_point_velocity, normal), normal))
    drive_direction = unit(wheel_tangential)
    wheel_surface_signed = dot(wheel_tangential, drive_direction)
    ball_surface_signed = dot(ball_tangential, drive_direction)
    slip_speed = magnitude(relative_tangential)
    slip_ratio = slip_speed / max(abs(wheel_surface_signed), abs(ball_surface_signed), 0.1)
    force_limit = friction * contact["normal_force_n"]
    utilization = magnitude(tangential_force) / force_limit if force_limit > 0 else 0.0
    ball_force = add(scale(contact["normal_force_n"], normal), tangential_force)
    wheel_speed = state[f"{prefix}_speed_rad_s"]
    motor_torque = state[f"{prefix}_torque_nm"]
    droop = abs(precontact_speed[side]) - abs(wheel_speed)
    return {
        "time_s": contact["time_s"],
        "time_from_first_contact_s": contact["time_s"] - start_time,
        "contact_id": int(contact["contact_id"]),
        "side": side,
        "ball_x_m": ball_position[0],
        "ball_y_m": ball_position[1],
        "ball_z_m": ball_position[2],
        "ball_vx_m_s": ball_linear[0],
        "ball_vy_m_s": ball_linear[1],
        "ball_vz_m_s": ball_linear[2],
        "ball_speed_m_s": magnitude(ball_linear),
        "ball_wx_rad_s": ball_angular[0],
        "ball_wy_rad_s": ball_angular[1],
        "ball_wz_rad_s": ball_angular[2],
        "compression_m": contact["compression_m"],
        "compression_rate_m_s": contact["compression_rate_m_s"],
        "normal_force_n": contact["normal_force_n"],
        "tangential_force_n": magnitude(tangential_force),
        "tangential_force_x_n": tangential_force[0],
        "tangential_force_y_n": tangential_force[1],
        "tangential_force_z_n": tangential_force[2],
        "wheel_speed_rad_s": wheel_speed,
        "wheel_speed_droop_rad_s": droop,
        "wheel_speed_droop_percent": 100.0 * droop / abs(precontact_speed[side]),
        "applied_motor_torque_nm": motor_torque,
        "motor_mechanical_power_w": motor_torque * wheel_speed,
        "wheel_rotational_energy_j": 0.5 * wheel_inertia * wheel_speed * wheel_speed,
        "wheel_surface_speed_at_contact_m_s": wheel_surface_signed,
        "ball_surface_speed_at_contact_m_s": ball_surface_signed,
        "relative_tangential_velocity_x_m_s": relative_tangential[0],
        "relative_tangential_velocity_y_m_s": relative_tangential[1],
        "relative_tangential_velocity_z_m_s": relative_tangential[2],
        "slip_velocity_m_s": slip_speed,
        "slip_ratio": slip_ratio,
        "friction_utilization": utilization,
        "near_coulomb_limit": utilization >= 0.95,
        "ball_contact_power_w": dot(ball_force, ball_point_velocity),
        "wheel_contact_power_w": dot(scale(-1.0, ball_force), wheel_point_velocity),
        "contact_x_m": point[0],
        "contact_y_m": point[1],
        "contact_z_m": point[2],
        "wheel_x_m": wheel_position[0],
        "wheel_y_m": wheel_position[1],
        "wheel_z_m": wheel_position[2],
        "wheel_axis_x": wheel_axis[0],
        "wheel_axis_y": wheel_axis[1],
        "wheel_axis_z": wheel_axis[2],
    }


def arc_angle(samples):
    total = 0.0
    previous = None
    for row in samples:
        radial = unit((row["contact_x_m"] - row["wheel_x_m"],
                       row["contact_y_m"] - row["wheel_y_m"],
                       row["contact_z_m"] - row["wheel_z_m"]))
        axis = (row["wheel_axis_x"], row["wheel_axis_y"], row["wheel_axis_z"])
        if previous is not None:
            total += math.atan2(dot(axis, cross(previous, radial)), dot(previous, radial))
        previous = radial
    return total


def summarize_contact(samples):
    duration = samples[-1]["time_s"] - samples[0]["time_s"]
    normal_impulse = integrate(samples, lambda row: row["normal_force_n"])
    tangential_impulse = integrate(samples, lambda row: row["tangential_force_n"])
    tangential_vector = [integrate(samples, lambda row, axis=axis: row[f"tangential_force_{axis}_n"])
                         for axis in "xyz"]
    near_duration = integrate(samples, lambda row: 1.0 if row["near_coulomb_limit"] else 0.0)
    loading_duration = integrate(samples, lambda row: 1.0 if row["compression_rate_m_s"] > 0 else 0.0)
    maximum = max(samples, key=lambda row: row["compression_m"])
    return {
        "contact_start_time_s": samples[0]["time_s"],
        "contact_end_time_s": samples[-1]["time_s"],
        "sampled_contact_duration_s": duration,
        "normal_impulse_n_s": normal_impulse,
        "tangential_impulse_n_s": tangential_impulse,
        "tangential_impulse_vector_n_s": tangential_vector,
        "mean_normal_force_n": normal_impulse / duration if duration else None,
        "mean_tangential_force_n": tangential_impulse / duration if duration else None,
        "peak_normal_force_n": max(row["normal_force_n"] for row in samples),
        "peak_tangential_force_n": max(row["tangential_force_n"] for row in samples),
        "maximum_per_wheel_compression_m": maximum["compression_m"],
        "maximum_compression_time_s": maximum["time_s"],
        "maximum_compression_position_world_m": [maximum[f"ball_{axis}_m"] for axis in "xyz"],
        "maximum_compression_position_launcher_local_m": list(local_position(
            tuple(maximum[f"ball_{axis}_m"] for axis in "xyz"))),
        "loading_duration_s": loading_duration,
        "unloading_duration_s": max(duration - loading_duration, 0.0),
        "near_coulomb_limit_duration_s": near_duration,
        "near_coulomb_limit_fraction": near_duration / duration if duration else None,
        "mean_friction_utilization": integrate(samples, lambda row: row["friction_utilization"]) / duration if duration else None,
        "maximum_friction_utilization": max(row["friction_utilization"] for row in samples),
        "mean_slip_velocity_m_s": integrate(samples, lambda row: row["slip_velocity_m_s"]) / duration if duration else None,
        "maximum_slip_velocity_m_s": max(row["slip_velocity_m_s"] for row in samples),
        "mean_slip_ratio": integrate(samples, lambda row: row["slip_ratio"]) / duration if duration else None,
        "contact_arc_rad": arc_angle(samples),
        "contact_arc_deg": math.degrees(arc_angle(samples)),
        "contact_arc_length_m": abs(arc_angle(samples)) * 0.1,
        "work_on_ball_at_contact_j": integrate(samples, lambda row: row["ball_contact_power_w"]),
        "work_on_wheel_at_contact_j": integrate(samples, lambda row: row["wheel_contact_power_w"]),
    }


def analyze_diagnostic(case_dir, wheel_mass, wheel_inertia, friction, timestep):
    base = analyze(case_dir, wheel_mass, wheel_inertia, friction, timestep)
    states = rows(case_dir / "state.csv")
    contacts = [row for row in rows(case_dir / "contacts.csv") if row["contact_id"] < 1.5]
    start = min(row["time_s"] for row in contacts)
    precontact_speed = {
        "left": base["left_actual_precontact_rad_s"],
        "right": base["right_actual_precontact_rad_s"],
    }
    series = [contact_row(contact, closest(states, contact["time_s"]), friction, start,
                          wheel_inertia, precontact_speed)
              for contact in contacts]
    by_side = {side: [row for row in series if row["side"] == side]
               for side in ("left", "right")}
    summaries = {side: summarize_contact(samples) for side, samples in by_side.items()}
    times_left = {row["time_s"] for row in by_side["left"]}
    times_right = {row["time_s"] for row in by_side["right"]}
    bilateral_times = sorted(times_left & times_right)
    bilateral_start, bilateral_end = bilateral_times[0], bilateral_times[-1]
    state_start, state_end = closest(states, bilateral_start), closest(states, bilateral_end)
    ball_start = tuple(state_start[f"ball_{axis}_m"] for axis in "xyz")
    ball_end = tuple(state_end[f"ball_{axis}_m"] for axis in "xyz")
    combined_by_time = {}
    for row in series:
        combined_by_time.setdefault(row["time_s"], {})[row["side"]] = row
    diametral = [entry["left"]["compression_m"] + entry["right"]["compression_m"]
                 for entry in combined_by_time.values() if set(entry) == {"left", "right"}]
    energy = base["energy_accounting"]
    ball_energy_gain = (energy["ball_exit_translational_j"] + energy["ball_exit_rotational_j"] +
                        energy["potential_energy_change_j"] - energy["ball_initial_translational_j"])
    wheel_energy_loss = energy["wheel_kinetic_pre_j"] - energy["wheel_kinetic_post_j"]
    diagnostics = {
        "friction_assumption": friction,
        "friction_classification": ("DIAGNOSTIC_ONLY_NOT_PHYSICAL_CALIBRATION"
                                    if friction > 0.9 else "REUSED_BOUNDED_SENSITIVITY_ASSUMPTION"),
        "contacts": summaries,
        "bilateral_contact": {
            "start_time_s": bilateral_start,
            "end_time_s": bilateral_end,
            "sampled_duration_s": bilateral_end - bilateral_start,
            "ball_start_position_world_m": list(ball_start),
            "ball_end_position_world_m": list(ball_end),
            "ball_travel_distance_m": magnitude(subtract(ball_end, ball_start)),
            "maximum_diametral_compression_m": max(diametral),
        },
        "energy_transfer": {
            "motor_available_energy_basis": "pre-contact rotational kinetic energy plus motor work during contact",
            "wheel_rotational_energy_pre_j": energy["wheel_kinetic_pre_j"],
            "wheel_rotational_energy_post_j": energy["wheel_kinetic_post_j"],
            "wheel_rotational_energy_loss_j": wheel_energy_loss,
            "postrelease_wheel_energy_fraction": energy["wheel_kinetic_post_j"] / energy["wheel_kinetic_pre_j"],
            "motor_work_during_contact_j": energy["motor_mechanical_work_event_j"],
            "ball_mechanical_energy_gain_j": ball_energy_gain,
            "contact_work_on_ball_integral_j": sum(item["work_on_ball_at_contact_j"] for item in summaries.values()),
            "contact_work_on_wheels_integral_j": sum(item["work_on_wheel_at_contact_j"] for item in summaries.values()),
            "contact_dissipation_j": energy["contact_dissipation_j"],
            "energy_residual_j": energy["residual_j"],
            "energy_residual_fraction": energy["residual_fraction"],
        },
        "secondary_checks": {
            "successful_launch": base["successful_launch"],
            "post_release_noncontact": base["post_release_geometry_clear"],
            "target_reachable": base["target_reachable"],
            "per_contact_compression_inside_calibrated_envelope": max(
                summaries[side]["maximum_per_wheel_compression_m"] for side in summaries
            ) <= 0.028161218979320318,
            "peak_force_below_contact_force_cap": max(
                summaries[side]["peak_normal_force_n"] for side in summaries) < 5000.0,
            "motor_current_inside_limit": max(
                base["motor_response"]["left_peak_event_current_a"],
                base["motor_response"]["right_peak_event_current_a"]) <= 20.0 + 1e-9,
            "estimated_bus_voltage_inside_limit": base["motor_response"]["estimated_peak_required_bus_voltage_v"] <= 12.8,
            "energy_residual_inside_limit": abs(energy["residual_fraction"]) <= 0.02,
            "finite_contact_telemetry": all(
                math.isfinite(value)
                for row in series
                for value in (row["normal_force_n"], row["tangential_force_n"],
                              row["slip_velocity_m_s"], row["friction_utilization"])),
        },
    }
    diagnostics["secondary_checks"]["passed"] = all(diagnostics["secondary_checks"].values())
    return base, diagnostics, series


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("case_dir", type=Path)
    parser.add_argument("--wheel-mass", type=float, default=0.90)
    parser.add_argument("--wheel-inertia", type=float, default=0.006751162108290868)
    parser.add_argument("--friction", type=float, required=True)
    parser.add_argument("--timestep", type=float, default=0.001)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--timeseries-output", type=Path)
    args = parser.parse_args()
    base, diagnostic, series = analyze_diagnostic(
        args.case_dir, args.wheel_mass, args.wheel_inertia, args.friction, args.timestep)
    output = args.output or args.case_dir / "diagnostic_result.json"
    timeseries = args.timeseries_output or args.case_dir / "diagnostic_timeseries.csv"
    output.write_text(json.dumps({"capability": base, "diagnostic": diagnostic}, indent=2,
                                 allow_nan=False) + "\n", encoding="utf-8")
    with timeseries.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(series[0]))
        writer.writeheader()
        writer.writerows(series)
    print(output)
    print(timeseries)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
