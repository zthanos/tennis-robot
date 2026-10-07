#!/usr/bin/env python3
"""Reduce one isolated flywheel native-Gazebo trial to a capability record."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path


BALL_MASS_KG = 0.058
BALL_INERTIA_KG_M2 = 4.2108e-5
GRAVITY_M_S2 = 9.8
WHEEL_RADIUS_M = 0.100
JOINT_DAMPING_NM_S_RAD = 0.002
MOTOR_TORQUE_CONSTANT_NM_A = 0.031
MOTOR_SPEED_CONSTANT_RPM_V = 270.0
MOTOR_PHASE_NEUTRAL_RESISTANCE_OHM = 0.039
MOTOR_BUS_VOLTAGE_V = 12.8
PLATE_HALF_LENGTH_M = 0.128
PLATE_INNER_BALL_CENTER_CLEARANCE_M = 0.006


def rows(path: Path) -> list[dict[str, float]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return [{key: float(value) for key, value in row.items()}
                for row in csv.DictReader(stream)]


def closest(states: list[dict[str, float]], time_s: float) -> dict[str, float]:
    return min(states, key=lambda row: abs(row["time_s"] - time_s))


def norm(values: tuple[float, ...]) -> float:
    return math.sqrt(sum(value * value for value in values))


def integrate_samples(samples: list[dict[str, float]], value) -> float:
    """Left-integrate telemetry while respecting the actual sample spacing."""
    total = 0.0
    for previous, current in zip(samples, samples[1:]):
        total += value(previous) * (current["time_s"] - previous["time_s"])
    return total


def integrate_contacts(samples: list[dict[str, float]], value) -> float:
    """Integrate each contact stream independently (rows are interleaved)."""
    contact_ids = sorted({row["contact_id"] for row in samples})
    return sum(
        integrate_samples([row for row in samples if row["contact_id"] == contact_id], value)
        for contact_id in contact_ids
    )


def analyze(case_dir: Path, wheel_mass: float, wheel_inertia: float,
            friction: float | None, timestep: float) -> dict:
    states = rows(case_dir / "state.csv")
    contacts = rows(case_dir / "contacts.csv")
    injection_rows = [row for row in states if row["injected"] > 0.5]
    wheel_contacts = [row for row in contacts if row["contact_id"] < 1.5]
    if not injection_rows or not wheel_contacts:
        return {
            "successful_launch": False,
            "failure_reason": "missing deterministic injection or bilateral wheel contact",
            "friction_assumption": friction,
            "wheel_mass_assumption_kg": wheel_mass,
            "wheel_inertia_assumption_kg_m2": wheel_inertia,
            "timestep_s": timestep,
            "validity_classification": "REJECTED_NO_WHEEL_LAUNCH",
        }

    injection_time = injection_rows[0]["injection_time_s"]
    contact_start = min(row["time_s"] for row in wheel_contacts)
    contact_end = max(row["time_s"] for row in wheel_contacts)
    pre_window = [row for row in states if contact_start - 0.2 <= row["time_s"] < contact_start]
    pre = pre_window[-1]
    exit_state = closest(states, contact_end + 2.0 * timestep)
    post = exit_state
    left = [row for row in wheel_contacts if row["contact_id"] < 0.5]
    right = [row for row in wheel_contacts if 0.5 <= row["contact_id"] < 1.5]

    velocity = tuple(exit_state[key] for key in ("ball_vx_m_s", "ball_vy_m_s", "ball_vz_m_s"))
    spin = tuple(exit_state[key] for key in ("ball_wx_rad_s", "ball_wy_rad_s", "ball_wz_rad_s"))
    speed = norm(velocity)
    horizontal = math.hypot(velocity[0], velocity[1])
    elevation = math.degrees(math.atan2(velocity[2], horizontal))
    azimuth = math.degrees(math.atan2(velocity[1], velocity[0]))
    lateral = (-velocity[1] / horizontal, velocity[0] / horizontal, 0.0) if horizontal else (0, 1, 0)
    topspin = sum(spin[i] * lateral[i] for i in range(3))
    sidespin = spin[2]

    actual_left = sum(row["left_speed_rad_s"] for row in pre_window) / len(pre_window)
    actual_right = sum(row["right_speed_rad_s"] for row in pre_window) / len(pre_window)
    target_left = pre["left_target_rad_s"]
    target_right = pre["right_target_rad_s"]
    in_contact_states = [row for row in states if contact_start <= row["time_s"] <= contact_end]
    left_min = min(abs(row["left_speed_rad_s"]) for row in in_contact_states)
    right_min = min(abs(row["right_speed_rad_s"]) for row in in_contact_states)
    left_max = max(abs(row["left_speed_rad_s"]) for row in in_contact_states)
    right_max = max(abs(row["right_speed_rad_s"]) for row in in_contact_states)
    left_droop = abs(actual_left) - left_min
    right_droop = abs(actual_right) - right_min

    recovery_time = None
    ready_for = 0.0
    prior_time = contact_end
    for row in states:
        if row["time_s"] <= contact_end:
            continue
        dt = row["time_s"] - prior_time
        prior_time = row["time_s"]
        if (abs(row["left_speed_rad_s"] - target_left) <= 1.0 and
                abs(row["right_speed_rad_s"] - target_right) <= 1.0):
            ready_for += dt
            if ready_for >= 0.2:
                recovery_time = row["time_s"] - contact_end
                break
        else:
            ready_for = 0.0

    settled_rows = [row for row in states if row["settled"] > 0.5 and row["time_s"] < contact_start]
    spinup_time = settled_rows[0]["time_s"] if settled_rows else None
    reachable = (abs(actual_left - target_left) <= 1.0 and
                 abs(actual_right - target_right) <= 1.0)

    event_states = [row for row in states if contact_start <= row["time_s"] <= post["time_s"]]
    motor_work = ((post["left_motor_work_j"] + post["right_motor_work_j"]) -
                  (pre["left_motor_work_j"] + pre["right_motor_work_j"]))
    drivetrain_loss = integrate_samples(
        event_states,
        lambda row: JOINT_DAMPING_NM_S_RAD *
        (row["left_speed_rad_s"] ** 2 + row["right_speed_rad_s"] ** 2),
    )
    elastic_stored = integrate_contacts(
        wheel_contacts,
        lambda row: max(row["elastic_force_n"] * row["compression_rate_m_s"], 0.0),
    )
    elastic_recovered = integrate_contacts(
        wheel_contacts,
        lambda row: max(-row["elastic_force_n"] * row["compression_rate_m_s"], 0.0),
    )
    elastic_hysteresis = max(elastic_stored - elastic_recovered, 0.0)
    damping_loss = integrate_contacts(
        wheel_contacts,
        lambda row: max(row["damping_force_n"] * row["compression_rate_m_s"], 0.0),
    )
    friction_loss = integrate_contacts(
        wheel_contacts,
        lambda row: norm((row["tangential_force_x_n"], row["tangential_force_y_n"],
                          row["tangential_force_z_n"])) * row["tangential_speed_m_s"],
    )
    contact_loss = elastic_hysteresis + damping_loss + friction_loss
    wheel_ke_pre_left = 0.5 * wheel_inertia * actual_left ** 2
    wheel_ke_pre_right = 0.5 * wheel_inertia * actual_right ** 2
    wheel_ke_post_left = 0.5 * wheel_inertia * post["left_speed_rad_s"] ** 2
    wheel_ke_post_right = 0.5 * wheel_inertia * post["right_speed_rad_s"] ** 2
    wheel_ke_pre = wheel_ke_pre_left + wheel_ke_pre_right
    wheel_ke_post = wheel_ke_post_left + wheel_ke_post_right
    ball_ke = 0.5 * BALL_MASS_KG * speed ** 2
    ball_rot_ke = 0.5 * BALL_INERTIA_KG_M2 * norm(spin) ** 2
    injection_speed = norm((injection_rows[4]["ball_vx_m_s"], injection_rows[4]["ball_vy_m_s"],
                            injection_rows[4]["ball_vz_m_s"])) if len(injection_rows) > 4 else 0.0
    ball_ke_initial = 0.5 * BALL_MASS_KG * injection_speed ** 2
    hold_z = injection_rows[0]["ball_z_m"]
    potential_change = BALL_MASS_KG * GRAVITY_M_S2 * (exit_state["ball_z_m"] - hold_z)
    residual = (wheel_ke_pre + ball_ke_initial + motor_work - wheel_ke_post - ball_ke -
                ball_rot_ke - potential_change - contact_loss - drivetrain_loss)
    input_energy = wheel_ke_pre + ball_ke_initial + max(motor_work, 0.0)

    after_exit = [row for row in states if row["time_s"] >= exit_state["time_s"]]
    apex = max(after_exit, key=lambda row: row["ball_z_m"])
    ground = [row for row in contacts if row["contact_id"] > 1.5 and row["time_s"] > contact_end]
    bounce_time = ground[0]["time_s"] if ground else None
    bounce_state = closest(states, bounce_time) if bounce_time is not None else None
    pre_bounce_candidates = [row for row in states if bounce_time is not None and row["time_s"] < bounce_time]
    pre_bounce_state = pre_bounce_candidates[-1] if pre_bounce_candidates else None
    pitch = math.radians(20.0)
    panel_clip = None
    post_release_interference = None
    previous = None
    for row in after_exit:
        local_x = math.cos(pitch) * row["ball_x_m"] + math.sin(pitch) * (row["ball_z_m"] - 0.350)
        local_z = -math.sin(pitch) * row["ball_x_m"] + math.cos(pitch) * (row["ball_z_m"] - 0.350)
        if previous is not None and (bounce_time is None or row["time_s"] < bounce_time):
            sample_dt = row["time_s"] - previous["time_s"]
            residual_delta_v = norm((
                row["ball_vx_m_s"] - previous["ball_vx_m_s"],
                row["ball_vy_m_s"] - previous["ball_vy_m_s"],
                row["ball_vz_m_s"] - previous["ball_vz_m_s"] + GRAVITY_M_S2 * sample_dt,
            ))
            if residual_delta_v > 0.05:
                post_release_interference = {
                    "time_s": row["time_s"], "local_x_m": local_x,
                    "local_z_m": local_z, "non_gravity_delta_v_m_s": residual_delta_v,
                }
                if (abs(local_x) <= PLATE_HALF_LENGTH_M and
                        abs(local_z) >= PLATE_INNER_BALL_CENTER_CLEARANCE_M):
                    panel_clip = dict(post_release_interference)
                break
        previous = row

    first_times = {
        "left_s": min((row["time_s"] for row in left), default=None),
        "right_s": min((row["time_s"] for row in right), default=None),
    }
    order = "bilateral" if (first_times["left_s"] is not None and first_times["right_s"] is not None and
                             abs(first_times["left_s"] - first_times["right_s"]) <= timestep) else (
        "left_then_right" if first_times["left_s"] < first_times["right_s"] else "right_then_left")

    recovery_state = closest(states, contact_end + recovery_time) if recovery_time is not None else None
    recovery_motor_work = (
        (recovery_state["left_motor_work_j"] + recovery_state["right_motor_work_j"] -
         post["left_motor_work_j"] - post["right_motor_work_j"])
        if recovery_state else None
    )
    peak_left_torque = max(abs(row["left_torque_nm"]) for row in event_states)
    peak_right_torque = max(abs(row["right_torque_nm"]) for row in event_states)
    max_abs_speed = max(abs(actual_left), abs(actual_right))
    back_emf_v = max_abs_speed * 60.0 / (2.0 * math.pi) / MOTOR_SPEED_CONSTANT_RPM_V
    required_voltage_v = back_emf_v + max(peak_left_torque, peak_right_torque) / MOTOR_TORQUE_CONSTANT_NM_A * MOTOR_PHASE_NEUTRAL_RESISTANCE_OHM

    return {
        "successful_launch": bool(left and right and reachable and post_release_interference is None),
        "wheel_contact_event_success": bool(left and right and reachable),
        "failure_reason": ("post-release ball clips cradle plate" if panel_clip else
                           "post-release fixed-component interference" if post_release_interference else None),
        "post_release_geometry_clear": post_release_interference is None,
        "first_panel_clip": panel_clip,
        "first_post_release_interference": post_release_interference,
        "mechanical_pitch_deg": 20.0,
        "left_target_rad_s": target_left,
        "right_target_rad_s": target_right,
        "left_actual_rad_s": actual_left,
        "right_actual_rad_s": actual_right,
        "left_actual_precontact_rad_s": actual_left,
        "right_actual_precontact_rad_s": actual_right,
        "target_reachable": reachable,
        "spinup_time_s": spinup_time,
        "surface_speed_m_s": 0.5 * (abs(actual_left) + abs(actual_right)) * WHEEL_RADIUS_M,
        "first_contact_time_s": first_times,
        "contact_order": order,
        "contact_duration_s": {
            "left": (max(row["time_s"] for row in left) - min(row["time_s"] for row in left) + timestep),
            "right": (max(row["time_s"] for row in right) - min(row["time_s"] for row in right) + timestep),
        },
        "exit_position_xyz_m": [exit_state[key] for key in ("ball_x_m", "ball_y_m", "ball_z_m")],
        "exit_velocity_vector_m_s": list(velocity),
        "exit_speed_m_s": speed,
        "transfer_ratio": speed / (0.5 * (abs(actual_left) + abs(actual_right)) * WHEEL_RADIUS_M),
        "elevation_deg": elevation,
        "azimuth_deg": azimuth,
        "spin_vector_rad_s": list(spin),
        "spin_equivalent_rpm": norm(spin) * 60.0 / (2.0 * math.pi),
        "topspin_rad_s": topspin,
        "sidespin_rad_s": sidespin,
        "max_ball_compression_m": max(row["compression_m"] for row in wheel_contacts),
        "left_peak_force_n": max(row["normal_force_n"] for row in left),
        "right_peak_force_n": max(row["normal_force_n"] for row in right),
        "left_peak_tangential_force_n": max(norm((row["tangential_force_x_n"], row["tangential_force_y_n"], row["tangential_force_z_n"])) for row in left),
        "right_peak_tangential_force_n": max(norm((row["tangential_force_x_n"], row["tangential_force_y_n"], row["tangential_force_z_n"])) for row in right),
        "left_droop_rad_s": left_droop,
        "right_droop_rad_s": right_droop,
        "left_rpm_droop": left_droop * 60.0 / (2.0 * math.pi),
        "right_rpm_droop": right_droop * 60.0 / (2.0 * math.pi),
        "left_droop_percent": 100.0 * left_droop / abs(actual_left),
        "right_droop_percent": 100.0 * right_droop / abs(actual_right),
        "left_min_contact_rad_s": left_min,
        "right_min_contact_rad_s": right_min,
        "recovery_time_s": recovery_time,
        "motor_response": {
            "left_precontact_rpm": actual_left * 60.0 / (2.0 * math.pi),
            "right_precontact_rpm": actual_right * 60.0 / (2.0 * math.pi),
            "left_min_contact_rpm_magnitude": left_min * 60.0 / (2.0 * math.pi),
            "right_min_contact_rpm_magnitude": right_min * 60.0 / (2.0 * math.pi),
            "left_max_contact_rpm_magnitude": left_max * 60.0 / (2.0 * math.pi),
            "right_max_contact_rpm_magnitude": right_max * 60.0 / (2.0 * math.pi),
            "left_precontact_torque_nm": pre["left_torque_nm"],
            "right_precontact_torque_nm": pre["right_torque_nm"],
            "left_precontact_current_a": abs(pre["left_torque_nm"]) / MOTOR_TORQUE_CONSTANT_NM_A,
            "right_precontact_current_a": abs(pre["right_torque_nm"]) / MOTOR_TORQUE_CONSTANT_NM_A,
            "left_peak_event_torque_nm": peak_left_torque,
            "right_peak_event_torque_nm": peak_right_torque,
            "left_peak_event_current_a": peak_left_torque / MOTOR_TORQUE_CONSTANT_NM_A,
            "right_peak_event_current_a": peak_right_torque / MOTOR_TORQUE_CONSTANT_NM_A,
            "estimated_peak_required_bus_voltage_v": required_voltage_v,
            "provisional_bus_voltage_v": MOTOR_BUS_VOLTAGE_V,
            "recovery_motor_mechanical_work_j": recovery_motor_work,
        },
        "apex_height_m": apex["ball_z_m"],
        "time_to_apex_s": apex["time_s"] - exit_state["time_s"],
        "first_bounce_xyz_m": ([bounce_state[key] for key in ("ball_x_m", "ball_y_m", "ball_z_m")]
                               if bounce_state else None),
        "first_bounce_time_s": (bounce_state["time_s"] - injection_time if bounce_state else None),
        "first_bounce_time_from_exit_s": (bounce_state["time_s"] - exit_state["time_s"]
                                           if bounce_state else None),
        "horizontal_range_m": (math.hypot(bounce_state["ball_x_m"] - exit_state["ball_x_m"],
                                           bounce_state["ball_y_m"] - exit_state["ball_y_m"])
                               if bounce_state else None),
        "lateral_deviation_m": (bounce_state["ball_y_m"] - exit_state["ball_y_m"]
                                 if bounce_state else None),
        "velocity_immediately_before_bounce_m_s": (
            [pre_bounce_state[key] for key in ("ball_vx_m_s", "ball_vy_m_s", "ball_vz_m_s")]
            if pre_bounce_state else None),
        "energy_accounting": {
            "left_wheel_kinetic_pre_j": wheel_ke_pre_left,
            "right_wheel_kinetic_pre_j": wheel_ke_pre_right,
            "wheel_kinetic_pre_j": wheel_ke_pre,
            "left_wheel_kinetic_post_j": wheel_ke_post_left,
            "right_wheel_kinetic_post_j": wheel_ke_post_right,
            "wheel_kinetic_post_j": wheel_ke_post,
            "motor_mechanical_work_event_j": motor_work,
            "ball_initial_translational_j": ball_ke_initial,
            "ball_exit_translational_j": ball_ke,
            "ball_exit_rotational_j": ball_rot_ke,
            "potential_energy_change_j": potential_change,
            "elastic_energy_stored_j": elastic_stored,
            "elastic_energy_recovered_j": elastic_recovered,
            "elastic_hysteresis_j": elastic_hysteresis,
            "normal_damping_dissipation_j": damping_loss,
            "tangential_friction_dissipation_j": friction_loss,
            "contact_dissipation_j": contact_loss,
            "drivetrain_damping_j": drivetrain_loss,
            "residual_j": residual,
            "residual_fraction": residual / input_energy if input_energy else None,
        },
        "friction_assumption": friction,
        "wheel_mass_assumption_kg": wheel_mass,
        "wheel_inertia_assumption_kg_m2": wheel_inertia,
        "timestep_s": timestep,
        "validity_classification": ("STOPPED_CRADLE_PANEL_CLIP" if panel_clip else
                                    "STOPPED_POST_RELEASE_INTERFERENCE" if post_release_interference else
                                    "PROVISIONAL_TYRE_FRICTION_NATIVE_DYNAMICS"),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("case_dir", type=Path)
    parser.add_argument("--wheel-mass", type=float, default=0.90)
    parser.add_argument("--wheel-inertia", type=float, default=0.006751162108290868)
    parser.add_argument("--friction", type=float)
    parser.add_argument("--timestep", type=float, default=0.001)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = analyze(args.case_dir, args.wheel_mass, args.wheel_inertia,
                     args.friction, args.timestep)
    output = args.output or args.case_dir / "result.json"
    output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
