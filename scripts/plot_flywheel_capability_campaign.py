#!/usr/bin/env python3
"""Generate final measured plots and stage frames for the flywheel campaign."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle


def csv_rows(path: Path) -> list[dict[str, float]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return [{key: float(value) for key, value in row.items()}
                for row in csv.DictReader(stream)]


def local_position(row: dict[str, float]) -> tuple[float, float, float]:
    pitch = math.radians(20.0)
    x, y, z = row["ball_x_m"], row["ball_y_m"], row["ball_z_m"] - 0.350
    return (math.cos(pitch) * x + math.sin(pitch) * z,
            y,
            -math.sin(pitch) * x + math.cos(pitch) * z)


def nearest(rows: list[dict[str, float]], time_s: float) -> dict[str, float]:
    return min(rows, key=lambda row: abs(row["time_s"] - time_s))


def plot_map(data: dict, output: Path) -> None:
    symmetric = data["symmetric_operating_points"]
    differential = data["differential_operating_points"]
    fig, axes = plt.subplots(3, 3, figsize=(15, 12), constrained_layout=True)
    colors = {0.3: "tab:blue", 0.6: "tab:orange", 0.9: "tab:green"}
    for mu, color in colors.items():
        points = [row for row in symmetric if row["tyre_friction_assumption"]["coefficient"] == mu]
        x = [0.5 * (abs(row["left_actual_precontact_rad_s"]) +
                    abs(row["right_actual_precontact_rad_s"])) for row in points]
        axes[0, 0].plot(x, [row["exit_speed_m_s"] for row in points], "o-", color=color, label=f"μ={mu}")
        axes[0, 1].plot(x, [row["elevation_deg"] for row in points], "o-", color=color, label=f"μ={mu}")
        axes[1, 2].plot([row["exit_speed_m_s"] for row in points],
                        [row["horizontal_range_m"] for row in points], "o-", color=color, label=f"μ={mu}")
        axes[2, 0].scatter([row["first_bounce_xyz_m"][0] for row in points],
                           [row["first_bounce_xyz_m"][1] for row in points], color=color, label=f"μ={mu}")
    for target in (12, 14, 16, 18):
        axes[0, 0].axhline(target, color="0.75", linestyle="--", linewidth=0.8)
    axes[0, 0].set(title="Exit-speed envelope", xlabel="actual wheel magnitude (rad/s)", ylabel="exit speed (m/s)")
    axes[0, 1].set(title="Measured launch elevation", xlabel="actual wheel magnitude (rad/s)", ylabel="elevation (deg)")

    for mu, color in colors.items():
        points = [row for row in symmetric if row["tyre_friction_assumption"]["coefficient"] == mu]
        axes[0, 2].scatter([row["energy_accounting"]["ball_exit_translational_j"] for row in points],
                           [0.5 * (row["left_rpm_droop"] + row["right_rpm_droop"]) for row in points],
                           color=color, label=f"μ={mu}")
        axes[1, 0].plot([0.5 * (abs(row["left_actual_precontact_rad_s"]) + abs(row["right_actual_precontact_rad_s"])) for row in points],
                        [row["recovery_time_s"] for row in points], "o-", color=color, label=f"μ={mu}")
    axes[0, 2].set(title="Droop vs ball launch energy", xlabel="ball translational energy (J)", ylabel="mean RPM droop")
    axes[1, 0].set(title="Wheel recovery", xlabel="actual wheel magnitude (rad/s)", ylabel="recovery time (s)")

    dx = [abs(row["left_actual_precontact_rad_s"]) - abs(row["right_actual_precontact_rad_s"])
          for row in differential]
    axes[1, 1].plot(dx, [row["topspin_rad_s"] for row in differential], "o-", label="top/backspin")
    axes[1, 1].plot(dx, [row["sidespin_rad_s"] for row in differential], "s-", label="sidespin")
    axes[1, 1].set(title="Differential-speed spin response", xlabel="left minus right magnitude (rad/s)", ylabel="spin component (rad/s)")
    axes[1, 1].legend()
    axes[1, 2].set(title="Gravity-only first-bounce range", xlabel="exit speed (m/s)", ylabel="horizontal range (m)")
    axes[2, 0].set(title="Gravity-only first-bounce map", xlabel="world X (m)", ylabel="world Y (m)")

    convergence = data["timestep_convergence"]["conditions"]
    for level, entry in convergence.items():
        comparisons = entry["comparisons"]
        axes[2, 1].plot([item["timestep_s"] * 1000 for item in comparisons],
                        [100 * item["metrics"]["exit_speed_relative_difference"] for item in comparisons],
                        "o-", label=level)
    axes[2, 1].set(title="Exit-speed timestep convergence", xlabel="timestep (ms)", ylabel="difference from 0.25 ms (%)")
    axes[2, 1].legend()
    axes[2, 2].scatter(
        [row["exit_speed_m_s"] for row in symmetric],
        [100 * row["energy_accounting"]["residual_fraction"] for row in symmetric],
        c=[colors[row["tyre_friction_assumption"]["coefficient"]] for row in symmetric])
    axes[2, 2].axhline(2, color="tab:red", linestyle="--", linewidth=0.8, label="frozen ±2% bound")
    axes[2, 2].axhline(-2, color="tab:red", linestyle="--", linewidth=0.8)
    axes[2, 2].set(title="Energy-ledger residual", xlabel="exit speed (m/s)", ylabel="residual (%)")
    axes[2, 2].legend()
    for ax in axes.flat:
        ax.grid(alpha=0.25)
    axes[0, 0].legend()
    axes[0, 1].legend()
    axes[1, 2].legend()
    axes[2, 0].legend()
    fig.suptitle("Standalone flywheel capability campaign — friction values are sensitivity assumptions", fontsize=15)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=180)
    plt.close(fig)


def plot_traces(campaign: Path, output: Path) -> None:
    cases = (("low", "sym_mu_0p6_w080"), ("medium", "sym_mu_0p6_w200"),
             ("high", "sym_mu_0p6_w300"))
    fig, axes = plt.subplots(2, 2, figsize=(13, 9), constrained_layout=True)
    for label, case_id in cases:
        states = csv_rows(campaign / case_id / "state.csv")
        contacts = [row for row in csv_rows(campaign / case_id / "contacts.csv") if row["contact_id"] < 1.5]
        start, end = min(row["time_s"] for row in contacts), max(row["time_s"] for row in contacts)
        state_window = [row for row in states if start - 0.05 <= row["time_s"] <= end + 0.7]
        axes[0, 0].plot([row["time_s"] - start for row in state_window],
                        [abs(row["left_speed_rad_s"]) * 60 / (2 * math.pi) for row in state_window], label=label)
        axes[0, 1].plot([row["time_s"] - start for row in state_window],
                        [math.sqrt(row["ball_vx_m_s"]**2 + row["ball_vy_m_s"]**2 + row["ball_vz_m_s"]**2) for row in state_window], label=label)
        left = [row for row in contacts if row["contact_id"] < 0.5]
        axes[1, 0].plot([row["time_s"] - start for row in left],
                        [row["normal_force_n"] for row in left], "o-", markersize=2, label=label)
        axes[1, 1].plot([row["time_s"] - start for row in left],
                        [1000 * row["compression_m"] for row in left], "o-", markersize=2, label=label)
    axes[0, 0].set(title="Left wheel RPM through contact and recovery", xlabel="time from first contact (s)", ylabel="RPM")
    axes[0, 1].set(title="Ball speed through launch", xlabel="time from first contact (s)", ylabel="speed (m/s)")
    axes[1, 0].set(title="Left normal force", xlabel="time from first contact (s)", ylabel="force (N)")
    axes[1, 1].set(title="Ball-law compression", xlabel="time from first contact (s)", ylabel="compression (mm)")
    for ax in axes.flat:
        ax.grid(alpha=0.25)
        ax.legend()
    fig.suptitle("Representative μ=0.6 launch traces", fontsize=15)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=180)
    plt.close(fig)


def plot_stages(campaign: Path, output: Path) -> None:
    cases = (("Low: 80 rad/s", "sym_mu_0p6_w080"),
             ("Medium: 200 rad/s", "sym_mu_0p6_w200"),
             ("High: 300 rad/s", "sym_mu_0p6_w300"))
    fig, axes = plt.subplots(3, 6, figsize=(18, 8), constrained_layout=True)
    for row_index, (condition, case_id) in enumerate(cases):
        states = csv_rows(campaign / case_id / "state.csv")
        contacts = [row for row in csv_rows(campaign / case_id / "contacts.csv") if row["contact_id"] < 1.5]
        first = min(row["time_s"] for row in contacts)
        maximum = max(contacts, key=lambda row: row["compression_m"])["time_s"]
        last = max(row["time_s"] for row in contacts)
        after = [row for row in states if row["time_s"] > last]
        clear = min(after, key=lambda row: abs(local_position(row)[0] - 0.145))
        free = min(after, key=lambda row: abs(local_position(row)[0] - 0.35))
        stages = ((first - 0.005, "Pre-contact"), (first, "Bilateral contact"),
                  (maximum, "Max compression"), (last + 0.002, "Release"),
                  (clear["time_s"], "Clear corridor"), (free["time_s"], "Free trajectory"))
        for column, (time_s, title) in enumerate(stages):
            state = nearest(states, time_s)
            x, _, z = local_position(state)
            ax = axes[row_index, column]
            # The corrected relief removes the inner plate edges downstream of x=60 mm.
            ax.add_patch(Rectangle((-0.128, -0.047), 0.188, 0.008, color="slategray", alpha=0.7))
            ax.add_patch(Rectangle((-0.128, 0.039), 0.188, 0.008, color="slategray", alpha=0.7))
            ax.plot([0.060, 0.190], [-0.039, -0.039], color="tab:green", linestyle="--", linewidth=1)
            ax.plot([0.060, 0.190], [0.039, 0.039], color="tab:green", linestyle="--", linewidth=1)
            ax.add_patch(Circle((x, z), 0.033, color="yellowgreen", alpha=0.85))
            ax.plot([x], [z], ".", color="black")
            ax.set(xlim=(-0.105, 0.40), ylim=(-0.085, 0.13), aspect="equal", title=title if row_index == 0 else None)
            ax.grid(alpha=0.2)
            ax.text(0.02, 0.95, f"{condition}\nt={state['time_s']:.3f}s\nx={x:.3f}m, z={z:.3f}m",
                    transform=ax.transAxes, va="top", fontsize=8)
            if column == 0:
                ax.set_ylabel("launcher-local z (m)")
            if row_index == 2:
                ax.set_xlabel("launcher-local x (m)")
    fig.suptitle("Measured low / medium / high launch stages through corrected exit relief", fontsize=15)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=180)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("campaign_dir", type=Path)
    parser.add_argument("capability_map", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    data = json.loads(args.capability_map.read_text(encoding="utf-8"))
    plot_map(data, args.output_dir / "flywheel-capability-campaign-map.png")
    plot_traces(args.campaign_dir, args.output_dir / "flywheel-capability-representative-traces.png")
    plot_stages(args.campaign_dir, args.output_dir / "flywheel-capability-representative-stages.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
