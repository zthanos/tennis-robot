#!/usr/bin/env python3
"""Plot measured evidence for the stopped low-energy flywheel capability gate."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle


def load(path: Path) -> list[dict[str, float]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return [{key: float(value) for key, value in row.items()}
                for row in csv.DictReader(stream)]


def local_position(row: dict[str, float]) -> tuple[float, float, float]:
    pitch = math.radians(20.0)
    x = row["ball_x_m"]
    y = row["ball_y_m"]
    z = row["ball_z_m"] - 0.350
    return (math.cos(pitch) * x + math.sin(pitch) * z,
            y,
            -math.sin(pitch) * x + math.cos(pitch) * z)


def nearest(states: list[dict[str, float]], time_s: float) -> dict[str, float]:
    return min(states, key=lambda row: abs(row["time_s"] - time_s))


def plot_traces(states: list[dict[str, float]], contacts: list[dict[str, float]], output: Path) -> None:
    wheel = [row for row in contacts if row["contact_id"] < 1.5]
    start = min(row["time_s"] for row in wheel)
    end = max(row["time_s"] for row in wheel)
    clip = 2.044
    times = [row["time_s"] for row in states]
    ball_speed = [math.sqrt(row["ball_vx_m_s"] ** 2 + row["ball_vy_m_s"] ** 2 +
                            row["ball_vz_m_s"] ** 2) for row in states]
    local_z = [local_position(row)[2] for row in states]

    fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    ax = axes[0, 0]
    ax.plot(times, [row["left_speed_rad_s"] for row in states], label="left")
    ax.plot(times, [-row["right_speed_rad_s"] for row in states], label="right magnitude")
    ax.axvspan(start, end, alpha=0.18, label="wheel contact")
    ax.set(xlabel="simulation time (s)", ylabel="wheel speed (rad/s)", title="Effort-limited wheel state")
    ax.legend()
    ax.grid(alpha=0.25)

    ax = axes[0, 1]
    ax.plot(times, ball_speed, label="ball speed")
    ax.axvline(start, linestyle="--", label="first bilateral contact")
    ax.axvline(clip, color="tab:red", linestyle=":", label="lower-plate clip")
    ax.set_xlim(start - 0.015, min(times[-1], clip + 0.04))
    ax.set(xlabel="simulation time (s)", ylabel="speed (m/s)", title="Ball speed through release and clip")
    ax.legend()
    ax.grid(alpha=0.25)

    ax = axes[1, 0]
    for contact_id, label in ((0.0, "left"), (1.0, "right")):
        data = [row for row in wheel if row["contact_id"] == contact_id]
        ax.plot([row["time_s"] for row in data], [row["normal_force_n"] for row in data], label=label)
    ax.set(xlabel="simulation time (s)", ylabel="normal force (N)", title="Analytical compliant tyre contact")
    ax.legend()
    ax.grid(alpha=0.25)

    ax = axes[1, 1]
    contact_states = [row for row in states if start - 0.005 <= row["time_s"] <= clip + 0.025]
    ax.plot([row["time_s"] for row in contact_states],
            [1000.0 * local_position(row)[2] for row in contact_states], label="ball centre local z")
    ax.axhline(-6.0, color="tab:red", linestyle="--", label="lower plate clearance limit")
    ax.axhline(6.0, color="tab:orange", linestyle="--", label="upper plate clearance limit")
    ax.axvline(clip, color="tab:red", linestyle=":")
    ax.set(xlabel="simulation time (s)", ylabel="launcher-local z (mm)",
           title="Post-release cradle clearance violation")
    ax.legend()
    ax.grid(alpha=0.25)

    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=180)
    plt.close(fig)


def plot_stages(states: list[dict[str, float]], contacts: list[dict[str, float]], output: Path) -> None:
    wheel = [row for row in contacts if row["contact_id"] < 1.5]
    first = min(row["time_s"] for row in wheel)
    maximum = max(wheel, key=lambda row: row["compression_m"])["time_s"]
    last = max(row["time_s"] for row in wheel)
    stages = [
        (first - 0.005, "Pre-contact"),
        (first, "Bilateral contact"),
        (maximum, "Maximum compression"),
        (last + 0.002, "Wheel release"),
        (2.045, "Trajectory stopped: plate clip"),
    ]
    fig, axes = plt.subplots(2, 5, figsize=(16, 6.5), constrained_layout=True)
    for column, (time_s, title) in enumerate(stages):
        row = nearest(states, time_s)
        x, y, z = local_position(row)
        top = axes[0, column]
        for wheel_y in (-0.129, 0.129):
            top.add_patch(Circle((0, wheel_y), 0.100, fill=False, linewidth=2, color="tab:blue"))
        top.add_patch(Circle((x, y), 0.033, color="yellowgreen", alpha=0.8))
        top.plot([x], [y], marker=".", color="black")
        top.set(xlim=(-0.105, 0.105), ylim=(-0.25, 0.25), aspect="equal", title=title,
                xlabel="local x (m)")
        if column == 0:
            top.set_ylabel("local y (m)")
        else:
            top.set_yticklabels([])
        top.grid(alpha=0.2)

        side = axes[1, column]
        side.add_patch(Rectangle((-0.128, -0.047), 0.256, 0.008, color="slategray", alpha=0.65))
        side.add_patch(Rectangle((-0.128, 0.039), 0.256, 0.008, color="slategray", alpha=0.65))
        side.add_patch(Circle((x, z), 0.033, color=("tab:red" if column == 4 else "yellowgreen"), alpha=0.8))
        side.axhline(0, color="black", linewidth=0.7, alpha=0.4)
        side.set(xlim=(-0.105, 0.13), ylim=(-0.07, 0.07), aspect="equal", xlabel="local x (m)")
        if column == 0:
            side.set_ylabel("local z (m)")
        else:
            side.set_yticklabels([])
        side.text(0.02, 0.95, f"t={row['time_s']:.3f} s\nz={z*1000:.2f} mm",
                  transform=side.transAxes, va="top", fontsize=9)
        side.grid(alpha=0.2)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=180)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("case_dir", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    states = load(args.case_dir / "state.csv")
    contacts = load(args.case_dir / "contacts.csv")
    plot_traces(states, contacts, args.output_dir / "flywheel-capability-low-energy-traces.png")
    plot_stages(states, contacts, args.output_dir / "flywheel-capability-measured-stages.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
