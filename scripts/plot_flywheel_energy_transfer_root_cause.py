#!/usr/bin/env python3
"""Generate the fourteen frozen flywheel energy-transfer diagnostic plots."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SUMMARY = ROOT / "docs/mechanism/flywheel-energy-transfer-case-summary.csv"
DEFAULT_TELEMETRY = ROOT / "docs/mechanism/flywheel-energy-transfer-contact-telemetry.csv"
DEFAULT_OUTPUT = ROOT / "docs/images"
TARGET_CASE = "conv_mu_3p0_w160_dt_0p25ms"
MUS = (0.3, 0.6, 0.9, 1.2, 1.5, 2.0, 2.5, 3.0)
SPEEDS = (80, 120, 160, 200, 240, 280, 300)


def read_csv(path: Path) -> list[dict]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def number(row: dict, key: str) -> float:
    return float(row[key])


def finish(fig, axis, path: Path, title: str, ylabel: str, xlabel: str) -> None:
    axis.set_title(title)
    axis.set_xlabel(xlabel)
    axis.set_ylabel(ylabel)
    axis.grid(True, alpha=0.25)
    if axis.get_legend_handles_labels()[0]:
        axis.legend(fontsize=8, ncol=2)
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def thresholds(axis) -> None:
    left, right = axis.get_xlim()
    label_x = left + 0.01 * (right - left)
    for value in (12, 14, 16, 18):
        axis.axhline(value, color="0.45", linestyle="--", linewidth=0.7)
        axis.text(label_x, value, f"{value} m/s", va="bottom", ha="left", fontsize=7,
                  color="0.35", backgroundcolor="white")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--summary", type=Path, default=DEFAULT_SUMMARY)
    parser.add_argument("--telemetry", type=Path, default=DEFAULT_TELEMETRY)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    summary = read_csv(args.summary)
    matrix = [row for row in summary if row["category"] == "friction_speed_matrix"]
    traces = [row for row in read_csv(args.telemetry)
              if row["case_id"] == TARGET_CASE and row["side"] == "left"]

    outputs = []

    fig, ax = plt.subplots(figsize=(8.4, 5.2))
    for mu in MUS:
        rows = sorted((row for row in matrix if number(row, "friction_coefficient") == mu), key=lambda row: number(row, "target_wheel_speed_rad_s"))
        ax.plot([number(row, "target_wheel_speed_rad_s") for row in rows], [number(row, "exit_speed_m_s") for row in rows], marker="o", label=f"μ={mu:g}")
    thresholds(ax)
    path = args.output_dir / "flywheel-energy-transfer-01-exit-vs-wheel-speed.png"; outputs.append(path)
    finish(fig, ax, path, "Exit speed vs wheel speed (frozen geometry)", "Exit speed (m/s)", "Command magnitude (rad/s)")

    fig, ax = plt.subplots(figsize=(8.4, 5.2))
    for speed in SPEEDS:
        rows = sorted((row for row in matrix if number(row, "target_wheel_speed_rad_s") == speed), key=lambda row: number(row, "friction_coefficient"))
        ax.plot([number(row, "friction_coefficient") for row in rows], [number(row, "exit_speed_m_s") for row in rows], marker="o", label=f"{speed} rad/s")
    thresholds(ax)
    path = args.output_dir / "flywheel-energy-transfer-02-exit-vs-friction.png"; outputs.append(path)
    finish(fig, ax, path, "Exit speed vs diagnostic friction", "Exit speed (m/s)", "Diagnostic μ (not calibrated)")

    time = [number(row, "time_from_first_contact_s") * 1000 for row in traces]
    trace_specs = [
        ("03-slip-velocity", "Slip velocity vs contact time", "Slip velocity (m/s)", [("slip_velocity_m_s", "slip")]),
        ("04-friction-utilization", "Friction utilization vs contact time", "|Ft| / (μ Fn)", [("friction_utilization", "utilization")]),
        ("05-normal-force", "Normal force vs contact time", "Normal force per wheel (N)", [("normal_force_n", "normal")]),
        ("06-tangential-force", "Tangential force vs contact time", "Tangential force per wheel (N)", [("tangential_force_n", "tangential")]),
        ("07-compression", "Per-wheel compression vs contact time", "Compression (mm)", [("compression_m", "compression")]),
        ("08-surface-speeds", "Wheel and ball surface speeds at contact", "Signed surface speed (m/s)", [("wheel_surface_speed_at_contact_m_s", "wheel"), ("ball_surface_speed_at_contact_m_s", "ball")]),
    ]
    for stem, title, ylabel, series in trace_specs:
        fig, ax = plt.subplots(figsize=(8.4, 4.8))
        for key, series_label in series:
            values = [number(row, key) * (1000 if key == "compression_m" else 1) for row in traces]
            ax.plot(time, values, label=series_label)
        if stem == "04-friction-utilization":
            ax.axhline(0.95, color="tab:red", linestyle="--", label="near-limit threshold")
        path = args.output_dir / f"flywheel-energy-transfer-{stem}.png"; outputs.append(path)
        finish(fig, ax, path, f"{title} — μ=3, ±160 rad/s, dt=0.25 ms", ylabel, "Time from first contact (ms)")

    rows300 = sorted((row for row in matrix if number(row, "target_wheel_speed_rad_s") == 300), key=lambda row: number(row, "friction_coefficient"))
    plot_specs = [
        ("09-contact-duration", "Contact duration vs diagnostic friction", "Contact duration (ms)", lambda row: number(row, "contact_duration_s") * 1000),
        ("10-tangential-impulse", "Tangential impulse vs diagnostic friction", "Tangential impulse per wheel (N·s)", lambda row: number(row, "tangential_impulse_n_s_per_wheel")),
        ("11-ball-kinetic-energy", "Ball mechanical energy gain vs diagnostic friction", "Ball mechanical energy gain (J)", lambda row: number(row, "ball_mechanical_energy_gain_j")),
        ("12-wheel-energy-transfer", "Wheel energy transferred vs diagnostic friction", "Wheel rotational energy loss (J)", lambda row: number(row, "wheel_energy_loss_j")),
        ("13-rpm-droop", "Wheel speed droop vs diagnostic friction", "Maximum wheel droop (%)", lambda row: number(row, "wheel_droop_percent")),
    ]
    for stem, title, ylabel, getter in plot_specs:
        fig, ax = plt.subplots(figsize=(7.4, 4.8))
        ax.plot([number(row, "friction_coefficient") for row in rows300], [getter(row) for row in rows300], marker="o", label="±300 rad/s")
        path = args.output_dir / f"flywheel-energy-transfer-{stem}.png"; outputs.append(path)
        finish(fig, ax, path, title, ylabel, "Diagnostic μ (not calibrated)")

    fig, ax = plt.subplots(figsize=(7.4, 5.0))
    ax.plot([number(row, "tangential_impulse_n_s_per_wheel") for row in rows300], [number(row, "exit_speed_m_s") for row in rows300], marker="o")
    for row in rows300:
        ax.annotate(f"μ={number(row, 'friction_coefficient'):g}", (number(row, "tangential_impulse_n_s_per_wheel"), number(row, "exit_speed_m_s")), xytext=(4, 3), textcoords="offset points", fontsize=7)
    thresholds(ax)
    path = args.output_dir / "flywheel-energy-transfer-14-exit-vs-tangential-impulse.png"; outputs.append(path)
    finish(fig, ax, path, "Exit speed vs tangential impulse", "Exit speed (m/s)", "Tangential impulse per wheel (N·s)")

    if len(outputs) != 14:
        raise RuntimeError(f"expected fourteen plots, generated {len(outputs)}")
    for path in outputs:
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
