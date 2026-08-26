#!/usr/bin/env python3
"""Render before/after and cutout evidence for the post-nip corridor audit."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle


def load_csv(path: Path) -> list[dict[str, float]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return [{key: float(value) for key, value in row.items()}
                for row in csv.DictReader(stream)]


def local(row: dict[str, float]) -> tuple[float, float]:
    pitch = math.radians(20.0)
    x = row["ball_x_m"]
    z = row["ball_z_m"] - 0.350
    return (math.cos(pitch) * x + math.sin(pitch) * z,
            -math.sin(pitch) * x + math.cos(pitch) * z)


def speed(row: dict[str, float]) -> float:
    return math.sqrt(sum(row[key] ** 2 for key in
                         ("ball_vx_m_s", "ball_vy_m_s", "ball_vz_m_s")))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--audit", type=Path, required=True)
    parser.add_argument("--before", type=Path, required=True)
    parser.add_argument("--after", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    audit = json.loads(args.audit.read_text(encoding="utf-8"))
    before = load_csv(args.before / "state.csv")
    after = load_csv(args.after / "state.csv")

    fig, axes = plt.subplots(2, 2, figsize=(13, 9), constrained_layout=True)
    ax = axes[0, 0]
    subset = [row for row in after if 2.035 <= row["time_s"] <= 2.095]
    path = [local(row) for row in subset]
    ax.plot([point[0] * 1000 for point in path], [point[1] * 1000 for point in path],
            label="measured free centre path")
    ax.fill_between([100, 320], -45, 45, alpha=0.12, label="CAD Ø90 nominal corridor")
    ax.add_patch(Rectangle((-128, -47), 256, 8, color="slategray", alpha=0.7,
                           label="pre-relief plates"))
    ax.add_patch(Rectangle((-128, 39), 256, 8, color="slategray", alpha=0.7))
    impact = audit["measured_path"]["first_cradle_contact_launcher_local_position_m"]
    ax.scatter([impact[0] * 1000], [impact[2] * 1000], color="tab:red", zorder=5,
               label="pre-relief plate impact")
    ax.set(xlim=(20, 330), ylim=(-55, 55), xlabel="launcher-local x (mm)",
           ylabel="launcher-local z (mm)", title="Measured path and nominal CAD corridor")
    ax.legend()
    ax.grid(alpha=0.25)

    ax = axes[0, 1]
    styles = (("zero_contact", "zero contact", "tab:red"),
              ("plus_2mm", "+2 mm", "tab:orange"),
              ("plus_5mm", "+5 mm practical", "tab:blue"))
    for key, label, color in styles:
        box = audit["cutout_study"][key]["bounding_box_xy_m"]
        # Bounding envelopes are intentionally shown; the active mesh follows
        # the narrower swept-volume curve within them.
        ax.add_patch(Rectangle((box["x"][0] * 1000, box["y"][0] * 1000),
                               (box["x"][1] - box["x"][0]) * 1000,
                               (box["y"][1] - box["y"][0]) * 1000,
                               fill=False, linewidth=2, edgecolor=color, label=label))
    ax.set(xlim=(0, 135), ylim=(-32, 32), xlabel="launcher-local x (mm)",
           ylabel="launcher-local y (mm)", title="Lower relief bounding envelopes")
    ax.legend()
    ax.grid(alpha=0.25)

    ax = axes[1, 0]
    for rows, label in ((before, "before: plate collision"), (after, "after: free flight")):
        data = [row for row in rows if 2.035 <= row["time_s"] <= 2.070]
        ax.plot([row["time_s"] for row in data], [speed(row) for row in data], label=label)
    ax.axvline(2.044, color="tab:red", linestyle=":", label="old impact onset")
    ax.set(xlabel="simulation time (s)", ylabel="ball speed (m/s)",
           title="Same low-energy case before / after relief")
    ax.legend()
    ax.grid(alpha=0.25)

    ax = axes[1, 1]
    ax.add_patch(Rectangle((-128, -157), 256, 314, fill=False, linewidth=2,
                           edgecolor="slategray", label="256 × 314 mm panel"))
    practical = audit["proposed_practical_cutout"]["lower_plate"]["bounding_box_xy_m"]
    ax.add_patch(Rectangle((practical["x"][0] * 1000, practical["y"][0] * 1000),
                           (practical["x"][1] - practical["x"][0]) * 1000,
                           (practical["y"][1] - practical["y"][0]) * 1000,
                           color="tab:blue", alpha=0.25, label="practical relief envelope"))
    for sign in (-1, 1):
        axis_y = sign * 129
        ax.add_patch(Circle((0, axis_y), 3, color="black"))
        for hx, hy in ((15, axis_y), (-15, axis_y), (0, axis_y + 15), (0, axis_y - 15)):
            ax.add_patch(Circle((hx, hy), 2, fill=False, edgecolor="tab:orange"))
    ax.set(xlim=(-140, 140), ylim=(-170, 170), aspect="equal",
           xlabel="launcher-local x (mm)", ylabel="launcher-local y (mm)",
           title="Relief remains remote from D5065 mounting pattern")
    ax.legend(loc="center left")
    ax.grid(alpha=0.2)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=180)
    plt.close(fig)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
