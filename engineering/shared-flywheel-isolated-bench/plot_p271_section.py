#!/usr/bin/env python3
"""P2-71 side section using the retained P2-69 trajectories."""
from __future__ import annotations

import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

ROOT = Path("engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_p269/matrix/a_75/r_75")
OUT = Path("cad/modular-test-chassis/p271-basket-grid-side.png")


def floor_z(x: float) -> float:
    return 70 + (x + 85) * (42 - 70) / (278 + 85)


fig, ax = plt.subplots(figsize=(13.2, 8.0), constrained_layout=True)
paths = sorted(ROOT.glob("v_*/tm_*/y_*/e_0p3/telemetry.csv"))
for index, path in enumerate(paths):
    rows = list(csv.DictReader(path.open()))
    trace = [(1000 * float(row["ball_base_x_m"]),
              1000 * float(row["ball_base_z_m"]))
             for row in rows if 2.45 <= float(row["time_s"]) <= 3.25]
    x, z = zip(*trace)
    ax.plot(x, z, color="#0072B2", lw=.75, alpha=.25,
            label="30 retained P2-69 trajectories" if index == 0 else None)

# Accepted R75/75 ski and overhead deflector.
ski = [(435, 10), (383, 10)]
for degrees in range(5, 76, 5):
    theta = math.radians(degrees)
    ski.append((383 - 75 * math.sin(theta), 10 + 75 * (1 - math.cos(theta))))
ax.plot(*zip(*ski), color="#00A0C6", lw=2.6, label="R75 / 75° ski")
ax.plot([250, 360], [363, 343], color="#AD1457", lw=9,
        solid_capstyle="butt", label="soft deflector")

# P2-71 basket geometry.
ax.plot([-118, 278], [floor_z(-118), 42], color="#4E7D52", lw=3,
        label="extended basket floor")
ax.plot([-118, -118], [floor_z(-118), 320], color="#2E7D32", lw=5,
        ls=(0, (3, 2)), label="rear upright grid X−118 / Z320")
ax.plot([225, 225], [46.1, 235], color="#43A047", lw=5,
        ls=(0, (3, 2)), label="front grid X225 / Z235")
ax.add_patch(Rectangle((222.5, 44.1), 5, 84, color="#F57C00", alpha=.96,
                       label="solid fail-closed door 84×84"))

# Accepted P2-70 pod, shown as section envelope.
ax.add_patch(Rectangle((-272, 55), 147, 295, facecolor="#263238",
                       edgecolor="#FF8F00", lw=1.8, alpha=.22,
                       label="accepted rear pod envelope"))
ax.annotate("5.0 mm", xy=(-120, 290), xytext=(-182, 290), fontsize=9,
            arrowprops={"arrowstyle":"<->", "color":"#EF6C00"}, color="#EF6C00")

ax.annotate("min flight surface clearance\nrear grid 106.95 mm",
            xy=(-118, 145), xytext=(-20, 292), fontsize=9, color="#2E7D32",
            arrowprops={"arrowstyle":"->", "color":"#2E7D32"})
ax.annotate("min flight surface clearance\nfront envelope 25.00 mm",
            xy=(225, 235), xytext=(65, 250), fontsize=9, color="#2E7D32",
            arrowprops={"arrowstyle":"->", "color":"#2E7D32"})

ax.axhline(390, color="#5C6BC0", ls=":", lw=1)
ax.text(-268, 394, "LiDAR plane Z390", fontsize=9, color="#5C6BC0")
ax.set(xlabel="X [mm]  (front →)", ylabel="Z [mm]  (+Z up)",
       title="P2-71 — simplified basket grids, geometry-only audit")
ax.set_xlim(-300, 480)
ax.set_ylim(0, 420)
ax.set_aspect("equal", adjustable="box")
ax.grid(True, alpha=.2)
ax.legend(loc="upper left", fontsize=8, ncol=2)
fig.savefig(OUT, dpi=170)
print(OUT)
