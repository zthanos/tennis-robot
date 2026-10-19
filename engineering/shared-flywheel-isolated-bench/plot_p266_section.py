#!/usr/bin/env python3
"""Render the requested P2-66 Y=0 section from nominal Gazebo traces."""
from __future__ import annotations

import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon, Rectangle

ROOT = Path("engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_p266/matrix")
OUT = Path("cad/modular-test-chassis/p266-ballistic-side.png")


def ski(radius: int, angle: int) -> tuple[list[float], list[float]]:
    points = [(435, 10), (383, 10)]
    for degrees in range(5, angle + 1, 5):
        theta = math.radians(degrees)
        points.append((383-radius*math.sin(theta), 10+radius*(1-math.cos(theta))))
    return [p[0] for p in points], [p[1] for p in points]


fig, ax = plt.subplots(figsize=(12.8, 8.0), constrained_layout=True)
colors = {(70,75): "#0072B2", (75,75): "#56B4E9", (80,75): "#009E73",
          (70,110): "#E69F00", (75,110): "#D55E00", (80,110): "#CC79A7"}
for angle in (70, 75, 80):
    for radius in (75, 110):
        path = ROOT / f"a_{angle}/r_{radius}/v_6/tm_0p6/y_0/e_0p3/telemetry.csv"
        if not path.exists():
            continue
        rows = list(csv.DictReader(path.open()))
        trace = [(1000*float(r["ball_base_x_m"]), 1000*float(r["ball_base_z_m"]))
                 for r in rows if 2.40 <= float(r["time_s"]) <= 2.72]
        if trace:
            x, z = zip(*trace)
            ax.plot(x, z, color=colors[(angle,radius)], lw=2,
                    label=f"R{radius} / {angle}°")
            # Show the tennis-ball envelope at the highest sampled point.
            i = max(range(len(z)), key=z.__getitem__)
            ax.add_patch(Circle((x[i], z[i]), 33, fill=False,
                                color=colors[(angle,radius)], alpha=.55, lw=1))
        sx, sz = ski(radius, angle)
        ax.plot(sx, sz, color=colors[(angle,radius)], lw=.8, alpha=.35)

# P2-66 basket/front/deflector.
ax.add_patch(Rectangle((276, 42), 4, 198, color="#2E7D32", alpha=.65))
ax.add_patch(Rectangle((276, 42), 4, 80, color="#F57C00", alpha=.95))
ax.plot([274, 82], [334, 370], color="#AD1457", lw=8,
        solid_capstyle="butt", label="soft deflector (10 mm)")
ax.plot([-85, 278], [70, 42], color="#4E7D52", lw=3)
ax.plot([-85, -85], [70, 375], color="#4E7D52", lw=3)

# Frozen full-v3 keep-outs at centre section.
ax.add_patch(Rectangle((305, 230), 50, 30, color="#616161", alpha=.78))
ax.text(330, 263, "arch X330", ha="center", va="bottom", fontsize=9)
ax.add_patch(Rectangle((292, 358), 76, 32, color="#5C6BC0", alpha=.55))
ax.text(330, 392, "LiDAR Z390", ha="center", va="bottom", fontsize=9)
camera = Polygon([(323,282),(386,335),(377,346),(314,293)], closed=True,
                 color="#8E24AA", alpha=.55)
ax.add_patch(camera)
ax.text(385, 338, "camera", fontsize=9)

ax.axhline(380, color="#C62828", ls="--", lw=1, label="ball top limit Z380")
ax.annotate("closed feed door\nX278 / Z42…122", xy=(278,82), xytext=(345,70),
            arrowprops={"arrowstyle":"->"}, fontsize=9)
ax.annotate("hard collision zone", xy=(320,235), xytext=(375,245),
            arrowprops={"arrowstyle":"->", "color":"#C62828"}, color="#C62828")
ax.set(xlabel="X [mm]  (front →)", ylabel="Z [mm]  (+Z up)",
       title="P2-66 — Y=0 section, nominal v=6 m/s / μ=0.6 / e=0.3")
ax.set_xlim(-120, 470); ax.set_ylim(0, 420); ax.set_aspect("equal", adjustable="box")
ax.grid(True, alpha=.2); ax.legend(loc="upper left", ncol=2, fontsize=8)
fig.savefig(OUT, dpi=170)
print(OUT)
