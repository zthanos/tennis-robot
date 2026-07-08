#!/usr/bin/env python3
"""Generate the roller-first intake-channel mesh for the sim URDF.

Roller-first geometry (replaces the old push-ramp scoop): the continuous intake
roller is the FIRST hard contact with the ball — the whole channel sits
BEHIND the roller's leading edge. The roller (Ø90, centre x=0.600) reaches
forward to x=0.645. The mesh-local channel centre is x=0.615 because the
URDF/SDF applies a -15 mm channel offset, giving an effective centre x=0.600.
The effective roller / channel centre is 112 mm above ground. The entry lip is
15 mm behind the roller axis so the ball meets the roller before it can meet
the channel. The channel radius is 108 mm: 63 mm clearance from the 45 mm roller radius,
i.e. 3 mm nominal interference for a rigid 66 mm simulation ball.

Profile, ground frame (m), x forward:
  tip / arc     x = 0.600, z ~= 0.005 (effective x=0.585)
  arc           z = ROLLER_Z - sqrt(CHANNEL_R^2 - (ROLLER_X - x)^2)
                from x = 0.600 back to x = 0.507 (z = 0.112)
  rear wall     near-vertical up to z = 0.155 at x = 0.5055

Roller (see tennis_robot.urdf.xacro): effective centre (0.600, 0.112), radius
0.045; channel radius 0.109 about the same effective centre.

The mesh is emitted in the funnel_link frame (origin = base_link x/y, shifted
down by chassis_z/2 = 7 mm; base_link is 45 mm above ground, so the ground
plane is z = -0.038 here) as one closed solid: flat underside on the ground
plane, channel surface on top. Placed in funnel.urdf.xacro with origin 0 0 0.
Keep in sync with tennis_robot.urdf.xacro intake_* properties.
"""

from __future__ import annotations

import math
import os
import struct
import sys
from pathlib import Path

# Geometry (m, ground frame).
# ROLLER_X/ROLLER_Z must track tennis_robot.urdf.xacro's intake_x/intake_z
# (same INTAKE_ROLLER_*_OFFSET_M envs) so the channel arc stays concentric
# with the ACTUAL roller position instead of the old fixed baseline — the
# offsets were previously only applied to the roller itself, silently
# desyncing the channel whenever intake tuning moved the roller.
_ROLLER_X_OFFSET_M = float(os.getenv("INTAKE_ROLLER_X_OFFSET_M", "0.0"))
_ROLLER_Z_OFFSET_M = float(os.getenv("INTAKE_ROLLER_Z_OFFSET_M", "0.0"))
LIP_X = 0.600 + _ROLLER_X_OFFSET_M
FLOOR_Z = 0.003          # keeps the sheet underside clear of the court
LIP_RAISE_M = max(0.0, float(os.getenv("INTAKE_LIP_RAISE_M", "0.0")))
LIP_RAISE_TAPER_M = 0.020
ROLLER_X = 0.615 + _ROLLER_X_OFFSET_M
ROLLER_Z = 0.112 + _ROLLER_Z_OFFSET_M
# 64 mm roller-to-arc passage: 2 mm nominal squeeze on a 66 mm ball, absorbed
# by the sprung roller carriage (grip force, not a rigid jam). 113 mm opened a
# dead pocket behind the axis where the ball lost roller contact
# (intake-debug-log #9). Keep in sync with generate_robot_urdf.py.
CHANNEL_R = 0.109
# WALL_TOP_Z now sets the concentric guide's release height (see profile()).
WALL_TOP_Z = 0.155       # rear wall top height
SCOOP_WIDTH = 0.180
SHEET_THICKNESS = 0.002
COLLISION_CLEARANCE = 0.001

ARC_STEPS = 40
GUIDE_STEPS = 10

# URDF frame constant (m).
GROUND_Z = -0.038        # ground plane in funnel_link frame
HALF_WIDTH = SCOOP_WIDTH / 2.0


def channel_z(x: float) -> float:
    """Channel-surface height for the arc ending at the mesh-local roller centre."""
    dx = ROLLER_X - x
    z = max(FLOOR_Z, ROLLER_Z - math.sqrt(max(CHANNEL_R * CHANNEL_R - dx * dx, 0.0)))
    if LIP_RAISE_M <= 0.0:
        return z
    lip_t = max(0.0, min(1.0, 1.0 - abs(LIP_X - x) / LIP_RAISE_TAPER_M))
    return z + LIP_RAISE_M * lip_t


def profile() -> list[tuple[float, float, float]]:
    """(x, z_bottom, z_top) in funnel frame, ordered lip -> back -> guide top."""
    pts: list[tuple[float, float]] = []
    # Arc around the roller centre. It begins behind the roller axis so there
    # is no hard channel edge in front of the roller's first ball contact.
    arc_back_x = ROLLER_X - CHANNEL_R
    for i in range(ARC_STEPS + 1):
        t = i / ARC_STEPS
        x = LIP_X + (arc_back_x - LIP_X) * t
        pts.append((x, channel_z(x)))
    # Concentric guide continuing up the BACK of the roller (replaces the old
    # near-vertical wall): the roller drives the ball TANGENTIALLY along the
    # whole guide, so there is no unpowered climb — the vertical wall left a
    # ~58 mm dead gap between roller reach (z~130) and the wall top that no
    # amount of friction could cross (intake-debug-log #12).
    sin_max = max(0.0, min(1.0, (WALL_TOP_Z - ROLLER_Z) / CHANNEL_R))
    phi_max = math.asin(sin_max)
    for i in range(1, GUIDE_STEPS + 1):
        phi = phi_max * i / GUIDE_STEPS
        pts.append((
            ROLLER_X - CHANNEL_R * math.cos(phi),
            ROLLER_Z + CHANNEL_R * math.sin(phi),
        ))
    # Thin curved sheet, not a solid wedge down to the court. The old flat
    # underside put the whole 180 mm-wide scoop in ground contact and acted as
    # a brake. Only the 2 mm front lip reaches the ground; everywhere else the
    # underside follows the channel surface.
    return [
        (
            x,
            GROUND_Z + max(COLLISION_CLEARANCE, z - SHEET_THICKNESS),
            GROUND_Z + z,
        )
        for x, z in pts
    ]


def build_triangles() -> list[tuple]:
    prof = profile()
    tris = []

    def quad(a, b, c, d):
        tris.append((a, b, c))
        tris.append((a, c, d))

    yl, yr = -HALF_WIDTH, HALF_WIDTH
    # top (channel) + bottom surfaces
    for (x0, zb0, zt0), (x1, zb1, zt1) in zip(prof, prof[1:]):
        quad((x0, yl, zt0), (x0, yr, zt0), (x1, yr, zt1), (x1, yl, zt1))  # top
        quad((x0, yr, zb0), (x0, yl, zb0), (x1, yl, zb1), (x1, yr, zb1))  # bottom
    # side walls
    for y, flip in ((yl, False), (yr, True)):
        for (x0, zb0, zt0), (x1, zb1, zt1) in zip(prof, prof[1:]):
            a, b, c, d = (x0, y, zb0), (x1, y, zb1), (x1, y, zt1), (x0, y, zt0)
            quad(*( (a, b, c, d) if flip else (d, c, b, a) ))
    # front (lip) face and rear face
    x0, zb0, zt0 = prof[0]
    quad((x0, yl, zb0), (x0, yl, zt0), (x0, yr, zt0), (x0, yr, zb0))
    x1, zb1, zt1 = prof[-1]
    quad((x1, yr, zb1), (x1, yr, zt1), (x1, yl, zt1), (x1, yl, zb1))
    return tris


def write_binary_stl(path: Path, tris: list[tuple]) -> None:
    with open(path, "wb") as f:
        f.write(b"roller-first intake channel (generate_curved_scoop_mesh.py)".ljust(80, b"\0"))
        f.write(struct.pack("<I", len(tris)))
        for a, b, c in tris:
            ux = (b[0]-a[0], b[1]-a[1], b[2]-a[2])
            vx = (c[0]-a[0], c[1]-a[1], c[2]-a[2])
            n = (ux[1]*vx[2]-ux[2]*vx[1], ux[2]*vx[0]-ux[0]*vx[2], ux[0]*vx[1]-ux[1]*vx[0])
            ln = max((n[0]**2+n[1]**2+n[2]**2) ** 0.5, 1e-12)
            f.write(struct.pack("<3f", *(v/ln for v in n)))
            for p in (a, b, c):
                f.write(struct.pack("<3f", *p))
            f.write(struct.pack("<H", 0))


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    if len(sys.argv) > 1:
        outs = [Path(sys.argv[1])]
    else:
        # Both copies are needed: the ROS package share (package:// for RViz /
        # robot_description) and the Gazebo model dir (model:// in the SDF via
        # GZ_SIM_RESOURCE_PATH).
        outs = [
            root / "ros2_ws/src/tennis_robot/meshes/curved_scoop.stl",
            root / "gazebo/models/tennis_robot/meshes/curved_scoop.stl",
        ]
    tris = build_triangles()
    for out in outs:
        out.parent.mkdir(parents=True, exist_ok=True)
        write_binary_stl(out, tris)
        print(f"wrote {out} ({len(tris)} triangles)")


if __name__ == "__main__":
    main()
