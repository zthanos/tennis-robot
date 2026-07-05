#!/usr/bin/env python3
"""Generate the roller-first intake-channel mesh for the sim URDF.

Roller-first geometry (replaces the old push-ramp scoop): the paddled intake
roller is the FIRST hard contact with the ball — the whole channel sits
BEHIND the roller's leading edge. The paddle envelope (Ø90, centre x=0.55)
reaches forward to x=0.595. The mesh-local tip is x=0.555; URDF/SDF applies
a -15 mm tuning offset, so the effective channel tip is x=0.540.
An approaching 66 mm ball meets the paddle envelope at centre x=0.580
(contact point 63 mm up the ball, above its centre, pulling it in and down)
well BEFORE it could touch the rear-shifted channel tip. The channel
floor then wraps in a circular arc around the roller centre — constant 60 mm
surface-to-envelope gap, i.e. ~6 mm paddle overlap on the ball — so the
paddles keep driving the ball around and up to the rear wall, where it is
flung up toward the deflector plate and basket (see funnel.urdf.xacro).

Profile, ground frame (m), x forward:
  tip           x = 0.555, z = 0.002 (behind the roller's leading edge)
  arc           z = ROLLER_Z - sqrt(CHANNEL_R^2 - (ROLLER_X - x)^2)
                from x = 0.550 (floor) back to x = 0.445 (z = 0.105)
  rear wall     near-vertical up to z = 0.155 at x = 0.4435

Roller (see tennis_robot.urdf.xacro): centre (0.550, 0.105), paddle envelope
radius 0.045; channel radius 0.105 about the same centre.

The mesh is emitted in the funnel_link frame (origin = base_link x/y, shifted
down by chassis_z/2 = 7 mm; base_link is 45 mm above ground, so the ground
plane is z = -0.038 here) as one closed solid: flat underside on the ground
plane, channel surface on top. Placed in funnel.urdf.xacro with origin 0 0 0.
Keep in sync with tennis_robot.urdf.xacro intake_* properties.
"""

from __future__ import annotations

import math
import struct
import sys
from pathlib import Path

# Geometry (m, ground frame).
LIP_X = 0.555
LIP_Z = 0.002
FLOOR_Z = 0.003          # keeps the sheet underside clear of the court
ROLLER_X = 0.550
ROLLER_Z = 0.105
CHANNEL_R = 0.105        # channel surface radius about the roller centre
WALL_END_X = 0.4435      # rear wall top x
WALL_TOP_Z = 0.155       # rear wall top height
SCOOP_WIDTH = 0.180
SHEET_THICKNESS = 0.002
COLLISION_CLEARANCE = 0.001

LEAD_STEPS = 6
ARC_STEPS = 40
WALL_STEPS = 4

# URDF frame constant (m).
GROUND_Z = -0.038        # ground plane in funnel_link frame
HALF_WIDTH = SCOOP_WIDTH / 2.0


def channel_z(x: float) -> float:
    """Channel-surface height (ground frame) for arc x in [0.445, 0.550]."""
    dx = ROLLER_X - x
    return max(FLOOR_Z, ROLLER_Z - math.sqrt(max(CHANNEL_R * CHANNEL_R - dx * dx, 0.0)))


def profile() -> list[tuple[float, float, float]]:
    """(x, z_bottom, z_top) in funnel frame, x strictly decreasing."""
    pts: list[tuple[float, float]] = []
    # Lip / flat throat lead-in.
    for i in range(LEAD_STEPS):
        t = i / LEAD_STEPS
        x = LIP_X + (ROLLER_X - LIP_X) * t
        pts.append((x, LIP_Z + (FLOOR_Z - LIP_Z) * t))
    # Arc around the roller centre, floor -> rear.
    arc_back_x = ROLLER_X - CHANNEL_R
    for i in range(ARC_STEPS + 1):
        t = i / ARC_STEPS
        x = ROLLER_X + (arc_back_x - ROLLER_X) * t
        pts.append((x, channel_z(x)))
    # Near-vertical rear wall.
    wall_base_z = ROLLER_Z
    for i in range(1, WALL_STEPS + 1):
        t = i / WALL_STEPS
        x = arc_back_x + (WALL_END_X - arc_back_x) * t
        pts.append((x, wall_base_z + (WALL_TOP_Z - wall_base_z) * t))
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
