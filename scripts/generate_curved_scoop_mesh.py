#!/usr/bin/env python3
"""Generate the intake elevation-ramp mesh for the sim URDF.

Dual-wheel intake (docs/dual-wheel-intake-design-el.md): the side wheels
capture and transport the ball rearward through the nip; this ramp receives
it just behind the throat and elevates it toward the basket. The entry sits
slightly ahead of the wheel nip so the ball is fed onto a rising surface
while still driven by the wheels.

The mesh is emitted in the funnel_link frame (origin = base_link x/y, shifted
down by chassis_z/2 = 7 mm; base_link is 45 mm above ground, so the ground
plane is z = -0.038 here). Keep in sync with tennis_robot.urdf.xacro intake_*
properties and scripts/generate_robot_urdf.py's Gazebo polyline collision.
"""

from __future__ import annotations

import os
import struct
import sys
from pathlib import Path

# Geometry (m, ground frame). Entry defaults to nip_x + 20 mm.
_NIP_X_M = float(os.getenv("INTAKE_NIP_X_M", "0.540"))
LIP_X = float(os.getenv("INTAKE_RAMP_ENTRY_X_M", str(_NIP_X_M - 0.040)))
LIP_HEIGHT_M = max(0.0, float(os.getenv("INTAKE_LIP_RAISE_M", "0.0")))
RAMP_CLEAR_RUN_M = max(0.004, float(os.getenv("INTAKE_RAMP_CLEAR_RUN_M", "0.030")))
RAMP_CLEAR_X = LIP_X - RAMP_CLEAR_RUN_M
RAMP_KNEE_X = float(os.getenv("INTAKE_RAMP_KNEE_X_M", "0.465"))
RAMP_END_X = float(os.getenv("INTAKE_RAMP_END_X_M", "0.425"))
RAMP_CLEAR_Z = max(LIP_HEIGHT_M, float(os.getenv("INTAKE_RAMP_CLEAR_Z_M", "0.004")))
RAMP_KNEE_Z = float(os.getenv("INTAKE_RAMP_KNEE_Z_M", "0.020"))
RAMP_END_Z = float(os.getenv("INTAKE_RAMP_END_Z_M", "0.045"))
SCOOP_WIDTH = 0.180
SHEET_THICKNESS = 0.002
COLLISION_CLEARANCE = 0.001

RAMP_STEPS = 28

# URDF frame constant (m).
GROUND_Z = -0.038        # ground plane in funnel_link frame
HALF_WIDTH = SCOOP_WIDTH / 2.0


def _smoothstep(t: float) -> float:
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


def ramp_z(x: float) -> float:
    """Ramp top height in ground frame."""
    if x >= RAMP_CLEAR_X:
        t = (LIP_X - x) / max(LIP_X - RAMP_CLEAR_X, 1e-6)
        return LIP_HEIGHT_M + (RAMP_CLEAR_Z - LIP_HEIGHT_M) * _smoothstep(t)
    if x >= RAMP_KNEE_X:
        t = (RAMP_CLEAR_X - x) / max(RAMP_CLEAR_X - RAMP_KNEE_X, 1e-6)
        return RAMP_CLEAR_Z + (RAMP_KNEE_Z - RAMP_CLEAR_Z) * _smoothstep(t)
    t = (RAMP_KNEE_X - x) / max(RAMP_KNEE_X - RAMP_END_X, 1e-6)
    return RAMP_KNEE_Z + (RAMP_END_Z - RAMP_KNEE_Z) * _smoothstep(t)


def profile() -> list[tuple[float, float, float]]:
    """(x, z_bottom, z_top) in funnel frame, ordered lip -> basket."""
    pts: list[tuple[float, float]] = []
    for i in range(RAMP_STEPS + 1):
        t = i / RAMP_STEPS
        x = LIP_X + (RAMP_END_X - LIP_X) * t
        pts.append((x, ramp_z(x)))
    # Thin ramp sheet. The first lip reaches the court-height bite point; the
    # underside follows the ramp so the scoop does not drag as a wide skid.
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
    # top (ramp) + bottom surfaces
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
        f.write(b"top-roller launcher scoop (generate_curved_scoop_mesh.py)".ljust(80, b"\0"))
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
