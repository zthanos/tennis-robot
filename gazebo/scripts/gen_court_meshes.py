#!/usr/bin/env python3
"""Generate DAE mesh files for the Gazebo tennis court model.

Run from the project root:
    python gazebo/scripts/gen_court_meshes.py

Output (committed to repo so Docker doesn't need to regenerate):
    gazebo/models/tennis_court/meshes/fence_long.dae   — 15 × 4 m panel (east/west)
    gazebo/models/tennis_court/meshes/fence_wide.dae   — 29.08 × 4 m panel (north/south)
    gazebo/models/tennis_court/meshes/net_mesh.dae     — 11.3 × 0.914 m rope mesh

Fence: diagonal ±45° wire ribbons, 4 cm rhombus diamonds, 1 mm wire radius.
Net:   horizontal ropes every 10 cm + vertical strings every 30 cm, 3 mm wire radius.

Coordinate convention (Gazebo Z-up):
    All panels are in the XZ plane (Y ≈ 0).
    X = horizontal width, Z = height from 0, centered at X=0.
    In the SDF the link pose applies a rotation to place the panel correctly.
"""

from __future__ import annotations

import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MESHES = ROOT / "gazebo" / "models" / "tennis_court" / "meshes"


# ─────────────────────────────────────────────────────────────── DAE writer ──

def _dae(geo_id: str, verts: list[tuple], tris: list[tuple]) -> str:
    pos = " ".join(f"{v:.5f}" for vtx in verts for v in vtx)
    idx = " ".join(str(i) for t in tris for i in t)
    n_v, n_t = len(verts), len(tris)
    return (
        '<?xml version="1.0" encoding="utf-8"?>\n'
        '<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">\n'
        '  <asset><up_axis>Z_UP</up_axis></asset>\n'
        '  <library_geometries>\n'
        f'    <geometry id="{geo_id}">\n'
        '      <mesh>\n'
        '        <source id="P">\n'
        f'          <float_array id="PA" count="{n_v*3}">{pos}</float_array>\n'
        '          <technique_common>\n'
        f'            <accessor source="#PA" count="{n_v}" stride="3">\n'
        '              <param name="X" type="float"/>\n'
        '              <param name="Y" type="float"/>\n'
        '              <param name="Z" type="float"/>\n'
        '            </accessor>\n'
        '          </technique_common>\n'
        '        </source>\n'
        '        <vertices id="V"><input semantic="POSITION" source="#P"/></vertices>\n'
        f'        <triangles count="{n_t}">\n'
        '          <input semantic="VERTEX" source="#V" offset="0"/>\n'
        f'          <p>{idx}</p>\n'
        '        </triangles>\n'
        '      </mesh>\n'
        '    </geometry>\n'
        '  </library_geometries>\n'
        '  <library_visual_scenes>\n'
        '    <visual_scene id="S">\n'
        f'      <node><instance_geometry url="#{geo_id}"/></node>\n'
        '    </visual_scene>\n'
        '  </library_visual_scenes>\n'
        '  <scene><instance_visual_scene url="#S"/></scene>\n'
        '</COLLADA>\n'
    )


# ─────────────────────────────────────────────────────── geometry helpers ──

def _add_ribbon(
    verts: list, tris: list,
    x0: float, z0: float,
    x1: float, z1: float,
    hw: float,
) -> None:
    """Flat ribbon quad in the XZ plane (Y=0) from (x0,0,z0) to (x1,0,z1), half-width hw."""
    dx, dz = x1 - x0, z1 - z0
    L = math.hypot(dx, dz)
    if L < 1e-9:
        return
    nx, nz = -dz / L, dx / L  # in-plane normal (perpendicular to wire)
    b = len(verts)
    verts += [
        (x0 + hw * nx, 0.0, z0 + hw * nz),
        (x0 - hw * nx, 0.0, z0 - hw * nz),
        (x1 - hw * nx, 0.0, z1 - hw * nz),
        (x1 + hw * nx, 0.0, z1 + hw * nz),
    ]
    tris += [(b, b + 1, b + 2), (b, b + 2, b + 3)]


def _cohen_sutherland(
    x0: float, y0: float, x1: float, y1: float,
    xmin: float, xmax: float, ymin: float, ymax: float,
) -> tuple[float, float, float, float] | None:
    """Clip line segment to axis-aligned rectangle. Returns clipped endpoints or None."""
    def code(x: float, y: float) -> int:
        return (1 if x < xmin else 2 if x > xmax else 0) | (4 if y < ymin else 8 if y > ymax else 0)

    c0, c1 = code(x0, y0), code(x1, y1)
    for _ in range(12):
        if not (c0 | c1):
            return x0, y0, x1, y1
        if c0 & c1:
            return None
        c = c0 or c1
        if c & 1:
            x = xmin
            y = y0 + (y1 - y0) * (xmin - x0) / (x1 - x0) if x1 != x0 else y0
        elif c & 2:
            x = xmax
            y = y0 + (y1 - y0) * (xmax - x0) / (x1 - x0) if x1 != x0 else y0
        elif c & 4:
            y = ymin
            x = x0 + (x1 - x0) * (ymin - y0) / (y1 - y0) if y1 != y0 else x0
        else:
            y = ymax
            x = x0 + (x1 - x0) * (ymax - y0) / (y1 - y0) if y1 != y0 else x0
        if c == c0:
            x0, y0, c0 = x, y, code(x, y)
        else:
            x1, y1, c1 = x, y, code(x, y)
    return None


# ─────────────────────────────────────────────────────────── generators ──

def gen_fence(
    width: float,
    height: float,
    diamond: float = 0.04,
    wire_r: float = 0.001,
) -> tuple[list, list]:
    """Chain-link fence panel in the XZ plane, centered in X.

    X ∈ [−width/2, +width/2], Z ∈ [0, height].
    Wire ribbons at ±45° with perpendicular spacing = diamond.
    """
    verts: list[tuple] = []
    tris: list[tuple] = []
    hw = wire_r
    step = diamond
    half_w = width / 2

    for sign in (+1.0, -1.0):
        # For "+" sign: "/" wires (x_entry, z=0) going in direction (+1, +sign)
        # For "-" sign: "\" wires (x_entry, z=0) going in direction (+1, -sign)
        i_min = -int((height + width) / step) - 2
        i_max = int((height + width) / step) + 2
        for i in range(i_min, i_max + 1):
            x_start = -half_w + i * step
            z_start = 0.0 if sign > 0 else height
            x_end = x_start + height
            z_end = height if sign > 0 else 0.0
            result = _cohen_sutherland(
                x_start, z_start, x_end, z_end,
                -half_w, half_w, 0.0, height,
            )
            if result:
                xa, za, xb, zb = result
                _add_ribbon(verts, tris, xa, za, xb, zb, hw)

    return verts, tris


def gen_net(
    width: float,
    height: float,
    h_step: float = 0.10,
    v_step: float = 0.30,
    wire_r: float = 0.003,
) -> tuple[list, list]:
    """Net mesh in the XZ plane, centered in X.

    X ∈ [−width/2, +width/2], Z ∈ [0, height].
    Horizontal ropes every h_step, vertical strings every v_step.
    Top 63 mm rendered thicker (top band).
    """
    verts: list[tuple] = []
    tris: list[tuple] = []
    half_w = width / 2

    # Horizontal ropes
    z = 0.0
    while z <= height + 1e-6:
        r = wire_r * 2.5 if z >= height - 0.063 - 1e-4 else wire_r
        _add_ribbon(verts, tris, -half_w, z, half_w, z, r)
        z = round(z + h_step, 6)

    # Vertical strings
    x = -half_w
    while x <= half_w + 1e-6:
        _add_ribbon(verts, tris, x, 0.0, x, height, wire_r)
        x = round(x + v_step, 6)

    return verts, tris


# ─────────────────────────────────────────────────────────────────── main ──

def main() -> None:
    MESHES.mkdir(parents=True, exist_ok=True)

    tasks = [
        ("fence_long",   gen_fence,  {"width": 15.0,   "height": 4.0}),
        ("fence_wide",   gen_fence,  {"width": 29.08,  "height": 4.0}),
        ("net_mesh",     gen_net,    {"width": 11.3,   "height": 0.914, "h_step": 0.05, "v_step": 0.05}),
    ]

    for geo_id, func, kwargs in tasks:
        verts, tris = func(**kwargs)
        path = MESHES / f"{geo_id}.dae"
        path.write_text(_dae(geo_id, verts, tris), encoding="utf-8")
        print(f"  {path.name}: {len(verts)} vertices, {len(tris)} triangles")

    print(f"\nMesh files written to {MESHES}")


if __name__ == "__main__":
    main()
