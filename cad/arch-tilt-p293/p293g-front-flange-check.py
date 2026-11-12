#!/usr/bin/env python3
"""Verify the measured front-flange CAD correction against P2-93g bodies.

Reads the fresh check-current-scene output.  It does not alter or substitute
the canonical drive envelope; X277 is already represented by the separate
front flange solids in the source CAD.
"""
import json
from pathlib import Path

import manifold3d as m3d
import numpy as np
import trimesh

H = Path(__file__).parent
OUT = H / "p293g-candidate-results"
SCENE = H.parent / "independent-check/out/scene"
report = json.loads((SCENE / "report.json").read_text())
baseline = json.loads((H / "p293h-analysis/baseline-report.json").read_text())
meshes = {
    key.rsplit("@", 1)[0]: trimesh.load(SCENE / filename, force="mesh")
    for key, filename in report["files"].items()
    if key.endswith("@0")
}


def solid(mesh):
    return m3d.Manifold(
        m3d.Mesh(
            vert_properties=np.asarray(mesh.vertices, np.float32),
            tri_verts=np.asarray(mesh.faces, np.uint32),
        )
    )


base = {name: solid(mesh) for name, mesh in meshes.items()}
P = np.array([387.0, 0.0, 51.837363636])
B = np.array([260.0, -205.0, 220.0])


def npos(angle):
    theta = np.radians(angle)
    return P + [30 * np.cos(theta) - 110 * np.sin(theta), -205,
                30 * np.sin(theta) + 110 * np.cos(theta)]


def frame(angle):
    axis = npos(angle) - B
    axis /= np.linalg.norm(axis)
    return np.column_stack([[-axis[2], 0, axis[0]], [0, -1, 0], axis])


def transform(name, angle):
    theta = np.radians(angle)
    rotation = np.array([[np.cos(theta), 0, -np.sin(theta)],
                         [0, 1, 0],
                         [np.sin(theta), 0, np.cos(theta)]])
    if name in ("moving_module_concept", "p293g_nut_arm"):
        return rotation, P - rotation @ P
    if name in ("p293g_nut_trunnion", "p293g_t8_nut_hardware"):
        rotation = frame(angle) @ frame(0).T
        return rotation, npos(angle) - rotation @ npos(0)
    if name.startswith("p293g_") and (
        "hardware" in name or name in ("p293g_cradle", "p293g_float_cap")
    ):
        rotation = frame(angle) @ frame(0).T
        return rotation, B - rotation @ B
    return np.eye(3), np.zeros(3)


def posed(name, angle):
    rotation, translation = transform(name, angle)
    return base[name].transform(np.column_stack([rotation, translation]))


body_names = [
    name for name in base
    if name.startswith(("p293", "arch_crossbar_center", "joint_sleeve_x"))
]
drive = base["drive_package_envelopes"]
rows = []
for angle in range(-10, 21):
    values = [(posed(name, angle).min_gap(drive, 100), name) for name in body_names]
    gap, body = min(values)
    rows.append({"angle_deg": angle, "minimum_gap_mm": gap, "body": body})

governing = min(rows, key=lambda row: row["minimum_gap_mm"])
minimum_required = 3.0
pair = lambda hit: tuple(sorted((hit["a"], hit["b"])))
baseline_pairs = {pair(hit) for hit in baseline["hits"]}
current_pairs = {pair(hit) for hit in report["hits"]}
new_pairs = sorted(current_pairs - baseline_pairs)
scene_ok = not new_pairs and not report["export_failures"]
gap_ok = governing["minimum_gap_mm"] >= minimum_required
result = {
    "status": "PASS_NO_NEW_COLLISIONS_AND_GE_3MM" if scene_ok and gap_ok else "FAIL",
    "measurements": {
        "date": "2026-11-12",
        "template_edge_X_mm": 271,
        "left_front_flange_gap_mm": 6,
        "right_front_flange_gap_mm": 6,
        "measured_front_flange_start_X_mm": 277,
        "old_cad_front_flange_start_X_mm": 280,
        "rear_motor_measurements": "pending; CAD unchanged",
    },
    "method": (
        "fresh check-current-scene manifolds; canonical X277 front flange solids; "
        "P2-93g kinematic poses at each integer angle"
    ),
    "acceptance_minimum_mm": minimum_required,
    "governing": governing,
    "angles": rows,
    "scene_gate": {
        "positions_deg": [-10, -3, 0, 15, 20],
        "bodies_per_angle": {
            angle: len(names) for angle, names in report["bodies"].items()
        },
        "export_failures": report["export_failures"],
        "baseline_collision_pairs": len(baseline_pairs),
        "current_collision_pairs": len(current_pairs),
        "new_collision_pairs": new_pairs,
        "global_gate_remains_nonzero_from_existing_out_of_scope_collisions": len(report["hits"]),
    },
    "p293h_candidate_solids": [],
    "notes": [
        "Motor/encoder envelopes retain conservative OD38-40/~80 mm dimensions.",
        "Rear drive envelopes are unchanged pending measurement.",
        "P2-93h is PARKED and has no candidate solids.",
    ],
}
(OUT / "front-flange-template-check.json").write_text(
    json.dumps(result, indent=2) + "\n"
)
print(json.dumps({"status": result["status"], "governing": governing}, indent=2))
if result["status"].startswith("FAIL"):
    raise SystemExit(1)
