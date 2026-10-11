import importlib.util
from pathlib import Path
import json
import subprocess
import sys


def _module():
    path = Path(__file__).resolve().parents[1] / "engineering/chassis-strength-bench/measure_middle_t_design.py"
    spec = importlib.util.spec_from_file_location("middle_t_measurements", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_closed_tetrahedron_volume_and_edges():
    triangles = [((0,0,0),(0,1,0),(1,0,0)), ((0,0,0),(1,0,0),(0,0,1)),
                 ((0,0,0),(0,0,1),(0,1,0)), ((1,0,0),(0,1,0),(0,0,1))]
    lines = ["solid tetra"]
    for triangle in triangles:
        lines.extend("vertex " + " ".join(map(str, vertex)) for vertex in triangle)
    result = _module().mesh_stats("\n".join(lines).encode())
    assert result["nonmanifold_edge_count"] == 0
    assert abs(result["solid_volume_cm3"] - 1/6000) < 1e-12
    assert result["envelope_mm"] == [1,1,1]


def test_part_quantities_include_all_middle_connections():
    parts = _module().PARTS
    assert parts["middle_crossbar_half"] == 2
    assert parts["straight_sleeve"] == 8


def test_ramp_back_design_measurements(tmp_path):
    root = Path(__file__).resolve().parents[1]
    output = tmp_path / "ramp_back.json"
    subprocess.run([sys.executable, str(root / "engineering/chassis-strength-bench/measure_ramp_back_design.py"),
                    "--output", str(output)], check=True, capture_output=True)
    result = json.loads(output.read_text())
    assert result["pla_solid_mass_g"] < result["previous_middle_t_pla_solid_mass_g"]
    assert result["geometry"]["brace_top_z_mm"] < result["geometry"]["motor_nut_channel_bottom_z_mm"]
    rows = {row["part"]: row for row in result["parts"]}
    assert rows["straight_sleeve"]["quantity"] == 5
    assert rows["ramp_back_sleeve"]["quantity"] == 3
    assert rows["ramp_back_half"]["envelope_mm"] == [50,160,20]
