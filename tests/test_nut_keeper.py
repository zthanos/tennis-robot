import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]


def test_addon_keeper_report_matches_current_files_and_has_no_overlap():
    report=json.loads((ROOT/'engineering/chassis-strength-bench/results/nut_keeper/report.json').read_text())
    cad=ROOT/'cad/drive-motor-mount/m3-side-nut-keeper.scad'
    stl=cad.parent/'nut-keeper-stl/sleeve.stl'
    assert report['source_sha256']==hashlib.sha256(cad.read_bytes()).hexdigest()
    assert report['stl_sha256']==hashlib.sha256(stl.read_bytes()).hexdigest()
    assert len(report['checks'])==3
    assert all(c['overlap_volume_cm3']<1e-8 for c in report['checks'])
    assert report['mesh']['positive_volume_shells']==1
    assert report['mesh']['nonmanifold_edge_count']==0
    assert report['mesh']['degenerate_triangles']==0
    assert report['assumptions']['entrance_to_screw_axis_mm']==9
