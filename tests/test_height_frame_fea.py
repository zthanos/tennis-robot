import importlib.util
import json
import math
from pathlib import Path
import sys

import pytest


def module():
    directory=Path(__file__).resolve().parents[1]/'engineering/chassis-strength-bench'
    sys.path.insert(0,str(directory))
    try:
        spec=importlib.util.spec_from_file_location('height_frame',directory/'run_height_frame_fea.py')
        m=importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
        return m
    finally:
        sys.path.pop(0)


def test_pose_matches_current_cad_and_crossbar_endpoints():
    m=module()
    cad=m.CAD.read_text()
    assert 'launcher_reference_nip_z=298;' in cad
    assert 'launcher_reference_pitch=20;' in cad
    assert 'plate_lock_x=[-115,115];' in cad
    assert 'plate_lock_y=[-145,145];' in cad
    for x in (350,440):
        p=m.xyz(m.mount_x(x),260,-57)
        assert p[0]==pytest.approx(x)
        assert p[2]==pytest.approx(m.mount_z(x))


def test_frame_welds_near_identical_nodes_and_vertical_members():
    m=module(); model=m.Model()
    model.add((-165,-220,37),(-165,-220,100))
    assert model.node((-165.000001,-220,100))==model.node((-165,-220,100))
    model.mesh(10)
    for ids,*props in model.beams:
        assert len(set(ids))==3
        normal=props[-1]
        assert math.sqrt(sum(v*v for v in normal))==pytest.approx(1)
        assert normal[2]==pytest.approx(0)


def flange_model(m):
    model=m.Model(); flange=[]
    for y in (-129,129):
        flange.append([model.node(m.xyz(15*math.cos(a*math.pi/2),y+15*math.sin(a*math.pi/2),43))
                       for a in range(4)])
    return model,flange


def test_motor_weight_and_torque_couples_preserve_resultants():
    m=module(); model,flange=flange_model(m)
    _,gravity=m.scenario_loads(model,flange,'gravity')
    _,torque=m.scenario_loads(model,flange,'motor_torque_reference')
    f=[sum(v for (_,d),v in gravity.items() if d==k+1) for k in range(3)]
    assert f==pytest.approx([0,0,-2*(.49+.9)*9.80665])
    for side,ns in zip((-1,1),flange):
        centre=m.xyz(0,side*129,43)
        moment=[0,0,0]; force=[0,0,0]
        for n in ns:
            df=[torque.get((n,k+1),0)-gravity.get((n,k+1),0) for k in range(3)]
            r=[model.nodes[n][k]-centre[k] for k in range(3)]
            mm=m.cross(r,df)
            moment=[moment[k]+mm[k] for k in range(3)]
            force=[force[k]+df[k] for k in range(3)]
        assert force==pytest.approx([0,0,0],abs=1e-8)
        assert moment==pytest.approx(m.rotate((0,0,side*620)),abs=.001)


def test_metal_density_uses_tonne_mm_units_and_pipe_area():
    m=module()
    profiles=m.material_profiles({'reference_elastic_modulus_gpa':1.5,'poisson_ratio':.38,'density_kg_m3':1270})
    assert profiles['PLATE']==(69000,.33,2.7e-9)
    assert m.section_area(12,12,4.35,'PIPE')==pytest.approx(math.pi*(6**2-1.65**2))


def test_panel_embedded_anchors_and_reliefs_exist():
    m=module()
    for upper in (False,True):
        geo=m.panel_geometry(upper,20)
        assert 'Mesh.ElementOrder=2' in geo
        assert 'BooleanDifference' in geo
        assert 'Point{1000} In Surface{s[0]}' in geo
        assert geo.count(' In Surface')==(12 if upper else 4)


def test_saved_full_matrix_has_shell_stresses_and_numerical_checks():
    m=module()
    report=json.loads((m.ROOT/'results/height_frame/report.json').read_text())
    assert len(report['cases'])==54
    assert len({(c['material'],c['scenario'],c['mesh_mm']) for c in report['cases']})==54
    assert report['validation']['convergence_gate']=='PASS'
    assert report['validation']['max_force_balance_relative_error']<.01
    for case in report['cases']:
        assert {'lower_plate','upper_plate'}<=case['peak_integration_stress_mpa_by_group'].keys()
        assert all(math.isfinite(v) for v in case['peak_integration_stress_mpa_by_group'].values())
    assert report['status']=='PRELIMINARY_IDEAL_BONDED_SCREEN_NOT_RELEASE'
