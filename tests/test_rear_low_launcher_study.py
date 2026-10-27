import math
from pathlib import Path
import re

CAD=(Path(__file__).resolve().parents[1]
     /'cad/modular-test-chassis/archive/v2-chassis/rear-low-launcher-study.scad')

def scalar(text,name):
    return float(re.search(rf'^{name}=(-?[\d.]+);',text,re.M)[1])

def test_alternative_keeps_original_pitch_plates_and_separate_sources():
    text=CAD.read_text()
    assert scalar(text,'pitch')==20
    assert 'rotate([0,0,180]) rotate([0,-pitch,0]) children()' in text
    assert 'launcher_panel_reference(false); launcher_panel_reference(true);' in text
    assert 'use <height-infrastructure-study.scad>' in text
    assert 'STUDY_ONLY' in text

def test_candidate_roots_stay_on_current_rails_and_above_them():
    text=CAD.read_text(); x=scalar(text,'nip_x'); z=scalar(text,'nip_z')
    a=math.radians(scalar(text,'pitch'))
    for local_x in (-100,70):
        root_x=x-math.cos(a)*local_x+math.sin(a)*-57
        root_z=z+math.sin(a)*local_x+math.cos(a)*-57
        assert -165<=root_x<=455
        assert root_z-10>52

def test_entry_lift_and_battery_are_explicit_not_gravity_claim():
    text=CAD.read_text()
    assert 'battery_size=[198,170,166]' in text
    assert scalar(text,'battery_mass_kg')==5.59
    assert scalar(text,'ground_clearance')==20
    assert scalar(text,'strip_t')==3
    assert 'battery_bottom=strip_bottom+strip_t' in text
    assert 'basket_entry_z=basket_floor_front+3' in text
    assert scalar(text,'feed_local_x')<-128-33-5
    assert 'missing active lift' in text
    assert 'Needs an escapement' in text

def test_center_floor_stack_and_l_layout_are_distinct():
    # Clear-brace configuration versus requested target20: surface heights.
    assert 43+3+166+5+3==220
    assert 20+3+166+5+3==197
    assert 220+48-35==233
    assert 165+3-35==133
    text=CAD.read_text()
    assert 'battery_layout="center"' in text
    assert 'mount_mode="clear"' in text
    assert 'battery_layout=="side_l" ? 85 : 0' in text
    assert 'qa_battery_frame' in text
    # L rear arm is 86 mm externally / 80 mm internally, for 66 mm balls.
    assert 166-80-6>66
    assert '[[80,-180],[374,-180],[374,-10],[166,-10],[166,180],[80,180]]' in text
