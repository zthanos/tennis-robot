import math
from pathlib import Path
import re

CAD=(Path(__file__).resolve().parents[1]
     /'cad/modular-test-chassis/archive/v2-chassis/rear-corner-launcher-study.scad')

def scalar(text,name):
    return float(re.search(rf'^{name}=(-?[\d.]+);',text,re.M)[1])

def test_study_is_separate_and_reuses_whole_launcher():
    text=CAD.read_text()
    assert scalar(text,'pitch')==20
    assert 'translate([nip_x+70,0,nip_z-175]) launcher_hardware();' in text
    assert 'use <rear-low-launcher-study.scad>' in text
    assert 'STUDY_ONLY' in text

def test_cradle_roots_fit_rear_extensions_and_keep_ground_clearance():
    text=CAD.read_text(); a=math.radians(scalar(text,'pitch'))
    for xx in (-100,70):
        x=scalar(text,'nip_x')-math.cos(a)*xx+math.sin(a)*-57
        z=scalar(text,'nip_z')+math.sin(a)*xx+math.cos(a)*-57
        assert scalar(text,'extension_x0')<=x<=scalar(text,'extension_x1')
        assert z-10>scalar(text,'strip_bottom')
    # Lowest corner of the tilted support rails remains above 20 mm.
    z=scalar(text,'nip_z')-128*math.sin(a)-67*math.cos(a)
    assert z>20

def test_low_basket_has_small_but_explicit_intake_step():
    text=CAD.read_text()
    assert scalar(text,'floor_front')-scalar(text,'intake_exit_z')==17
    assert scalar(text,'floor_rear')==48
    assert scalar(text,'battery_mass_kg')==5.59
    assert 'battery_position="front"' in text
    assert 'battery_center=[battery_x,162,137]' in text
    assert '[bay_x0,bay_y0],[bay_x0,185],[-120,185]' in text
    assert 166-6>66

def test_battery_and_drop_feed_do_not_overlap_in_x():
    text=CAD.read_text(); a=math.radians(scalar(text,'pitch'))
    feed_x=scalar(text,'nip_x')-math.cos(a)*scalar(text,'feed_local_x')
    assert feed_x+33<56
    assert 'qa_battery_frame' in text
    assert 'qa_feed_launcher' in text

def test_forward_side_bay_preserves_intake_pocket_and_fixed_brace():
    text=CAD.read_text()
    centre=[155,162,137]
    assert 'battery_x=battery_position=="rear" ? 40 : 155' in text
    dims=[float(v) for v in re.search(r'battery_size=\[([^\]]+)\]',text)[1].split(',')]
    assert dims==[198,166,170]
    lo=[centre[i]-dims[i]/2 for i in range(3)]
    hi=[centre[i]+dims[i]/2 for i in range(3)]
    assert lo==[56,79,52]
    assert hi==[254,245,222]
    assert min(hi[1],245)-max(lo[1],195)==50  # contact width on existing rail
    assert lo[2]==scalar(text,'rail_top')
    assert 269.4-hi[0]>=15
    assert 310-(hi[0]+10)==46
    assert (lo[0]-10)-(-120)-6==160  # rear collector inner width
    assert (lo[1]-10)-(-185)-6==248  # side channel inner width
    assert '[310,185],[bay_x1,185]' in text
    assert 'translate([308,0,85]) cube([12,190,90]' in text
    assert 'qa_entry_battery' in text and 'qa_entry_basket' in text

def test_rear_battery_reservation_constricts_collector():
    # Same basket bay rule, with comparison battery X40 instead of X155.
    rear_collector_inner=(40-198/2-10)-(-120)-6
    assert rear_collector_inner==45
    assert rear_collector_inner<66  # not approved for a 66 mm ball
    assert 'cube([battery_size[0],20,10]' in CAD.read_text()
