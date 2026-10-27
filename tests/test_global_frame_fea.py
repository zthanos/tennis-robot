import importlib.util
from pathlib import Path
import pytest


def module():
    path=Path(__file__).resolve().parents[1]/"engineering/chassis-strength-bench/run_global_frame_fea.py"
    spec=importlib.util.spec_from_file_location("global_frame",path)
    result=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def test_variants_share_load_points_and_supports():
    m=module()
    for variant in m.FRONT_STUDY_VARIANTS:
        nodes,elements,segments,fixed,front=m.build(variant,10)
        assert {nodes[n] for n in fixed}=={(-165,-220,37),(-165,220,37)}
        assert {nodes[n] for n in front}=={(300,-220,37),(300,220,37)}
        assert all(all(n in nodes for n in element[1:]) for element in elements)
        if variant=="ramp_back":
            assert any(segment[1:4]==(25,20,3.2) for segment in segments)


@pytest.mark.parametrize("height", [20,30])
def test_wide_front_preserves_ramp_facing_plane_and_bottom(height):
    nodes,elements,segments,_,_=module().build(f"ramp_back_50x{height}",10)
    crossbars=[s for s in segments if s[1:5]==(50,height,3.2,"BOX")
               and all(nodes[n][0]==294.4 for e in s[0] for n in elements[e-1][1:])]
    assert len(crossbars)==4
    assert 294.4+50/2==319.4
    assert (294.4,0,22+height/2) in nodes.values()


def test_deck_contains_hollow_sections_and_wheel_offset_moment(tmp_path):
    m=module()
    path=tmp_path/"frame.inp"
    m.write_deck(path,m.build("ramp_back"),
                 {"reference_elastic_modulus_gpa":2.3,"poisson_ratio":.36})
    text=path.read_text()
    assert "TYPE=B32R" in text
    assert "SECTION=BOX" in text
    assert "20,25,3.2,3.2,3.2,3.2" in text
    assert text.count(",6,-1300")==2


def test_parser_requires_both_front_displacements(tmp_path):
    path=tmp_path/"result.dat"
    path.write_text("displacements\n1 0.1 0.2 0\n2 -0.1 0.2 0\n")
    report=module().parse_dat(path,[1,2])
    assert report["relative_front_x_mm"]==.2
    assert report["mean_front_y_mm"]==.2
    with pytest.raises(RuntimeError):
        module().parse_dat(path,[1,3])
