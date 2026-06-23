import math
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"))
from tennis_robot import court_extraction as ce

def gen(runoff_b=3.0, runoff_s=2.5, is_doubles=True, theta=0.0, tx=0.0, ty=0.0,
        obstacle=None, drop_side=None):
    half_l=11.885; half_w=(10.97 if is_doubles else 8.23)/2.0
    fx=half_l+runoff_b; fy=half_w+runoff_s
    pts=[]
    def line(x0,y0,x1,y1,n=200):
        for i in range(n):
            t=i/(n-1); pts.append((x0+(x1-x0)*t, y0+(y1-y0)*t))
    if drop_side!='x_near': line(-fx,-fy,-fx,fy)
    if drop_side!='x_far':  line(fx,-fy,fx,fy)
    if drop_side!='y_left': line(-fx,-fy,fx,-fy)
    if drop_side!='y_right':line(-fx,fy,fx,fy)
    post=5.65 if is_doubles else 5.03
    line(0,-post,0,post,140)
    if obstacle:
        ox,oy,ow,oh=obstacle
        i=0
        while i*0.05<=ow:
            j=0
            while j*0.05<=oh:
                pts.append((ox-ow/2+i*0.05, oy-oh/2+j*0.05)); j+=1
            i+=1
    c=math.cos(theta); s=math.sin(theta)
    mpts=[{"x_m":tx+x*c-y*s,"y_m":ty+x*s+y*c} for x,y in pts]
    rbx,rby=(-fx-1.0),0.0
    locked={"map_x_m":tx,"map_y_m":ty,
            "robot_x_m":tx+rbx*c-rby*s,"robot_y_m":ty+rbx*s+rby*c}
    return mpts, locked

# T1 standard, axis-aligned
m,l=gen(); km=ce.extract_court_knowledge_model(m,l)
d=km["distances_to_fence_m"]
assert km["status"]=="OK" and km["court"]["is_doubles"]
assert abs(d["near_baseline"]-3.0)<0.2 and abs(d["far_baseline"]-3.0)<0.2
assert abs(d["left_sideline"]-2.5)<0.2 and abs(d["right_sideline"]-2.5)<0.2
assert abs(km["net"]["span_m"]-11.3)<0.3
print("T1 standard OK:", d, "doubles", km["court"]["is_doubles"], "span", km["net"]["span_m"])

# T2 rotated + translated -> same distances (frame-invariant)
m,l=gen(theta=0.6, tx=5.0, ty=-3.0); km2=ce.extract_court_knowledge_model(m,l)
d2=km2["distances_to_fence_m"]
for k in d: assert abs(d2[k]-d[k])<0.2, (k,d2[k],d[k])
print("T2 rotated/translated invariant OK:", d2)

# T3 interior obstacle (a bench ~0.4x1.2 at court x'=3,y'=1)
m,l=gen(obstacle=(3.0,1.0,0.4,1.2)); km3=ce.extract_court_knowledge_model(m,l)
obs=km3["obstacles"]
assert len(obs)>=1, obs
print("T3 obstacle OK:", [(o["class"],o["size_m"],o["point_count"]) for o in obs])

# T4 missing fence side -> fail-loud
try:
    m,l=gen(drop_side='y_left'); ce.extract_court_knowledge_model(m,l); print("T4 FAIL: no raise")
except ce.CourtExtractionError as e:
    assert "fence_side_missing" in e.reason or "fence" in e.reason, e.reason
    print("T4 missing-side fail-loud OK:", e.reason)

# T5 bad net (robot at net) -> fail
try:
    m,_=gen(); ce.extract_court_knowledge_model(m, {"map_x_m":0,"map_y_m":0,"robot_x_m":0,"robot_y_m":0})
    print("T5 FAIL: no raise")
except ce.CourtExtractionError as e:
    print("T5 bad-net fail-loud OK:", e.reason)

# T6 non-standard run-off (huge) -> fail
try:
    m,l=gen(runoff_b=20.0); ce.extract_court_knowledge_model(m,l); print("T6 FAIL: no raise")
except ce.CourtExtractionError as e:
    assert "nonstandard" in e.reason, e.reason
    print("T6 nonstandard fail-loud OK:", e.reason)

print("\nALL COURT EXTRACTION TESTS PASSED")

# T9 smart fence-artifact filter: parallel-to-fence scatter rejected; a real
# obstacle protruding inward (perpendicular) near the same fence is kept.
def _cluster(cx, cy, w, h, n=40):
    import random; random.seed(1)
    return [(cx + (random.random()-0.5)*w, cy + (random.random()-0.5)*h) for _ in range(n)]

_spec = ce.CourtSpec()
_fence = {"x_near": -16.6, "x_far": 16.4, "y_left": -8.5, "y_right": 8.8}
# strip hugging the near (vertical) fence, elongated along y' -> artifact
_strip = _cluster(-15.4, 4.0, 0.4, 1.0)
# real object near the same fence but protruding inward (elongated along x') -> keep
_real  = _cluster(-15.2, -2.0, 1.2, 0.4)
_obs = ce.extract_obstacles(_strip + _real, _fence, _spec)
_cxs = [round(o["center_court"][0], 1) for o in _obs]
assert not any(abs(c - (-15.4)) < 0.3 for c in _cxs), f"strip not rejected: {_cxs}"
assert any(abs(c - (-15.2)) < 0.3 for c in _cxs), f"real inward obstacle dropped: {_cxs}"
print("T9 fence-artifact filter OK: rejected parallel strip, kept inward obstacle")

# T10 compact fixture near a fence corner (e.g. light pole) is kept even though
# it is inside the normal edge exclusion band.
_corner_pole = _cluster(_fence["x_far"] - 0.35, _fence["y_right"] - 0.35, 0.35, 0.55, n=28)
_obs = ce.extract_obstacles(_corner_pole, _fence, _spec)
assert len(_obs) == 1, _obs
assert _obs[0]["class"] == "perimeter_fixture", _obs
print("T10 corner fixture OK:", _obs[0]["center_court"], _obs[0]["size_m"])

# T11 four corner fixtures are retained as one semantic fixture per corner even
# if one pole is split into adjacent sparse LiDAR fragments.
_corner_poles = []
for _cx in (_fence["x_near"] + 0.35, _fence["x_far"] - 0.35):
    for _cy in (_fence["y_left"] + 0.35, _fence["y_right"] - 0.35):
        _corner_poles += _cluster(_cx, _cy, 0.35, 0.45, n=12)
_corner_poles += _cluster(_fence["x_far"] - 0.85, _fence["y_right"] - 0.35, 0.20, 0.20, n=5)
_obs = ce.extract_obstacles(_corner_poles, _fence, _spec)
assert len(_obs) == 4, _obs
assert all(o["class"] == "perimeter_fixture" for o in _obs), _obs
print("T11 four corner fixtures OK:", [(o["center_court"], o["point_count"]) for o in _obs])
