import math
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"))
from tennis_robot import court_extraction as ce
from tennis_robot import court_coverage as cov

spec=ce.CourtSpec()
# frame rotated/translated to prove it's frame-relative
th=0.6; tx=5.0; ty=-3.0
fr=ce.CourtFrame(tx,ty, math.cos(th),math.sin(th), -math.sin(th),math.cos(th))

vps=cov.vantage_points(fr, spec)
assert len(vps)==2
# transform each vantage back to court frame -> expect x'=±L/4, y'=0
for vp in vps:
    xp,yp=fr.to_court(vp["x_m"],vp["y_m"])
    assert abs(abs(xp)-spec.half_length_m/2)<0.01 and abs(yp)<0.01, (xp,yp)
print("vantage points OK:", [(v["court_x"],v["court_y"]) for v in vps])

assert cov.is_recoverable_failure("fence_side_missing:x_far (3 pts)")
assert cov.is_recoverable_failure("coverage_incomplete: only 10 points")
assert cov.is_recoverable_failure("net_not_observed: robot->net vector too short")
assert not cov.is_recoverable_failure("nonstandard_or_bad_fit: near_baseline run-off 19m")
assert not cov.is_recoverable_failure("ambiguous_court_width: ...")
assert not cov.is_recoverable_failure("fence_not_rectangular: ...")
print("recoverable classification OK")
print("\nALL COVERAGE TESTS PASSED")
