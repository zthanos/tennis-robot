import sys, os, math
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"))
from tennis_robot import court_extraction as ce
from tennis_robot import court_coverage as cov

spec = ce.CourtSpec()
# rotated/translated frame -> waypoints are frame-relative
th, tx, ty = 0.6, 5.0, -3.0
fr = ce.CourtFrame(tx, ty, math.cos(th), math.sin(th), -math.sin(th), math.cos(th))

vps = cov.vantage_points(fr, spec)
assert len(vps) == 8, len(vps)  # 5 coverage + 3 return pass (loop-closure overlap)
xs = [v["court_x"] for v in vps]
assert min(xs) < 0 and max(xs) > 0, "covers both halves"
gaps = [v for v in vps if abs(v["court_y"]) > 1]
assert gaps and all(spec.post_half_span_doubles_m < abs(v["court_y"]) for v in gaps), "gap beyond post"
stop_short = [v for v in vps if v.get("stop_short")]
assert len(stop_short) == 2, stop_short
assert all(abs(v["court_y"]) < 0.01 for v in stop_short), stop_short
assert all(abs(v["court_x"]) > spec.half_length_m for v in stop_short), stop_short
assert all(not v.get("stop_short") for v in gaps), "gap crossings must not stop on the net"
# return pass crosses the net through BOTH gaps (positive and negative y')
assert any(v["court_y"] > 1 for v in gaps) and any(v["court_y"] < -1 for v in gaps), "return uses other gap"
for v in vps:  # map round-trips back to court coords
    cx, cy = fr.to_court(v["x_m"], v["y_m"])
    assert abs(cx - v["court_x"]) < 0.01 and abs(cy - v["court_y"]) < 0.01
print("gap-crossing + return waypoints OK:", [(v["court_x"], v["court_y"]) for v in vps])

assert cov.is_recoverable_failure("fence_side_missing:x_far (3 pts)")
assert cov.is_recoverable_failure("coverage_incomplete: only 10 points")
assert not cov.is_recoverable_failure("nonstandard_or_bad_fit: ...")
assert not cov.is_recoverable_failure("ambiguous_court_width: ...")
print("recoverable classification OK")
print("\nALL COVERAGE TESTS PASSED")
