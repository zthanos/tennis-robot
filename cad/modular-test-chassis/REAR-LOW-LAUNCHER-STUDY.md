# Alternative: rear, lower flywheel with fixed basket

**Historical first-layout notes below.** The active source now has two battery
layouts and a checked mounting-height selector. See
[BATTERY-LAYOUT-COMPARISON.md](BATTERY-LAYOUT-COMPARISON.md) for current coordinates,
limits and QA results. The earlier 60 mm battery bottom and seven-pair pass do not
describe the current low-battery comparison.

2026-10-11. Separate architecture/packaging study, not manufacturing release.
Source `rear-low-launcher-study.scad`. Existing chassis, intake, accepted relieved
plates and current front-launcher height study are unchanged. No new print STL.

## Candidate layout

Front intake → **active entry lift still missing** → fixed sloped basket →
descending funnel → **single-ball escapement/feed drive still missing** → rear
flywheel. Whole existing launcher is yawed 180°, not individually mirrored;
pitch remains 20°, plates remain aluminium 8 mm, spacers remain metal.
Nip candidate moves from front-study (410,0,298) to (-70,0,175) mm.

The two lower cradle crossbars have roots at X≈4.47 and -155.28 mm on the current
side rails, at Y±220. Connection reservations are not released joints. New basket
posts/tray are candidate supports, not tested attachments. Motors stay on the
upper plate, not below. Roof, LiDAR, covers, containment, electronics and cables
are not redesigned here. Rear launcher protrusion is visible; this is not a
claim of smaller overall dimensions or proven weight balance.

Battery is the old packaging envelope [166,198,170] mm rotated to [198,170,166],
bottom Z60, top Z226, X146..344. Actual dimensions, chemistry, permitted
orientation, terminals, ventilation, straps and removal access need confirmation.
Rotating this nearly cubic envelope saves only 4 mm of height.

Basket candidate floor rises from rear Z238 to front Z286 over X80..310,
roughly 11.8°. Width tapers from 370 mm at the rim to 190 mm at the floor.
This angle is a packaging choice, not a verified rolling/jamming threshold.
Nominal 12 mm between battery top and lowest floor surface is not service access.

The existing intake handoff surface remains Z35. Basket entry is Z286:
**251 mm of additional surface-height rise**, so this layout cannot collect
through a passive ramp alone. Orange ghost reserves an active lifting mechanism
near X370. Its inlet transfer, drive, clearance and mounting are not designed.

Funnel ends at ball centre (103.84,0,111.73), outside the upper plate input edge.
Blue side reserve denotes an unresolved escapement/feed-drive envelope, not a
working mechanism. The final approach to the unchanged 20° launcher is uphill;
gravity funnel delivery alone does not prove entry into the nip. Multi-ball
bridging, controlled one-ball release, contact forces and entry velocity need a
physical coupon or calibrated contact model. No previous front-layout FEA result
is valid as rear-layout strength approval.

## Geometric checks

OpenSCAD `qa_*` selectors check selected solid intersections. Empty intersection
exports exit 1 with “Current top level object is empty”; that is an expected
geometric pass, not a solver failure. Other exit errors must not be ignored.

- `qa_battery_hopper`: battery envelope against filled basket outer volume.
- `qa_launcher_battery`, `qa_launcher_hopper`: launcher against battery/basket.
- `qa_exit_frame`: straight Ø76 rear exit keepout against the chassis and new
  supports, not a measured ballistic path or containment test.
- `qa_feed_hardware`, `qa_feed_battery`: swept Ø66 balls in descending funnel
  against launcher and battery. Does not model ball-to-ball contact or flow.
- `qa_chute_hardware`: funnel walls/entry guide against launcher hardware.

Only these pairs are screened. Full assembly clashes, fabrication tolerances,
tool access, active lift and wiring remain open. Views `assembly`, `side`,
`launcher` keep this clearly separate from the fit-tested main CAD.

Final selected-pair check: all seven selectors above returned empty intersections.
The first feed path and funnel-wall intersections with the upper panel were
detected and removed by moving the feed endpoint and funnel mouth forward.
22 tests passed across the new rear-study, height-frame and chassis-integration
test modules. These checks do not establish functional ball flow or strength.

## Decision

Rear-low arrangement is worth comparing: the launcher is lower and the basket
need not move between collection and launch. It does **not** yet establish a
simpler complete mechanism: current battery height forces an entry lift, and
rear launch requires the robot to aim its rear towards the target. A flatter
confirmed battery or another battery placement may change that tradeoff.
Next gate is real battery envelope and measured intake delivery before detailed
lift/escapement design, then updated load/CG and structure checks.
