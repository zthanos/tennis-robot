# Low L basket / rear corner-extension architecture

## Current revision: upright battery on front side rail

2026-10-11. Upright battery envelope is X198 × Y166 × Z170 mm, mass 5.59 kg.
The long footprint axis follows the robot length; the narrow axis is across its
width. Centre is (155,162,137) mm. Bottom Z52 now sits at the side-rail top,
29 mm higher than the previous recessed bottom; top Z222 is 2 mm above the basket
rim. Bounds X[56,254], Y[79,245], Z[52,222]. The outer edge is flush with the
side-rail outer edge, giving 50 mm nominal contact width. Gap before the fixed
front brace footprint is 15.4 mm. Basket floor is NOT raised.
The old ground-level strips are replaced by a second inner support reservation
at Y95 (20×10 section) and two transverse ties at X85/225 (20×10 section).
These ties overlap the existing side rail intentionally. They are cantilever
packaging placeholders, NOT load-rated parts or released bolted connections.
Battery retention, terminal access and wheel/hardware clearance remain open.
The lateral mass first moment is 0.90558 kg·m (previously 0.57018); movement
must not be advertised as whole-robot stability improvement without a CG check.

Battery exclusion bay X46..264/Y69..185 leaves a rear branch 166 mm externally /
160 mm internally. Main side channel is 254/248 mm. A 46 mm front pocket
ahead of the exclusion bay opens the basket mouth
at the intake centre; this introduces a front return in the formerly simple L
footprint. Mouth cutout is 190 mm wide. Entry reference turns only 50 mm laterally
instead of 95 mm. This geometry follows the actual battery envelope rather than
independent hardcoded notch coordinates.

Optional `battery_position="rear"` shifts only X from 155 to 40 mm while keeping
the same upright rail-supported arrangement. Rear comparison bounds are
X[-59,139]; the positive-Y rear collector arm becomes 51 mm outer /45 mm inner,
too narrow to admit a 66 mm ball there without reshaping. This does not by itself
prove the centre feed corridor is blocked. The front default reserves a larger
rear collector area for funnel/service/one-ball feeding development. Front vs
rear here is a packaging comparison, not a verified wheel-load/CG optimisation.
Rear comparison preview: `rear-corner-battery-rear-comparison.png`; no strength
or complete collision approval is implied for that alternative.

Launcher, extensions, floor Z48..52, intake handoff Z35 and rear feeder are
unchanged. Battery does not intersect the reference entry sweep. However,
`qa_entry_basket` is deliberately NOT a pass: the straight candidate sweep
intersects the floor lip at Z48.70..51.93, not a vertical wall or the battery.
The existing 17 mm rise needs a real transfer profile/contact test; do not discard
that positive intersection or claim collection flow is validated. Multi-ball
turning/bridging and terminal/service clearances remain open.

Other selected pairs must be rerun after edits. No existing chassis/brace/intake
was cut, no manufacturing STL/xacro was generated. The initial revision below is
historical; its coordinates and nine-pair result do not describe this revision.
Current run: ten selected intersection checks empty; `qa_entry_basket` positive
at the floor lip as reported above. The three rear-layout/chassis test modules
passed 23 tests. Numerical test success does not override that geometric blocker.

## Initial revision (historical)

2026-10-11. Third, separate alternative reflecting the user's clarified route.
Source: `rear-corner-launcher-study.scad`. Earlier centre/side basket alternatives
are preserved. No manufactured STL, xacro, main chassis or existing intake changed.

## Intent and provisional coordinates

Front intake → left low basket channel → rear transverse leg → open funnel →
controlled one-ball feeder → rear-facing flywheel. Battery remains outside the
basket, in the rear/right bay, instead of underneath the basket floor.

Battery 198×170×166 mm, 5.59 kg: X[-29,169], Y[0,170], Z[23,189]. Two strip
reservations, underside Z20/thickness3; this is NOT a load-approved support or
verified clearance under full payload. Battery bay has 10 mm nominal separation
from the L notch. Retention, terminals, ventilation and removal access are open.

Basket footprint X[-120,310], Y[-185,185], notch removing the front/right region
from X=-39,Y=-10. The rear transverse leg has 81 mm outer / 75 mm inner width;
the main side channel has 175 mm outer /169 mm inner width. These are ball-fit
dimensions, not multi-ball flow approval. Concave turn still needs a diagonal or
rounded flow guide and a physical jam test. Battery is not a basket wall support.

Basket floor UPPER surface Z48..52, rim Z220. Existing intake handoff surface
Z35 stays unchanged: 17 mm upward transfer plus lateral deflection to the left
channel remains unresolved. Orange ghost denotes transfer space, not a solved
mechanism. Floor gradient is only 4/430 (~0.53°): rolling reliability under
friction, robot pitch and acceleration must be tested; no gravity-flow claim.

Launcher whole-assembly pose (-260,0,140), pitch20°, yaw180°. Existing relieved
8 mm panels, wheel/motor geometry and metal spacers are reused without mirroring
individual manufactured parts. Nip is lower; horizontal feed-entry ball centre
is approximately (-86.157,0,76.726). Final approach to the existing inclined nip
is uphill, so a short controlled feed/pusher remains necessary to investigate.
No launch angle, motor or plate design study has been replaced.

Two candidate rear rail extensions X[-365,-165], Y±220, 50×30 section, with
70×70 corner landing reservations. 200 mm length is deliberately a first clear
packaging hypothesis, **not a minimum or approved “small” extension**. Plate
overhang grows overall rear envelope further. Do not infer more compact geometry.
Cradle crossbar roots near X=-185.53/Z52.24 and X=-345.28/Z110.38 follow local
plate-support coordinates; actual sleeve/bolt/nut/compression joints are not
released. Near-flush first root is an intended integrated corner landing, not a
validated vertical socket. Old global/height FEA does not approve this cantilever.

## Verification boundaries and next gates

Selected solid intersections: battery vs fixed frame/extensions/cradle and filled
basket, launcher vs battery/basket, swept Ø66 drop-feed balls vs battery/launcher,
straight Ø76 exit vs selected support structure, chute vs launcher, basket vs
fixed frame. Empty intersection exports exit1 with the explicit empty-object
message; other errors must not be accepted as pass. Intentional mounting overlaps
are not clearance failures. These checks exclude full assembly collisions, tyre
deflection, hardware/tool access, feeder actuation and ball-to-ball contacts.

Run result: all nine selected pairs returned empty intersections; 21 tests passed
across the rear-corner, previous rear-layout and chassis-integration test modules.

Next: minimise rear extension while keeping actual balls, battery and motors
clear; design and physically test intake transfer and corner guide; calculate
empty/full CG and wheel loads including rear cantilever; design real joints and
5.59 kg battery supports; then isolated xacro variants with consistent masses,
inertias/collisions for turning/braking. Rigid-body motion simulation is not
printed joint strength approval. No xacro or new FEA was run at this step.
