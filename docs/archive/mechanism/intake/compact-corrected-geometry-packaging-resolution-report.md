# Corrected-geometry compact packaging resolution study

> **Archived: `HISTORICAL_SUPERSEDED`.** Moving-pod travel and its reliefs are
> not current intake requirements. See the
> [fixed-motor architecture](../../../mechanism/standalone-intake-fixed-motor-compliant-tyre.md).

Date: 2026-08-25

## Decision

An analysis-only static packaging solution exists without changing the corrected
intake geometry. Keep the intake datums fixed, extend each approved tyre pocket
through the full 8 mm outward compliance travel, add shaped 2 mm motor reliefs
to the bridge and lower launcher plate, and translate the complete launcher
rigidly upward.

The minimum accepted 1 mm-grid launcher translation is `(0, 0, +33) mm`. The
recommended translation is `(0, 0, +35) mm`: it retains the launcher X/Y datum,
pitch, nip and internal geometry, gives the hard motor/flywheel pair a
conservative polygonal-envelope separation of about 10.0 mm, and reduces the
2 mm cradle relief to 6,955.944 mm3.

This is a geometric pass only. The bridge and lower cradle plate both require
through-reliefs in structural material. No allowable-stress, plywood
edge-distance, plate fatigue, bearing-load or fastener criteria exist in the
model, so structural review is mandatory. The current analysis-only Candidate A
guide allocation still intersects the translated launcher and does not remain
viable.

No authoritative CAD, Xacro/URDF/SDF, contract, controller, intake datum or
generated mesh was changed by this study.

## A. Fresh corrected baseline

The existing corrected-axis analyzer was rerun with OpenSCAD 2026.08.19 and
exact CGAL intersections. It reproduced the supplied values.

- At 0 mm compliance: motor/bridge 2,852.471 mm3; complete direct-drive
  hardware/cradle 15,731.240 mm3; motor/same-side flywheel 4,120.529 mm3 per
  side; basket and hood clear.
- At 4 mm: wheel/basket 213.917 mm3; wheel/hood 144.010 mm3; motor/bridge
  5,229.233 mm3; hardware/cradle 15,303.497 mm3; motor/same-side flywheel
  3,476.648 mm3 per side.
- At 8 mm: wheel/basket 2,602.794 mm3; wheel/hood 1,264.355 mm3; motor/bridge
  8,764.746 mm3; hardware/cradle 14,100.248 mm3; motor/same-side flywheel
  2,773.736 mm3 per side.
- Continuous 0..8 mm: motor/bridge 8,764.963 mm3; hardware/cradle
  19,771.486 mm3; motor/same-side flywheel 5,218.578 mm3 per side;
  wheel/basket 2,634.260 mm3; wheel/hood 1,269.348 mm3.
- Positive complete-hardware swept clearances: ramp 2.01 mm sampled, cheeks
  42.29 mm, chassis 7.48 mm, battery
  480.01 mm; opposite pods have zero intersection.

The continuous moving solids are exact endpoint hulls of convex primitives.
This is exact for the locked linear 0..8 mm world-Y translation. Positive
clearance acceptance was checked by exact CGAL intersection against 2 mm
expanded polygonal envelopes; sampled distances are not used to override an
intersection.

## B. Motor to bridge diagnosis

The conflict is in the horizontal 18 mm plywood portal plate, not either
upright. It occupies the boundary between the existing central basket/service
notch and the 22 x 22 mm tilted-motor service opening. The motor crosses the
full plate thickness, so the required feature is a shaped through-opening.

For each side, the mathematical swept conflict is approximately
`X=355..385`, `Y=165..183.992` on the left (mirrored on the right), and
`Z=150..168 mm`. The total removed volume is 8,764.963 mm3.

The 2 mm relief is:

- total removed volume 13,726.790 mm3;
- per-side bounds approximately `X=353.017..386.983`,
  `Y=165..185.975`, `Z=150..168 mm`, mirrored in Y;
- maximum local opening span 33.966 x 20.975 mm through the 18 mm plate.

The optional 3 mm relief removes 16,629.831 mm3 and reaches
`X=352.026..387.974`, `|Y|=165..186.966`, through the same plate thickness.

Remaining geometric material for the 2 mm relief includes 59.025 mm from the
relief's outer-Y edge to the plate edge, about 10.025 mm in plan to the inner
face of the Y=205 mm upright, and a conservative 6.217 mm edge-to-edge ligament
to the nearest 5.6 mm M5 service hole. The central side merges into the existing
notch/service opening. This is geometrically possible but is not a structural
pass.

`MOTOR_BRIDGE_RELIEF_GEOMETRICALLY_FEASIBLE = true`

`GEOMETRIC_PASS_STRUCTURAL_REVIEW_REQUIRED`

## C. Full-compliance tyre-pocket analysis

The existing approved 128 x 77 mm tyre-shaped pockets are centred only on the
parked wheel positions. The current pocket already supplies 2 mm radial and
axial clearance at 0 mm. The new intersections occur only in the previously
uncovered outward part of the 0..8 mm sweep.

For mathematical zero intersection, retain the existing pocket and extend it
with the exact unexpanded wheel sweep. Its centreline extension is 6 mm beyond
the existing 2 mm parked allowance; the added shaped-envelope volume is
130,787.621 mm3 across both pockets.

For 2 mm clearance, extend the existing pocket envelope through the full 8 mm
outward travel. The added shaped-envelope volume is 242,384.894 mm3 across both
sides. It removes 5,415.829 mm3 from the basket and 2,375.992 mm3 from the hood.
Those figures include the existing unintended collision plus the positive
clearance allowance.

The extension changes only the outward boundary. It does not move the inward
edge of the approved parked pocket, so it does not narrow or move the 70 mm
receiving channel or the ball-entry datum. Exact 2 mm expanded-wheel CGAL tests
remain empty against the ramp, cheeks, bridge, chassis and battery. The
rerouted hood support geometry also remains outside the extension: the outer
post inner edge is at |Y|=180 mm versus a pocket maximum of 171.706 mm, and the
crossbar underside is Z=142 mm versus a pocket maximum of 137.444 mm. The
handoff and hood load-path geometry is therefore unchanged by this extension.

`FULL_0_TO_8MM_TIRE_POCKET_EXTENSION_FEASIBLE = true`

## D. Motor to launcher-cradle analysis

Only launcher plate `side_plate(-1)` (the lower world-Z plate after the locked
side-by-side transform) is involved. The upper plate has zero intersection.

At the unshifted launcher datum:

- swept motor overlap is 17,067.204 mm3 total, 8,533.602 mm3 per side;
- complete direct-drive hardware overlap is 19,771.486 mm3 total;
- the motor intersection bounds are `X=355..385`, `Y=-157..157`,
  `Z=127.056..146.199 mm` across both sides;
- the intersection traverses the full 8 mm plate thickness;
- the largest instantaneous motor overlap occurs at 0 mm compliance.

The motor-only 2 mm relief removes 20,123.133 mm3 at the baseline datum. The
complete-hardware 2 mm relief removes 25,172.531 mm3. Both open the lower
plate's two front/outer corners and therefore require structural review.

With the recommended +35 mm rigid launcher translation, the wheel no longer
touches the cradle and the required 2 mm motor relief falls to 6,955.944 mm3,
3,477.972 mm3 per side. Its world bounds are
`X=353.145..386.432`, `Y=-157..157`, `Z=162.056..180.956 mm`.

In launcher-local coordinates each 2 mm corner cut traverses the complete
8 mm plate thickness, reaches 33.450 mm inward from the front X edge, and
reaches 20.088 mm inward from the corresponding outer Z edge. It leaves
222.550 mm of the 256 mm plate width behind the deepest cut. The nearest
flywheel shaft/bearing centre is at local X=0 and Z=251 or 509 mm; the cut is
about 94.9 mm away in the plate plane. Actual bearing pockets and fastener
holes are absent from this non-manufacturing launcher envelope, so their edge
distances cannot be certified. The cut opens a structural edge and its load
path cannot be declared adequate.

`MOTOR_CRADLE_LOCAL_RELIEF_FEASIBLE = true`

`GEOMETRIC_PASS_STRUCTURAL_REVIEW_REQUIRED`

## E. Motor to same-side flywheel separation

The continuous swept motor/flywheel intersection is 5,218.578 mm3 per side.
The left bounds are `X=367.869..385`, `Y=133.938..177.650`,
`Z=157.416..183.865 mm`; the right bounds mirror in Y. The largest
instantaneous volume occurs at 0 mm compliance.

Convex-polyhedron separating-axis analysis of the exact exported polygonal
envelopes gives a 10.603 mm maximum penetration depth. The shortest
mathematical separating launcher vector is:

- left: `(9.435, -3.408, 3.434) mm`;
- right: `(9.435, +3.408, 3.434) mm`.

For 2 mm positive separation the shortest vectors become:

- left: `(11.215, -4.051, 4.082) mm`, magnitude 12.603 mm;
- right: `(11.215, +4.051, 4.082) mm`, magnitude 12.603 mm.

The opposite Y signs prove that a global Y translation is not useful for the
bilateral rigid launcher. Axis-only launcher motion required for zero/2 mm is:

- +X: 11.916 / 14.164 mm;
- -X: 81.885 / 87.732 mm;
- +Z: 24.384 / 26.510 mm;
- -Z: 101.107 / 103.236 mm.

For the left pair, -Y requires 25.860 / 30.382 mm and +Y requires
97.908 / 101.508 mm; the right pair mirrors those directions. The best
symmetric X/Z-only 2 mm vector is `(12.507, 0, 4.552) mm`, magnitude
13.310 mm. It clears the flywheels but is rejected as a complete-package
solution because shifting the cradle forward creates bridge interference
outside the minimum motor opening.

Two millimetres is a valid geometric target but is mechanically marginal for
printed tolerances, assembly stack-up, pod compliance and flywheel runout. The
recommended +35 mm package gives about 10.0 mm conservative separating-plane
clearance at the motor/flywheel pair, without silently enlarging the clearance
requirement used for the other interfaces.

`MOTOR_FLYWHEEL_2MM_CLEARANCE_WITHOUT_LAUNCHER_MOVE = false`

## F. Rigid launcher translation design space

The exact coarse search covered X and Z from -40 to +40 mm in 10 mm steps at
Y=0. Every point was evaluated against both continuous direct-drive hardware
sweeps and both immutable flywheels, plus the launcher assembly against the
basket, hood, bridge, chassis, battery and LiDAR after only the permitted 2 mm
bridge and tyre-pocket relief envelopes.

The only coarse passing points were `(0,0,40)`, `(10,0,40)` and
`(20,0,40) mm`. Refinement at 2 mm and then 1 mm resolution found passing
points at Z=33 mm for X=-1, 0 and +1 mm. Every tested point at Z=32 mm failed.
The minimum-norm 1 mm-grid point is therefore `(0,0,33) mm`. No range extension
or launcher rotation was required.

The selected +Z family is preferable to the much smaller flywheel-only +X
translation because it keeps the centred launcher outlet X/Y datum and avoids
requiring a second launcher-shaped bridge cut.

`RIGID_LAUNCHER_TRANSLATION_SOLUTION_EXISTS = true`

## G. Combined candidates

### Local reliefs only

Bridge and tyre-pocket reliefs are geometrically possible, and a baseline
cradle relief is possible subject to structural review. This family fails
because each motor still intersects its same-side flywheel. Candidate A is not
accepted.

### MINIMUM_CHANGE

- launcher translation `(0,0,+33) mm`;
- 2 mm bridge motor opening: 13,726.790 mm3 removed;
- full-sweep 2 mm tyre-pocket extension: 8 mm outward travel envelope;
- 2 mm lower-cradle relief: 7,703.728 mm3;
- conservative motor/flywheel separation approximately 8.1 mm;
- all exact 2 mm hard-conflict probes empty;
- fabrication complexity: two bridge openings, two extended tyre pockets, two
  corner scallops in one cradle plate, launcher mount moved vertically;
- structural review required.

### PRACTICAL_MARGIN — recommended

- launcher translation `(0,0,+35) mm`;
- same bridge and tyre-pocket reliefs;
- 2 mm lower-cradle relief: 6,955.944 mm3;
- conservative motor/flywheel separation approximately 10.0 mm;
- all exact 2 mm hard-conflict probes empty;
- two additional millimetres above the grid-boundary solution;
- structural review required.

### ROBUST_MARGIN

- launcher translation `(0,0,+38) mm`;
- same bridge and tyre-pocket reliefs;
- 2 mm lower-cradle relief: 5,849.075 mm3;
- conservative motor/flywheel separation approximately 12.8 mm;
- all exact 2 mm hard-conflict probes empty;
- largest feeder/outlet displacement and therefore the greatest downstream
  integration burden;
- structural review required.

## H. Complete 0..8 mm swept validation

All selected candidates use the same exact continuous endpoint-hull intake
sweeps, not endpoint-only checks. For +33, +35 and +38 mm:

- motor/flywheel and wheel/flywheel intersections are zero;
- the 2 mm expanded direct-drive sweep has zero intersection with both
  flywheels;
- after the shaped 2 mm relief, motor/bridge and motor/cradle are clear;
- after the extended 2 mm tyre pocket, wheel/basket and wheel/hood are clear;
- 2 mm expanded wheel sweeps have zero exact intersection with ramp, cheeks,
  bridge, chassis and battery;
- the translated 2 mm launcher assembly envelope has zero intersection with
  the combined relieved basket, hood and bridge, or with chassis, battery and
  LiDAR;
- opposite intake pods retain zero intersection and the protected nip/travel
  are unchanged.

The Candidate A guide allocation is not part of this pass. At +35 mm its moving
allocation still intersects the cradle by 46,861.176 mm3 and each same-side
flywheel by 785.374 mm3; its fixed allocation intersects the cradle by
17,843.856 mm3. It must be replaced by a later support/guide design rather than
treated as manufacturing geometry.

## I. Structural geometry review

The bridge relief removes less than one percent of the broad portal solid but
leaves only a roughly 6.2 mm ligament to the nearest M5 service hole and about
10.0 mm to the upright's inner face in plan. The lower-cradle relief removes
about 1.1 percent of that plate's gross volume at +35 mm, but it is a full-depth
open-edge cut close to the vertical level of the wheel shaft supports. Volume
percentage alone is not a strength criterion.

No material properties, load cases, allowable stresses, plywood edge-distance
rules, bearing reactions, fatigue spectrum or real fastener pattern are
available. Accordingly:

`STRUCTURAL_REVIEW_REQUIRED = true`

## J. Launcher outlet-position impact

The analysis uses the end of the 220 mm exit-guide envelope as the outlet
reference. Baseline world position is approximately
`(760.702, 0, 324.446) mm`; the nip reference is `(460,0,215) mm`.

- +33 candidate: nip `(460,0,248)`, outlet `(760.702,0,357.446) mm`;
- +35 candidate: nip `(460,0,250)`, outlet `(760.702,0,359.446) mm`;
- +38 candidate: nip `(460,0,253)`, outlet `(760.702,0,362.446) mm`.

X, Y, pitch, flywheel spacing and nip geometry do not change. Relative to the
basket THROWING position, the launcher and its future breech/feed interface are
33, 35 or 38 mm higher than before. A future feeder must bridge that added
vertical offset. The rigid translation does not destroy the launcher concept,
but throwing validity and the basket-to-launcher feeder path are explicitly not
validated here.

## K. Decision matrix summary

All three passing candidates use a 33.966 x 20.975 x 18 mm maximum per-side
bridge relief envelope and an 8 mm outward extension of each existing tyre
pocket. Their designed minimum bridge, cradle, basket and hood clearances are
2 mm. Their launcher X/Y positions are unchanged.

- MINIMUM_CHANGE: +33 mm Z; 7,703.728 mm3 cradle relief; at least 8.1 mm
  motor/flywheel separation; outlet +33 mm; medium fabrication burden;
  structural review required.
- PRACTICAL_MARGIN: +35 mm Z; 6,955.944 mm3 cradle relief; at least 10.0 mm
  motor/flywheel separation; outlet +35 mm; medium fabrication burden;
  structural review required; recommended.
- ROBUST_MARGIN: +38 mm Z; 5,849.075 mm3 cradle relief; at least 12.8 mm
  motor/flywheel separation; outlet +38 mm; medium fabrication burden plus the
  largest future feeder change; structural review required.

## L. Recommended next mechanical change

In a separate implementation task, first subject the bridge and lower-cradle
relief shapes to structural/load-path review using the real launcher bearing
and fastener pattern. If that review accepts the local cuts, implement the
PRACTICAL_MARGIN family: rigidly raise the complete launcher by 35 mm, extend
the two approved tyre pockets through the full 8 mm compliance travel, and add
only the measured 2 mm bridge and lower-cradle reliefs. Then design a new pod
support/guide allocation around the cleared envelopes. Do not reuse the current
Candidate A allocation as manufacturing geometry.

Only after static manufactured geometry passes should the corrected 35 degree
handoff be revalidated dynamically. Do not begin the basket launch-path or ball
launch-physics study yet.

## Required classifications

`MOTOR_BRIDGE_RELIEF_GEOMETRICALLY_FEASIBLE = true`

`FULL_0_TO_8MM_TIRE_POCKET_EXTENSION_FEASIBLE = true`

`MOTOR_CRADLE_LOCAL_RELIEF_FEASIBLE = true`

`MOTOR_FLYWHEEL_2MM_CLEARANCE_WITHOUT_LAUNCHER_MOVE = false`

`RIGID_LAUNCHER_TRANSLATION_SOLUTION_EXISTS = true`

`MINIMUM_RIGID_LAUNCHER_TRANSLATION = (0, 0, +33 mm)`

`RECOMMENDED_RIGID_LAUNCHER_TRANSLATION = (0, 0, +35 mm)`

`DIRECT_DRIVE_INTAKE_ARCHITECTURE_REMAINS_VIABLE = true`

`CANDIDATE_A_GUIDE_REMAINS_VIABLE = false`

`STATIC_COMPACT_INTAKE_LAUNCHER_PACKAGING_SOLUTION_EXISTS = true`

`STRUCTURAL_REVIEW_REQUIRED = true`

`COMPACT_PHYSICAL_INTAKE_MODEL_COMPLETE = false`

`COMPACT_INTAKE_HANDOFF_REVALIDATED_IN_SIM = false`

`COMPACT_BASKET_LAUNCH_PATH_VALIDATED_IN_SIM = false`

`BALL_LAUNCH_PHYSICS_NOT_VALIDATED = true`

`PHYSICAL_HARDWARE_PENDING = true`

`LAUNCHER_ORIENTATION_ARCHITECTURAL_CHANGE_REQUIRED = false`
