# Compact PARKED geometry minimum-relief study
Date: 2026-08-25

## Decision

The PARKED compact configuration can be made boolean-valid without moving any
launcher or flywheel datum. The mathematical minimum is a set of local
subtractive reliefs equal to 78,112.957 mm3 of currently interpenetrating
material. A 2 mm fabrication-clearance version removes 92,869.327 mm3 and is
the recommended geometry candidate for a subsequent implementation task.

This is an analysis-only result. No authoritative basket, hood, intake,
launcher, Xacro, contract or generated measurement was changed.

## Exact conflict decomposition

### Hood versus launcher

The flywheels do not intersect the hood. The entire 6,799.188 mm3 conflict is
with the fixed launcher cradle plates:

| Hood component / launcher component | Volume (mm3) | Bounds (mm) |
|---|---:|---|
| roof / plates | 6,577.198 | (353.058,-90,132.235)..(370.986,90,140.755) |
| cheeks / plates | 221.990 | (354.419,-95,127.056)..(366.173,95,130.833) |
| mounts / launcher | 0 | clear |
| hood / launcher wheels | 0 | clear |

Required change: a shaped launcher-plate clearance pocket in only the hood
roof/front-cheek material. For the recommended 2 mm candidate its envelope is
(351.126,-95,125.124)..(370.986,95,141.000) mm and removes 8,780.435 mm3 before
accounting for its small overlap with the wheel pocket. The launcher remains
unchanged.

### Basket and hood versus intake

| Moving component / intake component | Volume (mm3) |
|---|---:|
| hood / wheels | 43,994.213 |
| bin / wheels | 5,324.145 |
| bin / compact ramp | 17,324.993 |
| hood / compact ramp | 414.419 |
| bin or hood / curved cheeks | 0 |
| total basket assembly / intake | 67,057.769 |

The wheel conflict decomposes further:

| Component | Wheel intersection (mm3) |
|---|---:|
| hood roof | 571.926 |
| hood side cheeks | 33,273.849 |
| hood mounts | 10,230.596 |
| bin receiving chute | 5,206.478 |
| bin walls | 117.667 |
| bin floor, management tray and front retention | 0 |

Required changes:

1. Cut two tire-shaped clearance pockets from the bin receiving chute/lower
   wall and hood roof/cheeks. For 2 mm clearance, use the existing wheel axes
   and centres with a 128 mm diameter and 77 mm axial envelope (nominal 124 x
   73 mm wheel plus 2 mm on every surface). Do not replace this with a large
   rectangular opening.
2. Trim the compact intake ramp against a 2 mm axial envelope of the retained
   basket/chute/hood surfaces. The removed ramp envelope is
   (319.650,-94,17.000)..(345.350,94,49.780) mm. The ramp's overall external
   bounds remain (319.650,-94,0)..(360.350,94,53) mm; the basket receiving
   chute remains the handoff surface where the two parts previously occupied
   the same volume.
3. Reroute the hood supports around the tire pockets. Simply subtracting the
   pocket from the existing supports would interrupt their load path. The
   nominal tire Y envelope reaches about |Y|=126.5 mm and Z=135.46 mm; the 2 mm
   keep-clear reaches |Y|=128.5 mm and Z=137.46 mm. Support posts should land
   on real side-strip structure outside these envelopes and outside the basket
   flange, with final fastener geometry subject to structural review. If a
   transverse member is retained, its lower surface must be above both the
   wheel and launcher keep-clear envelopes (at least Z=143 mm in this study).

The tire pockets preserve the wheel surfaces that contact the ball; they
remove only basket/hood material occupying the purchased wheel envelopes.

### Basket versus chassis

The 4,616 mm3 PARKED intersection remains decomposed as 360 mm3 intentional
flange engagement and 4,256 mm3 unintended lower-wall penetration.

Required change: notch only the lower bin walls against the chassis plate
envelope; do not move the basket, floor or flange. With 2 mm clearance the
relief envelope is (-86,-140,36.068)..(2,146,53.932) mm and removes
5,430.566 mm3. The candidate retains exactly 360 mm3 flange/chassis engagement.

## Candidate comparison

All values are exact OpenSCAD mesh volumes. “Removed” is split between bin,
hood and intake ramp, so the totals do not double-count different parts.

| Candidate | Clearance | Bin removed | Hood removed | Ramp removed | Total removed | Residual launcher | Residual intake | Residual chassis |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Mathematical minimum | 0 mm | 9,580.145 | 50,793.401 | 17,739.411 | 78,112.957 | numerical surface only | 1.292 numerical sliver | 360 intentional |
| Minimum positive | 1 mm | 10,506.978 | 53,633.021 | 22,144.313 | 86,284.312 | 0 | 0 | 360 intentional |
| Recommended | 2 mm | 11,473.981 | 55,768.361 | 25,626.985 | 92,869.327 | 0 | 0 | 360 intentional |
| Robust study | 3 mm | 12,484.856 | 57,811.826 | 28,478.047 | 98,774.729 | 0 | 0 | 360 intentional |

The 0 mm candidate is the strict material-removal minimum but is not
manufacturable because it relies on coincident surfaces. One millimetre passes
the mathematical booleans but offers little tolerance for wheel runout,
plywood/mesh fabrication and CAD-envelope error. Two millimetres is recommended
as the smallest useful study allowance. Three millimetres remains available if
measured wheel runout or assembly tolerance requires it.

## Recommended 2 mm relief envelopes

| Relief | Removed/intersecting volume (mm3) | Exact analysis bounds (mm) |
|---|---:|---|
| lower wall / chassis | 5,430.566 | (-86,-140,36.068)..(2,146,53.932) |
| bin / expanded wheel pair | 6,043.415 | (311.719,-140,32.604)..(370.597,143.313,79.395) |
| hood / expanded wheel pair | 47,053.250 | (330,-120.849,37.5)..(370,120.849,137.200) |
| hood / expanded launcher plates | 8,780.435 | (351.126,-95,125.124)..(370.986,95,141) |
| ramp / basket axial keep-clear | 25,626.985 | (319.65,-94,17)..(345.35,94,49.780) |

The hood wheel and launcher pockets overlap slightly; the authoritative hood
removal is their boolean union, 55,768.361 mm3, not their arithmetic sum.

## Preserved PARKED geometry

The recommended analysis candidate retains the authoritative basket origin and
the external PARKED bounds to within the existing tolerance:

| Measurement | Before | 2 mm candidate |
|---|---|---|
| basket minimum bounds | (-112,-172,19) mm | unchanged |
| basket maximum Y/Z | (172,285) mm | unchanged |
| basket maximum X | 370.986 mm | 370.597 mm |
| compact ramp external bounds | (319.65,-94,0)..(360.35,94,53) mm | unchanged |
| launcher/flywheel datums | authoritative baseline | unchanged |
| intentional flange/chassis engagement | 360 mm3 | 360 mm3 |

The 0.389 mm maximum-X change is the removal of conflicting hood material and
is below the 2 mm CAD tolerance. The bin receiving chute still reaches
X=370.597 mm and remains the functional ball-entry boundary. Curved cheeks,
wheel centres, ramp outer datum, launcher origin, cradle, flywheel centres,
diameter, width, nip and pitch are unchanged.

## Mechanical caveats and next validation

- The current hood is described as chassis-mounted, but its supports intersect
  the wheels. Their rerouted structural load path must be designed; collision
  pockets alone are not sufficient manufacturing geometry.
- Mesh-frame wires at pocket boundaries need termination/perimeter rods; the
  boolean volumes do not specify weld details.
- The ramp-to-chute transition must be checked with a 66 mm ball rolling/contact
  analysis after the duplicate solid volume is removed.
- Wheel runout, compliance and manufacturing tolerances must be measured before
  freezing 2 versus 3 mm clearance.
- These PARKED reliefs do not validate either the rearward-first guide path or
  the throwing pose. A new swept-volume study must use the relieved source.

Recommended classification:

- `PARKED_COMPACT_GEOMETRY_RELIEF_2MM_GEOMETRIC_PASS`: **true in analysis**.
- `PARKED_COMPACT_HOOD_SUPPORT_REDESIGN_REQUIRED`: **true**.
- `PARKED_COMPACT_HANDOFF_DYNAMIC_REVALIDATION_REQUIRED`: **true**.
- `LAUNCHER_FLYWHEEL_DATUMS_CHANGED`: **false**.
- `COMPACT_BASKET_LAUNCH_PATH_VALIDATED_IN_SIM`: **false**.
