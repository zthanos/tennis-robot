# Compact basket architecture correction

Date: 2026-08-25

## Result and scope

The current compact CAD and Xacro now model the intended basket architecture:
the existing bin and hood, exactly two fixed guide envelopes, and two pairs of
plywood post/shelf support bodies for the configured raised throwing study. The
surrounding metal carriage/cage is no longer instantiated by any current
compact packaging, integration, URDF, collision-contract, or generated-model
path. `cover-lift-study.scad` remains unchanged as a historical mechanism
study; it is not called by the current compact model.

This correction does not validate a launch path. The guide followers, the
holder insertion/withdrawal method, and any required latch or retention device
remain mechanical design work.

## Removed architecture and classification

The pre-edit lift study was decomposed before removal:

| Geometry | Classification | Disposition / reason |
|---|---|---|
| `bin()`, flange, walls, floor, handles and `hood()` | real basket geometry | retained unchanged |
| two vertical guide locations | real/inferred guide envelope | retained as exactly two guide members plus mounting feet |
| `sliding_blocks()` | uncertain hardware represented as cage blocks | removed from the authoritative model; follower/bearing selection is pending |
| `carriage_frame()` longitudinal beams and transverse crossbars | obsolete/fictitious cage | removed; they surrounded and carried the basket but are not part of the intended machine |
| cosmetic surrounding cover and pull handle | obsolete/fictitious cage | removed; not part of the intended bin/hood |
| cage locking pins, lever, counterbalance and actuator envelopes | obsolete or unselected mechanism | removed; no selected hardware justifies physical CAD/URDF bodies |
| guide follower/attachment hardware | uncertain | deliberately not invented; the current guides are constraint envelopes |
| holder engagement, withdrawal and retention | uncertain | explicitly pending; no latch, hook or actuator was invented |

The authoritative changes are in `compact-basket-support.scad`,
`compact-packaging-study.scad`, `robot-integration.scad`, the compact Xacro
components, generator, validation exporter/contract, and generated-model tests.

## Real support architecture

Coordinates below are in the CAD ground frame, in millimetres.

| Part | Geometry / bounds | Parent and material assumption | Mechanical role |
|---|---|---|---|
| Left guide | 28 x 22 mm envelope, Z 52..455; included in aggregate guide bounds (-75,-215,52)..(15,215,455) | fixed to chassis through a 90 x 70 x 12 mm foot; hollow aluminium envelope | constrains basket motion; not the final selected rail/bearing |
| Right guide | mirror of left guide | same | constrains basket motion |
| Left holder | plywood post plus 120 x 12 mm tilted shelf; aggregate holder bounds (65.111,-196,52)..(187.958,196,216.190) | post bears on the intact chassis side strip; 600 kg/m3 plywood | supports the left basket flange in the configured 100 mm + 12 degree pose |
| Right holder | mirror of left holder | same | supports the right basket flange |
| Basket | unchanged bin and hood; collection bounds (-112,-172,19)..(370.986,172,285) | moving along `basket_guide_path_link`; 1.20 kg empty plus 2.565 kg payload | collection container and launch feed geometry |

The guide assembly mass is 0.82 kg using a hollow-aluminium-plus-feet estimate.
The configuration-specific holders total 0.457 kg using the existing 600
kg/m3 plywood assumption. Both have explicit positive-definite inertias; no
near-zero placeholder body remains.

## Load path and support geometry

- **PARKED/COLLECTION:** the basket occupies its unchanged chassis/opening
  datum. Its flange intentionally bears at the chassis top in one region; the
  two guides constrain vertical/lateral motion. The actual follower attachment
  still requires detailing.
- **VERTICAL LIFT:** the diagnostic prismatic joint moves the basket 100 mm
  along the two-guide path. The joint represents actuation, while the guides
  represent the physical constraint envelope. No metal cage carries the load.
- **RAISED/SUPPORTED:** the two basket side flanges bear on the two tilted
  plywood shelf surfaces; posts transfer load into the chassis side strips.

Each shelf/flange contact patch is 120 x 12 = 1,440 mm2, for 2,880 mm2 total.
The basket COM projection at X = 143.4 mm lies inside the longitudinal support
span, with approximately 74.0 mm rear and 43.4 mm front margin. The nearest
lateral support edge is 160 mm from the centred COM. The shelves are exactly
tangent to the flange surfaces in CAD: the boolean intersection is 0 mm3,
which is the intended contact rather than penetration.

These bodies are a geometrically valid, parameterized support envelope, not a
strength-approved design. A real mechanism must withdraw or install the
holders during lift and may require a latch/stop against uplift or rebound.
Those details and fastener/edge-distance checks remain pending.

## Preserved collection and functional datums

The before and after values are identical:

| Datum | Before | After |
|---|---:|---:|
| Basket collection bounds | (-112,-172,19)..(370.986,172,285) mm | unchanged |
| Chassis bounds/opening source | (-460,-290,38)..(460,290,52) mm | unchanged |
| Basket/intake collection intersection | 67,057.769 mm3 | 67,057.769 mm3 |
| Bridge width | 490 mm | 490 mm |
| LiDAR scan plane | 0.498 m | 0.498 m |
| Operational vertical lift | 100 mm | 100 mm |

Intake wheels, compliant carriages, cheeks, handoff ramp, launcher origin,
cradle, flywheel dimensions/centres, nip and -20 degree pitch were not edited.
The bridge remains `GEOMETRIC_PASS_STRUCTURAL_REVIEW_REQUIRED`.

## Exact post-correction interference matrix

All nonzero values below are exact OpenSCAD boolean volumes from physical
modules, not bounding-box estimates.

| State / pair | Volume (mm3) | Classification |
|---|---:|---|
| fixed launcher / bridge | 145,093.431 | unresolved central CAD-source blocker |
| lowered basket / chassis | 4,616.000 | 360 flange bearing/engagement + 4,256 real bin-wall/chassis interference |
| lowered basket / bridge | 0 | clear |
| lowered basket / intake | 67,057.769 | unchanged intended collection/handoff overlap requiring contact interpretation |
| lowered basket / battery | 0 | clear |
| guides / chassis | 0 | surface-supported, no penetration |
| guides / intake | 0 | clear |
| raised basket / bridge | 2,240.000 | unresolved |
| raised basket / launcher | 12,419.021 | unresolved |
| raised hood / launcher | 5,173.389 | unresolved |
| guides / bridge | 0 | clear |
| guides / launcher | 0 | clear |
| 12-degree basket / bridge | 1,053.233 | unresolved |
| 12-degree basket / launcher | 21,950.876 | unresolved |
| 12-degree hood / launcher | 15,281.463 | unresolved |
| 12-degree basket / holders | 0 | exact tangent support contact |
| launch basket / battery | 0 | clear |
| launch basket / LiDAR | 0 | clear |
| holders / chassis | <0.000001 | surface contact |
| holders / bridge or launcher | 0 | clear |

The former reported 6,799.19 mm3 “launcher / basket hood” value was rerun and
traced to a lowered-hood selector. It is not the 12-degree launch-pose result.
The correct current launch-pose hood value is 15,281.463 mm3. This is a
measurement-label correction, not a regression caused by removing the cage.

The 4,616 mm3 lowered basket/chassis value did not disappear with the cage.
Decomposition proves that 360 mm3 is flange/chassis parked engagement, 4,256
mm3 is real basket-wall/chassis interference, and the basket floor contributes
0. It is therefore not obsolete-cage overlap and the collection datum was not
moved to conceal it.

The launcher/bridge blocker is unchanged. Its exact intersection bounds are
(377.881,-201.564,150)..(468.287,201.564,168) mm, confirming a central overlap
rather than a bridge-edge width effect. No launcher or bridge cut-out was made.

## CAD, URDF and Gazebo validation

The generated URDF uses `basket_guide_path_link` for the moving path and
`basket_rails_link` for the two fixed guides. The deleted cage link and its
collision beams/blocks are absent. In the generated SDF the fixed basket body
is reduced onto the moving guide-path link, leaving no floating massless cage.
Holder bodies are enabled only in the explicitly configured supported pose,
because their real engagement mechanism is not yet designed.

All contract geometry passes the 2 mm CAD/URDF tolerance. The largest error is
0.982 mm at the triangulated launch basket; guides are exact and holder error is
0.0005 mm. Aggregate mass/COM results are 32.340 kg and
(0.045685,-0.002241,0.112434) m lowered, the same mass and
(0.045685,-0.002241,0.124076) m raised, and 32.797 kg including holders with
(0.049517,-0.002209,0.129743) m in the configured supported study. The holder
link COM is (0.127958,0,0.118094) m in the ground frame. Minimum
support-polygon margin is 0.365 m.

An isolated ROS 2 Jazzy/Gazebo build and headless spawn succeeded. Clock,
joint-state and controller health passed; intake actuation moved 0.56 rad and
stopped. The corrected basket completed two vertical diagnostic cycles with
4/4 transitions: 96.42 and 97.34 mm raised, 2.68 and 2.40 mm returned, at 120
mm/s peak tracking and without retries. The guide geometry is present in the
live model; the holder geometry is present in the generated configured launch
URDF/SDF, but is not engaged during the live collection/lift diagnostic because
the insertion mechanism is unresolved.

## Tests and status

- CAD exporter and mechanical-contract regeneration: passed.
- Exact boolean export and SAT audit: ran; SAT correctly reports the documented
  source blockers.
- Focused generated-model/mechanical tests: 29 passed.
- Isolated ROS package build and Gazebo sanity/lift cycle: passed.
- Full ROS-aware repository suite: 898 passed, 2 skipped.

Final capability status:

- `COMPACT_BASKET_ARCHITECTURE_TRUTHFUL`: **validated for the current CAD/Xacro envelope**.
- `COMPACT_COLLECTION_GEOMETRY_UNCHANGED`: **validated**.
- `COMPACT_BASKET_SUPPORT_STRENGTH_VALIDATED`: **false**.
- `COMPACT_BASKET_HOLDER_ENGAGEMENT_DESIGNED`: **false**.
- `COMPACT_BASKET_LAUNCH_PATH_VALIDATED_IN_SIM`: **false**.
- `PHYSICAL_HARDWARE_PENDING`: **true**.
