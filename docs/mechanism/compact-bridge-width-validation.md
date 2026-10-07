# Compact plywood bridge width change

Date: 2026-08-25

## Decision

The approved bridge-width datum is now 490 mm, centred at Y = 0. The change is
geometrically supported by the existing chassis side strips and creates the
requested 10 mm additional edge allowance on both sides. It does not resolve
the central launcher/bridge solid intersection; that blocker is independent of
the bridge outer edge. Structural adequacy remains review-required because no
minimum plywood overhang, fastener edge distance, or support-overlap requirement
is defined in the CAD or mechanical documentation.

## CAD source audit

| Path | Classification and role | Disposition |
|---|---|---|
| `cad/collector-intake-v1/option-a/bridge-params.scad` | authoritative bridge datums | added; owns the 490 mm width |
| `cad/collector-intake-v1/option-a/option-a.scad` | authoritative manufacturing-intent `plywood_bridge()` generator | updated to include shared datums |
| `cad/flywheel-launcher-v0/compact-packaging-study.scad` | authoritative compact integration and bridge cut-outs | updated to derive half-width from shared datum |
| `cad/flywheel-launcher-v0/robot-integration.scad` | current integration view importing `plywood_bridge()` | no independent dimension; automatically updated |
| `cad/flywheel-launcher-v0/compact-validation-export.scad` | derived deterministic export/boolean view | updated and regenerated |
| `config/compact_cad_measurements.json` | derived CAD measurements | regenerated |
| `config/compact_mechanical_contract.json` | generated-model contract | synchronized |
| `docs/mechanism/compact-cad-urdf-alignment-el.md` | historical pre-reconstruction analysis | intentionally unchanged |
| `docs/archive/mechanism/flywheel-launcher/flywheel-launcher-exploration-el.md` | historical design exploration | archived unchanged |

Repository search found no second current bridge solid generator and no stated
minimum support/edge-overlap requirement.

## Geometry and unchanged datums

| Datum | Before | After | Change |
|---|---:|---:|---:|
| Overall width | 470 mm | 490 mm | +20 mm |
| Left outer edge | +235 mm | +245 mm | +10 mm |
| Right outer edge | -235 mm | -245 mm | -10 mm |
| Centre | 0 mm | 0 mm | none |
| X envelope after compact shift | 270..500 mm | 270..500 mm | none |
| Z envelope | 52..168 mm | 52..168 mm | none |

The rear notch remains centred at Y = 0 and unchanged at +/-165 mm. Its side
lands therefore grow from 70 to 80 mm; the notch, motor arches, upright centres,
doublers, cheek mounts, launcher origin, flywheel centres/gap/pitch, basket lift
axis, basket pivot, and intake datums do not move.

## Base and support fit

The compact shift places each 18 mm plywood upright over the chassis from
X = 285 to 460 mm, giving 175 mm longitudinal contact. The last 35 mm of the
upright extends beyond the chassis front edge and is not counted as support.

| Measurement | Left | Right |
|---|---:|---:|
| Chassis/base outer Y | +290 mm | -290 mm |
| Chassis intact side strip | +170..+290 mm | -290..-170 mm |
| Upright contact range | +196..+214 mm | -214..-196 mm |
| New bridge outer edge | +245 mm | -245 mm |
| Bridge-to-chassis outer-edge margin | 45 mm | 45 mm |
| Upright contact width | 18 mm | 18 mm |
| Contact length | 175 mm | 175 mm |
| Nominal contact area per upright | 3,150 mm2 | 3,150 mm2 |
| Upright clearance to side-strip inner edge | 26 mm | 26 mm |
| Upright clearance to chassis outer edge | 76 mm | 76 mm |
| Top overhang beyond upright outer face | 31 mm | 31 mm |

Assessment: **GEOMETRIC PASS / STRUCTURAL REVIEW REQUIRED**. Both uprights retain
their full 18 mm lateral bearing width on intact chassis material, and the
bridge remains inside the base by 45 mm per side. This is geometric support
evidence, not a plywood stress or fastener pull-out validation.

## Lateral allowance

| Interface, measured to bridge edge | Before per side | After per side | Gain |
|---|---:|---:|---:|
| Flywheel outer envelope, Y = +/-229 mm | 6 mm | 16 mm | 10 mm |
| Basket outer envelope, Y = +/-172 mm | 63 mm | 73 mm | 10 mm |
| Cradle plate envelope, Y = +/-129 mm | 106 mm | 116 mm | 10 mm |

Left and right results are symmetric. The intended side allowance was achieved.
These are edge allowances; they do not affect central X/Z intersections.

## Exact OpenSCAD interference audit

| Pair/state | 470 mm bridge | 490 mm bridge | Result |
|---|---:|---:|---|
| Physical launcher vs bridge | 145,093.43 mm3 | 145,093.43 mm3 | pre-existing, unchanged |
| Basket/hood vs bridge, lowered | 0 mm3 | 0 mm3 | clear |
| Basket vs bridge, raised 100 mm | 2,240.00 mm3 | 2,240.00 mm3 | pre-existing, unchanged |
| Basket vs bridge, raised + 12 degrees | 1,053.23 mm3 | 1,053.23 mm3 | pre-existing, unchanged |
| Bridge vs cheeks | <0.000001 mm3 | <0.000001 mm3 | coincident intended interface, zero volume |
| Bridge vs chassis | 0 mm3 | 0 mm3 | surface contact only |
| Launcher vs basket hood | 6,799.19 mm3 | 6,799.19 mm3 | historical lowered-hood selector; see basket architecture correction |

The all-parts audit also confirms an unrelated, pre-existing 4,616.00 mm3
lowered-basket/chassis intersection. It was added to the contract and SAT gate;
it was not modified as part of this width-only decision.

## CAD to URDF and physics impact

| Measurement | Result |
|---|---:|
| CAD bridge bounds | (270,-245,52) to (500,245,168) mm |
| URDF bridge bounds | (270,-245,52) to (500,245,168) mm |
| Maximum bridge deviation | <0.001 mm, PASS |
| Bridge mass | 1.14 -> 1.19 kg |
| Total robot mass | 32.34 kg after obsolete basket cage removal; 32.797 kg with configured holders |
| Lowered COM | (0.04569,-0.00224,0.11243) m |
| Raised COM | (0.04569,-0.00224,0.12408) m |
| Launch-study COM with holders | (0.04952,-0.00221,0.12974) m |
| Minimum stability margin | 0.3655 m, stable |

LiDAR remains at 0.498 m, lift travel remains 100 mm, and launcher/flywheel
datums and dimensions are unchanged.

## Acceptance status

The bridge-width change itself is complete: all current CAD views agree, the
base fit has measured symmetric support, no new interference was introduced,
and CAD/URDF bounds agree. The overall compact physics model remains blocked by
the explicitly reported pre-existing CAD intersections.

Validation results: deterministic current and 470 mm baseline CAD exports
completed; generated URDF/SDF and SAT validation completed with the documented
source blockers; focused mechanical tests passed 28/28; the full ROS-aware test
suite now passes 898 with 2 skipped after the basket architecture correction.
