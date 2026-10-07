# Compact basket rearward-first two-guide design study
Date: 2026-08-25

## Study outcome

The proposed rearward-first mechanism is **not geometrically valid under the
stated invariants and acceptance rule**. No authoritative CAD, Xacro, URDF,
controller, Throwing Mode, contract, or generated measurement was changed.

The analysis-only wrapper
`cad/flywheel-launcher-v0/compact-basket-path-validation.scad` was extended
with a parameterized rear opening. It preserves the 920 x 580 x 14 mm chassis,
X=460 mm opening front, Y=+/-150 mm opening sides and every other fixed datum;
only the permitted rear edge is varied.

## A. Baseline

The failed vertical-first measurements were reconfirmed:

| Relationship | Exact intersection (mm3) |
|---|---:|
| PARKED basket/chassis | 4,616.000 total = 360 flange + 4,256 walls |
| +20 mm vertical basket/chassis | 48,864.000 |
| +35 mm vertical basket/launcher | 1,325.862 |
| +35 mm vertical basket/intake | 72,930.016 |
| +35 mm vertical basket/bridge | 18.667 |
| existing 12-degree basket/launcher | 21,950.876 |
| existing 12-degree hood/launcher | 15,281.463 |
| existing raised basket/bridge | 2,240.000 |
| existing 12-degree basket/bridge | 1,053.233 |
| fixed launcher/bridge | 145,093.431 |

The 6,799.188 mm3 value is not used as a launch-pose result. Exact analysis
does show that it is the physical PARKED hood/launcher intersection, which is
decisive for the continuous swept-path acceptance criterion.

## B. Rear chassis-opening sweep

The original opening rear edge is X=10 mm. An extension E moves it to X=10-E.
At PARKED, exact wall and flange intersections are:

| Extension (mm) | Rear edge X (mm) | Wall/chassis (mm3) | Flange/chassis (mm3) | Rear material length (mm) | Gap to battery front (mm) |
|---:|---:|---:|---:|---:|---:|
| 0 | 10 | 4,256 | 360 | 470 | 182 |
| 10 | 0 | 4,032 | 360 | 460 | 172 |
| 20 | -10 | 3,808 | 360 | 450 | 162 |
| 30 | -20 | 3,808 | 360 | 440 | 152 |
| 40 | -30 | 3,808 | 360 | 430 | 142 |
| 50 | -40 | 3,584 | 360 | 420 | 132 |
| 60 | -50 | 3,360 | 360 | 410 | 122 |
| 70 | -60 | 3,360 | 180 | 400 | 112 |
| 80 | -70 | 3,360 | 0 | 390 | 102 |
| 85 | -75 | 3,192 | 0 | 385 | 97 |
| 90 | -80 | 2,352 | 0 | 380 | 92 |
| 95 | -85 | 168 | 0 | 375 | 87 |
| 96 | -86 | 0 | 0 | 374 | 86 |

Thus none of the required 0--80 mm candidates removes the genuine PARKED
wall interference. The mathematical PARKED minimum is 96 mm. The loss of
positive flange intersection at 80 mm does not mean the basket floats: the
side flange and side strips remain tangent support surfaces, but their load
area/fasteners would require a new structural design.

For a zero-lift horizontal basket displacement R, the measured envelope gives
the minimum opening extension approximately E=96+R mm. The side strips remain
140 mm wide because opening Y is unchanged. Rear material length is 470-E mm,
and the central X gap between the cut and the fixed battery front is 182-E mm.
At E=159 only 23 mm remains before the battery envelope; at E=175 only 7 mm
remains. No plywood edge-distance or fastener requirement exists, so even a
geometrically clear cut would be
`GEOMETRIC_PASS_STRUCTURAL_REVIEW_REQUIRED`.

## C. Horizontal rearward sweep

The complete bin and hood move together at Z=0 and pitch=0. The chassis result
below assumes a sufficiently extended analysis opening; no authoritative cut
was made.

| Rearward (mm) | Basket/launcher | Basket/intake | Basket/bridge | Basket/battery |
|---:|---:|---:|---:|---:|
| 0 | 6,799.188 | 67,057.769 | 0 | 0 |
| 5 | 4,269.195 | 58,438.703 | 0 | 0 |
| 10 | 2,026.811 | 46,329.356 | 0 | 0 |
| 15 | 142.046 | 39,698.415 | 0 | 0 |
| 20 | 0 | 34,483.521 | 0 | 0 |
| 30 | 0 | 23,580.059 | 0 | 0 |
| 40 | 0 | 12,666.601 | 0 | 0 |
| 50 | 0 | 5,196.657 | 0 | 0 |
| 60 | 0 | 1,444.629 | 0 | 0 |
| 61 | 0 | 1,207.293 | 0 | 89.493 |
| 62 | 0 | 991.242 | 0 | 460.007 |
| 63 | 0 | 796.473 | 0 | 1,009.784 |
| 65 | 0 | 470.787 | 0 | 2,412.224 |
| 70 | 0 | 29.035 | 0 | 6,288.678 |
| 71 | 0 | 4.535 | 0 | 7,065.050 |
| 72 | 0 | 0 | 0 | 7,841.422 |
| 75 | 0 | 0 | 0 | 10,170.537 |
| 80 | 0 | 0 | 0 | 14,052.396 |

The launcher clears during horizontal withdrawal by 20 mm sampling. Battery
contact starts between 60 and 61 mm, while intake does not clear until 72 mm.
There is no purely horizontal state that is simultaneously clear of intake and
battery. A smooth upward transition would therefore have to start before
battery contact and clear the intake through combined X/Z motion.

That observation alone does not disprove a curved transition. The strict
full-path entrance condition does, as described next.

## D. Feasibility proof and transition decision

At the invariant s=0 PARKED pose, the moving physical assembly has positive
intersection with two unchanged fixed assemblies:

- hood/launcher = 6,799.188 mm3;
- basket/intake = 67,057.769 mm3.

For rigid solids under a continuous guide transform, intersection volume is
continuous near the initial pose. A positive unintended intersection at s=0
therefore remains positive for a nonzero interval after motion starts. Moving
only the rear edge of the chassis opening cannot affect either relationship.

The task requires maximum unintended intersection=0 mm3 over the complete
s=0..1 forward path and preserves PARKED, intake, handoff and launcher datums.
Those conditions cannot all be true. In particular, the PARKED
hood/launcher overlap is not an allowed support contact.

Consequently no valid `x_rear_geometric_min`, `x_transition_start`, transition
radius or vertical travel can be selected. Searching arcs or splines after an
already-invalid entrance cannot produce a passing complete swept path, so the
study stops before proposing guide fabrication geometry.

## E. Diagnostic boundary candidates

No viable region was found, so the requested Candidate A/B/C comparison cannot
honestly contain three passing designs. The closest boundary cases are shown
only to quantify why increasing the cut/retraction is not a solution:

| Diagnostic case | Opening E (mm) | Horizontal R (mm) | Intake intersection (mm3) | Battery intersection (mm3) | Forward/reverse pass | Structural classification |
|---|---:|---:|---:|---:|---|---|
| A, smallest earlier endpoint threshold | 159 | 62.4 | 910.780 | 662.591 | fail / fail | rejected; review required |
| B, rounded practical trial | 165 | 65 | 470.787 | 2,412.224 | fail / fail | rejected; review required |
| C, intake-clear trial | 175 | 75 | 0 | 10,170.537 | fail / fail | rejected; only 7 mm battery-edge gap |

All three also inherit the invalid positive intersections at s=0. Minimum
positive clearance values are therefore not defined; each case has penetration
rather than clearance. Transition radius and vertical travel are deliberately
listed as “not selected,” not silently guessed.

## F. Guides, orientation, holders and reverse path

Exactly two mirrored continuous guides remain a kinematically plausible
one-DOF concept in isolation. Preventing yaw/roll/racking would require at least
defined follower spacing and likely two longitudinal constraint points per
side; that hardware is pending and no surrounding cage is implied.

The 12-degree final orientation cannot be assigned to the guides without
follower/interface geometry. `FINAL_BASKET_ORIENTATION_MECHANISM_PENDING` and
`RAISED_BASKET_RETENTION_HARDWARE_PENDING` remain true. Existing plywood
holders were not moved to a rejected endpoint.

Reverse motion traverses the same invalid near-PARKED poses, so reverse
acceptance fails independently of actuation direction.

## G. Stability implication

No valid trajectory exists, so a path-wide stability claim is not made. As a
diagnostic upper bound only, moving the existing 3.765 kg loaded basket 75 mm
rearward shifts the approximately 32.34 kg robot COM about 8.7 mm rearward.
Starting from X=45.7 mm, this remains well inside the +/-415 mm longitudinal
support interval. Stability is not the limiting issue; physical interference
and the opening/battery structure are.

## H. Recommendation and capability classification

Do not implement a unified rearward-first guide from the current geometry.
Before another path study, explicitly authorize and revalidate at least the
PARKED hood/launcher relationship and decide whether the large basket/intake
boolean is a physically valid disengaging interface or invalid overlapping
solid geometry. Only then is a 96+ mm rear opening and a curved X/Z search
meaningful. Battery/service structure must also be reviewed before considering
cuts near E=159--175 mm.

- `REARWARD_FIRST_GUIDE_PATH_GEOMETRICALLY_VALID`: **false**.
- `CHASSIS_OPENING_STRUCTURAL_REVIEW_REQUIRED`: **true for every potentially useful extension**.
- `FINAL_BASKET_ORIENTATION_MECHANISM_PENDING`: **true**.
- `RAISED_BASKET_RETENTION_HARDWARE_PENDING`: **true**.
- `LAUNCHER_BRIDGE_FIXED_INTERFERENCE_PENDING`: **true**.
- `COMPACT_BASKET_LAUNCH_PATH_VALIDATED_IN_SIM`: **false; not part of this study**.
