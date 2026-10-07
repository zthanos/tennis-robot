# Compact basket guided-path measurement — STOP report
Date: 2026-08-25

## Decision

No guided launch trajectory was implemented in CAD, Xacro, controllers, or
Gazebo. Exact post-reconstruction measurements prove that the required initial
vertical segment cannot traverse the current fixed geometry. Satisfying the
task would require at least one prohibited change to the parked basket/hood,
chassis opening, intake handoff, or launcher.

This is therefore a deliberate STOP-condition result, not a claim that the
launch path is validated.

## A. Before-state

| Pair | Exact intersection (mm3) |
|---|---:|
| launch basket / launcher | 21,950.876 |
| launch hood / launcher | 15,281.463 |
| raised basket / bridge | 2,240.000 |
| launch basket / bridge | 1,053.233 |
| parked basket / chassis, total | 4,616.000 |
| guides / chassis, intake, bridge or launcher | 0 |
| fixed launcher / bridge | 145,093.431 |

The historical 6,799.19 mm3 hood/launcher result is not used as a launch-pose
value. It is, however, independently reproduced at the PARKED pose and is
important to the swept-volume diagnosis below.

## B. PARKED basket/chassis diagnosis

The parked 4,616 mm3 intersection decomposes exactly as follows:

| Participating basket surface | Volume (mm3) | Bounds (mm) | Disposition |
|---|---:|---|---|
| support flange | 360 | aggregate parked bounds extend to (-86,-172,38)..(10,172,52) | intentional parked engagement |
| bin rear/side wall envelopes | 4,256 | (-86,-140,38)..(2,146,52) | genuine physical interference |
| bin floor | 0 | none | clear at PARKED |

The chassis opening is X=10..460 mm and Y=+/-150 mm. After the authoritative
-100 mm compact shift, the bin rear wall is near X=-80 mm. Consequently roughly
90 mm of the rear portion lies behind the opening's X=10 mm rear edge. This is
not a cage artifact or a removable nonfunctional detail.

Result: `PARKED_BASKET_CHASSIS_MECHANICAL_BLOCKER`.

## C. Mandatory vertical segment

The parameterized wrapper
`compact-basket-path-validation.scad` transforms the complete bin and hood as
one moving assembly. Exact booleans at zero retraction and zero tilt give:

| Lift (mm) | Basket/chassis | Basket/bridge | Basket/launcher | Hood/launcher | Basket/intake |
|---:|---:|---:|---:|---:|---:|
| 0 | 4,616.000 | 0 | 6,799.188 | 6,799.188 | 67,057.769 |
| 5 | 4,256.000 | 0 | 1,452.442 | 1,452.442 | 55,992.618 |
| 10 | 11,624.000 | 93.333 | 1,325.862 | 1,325.862 | 49,879.901 |
| 15 | 27,728.000 | 653.333 | 1,325.862 | 1,325.862 | 50,532.105 |
| 20 | 48,864.000 | 896.000 | 1,325.862 | 1,325.862 | 53,237.753 |
| 25 | 37,520.000 | 896.000 | 1,325.862 | 1,325.862 | 58,653.990 |
| 30 | 16,000.000 | 578.667 | 1,325.862 | 1,325.862 | 68,113.024 |
| 35 | 0 | 18.667 | 1,325.862 | 1,325.862 | 72,930.016 |
| 40 | 0 | 0 | 1,325.862 | 1,325.862 | 72,110.100 |
| 50 | 0 | 0 | 1,325.862 | 1,325.862 | 66,457.664 |
| 70 | 0 | 0 | 1,754.866 | 1,754.866 | 36,991.199 |
| 90 | 0 | 0 | 15,016.890 | 14,950.332 | 3,977.368 |
| 100 | 0 | 2,240.000 | 12,419.021 | 5,173.389 | 246.929 |

The worst chassis crossing occurs at 20 mm lift. It consists of 16,864 mm3 of
wall/chassis penetration and 32,000 mm3 of floor/chassis penetration, with
bounds (-86,-140,39)..(10,146,52) mm. This proves that the moving basket does
not pass through the established opening.

For the chassis alone, the exact sampled geometric-clear height is 35 mm.
Allowing a 5 mm design margin would suggest 40 mm, but **40 mm is not a valid
`z_retraction_start`**: at that height launcher and intake intersections are
still 1,325.862 and 72,110.100 mm3 respectively. There is no
`z_geometric_clear` satisfying all required fixed structures anywhere on the
mandatory 0..100 mm vertical segment. Therefore no design value is selected.

## D. Endpoint-only rearward sweep

For diagnosis only, the existing 100 mm + 12 degree endpoint was translated
rearward while preserving every fixed datum:

| Retraction (mm) | Basket/launcher | Hood/launcher | Basket/bridge |
|---:|---:|---:|---:|
| 0 | 21,950.876 | 15,281.463 | 1,053.233 |
| 2 | 19,366.046 | 13,617.663 | 1,053.233 |
| 4 | 16,716.770 | 12,009.816 | 1,053.233 |
| 6 | 14,177.098 | 10,556.944 | 1,053.233 |
| 8 | 11,791.131 | 9,227.323 | 1,053.233 |
| 10 | 9,632.923 | 8,015.718 | 1,053.233 |
| 12 | 7,782.518 | 6,903.606 | 1,053.233 |
| 14 | 6,299.097 | 5,942.371 | 1,053.233 |
| 16 | 5,310.981 | 5,286.102 | 1,053.233 |
| 18 | 4,934.833 | 4,934.833 | 1,047.205 |
| 20 | 4,807.827 | 4,807.827 | 1,029.274 |
| 25 | 4,442.845 | 4,442.845 | 932.370 |
| 30 | 3,951.548 | 3,951.548 | 761.071 |
| 35 | 2,879.100 | 2,879.100 | 515.377 |
| 40 | 1,879.189 | 1,879.189 | 229.198 |
| 45 | 1,403.707 | 1,403.707 | 0 |
| 50 | 942.343 | 942.343 | 0 |
| 55 | 480.978 | 480.978 | 0 |
| 60 | 60.278 | 60.278 | 0 |
| 61 | 20.261 | 20.261 | 0 |
| 62 | 1.534 | 1.534 | 0 |
| 62.1 | 0.832 | 0.832 | 0 |
| 62.2 | 0.343 | 0.343 | 0 |
| 62.3 | 0.067 | 0.067 | 0 |
| 62.4 | 0 | 0 | 0 |
| 65 | 0 | 0 | 0 |
| 70 | 0 | 0 | 0 |

The endpoint-only mathematical threshold is 62.4 mm at 0.1 mm sampling, not
10--20 mm. At 58--75 mm the endpoint basket also has zero boolean intersection
with chassis, battery, LiDAR and intake. This does not establish clearance or a
valid swept path, so no “selected design retraction” or clearance margin is
claimed.

## E. Path, guides, holders and orientation

No path was selected. A smooth two-guide curve could geometrically connect a
vertical lower segment to a rearward-offset upper segment, but the moving body
would already be passing through chassis, launcher and intake solids before
that curve can legally begin. Adding shaped guides cannot remove those solid
intersections.

The 12 degree final orientation is therefore neither accepted nor rejected by
this task. It remains the protected functional intent pending resolution of the
lower-path contradiction. No tilt actuator, horizontal actuator, guide
hardware, latch or cage was introduced. Existing holders were not repositioned
to an unvalidated endpoint.

`RAISED_BASKET_RETENTION_HARDWARE_PENDING` remains true.

## F. Swept volume and collision classification

Forward validation fails during the mandatory initial vertical segment.
Reverse validation necessarily traverses the same geometric poses and also
fails. The failure is not caused by numerical directionality.

- The 360 mm3 parked flange overlap remains
  `INTENTIONAL_SUPPORT_CONTACT`.
- The 4,256 mm3 parked wall overlap and the larger wall/floor crossings during
  lift are `UNINTENDED_INTERFERENCE`.
- Basket/intake overlap at collection is part of the preserved handoff datum,
  but its very large persistence during lift requires a mechanical contact/
  withdrawal decision before it can be accepted as a swept path.
- Basket/launcher penetration is unintended and nonzero at every sampled
  point of the mandatory vertical segment.
- `LAUNCHER_BRIDGE_FIXED_INTERFERENCE_PENDING` remains independently
  145,093.431 mm3 and was excluded from the basket trajectory calculation.

Because the static/swept CAD gate fails, CAD-to-Xacro implementation, dynamic
mass/COM trajectory validation, Gazebo cycling, holder engagement testing and
collection regression were correctly not started.

## G. Preserved state and required decision

No protected datum was changed: PARKED basket, hood, intake, handoff, chassis
opening, 490 mm bridge, launcher/flywheels, battery and 0.498 m LiDAR plane all
remain as before. No Xacro/controller change was made.

Measured options requiring explicit mechanical authorization are:

1. revise the chassis opening/rear support topology so the basket floor and
   rear wall can pass vertically;
2. revise the basket/hood collection geometry while preserving the functional
   ball handoff by a newly validated design;
3. revise the fixed launcher/hood relationship; or
4. reject the strictly vertical-first requirement and define a different
   physical extraction mechanism, which would also require revalidating the
   opening constraint.

Until one option is authorized:

- `PARKED_BASKET_CHASSIS_MECHANICAL_BLOCKER`: **true**.
- `COMPACT_BASKET_LAUNCH_PATH_VALIDATED_IN_SIM`: **false**.
- Physical guide/holder strength and retention validation: **not performed**.
- Throwing Mode E2E and ball-launch physics validation: **not performed**.
