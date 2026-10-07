# Compact PARKED local-relief implementation report

Date: 2026-08-25

## Outcome

The approved 2 mm local reliefs are implemented in the shared compact CAD and
propagated to collision-bearing Xacro/URDF meshes. Exact OpenSCAD PARKED
booleans pass, including independent left/right wheel checks, and the intended
flange/chassis engagement remains 360 mm3.

The dynamic intake regression fails at the ramp/chute handoff before either
intake wheel contacts the ball. Per the task stop rule, no datum was moved and
no compensating geometry was introduced. Consequently
`COMPACT_PARKED_PACKAGING_VALIDATED_IN_SIM` is **not justified**.

## Geometry implemented

- Two tire-axis pockets: diameter 128 mm, axial width 77 mm, centred on the
  existing wheel centres at compact-local `(470,+/-90,70) mm`, with the
  existing 35 degree axes.
- Shaped launcher-plate pocket in the hood, generated from the real two plate
  solids expanded by 2 mm. Approved measured envelope:
  `(351.126,-95,125.124)..(370.986,95,141.000) mm`.
- Local ramp trim against the retained basket/hood with 2 mm clearance.
  Removed envelope: `(319.650,-94,17.000)..(345.350,94,49.780) mm`.
- Lower bin-wall-only chassis relief. Approved envelope:
  `(-86,-140,36.068)..(2,146,53.932) mm`.
- The removable bin, chassis-fixed hood/support portal, and relieved ramp are
  exported from the authoritative CAD into the URDF as real collision meshes;
  the intake-wheel collision cylinders are unchanged.

Authoritative source: `cad/flywheel-launcher-v0/compact-parked-reliefs.scad`.
Current derived views use that source through `compact-packaging-study.scad`,
`robot-integration.scad`, and `compact-validation-export.scad`. Generated
measurements and contract are in `config/compact_cad_measurements.json` and
`config/compact_mechanical_contract.json`.

## Hood support route

The old intersecting mounts are replaced by a chassis-fixed portal at compact
local X=430 mm:

- 8 x 8 mm posts land at Y=+/-184 mm on the chassis side strips, Z=52..148 mm;
- an 8 mm deep, 6 mm tall transverse member spans Y=-188..188 mm at
  Z=142..148 mm;
- two 8 x 8 mm hangers at Y=+/-35 mm connect the portal to the hood roof.

Load path: hood roof -> central hangers -> transverse portal -> outboard posts
-> chassis side strips. Exact support intersections with the expanded tires,
launcher, bridge and removable basket are all 0 mm3. The post feet make
surface contact with the chassis at Z=52 mm without penetrating it.

Classification: `GEOMETRIC_PASS_STRUCTURAL_REVIEW_REQUIRED`. Fasteners,
joint detailing, section stress, vibration and fatigue have not been sized.

## Exact PARKED intersections

Before values come from the approved analysis; after values are regenerated
from the authoritative implementation.

- hood/launcher: 6,799.188 -> 0 mm3
- basket+hood/intake: 67,057.769 -> 0 mm3
- left basket/nominal wheel: -> 0 mm3
- right basket/nominal wheel: -> 0 mm3
- unintended bin walls/chassis: 4,256 -> 0 mm3
- bin floor/chassis: 0 -> 0 mm3
- intentional flange/chassis: 360 -> 360 mm3
- full PARKED basket+hood/launcher: 0 mm3 after implementation
- full PARKED basket+hood/battery: 0 mm3
- hood supports versus wheels/launcher/bridge/basket: 0 mm3 each

The independent fixed launcher/bridge conflict remains 145,093.431 mm3 and is
not part of these reliefs: `LAUNCHER_BRIDGE_FIXED_INTERFERENCE_PENDING`.

## Datum preservation

- Basket minimum bound: `(-112,-172,19) mm` before and after.
- Basket maximum Y/Z: `(172,285) mm` before and after.
- Basket/entry maximum X: `370.986 -> 370.597 mm` (0.389 mm change, as
  approved; entry is still the receiving chute).
- Ramp external bounds remain
  `(319.65,-94,0)..(360.35,94,53) mm`.
- Intake-wheel centres, 35 degree axes, launcher origin, cradle, flywheel
  centres/dimensions, nip, pitch, battery, bridge and LiDAR datum are unchanged.

## CAD/URDF and mass properties

The regenerated validator measures 0 mm maximum envelope delta for the new
bin, fixed hood and ramp meshes (well inside the 2 mm acceptance limit). The
largest deviation among all compact major envelopes is 0.139 mm at the
tessellated intake-wheel bound.

Homogeneous solid-volume accounting:

- basket+hood material proxy: 720,606.830 -> 664,101.010 mm3,
  delta -56,505.820 mm3 (-7.84%);
- empty basket+hood estimate: 1.200 -> 1.106 kg, delta -0.094 kg;
- moving relieved bin: 1.019 kg; fixed relieved hood/supports: 0.087 kg;
- combined material-centroid proxy:
  `(242.909,1.431,92.849) -> (227.723,1.608,93.738) mm`,
  delta `(-15.186,+0.177,+0.889) mm` in compact-local coordinates;
- ramp material proxy: 147,706.800 -> 122,079.815 mm3; mass estimate
  0.090 -> 0.074 kg; centroid shifts approximately
  `(+1.678,0,-3.138) mm`.

## Dynamic intake regression

Configuration: current `compact`, PARKED basket, flywheel disabled, nominal
124 x 73 mm intake wheels, carriage state interfaces enabled, full intake,
headless Gazebo, and the simulation-health gate passed (clock, controller
clock and commanded-joint actuation).

Result: **FAIL**.

- wheel contact: left 0 samples, right 0 samples (bilateral criterion false);
- ramp-guide contact: 0 samples; ramp-climb criterion false;
- release criteria: 4/6; no physical roller release was measured;
- final ball centre: local `(0.4035,0,0.0330) m`;
- basket entry/settling/retention: 4/8, target never entered or settled;
- carriage interfaces were present in ros2_control, but no contact sample
  existed from which to measure travel/compliance.

The ball stops with its centre about one ball radius in front of the receiving
chute front edge at X=0.3706 m. This identifies the local failure as premature
contact with the retained receiving-chute/handoff edge before wheel-first
capture. The approved relief therefore clears rigid interferences but does not
produce a dynamically valid continuous handoff for the current compact datums.

Previous known-good collection evidence (`runtime/intake_sweeps/20260824_055401`)
had 43/43 bilateral wheel samples, 1.095 m/s release speed, 6/6 release and 8/8
entry/retention. The implemented compact run is a clear regression against
those metrics.

## Gazebo visual validation

Not run as a PASS/FAIL visual acceptance because the required dynamic gate
failed first. The headless physics result already exposes the handoff anomaly;
visual inspection cannot override it.

## Tests

- Authoritative exact OpenSCAD export and contract regeneration: PASS.
- Compact generated-model focused suite: 10 passed.
- ROS-aware full suite: 898 passed, 2 skipped.

## Final classification

- `COMPACT_PARKED_PACKAGING_VALIDATED_IN_SIM`: **not justified**
- `COMPACT_BASKET_LAUNCH_PATH_VALIDATED_IN_SIM`: **false / not tested**
- `COMPACT_PHYSICS_MODEL_VALIDATED_IN_SIM`: **false**
- `BALL_LAUNCH_PHYSICS_VALIDATED`: **false / not tested**
- `LAUNCHER_BRIDGE_FIXED_INTERFERENCE_PENDING`: **true**

Static packaging is exact-boolean valid, but collection dynamics are blocked
at the local ramp/receiving-chute transition. Launch-path design remains paused.
