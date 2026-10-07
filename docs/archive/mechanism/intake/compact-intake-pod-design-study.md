# Corrected compact direct-drive intake moving-pod packaging study

> **Archived: `HISTORICAL_SUPERSEDED`.** Direct drive remains current, but the
> translating-pod premise and its 0–8 mm sweep do not. See the
> [fixed-motor architecture](../../../mechanism/standalone-intake-fixed-motor-compliant-tyre.md).

Date: 2026-08-25
Status: intake-axis correction implemented; static packaging gate failed

## Executive decision

The previous study is invalid as evidence against or for the intended intake
because it modelled a 198 mm supported transmission shaft, bearings, coupler,
printed hub and associated motor-mount stack that will not exist.

This study removes those solids and re-runs the package from zero using the
actual architecture:

```text
FIT0186
  -> its own 6 mm D-shaft
  -> purchased 14-00012630 (6 mm shaft to 12 mm hex, L30 mm)
  -> supplied PRO117010 12 mm Raid interface
  -> PRO117010 wheel, diameter 124 x 73 mm
```

The old implementation also tilted both intake axes into the X-Z plane. That
root cause is now corrected throughout intake CAD, Xacro, generated URDF/SDF,
CAD-derived relief meshes, measurements, contract and tests. The launcher was
not reoriented. The corrected result still fails the locked package: the motor
intersects the bridge, launcher cradle and same-side launcher flywheel; the
wheel intersects the launcher cradle at rest and enters the basket and hood
during outward travel. Candidate A's analysis-only guide allocation also
intersects the launcher, independently of those hardware failures.

No locked datum was moved, no alternative drivetrain was invented, and no
basket launch-path study was resumed.

## A. Corrected architecture inventory

Each left/right pod translates independently and outward by 0..8 mm as one
rigid body. It contains:

- one measured FIT0186 motor body;
- the motor's own measured 6 mm D-shaft;
- one purchased `14-00012630` adapter;
- the supplied PRO117010 12 mm removable-hex interface;
- one PRO117010 wheel envelope;
- Candidate A moving followers, allocation spine and hard-stop tongue;
- future motor support, spring attachment, hard stop and cable service volume,
  which remain undefined rather than invented.

The fixed Candidate A family retains two 8 mm rods with replaceable
printed/plain-bearing followers and an anti-racking two-line layout.

The analysis scene simultaneously contains both complete intake pods, both
intake wheels, both motors, both purchased adapters, cheeks, handoff ramp,
receiving hood/channel, basket/bin, bridge, chassis, battery, launcher cradle,
left flywheel and right flywheel. The intake and launcher are permanent,
distinct mechanisms; neither was hidden or substituted for the other.

## B. Removed false/provisional drivetrain assumptions

The following former analysis solids were deleted from the collision model:

- 198 mm transmission shaft;
- external shaft-support bearings and bearing supports;
- bearing cartridge;
- flexible coupler;
- provisional printed wheel hub;
- motor clamp and mount dimensions derived from that long stack;
- any belt, pulley, gearing or torque-transfer telescope.

The old shaft/bearing/coupler/complete-pod launcher intersection numbers are
historical only and were not reused.

## C. Hardware evidence classification

### MEASURED_FROM_HARDWARE

- FIT0186 body: diameter 30 mm, length 70 mm excluding shaft.
- FIT0186 output: 6 mm D-shaft, approximately 20 mm projection.

The received 6 mm result supersedes the repository's earlier 5 mm ruler entry.
DFRobot's current product documentation independently specifies a 6 mm
D-shaft; its nominal 0.61 in projection is recorded as supplier context, while
the received approximately 20 mm measurement governs this model.

### PURCHASED_PART_DATUM

- `14-00012630`: 6 mm shaft side, 12 mm hex wheel side, total length 30 mm.
- Quantity purchased: six.

### MANUFACTURER_SPEC

- PRO117010 wheel external envelope: diameter 124 x 73 mm.
- Raid 6x30 removable interface, six M3 fasteners.
- Supplied narrow/wide 12 mm and 14 mm interfaces; intended intake interface
  is 12 mm.

### EXISTING_AUTHORITATIVE_CAD / DERIVED_FROM_LOCKED_DATUM

- wheel centres, mirrored 35 degree coaxial Y-Z axes, nip, PARKED geometry and 0..8 mm
  bilateral outward travel;
- relieved basket, repaired fixed hood/channel, handoff ramp, cheeks, 490 mm
  bridge, chassis, battery and physical launcher solids.

### ANALYSIS_ONLY_ASSUMPTION

- adapter maximum outside diameter: conservative 20 mm cylinder;
- nominal adapter engagement into the wheel-side hex: 8 mm;
- Candidate A follower, rod-support, spine and stop allocation solids.

### MEASUREMENT_PENDING

- actual `14-00012630` outside diameter and detailed profile;
- installed PRO117010 narrow/wide 12 mm hex offset, depth and seating;
- wheel-side axial retention details;
- motor lead exit and moving cable service envelope.

The nominal adapter seating was sensitivity-tested at 0, 8 and 12 mm. The
direct hardware fails at all three settings, so this pending value does not
drive the gate result.

## D. Corrected direct-drive pod geometry

The wheel is the supplier envelope centred at the locked intake datum. The
adapter begins at the wheel outer axial face minus the explicitly assumed
8 mm engagement. Its 30 mm purchased length overlaps the motor's own output
shaft; no duplicate transmission shaft is created. The motor body begins at
the adapter's motor-side end and remains coaxial with the wheel.

With `side=+1` left and `side=-1` right, the authoritative intake-only
transform is `rotate([-side*35,0,0])`. It maps local +Z to
`(0, side*sin(35), cos(35))`; no independent motor Y offset exists.

```text
LEFT  wheel [370,  90, 70] mm; axis [0,  0.573576436, 0.819152044]
      motor [370, 143.629397, 146.590716] mm; outward dot +53.629397 mm
RIGHT wheel [370, -90, 70] mm; axis [0, -0.573576436, 0.819152044]
      motor [370,-143.629397, 146.590716] mm; outward dot +53.629397 mm
```

The bilateral continuous direct-hardware swept bounding box is:

```text
min [308.000, -183.992,   4.539] mm
max [432.000,  183.992, 183.865] mm
```

This bounding box is descriptive only. Every collision decision below comes
from an exact OpenSCAD/CGAL Boolean intersection, not from box overlap.

## E. Candidate A guide geometry

Candidate A remains the preferred guide family in principle: two separated
8 mm rods, replaceable printed/plain-bearing followers, anti-racking spacing,
and independent 0..8 mm outward motion.

Current analysis allocations are:

```text
moving swept allocation: [385,-115,173] .. [487,115,201] mm
fixed guide allocation:   [384,-139,168] .. [490,139,194] mm
```

They are deliberately not a final carriage. Spring hardware, cable routing,
motor attachment, rod fits, stops and fasteners remain undefined.

## F. Exact 0 / 4 / 8 mm collision matrix

All volumes are exact CGAL intersection volumes in mm3. Bilateral totals are
reported unless a side is named. Zero relationships list sampled positive
surface distance only as a clearance estimate.

### Direct-drive hardware at 0 mm

- Motor to bridge: 2,852.471 total; 1,426.236 per side.
- Motor to launcher cradle: 14,627.457 total; 7,313.728 per side.
- Same-side motor to launcher flywheel: 4,120.529 per side.
- Wheel to launcher cradle: 1,103.783 total; 551.891 per side.
- Basket clearance: approximately 2.00 mm.
- Hood clearance: approximately 2.01 mm.
- Ramp clearance: approximately 2.04 mm.
- Cheek clearance: approximately 42.29 mm.
- Chassis clearance: approximately 14.25 mm.
- Battery clearance: approximately 480.03 mm.
- Opposite intake pod intersection: zero.

### Direct-drive hardware at 4 mm

- Motor to bridge: 5,229.233 total; 2,614.617 per side.
- Motor to launcher cradle: 14,199.715 total; 7,099.857 per side.
- Same-side motor to launcher flywheel: 3,476.648 per side.
- Wheel to launcher cradle: 1,103.783 total; 551.891 per side.
- Wheel to basket: 213.917 total; left 102.855, right 111.062.
- Wheel to hood: 144.010 total; 72.005 per side.
- Ramp clearance: approximately 4.30 mm.
- Cheek clearance: approximately 42.72 mm.
- Chassis clearance: approximately 10.74 mm.
- Battery clearance: approximately 480.02 mm.
- Opposite intake pod intersection: zero.

### Direct-drive hardware at 8 mm

- Motor to bridge: 8,764.746 total; 4,382.373 per side.
- Motor to launcher cradle: 12,996.465 total; 6,498.233 per side.
- Same-side motor to launcher flywheel: 2,773.736 per side.
- Wheel to launcher cradle: 1,103.783 total; 551.891 per side.
- Wheel to basket: 2,602.794 total; left 1,237.136, right 1,365.658.
- Wheel to hood: 1,264.355 total; 632.178 per side.
- Ramp clearance: approximately 6.59 mm.
- Cheek clearance: approximately 42.92 mm.
- Chassis clearance: approximately 7.75 mm.
- Battery clearance: approximately 480.05 mm.
- Opposite intake pod intersection: zero.

### Direct component decomposition

- The purchased adapter has zero exact intersection against every obstacle at
  0, 4 and 8 mm.
- The motor's own output shaft has zero exact intersection against every
  obstacle at 0, 4 and 8 mm.
- The PRO117010 hex placeholder has zero exact intersection against every
  obstacle at 0, 4 and 8 mm.
- The wheel is responsible for the cradle collision at every endpoint and the
  basket/hood collisions after rest travel.
- The measured motor body is responsible for the bridge and flywheel
  collisions and its share of the cradle collision.

### Candidate A carriage/guide decomposition

Moving allocation, at each endpoint:

- launcher cradle: 5,469.529 total;
- matching left/right flywheel: 44,589.922 each;
- bridge: zero, approximately 5.01 mm sampled clearance;
- basket, hood, ramp, cheeks, chassis and battery: zero intersections;
- opposite pod: zero.

Fixed guide allocation:

- launcher cradle: 11,206.341;
- left flywheel: 13,194.118;
- right flywheel: 13,194.118;
- all other obstacles: zero.

These carriage conflicts invalidate only the current analysis allocation,
not the direct-drive architecture. In this case the direct-drive hardware also
fails independently, so both findings must be carried forward.

## G. Continuous 0..8 mm swept-volume result

For each convex measured/purchased hardware primitive, the endpoint hull is
the exact continuous swept set under the locked linear translation. Exact CGAL
intersections against the fixed machine produce:

- Motor to bridge: 8,764.963 total; 4,382.482 per side.
- Motor to launcher cradle: 17,067.204 total; 8,533.602 per side.
- Same-side motor to launcher flywheel: 5,218.578 per side.
- Wheel to launcher cradle: 2,704.282 total; 1,352.141 per side.
- Wheel to basket: 2,634.260 total; left 1,252.869, right 1,381.391.
- Wheel to hood: 1,269.348 total; 634.674 per side.
- Direct hardware to ramp, cheeks, chassis and battery: zero.
- Opposite intake swept-volume intersection: zero.

The Candidate A moving swept allocation intersects the cradle by 10,824.501
total and each same-side flywheel by 81,920.675. Its follower bores are
conservatively filled for the sweep, which cannot explain the direct motor
collisions measured separately.

## H. Minimum positive clearances

Meaningful continuous-sweep direct-hardware clearances are:

- handoff ramp: approximately 2.01 mm;
- cheeks: approximately 42.29 mm;
- chassis: approximately 7.48 mm;
- battery: approximately 480.01 mm;
- opposite pod: zero intersection; the bilateral ball corridor remains open.

These are deterministic sampled estimates at nominal 4 mm surface spacing.
They do not replace exact collision decisions.

## I. Hardware collision locations and minimum relief gate

The exact swept intersection bounding boxes per affected side are:

```text
left motor / bridge:   [355.000, 165.000,150.000] .. [385.000,183.992,168.000]
right motor / bridge:  [355.000,-183.992,150.000] .. [385.000,-165.000,168.000]
left motor / cradle:   [355.000, 114.894,127.056] .. [385.000,157.000,146.199]
right motor / cradle:  [355.000,-157.000,127.056] .. [385.000,-114.894,146.199]
left motor / flywheel: [367.869, 133.938,157.416] .. [385.000,177.650,183.865]
right motor/flywheel:  [367.869,-177.650,157.416] .. [385.000,-133.938,183.865]
left wheel / basket:   [311.789,  96.139, 19.149] .. [330.000,146.000, 85.150]
right wheel / basket:  [311.789,-140.000, 19.149] .. [330.000,-96.139, 89.351]
left wheel / hood:     [330.000,  68.930,113.499] .. [370.000, 95.000,131.753]
right wheel / hood:    [330.000, -95.000,113.499] .. [370.000,-68.930,131.753]
```

No permitted translation inside the locked 0..8 mm motion clears the motor:
the motor collisions exist at rest, 4 mm, 8 mm and throughout the continuous
sweep. The adapter-seating sensitivity at 0/8/12 mm also fails.

The existing bridge service opening is 22 x 22 mm. A geometric screen for the
30 mm motor at 35 degrees and 8 mm outward sweep gives a minimum opening of
approximately 34 mm in X by 48.62 mm in Y when 2 mm clearance is included.
That is only a relief screen; changing the locked bridge is a new mechanical
decision.

The current tyre pocket has 2 mm rest clearance. Preserving 8 mm outward travel
and 2 mm clearance requires extending its outward swept boundary by 8 mm. The
locked pocket does not do so. Changing it is also outside this task.

The launcher cradle could only be cleared by resolving at least the exact
8,533.602 mm3 swept motor overlap per side plus the 1,352.141 mm3 wheel overlap
per side and chosen clearance. A physical flywheel cannot be relieved; clearing
the 5,218.578 mm3 same-side motor/flywheel
overlap requires moving a locked datum or changing the motor/wheel axial stack.
No such redesign is authorized here, so no misleading scalar translation is
invented.

## J. Available carriage design envelope

An available free carriage volume cannot be released while the purchased
hardware already occupies forbidden bridge/launcher/flywheel/basket space.
The measured bounds above remain useful allocation references only:

- hardware continuous sweep: `[308,-183.992,4.539] .. [432,183.992,183.865]`;
- Candidate A moving probe: `[385,-115,173] .. [487,115,201]`;
- Candidate A fixed probe: `[384,-139,168] .. [490,139,194]`.

No minimal printable carriage topology is proposed because the prerequisite
`DIRECT_DRIVE_HARDWARE_0_TO_8MM_SWEPT_CLEAR` is false.

## K. Remaining physical measurements

- Measure `14-00012630` maximum body diameter, step profile, set-screw
  projection and usable 6 mm bore depth.
- On wheel arrival, measure both supplied 12 mm Raid offsets, installed depth,
  seating face and fastener projection.
- Confirm the real adapter-to-wheel engagement and axial-retention stack.
- Verify both intake motors are the same FIT0186 variant and re-measure each
  shaft projection and cable exit.
- Define motor cable bend radius/service loop only after the architecture gate.
- Do not define springs, stops or manufacturing carriage fits yet.

## L. Explicit next mechanical step

Stop before redesign. Review the four fresh direct-hardware conflicts against
the locked-datum list and explicitly choose whether a new task may alter the
bridge service opening, tyre pockets, launcher placement/flywheel envelope, or
the direct-drive axial stack. Do not automatically add bearings, couplers,
belts, shafts or another transmission. After an authorized resolution, rebuild
the complete physical carriage collision model and rerun the validated intake
handoff tests before any basket launch-path study.

## Regression protection and final classifications

The prior simulation evidence (bilateral wheel contacts 48/48, approximately
5.021 mm travel each side, release 8/8 and retention 8/8) is historical only:
it used the incorrect X-Z intake axis. Dynamic revalidation was not run because
the required static packaging gate failed first.

```text
INTAKE_35_DEGREE_ORIENTATION_CORRECTED = true
LEFT_MOTOR_IS_OUTBOARD = true
RIGHT_MOTOR_IS_OUTBOARD = true
INTAKE_AXES_INTENDED_MIRROR_VALIDATED = true
CAD_XACRO_URDF_SDF_INTAKE_AXIS_ALIGNED = true
COMPACT_SIMULTANEOUS_INTAKE_AND_LAUNCHER_MODEL_VALIDATED = true
DIRECT_DRIVE_INTAKE_POD_PACKAGING_GEOMETRICALLY_VALID = false
DIRECT_DRIVE_HARDWARE_0_TO_8MM_SWEPT_CLEAR = false
CANDIDATE_A_GUIDE_PACKAGING_GEOMETRICALLY_VALID = false
INTAKE_MOVING_POD_ARCHITECTURAL_REDESIGN_REQUIRED = true
INTAKE_CARRIAGE_MANUFACTURING_CAD_PENDING = true
COMPACT_PHYSICAL_INTAKE_MODEL_COMPLETE = false
COMPACT_INTAKE_STATIC_PACKAGING_VALIDATED = false
COMPACT_PARKED_PACKAGING_VALIDATED_IN_SIM = false
COMPACT_INTAKE_HANDOFF_VALIDATED_IN_SIM = false
COMPACT_BASKET_LAUNCH_PATH_VALIDATED_IN_SIM = false
BALL_LAUNCH_PHYSICS_NOT_VALIDATED = true
PHYSICAL_HARDWARE_PENDING = true
```

The basket launch-path study was not resumed. Intake CAD, CAD-derived relief
meshes, the mechanical contract, Xacro and generated URDF/SDF were updated;
launcher transforms and controller behaviour were not changed.
