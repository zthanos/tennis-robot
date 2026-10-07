# Compact intake carriage physical definition — STOP report

> **Archived: `HISTORICAL_SUPERSEDED`.** A physical carriage is not required by
> the current fixed-motor architecture. This STOP remains useful as evidence of
> the former model contradiction only. See the
> [current intake definition](../../../mechanism/standalone-intake-fixed-motor-compliant-tyre.md).

Date: 2026-08-25

## Decision

```text
INTAKE_CARRIAGE_PHYSICAL_GEOMETRY_DEFINITION_REQUIRED
```

No carriage collision, CAD solid, mechanical contract entry, mass property,
validator rule, or generated-model test was added. The authoritative compact
CAD and the simulated intake kinematics contradict one another at the physical
carriage boundary, and the repository does not contain enough defined hardware
to resolve that contradiction without inventing a mechanism.

The basket launch-path study was not resumed.

## A. Confirmed generated-model defect

The generated `compact` URDF and SDF contain both
`intake_wheel_left_carriage_link` and
`intake_wheel_right_carriage_link`, but each link has one visual and zero
collisions. The visual is a 30 x 30 x 12 mm box. It is not traced to a physical
CAD part and therefore cannot be promoted to authoritative collision geometry.

## B. Authoritative-source contradiction

The current manufacturing CAD does not define a compliant intake carriage:

- `cad/archive/intake/straight-cheek-aluminium-rail-study/intake-structure.scad` calls its intake wheels
  references only and states that wheel/carriage CAD follows the motor fit
  check.
- `cad/collector-intake-v1/option-a/option-a.scad` defines the plywood portal
  as carrying two **fixed motor/shaft pods**.
- The same source defines a fixed, bearing-supported transmission and four
  fixed M5 pod bolts, explicitly stating **no lateral sliding carriage**.
- The bearing cartridge is provisional pending real bearing/shaft
  measurements, and the final motor clamp/face adapter is not defined because
  the motor mounting-hole pattern is missing.
- In contrast, `docs/archive/mechanism/intake/dual-wheel-intake-design-el.md` and the generated
  robot require each wheel to translate outward on a spring-loaded prismatic
  carriage over 0..8 mm.

This is not a choice between two equivalent collision approximations. A fixed
pod and a translating pod have different bodies, mounting holes, load paths,
shaft/coupler requirements and swept volumes. The task explicitly requires a
STOP when current mechanical representation is contradicted by direct CAD
evidence.

## C. Evidence classification

### MEASURED_FROM_CAD / measured physical input

- Motor body: approximately 30 mm diameter x 70 mm long.
- Motor shaft: approximately 5 mm diameter x 20 mm projection.
- Intake wheel: 124 mm diameter x 73 mm width in current Option A compact CAD.
- Wheel centre: compact-local `(470, +/-90, 70) mm` before the common
  `-100 mm` X shift.
- Wheel axis pitch: 35 degrees.
- Bridge thickness: 18 mm.
- Current fixed pod plate shown in CAD: 58 mm diameter x 6 mm thick.
- Current fixed pod bolt pattern: four M5 clearances at `X +/-26 mm`,
  `Y +/-24 mm` about each shaft service opening.

The last two items define the present **fixed** concept only. They do not define
a compliant carriage.

### DERIVED_FROM_EXISTING_MECHANICAL_DATUM

- Required compliance direction: outward `+Y` on the left and `-Y` on the
  right.
- Required travel: 0..8 mm.
- Rest wheel gap and wheel/nip/axis datums are protected.
- A physical design must maintain the motor/shaft/wheel coaxial relationship
  along the 35-degree axis and must prevent wheel impact/side load from being
  carried by the motor's 5 mm shaft.

These requirements constrain a future carriage but do not determine its
physical envelope.

### ASSUMED / provisional and not authoritative

- The 30 x 30 x 12 mm Xacro `carriage_vis` box.
- Carriage mass `0.05 kg`, COM at the wheel centre, and `1e-5 kg m2` diagonal
  inertia.
- Spring stiffness near 1000 N/m as a simulation starting point.
- 626 bearing envelope: 19 mm OD x 6 mm width.
- 6 mm transmission-shaft flat and final purchased shaft details.
- Motor clamp/face adapter and mounting-hole pattern.

None of these assumptions is sufficient to define a collision-bearing moving
assembly.

## D. Missing physical decisions and dimensions

The following must be selected or measured before implementation.

### Carriage and guide

- guide technology and purchased part, or fabricated slide cross-section;
- fixed guide length, position, orientation and attachment to the bridge;
- moving block/plate X, Y and Z dimensions;
- moving plate material and thickness;
- wheel-axis offset from the moving block in X, Y and Z;
- anti-rotation/anti-racking arrangement and bearing spacing;
- hard-stop geometry at 0 and 8 mm;
- spring type, installed position, preload, travel envelope and attachment
  points;
- fastener sizes, hole pattern and edge distances.

### Driveline ownership

- whether the GB37 motor translates with the carriage or remains fixed;
- if moving: motor face/mount pattern, moving motor support geometry, cable
  service envelope and total moving mass;
- if fixed: the specific torque-transfer mechanism that permits 8 mm lateral
  wheel motion while preserving the tilted axis;
- whether the bearing cartridge, transmission shaft, flexible coupler, hub and
  motor pod translate together or are divided across the joint;
- actual bearing OD/width, shaft length/flat, coupler dimensions and axial
  retention stack.

### Collision and mass data

- fabrication-clear physical envelope for every translating solid;
- material/density or measured mass for the carriage, motor support and moving
  driveline hardware;
- COM and inertia about the prismatic link frame;
- intended ball-access/contact surfaces on the carriage and guards around the
  motor/shaft hardware.

## E. Kinematic ownership cannot yet be assigned

The wheel is unambiguously the rotating child of the compliant degree of
freedom. The physical ownership of the support, bearings, shaft, coupler,
motor and pod plate is not defined consistently:

```text
CAD now:       bridge -> fixed pod/motor/bearings/shaft -> rotating wheel
simulation:    bridge -> translating placeholder -> rotating wheel
required CAD:  undecided
```

Assigning all fixed-CAD pod solids to the prismatic link would silently redesign
the bridge mount. Leaving them fixed would require an undefined sliding or
flexible driveline. Neither is authorized by current evidence.

## F. Validation not run

Because there is no credible carriage solid to validate, the following were
correctly not run:

- PARKED carriage exact booleans;
- independent basket/hood/ramp/launcher-to-carriage checks;
- bridge/chassis 0..8 mm swept-range checks;
- minimum positive carriage clearances;
- generated URDF/SDF collision-presence regression tests;
- Gazebo collection/handoff regression;
- ball-to-carriage contact ordering;
- new carriage mass/inertia validation.

Running those checks with an invented box would create false evidence.

## G. Protected state and classification

No authoritative CAD, Xacro, URDF, controller, PARKED geometry, receiving
channel, tire pocket, handoff ramp, basket, launcher, bridge, battery or LiDAR
source was modified.

```text
INTAKE_CARRIAGE_PHYSICAL_GEOMETRY_DEFINITION_REQUIRED = true
COMPACT_SIMULTANEOUS_INTAKE_AND_LAUNCHER_MODEL_REQUIRED = true
COMPACT_PHYSICAL_INTAKE_MODEL_COMPLETE = false

COMPACT_PARKED_PACKAGING_VALIDATED_IN_SIM = true
COMPACT_INTAKE_HANDOFF_VALIDATED_IN_SIM = true

COMPACT_BASKET_TWO_GUIDE_PATH_GEOMETRICALLY_VALID = not evaluated
COMPACT_BASKET_LAUNCH_PATH_VALIDATED_IN_SIM = false
FINAL_BASKET_ORIENTATION_MECHANISM_PENDING = true
BALL_LAUNCH_PHYSICS_NOT_VALIDATED = true
PHYSICAL_HARDWARE_PENDING = true
```

The existing PARKED and handoff classifications are preserved because no new
physical carriage geometry exists to provide contrary evidence. They must be
revalidated after a real compliant carriage is defined and modelled.

## H. Required next mechanical decision

Select and dimension one physical architecture:

1. a complete translating motor/bearing/shaft/wheel pod; or
2. a translating wheel/bearing support driven from a fixed motor through a
   specifically selected compliant/sliding torque-transfer mechanism.

That decision must be reflected first in the manufacturing CAD. Only then can
the same geometry become the OpenSCAD boolean solid, mechanical-contract
envelope, URDF/SDF carriage collision and future basket swept-path obstacle.
