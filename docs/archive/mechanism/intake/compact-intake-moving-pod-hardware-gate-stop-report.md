# Compact intake complete translating pod — hardware-definition STOP

> **Archived: `HISTORICAL_SUPERSEDED`.** The accepted translating-pod premise
> below was later corrected. Current FIT0186 motors and wheel centres are fixed;
> tyre and ball deformation provide compliance. See the
> [current intake definition](../../../mechanism/standalone-intake-fixed-motor-compliant-tyre.md).

Date: 2026-08-25

## Decision

```text
INTAKE_MOVING_POD_HARDWARE_DEFINITION_REQUIRED
```

The approved kinematic architecture is accepted:

```text
bridge -> fixed guide -> complete translating motor/bearing/shaft/wheel pod
```

The complete pod translates outward as one rigid assembly over 0..8 mm. The
motor does not remain fixed and no telescopic/sliding torque transfer is to be
introduced.

Authoritative manufacturing CAD was not created because several critical
purchased-part interfaces remain missing, provisional, or contradictory. No
arbitrary dimensions were promoted into the physical design. The basket-path
study was not resumed.

## A. Hardware evidence inventory

### Motor

**Repository hardware record — MEASURED_FROM_HARDWARE / received-part record**

- `docs/hardware/ordered-parts.md` records one DFRobot FIT0186 collector motor
  as received.
- `cad/collector-intake-v1/params.scad` records ruler-level dimensions:
  30 mm motor-body diameter, 70 mm motor length, 5 mm shaft diameter and
  20 mm shaft projection.
- The recorded `shaft_mount_reference_measured = 15 mm` is explicitly
  ambiguous and unused.

**DFRobot FIT0186 — MANUFACTURER_SPEC**

- Model: GB37Y3530-12V-251R with encoder.
- Output shaft: 6 mm D-shaft, 0.61 inch (approximately 15.5 mm) long.
- Face mount: six M3 threaded holes on a regular hexagon; adjacent-hole centre
  distance 15.5 mm.
- Maximum screw engagement: 3 mm.
- Mass: 205 g.

Official source:
<https://www.dfrobot.com/product-634.html>

**Status: CRITICAL CONFLICT**

The repository's claimed hardware measurement (5 x 20 mm) conflicts with the
manufacturer definition (6 x 15.5 mm). The received unit must be re-measured
and positively identified before the coupler, shaft stack and motor mount are
dimensioned. The project also requires two identical intake motors, while the
ordered-parts record documents only one received FIT0186.

### Motor mount

- Six-hole M3 face pattern and 3 mm engagement limit:
  `MANUFACTURER_SPEC`.
- Current 58 mm diameter x 6 mm CAD pod plate:
  `EXISTING_CAD_DATUM`, but it belongs to the rejected fixed-pod concept.
- Current four-M5 bridge bolt pattern at +/-26 mm X and +/-24 mm Y:
  `EXISTING_CAD_DATUM`, also fixed-pod architecture and not automatically a
  translating-guide mount.
- Moving motor bracket thickness, shape, M3 access and encoder/cable clearance:
  `MISSING`.

### Bearings and bearing support

- 626-style 19 mm OD x 6 mm width envelope: `PROVISIONAL`.
- Printed cartridge envelope, 34 mm diameter x 46 mm long: `PROVISIONAL`.
- Actual bearing manufacturer/part, inner diameter, fits/tolerances, seals,
  quantity and axial spacing: `MISSING`.
- Bearing-support wall thickness, material, fasteners and service method:
  `MISSING`.

No bearing has been selected or measured, so the nominal 626 dimensions cannot
become production datums.

### Supported transmission shaft

- Desired nominal diameter in Option A CAD: 6 mm, `PROVISIONAL`.
- Nominal flat depth 0.8 mm: `PROVISIONAL`.
- Exact shaft material, purchased part, total length, bearing-seat tolerances,
  shoulder/retaining features, threaded wheel-retention end and supported span:
  `MISSING`.

### Coupler

- CAD shows a generic 18 mm diameter x 25 mm envelope described as a 5-to-6 mm
  flexible coupler: `PROVISIONAL`.
- Manufacturer/part number, actual OD/length, bore pair, clamping/set-screw
  geometry, allowable misalignment and required shaft engagement: `MISSING`.
- The stated 5-to-6 mm bore assumption is itself affected by the motor-shaft
  conflict above.

### Wheel and hub

- Intended wheel envelope: 124 mm diameter x 73 mm width:
  `EXISTING_CAD_DATUM` and supplier product specification.
- Intended Pro-Line Raid removable interface: 12/14 mm RC hex:
  `PROVISIONAL UNTIL THE EXACT PURCHASED WHEEL IS VERIFIED`.
- Printed 6 mm D-bore to 12 mm male hex split-clamp prototype, with M4 clamp:
  `PROVISIONAL`.
- Actual wheel part in hand, installed hex option, hex depth/offset, axial
  retention stack, hub material and proof against wheel torque/side load:
  `MISSING`.

The repository purchase list names `PRO117010`, while the current supplier
description and the modern Pro-Line Raid range must be checked against the
actual delivered wheel before drilling or printing the hub.

### Guide / slide

- Required motion: outward, 0..8 mm: `EXISTING_FUNCTIONAL_DATUM`.
- Required anti-rotation, anti-racking, repeatable inner stop and outer hard
  stop: `APPROVED_REQUIREMENT`.
- Selected guide technology, cross-section, bearing/follower spacing, length,
  fastener pattern, stiffness/load rating and physical clearances: `MISSING`.

No commercial rail, drawer slide, plain-bearing block, flexure, shaft/bushing
pair or fabricated guide has been selected. This independently triggers the
task's hard STOP.

### Spring

- Compliance direction and 0..8 mm usable travel:
  `EXISTING_FUNCTIONAL_DATUM`.
- Prior simulated stiffness near 1000 N/m and measured peak travel near
  5.021 mm: `SIMULATION_REFERENCE`, not a hardware selection.
- Spring type, rate tolerance, free/installed length, preload, attachment
  points, fatigue life and buckling/retention provisions: `MISSING`.

```text
INTAKE_COMPLIANCE_SPRING_SELECTION_PENDING = true
```

### Fasteners

- General M3 motor-face requirement: `MANUFACTURER_SPEC`.
- General M4/M5 stock and purchasing intent: `EXISTING_BOM_DATUM`.
- Current fixed-pod four-M5 bridge holes: `EXISTING_CAD_DATUM`, not approved
  for the new guide.
- Final guide-to-bridge fastener size/grade/length, washers/backing plate,
  moving-pod fasteners, inserts/nuts and plywood edge distances: `MISSING`.

### Cable service

- The chassis document requests service loops for intake motors:
  `EXISTING_REQUIREMENT`.
- Connector pose, bend radius, loop length, anchor points, strain relief and
  swept cable envelope over 8 mm: `MISSING`.

```text
MOTOR_CABLE_SERVICE_ENVELOPE_DEFINED = false
```

## B. Approved moving-pod ownership

Subject to final part selection, all of the following are owned by the
prismatic carriage and translate together:

- GB37 motor and encoder body;
- motor mount;
- coupler;
- both bearing supports/bearings;
- supported transmission shaft;
- wheel hub and axial-retention hardware;
- intake wheel;
- moving guide follower/block;
- moving spring attachment and moving hard-stop faces;
- moving-side cable service allowance.

The fixed guide, fixed spring attachment, fixed hard-stop mates and bridge
backing/mount hardware remain fixed to the bridge.

Load path:

```text
ball -> wheel -> supported shaft -> bearings -> pod structure
     -> guide/follower -> bridge
```

Torque path:

```text
motor -> coupler -> supported shaft -> hub -> wheel
```

## C. Exact information required to release CAD

1. Re-measure the received FIT0186 shaft diameter, D-flat geometry and
   projection; confirm its label/model and whether a second identical motor is
   available or selected.
2. Select the two actual bearings and provide manufacturer/part number,
   `ID x OD x width`, fit class or intended printed/machined fit, and desired
   bearing spacing.
3. Select the supported shaft and define diameter, total length, material,
   bearing seats, shoulders/retainers, coupler engagement and wheel-end thread
   or retention.
4. Select the coupler and provide part number/drawing, OD, length, both bores,
   clamping method and engagement lengths.
5. Verify the exact wheel in hand and measure its installed hex size, hex
   depth, offset and axial-retention interface; approve a final hub material and
   fastening method.
6. Select or fully dimension the guide/slide cross-section, fixed length,
   follower length/spacing, material, clearances and fastener pattern.
7. Select a physical spring after the guide/pod geometry establishes attachment
   points and moving mass.

Items 1 through 6 are critical hardware gates. Item 7 remains an explicit
selection pending after those gates are closed.

## D. Manufacturing and validation work not run

Because the hardware gate fails, the following were deliberately not created
or executed:

- translating-pod manufacturing CAD;
- spring tabs and physical hard stops;
- bridge drilling pattern or plywood structural claim;
- cable swept envelope;
- rest/max/full 0..8 mm pod envelopes;
- pod mass, COM or inertia;
- static compact booleans or PARKED reclassification;
- mechanical-contract/exporter/validator changes;
- Xacro/URDF/SDF carriage collisions;
- generated-model regression tests;
- Gazebo intake revalidation.

## E. Preserved state and classification

No authoritative CAD, Xacro, URDF, SDF generator, controller, basket, handoff,
bridge, launcher, battery or LiDAR source was modified.

```text
INTAKE_MOVING_POD_HARDWARE_DEFINITION_REQUIRED = true
INTAKE_COMPLIANCE_SPRING_SELECTION_PENDING = true
MOTOR_CABLE_SERVICE_ENVELOPE_DEFINED = false

INTAKE_CARRIAGE_PHYSICAL_GEOMETRY_DEFINITION_REQUIRED = true
COMPACT_PHYSICAL_INTAKE_MODEL_COMPLETE = false
COMPACT_SIMULTANEOUS_INTAKE_AND_LAUNCHER_MODEL_REQUIRED = true

COMPACT_PARKED_PACKAGING_VALIDATED_IN_SIM = true
COMPACT_INTAKE_HANDOFF_VALIDATED_IN_SIM = true

COMPACT_BASKET_TWO_GUIDE_PATH_GEOMETRICALLY_VALID = not evaluated
COMPACT_BASKET_LAUNCH_PATH_VALIDATED_IN_SIM = false
FINAL_BASKET_ORIENTATION_MECHANISM_PENDING = true
BALL_LAUNCH_PHYSICS_NOT_VALIDATED = true
PHYSICAL_HARDWARE_PENDING = true
```

The PARKED and handoff classifications are unchanged only because no new pod
solid exists to contradict them. Both require fresh validation after the
hardware definition gate is closed and the complete pod is modelled.
