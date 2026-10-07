# SUPERSEDED — standalone translating-intake reconstruction and design gate

> **Status: `HISTORICAL_SUPERSEDED`.** This report assumed translating
> motor/wheel pods. The corrected physical architecture fixes both motors and
> wheel centres to the bridge and uses ball + Trencher tyre/foam compliance.
> See [`standalone-intake-fixed-motor-compliant-tyre.md`](../../../mechanism/standalone-intake-fixed-motor-compliant-tyre.md).

Date: 2026-08-26
Decision: **Case C — mechanical design incomplete**
Machine-readable status: [`config/standalone_intake_checkpoint.json`](config/standalone_intake_checkpoint.json)

## Executive decision

The current intent is unambiguous: two mirrored Pro-Line Trencher/Raid wheels,
each driven directly by an outboard FIT0186 through one purchased
`14-00012630` adapter. Each complete motor/adapter/wheel pod must translate
outward by 0–8 mm. The corrected 35 degree axes are consistent in the current
CAD, mechanical contract, and Xacro.

The intake is **not ready to freeze**. There is no current authoritative
compliant guide. Candidate A is an analysis allocation, was never manufacturing
geometry, and is explicitly invalid under the changed bridge allocation. No
newer rods/rail, followers, anti-racking spacing, hard stops, spring/preload, or
cable loop exists. The direct-drive chain is identified, but installed shaft
engagement, adapter profile/tool access, Raid seating, axial retention, and
motor mount details are still unmeasured or missing.

This activates the task's STOP rule before static or dynamic simulation. A
Gazebo run with the current collisionless/placeholder carriage, `mu=2.5`, and
ideal velocity source would create evidence for a mechanism that has not been
defined. No complete-robot geometry was changed and no flywheel/basket
packaging result was used to fail the standalone gate.

```text
INTAKE_ARCHITECTURE_FROZEN_PROVISIONALLY = false
INTAKE_MECHANICAL_DESIGN_INCOMPLETE = true
INTAKE_READY_FOR_COMPLETE_ROBOT_INTEGRATION = false
```

## Standalone boundary and datums

The permitted bench contains only the two wheels, two motors, two purchased
adapters, coherent moving pods, fixed guides/local support, compliance
hardware, one calibrated compliant ball, and a minimal datum. The launcher,
basket, complete bridge/chassis, route, navigation, and perception are outside
this phase.

Side `+1` is left and side `-1` is right. The wheel-to-motor direction is:

```text
(0, side*sin(35 deg), cos(35 deg))
OpenSCAD: rotate([-side * 35, 0, 0])
left:  (0,  0.573576436, 0.819152044)
right: (0, -0.573576436, 0.819152044)
```

Both motors are therefore outboard. The historical `rotate([0,35,0])` X-Z
orientation is not an intake datum and must not return.

The current supplier wheel envelope is 124 mm diameter by 73 mm width. At the
56 mm nominal gap, a centered 66 mm ball gives a first-order planar overlap of
10 mm, or 5 mm outward motion per pod. That lies inside the required 0–8 mm
travel, but it is only a geometric sanity screen; it is not a passage result.

## Source-of-truth audit

| Artifact | Purpose | Status | Geometry owner | Simulation owner | Evidence | Remain active |
|---|---|---|---:|---:|---|---:|
| `cad/collector-intake-v1/option-a/option-a.scad` | current wheel envelope, direct stack, axes | current intent; guide missing | yes | no | CAD + provisional interfaces | yes |
| `cad/flywheel-launcher-v0/compact-intake-pod-concept-study.scad` | corrected package analysis | analysis only; Candidate A invalid | no | no | CAD-derived + provisional | yes, as analysis |
| `ros2_ws/src/tennis_robot/urdf/components/drivetrain.urdf.xacro` | simulated axes and compliance DOF | current sim reference; not a physical guide | no | yes | provisional | yes |
| `config/compact_mechanical_contract.json` | corrected axes and envelopes | current, but complete-robot scope | no | no | CAD-derived | yes |
| `docs/archive/mechanism/intake/dual-wheel-intake-design-el.md` | functional/motor rationale | intent remains; older dimensions and physics are not validation | no | no | supplier + provisional | yes, qualified |
| `config/tennis_ball_compliance_calibration_results.json` | ball normal compliance | current; reuse without refit | no | yes | calibrated | yes |
| `docs/hardware/ordered-parts.md` | ownership/receipt | authoritative purchase record | no | no | purchased/measured | yes |
| `docs/hardware/prototype-purchase-list-el.md` | planned BOM | current plan contains superseded fixed-shaft text | no | no | purchase plan | yes, qualified |
| intake debug log and old bench sweep | old dynamic evidence | historical: wrong X-Z axis and pre-calibration ball/contact assumptions | no | no | historical sim | no |
| archived single-motor geared intake | old architecture | obsolete | no | no | historical CAD | no |

The old 198 mm transmission shaft, external wheel bearings, coupler, printed
torque hub, and remote fixed motor are superseded. They are not allowed to fill
the missing guide definition.

## Current architecture and direct-drive gate

The intended torque chain is:

```text
FIT0186 6 mm D-shaft
  -> purchased 14-00012630, 6 mm bore to 12 mm hex, nominal L30 mm
  -> supplied PRO117010 12 mm Raid interface
  -> PRO117010 Trencher/Raid wheel
```

The motor, its output shaft, purchased adapter, Raid interface, wheel, motor
mount, moving follower, spring attachment, stop face, and moving cable
allowance all belong to one translating pod. No telescopic torque transfer is
required and no printed torque hub is required.

The chain is not yet a released mechanical interface. Before it can own a
physical collision model, record both motors' shaft projection/D-flat, the
adapter's usable bore and external/set-screw profile, the selected Raid 12 mm
interface depth/offset/seating, installed engagement, wheel axial retention,
motor-face mount, and assembly/tool path. Conservative cylinders are adequate
for early packaging screens but not for shaft seating or retention validation.

## Compliant-guide gate

The required guide must provide independent 0–8 mm outward motion with low
racking, stiffness in all other directions, repeatable return, replaceable or
durable wear interfaces, debris tolerance, manufacturable fits, cable service,
two physical hard stops, and a retained spring/preload system. It must also
carry wheel impact and side reactions into the fixed bench without assigning
them to an invented shaft or bearing stack.

No repository artifact currently defines those features. The existing Xacro
has a prismatic joint and nominal spring patch, but it is kinematic simulation
intent, not guide geometry. Candidate A's two-rod family remains a useful
historical design idea only; the task explicitly disallows reviving its invalid
allocation. Therefore:

```text
INTAKE_GUIDE_ARCHITECTURE_DEFINED = false
INTAKE_GUIDE_ARCHITECTURE_VALIDATED_IN_SIM = false
INTAKE_GUIDE_ARCHITECTURE_UNRESOLVED = true
```

## Ball, tread, and motor evidence

The accepted calibrated tennis-ball normal model remains the normal-contact
source and needs no recalibration. Tread traction is a separate interface. No
flywheel tyre coefficient is transferred to the aggressive Trencher tread and
tennis-ball felt.

Repository manufacturer-spec evidence for the FIT0186 gives 26.3 rad/s
no-load speed, 1.77 N m stall torque, and 7 A stall current. With the 62 mm
wheel radius these imply 1.6306 m/s no-load surface speed and 28.55 N
stall-limit tangential force. These are derived bounds, not demonstrated
capture capability. The current Xacro velocity actuator is ideal and does not
represent torque-speed droop, transient current, recovery, or thermal margin.

Consequently both `INTAKE_MOTOR_CAPABILITY_VALIDATED_IN_SIM` and
`INTAKE_TREAD_FRICTION_PHYSICALLY_VALIDATED` remain false.

## Validation disposition

Static entry, quasi-static passage, dynamic capture, offset entry,
repeatability, motor load, and structural/wear runs were not executed because
the guide gate failed first. Existing successful intake runs cannot be reused:
they used the historical axis/contact model. No plots were produced because
there is no credible pod/guide/contact model from which to generate engineering
telemetry.

The eventual standalone campaign must record pod displacement, ball
compression, normal force, guide reaction, symmetry and return; then wheel
speed/load, ball path, capture versus entry speed/offset, and an unfitted tread
friction sensitivity. Physical felt/tread traction remains a hardware gate
even if that campaign succeeds geometrically.

## Load and wear screen status

The eventual screen must cover wheel reactions, the purchased adapter at up to
the motor torque bound, motor-shaft bending, guide reactions, spring/preload,
hard-stop impacts, follower/bushing loads, felt damage, tread biting, guide
wear, debris binding, and cable fatigue. It was not run because the load path,
guide, spring, and stop geometry are undefined. This is `NOT RUN`, not a
structural failure.

## Purchase and BOM audit

- One FIT0186 is recorded received: `ALREADY_OWNED` and
  `PHYSICAL_MEASUREMENT_REQUIRED`.
- A second identical FIT0186 is `TO_BUY`; verify the exact variant before
  ordering.
- One PRO117010 pair (two intake wheels) is still `TO_BUY` in the repository
  purchase plan; verify availability and measure the delivered Raid interfaces.
- Two of the six purchased `14-00012630` adapters are allocated to the intake:
  `ALREADY_OWNED` and `PHYSICAL_MEASUREMENT_REQUIRED`. The other four are not
  intake consumption.
- Guide hardware and springs are `VERIFY_BEFORE_ORDERING` because no parts are
  selected.
- Pod brackets, followers, stop features, and cable anchors are
  `CUSTOM_MANUFACTURE`/`3D_PRINT` only after the guide gate.
- Final fasteners and strain-relief parts remain `VERIFY_BEFORE_ORDERING`.

## Smallest next engineering task

Run one **standalone intake direct-interface metrology and guide-definition
gate**:

1. Measure the two motors, two allocated adapters, and the selected pair of
   12 mm Raid interfaces.
2. Select and dimension one guide family, including anti-racking spacing,
   fits/materials, 0/8 mm stops, spring/preload, cable loop, fasteners, and tool
   access.
3. Produce manufacturing CAD for one isolated complete pod and its fixed guide.
4. Demonstrate unobstructed 0–8 mm motion and repeatable return in that isolated
   CAD/bench.

Only then is a standalone Xacro/bench and the requested static-to-dynamic
campaign authorized. Complete-robot integration remains deferred.

## Final classifications

```text
INTAKE_CURRENT_ARCHITECTURE_IDENTIFIED = true
INTAKE_35_DEGREE_MIRRORED_AXES_VALIDATED = true
LEFT_INTAKE_MOTOR_OUTBOARD = true
RIGHT_INTAKE_MOTOR_OUTBOARD = true

INTAKE_DIRECT_DRIVE_INTERFACE_DEFINED = false
INTAKE_PRINTED_TORQUE_HUB_REQUIRED = false

INTAKE_COMPLIANT_TRAVEL_REQUIRED = true
INTAKE_COMPLIANT_TRAVEL_RANGE_MM = [0, 8]
INTAKE_GUIDE_ARCHITECTURE_DEFINED = false
INTAKE_GUIDE_ARCHITECTURE_UNRESOLVED = true
INTAKE_GUIDE_ARCHITECTURE_VALIDATED_IN_SIM = false

GEOMETRIC_CAPTURE_CAPABILITY_VALIDATED = false
INTAKE_STATIC_BALL_PASSAGE_VALIDATED = false
INTAKE_DYNAMIC_CAPTURE_VALIDATED_IN_SIM = false
INTAKE_CAPTURE_ENVELOPE_DEFINED = false
INTAKE_MOTOR_CAPABILITY_VALIDATED_IN_SIM = false
INTAKE_TREAD_FRICTION_PHYSICALLY_VALIDATED = false
PHYSICAL_FELT_TREAD_TRACTION_VALIDATED = false

INTAKE_STRUCTURAL_SCREEN_PASSED = false
INTAKE_PHYSICAL_HARDWARE_VALIDATED = false
INTAKE_ARCHITECTURE_FROZEN_PROVISIONALLY = false
INTAKE_MECHANICAL_DESIGN_INCOMPLETE = true
INTAKE_READY_FOR_COMPLETE_ROBOT_INTEGRATION = false
PHYSICAL_HARDWARE_PENDING = true
```
