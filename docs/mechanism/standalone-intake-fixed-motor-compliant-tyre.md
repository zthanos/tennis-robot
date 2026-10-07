# Standalone intake — fixed motor, compliant tyre checkpoint

Date: 2026-08-27
Physical architecture: **`FIXED_MOTOR_COMPLIANT_TYRE`**
Decision: **architecture corrected; tyre model and direct-drive metrology still block static/dynamic validation**

Machine-readable checkpoint: [`config/standalone_intake_fixed_motor_checkpoint.json`](../../config/standalone_intake_fixed_motor_checkpoint.json)
Analysis-only CAD: [`cad/standalone-intake-fixed-motor/standalone-intake-fixed-motor-compliant-tyre.scad`](../../cad/standalone-intake-fixed-motor/standalone-intake-fixed-motor-compliant-tyre.scad)

## Executive correction

The intended physical intake has no compliant carriage. Both FIT0186 motors
are fixed to the local bridge, both wheel centres are fixed, and the purchased
adapter and Raid/Trencher wheel rotate about fixed, parallel longitudinal
X-Z axes. The wheel centres are mirrored in Y; the axes are not splayed in Y.
Compliance
comes from the regulation tennis ball plus local radial deformation of the
Trencher rubber tyre and its foam insert.

```text
fixed bridge/support
  -> fixed FIT0186
  -> native 6 mm D-shaft
  -> purchased 14-00012630 adapter
  -> 12 mm Raid interface
  -> compliant Trencher tyre/open-cell insert on a fixed wheel centre
```

The previous translating-pod guide study is preserved only as historical
traceability and is now marked `SUPERSEDED_WRONG_COMPLIANCE_ARCHITECTURE`.
Its rails, springs, hard stops, moving-motor cable loops, and 0–8 mm travel are
not current physical requirements. **As of 2026-08-27 the prismatic Xacro/SDF
carriage no longer exists**: it was removed outright (not set to zero travel)
together with its SDF spring patch and state interfaces, so it cannot be
re-enabled by a parameter. Each side of the simulated intake is now one rigid
coaxial assembly welded to the bridge, matching this architecture. Tyre
compliance is not represented by any joint; it is measured by the reduced-order
solver `scripts/run_standalone_intake_handoff_study.py`.

The corrected architecture is selected and unambiguous, but is not frozen.
Physical tyre testing, direct-drive metrology, and final printed-mount fit-up
remain pending. A translating mechanism is not current architecture.

## Standalone boundary and fixed datums

Included: one minimal fixed support/bridge datum, two fixed FIT0186 motors, two
purchased adapters, two fixed-centre Raid/Trencher wheels, one tennis ball,
and the tyre/ball contact model. Excluded: basket, launcher, chassis, compact
packaging, navigation, and perception.

Corrected side convention and wheel-to-motor axis:

```text
side = +1 left; side = -1 right
wheel centre = (470, side*90, 70) mm
wheel->motor axis = (sin(35 deg), 0, cos(35 deg))
OpenSCAD = rotate([0, 35, 0])
left  = (0.573576436, 0, 0.819152044)
right = (0.573576436, 0, 0.819152044)
```

With robot +X forward, wheel→motor is forward/upward. The same unoriented line
is downward/rearward in the opposite direction. Therefore the 35-degree
inclination is visible in side view; front view shows parallel symmetric
assemblies rather than a V.

Current Option A local fixed positions are:

```text
wheel centres: left  [470, +90, 70] mm; right [470, -90, 70] mm
motor centres: left  [523.629397, +90, 146.590716] mm
               right [523.629397, -90, 146.590716] mm
```

The compact model applies a common -100 mm X shift, producing wheel centres
`[370, +/-90, 70] mm` and motor centres
`[423.629397, +/-90, 146.590716] mm`. Each motor is 53.629 mm forward and
76.591 mm above its wheel centre because it lies 93.5 mm along the same axis.
There is no independent motor translation or rotation.

## Manufacturer evidence and qualification

Pro-Line's manufacturer chart identifies `#1170-10` as the Trencher 2.8-inch
M2 tyre on the Raid 6x30 removable-hex wheel, with a 4.87 x 2.86 inch envelope
(approximately 124 x 73 mm). The same manufacturer chart identifies included
12 mm narrow/wide adapters and the 6x30 interface. See the
[Pro-Line tyre selector](https://teams.prolineracing.com/images/TireSelector-poster.pdf)
and [Pro-Line 6x30 removable-hex chart](https://teams.prolineracing.com/images/6x30-RemovableHexes_1-10_MT-SC.pdf).

The selected product record specifies M2 rubber and an open-cell insert. A
current closely related Pro-Line Trencher HP/Raid page independently confirms
the 124 x 73 mm envelope, M2 compound, open-cell insert, removable 6x30 wheel,
and supplied 12/14 mm adapters, but it is a different belted product number and
is not substituted for the selected `PRO117010`: [manufacturer product page](https://www.prolineracing.com/product/1-10-trencher-hp-belted-f-r-2.8-mt-tires-mtd-12mm-14mm-blk-raid-2/PRO1016810.html).
The actual received product, insert, and hex variant must therefore be verified
before calibration.

These sources establish construction and envelope, not radial stiffness,
damping, bottom-out travel, felt/tread friction, or allowable compression. No
material property is inferred from “M2” or “open cell.”

## Raised bridge and printed motor mounts

The standalone CAD now raises the intact 18 mm bridge to Z = 190..208 mm while
retaining the fixed wheel centres, motor centres, and both 35-degree coaxial
stacks. The 30 mm-diameter, 70 mm-long analysis motor envelope reaches a
maximum Z of 183.865 mm, leaving 6.135 mm vertical clearance to the bridge
underside. Therefore the bridge no longer needs a large oblique motor opening.

Two independent 3D-printed mounts hang below the bridge, one for each motor.
The user-measured motor diameter is 30 mm and each mount grips exactly the
topmost 10 mm axial band of the motor body. The upper edge of the ring is flush
with the motor top face; it does not begin 10 mm lower. The CAD shows a
provisional 30.6 mm bore, 42 mm outer collar, bridge pad, and two supporting
webs. The 0.6 mm diametral print-fit allowance is a prototype assumption, not
a released tolerance.

The retained axes cross the raised bridge underside at
`[554.025, +/-90, 190] mm` and its top at
`[566.629, +/-90, 208] mm`. These points are reference datums for positioning
the two mounts; they are not bridge openings.

Current fixed-mount disposition:

- **wheel centres/axes:** defined and retained;
- **motor centres/axes:** defined and retained;
- **bridge mounting plane:** raised and defined at Z = 190 mm underside;
- **motor mounts:** two provisional printed 30 mm body clamps, each 10 mm deep;
- **motor-body clearance:** 6.135 mm vertical clearance to the intact bridge;
- **cable clearance:** the top face remains open, while the actual cable tail
  diameter, connector and minimum bend envelope remain to be measured;
- **adapter clearance:** analysis uses an unmeasured 20 mm envelope and 8 mm
  seating assumption, so physical clearance remains unproved;
- **wheel clearance:** opposite wheels do not intersect, but the 56 mm ball
  corridor intentionally requires elastic contact;
- **ball corridor:** nominal fixed closure is 10 mm for a 66 mm ball.

Do not change the fixed wheel or motor coordinates to repair mount details.
Before printing, confirm the 30 mm diameter along the chosen grip band, run a
short bore-fit coupon, measure the cable-tail bend envelope, and release the
clamp split plus bridge bolt pattern. Then check bolt edge distance and pull-out
capacity against the actual bridge material.

## Direct-drive stack gate remains open

The correct stack remains:

```text
FIT0186 face -> native 6 mm D-shaft -> 14-00012630
-> 12 mm hex -> installed Raid interface -> wheel
```

Exact shaft projection/flat/shoulder, adapter bore depth/type/internal stop,
set-screw landing, hex length and seating face, Raid pocket depth/orientation,
centre clearance, wheel seating, and axial retention remain unmeasured. The
complete checklist in the superseded guide report remains useful **only as a
metrology checklist**; its guide conclusions do not. Missing stack dimensions
do not imply or justify a carriage.

```text
INTAKE_DIRECT_DRIVE_STACK_PHYSICALLY_DEFINED = false
INTAKE_SHAFT_ENGAGEMENT_DEFINED = false
INTAKE_ADAPTER_SEATING_DEFINED = false
INTAKE_AXIAL_RETENTION_DEFINED = false
INTAKE_WHEEL_HEX_INTERFACE_DEFINED = false
```

## Fixed-nip tyre compliance requirement

Nominal unloaded ball diameter is 66 mm and the current fixed tread gap is
56 mm. The centered geometric closure is therefore:

```text
C = D_ball - gap = 66 - 56 = 10 mm
```

For ball diametral deformation `delta_b` and local inward radial tyre
deflections `delta_tL`, `delta_tR`:

```text
10 mm = delta_b + delta_tL + delta_tR
symmetry: delta_tL = delta_tR = (10 mm - delta_b) / 2
equilibrium: F_ball(delta_b) = F_tyre(delta_tL) = F_tyre(delta_tR)
```

The accepted calibrated ball loading law is
`F_ball = 107309.294 * delta_b^1.5` with metres and newtons. Tyre force is
unknown, so the equilibrium cannot be solved. The admissible deformation
partitions are:

| Ball deformation | Ball loading force | Required tyre deformation per wheel |
|---:|---:|---:|
| 0.000 mm | 0.00 N | 5.000 mm |
| 1.000 mm | 3.39 N | 4.500 mm |
| 2.000 mm | 9.60 N | 4.000 mm |
| 2.761 mm | 15.57 N | 3.619 mm |
| 3.000 mm | 17.63 N | 3.500 mm |
| 4.000 mm | 27.15 N | 3.000 mm |
| 5.000 mm | 37.94 N | 2.500 mm |
| 6.500 mm | 56.24 N | 1.750 mm |
| 8.000 mm | 76.78 N | 1.000 mm |
| 9.261 mm | 95.64 N | 0.369 mm |
| 10.000 mm | 107.31 N | 0.000 mm |

Thus the full required tyre-compliance test envelope is **0–5 mm radial
deflection per wheel over 0–approximately 107 N**. The highest-value region
anchored by the ball model's ITF preload-to-forward-test window is
approximately **0.37–3.62 mm per wheel at 15.57–95.64 N**. This is a required
envelope, not evidence that the tyre supplies it.

The 124 mm outer diameter and nominal 2.8 inch (71.12 mm) wheel diameter imply
about 26.44 mm gross radial construction depth. That gross depth includes
tread, carcass, foam, glue geometry, and void and is not allowable travel.
Available elastic travel and foam bottom-out remain physical measurements.

The 56 mm gap is a current nominal nip datum. Because the wheel axis is tilted
and the ball rises through the tread width, exact local contact order and local
closure vary with the ball path. The 10 mm result is the required centered
planar screen; later static analysis must use the actual tread/insert envelope
and fixed 3D path without moving wheel centres.

## Tyre contact-model strategy

Recommended progression:

1. **Physical effective radial law:** measure loading/unloading force versus
   local radial tyre deflection at several tread phases on both wheels.
2. **Analytical compliant tyre-contact layer:** keep wheel and motor frames
   fixed, calculate penetration into an undeformed tread envelope, and apply
   the calibrated tyre normal law/hysteresis at the contact patch. This is the
   preferred standalone model because it preserves the physical kinematics.
3. **Bounded sensitivity:** before a full fit, measured lower/upper force curves
   may bound stiffness/damping. A guessed range is not defensible and cannot
   validate passage.
4. **Deformable wheel model:** acceptable if solver support and measured
   material/foam data justify it, but more complex than required for the first
   checkpoint.

A scalar effective radial spring law is acceptable only after calibration and
must include loading/unloading hysteresis or a measured conservative bound.
The 0–8 mm prismatic assembly has been removed from the simulation entirely
(2026-08-27); it survives only in archived run records. If a numerical fallback
is ever used,
it must be named `SIMULATION_ONLY_TYRE_COMPLIANCE_SURROGATE`, leave physical
CAD/rest centres fixed, and move only a contact-layer degree of freedom—not
the motor body or manufacturing geometry.

```text
INTAKE_TYRE_COMPLIANCE_MODEL_DEFINED = false
INTAKE_TYRE_COMPLIANCE_MODEL_CALIBRATED = false
```

## Minimum physical tyre test

Use one actual selected Trencher/Raid wheel, its installed insert, and a
regulation ball in a rigid radial compression fixture:

- lock the wheel hub/axis without loading the motor shaft;
- support the ball against a low-friction instrumented platen so actuator
  motion is radial to the representative tread contact;
- measure force with a calibrated load cell (target range at least 0–150 N,
  resolution <=0.5 N) and actuator displacement (resolution <=0.05 mm);
- independently observe tyre carcass/tread displacement with a dial indicator
  or calibrated side video so tyre and ball contributions are distinguishable;
- test 0–5 mm tyre deflection or stop earlier on observed foam bottom-out,
  damage, or 120 N force;
- use <=0.25 mm displacement increments, three slow loading/unloading cycles,
  and at least five circumferential phases covering lug crown and tread valley;
- test both delivered wheels and record temperature, insert/hex variant,
  preconditioning, dwell, and recovery time.

At each force on loading, the calibrated ball contribution can be checked as
`delta_b = (F / 107309.294)^(2/3)`; direct tyre displacement observation remains
preferred, especially on unloading where ball and tyre hysteresis coexist.

Record force/deflection curves, hysteresis-loop area, permanent set after 1 and
10 minutes, rebound/recovery, local tread collapse, foam bottoming, lug
folding, ball-felt snagging, slip onset, visible felt/rubber/foam damage, and
repeatability. Add a slow tangential pull/rolling test at representative normal
loads to bound felt/tread traction; do not tune friction to produce capture.

This single fixture supplies the minimum evidence needed to define an
effective tyre law and informs the later friction sensitivity range.

## Static and dynamic validation gates

Static fixed-geometry passage is **not run**. With no tyre force law there is
no unique split of the 10 mm closure, no normal force, no demonstrated tyre
travel, and no bottom-out result. Once measured curves exist, solve the series
equilibrium above at each 3D contact position and report ball deformation,
left/right tyre deformation, common normal force, available travel, bottom-out
margin, and symmetry. Motors and wheel centres remain fixed.

Dynamic capture is also **not run**. It requires fixed motor frames, rotating
fixed-centre wheels, calibrated ball and tyre normal laws, measured/bounded
felt-tread traction, and a non-ideal FIT0186 torque-speed/current model. Record
contact order, ball path, normal forces, tyre/ball deformation, wheel speed,
motor torque/current/transient recovery, slip, capture, and downstream release.
The `mu=2.5` tread friction was removed on 2026-08-27: felt/tread friction is
now a swept bound (0.3 / 0.6 / 0.9) in both instruments, and ball-to-ramp
friction is swept as a third unmeasured coefficient (0.20 / 0.40 / 0.60).
`gz_ros2_control` remains an ideal velocity source, so wheel droop, torque and
current are declared NOT measurable in Gazebo and are never reported from it.
Neither friction bound is physical evidence.

## Repository translating-carriage audit

Classification applies to the intake only; basket-lift carriages are unrelated
and remain current.

### `CURRENT_SUPPORTING_EVIDENCE`

- `cad/collector-intake-v1/option-a/option-a.scad`: fixed wheel centres and one
  coaxial wheel/adapter/motor transform per side. Its metrology allowance and
  final bridge bracket remain incomplete as noted above.
- `config/compact_mechanical_contract.json` section
  `intake_direct_drive_orientation`: corrected parallel longitudinal axes and fixed rest
  centres are valid.
- `config/compact_cad_measurements.json` fixed-rest wheel/direct-drive
  envelopes are useful envelope evidence; “pod” naming does not imply motion.
- `docs/hardware/ordered-parts.md`: received FIT0186 and purchased adapter
  records remain current.
- `docs/hardware/prototype-purchase-list-el.md` and
  `chassis-layout-4wd-dual-intake-el.md`: corrected to fixed FIT0186 mounts,
  direct drive, and pending physical metrology.

### `HISTORICAL_SUPERSEDED`

- `cad/collector-intake-v1/params.scad`: former
  `carriage_outward_travel = 8` was a stale physical-looking datum; it is now
  explicitly named as a legacy simulation-surrogate value.
- `docs/archive/mechanism/intake/dual-wheel-intake-design-el.md`: prismatic
  spring carriage presented as physical design; archived as superseded.
- `docs/archive/mechanism/intake/compact-basket-launch-path-reevaluation-stop-report.md`
  and `compact-intake-carriage-physical-definition-stop-report.md`: treat missing
  carriage collision/geometry as a physical blocker. The physical blocker is
  instead the fixed bracket/opening and tyre contact definition.
- The ambiguous 5 mm shaft ruler datum remains an incomplete fixed-mount input,
  not authority for motion.

### `CURRENT_SIMULATION_SURROGATE`

- `ros2_ws/src/tennis_robot/urdf/tennis_robot.urdf.xacro` and
  `urdf/components/drivetrain.urdf.xacro`: 0–8 mm prismatic links; the latter
  also parents adapter/motor collisions to the moving link.
- `scripts/generate_robot_urdf.py`: patches 1000 N/m by default through
  `INTAKE_WHEEL_SPRING_K`; now labelled legacy surrogate.
- `urdf/components/ros2_control.urdf.xacro`: optional carriage state exposure.
- `tennis_robot/sim_physics_probe.py`,
  `scripts/sim_debug/analyze_intake_release_criteria.py`, and
  `scripts/sim_debug/run_native_intake_sweep.sh`: record/require/tune surrogate
  travel and spring rate.
- `tests/test_compact_mechanical_model.py`: asserts carriage links, prismatic
  joints, 8 mm travel, and moving motor/adapter ownership. These are tests of
  the legacy simulation representation, not the physical architecture.
- `config/compact_mechanical_contract.json` component link mapping uses
  `*_carriage_link`; this maps the current simulation surrogate. The fixed CAD
  bbox remains useful only at rest.
- No simulation world file independently defines an intake carriage. Current
  launch/world runs inherit it through generated URDF/SDF.
- `docs/mechanism/compact-mechanical-reconstruction-report.md` and
  `compact-handoff-regression-repair.md` retain model/telemetry evidence under
  an explicit `NOT_PHYSICAL_INTAKE_ARCHITECTURE` warning.

The surrogate is retained temporarily for reproducibility, not accepted as a
credible tyre model. Its historical successes cannot close any corrected
static/dynamic gate.

### `HISTORICAL_SUPERSEDED`

- `docs/archive/mechanism/intake/standalone-intake-guide-definition-report.md`,
  `docs/archive/mechanism/intake/config/standalone_intake_guide_definition.json`,
  and `cad/archive/intake/standalone-guide-definition/standalone-intake-guide-definition.scad`: now
  explicitly `SUPERSEDED_WRONG_COMPLIANCE_ARCHITECTURE`.
- `docs/archive/mechanism/intake/standalone-intake-validation-report.md` and
  `docs/archive/mechanism/intake/config/standalone_intake_checkpoint.json`:
  translating-pod checkpoint,
  bannered/status-marked superseded.
- `cad/flywheel-launcher-v0/compact-intake-pod-concept-study.scad`,
  `scripts/analyze_compact_intake_pod_concepts.py`, and
  `docs/archive/mechanism/intake/compact-intake-pod-concept-study-measurements.json`: moving
  pod/Candidate A packaging study.
- `docs/archive/mechanism/intake/compact-intake-pod-design-study.md`,
  `compact-intake-moving-pod-hardware-gate-stop-report.md`, and
  `compact-corrected-geometry-packaging-resolution-report.md`: moving-pod sweep
  results are historical and must not constrain the fixed physical intake.

## Stop disposition and minimum next evidence

The corrected task stops before static passage and dynamic capture because:

1. tyre/foam radial compliance and bottom-out are unmeasured;
2. no defensible bounded tyre contact law exists;
3. felt/tread traction is unmeasured;
4. the direct-drive seating/retention stack remains unmeasured; and
5. the fixed bridge bracket/body/cable opening is not physically defined.

The minimum next measurement is the one-wheel/one-ball radial compression test
above, accompanied by direct-drive and fixed-mount metrology. No carriage,
spring, rail, moving cable loop, or pod stop is part of that work.

## Final classifications

```text
INTAKE_PHYSICAL_ARCHITECTURE = FIXED_MOTOR_COMPLIANT_TYRE
INTAKE_MOTORS_FIXED_TO_BRIDGE = true
INTAKE_TRANSLATING_MOTOR_PODS_REQUIRED = false
INTAKE_LINEAR_GUIDES_REQUIRED = false
INTAKE_RETURN_SPRINGS_REQUIRED = false
INTAKE_POD_HARD_STOPS_REQUIRED = false

INTAKE_35_DEGREE_LONGITUDINAL_AXES_VALIDATED = true
INTAKE_FRONT_VIEW_AXES_PARALLEL = true
INTAKE_WHEEL_ADAPTER_MOTOR_COAXIAL = true
INTAKE_MOTOR_Y_ALIGNED_WITH_WHEEL_CENTRE = true
INTAKE_MOTOR_FORWARD_OF_WHEEL_CENTRE = true

INTAKE_FIXED_BRIDGE_MOTOR_MOUNT_PHYSICALLY_DEFINED = false
INTAKE_DIRECT_DRIVE_STACK_PHYSICALLY_DEFINED = false
INTAKE_SHAFT_ENGAGEMENT_DEFINED = false
INTAKE_ADAPTER_SEATING_DEFINED = false
INTAKE_AXIAL_RETENTION_DEFINED = false
INTAKE_WHEEL_HEX_INTERFACE_DEFINED = false

INTAKE_TYRE_COMPLIANCE_REQUIRED = true
INTAKE_TYRE_COMPLIANCE_PHYSICALLY_MEASURED = false
INTAKE_TYRE_COMPLIANCE_MODEL_DEFINED = false
INTAKE_TYRE_COMPLIANCE_MODEL_CALIBRATED = false

INTAKE_STATIC_FIXED_GEOMETRY_PASSAGE_VALIDATED = false
INTAKE_DYNAMIC_CAPTURE_VALIDATED_IN_SIM = false
INTAKE_TREAD_FRICTION_PHYSICALLY_VALIDATED = false

INTAKE_PREVIOUS_GUIDE_STUDY_SUPERSEDED = true
INTAKE_ACTIVE_DESIGN_UNAMBIGUOUS = true
INTAKE_FIXED_MOTOR_COMPLIANT_TYRE_CURRENT = true
INTAKE_COMPLIANCE_SOURCE_IS_TYRE_AND_BALL = true
INTAKE_ACTIVE_TRANSLATING_POD_DESIGN = false
INTAKE_ACTIVE_BEARING_SUPPORTED_REMOTE_SHAFT_DESIGN = false
INTAKE_SUPERSEDED_ARTIFACTS_ARCHIVED = true
INTAKE_TRACEABILITY_PRESERVED = true
INTAKE_SIMULATION_SURROGATES_EXPLICITLY_LABELLED = true
INTAKE_STALE_ACTIVE_REFERENCES = 0
INTAKE_ARCHITECTURE_FROZEN_PROVISIONALLY = false
INTAKE_PHYSICAL_VALIDATION_PENDING = true
INTAKE_READY_FOR_COMPLETE_ROBOT_INTEGRATION = false
PHYSICAL_HARDWARE_PENDING = true
```
