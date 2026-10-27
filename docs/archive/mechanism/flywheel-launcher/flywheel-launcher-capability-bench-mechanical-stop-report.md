# Isolated flywheel-launcher capability bench — mechanical completeness stop

Date: 2026-08-25

## Decision

The requested capability trials were stopped at validation Gate A. The current
standalone CAD/Xacro representation is not the complete physical launcher
assembly required by the task. It contains two 200 x 50 mm flywheels, two
cradle plates and ideal velocity joints, but it does not visibly or physically
represent either D5065 motor, the motor mounting solution, shafts, wheel hubs,
attachment fasteners, axial retention, or pitch hardware.

The manufacturer drawing closes the D5065 envelope question but cannot define
the launcher-specific load path. No launch, contact sensitivity, motor-power,
RPM, spin, angle, trajectory, court, or repeatability trial was run. This is the
required stop rather than a negative capability result.

The complete machine-readable audit and final classifications are in
`docs/archive/mechanism/flywheel-launcher/config/flywheel_launcher_capability_gate_results.json`.

## Preserved independent ball reference

`config/tennis_ball_compliance_calibration_results.json` remains unchanged at
SHA-256
`a7aa85327219d624c562b4c528f946bb62e1326d0b58dc7064f7439a07731a8a`.
No ball parameter was tuned, scaled, or refit, and no launcher result was used
for contact-model fitting.

## Gate A audit

The current source is
`ros2_ws/src/tennis_robot/urdf/components/flywheel_launcher_module.urdf.xacro`.
Its fixed link combines two plate volumes and two nominal 0.49 kg motor masses
into one box-envelope inertia. A mass comment is not motor geometry or a motor
mounting solution. The only moving links are provisional 0.40 kg solid
cylinders, with 0.002 kg m^2 axial inertia each.

The source CAD in `cad/flywheel-launcher-v0/launcher-envelope.scad` is explicitly
a non-manufacturing packaging model. Its side plates, wheel envelopes, guards,
feed keep-out and exit guide do not define motor mounting or wheel attachment.
The standalone wrapper only selects the side-by-side orientation and 20 degree
pitch.

Existing supported launcher datums are:

- 66 mm reference ball diameter: `EXISTING_CAD_DATUM`;
- 200 x 50 mm wheels: `EXISTING_CAD_DATUM`;
- 58 mm nip: `EXISTING_CAD_DATUM`;
- 258 mm wheel-centre spacing: `DERIVED`;
- 256 x 314 x 8 mm cradle plates: `DERIVED`;
- 20 degree fixed pitch transform: `EXISTING_CAD_DATUM`.

Important unresolved assembly datums are:

- launcher-specific motor plate/bracket geometry: `MISSING`;
- wheel hub geometry and material: `MISSING`;
- shaft engagement and attachment fasteners: `MISSING`;
- axial retention solution: `MISSING`;
- actual wheel mass and inertia: `PROVISIONAL`;
- motor rotor inertia: `MISSING`;
- pitch pivot, bearings, lock and datum: `MISSING`.

These affect rotating inertia, torque transfer, bearing loads, wheel location,
run-out and the assembly collision envelope. Guessing them would make RPM droop,
recovery, energy accounting and even static nip credibility dependent on an
invented assembly.

## D5065 evidence found

The official ODrive product page and shaft drawing support a 50 mm body
diameter, 65 mm body length, 8 mm primary shaft with flat, 30 mm primary shaft
projection, 24 mm flat length, 0.5 mm flat depth, 16 mm secondary shaft
projection, and four 4 mm holes on a 30 mm pitch circle. The product page also
states 7 pole pairs, 12 stator slots, dual shafts and a winding thermistor.

The official motor characteristics support 270 RPM/V, 0.031 N m/A,
515.67 RPM/N m speed/torque gradient, 39 mOhm phase-neutral resistance,
16 uH phase-neutral inductance, 45 A free-air / 65 A forced-air continuous
current and 85 A three-second peak current. ODrive explicitly classifies these
figures and its torque-speed construction as approximate and dependent on the
application and cooling.

Sources:

- https://shop.odriverobotics.com/products/odrive-custom-motor-d5065
- https://docs.odriverobotics.com/v/latest/hardware/odrive-motors.html

The repository's 12.8 V bus and 18–20 A per-motor current limit are provisional
purchase-list starting points, not a selected and validated motor-controller
contract. A reduced-order motor model was therefore not installed into the
bench after Gate A failed.

## Contact status

The calibrated ball normal compliance remains authoritative. Launcher tyre
normal compliance and tyre/ball friction remain independent unresolved
quantities:

```text
LAUNCHER_TYRE_FRICTION_CALIBRATED = false
LAUNCHER_TYRE_NORMAL_COMPLIANCE_VALIDATED = false
```

A bounded sensitivity study would be appropriate only after mechanical
completeness, geometry alignment and motor-model gates pass. Running it now
would violate the mandated validation order.

## Visual evidence status

The existing standalone render at `docs/images/flywheel-launcher-authoritative-standalone.png`
visibly shows the failure: flywheels and two plates are present, but no D5065
motor body, mounting pattern, bracket, hub, shaft retention or pitch mechanism
is visible. It cannot serve as the required proof-of-part render.

No requested performance plots were produced because there is no valid launch
trial. Empty plots or plots from ideal velocity joints would imply evidence
that does not exist.

## Required next evidence

Before resuming Gate A, define or measure the selected wheel/hub/retention
assembly, wheel mass properties, motor-to-cradle mount and complete pitch
hardware. The assembly CAD must show two instances of the manufacturer-backed
D5065 envelope and expose the actual rotating inertia and shaft load path. Once
that is authoritative, mirror it into Xacro/SDF and resume at Gate A; do not
skip directly to motor or contact trials.

## Final classifications

`CALIBRATED_BALL_MODEL_UNCHANGED` is true. Every requested launcher validation
or capability classification is false because the prerequisite mechanical
gate failed. In particular,
`STANDALONE_FLYWHEEL_LAUNCHER_MECHANICALLY_COMPLETE`,
`D5065_MOTOR_MODEL_EVIDENCE_SUPPORTED`, `D5065_POWER_LIMITS_ENFORCED`, and
`LAUNCHER_CAPABILITY_MAP_GENERATED` are false. These values mean “not
validated,” not “physically incapable.”
