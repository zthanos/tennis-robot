# Standalone flywheel launcher — provisional frozen baseline

Checkpoint date: 2026-08-26
Scope: isolated launcher module; not the complete robot
Status: **AUTHORITATIVE provisional architecture, SIMULATION_VALIDATED evidence, PHYSICAL_VALIDATION_PENDING hardware**

## Purpose and decision

This is the single entry point for building and testing the standalone flywheel module without reconstructing the project history. The architecture is provisionally frozen so the next work is physical contact and hardware validation, not another simulation optimization or launcher redesign.

`FLYWHEEL_ARCHITECTURE_FROZEN_PROVISIONALLY = true`

The practical training target is 12–14 m/s. This is a design target, not a physically demonstrated capability.

## Status vocabulary

- **AUTHORITATIVE:** current repository datum or decision for the provisional standalone build.
- **PROVISIONAL:** controlled assumption that must be replaced by received-part measurement.
- **SIMULATION_VALIDATED:** passed the isolated native Gazebo or deterministic repository gate stated by the linked evidence.
- **PHYSICAL_VALIDATION_PENDING:** no manufactured assembly or real-ball result supports the claim yet.

## Frozen provisional geometry

The **AUTHORITATIVE** standalone configuration is:

- two side-by-side, counter-rotating, direct-drive flywheels;
- two ODrive D5065-270KV motors;
- flywheel envelope 200 mm diameter × 50 mm axial width;
- wheel centres at launcher-local `(x,y,z) = (0,+129,0)` and `(0,-129,0)` mm;
- wheel-centre spacing 258 mm;
- geometric nip 58 mm for a nominal 66 mm tennis ball;
- complete-module pitch 20°;
- two cradle panels, each 256 × 314 × 8 mm, centred at local `z=±43 mm`;
- upper-panel inside/outside faces at local `z=+39/+47 mm`;
- motor mounting-face centres at `(0,±129,+47)` mm;
- corrected post-nip exit relief in both panels.

The launcher-local origin is the midpoint of the two wheel axes at the nip. Local `x` follows the ball exit direction before the 20° module pitch is applied, local `y` joins the wheel centres, and local `z` is the common wheel/motor shaft direction. World placement belongs to the bench or future robot integration; it is not owned by this module.

## Mechanical and motor architecture

Each D5065 mounts directly to the outside face of the 8 mm upper panel. The motor body remains outside the cradle; the primary shaft points inward and is coaxial with its wheel. The hub/arbor and flywheel occupy the launcher interior. There is no separate motor bracket, external bearing-supported shaft, coupler, belt, pulley, gearbox, independent motor-pitch mechanism or printed high-speed torque hub.

The whole module supplies the 20° pitch. The pitch does not come from a separate motor bracket.

The direct-panel mount is **SIMULATION_VALIDATED** for geometry and has passed a conservative structural screen. It is not physically structurally validated. Motor-feature thread type/depth, final fasteners, panel alloy/temper, lead exit, tool access, balance, runout, vibration and dynamic panel response still require hardware evidence.

## Motor assumptions

Repository-captured manufacturer evidence for the D5065 includes:

- 50 mm body diameter, 65 mm body length and approximately 0.49 kg mass;
- 8 mm primary shaft with flat, 30 mm projection, 24 mm flat length and 0.5 mm flat depth;
- dual-shaft construction with an 8 mm secondary shaft;
- 12N14P winding/rotor arrangement, or 7 pole pairs;
- four nominal 4 mm mounting features on a 30 mm pitch circle;
- 270 rpm/V, torque constant 0.031 N·m/A and phase-neutral resistance 0.039 Ω;
- NTC 10 kΩ thermistor and phase leads with 4 mm bullet connectors.

The bench uses a **PROVISIONAL** 12.8 V bus and 20 A limit per motor, corresponding to 0.62 N·m. The effort-limited native capability controller models speed, current/back-EMF, droop and recovery. Motor rotor inertia, inverter losses, battery sag and thermal accumulation remain unmeasured.

## Wheel and hub interface

The selected **PROVISIONAL** wheel candidate is the already evaluated AliExpress electric-skateboard/off-road wheel:

- seller nominal diameter 200 mm;
- seller nominal width 50 mm;
- seller nominal axle/bore datum 10 mm;
- aluminium-alloy hub plus rubber tyre;
- seller-listed mass approximately 900 g.

None of those seller values is measured hardware. The 10 mm datum must be inspected first. If it is a bearing inner race rather than a rigid, concentric through-bore with clampable aluminium faces, mechanical Gate A reopens before any final hub is manufactured.

The current analysis interface is:

```text
D5065 8 mm D-shaft
  -> one-piece metal positive-clamping adaptor/arbor
  -> 10 mm wheel interface
  -> removable positive axial retention
```

The study uses approximately 21.5 mm D-shaft engagement, a split clamp registered to the flat, and removable distal retention. This is **PROVISIONAL simulation geometry**, not released manufacturing CAD. A printed torque-transmitting hub is prohibited. Final arbor diameter, wheel shoulder, clamping faces, retention, fasteners and assembly tooling depend on measurements of the purchased wheel and motor.

## Cradle, service cutouts and exit relief

The shaped lower and upper-panel reliefs are the active simulation geometry. The upper panel also contains provisional 12 mm shaft/service cutouts. Those cutouts are not a final drill drawing: their released diameter depends on the final hub, clamp-tool and retention installation path.

The CAD cylinder downstream of the wheels is an exit/reference keep-out only. It is not a barrel, tube or physical guide. The native post-nip audit found the corrected path noncontacting, so no physical post-wheel barrel is currently required.

`FLYWHEEL_POST_NIP_PATH_VALIDATED_IN_SIM = true`

## Standalone model ownership

The current isolated model is [`flywheel_launcher_module.urdf.xacro`](../../ros2_ws/src/tennis_robot/urdf/components/flywheel_launcher_module.urdf.xacro), instantiated by [`flywheel_launcher_bench.urdf.xacro`](../../ros2_ws/src/tennis_robot/urdf/flywheel_launcher_bench.urdf.xacro). It owns the provisional wheel mass/inertia, motor bodies, shafts, hub envelopes, relieved panels and effort-limited joints.

The nominal wheel mass is the seller-listed 0.90 kg and remains explicitly provisional. Polar inertia is bounded from `0.5mR²` to `mR²`; the nominal model uses the midpoint law plus the analysis hub. D5065 rotor inertia remains a separate sensitivity because no manufacturer-backed value was found.

No complete-robot Xacro or geometry is an input to this checkpoint. Complete-robot packaging studies are separate and must not silently override standalone datums.

## Tennis-ball model

The normal contact law is the independently calibrated compliant tennis-ball model in [`tennis_ball_compliance_calibration_results.json`](../../config/tennis_ball_compliance_calibration_results.json) and [`gazebo/models/tennis_ball_compliant/model.sdf`](../../gazebo/models/tennis_ball_compliant/model.sdf). Launcher results were not used to refit it.

The felt-to-tread tangential coefficient is not calibrated. Diagnostic μ values are simulation sensitivity parameters and must never be copied into a wheel material specification.

## Simulation evidence

The final capability campaign established a corrected, noncontacting post-wheel path, converged launch contact, bounded energy accounting and repeatable operation. The original physically uncalibrated μ≤0.9 sensitivity envelope saturated at 8.2475 m/s.

The root-cause campaign then demonstrated:

- the plateau is caused primarily by felt/tread traction transfer, coupled to finite contact duration;
- increasing wheel RPM at fixed μ adds almost no exit speed after approximately 120–160 rad/s;
- motor torque and stored wheel energy are not the primary current limitation;
- the first converged diagnostic point at or above 14 m/s is 14.38935 m/s at μ=3.0, ±160 rad/s and 0.25 ms timestep;
- μ=3.0 is not a validated material property;
- the μ=10 result is an ideal diagnostic upper bound, not a physical launch result.

Therefore the geometry has **SIMULATION_VALIDATED kinematic potential** above 14 m/s, but physical exit speed, range and spin remain unvalidated.

## Physical validation plan

The next engineering phase is a standalone physical bench:

1. Receive and measure both wheels and both motors before releasing the arbor or panel service cutout.
2. Screen regulation new and worn tennis-ball felt against candidate urethane, nitrile/NBR and butyl tread coupons at representative normal load, surface speed, slip speed, temperature and wear cycles.
3. Manufacture the controlled metal hub/arbor only after the 10 mm wheel interface, hub faces and motor shaft are measured.
4. Verify balance, radial/axial runout, retention, panel vibration and structural behaviour across the intended speed range behind a suitable guard.
5. Instrument wheel speed, current, temperature, slip, exit speed, repeatability and felt wear.
6. Demonstrate approximately 12–14 m/s before beginning trajectory placement and spin validation.

Acceptance requires acceptable wheel recovery, slip, repeatability, felt wear, vibration/runout, motor current/temperature and structural behaviour.

## Reopen criteria

If physical testing reaches 12–14 m/s with the acceptance topics above, retain this architecture and proceed to trajectory, placement and spin validation.

If it does not, reopen in this order:

1. tread material and dynamic contact behaviour;
2. contact arc, nip and normal load;
3. wheel mechanical interface;
4. only then larger launcher architecture changes.

Do not infer that the D5065 motors are inadequate before those gates are resolved.

Mechanical Gate A also reopens immediately if the received 10 mm wheel datum is a bearing race, the measured wheel mass/inertia leaves the accepted sensitivity bounds, D-shaft engagement cannot reach approximately 21.5 mm, or the clamp, retention, runout, balance, panel or vibration checks fail.

## Manufacturing release status

Safe now:

- reproduce the standalone model and regression tests;
- prepare measurement fixtures, guarded bench instrumentation and coupon tests;
- prepare non-manufacturing assembly and test procedures.

Hold until received hardware is measured:

- final hub/arbor machining dimensions;
- final upper-panel service cutout;
- wheel-side fasteners, thread engagement and clamp preload;
- final assembly order/tool clearance;
- balance/runout/retention release and any structural reinforcement decision.

The panel is structurally screened, not physically validated. No final high-speed rotating component is released for manufacture by this checkpoint.

## Authoritative evidence chain

- checkpoint status: [`config/flywheel_launcher_checkpoint.json`](../../config/flywheel_launcher_checkpoint.json)
- provisional mechanical Gate A: [`flywheel-wheel-candidate-provisional-gate-a.md`](flywheel-wheel-candidate-provisional-gate-a.md)
- direct-panel mounting evidence: [`flywheel-launcher-direct-drive-mechanical-definition.md`](flywheel-launcher-direct-drive-mechanical-definition.md)
- post-nip audit: [`flywheel-launcher-post-nip-exit-corridor-audit.md`](flywheel-launcher-post-nip-exit-corridor-audit.md)
- capability campaign: [`flywheel-launcher-capability-validation-report.md`](flywheel-launcher-capability-validation-report.md)
- transfer root cause: [`flywheel-energy-transfer-root-cause-report.md`](flywheel-energy-transfer-root-cause-report.md)
- standalone reconstruction/ball gate: [`flywheel-launcher-standalone-reconstruction-ball-model-gate.md`](flywheel-launcher-standalone-reconstruction-ball-model-gate.md)
- active CAD sources: [`launcher-envelope.scad`](../../cad/flywheel-launcher-v0/launcher-envelope.scad), [`provisional-wheel-hub-gate-a-study.scad`](../../cad/flywheel-launcher-v0/provisional-wheel-hub-gate-a-study.scad), [`provisional-cradle-exit-clearance.scad`](../../cad/flywheel-launcher-v0/provisional-cradle-exit-clearance.scad)
- archived superseded attempts: [`docs/archive/mechanism/flywheel-launcher/README.md`](../archive/mechanism/flywheel-launcher/README.md)

## Final checkpoint classifications

- `FLYWHEEL_ARCHITECTURE_FROZEN_PROVISIONALLY = true`
- `FLYWHEEL_STANDALONE_MODEL_DEFINED = true`
- `FLYWHEEL_POST_NIP_PATH_VALIDATED_IN_SIM = true`
- `FLYWHEEL_KINEMATIC_14MPS_POTENTIAL_DEMONSTRATED = true`
- `FLYWHEEL_TARGET_EXIT_SPEED_MIN_M_S = 12`
- `FLYWHEEL_TARGET_EXIT_SPEED_MAX_M_S = 14`
- `FLYWHEEL_MOTOR_REDESIGN_REQUIRED = false`
- `FLYWHEEL_NIP_REDESIGN_REQUIRED = false`
- `FLYWHEEL_TRACTION_IS_PRIMARY_UNCERTAINTY = true`
- `FLYWHEEL_WHEEL_CANDIDATE_SELECTED_PROVISIONALLY = true`
- `FLYWHEEL_WHEEL_PHYSICALLY_MEASURED = false`
- `FLYWHEEL_FINAL_HUB_RELEASED_FOR_MANUFACTURE = false`
- `FLYWHEEL_PHYSICAL_EXIT_SPEED_VALIDATED = false`
- `FLYWHEEL_PHYSICAL_RANGE_VALIDATED = false`
- `FLYWHEEL_PHYSICAL_SPIN_VALIDATED = false`
- `FLYWHEEL_PHYSICAL_HARDWARE_PENDING = true`
- `FLYWHEEL_READY_FOR_NEXT_PROJECT_PHASE = true`
