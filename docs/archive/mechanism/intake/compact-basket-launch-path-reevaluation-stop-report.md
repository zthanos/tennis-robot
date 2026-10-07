# Compact basket launch-path re-evaluation — model-completeness STOP

> **Archived: `HISTORICAL_SUPERSEDED`.** This report assumes physical intake
> carriages. Current architecture fixes both motors and wheel centres; see
> [`standalone-intake-fixed-motor-compliant-tyre.md`](../../../mechanism/standalone-intake-fixed-motor-compliant-tyre.md).

Date: 2026-08-25

## Decision

```text
COMPACT_SIMULTANEOUS_INTAKE_AND_LAUNCHER_MODEL_REQUIRED
```

The freshly generated `compact` model contains the intake, basket and
flywheel-launcher link hierarchies simultaneously. However, it does **not**
contain collision geometry for the two physical intake carriages. The
carriages are visual-only bodies in both generated URDF and generated SDF.

The requested swept-path study explicitly requires the complete installed
intake, including both carriages, and prohibits making either mechanism
visual-only. Clearance measurements made with the carriage solids absent would
therefore not describe the intended compact machine. The trajectory study is
stopped at its architecture/model-completeness gate.

No authoritative CAD, Xacro, URDF source, controller, Throwing Mode, basket
actuator, PARKED datum, receiving channel, tire pocket, launcher datum,
battery, bridge or LiDAR geometry was modified.

## A. Fresh compact architecture inventory

Generation command:

```text
source /opt/ros/jazzy/setup.bash
python3 scripts/generate_robot_urdf.py \
  --packaging-variant compact \
  --output /tmp/compact-path-study.urdf \
  --sdf-output /tmp/compact-path-study.sdf \
  --controllers-config ros2_ws/src/tennis_robot/config/controllers.yaml
```

Top-level presence result:

```text
INTAKE_PRESENT = true
FLYWHEEL_LAUNCHER_PRESENT = true
BASKET_PRESENT = true
BOTH_PRESENT_SIMULTANEOUSLY = true
COMPLETE_INTAKE_COLLISION_MODEL = false
```

| Assembly/component | Generated link | Collision geometry | Result |
|---|---|---:|---|
| intake left wheel | `intake_wheel_left_link` | 1 cylinder | physical |
| intake right wheel | `intake_wheel_right_link` | 1 cylinder | physical |
| intake left carriage | `intake_wheel_left_carriage_link` | **0** | **visual-only; blocker** |
| intake right carriage | `intake_wheel_right_carriage_link` | **0** | **visual-only; blocker** |
| intake cheeks/flanges | `compact_intake_cheeks_link` | 20 boxes | physical |
| handoff ramp | `compact_handoff_ramp_link` | 1 relieved mesh | physical |
| receiving interface/entry hood | `compact_fixed_entry_hood_link` | 1 relieved mesh | physical, fixed to chassis |
| basket bin | `basket_link` | 1 relieved mesh | physical, moving |
| launcher cradle | `flywheel_launcher_frame_link` | 2 plates | physical |
| left flywheel | `flywheel_left_link` | 1 cylinder | physical |
| right flywheel | `flywheel_right_link` | 1 cylinder | physical |

The carriage links are real prismatic children of `compact_bridge_link`, and
the wheel links are their rotating children. Their missing collision bodies
therefore cannot be substituted by the wheel collisions: a carriage body and
its wheel are separate moving solids.

Fresh SDF inspection gives the same result for each carriage:

```text
collision_count = 0
visual_count = 1
```

## B. Exact model-generation defect

`ros2_ws/src/tennis_robot/urdf/components/drivetrain.urdf.xacro` creates a
30 x 30 x 12 mm `carriage_vis` box on each carriage link, but creates no
corresponding `<collision>` element. `scripts/generate_robot_urdf.py` preserves
that omission when converting the compact URDF to SDF.

Repository search found no separate authoritative carriage solid or collision
mesh in the compact CAD/contract that could be included in a mechanical path
boolean instead. Thus this is not only a URDF export omission: the physical
carriage envelope is absent from the current authoritative collision contract.

The current compact generated-model test suite passes despite this defect. It
asserts that the carriage links exist and have positive inertia, but does not
assert that they have collision geometry.

## C. Required correction before path analysis

1. Define the authoritative physical envelope for each complete compliant
   carriage/motor mount (not merely a convenient copy of the current visual).
2. Add that geometry as collision bodies owned by
   `intake_wheel_left_carriage_link` and
   `intake_wheel_right_carriage_link`.
3. Add the carriage envelopes to the compact CAD measurements and mechanical
   collision contract so OpenSCAD and generated-model checks describe the same
   solids.
4. Extend `tests/test_compact_mechanical_model.py` to require non-empty
   carriage collision geometry in freshly generated URDF and SDF.
5. Regenerate and validate the simultaneous compact model, then restart the
   PARKED baseline and X/Z configuration-space study from the current protected
   PARKED datum.

## D. Requested report sections not run

Sections B through M of the requested path report (fresh PARKED booleans,
historical comparison, clearance thresholds, transition interval, chassis
opening, guide dimensions, forward/reverse sweeps, orientation, holders and
stability) are deliberately **not reported**. Running them while omitting the
carriage swept volumes could incorrectly classify a path as valid.

No historical 96 mm, 20 mm, 60–61 mm, 72 mm or 62.4 mm result was reused as a
current design input.

## E. Preserved classifications

No fresh evidence from this architecture inventory invalidates the already
validated PARKED package or intake handoff:

```text
COMPACT_PARKED_PACKAGING_VALIDATED_IN_SIM = true
COMPACT_INTAKE_HANDOFF_VALIDATED_IN_SIM = true
COMPACT_BASKET_TWO_GUIDE_PATH_GEOMETRICALLY_VALID = not evaluated
COMPACT_BASKET_LAUNCH_PATH_VALIDATED_IN_SIM = false
LAUNCHER_BRIDGE_FIXED_INTERFERENCE_PENDING = true
FINAL_BASKET_ORIENTATION_MECHANISM_PENDING = true
BALL_LAUNCH_PHYSICS_NOT_VALIDATED = true
PHYSICAL_HARDWARE_PENDING = true
```

The known fixed launcher/bridge issue was not re-measured because the required
path study stopped before its fresh-baseline phase. It remains classified as a
separate fixed/fixed packaging issue, not permission to remove or move the
launcher.
