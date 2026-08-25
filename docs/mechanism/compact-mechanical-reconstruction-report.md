# Compact mechanical reconstruction and validation

Date: 2026-08-25

## Result

The `compact` Xacro is now a physics decomposition of
`cad/flywheel-launcher-v0/compact-packaging-study.scad`, rather than a visual
packaging approximation. It includes the plywood bridge, curved intake cheeks,
handoff ramp, compliant intake-wheel carriages, two basket guides, basket,
the two launcher cradle plates, motors, and two independently actuated
flywheels. Fixed assemblies remain separate links/contact owners in the SDF.

The reconstruction is dimensionally aligned and dynamically usable, but it
cannot honestly be declared collision-clean: exact OpenSCAD boolean
intersections prove that the source CAD itself has two physical interferences.
The Xacro deliberately preserves those source solids. Removing them requires a
mechanical design decision, not a URDF correction.

## Coordinate and transform convention

- Units are metres and radians in URDF; the CAD source is millimetres and
  degrees.
- The comparison frame is the CAD ground frame: +X forward, +Y left, +Z up.
- `base_footprint` is on the ground plane. `base_link` is at Z = 0.045 m.
- The CAD functional assemblies use the source's -0.100 m X packaging shift.
- The launcher nip frame is X = 0.460 m, Z = 0.215 m with -20 degrees pitch.
- Basket collection is the zero lift pose. Raised is +0.100 m. The configured
  launch study is the raised pose plus 12 degrees about the CAD pivot.

## Kinematic hierarchy and actuation

| Component | Authoritative CAD module/source | CAD parent | Fixed/moving | Mass source | Collision role | URDF joint |
|---|---|---|---|---|---|---|
| Chassis/body | `fixed_chassis_and_drive()` | world assembly | floating rigid body | existing measured/estimated chassis model | ground-clearance body | `base_footprint_joint` |
| Drive motors | `fixed_chassis_and_drive()` | chassis | fixed | existing 0.65 kg each | motor envelope | four motor mount joints |
| Drive wheels | `fixed_chassis_and_drive()` | motors/chassis | rotating | existing 0.25 kg each | support and traction | four wheel joints |
| Battery/electronics | `rear_electronics_packaging()` and motion-tray sources | chassis | fixed | existing battery/part data | packaging bodies | fixed mounts |
| LiDAR | compact study datum | chassis | fixed | existing sensor estimate | sensor housing; scan datum | fixed mount |
| Plywood bridge | `compact_bridge()` | chassis | fixed | plywood/doubler volume estimate | structural and contact body | `compact_bridge_mount_joint` |
| Intake carriages | `shifted_option_a_intake()` / Option A intake | bridge | compliant translation | 0.05 kg each estimate | wheel support/contact ownership | two carriage prismatic joints |
| Intake wheels | Option A `intake_wheel()` | carriages | rotating | 0.20 kg each estimate | ball-contact rollers | two intake wheel joints |
| Funnel cheeks/guards | Option A curved cheek modules | bridge | fixed | HDPE volume estimate | ball funnel/contact surface | `compact_intake_cheeks_joint` |
| Handoff ramp | `compact_handoff_ramp()` | bridge | fixed | sheet/printed estimate | ball-transfer surface | `compact_handoff_ramp_joint` |
| Basket guides | `compact_basket_guides()` | chassis | fixed | 0.82 kg hollow-aluminium envelope estimate | two motion-constraint envelopes | `basket_rails_mount_joint` |
| Basket/bin/hood | `basket_collect_pose()` and basket-bin-v2 sources | guide path | two configured poses | 1.20 kg empty + measured ball payload | storage, chute, and hood contact | `basket_launch_pose_joint` |
| Raised holders | `compact_raised_basket_holders()` | chassis | launch-study configuration only | 0.457 kg plywood estimate | real flange support surfaces | `basket_raised_holders_mount_joint` |
| Launcher cradle/motors | `launcher_cradle()` | bridge | fixed | plate volume + documented motor mass | shaft support/contact body | `flywheel_launcher_mount_joint` |
| Flywheels | `wheel_envelope()` physical wheels | cradle | independently rotating | volume/density estimate | ball-contact launch wheels | two flywheel joints |

| Assembly | Parent | Joint / state | Controller and feedback |
|---|---|---|---|
| Chassis | `base_footprint` | fixed | Gazebo body pose / odometry |
| Four drive wheels | `base_link` | continuous | `diff_drive_controller`; wheel joint state and odometry |
| Plywood bridge | `base_link` | fixed, preserved | structural/contact owner |
| Intake carriages | `compact_bridge_link` | prismatic, 0..8 mm compression | passive Gazebo spring/damper; optional joint-state exposure |
| Intake wheels | respective carriage | continuous, CAD 124 x 73 mm | `intake_wheel_velocity_controller`; joint state |
| Intake cheeks | `compact_bridge_link` | fixed, preserved | contact owner |
| Handoff ramp | `compact_bridge_link` | fixed, preserved | contact owner |
| Basket rails | `base_link` | fixed, preserved | structural/contact owner |
| Basket guide path | `basket_rails_link` | prismatic, 0..100 mm operating travel | `basket_lift_controller`; joint state |
| Basket | guide path | fixed collection/launch transform | no invented tilt actuator or controller |
| Raised holders | `base_link` | fixed, configured launch study only | engagement/retention mechanism pending |
| Launcher cradle | `compact_bridge_link` | fixed, preserved | structural/contact owner |
| Left/right flywheels | launcher cradle | independent continuous joints | `flywheel_velocity_controller`; joint state |

The configured 12-degree basket launch transform is a pose-study parameter,
not a claim that tilt hardware exists. No launcher exit rail was invented: the
CAD exit guide is an envelope, not a physical part.

## Geometry traceability

The validation wrapper exports each authoritative OpenSCAD module on its own.
`scripts/export_compact_cad_envelopes.py` invokes OpenSCAD with reference balls,
keep-outs, feed envelopes, and guards disabled, then derives STL bounds. The
generated URDF collision bodies are transformed through the actual joint tree
and compared against `config/compact_mechanical_contract.json`.

| Component | CAD bounds, min to max (mm) | URDF bounds, min to max (mm) | Max delta | Result |
|---|---|---|---:|---|
| Chassis | (-460,-290,38) to (460,290,52) | (-460,-290,38) to (460,290,52) | <0.001 mm | PASS |
| Drive wheels | (-415,-390,0) to (415,390,170) | (-415,-390.0003,-0.0001) to (415,390.0003,170.0001) | <0.001 mm | PASS |
| Intake wheels | (298.2,-152,4.4) to (441.8,152,135.6) | (298.277,-152,4.539) to (441.723,152,135.461) | 0.139 mm | PASS |
| Intake cheeks | (455,-208,18) to (708,208,150) | (455,-208,18) to (708,208,150) | <0.001 mm | PASS |
| Handoff ramp | (319.6,-94,0) to (360.4,94,53) | (319.585,-94,-0.001) to (359.999,94,52.280) | 0.720 mm | PASS |
| Plywood bridge | (270,-245,52) to (500,245,168) | (270,-245,52) to (500,245,168) | <0.001 mm | PASS |
| Launcher cradle | (323.6,-129,127.1) to (596.4,129,302.9) | (323.644,-129,127.056) to (596.356,129,302.944) | 0.044 mm | PASS |
| Flywheels | (357.5,-229,157.3) to (562.5,229,272.7) | (357.480,-229,157.306) to (562.520,229,272.694) | 0.020 mm | PASS |
| Basket, collection | (-112,-172,19) to (370.6,172,285) | (-112,-172,19) to (370.597,172,285) | 0.003 mm | PASS |
| Basket, launch | (-98.972,-172,129.854) to (390.999,172,444.930) | (-98.972,-172,129.854) to (390.017,172,444.930) | 0.982 mm | PASS |
| LiDAR scan plane | Z = 498 mm | Z = 498 mm | 0 mm | PASS |

All are within the declared 2 mm tolerance. Curves are decomposed into eight
convex segments per side/path, avoiding concave dynamic collision meshes.

## Mass, inertia, and stability

| Assembly | Modelled mass | Basis |
|---|---:|---|
| Plywood bridge and aluminium doublers | 1.19 kg | measured CAD volumes; 600 / 2700 kg/m3 |
| Intake cheeks and flanges | 0.41 kg | 950 kg/m3 HDPE estimate |
| Handoff ramp and walls | 0.09 kg | thin sheet/printed assembly estimate |
| Two basket guides and feet | 0.82 kg | hollow aluminium envelope estimate; no brace or cage |
| Raised plywood holders | 0.457 kg | 600 kg/m3 plywood; launch-study configuration only |
| Empty basket body | 1.20 kg | source basket construction estimate |
| Basket payload | 2.565 kg | 45 regulation tennis balls at 57 g |
| Launcher cradle and two motors | 3.83 kg | two 8 mm aluminium plates plus two 0.49 kg D5065 motors |
| Flywheels | 0.40 kg each | tyre/hub solid-volume estimate |

Every moving or contact-bearing link has positive mass and positive-definite,
explicit inertia. Total compact robot mass is 32.34 kg without holders and
32.797 kg in the supported launch-study configuration. Aggregate results:

| Reconstructed link | Mass | Modelled link COM in ground frame, lowered pose (m) | Inertia source |
|---|---:|---|---|
| `compact_bridge_link` | 1.19 kg | (0.412, 0, 0.128) | combined box envelope about assembly COM |
| `basket_rails_link` | 0.82 kg | (-0.030, 0, 0.207) | two hollow guide envelopes and feet |
| Each intake carriage | 0.05 kg | (0.370, +/-0.090, 0.070) | carriage box estimate |
| Each intake wheel | 0.20 kg | (0.370, +/-0.090, 0.070) | cylinder inertia |
| `compact_intake_cheeks_link` | 0.41 kg | (0.600, 0, 0.084) | curved assembly box-envelope estimate |
| `compact_handoff_ramp_link` | 0.09 kg | (0.340, 0, 0.018) | thin ramp/wall envelope estimate |
| `basket_guide_path_link`, loaded | 3.765 kg | (0.120, 0, 0.12625) | basket body plus distributed 45-ball payload; no cage mass |
| `basket_raised_holders_link` | 0.457 kg | (0.128, 0, 0.118) | plywood posts and tilted shelves |
| `flywheel_launcher_frame_link` | 3.83 kg | (0.460, 0, 0.215) | two plates plus point/box motor contribution |
| Each flywheel | 0.40 kg | (0.460, +/-0.129, 0.215) | solid cylinder; spin inertia 0.0020 kg m2 |

| State | Centre of mass (m) | Support-polygon margin | Stable |
|---|---|---:|---|
| Basket lowered | (0.04569, -0.00224, 0.11243) | 0.3693 m | yes |
| Basket raised | (0.04569, -0.00224, 0.12408) | 0.3693 m | yes |
| Raised + 12-degree pose, holders included | (0.04952, -0.00221, 0.12974) | 0.3655 m | yes |

The conservative support polygon is X +/-0.415 m and Y +/-0.390 m.

## Collision audit and source blocker

Convex SAT checks pass in lowered, raised, and launch poses for flywheels versus
cheeks, flywheels versus ramp, basket versus LiDAR, drive wheels versus chassis,
and intake carriages versus bridge. The remaining failures reproduce the CAD:

| Source solids | Exact OpenSCAD intersection | Intersection bounds (mm) |
|---|---:|---|
| Launcher vs plywood bridge | 145,093.43 mm3 | (377.881,-201.564,150) to (468.287,201.564,168) |
| Launcher vs basket hood, current 12-degree pose | 15,281.46 mm3 | (344.971,-119,137.555) to (390.999,119,238.913) |
| Raised basket vs bridge | 2,240.00 mm3 | (280,-172,152) to (320,172,156) |
| 12-degree launch basket vs bridge | 1,053.23 mm3 | (295.987,-172,162.133) to (324.419,172,168) |
| Lowered basket vs chassis | 4,616.00 mm3 | (-86,-172,38) to (10,172,52) |

These were measured by exporting boolean intersections of physical modules,
not inferred from URDF bounding boxes. Consequently the validator exits nonzero
with `CAD_SOURCE_BLOCKER`. Collision-clean physics requires an authorized CAD
change such as a bridge cut-out, launcher relocation, or hood redesign.

## Gazebo validation

A headless compact bench was built and launched under ROS 2 Jazzy/Gazebo. The
model spawned, all drive/intake/basket/flywheel controllers became active, and
preserved fixed contact-owner links survived URDF-to-SDF conversion.

- Simulation clock and controller-manager time advanced with 0.000 s measured
  skew.
- The intake wheel moved 0.56 rad under command and stopped after zero command.
- Flywheel left moved 2.297 rad under a [55, -55] rad/s command and stopped.
- Two basket cycles passed with 4/4 transitions: 96.42/97.34 mm raised and
  2.68/2.40 mm lowered, 120 mm/s peak tracking speed, without retries.
- After settling, the chassis pitch was approximately -0.0000006 rad and the
  wheel centres were at Z = 0.085 m: the robot rested level on the ground.
- A brief 0.2 m/s drive command was accepted by the controller; its odometry
  capture was interrupted and is not counted as a passed quantitative check.

The intake compression/contact loaded-ball sweep and full throwing
orchestration were not rerun because the CAD-source solid interference already
blocks the physics acceptance gate. The compliant carriage structure and
controller contracts are nevertheless retained and regression-tested.

## Reproduction

```bash
python3 scripts/generate_robot_urdf.py --packaging-variant compact --output /tmp/compact.urdf
python3 scripts/export_compact_cad_envelopes.py
python3 scripts/validate_compact_mechanics.py --urdf /tmp/compact.urdf
pytest -q tests/test_compact_mechanical_model.py tests/test_mechanical_variants.py
```

The full repository test run used the ROS overlay and a writable ROS log path;
The full ROS-aware result is 898 passed, 2 skipped. The compact/mechanical
focused result is 29 passed.

## Capability status

- `COMPACT_CAD_URDF_ALIGNED`: **validated**.
- `COMPACT_PHYSICS_MODEL_VALIDATED_IN_SIM`: **blocked — CAD source interference**.
- `THROWING_ORCHESTRATION_VALIDATED_IN_SIM`: **not revalidated by this work**.
- `BALL_LAUNCH_PHYSICS_NOT_VALIDATED`: **true**.
- `PHYSICAL_HARDWARE_PENDING`: **true**.
