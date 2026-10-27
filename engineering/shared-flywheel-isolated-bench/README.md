# Phase-2 v3 isolated bench — 5a executed, P2-42 5b harness

This directory is deliberately outside the production `tennis_robot` package.
Nothing here is included by the full robot stack.  Do not expand the xacro,
build ROS, or start Gazebo until the user gives a fresh explicit approval.
`v3_bench_contract.json` freezes the input paths, hashes and numerical
parameters that the first approved run must verify.

The intended runtime is the separately prepared Ubuntu 24.04 / ROS 2 Jazzy
container.  Humble and the production robot launch files are out of scope.

## Cases already represented

- `drive_test_ballast:=false basket_full_45:=false`: v3 chassis, Parkside pair,
  UGREEN, empty large basket, P16 tilt joint/stops and the 4.610 kg P2-40 flywheel
  module.
- `drive_test_ballast:=false basket_full_45:=true`: same, with 45 balls / 2.565
  kg at the provisional stored-ball CG X=105 mm.
- `drive_test_ballast:=true`: physical drive-test surrogate, replacing the
  flywheel module with the 5.4479 kg plate-plus-ballast assembly at X=360 mm.

Geometry is four-wheel skid steer: wheel-centre track 0.594 m,
`drive_axle_y=±0.297 m`, rear/front axle X=−0.065/0.300 m and radius 0.085 m.
The P2-42 isolated reference pivot is X=0.387 m, Z=0.049837 m at R=85
mm/h=29 mm (2 mm below P2-40 to target 10 mm ball-to-usable-edge margin),
55 mm behind the nip. The selected operating points are −3° collection, 0°
transport and +15…+20° stationary launch. The replaceable lower-stop rail
remains adjustable −10…−3°, while the validated URDF joint lower limit uses
the selected −3° seat. The P16 is represented by its 95 g body, 40 mm linkage
design and ±12.6 N·m effort limit. Launch reaction is credited to the +20°
seat/joint stop, not to the actuator screw.

The isolated module now uses the accepted printed TPU 60 mm/r2 estimate:
0.878977 kg including hub, polar inertia 0.00508982 kg·m² and transverse
inertia 0.00267808 kg·m² per wheel. These remain primitive-volume estimates
until slicer and assembled measurements replace them.

The D5065 constants are intentionally unchanged from
`config/flywheel_launcher_capability_protocol.json`: 270 KV, 0.031 N·m/A,
0.039 Ω phase-neutral and provisional 20 A / 0.62 N·m per motor.  The existing
capability analysis supplies the current/back-EMF calculation; the xacro does
not invent a second motor law.  The Parkside bus is 15–21 V, but no higher
electrical limit is credited before controller and pack-current measurements.

For later ball-contact runs, use the repository's existing
`gazebo/models/tennis_ball_compliant/model.sdf` verbatim.  Its calibrated
loading stiffness 107309.294 N/m^1.5, unloading exponent, damping and 58 g
mass must not be refitted from launcher results.

The controller YAML only exposes measured wheel feedback, four drive velocity
interfaces, and tilt effort.  It does not add a production node, Arduino code,
wiring definition, forced ball transfer or forced ball velocity.

## Executed scope and next approval gate

With explicit approval, the three xacro cases were expanded and validated in
the Jazzy container and the complete Gazebo 5a matrix was run headless for
three payloads and μ=0.5/0.7/0.9.  See
`docs/mechanism/shared-flywheel-intake-launcher-phase2-gazebo-5a-el.md` and the
`results/shared_flywheel_phase2/gazebo_5a` artifacts.  The next hard gate is
before every 5b–5d actuator, collection or launch execution.
# Gazebo 5b (P2-42)

`criteria_5b.json` freezes the gate before execution.  The P16 is an abstract
12.6 N m-limited joint actuator at 23 deg / 8.3 s; linkage force, stress,
singularity and current claims are excluded.  Physical stop impact is also
excluded and the revolute joint limits act as stops.  `empty` and `full45`
execute the seven-position transition.  `ballast` is spawn/stability-only.

The X318 legs, X318-to-X330 doglegs, X330 crossmember and LiDAR are
collision-only validation fixtures mapped to frozen CAD in
`p2_40_collision_fixture_mapping.json`.  They are enabled only for 5b; they
are not production CAD or an STL release.

Build and run inside the Jazzy container:

```sh
cmake -S engineering/shared-flywheel-isolated-bench -B engineering/shared-flywheel-isolated-bench/build_5b
cmake --build engineering/shared-flywheel-isolated-bench/build_5b -j2
engineering/shared-flywheel-isolated-bench/run_5b_case.sh empty
engineering/shared-flywheel-isolated-bench/run_5b_case.sh full45
engineering/shared-flywheel-isolated-bench/run_5b_case.sh ballast
```

## 5d-1 P2-59 single-ball launch

P2-59 reuses the frozen P2-40/P2-51 geometry and the calibrated 5c-3 ball
contact. Run the complete headless matrix only through the Jazzy wrapper:

```sh
ROS_DOMAIN_ID=88 GAZEBO_HEADLESS=true docker/jazzy/run.sh exec \
  "timeout 3600 engineering/shared-flywheel-isolated-bench/run_5d1_p259_campaign.sh"
```

`criteria_5d1_p259.json` is the frozen acceptance input. The campaign runs
+15/+20 deg, tread μ 0.6/0.9 and 12/16/20/24 m/s surface speed with three
repeats, then records the four 24 m/s boundary cases. Offline beam and rotor
screens are produced by `analyze_5d1_p259_break_beam.py` and
`analyze_5d1_p259_rotor_static.py`. Results live under
`results/shared_flywheel_phase2/gazebo_5d1_p259/`.

Paused playback of a recorded case is, for example:

```sh
engineering/shared-flywheel-isolated-bench/replay_5d1_p259_case.sh 15 0.9 24 1
```

The replay helper reuses `5c3_side_gui.config` and does not modify it.

## P2-60 launch-saturation diagnostic

`criteria_5d1_p260.json` freezes a diagnostic-only 27-case matrix at +20°
and 20 m/s surface speed. It varies diametral nip compression 8/11/14 mm,
tread μ 0.6/0.9/1.2 and contact-entry feed speed 0/2/4 m/s. The source
P2-40/P2-51 geometry is not edited: `make_5d1_p259_world.py` applies the
wheel-centre offsets only to each generated P2-60 world.

```sh
ROS_DOMAIN_ID=88 GAZEBO_HEADLESS=true docker/jazzy/run.sh exec \
  "timeout 3600 engineering/shared-flywheel-isolated-bench/run_5d1_p260_campaign.sh"
```

`aggregate_5d1_p260.py` includes the analytical Hertz-like traction bound,
measured contact impulses, factor effects and record manifest. Paused replay:

```sh
engineering/shared-flywheel-isolated-bench/replay_5d1_p260_case.sh 14 0.9 0
```
