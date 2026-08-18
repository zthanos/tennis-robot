# Single-motor geared intake concept

OpenSCAD-only concept view of the intake mechanism: one vertically mounted
GB37-style motor, a visible pitch-contact gear train, and two side-pinch
intake wheels. This is a mechanical visualization for discussion, not the
robot source of truth.

It intentionally does not modify or feed:

- `ros2_ws/src/tennis_robot/urdf/`
- Gazebo generated SDF files
- controller configuration
- runtime sweep artifacts

The current simulated intake direction remains documented in
`docs/mechanism/dual-wheel-intake-design-el.md`, where the validated default is two
motors. This CAD folder exists to inspect the packaging of the earlier
"one motor + gears" idea without touching robot behavior.

Render:

```bash
docker compose run --rm openscad openscad -o /tmp/single-motor-geared-intake.png cad/archive/single-motor-geared-intake/assembly.scad
```

Useful preview toggles inside `assembly.scad`:

- `show_ball`
- `show_funnel_context`
- `show_drive_arrows`
- `show_pitch_circles`
- `explode_gears`

The idler centers are calculated from gear pitch radii so adjacent gears sit
on contact tangencies in top view. The left output path uses one idler; the
right output path uses two idlers so the two intake wheel shafts counter-rotate.
