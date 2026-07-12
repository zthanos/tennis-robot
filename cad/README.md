# Active CAD status

There is currently no active OpenSCAD manufacturing model in this directory.

The previous `3d-printable-base` and `cyber-shell` models described superseded
single-roller, scoop, launcher, 760 x 430 mm chassis, and cosmetic-shell
concepts. They were removed from the working tree on the
`feat/dual-wheel-intake-concept` branch; Git history remains the archive.

Current mechanical sources of truth:

- `ros2_ws/src/tennis_robot/urdf/tennis_robot.urdf.xacro`
- `ros2_ws/src/tennis_robot/urdf/components/basket.urdf.xacro`
- `docs/dual-wheel-intake-design-el.md`
- `docs/basket-bin-redesign-spec-el.md`
- `docs/intake-debug-log-el.md`

The next SCAD model should be generated from the validated 920 x 580 mm 4WD
chassis, dual-wheel intake, and basket v2.1 dimensions. Do not restore an old
model as the default without reconciling it against those sources.

