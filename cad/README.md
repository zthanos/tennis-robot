# Active CAD status

Active model: **`basket-bin-v2/`** — the sim-validated basket bin v2.1 +
entry hood (debug log #45-#56), parametric OpenSCAD with all dimensions in
`basket-bin-v2/params.scad`. Renders/exports run through the
`docker compose --profile cad` services.

Active intake work is **`collector-intake-v1/option-a/`**. This self-contained
directory holds the curved-cheek, plywood-bridge, 124 x 73 mm RC-wheel assembly,
previews, export script and all currently exportable Option A solids. Provisional bearing,
hub, IR-sensor and motor interfaces are called out in its README rather than
being mixed with the earlier straight-cheek/rail exports.

The previous `3d-printable-base` and `cyber-shell` models described superseded
single-roller, scoop, launcher, 760 x 430 mm chassis, and cosmetic-shell
concepts. They were removed from the working tree on the
`feat/dual-wheel-intake-concept` branch; Git history remains the archive.

`archive/single-motor-geared-intake/` is also historical: it visualizes the
rejected one-motor gear-train option. The active intake has two independent
motors, one per side-pinch wheel.

Current mechanical sources of truth:

- `ros2_ws/src/tennis_robot/urdf/tennis_robot.urdf.xacro`
- `ros2_ws/src/tennis_robot/urdf/components/basket.urdf.xacro`
- `docs/mechanism/dual-wheel-intake-design-el.md`
- `docs/mechanism/basket-bin-redesign-spec-el.md`
- `docs/mechanism/intake-debug-log-el.md`
- `docs/hardware/chassis-layout-4wd-dual-intake-el.md`

The next SCAD model should be generated from the validated 920 x 580 mm 4WD
chassis, dual-wheel intake, and basket v2.1 dimensions. Do not restore an old
model as the default without reconciling it against those sources.
