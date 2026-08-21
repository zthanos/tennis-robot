# Flywheel launcher v0

Exploratory envelope model for the launcher described in
`docs/mechanism/flywheel-launcher-exploration-el.md`.

This model is intentionally **not printable** and contains no motor shafts,
bearings, fastener holes or purchased-wheel interfaces. It exists to compare
wheel size, nip compression, pitch, guards and feed clearance before hardware
selection.

Compile/render with the repository's reproducible Docker service:

```bash
docker compose --profile cad run --rm openscad \
  openscad -o cad/flywheel-launcher-v0/preview_launcher.png \
  cad/flywheel-launcher-v0/launcher-envelope.scad
```

The manual basket lift/tilt envelope is a separate context study:

```bash
docker compose --profile cad run --rm openscad \
  openscad -o cad/flywheel-launcher-v0/preview_basket_lift.png \
  cad/flywheel-launcher-v0/basket-lift-study.scad
```

The structural two-position cover/rail study is:

```bash
docker compose --profile cad run --rm openscad openscad \
  -D 'mode="both"' \
  -o cad/flywheel-launcher-v0/preview_cover_lift_both.png \
  cad/flywheel-launcher-v0/cover-lift-study.scad
```

Its color key is silver=fixed rails, blue=load-bearing carriage, gold=hard
stops/locking pins, light-blue=non-structural cover outline, and wire
mesh=unchanged basket v2.1.

The lift study now preserves both actuation generations in the same CAD:

- **V1 manual:** red removable lever, gold adjustable pull link, grey gas
  spring/counterbalance and positive gold locking pins in both positions;
- **V2 provision:** translucent purple electric linear-actuator envelope,
  permanent fixed/moving clevis hardpoints and its swept keep-out;
- **controls provision:** orange upper/lower limit-switch envelopes, a purple
  driver keep-out and protected cable-route envelope.

The rails and blue carriage remain the only intended basket load path.  The
lever, gas spring or future actuator moves the carriage; none of them replaces
the hard stops and locking pins.  The actuator geometry is deliberately not a
purchased-part selection: stroke and force must be chosen only after measuring
the full-basket load, manual handle force and real travel on V1.

Useful visibility switches at the top of `cover-lift-study.scad` are
`show_manual_lever`, `show_counterbalance`, `show_future_actuator`,
`show_control_provision`, and `show_actuator_swept_keepout`.

The installed `openscad-nightly` snap can also be used interactively on the
host. Automated repository exports use the pinned Docker image so results do
not depend on the host snap revision.

## Full robot integration

`robot-integration.scad` imports the active Option A modules read-only and adds
the basket v2.1, battery, 4WD/sensor references, basket lift envelope and
launcher. No file under `collector-intake-v1/option-a/` is modified.

```bash
docker compose --profile cad run --rm openscad openscad \
  -D 'mode="launch"' -D 'launcher_layout="front"' \
  -D 'front_feed_mode="opening"' \
  -o cad/flywheel-launcher-v0/preview_robot_launch.png \
  cad/flywheel-launcher-v0/robot-integration.scad
```

Modes are `collect`, `launch`, and `both`. The required launcher layout is
`front`; `side` and `rear` remain comparison studies. Front feed modes are
`opening` (baseline: raised basket's existing front opening) and `rail`
(launcher parked forward in collect, slid rearward and positively locked for
launch).

The full-robot default is now `launcher_orientation="side_by_side"` with a
215 mm nip height.  This keeps the Ø200 mm wheel pair below the raised basket
rim and below the approximately 500 mm LiDAR scan plane.  The earlier tall
`over_under` stack remains available only as a comparison because it cannot
satisfy those two packaging constraints with the current 100 mm basket lift.
The side-by-side arrangement changes the available spin axis, so wheel
orientation remains an explicit design parameter until launch testing.

Set `show_height_datums=true` for the selected-layout audit view. It displays
the 142 mm basket outlet, 215 mm nip, 275 mm feeder crest, provisional 370 mm
launcher ceiling, approximately 444 mm raised-basket rim and 498 mm LiDAR scan
plane. These are packaging references, not manufacturing tolerances.

Useful overrides:

```bash
docker compose --profile cad run --rm openscad openscad \
  -D 'pitch_deg=30' \
  -D 'wheel_d=220' \
  -D 'nip_gap=60' \
  -o /tmp/launcher-variant.png \
  cad/flywheel-launcher-v0/launcher-envelope.scad
```

Color key:

- orange: flywheels;
- translucent red: mandatory moving-part guard envelope;
- blue: ball path and reference balls;
- grey: provisional cradle side plates;
- green: 90 mm feed-interface keep-clear volume.

In `basket-lift-study.scad`, the solid basket is the validated collect pose and
the transparent basket is the provisional raised/side-tilted launch pose. The
green volume is the side feed dock that becomes aligned only in launch mode.
It shows required space only; rails, springs, cam gate, hinges and locks are
not designed yet.

Do not export manufacturing STLs from this directory. The source becomes a
manufacturing model only after the real wheels, motors, shafts and bearings are
selected and measured.
