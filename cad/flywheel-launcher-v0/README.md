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

## External panels and appearance

`external-panel-study.scad` adds a non-structural bodywork study around the
selected low launcher integration. The selected `uniform` + `rounded` profile
finishes at 463 mm, 35 mm below the LiDAR scan datum. It keeps panel loads on a
fixed perimeter subframe; no cosmetic skin carries the basket.

The default `appearance_mode=true` expresses the intended commercial sports
robot language without changing the rectangular mechanical architecture: a
light cool-grey upper shell, concealed internal panel brackets and a continuous
dark lower chassis belt ending at `Z=190 mm`, aligned with the OAK-D lower
edge. Set it to `false` to expose mount
tabs, service keep-outs and the semi-transparent engineering context.

The baseline contains:

- fixed lower side skins with four wheel arches and removable service zones;
- a removable rear battery/electronics hatch and ventilation openings;
- a uniform-height fixed shell and large removable basket hatch;
- the earlier moving upper cowl as an optional `stepped` comparison;
- front bodywork whose lower edge stops at the 168 mm Option A bridge top,
  leaving the curved cheeks and intake mouth fully open;
- a closed flywheel fascia/roof with one 116 mm angled circular opening around
  the 90 mm launcher exit guide;
- a structural OAK-D mount on the closed fascia below that cylinder;
- 55 mm rounded plan corners, a smoothly tapered nose, rounded top hatches and
  rounded side service doors;
- a bowed rear profile extending 45 mm behind the original flat rear datum;
- aft panel supports inset by 30 mm, with the side rails converging from
  `X=-260 mm` so the rear skin can form a smoother continuous curve;
- a short LiDAR bracket mounted to the structural upper rear crossmember,
  replacing the long chassis-to-sensor mast;
- provisional M5 panel tabs on fixed and moving frames;
- optional battery and basket removal keep-out volumes.

The current appearance iteration also adds a large rounded smoked-polycarbonate
window on each basket side and a dark front equipment mask shared by the
launcher exit and OAK-D. These are visual/guard panels, not structural members.
The basket window retains 44 mm of upper shell material and 31 mm above the
dark lower belt. The front mask starts 1 mm above the Option A bridge datum and
retains 24 mm below the roof edge, so it does not reduce the intake opening or
launcher aperture.

```bash
/snap/bin/openscad-nightly \
  -D 'mode="launch"' \
  -D 'show_service_keepouts=false' \
  -o cad/flywheel-launcher-v0/preview_external_panels_launch.png \
  cad/flywheel-launcher-v0/external-panel-study.scad
```

Useful switches are `show_fixed_panels`, `show_moving_cowl`,
`show_top_hatches`, `show_panel_mounts`, and `show_service_keepouts`. Set
`shell_profile="stepped"` and `show_moving_cowl=true` to inspect the earlier
profile; set `shell_style="faceted"` for the angular body comparison. Panel
thickness, bends,
materials, quarter-turn fasteners and final OAK-D optical window remain open
until the physical mounts and service clearances are measured.

The default `shell_alpha=0.70` keeps internal collision context visible. Use
approximately `shell_alpha=0.92` for appearance renders; opacity does not alter
the geometry or openings.

Current envelope checks are 17.5 mm nominal radial clearance around each
170 mm drive wheel, 27 mm lateral clearance between the launcher guard and
each nose side, and 35 mm between the uniform 463 mm shell top and the 498 mm
LiDAR scan datum. The closed hatch has only about 16 mm nominal inner clearance
over the raised basket rim, so full-load/flex clearance remains a physical test.
The fixed body is 900 x 564 mm over the 920 x 580 mm chassis plate; the
external wheels still define the approximately 780 mm overall robot width.
The rear hatch includes a 70 mm LiDAR-mast cutout rather than relying on a
visual overlap. In the selected `upper_frame` mount this opening carries only
the short bracket/cable pass-through; `lidar_mount_style="base_mast"` remains a
historical comparison.

The front fascia begins 18 mm above the Option A cheek tops. The launcher exit
opening provides 13 mm nominal radial clearance; its centre follows the 20
degree launch axis and is approximately Z=299 mm at the fascia. The OAK-D
envelope is centred at X=800, Z=205 mm on a structural crossbar, not on the
cosmetic skin. Final optical FOV and vibration validation remain required.

## Electronics packaging comparison

`electronics-packaging-study.scad` imports the real `240 x 180 mm` motion tray
and compares two placements without changing its source geometry. The selected
`low_split` layout rotates the tray 90 degrees into the rear floor bay behind
the battery, places a provisional metal Pi-case envelope beside the battery,
and holds the DFR0753 buck vertically between the battery and Pi. Its nominal
clearances are 24 mm tray-to-battery, 46 mm Pi-to-battery, 47 mm Pi-to-side-skin
and 13 mm from each broad face of the buck keep-out. The `stacked` comparison
demonstrates the battery-service obstruction created by putting the motion tray
above the battery and raises its electronics envelope to approximately Z=291
mm. The Pi-case `120 x 90 x 50 mm` envelope is deliberately provisional and
must be replaced by measurements of the actual case before manufacturing
release.
