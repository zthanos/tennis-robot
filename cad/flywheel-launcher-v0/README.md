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

The lift study now preserves the selected manual interface and the future
actuation provision in the same CAD:

- **V1 manual:** the original central top pull handle, grey gas
  spring/counterbalance and positive gold locking pins in both positions;
- **V2 provision:** translucent purple electric linear-actuator envelope,
  permanent fixed/moving clevis hardpoints and its swept keep-out;
- **controls provision:** orange upper/lower limit-switch envelopes, a purple
  driver keep-out and protected cable-route envelope.

The rails and blue carriage remain the only intended basket load path. The top
handle is tied to that carriage rather than the cosmetic hatch. The handle,
gas spring or future actuator moves the carriage; none of them replaces
the hard stops and locking pins.  The actuator geometry is deliberately not a
purchased-part selection: stroke and force must be chosen only after measuring
the full-basket load, direct manual handle force and real travel on V1. The
removable basket hatch must be open while the top handle is being used.

The top handle is now centred at `X=220 mm`, directly over the basket centroid
and the `X=225 mm` carriage beam. Its grip was lowered 20 mm: the lower-position
top is `Z=337 mm` and the raised-position top is `Z=437 mm`, leaving 23 mm below
the `Z=460 mm` inner roof. The supports sit at `Y=+/-165 mm` on the longitudinal
carriage beams, so no handle leg enters the ball volume. Their clearance is
10 mm, measured against the outer basket envelope at `Y=146 mm` (mesh wall
frame, flange drop struts, moulded carry handles) rather than the `Y=140 mm`
interior half width. This is a static assembly gap: legs and basket both ride
the carriage, and the swept leg path at `X=211...229 mm` misses the fixed rails
and cross-brace at `X=56...84 mm`. The pre-existing longitudinal carriage beam
is tighter still, at 7 mm on the same datum.

The grip is held at the centre but reacted at the legs, so the rod works in
bending over a 330 mm unsupported span. It is a purchased metal section, not a
printed part: an 18 mm square in PLA reaches about 26 MPa and sags roughly 9 mm
under a 300 N pull, against about 0.4 mm in aluminium at the same stress.

Manual access uses a flat gasketed `110 x 170 mm` hatch centred at `X=220 mm`
inside the existing large basket hatch. There is no recessed well or collar.
The low-position reach from the outer roof datum is 126 mm. The small hatch is
a single printable tile on a `220 x 220 mm` bed; its 170 mm dimension follows Y
and exposes the central portion of the transverse grip. It is opened only in
manual/service mode, with autonomous motion disabled.

The hatch folds flat onto the basket hatch rather than standing upright. Its
free edge sits at `hatch_z + 110 * sin(theta)`, so a `Z<=478 mm` limit against
the `Z=498 mm` scan plane admits only `theta <= 6.8 deg` or `theta >= 173.2 deg`:
every intermediate angle raises the panel through the LiDAR plane, and there is
no usable half-open position. `handle_access_hinge_open_deg = 175` puts the free
edge at `Z=474.6 mm`, 23.4 mm below the scan plane, resting over `X=55...165 mm`.
The stop must hold that folded position rather than an intermediate one, and
because the finished face rests downward it needs bumpers on the roof.

Useful visibility switches at the top of `cover-lift-study.scad` are
`show_top_pull_handle`, `show_manual_lever` (legacy comparison only),
`show_counterbalance`, `show_future_actuator`,
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

## Compact packaging study (fixed chassis)

`compact-packaging-study.scad` tests the shorter functional silhouette without
cutting the `920 x 580 mm` Option A chassis or changing its wheelbase.  The
complete `intake -> bridge -> basket -> feeder -> launcher` group moves
rearward by a provisional 100 mm, preserving all validated relative geometry.
Option A remains a read-only `use<>` dependency.

One relative datum is deliberately corrected only in this study: the imported
Option A handoff ramp is not rendered. Its `X=520 mm`, `Z=1.5 mm` front lip
would touch a centred 66 mm ball at ball-centre `X~=529.8 mm`, whereas the
finite tilted-wheel geometry does not contact that ball until `X~=481.2 mm`.
The hard lip would therefore lead the compliant tires by about 48.6 mm and
could reject the ball. The compact study substitutes a short lip at
`X=460 mm`, giving the wheels approximately 11.4 mm of first-contact lead. The
new `X=460..420 mm`, `Z=1.5..35 mm` handoff is a simulation hypothesis, not a
manufacturing change; loaded-ball and physical rolling tests remain mandatory.

The study-only wooden bridge subtracts an open rear notch across the basket's
vertical service corridor and two open-bottom `D=80 mm` motor arches.  Against
the nominal `D=60 mm` drive motors, each arch leaves 10 mm radial clearance
and 25 mm of wood above the crown.  External angle/doubler references stay
outside the basket keep-out.  The notch covers the basket's outer
`Y=+/-146 mm` geometry plus a provisional 14 mm service margin, so neither the
bridge nor its reinforcement blocks lift, tilt or vertical basket removal. It
extends to local `X=484 mm`, covering the receiving chute/hood at `X=470 mm`
plus the same 14 mm margin; stopping at the main bin's `X=420 mm` edge would
not clear the complete removable assembly.

The battery moves rearward to `X=-255 mm`; the existing `240 x 180 mm` motion
tray stands vertically at `X=-425 mm`, with its components facing forward.
The resulting hypothesis clearances are 35 mm from the chassis rear, 32 mm
tray-to-battery and 92 mm battery-to-shifted-basket.  These are clearance-study
datums, not released mounting holes.  The flat roof hatch follows the basket
to `X=120 mm`; the LiDAR remains fixed.

Render all keep-outs and both basket poses with:

```bash
/snap/bin/openscad-nightly \
  -o cad/flywheel-launcher-v0/compact-packaging-study.png \
  --imgsize=1600,1000 --viewall --autocenter \
  cad/flywheel-launcher-v0/compact-packaging-study.scad
```

For the non-misleading top view of the intake only, set
`show_intake_ball_path=true` and disable the basket, launcher, drive, shell and
rear electronics. In that view the long orange curves are the two outboard
cheeks at `Y=+/-205 mm`, not a transverse ramp; the corrected yellow handoff
starts behind the black intake wheels.

The corresponding Gazebo model is selected without changing the preserved
baseline:

```bash
ROBOT_PACKAGING_VARIANT=compact ros2 launch tennis_robot sim.launch.py headless:=true
```

It carries the same `-100 mm` functional shift, rear battery, vertical motion
tray, Pi case, separate buck converter, loaded basket and front dual-flywheel
launcher.  The generated SDF replaces both the handoff collision and its visual
with matching short segments behind the intake-wheel nip, so the simulation no
longer displays the obsolete forward lip. See
`docs/mechanism/flywheel-launcher-exploration-el.md` for masses, CoM and the
initial baseline/compact motion comparison.

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

### Tennis-trainer identity pass

Three changes, all driven by one measurement: a full 45-ball load only reaches
about `Z=150 mm` (floor top `25`, two layers of `66 mm` balls, ~24 per layer in
the `400 x 280 mm` bin), while the smoked basket window spans `Z=221...419 mm`.
**That window starts 71 mm above a full load and can never show a ball.** It
shows mesh, carriage and gas spring instead, which is what makes the shell read
as a prototype rather than as court equipment.

- **Launcher recess.** `performance_launcher_details()` drew flat annuli on the
  fascia at `X=794` and `796.5`, with no depth. It is now a real cone on the
  20 degree axis: throat `= front_exit_open_d` exactly, widening outward only,
  so launcher clearance is untouched. The cone is over-extended and then cut by
  the fascia, and the accent ring is the *intersection* of the shell with a
  `launcher_ring_w` slab at the fascia. That construction matters: the cone
  mouth is perpendicular to the 20 degree axis while the fascia is vertical, so
  a drawn ellipse renders as a crescent instead of a ring.
- **Ball-level ports** (`show_ball_ports`), `180 x 62 mm` at `X=115, Z=140` in
  each side skin, inside the graphite belt. Clearances are asserted: `13 mm` to
  the belt top, `45 mm` above the lower skin edge, `16.5 mm` to the forward
  wheel arch at `X=227.5`. The port straddles the fill line, so it reads as both
  identity and a level gauge.
- **Basket window reduced** from `390 x 184` to `290 x 100`. At the old size it
  was the largest single element on the robot and it showed the one thing worth
  hiding. The two side apertures are now one family: the same `~2.9:1` aspect
  ratio and a shared rear datum, with both clear openings starting at `X=19 mm`
  (`ball_port_center` X was set to `109` for exactly that). Corner radius drops
  to `18 mm`, joining the aperture family. Their glazing is deliberately
  inverted: the port is near-clear so a yellow load reads, the window is smoked
  to `basket_glazing_alpha = 0.66` so carriage and gas spring do not.
  Clearances: `96 mm` window to roof, `63 mm` window to belt top.
- **Side accent withdrawn** (`show_side_accent = false`). The tapered blade read
  as a vehicle stripe, which is the wrong product category for a court trainer.
  Kept switchable rather than deleted.

Sight-line audit for the ports: on `-Y` only the gas spring clips the bottom
~9 mm. On `+Y` the V2 actuator swept keepout (`Y=202...268`, `X=-18...258`,
`Z=45...253`) encloses the entire port opening, so that port would see hardware
rather than balls if the actuator is fitted.

### Roof grid

The roof panels are derived from the fixed subframe rather than sized by eye.
`fixed_panel_subframe()` runs longitudinal rails at `Y=+/-268 mm` and transverse
members at `X=-438 mm` and `X=405 mm`; `roof_inset = 14 mm` puts every panel
edge on those rails, and both roof joints sit over transverse members.

This replaced three separate defects found by measurement:

- the old rear hatch (`450 x 426`, centred `X=-270`) **overhung the body plan**
  by up to `15.5 mm` at both rear quarters, `Y=+/-142...209`, confirmed by a 2D
  `hatch - plan` boolean returning non-empty geometry;
- the basket hatch (`0...440`) and the nose roof (`420...790`) **overlapped by
  20 mm** while nearly coplanar, so that joint had no shut line at all;
- the `45 mm` strip between the two hatches had **no member under it**, leaving
  two free panel edges across an unsupported void.

There was also no fixed roof at all: the two removable panels *were* the roof
and left a `62...69 mm` open strip down each side. `rounded_top_system()` now
draws a fixed border first, and the border and both panel edges share the same
rail.

Panels come from `roof_panel_2d(x_lo, x_hi)`, which insets the body plan and
cuts it to an X band. Deriving them from the plan keeps a constant border, makes
the rear panel follow the bowed rear instead of overhanging it, and inherits the
body corner radius instead of adding another one to the family. Openings are the
same outline grown by `roof_shut_gap = 3 mm`, the value the handle hatch already
used, now applied at every roof joint.

A transverse member was added at `roof_joint_mid_x = -22 mm`, `Z=445 mm` to
carry the rear/basket shut line.

The LiDAR bore still passes through the rear panel at `70 mm`. With the panel
tip now at `X=-487 mm` and the bore reaching `X=-455 mm` there is `32 mm` of
material behind it; `lidar-pod-study.scad` reduces that bore to a
`penetration_reach = 26.5 mm` fastener/cable pattern.

Use hierarchy on the roof is carried by the handle hatch's dark gasket ring, not
by panel tone: `upper_hatch_color` is deliberately not applied to any roof panel
and is reserved for a future control-surface tone.

### Performance shell v1

Set `design_variant="performance_v1"` (the current study default) for the
first controlled Tennis Performance iteration. It changes only four visual
systems relative to `design_variant="commercial"`:

- the smoked basket window has a 10-degree forward-raked leading edge; the
  value remains adjustable within the 8-12 degree study range;
- the launcher fascia becomes a shaped inset face with a projected elliptical
  tennis-yellow ring around the existing angled exit opening;
- a short tapered yellow blade follows the window direction and terminates
  before it can read as a full vehicle stripe;
- the graphite belt rises in two controlled steps after the forward wheel arch
  and stops before the tapered nose.

The ring's 8 mm nominal width is a visual starting value, not a manufacturing
constraint. Wheel-arch surrounds, a LiDAR pod and any physical roof rake are
explicitly deferred to a later iteration. In particular, the roof remains at
the validated 463 mm datum because its nominal raised-basket clearance is only
about 16 mm.

```bash
/snap/bin/openscad-nightly \
  -D 'mode="launch"' \
  -D 'show_service_keepouts=false' \
  -o cad/flywheel-launcher-v0/preview_external_panels_launch.png \
  cad/flywheel-launcher-v0/external-panel-study.scad
```

Useful switches are `show_fixed_panels`, `show_moving_cowl`,
`show_top_hatches`, `show_handle_access_hatch`,
`handle_access_hatch_open`, `show_panel_mounts`, and
`show_service_keepouts`. Set
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

## LiDAR pod study

`lidar-pod-study.scad` is a standalone envelope study for the Slamtec RPLIDAR C1
on the shell roof. It mirrors the shell datums (`463 mm` roof, `3 mm` panel,
`498 mm` scan plane, `X=-420 mm`) rather than importing them, so it renders on
its own.

Hand-measured sensor values (approximately +/- 1 mm, not micrometer work):
`55 x 55 mm` square base, `23 mm` base section, `43 mm` total height, four brass
inserts near the base corners, horizontal cable exit from one side. The derived
half diagonal is `38.9 mm`.

Two values are still `TBD` and are exposed as parameters:

- `sensor_scan_h` — base underside to the centre of the optical band. This one
  decides the pod class: the sensor base sits at `498 - sensor_scan_h`, so above
  `32 mm` the base falls below the `466 mm` roof and the pod becomes a recessed
  well instead of a raised fairing.
- `insert_pitch_x` / `insert_pitch_y` — brass insert centres, needed so the
  flange locates on the sensor's own threads and nothing clamps the housing.

`pod_variant="open"` is the selected variant: the fairing alone hides the mount
plate, bracket, roof penetration and cabling with **no blind sector at all**.

The cap was considered and rejected. It offers no optical benefit: the scan
plane is horizontal, so the receiver looks horizontally through a narrow
vertical aperture, and the sunlight that actually reaches it arrives
near-horizontally through the very gap a cap must leave open. A horizontal cap
blocks steep overhead rays, which that narrow aperture already rejects. Its only
real justification was ball-strike protection, which was declined. Outdoor
sunlight is in any case classified `NOT TESTED` in
`docs/hardware/real-lidar-bringup.md`, so a cap would have been a fix for an
unmeasured problem; if court testing ever shows one, the answer is a vertical
lip around the optical window or filtering, not this cage.

With no cap, the fairing stops `8 mm` below the scan plane and no pod surface
sits in the beam, so near-field reflection off the pod stops being a concern
beyond the top rim itself.

`"caged"` is kept as a comparison in case impact testing reopens the question.
It adds a cap on three posts at `post_r=45 mm`, chosen because the measured
`range_min = 0.05 m` means post returns fall inside the minimum range and are
discarded without any angular filter. The posts sit at `90/210/330 degrees`,
deliberately off the base diagonals where the `38.9 mm` half diagonal leaves the
least room; the worst clearance is then `10.7 mm` with a `5 mm` post.

At `sensor_scan_h=30` the caged pod echoes:

```
sensor_base_z=468  roof_offset=+2  fairing_h=24  cap_top_z=517
pod_above_roof=51  blind_deg=19.1  blind_samples=38/720  min_post_gap=10.7
```

```bash
/snap/bin/openscad-nightly -D 'pod_variant="caged"' \
  -o cad/flywheel-launcher-v0/preview_lidar_pod_caged.png \
  cad/flywheel-launcher-v0/lidar-pod-study.scad
```

Nothing here is a manufacturing model: fastener heads, gasket grooves, draft and
wall bosses are all deferred until the two TBD measurements land.

## Scaled appearance model for printing

`appearance-export.scad` is the **only** file here that produces STLs, and it is
not an exception to the rule above. It is a solid massing model at
`model_scale = 1/6`, printed so the silhouette and the aperture composition can
be judged in the hand rather than in a flat OpenSCAD render. No panel, bracket
or fastener geometry is exported, and nothing here is dimensioned for assembly.

It pulls the study's own 2D outlines through `use<>`, so the printed shape
cannot drift: change the study, re-export, they agree.

```bash
for p in body_left body_right wheel; do
  /snap/bin/openscad-nightly -D "part=\"$p\"" \
    -o cad/flywheel-launcher-v0/stl/appearance_$p.stl \
    cad/flywheel-launcher-v0/appearance-export.scad
done
```

`part="assembled"` previews the glued result; `part="layout"` shows all parts on
one plate.

Output at 1/6, for a Bambu Lab P2S (verify the real build volume; if it differs,
only `model_scale` moves):

| Part | Size | Qty |
| --- | --- | --- |
| `appearance_body_left` | `215.2 x 75.5 x 61.7 mm` | 1 |
| `appearance_body_right` | same | 1 |
| `appearance_wheel` | `28.3 x 28.3 x 13.3 mm` | 4 |

The body is split on the centre plane so each half prints on its own cut face:
the side surfaces that carry the design never touch a support. Both halves plus
the four wheels fit one plate.

Feature sizes are held at **model** scale, not scaled down with the body — a
3 mm shut gap would become 0.5 mm at 1/6 and vanish under a 0.4 nozzle. Grooves
are `0.9 mm` wide and `0.7 mm` deep as printed, pockets `1.6 mm`, and the axle
pegs `3 mm` (a scaled 9 mm shaft would have printed at 1.5 mm and snapped).

Printing notes: a layer height of `0.16...0.20 mm` resolves the grooves; the
launcher recess becomes a half-cone opening upward off the bed and its crown is
a ~27 mm unsupported span, so paint on support there or accept some droop; glue
the halves and press the wheels onto the pegs.

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
