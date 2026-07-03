# 3D Printable Base Prototype

Parametric OpenSCAD sources for a first printable tennis-robot base.

Scope:

- printable modular base tiles
- full-size base mounting/drill template with component mounting points
- printable motor pods (x4, one per driven wheel — 4WD)
- printable direct-drive wheels
- printable stabilizer feet
- printable trolley-handle sockets
- first-pass collector funnel, wide roller bracket, and receiving bin
- full concept assembly showing the expected wooden-base robot layout

Out of scope for this folder:

- launcher flywheels
- metal axle-based wheel assemblies

## Design intent

The base should be printed as multiple bolted modules, not as one huge print.
The large drive wheels mount directly to the motor shaft or motor hub. This avoids
separate metal wheel axles.

Important caveat: a real motor still has a metal output shaft. This design avoids
extra metal axles, but it does not replace the motor shaft itself. Fully printed
load-bearing axles are not recommended for a tennis robot base.

## Suggested materials

| Part | Material | Notes |
|---|---|---|
| Base tiles | PETG, ASA, or nylon-CF | PLA can creep in heat and sunlight. |
| Motor pods | PETG/ASA minimum, nylon-CF preferred | High stress part. Use thick walls. |
| Wheel core | PETG/ASA/nylon-CF | Print strong, then add rubber/TPU tire if possible. |
| Tire sleeve | TPU 95A | Optional but strongly recommended for grip. |
| Stabilizer feet | TPU or PETG with rubber pad | Rubber contact is better on court. |
| Handle sockets | PETG/ASA | Bolt to the base tiles and inner frame. |

## Editing and exporting

### In Docker (browser GUI)

From the repo root:

```powershell
docker compose --profile cad up --build openscad-gui
```

Then open `http://localhost:6081/vnc.html`. The repo is mounted at `/workspace`, so you can open any file under `cad/3d-printable-base/`.

### On the host

Open each `.scad` in a local OpenSCAD install and export the selected module to STL.

Recommended first exports:

1. `base_tile.scad`
2. `base_mounting_plate.scad`
3. `motor_pod.scad`
4. `drive_wheel_direct_hub.scad`
5. `stabilizer_foot.scad`
6. `handle_socket.scad`
7. `collector_funnel_bin.scad`
8. `full_robot_concept.scad`
9. `collector_curved_scoop.scad`

### Curved collector scoop

`collector_curved_scoop.scad` is the parametric centre ramp for lifting a tennis
ball smoothly from floor level. Its defaults are 180 mm wide, 180 mm long,
100 mm high, and 4 mm thick. `curve_exponent` controls the profile: larger
values keep the front flatter and make the rear steeper.

A continuous keyed rail is enabled across the rear edge. Print two copies of
`collector_curved_scoop_mounting_ear.stl`; the identical detachable ears slide
onto the rail from its ends and each contains a 5.5 mm clearance hole for an M5
bolt. `mount_fit_clearance` defaults to 0.30 mm per side for a PETG prototype.
Print one ear first and tune this clearance before committing to the scoop.
In the slicer, stand each ear on its narrow end so the keyed channel is vertical;
it then prints without internal supports.

The assembly preview also contains the 120 mm wide x 45 mm diameter roller with
a 6.35 mm shaft bore, a 190 mm reference shaft, two provisional supports, a shaft
coupler, and a 37 mm motor envelope. The shaft and motor are reference geometry,
not printable plastic parts. The support and motor-hole geometry must be
finalized from measurements of the real bearings, coupler, and motor face.

The full scoop fits on common hobby printers. If a smaller printer or easier
prototype is preferred, export two 90 mm halves from the repository root:

```powershell
docker compose --profile cad run --rm openscad openscad `
  -D 'render_part="left"' `
  -o cad/3d-printable-base/stl/collector_curved_scoop_left.stl `
  cad/3d-printable-base/collector_curved_scoop.scad

docker compose --profile cad run --rm openscad openscad `
  -D 'render_part="right"' `
  -o cad/3d-printable-base/stl/collector_curved_scoop_right.stl `
  cad/3d-printable-base/collector_curved_scoop.scad
```

For a first PETG prototype, rotate the full scoop 90 degrees and print it on one
straight side edge. It then occupies roughly 208 x 100 mm of bed area including
the rear mounting tabs and normally needs no support. Use 0.24-0.28 mm layers,
5 perimeters, 6 top/bottom layers, and 20-30% gyroid infill. A brim is
recommended because the part stands 180 mm tall in this orientation. Each
optional half can instead be printed on its straight split edge.

(`front_caster_mount.scad` is deprecated — the 4WD base uses four `motor_pod.scad` pods instead of casters.)

## Starting print settings

- Layer height: 0.24-0.32 mm for structural parts
- Perimeters/walls: 5-8
- Infill: 35-55% gyroid/cubic
- Top/bottom layers: 6-8
- Heat-set inserts: M4 or M5 where repeated assembly is expected
- Use washers under bolt heads so plastic does not crush locally

## Mechanical notes

- A wooden base is a practical first mobile prototype. Use 21 mm birch marine
  plywood for the rugged first chassis, or 9-12 mm plywood after the geometry is
  proven and reinforced with rails. For the 21 mm cut plan, see
  `docs/plywood-cut-list.md`. Bolt the four printed motor pods,
  collector rig, electronics, and battery onto it. This is
  faster to drill, adjust, and replace than a fully printed chassis while the
  robot geometry is still changing.
- Use `base_mounting_plate.scad` as the first drill/CAD reference for the
  physical chassis. It keeps the mounting points for the four motor pods,
  collector, battery straps, electronics standoffs, handle sockets, stabilizer
  brackets, and a reserved launcher/feed zone visible in one model. With the
  default `show_verticals=true`, it also shows the upright frame, electronics
  trays, collector uprights, battery retainers, handle rails, and future
  launcher/feed uprights.
- The battery bay is intentionally removable: split cross rails leave side
  access, and the model shows a removable top strap/clamp instead of glued
  blocks that trap the battery.
- During launch, stabilizer feet still help damp recoil even with 4WD. Use them.
- The base is 4WD skid-steer: four driven 180 mm wheels, two per side, placed
  symmetrically about the chassis center (no casters). The two motors on each
  side are wired in parallel to one BTS7960 driver. This gives more traction and
  removes the caster scrub that previously stalled in-place turns. Avoid servo
  steering — skid-steer turning needs no steering mechanism.
- The concept assembly shows a level (un-pitched) body: all four wheels are the
  same diameter and sit coplanar, so the wooden body and mounted modules stay
  flat to the ground.
- Keep the battery low and near the middle of the footprint.
- Make motor pods replaceable; they will be the first parts to revise.
- Print one wheel at reduced width first to verify motor shaft fit.
- Add rubber/TPU tread to the wheel. Hard plastic wheels will slip and chatter.
- Treat `collector_funnel_bin.scad` as a tunable bench rig, not a final enclosure.
  The throat width, roller gap, and bin geometry should follow the Webots physics
  experiments before printing a full-size revision.
- Keep the intake roller close to the front edge of the base. The low scoop can
  guide the ball, but the roller contact line should arrive before the chassis
  can push the ball away.
- Use `full_robot_concept.scad` as an assembly/reference model. It is meant to
  communicate layout and mounting relationships: wooden base, upper frame,
  electronics/battery module, receiving bin, collector intake, launcher wheels,
  the four drive wheels, cover mounting rails, removable panels, and
  transport handle. Do not print it as one object.
- The intended ball path in `full_robot_concept.scad` is:
  loose ball -> floor-level left/right funnels -> 12 cm full-width front roller
  -> collection channel -> hopper behind the roller -> metering/feed throat ->
  front dual flywheels with a forward-facing guarded opening. The OAK-D
  reference mount sits at the front, roughly 45 cm above ground, between the
  roller and flywheel opening. It looks forward with only a 10-15 degree
  downward tilt so court lines, net, fences, and tennis balls remain in view.
  A raised LiDAR mast sits above the protective frame for clear wide-area
  visibility, and an IR break-beam pair sits immediately after the roller throat
  to detect balls entering the collector path.
- The transparent cover panels in the concept are placeholders for removable
  polycarbonate/ABS panels. They show where the outer shell can land on rails
  and standoffs while leaving service access to the intake, feed gate, and
  launcher.
