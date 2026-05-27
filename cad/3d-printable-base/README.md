# 3D Printable Base Prototype

Parametric OpenSCAD sources for a first printable tennis-robot base.

Scope:

- printable modular base tiles
- full-size base mounting/drill template with component mounting points
- printable side motor pods
- printable direct-drive wheels
- printable stabilizer feet
- printable passive front caster mounts
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
| Front caster mounts | PETG/ASA/nylon-CF | Use with bought swivel caster wheels. |
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
8. `front_caster_mount.scad`
9. `full_robot_concept.scad`

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
  `docs/plywood-cut-list.md`. Bolt the printed motor pods,
  front caster mounts, collector rig, electronics, and battery onto it. This is
  faster to drill, adjust, and replace than a fully printed chassis while the
  robot geometry is still changing.
- Use `base_mounting_plate.scad` as the first drill/CAD reference for the
  physical chassis. It keeps the mounting points for motor pods, front casters,
  collector, battery straps, electronics standoffs, handle sockets, stabilizer
  brackets, and a reserved launcher/feed zone visible in one model. With the
  default `show_verticals=true`, it also shows the upright frame, electronics
  trays, collector uprights, battery retainers, handle rails, and future
  launcher/feed uprights.
- The battery bay is intentionally removable: split cross rails leave side
  access, and the model shows a removable top strap/clamp instead of glued
  blocks that trap the battery.
- Do not support the robot only on two wheels during launch. Use stabilizer feet.
- For the collection-only MVP, prefer two driven side wheels plus two passive
  front swivel casters. Avoid servo steering until the differential-drive base
  proves insufficient.
- Keep the battery low and near the middle of the footprint.
- Make motor pods replaceable; they will be the first parts to revise.
- Print one wheel at reduced width first to verify motor shaft fit.
- Add rubber/TPU tread to the wheel. Hard plastic wheels will slip and chatter.
- Treat `collector_funnel_bin.scad` as a tunable bench rig, not a final enclosure.
  The throat width, roller gap, and bin geometry should follow the Webots physics
  experiments before printing a full-size revision.
- Use `full_robot_concept.scad` as an assembly/reference model. It is meant to
  communicate layout and mounting relationships: wooden base, upper frame,
  electronics/battery module, receiving bin, collector intake, launcher wheels,
  rear drive wheels, front casters, cover mounting rails, removable panels, and
  transport handle. Do not print it as one object.
- The intended ball path in `full_robot_concept.scad` is:
  front intake -> wide compliant roller -> transfer chute -> open receiving bin -> sloped
  bin floor -> narrow feed channel -> metering gate -> dual flywheels -> guarded
  launch chute.
- The transparent cover panels in the concept are placeholders for removable
  polycarbonate/ABS panels. They show where the outer shell can land on rails
  and standoffs while leaving service access to the intake, feed gate, and
  launcher.
