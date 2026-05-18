# 3D Printable Base Prototype

Parametric OpenSCAD sources for a first printable tennis-robot base.

Scope:

- printable modular base tiles
- printable side motor pods
- printable direct-drive wheels
- printable stabilizer feet
- printable trolley-handle sockets

Out of scope for this folder:

- ball basket / hopper
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

## Exporting to STL

Open each `.scad` in OpenSCAD and export the selected module to STL.

Recommended first exports:

1. `base_tile.scad`
2. `motor_pod.scad`
3. `drive_wheel_direct_hub.scad`
4. `stabilizer_foot.scad`
5. `handle_socket.scad`

## Starting print settings

- Layer height: 0.24-0.32 mm for structural parts
- Perimeters/walls: 5-8
- Infill: 35-55% gyroid/cubic
- Top/bottom layers: 6-8
- Heat-set inserts: M4 or M5 where repeated assembly is expected
- Use washers under bolt heads so plastic does not crush locally

## Mechanical notes

- Do not support the robot only on two wheels during launch. Use stabilizer feet.
- Keep the battery low and near the middle of the footprint.
- Make motor pods replaceable; they will be the first parts to revise.
- Print one wheel at reduced width first to verify motor shaft fit.
- Add rubber/TPU tread to the wheel. Hard plastic wheels will slip and chatter.

