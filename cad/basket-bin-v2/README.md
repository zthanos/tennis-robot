# Basket bin v2.1 + entry hood — OpenSCAD

Manufacturing-reference model of the sim-validated basket (debug log
#45-#56): sunken removable wire-mesh bin, load-management tray, receiving
chute, front retention (corner guards + centre lip) and the chassis-mounted
entry hood that closed the 45-ball load gate.

All dimensions live in `params.scad` (mm, ground frame, +x forward) and are
copied from the validated sources — `docs/basket-bin-redesign-spec-el.md`
and `ros2_ws/src/tennis_robot/urdf/components/basket.urdf.xacro`. Do not
edit a number here without re-validating in the Gazebo bench.

## Files

| File | Content |
| --- | --- |
| `params.scad` | Every dimension, with spec/log citations |
| `lib.scad` | Mesh-panel / wall / handle primitives (40 mm grid, 4 mm wire, 6 mm frame) |
| `bin.scad` | The removable weldment: floor, walls, tray, chute, guards, lip, flange, handles |
| `hood.scad` | Chassis-mounted entry hood: inclined mesh roof + side cheeks + mounts |
| `chassis_context.scad` | Reference-only plate w/ real opening, battery, IR pair, scale balls |
| `assembly.scad` | Everything together; `-D 'explode=160'` lifts the bin out |

## Render / export

```bash
docker compose run --rm openscad openscad -o out.png cad/basket-bin-v2/assembly.scad
docker compose run --rm openscad openscad -o bin.stl --export-format binstl cad/basket-bin-v2/bin.scad
```

## Deliberate deviations from the sim model

1. **Hood cheeks trimmed to x 430-470** (sim: 420-470). The sim volume
   interpenetrates the bin's corner guards; fixed links never collide in
   Gazebo, but these are two separate physical parts. The leftover 10 mm
   slot above the guards is far below ball diameter — no functional change.
2. **Bin removal is hood-off-first.** The receiving chute (part of the bin
   weldment) sits between the hood cheeks; a straight vertical lift fouls
   the hood roof after ~85 mm. The hood mounts must therefore be bolted or
   hinged (flip-up), not welded. Removing/tilting the hood first gives a
   clean straight lift through the plate opening.
3. Mesh visuals in the sim are indicative wires; here the panels carry a
   real 6 mm perimeter frame and <= 40 mm grid pitch as buildable geometry.
4. The front floor skid (spec §2) is realised by the chute/floor frame rod
   radius; no separate part.

## Still open

- Hood mount detail: modelled as a transverse bar on the plate side strips
  (`hood_mounts()`); mounting off the funnel frame is the alternative if
  the bar shades the camera. Decide at frame build time.
- IR beam vs mesh alignment (locally larger grid eye if a wire lands on
  the beam line at x 445, z 70.5).
- Plywood-cut-list revision (spec §6.6) once the frame layout is frozen.
