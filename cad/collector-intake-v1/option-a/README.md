# Collector intake — Option A

This directory is the single source for the agreed curved-cheek, plywood-bridge
collector intake. Do not mix its exports with the older straight-cheek/rail
files one directory above.

## Contents

- `option-a.scad`: assembly plus every current export selector;
- `params.scad`: self-contained measured and provisional inputs;
- `export-stls.sh`: regenerates every currently exportable Option A solid;
- `stl/`: generated binary STL files;
- `previews/`: assembly, orthographic and IR-placement renders.

## STL inventory

| STL | Qty | Status |
|---|---:|---|
| `cheek_left.stl` | 1 | Current Option A geometry |
| `cheek_right.stl` | 1 | Current Option A geometry |
| `ramp.stl` | 1 | Current Option A geometry; lip x=520 mm |
| `hex_hub.stl` | 2 | Provisional 6 mm D-bore / 12 mm RC hex |
| `bearing_cartridge.stl` | 2 | Provisional 626 envelope; measure bearings first |
| `ir_entry_bracket.stl` | 2 | Universal zip-tie carrier; provisional sensor envelope |
| `ir_confirmation_bracket.stl` | 2 | Universal zip-tie carrier; provisional sensor envelope |

The 18 mm bridge and uprights are plywood and therefore intentionally have no
STL. Purchased wheels, motors, shafts, bearings, couplers and fasteners also
have no manufacturing STL.

The exported full curved cheek is 253 x 128 x 132 mm because its bridge flange
extends rearward. It is geometrically inside the 256 x 256 mm P2S volume with
only 1.5 mm nominal margin at each X edge. Verify the slicer's real printable
area and disable/relocate any purge line before printing; do not assume that
the advertised build volume guarantees clearance. Split cheeks will be needed
for a 220 x 220 mm printer or if the P2S keep-out area cannot be cleared.

## Provisional parts

All files are exported together to prevent version mixing; that does not turn
unmeasured interfaces into production-ready parts. In particular:

- confirm the wheel's actual 12 mm hex depth and the purchased 6 mm shaft flat
  before printing the hubs as production parts;
- confirm the real bearing OD/width before printing cartridges;
- confirm the IR module body and optical-centre dimensions before printing the
  drop brackets;
- the final motor clamp/face adapter still requires the actual mounting-hole
  pattern. The assembly currently shows only the measured motor envelope.

Regenerate and manifold-check the set with:

```bash
./cad/collector-intake-v1/option-a/export-stls.sh
```
