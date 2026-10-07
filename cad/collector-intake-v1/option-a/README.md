# Collector intake — fixed-motor Option A supporting CAD

This directory supports the current fixed-motor, direct-drive,
compliant-tyre architecture. The authoritative architecture and open gates are
in [`standalone-intake-fixed-motor-compliant-tyre.md`](../../../docs/mechanism/standalone-intake-fixed-motor-compliant-tyre.md).
The motors and wheel centres do not translate.

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
| `ir_entry_bracket.stl` | 2 | Universal zip-tie carrier; provisional sensor envelope |
| `ir_confirmation_bracket.stl` | 2 | Universal zip-tie carrier; provisional sensor envelope |

The 18 mm bridge and uprights are plywood and therefore intentionally have no
STL. Purchased wheels, motors, `14-00012630` adapters and fasteners also have
no manufacturing STL. There is no remote transmission shaft, bearing
cartridge, coupler, or printed torque hub in the current intake architecture.

The exported full curved cheek is 253 x 128 x 132 mm because its bridge flange
extends rearward. It is geometrically inside the 256 x 256 mm P2S volume with
only 1.5 mm nominal margin at each X edge. Verify the slicer's real printable
area and disable/relocate any purge line before printing; do not assume that
the advertised build volume guarantees clearance. Split cheeks will be needed
for a 220 x 220 mm printer or if the P2S keep-out area cannot be cleared.

## Provisional parts

All files are exported together to prevent version mixing; that does not turn
unmeasured interfaces into production-ready parts. In particular:

- measure the FIT0186 shaft projection/flat/shoulder, purchased adapter seating
  and stop, Raid hex pocket, and removable axial retention before releasing the
  direct-drive stack;
- confirm the IR module body and optical-centre dimensions before printing the
  drop brackets;
- the final fixed motor bracket and bridge service opening require physical
  metrology. The invalid 22 mm vertical opening and invented M5 pattern were
  removed; CAD now cuts only the exact zero-clearance 30 mm motor envelope
  along the retained X-Z axis, without releasing manufacturing clearance.

Regenerate and manifold-check the set with:

```bash
./cad/collector-intake-v1/option-a/export-stls.sh
```
