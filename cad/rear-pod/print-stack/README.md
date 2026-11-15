# P2-97 Phase 1 — FIT-CHECK stackable electronics tower

Status: **FIT-CHECK, do not batch-print yet.**  All parts are support-free in
the exported orientation and fit a 250 x 250 mm bed.  The common nominal box
footprint is 138 x 190 mm; small keyed/seam details keep every exported mesh
below 143 x 195 mm.

The power and logic floors use a plain through-M3 mounting grid: Ø3.4 mm
holes on a 20 mm pitch, carried by 8 mm orthogonal ribs.  This is not
Gridfinity and has no bins, baseplates, magnets or proprietary pitch.  The
measured BTS positions and the removable Mega/perfboard/Pi carriers remain.

## What differs from the expected rod tower

The four continuous M5 rods were not selected.  Each floor is a separately
removable open-rear box.  Four 12 x 12 x 2.2 mm keys at every seam take shear;
four M3 screws provide clamp load into Ruthex RX-M3x5.7 inserts in the box
below.  The screws are reached from inside the box above, so the chassis/base
joint and all lower seams stay assembled during floor service.

The old 300 mm-high thin-post load path is also gone.  Lateral load goes from
the existing control panel into the collar, the X-braced side/front walls, the
keyed seams and the adapter rails, then through the existing P2-90d base holes.
No chassis hole is added.

## Installed stack and access

Installed datums are measured from ground/chassis coordinates:

- base adapter: Z64..72, seated on the existing P2-90d locating bosses;
- empty Phase-1 energy spacer: Z72.25..142.25 (rear open; no provisional drawers);
- power box: Z142.50..212.50, four measured 50 x 50 BTS7960 boards on measured
  40 x 40 patterns, plus adjustable fuse/DC-DC lands;
- logic box: Z212.75..294.75, three rear-removable horizontal trays at Z+10/+36/+62;
- collar: Z295.00..319.00;
- existing control-panel top: Z323; E-stop mushroom top: approximately Z344.

Mega and Pi USB extension leads rise in the continuous rear cable channel and
exit through the existing 24 x 8 panel slot.  The panel keeps its four existing
Ø8.6 locating holes; M4 washers/nylocs clamp it to the collar.

Mounting patterns used by the FIT-CHECK carriers:

- Mega 2560 in the user's measured 60 x 110 acrylic case: upper pair 50 mm,
  lower/middle pair 37 mm, as recorded in `motion-electronics-tray/params.scad`;
- user's 80 x 120 perfboard: measured 75 x 115 mm hole centres, represented by
  8 mm cross-slots because the source measurement was made with a ruler;
- Raspberry Pi 5 bare PCB: official 58 x 49 mm pattern, M2.5 fasteners;
- BTS7960/HW-039: measured 40 x 40 mm pattern on 50 x 50 boards.

If the Pi 5 is to remain in its metal/M.2 case, measure that case's mounting
holes before printing; the official bare-PCB holes are not assumed to pass
through the case.

## PETG and print-time planning estimate

The repository has no configured slicer profile, so these are geometry-based
planning numbers, not G-code times.  They use watertight STL volume, PETG at
1.27 g/cm³, and an effective 32 g/h for a 0.4 mm nozzle / 0.2 mm layer,
including a small per-part handling/travel allowance.  Expect ±20% until the
user's actual slicer profile is run.

| STL | PETG | Estimated time |
| --- | ---: | ---: |
| `p297_base_adapter.stl` | 72 g | 2.4 h |
| `p297_energy_spacer.stl` | 105 g | 3.4 h |
| `p297_power_box.stl` | 167 g | 5.3 h |
| `p297_logic_box.stl` | 216 g | 6.9 h |
| `p297_collar.stl` | 47 g | 1.6 h |
| `p297_mega_carrier.stl` | 31 g | 1.1 h |
| `p297_perf_carrier.stl` | 43 g | 1.5 h |
| `p297_pi5_carrier.stl` | 23 g | 0.8 h |
| **Phase 1 total** | **about 704 g** | **about 23.0 h** |

The values above are rounded targets; regenerate the table from final meshes
if geometry changes.  No individual estimate exceeds 8 h.

## 50 N stiffness comparison

This is a conservative first-order truss estimate, not FEA.  It uses printed
PETG E=1.2 GPa, the 4.1 x 3.2 mm diagonal section, axial X-brace stiffness
`2EA/L * (b/L)^2`, and adds explicit allowances for panel bending, adapter/
collar bending and joint slip.

| Load-path term at 50 N | Bolted keyed boxes | Four M5 rods + same lattice |
| --- | ---: | ---: |
| lattice shear, floors in series | 0.32 mm | 0.32 mm |
| panel + collar + base adapter | 0.62 mm | 0.62 mm |
| four seams / clamp slip allowance | 0.24 mm | 0.12 mm |
| **estimated E-stop displacement** | **1.18 mm** | **1.06 mm** |

Both clear the 1.5 mm requirement.  The box system is selected because the
~0.12 mm stiffness penalty is small and the service split is much better.
Four M5 steel rods would be cut to about **285 mm** (259 mm clamp stack plus
washers/nuts and adjustment), weighing about 176 g before hardware.  Sixteen
M3 screws/inserts plus printed seam keys/bosses are about 43 g.  The rod variant
would save roughly 0.7 h and 22 g of printed seam material, but finishes about
108 g heavier overall and requires disturbing the whole stack to remove a
middle floor.

After adding the 20 mm M3 grid and integrating all formerly floating bodies,
the same volume model gives 703.5 g / 22.944 h, versus 703.5 g / 22.95 h for
the preceding FIT-CHECK export: no print-time increase within model precision.

## Hardware — Phase 1

- 16 x Ruthex RX-M3x5.7 heat-set inserts (four per seam);
- 16 x M3x10 socket-head screws and M3 washers;
- 4 x existing-base M4x20 screws, washers and nylocs for the adapter;
- 4 x M4x20 screws, broad washers and nylocs for the existing panel holes;
- 16 x M3 board screws plus 16 x 20 mm spacers for the four BTS7960 boards;
- 4 x M3 screws/spacers for the measured Mega acrylic case;
- 4 x M3 screws/spacers for the user's perfboard cross-slots;
- 4 x M2.5 screws, washers and 6 mm spacers for a bare Raspberry Pi 5;
- 2 x short panel-mount USB extensions (Mega and Pi), length selected after
  the real cable route is checked;
- fuse/DC-DC fasteners remain FIT-CHECK slots until those two bodies are
  measured.

## Retired P2-90d/P2-90 assets

The Phase-1 current scene no longer uses `side_frame.stl`, `panel_posts.stl`,
`vertical_electronics_plate.stl`, `pi_shelf.stl`, `parkside_dock_left/right.stl`,
`parkside_drawer_left/right.stl`, `ugreen_post_holder.stl`,
`ugreen_clamp_caps.stl`, or `ugreen_rear_retainer.stl`.  The existing
`base_left.stl`, `base_right.stl`, `corner_tie.stl`, and `control_panel.stl`
remain active/reused.

Phase 2 begins only after measuring the real Parkside batteries/contact docks;
the empty energy spacer is intentionally not a guessed drawer system.
