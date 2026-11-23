# P2-97 Phase 1 — FIT-CHECK stackable electronics tower

## P2-99f — corrected RPLIDAR C1 pedestal; panel/collar unchanged

The active top of the P2-97 stack is now `p299_panel_collar.stl`; the former
`p297_collar.stl` and separate `stl-v2/control_panel.stl` are not printed for
this build. The new body reuses `control_panel_v2()` without changing the
released E-stop Ø22.5, toggle Ø12.3, START Ø16.3, status Ø8, 24×8 capsule or
four Ø8.6 locating features. The released L-beam at X=-69..-49 and all panel
ribs are fused into the collar. The engaged female seam remains 0.35 mm per
side with the same 0.25 mm vertical stack gap. Its support-free backing was
changed to 45-degree ramps because the required print orientation places the
control face on the bed. Exported footprint is 138×190×28 mm, inside 250×250.

`p299_c1_pedestal.stl` is a separate 62×73×67 mm print, exported upside-down
with its continuous 62×73×4 mm C1 plate on the bed. Four asymmetric M3 axes
key it to blind Ø4.0×6.7 Ruthex seats in the unified body. Four Ø2.9 M2.5
clearance bores use the official 43×43 mm C1 pattern; M2.5×8 screws through
the 4 mm plate enter the C1 by exactly 4 mm. The nominal C1 centre is
panel-local `[0,+110]`, opposite the E-stop, with base Z390 and scan plane
Z419.8. The nearest pedestal edge has 25.85 mm clear from the Ø16.3 START
edge. The 78 mm C1-centre-to-START datum and the installed position were
confirmed by the user before P2-99f.

The measured XH2.54-5P housing passes the unchanged 17×10 mm opening. The
70 cm captive lead leaves the middle of the C1's negative-Y side at its base
plane, passes through the new 17×26.2 plate notch, descends inside a 19 mm
service chute, is retained through four Ø5 tie passages,
and enters the logic volume without crossing tilt or basket motion. Keep
R≥30 mm. The 35×30×5 adapter is tied to the inside +Y logic lattice near the
Pi, with a short USB-C lead; connector-side orientation must be checked on
the real adapter.

Both STLs pass one-body, floating-island, downward-face and slice gates.
The unified body additionally passes all nine declared released circular/
central-slot axes; the pedestal passes all four C1 plus four unchanged M3
axes (8/8). The full raw scene is
unchanged at 48 historical pairs, zero contained pairs and zero export
failures at -10/-3/0/15/20 degrees.

P2-99f pedestal status: **READY TO PRINT (2026-11-23).** Its four Ø4.5
head/key corridors have exact-solid intersection 0.000000 mm³; the geometric
scan band is clear at ±2° and the scan plane remains Z419.8 mm. Full report:
[P2-99f](../../../docs/hardware/p2-99f-c1-pedestal-first-print-correction-el.md).
All parts are support-free in the exported orientation
and fit a 250 x 250 mm bed.  The common nominal box footprint is 138 x 190 mm;
the complete perimeter seam stays inside it.  The base adapter alone has two
local front mounting ears and is 141.5 x 190 mm, still well inside the bed and
the checked F-rear envelope.

The BTS7960/IBT-2 geometry was measured by the user on 2026-11-16: 50 x 50 mm
PCB, 40 x 40 mm holes, 50 x 30 x 20 mm centred heatsink below the PCB, 15 mm
maximum component height above it and about 36 mm total.  The print set was
exported with `P297_BTS_HEIGHT_MM=36`.  The script stops before export if the
value is absent/invalid or does not match the measured 20+1+15 mm stack.

The power and logic floors use a plain through-M3 mounting grid: Ø3.4 mm
holes on a 20 mm pitch, carried by 8 mm orthogonal ribs.  This is not
Gridfinity and has no bins, baseplates, magnets or proprietary pitch.  The
measured BTS positions and the removable Mega/perfboard/Pi carriers remain.

## What differs from the expected rod tower

The four continuous M5 rods were not selected.  Each floor is a separately
removable open-rear box.  An 8 mm-high continuous tongue enters the box above
along both side walls and the closed +X/front wall.  This three-sided
perimeter—not the screws or four point keys—carries all seam shear.

Four M3 screws into Ruthex RX-M3x5.7 inserts only stop a floor lifting off.
Two would carry the nominal lift load, but four are selected to keep the large
perimeter seated symmetrically and retain it after one missed/loose fastener.
No screw shear is credited.  Access from inside the box above leaves every
lower seam assembled while a floor is removed.

The old 300 mm-high thin-post load path is also gone.  Lateral load goes from
the existing control panel into the collar, the X-braced side/front walls, the
perimeter seams and the adapter rails, then through the existing P2-90d base holes.
No chassis hole is added.

The adapter reuses the four P2-90d M4 axes at local X=-58/+66.5, Y=±54 mm.
All four are now Ø4.5 through-bores across the full adapter/tongue height.  At
each +66.5 mm front axis, a bed-supported 12 x 14 mm ear leaves 3.75 mm of
material in X and 4.75 mm in Y around the bore.  An Ø11 vertical notch through
the tongue accepts an M4 head and Ø9 washer with 1 mm radial tool clearance.
Tighten these two fasteners from above before seating the energy spacer; its
matching head pocket has a 45° roof, so the installed hardware does not alter
the 0.35 mm seam fit or require support.

## Installed stack and access

Installed datums are measured from ground/chassis coordinates:

- base adapter: Z64..72, seated on the existing P2-90d locating bosses;
- empty Phase-1 energy spacer: Z72.25..142.25 (rear open; no provisional drawers);
- power box: Z142.50..212.50, four measured 50 x 50 BTS7960 boards on measured
  40 x 40 patterns.  PCB undersides are at Z167.70; heatsink bottoms at
  Z147.70, 2.0 mm above the floor-grid top; the 15 mm top envelope ends at
  Z183.70.  Fuse/DC-DC use the general M3 grid after their bodies are measured;
- logic box: Z212.75..294.75, three rear-removable horizontal trays at Z+10/+36/+62;
- unified panel/collar: Z295.00..323.00; E-stop mushroom top: approximately Z344;
- C1 pedestal: Z323.00..390.00; C1 scan plane: Z419.80.

Mega and Pi USB extension leads retain the continuous rear cable channel and
existing 24×8 slot. The C1 lead uses its new provisional 17×10 opening. The
four existing Ø8.6 locating holes remain exact but no longer fasten panel to
collar, because those two solids are one print.

Mounting patterns used by the FIT-CHECK carriers:

- Mega 2560 in the user's measured 60 x 110 acrylic case: upper pair 50 mm,
  lower/middle pair 37 mm, as recorded in `motion-electronics-tray/params.scad`;
- user's 80 x 120 perfboard: measured 75 x 115 mm hole centres, represented by
  8 mm cross-slots because the source measurement was made with a ruler;
- Raspberry Pi 5 bare PCB: official 58 x 49 mm pattern, M2.5 fasteners;
- BTS7960/HW-039: measured 40 x 40 mm pattern on 50 x 50 boards.

The BTS centres are X=±30, Y=±40 mm.  The positive-Y pair has its terminal
edge toward +Y; the negative-Y pair is rotated 180° toward −Y.  This leaves
26.8 mm between each 50 mm board edge and the inner wall face, exceeding the
25 mm screwdriver/1.5 mm² cable requirement.  Signal headers face the 30 mm
central aisle and then the open −X rear/cable channel.  Four framed 52 mm-wide
side-lattice windows align with the terminal edges for motor-cable exit; their
upper edge is generated from the measured 15 mm top envelope plus 5 mm.

Each PCB rests on four integrated Ø8 x 22 mm standoffs.  A Ø3.4 through-hole
takes an M3x30 screw, top washer and washer/nyloc below the floor.  The screw
axis is 5 mm outside the 30 mm heatsink width; after the M3 radius it retains
3.3 mm nominal clearance, while the boss itself retains 1.0 mm.  Dry-fit one
board before installing all four because the heatsink outline is hand-measured.
The heatsink bottom has 2.0 mm free air above the floor.

The heatsink fins run along X.  Six 130 x 5 mm floor slots—three beneath each
Y row—form continuous channels below both heatsinks and open through the −X
rear edge.  They avoid all sixteen standoff feet.  The former fuse/DC-DC rear
lands were removed because they occupied this airflow path; the existing
20 mm-pitch M3 grid remains their adjustable mounting interface.

If the Pi 5 is to remain in its metal/M.2 case, measure that case's mounting
holes before printing; the official bare-PCB holes are not assumed to pass
through the case.

## Perimeter seam, corner section and fit coupon

Each seam uses full-height 14 x 14 mm solid corner columns centred at X=±44,
Y=±82 mm.  Their complete footprints stay inside the 138 x 190 mm outline.
The centred Ø4.0 insert pocket leaves 5.0 mm radial material, is 6.7 mm deep,
and is surrounded by uninterrupted column material for its full depth.

The continuous tongue is 1.40 mm thick (2.26 extrusion widths at 0.62 mm) and
8 mm high.  Its groove has parameterized clearance of 0.35 mm per side by
default (`P297_SEAM_CLEARANCE_MM`) and a 45° self-supporting roof.  Short
calibration ribs leave 0.05 mm per side locally to suppress rattle; the rest
of the perimeter retains the full 0.35 mm/side assembly clearance.

Print `p297_seam_coupon_lower.stl` and `p297_seam_coupon_upper.stl` first.
They are exact 30 x 30 mm corner crops of the male/female seam, including one
calibration rib and the roof.  Each is one body; together they are 5.1 g and
0.33 h, below 30 minutes.  Accept only if they seat fully by hand, show no
visible lateral rattle, and separate without levering.  Otherwise change
`P297_SEAM_CLEARANCE_MM` and re-export both before any large print.

Before: the old 6 mm post, insert boss and shear key used separate axes, and
the post exceeded the nominal outline.

![P2-97 corner before](p297-corner-before.png)

After: section through the assembled continuous male/female perimeter corner.

![P2-97 corner after section](p297-corner-after-section.png)

All formerly flat elevated undersides were changed to 45° V/chamfered forms:
the wall rails, BTS service-window rails, six logic tray rails, collar supports
and seam-groove roofs.  `check-stl-downward-faces.py` groups connected
coplanar downward triangles and rejects any elevated patch over 40 mm².  In
the P2-99b export both new STL files pass; the pedestal maximum is 26.07 mm²
and the unified body maximum is 12.00 mm².

`check-stl-through-holes.py` independently ray-casts each declared hole axis
through the exported mesh.  The export declares all four adapter M4 axes from
Z=-1 to Z=17 and stops if any STL triangle blocks one.  The final adapter
passes 4/4; this check reproduces the earlier fault as 2/4 before the tongue
notches were added.

For the user's 0.6 mm nozzle profile, the assumed extrusion width is 0.62 mm.
The X braces have a 4.10 x 3.20 mm minimum section, corresponding to 6.61 and
5.16 extrusion widths respectively—at least five paths in the thinner axis,
well above the required two.  At 0.30 mm layers the 45° chamfers advance only
0.30 mm per layer, less than one 0.62 mm extrusion width.

## PETG and print-time planning estimate

The repository has no configured slicer profile, so these are geometry-based
planning numbers, not G-code times.  They use watertight STL volume, PETG at
1.27 g/cm³, 40 g/h and 0.10 h handling per STL.  The 40 g/h rate corresponds
to the requested 0.6 mm nozzle / 0.30 mm layer profile (0.62 mm line width,
about 9 mm³/s).  The actual slicer remains the acceptance source.

| STL | PETG | Estimated time |
| --- | ---: | ---: |
| `p297_base_adapter.stl` | 105.9 g | 2.75 h |
| `p297_energy_spacer.stl` | 150.2 g | 3.86 h |
| `p297_power_box.stl` | 203.2 g | 5.18 h |
| `p297_logic_box.stl` | 256.7 g | 6.52 h |
| `p299_panel_collar.stl` | 312.9 g | 7.92 h |
| `p299_c1_pedestal.stl` | 63.5 g | 1.69 h |
| `p297_mega_carrier.stl` | 30.9 g | 0.87 h |
| `p297_perf_carrier.stl` | 43.3 g | 1.18 h |
| `p297_pi5_carrier.stl` | 23.2 g | 0.68 h |
| **P2-99b installed print-set total** | **1189.8 g** | **30.65 h** |
| two seam coupons (print first) | 5.1 g | 0.33 h |

The values above are rounded targets; regenerate the table from final meshes
if geometry changes.  No individual estimate exceeds 8 h.

## 50 N stiffness comparison

This is a conservative first-order truss estimate, not FEA.  It uses printed
PETG E=1.2 GPa, the 4.1 x 3.2 mm diagonal section, axial X-brace stiffness
`2EA/L * (b/L)^2`, and adds explicit allowances for panel bending, adapter/
collar bending and joint slip.

| Load-path term at 50 N | Perimeter-seam boxes | Four M5 rods + same lattice |
| --- | ---: | ---: |
| lattice shear, floors in series | 0.32 mm | 0.32 mm |
| panel + collar + base adapter | 0.62 mm | 0.62 mm |
| four seated seams / local fit take-up | 0.20 mm | — |
| perimeter bearing deformation | 0.02 mm | — |
| rod clamp/joint allowance | — | 0.12 mm |
| **estimated E-stop displacement** | **1.16 mm** | **1.06 mm** |

Both clear the 1.5 mm requirement.  The box system is selected because the
~0.10 mm stiffness penalty is small and the service split is much better.
Four M5 steel rods would be cut to about **285 mm** (259 mm clamp stack plus
washers/nuts and adjustment), weighing about 176 g before nuts/washers.
Sixteen M3 screws/inserts are about 43 g.  The continuous seams raise the
installed plastic total to 888.6 g, but the complete M3 box stack remains
about 20 g lighter than the equivalent rod stack after four rods, nuts and
washers are included.  A middle floor also remains independently removable.

The 1.16 mm result assumes the coupon proves that the local 0.05 mm/side ribs
seat and are not removed.  With only the unrestrained 0.35 mm/side nominal
gap, clearance take-up could exceed 1.5 mm; a rattling coupon is therefore a
failed stiffness gate, not a cosmetic issue.

The unified top pushes the complete planning estimate above the former 25 h
Phase-1 target, although no individual part exceeds 8 h. These remain
geometry/throughput estimates, not slicer results. The power row is from the
measured 36 mm BTS export, not the former 20 mm placeholder.

## Hardware — Phase 1

- 16 x Ruthex RX-M3x5.7 heat-set inserts (four per seam);
- 16 x M3x10 socket-head screws and M3 washers;
- 4 x existing-base M4x20 screws, washers and nylocs for the adapter;
- 4 x M3 pedestal screws and 4 x Ruthex RX-M3x5.7 inserts in the asymmetric
  P2-99b panel pattern;
- 4 x M2.5x8 screws for the C1; the 4 mm pedestal pad limits sensor-base
  penetration to 4 mm;
- 16 x M3x30 board screws, 32 x washers and 16 x nylocs for the four BTS7960
  boards; the 22 mm standoffs are integrated in the power-box STL;
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
`base_left.stl`, `base_right.stl`, and `corner_tie.stl` remain reused.
`control_panel.stl` and `p297_collar.stl` are superseded by the unified P2-99b
body.

Phase 2 begins only after measuring the real Parkside batteries/contact docks;
the empty energy spacer is intentionally not a guessed drawer system.
