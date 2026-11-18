# P2-97 full-scene gate note

P2-99b addendum (2026-11-18): the active exporter now emits 11 files. It
removes `p297_collar.stl` and adds `p299_panel_collar.stl` plus
`p299_c1_pedestal.stl`. Both new files pass the permanent one-body,
floating-island, downward-face and slice gates; declared hole axes pass 9/9
and 4/4 respectively. The final raw scene has 94 bodies per angle, zero
exports, zero contained pairs and the same 48 historical collision pairs at
-10/-3/0/15/20. The complete record is
[P2-99b](../../../docs/hardware/p2-99b-panel-collar-rplidar-c1-pedestal-el.md).

Commit `0092fcd6` fixed printed-part recognition for STL files renamed by
export scripts.  P2-97c restores the correct pre-`5e3404ad` baseline of
**48 collision pairs**, with zero contained pairs and zero export failures at
-10/-3/0/15/20 degrees.  It removes `control_panel_v2 × p297_logic_box`
(52.5 mm3) and `p297_logic_box × p297_power_box` (20 mm3), which were new
P2-97 overlaps and were incorrectly accepted by P2-97b.

The earlier comparison with a stored count of 76 is no longer valid: the old
report used coarser body decomposition and included pod bodies intentionally
retired by P2-97. Collision acceptance uses the historical 48-pair set before
the P2-97 stack changes, not the immediate parent HEAD.

The P2-97 export runs the common `check-stl-printability.sh` after every
generated STL.  It includes `check-stl-components.py`,
`check-stl-downward-faces.py` and `check-stl-slices.py`; export stops on an
unclassified failure.  The printability test set passes all ten files,
including both seam coupons; its worst unclassified downward result is
38.93 mm².
OpenSCAD runs with `--hardwarnings`, so assertion/warning diagnostics also
stop the script instead of leaving a misleading STL behind.  The power box
was exported with the measured `P297_BTS_HEIGHT_MM=36` geometry.

P2-97 now additionally runs `check-stl-slices.py` on the five structural
stack exports.  At 0.2 mm layer spacing above Z=0.5 it rejects an opposing
material-boundary buffer width below 1.6 mm or a material island below
25 mm².  The 2026-11-17 export results are:

| STL | sections | minimum buffer width | minimum island |
|---|---:|---:|---:|
| `p297_base_adapter.stl` | 77 | 1.600 mm | 153.60 mm² |
| `p297_energy_spacer.stl` | 387 | 1.600 mm | 25.00 mm² |
| `p297_power_box.stl` | 387 | 1.600 mm | 25.00 mm² |
| `p297_logic_box.stl` | 447 | 1.600 mm | 25.00 mm² |
| `p297_collar.stl` | 117 | no violation (>=1.600 mm) | 49.94 mm² |

The common socket has 2.4 mm walls and a 1.6 mm tongue without the former
0.05 mm tuning ribs.  The 24 mm collar uses continuous 2.4 mm side walls;
the taller floors retain a lattice with 5 mm wall depth, 5 mm posts and
7.2 mm diagonals.  The segmented 1.6 mm V-beam ridges keep every generated
layer and every downward patch inside both export gates.

For the base adapter, export additionally runs `check-stl-through-holes.py`
against the four declared P2-90d M4 axes.  Its independent finite ray-cast
must find no mesh triangle between Z=-1 and Z=17.  The corrected adapter is
one body, passes all four Ø4.5 axes, and its two front Ø11 tongue access
notches plus the energy-spacer head pockets still pass the 40 mm² support gate.
