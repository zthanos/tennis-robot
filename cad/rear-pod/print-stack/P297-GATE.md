# P2-97 full-scene gate note

Commit `0092fcd6` fixed printed-part recognition for STL files renamed by
export scripts.  The corrected independent checker reports **48 collision
pairs** for the integrated Phase-1 scene, zero contained pairs and zero export
failures at -10/-3/0/15/20 degrees.  No pair contains a `p297_*` or `p293*`
body.

The earlier comparison with a stored count of 76 is no longer valid: the old
report used coarser body decomposition and included pod bodies intentionally
retired by P2-97.  The acceptance datum is now the corrected 48-pair report;
P2-97 adds no collision to it.

The P2-97 export also runs `check-stl-components.py` and
`check-stl-downward-faces.py` after every generated STL.  Export stops unless
each file is one material body and every connected horizontal downward patch
above the print bed is at most 40 mm².  The printability test set passes all
ten files, including both seam coupons; its worst result is 38.93 mm².
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
