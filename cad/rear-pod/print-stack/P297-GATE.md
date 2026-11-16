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
ten files, including both seam coupons; its worst result is 35.28 mm².
OpenSCAD runs with `--hardwarnings`, so assertion/warning diagnostics also
stop the script instead of leaving a misleading STL behind.  The power box
was exported with the measured `P297_BTS_HEIGHT_MM=36` geometry.
