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
