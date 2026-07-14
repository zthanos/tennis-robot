// Basket bin v2.1 + entry hood assembly.
//
//   openscad assembly.scad                      # full assembly
//   openscad -D 'explode=80' assembly.scad      # bin lifted out
//   openscad -D 'show_context=false' ...        # weldments only
//
// Frames: mm, ground frame, +x forward (see params.scad).
include <params.scad>
use <bin.scad>
use <hood.scad>
use <chassis_context.scad>

show_bin     = true;
show_hood    = true;
show_context = true;
show_balls   = true;
explode      = 0;   // mm to lift the bin for the removal check

if (show_context) chassis_context(with_balls = show_balls && explode == 0);
if (show_bin) translate([0, 0, explode]) bin();
if (show_hood) hood();
