// Non-printed fabrication drawing/context for the Phase-2 drive-test ballast.
// Units mm. This plate is plywood or metal, not an STL release.
$fn=48;
use <modular-chassis-v3.scad>

show_frame=true;
show_ballast=true;
plate_size=[160,480,12];
plate_center=[360,0,58]; // bottom Z52: seats on the 30 mm-high front terminals
target_total_mass_kg=5.4479; // plate + fasteners + secured loose ballast

// Eight existing X300 top-grid positions use 10 mm rigid spacers because the
// X300 crossbar top is Z42. The forward supports are removable inward-bolted
// C-hook clamps around the final front terminals; no Gamma feet or rail holes.
x300_holes=[[300,-150],[300,-125],[300,-100],[300,-75],
            [300,75],[300,100],[300,125],[300,150]];
terminal_saddles=[[310,-185],[330,-185],[310,185],[330,185]];

module plate() {
    difference() {
        translate(plate_center) cube(plate_size,center=true);
        for(p=x300_holes)
            translate([p[0],p[1],plate_center[2]-plate_size[2]/2-1])
                cylinder(d=4.5,h=plate_size[2]+2);
        // M6 through holes for inward legs of removable C-hook clamps. Each
        // hook reaches under its 50 mm terminal without entering tyre space.
        for(p=terminal_saddles)
            translate([p[0],p[1],plate_center[2]-plate_size[2]/2-1])
                cylinder(d=6.5,h=plate_size[2]+2);
        // Four alternative M8 ballast tie-down holes around X360.
        for(x=[335,385],y=[-60,60])
            translate([x,y,plate_center[2]-plate_size[2]/2-1])
                cylinder(d=8.5,h=plate_size[2]+2);
    }
}

if(show_frame) %frame_assembly();
color([0.72,0.55,0.25,0.75]) plate();
if(show_ballast) %color([0.20,0.20,0.22,0.65])
    translate([360,0,105]) cube([100,150,70],center=true);

echo("DRIVE_TEST_BALLAST_PLATE", plate_mm=plate_size,
     plate_center_mm=plate_center, x300_m4_holes_mm=x300_holes,
     terminal_saddle_m6_holes_mm=terminal_saddles,
     total_target_mass_kg=target_total_mass_kg,
     note="target includes plate, fasteners and secured loose ballast; weigh assembly");
