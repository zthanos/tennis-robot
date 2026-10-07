// PROVISIONAL simulation mesh only; not manufacturing CAD.
// Accepted 256 x 314 x 8 mm upper panel with two 12 mm shaft/service openings.
$fn = 128;

difference() {
    cube([256, 314, 8], center=true);
    for (y = [-129, 129])
        translate([0, y, 0]) cylinder(d=12, h=10, center=true);
}
