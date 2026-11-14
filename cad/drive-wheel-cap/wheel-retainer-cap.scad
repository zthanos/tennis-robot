// Drive wheel retaining cap: an M4 from the outside clamps the wheel on its
// hex hub.  The spigot enters the hub bore from the outer face; the flange
// bears on the wheel face.  Printed flange-down, spigot up: no supports.
//
// User measurement 2026-11-14: bore ~10 mm, insertion ~5 mm, outer Ø15.

spigot_bore_d = 10.0;    // measured hub bore
spigot_clearance = 0.3;  // diametral running fit for a printed spigot
spigot_len = 5;
flange_d = 15;
flange_t = 3;
m4_clear = 4.5;
tip_chamfer = 0.6;       // eases the spigot into the bore

$fn = 96;

spigot_d = spigot_bore_d - spigot_clearance;

module wheel_retainer_cap() {
    difference() {
        union() {
            cylinder(d=flange_d, h=flange_t);
            translate([0, 0, flange_t]) {
                cylinder(d=spigot_d, h=spigot_len - tip_chamfer);
                translate([0, 0, spigot_len - tip_chamfer])
                    cylinder(d1=spigot_d, d2=spigot_d - 2*tip_chamfer, h=tip_chamfer);
            }
        }
        translate([0, 0, -1]) cylinder(d=m4_clear, h=flange_t + spigot_len + 2);
    }
}

wheel_retainer_cap();
