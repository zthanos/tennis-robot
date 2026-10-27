// PROVISIONAL_HUB_FOR_SIMULATION -- analysis envelope, not manufacturing CAD.
// Units: mm; launcher-local shaft axis is Z.
$fn = 128;
view_mode = "iso"; // [iso,axial,top]

wheel_d = 200;
wheel_w = 50;
wheel_y = 129;
panel_x = 256;
panel_y = 314;
panel_t = 8;
panel_z = 43;
cutout_d = 12;

module upper_panel() {
    color([0.35, 0.40, 0.46, 1.0])
        difference() {
            translate([0, 0, panel_z]) cube([panel_x, panel_y, panel_t], center=true);
            for (y = [-wheel_y, wheel_y])
                translate([0, y, panel_z]) cylinder(d=cutout_d, h=panel_t + 2, center=true);
        }
}

module lower_panel() {
    color([0.35, 0.40, 0.46, 1.0])
        translate([0, 0, -panel_z]) cube([panel_x, panel_y, panel_t], center=true);
}

module provisional_hub(y) {
    // Split D-clamp collar, 10 mm wheel pilot, distal stem, washer and nut.
    color([0.12, 0.48, 0.78]) {
        translate([0, y, 31.75]) difference() {
            cylinder(d=22, h=13.5, center=true);
            cylinder(d=8, h=15.5, center=true);
        }
        translate([0, y, 21]) difference() {
            cylinder(d=10, h=8, center=true);
            cylinder(d=8, h=10, center=true);
        }
        translate([0, y, -4]) cylinder(d=10, h=42, center=true);
        translate([0, y, -29]) cylinder(d=8, h=8, center=true);
        translate([0, y, -26]) difference() {
            cylinder(d=22, h=2, center=true);
            cylinder(d=8.4, h=4, center=true);
        }
        translate([0, y, -30]) difference() {
            cylinder(d=13, h=6, center=true, $fn=6);
            cylinder(d=8, h=8, center=true);
        }
    }
    // Dog-point screw registered to the D-flat; analysis marker only.
    color("silver") translate([7, y, 33]) rotate([0, 90, 0]) cylinder(d=3, h=10, center=true);
}

module motor_shaft_wheel(y) {
    color([0.18, 0.19, 0.22]) translate([0, y, 79.5]) cylinder(d=50, h=65, center=true);
    color("silver") translate([0, y, 32]) cylinder(d=8, h=30, center=true);
    provisional_hub(y);
    color([0.95, 0.40, 0.04]) translate([0, y, 0]) cylinder(d=wheel_d, h=wheel_w, center=true);
}

module assembly(single=false) {
    upper_panel();
    lower_panel();
    if (single)
        motor_shaft_wheel(0);
    else
        for (y = [-wheel_y, wheel_y]) motor_shaft_wheel(y);
}

if (view_mode == "axial")
    assembly(true);
else
    assembly(false);
