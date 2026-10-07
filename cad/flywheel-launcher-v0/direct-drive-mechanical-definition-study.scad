// Non-authoritative analysis CAD for the D5065 direct-panel architecture.
// Units: mm.  This file deliberately does not define the missing wheel-side
// hub, axial retention, final service cut-out, or manufacturing fasteners.

$fn = 96;
view_mode = "section"; // [top,side,front,section,iso]

wheel_d = 200;
wheel_width = 50;
wheel_y = 129;
panel_x = 256;
panel_y = 314;
panel_t = 8;
panel_z = 43;

motor_d = 50;
motor_length = 65;
motor_face_z = 47;
shaft_d = 8;
shaft_projection = 30;
mount_pcd = 30;
mount_hole_d = 4;

// Sensitivity display only.  The gate does NOT release this as a cut-out.
analysis_cutout_d = 18;
mount_clocking_deg = 0; // exact wire/connector clocking remains unmeasured

module panel(z_pos, upper=false) {
    color([0.35, 0.40, 0.46, 0.78])
        difference() {
            translate([0, 0, z_pos]) cube([panel_x, panel_y, panel_t], center=true);
            if (upper)
                for (yy = [-wheel_y, wheel_y]) {
                    // The magenta 18 mm opening is a ligament sensitivity case.
                    translate([0, yy, z_pos]) cylinder(d=analysis_cutout_d, h=panel_t + 2, center=true);
                    for (angle = [0:90:270])
                        translate([
                            mount_pcd/2 * cos(angle + mount_clocking_deg),
                            yy + mount_pcd/2 * sin(angle + mount_clocking_deg),
                            z_pos
                        ]) cylinder(d=mount_hole_d, h=panel_t + 2, center=true);
                }
        }
}

module wheel(yy) {
    color("darkorange") translate([0, yy, 0]) cylinder(d=wheel_d, h=wheel_width, center=true);
}

module motor_and_shaft(yy) {
    color([0.18, 0.22, 0.27])
        translate([0, yy, motor_face_z + motor_length/2])
            cylinder(d=motor_d, h=motor_length, center=true);
    color("silver")
        translate([0, yy, motor_face_z - shaft_projection/2])
            cylinder(d=shaft_d, h=shaft_projection, center=true);
    color([0.20, 0.65, 0.85, 0.70])
        for (angle = [0:90:270])
            translate([
                mount_pcd/2 * cos(angle + mount_clocking_deg),
                yy + mount_pcd/2 * sin(angle + mount_clocking_deg),
                motor_face_z
            ]) cylinder(d=mount_hole_d, h=12, center=true);
}

module unresolved_hub_zone(yy) {
    // Only the known axial gap / shaft reach is shown. Diameter and interfaces
    // are intentionally not modeled as a candidate manufactured part.
    color([0.85, 0.05, 0.55, 0.28])
        translate([0, yy, (39 + 17)/2])
            difference() {
                cylinder(d=analysis_cutout_d, h=22, center=true);
                cylinder(d=shaft_d + 0.5, h=24, center=true);
            }
}

module labels() {
    color("black")
        translate([1, -28, 81]) rotate([0, 90, 0])
            linear_extrude(0.8) text("D5065 OUTSIDE", size=7, halign="center");
    color([0.75, 0, 0.45])
        translate([1, -45, 30]) rotate([0, 90, 0])
            linear_extrude(0.8) text("HUB + RETENTION MISSING", size=6, halign="center");
    color("black")
        translate([1, -50, -8]) rotate([0, 90, 0])
            linear_extrude(0.8) text("FLYWHEEL INSIDE", size=7, halign="center");
}

module full_analysis_assembly() {
    panel(-panel_z, false);
    panel(panel_z, true);
    for (yy = [-wheel_y, wheel_y]) {
        wheel(yy);
        motor_and_shaft(yy);
        unresolved_hub_zone(yy);
    }
}

if (view_mode == "section") {
    // Representative single-axis axial elevation.  With all bodies centred on
    // x=0 it exposes the same Y-Z stack as a centre-plane section while
    // retaining evidence/status colours in a deterministic render.
    panel(-panel_z, false);
    panel(panel_z, true);
    wheel(0);
    motor_and_shaft(0);
    unresolved_hub_zone(0);
} else {
    full_analysis_assembly();
}
