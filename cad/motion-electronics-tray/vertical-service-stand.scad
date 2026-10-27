// Vertical bench/service stand for the existing 180 x 240 mm electronics tray.
// Print as one part with the base plate on the bed. All dimensions are in mm.

include <params.scad>

$fn = 48;

show_reference_tray = false;

stand_size = [210, 130];
base_t = 6;
base_corner_r = 7;

tray_x = (stand_size[0] - tray_size[0]) / 2;
tray_back_y = 62;       // rear face of the 4 mm tray
tray_bottom_z = 20;     // raises the tray above the base and exposes its edge
tray_clearance = 0.8;

upright_w = 24;
upright_t = 8;
upright_h = 88;
rear_brace_depth = 52;

mount_x = [tray_x + chassis_slots[0][0],
           tray_x + chassis_slots[1][0]];
mount_z = tray_bottom_z + chassis_slots[0][1];
mount_slot_h = 10;
mount_hole_d = 5.8;

shelf_w = 34;
shelf_depth = 18;

module rounded_rect_2d(size, r) {
    hull()
        for (x = [r, size[0] - r], y = [r, size[1] - r])
            translate([x, y]) circle(r = r);
}

module capsule_2d(length, width, horizontal = true) {
    hull() {
        if (horizontal) {
            translate([-(length - width) / 2, 0]) circle(d = width);
            translate([ (length - width) / 2, 0]) circle(d = width);
        } else {
            translate([0, -(length - width) / 2]) circle(d = width);
            translate([0,  (length - width) / 2]) circle(d = width);
        }
    }
}

module base_slot(pos, length = 14, width = 4.5, horizontal = true) {
    translate([pos[0], pos[1], -1])
        linear_extrude(height = base_t + 2)
            capsule_2d(length, width, horizontal);
}

module vertical_mount_slot(x) {
    // Horizontal cylinder axis through the upright, hulled vertically to give
    // +/- 2 mm tolerance for the measured tray-slot height.
    hull()
        for (zoff = [-2, 2])
            translate([x, tray_back_y + upright_t + 1, mount_z + zoff])
                rotate([90, 0, 0])
                    cylinder(d = mount_hole_d, h = upright_t + 3);
}

module upright_with_brace(x) {
    // Rear wall supports the tray; the hull forms a broad, support-free brace.
    hull() {
        translate([x - upright_w / 2, tray_back_y, base_t])
            cube([upright_w, upright_t, upright_h - base_t]);
        translate([x - upright_w / 2, tray_back_y + rear_brace_depth,
                   base_t])
            cube([upright_w, 4, 2]);
    }

    // The tray bottom rests on this ledge instead of hanging from the M5 bolts.
    translate([x - shelf_w / 2, tray_back_y - shelf_depth, base_t])
        cube([shelf_w, shelf_depth + upright_t, tray_bottom_z - base_t]);
}

module vertical_service_stand() {
    difference() {
        union() {
            linear_extrude(height = base_t)
                rounded_rect_2d(stand_size, base_corner_r);

            for (x = mount_x) upright_with_brace(x);

            // Low front rail keeps a terminal/WAGO carrier from sliding.
            translate([55, 5, base_t]) cube([100, 3, 5]);
        }

        for (x = mount_x) vertical_mount_slot(x);

        // Four front slots accept cable ties around a temporary distribution
        // block; the two rear slots provide strain relief for supply leads.
        for (p = [[68, 22], [142, 22], [68, 43], [142, 43]])
            base_slot(p, 16, 4.5, false);
        for (p = [[35, 105], [175, 105]])
            base_slot(p, 18, 5.5, true);

        // Optional holes for clamping/screwing the stand to a sacrificial bench.
        for (p = [[12, 12], [198, 12], [12, 118], [198, 118]])
            translate([p[0], p[1], -1]) cylinder(d = 5.5, h = base_t + 2);
    }
}

vertical_service_stand();

if (show_reference_tray) {
    // Original tray local Y becomes vertical Z; its component side faces front.
    color("SeaGreen", 0.35)
        translate([tray_x, tray_back_y, tray_bottom_z])
            rotate([90, 0, 0])
                cube([tray_size[0], tray_size[1], tray_t]);
}

