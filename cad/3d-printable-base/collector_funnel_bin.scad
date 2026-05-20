include <common.scad>

// Concept A collector prototype: front funnel, lift-wheel bracket, back plate,
// and small receiving bin. Dimensions are aligned with the current Webots MVP
// geometry but kept adjustable for bench testing.

ball_d = 67;
wall = 4;
plate_t = 5;
mount_hole_d = 5.2;

funnel_len = 360;
funnel_mouth_w = 300;
throat_w = 82;
funnel_wall_h = 120;
lip_h = 12;

lift_wheel_d = 90;
lift_wheel_w = 180;
lift_wheel_center_x = 210;
lift_wheel_center_z = 105;
wheel_gap_to_back_plate = 62;

bin_len = 260;
bin_w = 300;
bin_h = 190;
bin_floor_tilt_deg = 8;

module mounting_slot(len=24, d=mount_hole_d, h=plate_t + 4) {
    hull() {
        translate([-len / 2, 0, -1]) cylinder(h=h, d=d);
        translate([len / 2, 0, -1]) cylinder(h=h, d=d);
    }
}

module funnel_side(sign=1) {
    angle = atan((funnel_mouth_w - throat_w) / 2 / funnel_len);
    translate([funnel_len / 2, sign * (throat_w / 2 + wall / 2), funnel_wall_h / 2])
        rotate([0, 0, sign * angle])
            cube([funnel_len, wall, funnel_wall_h], center=true);
}

module low_intake_lip() {
    translate([funnel_len / 2, 0, lip_h / 2])
        cube([funnel_len, throat_w + 2 * wall, lip_h], center=true);
}

module back_plate() {
    translate([lift_wheel_center_x - wheel_gap_to_back_plate, 0, lift_wheel_center_z])
        rotate([0, 12, 0])
            cube([plate_t, throat_w + 38, 160], center=true);
}

module lift_wheel_placeholder() {
    color([0.03, 0.03, 0.03])
        translate([lift_wheel_center_x, 0, lift_wheel_center_z])
            rotate([90, 0, 0])
                cylinder(h=lift_wheel_w, d=lift_wheel_d, center=true);
}

module lift_wheel_bracket() {
    difference() {
        union() {
            for (side = [-1, 1]) {
                translate([lift_wheel_center_x, side * (lift_wheel_w / 2 + 12), lift_wheel_center_z])
                    rounded_plate([85, 8, 120], 4);
            }
            translate([lift_wheel_center_x - 42, -lift_wheel_w / 2 - 16, 45])
                cube([84, lift_wheel_w + 32, 10]);
        }

        for (side = [-1, 1]) {
            translate([lift_wheel_center_x, side * (lift_wheel_w / 2 + 18), lift_wheel_center_z])
                rotate([90, 0, 0])
                    cylinder(h=22, d=12, center=true);
        }
    }
}

module receiving_bin() {
    translate([-bin_len / 2 + 70, 0, 150])
        rotate([0, -bin_floor_tilt_deg, 0])
            difference() {
                union() {
                    cube([bin_len, bin_w, wall], center=true);
                    translate([0, -bin_w / 2 + wall / 2, bin_h / 2])
                        cube([bin_len, wall, bin_h], center=true);
                    translate([0, bin_w / 2 - wall / 2, bin_h / 2])
                        cube([bin_len, wall, bin_h], center=true);
                    translate([-bin_len / 2 + wall / 2, 0, bin_h / 2])
                        cube([wall, bin_w, bin_h], center=true);
                    translate([bin_len / 2 - wall / 2, 0, 45])
                        cube([wall, bin_w, 90], center=true);
                }

                for (x = [-80, 0, 80]) {
                    for (y = [-105, 105]) {
                        translate([x, y, -4]) mounting_slot();
                    }
                }
            }
}

module collector_mount_plate() {
    difference() {
        rounded_plate([funnel_len + 70, funnel_mouth_w + 60, plate_t], 10);
        for (x = [45, 135, 225, 315]) {
            for (y = [-125, 125]) {
                translate([x, y + (funnel_mouth_w + 60) / 2, -1])
                    mounting_slot();
            }
        }
    }
}

module collector_funnel_bin(show_wheel=true) {
    collector_mount_plate();
    translate([0, funnel_mouth_w / 2 + 30, plate_t]) {
        funnel_side(1);
        funnel_side(-1);
        low_intake_lip();
        back_plate();
        lift_wheel_bracket();
        receiving_bin();
        if (show_wheel) {
            lift_wheel_placeholder();
        }
    }
}

collector_funnel_bin();
