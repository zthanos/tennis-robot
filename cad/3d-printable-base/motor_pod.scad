include <common.scad>

// Side motor pod for direct-drive wheel mounting.
// The wheel sits outside the pod and attaches directly to the motor shaft/hub.
// No separate metal wheel axle is used.

pod_len = 150;
pod_w = 76;
pod_h = 96;
wall = 8;
motor_body_d = 50;     // adjust for chosen gear motor
shaft_center_z = 52;
shaft_clearance_d = 24;
mount_hole_d = 5.2;

module motor_face_holes() {
    // generic square mount pattern around shaft
    for (x = [-22, 22]) {
        for (z = [-22, 22]) {
            translate([x, 0, z]) rotate([90, 0, 0]) screw_hole(4.3, 30);
        }
    }
}

module motor_pod() {
    difference() {
        union() {
            rounded_box([pod_len, pod_w, pod_h], 8);

            // base flange
            translate([-12, -12, 0]) rounded_plate([pod_len + 24, pod_w + 24, 10], 10);

            // triangular reinforcement webs
            translate([18, pod_w + 5, 10]) rib(95, 52, 7);
            translate([pod_len - 18, pod_w + 12, 10]) mirror([1, 0, 0]) rib(95, 52, 7);
        }

        // hollow motor pocket
        translate([pod_len / 2, pod_w / 2, shaft_center_z])
            rotate([90, 0, 0])
                cylinder(h=pod_w + 4, d=motor_body_d);

        // shaft exit
        translate([pod_len / 2, -16, shaft_center_z])
            rotate([90, 0, 0])
                cylinder(h=40, d=shaft_clearance_d);

        // front motor face bolt pattern
        translate([pod_len / 2, -1, shaft_center_z]) motor_face_holes();

        // chassis flange holes
        for (x = [18, pod_len / 2, pod_len - 18]) {
            for (y = [10, pod_w + 2]) {
                translate([x, y, 0]) screw_hole(mount_hole_d, 24);
            }
        }

        // cable exit
        translate([pod_len - 22, pod_w - 10, 30]) cube([18, 28, 24]);
    }
}

motor_pod();

