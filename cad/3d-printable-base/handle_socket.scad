include <common.scad>

// Socket block for a telescopic luggage/trolley handle rail.
// Print two sockets per handle rail pair and bolt them to the inner chassis.

socket_len = 78;
socket_w = 44;
socket_h = 54;
rail_w = 22;
rail_t = 14;
mount_hole_d = 5.2;

module handle_socket() {
    difference() {
        union() {
            rounded_box([socket_len, socket_w, socket_h], 7);
            translate([-12, -10, 0]) rounded_plate([socket_len + 24, socket_w + 20, 8], 9);
        }

        // rectangular rail pocket
        translate([socket_len / 2 - rail_w / 2, socket_w / 2 - rail_t / 2, 12])
            cube([rail_w, rail_t, socket_h]);

        // relief slot for clamp flex
        translate([socket_len / 2 - 2, socket_w / 2 + rail_t / 2 - 1, 8])
            cube([4, socket_w / 2, socket_h]);

        // cross clamp screw
        translate([socket_len / 2, socket_w - 8, socket_h / 2])
            rotate([90, 0, 0])
                cylinder(h=socket_w + 4, d=4.3);

        // base holes
        for (x = [14, socket_len - 14]) {
            for (y = [8, socket_w + 2]) {
                translate([x, y, -1]) screw_hole(mount_hole_d, 18);
            }
        }
    }
}

handle_socket();

