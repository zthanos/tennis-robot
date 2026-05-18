include <common.scad>

// Printable modular chassis tile.
// Default size fits common 220 x 220 mm printers.

tile_x = 190;
tile_y = 190;
plate_t = 7;
corner_r = 12;
grid_pitch = 40;
mount_hole_d = 5.2; // M5 clearance, or M4 with washer

module base_tile() {
    difference() {
        union() {
            rounded_plate([tile_x, tile_y, plate_t], corner_r);

            // underside ribs
            for (y = [35, tile_y / 2, tile_y - 35]) {
                translate([25, y, -16]) cube([tile_x - 50, 6, 16]);
            }
            for (x = [35, tile_x / 2, tile_x - 35]) {
                translate([x, 25, -16]) cube([6, tile_y - 50, 16]);
            }

            // printed alignment bosses for neighboring tiles
            for (x = [20, tile_x - 20]) {
                translate([x, -5, 0]) cylinder(h=plate_t + 3, d=12);
                translate([x, tile_y + 5, 0]) cylinder(h=plate_t + 3, d=12);
            }
        }

        // grid holes
        for (x = [35:grid_pitch:tile_x - 30]) {
            for (y = [35:grid_pitch:tile_y - 30]) {
                translate([x, y, 0]) screw_hole(mount_hole_d, plate_t + 20);
            }
        }

        // lightening window in center
        translate([tile_x / 2 - 42, tile_y / 2 - 42, -1])
            rounded_plate([84, 84, plate_t + 2], 10);
    }
}

base_tile();

