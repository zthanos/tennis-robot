include <common.scad>

// Passive front caster/stabilizer wheel mount for the first mobile prototype.
// Use two of these at the front corners with off-the-shelf swivel casters.
// The printed part is the bolt-on bracket; the caster wheel itself is a bought part.

plate_x = 92;
plate_y = 78;
plate_t = 8;
corner_r = 8;

caster_plate_x = 60;
caster_plate_y = 48;
caster_hole_x = 42;
caster_hole_y = 32;
caster_hole_d = 5.2;

wood_mount_hole_d = 5.2;
wood_mount_slot_len = 24;

stand_off_h = 18;
stand_off_d = 16;

module slot_hole(len=wood_mount_slot_len, d=wood_mount_hole_d, h=plate_t + 4) {
    hull() {
        translate([-len / 2, 0, -1]) cylinder(h=h, d=d);
        translate([len / 2, 0, -1]) cylinder(h=h, d=d);
    }
}

module front_caster_mount() {
    difference() {
        union() {
            rounded_plate([plate_x, plate_y, plate_t], corner_r);

            // Raised bosses keep the caster plate from crushing the printed base.
            for (x = [-caster_hole_x / 2, caster_hole_x / 2]) {
                for (y = [-caster_hole_y / 2, caster_hole_y / 2]) {
                    translate([plate_x / 2 + x, plate_y / 2 + y, plate_t])
                        cylinder(h=stand_off_h, d=stand_off_d);
                }
            }

            // Low front skid lip in case the caster momentarily unloads.
            translate([plate_x / 2, 4, plate_t + 5])
                cube([plate_x - 18, 8, 10], center=true);
        }

        // Caster plate bolt pattern.
        for (x = [-caster_hole_x / 2, caster_hole_x / 2]) {
            for (y = [-caster_hole_y / 2, caster_hole_y / 2]) {
                translate([plate_x / 2 + x, plate_y / 2 + y, -1])
                    screw_hole(caster_hole_d, plate_t + stand_off_h + 4);
            }
        }

        // Slots for bolting this bracket to a wooden or printed chassis.
        for (x = [22, plate_x - 22]) {
            translate([x, 12, -1]) slot_hole();
            translate([x, plate_y - 12, -1]) rotate([0, 0, 90]) slot_hole();
        }

        // Center relief for caster swivel nut/head clearance.
        translate([plate_x / 2, plate_y / 2, plate_t + 6])
            cylinder(h=stand_off_h + 6, d=28);
    }
}

// Reference footprint for a common small swivel caster plate.
module caster_plate_reference() {
    color([0.1, 0.1, 0.1, 0.35])
        translate([plate_x / 2, plate_y / 2, plate_t + stand_off_h + 1])
            cube([caster_plate_x, caster_plate_y, 2], center=true);
}

front_caster_mount();
// caster_plate_reference();
