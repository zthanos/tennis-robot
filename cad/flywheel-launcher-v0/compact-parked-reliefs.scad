// Authoritative compact PARKED local-relief geometry.
//
// Approved 2026-08-25 from compact-parked-geometry-relief-study.md. These
// modules preserve all functional datums and remove only measured physical
// intersections, with 2 mm radial/axial clearance. Units: mm, compact local
// frame before functional_shift_x is applied.

use <../basket-bin-v2/bin.scad>
use <../basket-bin-v2/hood.scad>
use <../collector-intake-v1/option-a/option-a.scad>
use <launcher-envelope.scad>
include <params.scad>

compact_relief_clearance = 2;
compact_functional_shift_x = -100;
compact_intake_wheel_x = 470;
compact_intake_wheel_y = 90;
compact_intake_wheel_z = 70;
compact_intake_wheel_tilt = 35;
compact_intake_wheel_pocket_d = 128; // 124 mm tyre + 2 mm radial clearance
compact_intake_wheel_pocket_w = 77; // 73 mm tyre + 2 mm each axial side
compact_launcher_origin = [560, 0, 0];

// Minimum centreline lane at the protected x=470 entry plane. The 70 mm
// opening is one 66 mm ball diameter plus the approved 2 mm/side clearance.
// The chassis-fixed hood retains the external entry datum; only the local
// moving-chute contact surface is interrupted.
compact_chute_leadin_half_width = 35;
compact_chute_front_x = 470;
compact_chute_front_top_z = 40;
compact_chute_frame_d = 6;

module compact_axis_expanded(clearance=compact_relief_clearance) {
    children();
    if (clearance > 0)
        for (offset = [[clearance,0,0], [-clearance,0,0],
                       [0,clearance,0], [0,-clearance,0],
                       [0,0,clearance], [0,0,-clearance]])
            translate(offset) children();
}

module compact_intake_wheel_clearance_pockets() {
    for (sy = [-1, 1])
        translate([compact_intake_wheel_x,
                   sy * compact_intake_wheel_y,
                   compact_intake_wheel_z])
            rotate([0, compact_intake_wheel_tilt, 0])
                cylinder(d=compact_intake_wheel_pocket_d,
                         h=compact_intake_wheel_pocket_w, center=true);
}

module compact_launcher_plate_clearance() {
    minkowski() {
        translate(compact_launcher_origin)
            translate([0, 0, 215])
                rotate([0, -pitch_deg, 0])
                    rotate([90, 0, 0])
                        translate([0, 0, -path_z]) {
                            side_plate(-1);
                            side_plate(1);
                        }
        sphere(r=compact_relief_clearance, $fn=12);
    }
}

module compact_chassis_plate_clearance(
        functional_shift_x=compact_functional_shift_x) {
    // Express the world-fixed plate in compact-local coordinates, then add the
    // approved 2 mm relief. Only bin_walls() is cut by this envelope.
    translate([-functional_shift_x, 0, 0])
        minkowski() {
            chassis_plate_option_a();
            sphere(r=compact_relief_clearance, $fn=12);
        }
}

module compact_bin_without_walls() {
    bin_floor();
    bin_management_tray();
    compact_repaired_receiving_chute();
    bin_flange();
    bin_handles();
    bin_front_retention();
}

module compact_repaired_receiving_chute() {
    difference() {
        bin_receiving_chute();
        // Fabrication-realistic centre lane through the 50 mm receiving tile.
        // Trimming the front rod alone leaves the longitudinal-wire end faces
        // as a second blocking step. Oversize X/Z only to avoid remnants.
        translate([compact_chute_front_x - 25, 0,
                   compact_chute_front_top_z - compact_chute_frame_d / 2])
            cube([50 + compact_chute_frame_d + 4,
                  2 * compact_chute_leadin_half_width,
                  2 * compact_chute_frame_d + 8], center=true);
    }
}

// Diagnostic baseline used by the handoff A/B export.
module compact_prerepair_receiving_chute() {
    difference() {
        bin_receiving_chute();
        compact_intake_wheel_clearance_pockets();
    }
}

module compact_prerepair_relieved_bin(
        functional_shift_x=compact_functional_shift_x) {
    difference() {
        union() {
            bin_floor();
            bin_management_tray();
            bin_receiving_chute();
            bin_flange();
            bin_handles();
            bin_front_retention();
        }
        compact_intake_wheel_clearance_pockets();
    }
    compact_relieved_bin_walls(functional_shift_x);
}

module compact_relieved_bin_nonwalls() {
    difference() {
        compact_bin_without_walls();
        compact_intake_wheel_clearance_pockets();
    }
}

module compact_relieved_bin_walls(
        functional_shift_x=compact_functional_shift_x) {
    difference() {
        bin_walls();
        compact_intake_wheel_clearance_pockets();
        compact_chassis_plate_clearance(functional_shift_x);
    }
}

module compact_relieved_bin(
        functional_shift_x=compact_functional_shift_x) {
    compact_relieved_bin_nonwalls();
    compact_relieved_bin_walls(functional_shift_x);
}

module compact_rerouted_hood_supports() {
    support_x = 430;
    post_y = 184;
    post_size = 8;
    chassis_top_z = 52;
    crossbar_t = 6;
    crossbar_bottom_z = 142;
    crossbar_top_z = crossbar_bottom_z + crossbar_t;
    roof_attach_z = 120 + 6 + (support_x - 380) * tan(atan2(15, 90));

    // Chassis side-strip posts, clear of basket flange and bridge uprights.
    for (sy = [-1, 1])
        translate([support_x, sy * post_y,
                   (chassis_top_z + crossbar_top_z) / 2])
            cube([post_size, post_size,
                  crossbar_top_z - chassis_top_z], center=true);

    // 142..148 mm portal: above the expanded tyres, 2 mm below bridge.
    translate([support_x, 0,
               (crossbar_bottom_z + crossbar_top_z) / 2])
        cube([post_size, 2 * post_y + post_size, crossbar_t], center=true);

    // Twin central hangers remain inside the expanded inter-wheel gap.
    for (sy = [-1, 1])
        translate([support_x, sy * 35,
                   (roof_attach_z + crossbar_bottom_z) / 2])
            cube([post_size, post_size,
                  crossbar_bottom_z - roof_attach_z], center=true);
}

module compact_relieved_hood_shell() {
    difference() {
        union() { hood_roof(); hood_cheeks(); }
        compact_intake_wheel_clearance_pockets();
        compact_launcher_plate_clearance();
    }
}

// The entry hood is chassis-mounted.  Keep its load-bearing portal out of the
// moving basket solid so a later basket-path study cannot accidentally carry
// the chassis supports with the bin.
module compact_fixed_hood() {
    compact_relieved_hood_shell();
    compact_rerouted_hood_supports();
}

// Backward-compatible name for current views that display the complete fixed
// hood assembly.
module compact_relieved_hood() compact_fixed_hood();

// Wrap the existing compact ramp source. The basket receiving chute remains
// the handoff surface where the two former solids overlapped.
module compact_parked_ramp_relief() {
    difference() {
        children();
        // Subtract the actual repaired PARKED solids, not the obsolete full
        // chute. This restores the ramp contact skin only inside the new lane.
        compact_axis_expanded() {
            compact_relieved_bin();
            compact_fixed_hood();
        }
        // A restored skin must never fill either approved tyre pocket.
        compact_intake_wheel_clearance_pockets();
    }
}
