// Printable funnel cheeks, launch ramp, and chassis rail mounts.
// Geometry follows the active URDF in millimetres. The aluminium rails shown
// in the assembly are reference hardware and are not exported as STL.

include <params.scad>

part = "assembly";
explode = 0;

module slot_x(length, diameter, height) {
    hull() {
        translate([-length / 2, 0, 0]) cylinder(d=diameter, h=height);
        translate([ length / 2, 0, 0]) cylinder(d=diameter, h=height);
    }
}

// A cheek is split at its midpoint so every section fits a 220 x 220 bed.
// Print coordinates: X follows the cheek, Y is panel height, Z is thickness.
module cheek_segment(which="rear") {
    segment_l = cheek_length / 2;
    seam_x = which == "rear" ? segment_l - 15 : 15;
    support_x = which == "rear" ? 38 : segment_l - 38;
    difference() {
        cube([segment_l, cheek_height, cheek_print_thickness]);
        // Two M4 seam holes per half.
        for (yy = [25, 75])
            translate([seam_x, yy, -0.1])
                cylinder(d=m4_clearance_d, h=cheek_print_thickness + 0.2);
        // Two mounting points for the outer support bracket.
        for (yy = [18, 82])
            translate([support_x, yy, -0.1])
                slot_x(12, m5_clearance_d, cheek_print_thickness + 0.2);
    }
}

module cheek_joiner() {
    joiner_size = [50, 76, 4];
    difference() {
        cube(joiner_size);
        for (xx = [10, 40], yy = [13, 63])
            translate([xx, yy, -0.1])
                cylinder(d=m4_clearance_d, h=joiner_size[2] + 0.2);
    }
}

function _ramp_t(x) = (ramp_entry_x - x) / (ramp_entry_x - ramp_exit_x);
function _h00(t) = 2*t*t*t - 3*t*t + 1;
function _h01(t) = -2*t*t*t + 3*t*t;
function _h11(t) = t*t*t - t*t;
function ramp_top_z(x) =
    let(t = max(0, min(1, _ramp_t(x))),
        run = ramp_entry_x - ramp_exit_x,
        exit_tangent = tan(ramp_exit_angle_deg) * run)
    _h00(t) * ramp_entry_top_z
        + _h01(t) * ramp_exit_top_z
        + _h11(t) * exit_tangent;

module _ramp_station(x, half_width, extra_h=0) {
    station_t = 0.7;
    z = max(0.8, ramp_top_z(x) + extra_h);
    translate([x - station_t / 2, -half_width, 0])
        cube([station_t, 2 * half_width, z]);
}

module _ramp_solid() {
    union() {
        // Slicer infill hollows this wedge while preserving the validated top.
        for (i = [0 : ramp_steps - 1]) {
            x0 = ramp_entry_x + (ramp_exit_x-ramp_entry_x) * i/ramp_steps;
            x1 = ramp_entry_x + (ramp_exit_x-ramp_entry_x) * (i+1)/ramp_steps;
            hull() {
                _ramp_station(x0, ramp_width/2);
                _ramp_station(x1, ramp_width/2);
            }
        }
        // External walls: the clear 180 mm ball corridor is unchanged.
        for (sy = [-1, 1], i = [0 : ramp_steps - 1]) {
            x0 = ramp_entry_x + (ramp_exit_x-ramp_entry_x) * i/ramp_steps;
            x1 = ramp_entry_x + (ramp_exit_x-ramp_entry_x) * (i+1)/ramp_steps;
            hull() {
                translate([0, sy * (ramp_width/2 + ramp_side_wall_t/2), 0])
                    _ramp_station(x0, ramp_side_wall_t/2, ramp_side_wall_h);
                translate([0, sy * (ramp_width/2 + ramp_side_wall_t/2), 0])
                    _ramp_station(x1, ramp_side_wall_t/2, ramp_side_wall_h);
            }
        }
    }
}

module ramp() {
    // Export in compact print coordinates: rear/exit is X=0.
    translate([-ramp_exit_x, 0, 0])
        difference() {
            _ramp_solid();
            // Two transverse M4 mounting holes through each external wall.
            for (xx = [480, 520])
                translate([xx, 0, ramp_top_z(xx) + 10])
                    rotate([90, 0, 0])
                        cylinder(d=m4_clearance_d,
                                 h=ramp_width + 2*ramp_side_wall_t + 4,
                                 center=true);
        }
}

module rail_saddle() {
    base = [100, 56, 6];
    side_t = 6;
    side_h = support_rail_size + 6;
    difference() {
        union() {
            translate([-base[0]/2, -base[1]/2, 0]) cube(base);
            for (sy = [-1, 1])
                translate([-40,
                           sy*(support_rail_size/2 + side_t/2) - side_t/2,
                           base[2]])
                    cube([80, side_t, side_h]);
        }
        // Four M5 chassis slots absorb ruler and drilling error.
        for (xx = [-30, 30], yy = [-23, 23])
            translate([xx, yy, -0.1])
                slot_x(14, m5_clearance_d, base[2] + 0.2);
        // M4 cap bolts; use nuts or heat-set inserts in the saddle walls.
        for (xx = [-30, 30], yy = [-13, 13])
            translate([xx, yy, base[2] - 0.1])
                cylinder(d=m4_clearance_d, h=side_h + 0.2);
    }
}

module rail_cap() {
    cap = [80, 38, 5];
    difference() {
        translate([-cap[0]/2, -cap[1]/2, 0]) cube(cap);
        for (xx = [-30, 30], yy = [-13, 13])
            translate([xx, yy, -0.1])
                cylinder(d=m4_clearance_d, h=cap[2] + 0.2);
    }
}

module cheek_world(side=1) {
    // Rebuild both printable halves in the URDF local panel frame.
    translate([cheek_origin_x, side*cheek_origin_y, cheek_origin_ground_z])
        rotate([0, cheek_pitch_deg, side*cheek_yaw_deg])
            rotate([90, 0, 0]) {
                translate([-cheek_length/2, -cheek_height/2,
                           -cheek_print_thickness/2])
                    cheek_segment("rear");
                translate([0, -cheek_height/2,
                           -cheek_print_thickness/2])
                    cheek_segment("front");
            }
}

module assembly() {
    // Chassis reference.
    color("burlywood", 0.65)
        translate([-460, -290, chassis_plate_top_z-chassis_plate_thickness])
            difference() {
                cube([920, 580, chassis_plate_thickness]);
                translate([470, 140, -1]) cube([450, 300, 16]);
            }

    // Purchased 20 mm square rails and printed chassis clamps.
    for (sy = [-1, 1]) {
        color("silver")
            translate([support_rail_start_x, sy*support_rail_y-support_rail_size/2,
                       support_rail_bottom_z])
                cube([support_rail_end_x-support_rail_start_x,
                      support_rail_size, support_rail_size]);
        color("dimgray")
            translate([405, sy*support_rail_y, chassis_plate_top_z]) rail_saddle();
        color("gray")
            translate([405, sy*support_rail_y,
                       support_rail_bottom_z+support_rail_size]) rail_cap();
    }

    color("darkorange") translate([ramp_exit_x, 0, 0]) ramp();
    color("orange") cheek_world(1);
    color("orange") cheek_world(-1);

    // Wheel references only; wheel/carriage CAD follows the motor fit check.
    for (sy = [-1, 1])
        color("black", 0.55)
            translate([540, sy*(intake_gap/2+intake_wheel_d/2), 70])
                rotate([0, intake_axis_tilt_deg, 0])
                    cylinder(d=intake_wheel_d, h=intake_wheel_height,
                             center=true);
}

if (part == "assembly") assembly();
else if (part == "cheek_rear") cheek_segment("rear");
else if (part == "cheek_front") cheek_segment("front");
else if (part == "cheek_joiner") cheek_joiner();
else if (part == "ramp") ramp();
else if (part == "rail_saddle") rail_saddle();
else if (part == "rail_cap") rail_cap();
else assert(false, str("Unknown part: ", part));
