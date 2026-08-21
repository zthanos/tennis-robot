// Collector manufacturing concept — Option A.
//
// Key decisions:
//   * 340 mm chassis opening;
//   * intake wheels tucked approximately halfway into the front edge;
//   * mandatory 35 degree forward tilt retained from the working intake;
//   * 18 mm plywood portal bridge carries both fixed motor/shaft pods;
//   * commercial 124 x 73 mm RC wheels use printed 6 mm -> 12 mm hex hubs;
//   * curved cheeks soften first contact;
//   * cheek-to-base transition is intentionally NOT frozen yet.
//
// Units: mm, ground frame, robot +X points forward.

include <params.scad>

$fn = 64;
explode = 0;
show_balls = true;
show_ir_beams = true;
part = "assembly"; // see export selector at end of file

// ---- Option A layout ----
oa_opening_half_width = 170;
oa_wheel_x = 470;
oa_wheel_d = 124;
oa_wheel_width = 73;
// Override from the CLI with: -D 'oa_gap=58'. Supported trial values are
// 56, 58 and 60 mm; 56 mm remains the first-build setting.
oa_gap = 56;
oa_wheel_y = oa_gap / 2 + oa_wheel_d / 2; // +/-90 at 56 mm gap
oa_wheel_z = 70;
oa_wheel_tilt = 35;

// Fixed, bearing-supported transmission. The motor's measured 5 mm D-shaft
// ends at the coupler and does not carry wheel impact or side loads.
oa_transmission_shaft_d = 6;
oa_transmission_shaft_clearance_d = 6.2;
oa_transmission_shaft_flat = 0.8; // provisional until the 6 mm shaft is bought
oa_bearing_od = 19;               // provisional 626 bearing envelope
oa_bearing_width = 6;
oa_hex_af = 12;
oa_hex_depth = 6;
oa_hub_collar_d = 26;
oa_hub_collar_h = 14;
oa_hub_clamp_bolt_d = 4.5;
oa_hub_clamp_nut_af = 7.2;
oa_hub_clamp_nut_depth = 3.5;

// IR beam packaging references. Beam #1 sits in the clean gap after the
// cheek tips and before the tilted wheel envelope. The legacy basket beam at
// x=445/z=70.5 intersects the new wheels, so Beam #2 moves to the basket edge
// and below their rear/lower envelope. Final holes follow the real sensors.
oa_ir1_x = 560;
oa_ir1_z = 40;
oa_ir1_mount_y = 110;
oa_ir2_x = 420;
oa_ir2_z = 50;
oa_ir2_mount_y = 175;
oa_ir_sensor_size = [20, 16, 16];
oa_ir_drop_t = 6;
oa_ir_beam_d = 3;

oa_bridge_center_x = 490;
oa_bridge_depth = 220;
oa_bridge_width = 470;
oa_bridge_t = 18;
oa_bridge_under_z = 150;
oa_bridge_top_z = oa_bridge_under_z + oa_bridge_t;

oa_upright_x0 = 385;
oa_upright_x1 = 595;
oa_upright_t = 18;
oa_upright_y = 205;

// Curved cheek centreline, front/mouth -> rear/throat.
oa_cheek_p0 = [805, 205];
oa_cheek_p1 = [749, 205];
oa_cheek_p2 = [640, 83];
oa_cheek_p3 = [585, 83];
oa_cheek_t = 6;
oa_cheek_bottom_z = 18;
// The cheeks now meet the plywood bridge directly; no support rods.
oa_cheek_top_z = oa_bridge_under_z;
oa_cheek_steps = 24;
oa_cheek_flange_x0 = 555;
oa_cheek_flange_x1 = 615;
oa_cheek_flange_outer_y = 150;
oa_cheek_flange_t = 4;
oa_cheek_mount_hole_x = [565, 585];
oa_cheek_mount_hole_y = 132;

// Short handoff replaces the old long launch ramp in this packaging study.
// Recessed behind the nominal wheel leading edge (x=532 mm), ensuring that
// the compliant tires—not the hard ramp lip—make first contact with the ball.
oa_ramp_front_x = 520;
oa_ramp_rear_x = 420;
oa_ramp_front_z = 1.5;
oa_ramp_rear_z = 35;
oa_ramp_width = 180;
oa_ramp_wall_h = 18;

// Approximate motor envelope from ruler measurements.
oa_motor_d = motor_body_d_measured;
oa_motor_l = motor_length_measured;
oa_motor_shaft_d = shaft_d_measured;

function bezier3(p0, p1, p2, p3, t) =
    p0*pow(1-t, 3)
    + p1*(3*pow(1-t, 2)*t)
    + p2*(3*(1-t)*t*t)
    + p3*(t*t*t);

function oa_ramp_z(x) =
    let(t = max(0, min(1, (oa_ramp_front_x-x)/(oa_ramp_front_x-oa_ramp_rear_x))))
    oa_ramp_front_z + (oa_ramp_rear_z-oa_ramp_front_z)
        * (t*t*(3-2*t));

module chassis_plate_option_a() {
    color("burlywood")
        translate([0, 0, chassis_plate_top_z-chassis_plate_thickness/2])
            difference() {
                cube([920, 580, chassis_plate_thickness], center=true);
                translate([(10+chassis_front_x)/2, 0, 0])
                    cube([chassis_front_x-10,
                          2*oa_opening_half_width,
                          chassis_plate_thickness+2], center=true);
            }
}

module basket_reference() {
    color("seagreen", 0.23) {
        translate([(20+420)/2, 0, (25+250)/2])
            cube([400, 280, 225], center=true);
        // Receiving/management surface reference.
        hull() {
            translate([420, 0, 35]) cube([1, 180, 3], center=true);
            translate([380, 0, 31]) cube([1, 180, 3], center=true);
        }
    }
}

module plywood_bridge() {
    difference() {
        union() {
            // Horizontal motor bridge.
            translate([oa_bridge_center_x, 0,
                       oa_bridge_under_z+oa_bridge_t/2])
                cube([oa_bridge_depth, oa_bridge_width, oa_bridge_t],
                     center=true);
            // Two plywood uprights land on the intact chassis side strips.
            for (sy = [-1, 1])
                translate([(oa_upright_x0+oa_upright_x1)/2,
                           sy*oa_upright_y,
                           chassis_plate_top_z
                             +(oa_bridge_under_z-chassis_plate_top_z)/2])
                    cube([oa_upright_x1-oa_upright_x0,
                          oa_upright_t,
                          oa_bridge_under_z-chassis_plate_top_z],
                         center=true);
        }
        // Fixed service openings for the tilted shafts. Final hole shape is
        // drilled from the printed pod template after checking the bearings.
        axis_dx_top = (oa_bridge_top_z-oa_wheel_z)*tan(oa_wheel_tilt);
        slot_x = oa_wheel_x + axis_dx_top;
        for (sy = [-1, 1])
            translate([slot_x, sy*oa_wheel_y, oa_bridge_under_z-1]) {
                cube([22, 22, oa_bridge_t+2], center=true);
                // Four fixed M5 pod bolts; no lateral sliding carriage.
                for (dx = [-26, 26], dy = [-24, 24])
                    translate([dx, dy, 0])
                        cylinder(d=m5_clearance_d, h=oa_bridge_t+2,
                                 center=true);
            }
        // Matching through-holes for the two cheek top flanges. They sit
        // outboard of the moving motor/shaft service slots.
        for (sy = [-1, 1], xx = oa_cheek_mount_hole_x)
            translate([xx, sy*oa_cheek_mount_hole_y,
                       oa_bridge_under_z-1])
                cylinder(d=m5_clearance_d, h=oa_bridge_t+2);
    }
}

// D-shaped hole for the selected 6 mm transmission shaft. The flat remains
// provisional, and the clamp slit/M4 fastener provide secondary retention.
module oa_d_bore(d, flat_depth, h) {
    intersection() {
        cylinder(d=d, h=h);
        translate([-d, -d, 0])
            cube([d + d/2-flat_depth, 2*d, h]);
    }
}

// Printable split-clamp adapter: 6 mm D-bore to male 12 mm RC hex. Print the
// hex on the bed in PETG/PA, 5-6 perimeters. The hub is a replaceable part,
// not a bearing surface. Wheel axial retention is provided by an M4 bolt into
// the tapped end of the steel shaft, with a broad washer outside the wheel.
module printed_hex_hub() {
    hub_h = oa_hex_depth + oa_hub_collar_h;
    difference() {
        union() {
            cylinder(d=oa_hex_af/cos(30), h=oa_hex_depth, $fn=6);
            translate([0, 0, oa_hex_depth])
                cylinder(d=oa_hub_collar_d, h=oa_hub_collar_h);
        }
        translate([0, 0, -1])
            oa_d_bore(oa_transmission_shaft_clearance_d,
                      oa_transmission_shaft_flat, hub_h+2);

        // Radial slit permits real clamping instead of relying on a set screw.
        translate([-0.7, oa_transmission_shaft_clearance_d/2-0.2,
                   oa_hex_depth-1])
            cube([1.4, oa_hub_collar_d/2, oa_hub_collar_h+2]);

        // M4 bolt crosses the slit; the opposite side traps an M4 nut.
        translate([-oa_hub_collar_d/2-1, 8,
                   oa_hex_depth+oa_hub_collar_h/2])
            rotate([0, 90, 0])
                cylinder(d=oa_hub_clamp_bolt_d,
                         h=oa_hub_collar_d+2);
        translate([oa_hub_collar_d/2-oa_hub_clamp_nut_depth, 8,
                   oa_hex_depth+oa_hub_collar_h/2])
            rotate([0, 90, 0])
                cylinder(d=oa_hub_clamp_nut_af/cos(30),
                         h=oa_hub_clamp_nut_depth+1, $fn=6);
    }
}

// Provisional two-bearing sleeve represented in the assembly. Exported so all
// current Option A solids stay together, but do not print it as a final part
// until the real bearings and shaft have been measured.
module bearing_cartridge() {
    difference() {
        cylinder(d=34, h=46);
        translate([0,0,-1])
            cylinder(d=oa_transmission_shaft_clearance_d, h=48);
        translate([0,0,2])
            cylinder(d=oa_bearing_od, h=oa_bearing_width);
        translate([0,0,38])
            cylinder(d=oa_bearing_od, h=oa_bearing_width+1);
    }
}

// Universal zip-tie sensor carrier. The entry and confirmation variants only
// differ in drop length. Sensor pockets remain deliberately universal because
// the exact outdoor break-beam modules have not been purchased.
module ir_drop_bracket(sensor_z) {
    sensor_h = oa_ir_sensor_size[2];
    drop_h = oa_bridge_under_z-sensor_z-sensor_h/2;
    spine_w = 28;
    spine_t = 6;
    top_size = [44, 34, 5];
    shelf_size = [30, 28, 5];

    difference() {
        union() {
            translate([-spine_w/2, -spine_t/2, 0])
                cube([spine_w, spine_t, drop_h]);
            translate([-top_size[0]/2, -top_size[1]/2, drop_h])
                cube(top_size);
            translate([-shelf_size[0]/2, -shelf_size[1]/2, 0])
                cube(shelf_size);
            // Low backstop keeps the sensor square while a zip tie is fitted.
            translate([-spine_w/2, -spine_t/2, 0])
                cube([spine_w, spine_t, sensor_h+4]);
        }
        // Two M4 bridge holes.
        for (xx = [-14, 14])
            translate([xx, 0, drop_h-1])
                cylinder(d=m4_clearance_d, h=top_size[2]+2);
        // Two zip-tie slots through the sensor shelf.
        for (xx = [-8, 8])
            translate([xx-1.5, -shelf_size[1]/2-1, -1])
                cube([3, shelf_size[1]+2, shelf_size[2]+2]);
    }
}

module bridge_mounted_ir_pair(x, z, mount_y, pair_color="red") {
    // Drop brackets stay outside the ball corridor and keep both optical
    // elements aligned from the same rigid plywood reference.
    for (sy = [-1, 1]) {
        color("dimgray")
            translate([x, sy*mount_y,
                       (oa_bridge_under_z+z+oa_ir_sensor_size[2]/2)/2])
                cube([oa_ir_sensor_size[0], oa_ir_drop_t,
                      oa_bridge_under_z-z-oa_ir_sensor_size[2]/2],
                     center=true);
        color(pair_color)
            translate([x, sy*mount_y, z])
                cube(oa_ir_sensor_size, center=true);
    }

    color(pair_color, 0.75)
        translate([x, 0, z])
            rotate([90, 0, 0])
                cylinder(d=oa_ir_beam_d, h=2*mount_y,
                         center=true, $fn=20);
}

module intake_ir_beams() {
    // Amber: entry/capture timing. Cyan: confirmed handoff into the basket.
    bridge_mounted_ir_pair(oa_ir1_x, oa_ir1_z, oa_ir1_mount_y,
                           "darkorange");
    bridge_mounted_ir_pair(oa_ir2_x, oa_ir2_z, oa_ir2_mount_y,
                           "deepskyblue");
}

module curved_cheek(side=1) {
    // The ball-facing curve is smooth in plan. The final structural tail that
    // blends this surface into the wooden portal remains intentionally open.
    color("darkorange")
        difference() {
            union() {
                for (i = [0:oa_cheek_steps-1]) {
                    t0 = i/oa_cheek_steps;
                    t1 = (i+1)/oa_cheek_steps;
                    p0 = bezier3(oa_cheek_p0, oa_cheek_p1,
                                 oa_cheek_p2, oa_cheek_p3, t0);
                    p1 = bezier3(oa_cheek_p0, oa_cheek_p1,
                                 oa_cheek_p2, oa_cheek_p3, t1);
                    hull() {
                        translate([p0[0], side*p0[1], oa_cheek_bottom_z])
                            cylinder(d=oa_cheek_t,
                                     h=oa_cheek_top_z-oa_cheek_bottom_z);
                        translate([p1[0], side*p1[1], oa_cheek_bottom_z])
                            cylinder(d=oa_cheek_t,
                                     h=oa_cheek_top_z-oa_cheek_bottom_z);
                    }
                }
                // Horizontal top flange grows OUTWARD from the ball-facing
                // wall and overlaps the plywood bridge. It avoids the motor
                // slots near y=+/-92 mm and replaces all cheek support rods.
                translate([(oa_cheek_flange_x0+oa_cheek_flange_x1)/2,
                           side*(oa_cheek_p3[1]+oa_cheek_flange_outer_y)/2,
                           oa_bridge_under_z-oa_cheek_flange_t/2])
                    cube([oa_cheek_flange_x1-oa_cheek_flange_x0,
                          oa_cheek_flange_outer_y-oa_cheek_p3[1],
                          oa_cheek_flange_t], center=true);
            }
            for (xx = oa_cheek_mount_hole_x)
                translate([xx, side*oa_cheek_mount_hole_y,
                           oa_bridge_under_z-oa_cheek_flange_t-1])
                    cylinder(d=m5_clearance_d,
                             h=oa_cheek_flange_t+2);
        }
}

module short_handoff_ramp() {
    color("goldenrod")
        union() {
            for (i = [0:24-1]) {
                x0 = oa_ramp_front_x
                    +(oa_ramp_rear_x-oa_ramp_front_x)*i/24;
                x1 = oa_ramp_front_x
                    +(oa_ramp_rear_x-oa_ramp_front_x)*(i+1)/24;
                hull() {
                    translate([x0, 0, oa_ramp_z(x0)/2])
                        cube([0.7, oa_ramp_width, oa_ramp_z(x0)], center=true);
                    translate([x1, 0, oa_ramp_z(x1)/2])
                        cube([0.7, oa_ramp_width, oa_ramp_z(x1)], center=true);
                }
                for (sy = [-1, 1])
                    hull() {
                        translate([x0,
                                   sy*(oa_ramp_width/2+2),
                                   (oa_ramp_z(x0)+oa_ramp_wall_h)/2])
                            cube([0.7, 4,
                                  oa_ramp_z(x0)+oa_ramp_wall_h], center=true);
                        translate([x1,
                                   sy*(oa_ramp_width/2+2),
                                   (oa_ramp_z(x1)+oa_ramp_wall_h)/2])
                            cube([0.7, 4,
                                  oa_ramp_z(x1)+oa_ramp_wall_h], center=true);
                    }
            }
        }
}

module tilted_wheel_motor_pod(side=1) {
    sy = side*oa_wheel_y;
    axis_to_bridge_under = (oa_bridge_under_z-oa_wheel_z)
        / cos(oa_wheel_tilt);
    axis_to_bridge_top = (oa_bridge_top_z-oa_wheel_z)
        / cos(oa_wheel_tilt);

    translate([oa_wheel_x, sy, oa_wheel_z])
        rotate([0, oa_wheel_tilt, 0]) {
            // Purchased Pro-Line 2.8-inch wheel/tire envelope.
            color("#25282b")
                difference() {
                    cylinder(d=oa_wheel_d,
                             h=oa_wheel_width, center=true);
                    cylinder(d=42, h=oa_wheel_width+2, center=true);
                }
            color("black")
                cylinder(d=44, h=oa_wheel_width-4, center=true);

            // Replaceable printed 6 mm -> 12 mm RC hex adapter.
            color("gold")
                translate([0, 0, oa_wheel_width/2-oa_hex_depth])
                    printed_hex_hub();

            // Supported transmission shaft between wheel and bridge.
            color("silver")
                translate([0, 0, -oa_wheel_width/2-2])
                    cylinder(d=oa_transmission_shaft_d,
                             h=axis_to_bridge_top+oa_wheel_width/2+18);

            // Fixed two-bearing cartridge straddles the plywood bridge.
            color("slategray")
                translate([0, 0, axis_to_bridge_under-24])
                    bearing_cartridge();

            // 5-to-6 mm flexible coupler, above both supporting bearings.
            color("silver")
                translate([0, 0, axis_to_bridge_top+18])
                    difference() {
                        cylinder(d=18, h=25);
                        translate([0,0,-1])
                            cylinder(d=oa_transmission_shaft_clearance_d,
                                     h=27);
                    }

            // Approximate motor body, aligned with the tilted shaft.
            color("steelblue")
                translate([0, 0, axis_to_bridge_top+43])
                    cylinder(d=oa_motor_d, h=oa_motor_l);

            // Fixed pod plate on top of the wooden bridge.
            color("dimgray")
                translate([0, 0, axis_to_bridge_top+2])
                    cylinder(d=58, h=6);
        }
}

module scale_ball(x, y=0, z=33) {
    color("greenyellow", 0.8) translate([x,y,z]) sphere(d=66);
}

module option_a() {
    chassis_plate_option_a();
    basket_reference();
    // Semi-transparent for concept review so the tilted wheel pods remain
    // visible through the plywood portal in top/perspective renders.
    color("peru", 0.58) plywood_bridge();
    curved_cheek(1);
    curved_cheek(-1);
    short_handoff_ramp();
    tilted_wheel_motor_pod(1);
    tilted_wheel_motor_pod(-1);
    if (show_ir_beams) intake_ir_beams();

    if (show_balls) {
        scale_ball(780);
        scale_ball(650, 35);
        scale_ball(505);
        scale_ball(390, 0, 62);
    }
}

// Cropped review assembly: removes the large chassis/basket and hides the
// plywood slab so both beam paths and full drop brackets are visible. Their
// upper ends still show the exact bridge underside attachment plane.
module option_a_ir_review() {
    curved_cheek(1);
    curved_cheek(-1);
    short_handoff_ramp();
    tilted_wheel_motor_pod(1);
    tilted_wheel_motor_pod(-1);
    intake_ir_beams();

    // Two simultaneous scale references: entry and confirmed handoff.
    scale_ball(oa_ir1_x, 0, 33);
    scale_ball(oa_ir2_x, 0, 68);
}

if (part == "hex_hub")
    printed_hex_hub();
else if (part == "cheek_left")
    curved_cheek(1);
else if (part == "cheek_right")
    curved_cheek(-1);
else if (part == "ramp")
    short_handoff_ramp();
else if (part == "bearing_cartridge")
    bearing_cartridge();
else if (part == "ir_entry_bracket")
    ir_drop_bracket(oa_ir1_z);
else if (part == "ir_confirmation_bracket")
    ir_drop_bracket(oa_ir2_z);
else if (part == "ir_review")
    option_a_ir_review();
else if (part == "assembly")
    option_a();
else
    assert(false, str("Unknown Option A part: ", part));
