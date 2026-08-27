// ANALYSIS ONLY — fixed-motor, compliant-tyre standalone intake.
// Units: mm; local standalone frame, robot +X forward.
//
// Physical invariant: both FIT0186 motors and both Raid wheel centres are
// fixed. Compliance belongs to the tennis ball and the Trencher tyre/open-cell
// insert. This file contains no rail, prismatic pod, return spring, pod stop,
// or moving-motor cable loop.
//
// Purchased-interface details remain deliberately unresolved: motor D-flat,
// adapter bore/stop/set screw, Raid pocket/seating, and axial retention are not
// modelled. The printed mounts are provisional because their bolt pattern,
// print allowance and cable-tail envelope are not released. The 35 degree axis lies
// in the robot longitudinal X-Z plane. Left and right axes are parallel; the
// wheel centres alone are mirrored across Y=0.

$fn = 96;

part = "assembly"; // [assembly,bridge_interface,left_stack,left_mount,tyre_envelope]
show_labels = true;

tilt_deg = 35;
wheel_d = 124;
wheel_width = 73;
wheel_y = 90;
wheel_z = 70;
ball_d = 66;
fixed_gap = 56;
total_closure = ball_d-fixed_gap;
tyre_required_radial_envelope = total_closure/2; // 0..5 mm per wheel

adapter_length_spec = 30;
adapter_d_unmeasured_envelope = 20;
adapter_seating_unmeasured = 8;
motor_d_analysis = 30;
motor_length_analysis = 70;

// User-measured mount inputs. Each printed mount grips the topmost 10 mm axial
// band of the motor body; its upper edge is flush with the motor top face.
mount_motor_d = 30;
mount_depth = 10;
mount_bore_print_allowance = 0.6; // provisional FDM fit allowance
mount_outer_d = 42;
mount_bridge_pad = [50, 42, 5];

bridge_depth = 220;
bridge_width = 490;
bridge_under_z = 190;
bridge_t = 18;
bridge_top_z = bridge_under_z+bridge_t;

wheel_color = [0.08, 0.09, 0.10, 0.90];
raid_color = [0.18, 0.20, 0.23, 0.96];
motor_color = [0.22, 0.38, 0.58, 0.94];
adapter_unknown_color = [0.92, 0.58, 0.05, 0.55];
bridge_color = [0.63, 0.42, 0.22, 0.48];
fixed_mount_unknown_color = [0.62, 0.18, 0.72, 0.32];
printed_mount_color = [0.72, 0.24, 0.82, 0.92];
tyre_compliance_color = [0.22, 0.82, 0.32, 0.28];
ball_color = [0.70, 0.92, 0.12, 0.68];
interference_color = [0.94, 0.05, 0.04, 0.90];
closure_color = [1.00, 0.25, 0.05, 0.62];

function axis_x(distance) = distance*sin(tilt_deg);
function axis_y(side) = side*wheel_y;
function axis_z(distance) = wheel_z+distance*cos(tilt_deg);
function axis_distance_at_z(z) = (z-wheel_z)/cos(tilt_deg);

motor_face_s = wheel_width/2-adapter_seating_unmeasured+adapter_length_spec;
motor_centre_s = motor_face_s+motor_length_analysis/2;
bridge_under_s = axis_distance_at_z(bridge_under_z);
bridge_top_s = axis_distance_at_z(bridge_top_z);
bridge_under_axis_x = axis_x(bridge_under_s);
bridge_top_axis_x = axis_x(bridge_top_s);
motor_section_half_x = motor_d_analysis/(2*cos(tilt_deg));
vertical_cut_min_x = min(bridge_under_axis_x, bridge_top_axis_x)-motor_section_half_x;
vertical_cut_max_x = max(bridge_under_axis_x, bridge_top_axis_x)+motor_section_half_x;
vertical_cut_minimum_x = vertical_cut_max_x-vertical_cut_min_x;

module bridge_solid() {
    // Raised intact datum. The motor envelopes stay below it; the two printed
    // mounts provide the load path without moving either coaxial stack.
    translate([0, 0, bridge_under_z+bridge_t/2])
        cube([bridge_depth, bridge_width, bridge_t], center=true);
}

module tyre_and_raid(side) {
    translate([0, side*wheel_y, wheel_z])
        rotate([0, tilt_deg, 0]) {
            color(wheel_color)
                difference() {
                    cylinder(d=wheel_d, h=wheel_width, center=true);
                    cylinder(d=44, h=wheel_width+2, center=true);
                }
            color(raid_color) cylinder(d=44, h=wheel_width-4, center=true);

            // Full circumferential shell is a visual bound only. Physical
            // compression is local at the ball/tread contact patch.
            color(tyre_compliance_color)
                difference() {
                    cylinder(d=wheel_d+0.5, h=wheel_width+1, center=true);
                    cylinder(d=wheel_d-2*tyre_required_radial_envelope,
                             h=wheel_width+3, center=true);
                }
        }
}

module unresolved_adapter(side) {
    adapter_s0 = wheel_width/2-adapter_seating_unmeasured;
    translate([0, side*wheel_y, wheel_z])
        rotate([0, tilt_deg, 0])
            color(adapter_unknown_color)
                translate([0, 0, adapter_s0])
                    cylinder(d=adapter_d_unmeasured_envelope,
                             h=adapter_length_spec);
}

module motor_body(side) {
    adapter_s0 = wheel_width/2-adapter_seating_unmeasured;
    translate([0, side*wheel_y, wheel_z])
        rotate([0, tilt_deg, 0])
            translate([0, 0, motor_face_s])
                cylinder(d=motor_d_analysis, h=motor_length_analysis);
}

module motor_axis_cylinder(side, s0, diameter, depth) {
    translate([0, side*wheel_y, wheel_z])
        rotate([0, tilt_deg, 0])
            translate([0, 0, s0])
                cylinder(d=diameter, h=depth);
}

module printed_motor_mount(side) {
    mount_s0 = motor_face_s + motor_length_analysis - mount_depth;
    mount_sc = mount_s0 + mount_depth/2;
    mount_xc = axis_x(mount_sc);
    mount_zc = axis_z(mount_sc);
    pad_zc = bridge_under_z-mount_bridge_pad[2]/2;

    color(printed_mount_color)
        difference() {
            union() {
                // Short collar: exactly 10 mm deep along the motor axis.
                motor_axis_cylinder(side, mount_s0, mount_outer_d, mount_depth);

                // Bridge pad and two compact hanger webs. Bridge holes and a
                // final clamp split/fastener remain pending physical fit-up.
                translate([mount_xc, side*wheel_y, pad_zc])
                    cube(mount_bridge_pad, center=true);
                for (dy = [-1, 1])
                    hull() {
                        translate([mount_xc,
                                   side*wheel_y+dy*(mount_outer_d/2-3),
                                   mount_zc+mount_outer_d/2-2])
                            sphere(d=6);
                        translate([mount_xc,
                                   side*wheel_y+dy*(mount_bridge_pad[1]/2-4),
                                   bridge_under_z-mount_bridge_pad[2]])
                            cube([8, 8, 2], center=true);
                    }
            }

            // The measured Ø30 mm motor is retained; 0.6 mm diametral
            // clearance is only a provisional print-fit allowance.
            motor_axis_cylinder(side, mount_s0-1,
                                mount_motor_d+mount_bore_print_allowance,
                                mount_depth+2);
        }
}

module fixed_axis_marker(side) {
    color([0.05, 0.62, 0.90, 0.85])
        translate([0, side*wheel_y, wheel_z])
            rotate([0, tilt_deg, 0])
                cylinder(d=2, h=150, center=true);
}

module bridge_axis_intersections(side) {
    color([0.02, 0.75, 0.95, 0.95]) {
        translate([bridge_under_axis_x, axis_y(side), bridge_under_z])
            sphere(d=5);
        translate([bridge_top_axis_x, axis_y(side), bridge_top_z])
            sphere(d=5);
    }
}

module fixed_stack(side) {
    tyre_and_raid(side);
    unresolved_adapter(side);
    color(motor_color) motor_body(side);
    printed_motor_mount(side);
    fixed_axis_marker(side);
}

module ball_and_planar_closure() {
    // Centre at wheel-centre Z to show the nominal 56/66 planar nip screen.
    // Actual 3D contact follows the rising ball path and tilted tread width.
    color(ball_color) translate([0, 0, wheel_z]) sphere(d=ball_d);

    for (side = [-1, 1])
        color(closure_color)
            translate([0, side*(fixed_gap/2+total_closure/4), wheel_z])
                cube([18, total_closure/2, 18], center=true);
}

module bridge_motor_interference() {
    color(interference_color)
        intersection() {
            bridge_solid();
            union() {
                motor_body(-1);
                motor_body(1);
            }
        }
}

module bridge_interface_view() {
    color(bridge_color) bridge_solid();
    color(motor_color) { motor_body(-1); motor_body(1); }
    fixed_axis_marker(-1);
    fixed_axis_marker(1);
    bridge_axis_intersections(-1);
    bridge_axis_intersections(1);
    bridge_motor_interference();
}

module labels() {
    if (show_labels) {
        color([0.05, 0.05, 0.05])
            translate([0, -225, 3]) linear_extrude(1)
                text("FIXED MOTOR + COMPLIANT TYRE", size=12, halign="center");
        color([0.05, 0.45, 0.08])
            translate([0, -225, 20]) linear_extrude(1)
                text("0..5 mm REQUIRED RADIAL TYRE ENVELOPE", size=7,
                     halign="center");
        color([0.48, 0.05, 0.62])
            translate([0, 185, 218]) rotate([90, 0, 0]) linear_extrude(1)
                text(str("2 PRINTED MOUNTS: DIA 30 mm x 10 mm GRIP; BRIDGE Z=",
                         bridge_under_z, "..", bridge_top_z, " mm"),
                     size=6, halign="center");
    }
}

module assembly() {
    color(bridge_color) bridge_solid();
    fixed_stack(-1);
    fixed_stack(1);
    ball_and_planar_closure();
    bridge_motor_interference();
    bridge_axis_intersections(-1);
    bridge_axis_intersections(1);
    labels();
}

if (part == "bridge_interface") {
    bridge_interface_view();
} else if (part == "left_stack") {
    fixed_stack(1);
} else if (part == "left_mount") {
    printed_motor_mount(1);
} else if (part == "tyre_envelope") {
    tyre_and_raid(-1);
    tyre_and_raid(1);
    ball_and_planar_closure();
} else {
    assembly();
}
