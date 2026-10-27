// HISTORICAL_SUPERSEDED -- NOT_PHYSICAL_INTAKE_ARCHITECTURE.
// Retained only because corrected launcher-packaging analysis imports it.
// The physical intake has fixed FIT0186 motors/wheel centres and compliant
// tyres; the moving-pod sweep below is not current manufacturing geometry.
//
// Intended drivetrain per side:
// FIT0186 -> its 6 mm D-shaft -> purchased 14-00012630 -> PRO117010 12 mm
// removable hex -> PRO117010 Raid/Trencher wheel.
//
// Evidence classes used below: MEASURED_FROM_HARDWARE, MANUFACTURER_SPEC,
// PURCHASED_PART_DATUM, EXISTING_AUTHORITATIVE_CAD,
// DERIVED_FROM_LOCKED_DATUM, ANALYSIS_ONLY_ASSUMPTION, MEASUREMENT_PENDING.
// There is intentionally no transmission shaft, external bearing, bearing
// cartridge, coupler, belt, pulley, reduction, or printed wheel hub.

use <compact-packaging-study.scad>
use <compact-validation-export.scad>
use <../collector-intake-v1/option-a/option-a.scad>
use <compact-parked-reliefs.scad>
use <launcher-envelope.scad>
include <params.scad>

$fn = 48;

displacement = 0;              // DERIVED_FROM_LOCKED_DATUM: 0..8 mm outward
part = "assembly";
target = "bridge";
component = "direct_drive_hardware";
selected_side = 0;             // +1 left, -1 right, 0 both
adapter_seating_mm = 8;        // ANALYSIS_ONLY_ASSUMPTION

functional_shift_x = -100;     // EXISTING_AUTHORITATIVE_CAD
intake_wheel_x = 470;          // locked compact-local centre
intake_wheel_y = 90;
intake_wheel_z = 70;
intake_wheel_d = 124;          // MANUFACTURER_SPEC
intake_wheel_width = 73;
intake_axis_pitch = 35;

motor_d = 30;                  // MEASURED_FROM_HARDWARE
motor_length = 70;
motor_shaft_d = 6;
motor_shaft_projection = 20;

adapter_length = 30;           // PURCHASED_PART_DATUM
adapter_hex_af = 12;
// MEASUREMENT_PENDING: actual maximum outside profile. 20 mm is an
// explicitly conservative ANALYSIS_ONLY_ASSUMPTION for collision screening.
adapter_analysis_envelope_d = 20;
// MEASUREMENT_PENDING: exact installed Raid narrow/wide hex offset/depth.

wheel_outer_face_s = intake_wheel_width / 2;
adapter_s0 = wheel_outer_face_s - adapter_seating_mm;
motor_face_s = adapter_s0 + adapter_length;
motor_body_s0 = motor_face_s;
motor_shaft_s0 = motor_face_s - motor_shaft_projection;

assert(displacement >= 0 && displacement <= 8,
       "pod displacement must remain inside locked 0..8 mm travel");
assert(adapter_seating_mm >= 0 && adapter_seating_mm <= adapter_length,
       "analysis adapter seating must be within purchased 30 mm length");

module shifted_local() { translate([functional_shift_x, 0, 0]) children(); }

module axis_place(side=1, travel=0, s=0) {
    shifted_local()
        translate([intake_wheel_x,
                   side * (intake_wheel_y + travel),
                   intake_wheel_z])
            rotate([0, intake_axis_pitch, 0])
                translate([0, 0, s]) children();
}

// MANUFACTURER_SPEC external envelope. The full cylinder is conservative
// relative to tread/rim voids and is appropriate for package decisions.
module pro117010_wheel(side=1, travel=0) {
    axis_place(side, travel)
        cylinder(d=intake_wheel_d, h=intake_wheel_width, center=true);
}

// The interface family and 12 mm AF are supplier/purchased datums. Installed
// depth is MEASUREMENT_PENDING, so this is not a manufacturing profile.
module pro117010_hex_interface(side=1, travel=0) {
    axis_place(side, travel, adapter_s0 - 1)
        cylinder(d=adapter_hex_af / cos(30),
                 h=adapter_seating_mm + 2, $fn=6);
}

module purchased_14_00012630(side=1, travel=0) {
    axis_place(side, travel, adapter_s0)
        cylinder(d=adapter_analysis_envelope_d, h=adapter_length);
}

module fit0186_output_shaft(side=1, travel=0) {
    axis_place(side, travel, motor_shaft_s0)
        cylinder(d=motor_shaft_d, h=motor_shaft_projection);
}

module fit0186_motor_body(side=1, travel=0) {
    axis_place(side, travel, motor_body_s0)
        cylinder(d=motor_d, h=motor_length);
}

module direct_drive_hardware(side=1, travel=0) {
    pro117010_wheel(side, travel);
    pro117010_hex_interface(side, travel);
    purchased_14_00012630(side, travel);
    fit0186_output_shaft(side, travel);
    fit0186_motor_body(side, travel);
}

module y_cylinder(d, h) {
    rotate([90, 0, 0]) cylinder(d=d, h=h, center=true);
}

// Candidate A remains the guide-family baseline. These are only allocation
// probes, not final motor support, spring, cable, stop, or fastening CAD.
module candidate_a_moving_structure(side=1, travel=0) {
    sy = side * (intake_wheel_y + travel);
    shifted_local() {
        for (xx = [498, 578])
            translate([xx, sy, 182])
                difference() {
                    y_cylinder(18, 34);
                    y_cylinder(8.6, 36);
                }
        translate([538, sy, 195]) cube([98, 34, 12], center=true);
        translate([490, sy, 182]) cube([10, 24, 18], center=true);
    }
}

module candidate_a_fixed_guide(side=1) {
    sy = side * 94;
    shifted_local() {
        for (xx = [498, 578]) {
            translate([xx, sy, 182]) y_cylinder(8, 78);
            for (yy = [side * 55, side * 133])
                translate([xx, yy, 178]) cube([24, 12, 20], center=true);
        }
        for (yy = [side * 69, side * 111])
            translate([490, yy, 182]) cube([12, 5, 24], center=true);
    }
}

module complete_moving_pod(side=1, travel=displacement) {
    direct_drive_hardware(side, travel);
    candidate_a_moving_structure(side, travel);
}

module selected_component_one_side(side, travel=displacement) {
    if (component == "wheel") pro117010_wheel(side, travel);
    else if (component == "wheel_hex") pro117010_hex_interface(side, travel);
    else if (component == "adapter") purchased_14_00012630(side, travel);
    else if (component == "motor_output_shaft") fit0186_output_shaft(side, travel);
    else if (component == "motor") fit0186_motor_body(side, travel);
    else if (component == "direct_drive_hardware") direct_drive_hardware(side, travel);
    else if (component == "carriage_moving") candidate_a_moving_structure(side, travel);
    else if (component == "complete_moving_pod") complete_moving_pod(side, travel);
    else assert(false, str("Unknown component: ", component));
}

module selected_component_at(travel=displacement) {
    if (selected_side == 0) {
        selected_component_one_side(1, travel);
        selected_component_one_side(-1, travel);
    } else selected_component_one_side(selected_side, travel);
}

module swept_hardware_one_side(side) {
    hull() { pro117010_wheel(side, 0); pro117010_wheel(side, 8); }
    hull() { pro117010_hex_interface(side, 0); pro117010_hex_interface(side, 8); }
    hull() { purchased_14_00012630(side, 0); purchased_14_00012630(side, 8); }
    hull() { fit0186_output_shaft(side, 0); fit0186_output_shaft(side, 8); }
    hull() { fit0186_motor_body(side, 0); fit0186_motor_body(side, 8); }
}

// Pure linear translation: each convex hardware primitive's endpoint hull is
// its continuous swept set. Carriage hulls conservatively fill follower bores.
module swept_component_one_side(side) {
    if (component == "wheel")
        hull() { pro117010_wheel(side, 0); pro117010_wheel(side, 8); }
    else if (component == "wheel_hex")
        hull() { pro117010_hex_interface(side, 0); pro117010_hex_interface(side, 8); }
    else if (component == "adapter")
        hull() { purchased_14_00012630(side, 0); purchased_14_00012630(side, 8); }
    else if (component == "motor_output_shaft")
        hull() { fit0186_output_shaft(side, 0); fit0186_output_shaft(side, 8); }
    else if (component == "motor")
        hull() { fit0186_motor_body(side, 0); fit0186_motor_body(side, 8); }
    else if (component == "direct_drive_hardware") swept_hardware_one_side(side);
    else if (component == "carriage_moving")
        hull() { candidate_a_moving_structure(side, 0);
                 candidate_a_moving_structure(side, 8); }
    else if (component == "complete_moving_pod") {
        swept_hardware_one_side(side);
        hull() { candidate_a_moving_structure(side, 0);
                 candidate_a_moving_structure(side, 8); }
    }
    else assert(false, str("Unknown swept component: ", component));
}

module swept_selected_component() {
    if (selected_side == 0) {
        swept_component_one_side(1);
        swept_component_one_side(-1);
    } else swept_component_one_side(selected_side);
}

module launcher_transform() {
    shifted_local() translate([560, 0, 0])
        translate([0, 0, 215])
            rotate([0, -pitch_deg, 0])
                rotate([90, 0, 0])
                    translate([0, 0, -path_z]) children();
}

module physical_launcher_cradle_only() {
    launcher_transform() { side_plate(-1); side_plate(1); }
}

// Side-by-side rotation maps lower_wheel_z to world +Y (left).
module physical_left_flywheel() {
    launcher_transform() wheel_envelope(lower_wheel_z);
}
module physical_right_flywheel() {
    launcher_transform() wheel_envelope(upper_wheel_z);
}

module fixed_environment(name) {
    if (name == "basket") shifted_local() compact_relieved_bin();
    else if (name == "hood") shifted_local() compact_fixed_hood();
    else if (name == "ramp") shifted_local() compact_handoff_ramp();
    else if (name == "cheeks") shifted_local() { curved_cheek(1); curved_cheek(-1); }
    else if (name == "bridge") shifted_local() compact_bridge();
    else if (name == "chassis") chassis_plate_option_a();
    else if (name == "battery") compact_battery();
    else if (name == "launcher_cradle") physical_launcher_cradle_only();
    else if (name == "left_flywheel") physical_left_flywheel();
    else if (name == "right_flywheel") physical_right_flywheel();
    else assert(false, str("Unknown environment target: ", name));
}

module selected_target() {
    if (target == "opposite_pod") {
        assert(selected_side != 0,
               "opposite_pod target requires selected_side +/-1");
        complete_moving_pod(-selected_side, displacement);
    } else fixed_environment(target);
}

module both_fixed_guides() {
    candidate_a_fixed_guide(1);
    candidate_a_fixed_guide(-1);
}

module complete_machine_scene() {
    // All required mechanisms are simultaneous physical solids.
    color("burlywood", 0.45) fixed_environment("chassis");
    color("peru", 0.55) fixed_environment("bridge");
    color("darkorange", 0.50) fixed_environment("cheeks");
    color("goldenrod", 0.65) fixed_environment("ramp");
    color("seagreen", 0.22) fixed_environment("basket");
    color("lightsteelblue", 0.32) fixed_environment("hood");
    color("darkslategray", 0.55) fixed_environment("battery");
    color("slategray", 0.55) fixed_environment("launcher_cradle");
    color("crimson", 0.50) fixed_environment("left_flywheel");
    color("crimson", 0.50) fixed_environment("right_flywheel");
    color("silver") both_fixed_guides();
    color("darkorange") { complete_moving_pod(1); complete_moving_pod(-1); }
}

if (part == "component") selected_component_at();
else if (part == "swept_component") swept_selected_component();
else if (part == "environment") fixed_environment(target);
else if (part == "fixed_guides") both_fixed_guides();
else if (part == "component_environment_intersection")
    intersection() { selected_component_at(); selected_target(); }
else if (part == "swept_component_environment_intersection")
    intersection() { swept_selected_component(); fixed_environment(target); }
else if (part == "fixed_guide_environment_intersection")
    intersection() { both_fixed_guides(); fixed_environment(target); }
else if (part == "opposite_pod_intersection")
    intersection() { complete_moving_pod(1); complete_moving_pod(-1); }
else if (part == "swept_opposite_pod_intersection")
    intersection() {
        swept_component_one_side(1);
        swept_component_one_side(-1);
    }
else if (part == "assembly") complete_machine_scene();
else assert(false, str("Unknown part: ", part));
