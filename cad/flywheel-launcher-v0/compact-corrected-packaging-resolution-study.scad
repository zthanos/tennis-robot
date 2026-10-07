// ANALYSIS ONLY -- corrected-geometry compact packaging resolution probes.
// This file does not define or replace manufacturing geometry.

use <compact-intake-pod-concept-study.scad>
use <compact-validation-export.scad>
use <compact-parked-reliefs.scad>
use <launcher-envelope.scad>
include <params.scad>

$fn = 48;

part = "scene";
target = "bridge";
component = "motor";
launcher_component = "assembly";
selected_side = 0;
selected_plate = 0;
launcher_dx = 0;
launcher_dy = 0;
launcher_dz = 0;
clearance = 0;
pocket_clearance = 2;

intake_wheel_x = 470;
intake_wheel_y = 90;
intake_wheel_z = 70;
intake_axis_pitch = 35;
compact_intake_wheel_pocket_d = 128;
compact_intake_wheel_pocket_w = 77;

module clearance_expand(amount=clearance) {
    if (amount <= 0) children();
    else minkowski() {
        children();
        sphere(r=amount, $fn=24);
    }
}

module relocated_launcher_transform() {
    translate([launcher_dx, launcher_dy, launcher_dz]) launcher_transform()
        children();
}

module relocated_launcher_plate(sign) {
    relocated_launcher_transform() side_plate(sign);
}

module relocated_launcher_component(name=launcher_component) {
    if (name == "lower_plate") relocated_launcher_plate(-1);
    else if (name == "upper_plate") relocated_launcher_plate(1);
    else if (name == "cradle") {
        relocated_launcher_plate(-1);
        relocated_launcher_plate(1);
    }
    else if (name == "left_flywheel")
        relocated_launcher_transform() wheel_envelope(lower_wheel_z);
    else if (name == "right_flywheel")
        relocated_launcher_transform() wheel_envelope(upper_wheel_z);
    else if (name == "flywheels") {
        relocated_launcher_transform() wheel_envelope(lower_wheel_z);
        relocated_launcher_transform() wheel_envelope(upper_wheel_z);
    }
    else if (name == "assembly") {
        relocated_launcher_component("cradle");
        relocated_launcher_component("flywheels");
    }
    else assert(false, str("Unknown launcher component: ", name));
}

module stationary_environment(name=target) {
    if (name == "lidar") compact_lidar();
    else fixed_environment(name);
}

module existing_parked_pocket_one_side(side) {
    shifted_local()
        translate([intake_wheel_x, side * intake_wheel_y, intake_wheel_z])
            rotate([0, intake_axis_pitch, 0])
                cylinder(d=compact_intake_wheel_pocket_d,
                         h=compact_intake_wheel_pocket_w, center=true);
}

module existing_parked_pockets() {
    existing_parked_pocket_one_side(1);
    existing_parked_pocket_one_side(-1);
}

module selected_swept_component() {
    swept_selected_component();
}

module full_swept_motors() {
    for (side = [-1, 1])
        hull() { fit0186_motor_body(side, 0); fit0186_motor_body(side, 8); }
}

module full_swept_wheels() {
    for (side = [-1, 1])
        hull() { pro117010_wheel(side, 0); pro117010_wheel(side, 8); }
}

module selected_relief_envelope() {
    clearance_expand() selected_swept_component();
}

module selected_launcher_relief_envelope() {
    clearance_expand() selected_swept_component();
}

module launcher_protected_environment() {
    // Combined-candidate environment: the only allowed fixed-structure
    // changes are the full-sweep tyre-pocket extension and motor bridge relief.
    for (name = ["basket", "hood"])
        difference() {
            stationary_environment(name);
            clearance_expand(pocket_clearance) full_swept_wheels();
        }
    difference() {
        stationary_environment("bridge");
        clearance_expand(clearance) full_swept_motors();
    }
    for (name = ["chassis", "battery", "lidar"])
        stationary_environment(name);
}

module combined_environment(name=target) {
    if (name == "basket" || name == "hood")
        difference() {
            stationary_environment(name);
            clearance_expand(pocket_clearance) full_swept_wheels();
        }
    else if (name == "bridge")
        difference() {
            stationary_environment("bridge");
            clearance_expand(clearance) full_swept_motors();
        }
    else stationary_environment(name);
}

module launcher_hard_conflict_probe() {
    // Cradle intersections are intentionally excluded: those are evaluated as
    // shaped local reliefs. Flywheels may never be relieved.
    intersection() {
        clearance_expand() selected_swept_component();
        relocated_launcher_component("flywheels");
    }
    intersection() {
        clearance_expand() relocated_launcher_component("assembly");
        launcher_protected_environment();
    }
}

if (part == "swept_component") selected_swept_component();
else if (part == "launcher_component") relocated_launcher_component();
else if (part == "stationary_environment") stationary_environment();
else if (part == "swept_launcher_intersection")
    intersection() {
        selected_swept_component();
        relocated_launcher_component();
    }
else if (part == "fixed_guides_launcher_intersection")
    intersection() {
        both_fixed_guides();
        relocated_launcher_component();
    }
else if (part == "launcher_environment_intersection")
    intersection() {
        relocated_launcher_component();
        stationary_environment();
    }
else if (part == "launcher_combined_environment_intersection")
    intersection() {
        clearance_expand() relocated_launcher_component();
        combined_environment();
    }
else if (part == "relief_intersection")
    intersection() {
        selected_relief_envelope();
        stationary_environment();
    }
else if (part == "cradle_relief_intersection")
    intersection() {
        selected_launcher_relief_envelope();
        relocated_launcher_component("cradle");
    }
else if (part == "plate_relief_intersection")
    intersection() {
        selected_launcher_relief_envelope();
        relocated_launcher_plate(selected_plate);
    }
else if (part == "existing_pocket") existing_parked_pockets();
else if (part == "pocket_extension")
    difference() {
        selected_relief_envelope();
        existing_parked_pockets();
    }
else if (part == "hard_conflict_probe") launcher_hard_conflict_probe();
else if (part == "scene") {
    complete_machine_scene();
    color("magenta", 0.45) relocated_launcher_component();
}
else assert(false, str("Unknown part: ", part));
