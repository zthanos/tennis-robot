// SUPERSEDED_WRONG_COMPLIANCE_ARCHITECTURE — HISTORICAL ANALYSIS ONLY.
// Units: mm.
//
// This file visualises a rejected translating-pod assumption. The intended
// physical intake has FIXED FIT0186 motors and FIXED wheel centres; compliance
// comes from the tennis ball and the Trencher tyre/open-cell insert. Nothing in
// this file is current intake manufacturing geometry or physical simulation
// evidence. See cad/standalone-intake-fixed-motor instead.
//
// Historically this file showed dual-size-9-profile rails at 0, 4, and 8 mm.
// It is deliberately not manufacturing CAD. The FIT0186
// D-flat/shoulder, 14-00012630 bore/stop/set screw, Raid pocket/seating, wheel
// retention, final motor saddle holes, commercial rail details, and selected
// spring are not known and are not invented here.
//
// It owns no complete-robot, bridge, basket, launcher, or chassis geometry.

$fn = 64;

view_mode = "three_states"; // [three_states,rest,mid,outer]

tilt_deg = 35;
wheel_d = 124;
wheel_width = 73;
wheel_gap = 56;
wheel_y = 90; // retained standalone reference used by current intake studies
wheel_z = 70;

motor_d = 30;
motor_length = 70;
adapter_d_analysis = 20; // translucent allocation, not a measured profile
adapter_length_spec = 30;
adapter_engagement_analysis = 8; // display placement only, not a stack datum

rail_spacing = 70;
rail_length = 80;
rail_width_analysis = 9;
rail_height_analysis = 6;
block_length_analysis = 30;
block_width_analysis = 20;
travel_max = 8;

spring_x = 25;
spring_free_length_analysis = 31;
spring_installed_rest_analysis = 27;
cable_bend_radius_min = 30;

// Evidence colours: physical-envelope context, selected-guide analysis, and
// missing-interface allocation are intentionally visually distinct.
wheel_color = [0.08, 0.09, 0.10, 0.92];
hub_color = [0.18, 0.20, 0.23, 0.95];
motor_color = [0.28, 0.31, 0.34, 0.95];
missing_interface_color = [0.88, 0.05, 0.58, 0.45];
fixed_color = [0.32, 0.40, 0.50, 0.82];
rail_color = [0.72, 0.75, 0.78, 0.98];
moving_color = [0.96, 0.48, 0.08, 0.88];
spring_color = [0.25, 0.72, 0.30, 0.95];
inner_stop_color = [0.20, 0.72, 0.88, 0.95];
outer_stop_color = [0.92, 0.18, 0.12, 0.95];
cable_color = [0.15, 0.18, 0.22, 0.95];
cable_envelope_color = [0.15, 0.55, 0.95, 0.16];

module torus(major_r, minor_r) {
    rotate_extrude(convexity=8)
        translate([major_r, 0, 0]) circle(r=minor_r);
}

module spring_symbol_y(x, y0, y1, z) {
    // Ring stack is a deterministic analysis symbol, not spring wire CAD.
    rings = 9;
    for (i = [0:rings-1]) {
        yy = y0 + (y1-y0)*i/(rings-1);
        color(spring_color)
            translate([x, yy, z]) rotate([90, 0, 0]) torus(4.5, 0.75);
    }
}

module capsule_segment(p0, p1, diameter=4) {
    hull() {
        translate(p0) sphere(d=diameter);
        translate(p1) sphere(d=diameter);
    }
}

module cable_path(points) {
    for (i = [0:len(points)-2])
        color(cable_color) capsule_segment(points[i], points[i+1], 4);
}

module wheel_envelope(side, travel) {
    y = side*(wheel_y + travel);
    translate([0, y, wheel_z]) rotate([-side*tilt_deg, 0, 0]) {
        color(wheel_color)
            difference() {
                cylinder(d=wheel_d, h=wheel_width, center=true);
                cylinder(d=42, h=wheel_width+2, center=true);
            }
        color(hub_color) cylinder(d=42, h=wheel_width*0.72, center=true);
        color(missing_interface_color)
            translate([0, 0, wheel_width/2-adapter_engagement_analysis+adapter_length_spec/2])
                cylinder(d=adapter_d_analysis, h=adapter_length_spec, center=true);
        color(motor_color)
            translate([0, 0, 93.5]) cylinder(d=motor_d, h=motor_length, center=true);
        // The magenta shaft/adapter overlap is intentionally unresolved.
        color(missing_interface_color)
            translate([0, 0, 59]) cylinder(d=6, h=24, center=true);
    }
}

module fixed_guide(side) {
    rail_centre_y = side*145;

    color(fixed_color)
        translate([0, rail_centre_y, 181]) cube([100, rail_length+10, 4], center=true);

    for (xx = [-rail_spacing/2, rail_spacing/2]) {
        color(rail_color)
            translate([xx, rail_centre_y, 176])
                cube([rail_width_analysis, rail_length, rail_height_analysis], center=true);

        // Analysis-only M3 rail fastener positions.
        for (dy = [-30, -10, 10, 30])
            color([0.12, 0.14, 0.16])
                translate([xx, rail_centre_y + side*dy, 172.8]) cylinder(d=3, h=3, center=true);
    }

    // Fixed spring anchors, deliberately symmetric about X=0.
    for (xx = [-spring_x, spring_x])
        color(fixed_color)
            translate([xx, side*178, 190]) cube([8, 5, 18], center=true);

    // Replaceable inner pads and outer polyurethane stop washers.
    for (xx = [-spring_x, spring_x]) {
        color(inner_stop_color)
            translate([xx, side*158.5, 158]) cube([10, 3, 12], center=true);
        color(outer_stop_color)
            translate([xx, side*166.5, 158]) cube([10, 3, 12], center=true);
        // Secondary 8.5 mm metal catch.
        color([0.55, 0.08, 0.06])
            translate([xx, side*167.8, 151]) cube([12, 2, 7], center=true);
    }
}

module moving_guide(side, travel) {
    block_y = side*(137 + travel);

    for (xx = [-rail_spacing/2, rail_spacing/2]) {
        color(moving_color)
            translate([xx, block_y, 168])
                cube([block_width_analysis, block_length_analysis, 10], center=true);
        // Two separated pod beams avoid pretending a final motor-face plate.
        color(moving_color)
            translate([xx, block_y, 159]) cube([22, 40, 5], center=true);
    }
    color(moving_color)
        translate([0, side*(151 + travel), 159]) cube([80, 8, 5], center=true);

    // Symmetric moving spring seats and stop faces.
    for (xx = [-spring_x, spring_x]) {
        color(moving_color)
            translate([xx, side*(151 + travel), 190]) cube([8, 5, 18], center=true);
        color(moving_color)
            translate([xx, side*(157 + travel), 158]) cube([10, 2, 12], center=true);

        spring_symbol_y(
            xx,
            side*(151 + travel),
            side*178,
            190
        );
    }

    // Analysis motor saddle rings: support allocation only; no holes/fits.
    saddle_y = side*(143.63 + travel);
    saddle_z = 146.59;
    color(moving_color)
        translate([0, saddle_y, saddle_z]) rotate([-side*tilt_deg, 0, 0])
            difference() {
                cylinder(d=38, h=8, center=true);
                cylinder(d=31, h=10, center=true);
            }
}

module cable_service(side, travel) {
    moving_y = side*(164 + travel);
    points = [
        [0, moving_y, 176],
        [0, moving_y, 203],
        [0, side*195, 235],
        [0, side*220, 205]
    ];

    cable_path(points);
    color(cable_envelope_color)
        translate([0, side*195, 205]) cube([25, 80, 70], center=true);

    color(moving_color)
        translate([0, moving_y, 203]) cube([12, 6, 10], center=true);
    color(fixed_color)
        translate([0, side*220, 205]) cube([14, 6, 12], center=true);
}

module state_label(travel) {
    color([0.08, 0.08, 0.08])
        translate([0, -235, 0]) linear_extrude(1.2)
            text(str(travel, " mm OUTWARD"), size=14, halign="center");
    color([0.90, 0.02, 0.02])
        translate([0, -235, 20]) linear_extrude(1.2)
            text("SUPERSEDED - NOT PHYSICAL", size=9, halign="center");
}

module standalone_state(travel) {
    // Minimal floor datum only; no robot bridge/chassis context.
    color([0.85, 0.86, 0.88, 0.35])
        translate([0, 0, -3]) cube([180, 470, 4], center=true);

    for (side = [-1, 1]) {
        fixed_guide(side);
        moving_guide(side, travel);
        wheel_envelope(side, travel);
        cable_service(side, travel);
    }
    state_label(travel);
}

if (view_mode == "three_states") {
    translate([-250, 0, 0]) standalone_state(0);
    standalone_state(4);
    translate([250, 0, 0]) standalone_state(8);
} else if (view_mode == "rest") {
    standalone_state(0);
} else if (view_mode == "mid") {
    standalone_state(4);
} else {
    standalone_state(8);
}
