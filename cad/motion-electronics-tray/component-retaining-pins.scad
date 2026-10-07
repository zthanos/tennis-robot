// Printable push-pins for the motion electronics tray — dimensions in mm.
//
// Insert each pin from above, through the component mounting hole and the
// tray's M3 cross-slot. A separate C-clip locks into its groove below the tray
// and bridges the wide adjustment slot.
// These are prototype retainers; use M3 nylon/metal hardware for prolonged
// vibration or outdoor service.

include <params.scad>

$fn = 48;

part = "plate"; // [plate,pins,washers,washer,perf,mega,l298,bts]

pcb_t = 1.6;
mega_mount_t = 3.0; // measured case-hole diameter, provisional acrylic thickness
fit_allowance = 0.30;
// The 2.2 mm trial was too thin; 2.8 mm remains 0.4 mm below the original
// 3.2 mm shaft that would not pass through the physical component holes.
shaft_d = 2.80;
groove_d = 2.20;
groove_h = 1.80;
groove_below_tray = 0.50;
tip_h = 2.50;
head_d = 8.0;
head_t = 1.8;

clip_od = 12.0;
clip_t = 1.60;
clip_hole_d = 2.35;
clip_mouth_w = 2.00;

function grip_for(standoff_h, component_t=pcb_t) =
    tray_t + standoff_h + component_t + fit_allowance + groove_below_tray;

perf_grip = grip_for(perf_standoff_h);
mega_grip = grip_for(mega_case_standoff_h, mega_mount_t);
l298_grip = grip_for(l298_standoff_h);
// Physical fit correction: the +10 mm trial projected about 13 mm below the
// assembly. Remove that addition, leaving roughly 3 mm for groove/clip access.
bts_pin_extra_length = 0;
bts_grip = grip_for(bts_standoff_h + bts_spacer_h) + bts_pin_extra_length;

module retaining_pin(grip) {
    union() {
        cylinder(d=head_d, h=head_t);
        translate([0, 0, head_t])
            cylinder(d=shaft_d, h=grip);
        translate([0, 0, head_t + grip])
            cylinder(d=groove_d, h=groove_h);
        // Full-diameter lower shoulder keeps the C-clip captured in the groove;
        // its tapered end still passes freely through the tray slot.
        translate([0, 0, head_t + grip + groove_h])
            cylinder(d1=shaft_d, d2=1.8, h=tip_h);
    }
}

module locking_clip() {
    difference() {
        cylinder(d=clip_od, h=clip_t);
        translate([0, 0, -1])
            cylinder(d=clip_hole_d, h=clip_t+2);
        // Side entry turns the washer into a flexible C-clip. Slide it into the
        // 2.2 mm groove after the pin projects below the tray.
        translate([0, -clip_mouth_w/2, -1])
            cube([clip_od/2+1, clip_mouth_w, clip_t+2]);
    }
}

module pin_group(grip, count=4, pitch=13) {
    cols = count <= 4 ? count : 4;
    for (i = [0:count-1])
        translate([(i % cols)*pitch, floor(i/cols)*pitch, 0])
            retaining_pin(grip);
}

module pins_plate() {
    // 4 perfboard + 4 Mega-case + 4 L298N + 8 BTS pins, plus one spare of
    // each type. Rows are separated for easy identification in the slicer.
    pin_group(perf_grip, 5);
    translate([0, 16, 0]) pin_group(mega_grip, 5);
    translate([0, 32, 0]) pin_group(l298_grip, 5);
    translate([0, 48, 0]) pin_group(bts_grip, 9);
}

module washers_plate(count=24, pitch=12) {
    cols = 6;
    for (i = [0:count-1])
        translate([(i % cols)*pitch, floor(i/cols)*pitch, 0]) locking_clip();
}

module print_plate() {
    pins_plate();
    translate([60, 0, 0]) washers_plate();
}

if (part == "perf") retaining_pin(perf_grip);
else if (part == "mega") retaining_pin(mega_grip);
else if (part == "l298") retaining_pin(l298_grip);
else if (part == "bts") retaining_pin(bts_grip);
else if (part == "washer") locking_clip();
else if (part == "washers") washers_plate();
else if (part == "pins") pins_plate();
else print_plate();

assert(bts_spacer_h == 20, "separate BTS spacer height must remain 20 mm");
assert(shaft_d < m3_slot_w, "pin shaft must pass through tray cross-slot");
assert(clip_od > board_adjust_span,
       "locking clip must bridge the complete tray adjustment slot");
assert(clip_hole_d > groove_d, "locking clip must enter the pin groove");
assert(clip_mouth_w < groove_d, "locking clip mouth must snap onto the groove");
