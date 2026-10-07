// Two-piece detachable cable crossbar between the cheek platforms of the
// independent Gamma brackets. Units: mm.
//
// Both parts use the existing two M5 cheek holes on their side. They sit on
// the upper face of the Gamma strips while the existing cheek flange remains
// below the strips, so the cheek datum is not moved. The local printable
// orientation has the mounting face on the bed.

$fn = 48;

part = "assembly"; // [assembly,crossbar_only,left_print,right_print,left_world,right_world,cover_print,cover_world]

// Frozen interface from dual-outboard-gamma-brackets.scad.
cheek_hole_x = [475, 495];
cheek_hole_y = 132;
cheek_bearing_z = 150;
gamma_plate_t = 8;

// Cable duct sits behind the front cheek hole while the mounting ears reach
// both holes. Installed span is 284 mm; each printable half is below 180 mm.
duct_center_x = 479;
duct_half_y = 122;
duct_w = 20;
duct_h = 14;
top_t = 4;
wall_t = 3;
divider_t = 1.6;
outer_entry_clear_len = 15; // accepts the approximately 13 mm motor connector

tab_x0 = 468;
tab_x1 = 502;
tab_y0 = 122;
tab_y1 = 142;
bolt_clear_d = 5.7;

joint_overlap = 36;
joint_gap = 0.5;
joint_tongue_w = 10;
joint_ridge_w = 2.4;
joint_lock_d = 3.4; // M3 through-bolt clearance
joint_lock_y = 34;
joint_lock_z = 11.5; // high in the channel, leaving wiring below it

// Both cable runs terminate in this open-top centre breakout bay.
exit_bay_right_y = -24;
exit_bay_left_y = 36;

retainer_t = 3;
retainer_y = [48, 88];

// Removable one-piece cover. It is intentionally shorter than the 244 mm
// duct, leaving 3 mm exposed at both cable-entry ends. Five short clips per
// side flex independently; a continuous PETG-CF skirt would be too stiff.
cover_len = 238;
cover_top_t = 2.4;
cover_clearance = 0.30;
cover_side_t = 2.0;
cover_clip_h = 5.0;
cover_bead = 0.55;
cover_clip_len = 12;
cover_clip_y = [-100, -65, -32, 65, 100];
cover_outer_w = duct_w + 2*(cover_clearance+cover_side_t);

// Two upward cable exits, one over each divided lane. Connectors enter from
// the uncovered outer ends before the cover is clipped in place.
cover_exit_slot_w = 6;
cover_exit_slot_len = 48;
cover_exit_slot_x = [-5.2, 5.2];

// Local coordinates: X=fore/aft, Y=robot left/right, Z=up from the Gamma
// strip's upper face. This is also the support-free print orientation.
module outer_duct(y0, y1, retainers=[]) {
    union() {
        translate([-duct_w/2, y0, 0])
            cube([duct_w, y1-y0, top_t]);

        for (xx = [-duct_w/2, duct_w/2-wall_t])
            translate([xx, y0, top_t])
                cube([wall_t, y1-y0, duct_h-top_t]);

        // Short bridges retain the cables in the installed open-top channel.
        for (yy = retainers)
            translate([-duct_w/2, yy-retainer_t/2, duct_h-retainer_t])
                cube([duct_w, retainer_t, retainer_t]);
    }
}

module cable_divider(y0, y1) {
    // Shallow divider separates motor-power wiring from encoder signals. It
    // stops before the centre so both paths can turn upward through the outlet.
    translate([-divider_t/2, y0, top_t])
        cube([divider_t, y1-y0, duct_h-top_t-3]);
}

module mounting_tab(side=1) {
    difference() {
        translate([tab_x0-duct_center_x,
                   side > 0 ? tab_y0 : -tab_y1,
                   0])
            cube([tab_x1-tab_x0, tab_y1-tab_y0, top_t]);

        for (hx = cheek_hole_x)
            translate([hx-duct_center_x, side*cheek_hole_y, -1])
                cylinder(d=bolt_clear_d, h=top_t+2);
    }
}

module joint_lock_hole() {
    translate([-duct_w/2-1, joint_lock_y, joint_lock_z])
        rotate([0, 90, 0])
            cylinder(d=joint_lock_d, h=duct_w+2);
}

module left_local() {
    difference() {
        union() {
            // Positive-Y half is the female half of the centre joint.
            outer_duct(0, duct_half_y, retainer_y);
            cable_divider(exit_bay_left_y,
                          duct_half_y-outer_entry_clear_len);
            mounting_tab(1);
        }
        // Receiving slot for the flat, bed-printed male tongue. The slot also
        // removes the divider locally while retaining both outer walls.
        translate([-(joint_tongue_w/2+joint_gap), -0.1, -1])
            cube([joint_tongue_w+2*joint_gap,
                  joint_overlap+joint_gap+0.1, duct_h+2]);
        joint_lock_hole();
    }
}

module male_tongue() {
    // Flat tongue starts directly on the bed. Its central ridge controls twist
    // and carries the M3 lock without creating a floating print cantilever.
    translate([-joint_tongue_w/2, 0, 0])
        cube([joint_tongue_w, joint_overlap, top_t]);
    translate([-joint_ridge_w/2, 0, top_t])
        cube([joint_ridge_w, joint_overlap, duct_h-top_t]);
}

module right_local() {
    difference() {
        union() {
            outer_duct(-duct_half_y, 0, [-retainer_y[1], -retainer_y[0]]);
            cable_divider(-duct_half_y+outer_entry_clear_len,
                          exit_bay_right_y);
            mounting_tab(-1);
            male_tongue();
        }
        joint_lock_hole();
    }
}

module rounded_slot(cx, slot_w, slot_len, h) {
    hull()
        for (yy = [-(slot_len-slot_w)/2, (slot_len-slot_w)/2])
            translate([cx, yy, -1]) cylinder(d=slot_w, h=h+2);
}

module cover_clip_segment(side=1, yy=0) {
    inner_x = duct_w/2 + cover_clearance;
    wall_x0 = side > 0 ? inner_x : -inner_x-cover_side_t;

    union() {
        translate([wall_x0, yy-cover_clip_len/2, cover_top_t])
            cube([cover_side_t, cover_clip_len, cover_clip_h]);

        // Shallow inward bead retains the lid below the duct-wall shoulder.
        translate([side > 0 ? inner_x-cover_bead : -inner_x,
                   yy-cover_clip_len/2,
                   cover_top_t+cover_clip_h-1.0])
            cube([cover_bead, cover_clip_len, 1.0]);
    }
}

module cover_print() {
    difference() {
        union() {
            translate([-cover_outer_w/2, -cover_len/2, 0])
                cube([cover_outer_w, cover_len, cover_top_t]);

            for (side = [-1, 1], yy = cover_clip_y)
                cover_clip_segment(side, yy);
        }

        for (cx = cover_exit_slot_x)
            rounded_slot(cx, cover_exit_slot_w,
                         cover_exit_slot_len, cover_top_t);
    }
}

module cover_local_installed() {
    // The broad outside face prints on the bed; flip it only for assembly.
    translate([0, 0, duct_h+cover_clearance+cover_top_t])
        mirror([0, 0, 1]) cover_print();
}

module cover_world() {
    color([0.18, 0.62, 0.88, 0.94])
        translate([duct_center_x, 0, cheek_bearing_z+gamma_plate_t])
            cover_local_installed();
}

module left_world() {
    translate([duct_center_x, 0, cheek_bearing_z+gamma_plate_t])
        left_local();
}

module right_world() {
    translate([duct_center_x, 0, cheek_bearing_z+gamma_plate_t])
        right_local();
}

module cheek_crossbar_world() {
    color([0.12, 0.48, 0.82, 0.92]) {
        left_world();
        right_world();
    }
}

module cheek_crossbar_with_cover_world() {
    cheek_crossbar_world();
    cover_world();
}

if (part == "left_print") left_local();
else if (part == "right_print") right_local();
else if (part == "left_world") left_world();
else if (part == "right_world") right_world();
else if (part == "cover_print") cover_print();
else if (part == "cover_world") cover_world();
else if (part == "crossbar_only") cheek_crossbar_world();
else cheek_crossbar_with_cover_world();

assert(2*tab_y1 == 284, "installed cheek-crossbar span must remain 284 mm");
assert(duct_half_y + tab_y1-tab_y0 + joint_overlap < 180,
       "each crossbar half must remain below 180 mm for easy P2S placement");
assert(tab_x0 <= min(cheek_hole_x)-bolt_clear_d/2 &&
       tab_x1 >= max(cheek_hole_x)+bolt_clear_d/2,
       "mounting tabs must surround both cheek holes");
assert(cover_len < 256 && cover_outer_w < 256,
       "one-piece cover must fit the P2S bed");
assert(cover_exit_slot_x[1]+cover_exit_slot_w/2 < duct_w/2,
       "cover cable exits must remain inside the duct walls");
