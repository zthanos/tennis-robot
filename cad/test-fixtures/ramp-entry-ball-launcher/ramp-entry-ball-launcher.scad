// Removable elastic pusher for repeatable tennis-ball tests at the entrance
// of the active P2-85 R75/75 ski.  The ball starts seated in the
// cocked pusher cup; both accelerate together, then the pusher hits its hard
// stop and the ball continues onto the ramp without an initial impact strike.
//
// The ball travels toward +X in fixture-local coordinates.  In robot_check
// the complete fixture is rotated into the physical robot frame, where the
// ball/ramp travel direction is -X.
//
// This is a short-life supervised test fixture, not a robot mechanism.

part = "assembly"; // [assembly,base,pusher,spool_anchor,spool_pusher,robot_check,qa_cocked,qa_forward,qa_ball_cup,qa_ramp,qa_band_hardware]

$fn = 64;

// Stable interfaces from the active P2-85 R75/75 ski.
ball_d = 66;
ramp_surface_z = 11;          // top of the active flat (CAD nominal)
ramp_front_robot_x = 435.4;   // physical front edge of installed STL
ramp_entry_floor_w = 61;
runway_w = 30;

// Fixture envelope and running fit.
base_x0 = -148;
base_x1 = 0;
base_outer_y = 64;
rail_front_x = -34;
rail_center_y = 47;
rail_wall_t = 3;
runner_w = 10;
runner_clearance = 0.45;
channel_w = runner_w + 2 * runner_clearance;
rail_floor_t = 3;
rail_wall_h = 15;

// Ball and pusher handoff datums.  At the forward stop the ball reaches the
// raised runway end and just touches the start of the active ski flat.  When
// cocked, the ball moves back by the same stroke as the pusher and stays
// seated in its spherical cup.
ball_r = ball_d / 2;
ball_center = [-10, 0, ramp_surface_z + ball_r];
strike_origin_x = ball_center[0] - ball_r;
pusher_runner_x0 = -16;
pusher_runner_x1 = 8;
pusher_pin_x = -12;
pusher_band_x = -4;

// Four repeatable pull-back settings.  Anchor-axis spacing is 24 mm at the
// stop and 44/64/84/104 mm at positions 1-4; size the loops so position 4
// stays below ~3x stretch (see README).
cocking_strokes = [20, 40, 60, 80];
pin_d = 4.4;             // 4 mm smooth steel rod or straight pin
band_hole_d = 5.5;       // M5 anchor-bolt clearance; latex never touches print
anchor_x = -23;
band_axis_z = 24;        // 1 mm clearance below a nominal Ø10 head/washer
anchor_hole_z = band_axis_z;

// Generic temporary mounting slots.  They intentionally do not claim a
// frozen hole pattern: the printed ramp itself has no released mounting holes.
mount_slot_x = [-119, -57];
mount_slot_len = 20;
mount_slot_w = 5.5;

eps = 0.02;

// Ties that cross the ball lane must never stand proud of the runway, whatever
// the measured ramp height: a raised tie is a bump under the rolling ball.
tie_t = min(rail_floor_t, ramp_surface_z);

module rounded_slot_2d(len, width) {
    hull()
        for (xx = [-len/2 + width/2, len/2 - width/2])
            translate([xx, 0]) circle(d=width);
}

module bore_y(d, length) {
    rotate([90, 0, 0]) cylinder(d=d, h=length, center=true);
}

module rail_channel(side=1) {
    yc = side * rail_center_y;
    wall_offset = channel_w/2 + rail_wall_t/2;
    rail_len = rail_front_x - base_x0;

    // U-channel floor.
    translate([(base_x0 + rail_front_x)/2, yc, rail_floor_t/2])
        cube([rail_len, channel_w + 2*rail_wall_t, rail_floor_t], center=true);

    // Inner and outer walls.
    for (dy = [-wall_offset, wall_offset])
        translate([(base_x0 + rail_front_x)/2, yc + dy,
                   rail_floor_t + rail_wall_h/2])
            cube([rail_len, rail_wall_t, rail_wall_h], center=true);
}

module anchor_tower(side=1) {
    translate([anchor_x, side*rail_center_y, 0])
        difference() {
            hull() {
                translate([0, 0, 6]) cube([18, 14, 12], center=true);
                translate([0, 0, anchor_hole_z])
                    cube([16, 12, 16], center=true);
            }
            translate([0, 0, anchor_hole_z]) bore_y(band_hole_d, 18);
        }
}

module base_raw() {
    // Two rails leave a 77 mm clear central lane around the Ø66 mm ball.
    rail_channel(1);
    rail_channel(-1);

    // Rear tie keeps the two channels parallel.
    translate([base_x0 + 5, 0, tie_t/2])
        cube([10, 2*base_outer_y, tie_t], center=true);

    // Continuous raised runway: the ball is accelerated while supported at
    // the same nominal surface height as the active P2-85 flat.  The last
    // 30 mm widen to the ski's 61 mm entry floor for a clean handoff.
    translate([base_x0/2, 0, ramp_surface_z/2])
        cube([-base_x0, runway_w, ramp_surface_z], center=true);
    hull() {
        translate([-30, 0, ramp_surface_z/2])
            cube([1, runway_w, ramp_surface_z], center=true);
        translate([-0.5, 0, ramp_surface_z/2])
            cube([1, ramp_entry_floor_w, ramp_surface_z], center=true);
    }

    // Low cross-ties connect runway and guide rails without entering the
    // ball-contact surface.
    for (xx = [-80, -38])
        translate([xx, 0, tie_t/2])
            cube([6, 2*base_outer_y, tie_t], center=true);

    // Positive front stops.  The pusher runners hit these before the cup can
    // enter the ramp.
    for (side = [-1, 1])
        translate([-31, side*rail_center_y,
                   rail_floor_t + rail_wall_h/2])
            cube([8, channel_w, rail_wall_h], center=true);

    anchor_tower(1);
    anchor_tower(-1);

    // Small outer flanges for tape, straps or temporary screws.  They start
    // 1 mm inside the rail's outer wall (Y55.45) so each fuses with the base
    // instead of printing as a loose island.
    flange_y0 = rail_center_y + channel_w/2 + rail_wall_t - 1;
    for (side = [-1, 1], xx = mount_slot_x)
        translate([xx, side*(flange_y0 + base_outer_y)/2, rail_floor_t/2])
            cube([mount_slot_len+8, base_outer_y - flange_y0, rail_floor_t],
                 center=true);
}

module base_labels_cut() {
    // Engrave the four settings on the outside of the +Y rail.
    outer_face_y = rail_center_y + channel_w/2 + rail_wall_t;
    for (i = [0:len(cocking_strokes)-1]) {
        hx = strike_origin_x - cocking_strokes[i] + pusher_pin_x;
        translate([hx, outer_face_y + 0.2, 7])
            rotate([90, 0, 0])
                linear_extrude(height=0.8)
                    text(str(i+1), size=6, halign="center", valign="center");
    }
}

module base_part() {
    difference() {
        base_raw();

        // One cross-pin hole per cocking position.
        for (stroke = cocking_strokes)
            translate([strike_origin_x - stroke + pusher_pin_x, 0, 9])
                bore_y(pin_d, 2*base_outer_y + 4);

        // Temporary mounting slots in the outer flanges.
        for (side = [-1, 1], xx = mount_slot_x)
            translate([xx, side*(base_outer_y-4), -eps])
                linear_extrude(height=rail_floor_t + 2*eps)
                    rounded_slot_2d(mount_slot_len, mount_slot_w);

        base_labels_cut();
    }
}

module pusher_raw() {
    // Two runners and a bridge.  Runner front faces define the hard stop.
    for (side = [-1, 1]) {
        translate([(pusher_runner_x0+pusher_runner_x1)/2,
                   side*rail_center_y, 9.5])
            cube([pusher_runner_x1-pusher_runner_x0, runner_w, 12], center=true);

        // Raised band ear keeps the latex clear of the guide channel.
        translate([pusher_band_x, side*rail_center_y, 0])
            hull() {
                translate([0, 0, 16]) cube([16, 10, 12], center=true);
                translate([0, 0, band_axis_z]) cube([14, 10, 14], center=true);
            }
    }

    // The bridge passes 1 mm above the 18 mm-high guide walls.
    translate([-6, 0, 23]) cube([12, 2*rail_center_y, 8], center=true);

    // Central cup carrier and two simple gussets.
    translate([-1, 0, ball_center[2]])
        cube([14, 52, 50], center=true);
    for (side = [-1, 1])
        hull() {
            translate([-6, side*22, 18]) cube([10, 8, 8], center=true);
            translate([-5, side*20, ball_center[2]-13])
                cube([10, 8, 10], center=true);
        }
}

module pusher_part() {
    difference() {
        pusher_raw();

        // Shallow spherical cup matching the ball at the forward stop.
        translate([ball_r, 0, ball_center[2]]) sphere(d=ball_d + 0.5);

        // Cross release pin.
        translate([pusher_pin_x, 0, 9])
            bore_y(pin_d, 2*rail_center_y + 18);

        // One band/cord hole on each side.
        for (side = [-1, 1])
            translate([pusher_band_x, side*rail_center_y, band_axis_z])
                bore_y(band_hole_d, 14);
    }
}

module band_preview(side=1, stroke=60) {
    // Visual-only elastic centreline.
    color([0.9, 0.15, 0.12])
        hull() {
            translate([anchor_x, side*rail_center_y, anchor_hole_z])
                sphere(d=3);
            translate([strike_origin_x-stroke+pusher_band_x,
                       side*rail_center_y, band_axis_z]) sphere(d=3);
        }
}

module band_hardware_preview(stroke=60) {
    // Visual/QA envelope for M5 anchor bolts plus Ø10 heads/washers.  The
    // actual latex loop sits on a smooth sleeve outside each printed lug.
    color("silver") for (side=[-1,1]) {
        translate([anchor_x, side*rail_center_y, band_axis_z])
            bore_y(5, 24);
        translate([strike_origin_x-stroke+pusher_band_x,
                   side*rail_center_y, band_axis_z])
            bore_y(5, 24);
    }
}

module fixture_assembly(stroke=60, show_ball=true, show_bands=true) {
    color([0.18, 0.48, 0.82]) base_part();
    color([0.95, 0.55, 0.10])
        translate([strike_origin_x-stroke, 0, 0]) pusher_part();

    if (show_ball)
        color([0.78, 0.9, 0.16])
            translate(ball_center-[stroke, 0, 0]) sphere(d=ball_d);
    if (show_bands) {
        band_preview(1, stroke);
        band_preview(-1, stroke);
        band_hardware_preview(stroke);
    }

    // Visual-only 4 mm release pin at the selected (position 3) hole.
    color("silver")
        translate([strike_origin_x-stroke+pusher_pin_x, 0, 9])
            bore_y(4, 2*base_outer_y + 12);
}

module active_ramp() {
    // Active P2-85 R75/75 ski, already stored in installed robot coordinates.
    color([0.42, 0.72, 0.32, 0.85])
        import("../../modular-test-chassis/stl-ski-mount/r75-75-ski.stl");
}

module fixture_robot_pose(stroke=60, complete=true) {
    // Local +X travel becomes robot -X travel.
    translate([ramp_front_robot_x, 0, 0]) rotate([0, 0, 180])
        if (complete) fixture_assembly(stroke=stroke);
        else base_part();
}

module robot_check() {
    active_ramp();
    fixture_robot_pose();
}

module slide_clearance_check(stroke=60) {
    intersection() {
        base_part();
        translate([strike_origin_x-stroke, 0, 0]) pusher_part();
    }
}

module ball_cup_clearance_check(stroke=60) {
    intersection() {
        translate([strike_origin_x-stroke, 0, 0]) pusher_part();
        translate(ball_center-[stroke, 0, 0]) sphere(d=ball_d);
    }
}

module active_ramp_clearance_check() {
    intersection() {
        active_ramp();
        fixture_robot_pose(60, false);
    }
}

module band_hardware_clearance_check(stroke=60) {
    intersection() {
        // Conservative Ø10 full-length envelope covers bolt heads, washers
        // and a small smooth sleeve around the M5 fastener.
        union() for (side=[-1,1])
            translate([strike_origin_x-stroke+pusher_band_x,
                       side*rail_center_y, band_axis_z])
                bore_y(10, 26);
        union() {
            rail_channel(1);
            rail_channel(-1);
        }
    }
}

assert((rail_center_y-channel_w/2-rail_wall_t) > ball_d/2,
       "rail inner walls must clear the tennis ball");
// Printed spool for the exposed part of each M5 band bolt, so the latex
// never touches thread or a PETG edge.  The flanges keep the loop centred
// between the washers.  Lengths fill the exposed M5x30 shank: 30 minus the
// lug (12 mm anchor, 10 mm pusher), the inner washer/nyloc (6) and one
// 1 mm washer on each side of the spool (the anchor lug face is sloped, so
// the spool must never bear on it directly).
spool_bore = 5.5;
spool_core_d = 9;
spool_flange_d = 11;   // bottom stays above the Z18 rail wall it slides past
spool_flange_t = 1.5;
spool_anchor_len = 10;
spool_pusher_len = 12;

module band_spool(len) {
    difference() {
        union() {
            cylinder(d=spool_core_d, h=len);
            for (z = [0, len - spool_flange_t])
                translate([0, 0, z]) cylinder(d=spool_flange_d, h=spool_flange_t);
        }
        translate([0, 0, -1]) cylinder(d=spool_bore, h=len + 2);
    }
}

assert(strike_origin_x + pusher_runner_x1 <= rail_front_x,
       "runner must hit the front stop before the cup enters the ramp");

if (part == "assembly") fixture_assembly(stroke=60);
else if (part == "base") base_part();
else if (part == "pusher")
    // Cup face on the bed: compact, support-free print orientation.
    translate([0, 0, 16]) rotate([0, -90, 0]) pusher_part();
else if (part == "spool_anchor") band_spool(spool_anchor_len);
else if (part == "spool_pusher") band_spool(spool_pusher_len);
else if (part == "robot_check") robot_check();
else if (part == "qa_cocked") slide_clearance_check(60);
else if (part == "qa_forward") slide_clearance_check(0);
else if (part == "qa_ball_cup") ball_cup_clearance_check(60);
else if (part == "qa_ramp") active_ramp_clearance_check();
else if (part == "qa_band_hardware") band_hardware_clearance_check(80);
