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

part = "assembly"; // [assembly,base,pusher,pusher_local,spool_anchor,robot_check,qa_cocked,qa_forward,qa_pos4,qa_ball_cup,qa_ramp,qa_band_hardware]

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
base_outer_y = 66;
rail_front_x = -34;
rail_center_y = 47;
rail_wall_t = 3;
// The printed base channel is 10.9 mm wide (v1: 10 mm runner + 2 x 0.45).
// Pusher v2 keeps the channel and narrows only the runner.
channel_w = 10.9;
runner_clearance = 0.75;
runner_w = channel_w - 2 * runner_clearance;   // 9.4
rail_floor_t = 3;
rail_wall_h = 15;

// Ball and pusher handoff datums.  At the forward stop the ball reaches the
// raised runway end and just touches the start of the active ski flat.  When
// cocked, the ball moves back by the same stroke as the pusher and stays
// seated in its spherical cup.
ball_r = ball_d / 2;
ball_center = [-10, 0, ramp_surface_z + ball_r];
strike_origin_x = ball_center[0] - ball_r;
pusher_runner_x0 = -24;  // rear end 1 mm inside the channel at position 4
pusher_runner_x1 = 7.95;  // 0.05 mm modelled gap to the stop face (contact by design)
pusher_pin_x = -12;
pusher_band_x = -4;

// Four repeatable pull-back settings.  Anchor-axis spacing is 24 mm at the
// stop and 44/64/84/104 mm at positions 1-4; size the loops so position 4
// stays below ~3x stretch (see README).
cocking_strokes = [20, 40, 60, 80];
pin_d = 4.4;             // 4 mm smooth steel rod or straight pin
band_hole_d = 5.5;       // base M5 anchor-bolt clearance (base side only)
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
    // Intentionally empty.  The former engraved digits generated 0.08 mm
    // horizontal knife edges.  The four 20 mm-spaced holes remain unambiguous
    // and are numbered in the assembly/use instructions.
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
            translate([xx, side*(base_outer_y-5), -eps])
                linear_extrude(height=rail_floor_t + 2*eps)
                    rounded_slot_2d(mount_slot_len, mount_slot_w);

        base_labels_cut();
    }
}

// ---------------------------------------------------------------------------
// Pusher v2 (2026-11-26).  Only the pusher is reprinted; the base above is
// frozen.  Changes against v1: integrated J-hook band anchors (no M5, no
// spools), 0.75 mm runner clearance, 32 mm runners, 0.6 mm elephant-foot and
// runner-bottom chamfers, and a 0.1 mm modelled floor gap so the Ø4.4 pin hole
// stays aligned when the pusher physically rests on the channel floor.
//
// Print frame: X-up, rear face (x = pusher_runner_x0) on the bed, cup facing
// up.  Every outboard feature is a prism standing on that bed face; the only
// unsupported spans are short bridges between bed-standing walls.
bed_chamfer = 0.6;          // elephant foot at the bed face
runner_chamfer = 0.6;       // runner bottom long edges (channel floor corners)
runner_floor_gap = 0.1;     // modelled; < 0.2 mm radial pin slack
runner_z0 = rail_floor_t + runner_floor_gap;
runner_z1 = 15.5;
runner_y0 = rail_center_y - runner_w/2;   // 42.30
runner_y1 = rail_center_y + runner_w/2;   // 51.70

// J-hook.  The bar axis stays level with the fixed base anchor (Z24) so the
// band is horizontal over the whole stroke and puts no lift on the pusher in
// the open-top channel (see README, "Ύψος έλξης").
hook_bar_r = 3.5;           // Ø7 latex seat
hook_tip_r = 1.5;           // rounded rear corners (latex contact, >= 1.5)
hook_flat = 2;
// Corner-circle centre giving exactly 45-degree flanks.
hook_tip_x = hook_flat/2 - (hook_bar_r - hook_tip_r) * sqrt(2);
hook_rear_x = hook_tip_x - hook_tip_r;         // -3.33 from the bar axis
hook_lip = 1.5;             // lip height beyond the bar on both side walls
hook_y0 = 55.5;             // inner lip face = base spool inner flange face
hook_zone = 7.5;            // band zone; base spool zone is 7.0
hook_y1 = hook_y0 + hook_zone;
hook_curl_t = 2.5;          // bed stem; 4 mm where it laps the bar
hook_curl_head_t = 4;
ear_x1 = 1.5;               // front of the over-wall ear
// Lip face must cover the bar rear apex (x -8.2) plus a lip.
ear_full_x0 = pusher_band_x + hook_rear_x - hook_lip
              - (hook_y0 - (rail_center_y + runner_w/2));
ear_z0 = rail_floor_t + rail_wall_h + 1;  // 1 mm over the Z18 guide walls
ear_z1 = band_axis_z + hook_bar_r + hook_lip;   // 29

bridge_x0 = -12.5;
bridge_x1 = -0.5;
bridge_z1 = ear_z0 + 6;
carrier_x0 = bridge_x1;     // 2.1 mm behind the cup flat bottom
carrier_x1 = 4;            // cup 2.4 mm deep beyond its flat (v1: 4.4)
cup_rim_r = sqrt(pow(ball_d/2 + 0.25, 2) - pow(ball_r - carrier_x1, 2));
carrier_half_w = ceil(cup_rim_r + 1.7);        // >= 1.6 mm wall at the rim
carrier_z0 = ear_z0;       // carries the bridge arms
carrier_z1 = ceil(ball_center[2] + cup_rim_r + 1.7);
gusset_t = 2.0;

// 2D profile (u = Y, v = Z) extruded along +X from x0 to x1.  When x0 is the
// bed face the first `c` mm shrink by `c` (45-degree elephant-foot chamfer).
// Children must be convex.
module prism_x(x0, x1, c=0) {
    multmatrix([[0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]])
        hull() {
            if (c > 0)
                translate([0, 0, x0]) linear_extrude(height=0.01)
                    offset(delta=-c) children();
            translate([0, 0, x0 + c]) linear_extrude(height=x1 - x0 - c)
                children();
        }
}

module runner_profile() {
    c = runner_chamfer;
    polygon([[runner_y0 + c, runner_z0], [runner_y1 - c, runner_z0],
             [runner_y1, runner_z0 + c], [runner_y1, runner_z1],
             [runner_y0, runner_z1], [runner_y0, runner_z0 + c]]);
}

// Hook bar section in the XZ plane (u = X, v = Z): Ø7 with 45-degree flanks
// toward -X (print-down) ending in a 2 mm flat whose two corners are rounded
// r1.5.  The latex wraps exactly this rear half; the flat is the first,
// bridged layer of the bar (>= 1.6 mm wide).
module hook_bar_section() {
    hull() {
        circle(r=hook_bar_r);
        for (v = [-hook_flat/2, hook_flat/2])
            translate([hook_tip_x, v]) circle(r=hook_tip_r);
    }
}

module pusher_side(side=1) {
    x0 = pusher_runner_x0;
    bx = pusher_band_x;
    bz = band_axis_z;
    mirror([0, side < 0 ? 1 : 0, 0]) {
        // Runner (in the channel).
        prism_x(x0, pusher_runner_x1, bed_chamfer) runner_profile();

        // Ear column above the runner: full height only where the hook
        // needs it, tapered at 45 degrees down to the bed behind that.
        // Above the bridge arms only the outer 4.7 mm is kept.
        for (yr = [[runner_y0, bridge_z1], [rail_center_y, ear_z1]])
            hull() {
                prism_x(x0, x0 + 0.01 + bed_chamfer, bed_chamfer)
                    translate([yr[0], runner_z1 - 0.01])
                        square([runner_y1 - yr[0], max(0.01, yr[1] - runner_z1
                                - (ear_full_x0 - x0 - bed_chamfer))]);
                translate([ear_full_x0, yr[0], runner_z1 - 0.01])
                    cube([ear_x1 - ear_full_x0, runner_y1 - yr[0],
                          yr[1] - runner_z1 + 0.01]);
            }

        // Over-wall ear, grown outward at 45 degrees from the column; its
        // outer face is the inner hook lip.  Front outer edge rounded r2
        // where the band strands run past it.
        hull() {
            translate([ear_full_x0, runner_y1 - 0.01, ear_z0])
                cube([0.01, 0.01, ear_z1 - ear_z0]);
            translate([ear_full_x0 + (hook_y0 - runner_y1), runner_y1 - 0.01,
                       ear_z0])
                cube([ear_x1 - 2 - ear_full_x0 - (hook_y0 - runner_y1),
                      hook_y0 - runner_y1 + 0.01, ear_z1 - ear_z0]);
            translate([ear_x1 - 2, runner_y1 - 0.01, ear_z0])
                cube([2, hook_y0 - 2 - runner_y1 + 0.01, ear_z1 - ear_z0]);
            translate([ear_x1 - 2, hook_y0 - 2, ear_z0])
                cylinder(r=2, h=ear_z1 - ear_z0);
        }

        // Hook bar: a short bridge between the ear lip and the curl.
        translate([bx, hook_y0 - 0.01, bz]) rotate([-90, 0, 0])
            linear_extrude(height=hook_zone + 0.02) hook_bar_section();

        // Curl / outer lip: free J tip standing on the bed face.  The loop is
        // slipped over its rear end, slid forward and seats on the bar; under
        // tension it cannot walk back off.
        // Inner front edge rounded r1.5 (strands run past it); the head is
        // 4 mm thick so its last print layers stay >= 25 mm2 / 1.6 mm.
        curl_x1 = bx + hook_bar_r + hook_lip;
        curl_head_x0 = bx + hook_rear_x - hook_lip;
        hull() {
            prism_x(x0, curl_head_x0 - (hook_curl_head_t - hook_curl_t),
                    bed_chamfer)
                translate([hook_y1, bz - hook_bar_r - hook_lip])
                    square([hook_curl_t, 2 * (hook_bar_r + hook_lip)]);
            translate([curl_head_x0, hook_y1, bz - hook_bar_r - hook_lip])
                cube([curl_x1 - 1.5 - curl_head_x0, hook_curl_head_t,
                      2 * (hook_bar_r + hook_lip)]);
            translate([curl_x1 - 1.5, hook_y1 + 1.5, bz - hook_bar_r - hook_lip])
                cylinder(r=1.5, h=2 * (hook_bar_r + hook_lip));
            translate([curl_x1 - 0.01, hook_y1 + 1.5,
                       bz - hook_bar_r - hook_lip])
                cube([0.01, hook_curl_head_t - 1.5,
                      2 * (hook_bar_r + hook_lip)]);
        }

        // Gusset under the cup carrier (bed-standing, <= 45 degrees).
        hull() {
            prism_x(x0, x0 + 2, bed_chamfer)
                translate([carrier_half_w - gusset_t, ear_z0])
                    square([gusset_t, carrier_z1 - (carrier_x0 - 2 - x0 - 2) - ear_z0]);
            translate([carrier_x0 - 2, carrier_half_w - gusset_t, ear_z0])
                cube([2, gusset_t, carrier_z1 - ear_z0]);
        }
    }
}

module pusher_raw() {
    pusher_side(1);
    pusher_side(-1);

    // Two bridge arms from the ear columns into the cup carrier, 1 mm above
    // the inner guide walls.  Each underside bridges gusset -> column.
    for (side = [-1, 1])
        translate([bridge_x0, side > 0 ? carrier_half_w - 5 : -(runner_y0 + 1),
                   ear_z0])
            cube([bridge_x1 - bridge_x0, runner_y0 + 1 - carrier_half_w + 5,
                  bridge_z1 - ear_z0]);

    // Cup carrier.
    translate([carrier_x0, -carrier_half_w, carrier_z0])
        cube([carrier_x1 - carrier_x0, 2 * carrier_half_w,
              carrier_z1 - carrier_z0]);
}

module pusher_part() {
    difference() {
        pusher_raw();

        // Shallow spherical cup matching the ball at the forward stop.  Clip
        // its theoretical point to a flat bottom (no knife edge).
        intersection() {
            translate([ball_r, 0, ball_center[2]]) sphere(d=ball_d + 0.5);
            translate([20.8, 0, ball_center[2]])
                cube([38.4, 80, 80], center=true);
        }

        // Cross release pin.
        translate([pusher_pin_x, 0, 9])
            bore_y(pin_d, 2*rail_center_y + 18);
    }
}

module band_preview(side=1, stroke=60) {
    // Visual-only elastic centreline, base spool -> pusher hook bar.
    color([0.9, 0.15, 0.12])
        hull() {
            translate([anchor_x, side*(hook_y0 + hook_zone/2), anchor_hole_z])
                sphere(d=3);
            translate([strike_origin_x-stroke+pusher_band_x,
                       side*(hook_y0 + hook_zone/2), band_axis_z]) sphere(d=3);
        }
}

// Base anchor hardware that stays: M5 + washers + nyloc + printed spool.
module base_anchor_hardware_envelope() {
    for (side=[-1,1])
        translate([anchor_x, side*51.5, band_axis_z]) bore_y(12, 37);
}

module band_hardware_preview(stroke=60) {
    color("silver") base_anchor_hardware_envelope();
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

// Band envelope (one 1.5 mm layer on the Ø9 base spool and on the Ø7 hook
// bar) must never meet the frozen base; the pusher hook must never meet the
// base anchor hardware; the band must clear the pusher apart from the bar.
module band_envelope(stroke) {
    for (side=[-1,1])
        hull() {
            translate([anchor_x, side*(hook_y0 + hook_zone/2), anchor_hole_z])
                bore_y(9 + 3, hook_zone - 0.2);
            translate([strike_origin_x-stroke+pusher_band_x,
                       side*(hook_y0 + hook_zone/2), band_axis_z])
                bore_y(2*hook_bar_r + 3, hook_zone - 0.2);
        }
}

module band_hardware_clearance_check(stroke=80) {
    for (s = [0, stroke]) {
        intersection() { band_envelope(s); base_part(); }
        intersection() {
            translate([strike_origin_x-s, 0, 0]) pusher_part();
            base_anchor_hardware_envelope();
        }
        // Band vs pusher, excluding the bar wrap zone (x within the bar).
        intersection() {
            band_envelope(s);
            difference() {
                translate([strike_origin_x-s, 0, 0]) pusher_part();
                translate([strike_origin_x-s+pusher_band_x-20, -100, 0])
                    cube([20 + hook_bar_r + 0.01, 200, 100]);
            }
        }
    }
}

assert((rail_center_y-channel_w/2-rail_wall_t) > ball_d/2,
       "rail inner walls must clear the tennis ball");
// Printed spool for the exposed part of each base M5 band bolt, so the latex
// never touches thread or a PETG edge.  The flanges keep the loop centred
// between the washers.  Length fills the exposed M5x30 shank: 30 minus the
// 12 mm anchor lug, the inner washer/nyloc (6) and one 1 mm washer on each
// side of the spool (the anchor lug face is sloped, so the spool must never
// bear on it directly).  The v1 pusher spool is obsolete (pusher v2 hooks).
spool_bore = 5.5;
spool_core_d = 9;
spool_flange_d = 11;   // bottom stays above the Z18 rail wall it slides past
spool_flange_t = 1.5;
spool_anchor_len = 10;

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
assert(strike_origin_x - cocking_strokes[3] + pusher_runner_x0 >= base_x0,
       "runner rear must stay inside the channel at position 4");
assert(ear_z0 - hook_lip - hook_bar_r <= band_axis_z - 2*hook_bar_r + 0.01,
       "hook lips must be deeper than the bar");
assert(ear_z0 > rail_floor_t + rail_wall_h,
       "over-wall ear and bridge must clear the Z18 guide walls");
assert(hook_y0 > rail_center_y + channel_w/2 + rail_wall_t,
       "band zone must lie outboard of the outer guide wall face");

if (part == "assembly") fixture_assembly(stroke=60);
else if (part == "base") base_part();
else if (part == "pusher")
    // X-up: rear face (runners, ears, hook curls, gusset roots) on the bed,
    // concave cup facing up.  Support-free; see pusher v2 notes above.
    translate([0, 0, -pusher_runner_x0]) rotate([0, -90, 0]) pusher_part();
else if (part == "pusher_local") pusher_part();
else if (part == "spool_anchor") band_spool(spool_anchor_len);
else if (part == "robot_check") robot_check();
else if (part == "qa_cocked") slide_clearance_check(60);
else if (part == "qa_forward") slide_clearance_check(0);
else if (part == "qa_pos4") slide_clearance_check(80);
else if (part == "qa_ball_cup") ball_cup_clearance_check(60);
else if (part == "qa_ramp") active_ramp_clearance_check();
else if (part == "qa_band_hardware") band_hardware_clearance_check(80);
