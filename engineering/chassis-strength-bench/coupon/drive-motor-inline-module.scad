// Printable local validation coupon for one 37 mm steel drive-motor mount.
// Units: mm. Print in the shown orientation, rail bottom on the bed.

$fn = 48;

coupon_length = 200;
rail_width = 50;
rail_height = 30;
rail_wall = 3.2;
internal_web = 2.4;

// One non-terminal motor module: the validated drive zone is centred between
// two unobstructed female sockets for separate internal splice sleeves.
drive_zone_length = 70;
splice_socket_length = 65;
left_end_x = -coupon_length/2;
right_end_x = coupon_length/2;
node_center_x = 0;
webbed_length = drive_zone_length;
webbed_center_x = node_center_x;

// The drive zone is flush with the rail top. Its extra material grows inward,
// so it does not raise the bracket or reduce ground clearance.
node_length = drive_zone_length;
node_width = 48;
local_roof_thickness = 14;

// +Y is the wheel/motor/upright side for this right-side coupon. The purchased
// 40 mm foot has one transverse hole row 9 mm from that upright and the second
// 25 mm farther inward. The 30 mm column spacing runs along the rail (X).
motor_side = 1;
mount_foot_width = 40;
mount_hole_x = [node_center_x - 15, node_center_x + 15];
mount_hole_y = [
    motor_side * (mount_foot_width/2 - 34),
    motor_side * (mount_foot_width/2 - 9)
];
mount_clearance_d = 3.4;
nut_across_flats = 6.2;  // widened after physical M3 nut fit test
nut_channel_height = 3.7; // physical fit test requires ~1 mm more Z clearance
nut_corner_diameter = nut_across_flats / cos(30);
// The nut enters with its corners inside the guide. A broad side mouth avoids
// insertion jams, then a shallow taper removes lateral play at the screw axis.
nut_entry_clearance = 0.40;
nut_seat_clearance = 0.20;
nut_entry_width = nut_corner_diameter + nut_entry_clearance;
nut_seat_width = nut_corner_diameter + nut_seat_clearance;
nut_tower_diameter = 17.0;
mount_screw_length = 12.0;     // available M3x12, measured below the head
mount_foot_thickness = 2.0;
mount_washer_thickness = 0.5;
nut_top_cover = 4.0;
screw_tip_clearance = 1.5;

part = "coupon"; // [coupon,right,left,section_preview,nut_fit,nut_fit_plan,inline_plan]

// Universal chassis splice lock. Every 50 x 30 mm module uses the same
// straight sleeve with nuts at +/-30 mm from the joint and +/-12.5 mm across
// the beam. The motor module therefore needs one transverse pair 30 mm inside
// each open end. These holes cross only the top rail wall; the captive nuts
// live in the removable sleeve, not in the motor module.
splice_lock_x = coupon_length/2 - 30;
splice_lock_y = 12.5;
splice_lock_clearance_d = 3.4;

module beam_between_yz(y0, z0, y1, z1, length, thickness, x_center=0) {
    // Hull of two longitudinal bars creates a printable diagonal web.
    hull() {
        translate([x_center - length/2,
                   y0 - thickness/2, z0 - thickness/2])
            cube([length, thickness, thickness]);
        translate([x_center - length/2,
                   y1 - thickness/2, z1 - thickness/2])
            cube([length, thickness, thickness]);
    }
}

module rail_shell() {
    difference() {
        translate([-coupon_length/2, -rail_width/2, 0])
            cube([coupon_length, rail_width, rail_height]);
        // Open ends are intentional: this is a beam coupon, not a sealed box.
        translate([-coupon_length/2 - 1,
                   -rail_width/2 + rail_wall,
                   rail_wall])
            cube([coupon_length + 2,
                  rail_width - 2*rail_wall,
                  rail_height - 2*rail_wall]);
    }
}

module printable_corrugated_web() {
    inner_y = rail_width/2 - rail_wall;
    bottom_z = rail_wall;
    top_z = rail_height - rail_wall;

    // W-shaped cross-section: every roof span is <=22 mm and every web is
    // approximately 50 degrees from horizontal. It also creates closed cells
    // that resist torsion better than a single vertical divider.
    beam_between_yz(-inner_y, bottom_z, -inner_y/3, top_z,
                    webbed_length, internal_web, webbed_center_x);
    beam_between_yz(-inner_y/3, top_z, inner_y/3, bottom_z,
                    webbed_length, internal_web, webbed_center_x);
    beam_between_yz(inner_y/3, bottom_z, inner_y, top_z,
                    webbed_length, internal_web, webbed_center_x);
}

module drive_zone_central_web() {
    // The central blade ties the W cells together throughout the motor zone.
    // Both 65 mm end regions remain completely clear for splice sleeves.
    translate([node_center_x - drive_zone_length/2,
               -internal_web/2,
               rail_wall])
        cube([drive_zone_length,
              internal_web,
              rail_height - 2*rail_wall]);
}

module captive_nut_towers() {
    // These columns make the nut traps real supported features rather than
    // pockets cut into empty beam space. They start on the bottom skin, print
    // without support, and carry clamp/load paths through the rail depth.
    for (x = mount_hole_x, y = mount_hole_y)
        translate([x, y, rail_wall - 0.1])
            cylinder(d=nut_tower_diameter,
                     h=rail_height - 2*rail_wall + 0.2);
}

module flush_drive_reinforcement() {
    // A solid local roof/load spreader whose top is exactly coplanar with the
    // rail. The normal shell roof overlaps it; no external pad is added.
    translate([node_center_x - node_length/2, -node_width/2,
               rail_height - local_roof_thickness])
        cube([node_length, node_width, local_roof_thickness]);
}

module solid_coupon() {
    union() {
        rail_shell();
        printable_corrugated_web();
        drive_zone_central_web();
        captive_nut_towers();
        flush_drive_reinforcement();
    }
}

module mount_holes() {
    blind_bottom_z = rail_height
        + mount_foot_thickness
        + mount_washer_thickness
        - mount_screw_length
        - screw_tip_clearance;

    for (x = mount_hole_x, y = mount_hole_y)
        // Only the required M3x12 envelope is removed. The tower stays solid
        // below the screw tip instead of becoming a near-through bore.
        translate([x, y, blind_bottom_z])
            cylinder(d=mount_clearance_d,
                     h=rail_height - blind_bottom_z + 6);
}

module captive_nut_access() {
    pocket_z = rail_height - nut_top_cover - nut_channel_height/2;
    // Open three-sided nut seat: the full-width rectangular approach removes
    // the near half of a hexagon all the way to the screw axis. Only the two
    // far diagonal faces and the far flat face remain to locate the nut.

    for (x = mount_hole_x, y = mount_hole_y) {
        side = y < 0 ? -1 : 1;

        // With one hex vertex on X, the far face is perpendicular to the
        // insertion direction. The seat has only 0.20 mm total corner
        // clearance, so the nut is centred before the screw reaches it.
        translate([x, y, pocket_z])
            cylinder(d=nut_seat_width, h=nut_channel_height,
                     center=true, $fn=6);

        // Tapered side guide: full corner clearance at the outside mouth,
        // reducing by 0.20 mm at the screw axis. The 0.05 mm overlap prevents
        // a slicer-thin membrane while retaining the three far hex faces.
        if (side > 0)
            translate([0, 0, pocket_z - nut_channel_height/2])
                linear_extrude(height=nut_channel_height)
                    polygon([
                        [x - nut_seat_width/2, y - 0.05],
                        [x + nut_seat_width/2, y - 0.05],
                        [x + nut_entry_width/2, rail_width/2 + 1],
                        [x - nut_entry_width/2, rail_width/2 + 1]
                    ]);
        else
            translate([0, 0, pocket_z - nut_channel_height/2])
                linear_extrude(height=nut_channel_height)
                    polygon([
                        [x - nut_entry_width/2, -rail_width/2 - 1],
                        [x + nut_entry_width/2, -rail_width/2 - 1],
                        [x + nut_seat_width/2, y + 0.05],
                        [x - nut_seat_width/2, y + 0.05]
                    ]);
    }
}

module splice_lock_holes() {
    for (x = [-splice_lock_x, splice_lock_x],
         y = [-splice_lock_y, splice_lock_y])
        translate([x, y, rail_height - rail_wall - 0.1])
            cylinder(d=splice_lock_clearance_d, h=rail_wall + 0.2);
}

module coupon() {
    difference() {
        solid_coupon();
        mount_holes();
        captive_nut_access();
        splice_lock_holes();
    }
}

// Public assembly entry point. side=+1 points the motor toward local +Y;
// side=-1 mirrors the complete validated nut guides and hole pattern.
module drive_motor_inline(side=1) {
    if (side > 0) coupon();
    else mirror([0, 1, 0]) coupon();
}

module nut_fit_coupon() {
    // One corner of the real part, preserving one complete side-loaded nut
    // tower and the actual roof/pad thickness. Print this before the 200 mm
    // coupon when calibrating a new printer or filament.
    intersection() {
        coupon();
        translate([mount_hole_x[0] - 9,
                   -rail_width/2,
                   0])
            cube([18, rail_width/2 - 5,
                  rail_height]);
    }
}

module nut_fit_plan() {
    // Exact horizontal section through the nut guide for quick slicer/CAD QA.
    projection(cut=true)
        translate([0, 0,
                   -(rail_height - nut_top_cover - nut_channel_height/2)])
            nut_fit_coupon();
}

module inline_plan() {
    // Horizontal mid-height section: both sleeve sockets must be visibly open
    // from their outside ends to the central 70 mm drive structure.
    projection(cut=true)
        translate([0, 0, -rail_height/2])
            coupon();
}

if (part == "right") coupon();
else if (part == "left") mirror([0, 1, 0]) coupon();
else if (part == "section_preview")
    intersection() {
        coupon();
        translate([-1, -rail_width, -1])
            cube([2, 2*rail_width, rail_height + 2]);
    }
else if (part == "nut_fit") nut_fit_coupon();
else if (part == "nut_fit_plan") nut_fit_plan();
else if (part == "inline_plan") inline_plan();
else coupon();
