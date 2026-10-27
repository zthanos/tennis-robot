// External panel bases and appearance study for the low flywheel baseline.
// Envelope only: no manufacturing bends, fastener pitch or material choice.
// Units: mm, ground frame, robot +X forward, +Y left.

use <robot-integration.scad>

$fn = 64;

mode = "launch"; // "collect", "launch" or "both"
design_variant = "performance_v1"; // "commercial" or "performance_v1"
shell_profile = "uniform"; // "uniform" baseline or "stepped" comparison
shell_style = "rounded"; // "rounded" baseline or "faceted" comparison
lidar_mount_style = "upper_frame";
appearance_mode = true; // clean commercial view; false exposes engineering detail
show_robot_context = true;
show_fixed_panels = true;
show_moving_cowl = false; // internal/stepped comparison; not needed outside
show_top_hatches = true;
show_handle_access_hatch = true;
handle_access_hatch_open = false;
show_basket_windows = true;
show_front_mask = true;
show_panel_mounts = !appearance_mode;
show_service_keepouts = !appearance_mode;
show_height_datums = false;

panel_t = 3;
panel_clearance = 6;
panel_mount_hole_d = 5.6; // provisional M5 clearance
shell_alpha = appearance_mode ? 1.0 : 0.70;
upper_shell_color = "#DCE7EC";
// Reserved for a future control-surface tone. Deliberately not applied to any
// roof panel: the handle hatch is differentiated by its dark gasket ring, and
// #C9DCE5 against #DCE7EC is too close to read as hierarchy on its own.
upper_hatch_color = "#C9DCE5";
lower_belt_color = "#28333B";
service_panel_color = "#52616B";
basket_glazing_color = "#263B46";
basket_window_trim_color = "#1F2A30";
front_mask_color = "#202B31";
lower_belt_top_z = 190; // aligns the technical belt with the OAK-D lower edge
body_corner_r = 55;
rear_curve_depth = 45;
rear_mount_inset = 30; // pulls the aft supports inside the side-skin datum
rear_taper_start_x = -260;
rear_support_x = -438;
hatch_corner_r = 28;
service_door_corner_r = 18;
// Reduced from 390 x 184. At the old size this was the largest single element
// on the robot and it showed the one thing worth hiding: mechanism. The ball
// port now carries the "what is inside" story, so this becomes a modest slot
// with the port's aspect ratio (~2.9:1) and a shared rear datum at X=19.
basket_window_center = [164, 310];
basket_window_size = [290, 100];
basket_window_cutout_size = [304, 114];
basket_window_corner_r = 18; // aperture family, same as the ball port
// Deliberate inverse of the ball port: near-clear where a yellow load should
// read, heavily smoked where carriage and gas spring should not.
basket_glazing_alpha = 0.66;
front_mask_center_z = 304;
front_mask_size = [300, 270]; // Y/Z
front_mask_corner_r = 42;
handle_access_center = [220, 0];
handle_access_size = [110, 170]; // X/Y; one-piece on a 220 x 220 bed
handle_access_gap = 3;
handle_access_corner_r = 18;
// The hatch folds flat onto the roof. Any intermediate angle raises its
// free edge through the LiDAR scan plane, so there is no usable
// half-open position; see handle_access_lidar_clearance below.
handle_access_hinge_open_deg = 175;

// Roof grid. Panel edges are derived from the fixed subframe, not chosen for
// looks: fixed_panel_subframe() runs longitudinal rails at y=+/-268 and
// transverse members at x=-438 and x=405. roof_inset puts every panel edge on
// the y=+/-268 rail, and the two roof joints sit over transverse members.
roof_inset = 14;
roof_shut_gap = 3;
roof_joint_mid_x = -22;    // needs a new transverse member; see subframe
roof_joint_front_x = 405;  // existing transverse member
roof_panel_corner_r = 36;
performance_window_rake_deg = 10; // design hypothesis within the 8-12 deg range
performance_accent_color = "#D7FF16";
launcher_ring_w = 8; // visual starting value; deliberately not a hard constraint
launcher_recess_outer_d = 164;
// Real conical recess on the 20 degree axis instead of a flat annulus printed
// on the fascia. The throat equals the validated 116 mm opening, so the recess
// never reduces launcher clearance; the cone only widens outward from there.
launcher_recess_depth = 42;
launcher_recess_wall = 4;

// Ball-level port, low in the graphite belt. The smoked basket window spans
// z=221...419 while a full 45-ball load only reaches about z=150, so that
// window can never show a ball. This one looks straight at the load.
show_ball_ports = true;
ball_port_center = [109, 140];      // X/Z in each side skin
// X chosen so the clear opening starts at X=19, the same rear datum as
// the basket window above it. The two apertures share that edge and an
// aspect ratio of about 2.9:1.
ball_port_size = [180, 62];
ball_port_cutout_size = [192, 74];
ball_port_corner_r = 18;
ball_port_glazing_color = "#C6CFC8";

// The tapered side blade reads as a vehicle stripe, which is the wrong product
// category for a court trainer. Kept switchable rather than deleted.
show_side_accent = false;

// Fixed lower shell stays below the basket opening and below the LiDAR plane.
side_skin_y = 282;
fixed_skin_bottom_z = 58;
// Retained by decision (2026-08-22): the uniform roof stays until every
// mechanical part works, then the profiled-body architecture is re-examined.
// This 463 exists solely to enclose the raised basket rear rim at 439, which
// is itself 250 rim + 100 lift + 89 from the unconfirmed 12 degree launch
// tilt. See the architecture section in the exploration doc.
uniform_shell_top_z = 463; // 35 mm below the 498 mm LiDAR scan datum
fixed_skin_top_z = shell_profile == "uniform" ? uniform_shell_top_z : 250;
rear_skin_x = -456;

// The cowl is attached to the blue lift carriage, not to the basket mesh.
cowl_side_y = 205;
cowl_rear_x = -10;
cowl_front_x = 455;
cowl_bottom_z = 82;
cowl_top_z = 350;
launch_cowl_lift_z = 100;

// Low nose surrounds the launcher laterally but leaves the intake, OAK-D and
// launch axis open through the centre.
nose_side_y = 274;
nose_rear_x = 420;
nose_front_x = 790;
nose_bottom_z = 62;
nose_top_z = shell_profile == "uniform" ? uniform_shell_top_z : 390;
front_center_open_half_y = 112;
option_a_cheek_top_z = 150;
option_a_bridge_top_z = 168;
front_fascia_bottom_z = option_a_bridge_top_z;
front_exit_open_d = 116;
launcher_pitch_ref_deg = 20;
launcher_nip_z_ref = 215;
launcher_origin_x_ref = 560;
front_exit_center_z = launcher_nip_z_ref
                    + (nose_front_x - launcher_origin_x_ref)
                    * tan(launcher_pitch_ref_deg);

lidar_scan_z_ref = 498;
wheel_center_z_ref = 85;
drive_wheel_d_ref = 170;
wheel_arch_d = 205;
launcher_pair_half_width_ref = (200 + 58) / 2 + (200 + 2 * 18) / 2;

wheel_arch_radial_clearance = (wheel_arch_d - drive_wheel_d_ref) / 2;
nose_launcher_lateral_clearance = nose_side_y
                                - launcher_pair_half_width_ref;
raised_cowl_lidar_clearance = lidar_scan_z_ref
                            - (cowl_top_z + launch_cowl_lift_z);
uniform_shell_lidar_clearance = lidar_scan_z_ref - uniform_shell_top_z;
raised_basket_rim_ref = 443.56;
uniform_roof_basket_clearance = uniform_shell_top_z - panel_t
                              - raised_basket_rim_ref;
front_cheek_vertical_clearance = front_fascia_bottom_z
                               - option_a_cheek_top_z;
front_exit_radial_clearance = (front_exit_open_d - 90) / 2;
basket_window_roof_clearance = uniform_shell_top_z
                             - (basket_window_center[1]
                                + basket_window_cutout_size[1] / 2);
basket_window_belt_clearance = (basket_window_center[1]
                                - basket_window_cutout_size[1] / 2)
                               - lower_belt_top_z;
front_mask_roof_clearance = uniform_shell_top_z
                          - (front_mask_center_z + front_mask_size[1] / 2);
front_mask_intake_clearance = front_mask_center_z
                            - front_mask_size[1] / 2
                            - front_fascia_bottom_z;
top_handle_upper_top_z_ref = 328 + 100 + 18 / 2;
top_handle_roof_clearance = uniform_shell_top_z - panel_t
                          - top_handle_upper_top_z_ref;
top_handle_access_depth_low = uniform_shell_top_z
                            - (328 + 18 / 2);
roof_panel_half_y = side_skin_y - roof_inset;
roof_rear_panel_hi = roof_joint_mid_x - roof_shut_gap / 2;
roof_basket_panel_lo = roof_joint_mid_x + roof_shut_gap / 2;
roof_basket_panel_hi = roof_joint_front_x - roof_shut_gap / 2;
nose_roof_lo = roof_joint_front_x + roof_shut_gap / 2;
handle_access_open_edge_z = uniform_shell_top_z + 2
                          + handle_access_size[0]
                            * sin(handle_access_hinge_open_deg);
handle_access_lidar_clearance = lidar_scan_z_ref - handle_access_open_edge_z;

assert(fixed_skin_top_z < lidar_scan_z_ref,
       "fixed body skin must stay below the LiDAR scan plane");
assert(nose_top_z < lidar_scan_z_ref,
       "nose bodywork must stay below the LiDAR scan plane");
assert(cowl_top_z + launch_cowl_lift_z < lidar_scan_z_ref,
       "raised cowl must stay below the LiDAR scan plane");
assert(wheel_arch_radial_clearance >= 15,
       "wheel arches need at least 15 mm radial clearance");
assert(nose_launcher_lateral_clearance >= 20,
       "nose sides need at least 20 mm around the launcher guard envelope");
assert(raised_cowl_lidar_clearance >= 40,
       "raised cowl needs at least 40 mm below the LiDAR scan datum");
assert(uniform_shell_lidar_clearance >= 30
       && uniform_shell_lidar_clearance <= 40,
       "uniform shell should finish 30-40 mm below the LiDAR datum");
assert(uniform_roof_basket_clearance >= 15,
       "uniform roof needs at least 15 mm above the raised basket rim");
assert(front_cheek_vertical_clearance >= 15,
       "front fascia must clear the top of the Option A cheeks");
assert(front_exit_radial_clearance >= 10,
       "front cylinder opening needs at least 10 mm radial clearance");
assert(basket_window_roof_clearance >= 40,
       "basket glazing needs at least 40 mm of upper shell structure");
assert(basket_window_belt_clearance >= 25,
       "basket glazing needs at least 25 mm above the lower belt");
assert(front_mask_roof_clearance >= 20,
       "front mask needs at least 20 mm below the roof edge");
assert(front_mask_intake_clearance >= 0,
       "front mask must not descend into the intake opening");
assert(top_handle_roof_clearance >= 20,
       "raised top handle needs at least 20 mm below the inner roof");
assert(handle_access_size[0] <= 220 && handle_access_size[1] <= 220,
       "handle access hatch must remain a single 220 x 220 print tile");
assert(handle_access_lidar_clearance >= 20,
       "opened handle hatch must fold flat, not stand up through the scan plane");
ball_port_belt_clearance = lower_belt_top_z
                         - (ball_port_center[1] + ball_port_cutout_size[1] / 2);
ball_port_skin_clearance = (ball_port_center[1]
                            - ball_port_cutout_size[1] / 2)
                         - fixed_skin_bottom_z;
ball_port_arch_clearance = (330 - wheel_arch_d / 2)
                         - (ball_port_center[0]
                            + ball_port_cutout_size[0] / 2);

assert(ball_port_belt_clearance >= 10,
       "ball port must stay inside the graphite belt");
assert(ball_port_skin_clearance >= 20,
       "ball port must keep material above the lower skin edge");
assert(ball_port_arch_clearance >= 10,
       "ball port must stop clear of the forward wheel arch");
assert(launcher_recess_depth > 0
       && front_exit_open_d < launcher_recess_outer_d,
       "launcher recess must widen outward from the validated throat");
assert(abs(roof_panel_half_y - 268) <= 9,
       "roof panel edges must land on the y=+/-268 subframe rail");
assert(roof_inset > roof_shut_gap,
       "roof border must be wider than its own shut gap");

module rounded_rect_2d(size_xy, radius) {
    assert(size_xy[0] > 2 * radius && size_xy[1] > 2 * radius,
           "rounded rectangle radius is too large");
    offset(r=radius)
        square([size_xy[0] - 2 * radius,
                size_xy[1] - 2 * radius], center=true);
}

module performance_window_2d(size_xz, radius,
                             rake_deg=performance_window_rake_deg) {
    rake_dx = size_xz[1] * tan(rake_deg);
    half_w = size_xz[0] / 2;
    half_h = size_xz[1] / 2;
    offset(r=radius)
        offset(delta=-radius)
            polygon(points=[
                [-half_w, -half_h],
                [ half_w - rake_dx / 2, -half_h],
                [ half_w + rake_dx / 2,  half_h],
                [-half_w,  half_h]
            ]);
}

module launcher_face_2d() {
    // Local X becomes world Z and local Y remains world Y after rotation.
    // The faceted capsule is visually directional without changing the fascia.
    half_h = front_mask_size[1] / 2;
    offset(r=24)
        offset(delta=-24)
            polygon(points=[
                [-half_h, -108],
                [-half_h,  108],
                [-105,  142],
                [  82,  150],
                [ half_h, 116],
                [ half_h,-116],
                [  82, -150],
                [-105, -142]
            ]);
}

module rounded_panel_xy(center_pos, size_xy, radius, thickness=panel_t,
                        tint="lightsteelblue", alpha=0.72) {
    color(tint, alpha)
        translate(center_pos)
            linear_extrude(height=thickness, center=true)
                rounded_rect_2d(size_xy, radius);
}

module rounded_panel_xz(center_pos, size_xz, radius, thickness=panel_t,
                        tint="lightsteelblue", alpha=0.82) {
    color(tint, alpha)
        translate(center_pos)
            rotate([90, 0, 0])
                linear_extrude(height=thickness, center=true)
                    rounded_rect_2d(size_xz, radius);
}

module rounded_panel_yz(center_pos, size_yz, radius, thickness=panel_t,
                        tint="lightsteelblue", alpha=0.82) {
    color(tint, alpha)
        translate(center_pos)
            rotate([0, 90, 0])
                linear_extrude(height=thickness, center=true)
                    rounded_rect_2d([size_yz[1], size_yz[0]], radius);
}

module basket_window_cutout(side) {
    translate([basket_window_center[0],
               side * side_skin_y,
               basket_window_center[1]])
        rotate([90, 0, 0])
            linear_extrude(height=30, center=true)
                if (design_variant == "performance_v1")
                    performance_window_2d(basket_window_cutout_size,
                                          basket_window_corner_r + 5);
                else
                    rounded_rect_2d(basket_window_cutout_size,
                                    basket_window_corner_r + 5);
}

module performance_belt_overlay(side) {
    color(lower_belt_color, shell_alpha)
        translate([0, side * (side_skin_y + 3.2), 0])
            rotate([90, 0, 0])
                linear_extrude(height=2, center=true)
                    difference() {
                        // Two controlled rises make the belt technical rather
                        // than a continuous vehicle-like stripe.
                        polygon(points=[
                            [390, lower_belt_top_z],
                            [432, 205],
                            [585, 205],
                            [625, 226],
                            [700, 240],
                            [700, lower_belt_top_z]
                        ]);
                        for (xx = [-330, 330])
                            translate([xx, wheel_center_z_ref])
                                circle(d=wheel_arch_d);
                    }
}

module performance_side_accent(side) {
    // Short tapered blade: it follows the window rake and terminates as a
    // visual arrow toward the launcher rather than becoming a body stripe.
    color(performance_accent_color, shell_alpha)
        translate([0, side * (side_skin_y + 4.5), 0])
            rotate([90, 0, 0])
                linear_extrude(height=2.2, center=true)
                    offset(r=2.5)
                        offset(delta=-2.5)
                            polygon(points=[
                                [15, 207],
                                [15, 211],
                                [350, 219],
                                [438, 235],
                                [438, 246],
                                [345, 230]
                            ]);
}

module launcher_recess_shell() {
    // Cone shell on the launch axis. The throat is exactly the validated
    // 116 mm opening and the cone only widens outward, so clearance is
    // untouched. It is deliberately over-extended and then cut by the fascia,
    // which is the only way the mouth rim lands correctly: the cone mouth is
    // perpendicular to the 20 degree axis while the fascia is vertical, so the
    // two planes intersect and a drawn ellipse would read as a crescent.
    over = 1.5;
    inner_d1 = front_exit_open_d;
    outer_d1 = front_exit_open_d + 2 * launcher_recess_wall;
    outer_d2 = outer_d1
             + (launcher_recess_outer_d - outer_d1) * over;
    translate([nose_front_x, 0, front_exit_center_z])
        rotate([0, 90 - launcher_pitch_ref_deg, 0])
            translate([0, 0, -launcher_recess_depth])
                difference() {
                    cylinder(d1=outer_d1, d2=outer_d2,
                             h=launcher_recess_depth * over);
                    translate([0, 0, -1])
                        cylinder(d1=inner_d1,
                                 d2=outer_d2 - 2 * launcher_recess_wall,
                                 h=launcher_recess_depth * over + 2);
                }
}

module fascia_half_space() {
    translate([nose_front_x - 400, 0, front_exit_center_z])
        cube([800, 800, 800], center=true);
}

module fascia_rim_slab() {
    translate([nose_front_x - launcher_ring_w / 2, 0, front_exit_center_z])
        cube([launcher_ring_w, 800, 800], center=true);
}

module performance_launcher_details() {
    // Depth is what reads as a muzzle; a flat annulus printed on the panel does
    // not. The accent is the cone's own rim, cut by the fascia plane.
    color("#11191D", shell_alpha)
        intersection() {
            launcher_recess_shell();
            fascia_half_space();
        }
    color(performance_accent_color, shell_alpha)
        intersection() {
            launcher_recess_shell();
            fascia_rim_slab();
        }
}

module ball_port_2d(size_xz) {
    rounded_rect_2d(size_xz, ball_port_corner_r);
}

module ball_port_cutout(side) {
    translate([ball_port_center[0], side * (side_skin_y + 2),
               ball_port_center[1]])
        rotate([90, 0, 0])
            linear_extrude(height=30, center=true)
                ball_port_2d(ball_port_cutout_size);
}

module ball_ports(side) {
    // Sight-line audit. On -Y only the gas spring (y -186...-214, x 110...240,
    // z 72...118) clips the bottom ~9 mm of the port. On +Y the V2 actuator
    // swept keepout (y 202...268, x -18...258, z 45...253) encloses the whole
    // port opening between the skin at 282 and the basket wall at 146, so if
    // that actuator is ever fitted the +Y port sees hardware, not balls. This
    // is the third independent reason to move the actuator inboard.
    //
    // Trim first, then near-clear glazing: the point is that a yellow load
    // reads against the graphite belt, so this glazing is not smoked.
    color(basket_window_trim_color, shell_alpha)
        translate([ball_port_center[0], side * (side_skin_y + 2.6),
                   ball_port_center[1]])
            rotate([90, 0, 0])
                linear_extrude(height=panel_t + 1, center=true)
                    difference() {
                        ball_port_2d(ball_port_cutout_size);
                        ball_port_2d(ball_port_size);
                    }
    color(ball_port_glazing_color, appearance_mode ? 0.30 : 0.18)
        translate([ball_port_center[0], side * (side_skin_y + 3.4),
                   ball_port_center[1]])
            rotate([90, 0, 0])
                linear_extrude(height=2, center=true)
                    ball_port_2d(ball_port_size);
}

// Rounded plan: smoothly tapered front nose and a deliberately bowed rear.
module rounded_body_plan_2d(inset=0) {
    offset(delta=-inset)
        offset(r=body_corner_r)
            offset(delta=-body_corner_r)
                polygon(points=[
                    [rear_taper_start_x, -282],
                    [700, -282],
                    [790, -205],
                    [790, 205],
                    [700, 282],
                    [rear_taper_start_x, 282],
                    [-370, 274],
                    [rear_support_x, 252],
                    [-476, 210],
                    [-497, 135],
                    [-456 - rear_curve_depth, 0],
                    [-497, -135],
                    [-476, -210],
                    [rear_support_x, -252],
                    [-370, -274]
                ]);
}

module rounded_shell_skin_geometry(z_bottom=fixed_skin_bottom_z,
                                   z_top=fixed_skin_top_z) {
    shell_h = z_top - z_bottom;
    difference() {
        translate([0, 0, z_bottom])
            linear_extrude(height=shell_h)
                rounded_body_plan_2d();
        translate([0, 0, z_bottom - 1])
            linear_extrude(height=shell_h + 2)
                rounded_body_plan_2d(inset=panel_t);

        // Circular wheel arches through both side skins.
        for (xx = [-330, 330])
            translate([xx, 0, wheel_center_z_ref])
                rotate([90, 0, 0])
                    cylinder(d=wheel_arch_d, h=700, center=true);

        // Flush battery/electronics access on both sides.
        for (side = [-1, 1])
            translate([-155, side * side_skin_y, 160])
                cube([250, 36, 180], center=true);

        // A large glazed basket aperture explains the machine's function and
        // breaks the long opaque side without changing the basket envelope.
        if (show_basket_windows)
            for (side = [-1, 1]) basket_window_cutout(side);

        // Low port that looks straight at the stored balls, unlike the window
        // above it, which sits entirely over a full load.
        if (show_ball_ports)
            for (side = [-1, 1]) ball_port_cutout(side);

        // The nose skin stops at the Option A bridge top. Everything below
        // stays open for the curved cheeks and incoming tennis ball.
        translate([(420 + nose_front_x + 30) / 2, 0,
                   (front_fascia_bottom_z + nose_bottom_z) / 2])
            cube([nose_front_x + 30 - 420, 650,
                  front_fascia_bottom_z - nose_bottom_z + 4],
                 center=true);

        // Closed front fascia above the intake has only the launcher exit
        // opening. The OAK-D body mounts externally below this cylinder.
        translate([nose_front_x, 0, front_exit_center_z])
            rotate([0, 90 - launcher_pitch_ref_deg, 0])
                cylinder(d=front_exit_open_d, h=90, center=true);

        // Rear connector/service opening.
        translate([-456 - rear_curve_depth, 0, 146])
            cube([20, 240, 86], center=true);

        // Rounded-looking vertical cooling slots on both rear side skins.
        for (side = [-1, 1], xx = [-420 : 35 : -280])
            translate([xx, side * side_skin_y, 330])
                cube([20, 24, 68], center=true);
    }
}

module rounded_shell_band() {
    // The color break is deliberately simple: a light upper product shell and
    // a dark technical belt around the chassis. Both use the same skin, so the
    // visual treatment costs no internal package volume.
    color(lower_belt_color, shell_alpha)
        rounded_shell_skin_geometry(fixed_skin_bottom_z, lower_belt_top_z);
    color(upper_shell_color, shell_alpha)
        rounded_shell_skin_geometry(lower_belt_top_z, fixed_skin_top_z);

    // Flush rounded service doors sit on the panel bases, not on the battery.
    for (side = [-1, 1])
        rounded_panel_xz([-155, side * (side_skin_y + 2), 160],
                         [238, 168], service_door_corner_r,
                         panel_t, service_panel_color, shell_alpha);

    if (show_basket_windows)
        for (side = [-1, 1]) {
            // Dark trim is slightly larger than the glazing and makes the
            // opening read as an intentional product feature.
            color(basket_window_trim_color, shell_alpha)
                translate([basket_window_center[0],
                           side * (side_skin_y + 1.2),
                           basket_window_center[1]])
                    rotate([90, 0, 0])
                        linear_extrude(height=panel_t + 1, center=true)
                            difference() {
                                if (design_variant == "performance_v1")
                                    performance_window_2d(
                                        basket_window_cutout_size,
                                        basket_window_corner_r + 5);
                                else
                                    rounded_rect_2d(
                                        basket_window_cutout_size,
                                        basket_window_corner_r + 5);
                                if (design_variant == "performance_v1")
                                    performance_window_2d(
                                        basket_window_size,
                                        basket_window_corner_r);
                                else
                                    rounded_rect_2d(
                                        basket_window_size,
                                        basket_window_corner_r);
                            }
            if (design_variant == "performance_v1")
                color(basket_glazing_color,
                      appearance_mode ? basket_glazing_alpha : 0.30)
                    translate([basket_window_center[0],
                               side * (side_skin_y + 3.4),
                               basket_window_center[1]])
                        rotate([90, 0, 0])
                            linear_extrude(height=2, center=true)
                                performance_window_2d(
                                    basket_window_size,
                                    basket_window_corner_r);
            else
                rounded_panel_xz([basket_window_center[0],
                                  side * (side_skin_y + 3.4),
                                  basket_window_center[1]],
                                 basket_window_size,
                                 basket_window_corner_r,
                                 2,
                                 basket_glazing_color,
                                 appearance_mode ? basket_glazing_alpha : 0.30);
        }

    if (show_front_mask)
        color(front_mask_color, shell_alpha)
            difference() {
                translate([nose_front_x + 1.5, 0, front_mask_center_z])
                    rotate([0, 90, 0])
                        linear_extrude(height=panel_t + 1, center=true)
                            if (design_variant == "performance_v1")
                                launcher_face_2d();
                            else
                                rounded_rect_2d([front_mask_size[1],
                                                 front_mask_size[0]],
                                                front_mask_corner_r);
                translate([nose_front_x, 0, front_exit_center_z])
                    rotate([0, 90 - launcher_pitch_ref_deg, 0])
                        cylinder(d=front_exit_open_d,
                                 h=90, center=true);
            }

    if (show_ball_ports)
        for (side = [-1, 1]) ball_ports(side);

    if (design_variant == "performance_v1") {
        for (side = [-1, 1]) {
            performance_belt_overlay(side);
            if (show_side_accent) performance_side_accent(side);
        }
        performance_launcher_details();
    } else
        assert(design_variant == "commercial",
               str("Unknown design_variant: ", design_variant));
}

// Removable roof panel outline: the body plan inset by roof_inset, cut to an X
// band at the two structural joints. Deriving it from the plan keeps a constant
// border, follows the bowed rear instead of overhanging it, and inherits the
// body corner radius rather than adding another one.
module roof_panel_2d(x_lo, x_hi) {
    offset(r=roof_panel_corner_r)
        offset(delta=-roof_panel_corner_r)
            intersection() {
                rounded_body_plan_2d(roof_inset);
                translate([(x_lo + x_hi) / 2, 0])
                    square([x_hi - x_lo, 900], center=true);
            }
}

module roof_opening_2d(x_lo, x_hi) {
    offset(delta=roof_shut_gap) roof_panel_2d(x_lo, x_hi);
}

module rounded_top_system() {
    if (shell_profile == "uniform" && show_top_hatches) {
        // Fixed roof border. Without it the removable panels are the only roof
        // and each side is left open; the border and both panel edges now share
        // the same y=+/-268 rail.
        color(upper_shell_color, shell_alpha)
            translate([0, 0, uniform_shell_top_z])
                linear_extrude(height=panel_t)
                    difference() {
                        intersection() {
                            rounded_body_plan_2d();
                            translate([(-700 + nose_roof_lo) / 2, 0])
                                square([nose_roof_lo + 700, 900], center=true);
                        }
                        roof_opening_2d(-700, roof_rear_panel_hi);
                        roof_opening_2d(roof_basket_panel_lo,
                                        roof_basket_panel_hi);
                    }

        // Rear battery/electronics panel. The LiDAR bore still passes through
        // it; the pod study will shrink that to a fastener/cable pattern.
        color(upper_shell_color, shell_alpha)
            difference() {
                translate([0, 0, uniform_shell_top_z + 2])
                    linear_extrude(height=panel_t, center=true)
                        roof_panel_2d(-700, roof_rear_panel_hi);
                translate([-420, 0, uniform_shell_top_z + 2])
                    cylinder(d=70, h=panel_t + 4, center=true);
            }

        // Basket panel, carrying the handle-access control.
        color(upper_shell_color, shell_alpha)
            difference() {
                translate([0, 0, uniform_shell_top_z + 2])
                    linear_extrude(height=panel_t, center=true)
                        roof_panel_2d(roof_basket_panel_lo,
                                      roof_basket_panel_hi);
                if (show_handle_access_hatch)
                    translate([handle_access_center[0],
                               handle_access_center[1],
                               uniform_shell_top_z + 2])
                        linear_extrude(height=panel_t + 4, center=true)
                            rounded_rect_2d([
                                handle_access_size[0] + 2 * handle_access_gap,
                                handle_access_size[1] + 2 * handle_access_gap
                            ], handle_access_corner_r + handle_access_gap);
            }

        if (show_handle_access_hatch) handle_access_hatch();

        // Full rounded nose roof closes the volume above the flywheel. Its rear
        // edge is now the shut line over the x=405 transverse member.
        color(upper_shell_color, shell_alpha)
            translate([0, 0, uniform_shell_top_z])
                linear_extrude(height=panel_t)
                    intersection() {
                        rounded_body_plan_2d();
                        translate([(nose_roof_lo + 800) / 2, 0])
                            square([800 - nose_roof_lo, 900], center=true);
                    }
    }
}

module handle_access_hatch() {
    hatch_z = uniform_shell_top_z + 2;
    // Thin dark perimeter represents the compressed gasket/visual shut line.
    color("#303A40", shell_alpha)
        translate([handle_access_center[0], handle_access_center[1], hatch_z])
            linear_extrude(height=1.2, center=true)
                difference() {
                    rounded_rect_2d([
                        handle_access_size[0] + 2 * handle_access_gap,
                        handle_access_size[1] + 2 * handle_access_gap
                    ], handle_access_corner_r + handle_access_gap);
                    rounded_rect_2d(handle_access_size,
                                    handle_access_corner_r);
                }

    if (!handle_access_hatch_open)
        rounded_panel_xy([handle_access_center[0],
                          handle_access_center[1], hatch_z + 0.3],
                         handle_access_size, handle_access_corner_r,
                         panel_t, upper_shell_color, shell_alpha);
    else
        // Rear-edge hinge that folds the panel flat onto the basket hatch
        // instead of standing it upright, which would block the LiDAR. No well
        // or collar descends into the basket volume. The finished face rests
        // downward when folded, so the stop needs bumpers on the roof.
        color(upper_shell_color, shell_alpha)
            translate([handle_access_center[0] - handle_access_size[0] / 2,
                       handle_access_center[1], hatch_z])
                rotate([0, -handle_access_hinge_open_deg, 0])
                    translate([handle_access_size[0] / 2, 0, 0])
                        linear_extrude(height=panel_t, center=true)
                            rounded_rect_2d(handle_access_size,
                                            handle_access_corner_r);
}

function fixed_mount_y(x_pos) =
    x_pos <= rear_support_x ? side_skin_y - 8 - rear_mount_inset
    : x_pos < rear_taper_start_x
      ? side_skin_y - 8
        - rear_mount_inset
          * (rear_taper_start_x - x_pos)
          / (rear_taper_start_x - rear_support_x)
      : side_skin_y - 8;

module side_mount_tab(x_pos, side, z_pos, moving=false) {
    y_pos = side * (moving ? cowl_side_y - 8 : fixed_mount_y(x_pos));
    color(moving ? "steelblue" : "goldenrod")
        translate([x_pos, y_pos, z_pos])
            difference() {
                cube([32, 18, 42], center=true);
                rotate([90, 0, 0])
                    cylinder(d=panel_mount_hole_d, h=24, center=true);
            }
}

module fixed_panel_subframe() {
    upper_rail_z = fixed_skin_top_z - 18;
    rear_support_y = side_skin_y - 14 - rear_mount_inset;
    straight_rail_center_x = (rear_taper_start_x + 440) / 2;
    straight_rail_length = 440 - rear_taper_start_x;
    color("dimgray") {
        // The rear part of each rail steps inward gradually. This lets the
        // outer skin form one continuous curve instead of bulging around a
        // full-width aft mounting hoop.
        for (side = [-1, 1], zz = [72, upper_rail_z]) {
            translate([straight_rail_center_x, side * 268, zz])
                cube([straight_rail_length, 18, 18], center=true);
            hull() {
                translate([rear_taper_start_x, side * 268, zz])
                    cube([18, 18, 18], center=true);
                translate([rear_support_x, side * rear_support_y, zz])
                    cube([18, 18, 18], center=true);
            }
        }

        // Rear and front hoops stop the side skins racking independently.
        for (side = [-1, 1])
            translate([rear_support_x, side * rear_support_y,
                       (72 + upper_rail_z) / 2])
                cube([18, 18, upper_rail_z - 72], center=true);
        for (side = [-1, 1])
            translate([405, side * 268, (72 + upper_rail_z) / 2])
                cube([18, 18, upper_rail_z - 72], center=true);
        for (zz = [72, upper_rail_z]) {
            translate([rear_support_x, 0, zz])
                cube([18, 2 * rear_support_y, 18], center=true);
            translate([roof_joint_front_x, 0, zz])
                cube([18, 536, 18], center=true);
        }

        // Transverse member under the rear<->basket roof shut line. Without it
        // both panels end in a free edge across an unsupported void, which a
        // 3 mm printed panel spanning 450 mm cannot carry.
        translate([roof_joint_mid_x, 0, upper_rail_z])
            cube([18, 536, 18], center=true);
    }

    if (show_panel_mounts)
        for (side = [-1, 1], xx = [-420, -235, -35, 190, 390],
             zz = shell_profile == "uniform" ? [92, 255, 425] : [92, 210])
            side_mount_tab(xx, side, zz);
}

module fixed_side_skin(side) {
    panel_h = fixed_skin_top_z - fixed_skin_bottom_z;

    color("lightslategray", 0.72)
        translate([0, side * side_skin_y,
                   fixed_skin_bottom_z + panel_h / 2])
            difference() {
                cube([900, panel_t, panel_h], center=true);

                // Wheel arches retain access to the four external drive pods.
                for (xx = [-330, 330])
                    translate([xx, 0,
                               wheel_center_z_ref
                               - (fixed_skin_bottom_z + panel_h / 2)])
                        rotate([90, 0, 0])
                            cylinder(d=wheel_arch_d,
                                     h=panel_t + 4, center=true);

                // Rear battery/electronics panel is removable as one piece.
                translate([-155, 0,
                           160 - (fixed_skin_bottom_z + panel_h / 2)])
                    cube([250, panel_t + 4, 180], center=true);

                // Cooling slots for the driver/electronics bay.
                for (xx = [-420 : 35 : -280])
                    translate([xx, 0,
                               330 - (fixed_skin_bottom_z + panel_h / 2)])
                        cube([20, panel_t + 4, 68], center=true);
            }

    // Flush removable side service door over the battery/electronics opening.
    color("lightsteelblue", 0.82)
        translate([-155, side * (side_skin_y + 2), 160])
            cube([238, panel_t, 168], center=true);

    // Dark frames show independent removable panel boundaries.
    color("#30343a")
        for (xx = [-255, -25, 255, 448])
            translate([xx, side * (side_skin_y + side * 0.8),
                       (fixed_skin_bottom_z + fixed_skin_top_z) / 2])
                cube([5, panel_t + 1,
                      fixed_skin_top_z - fixed_skin_bottom_z - 20],
                     center=true);
}

module rear_skin() {
    rear_h = fixed_skin_top_z - fixed_skin_bottom_z;
    color("lightslategray", 0.72)
        translate([rear_skin_x, 0,
                   fixed_skin_bottom_z + rear_h / 2])
            difference() {
                cube([panel_t, 548, rear_h], center=true);
                // Central upper notch provides mast/cable service access.
                translate([0, 0, rear_h / 2 - 36])
                    cube([panel_t + 4, 72, 76], center=true);
                // Rear connector/service opening.
                translate([0, 0, -rear_h / 2 + 88])
                    cube([panel_t + 4, 240, 86], center=true);
            }
}

module rear_deck_and_hatch() {
    hatch_z = shell_profile == "uniform" ? uniform_shell_top_z : 242;
    // Fixed narrow frame around a removable top battery/electronics hatch.
    color("slategray", 0.72) {
        translate([-235, 258, hatch_z]) cube([430, 42, panel_t], center=true);
        translate([-235, -258, hatch_z]) cube([430, 42, panel_t], center=true);
        translate([-438, 0, hatch_z]) cube([24, 474, panel_t], center=true);
        translate([-28, 0, hatch_z]) cube([24, 474, panel_t], center=true);
    }
    if (show_top_hatches)
        color("lightsteelblue", 0.72)
            difference() {
                translate([-235, 0, hatch_z + 2])
                    cube([374, 426, panel_t], center=true);
                // The rear-mounted LiDAR mast passes through the hatch edge.
                translate([-420, 0, hatch_z + 2])
                    cylinder(d=70, h=panel_t + 4, center=true);
            }
}

module basket_top_hatch() {
    if (shell_profile == "uniform" && show_top_hatches) {
        // Large removable/hinged hatch. Its 16 mm nominal inner clearance over
        // the raised rim must be validated with a fully loaded physical basket.
        color("powderblue", 0.58)
            translate([220, 0, uniform_shell_top_z + 2])
                cube([440, 440, panel_t], center=true);
        color("slategray", 0.78)
            for (side = [-1, 1])
                translate([220, side * 226, uniform_shell_top_z])
                    cube([454, 12, 12], center=true);
    }
}

module nose_side_fairing(side) {
    // Full-height faceted side blade. It stays wide around the flywheels and
    // tapers inward only near the front camera/exit opening.
    color("lightsteelblue", 0.64)
        hull() {
            translate([nose_rear_x, side * nose_side_y, 115])
                cube([8, panel_t, 106], center=true);
            translate([nose_rear_x, side * nose_side_y, nose_top_z - 8])
                cube([8, panel_t, 16], center=true);
            translate([700, side * nose_side_y, nose_top_z - 8])
                cube([8, panel_t, 16], center=true);
            translate([700, side * nose_side_y, 105])
                cube([8, panel_t, 72], center=true);
            translate([nose_front_x, side * 205, nose_top_z - 8])
                cube([8, panel_t, 16], center=true);
            translate([nose_front_x - 30, side * 205, 105])
                cube([8, panel_t, 72], center=true);
        }

    // Two front fascia wings deliberately leave a central camera/ball opening.
    color("lightslategray", 0.68)
        translate([nose_front_x, side * 175,
                   (nose_bottom_z + nose_top_z) / 2])
            cube([panel_t, 120, nose_top_z - nose_bottom_z], center=true);
}

module nose_top_shoulder(side) {
    if (shell_profile == "uniform" && show_top_hatches)
        color("lightsteelblue", 0.62)
            hull() {
                for (yy = [front_center_open_half_y, nose_side_y])
                    translate([nose_rear_x, side * yy,
                               uniform_shell_top_z + 2])
                        cube([8, 8, panel_t], center=true);
                for (yy = [front_center_open_half_y, 205])
                    translate([nose_front_x, side * yy,
                               uniform_shell_top_z + 2])
                        cube([8, 8, panel_t], center=true);
            }
}

module moving_cowl_for_pose(lift=0, alpha=0.68) {
    cowl_h = cowl_top_z - cowl_bottom_z;

    color("powderblue", alpha)
        for (side = [-1, 1])
            translate([(cowl_rear_x + cowl_front_x) / 2,
                       side * cowl_side_y,
                       cowl_bottom_z + lift + cowl_h / 2])
                difference() {
                    cube([cowl_front_x - cowl_rear_x,
                          panel_t, cowl_h], center=true);
                    // Large hand/service opening; remaining perimeter is a
                    // lightweight removable panel rather than structure.
                    translate([20, 0, 8])
                        cube([275, panel_t + 4, 145], center=true);
                }

    // Rear cap follows the carriage while leaving the basket top open.
    color("powderblue", alpha)
        translate([cowl_rear_x, 0,
                   cowl_bottom_z + lift + cowl_h / 2])
            cube([panel_t, 2 * cowl_side_y, cowl_h], center=true);

    if (show_panel_mounts)
        for (side = [-1, 1], xx = [20, 220, 430],
             zz = [118 + lift, 310 + lift])
            side_mount_tab(xx, side, zz, moving=true);
}

module moving_cowl_context() {
    if (mode == "collect")
        moving_cowl_for_pose(0);
    else if (mode == "launch")
        moving_cowl_for_pose(launch_cowl_lift_z);
    else if (mode == "both") {
        moving_cowl_for_pose(0, 0.18);
        moving_cowl_for_pose(launch_cowl_lift_z, 0.68);
    } else
        assert(false, str("Unknown mode: ", mode));
}

module service_keepouts() {
    if (show_service_keepouts) {
        // Yellow: battery removal path. Green: basket vertical removal path.
        color("gold", 0.16)
            translate([-143, 0, 365]) cube([205, 240, 285], center=true);
        color("mediumseagreen", 0.12)
            translate([220, 0, 520]) cube([440, 330, 520], center=true);
    }
}

module external_panel_system() {
    fixed_panel_subframe();
    if (show_fixed_panels) {
        if (shell_style == "rounded") {
            rounded_shell_band();
            rounded_top_system();
        } else if (shell_style == "faceted") {
            fixed_side_skin(-1);
            fixed_side_skin(1);
            rear_skin();
            rear_deck_and_hatch();
            basket_top_hatch();
            nose_side_fairing(-1);
            nose_side_fairing(1);
            nose_top_shoulder(-1);
            nose_top_shoulder(1);
        } else
            assert(false, str("Unknown shell_style: ", shell_style));
    }
    if (show_moving_cowl) moving_cowl_context();
    service_keepouts();
}

if (show_robot_context) full_robot_context();
external_panel_system();
