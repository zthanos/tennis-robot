// LiDAR pod v1 — envelope study for the Slamtec RPLIDAR C1 on the shell roof.
// Units: mm, ground frame, robot +X forward, +Y left.
//
// Envelope only: no fastener heads, gasket grooves, draft or wall bosses.
// It exists to fix the pod class (open fairing vs caged), the radial envelope
// and the scan-plane budget before any of that is drawn.
//
// With the cap gone the fairing stops 8 mm below the scan plane, so no pod
// surface sits in the beam and near-field reflection off the pod is no longer a
// concern beyond the top rim itself.

$fn = 96;

// Selected: "open". A cap was considered and rejected: it gives no optical
// benefit, because the scan plane is horizontal and the sunlight that
// actually reaches the receiver arrives near-horizontally through the gap a
// cap must leave open. Its only justification was ball-strike protection.
// "caged" is kept as a comparison in case impact testing reopens that.
pod_variant = "open";
show_sensor = true;
show_roof = true;
show_scan_plane = true;

// ---- Shell datums (mirror external-panel-study.scad; do not diverge) ----
uniform_shell_top_z = 463;
panel_t = 3;
roof_top_z = uniform_shell_top_z + panel_t; // 466
lidar_scan_z = 498;
lidar_x = -420;

// ---- Measured RPLIDAR C1 (hand measurement, +/- ~1 mm) ----
sensor_side = 55;       // square base footprint
sensor_total_h = 43;
sensor_base_h = 23;     // square base section; rotating head above it
// TBD 1: base underside -> centre of the glossy optical band. This is the one
// number that decides whether the pod sits on the roof or sinks into it.
sensor_scan_h = 30;
// Measured: brass insert centre-to-centre, four inserts near the base corners.
insert_pitch_x = 45;
insert_pitch_y = 45;
// Rearmost penetration feature = pitch/2 + fastener clearance radius. This is
// what the rear roof hatch has to stay clear of, not the old 70 mm bracket bore.
fastener_clear_r = 4;
insert_d = 3.4;         // M3 clearance in the pod flange
cable_side_az = 180;    // horizontal cable exit, rear-facing
cable_slot_w = 14;
cable_slot_h = 9;

// ---- Pod ----
pod_side = 80;
pod_corner_r = 16;
pod_wall = 3;
scan_clearance = 8;     // fairing must stop this far below the scan plane
post_r = 45;            // < range_min so post returns are auto-rejected
post_w = 5;
post_az = [90, 210, 330]; // kept off the base diagonals, where clearance is worst
cap_gap = 3;
cap_t = 3;
lidar_range_min = 50;   // measured on the device: range_min = 0.050 m

// ---- Derived ----
sensor_half = sensor_side / 2;
sensor_half_diag = sensor_half * sqrt(2);
sensor_base_z = lidar_scan_z - sensor_scan_h;
sensor_top_z = sensor_base_z + sensor_total_h;
sensor_roof_offset = sensor_base_z - roof_top_z; // negative = recessed
fairing_top_z = lidar_scan_z - scan_clearance;
fairing_h = fairing_top_z - roof_top_z;
cap_bottom_z = sensor_top_z + cap_gap;
cap_top_z = cap_bottom_z + cap_t;
pod_half = pod_side / 2;

// Square boundary distance at an azimuth: the flats are nearest, the corners
// farthest, so the worst post position is on a diagonal.
function square_reach(az, half) =
    half / max(abs(cos(az)), abs(sin(az)));
function post_gap(az) =
    post_r - post_w / 2 - square_reach(az, sensor_half);

blind_per_post = 2 * atan((post_w / 2) / post_r);
blind_total = len(post_az) * blind_per_post;
blind_fraction = blind_total / 360;
blind_samples = round(720 * blind_fraction); // 0.5 deg increment, 720 samples
min_post_gap = min([for (az = post_az) post_gap(az)]);

assert(fairing_top_z <= lidar_scan_z - scan_clearance,
       "fairing must stop clear of the scan plane");
assert(fairing_h > 0,
       "fairing has no height: the scan plane is at or below the roof");
assert(pod_half - sensor_half >= pod_wall + 3,
       "pod needs wall thickness plus assembly clearance around the sensor");
assert(sensor_half_diag < pod_half + pod_corner_r,
       "pod footprint must enclose the sensor base corners");
assert(post_r < lidar_range_min,
       "posts must sit inside range_min so their returns are discarded");
assert(min_post_gap >= 8,
       "posts need at least 8 mm from the sensor body at their azimuth");

penetration_reach = insert_pitch_x / 2 + fastener_clear_r;

echo("scan_h(TBD)=", sensor_scan_h,
     " penetration_reach=", penetration_reach,
     " sensor_base_z=", sensor_base_z,
     " roof_offset=", sensor_roof_offset,
     " fairing_h=", fairing_h,
     " cap_top_z=", cap_top_z,
     " pod_above_roof=", cap_top_z - roof_top_z,
     " blind_deg=", blind_total,
     " blind_samples=", blind_samples,
     " min_post_gap=", min_post_gap);

module rounded_square_2d(side, r) {
    offset(r=r) offset(delta=-r) square(side, center=true);
}

module sensor_reference() {
    translate([lidar_x, 0, sensor_base_z]) {
        color("#181818")
            linear_extrude(height=sensor_base_h)
                rounded_square_2d(sensor_side, 6);
        // Rotating head. The glossy optical band straddles the scan plane.
        color("#242424")
            translate([0, 0, sensor_base_h])
                cylinder(d=sensor_side - 5, h=sensor_total_h - sensor_base_h);
        color("#3D4A50", 0.9)
            translate([0, 0, sensor_scan_h - 5])
                cylinder(d=sensor_side - 4, h=10);
    }
}

module pod_fairing() {
    color("#DCE7EC")
        translate([lidar_x, 0, roof_top_z])
            difference() {
                linear_extrude(height=fairing_h)
                    rounded_square_2d(pod_side, pod_corner_r);
                translate([0, 0, -1])
                    linear_extrude(height=fairing_h + 2)
                        rounded_square_2d(pod_side - 2 * pod_wall,
                                          pod_corner_r - pod_wall);
                // Horizontal cable exit with strain-relief width.
                rotate([0, 0, cable_side_az])
                    translate([pod_half - pod_wall / 2, 0,
                               cable_slot_h / 2 + 2])
                        cube([pod_wall * 4, cable_slot_w, cable_slot_h],
                             center=true);
            }

    // Mounting flange: the pod locates on the sensor's own inserts, so nothing
    // clamps the housing itself.
    color("#9AA7AE")
        translate([lidar_x, 0, roof_top_z])
            difference() {
                linear_extrude(height=panel_t)
                    rounded_square_2d(pod_side, pod_corner_r);
                for (sx = [-1, 1], sy = [-1, 1])
                    translate([sx * insert_pitch_x / 2,
                               sy * insert_pitch_y / 2, -1])
                        cylinder(d=insert_d, h=panel_t + 2);
                translate([0, 0, -1])
                    linear_extrude(height=panel_t + 2)
                        square(sensor_side - 12, center=true);
            }
}

module pod_cage() {
    color("#DCE7EC") {
        // Posts are placed about the sensor axis, not the world origin.
        for (az = post_az)
            translate([lidar_x + post_r * cos(az), post_r * sin(az),
                       (fairing_top_z + cap_bottom_z) / 2])
                cube([post_w, post_w, cap_bottom_z - fairing_top_z],
                     center=true);
        translate([lidar_x, 0, cap_bottom_z])
            linear_extrude(height=cap_t)
                rounded_square_2d(pod_side, pod_corner_r);
    }
}

module roof_reference() {
    color("#C9DCE5", 0.55)
        translate([lidar_x, 0, uniform_shell_top_z])
            difference() {
                linear_extrude(height=panel_t)
                    square([340, 300], center=true);
                translate([0, 0, -1])
                    linear_extrude(height=panel_t + 2)
                        square(sensor_side - 12, center=true);
            }
}

if (show_roof) roof_reference();
pod_fairing();
if (pod_variant == "caged") pod_cage();
else assert(pod_variant == "open",
            str("Unknown pod_variant: ", pod_variant));
if (show_sensor) sensor_reference();
if (show_scan_plane)
    color("crimson", 0.16)
        translate([lidar_x, 0, lidar_scan_z])
            cube([300, 300, 0.6], center=true);
