// PROVISIONAL simulation geometry only; not manufacturing CAD.
// Resolves the measured low-energy post-nip lower-plate interference without
// changing wheel, nip, motor, hub, pitch, or calibrated ball parameters.
$fn = 96;

part = "assembly"; // [assembly,lower,upper,cutout]

panel_x = 256;
panel_y = 314;
panel_t = 8;
panel_z = 43;
shaft_y = 129;
shaft_hole_d = 12;

// Frozen accepted wheel-exit state, launcher-local SI values converted below.
release_x = 38.8738059257;
release_z = -4.57450083064;
release_vx = 5269.52537961;
release_vz = -196.876959017;
gravity_x = -3351.79740459;
gravity_z = -9208.98768370;
ball_swept_r = 38; // 33 mm ball radius + 5 mm practical clearance.
sample_dt = 0.0005;
sample_count = 140;

// Explicit analysis-only CAD corridor from launcher-envelope.scad.
cad_corridor_x0 = 100;
cad_corridor_x1 = 320;
cad_corridor_r = 45;
lower_inner_z = -39;
cad_plate_half_width = sqrt(cad_corridor_r * cad_corridor_r
                            - lower_inner_z * lower_inner_z);

function path_x(i) = release_x + release_vx * (i * sample_dt)
                   + 0.5 * gravity_x * pow(i * sample_dt, 2);
function path_z(i) = release_z + release_vz * (i * sample_dt)
                   + 0.5 * gravity_z * pow(i * sample_dt, 2);
function plate_cross_r(i) =
    let(distance = path_z(i) - lower_inner_z)
    distance >= 0 && distance < ball_swept_r
        ? sqrt(ball_swept_r * ball_swept_r - distance * distance)
        : 0;

module measured_lower_swept_cutout_2d() {
    union()
        for (i = [0:sample_count - 1])
            if (plate_cross_r(i) > 0 && plate_cross_r(i + 1) > 0)
                hull() {
                    translate([path_x(i), 0]) circle(r=plate_cross_r(i));
                    translate([path_x(i + 1), 0]) circle(r=plate_cross_r(i + 1));
                }
}

module cad_corridor_plate_cutout_2d() {
    translate([(cad_corridor_x0 + cad_corridor_x1) / 2, 0])
        square([cad_corridor_x1 - cad_corridor_x0,
                2 * cad_plate_half_width], center=true);
}

module practical_lower_cutout_2d() {
    union() {
        measured_lower_swept_cutout_2d();
        cad_corridor_plate_cutout_2d();
    }
}

module lower_panel() {
    difference() {
        cube([panel_x, panel_y, panel_t], center=true);
        linear_extrude(height=panel_t + 2, center=true)
            practical_lower_cutout_2d();
    }
}

module upper_panel() {
    difference() {
        cube([panel_x, panel_y, panel_t], center=true);
        for (y = [-shaft_y, shaft_y])
            translate([0, y, 0]) cylinder(d=shaft_hole_d, h=panel_t + 2, center=true);
        linear_extrude(height=panel_t + 2, center=true)
            cad_corridor_plate_cutout_2d();
    }
}

module assembly() {
    color([0.35, 0.40, 0.46]) translate([0, 0, -panel_z]) lower_panel();
    color([0.35, 0.40, 0.46]) translate([0, 0, panel_z]) upper_panel();
    color("steelblue", 0.20)
        translate([(cad_corridor_x0 + cad_corridor_x1) / 2, 0, 0])
            rotate([0, 90, 0])
                cylinder(r=cad_corridor_r,
                         h=cad_corridor_x1 - cad_corridor_x0, center=true);
}

if (part == "lower") lower_panel();
else if (part == "upper") upper_panel();
else if (part == "cutout")
    linear_extrude(height=panel_t, center=true) practical_lower_cutout_2d();
else assembly();
