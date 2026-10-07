// Unified-header perfboard and 3D-printed label faceplate.
//
// The model uses the measured 120 x 80 mm perfboard and 115 x 75 mm
// mounting-hole pattern already captured in params.scad.  Header body sizes
// are nominal 2.54 mm IDC dimensions; measure the purchased parts before the
// final print and adjust header_end_margin/header_body_w if necessary.
//
// Examples:
//   openscad -D 'part="assembly"' -o perfboard-faceplate-assembly.png perfboard-faceplate.scad
//   openscad -D 'part="exploded"' -o perfboard-faceplate-exploded.png perfboard-faceplate.scad
//   openscad -D 'part="faceplate"' --export-format binstl -o perfboard-faceplate.stl perfboard-faceplate.scad

$fn = 36;

part = "assembly";             // assembly | exploded | faceplate | board
show_perf_holes = true;

board_size = [120, 80];
board_t = 1.6;
mount_pattern = [115, 75];
mount_hole_d = 4.0;

pitch = 2.54;
// Verified from photographs of the actual board: 42 holes along the 120 mm
// side and 30 holes along the 80 mm side (1,260 plated-through holes total).
grid_count = [42, 30];
grid_origin = [
    (board_size[0] - (grid_count[0] - 1) * pitch) / 2,
    (board_size[1] - (grid_count[1] - 1) * pitch) / 2
];
perf_hole_d = 1.0;

faceplate_t = 1.2;
faceplate_gap = 0.00;
faceplate_corner_r = 1.0;
faceplate_mount_hole_d = 3.4;
cutout_clearance = 0.50;
label_h = 0.50;
label_size = 3.2;

header_body_w = 9.0;
header_end_margin = 5.0;
header_h = 8.5;
header_wall = 1.15;
header_slot_w = 3.3;
pin_size = 0.64;
pin_h = 2.5;

// Coordinates use the verified 42 x 30 component-side grid.  In landscape
// orientation column 1 is at the left and row 1 is at the lower edge.
function grid_x(column) = grid_origin[0] + (column - 1) * pitch;
function grid_y(row) = grid_origin[1] + (row - 1) * pitch;
function grid_mid(a, b) = (a + b) / 2;

// [name, first column, last column, first row, last row, number of pairs,
//  rotation, colour, label x, label y, label rotation]
headers = [
    ["MEGA",   3,  4,  6, 22, 17, 90, "SlateBlue", 25.5, 36.0, 90],
    ["MOTION",16, 29,  6,  7, 14,  0, "SeaGreen", 62.5, 24.5,  0],
    ["INTAKE",17, 24, 13, 14,  8,  0, "DarkOrange",57.5, 41.5,  0],
    ["IR",    31, 35, 13, 14,  5,  0, "SteelBlue", 89.0, 41.5,  0],
    ["USER",  17, 19, 20, 21,  3,  0, "MediumPurple",50.0,59.0, 0],
    ["GYRO",  24, 25, 20, 21,  2,  0, "Goldenrod", 68.0,59.0, 0]
];

power_header = ["PWR", 32, 34, 20, 20, 3, 0, "FireBrick", 88.5, 59.0, 0];
power_input_header = ["5V_IN", 37, 38, 20, 20, 2, 0, "FireBrick", 103.5, 59.0, 0];

function header_center(h) = [
    grid_x(grid_mid(h[1], h[2])),
    grid_y(grid_mid(h[3], h[4]))
];
function idc_body_l(pair_count) = pair_count * pitch + header_end_margin;

mount_holes = [
    [(board_size[0] - mount_pattern[0]) / 2,
     (board_size[1] - mount_pattern[1]) / 2],
    [(board_size[0] + mount_pattern[0]) / 2,
     (board_size[1] - mount_pattern[1]) / 2],
    [(board_size[0] - mount_pattern[0]) / 2,
     (board_size[1] + mount_pattern[1]) / 2],
    [(board_size[0] + mount_pattern[0]) / 2,
     (board_size[1] + mount_pattern[1]) / 2]
];

module rounded_plate_2d(size, r) {
    translate([r, r])
        offset(r=r)
            square([size[0] - 2*r, size[1] - 2*r]);
}

module perfboard() {
    color("SeaGreen")
        difference() {
            linear_extrude(board_t)
                rounded_plate_2d(board_size, 1.0);

            for (p = mount_holes)
                translate([p[0], p[1], -0.2])
                    cylinder(d=mount_hole_d, h=board_t + 0.4);

            if (show_perf_holes)
                for (column = [1:grid_count[0]])
                    for (row = [1:grid_count[1]])
                        translate([grid_x(column), grid_y(row), -0.2])
                            cylinder(d=perf_hole_d, h=board_t + 0.4,
                                     $fn=12);
        }
}

module idc_box_header(pair_count, center, rotation=0, body_colour="DimGray") {
    body_l = idc_body_l(pair_count);
    body_w = header_body_w;

    translate([center[0], center[1], board_t])
        rotate([0, 0, rotation]) {
            color(body_colour)
                difference() {
                    translate([-body_l/2, -body_w/2, 0])
                        cube([body_l, body_w, header_h]);
                    translate([-body_l/2 + header_wall,
                               -header_slot_w/2,
                               header_wall])
                        cube([body_l - 2*header_wall,
                              header_slot_w,
                              header_h]);
                }

            for (i = [0:pair_count-1])
                for (side = [-1, 1])
                    color("Gold")
                        translate([(i - (pair_count-1)/2) * pitch,
                                   side * pitch/2,
                                   4.25])
                            cube([pin_size, pin_size, header_h + 5.0],
                                 center=true);

            // Pin-1 marker on the shroud.
            color("FireBrick")
                translate([-(pair_count-1)*pitch/2,
                           -body_w/2 - 0.25,
                           header_h - 1.2])
                    sphere(d=1.4, $fn=18);
        }
}

module straight_pin_header(pin_count, center, rotation=0) {
    body_l = (pin_count - 1) * pitch + 2.2;

    translate([center[0], center[1], board_t])
        rotate([0, 0, rotation]) {
            color("FireBrick")
                translate([-body_l/2, -pitch/2, 0])
                    cube([body_l, pitch, 2.5]);
            for (i = [0:pin_count-1])
                color("Gold")
                    translate([(i - (pin_count-1)/2) * pitch, 0, 3.0])
                        cube([pin_size, pin_size, 11.0], center=true);
        }
}

module header_cutout(h) {
    center = header_center(h);
    body_l = idc_body_l(h[5]) + 2*cutout_clearance;
    body_w = header_body_w + 2*cutout_clearance;

    translate([center[0], center[1], -0.2])
        rotate([0, 0, h[6]])
            translate([-body_l/2, -body_w/2, 0])
                cube([body_l, body_w, faceplate_t + 0.4]);
}

module straight_header_cutout(h) {
    center = header_center(h);
    body_l = (h[5] - 1) * pitch + 2.2 +
             2*cutout_clearance;
    body_w = pitch + 2*cutout_clearance;

    translate([center[0], center[1], -0.2])
        translate([-body_l/2, -body_w/2, 0])
            cube([body_l, body_w, faceplate_t + 0.4]);
}

module raised_label(label, position, rotation=0, size=label_size) {
    color("Black")
        translate([position[0], position[1], faceplate_t])
            rotate([0, 0, rotation])
                linear_extrude(label_h)
                    text(label, size=size, halign="center", valign="center",
                         font="Liberation Sans:style=Bold");
}

module faceplate() {
    color("Gainsboro")
        difference() {
            linear_extrude(faceplate_t)
                rounded_plate_2d(board_size, faceplate_corner_r);

            for (p = mount_holes)
                translate([p[0], p[1], -0.2])
                    cylinder(d=faceplate_mount_hole_d,
                             h=faceplate_t + 0.4);

            for (h = headers)
                header_cutout(h);

            straight_header_cutout(power_header);
            straight_header_cutout(power_input_header);
        }

    for (h = headers)
        raised_label(h[0], [h[8], h[9]], h[10]);

    raised_label(power_header[0],
                 [power_header[8], power_header[9]],
                 power_header[10], 2.8);
    raised_label(power_input_header[0],
                 [power_input_header[8], power_input_header[9]],
                 power_input_header[10], 2.5);

    // The two isolated rows of elongated edge pads become the logic buses
    // after they are bridged with continuous copper wire on the solder side.
    raised_label("+5V RAIL", [4.8, 40.0], 90, 2.6);
    raised_label("GND RAIL", [115.2, 40.0], 90, 2.6);
    raised_label("PIN 1 = RED DOT", [58.0, 73.0], 0, 2.4);
}

module all_headers() {
    for (h = headers)
        idc_box_header(h[5], header_center(h), h[6], h[7]);

    straight_pin_header(power_header[5], header_center(power_header),
                        power_header[6]);
    straight_pin_header(power_input_header[5],
                        header_center(power_input_header),
                        power_input_header[6]);
}

module assembly(explode=0) {
    perfboard();
    all_headers();

    translate([0, 0, board_t + faceplate_gap + explode])
        faceplate();
}

if (part == "assembly")
    assembly(0);
else if (part == "exploded")
    assembly(14);
else if (part == "faceplate")
    faceplate();
else if (part == "board") {
    perfboard();
    all_headers();
}
else
    assert(false, str("Unknown part: ", part));
