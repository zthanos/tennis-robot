// New 120x80 mm perfboard mechanical layout study, NOT a soldering netlist.
// Four BTS7960 logic harnesses use integrated 2-pin power and 4-pin control
// keyways matching cad/bts7960-control-plug/perfboard-male-end-v1.scad.
// The remaining openings reserve Mega, 6 encoders, IR, user, IMU and 5V.
// Fit and wiring must be checked before any header is soldered.

$fn = 36;

part = "assembly"; // [assembly,faceplate,board,exploded,port_test]
show_perf_holes = false;

board_size = [120, 80];
board_t = 1.60;
mount_pattern = [115, 75];
mount_hole_d = 4.00;
faceplate_t = 1.20;
faceplate_mount_hole_d = 3.40;
faceplate_corner_r = 1.00;

pitch = 2.54;
grid_count = [42, 30];
grid_origin = [
    (board_size[0]-(grid_count[0]-1)*pitch)/2,
    (board_size[1]-(grid_count[1]-1)*pitch)/2
];
perf_hole_d = 1.00;

// Matching interface values from perfboard-male-end-v1.scad.
single_x = 2.54;
pack_fit_x = 0.50;
male_nose_x_extra = 2.40;
male_nose_y = 2.54+0.50+2.40;
key_width = 1.40;
key_projection = 1.10;
collar_clearance = 0.55;
collar_wall = 2.00;
collar_h = 7.50; // total height above board, including faceplate thickness

aux_clearance = 0.50;
idc_body_w = 9.00;
idc_end_margin = 5.00;
encoder_body_end_margin = 2.20;
label_h = 0.45;

// [name, power first column, control first column, board row]
// Motion pair on lower row, intake pair on upper row. Names are provisional.
drivers = [
    ["M-L", 10, 17,  8],
    ["M-R", 27, 34,  8],
    ["I-L", 10, 17, 18],
    ["I-R", 27, 34, 18]
];

// Six separately accessible 1x4 encoder headers across the upper band.
encoder_first_columns = [7, 13, 19, 25, 31, 37];
encoder_names = ["LF", "LR", "RF", "RR", "IL", "IR"];
encoder_row = 26;

// Other retained interfaces, moved into the lower service band.
// [label, first column, pin columns, first row, pin rows]
idc_headers = [
    ["MEGA",  3, 2,  6, 17],
    ["IR",   15, 5,  2,  2],
    ["USER", 24, 3,  2,  2],
    ["IMU",  31, 2,  2,  2]
];
straight_headers = [
    ["JP_PWR", 36, 3, 3],
    ["5V_IN", 41, 2, 3]
];

function gx(c) = grid_origin[0]+(c-1)*pitch;
function gy(r) = grid_origin[1]+(r-1)*pitch;
function xmid(first, count) = (gx(first)+gx(first+count-1))/2;
function ymid(first, count) = (gy(first)+gy(first+count-1))/2;
function port_nose_x(n) = n*single_x+pack_fit_x+male_nose_x_extra;
function port_inner_x(n) = port_nose_x(n)+collar_clearance;
function port_inner_y() = male_nose_y+collar_clearance;
function port_outer_x(n) = port_inner_x(n)+2*collar_wall;
function port_outer_y() = port_inner_y()+2*collar_wall;
function port_key_x(n) = n == 2 ? 0 : pitch;
function port_key_side(n) = n == 2 ? 1 : -1;

function driver_power_center(d) = [xmid(d[1],2), gy(d[3])];
function driver_control_center(d) = [xmid(d[2],4), gy(d[3])];

mount_holes = [
    [(board_size[0]-mount_pattern[0])/2,
     (board_size[1]-mount_pattern[1])/2],
    [(board_size[0]+mount_pattern[0])/2,
     (board_size[1]-mount_pattern[1])/2],
    [(board_size[0]-mount_pattern[0])/2,
     (board_size[1]+mount_pattern[1])/2],
    [(board_size[0]+mount_pattern[0])/2,
     (board_size[1]+mount_pattern[1])/2]
];

module rounded_plate_2d() {
    translate([faceplate_corner_r, faceplate_corner_r])
        offset(r=faceplate_corner_r)
            square([board_size[0]-2*faceplate_corner_r,
                    board_size[1]-2*faceplate_corner_r]);
}

module board_mockup() {
    color("SeaGreen")
        difference() {
            linear_extrude(board_t) rounded_plate_2d();
            for (p = mount_holes)
                translate([p[0],p[1],-0.2])
                    cylinder(d=mount_hole_d,h=board_t+0.4);
            if (show_perf_holes)
                for (c=[1:grid_count[0]],r=[1:grid_count[1]])
                    translate([gx(c),gy(r),-0.2])
                        cylinder(d=perf_hole_d,h=board_t+0.4,$fn=12);
        }
}

module port_outer(n, center) {
    translate([center[0]-port_outer_x(n)/2,
               center[1]-port_outer_y()/2,0])
        cube([port_outer_x(n),port_outer_y(),collar_h]);
}

module port_opening(n, center) {
    // The through-opening receives both the male plug nose and the female
    // perfboard header. The side keyway must match its cable-plug rib.
    translate([center[0]-port_inner_x(n)/2,
               center[1]-port_inner_y()/2,-0.2])
        cube([port_inner_x(n),port_inner_y(),collar_h+0.4]);
    key_slot_x = key_width+collar_clearance;
    translate([center[0]+port_key_x(n)-key_slot_x/2,
               center[1]+(port_key_side(n)>0 ? male_nose_y/2 :
                         -male_nose_y/2-key_projection-collar_clearance/2),
               -0.2])
        cube([key_slot_x,key_projection+collar_clearance/2,
              collar_h+0.4]);
}

module rectangular_cutout(center, size) {
    translate([center[0]-size[0]/2,center[1]-size[1]/2,-0.2])
        cube([size[0],size[1],collar_h+0.4]);
}

module idc_cutout(h) {
    body_w = idc_body_w+2*aux_clearance;
    // MEGA uses two columns x seventeen rows, so rotate body 90 degrees.
    center = [xmid(h[1],h[2]),ymid(h[3],h[4])];
    if (h[0] == "MEGA")
        rectangular_cutout(center,[body_w,h[4]*pitch+idc_end_margin+
                                   2*aux_clearance]);
    else
        rectangular_cutout(center,[h[2]*pitch+idc_end_margin+
                                   2*aux_clearance,body_w]);
}

module straight_cutout(first_col, n, row) {
    rectangular_cutout([xmid(first_col,n),gy(row)],
                       [(n-1)*pitch+encoder_body_end_margin+
                        2*aux_clearance,pitch+2*aux_clearance]);
}

module faceplate_geometry() {
    difference() {
        union() {
            linear_extrude(faceplate_t) rounded_plate_2d();
            for (d=drivers) {
                port_outer(2,driver_power_center(d));
                port_outer(4,driver_control_center(d));
            }
        }
        for (p=mount_holes)
            translate([p[0],p[1],-0.2])
                cylinder(d=faceplate_mount_hole_d,h=collar_h+0.4);
        for (d=drivers) {
            port_opening(2,driver_power_center(d));
            port_opening(4,driver_control_center(d));
        }
        for (h=idc_headers) idc_cutout(h);
        for (i=[0:5])
            straight_cutout(encoder_first_columns[i],4,encoder_row);
        for (h=straight_headers)
            straight_cutout(h[1],h[2],h[3]);
    }
}

module raised_label(label, pos, size=2.4, rotation=0) {
    translate([pos[0],pos[1],faceplate_t])
        rotate([0,0,rotation])
            linear_extrude(label_h)
                text(label,size=size,halign="center",valign="center",
                     font="Liberation Sans:style=Bold");
}

module faceplate_labels() {
    for (d=drivers) {
        p=driver_power_center(d);
        q=driver_control_center(d);
        raised_label(str(d[0]," PWR"),[p[0],p[1]+9.0],2.25);
        raised_label(str(d[0]," CTRL"),[q[0],q[1]+9.0],2.25);
        raised_label("+",[p[0]-pitch/2,p[1]+6.2],1.8);
        raised_label("G",[p[0]+pitch/2,p[1]+6.2],1.8);
    }
    raised_label("MEGA",[22.5,36.0],2.7,90);
    for (i=[0:5])
        raised_label(encoder_names[i],
                     [xmid(encoder_first_columns[i],4),72.2],2.5);
    for (h=idc_headers) if (h[0] != "MEGA")
        raised_label(h[0],[xmid(h[1],h[2]),13.5],2.3);
    for (h=straight_headers)
        raised_label(h[0] == "JP_PWR" ? "JP" : "5V",
                     [xmid(h[1],h[2]),13.5],2.3);
}

module faceplate() {
    color("Gainsboro") faceplate_geometry();
    color("Black") faceplate_labels();
}

module port_pair_fit_coupon() {
    // Small first print: one integrated keyed 2+4 pair at the exact spacing
    // used on the full faceplate, before committing to a 120x80 mm print.
    d=drivers[0];
    p=driver_power_center(d);
    q=driver_control_center(d);
    x0=p[0]-port_outer_x(2)/2-4.0;
    x1=q[0]+port_outer_x(4)/2+4.0;
    y0=p[1]-port_outer_y()/2-4.0;
    y1=p[1]+port_outer_y()/2+4.0;
    difference() {
        union() {
            translate([x0,y0,0])
                cube([x1-x0,y1-y0,faceplate_t]);
            port_outer(2,p);
            port_outer(4,q);
        }
        port_opening(2,p);
        port_opening(4,q);
    }
}

module female_header_mockup(n, center) {
    body_x = n*pitch;
    body_y = pitch;
    color("DimGray")
        translate([center[0]-body_x/2,center[1]-body_y/2,board_t])
            cube([body_x,body_y,8.50]);
}

module header_mockups() {
    for (d=drivers) {
        female_header_mockup(2,driver_power_center(d));
        female_header_mockup(4,driver_control_center(d));
    }
    for (i=[0:5])
        female_header_mockup(4,
           [xmid(encoder_first_columns[i],4),gy(encoder_row)]);
    for (h=idc_headers) {
        center=[xmid(h[1],h[2]),ymid(h[3],h[4])];
        size=h[0] == "MEGA" ?
             [idc_body_w,h[4]*pitch+idc_end_margin] :
             [h[2]*pitch+idc_end_margin,idc_body_w];
        color("SlateGray")
            translate([center[0]-size[0]/2,center[1]-size[1]/2,board_t])
                cube([size[0],size[1],8.50]);
    }
    for (h=straight_headers) {
        size=[(h[2]-1)*pitch+encoder_body_end_margin,pitch];
        color("FireBrick")
            translate([xmid(h[1],h[2])-size[0]/2,
                       gy(h[3])-size[1]/2,board_t])
                cube([size[0],size[1],8.50]);
    }
}

module assembly(explode=0) {
    board_mockup();
    header_mockups();
    translate([0,0,board_t+explode]) faceplate();
}

if (part == "assembly") assembly();
else if (part == "exploded") assembly(12);
else if (part == "faceplate") faceplate();
else if (part == "board") {board_mockup();header_mockups();}
else if (part == "port_test") port_pair_fit_coupon();
else assert(false,str("Unknown part: ",part));
