// P2-77 under-deck torsion-node repair.
// Reuses the four printed torsion_diagonal_half parts after trimming their
// centre ends and drilling one transverse M4 hole per half with the jig.
// Robot coordinates: +X front, +Y left, +Z up.  Printed parts use local axes.

$fn = 48;
part = "assembly";
show_context = true;
show_floor = true;
show_ballast = true;

m4_clear = 4.5;
brace_w = 20;
brace_h = 10;
node_z = 12;                 // brace/node range Z12..22 below crossbars
node_stub = 36;
socket_len = 30;
cheek_t = 4;
arm_angle = atan2(150,30);   // 78.690 deg from +X

module capsule_2d(length,width) {
    hull() for (x=[-(length-width)/2,(length-width)/2])
        translate([x,0]) circle(d=width);
}

// One radial arm of the shared X node.  The solid core ends at 36 mm from
// centre; two integral cheeks continue to 60 mm and receive the trimmed old
// half.  The old half butts against the core rather than crossing the centre.
module node_arm() {
    difference() {
        union() {
            translate([node_stub/2,0,brace_h/2])
                cube([node_stub,brace_w,brace_h],center=true);
            for (s=[-1,1])
                translate([node_stub+socket_len/2,
                           s*(brace_w/2+cheek_t/2),brace_h/2])
                    cube([socket_len,cheek_t,brace_h],center=true);
        }
        // Horizontal pilot. Drill/ream to 4.5 mm after printing.
        translate([node_stub+18,-brace_w/2-cheek_t-1,brace_h/2])
            rotate([-90,0,0]) cylinder(d=3.2,h=brace_w+2*cheek_t+2);
    }
}

module torsion_centre_node() {
    union()
        for (a=[arm_angle,-arm_angle,180+arm_angle,180-arm_angle])
            rotate([0,0,a]) node_arm();
}

// Clips over a cut 20x10 brace end and locates the transverse pilot 18 mm
// from the cut.  Split underside makes it support-free and removable.
module brace_drill_jig() {
    difference() {
        translate([20,0,8]) cube([40,28,16],center=true);
        translate([-1,-10.35,-1]) cube([42,20.7,11.4]);
        translate([18,-16,5]) rotate([-90,0,0]) cylinder(d=3.2,h=32);
        translate([5,-16,-1]) cube([10,32,6]);
    }
}

// MPU6050 carrier for the inner face of the +Y longitudinal rail.  Its two
// chassis holes reuse the existing X100/X125 side-grid holes at global Z47.
// The four cross slots accept the existing MPU board screws.
module mpu6050_rail_side_mount() {
    difference() {
        union() {
            translate([0,0,2]) cube([76,38,4],center=true);
            for (x=[-10,10],z=[-10,10])
                translate([x,z,4]) cylinder(d=8,h=4);
        }
        for (x=[-12.5,12.5])
            translate([x,0,-1]) cylinder(d=m4_clear,h=10);
        for (x=[-10,10],z=[-10,10])
            translate([x,z,-1]) linear_extrude(height=10)
                union() {
                    capsule_2d(8,3.2);
                    rotate(90) capsule_2d(8,3.2);
                }
        for (x=[-30,30],z=[-12,12])
            translate([x,z,-1]) linear_extrude(height=6)
                capsule_2d(12,4);
    }
}

// Conservative installed brace envelopes after cutting.  The endpoint-to-
// node pieces stop at the core and sit directly under the X240/X300 tubes.
module installed_brace(a,b) {
    v=[b[0]-a[0],b[1]-a[1]];
    l=sqrt(v[0]*v[0]+v[1]*v[1]);
    ang=atan2(v[1],v[0]);
    translate([a[0],a[1],node_z]) rotate([0,0,ang])
        translate([l/2,0,brace_h/2]) cube([l,brace_w,brace_h],center=true);
}

module chassis_context() {
    // Exact relevant envelopes: both front crossbars, longitudinal rails,
    // feed channel/future basket-floor lower surface, skis and ballast disc.
    color([0.25,0.27,0.3,0.35]) {
        translate([240,0,32]) cube([50,390,20],center=true);
        translate([300,0,32]) cube([20,390,20],center=true);
        for (y=[-220,220]) translate([270,y,37]) cube([180,50,30],center=true);
    }
    color([0.9,0.45,0.1,0.3])
        hull() {
            translate([227,0,46.1]) cube([2,80,4],center=true);
            translate([278,0,42]) cube([2,80,4],center=true);
        }
    if (show_floor) color([0.1,0.55,0.8,0.18])
        hull() {
            translate([-85,0,70]) cube([2,280,4],center=true);
            translate([278,0,42]) cube([2,280,4],center=true);
        }
    color([0.2,0.65,0.3,0.3])
        for (y=[-82,82]) translate([300,y,25.5]) cube([35,49,33],center=true);
    if (show_ballast) color([0.55,0.35,0.15,0.25])
        translate([320,0,70]) cylinder(d=230,h=30);
}

module repair_structure() {
    union() {
        for (p=[[[240,150],[270-36*cos(arm_angle),36*sin(arm_angle)]],
                [[300,-150],[270+36*cos(arm_angle),-36*sin(arm_angle)]],
                [[240,-150],[270-36*cos(arm_angle),-36*sin(arm_angle)]],
                [[300,150],[270+36*cos(arm_angle),36*sin(arm_angle)]]])
            installed_brace([p[0][0],p[0][1]], [p[1][0],p[1][1]]);
        translate([270,0,node_z]) torsion_centre_node();
    }
}

module assembly() {
    if (show_context) chassis_context();
    color([0.15,0.15,0.17]) repair_structure();
    color([0.9,0.55,0.05]) translate([270,0,node_z]) torsion_centre_node();
    color([0.65,0.2,0.75])
        translate([112.5,195,47]) rotate([90,0,0]) mpu6050_rail_side_mount();
}

module print_set() {
    torsion_centre_node();
    translate([120,0,0]) brace_drill_jig();
    translate([195,0,0]) mpu6050_rail_side_mount();
}

if (part=="torsion_centre_node") torsion_centre_node();
else if (part=="brace_drill_jig") brace_drill_jig();
else if (part=="mpu6050_rail_side_mount") mpu6050_rail_side_mount();
else if (part=="print_set") print_set();
else assembly();
