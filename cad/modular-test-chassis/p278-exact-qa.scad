// P2-78 exact-solid checks.  Every selectable QA export must be empty.
use <modular-chassis-v3.scad>
use <shared-flywheel-p273-bolted-ski.scad>
use <p277-torsion-node.scad>

$fn=48;
qa="x240_sleeve_fit";
rail_z=22;
rail_y=220;
front_motor_x=300;
p278_crossbar_len=130;
p278_diagonal_h=15;

module p278_half(x,width,side=1) {
    translate([x,0,rail_z]) rotate([0,0,side>0?0:180])
        p278_crossbar_half(width);
}

module p278_end_sleeve(x,width,height,side=1) {
    translate([x,side*p278_crossbar_len,rail_z+10]) rotate([0,0,90])
        p278_terminal_sleeve(width,height);
}

module p278_terminal(side=1) {
    translate([front_motor_x,side*rail_y,rail_z]) front_terminal_v3(side);
}

module p278_sockets_only(side=1) {
    translate([front_motor_x,side*rail_y,rail_z]) union() {
        p278_terminal_socket(side,-60,50);
        p278_terminal_socket(side,0,20);
    }
}

module p278_joint_material(x,width,sleeve_w_local,sleeve_h_local,side=1) {
    union() {
        p278_half(x,width,side);
        p278_terminal(side);
        p278_end_sleeve(x,sleeve_w_local,sleeve_h_local,side);
    }
}

module p278_hole_probes(x,width,side=1) {
    // Crossbar grids Y75/100/125, with Y100 also the -30 mm lock; socket grid
    // Y150 and +30 mm lock Y160.  Vertical/transverse bores must stay clear.
    for(y=[75,100,125,150,160]) {
        translate([x,side*y,rail_z-2]) cylinder(d=4.4,h=25);
        translate([x-width/2-2,side*y,rail_z+10]) rotate([0,90,0])
            cylinder(d=4.4,h=width+4);
    }
}

module p278_fit0403_and_cable() {
    p265_front_fit0403_envelope();
    mirror([0,1,0]) p265_front_fit0403_envelope();
}

module p278_wheels() {
    for(side=[-1,1])
        translate([300,side*297,85]) rotate([90,0,0])
            cylinder(r=85,h=83,center=true);
}

module p278_feed_floor_ballast() {
    hull() {
        translate([227,0,46.1]) cube([2,80,4],center=true);
        translate([278,0,42]) cube([2,80,4],center=true);
    }
    hull() {
        translate([-85,0,70]) cube([2,280,4],center=true);
        translate([278,0,42]) cube([2,280,4],center=true);
    }
    translate([320,0,70]) cylinder(d=230,h=30);
}

if(qa=="x240_sleeve_fit") intersection() {
    p278_end_sleeve(240,43,13,1);
    union() { p278_half(240,50,1); p278_terminal(1); }
}
else if(qa=="x300_sleeve_fit") intersection() {
    p278_end_sleeve(300,13,13,1);
    union() { p278_half(300,20,1); p278_terminal(1); }
}
else if(qa=="x240_holes") intersection() {
    p278_joint_material(240,50,43,13,1); p278_hole_probes(240,50,1);
}
else if(qa=="x300_holes") intersection() {
    p278_joint_material(300,20,13,13,1); p278_hole_probes(300,20,1);
}
else if(qa=="side_braces_frame") intersection() {
    p278_side_braces_assembly();
    union() {
        for(side=[-1,1]) {
            p278_half(240,50,side); p278_half(300,20,side); p278_terminal(side);
        }
    }
}
else if(qa=="side_braces_ski_brackets") intersection() {
    p278_side_braces_assembly(); union() for(side=[-1,1]) p273_mount_bracket(side);
}
else if(qa=="side_braces_feed_floor_ballast") intersection() {
    p278_side_braces_assembly(); p278_feed_floor_ballast();
}
else if(qa=="terminal_fit0403_cable") intersection() {
    union() for(side=[-1,1]) p278_sockets_only(side); p278_fit0403_and_cable();
}
else if(qa=="terminal_wheels") intersection() {
    union() for(side=[-1,1]) p278_terminal(side); p278_wheels();
}
else if(qa=="terminal_ski_brackets") intersection() {
    union() for(side=[-1,1]) p278_terminal(side);
    union() for(side=[-1,1]) p273_mount_bracket(side);
}
else if(qa=="side_braces_mpu") intersection() {
    p278_side_braces_assembly();
    translate([112.5,195,37]) rotate([90,0,0]) mpu6050_rail_side_mount();
}
else if(qa=="side_braces_fit0403_cable") intersection() {
    p278_side_braces_assembly(); p278_fit0403_and_cable();
}
else if(qa=="ground") intersection() {
    // Physical datum is about 2 mm lower than CAD: physical Z15 maps to CAD Z17.
    p278_side_braces_assembly(); translate([270,0,8.5]) cube([800,800,17],center=true);
}
else assert(false,str("Unknown P2-78 QA target: ",qa));

assert(p278_crossbar_len==130,"P2-78 crossbar/socket interface moved");
assert(rail_z==22,"P2-78 brace bottom datum changed");
