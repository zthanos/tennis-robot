// Modular chassis v3 -- Phase-2 printable frame and ballast-test set.
// Units mm.  +X front, +Y left, +Z up.  Ground is Z=0.
// This file does not contain the flywheel mechanism, electronics or wiring.

$fn=48;
use <../../engineering/chassis-strength-bench/coupon/drive-motor-inline-module.scad>

part="assembly";
show_temporary_gamma=false;

rail_w=50;
rail_h=30;
rail_wall=3.2;
rail_z=22;
rail_y=220;
rear_x=-165;
rear_motor_x=-65;
rear_joint_x=35;
front_joint_x=200;
front_motor_x=300;
front_end_x=330;
x240=240;
x300=300;

sleeve_w=43.0;
sleeve_h=23.0;
sleeve_len=120;
sleeve_wall=3.0;
insert_len=60;
m4_clear=4.5;
m4_nut_af=7.0;
m4_nut_clear=0.30;
m4_nut_h=3.4;
grid_pitch=25;
// 2026-10-16: half crossbars end at the rail inner face, not the rail centreline.
rear_crossbar_len=rail_y-rail_w/2;   // 195
front_crossbar_len=rail_y-rail_w/2;  // 195; X240/X300 stop at inner rail face
p265_rail_bolt_z=24;                 // global Z46 after the rail_z translation
p278_socket_len=65;
p278_crossbar_len=front_crossbar_len-p278_socket_len; // 130 mm
p278_diagonal_w=20;
p278_diagonal_h=15;
p278_side_brace_tab=23;               // 9.25 mm clear material beyond Ø4.5 bore
p278_side_brace_tab_h=23;

frame_color=[0.16,0.48,0.78,0.94];
sleeve_color=[0.95,0.53,0.10,0.95];
temp_color=[0.78,0.26,0.08,0.42];

module box_x(length,width=rail_w,height=rail_h,wall=rail_wall) {
    difference() {
        cube([length,width,height],center=true);
        cube([length+2,width-2*wall,height-2*wall],center=true);
    }
}

module box_y(length,width=rail_w,height=rail_h,wall=rail_wall) {
    rotate([0,0,90]) box_x(length,width,height,wall);
}

// 2026-10-15: all joints are through-bolted with external nuts; side holes
// pass through both walls and the enclosed nut pockets were removed.
module m4_hex(axis="z",center=[0,0,0]) {
    d=(m4_nut_af+m4_nut_clear)/cos(30);
    translate(center)
        if(axis=="z") cylinder(d=d,h=m4_nut_h,center=true,$fn=6);
        else if(axis=="y") rotate([90,0,0]) cylinder(d=d,h=m4_nut_h,center=true,$fn=6);
        else rotate([0,90,0]) cylinder(d=d,h=m4_nut_h,center=true,$fn=6);
}

// Top and side screws preload the sleeve against two socket datum faces.
// Clearance remains for insertion, but is not left as free running play.
module clamp_sleeve_x() {
    difference() {
        union() {
            difference() {
                cube([sleeve_len,sleeve_w,sleeve_h],center=true);
                cube([sleeve_len+2,sleeve_w-2*sleeve_wall,
                      sleeve_h-2*sleeve_wall],center=true);
            }
            for(xx=[-30,30]) {
                translate([xx,0,0]) cube([14,sleeve_w-2*sleeve_wall,
                                          sleeve_h-2*sleeve_wall],center=true);
            }
        }
        for(xx=[-30,30]) {
            // Vertical clamp: nut is loaded from the hollow end before assembly.
            translate([xx,0,-sleeve_h/2-1]) cylinder(d=m4_clear,h=sleeve_h+2);
            // Symmetric side pockets allow one sleeve STL on left/right rails.
            for(side=[-1,1]) {
                translate([xx,-side*(sleeve_w/2+1),0]) rotate([-side*90,0,0])
                    cylinder(d=m4_clear,h=sleeve_w/2+8);
            }
        }
    }
}

module clamp_sleeve_y() { rotate([0,0,90]) clamp_sleeve_x(); }

module low_clamp_sleeve(width=43,height=13) {
    difference() {
        cube([sleeve_len,width,height],center=true);
        cube([sleeve_len+2,width-2.6,height-2.6],center=true);
        for(xx=[-30,30]) {
            translate([xx,0,-height/2-1]) cylinder(d=m4_clear,h=height+2);
            translate([xx,-width/2-1,0]) rotate([-90,0,0])
                cylinder(d=m4_clear,h=width+2);
        }
    }
}

module joint_holes_x(joint_x,side,outer_only=true,height=rail_h) {
    // One top and one outer-side M4 clamp per half-joint, 30 mm from interface.
    inside_x=joint_x + (joint_x==rear_x ? 30 : -30);
    translate([inside_x,side*rail_y,rail_z-1]) cylinder(d=m4_clear,h=height+2);
    translate([inside_x,side*(rail_y+rail_w/2+1),rail_z+height/2])
        rotate([90,0,0]) cylinder(d=m4_clear,h=rail_w+2);
}

module end_lock_holes_local(length,side,terminal_front=false) {
    positions=terminal_front ? [-length/2+30] : [-length/2+30,length/2-30];
    for(xx=positions) {
        translate([xx,0,-1]) cylinder(d=m4_clear,h=rail_h+2);
        translate([xx,side*(rail_w/2+1),rail_h/2]) rotate([90,0,0])
            cylinder(d=m4_clear,h=rail_w+2);
    }
}

module grid_tower_top(x) {
    translate([x,0,rail_h/2]) cylinder(d=13,h=rail_h-2*rail_wall,center=true);
}

module grid_tower_side(x,side) {
    translate([x,0,rail_h/2]) rotate([90,0,0])
        cylinder(d=13,h=rail_w-2*rail_wall,center=true);
}

module accessory_grid_cut_x(length,side,xs) {
    for(xx=xs) {
        translate([xx,0,-1]) cylinder(d=m4_clear,h=rail_h+2);
        translate([xx,side*(rail_w/2+1),rail_h/2]) rotate([90,0,0])
            cylinder(d=m4_clear,h=rail_w+2);
    }
}

module rail_extension_v3(side=1) {
    length=front_joint_x-rear_joint_x;
    xs=[-17.5,7.5]; // global X=100,125; clear of both 60 mm insertions.
    difference() {
        union() {
            translate([0,0,rail_h/2]) box_x(length);
            for(xx=xs) { grid_tower_top(xx); grid_tower_side(xx,side); }
        }
        end_lock_holes_local(length,side);
        accessory_grid_cut_x(length,side,xs);
    }
}

module motor_module_v3(side=1) {
    difference() {
        drive_motor_inline(side);
        end_lock_holes_local(200,side);
    }
}

module p278_terminal_socket(side=1,x_local=-60,width=50) {
    socket_centre_y=-side*(rail_w/2+p278_socket_len/2-0.5);
    difference() {
        // One millimetre of root overlap fuses the hollow socket to the inner
        // rail wall without entering the X200 longitudinal-sleeve cavity.
        translate([x_local,socket_centre_y,10])
            box_y(p278_socket_len+1,width,20,rail_wall);
        // Retained Y150 accessory/diagonal station.
        translate([x_local,-side*70,-1]) cylinder(d=m4_clear,h=22);
        translate([x_local-width/2-1,-side*70,10]) rotate([0,90,0])
            cylinder(d=m4_clear,h=width+2);
        // Sleeve lock 30 mm into the socket from the Y130 interface (Y160).
        translate([x_local,-side*60,-1]) cylinder(d=m4_clear,h=22);
        translate([x_local-width/2-1,-side*60,10]) rotate([0,90,0])
            cylinder(d=m4_clear,h=width+2);
        // Retain the drive-test C-hook envelope at global X310/Y+/-185.
        if(width==20)
            for(xedge=[x_local-width/2,x_local+width/2])
                translate([xedge,-side*35,-1]) cylinder(d=9,h=22);
    }
}

module p278_terminal_socket_cavity(side=1,x_local=-60,width=50) {
    // Open the rail inner wall into the socket.  Stop 0.5 mm before the
    // existing X200 longitudinal sleeve envelope at the X240 station.
    translate([x_local,-side*(rail_w/2+p278_socket_len/2),10])
        cube([width-2*rail_wall,p278_socket_len+6,20-2*rail_wall],center=true);
}

module front_terminal_v3(side=1) {
    // Keep the validated rear socket and motor land, remove the old Gamma foot.
    difference() {
        union() {
            intersection() {
                drive_motor_inline(side);
                translate([-100,-rail_w,-1]) cube([130,2*rail_w,rail_h+2]);
            }
            translate([26.8,-rail_w/2,0]) cube([3.2,rail_w,rail_h]);
            p278_terminal_socket(side,-60,50); // global X240
            p278_terminal_socket(side,0,20);   // global X300
        }
        // 2026-10-15 fix: the part spans local X-100..30, so the rear joint
        // hole must be 30 mm from X-100 (X-70), not at the part centre (X-35).
        translate([-35,0,0]) end_lock_holes_local(130,side,true);
        p278_terminal_socket_cavity(side,-60,50);
        p278_terminal_socket_cavity(side,0,20);
    }
}

module p278_crossbar_half(width=50) {
    // Short half to the terminal-socket interface.  The sleeves back every
    // retained grid bore, so no internal tower may block insertion.
    difference() {
        translate([0,p278_crossbar_len/2,10])
            box_y(p278_crossbar_len,width,20,rail_wall);
        for(yy=[30,75,100,125]) {
            translate([0,yy,-1]) cylinder(d=m4_clear,h=22);
            translate([-(width/2+1),yy,10]) rotate([0,90,0])
                cylinder(d=m4_clear,h=width+2);
        }
    }
}

module p278_terminal_sleeve(width=43,height=13) {
    // 120 mm: 60 mm in the crossbar + 60 mm in the integrated socket.
    difference() {
        low_clamp_sleeve(width,height);
        // Retained grid bores at global Y75/100/125/150 relative to Y130.
        // Y100 is also the required -30 mm sleeve lock already cut above.
        for(xx=[-55,-5,20]) {
            translate([xx,0,-height/2-1]) cylinder(d=m4_clear,h=height+2);
            translate([xx,-width/2-1,0]) rotate([-90,0,0])
                cylinder(d=m4_clear,h=width+2);
        }
    }
}

module half_crossbar(length=220,width=50,height=30,grid=true,grid_ys=[75,100,125,150]) {
    ys=grid ? grid_ys : [];
    difference() {
        union() {
            translate([0,length/2,height/2]) box_y(length,width,height);
            for(yy=ys) {
                translate([0,yy,height/2]) cylinder(d=13,h=height-2*rail_wall,center=true);
                translate([0,yy,height/2]) rotate([0,90,0])
                    cylinder(d=13,h=width-2*rail_wall,center=true);
            }
        }
        // Sleeve/locking zones occupy 0..60 and 160..220; grid stays between.
        for(yy=[30,length-30]) {
            translate([0,yy,-1]) cylinder(d=m4_clear,h=height+2);
            translate([-(width/2+1),yy,height/2]) rotate([0,90,0])
                cylinder(d=m4_clear,h=width+2);
        }
        for(yy=ys) {
            translate([0,yy,-1]) cylinder(d=m4_clear,h=height+2);
            translate([-(width/2+1),yy,height/2]) rotate([0,90,0])
                cylinder(d=m4_clear,h=width+2);
        }
    }
}

// P2-65 end connector for the X240/X300 crossbars.  The 60 mm arm enters the
// crossbar and uses its existing top/side lock holes.  The narrow upright at
// the rail face takes one field-drilled through-M4 across the complete front
// terminal; the bolt head is outboard and the washer/nut remain accessible
// inboard.  A small end notch in the new crossbar is required for the upright.
module crossbar_t_clamp(width=50,height=20,tab_width=24,grid_pass=true) {
    arm_w=width-7;
    arm_h=height-7;
    arm_y=front_crossbar_len-insert_len/2;
    difference() {
        union() {
            translate([0,arm_y,height/2])
                cube([arm_w,insert_len,arm_h],center=true);
            translate([0,front_crossbar_len-3,(height+30)/2])
                cube([tab_width,6,30-height],center=true);
            // Neck joins the internal arm to the exposed upright through the
            // matching open notch at the very end of the new crossbar.
            translate([0,front_crossbar_len-3,(arm_h+height+2)/2])
                cube([min(tab_width,arm_w),6,height+2-arm_h],center=true);
        }
        // Crossbar lock screws, 30 mm from its rail-side end.
        translate([0,front_crossbar_len-30,-1])
            cylinder(d=m4_clear,h=height+2);
        translate([-(width/2+1),front_crossbar_len-30,height/2])
            rotate([0,90,0]) cylinder(d=m4_clear,h=width+2);
        // Preserve the Y150 accessory/ballast/diagonal grid through the arm.
        if(grid_pass) {
            translate([0,150,-1]) cylinder(d=m4_clear,h=height+2);
            translate([-(width/2+1),150,height/2]) rotate([0,90,0])
                cylinder(d=m4_clear,h=width+2);
        }
        // Field-drilled rail bolt at global Z46.
        translate([0,front_crossbar_len+1,p265_rail_bolt_z]) rotate([90,0,0])
            cylinder(d=m4_clear,h=12);
    }
}

module p265_crossbar_half(width=50,tab_width=24) {
    difference() {
        // 2026-10-16 fix: no internal grid tower at Y150, it sat inside the
        // 60 mm T-clamp insertion zone (Y135..195) and blocked the arm. The
        // solid arm now backs the Y150 grid holes instead of a tower.
        half_crossbar(front_crossbar_len,width,20,true,grid_ys=[75,100,125]);
        translate([0,150,-1]) cylinder(d=m4_clear,h=20+2);
        translate([-(width/2+1),150,20/2]) rotate([0,90,0])
            cylinder(d=m4_clear,h=width+2);
        // Open only the end/top strip occupied by the connector neck.
        translate([0,front_crossbar_len-3,24])
            cube([tab_width+0.6,6.2,16],center=true);
        // The drive-test plate's existing M6 C-hook at global X310/Y+/-185
        // clipped the old 20 mm X300 tube edge.  Symmetric 9 mm edge reliefs
        // keep one handed STL and leave 1.25 mm nominal radial clearance.
        if(width==20)
            for(xx=[-width/2,width/2])
                translate([xx,185,-1]) cylinder(d=9,h=22);
    }
}

module corner_clamp_positive() {
    // 2026-10-16 fix: the rail (along +X) runs to its end at the corner
    // origin; the rear crossbar (along -Y) now stops at the rail's inner face
    // (local y=-rail_w/2) instead of overlapping 25x25 mm with it.
    // Rail arm: x 0..60 inside the rail, bolt 30 mm from the rail end.
    // Crossbar arm: y -25..-85 inside the crossbar, bolt 30 mm from its end.
    // Knuckle: solid block in the free pocket behind the rail end and beside
    // the crossbar end, tying the two arms together, flush with both tubes.
    xb=-rail_w/2-insert_len/2;   // crossbar bolt, local y
    difference() {
        union() {
            translate([insert_len/2,0,0]) cube([insert_len,sleeve_w,sleeve_h],center=true);
            translate([0,xb,0]) cube([sleeve_w,insert_len,sleeve_h],center=true);
            translate([-rail_w/4+0.005,0,0]) cube([rail_w/2+0.01,rail_w,sleeve_h],center=true); // sleeve height: prints flat
        }
        translate([30,0,-rail_h/2-1]) cylinder(d=m4_clear,h=rail_h+2);
        translate([0,xb,-rail_h/2-1]) cylinder(d=m4_clear,h=rail_h+2);
        translate([30,sleeve_w/2+1,0]) rotate([90,0,0])
            cylinder(d=m4_clear,h=sleeve_w+2);
        translate([-sleeve_w/2-1,xb,0]) rotate([0,90,0])
            cylinder(d=m4_clear,h=sleeve_w+2);
    }
}

module corner_clamp(side=1) {
    if(side>0) corner_clamp_positive();
    else mirror([0,1,0]) corner_clamp_positive();
}

module torsion_diagonal_half() {
    // One half of a diagonal from [X240,Y+/-150] to the centre [X270,Y0].
    // The first hole is the single-shear crossbar pin; the last two mate to
    // torsion_joiner.  Use a wide washer/metal backing plate at the endpoint.
    length=sqrt(30*30+150*150)+12;
    difference() {
        translate([length/2,0,5]) cube([length,20,10],center=true);
        for(xx=[12,length-28,length-8])
            translate([xx,0,-1]) cylinder(d=m4_clear,h=12);
    }
}

module torsion_joiner() {
    difference() {
        translate([0,0,3]) cube([64,34,6],center=true);
        for(xx=[-28,-8,8,28]) translate([xx,0,-1]) cylinder(d=m4_clear,h=8);
    }
}

module sleeve_fit_coupon() {
    difference() {
        // 2026-10-15 fix: no internal boss. Real rails are hollow at the
        // lock holes; the boss stopped the sleeve ~23 mm in, before the hole.
        translate([0,0,rail_h/2]) box_x(80);
        translate([10,0,-1]) cylinder(d=m4_clear,h=rail_h+2);
        translate([10,rail_w/2+1,rail_h/2]) rotate([90,0,0])
            cylinder(d=m4_clear,h=rail_w+2);
    }
}

module ballast_tab() {
    difference() {
        translate([0,0,4]) cube([70,30,8],center=true);
        for(xx=[-25,0,25]) translate([xx,0,-1]) cylinder(d=m4_clear,h=10);
    }
}

module temporary_gamma_context(side=1) {
    if(show_temporary_gamma) %color(temp_color)
        if(side>0)
            translate([200,220,22]) import("stl-v2/front_motor_gamma_left.stl");
        else
            translate([200,-220,22]) import("stl-v2/front_motor_gamma_right.stl");
}

module diagonal_half_placed(e) {
    // e = crossbar end hole [x,y]; the bar runs to the centre [270,0].
    d=[270-e[0],-e[1]]; a=atan2(d[1],d[0]);
    translate([e[0],e[1],rail_z+20]) rotate([0,0,a]) translate([-12,0,0])
        torsion_diagonal_half();
}

module single_diagonal_assembly() {
    diagonal_half_placed([x240,150]);
    diagonal_half_placed([x300,-150]);
    a=atan2(-150-150,x300-x240);
    translate([270,0,rail_z+30]) rotate([0,0,a]) torsion_joiner();
}

module torsion_side_brace() {
    // One reusable brace for either side. Installed between the facing walls
    // of X240 and X300 at Y150 -> Y125; no material drops below rail Z22.
    tab_t=4.8;
    p0=[tab_t/2,0];
    p1=[20+tab_t/2,-25];
    difference() {
        union() {
            // 20x15 diagonal body, with 23x23 end lands for >=9 mm clear edge
            // distance around the transverse M4 holes.
            hull() for(p=[p0,p1])
                translate([p[0],p[1],p278_diagonal_h/2])
                    cube([tab_t,p278_diagonal_w,p278_diagonal_h],center=true);
            for(p=[p0,p1])
                translate([p[0],p[1],p278_side_brace_tab_h/2])
                    cube([tab_t,p278_side_brace_tab,p278_side_brace_tab_h],center=true);
        }
        for(p=[p0,p1])
            translate([p[0]-4,p[1],p278_side_brace_tab_h/2]) rotate([0,90,0])
                cylinder(d=m4_clear,h=12.8);
        // X240 land only: clear the future basket-floor lower corner while
        // retaining >9 mm material from the Ø4.5 bore edge to the relief.
        translate([-1,-12,19.5]) cube([tab_t+2,2.6,5]);
    }
}

module p278_side_braces_assembly() {
    for(side=[-1,1])
        translate([265.1,side*150,rail_z-1.5])
            if(side>0) torsion_side_brace();
            else mirror([0,1,0]) torsion_side_brace();
}

module frame_assembly() {
    color(frame_color) {
        for(side=[-1,1]) {
            translate([rear_motor_x,side*rail_y,rail_z]) motor_module_v3(side);
            translate([(rear_joint_x+front_joint_x)/2,side*rail_y,rail_z])
                rail_extension_v3(side);
            translate([front_motor_x,side*rail_y,rail_z]) front_terminal_v3(side);
        }
        for(side=[-1,1]) {
            translate([rear_x,0,rail_z]) rotate([0,0,side>0?0:180])
                half_crossbar(rear_crossbar_len,grid_ys=[75,100,125]);
            translate([x240,0,rail_z]) rotate([0,0,side>0?0:180])
                p278_crossbar_half(50);
            translate([x300,0,rail_z]) rotate([0,0,side>0?0:180])
                p278_crossbar_half(20);
        }
        // P2-78: two short 20x15 braces in the X240-X300 bay.  Their bottom
        // is flush with the crossbar bottom; nothing hangs below the frame.
        p278_side_braces_assembly();
    }
    color(sleeve_color) {
        for(side=[-1,1],xx=[rear_joint_x,front_joint_x])
            translate([xx,side*rail_y,rail_z+rail_h/2]) clamp_sleeve_x();
        translate([rear_x,0,rail_z+rail_h/2]) clamp_sleeve_y();
        translate([x240,0,rail_z+20/2]) rotate([0,0,90]) low_clamp_sleeve(43,13);
        translate([x300,0,rail_z+20/2]) rotate([0,0,90]) low_clamp_sleeve(13,13);
        for(side=[-1,1]) {
            translate([x240,side*p278_crossbar_len,rail_z+10])
                rotate([0,0,90]) p278_terminal_sleeve(43,13);
            translate([x300,side*p278_crossbar_len,rail_z+10])
                rotate([0,0,90]) p278_terminal_sleeve(13,13);
        }
        translate([rear_x,rail_y,rail_z+rail_h/2]) corner_clamp(1);
        translate([rear_x,-rail_y,rail_z+rail_h/2]) corner_clamp(-1);
    }
    temporary_gamma_context(1); temporary_gamma_context(-1);
}

// Exact P2-65 collision probes.  Empty exports are passes; deliberate mating
// faces have zero-volume contact only.  The tyre probe uses the measured
// Y255.5 inner face and the conservative 85 mm loaded-radius sweep.
module p265_positive_connection(x,width,tab_width) {
    translate([x,0,rail_z]) {
        p265_crossbar_half(width,tab_width);
        crossbar_t_clamp(width,20,tab_width,true);
    }
}

module p265_front_fit0403_envelope() {
    // Same measured/assumed positive envelope used by the Phase-1 drive QA.
    translate([300,220,53]) cube([40,40,2],center=true);
    translate([300,239,77.5]) cube([40,2,47],center=true);
    translate([300,226.5,85]) rotate([90,0,0]) cylinder(d=40,h=25,center=true);
    translate([300,193,85]) rotate([90,0,0]) cylinder(d=38,h=42,center=true);
    translate([300,166,85]) rotate([90,0,0]) cylinder(d=40,h=12,center=true);
    translate([300,166,61]) cube([18,14,18],center=true);
    // Required P2-65 cable path: jog rearward above the deck, descend at
    // X275 while still inboard, then cross below the rail bottom (Z22).
    hull() {
        translate([300,166,53]) cube([8,8,8],center=true);
        translate([275,166,53]) cube([8,8,8],center=true);
    }
    hull() {
        translate([275,166,53]) cube([8,8,8],center=true);
        translate([275,185,16]) cube([8,8,8],center=true);
    }
    hull() {
        translate([275,185,16]) cube([8,8,8],center=true);
        translate([275,220,16]) cube([8,8,8],center=true);
    }
}

module p265_qa(target="x240_terminal") {
    if(target=="x240_terminal") intersection() {
        p265_positive_connection(x240,50,24);
        translate([front_motor_x,rail_y,rail_z]) front_terminal_v3(1);
    }
    else if(target=="x300_terminal") intersection() {
        p265_positive_connection(x300,20,11);
        translate([front_motor_x,rail_y,rail_z]) front_terminal_v3(1);
    }
    else if(target=="x300_ballast") intersection() {
        p265_positive_connection(x300,20,11);
        translate([360,0,58]) cube([160,480,12],center=true);
    }
    else if(target=="x300_chook") intersection() {
        p265_positive_connection(x300,20,11);
        for(xx=[310,330]) translate([xx,185,20]) cylinder(d=6.5,h=45);
    }
    else if(target=="x300_tyre") intersection() {
        // Low-profile M4 head, maximum 4 mm outboard projection.
        translate([x300,rail_y+rail_w/2+4,p265_rail_bolt_z+rail_z])
            rotate([90,0,0]) cylinder(d=8,h=4);
        translate([x300,297,85]) rotate([90,0,0])
            cylinder(r=85,h=83,center=true);
    }
    else if(target=="x300_fit0403") intersection() {
        p265_positive_connection(x300,20,11);
        p265_front_fit0403_envelope();
    }
}

if(part=="assembly") frame_assembly();
else if(part=="rear_motor_left") motor_module_v3(1);
else if(part=="rear_motor_right") motor_module_v3(-1);
else if(part=="rail_extension_left") rail_extension_v3(1);
else if(part=="rail_extension_right") rail_extension_v3(-1);
else if(part=="front_terminal_left") front_terminal_v3(1);
else if(part=="front_terminal_right") front_terminal_v3(-1);
else if(part=="rear_crossbar_half") half_crossbar(rear_crossbar_len,grid_ys=[75,100,125]);
else if(part=="x240_crossbar_half") p278_crossbar_half(50);
else if(part=="x300_crossbar_half") p278_crossbar_half(20);
else if(part=="x240_terminal_sleeve") p278_terminal_sleeve(43,13);
else if(part=="x300_terminal_sleeve") p278_terminal_sleeve(13,13);
else if(part=="x240_t_clamp") crossbar_t_clamp(50,20,24,true);
else if(part=="x300_t_clamp") crossbar_t_clamp(20,20,11,true);
else if(part=="clamp_sleeve") clamp_sleeve_x();
else if(part=="x240_clamp_sleeve") low_clamp_sleeve(43,13);
else if(part=="x300_clamp_sleeve") low_clamp_sleeve(13,13);
else if(part=="corner_clamp_left") corner_clamp(1);
else if(part=="corner_clamp_right") corner_clamp(-1);
else if(part=="torsion_diagonal_half") torsion_diagonal_half();
else if(part=="torsion_joiner") torsion_joiner();
else if(part=="torsion_side_brace") rotate([0,0,45]) torsion_side_brace();
else if(part=="sleeve_fit_coupon") sleeve_fit_coupon();
else if(part=="ballast_tab") ballast_tab();
else if(part=="qa_p265_x240_terminal") p265_qa("x240_terminal");
else if(part=="qa_p265_x300_terminal") p265_qa("x300_terminal");
else if(part=="qa_p265_x300_ballast") p265_qa("x300_ballast");
else if(part=="qa_p265_x300_chook") p265_qa("x300_chook");
else if(part=="qa_p265_x300_tyre") p265_qa("x300_tyre");
else if(part=="qa_p265_x300_fit0403") p265_qa("x300_fit0403");
else assert(false,str("Unknown v3 part: ",part));

assert(front_end_x==330,"Phase-2 rails must end at X330");
assert(front_joint_x+130==front_end_x,"Terminal motor segment length mismatch");
assert(2*insert_len==sleeve_len,"Sleeve must engage 60 mm per side");
