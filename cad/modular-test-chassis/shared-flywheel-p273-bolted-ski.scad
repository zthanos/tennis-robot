// P2-73/P2-74 printable R75/75 ski and X300 mounting brackets.
// Units mm; +X front, +Y left, +Z up. No launcher/arch attachment.
use <modular-chassis-v3.scad>
use <shared-flywheel-transition-p252.scad>
use <shared-flywheel-p267-tilted-arch.scad>
use <shared-flywheel-p268-upright-wall.scad>

p273_part = "assembly";
p273_angle = 15;
p273_height = 0;       // installed datum; permitted service adjustment 0..+20
p273_rail_y = 55;      // accepted P2-74 selector rails
p273_shell_t = 2;       // exact frozen p267_ski strip thickness
p273_inner_w = 94;
p273_wall_t = 3;
p273_wall_h = 18;
p273_slot_travel = 20;
p273_m4 = 4.5;
p273_probe_x = 400;
p273_check_angle = -10;
p273_boss_x = 315;
p273_boss_z_add = 0;
p273_probe_i = 1;
// P2-85: per-side wall fork. It hangs on the X315 bolt (replacing the
// retrofit spacer), clamps both edges of the ski wall at arc t26..35 and is
// locked to the bracket by a second bolt in the same vertical slot.
p285_fork_s = 14;      // vertical spacing X315 bolt -> fork lock bolt
p285_lip_t0 = 26;      // fork lip arc range on the ski wall
p285_lip_t1 = 35;
p285_lip_clear = 0.3;  // lip-to-wall clearance (XZ)
p285_web_y0 = 61;      // inner web face, unchanged from P2-73
p285_web_t = 8;
p285_rear_mat = 10;    // material behind the slot (X), min spec 8
p285_front_mat = 12;   // free front ligament; 12 keeps bearing bending <10 MPa
p285_end_mat = 6;      // material above/below each slot (Z)
p285_grow = 0;         // clearance-margin probes only
p285_ski_bosses = false; // P2-85 ski is boss-free (fork hub bears on wall)
$fn = 48;

function p273_surface_z(x) =
    x >= 383 ? 10 : 10 + 75*(1-cos(asin((383-x)/75)));
function p273_bolt_z(x) = p273_surface_z(x) + 11;
function p273_mount_z(x) = p273_bolt_z(x) + p273_boss_z_add;
function p273_segment_w(i) = min(94,61+i*6);
function p273_wall_enabled(i) = i < 3 || i > 5;
function p285_b1() = [p273_boss_x,p273_mount_z(p273_boss_x)];
function p285_b1f() = p285_b1()+[0,p285_fork_s];
function p285_slot_len() = p273_slot_travel+p285_fork_s;

module p273_xz_strip(p0,p1,width=p273_inner_w,t=p273_shell_t) {
    dx=p1[0]-p0[0]; dz=p1[1]-p0[1]; l=sqrt(dx*dx+dz*dz); a=atan2(dz,dx);
    translate([(p0[0]+p1[0])/2,0,(p0[1]+p1[1])/2])
        rotate([0,-a,0]) cube([l+.8,width,t],center=true);
}

module p273_wall_segment(i) {
    pts=p267_ski_points(75); p0=pts[i]; p1=pts[i+1];
    dx=p1[0]-p0[0]; dz=p1[1]-p0[1]; l=sqrt(dx*dx+dz*dz); a=atan2(dz,dx);
    for(side=[-1,1])
        translate([(p0[0]+p1[0])/2,side*(p273_segment_w(i)+p273_wall_t)/2,
                   (p0[1]+p1[1])/2+p273_wall_h/2])
            rotate([0,-a,0]) cube([l+.8,p273_wall_t,p273_wall_h],center=true);
}

module p273_floor_only() {
    pts=p267_ski_points(75);
    union() for(i=[0:len(pts)-2])
        p273_xz_strip(pts[i],pts[i+1],p273_segment_w(i),p273_shell_t);
}

module p273_ski_raw() {
    pts=p267_ski_points(75);
    union() {
        // Exact frozen floor: same points, 2 mm thickness and 61->94 mm width.
        for(i=[0:len(pts)-2])
            p273_xz_strip(pts[i],pts[i+1],p273_segment_w(i),p273_shell_t);
        // The D200 sweep permits the frozen 61 mm flat floor, but no tall
        // wall there. Sidewalls therefore start at the first arc segment.
        for(side=[-1,1],i=[1:len(pts)-2]) if(p273_wall_enabled(i)) {
            p0=pts[i]; p1=pts[i+1];
            dx=p1[0]-p0[0]; dz=p1[1]-p0[1]; l=sqrt(dx*dx+dz*dz); a=atan2(dz,dx);
            translate([(p0[0]+p1[0])/2,side*(p273_segment_w(i)+p273_wall_t)/2,
                       (p0[1]+p1[1])/2+p273_wall_h/2])
                rotate([0,-a,0]) cube([l+.8,p273_wall_t,p273_wall_h],center=true);
        }
        // Only the proven-clear rear X315 point is used.
        // 2026-10-19 fix: boss Y49.5..60.5. It overlaps the 94 mm wall
        // (outer face Y50) by 0.5 mm and keeps 0.5 mm to the bracket web
        // (Y61). The 2026-10-18 version (Y50.5..60.5) floated 0.5 mm off the
        // wall, so slicers dropped it.
        // P2-85: bosses dropped; the fork hub is the bearing spacer.
        if(p285_ski_bosses) for(side=[-1,1],x=[p273_boss_x])
            translate([x,side*55,p273_mount_z(x)]) rotate([90,0,0])
                cylinder(d=11,h=11,center=true);
    }
}

// Diagnostic-only frozen riding shell/walls without mounting bosses.
module p273_ski_path_only() {
    pts=p267_ski_points(75);
    union() {
        for(i=[0:len(pts)-2])
            p273_xz_strip(pts[i],pts[i+1],p273_segment_w(i),p273_shell_t);
        for(side=[-1,1],i=[1:len(pts)-2]) if(p273_wall_enabled(i)) {
            p0=pts[i]; p1=pts[i+1];
            dx=p1[0]-p0[0]; dz=p1[1]-p0[1]; l=sqrt(dx*dx+dz*dz); a=atan2(dz,dx);
            translate([(p0[0]+p1[0])/2,side*(p273_segment_w(i)+p273_wall_t)/2,
                       (p0[1]+p1[1])/2+p273_wall_h/2])
                rotate([0,-a,0]) cube([l+.8,p273_wall_t,p273_wall_h],center=true);
        }
    }
}

module p273_ski(height=p273_height) {
    translate([0,0,height]) difference() {
        p273_ski_raw();
        for(side=[-1,1],x=[p273_boss_x])
            translate([x,side*56,p273_mount_z(x)]) rotate([90,0,0])
                cylinder(d=p273_m4,h=24,center=true);
    }
}

module p273_y_slot(x,y,z,len=p273_slot_travel,d=p273_m4) {
    hull() for(dz=[-len/2,len/2])
        translate([x,y,z+dz]) rotate([90,0,0]) cylinder(d=d,h=10,center=true);
}

// P2-85 bracket. The P2-73 slot ran open above the web top and left two
// 2.75 mm teeth printed across their layers; it broke on assembly. The slot
// is now closed (>=6 mm above/below, >=8 mm each side) and long enough for
// the X315 bolt plus the fork lock bolt over the 20 mm travel. Web 8 mm,
// inner face Y61 kept, grown outward. Print on the Y61 face.
module p285_web_profile(g=0) {
    b=p285_b1(); r=p273_m4/2; sz=r+p285_end_mat;
    offset(delta=g) offset(r=2) offset(delta=-2) hull() {
        translate([285,42]) square([28,8]);
        translate([b[0]-r-p285_rear_mat,b[1]-sz])
            square([2*r+p285_rear_mat+p285_front_mat,p285_slot_len()+2*sz]);
    }
}

module p285_slot(p,g=0) {
    hull() for(dz=[0,p285_slot_len()])
        translate([p[0],0,p[1]+dz]) rotate([90,0,0])
            cylinder(d=p273_m4-2*g,h=40,center=true);
}

module p285_rib(x0,w,z_web,y_foot) {
    // Foot->web gusset in a YZ plane, outboard of the web.
    translate([x0,0,0]) rotate([90,0,90]) linear_extrude(w)
        polygon([[p285_web_y0+p285_web_t-.5,47.5],
                 [p285_web_y0+p285_web_t-.5,z_web],[y_foot,47.5]]);
}

module p273_mount_bracket(side=1,g=p285_grow) {
    yo=p285_web_y0+p285_web_t;
    mirror([0,side>0?0:1,0]) difference() {
        union() {
            translate([0,yo+g,0]) rotate([90,0,0])
                linear_extrude(p285_web_t+2*g) p285_web_profile(g);
            // Foot on the existing X300 top-grid holes Y75/Y100, unchanged.
            translate([285-g,65,42]) cube([30+2*g,45+g,6+g]);
            // R4 root fillet along the foot/web outer corner.
            translate([285,yo-.5,47.5]) difference() {
                cube([30,4.5,4.5]);
                translate([-1,4.5,4.5]) rotate([0,90,0]) cylinder(r=4,h=32);
            }
            // Rear tall gusset; front gusset stays below the X315 nut sweep.
            p285_rib(285,5,78,98);
            p285_rib(309,5,56,88);
        }
        for(y=[75,100]) translate([300,y,39]) cylinder(d=p273_m4,h=14);
        // At installed height 0 the X315 bolt sits at the lower slot end and
        // the lock bolt 14 mm above it; both travel 20 mm (ski + module 1:1).
        translate([0,56,0]) p285_slot(p285_b1(),g);
    }
}

// ---- P2-85 wall fork (one per side, mirrored) ---------------------------
function p285_arc(t,s) = [383-(75-s)*sin(t), 85-(75-s)*cos(t)];
p285_arm_end = [346,57];

module p285_fork_profile() {
    b=p285_b1(); f=p285_b1f();
    union() {
        hull() { translate(b) circle(r=7); translate(f) circle(r=7); }
        hull() { translate(b) circle(r=8); translate(p285_arm_end) circle(r=8); }
        hull() {
            translate(p285_arm_end) circle(r=8);
            polygon(concat([for(t=[p285_lip_t0:1:p285_lip_t1]) p285_arc(t,21)],
                           [for(t=[p285_lip_t1:-1:p285_lip_t0]) p285_arc(t,-5)]));
        }
    }
}

// Ski walls grown by the lip clearance in XZ and spanning the lip depth in Y.
module p285_wall_clear() { p285_wall_clear_c(p285_lip_clear); }
module p285_wall_clear_c(c) {
    pts=p267_ski_points(75);
    for(i=[1:len(pts)-2]) if(p273_wall_enabled(i)) {
        p0=pts[i]; p1=pts[i+1];
        dx=p1[0]-p0[0]; dz=p1[1]-p0[1]; l=sqrt(dx*dx+dz*dz); a=atan2(dz,dx);
        translate([(p0[0]+p1[0])/2,48.3,(p0[1]+p1[1])/2+p273_wall_h/2])
            rotate([0,-a,0]) cube([l+.8+2*c,4.6,p273_wall_h+2*c],center=true);
    }
}

p285_fork_y_in = 48;     // lips reach 2 mm over the 3 mm wall (Y47..50)
p285_fork_y_out = 60.7;  // 0.3 mm print tolerance to the web inner face
module p285_fork(side=1,height=p273_height,g=p285_grow) {
    b=p285_b1(); f=p285_b1f();
    translate([0,0,height]) mirror([0,side>0?0:1,0]) difference() {
        union() {
            difference() {
                translate([0,p285_fork_y_out+g,0]) rotate([90,0,0])
                    linear_extrude(p285_fork_y_out-p285_fork_y_in+2*g)
                        offset(delta=g) p285_fork_profile();
                p285_wall_clear();
            }
            // Hub face bears on the wall at Y50 (old retrofit spacer role).
            translate([b[0],50,b[1]]) rotate([-90,0,0]) cylinder(d=10,h=1);
        }
        for(p=[b,f]) translate([p[0],0,p[1]]) rotate([-90,0,0])
            translate([0,0,40]) cylinder(d=p273_m4,h=30);
        // Captive DIN933 M4 hex head (AF7, k2.8) for the lock bolt.
        translate([f[0],p285_fork_y_in-1,f[1]]) rotate([-90,0,0])
            rotate([0,0,30]) cylinder(d=7.25/cos(30),h=4,$fn=6);
    }
}

// Fasteners per side at ski height h: X315 low head inside, shank, Ø12
// washer + nyloc outside; lock bolt (head captive in the fork) shank +
// washer + nyloc outside.
module p273_fastener_envelopes(height=p273_height) {
    for(side=[-1,1]) mirror([0,side>0?0:1,0]) {
        b=p285_b1(); f=p285_b1f();
        translate([b[0],0,b[1]+height]) rotate([-90,0,0]) {
            translate([0,0,43.5]) cylinder(d=8,h=3.5);
            translate([0,0,47]) cylinder(d=p273_m4-.1,h=22);
        }
        translate([f[0],0,f[1]+height]) rotate([-90,0,0])
            translate([0,0,51]) cylinder(d=p273_m4-.1,h=18);
        // X315 M4x30 ends at Y77; lock M4x30 (hex head bearing Y51) at Y81.
        for(p=[[b,78.5],[f,82]]) translate([p[0][0],0,p[0][1]+height])
            rotate([-90,0,0]) translate([0,0,p285_web_y0+p285_web_t])
                cylinder(d=12,h=p[1]-(p285_web_y0+p285_web_t));
    }
}

module p285_forks(height=p273_height) {
    for(side=[-1,1]) p285_fork(side,height);
}

module p273_ski_mount_assembly(height=p273_height) {
    color([.08,.62,.82]) p273_ski(height);
    color([.82,.48,.12]) for(side=[-1,1]) p273_mount_bracket(side);
    color([.55,.30,.75]) for(side=[-1,1]) p285_fork(side,height);
    color("silver") p273_fastener_envelopes(height);
}

module p273_selector(angle=p273_angle) {
    p268_xz_strip([278,42],p268_mouth_floor(angle),72,5);
    p268_xz_rails_at_y([278,42],p268_mouth_floor(angle),p273_rail_y,3,16);
}

module p273_launcher_sweep() {
    union() for(a=[-10:1:20]) p253_frozen_launcher(a);
}
module p273_hard_without_throat(a) {
    module_pose(a) {
        upper_plate();
        for(yy=[-129,129]) wheel_and_motor(yy);
        trunnions_and_standoffs(); side_cheeks();
    }
}
module p273_hard_without_throat_sweep() {
    union() for(a=[-10:1:20]) p273_hard_without_throat(a);
}
module p273_walls_only(height=p273_height) {
    pts=p267_ski_points(75);
    translate([0,0,height]) union()
        for(i=[1:len(pts)-2]) if(p273_wall_enabled(i)) p273_wall_segment(i);
}
module p273_throat_sweep() {
    union() for(a=[-10:1:20]) module_pose(a) moving_throat_and_guide();
}
module p273_plate_sweep() {
    union() for(a=[-10:1:20]) module_pose(a) upper_plate();
}
module p273_wheel_sweep() {
    union() for(a=[-10:1:20]) module_pose(a)
        for(yy=[-129,129]) wheel_and_motor(yy);
}
module p273_closed_feed() {
    p268_fixed_feed_channel(); p268_feed_door(0,false);
}
module p273_adjustment_sweep() {
    union() {
        for(h=[0:1:20]) {
            p273_ski(h); p273_fastener_envelopes(h); p285_forks(h);
        }
        for(side=[-1,1]) p273_mount_bracket(side);
    }
}
module p273_coupled_hard_adjustment_probe() {
    union() for(h=[0:1:20]) intersection() {
        union() {
            p273_ski(h); p273_fastener_envelopes(h); p285_forks(h);
            for(side=[-1,1]) p273_mount_bracket(side);
        }
        translate([0,0,h]) p273_hard_without_throat_sweep();
    }
}
module p273_coupled_wall_throat_probe() {
    union() for(h=[0:1:20]) intersection() {
        p273_walls_only(h);
        translate([0,0,h]) p273_throat_sweep();
    }
}

if(p273_part=="assembly") {
    frame_assembly(); p273_ski_mount_assembly(); p273_selector(p273_angle);
}
else if(p273_part=="ski") p273_ski();
else if(p273_part=="bracket_left") p273_mount_bracket(1);
else if(p273_part=="bracket_right") p273_mount_bracket(-1);
else if(p273_part=="print_ski")
    // Tangent underside on the bed. Supports are confined to the steep arc;
    // the riding surface, walls and M4 boss holes stay support-free.
    translate([-300,0,-8.95]) p273_ski();
// P2-85: brackets print on the web face; use print_p285_bracket_left/right.
else if(p273_part=="print_bracket_left")
    translate([-285,106.6,-61]) rotate([90,0,0]) p273_mount_bracket(1);
else if(p273_part=="print_bracket_right")
    translate([-285,-41,-61]) rotate([-90,0,0]) p273_mount_bracket(-1);
else if(p273_part=="qa_selector") intersection() {
    p273_ski_mount_assembly(); p273_selector(p273_angle);
}
else if(p273_part=="qa_floor_minus_frozen") difference() {
    p273_floor_only(); p267_ski(75);
}
else if(p273_part=="qa_frozen_minus_floor") difference() {
    p267_ski(75); p273_floor_only();
}
else if(p273_part=="qa_ski_selector") intersection() {
    p273_ski(); p273_selector(p273_angle);
}
else if(p273_part=="qa_bracket_selector") intersection() {
    union() for(side=[-1,1]) p273_mount_bracket(side);
    p273_selector(p273_angle);
}
else if(p273_part=="qa_fastener_selector") intersection() {
    p273_fastener_envelopes(); p273_selector(p273_angle);
}
else if(p273_part=="qa_boss_selector") intersection() {
    union() for(side=[-1,1])
        translate([p273_probe_x,side*56,p273_bolt_z(p273_probe_x)])
            rotate([90,0,0]) cylinder(d=11,h=12,center=true);
    p273_selector(p273_angle);
}
else if(p273_part=="failure_side") rotate([-90,0,0]) {
    color([.08,.62,.82,.72]) p273_ski();
    color([.82,.48,.12,.72]) for(side=[-1,1]) p273_mount_bracket(side);
    color([.88,.68,.12,.72]) p273_selector(15);
    color("red") intersection() {
        union() for(side=[-1,1])
            translate([327,side*56,p273_bolt_z(327)]) rotate([90,0,0])
                cylinder(d=11,h=12,center=true);
        p273_selector(15);
    }
}
else if(p273_part=="failure_top") {
    color([.08,.62,.82,.72]) p273_ski();
    color([.82,.48,.12,.72]) for(side=[-1,1]) p273_mount_bracket(side);
    color([.88,.68,.12,.72]) p273_selector(15);
    color("red") intersection() {
        union() for(side=[-1,1])
            translate([327,side*56,p273_bolt_z(327)]) rotate([90,0,0])
                cylinder(d=11,h=12,center=true);
        p273_selector(15);
    }
}
else if(p273_part=="failure_side_2d") {
    color([.08,.62,.82]) linear_extrude(.10) projection()
        rotate([-90,0,0]) p273_ski();
    color([.82,.48,.12]) translate([0,0,.12]) linear_extrude(.10) projection()
        rotate([-90,0,0]) union() for(side=[-1,1]) p273_mount_bracket(side);
    color([.88,.68,.12]) translate([0,0,.24]) linear_extrude(.10) projection()
        rotate([-90,0,0]) p273_selector(15);
    color("red") translate([0,0,.40]) linear_extrude(.12) projection()
        rotate([-90,0,0]) intersection() {
            union() for(side=[-1,1])
                translate([327,side*56,p273_bolt_z(327)]) rotate([90,0,0])
                    cylinder(d=11,h=12,center=true);
            p273_selector(15);
        }
}
else if(p273_part=="failure_top_2d") {
    color([.08,.62,.82]) linear_extrude(.10) projection() p273_ski();
    color([.82,.48,.12]) translate([0,0,.12]) linear_extrude(.10) projection()
        union() for(side=[-1,1]) p273_mount_bracket(side);
    color([.88,.68,.12]) translate([0,0,.24]) linear_extrude(.10) projection()
        p273_selector(15);
    color("red") translate([0,0,.40]) linear_extrude(.12) projection()
        intersection() {
            union() for(side=[-1,1])
                translate([327,side*56,p273_bolt_z(327)]) rotate([90,0,0])
                    cylinder(d=11,h=12,center=true);
            p273_selector(15);
        }
}
else if(p273_part=="failure_r2_side_2d") {
    color([.08,.62,.82]) linear_extrude(.10) projection()
        rotate([-90,0,0]) p273_ski_path_only();
    color([.82,.48,.12]) translate([0,0,.12]) linear_extrude(.10) projection()
        rotate([-90,0,0]) union() for(side=[-1,1]) p273_mount_bracket(side);
    color([.95,.42,.06]) translate([0,0,.24]) linear_extrude(.10) projection()
        rotate([-90,0,0]) module_pose(-10)
            for(yy=[-129,129]) wheel_and_motor(yy);
    color("red") translate([0,0,.40]) linear_extrude(.12) projection()
        rotate([-90,0,0]) intersection() {
            p273_ski_path_only();
            module_pose(-10) for(yy=[-129,129]) wheel_and_motor(yy);
        }
}
else if(p273_part=="failure_r2_top_2d") {
    color([.08,.62,.82]) linear_extrude(.10) projection() p273_ski_path_only();
    color([.82,.48,.12]) translate([0,0,.12]) linear_extrude(.10) projection()
        union() for(side=[-1,1]) p273_mount_bracket(side);
    color([.95,.42,.06]) translate([0,0,.24]) linear_extrude(.10) projection()
        module_pose(-10) for(yy=[-129,129]) wheel_and_motor(yy);
    color("red") translate([0,0,.40]) linear_extrude(.12) projection()
        intersection() {
            p273_ski_path_only();
            module_pose(-10) for(yy=[-129,129]) wheel_and_motor(yy);
        }
}
else if(p273_part=="qa_launcher") intersection() {
    p273_ski_mount_assembly(); p273_launcher_sweep();
}
else if(p273_part=="qa_hard_without_throat") intersection() {
    p273_ski_mount_assembly(); p273_hard_without_throat_sweep();
}
else if(p273_part=="qa_walls_throat") intersection() {
    p273_walls_only(); p273_throat_sweep();
}
else if(p273_part=="qa_walls_throat_at_angle") intersection() {
    p273_walls_only(); module_pose(p273_check_angle) moving_throat_and_guide();
}
else if(p273_part=="qa_drive_hardware") intersection() {
    p273_ski_mount_assembly(); drive_package_envelopes();
}
else if(p273_part=="qa_adjust_selector") intersection() {
    p273_adjustment_sweep(); p273_selector(p273_angle);
}
else if(p273_part=="qa_adjust_hard") intersection() {
    p273_adjustment_sweep(); p273_hard_without_throat_sweep();
}
else if(p273_part=="qa_adjust_hard_coupled") p273_coupled_hard_adjustment_probe();
else if(p273_part=="qa_adjust_walls_throat") intersection() {
    union() for(h=[0:1:20]) p273_walls_only(h);
    p273_throat_sweep();
}
else if(p273_part=="qa_adjust_walls_throat_coupled")
    p273_coupled_wall_throat_probe();
else if(p273_part=="qa_adjust_drive") intersection() {
    p273_adjustment_sweep(); drive_package_envelopes();
}
else if(p273_part=="qa_adjust_closed_feed") intersection() {
    p273_adjustment_sweep(); p273_closed_feed();
}
else if(p273_part=="qa_adjust_frame_unintended") intersection() {
    union() for(h=[0:1:20]) p273_ski(h);
    frame_assembly();
}
else if(p273_part=="qa_ground") intersection() {
    p273_ski_mount_assembly(); translate([350,0,-50]) cube([500,500,100],center=true);
}
else if(p273_part=="qa_launcher_at_angle") intersection() {
    p273_ski_mount_assembly(); p253_frozen_launcher(p273_check_angle);
}
else if(p273_part=="qa_ski_launcher_at_angle") intersection() {
    p273_ski(); p253_frozen_launcher(p273_check_angle);
}
else if(p273_part=="qa_bracket_launcher_at_angle") intersection() {
    union() for(side=[-1,1]) p273_mount_bracket(side);
    p253_frozen_launcher(p273_check_angle);
}
else if(p273_part=="qa_fastener_launcher_at_angle") intersection() {
    p273_fastener_envelopes(); p253_frozen_launcher(p273_check_angle);
}
else if(p273_part=="qa_path_launcher_at_angle") intersection() {
    p273_ski_path_only(); p253_frozen_launcher(p273_check_angle);
}
else if(p273_part=="qa_path_wheels_at_angle") intersection() {
    p273_ski_path_only(); module_pose(p273_check_angle)
        for(yy=[-129,129]) wheel_and_motor(yy);
}
else if(p273_part=="qa_path_plate_at_angle") intersection() {
    p273_ski_path_only(); module_pose(p273_check_angle) upper_plate();
}
else if(p273_part=="qa_path_throat_at_angle") intersection() {
    p273_ski_path_only(); module_pose(p273_check_angle) moving_throat_and_guide();
}
else if(p273_part=="qa_path_cheeks_at_angle") intersection() {
    p273_ski_path_only(); module_pose(p273_check_angle) side_cheeks();
}
else if(p273_part=="qa_path_trunnions_at_angle") intersection() {
    p273_ski_path_only(); module_pose(p273_check_angle) trunnions_and_standoffs();
}
else if(p273_part=="qa_wall_segment_throat_at_angle") intersection() {
    p273_wall_segment(p273_probe_i);
    module_pose(p273_check_angle) moving_throat_and_guide();
}
else if(p273_part=="qa_boss_wheels_at_angle") intersection() {
    union() for(side=[-1,1])
        translate([p273_boss_x,side*56,p273_mount_z(p273_boss_x)])
            rotate([90,0,0]) cylinder(d=11,h=12,center=true);
    module_pose(p273_check_angle) for(yy=[-129,129]) wheel_and_motor(yy);
}
else if(p273_part=="qa_boss_plate_at_angle") intersection() {
    union() for(side=[-1,1])
        translate([p273_boss_x,side*56,p273_mount_z(p273_boss_x)])
            rotate([90,0,0]) cylinder(d=11,h=12,center=true);
    module_pose(p273_check_angle) upper_plate();
}
else if(p273_part=="qa_boss_throat_at_angle") intersection() {
    union() for(side=[-1,1])
        translate([p273_boss_x,side*56,p273_mount_z(p273_boss_x)])
            rotate([90,0,0]) cylinder(d=11,h=12,center=true);
    module_pose(p273_check_angle) moving_throat_and_guide();
}
else if(p273_part=="qa_boss_cheeks_at_angle") intersection() {
    union() for(side=[-1,1])
        translate([p273_boss_x,side*56,p273_mount_z(p273_boss_x)])
            rotate([90,0,0]) cylinder(d=11,h=12,center=true);
    module_pose(p273_check_angle) side_cheeks();
}
else if(p273_part=="qa_boss_trunnions_at_angle") intersection() {
    union() for(side=[-1,1])
        translate([p273_boss_x,side*56,p273_mount_z(p273_boss_x)])
            rotate([90,0,0]) cylinder(d=11,h=12,center=true);
    module_pose(p273_check_angle) trunnions_and_standoffs();
}
else if(p273_part=="qa_plate") intersection() {
    p273_ski_mount_assembly(); p273_plate_sweep();
}
else if(p273_part=="qa_wheels") intersection() {
    p273_ski_mount_assembly(); p273_wheel_sweep();
}
else if(p273_part=="qa_closed_feed") intersection() {
    p273_ski_mount_assembly(); p273_closed_feed();
}
else if(p273_part=="qa_frame_unintended") intersection() {
    // Intentional bracket-foot contact with X300 is excluded from this probe.
    p273_ski(); frame_assembly();
}
else if(p273_part=="side") rotate([-90,0,0]) {
    frame_assembly(); p273_ski_mount_assembly(); p273_selector(p273_angle);
    p253_frozen_launcher(p273_angle); p268_fixed_feed_channel(); p268_feed_door(0,false);
}
else if(p273_part=="top") {
    frame_assembly(); p273_ski_mount_assembly(); p273_selector(p273_angle);
    p253_frozen_launcher(p273_angle); p268_fixed_feed_channel(); p268_feed_door(0,false);
}
else if(p273_part=="assembly_side_2d") {
    color([.16,.48,.78]) linear_extrude(.10) projection()
        rotate([-90,0,0]) frame_assembly();
    color([.08,.62,.82]) translate([0,0,.12]) linear_extrude(.10) projection()
        rotate([-90,0,0]) p273_ski();
    color([.82,.48,.12]) translate([0,0,.24]) linear_extrude(.10) projection()
        rotate([-90,0,0]) union() for(side=[-1,1]) p273_mount_bracket(side);
    color([.95,.42,.06]) translate([0,0,.36]) linear_extrude(.10) projection()
        rotate([-90,0,0]) p253_frozen_launcher(15);
    color([.88,.68,.12]) translate([0,0,.48]) linear_extrude(.10) projection()
        rotate([-90,0,0]) p273_selector(15);
}
else if(p273_part=="assembly_top_2d") {
    color([.16,.48,.78]) linear_extrude(.10) projection() frame_assembly();
    color([.08,.62,.82]) translate([0,0,.12]) linear_extrude(.10) projection() p273_ski();
    color([.82,.48,.12]) translate([0,0,.24]) linear_extrude(.10) projection()
        union() for(side=[-1,1]) p273_mount_bracket(side);
    color([.95,.42,.06]) translate([0,0,.36]) linear_extrude(.10) projection()
        p253_frozen_launcher(15);
    color([.88,.68,.12]) translate([0,0,.48]) linear_extrude(.10) projection()
        p273_selector(15);
}
else if(p273_part=="print_layout") {
    p273_ski();
    translate([20,125,-42]) p273_mount_bracket(1);
    translate([70,-125,-42]) p273_mount_bracket(-1);
}

// 2026-10-19 retrofit for skis printed without bosses: M4 spacer tube that
// fills the 11 mm gap between the wall (Y50) and the bracket web (Y61).
// OBSOLETE since P2-85: the wall fork hub replaces it.
module p273_boss_spacer() {
    difference() { cylinder(d=10,h=10.8); translate([0,0,-1]) cylinder(d=p273_m4,h=13); }
}
if(p273_part=="boss_spacer") p273_boss_spacer();

// ---------------------------------------------------------------- P2-85 parts
// Print orientation: bracket on its flat inner web face (Y61) so the layers
// are parallel to the web loads; fork on its outer face, lips/pocket up.
if(p273_part=="print_p285_bracket_left")
    translate([-285,106.6,-61]) rotate([90,0,0]) p273_mount_bracket(1);
else if(p273_part=="print_p285_bracket_right")
    translate([-285,-41,-61]) rotate([-90,0,0]) p273_mount_bracket(-1);
else if(p273_part=="fork_left") p285_fork(1);
else if(p273_part=="fork_right") p285_fork(-1);
else if(p273_part=="print_p285_fork_left")
    translate([-300,-40,60.7]) rotate([-90,0,0]) p285_fork(1);
else if(p273_part=="print_p285_fork_right")
    translate([-300,70,60.7]) rotate([90,0,0]) p285_fork(-1);

// ---------------------------------------------------------------- P2-85 views
module p285_view_set(a=20,h=0,ghost=true) {
    color([.08,.62,.82,.85]) p273_ski(h);
    color([.82,.48,.12]) p273_mount_bracket(1);
    color([.55,.30,.75]) p285_fork(1,h);
    color("silver") intersection() {
        p273_fastener_envelopes(h); translate([0,100,0]) cube(200,center=true);
    }
    if(ghost) {
        color([.88,.68,.12,.55]) translate([0,0,h]) p273_selector(15);
        color([.24,.68,.36,.45]) translate([0,0,h]) module_pose(a)
            moving_throat_and_guide();
        color([.95,.42,.06,.30]) translate([0,0,h]) intersection() {
            module_pose(a) for(yy=[-129,129]) wheel_and_motor(yy);
            translate([350,50,40]) cube([90,100,100],center=true);
        }
        color([.16,.48,.78,.7]) intersection() {
            frame_assembly(); translate([300,90,30]) cube([80,60,40],center=true);
        }
    }
}
module p285_side_layer(z,c) {
    color(c) translate([0,0,z]) linear_extrude(.1) projection()
        rotate([-90,0,0]) intersection() {
            children(); translate([350,75,60]) cube([300,150,300],center=true);
        }
}
module p285_slab_layer(z,c,y0,y1) {
    color(c) translate([0,0,z]) linear_extrude(.1) projection()
        rotate([-90,0,0]) intersection() {
            children(); translate([250,y0,-100]) cube([200,y1-y0,300]);
        }
}
// Side views (+Z up, +X right = front), +Y half only.
if(p273_part=="p285_side_2d") {
    a=p273_check_angle;
    p285_side_layer(0,[.16,.48,.78]) intersection() {
        frame_assembly(); translate([300,90,30]) cube([80,60,40],center=true);
    }
    p285_side_layer(.2,[.95,.42,.06,.5]) intersection() {
        module_pose(a) for(yy=[-129,129]) wheel_and_motor(yy);
        translate([355,90,50]) cube([50,40,80],center=true);
    }
    p285_side_layer(.4,[.08,.62,.82]) p273_ski();
    p285_side_layer(.6,[.88,.68,.12]) p273_selector(15);
    p285_side_layer(.8,[.24,.68,.36]) module_pose(a) moving_throat_and_guide();
    p285_side_layer(1.0,[.82,.48,.12]) p273_mount_bracket(1);
    p285_side_layer(1.2,[.55,.30,.75]) p285_fork(1);
    p285_side_layer(1.4,"silver") p273_fastener_envelopes();
}
else if(p273_part=="p285_lip_section_2d") {
    // Y47..50.6 slab: ski wall (blue) between the fork lips (purple).
    p285_slab_layer(0,[.55,.30,.75],47,50.6) p285_fork(1);
    p285_slab_layer(.2,[.08,.62,.82],47,50) p273_ski();
}
else if(p273_part=="p285_iso_view") {
    color([.08,.62,.82]) p273_ski();
    color([.82,.48,.12]) p273_mount_bracket(1);
    color([.55,.30,.75]) p285_fork(1);
    color("silver") intersection() {
        p273_fastener_envelopes(); translate([300,100,0]) cube(200,center=true);
    }
}
else if(p273_part=="p285_fork_iso") {
    color([.08,.62,.82]) p273_ski();
    color([.55,.30,.75]) p285_fork(1);
}

// ---------------------------------------------------------------- P2-85 QA
module p285_brackets() { for(side=[-1,1]) p273_mount_bracket(side); }
p285_h_step = 1;
if(p273_part=="qa_p285_ski_bracket") union() for(h=[0:p285_h_step:20])
    intersection() { p273_ski(h); p285_brackets(); }
else if(p273_part=="qa_p285_fast_bracket") union() for(h=[0:p285_h_step:20])
    // Fastener shanks pass the slots; any hit here is a slot/web error.
    intersection() { p273_fastener_envelopes(h); p285_brackets(); }
else if(p273_part=="qa_p285_bracket_selector_coupled")
    union() for(h=[0:p285_h_step:20]) intersection() {
        p285_brackets(); translate([0,0,h]) p273_selector(p273_angle);
    }
else if(p273_part=="qa_p285_throat_coupled")
    union() for(h=[0:p285_h_step:20]) intersection() {
        union() { p285_forks(h); p273_fastener_envelopes(h);
                  p285_brackets(); p273_walls_only(h); }
        translate([0,0,h]) p273_throat_sweep();
    }
else if(p273_part=="qa_p285_launcher_coupled")
    // Full frozen launcher (throat included) against everything except
    // the intentional ski floor/throat handoff.
    union() for(h=[0:p285_h_step:20]) intersection() {
        union() { p285_forks(h); p273_fastener_envelopes(h); p285_brackets(); }
        translate([0,0,h]) p273_launcher_sweep();
    }
else if(p273_part=="qa_p285_bracket_frame") intersection() {
    p285_brackets(); frame_assembly();
}
else if(p273_part=="qa_p285_fast_frame") union() for(h=[0:p285_h_step:20])
    intersection() { p273_fastener_envelopes(h); frame_assembly(); }
else if(p273_part=="qa_p285_lip_engage") intersection() {
    // Must be SOLID: walls grown 0.1 mm past the lip clearance hit both lips.
    p285_fork(1,0,0);
    translate([0,0,0]) scale(1) p285_wall_clear_c(p285_lip_clear+.1);
}
else if(p273_part=="qa_p285_fork_ski") union() for(h=[0,20])
    // Hub face on the wall (Y50) is the intended clamp contact: zero volume.
    intersection() { p285_forks(h); p273_ski(h); }
else if(p273_part=="qa_p285_fork_bracket") union() for(h=[0:p285_h_step:20])
    intersection() { p285_forks(h); p285_brackets(); }
else if(p273_part=="qa_p285_fork_frame") union() for(h=[0:p285_h_step:20])
    intersection() { p285_forks(h); frame_assembly(); }
else if(p273_part=="qa_p285_bracket_drive") intersection() {
    p285_brackets(); drive_package_envelopes();
}
else if(p273_part=="qa_p285_bracket_feed") intersection() {
    p285_brackets(); p273_closed_feed();
}
if(p273_part=="p285_print_layout") {
    color([.82,.48,.12]) translate([0,0,0]) {
        translate([-285,106.6,-61]) rotate([90,0,0]) p273_mount_bracket(1);
    }
    color([.55,.30,.75]) translate([60,10,0])
        translate([-300,-40,60.7]) rotate([-90,0,0]) p285_fork(1);
    %translate([-20,-60,-0.5]) cube([160,140,0.5]);
}
