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
$fn = 48;

function p273_surface_z(x) =
    x >= 383 ? 10 : 10 + 75*(1-cos(asin((383-x)/75)));
function p273_bolt_z(x) = p273_surface_z(x) + 11;
function p273_mount_z(x) = p273_bolt_z(x) + p273_boss_z_add;
function p273_segment_w(i) = min(94,61+i*6);
function p273_wall_enabled(i) = i < 3 || i > 5;

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
        for(side=[-1,1],x=[p273_boss_x])
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

module p273_mount_bracket(side=1) {
    sy=side;
    difference() {
        union() {
            // Foot uses the existing X300 top-grid holes at Y75 and Y100.
            translate([300,sy*87.5,45]) cube([30,45,6],center=true);
            // Inner XZ web stays beyond the selector rail outer face (56.5).
            hull() {
                translate([300,sy*64,48]) cube([20,6,8],center=true);
                translate([p273_boss_x,sy*64,p273_mount_z(p273_boss_x)+10])
                    cube([10,6,12],center=true);
            }
            // Two 45-degree-style gussets between foot and slotted web.
            for(x=[294,306]) hull() {
                translate([x,sy*82,48]) cube([5,26,6],center=true);
                translate([x,sy*64,58]) cube([5,6,8],center=true);
            }
        }
        for(y=[75,100]) translate([300,sy*y,39]) cylinder(d=p273_m4,h=14);
        // At installed height 0 the bolts sit at the lower slot ends. Moving
        // the ski upward provides 20 mm travel without sacrificing ground gap.
        for(x=[p273_boss_x])
            p273_y_slot(x,sy*64,p273_mount_z(x)+p273_slot_travel/2);
    }
}

module p273_fastener_envelopes(height=p273_height) {
    for(side=[-1,1],x=[p273_boss_x])
        translate([x,side*57,p273_mount_z(x)+height]) rotate([90,0,0])
            cylinder(d=8,h=18,center=true);
}

module p273_ski_mount_assembly(height=p273_height) {
    color([.08,.62,.82]) p273_ski(height);
    color([.82,.48,.12]) for(side=[-1,1]) p273_mount_bracket(side);
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
            p273_ski(h); p273_fastener_envelopes(h);
        }
        for(side=[-1,1]) p273_mount_bracket(side);
    }
}
module p273_coupled_hard_adjustment_probe() {
    union() for(h=[0:1:20]) intersection() {
        union() {
            p273_ski(h); p273_fastener_envelopes(h);
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
else if(p273_part=="print_bracket_left")
    translate([-285,-60,-42]) p273_mount_bracket(1);
else if(p273_part=="print_bracket_right")
    translate([-285,110,-42]) p273_mount_bracket(-1);
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
module p273_boss_spacer() {
    difference() { cylinder(d=10,h=10.8); translate([0,0,-1]) cylinder(d=p273_m4,h=13); }
}
if(p273_part=="boss_spacer") p273_boss_spacer();
