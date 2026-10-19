// Upper-frame architecture study ONLY. No manufacturing export or load rating.
// Does not change modular-chassis-v2.scad or its fit-tested STL.
// mm; +X forward, +Y left, +Z up. All new coordinates are candidate positions.
$fn=32;
use <modular-chassis-v2.scad>
use <../../../flywheel-launcher-v0/direct-drive-mechanical-definition-study.scad>
use <../../../flywheel-launcher-v0/provisional-cradle-exit-clearance.scad>

view="assembly"; // [assembly,body,launcher,launcher_cover,plate_connections,socket_detail,qa_exit_structure,qa_motor_support_hardware,qa_motor_support_cover,qa_plate_lock_bores]
show_cover_reference=true;
show_launcher_reference=true;
launcher_reference_x=410;
launcher_reference_nip_z=298; // lower visual hypothesis, not a released mounting datum
launcher_reference_pitch=20;
plate_lock_x=[-115,115];
plate_lock_y=[-145,145];
plate_lock_d=4.5; // provisional M4 clearance; not a released bolt choice
plate_lock_boss_d=16;
plate_spacer_d=12; // metal threaded standoff reference, NOT a printed rotating part
plate_spacer_h=78; // between lower/upper panel inner faces -39/+39
body_height=350; // from rail top Z=52 to roof underside: provisional, NOT old LiDAR datum
gamma_bridge_top=198; // existing Gamma bearing plane 190 + 8 mm platform
launcher_bridge_gap=15; // candidate 10-20 mm gap ABOVE Gamma, not intake hardware clearance
launcher_support_height=gamma_bridge_top+launcher_bridge_gap-52;
post_w=20; // NEW vertical interface; not the horizontal 50x30 / 50x20 interface
post_wall=2.4;
socket_clearance=0.30; // each face; untested
socket_wall=3.2;
socket_depth=40;
socket_outer=post_w+2*socket_clearance+2*socket_wall;
rail_top=52;
body_x=[-115,250];
body_y=220;
launcher_x=[350,440];
launcher_y=260; // outboard of Gamma foot; attachment requires a new braced side node
roof_t=20;
// Packaging reservations ONLY: actual launcher world pose is not defined yet.
launcher_cover_x=[250,570];
launcher_cover_top=rail_top+body_height;
launcher_cover_bottom=rail_top+launcher_support_height;
launcher_cover_post_start=launcher_cover_bottom+20; // on top of lower frame
cover_skin=2;
exit_width=100;
exit_height=78; // limited by front roof beam; only a provisional ball-axis clearance
exit_plane_x=launcher_cover_x[1]+post_w/2+cover_skin/2;
exit_center_z=launcher_reference_nip_z+
    (exit_plane_x-launcher_reference_x)*tan(launcher_reference_pitch);
// Local lower-panel underside is -47; support rail centre -57 (20 mm deep).
launcher_mount_local_z=-57;
function mount_local_x(xx)=
    (xx-launcher_reference_x+launcher_mount_local_z*sin(launcher_reference_pitch)) /
        cos(launcher_reference_pitch);
function mount_world_z(xx)=launcher_reference_nip_z+
    (xx-launcher_reference_x)*tan(launcher_reference_pitch)+
        launcher_mount_local_z/cos(launcher_reference_pitch);
assert(body_height>socket_depth+roof_t);
assert(launcher_support_height>socket_depth);
assert(launcher_cover_top>launcher_cover_bottom);
assert(exit_center_z-exit_height/2>launcher_cover_bottom);
assert(exit_center_z+exit_height/2<launcher_cover_top+roof_t);

module tube_z(height) {
    difference() {
        translate([-post_w/2,-post_w/2,0]) cube([post_w,post_w,height]);
        translate([-post_w/2+post_wall,-post_w/2+post_wall,-1])
            cube([post_w-2*post_wall,post_w-2*post_wall,height+2]);
    }
}

module socket_body() {
    difference() {
        translate([-socket_outer/2,-socket_outer/2,0])
            cube([socket_outer,socket_outer,socket_depth]);
        translate([-(post_w+2*socket_clearance)/2,-(post_w+2*socket_clearance)/2,-1])
            cube([post_w+2*socket_clearance,post_w+2*socket_clearance,socket_depth+2]);
        // Illustrative two transverse locks; spacers/metal hardware not designed.
        for (zz=[12,28]) translate([-socket_outer/2-1,0,zz])
            rotate([0,90,0]) cylinder(d=4.4,h=socket_outer+2);
    }
}

module horizontal_x(length,w=post_w,h=20) {
    difference() {
        cube([length,w,h],center=true);
        cube([length+2,w-2*post_wall,h-2*post_wall],center=true);
    }
}

module edge_frame(xs,half_y,z) {
    for (side=[-1,1]) translate([(xs[0]+xs[1])/2,side*half_y,z+10])
        horizontal_x(xs[1]-xs[0]);
    for (xx=xs) translate([xx,0,z+10]) rotate([0,0,90]) horizontal_x(2*half_y);
}

module candidate_base(xx,yy,outboard=false) {
    // Solid colour denotes an attachment reservation, not a drill drawing.
    // Outboard launcher seat returns to the SIDE of the chassis, not Gamma.
    color([0.95,0.55,0.12]) {
        translate([xx,yy,rail_top]) socket_body();
        if (outboard) {
            side=yy>0 ? 1 : -1;
            hull() {
                translate([xx,side*243,rail_top-10]) cube([30,4,20],center=true);
                translate([xx,yy,rail_top-2]) cube([30,post_w,4],center=true);
            }
        } else translate([xx,yy,rail_top-2]) cube([30,50,4],center=true);
    }
}

module body_frame() {
    for (xx=body_x,side=[-1,1]) {
        candidate_base(xx,side*body_y);
        color([0.15,0.60,0.65]) translate([xx,side*body_y,rail_top]) tube_z(body_height);
    }
    color([0.15,0.60,0.65]) edge_frame(body_x,body_y,rail_top+body_height);
    // Removable roof skin is a nonstructural reference, not a load spreader.
    %translate([(body_x[0]+body_x[1])/2,0,rail_top+body_height+roof_t+1])
        cube([body_x[1]-body_x[0]+post_w,2*body_y+post_w,2],center=true);
    // Replaceable sensor crossbeam / generic mounting plate, no LiDAR hole pattern.
    color([0.15,0.60,0.65]) translate([-100,0,rail_top+body_height+10])
        rotate([0,0,90]) horizontal_x(2*body_y);
    color([0.65,0.65,0.65]) translate([-100,0,rail_top+body_height+roof_t+4])
        cube([70,70,8],center=true);
    // Two back-plane diagonals keep the four posts from being a loose rectangle.
    color([0.15,0.60,0.65]) for (side=[-1,1]) hull() {
        translate([body_x[0],side*(body_y-10),rail_top+45]) cube([6,6,6],center=true);
        translate([body_x[0],-side*(body_y-10),rail_top+body_height-10]) cube([6,6,6],center=true);
    }
}

module launcher_mount_structure() {
    for (xx=launcher_x,side=[-1,1]) {
        candidate_base(xx,side*launcher_y,true);
        color([0.60,0.35,0.65]) translate([xx,side*launcher_y,rail_top])
            tube_z(mount_world_z(xx)-rail_top);
    }
    // Replace the flat transverse platform with a cradle BELOW the lower panel.
    // Rear support is lower than front support; both follow the 20-degree plane.
    color([0.60,0.35,0.65])
        translate([launcher_reference_x,0,launcher_reference_nip_z])
            rotate([0,-launcher_reference_pitch,0]) {
                for (side=[-1,1]) lower_plate_lock_rail(side);
                for (xx=launcher_x) translate([mount_local_x(xx),0,launcher_mount_local_z])
                    rotate([0,0,90]) horizontal_x(2*launcher_y);
            }
    // World placement below is a NEW packaging hypothesis, not a standalone datum.
    // These side-plane diagonals are load-path intent, not manufactured gussets.
    color([0.60,0.35,0.65]) for (side=[-1,1]) hull() {
        translate([launcher_x[0],side*launcher_y,rail_top+45]) cube([6,6,6],center=true);
        translate([launcher_x[1],side*launcher_y,mount_world_z(launcher_x[1])-10]) cube([6,6,6],center=true);
    }
}

module launcher_support() {
    launcher_mount_structure();
    launcher_cover_frame();
    launcher_plate_connections();
    if (show_cover_reference) %launcher_cover_panels();
    if (show_launcher_reference) launcher_visual_reference();
}

module plate_lock_bores() {
    for (xx=plate_lock_x,yy=plate_lock_y)
        translate([xx,yy,-68]) cylinder(d=plate_lock_d,h=120);
}

module lower_plate_lock_rail(side) {
    // Full compression towers carry screw preload through the otherwise hollow rail.
    difference() {
        union() {
            translate([0,side*145,launcher_mount_local_z]) horizontal_x(256);
            for (xx=plate_lock_x) translate([xx,side*145,-67])
                cylinder(d=plate_lock_boss_d,h=20);
        }
        plate_lock_bores();
        // Recessed underside head access; tower still supplies a bearing shoulder.
        for (xx=plate_lock_x) translate([xx,side*145,-68]) cylinder(d=9,h=6);
    }
}

module launcher_plate_connections() {
    translate([launcher_reference_x,0,launcher_reference_nip_z])
        rotate([0,-launcher_reference_pitch,0])
            for (xx=plate_lock_x,yy=plate_lock_y) {
                color("silver") translate([xx,yy,-39]) difference() {
                    cylinder(d=plate_spacer_d,h=plate_spacer_h);
                    // Pilot only: end threads/specification must be chosen and checked.
                    translate([0,0,-1]) cylinder(d=3.3,h=plate_spacer_h+2);
                }
                // Reference bolts only: lower ~M4x35, upper ~M4x16; not a parts order.
                color([0.25,0.25,0.25]) {
                    translate([xx,yy,-65]) cylinder(d=7,h=3);
                    translate([xx,yy,-62]) cylinder(d=4,h=35);
                    translate([xx,yy,31]) cylinder(d=4,h=16);
                    translate([xx,yy,47]) cylinder(d=7,h=4);
                }
            }
}

module launcher_panel_reference(upper=false) {
    difference() {
        translate([0,0,upper ? 43 : -43])
            if (upper) upper_panel(); else lower_panel();
        plate_lock_bores();
        // Existing motor flange pattern: four features on 30 mm PCD.
        // Actual hole/thread type and screw depth remain a hardware measurement gate.
        if (upper) for (yy=[-129,129],aa=[0:90:270])
            translate([15*cos(aa),yy+15*sin(aa),38]) cylinder(d=4.5,h=10);
    }
}

module launcher_visual_reference() {
    // Reuse accepted relieved panels and the existing motor/wheel envelopes.
    // No fabricated hub, mount hardware or manufacturing geometry is added.
    translate([launcher_reference_x,0,launcher_reference_nip_z])
        rotate([0,-launcher_reference_pitch,0]) {
            color([0.35,0.40,0.46]) {
                launcher_panel_reference(false);
                launcher_panel_reference(true);
            }
            for (yy=[-129,129]) {
                wheel(yy);
                motor_and_shaft(yy);
                unresolved_hub_zone(yy);
            }
            // Straight launch-axis keep-out, NOT a physical barrel/ball trajectory.
            %color([0.1,0.65,0.95,0.25]) translate([95,0,0])
                rotate([0,90,0]) cylinder(d=66,h=230,center=true);
            color([0.1,0.65,0.95]) sphere(d=66);
        }
}

module launcher_cover_frame() {
    // Extend the support in X before adding cover posts; no free-floating posts.
    // Existing flywheel support remains separate from the removable skins.
    color([0.60,0.35,0.65]) {
        edge_frame(launcher_cover_x,launcher_y,rail_top+launcher_support_height);
        for (xx=launcher_cover_x,side=[-1,1])
            translate([xx,side*launcher_y,launcher_cover_post_start])
                tube_z(launcher_cover_top-launcher_cover_post_start);
        edge_frame(launcher_cover_x,launcher_y,launcher_cover_top);
    }
}

module launcher_cover_panels() {
    // Nonstructural reference skins, NOT a validated high-speed containment guard.
    // Rear ball-feed/service penetration is still to be defined with launcher pose.
    cx=(launcher_cover_x[0]+launcher_cover_x[1])/2;
    span=launcher_cover_x[1]-launcher_cover_x[0]+post_w;
    z0=launcher_cover_bottom;
    z1=launcher_cover_top+roof_t;
    for (side=[-1,1]) translate([cx,side*(launcher_y+post_w/2+cover_skin/2),(z0+z1)/2])
        cube([span,cover_skin,z1-z0],center=true);
    translate([launcher_cover_x[0]-post_w/2-cover_skin/2,0,(z0+z1)/2])
        cube([cover_skin,2*launcher_y+post_w,z1-z0],center=true);
    translate([cx,0,z1+cover_skin/2])
        cube([span,2*launcher_y+post_w+2*cover_skin,cover_skin],center=true);
    difference() {
        translate([launcher_cover_x[1]+post_w/2+cover_skin/2,0,(z0+z1)/2])
            cube([cover_skin,2*launcher_y+post_w,z1-z0],center=true);
        translate([launcher_cover_x[1]+post_w/2+cover_skin/2,0,exit_center_z])
            cube([cover_skin+2,exit_width,exit_height],center=true);
    }
}

module socket_detail() {
    color([0.95,0.55,0.12]) intersection() {
        socket_body(); translate([-30,-30,-1]) cube([60,30,60]);
    }
    color([0.15,0.60,0.65]) difference() {
        translate([0,0,2]) tube_z(95);
        for (zz=[12,28]) translate([-post_w/2-1,0,zz])
            rotate([0,90,0]) cylinder(d=4.4,h=post_w+2);
    }
    // Illustrative internal compression spacers; material/specification pending.
    color([0.65,0.65,0.65]) for (zz=[12,28])
        translate([-post_w/2+post_wall,0,zz]) rotate([0,90,0]) difference() {
            cylinder(d=7,h=post_w-2*post_wall);
            translate([0,0,-0.1]) cylinder(d=4.4,h=post_w-2*post_wall+0.2);
        }
    color([0.65,0.65,0.65]) for (zz=[12,28])
        translate([-socket_outer/2-4,0,zz]) rotate([0,90,0]) cylinder(d=4,h=socket_outer+8);
}

if (view=="qa_exit_structure") intersection() {
    union() { launcher_mount_structure(); launcher_plate_connections(); launcher_cover_frame(); body_frame(); }
    translate([launcher_reference_x,0,launcher_reference_nip_z])
        rotate([0,-launcher_reference_pitch,0])
            translate([95,0,0]) rotate([0,90,0]) cylinder(d=76,h=230,center=true);
}
else if (view=="qa_motor_support_hardware") intersection() {
    launcher_plate_connections();
    translate([launcher_reference_x,0,launcher_reference_nip_z])
        rotate([0,-launcher_reference_pitch,0])
            for (yy=[-129,129]) { wheel(yy); motor_and_shaft(yy); }
}
else if (view=="qa_motor_support_cover") intersection() {
    launcher_plate_connections(); launcher_cover_panels();
}
else if (view=="qa_plate_lock_bores")
    translate([launcher_reference_x,0,launcher_reference_nip_z])
        rotate([0,-launcher_reference_pitch,0]) intersection() {
            union() {
                lower_plate_lock_rail(1); lower_plate_lock_rail(-1);
                launcher_panel_reference(false); launcher_panel_reference(true);
            }
            for (xx=plate_lock_x,yy=plate_lock_y)
                translate([xx,yy,-68]) cylinder(d=4.4,h=120);
        }
else if (view=="socket_detail") socket_detail();
else {
    frame_structure();
    gamma_mount_reference(1); gamma_mount_reference(-1);
    for (side=[-1,1],xx=[-65,300]) wheel_context(side,xx);
    if (view=="assembly" || view=="body") body_frame();
    if (view=="assembly" || view=="launcher" || view=="launcher_cover" || view=="plate_connections") launcher_support();
    if (view=="launcher_cover") color([0.70,0.55,0.75,0.35]) launcher_cover_panels();
}
