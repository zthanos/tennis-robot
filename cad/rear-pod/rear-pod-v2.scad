// P2-76 printable rear pod v2 — upright electronics carrier.
// Select a printable part with -D 'part="..."'.

include <params-v2.scad>

part = "assembly";
show_ghosts = true;
show_panel = true;

module rounded_rect_2d(size, r=5) {
    hull()
        for (x=[-size[0]/2+r,size[0]/2-r], y=[-size[1]/2+r,size[1]/2-r])
            translate([x,y]) circle(r=r);
}

module capsule_2d(length, width, horizontal=true) {
    hull()
        if (horizontal)
            for (x=[-(length-width)/2,(length-width)/2]) translate([x,0]) circle(d=width);
        else
            for (y=[-(length-width)/2,(length-width)/2]) translate([0,y]) circle(d=width);
}

module p276_footprint_2d() {
    polygon(points=[
        [pod_x_front,-pod_y_half],
        [pod_x_rear+pod_rear_chamfer,-pod_y_half],
        [pod_x_rear,-pod_y_half+pod_rear_chamfer],
        [pod_x_rear,pod_y_half-pod_rear_chamfer],
        [pod_x_rear+pod_rear_chamfer,pod_y_half],
        [pod_x_front,pod_y_half]
    ]);
}

module small_mount_boss(x,y,h=5,d=12,pilot=m3_pilot) {
    translate([x,y,plate_t]) difference() {
        cylinder(d=d,h=h);
        if (pilot > 0) translate([0,0,-1]) cylinder(d=pilot,h=h+2);
    }
}

module base_rear_rib(y) {
    // The rear-only ribs stop at the saddle rear face, so they cannot enter
    // the 50x30 rear-crossbar solid at X-190..-140 / Z22..52.
    translate([0,y,0]) rotate([90,0,0]) linear_extrude(height=6,center=true)
        polygon(points=[
            [pod_x_rear+5,base_bottom_local],
            [rear_crossbar_x-rear_crossbar_w/2,base_bottom_local],
            [rear_crossbar_x-rear_crossbar_w/2-10,base_rib_bottom_local],
            [pod_x_rear+18,base_rib_bottom_local]
        ]);
}

module base_half_v2(side=1) {
    difference() {
        union() {
            intersection() {
                translate([0,0,base_bottom_local])
                    linear_extrude(height=base_t) p276_footprint_2d();
                translate([pod_x_rear-1,side>0?0:-pod_y_half-1,base_bottom_local-1])
                    cube([pod_x_front-pod_x_rear+2,pod_y_half+1,base_t+2]);
            }
            // The side-frame bosses are only locating lands; their M4 load
            // path passes through the complete base to a washer/nut below.
            for (x=[panel_center_x-58,panel_center_x+58])
                small_mount_boss(x,side*54,5,12,0);
            // Four independent Pi shelf bosses; no shelf stack remains.
            for (x=[pi_center_x-42,pi_center_x+42])
                small_mount_boss(x,side*32,5,10);
            for (y=[side*18,side*54,side*96,side*142]) base_rear_rib(y);
        }
        // A 1 mm underside relief leaves a seven-millimetre saddle bearing on
        // the full rear-crossbar width instead of four 3 mm spacers.
        translate([rear_crossbar_x-rear_crossbar_w/2,-pod_y_half-1,-20])
            cube([rear_crossbar_w,pod_y_half*2+2,20+base_saddle_bottom_local]);
        // Preserve 0.5 mm exact-solid clearance below both Parkside reserves;
        // the local base section remains 6.5 mm thick.
        for (s=[-1,1])
            translate([pod_x_rear-0.5,
                       s*parkside_center_y-parkside_clear[1]/2-0.5,2.5])
                cube([parkside_center_x+parkside_clear[0]/2-(pod_x_rear-0.5),
                      parkside_clear[1]+1,plate_t]);
        for (y=[75,125])
            translate([rear_crossbar_x,side*y,-20]) cylinder(d=m4_clear,h=40);
        // Through-M4 side-frame fasteners; nuts and washers stay accessible
        // below the base, clear of the rear-only stiffening ribs.
        for (x=[panel_center_x-58,panel_center_x+58])
            translate([x,side*54,-20]) cylinder(d=m4_clear,h=40);
        for (x=[-245,-210,-175,-140])
            translate([x,side*145,-20]) linear_extrude(height=40)
                capsule_2d(14,x==-140?m4_clear:4,true);
    }
}

// Reuses the X-135 outer-side M4 of each rear corner clamp.  One vertical M4
// through the existing X-140/Y+/-145 base slot closes the second attachment
// line; the same STL is flipped for the opposite side.
module corner_tie() {
    difference() {
        union() {
            translate([7.5,0,11]) cube([15,30,6],center=true);
            // Run above the chassis top (global Z52) and turn down only
            // outside the rail face, avoiding the former diagonal intrusion.
            translate([7.5,50,17.6]) cube([15,100,5],center=true);
            translate([7.5,103.1,0]) cube([15,5.8,24],center=true);
        }
        translate([0,0,-2]) cylinder(d=m4_clear,h=24);
        translate([5,97,0]) rotate([-90,0,0]) cylinder(d=m4_clear,h=16);
    }
}

module corner_tie_installed(side=1) {
    translate([-140,side*145,corner_side_bolt_z])
        if (side>0) corner_tie(); else mirror([0,1,0]) corner_tie();
}

module pi_shelf_v2() {
    difference() {
        linear_extrude(height=plate_t) rounded_rect_2d([100,80],4);
        for (x=[-42,42],y=[-32,32])
            translate([x,y,-1]) cylinder(d=m3_clear,h=plate_t+2);
        for (x=[-30,-15,0,15,30],y=[-16,0,16])
            translate([x,y,-1]) linear_extrude(height=plate_t+2)
                capsule_2d(24,4,true);
        for (x=[-31,31],y=[-33,33])
            translate([x,y,-1]) linear_extrude(height=plate_t+2)
                capsule_2d(22,4,true);
    }
    for (x=[-47,47],y=[-37,37])
        translate([x,y,plate_t]) cylinder(d=5,h=4);
}

// P2-81 control panel. The top face, apertures and four locating holes are
// identical to v1/P2-79. Eight-millimetre underside ribs give a 12 mm total
// E-stop load-path depth while explicit keepouts preserve control bodies,
// post pegs and tool access.
module control_panel_ribs_2d() {
    union() {
        // E-stop box rails: 16 mm total effective rib width for the
        // conservative 40 mm strip calculation.
        for (y=[-78,-38]) translate([0,y]) square([124,8],center=true);
        // Direct split rail to the rear mounting pair, clear of the body.
        for (x=[-42,42]) translate([x,-58]) square([40,10],center=true);

        // Toggle and START frames share two continuous longitudinal spines.
        for (x=[-22,22]) translate([x,-17]) square([8,126],center=true);
        for (y=[-25,1,20,44]) translate([0,y]) square([104,7],center=true);

        // Outer collectors transfer the local grid into the four panel lands.
        for (x=[-52,52]) translate([x,-17]) square([8,126],center=true);
    }
}

module control_panel_v2() {
    difference() {
        union() {
            linear_extrude(height=plate_t) rounded_rect_2d([138,190],6);
            translate([0,0,-8]) linear_extrude(height=8) control_panel_ribs_2d();
        }
        // Existing four Ø8.6 locating holes; the P2-79 fastener is through-M4.
        for (x=[-58,58],y=[-56,56])
            translate([x,y,-9]) cylinder(d=8.6,h=14);
        translate([0,-58,-9]) cylinder(d=estop_hole,h=14);
        translate([0,-12,-9]) cylinder(d=toggle_hole,h=14);
        translate([0,32,-9]) cylinder(d=arm_hole,h=14);
        translate([-42,70,-9]) linear_extrude(height=14)
            capsule_2d(24,8,true);

        // Underside keepouts: actual ghost body plus 1.5 mm per side.
        translate([-19,-74.5,-9]) cube([38,33,10]);
        translate([-15,-21,-9]) cube([30,18,10]);
        translate([0,32,-9]) cylinder(d=18,h=10);
        // P2-79 Ø23 post end / washer access around every mounting land.
        for (x=[-58,58],y=[-56,56])
            translate([x,y,-9]) cylinder(d=25,h=10);
    }
}

// Carrier coordinates are global Y/Z in the printable XY plane. The 248 mm
// lower width and 145 mm total height fit a 250x250 bed. The upper-right notch
// preserves the real P2-70 Ugreen reservation at Y16..68, Z232..284.
module carrier_outline_2d() {
    union() {
        polygon(points=[
            [carrier_y_min,carrier_main_z0],
            [carrier_y_lower_max,carrier_main_z0],
            [carrier_y_lower_max,carrier_step_z],
            [carrier_y_upper_max,carrier_step_z],
            [carrier_y_upper_max,carrier_top_z],
            [carrier_y_min,carrier_top_z]
        ]);
        for (y=[-54,54]) translate([y,144]) square([8,12],center=true);
    }
}

module cross_slot_2d(span=7,width=3.6) {
    union() {
        capsule_2d(span,width,true);
        capsule_2d(span,width,false);
    }
}

module carrier_boss(y,z,h,span=7) {
    difference() {
        translate([y,z,plate_t]) cylinder(d=10,h=h);
        translate([y,z,-1]) linear_extrude(height=plate_t+h+2)
            cross_slot_2d(span,3.6);
    }
}

module vertical_electronics_plate() {
    difference() {
        union() {
            linear_extrude(height=plate_t) carrier_outline_2d();
            // Rear-face bosses: perfboard rotated 90 degrees.
            for (y=[-151.5,-36.5],z=[154,229]) carrier_boss(y,z,10,8);
            // Rear-face Mega case rotated 90 degrees.
            for (p=[[78,156],[78,206],[16,162.5],[16,199.5]])
                carrier_boss(p[0],p[1],4,6);
        }
        // M4 clamp holes through the two mounting tongues.
        for (y=[-54,54]) translate([y,159,-1]) cylinder(d=m4_clear,h=plate_t+18);
        // Front-face BTS pins/spacers: four boards in a 2x2 negative-Y field.
        for (cy=[-130,-74],cz=[200,256],dy=[-20,20],dz=[-20,20])
            translate([cy+dy,cz+dz,-1]) linear_extrude(height=plate_t+2)
                cross_slot_2d(6,3.6);
        // Vent slots remain vertical after installation, aiding convection.
        for (cy=[-130,-74],cz=[200,256],dy=[-13,0,13],dz=[-11,0,11])
            translate([cy+dy,cz+dz,-1]) linear_extrude(height=plate_t+2)
                capsule_2d(20,4,true);
        // Service/tie slots around both rear-face boards.
        for (p=[[-94,158],[-94,220],[26,158],[26,204]])
            translate([p[0],p[1],-1]) linear_extrude(height=plate_t+2)
                capsule_2d(18,4,true);
    }
}

// Side frame: print two. It bolts to the revised base bosses at Y+/-54,
// supports the old panel datum, and clamps a carrier tongue in a 4.8 mm jaw.
module frame_web_2d() {
    union() {
        for (x=[-58,58]) translate([x-4,4]) square([8,86]);
        polygon(points=[[-58,4],[-50,4],[24,76],[16,84]]);
        polygon(points=[[58,4],[50,4],[15,76],[23,84]]);
        translate([12,75]) square([15,31]);
        translate([-58,4]) square([116,7]);
    }
}

module side_frame() {
    difference() {
        union() {
            rotate([90,0,0]) linear_extrude(height=8,center=true) frame_web_2d();
            for (x=[-58,58]) translate([x,0,2]) cylinder(d=12,h=4,center=true);
        }
        for (x=[-58,58]) translate([x,0,-1]) cylinder(d=m4_clear,h=8);
        // Carrier plate is X17.5..21.5 relative to panel center.
        translate([17.1,-6,74]) cube([4.8,12,34]);
        // M4 transverse clamp screw through jaw and carrier tongue.
        translate([8,0,95]) rotate([0,90,0]) cylinder(d=m4_clear,h=24);
        // Through-M4 cross-bolts for the two upper-panel posts.
        for (x=[-58,58]) translate([x,-6,90]) rotate([-90,0,0])
            cylinder(d=m4_clear,h=12);
    }
}

// Upper post paths are local Y/Z offsets from Z149. Rounded 12 mm columns use
// gentle <=45-degree detours around the front BTS field and the fixed Ugreen.
module path_rod(path,d=post_d) {
    for (i=[0:len(path)-2]) hull() {
        translate([0,path[i][0],path[i][1]]) sphere(d=d);
        translate([0,path[i+1][0],path[i+1][1]]) sphere(d=d);
    }
}

module panel_post(kind="rear_negative") {
    path = kind=="front_negative" ? [[0,6],[24,30],[24,119],[-2,159],[-2,192]] :
           kind=="positive" ? [[0,6],[36,73],[36,145],[2,185],[2,192]] :
           [[0,6],[-2,192]];
    top_y = path[len(path)-1][0];
    difference() {
        union() {
            // P2-81: terminate the Ø23 column as a true planar shoulder at
            // local Z198 (global panel underside Z347).  The former terminal
            // sphere extended through the panel skin up to Z351.
            intersection() {
                union() {
                    path_rod(path,post_d);
                    translate([0,top_y,180]) cylinder(d=post_d,h=18);
                    translate([0,0,3]) cylinder(d=18,h=12,center=true);
                }
                translate([-100,-100,-50]) cube([200,200,248]);
            }
            // Locating peg only; clamp load is carried by through-M4,
            // washer and nyloc rather than a printed thread.
            translate([0,top_y,198]) cylinder(d=8.2,h=4.2);
        }
        // Bottom clevis receives the 8 mm side-frame rib.
        translate([-5,-4.2,-1]) cube([10,8.4,13]);
        translate([-14,0,5]) rotate([0,90,0]) cylinder(d=m4_clear,h=28);
        // Through-M4 panel retention. The existing Ø8.6 panel holes still
        // locate on the peg; an M4 washer and nut carry the clamp load.
        translate([0,top_y,194]) cylinder(d=m4_clear,h=10);
    }
}

module panel_posts_print_set() {
    translate([0,0,0]) panel_post("rear_negative");
    translate([26,0,0]) panel_post("front_negative");
    translate([60,0,0]) panel_post("positive");
    translate([94,0,0]) panel_post("positive");
}

// Retaining pins/C-clips adapted from motion-electronics-tray. Same geometry
// as v1; quantities increase for four BTS positions.
pin_shaft_d = 2.8;
pin_groove_d = 2.2;
pin_groove_h = 1.8;
pin_tip_h = 2.5;
pin_head_d = 8;
pin_head_t = 1.8;
clip_od = 12;
clip_t = 1.6;
clip_hole_d = 2.35;
clip_mouth_w = 2;
pin_fit = 0.8;
mega_pin_grip = plate_t + 4 + 3.0 + pin_fit;
perf_pin_grip = plate_t + 10 + 1.6 + pin_fit;
bts_pin_grip = plate_t + 20 + 1.6 + pin_fit;

module retaining_pin(grip) {
    union() {
        cylinder(d=pin_head_d,h=pin_head_t);
        translate([0,0,pin_head_t]) cylinder(d=pin_shaft_d,h=grip);
        translate([0,0,pin_head_t+grip]) cylinder(d=pin_groove_d,h=pin_groove_h);
        translate([0,0,pin_head_t+grip+pin_groove_h])
            cylinder(d1=pin_shaft_d,d2=1.8,h=pin_tip_h);
    }
}

module locking_clip() {
    difference() {
        cylinder(d=clip_od,h=clip_t);
        translate([0,0,-1]) cylinder(d=clip_hole_d,h=clip_t+2);
        translate([0,-clip_mouth_w/2,-1]) cube([clip_od/2+1,clip_mouth_w,clip_t+2]);
    }
}

module component_pin_plate() {
    // 4+1 Mega, 4+1 perfboard, 16+1 BTS.
    for (i=[0:4]) translate([i*13,0,0]) retaining_pin(mega_pin_grip);
    for (i=[0:4]) translate([i*13,15,0]) retaining_pin(perf_pin_grip);
    for (i=[0:16]) translate([(i%6)*13,32+floor(i/6)*15,0]) retaining_pin(bts_pin_grip);
}

module component_clip_plate(count=32) {
    for (i=[0:count-1]) translate([(i%8)*14,floor(i/8)*14,0]) locking_clip();
}

module bts_spacer() {
    difference() {
        cylinder(d=10,h=20);
        translate([0,0,-1]) cylinder(d=m3_clear,h=22);
    }
}

module bts_spacer_plate() {
    for (i=[0:17]) translate([(i%6)*14,floor(i/6)*14,0]) bts_spacer();
}

module upright_mount_fit_coupon() {
    // Only the new upright-board jaw is tested: a real 4 mm tongue, the
    // 4.8 mm receiving slot and the M4 transverse clamp.
    difference() {
        union() {
            translate([0,0,0]) cube([32,24,18]);
            translate([42,8,0]) cube([4,8,30]);
        }
        translate([13.6,-1,7]) cube([4.8,26,13]);
        translate([-1,12,13]) rotate([0,90,0]) cylinder(d=m4_clear,h=34);
        translate([41,12,13]) rotate([0,90,0]) cylinder(d=m4_clear,h=6);
    }
}

module carrier_installed() {
    // local X/Y/Z -> global Y/Z/-X, with local Z=0 at global X-177.
    multmatrix([[0,0,-1,carrier_front_x],[1,0,0,0],[0,1,0,0],[0,0,0,1]])
        vertical_electronics_plate();
}

module panel_post_installed(x,side,kind) {
    // Positive-path detours are outward; negative side custom paths are given
    // explicitly by kind. Rotate local X onto robot X without changing Z.
    translate([x,side*54,149]) panel_post(kind);
}

module reserved_ghosts() {
    for (side=[-1,1]) color([.10,.58,.22,.24])
        translate([parkside_center_x,side*parkside_center_y,parkside_center_z])
            cube(parkside_clear,center=true);
    color([.16,.18,.25,.36]) translate(ugreen_center) cube(ugreen_clear,center=true);
    color([.10,.14,.20,.58])
        translate([pi_center_x,0,pi_shelf_z+plate_t+pi_clear[2]/2])
            cube(pi_clear,center=true);
}

module electronics_ghosts() {
    // Rear face: keyed perfboard full solid projection plus separate cable bend.
    color([.18,.55,.75,.70])
        translate([carrier_rear_x-perf_solid_projection/2,-94,191.5])
            cube([perf_solid_projection,120,80],center=true);
    color([.12,.42,.72,.70])
        translate([carrier_rear_x-25,26,181]) cube([50,110,60],center=true);
    // Front face: four 50x50 BTS envelopes with 20 mm spacers/heatsinks.
    for (y=[-130,-74],z=[200,256]) color([.74,.18,.12,.72])
        translate([-156,y,z]) cube([42,50,50],center=true);
    // Routed 20 mm bend zone drops below the carrier, then runs to X-130.
    color([.10,.78,.86,.22])
        translate([-195,-8,136.5]) cube([130,54,25],center=true);
}

module controls_ghosts() {
    color([.85,.08,.08,.65]) translate([panel_center_x,-58,panel_z-25]) cube([35,30,50],center=true);
    color([.30,.30,.32,.60]) translate([panel_center_x,-12,panel_z-15]) cube([27,15,30],center=true);
    color([.18,.70,.25,.60]) translate([panel_center_x,32,panel_z-12.5]) cylinder(d=15,h=25,center=true);
    color([.86,.08,.08,.80]) translate([panel_center_x,-58,panel_z+14.5]) cylinder(d=38,h=21,center=true);
}

module priority_keepouts() {
    for (side=[-1,1])
        translate([parkside_center_x,side*parkside_center_y,parkside_center_z])
            cube(parkside_clear,center=true);
    translate(ugreen_center) cube(ugreen_clear,center=true);
    translate([panel_center_x,-58,panel_z-25]) cube([35,30,50],center=true);
    translate([panel_center_x,-12,panel_z-15]) cube([27,15,30],center=true);
    translate([panel_center_x,32,panel_z-12.5]) cylinder(d=15,h=25,center=true);
}

module installed_structure_without_panel_or_base() {
    translate([pi_center_x,0,pi_shelf_z]) pi_shelf_v2();
    carrier_installed();
    electronics_ghosts();
    for (side=[-1,1]) translate([panel_center_x,side*54,64]) side_frame();
    panel_post_installed(panel_center_x-58,-1,"rear_negative");
    panel_post_installed(panel_center_x+58,-1,"front_negative");
    panel_post_installed(panel_center_x-58,1,"positive");
    panel_post_installed(panel_center_x+58,1,"positive");
}

module installed_p279_structure() {
    translate([0,0,pod_z_bottom]) base_half_v2(-1);
    translate([0,0,pod_z_bottom]) base_half_v2(1);
    corner_tie_installed(-1);
    corner_tie_installed(1);
    installed_structure_without_panel_or_base();
    translate([panel_center_x,0,panel_z])
        control_panel_v2();
}

module control_body_solids() {
    translate([panel_center_x,-58,panel_z-25]) cube([35,30,50],center=true);
    translate([panel_center_x,-12,panel_z-15]) cube([27,15,30],center=true);
    translate([panel_center_x,32,panel_z-12.5]) cylinder(d=15,h=25,center=true);
    translate([panel_center_x,-58,panel_z+14.5]) cylinder(d=38,h=21,center=true);
}

module control_body_penetration_solids(eps=0.01) {
    // Remove the two intentional seating planes: threaded bodies stop at the
    // panel underside and the E-stop cap begins at the panel top.
    translate([panel_center_x,-58,panel_z-25-eps/2])
        cube([35,30,50-eps],center=true);
    translate([panel_center_x,-12,panel_z-15-eps/2])
        cube([27,15,30-eps],center=true);
    translate([panel_center_x,32,panel_z-12.5-eps/2])
        cylinder(d=15,h=25-eps,center=true);
    translate([panel_center_x,-58,panel_z+14.5+eps/2])
        cylinder(d=38,h=21-eps,center=true);
}

module installed_panel() {
    translate([panel_center_x,0,panel_z]) control_panel_v2();
}

// P2-81 exact-solid gates.  The post gate excludes no intended interface:
// the Ø8.2 locating pegs pass freely through unchanged Ø8.6 holes and the
// Ø23 post ends are cleared by Ø25 underside service pockets.
module qa_control_bodies_collision() {
    intersection() { installed_panel(); control_body_penetration_solids(); }
}

module qa_control_ugreen_collision() {
    intersection() {
        installed_panel();
        translate(ugreen_center) cube(ugreen_clear,center=true);
    }
}

module qa_control_posts_collision() {
    intersection() {
        installed_panel();
        union() {
            panel_post_installed(panel_center_x-58,-1,"rear_negative");
            panel_post_installed(panel_center_x+58,-1,"front_negative");
            panel_post_installed(panel_center_x-58,1,"positive");
            panel_post_installed(panel_center_x+58,1,"positive");
        }
    }
}

module qa_control_posts_penetration() {
    // Remove a 0.01 mm slab around the intended Z347 shoulder contact.  Any
    // remaining object is true volume penetration, not a coplanar interface.
    difference() {
        qa_control_posts_collision();
        translate([-400,-250,panel_z-0.005]) cube([500,500,0.01]);
    }
}

module qa_control_carrier_collision() {
    intersection() { installed_panel(); carrier_installed(); }
}

module p281_post_collision_debug() {
    color([0.15,0.42,0.75,0.55]) installed_panel();
    color([0.75,0.75,0.78,0.65]) {
        panel_post_installed(panel_center_x-58,-1,"rear_negative");
        panel_post_installed(panel_center_x+58,-1,"front_negative");
        panel_post_installed(panel_center_x-58,1,"positive");
        panel_post_installed(panel_center_x+58,1,"positive");
    }
    color([0.95,0.05,0.03]) qa_control_posts_collision();
}

module p281_exploded_preview() {
    // Exact geometry, panel lifted 18 mm only for visibility of ribs and the
    // four flat shoulders.  No release dimension is derived from this view.
    color([0.15,0.42,0.75]) translate([0,0,18]) installed_panel();
    color([0.92,0.56,0.12]) intersection() {
        union() {
            panel_post_installed(panel_center_x-58,-1,"rear_negative");
            panel_post_installed(panel_center_x+58,-1,"front_negative");
            panel_post_installed(panel_center_x-58,1,"positive");
            panel_post_installed(panel_center_x+58,1,"positive");
        }
        translate([-300,-100,317]) cube([210,200,36]);
    }
}

module qa_priority_collision() {
    intersection() { installed_structure_without_panel_or_base(); priority_keepouts(); }
}

module qa_pi_collision() {
    intersection() {
        translate([pi_center_x,0,pi_shelf_z+plate_t+pi_clear[2]/2])
            cube(pi_clear,center=true);
        union() {
            carrier_installed();
            electronics_ghosts();
            for (side=[-1,1]) translate([panel_center_x,side*54,64]) side_frame();
            panel_post_installed(panel_center_x-58,-1,"rear_negative");
            panel_post_installed(panel_center_x+58,-1,"front_negative");
            panel_post_installed(panel_center_x-58,1,"positive");
            panel_post_installed(panel_center_x+58,1,"positive");
        }
    }
}


module qa_all_reserved_collision() {
    intersection() {
        installed_p279_structure();
        union() { priority_keepouts();
            translate([pi_center_x,0,pi_shelf_z+plate_t+pi_clear[2]/2])
                cube(pi_clear,center=true);
        }
    }
}

module assembly() {
    color([.80,.55,.12]) {
        translate([0,0,pod_z_bottom]) base_half_v2(-1);
        translate([0,0,pod_z_bottom]) base_half_v2(1);
    }
    color([.18,.45,.72]) translate([pi_center_x,0,pi_shelf_z]) pi_shelf_v2();
    color([.92,.63,.14]) for (side=[-1,1])
        translate([panel_center_x,side*54,64]) side_frame();
    color([.28,.52,.70]) carrier_installed();
    color([.82,.48,.10]) {
        corner_tie_installed(-1);
        corner_tie_installed(1);
    }

    // One rear-negative post, one front-negative detour and two positive
    // detours. The
    // positive rear/front pair route outside Y78 around the fixed Ugreen.
    color([.92,.63,.14]) {
        panel_post_installed(panel_center_x-58,-1,"rear_negative");
        panel_post_installed(panel_center_x+58,-1,"front_negative");
        panel_post_installed(panel_center_x-58,1,"positive");
        panel_post_installed(panel_center_x+58,1,"positive");
    }
    if (show_panel) color([.18,.45,.72])
        translate([panel_center_x,0,panel_z]) control_panel_v2();

    // Front/chassis-side bulkhead retained exactly from v1.
    color([.24,.30,.35]) {
        multmatrix([[0,0,-1,-125],[1,0,0,0],[0,1,0,110],[0,0,0,1]])
            import("stl-v2/cable_bulkhead.stl",convexity=10);
        for (side=[-1,1]) translate([-137,side*40,pod_z_bottom]) rotate([0,0,-90])
            import("stl-v2/bulkhead_foot.stl",convexity=10);
    }
    if (show_panel) controls_ghosts();
    if (show_ghosts) { reserved_ghosts(); electronics_ghosts(); }
}

module envelope_ghost() {
    color([.95,.55,.08,.08]) translate([0,0,pod_z_bottom])
        linear_extrude(height=pod_body_top-pod_z_bottom) p276_footprint_2d();
}

assert(carrier_y_lower_max-carrier_y_min == 248,
       "carrier printable width must stay <=250 mm");
assert(carrier_top_z-139 <= 250,
       "carrier printable height must stay <=250 mm");
assert(114 + 10 <= 124, "Pi-to-cable-corridor clearance must be >=10 mm");
assert(149 + 198 == panel_z,
       "P2-81 flat post shoulder must coincide with panel underside");
assert(8.2 < 8.6,
       "P2-81 locating peg must remain smaller than mounting aperture");
assert(estop_hole == 22.5 && toggle_hole == 12.3 && arm_hole == 16.3,
       "P2-81 control apertures must remain frozen");

if (part=="base_left") base_half_v2(1);
else if (part=="base_right") base_half_v2(-1);
else if (part=="pi_shelf") pi_shelf_v2();
else if (part=="vertical_electronics_plate") vertical_electronics_plate();
else if (part=="side_frame") rotate([90,0,0]) side_frame();
else if (part=="panel_posts") panel_posts_print_set();
else if (part=="corner_tie") corner_tie();
else if (part=="component_pins") component_pin_plate();
else if (part=="component_clips") component_clip_plate();
else if (part=="bts_spacers") bts_spacer_plate();
else if (part=="upright_mount_fit_coupon") upright_mount_fit_coupon();
else if (part=="control_panel") control_panel_v2();
else if (part=="qa_priority_collision") qa_priority_collision();
else if (part=="qa_pi_collision") qa_pi_collision();
else if (part=="qa_all_reserved_collision") qa_all_reserved_collision();
else if (part=="qa_control_bodies_collision") qa_control_bodies_collision();
else if (part=="qa_control_ugreen_collision") qa_control_ugreen_collision();
else if (part=="qa_control_posts_collision") qa_control_posts_collision();
else if (part=="qa_control_posts_penetration") qa_control_posts_penetration();
else if (part=="qa_control_carrier_collision") qa_control_carrier_collision();
else if (part=="p281_post_collision_debug") p281_post_collision_debug();
else if (part=="p281_exploded_preview") p281_exploded_preview();
else if (part=="envelope") envelope_ghost();
else if (part=="assembly") assembly();
else { envelope_ghost(); assembly(); }
