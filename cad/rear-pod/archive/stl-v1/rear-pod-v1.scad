// P2-75 printable rear pod v1 for the 12 V PSU motion test.
// Select a printable part with -D 'part="..."'.

include <params-v1.scad>

part = "assembly";
pillar_length = 38;
show_ghosts = true;

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

module cross_slot_2d(span=board_slot_span, width=m3_slot_w) {
    union() {
        capsule_2d(span,width,true);
        capsule_2d(span,width,false);
    }
}

module plate(size, r=5, t=plate_t) {
    linear_extrude(height=t) rounded_rect_2d(size,r);
}

module vent_field(xs, ys, length=28, width=4.5) {
    for (x=xs,y=ys)
        translate([x,y,-1]) linear_extrude(height=plate_t+2)
            capsule_2d(length,width,true);
}

module board_boss(pos, h, slot_span=board_slot_span) {
    difference() {
        translate([pos[0],pos[1],plate_t]) cylinder(d=10,h=h);
        translate([pos[0],pos[1],-1]) linear_extrude(height=plate_t+h+2)
            cross_slot_2d(slot_span,m3_slot_w);
    }
}

module pillar_holes(grid) {
    for (p=grid) translate([p[0],p[1],-1]) cylinder(d=pillar_hole_d,h=plate_t+2);
}

module all_pillar_holes() {
    pillar_holes(pillar_grid_a);
    pillar_holes(pillar_grid_b);
}

module p275_footprint_2d() {
    polygon(points=[
        [pod_x_front,-pod_y_half],
        [pod_x_rear+pod_rear_chamfer,-pod_y_half],
        [pod_x_rear,-pod_y_half+pod_rear_chamfer],
        [pod_x_rear,pod_y_half-pod_rear_chamfer],
        [pod_x_rear+pod_rear_chamfer,pod_y_half],
        [pod_x_front,pod_y_half]
    ]);
}

module female_socket_cut(top_z) {
    translate([0,0,top_z-pillar_socket_depth])
        cylinder(d=pillar_socket_d,h=pillar_socket_depth+1);
    translate([0,0,top_z-pillar_socket_depth-pillar_screw_engagement])
        cylinder(d=pillar_screw_pilot_d,
                 h=pillar_screw_engagement+pillar_socket_depth+1);
}

module base_socket_boss(x,y) {
    translate([x,y,plate_t]) difference() {
        cylinder(d=pillar_body_d,h=5);
        female_socket_cut(5);
    }
}

module base_half(side=1) {
    difference() {
        union() {
            intersection() {
                linear_extrude(height=plate_t) p275_footprint_2d();
                translate([pod_x_rear-1,side>0?0:-pod_y_half-1,-1])
                    cube([pod_x_front-pod_x_rear+2,pod_y_half+1,plate_t+2]);
            }
            // Fixed five-millimetre sockets preserve the P2-72 Mega datum Z64.
            for (x=[-198.5-58,-198.5+58]) base_socket_boss(x,side*56);
        }
        // Existing rear-crossbar top grid: no new chassis holes.
        for (y=[75,125])
            translate([-165,side*y,-1]) cylinder(d=m4_clear,h=plate_t+2);
        // M3 pilots continue through the base below the integral sockets.
        for (x=[-198.5-58,-198.5+58])
            translate([x,side*56,-1])
                cylinder(d=pillar_screw_pilot_d,h=plate_t+7);
        // Tie-down slots only; future battery rails remain unreleased.
        for (x=[-245,-210,-175,-140])
            translate([x,side*145,-1]) linear_extrude(height=plate_t+2)
                capsule_2d(14,4,true);
    }
}

module mega_shelf() {
    difference() {
        plate([138,124],5);
        all_pillar_holes();
        for (x=[-42,0,42])
            translate([x,0,-1]) linear_extrude(height=plate_t+2)
                capsule_2d(42,5,false);
        for (y=[-34,34])
            translate([0,y,-1]) linear_extrude(height=plate_t+2)
                capsule_2d(18,4,true);
    }
    // Measured Mega enclosure holes; through cross-slots accept the 2.8 mm pins.
    for (p=[[52,-25],[52,25],[-10,-18.5],[-10,18.5]]) board_boss(p,4,6);
}

module perf_shelf() {
    difference() {
        plate([138,124],5);
        all_pillar_holes();
        vent_field([-40,0,40],[-22,0,22],40,4.5);
        for (y=[-48,48])
            translate([0,y,-1]) linear_extrude(height=plate_t+2)
                capsule_2d(18,4,true);
    }
    for (x=[-57.5,57.5],y=[-37.5,37.5]) board_boss([x,y],10,8);
}

module bts_shelf() {
    difference() {
        plate([138,124],5);
        all_pillar_holes();
        for (cy=[-29,29])
            vent_field([-13,0,13],[cy-11,cy,cy+11],24,4.5);
        for (x=[-46,46],y=[-29,29])
            translate([x,y,-1]) linear_extrude(height=plate_t+2)
                capsule_2d(14,4,false);
        // Through cross-slots below the eight separate 20 mm spacers.
        for (cy=[-29,29],x=[-20,20],y=[cy-20,cy+20])
            translate([x,y,-1]) linear_extrude(height=plate_t+2)
                cross_slot_2d(6,m3_slot_w);
    }
}

module pi_shelf() {
    difference() {
        plate([138,124],5);
        all_pillar_holes();
        vent_field([-34,-17,0,17,34],[-18,0,18],28,4.5);
        for (x=[-30,30],y=[-48,48])
            translate([x,y,-1]) linear_extrude(height=plate_t+2)
                capsule_2d(22,4,true);
    }
    for (x=[-51,51],y=[-39,39])
        translate([x,y,plate_t]) cylinder(d=6,h=6);
}

module control_panel() {
    difference() {
        plate([138,190],6);
        // The panel is the top of the stack and only needs its receiving grid.
        pillar_holes(pillar_grid_a);
        translate([0,-58,-1]) cylinder(d=estop_hole,h=plate_t+2);
        translate([0,-12,-1]) cylinder(d=toggle_hole,h=plate_t+2);
        translate([0,32,-1]) cylinder(d=arm_hole,h=plate_t+2);
        // Harness exit only.  Both switch LEDs are integrated in their controls.
        translate([-42,70,-1]) linear_extrude(height=plate_t+2)
            capsule_2d(24,8,true);
    }
}

// Male snap below, female locating socket and M3 pilot above.  The axial span
// is the clear distance between adjacent shelves, excluding the 4 mm plates.
module snap_pillar(span=38) {
    difference() {
        union() {
            // 45-degree lead-in and split annular barb: printable upright.
            cylinder(d1=pillar_snap_neck_d-1.2,d2=pillar_snap_barb_d,h=1.6);
            translate([0,0,1.6]) cylinder(d=pillar_snap_neck_d,h=pillar_snap_h-1.6);
            translate([0,0,pillar_snap_h]) cylinder(d=pillar_body_d,h=span);
        }
        // Two relief slots let the PETG snap compress in the 8.6 mm coupon hole.
        translate([-0.65,-pillar_snap_barb_d/2-1,-0.2])
            cube([1.3,pillar_snap_barb_d+2,pillar_snap_h+0.4]);
        translate([-pillar_snap_barb_d/2-1,-0.65,-0.2])
            cube([pillar_snap_barb_d+2,1.3,pillar_snap_h+0.4]);
        female_socket_cut(pillar_snap_h+span);
    }
}

module shelf_coupler() {
    difference() {
        union() {
            cylinder(d=coupler_head_d,h=coupler_head_t);
            translate([0,0,coupler_head_t])
                cylinder(d1=coupler_d,d2=coupler_d-0.3,h=pillar_socket_depth);
        }
        translate([0,0,-1]) cylinder(d=3.4,h=coupler_head_t+pillar_socket_depth+2);
    }
}

module installed_couplers(grid,z) {
    for (p=grid)
        translate([p[0],p[1],z+plate_t+coupler_head_t])
            rotate([180,0,0]) shelf_coupler();
}

module pillar_group(span=38) {
    for (i=[0:3]) translate([(i%2)*24,floor(i/2)*24,0]) snap_pillar(span);
}

module coupler_group(count=24) {
    for (i=[0:count-1]) translate([(i%5)*18,floor(i/5)*18,0]) shelf_coupler();
}

module bts_spacer() {
    difference() {
        cylinder(d=10,h=bts_spacer_h);
        translate([0,0,-1]) cylinder(d=3.4,h=bts_spacer_h+2);
    }
}

module bts_spacer_plate() {
    for (i=[0:9]) translate([(i%5)*14,floor(i/5)*14,0]) bts_spacer();
}

module chassis_spacer_plate() {
    for (i=[0:3]) translate([i*16,0,0]) difference() {
        cylinder(d=12,h=3);
        translate([0,0,-1]) cylinder(d=m4_clear,h=5);
    }
}

// Adapted from cad/motion-electronics-tray/component-retaining-pins.scad.
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
bts_pin_grip = plate_t + bts_spacer_h + 1.6 + pin_fit;

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
        translate([0,-clip_mouth_w/2,-1])
            cube([clip_od/2+1,clip_mouth_w,clip_t+2]);
    }
}

module pin_row(grip,count,y) {
    for (i=[0:count-1]) translate([(i%5)*13,y+floor(i/5)*13,0]) retaining_pin(grip);
}

module component_pin_plate() {
    // 4 Mega + spare, 4 perf + spare, 8 BTS + spare.
    pin_row(mega_pin_grip,5,0);
    pin_row(perf_pin_grip,5,18);
    pin_row(bts_pin_grip,9,36);
}

module component_clip_plate(count=24) {
    for (i=[0:count-1])
        translate([(i%6)*14,floor(i/6)*14,0]) locking_clip();
}

module cable_bulkhead() {
    difference() {
        plate([130,90],5);
        for (p=[[-50,-18],[-36,8],[36,8],[50,-18]])
            translate([p[0],p[1],-1]) linear_extrude(height=plate_t+2)
                capsule_2d(13,7,true);
        for (x=[-18,-6,6,18],y=[-23,3])
            if ((y==-23 && abs(x)>10) || (y==3 && abs(x)<10))
                translate([x,y,-1]) linear_extrude(height=plate_t+2)
                    capsule_2d(9,5,true);
        translate([-10,27,-1]) cube([20,10,plate_t+2]);
        translate([13,29,-1]) linear_extrude(height=plate_t+2)
            capsule_2d(10,5,true);
        translate([-55,31,-1]) linear_extrude(height=plate_t+2)
            capsule_2d(18,9,true);
        for (x=[-48,-32],y=[29,40])
            translate([x,y,-1]) linear_extrude(height=plate_t+2)
                capsule_2d(12,4,false);
        for (x=[-58,58],y=[-38,38])
            translate([x,y,-1]) linear_extrude(height=plate_t+2)
                capsule_2d(10,m4_clear,true);
    }
    for (x=[-27,27]) translate([x,-42,plate_t]) cube([3,84,1]);
}

module bulkhead_foot() {
    difference() {
        union() {
            translate([0,0,2]) cube([55,24,4],center=true);
            translate([0,10,17]) cube([55,4,30],center=true);
            for (x=[-22,22]) hull() {
                translate([x,5,4]) cube([4,10,4],center=true);
                translate([x,9,18]) cube([4,2,4],center=true);
            }
        }
        for (x=[-18,18]) translate([x,-2,-1]) cylinder(d=m4_clear,h=8);
        for (x=[-18,18]) translate([x,13,17]) rotate([90,0,0])
            cylinder(d=m4_clear,h=8);
    }
}

module mpu6050_chassis_mount() {
    difference() {
        plate([76,38],4);
        for (x=[-30,30]) translate([x,0,-1]) cylinder(d=m4_clear,h=6);
        for (x=[-10,10],y=[-10,10])
            translate([x,y,-1]) linear_extrude(height=6)
                cross_slot_2d(8,3.2);
    }
}

module fit_coupon_tile() {
    difference() {
        plate([70,42],3);
        translate([-24,0,-1]) cylinder(d=pillar_hole_d,h=plate_t+2);
    }
    board_boss([3,-11],4,6);
    board_boss([3,11],10,8);
}

module fit_coupon_set() {
    // Detach the loose samples, then test both pin heights and the complete
    // male-snap/female-socket/M3 coupling before printing full shelves.
    translate([35,21,0]) fit_coupon_tile();
    translate([84,10,0]) snap_pillar(28);
    translate([105,10,0]) shelf_coupler();
    translate([84,35,0]) retaining_pin(mega_pin_grip);
    translate([98,35,0]) retaining_pin(perf_pin_grip);
    translate([84,52,0]) locking_clip();
    translate([100,52,0]) locking_clip();
}

module ghost_components() {
    color([.10,.42,.72,.55]) translate([-198.5,0,81]) cube([110,60,18],center=true);
    color([.18,.55,.75,.55]) translate([-198.5,0,129]) cube([120,80,18],center=true);
    for (y=[-29,29])
        color([.74,.18,.12,.55]) translate([-198.5,y,192]) cube([50,50,20],center=true);
    color([.10,.14,.20,.60]) translate([-198.5,0,242]) cube(pi_clear,center=true);
    for (side=[-1,1])
        color([.10,.58,.22,.22]) translate([-198.5,side*115.5,98.5])
            cube(parkside_clear,center=true);
    color([.16,.18,.25,.22]) translate([-198.5,112,264]) cube(ugreen_clear,center=true);
    color([.72,.62,.12,.20]) translate([-205,-108,185]) cube([120,44,42],center=true);
}

module placed_pillars(grid,z,span) {
    color([.92,.63,.14]) for (p=grid)
        translate([-198.5+p[0],p[1],z]) snap_pillar(span);
}

module placed_couplers(grid,z) {
    color([.95,.78,.28])
        translate([-198.5,0,0]) installed_couplers(grid,z);
}

module assembly() {
    color([.80,.55,.12]) {
        translate([0,0,pod_z_bottom]) base_half(-1);
        translate([0,0,pod_z_bottom]) base_half(1);
    }
    color([.18,.45,.72]) {
        translate([-198.5,0,mega_z]) mega_shelf();
        translate([-198.5,0,perf_z]) perf_shelf();
        translate([-198.5,0,bts_z]) bts_shelf();
        translate([-198.5,0,pi_z]) pi_shelf();
        translate([-198.5,0,panel_z]) control_panel();
    }
    placed_couplers(pillar_grid_a,mega_z);
    placed_pillars(pillar_grid_b,mega_z,mega_span_default);
    placed_couplers(pillar_grid_b,perf_z);
    placed_pillars(pillar_grid_a,perf_z,perf_span_default);
    placed_couplers(pillar_grid_a,bts_z);
    placed_pillars(pillar_grid_b,bts_z,bts_span_default);
    placed_couplers(pillar_grid_b,pi_z);
    placed_pillars(pillar_grid_a,pi_z,pi_span_default);
    placed_couplers(pillar_grid_a,panel_z);

    // Front/chassis-side bulkhead: its thickness points inward to X-, so the
    // frozen X=-125 envelope face is not crossed.
    color([.24,.30,.35]) {
        multmatrix([[0,0,-1,-125],[1,0,0,0],[0,1,0,110],[0,0,0,1]])
            cable_bulkhead();
        for (side=[-1,1])
            translate([-137,side*40,pod_z_bottom]) rotate([0,0,-90]) bulkhead_foot();
    }
    color([.85,.08,.08,.65]) translate([-198.5,-58,panel_z-25]) cube([35,30,50],center=true);
    color([.30,.30,.32,.60]) translate([-198.5,-12,panel_z-15]) cube([27,15,30],center=true);
    color([.18,.70,.25,.60]) translate([-198.5,32,panel_z-12.5]) cylinder(d=15,h=25,center=true);
    color([.86,.08,.08,.80]) translate([-198.5,-58,panel_z+14.5]) cylinder(d=38,h=21,center=true);
    if (show_ghosts) ghost_components();
}

module envelope_ghost() {
    color([.95,.55,.08,.09]) translate([0,0,pod_z_bottom])
        linear_extrude(height=pod_body_top-pod_z_bottom) p275_footprint_2d();
}

assert(138 <= 250 && 190 <= 250, "printable plate exceeds P2S 250 mm limit");
assert(panel_z + plate_t + 21 <= pod_absolute_top, "E-stop top exceeds Z375");
assert(pin_shaft_d < m3_slot_w, "pin shaft must pass through shelf cross-slot");
assert(clip_od > board_slot_span, "C-clip must bridge the adjustment slot");
assert(pillar_snap_barb_d > pillar_hole_d, "snap barb needs positive interference");
assert(pillar_snap_neck_d < pillar_hole_d, "snap neck must clear receiver hole");
assert(abs(pillar_grid_a[0][0]-pillar_grid_b[0][0]) >
       (pillar_body_d+coupler_head_d)/2,
       "alternating pillar and coupler grids must not overlap");

if (part=="base_left") base_half(1);
else if (part=="base_right") base_half(-1);
else if (part=="mega_shelf") mega_shelf();
else if (part=="perf_shelf") perf_shelf();
else if (part=="bts_shelf") bts_shelf();
else if (part=="pi_shelf") pi_shelf();
else if (part=="control_panel") control_panel();
else if (part=="pillar") pillar_group(pillar_length);
else if (part=="shelf_couplers") coupler_group();
else if (part=="component_pins") component_pin_plate();
else if (part=="component_clips") component_clip_plate();
else if (part=="bts_spacers") bts_spacer_plate();
else if (part=="chassis_spacers") chassis_spacer_plate();
else if (part=="cable_bulkhead") cable_bulkhead();
else if (part=="bulkhead_foot") bulkhead_foot();
else if (part=="mpu6050_mount") mpu6050_chassis_mount();
else if (part=="fit_coupon") fit_coupon_set();
else if (part=="envelope") envelope_ghost();
else if (part=="assembly") assembly();
else { envelope_ghost(); assembly(); }
