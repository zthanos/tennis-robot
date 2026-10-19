// P2-72 printable rear pod v0 for the 12 V PSU motion test.
// No battery docks, relay, DC-DC, Ugreen or D5065-controller mounts are released.
// Select a printable part with -D 'part="..."'.

include <params.scad>

part = "assembly";
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

module rod_holes() {
    // Shelf-local rod coordinates around rack centre X=-198.5/Y=0.
    for (x=[-59.5,59.5],y=[-58,58])
        translate([x,y,-1]) cylinder(d=m4_clear,h=plate_t+2);
}

module p272_footprint_2d() {
    polygon(points=[
        [pod_x_front,-pod_y_half],
        [pod_x_rear+pod_rear_chamfer,-pod_y_half],
        [pod_x_rear,-pod_y_half+pod_rear_chamfer],
        [pod_x_rear,pod_y_half-pod_rear_chamfer],
        [pod_x_rear+pod_rear_chamfer,pod_y_half],
        [pod_x_front,pod_y_half]
    ]);
}

module base_half(side=1) {
    difference() {
        intersection() {
            linear_extrude(height=plate_t) p272_footprint_2d();
            translate([pod_x_rear-1,side>0?0:-pod_y_half-1,-1])
                cube([pod_x_front-pod_x_rear+2,pod_y_half+1,plate_t+2]);
        }
        // Existing rear-crossbar top grid: no new chassis holes.
        for (y=[75,125])
            translate([-165,side*y,-1]) cylinder(d=m4_clear,h=plate_t+2);
        // M4 threaded rack rods; each half carries one +/-Y row.
        for (x=[-258,-139])
            translate([x,side*58,-1]) cylinder(d=m4_clear,h=plate_t+2);
        // Tie-down slots only; future battery rails remain unreleased.
        for (x=[-245,-210,-175,-140])
            translate([x,side*145,-1]) linear_extrude(height=plate_t+2)
                capsule_2d(14,4,true);
    }
}

module mega_shelf() {
    // 138x124 stays 8 mm clear of the future Parkside reservations at |Y|=70.
    difference() {
        plate([138,124],5);
        rod_holes();
        for (x=[-42,0,42])
            translate([x,0,-1]) linear_extrude(height=plate_t+2)
                capsule_2d(42,5,false);
        for (y=[-34,34])
            translate([0,y,-1]) linear_extrude(height=plate_t+2)
                capsule_2d(18,4,true);
    }
    // Rotated measured Mega enclosure hole pattern. Cross-slots absorb ruler error.
    for (p=[[52,-25],[52,25],[-10,-18.5],[-10,18.5]]) board_boss(p,4,6);
}

module perf_shelf() {
    difference() {
        plate([138,124],5);
        rod_holes();
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
        rod_holes();
        for (cy=[-29,29])
            vent_field([-13,0,13],[cy-11,cy,cy+11],24,4.5);
        // Cable ties at the power-terminal edges.
        for (x=[-46,46],y=[-29,29])
            translate([x,y,-1]) linear_extrude(height=plate_t+2)
                capsule_2d(14,4,false);
        // Cross-slots below the eight separate 20 mm spacers.
        for (cy=[-29,29],x=[-20,20],y=[cy-20,cy+20])
            translate([x,y,-1]) linear_extrude(height=plate_t+2)
                cross_slot_2d(6,m3_slot_w);
    }
}

module pi_shelf() {
    difference() {
        plate([138,124],5);
        rod_holes();
        vent_field([-34,-17,0,17,34],[-18,0,18],28,4.5);
        // Two 20 mm strap paths; 98x74 clear envelope is bounded by the slots.
        for (x=[-30,30],y=[-48,48])
            translate([x,y,-1]) linear_extrude(height=plate_t+2)
                capsule_2d(22,4,true);
    }
    // Low corner keepers provide the 3 mm/side plan clearance without a tight fit.
    for (x=[-51,51],y=[-39,39])
        translate([x,y,plate_t]) cylinder(d=6,h=6);
}

module control_panel() {
    difference() {
        plate([138,190],6);
        rod_holes();
        translate([0,-58,-1]) cylinder(d=estop_hole,h=plate_t+2);
        translate([0,-12,-1]) cylinder(d=toggle_hole,h=plate_t+2);
        translate([0,32,-1]) cylinder(d=arm_hole,h=plate_t+2);
        // Replaceable LED insert aperture; LED hole waits for user measurement.
        translate([0,72,-1]) cube([26,20,plate_t+2],center=true);
        for (x=[-13,13]) translate([x,72,-1]) cylinder(d=3.4,h=plate_t+2);
        // Harness exit behind the controls.
        translate([-42,70,-1]) linear_extrude(height=plate_t+2)
            capsule_2d(24,8,true);
    }
}

module led_blank_insert() {
    // Friction-free blank retained by two M3 screws. Drill only after LED measurement.
    difference() {
        plate([34,28],3,2.4);
        for (x=[-13,13]) translate([x,0,-1]) cylinder(d=3.4,h=5);
    }
}

module bts_spacer() {
    difference() {
        cylinder(d=10,h=bts_spacer_h);
        translate([0,0,-1]) cylinder(d=3.4,h=bts_spacer_h+2);
    }
}

module bts_spacer_plate() {
    // Eight required plus two spares.
    for (i=[0:9]) translate([(i%5)*14,floor(i/5)*14,0]) bts_spacer();
}

module chassis_spacer_plate() {
    // Four 3 mm packers bridge rear-crossbar top Z52 to pod base Z55.
    for (i=[0:3]) translate([i*16,0,0]) difference() {
        cylinder(d=12,h=3);
        translate([0,0,-1]) cylinder(d=m4_clear,h=5);
    }
}

module cable_bulkhead() {
    // Printed flat. Local X becomes robot Y, local Y becomes robot Z.
    difference() {
        plate([130,90],5);
        // Two motor lines per side, kept outside the signal zone.
        for (p=[[-50,-18],[-36,8],[36,8],[50,-18]])
            translate([p[0],p[1],-1]) linear_extrude(height=plate_t+2)
                capsule_2d(13,7,true);
        // Four encoder leads: dedicated tie slots.
        for (x=[-18,-6,6,18],y=[-23,3])
            if ((y==-23 && abs(x)>10) || (y==3 && abs(x)<10))
                translate([x,y,-1]) linear_extrude(height=plate_t+2)
                    capsule_2d(9,5,true);
        // USB Pi-Mega and MPU6050 cable.
        translate([-10,27,-1]) cube([20,10,plate_t+2]);
        translate([13,29,-1]) linear_extrude(height=plate_t+2)
            capsule_2d(10,5,true);
        // 12 V PSU lead and adjustable inline-fuse tie field. Connector/holder
        // bodies remain MEASUREMENT USER.
        translate([-55,31,-1]) linear_extrude(height=plate_t+2)
            capsule_2d(18,9,true);
        for (x=[-48,-32],y=[29,40])
            translate([x,y,-1]) linear_extrude(height=plate_t+2)
                capsule_2d(12,4,false);
        // Four M4 mounting slots to base/panel brackets.
        for (x=[-58,58],y=[-38,38])
            translate([x,y,-1]) linear_extrude(height=plate_t+2)
                capsule_2d(10,m4_clear,true);
    }
    // Two shallow in-plane ribs mark the boundary between power and signals.
    // They do not project into the shelf stack.
    for (x=[-27,27]) translate([x,-42,plate_t]) cube([3,84,1]);
}

module bulkhead_foot() {
    // Print upright on the base; 45 degree ribs avoid support material.
    difference() {
        union() {
            translate([0,0,2]) cube([55,24,4],center=true);
            translate([0,10,17]) cube([55,4,30],center=true);
            for (x=[-22,22])
                hull() {
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
    // Fixed to the X240 crossbar top at the existing Y +/-30 sleeve holes.
    // The board footprint/hole pattern remains MEASUREMENT USER, hence cross-slots.
    difference() {
        plate([76,38],4);
        for (x=[-30,30]) translate([x,0,-1]) cylinder(d=m4_clear,h=6);
        for (x=[-10,10],y=[-10,10])
            translate([x,y,-1]) linear_extrude(height=6)
                cross_slot_2d(8,3.2);
    }
}

module ghost_components() {
    // Installed current components.
    color([.10,.42,.72,.55]) translate([-198.5,0,81]) cube([110,60,18],center=true);
    color([.18,.55,.75,.55]) translate([-198.5,0,129]) cube([120,80,18],center=true);
    for (y=[-29,29])
        color([.74,.18,.12,.55]) translate([-198.5,y,192]) cube([50,50,20],center=true);
    color([.10,.14,.20,.60]) translate([-198.5,0,242]) cube(pi_clear,center=true);

    // Future accepted reservations only; no corresponding printed mounts in v0.
    for (side=[-1,1])
        color([.10,.58,.22,.22]) translate([-198.5,side*115.5,98.5])
            cube(parkside_clear,center=true);
    color([.16,.18,.25,.22]) translate([-198.5,112,264]) cube(ugreen_clear,center=true);
    color([.72,.62,.12,.20]) translate([-205,-108,185]) cube([120,44,42],center=true);
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
    // Purchased M4 threaded rods; not printable parts.
    color([.72,.72,.74]) for (p=rod_xy)
        translate([p[0],p[1],pod_z_bottom+plate_t]) cylinder(d=4,h=panel_z-pod_z_bottom);
    // Cable bulkhead: local X->global Y, local Y->global Z, local Z->global X.
    color([.24,.30,.35]) {
        multmatrix([[0,0,1,-272],[1,0,0,0],[0,1,0,110],[0,0,0,1]]) cable_bulkhead();
        for (side=[-1,1])
            translate([-260,side*40,pod_z_bottom]) rotate([0,0,90]) bulkhead_foot();
    }
    // Control body clearance ghosts: deepest assumed body is 50 mm.
    color([.85,.08,.08,.65]) translate([-198.5,-58,panel_z-25]) cube([35,30,50],center=true);
    color([.30,.30,.32,.60]) translate([-198.5,-12,panel_z-15]) cube([27,15,30],center=true);
    color([.18,.70,.25,.60]) translate([-198.5,32,panel_z-12.5]) cylinder(d=15,h=25,center=true);
    color([.86,.08,.08,.80]) translate([-198.5,-58,panel_z+14.5]) cylinder(d=38,h=21,center=true);
    if (show_ghosts) ghost_components();
}

module envelope_ghost() {
    color([.95,.55,.08,.09]) translate([0,0,pod_z_bottom])
        linear_extrude(height=pod_body_top-pod_z_bottom) p272_footprint_2d();
}

assert(138 <= 250 && 190 <= 250, "Printable plate exceeds P2S 250 mm limit");
assert(panel_z + plate_t + 21 <= pod_absolute_top,
       "E-stop top exceeds Z375");
assert(max([for(p=rod_xy) abs(p[1])]) < 70,
       "Rack intrudes into Parkside side reservation");

if (part=="base_left") base_half(1);
else if (part=="base_right") base_half(-1);
else if (part=="mega_shelf") mega_shelf();
else if (part=="perf_shelf") perf_shelf();
else if (part=="bts_shelf") bts_shelf();
else if (part=="pi_shelf") pi_shelf();
else if (part=="control_panel") control_panel();
else if (part=="led_blank_insert") led_blank_insert();
else if (part=="bts_spacers") bts_spacer_plate();
else if (part=="chassis_spacers") chassis_spacer_plate();
else if (part=="cable_bulkhead") cable_bulkhead();
else if (part=="bulkhead_foot") bulkhead_foot();
else if (part=="mpu6050_mount") mpu6050_chassis_mount();
else if (part=="envelope") envelope_ghost();
else { envelope_ghost(); assembly(); }
