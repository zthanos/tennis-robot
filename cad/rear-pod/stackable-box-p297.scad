// P2-97 Phase 1 — FIT-CHECK stackable electronics box system.
// One tower, keyed shear seams and four M3 screws per seam into
// Ruthex RX-M3x5.7 heat-set inserts.  Select with -D 'part="..."'.

use <rear-pod-v2.scad>
include <params-v2.scad>

part = "p297_assembly";
show_ghosts = true;

// Frozen external envelope: the panel is 138 x 190 and defines the tower.
p297_cx = panel_center_x;
p297_outer = [138,190];
p297_wall = 4;
p297_floor = 3.2;
p297_corner = 11;
p297_lattice = 4.10;
p297_lattice_t = 3.2;
p297_key_h = 2.2;
p297_key_clear = 0.30;
p297_seam_gap = 0.25;  // avoids coplanar CGAL false positives at lattice walls
p297_seam_x = 58;
p297_seam_y = 84;
p297_insert_hole = 4.0;
p297_insert_depth = 6.7;
p297_m3_clear = 3.4;
p297_grid_pitch = 20;
p297_grid_x = [-50:20:50];
p297_grid_y = [-90:20:90];
p297_grid_rib = 8;

// Installed Z datums.  Panel datum is reduced by 26 mm from P2-90d.
p297_base_z = 64;       // seats on the existing 5 mm P2-90d locating bosses
p297_adapter_h = 8;
p297_energy_h = 70;
p297_power_h = 70;
p297_logic_h = 82;
p297_collar_h = 24;
p297_energy_z = p297_base_z + p297_adapter_h + p297_seam_gap;
p297_power_z = p297_energy_z + p297_energy_h + p297_seam_gap;
p297_logic_z = p297_power_z + p297_power_h + p297_seam_gap;
p297_collar_z = p297_logic_z + p297_logic_h + p297_seam_gap;
p297_panel_z = p297_collar_z + p297_collar_h;

// Physical board patterns.  Mega/perf/BTS values are measured parts recorded
// in motion-electronics-tray/params.scad.  Pi 5 is Raspberry Pi drawing
// RP-008347-DS-1 (85 x 56 board; 58 x 49 hole centres).
p297_bts_pattern = [40,40];
p297_perf_pattern = [75,115];
p297_mega_holes = [[-25,52], [25,52], [-18.5,-10], [18.5,-10]];
p297_pi_pattern = [58,49];

module p297_capsule_2d(length,width) {
    hull() for (x=[-(length-width)/2,(length-width)/2])
        translate([x,0]) circle(d=width,$fn=24);
}

module p297_slot(length=8,width=3.4,vertical=false,h=8) {
    linear_extrude(height=h)
        if (vertical) rotate(90) p297_capsule_2d(length,width);
        else p297_capsule_2d(length,width);
}

module p297_key_positions() {
    for (x=[-54,54],y=[-80,80]) translate([x,y,0]) children();
}

module p297_seam_positions() {
    for (x=[-p297_seam_x,p297_seam_x],y=[-p297_seam_y,p297_seam_y])
        translate([x,y,0]) children();
}

// Insert axes open upward.  The surrounding boss is >=9 mm diameter, above
// the 8.6 mm minimum derived from RX-M3x5.7 OD plus 2 mm radial wall.
module p297_insert_boss(h=10) {
    difference() {
        cylinder(d=10,h=h,$fn=36);
        translate([0,0,h-p297_insert_depth])
            cylinder(d=p297_insert_hole,h=p297_insert_depth+0.2,$fn=28);
    }
}

module p297_top_keys(z) {
    p297_key_positions()
        translate([0,0,z]) cube([12,12,p297_key_h],center=true);
}

module p297_bottom_key_pockets() {
    p297_key_positions()
        translate([0,0,-p297_floor/2+(p297_key_h+0.4)/2-0.1])
            cube([12+2*p297_key_clear,
                  12+2*p297_key_clear,
                  p297_key_h+0.4],center=true);
}

module p297_floor_plate(full=true) {
    difference() {
        union() {
            if (full) {
                // Open mounting grid: 8 mm ribs on a 20 mm pitch.  Every
                // intersection receives a through-M3 hole below.  It uses
                // less PETG than the former nine large floor windows while
                // providing regular mounting points over the entire floor.
                linear_extrude(height=p297_floor,center=true) union() {
                    for (x=p297_grid_x)
                        translate([x,0]) square([p297_grid_rib,p297_outer[1]],center=true);
                    for (y=p297_grid_y)
                        translate([0,y]) square([p297_outer[0],p297_grid_rib],center=true);
                    difference() {
                        square(p297_outer,center=true);
                        square([p297_outer[0]-16,p297_outer[1]-16],center=true);
                    }
                }
            }
            else difference() {
                cube([p297_outer[0],p297_outer[1],p297_floor],center=true);
                cube([p297_outer[0]-16,p297_outer[1]-16,p297_floor+2],center=true);
            }
            p297_seam_positions() cylinder(d=12,h=p297_floor,center=true,$fn=36);
        }
        p297_bottom_key_pockets();
        p297_seam_positions()
            translate([0,0,-p297_floor]) cylinder(d=p297_m3_clear,h=3*p297_floor,$fn=28);
        if (full)
            for (x=p297_grid_x,y=p297_grid_y)
                translate([x,y,-p297_floor])
                    cylinder(d=p297_m3_clear,h=3*p297_floor,$fn=28);
    }
}

// Diagonal lattice is extruded in the wall plane.  All diagonal rises are at
// least their runs, so the box prints upright without supports at 0.2 mm.
module p297_x_lattice_xy(x0,x1,z0,z1,w=p297_lattice) {
    hull() {
        translate([x0,z0]) circle(d=w,$fn=20);
        translate([x1,z1]) circle(d=w,$fn=20);
    }
    hull() {
        translate([x0,z1]) circle(d=w,$fn=20);
        translate([x1,z0]) circle(d=w,$fn=20);
    }
}

module p297_side_wall(y,h) {
    // X/Z sketch extruded through Y.
    translate([0,y,0]) rotate([90,0,0]) linear_extrude(height=p297_lattice_t,center=true)
        union() {
            for (x=[-69,-34.5,0,34.5,69])
                translate([x,h/2]) square([p297_wall,h],center=true);
            translate([0,2]) square([138,p297_wall],center=true);
            translate([0,h-2]) square([138,p297_wall],center=true);
            for (x=[-51.75,-17.25,17.25,51.75])
                p297_x_lattice_xy(x-17.25,x+17.25,5,h-5);
        }
}

module p297_front_wall(x,h) {
    // Y/Z sketch extruded through X.  Rear (-X) stays open for Phase 2.
    translate([x,0,0]) rotate([90,0,90]) linear_extrude(height=p297_lattice_t,center=true)
        union() {
            for (y=[-95,-47.5,0,47.5,95])
                translate([y,h/2]) square([p297_wall,h],center=true);
            translate([0,2]) square([190,p297_wall],center=true);
            translate([0,h-2]) square([190,p297_wall],center=true);
            for (y=[-71.25,-23.75,23.75,71.25])
                p297_x_lattice_xy(y-23.75,y+23.75,5,h-5);
        }
}

module p297_corner_posts(h) {
    for (x=[-67,67],y=[-93,93]) translate([x,y,h/2]) cube([6,6,h],center=true);
}

module p297_open_box(h,full_floor=true,front_lattice=true,top_keys=true,top_inserts=true) {
    difference() {
        union() {
            translate([0,0,p297_floor/2]) p297_floor_plate(full_floor);
            p297_side_wall(-93.4,h);
            p297_side_wall(93.4,h);
            if (front_lattice) p297_front_wall(66.4,h);
            p297_corner_posts(h);
            if (top_inserts) p297_seam_positions() p297_insert_boss(h);
            if (top_keys) p297_top_keys(h+p297_key_h/2);
        }
        // Bottom seam fasteners remain reachable from above.
        p297_seam_positions()
            translate([0,0,-0.1]) cylinder(d=p297_m3_clear,h=p297_floor+1,$fn=28);
        // Cut the female key pockets through the complete box union as well
        // as the floor submodule; this also clears any coincident corner/grid
        // material added outside p297_floor_plate().
        p297_key_positions()
            translate([0,0,(p297_key_h+p297_seam_gap+0.5)/2-0.1])
                cube([12+2*p297_key_clear,
                      12+2*p297_key_clear,
                      p297_key_h+p297_seam_gap+0.5],center=true);
    }
}

module p297_base_adapter() {
    difference() {
        union() {
            // Two light rails join the exact P2-90d base axes.
            for (y=[-54,54]) translate([0,y,p297_adapter_h/2])
                cube([133,14,p297_adapter_h],center=true);
            for (x=[-64,64]) translate([x,0,p297_adapter_h/2])
                cube([10,178,p297_adapter_h],center=true);
            p297_seam_positions() p297_insert_boss(p297_adapter_h);
            p297_top_keys(p297_adapter_h+p297_key_h/2);
        }
        // Existing rear and field-jigged front holes; no chassis drilling.
        for (x=[side_frame_rear_x-panel_center_x,
                side_frame_front_x-panel_center_x],y=[-54,54])
            translate([x,y,-1]) cylinder(d=m4_clear,h=p297_adapter_h+2,$fn=32);
    }
}

module p297_energy_spacer() {
    // Phase 1 contains no Parkside geometry.  Rear is completely open and the
    // floor is only a perimeter ring, ready for measured Phase 2 drawers.
    p297_open_box(p297_energy_h,false,true);
}

module p297_bts_standoffs() {
    // Centres are aligned to the 20 mm mounting grid so all sixteen bosses
    // grow directly from grid intersections instead of floating in cutouts.
    for (cx=[-30,30],cy=[-30,30])
        for (dx=[-20,20],dy=[-20,20])
            translate([cx+dx,cy+dy,p297_floor]) difference() {
                cylinder(d=9,h=8,$fn=32);
                translate([0,0,-1]) cylinder(d=4.2,h=10,$fn=28);
            }
}

module p297_power_box() {
    difference() {
        union() {
            p297_open_box(p297_power_h,true,true);
            p297_bts_standoffs();
            // Fuse and DC-DC adjustable lands on the open rear edge.
            for (y=[-50,50]) translate([-56,y,p297_floor]) cube([22,40,5],center=true);
        }
        // Fuse/DC-DC dimensions vary: paired 18 mm slots are the FIT-CHECK gate.
        for (y=[-58,-42,42,58],x=[-61,-51])
            translate([x,y,p297_floor-1]) p297_slot(18,3.4,false,9);
    }
}

module p297_tray_rail(y,z) {
    // Horizontal rear-service runners.  Each carrier pulls toward -X without
    // disturbing the floors below or removing the control panel.
    translate([0,y,z]) union() {
        front_post_y = sign(y)*47.5;
        front_dogleg = front_post_y-y;
        difference() {
            cube([112,8,7],center=true);
            translate([0,-sign(y)*2,2.1]) cube([104,4.2,3.4],center=true);
        }
        // Short stop/bridge overlaps the rail and the X=66.4 front wall.
        // The rear remains open for independent tray withdrawal.
        translate([61,0,0]) cube([11,4,4],center=true);
        // Join that bridge to the nearest continuous Y=+/-47.5 front-wall
        // vertical, independent of the open X-brace bay at this rail's Y.
        translate([64,front_dogleg/2,0])
            cube([4,abs(front_dogleg)+4,4],center=true);
    }
}

module p297_logic_box() {
    difference() {
        union() {
            p297_open_box(p297_logic_h,true,true);
            // Three independent horizontal carriers, inserted from the open
            // rear.  Mega low, perfboard middle, Pi high; USB extension leads
            // rise through the common rear cable channel to the panel slot.
            for (y=[-58,58]) p297_tray_rail(y,10);
            for (y=[-63,63]) p297_tray_rail(y,36);
            for (y=[-32,32]) p297_tray_rail(y,62);
            // Continuous cable channel, aligned through all occupied floors.
            translate([-58,0,31]) cube([12,22,58],center=true);
        }
        translate([-60,0,32]) cube([10,14,62],center=true);
    }
}

module p297_board_carrier(size=[100,70],holes=[],cross_slots=false,label="") {
    difference() {
        union() {
            translate([0,0,1.6]) cube([size[0],size[1],3.2],center=true);
            // Rear pull tab remains reachable through the open -X face.
            translate([-size[0]/2-4,0,2.2]) cube([8,min(size[1]-12,38),4.4],center=true);
        }
        for (p=holes) translate([p[0],p[1],-1]) {
            if (cross_slots) {
                p297_slot(8,3.4,false,7);
                rotate([0,0,90]) p297_slot(8,3.4,false,7);
            } else cylinder(d=3.2,h=7,$fn=28);
        }
        // Ventilation/cable access strip.
        for (x=[-size[0]/2+14:16:size[0]/2-14])
            translate([x,0,-1]) p297_slot(min(24,size[1]-20),5,true,7);
    }
}

module p297_mega_carrier() {
    p297_board_carrier([66,116],p297_mega_holes,true,"MEGA");
}

module p297_perf_carrier() {
    p297_board_carrier([86,126],
        [[-37.5,-57.5],[37.5,-57.5],[-37.5,57.5],[37.5,57.5]],true,"PERF");
}

module p297_pi5_carrier() {
    p297_board_carrier([92,64],
        [[-29,-24.5],[29,-24.5],[-29,24.5],[29,24.5]],false,"PI5");
}

module p297_collar() {
    difference() {
        union() {
            p297_open_box(p297_collar_h,false,false,false,false);
            // Panel lands align to the four existing Ø8.6 panel holes.
            for (x=[-58,58],y=[-56,56]) translate([x,y,p297_collar_h-4])
                cylinder(d=16,h=4,$fn=36);
            // Each panel land has a continuous support-free path to the side
            // floor ring: a vertical column plus a floor-level radial spoke.
            // This replaces the former top crossbars, which were cut into
            // floating strips by the exact control-panel keepouts.
            for (x=[-58,58],y=[-56,56]) {
                translate([x,y,10.25]) cylinder(d=8,h=20.5,center=true,$fn=32);
                // Shift 7 mm outward in X to bypass the adjacent 12.6 mm
                // female key pocket while retaining 1 mm pillar overlap.
                translate([x+sign(x)*7,y+sign(y)*18.5,p297_floor/2])
                    cube([8,37,p297_floor],center=true);
            }
        }
        for (x=[-58,58],y=[-56,56]) translate([x,y,p297_collar_h-6])
            cylinder(d=m4_clear,h=8,$fn=32);
        // The existing panel has ribs down to local Z-20.  Collar material is
        // removed against the exact released panel solid, leaving the panel
        // lands in its existing holes but no scene-check overlap.
        for (dx=[-0.15,0.15],dy=[-0.15,0.15],dz=[-0.2,0.2])
            translate([dx,dy,p297_collar_h+dz]) control_panel_v2();
        // Explicit E-stop service keepout through the front crossbar.
        translate([0,-58,p297_collar_h-25]) cube([39,34,54],center=true);
    }
}

module p297_controls_ghosts() {
    color([.85,.08,.08,.55]) translate([0,-58,p297_panel_z-p297_collar_z-25])
        cube([35,30,50],center=true);
    color([.86,.08,.08,.80]) translate([0,-58,p297_panel_z-p297_collar_z+14.5])
        cylinder(d=38,h=21,center=true,$fn=48);
}

module p297_stack_only(ghosts=false) {
    translate([0,0,p297_base_z]) p297_base_adapter();
    translate([0,0,p297_energy_z]) p297_energy_spacer();
    translate([0,0,p297_power_z]) p297_power_box();
    translate([0,0,p297_logic_z]) p297_logic_box();
    translate([0,0,p297_collar_z]) p297_collar();
    translate([0,0,p297_panel_z]) control_panel_v2();
    if (ghosts) {
        // Four BTS 50 x 50 x 20 envelopes.
        for (x=[-30,30],y=[-30,30]) color([.75,.15,.10,.55])
            translate([x,y,p297_power_z+20]) cube([50,50,20],center=true);
        translate([0,0,p297_collar_z]) p297_controls_ghosts();
    }
}

module p297_assembly(ghosts=false) {
    // Reuse released P2-79/P2-90d base halves and existing corner ties.
    translate([0,0,55]) {
        base_half_v2(1);
        base_half_v2(-1);
    }
    for (side=[-1,1]) corner_tie_installed(side);
    translate([p297_cx,0,0]) p297_stack_only(ghosts);
}

// Print exports use a common local origin and their natural support-free pose.
if (part=="p297_base_adapter") p297_base_adapter();
else if (part=="p297_energy_spacer") p297_energy_spacer();
else if (part=="p297_power_box") p297_power_box();
else if (part=="p297_logic_box") p297_logic_box();
else if (part=="p297_collar") p297_collar();
else if (part=="p297_mega_carrier") p297_mega_carrier();
else if (part=="p297_perf_carrier") p297_perf_carrier();
else if (part=="p297_pi5_carrier") p297_pi5_carrier();
else if (part=="p297_assembly") p297_assembly(show_ghosts);

assert(p297_outer[0] <= 250 && p297_outer[1] <= 250,
       "P2-97 floor exceeds 250 x 250 mm");
assert(p297_panel_z + plate_t + 21 <= pod_absolute_top,
       "P2-97 E-stop exceeds rear-pod absolute Z envelope");
