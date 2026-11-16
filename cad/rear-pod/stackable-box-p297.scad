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
p297_corner = 12;
p297_corner_x = 44;
p297_corner_y = 82;
p297_lattice = 4.10;
p297_lattice_t = 3.2;
p297_extrusion_w = 0.62; // user's 0.6 mm nozzle profile
p297_key_h = 2.2;
p297_key_size = 10;
p297_key_clear = 0.30;
p297_seam_gap = 0.25;  // avoids coplanar CGAL false positives at lattice walls
p297_key_straight_h = p297_key_h+p297_seam_gap+1.5;
p297_seam_x = p297_corner_x;
p297_seam_y = p297_corner_y;
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
p297_bts_size = [50,50];
// Required physical measurement.  Keep this invalid in source so a power-box
// or assembly export cannot silently reuse the former nominal 20 mm ghost.
// Supply with -D 'p297_bts_measured_h=<underside-to-heatsink-top mm>'.
p297_bts_measured_h = -1;
p297_bts_design_h = p297_bts_measured_h > 0 ? p297_bts_measured_h : 20;
p297_bts_centres = [[-30,-40],[30,-40],[-30,40],[30,40]];
p297_bts_board_z = p297_floor + 8;
p297_bts_terminal_clear = 25;
p297_bts_side_inner_y = 93.4 - p297_lattice_t/2;
p297_bts_service_z0 = 6;
p297_bts_service_z1 = p297_bts_board_z + p297_bts_design_h + 5;
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
    for (x=[-p297_corner_x,p297_corner_x],
         y=[-p297_corner_y,p297_corner_y]) translate([x,y,0]) children();
}

module p297_seam_positions() {
    for (x=[-p297_seam_x,p297_seam_x],y=[-p297_seam_y,p297_seam_y])
        translate([x,y,0]) children();
}

// A horizontal beam whose lower face is a 45-degree V instead of a flat
// bridge.  The ridge is deliberately only 0.05 mm wide, so its tessellated
// downward patch remains far below the 40 mm2 export limit.
module p297_beam_x(length,depth,height) {
    eps = 0.05;
    union() {
        hull() {
            translate([0,0,-height/2]) cube([length,eps,eps],center=true);
            for (y=[-depth/2,depth/2])
                translate([0,y,-height/2+depth/2])
                    cube([length,eps,eps],center=true);
        }
        translate([0,0,depth/4])
            cube([length,depth,height-depth/2],center=true);
    }
}

module p297_top_keys(z) {
    p297_key_positions()
        translate([0,0,z]) cube([p297_key_size,p297_key_size,p297_key_h],center=true);
}

module p297_key_pocket_cavity(z0) {
    width = p297_key_size+2*p297_key_clear;
    straight_h = p297_key_straight_h;
    p297_key_positions() {
        translate([0,0,z0+straight_h/2])
            cube([width,width,straight_h+0.2],center=true);
        // The former flat pocket roof was a 122 mm2 unsupported ceiling.
        // Close it as a 45-degree pyramid above the required straight key bay.
        hull() {
            translate([0,0,z0+straight_h-0.05])
                cube([width,width,0.1],center=true);
            translate([0,0,z0+straight_h+(width-0.1)/2])
                cube([0.1,0.1,0.1],center=true);
        }
    }
}

module p297_corner_nodes(h) {
    p297_seam_positions()
        translate([0,0,h/2]) cube([p297_corner,p297_corner,h],center=true);
}

module p297_insert_pockets(h) {
    // Pocket opens through the 10 mm key.  At its narrowest the key leaves
    // (10-4)/2 = 3 mm radial wall; the 12 mm column leaves 4 mm.  The complete
    // 6.7 mm pocket depth remains surrounded by the full-height solid node.
    p297_seam_positions()
        translate([0,0,h+p297_key_h-p297_insert_depth])
            cylinder(d=p297_insert_hole,h=p297_insert_depth+0.3,$fn=28);
}

// Audit-only reference of the released pre-fix corner.  Coordinates are
// shifted from the former key centre [54,80]: the 6 mm post reaches 1 mm
// outside the 138 x 190 outline and the key/boss/post have separate axes.
module p297_corner_before_reference() {
    old_h = 24;
    union() {
        translate([7.5,8,1.6]) cube([15,16,p297_floor],center=true);
        translate([8,7.5,1.6]) cube([16,15,p297_floor],center=true);
        translate([13,13,old_h/2]) cube([6,6,old_h],center=true);
        translate([4,4,old_h/2]) cylinder(d=10,h=old_h,center=true,$fn=36);
        translate([0,0,old_h+p297_key_h/2])
            cube([12,12,p297_key_h],center=true);
    }
}

// Exact local section of the corrected keyed joint.  The front half is
// removed only for the documentation render, exposing the 6.7 mm insert bay
// and the 45-degree female-key roof in the box above.
module p297_corner_after_section() {
    lower_h = 18;
    upper_z = lower_h+p297_key_h+p297_seam_gap;
    upper_h = 18;
    pocket_w = p297_key_size+2*p297_key_clear;
    pocket_straight = p297_key_straight_h;
    difference() {
        union() {
            difference() {
                union() {
                    translate([0,0,lower_h/2])
                        cube([p297_corner,p297_corner,lower_h],center=true);
                    translate([0,0,lower_h+p297_key_h/2])
                        cube([p297_key_size,p297_key_size,p297_key_h],center=true);
                }
                translate([0,0,lower_h+p297_key_h-p297_insert_depth])
                    cylinder(d=p297_insert_hole,h=p297_insert_depth+0.3,$fn=28);
            }
            difference() {
                translate([0,0,upper_z+upper_h/2])
                    cube([p297_corner,p297_corner,upper_h],center=true);
                translate([0,0,upper_z-0.1+pocket_straight/2])
                    cube([pocket_w,pocket_w,pocket_straight+0.2],center=true);
                hull() {
                    translate([0,0,upper_z-0.15+pocket_straight])
                        cube([pocket_w,pocket_w,0.1],center=true);
                    translate([0,0,upper_z-0.1+pocket_straight+(pocket_w-0.1)/2])
                        cube([0.1,0.1,0.1],center=true);
                }
            }
        }
        translate([-20,-20,-1]) cube([40,20,50]);
    }
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
        p297_key_pocket_cavity(-p297_floor/2-0.1);
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
    union() {
        translate([0,y,0]) rotate([90,0,0]) linear_extrude(height=p297_lattice_t,center=true)
          union() {
            for (x=[-69,-34.5,0,34.5,69])
                translate([x,h/2]) square([p297_wall,h],center=true);
            for (x=[-51.75,-17.25,17.25,51.75])
                p297_x_lattice_xy(x-17.25,x+17.25,5,h-5);
          }
        translate([0,y,2]) p297_beam_x(138,p297_lattice_t,p297_wall);
        translate([0,y,h-2]) p297_beam_x(138,p297_lattice_t,p297_wall);
    }
}

module p297_front_wall(x,h) {
    // Y/Z sketch extruded through X.  Rear (-X) stays open for Phase 2.
    union() {
        translate([x,0,0]) rotate([90,0,90]) linear_extrude(height=p297_lattice_t,center=true)
          union() {
            for (y=[-95,-47.5,0,47.5,95])
                translate([y,h/2]) square([p297_wall,h],center=true);
            for (y=[-71.25,-23.75,23.75,71.25])
                p297_x_lattice_xy(y-23.75,y+23.75,5,h-5);
          }
        translate([x,0,2]) rotate([0,0,90])
            p297_beam_x(190,p297_lattice_t,p297_wall);
        translate([x,0,h-2]) rotate([0,0,90])
            p297_beam_x(190,p297_lattice_t,p297_wall);
    }
}

module p297_open_box(h,full_floor=true,front_lattice=true,top_keys=true,top_inserts=true) {
    difference() {
        union() {
            translate([0,0,p297_floor/2]) p297_floor_plate(full_floor);
            p297_side_wall(-93.4,h);
            p297_side_wall(93.4,h);
            if (front_lattice) p297_front_wall(66.4,h);
            p297_corner_nodes(h);
            if (top_keys) p297_top_keys(h+p297_key_h/2);
        }
        // Bottom seam fasteners remain reachable from above.
        p297_seam_positions()
            translate([0,0,-0.1]) cylinder(d=p297_m3_clear,h=p297_floor+1,$fn=28);
        // Cut the female key pockets through the complete box union as well
        // as the floor submodule; this also clears any coincident corner/grid
        // material added outside p297_floor_plate().
        p297_key_pocket_cavity(-0.1);
        if (top_inserts) p297_insert_pockets(h);
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
            p297_corner_nodes(p297_adapter_h);
            // Bed-supported links carry the inboard corner nodes into the
            // existing X=+/-64 adapter rails without a suspended shelf.
            for (x=[-54,54],y=[-p297_corner_y,p297_corner_y])
                translate([x,y,p297_adapter_h/2])
                    cube([20,p297_corner,p297_adapter_h],center=true);
            p297_top_keys(p297_adapter_h+p297_key_h/2);
        }
        // Existing rear and field-jigged front holes; no chassis drilling.
        for (x=[side_frame_rear_x-panel_center_x,
                side_frame_front_x-panel_center_x],y=[-54,54])
            translate([x,y,-1]) cylinder(d=m4_clear,h=p297_adapter_h+2,$fn=32);
        p297_insert_pockets(p297_adapter_h);
    }
}

module p297_energy_spacer() {
    // Phase 1 contains no Parkside geometry.  Rear is completely open and the
    // floor is only a perimeter ring, ready for measured Phase 2 drawers.
    p297_open_box(p297_energy_h,false,true);
}

module p297_bts_standoffs() {
    // The two rows move out to Y +/-40: the terminal face is at Y +/-65 and
    // retains 26.8 mm to the inner side-wall face.  Hole centres remain on
    // the 20 mm grid, so every boss still grows directly from a floor rib.
    for (c=p297_bts_centres) let(cx=c[0],cy=c[1])
        for (dx=[-20,20],dy=[-20,20])
            translate([cx+dx,cy+dy,p297_floor]) difference() {
                cylinder(d=9,h=8,$fn=32);
                translate([0,0,-1]) cylinder(d=4.2,h=10,$fn=28);
            }
}

module p297_power_service_openings() {
    // One screwdriver/cable window per board, through each +/-Y lattice.
    // The upper edge follows the measured heatsink envelope plus 5 mm.
    for (side=[-1,1],cx=[-30,30])
        translate([cx,side*93.4,(p297_bts_service_z0+p297_bts_service_z1)/2])
            cube([52,p297_lattice_t+4,
                  p297_bts_service_z1-p297_bts_service_z0],center=true);
}

module p297_power_service_frames() {
    // Portal frames restore the side-wall load path removed by the four
    // service windows.  X=0 lies in the 10 mm gap between the two boards.
    for (side=[-1,1]) {
        for (x=[-58,0,58])
            translate([x,side*93.4,
                       (p297_bts_service_z0+p297_bts_service_z1)/2])
                cube([4,p297_lattice_t,
                      p297_bts_service_z1-p297_bts_service_z0+8],center=true);
        for (z=[p297_bts_service_z0-2,p297_bts_service_z1+2])
            translate([0,side*93.4,z])
                p297_beam_x(120,p297_lattice_t,4);
    }
}

module p297_power_box() {
    union() {
        difference() {
            union() {
                p297_open_box(p297_power_h,true,true);
                p297_bts_standoffs();
                // Fuse and DC-DC adjustable lands on the open rear edge.
                // Start on the bed rather than at Z0.7 over the open grid.
                for (y=[-50,50]) translate([-56,y,2.5]) cube([22,40,5],center=true);
            }
            // Fuse/DC-DC dimensions vary: paired 18 mm slots are the FIT-CHECK gate.
            for (y=[-58,-42,42,58],x=[-61,-51])
                translate([x,y,p297_floor-1]) p297_slot(18,3.4,false,9);
            p297_power_service_openings();
        }
        p297_power_service_frames();
    }
}

module p297_bts_ghost(cx,cy) {
    // Local +Y is the terminal edge; local -Y is the signal-header edge.
    // Flipping the negative-Y row points all terminals outward and all signal
    // headers toward the central 30 mm harness aisle/rear cable channel.
    translate([cx,cy,p297_bts_board_z])
        rotate([0,0,cy > 0 ? 0 : 180]) {
            color([.75,.15,.10,.55])
                translate([0,0,p297_bts_design_h/2])
                    cube([p297_bts_size[0],p297_bts_size[1],
                          p297_bts_design_h],center=true);
            color([.10,.35,.85,.75])
                for (x=[-13,13]) translate([x,23,6]) cube([20,6,12],center=true);
            color([.10,.10,.10,.80])
                translate([0,-23,5]) cube([32,5,10],center=true);
            color([.10,.55,.85,.16])
                translate([0,25+p297_bts_terminal_clear/2,
                           p297_bts_design_h/2])
                    cube([50,p297_bts_terminal_clear,
                          p297_bts_design_h+10],center=true);
        }
}

module p297_power_fitcheck() {
    p297_power_box();
    for (c=p297_bts_centres) p297_bts_ghost(c[0],c[1]);
}

module p297_tray_rail(y,z) {
    // Horizontal rear-service runners.  Each carrier pulls toward -X without
    // disturbing the floors below or removing the control panel.
    translate([0,y,z]) union() {
        front_post_y = sign(y)*47.5;
        front_dogleg = front_post_y-y;
        difference() {
            p297_beam_x(112,8,7);
            translate([0,-sign(y)*2,2.1]) cube([104,4.2,3.4],center=true);
        }
        // Short stop/bridge overlaps the rail and the X=66.4 front wall.
        // The rear remains open for independent tray withdrawal.
        translate([61,0,0]) p297_beam_x(11,4,4);
        // Join that bridge to the nearest continuous Y=+/-47.5 front-wall
        // vertical, independent of the open X-brace bay at this rail's Y.
        translate([64,front_dogleg/2,0])
            rotate([0,0,90]) p297_beam_x(abs(front_dogleg)+4,4,4);
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
                // 45-degree D8 -> D16 transition removes the former flat
                // underside of each panel land at Z20.
                translate([x,y,16]) cylinder(d1=8,d2=16,h=4,$fn=36);
                // Dogleg inward to X=+/-50 before reaching the side ring.  It
                // bypasses the new full 12 mm corner node and its tapered key
                // pocket while retaining overlap with the D8 panel column.
                translate([x-sign(x)*4,y,p297_floor/2])
                    cube([12,8,p297_floor],center=true);
                translate([x-sign(x)*8,y+sign(y)*20,p297_floor/2])
                    cube([8,40,p297_floor],center=true);
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
        // Collar floor spokes are added outside p297_open_box(); reopen the
        // four female key bays through those spokes as well.
        p297_key_pocket_cavity(-0.1);
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
        // Four oriented BTS envelopes plus their 25 mm terminal keepouts.
        translate([0,0,p297_power_z])
            for (c=p297_bts_centres) p297_bts_ghost(c[0],c[1]);
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
else if (part=="p297_power_fitcheck") p297_power_fitcheck();
else if (part=="p297_corner_before") p297_corner_before_reference();
else if (part=="p297_corner_after_section") p297_corner_after_section();
else if (part=="p297_logic_box") p297_logic_box();
else if (part=="p297_collar") p297_collar();
else if (part=="p297_mega_carrier") p297_mega_carrier();
else if (part=="p297_perf_carrier") p297_perf_carrier();
else if (part=="p297_pi5_carrier") p297_pi5_carrier();
else if (part=="p297_assembly") p297_assembly(show_ghosts);

assert(p297_outer[0] <= 250 && p297_outer[1] <= 250,
       "P2-97 floor exceeds 250 x 250 mm");
assert(p297_corner_x+p297_corner/2 <= p297_outer[0]/2
       && p297_corner_y+p297_corner/2 <= p297_outer[1]/2,
       "P2-97 corner column exceeds the box outline");
assert((p297_corner-p297_insert_hole)/2 >= 2
       && (p297_key_size-p297_insert_hole)/2 >= 2,
       "P2-97 Ruthex pocket has less than 2 mm radial wall");
assert(p297_insert_depth >= 6.7,
       "P2-97 Ruthex pocket is shallower than 6.7 mm");
assert(p297_panel_z + plate_t + 21 <= pod_absolute_top,
       "P2-97 E-stop exceeds rear-pod absolute Z envelope");
assert(p297_bts_side_inner_y-(40+p297_bts_size[1]/2)
       >= p297_bts_terminal_clear,
       "P2-97 BTS terminal service clearance is below 25 mm");
assert(2*(40-p297_bts_size[1]/2) >= 25,
       "P2-97 BTS signal-header centre aisle is below 25 mm");
assert(p297_bts_service_z1+4 < p297_power_h,
       "P2-97 measured BTS height exceeds the power-box/service-window envelope");
assert((part!="p297_power_box" && part!="p297_power_fitcheck"
        && part!="p297_assembly")
       || p297_bts_measured_h > 0,
       "P2-97: supply measured BTS underside-to-heatsink-top height with -D p297_bts_measured_h=<mm>");
assert(min(p297_lattice,p297_lattice_t) >= 2*p297_extrusion_w,
       "P2-97 lattice is thinner than two 0.62 mm extrusion lines");
