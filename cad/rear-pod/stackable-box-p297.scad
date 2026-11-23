// P2-97 Phase 1 — FIT-CHECK stackable electronics box system.
// One tower, continuous three-sided shear seams and four lift-retention M3
// screws per seam into Ruthex RX-M3x5.7 inserts. Select with -D 'part="..."'.

use <rear-pod-v2.scad>
include <params-v2.scad>

part = "p297_assembly";
show_ghosts = true;

// Frozen external envelope: the panel is 138 x 190 and defines the tower.
p297_cx = panel_center_x;
p297_outer = [138,190];
p297_wall = 5;
p297_floor = 3.2;
p297_corner = 14;
p297_corner_x = 44;
p297_corner_y = 82;
p297_lattice = 7.2;
p297_lattice_t = 5.0;
p297_extrusion_w = 0.62; // user's 0.6 mm nozzle profile
p297_seam_lip_h = 8;
p297_seam_clearance = 0.35; // per side; override for the coupon/export
p297_seam_lip_t = 1.60;
p297_seam_band = p297_seam_lip_t+2*p297_seam_clearance+2*2.4;
// Centre the common tongue/groove so its outer socket wall is 2.4 mm while
// the complete band remains inside the frozen 138 x 190 mm envelope.
p297_seam_front_x = p297_outer[0]/2-p297_seam_band/2;
p297_seam_side_y = p297_outer[1]/2-p297_seam_band/2;
p297_seam_band_front_x = p297_seam_front_x;
p297_seam_band_side_y = p297_seam_side_y;
p297_seam_gap = 0.25;  // vertical assembly relief
p297_seam_straight_h = p297_seam_lip_h+0.15;
p297_seam_socket_h = p297_seam_straight_h
                         +(p297_seam_lip_t+2*p297_seam_clearance)/2+0.25;
p297_seam_x = p297_corner_x;
p297_seam_y = p297_corner_y;
p297_insert_hole = 4.0;
p297_insert_depth = 6.7;
p297_m3_clear = 3.4;
p297_base_rear_x = side_frame_rear_x-panel_center_x;
p297_base_front_x = side_frame_front_x-panel_center_x;
p297_base_mount_y = 54;
p297_base_m4_clear = 4.5;
p297_base_front_ear = [12,14];
p297_base_m4_access_d = 11; // Ø9 washer/head plus 1 mm radial tool clearance
p297_base_m4_head_h = 5.5;
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

// P2-99b: the released control panel and the P2-97 collar become one print.
// Keep the installed datum explicit; the export wrapper turns the released
// panel face onto Z=0, as required by the support-free print orientation.
p299_panel_collar_h = p297_collar_h + plate_t;
p299_c1_center = [0,110]; // user-confirmed CAD datum; 78 mm from START centre
p299_c1_base_z = 390;
p299_c1_scan_offset = 29.8;
p299_c1_size = [55.6,55.6,41.3];
p299_pedestal_h = p299_c1_base_z-(p297_panel_z+plate_t);
p299_c1_pattern = 43;
p299_c1_clear_d = 2.9;
p299_c1_head_d = 4.5;
p299_c1_plate_t = 4;
p299_c1_plate_size = [62,73];
p299_c1_plate_center = [0,102.5];
p299_c1_cheek_t = 4;
p299_c1_cheek_inner_x = 27;
p299_c1_cable_slot = [17,26.2];
p299_panel_mounts = [[-18,72],[18,72],[-18,90],[22,88]];
p299_xh_opening = [17,10]; // user housing 13 x 10 x 4 mm passes this released opening
p299_xh_opening_center = [0,82];

// Physical board patterns.  Mega/perf/BTS values are measured parts recorded
// in motion-electronics-tray/params.scad.  Pi 5 is Raspberry Pi drawing
// RP-008347-DS-1 (85 x 56 board; 58 x 49 hole centres).
p297_bts_pattern = [40,40];
p297_bts_size = [50,50];
// User measurement 2026-11-16: 20 mm heatsink below a ~1 mm PCB and 15 mm
// components above it, 36 mm total. Keep total invalid in source so a power
// export must explicitly acknowledge the measured assembly.
p297_bts_measured_h = -1;
p297_bts_heatsink_below = 20;
p297_bts_pcb_t = 1;
p297_bts_above = 15;
p297_bts_heatsink_size = [50,30]; // fins and 50 mm length run along local X
p297_bts_centres = [[-30,-40],[30,-40],[-30,40],[30,40]];
p297_bts_standoff_h = 22;
p297_bts_board_z = p297_floor + p297_bts_standoff_h; // PCB underside
p297_bts_air_gap = p297_bts_board_z-p297_bts_heatsink_below-p297_floor;
p297_bts_terminal_clear = 25;
p297_bts_side_inner_y = 92.5 - p297_lattice_t/2;
p297_bts_service_z0 = 11; // stays above the continuous 9.45 mm socket band
p297_bts_service_z1 = p297_bts_board_z+p297_bts_pcb_t+p297_bts_above+5;
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

module p297_seam_positions() {
    for (x=[-p297_seam_x,p297_seam_x],y=[-p297_seam_y,p297_seam_y])
        translate([x,y,0]) children();
}

module p297_base_mount_positions() {
    for (x=[p297_base_rear_x,p297_base_front_x],
         y=[-p297_base_mount_y,p297_base_mount_y])
        translate([x,y,0]) children();
}

module p297_base_front_mount_positions() {
    for (y=[-p297_base_mount_y,p297_base_mount_y])
        translate([p297_base_front_x,y,0]) children();
}

// A horizontal beam whose lower face is an exact 45-degree V instead of a
// flat bridge.  A true triangular-prism ridge avoids the former 0.05 mm
// tessellation ribs, which appeared as sub-line-width material in sections.
module p297_beam_segment_x(length,depth,height) {
    ridge = 1.6;
    section = [[-ridge/2,-height/2],
               [ridge/2,-height/2],
               [depth/2,-height/2+(depth-ridge)/2],
               [depth/2,height/2],
               [-depth/2,height/2],
               [-depth/2,-height/2+(depth-ridge)/2]];
    points = [for (x=[-length/2,length/2], yz=section) [x,yz[0],yz[1]]];
    polyhedron(points=points,
        faces=[[0,1,2,3,4,5],[11,10,9,8,7,6],
               [6,7,1,0],[7,8,2,1],[8,9,3,2],
               [9,10,4,3],[10,11,5,4],[11,6,0,5]],convexity=2);
}

module p297_beam_x(length,depth,height) {
    // One continuous member.  Its 45-degree transverse underside is printable
    // and the vertical lattice/post intersections divide the 1.6 mm centre
    // ridge into real bridges with material at both ends.  Never introduce
    // artificial gaps merely to reduce the downward-face area.
    p297_beam_segment_x(length,depth,height);
}

// Three closed sides: both Y walls and the +X/front wall. The -X/rear side
// remains open for the Phase-2 drawers and the common cable/service route.
module p297_seam_path_2d(width) {
    union() {
        // Side legs terminate on the front-leg centreline.  Extending both
        // rectangles through one another made four socket quadrants at each
        // bend, including a detached 2.4 x 2.4 outer island.
        side_length = p297_seam_front_x+p297_outer[0]/2;
        side_centre_x = (p297_seam_front_x-p297_outer[0]/2)/2;
        for (y=[-p297_seam_side_y,p297_seam_side_y])
            translate([side_centre_x,y])
                square([side_length,width],center=true);
        translate([p297_seam_front_x,0])
            square([width,2*p297_seam_side_y+width],center=true);
    }
}

module p297_seam_band_path_2d(width) {
    union() {
        side_length = p297_seam_band_front_x+p297_outer[0]/2;
        side_centre_x = (p297_seam_band_front_x-p297_outer[0]/2)/2;
        for (y=[-p297_seam_band_side_y,p297_seam_band_side_y])
            translate([side_centre_x,y])
                square([side_length,width],center=true);
        translate([p297_seam_band_front_x,0])
            square([width,p297_outer[1]],center=true);
    }
}

module p297_top_seam_lip(z) {
    translate([0,0,z]) linear_extrude(height=p297_seam_lip_h)
        p297_seam_path_2d(p297_seam_lip_t);
}

module p297_socket_band() {
    union() {
        linear_extrude(height=p297_seam_socket_h)
            p297_seam_band_path_2d(p297_seam_band);
        // Solid outer elbows keep the two 2.4 mm socket walls joined through
        // the 90-degree bend; the raw rectangle union narrows to a sub-line
        // crescent while the self-supporting roof closes.
        for (side=[-1,1])
            translate([66.5,side*93.8,p297_seam_socket_h/2])
                cube([5,2.4,p297_seam_socket_h],center=true);
    }
}

module p297_socket_cavity(z0=-0.1) {
    groove = p297_seam_lip_t+2*p297_seam_clearance;
    straight = p297_seam_straight_h;
    side_length = p297_seam_front_x+p297_outer[0]/2;
    side_centre_x = (p297_seam_front_x-p297_outer[0]/2)/2;
    // Straight receiving bay.
    translate([0,0,z0]) linear_extrude(height=straight+0.01)
        p297_seam_path_2d(groove);
    // 45-degree self-supporting roofs; no horizontal bridge is introduced.
    for (y=[-p297_seam_side_y,p297_seam_side_y]) hull() {
        translate([side_centre_x,y,z0+straight+0.005])
            cube([side_length,groove,0.01],center=true);
        translate([side_centre_x,y,z0+straight+groove/2])
            cube([side_length,0.01,0.01],center=true);
    }
    hull() {
        translate([p297_seam_front_x,0,z0+straight+0.005])
            cube([groove,2*p297_seam_side_y+groove,0.01],center=true);
        translate([p297_seam_front_x,0,z0+straight+groove/2])
            cube([0.01,2*p297_seam_side_y+groove,0.01],center=true);
    }
}

module p297_corner_nodes(h) {
    p297_seam_positions()
        translate([0,0,h/2]) cube([p297_corner,p297_corner,h],center=true);
}

module p297_insert_pockets(h) {
    // These screws only retain a floor during lifting; the perimeter seam
    // carries shear. The 14 mm node leaves 5 mm radial material around Ø4.
    p297_seam_positions()
        translate([0,0,h-p297_insert_depth])
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
        translate([0,0,old_h+1.1]) cube([12,12,2.2],center=true);
    }
}

// Exact local section of the continuous perimeter joint. The front half is
// removed only for the documentation render to expose the female roof.
module p297_seam_coupon_lower() {
    translate([-54,-80,0]) intersection() {
        union() {
            linear_extrude(height=4) p297_seam_band_path_2d(p297_seam_band);
            p297_top_seam_lip(4);
        }
        translate([54,80,7]) cube([30,30,14],center=true);
    }
}

module p297_seam_coupon_upper() {
    translate([-54,-80,0]) intersection() {
        difference() {
            p297_socket_band();
            p297_socket_cavity();
        }
        translate([54,80,p297_seam_socket_h/2])
            cube([30,30,p297_seam_socket_h+0.2],center=true);
    }
}

module p297_corner_after_section() {
    difference() {
        union() {
            p297_seam_coupon_lower();
            translate([0,0,4+p297_seam_gap]) p297_seam_coupon_upper();
        }
        translate([-20,-20,-1]) cube([40,20,30]);
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
            p297_seam_positions() cylinder(d=14,h=p297_floor,center=true,$fn=36);
        }
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
                p297_x_lattice_xy(x-17.25,x+17.25,
                                  p297_wall-1,h-(p297_wall-1));
          }
        translate([0,y,p297_wall/2])
            p297_beam_x(138,p297_lattice_t,p297_wall);
        // The 2.4 mm outer socket wall moves the tongue inward.  Widen and
        // shift only the self-supporting top beam so the tongue has a 45 deg
        // backed underside rather than a horizontal overhang.
        translate([0,sign(y)*92.5,h-p297_wall/2])
            p297_beam_x(138,5,p297_wall);
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
                p297_x_lattice_xy(y-23.75,y+23.75,
                                  p297_wall-1,h-(p297_wall-1));
          }
        translate([x,0,p297_wall/2]) rotate([0,0,90])
            p297_beam_x(190,p297_lattice_t,p297_wall);
        translate([66.4,0,h-p297_wall/2]) rotate([0,0,90])
            p297_beam_x(190,5,p297_wall);
    }
}

module p297_solid_side_walls(h,t=2.4) {
    for (y=[-p297_outer[1]/2+t/2,p297_outer[1]/2-t/2])
        translate([0,y,h/2]) cube([p297_outer[0],t,h],center=true);
}

module p297_open_box(h,full_floor=true,front_lattice=true,top_keys=true,
                     top_inserts=true,side_lattice=true,corner_nodes=true) {
    difference() {
        union() {
            translate([0,0,p297_floor/2]) p297_floor_plate(full_floor);
            if (side_lattice) {
                p297_side_wall(-92.5,h);
                p297_side_wall(92.5,h);
            }
            else p297_solid_side_walls(h);
            if (front_lattice) p297_front_wall(66.4,h);
            if (front_lattice && side_lattice)
                for (y=[-92.5,92.5])
                    translate([66.4,y,h/2]) cube([5,5,h],center=true);
            p297_socket_band();
            if (corner_nodes) p297_corner_nodes(h);
            if (top_keys) p297_top_seam_lip(h);
        }
        // Bottom seam fasteners remain reachable from above.
        p297_seam_positions()
            translate([0,0,-0.1]) cylinder(d=p297_m3_clear,h=p297_floor+1,$fn=28);
        p297_socket_cavity();
        if (top_inserts) p297_insert_pockets(h);
    }
}

module p297_base_adapter() {
    assert((p297_base_front_ear[0]-p297_base_m4_clear)/2 >= 3,
        "P2-97 front M4 ears need >=3 mm material in X");
    assert((p297_base_front_ear[1]-p297_base_m4_clear)/2 >= 3,
        "P2-97 front M4 ears need >=3 mm material in Y");
    difference() {
        union() {
            // Two light rails join the exact P2-90d base axes.
            for (y=[-54,54]) translate([0,y,p297_adapter_h/2])
                cube([133,14,p297_adapter_h],center=true);
            for (x=[-64,64]) translate([x,0,p297_adapter_h/2])
                cube([10,178,p297_adapter_h],center=true);
            // The adapter has no box wall below it, so this bed-supported
            // perimeter band carries the complete three-sided tongue.
            linear_extrude(height=p297_adapter_h)
                p297_seam_band_path_2d(p297_seam_band);
            p297_corner_nodes(p297_adapter_h);
            // Bed-supported links carry the inboard corner nodes into the
            // existing X=+/-64 adapter rails without a suspended shelf.
            for (x=[-54,54],y=[-p297_corner_y,p297_corner_y])
                translate([x,y,p297_adapter_h/2])
                    cube([20,p297_corner,p297_adapter_h],center=true);
            // Local front ears move the outer edge 3.75 mm beyond each Ø4.5
            // bore.  They are bed-supported and merge with both the Y rail
            // and the three-sided seam band.
            p297_base_front_mount_positions()
                translate([0,0,p297_adapter_h/2])
                    cube([p297_base_front_ear[0],p297_base_front_ear[1],
                          p297_adapter_h],center=true);
            p297_top_seam_lip(p297_adapter_h);
        }
        // Existing rear and field-jigged front holes; no chassis drilling.
        // The bore crosses the complete 8 mm adapter and its 8 mm tongue.
        p297_base_mount_positions()
            translate([0,0,-1])
                cylinder(d=p297_base_m4_clear,
                         h=p297_adapter_h+p297_seam_lip_h+2,$fn=32);
        // The front tongue alone is opened to Ø11.  A Ø9 washer/head and its
        // driver therefore approach vertically while the full tongue remains
        // continuous on both sides of the local notch.
        p297_base_front_mount_positions() {
            translate([0,0,p297_adapter_h])
                cylinder(d=p297_base_m4_access_d,
                         h=p297_seam_lip_h+1,$fn=40);
            // A round Ø11 access cut only grazes the 1.6 mm tongue at each
            // side, producing long 0.08..0.15 mm crescents.  Open a clean
            // rectangular service break around the same washer instead.
            translate([0,0,p297_adapter_h+p297_seam_lip_h/2])
                cube([p297_seam_band+2,12,p297_seam_lip_h],center=true);
        }
        p297_insert_pockets(p297_adapter_h);
    }
}

module p297_base_front_head_reliefs() {
    // The energy spacer receives the installed M4 head/Ø9 washer.  Use a
    // clean, open service notch from the inner floor edge through the front
    // band: the former tangent/conical pocket left sub-line-width crescents
    // at every intermediate section.
    p297_base_front_mount_positions() {
        translate([-1.5,0,p297_seam_socket_h/2-0.5])
            cube([8.2,p297_base_m4_access_d,p297_seam_socket_h+1],center=true);
    }
}

module p297_energy_spacer() {
    // Phase 1 contains no Parkside geometry.  Rear is completely open and the
    // floor is only a perimeter ring, ready for measured Phase 2 drawers.
    difference() {
        p297_open_box(p297_energy_h,false,true);
        p297_base_front_head_reliefs();
    }
}

module p297_bts_standoffs() {
    // The two rows move out to Y +/-40: the terminal face is at Y +/-65 and
    // retains 26.8 mm to the inner side-wall face. The 40 x 40 hole centres
    // lie 5 mm outside the 30 mm-wide heatsink, so Ø8 bosses and M3 screws do
    // not enter its envelope. Every boss grows from a 20 mm floor-grid rib.
    for (c=p297_bts_centres) let(cx=c[0],cy=c[1])
        for (dx=[-20,20],dy=[-20,20])
            translate([cx+dx,cy+dy,p297_floor]) difference() {
                cylinder(d=8,h=p297_bts_standoff_h,$fn=32);
                translate([0,0,-1])
                    cylinder(d=p297_m3_clear,h=p297_bts_standoff_h+2,$fn=28);
            }
}

module p297_bts_fastener_holes() {
    // True through-holes: M3x30 installs from the PCB side and receives a
    // washer/nyloc below the floor before this box is stacked.
    for (c=p297_bts_centres) let(cx=c[0],cy=c[1])
        for (dx=[-20,20],dy=[-20,20])
            translate([cx+dx,cy+dy,-1])
                cylinder(d=p297_m3_clear,h=p297_bts_board_z+3,$fn=28);
}

module p297_bts_air_channels() {
    // Six continuous X slots run below the heatsink fins of both board rows
    // and open through the -X/rear floor rim. They avoid all 16 standoff feet.
    for (cy=[-40,40],dy=[-10,0,10])
        translate([-8,cy+dy,-1])
            p297_slot(136,4.8,false,p297_floor+3);
}

module p297_power_service_openings() {
    // One screwdriver/cable window per board, through each +/-Y lattice.
    // The upper edge follows the measured 15 mm top-component envelope +5.
    opening_z1 = p297_bts_service_z1+4;
    for (side=[-1,1],cx=[-30,30])
        translate([cx,side*93.4,(p297_bts_service_z0+opening_z1)/2])
            cube([52,p297_lattice_t+4,
                  opening_z1-p297_bts_service_z0],center=true);
}

module p297_power_service_frames() {
    // Portal frames restore the side-wall load path removed by the four
    // service windows.  X=0 lies in the 10 mm gap between the two boards.
    frame_z0 = max(p297_bts_service_z0-4,p297_seam_socket_h+0.5);
    frame_z1 = p297_bts_service_z1+4;
    for (side=[-1,1]) {
        for (x=[-58.5,0,58.5])
            translate([x,side*93.4,(frame_z0+frame_z1)/2])
                // 8 mm outer and 9 mm centre posts leave exactly 50 mm
                // between supports for each continuous portal beam.
                cube([x == 0 ? 9 : 8,p297_lattice_t,
                      frame_z1-frame_z0],center=true);
        // Keep the lower portal beam above the 8 mm male lip. This frame is
        // added after the box boolean, so it must not refill the socket bay.
        for (z=[max(p297_bts_service_z0-2,p297_seam_socket_h+2),
                p297_bts_service_z1+2])
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
            }
            p297_bts_air_channels();
            p297_bts_fastener_holes();
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
            // Measured central heatsink below the PCB. Its fins run in X,
            // aligned with the continuous floor-air slots and open rear.
            color([.45,.45,.45,.75])
                translate([0,0,-p297_bts_heatsink_below/2])
                    cube([p297_bts_heatsink_size[0],
                          p297_bts_heatsink_size[1],
                          p297_bts_heatsink_below],center=true);
            color([.75,.15,.10,.55])
                translate([0,0,p297_bts_pcb_t/2])
                    cube([p297_bts_size[0],p297_bts_size[1],
                          p297_bts_pcb_t],center=true);
            color([.10,.35,.85,.75])
                for (x=[-13,13])
                    translate([x,23,p297_bts_pcb_t+6])
                        cube([20,6,12],center=true);
            color([.10,.10,.10,.80])
                translate([0,-23,p297_bts_pcb_t+5])
                    cube([32,5,10],center=true);
            color([.15,.15,.15,.8])
                translate([0,0,p297_bts_pcb_t])
                    cylinder(d=10,h=p297_bts_above,$fn=32);
            color([.10,.55,.85,.16])
                translate([0,25+p297_bts_terminal_clear/2,
                           (p297_bts_pcb_t+p297_bts_above)/2])
                    cube([50,p297_bts_terminal_clear,
                          p297_bts_pcb_t+p297_bts_above+4],center=true);
        }
}

module p297_power_fitcheck() {
    p297_power_box();
    for (c=p297_bts_centres) p297_bts_ghost(c[0],c[1]);
}

module p297_tray_rail(y,z,floor_supported=false) {
    // Horizontal rear-service runners.  Each carrier pulls toward -X without
    // disturbing the floors below or removing the control panel.
    translate([0,y,z]) union() {
        front_post_y = sign(y)*47.5;
        front_dogleg = front_post_y-y;
        difference() {
            union() {
                p297_beam_x(100,8,7);
                // The outer top spine carries the tray and stiffens the rail.
                translate([0,sign(y)*3,3]) cube([100,2,1],center=true);
            }
            translate([0,-sign(y)*2,2.1]) cube([92,4.2,3.4],center=true);
        }
        // One straight bridge runs from the supported X=50 rail end to the
        // continuous X=66.4, Y=+/-47.5 front-wall post.  The former two-leg L
        // had a combined lower face with one unsupported end.
        front_dx = 16.4;
        hull() {
            translate([50,0,0]) cube([5,5,4],center=true);
            translate([66.4,front_dogleg,0]) cube([5,5,4],center=true);
        }
        hull() {
            // 5 x 8 keeps the only horizontal starter face exactly at the
            // 40 mm2 limit while burying the diagonal bridge transition.
            translate([66.4,front_dogleg,-4]) cube([5,8,1],center=true);
            translate([66.4,front_dogleg,0]) cube([9,13,1],center=true);
        }
    }
    // Every ridge span is a real bridge no longer than 20 mm.  The low
    // Mega rail can rise directly from the floor.  Higher rails remain clear
    // of the removable carriers below: 45-degree braces approach from the
    // outside of the carrier envelope and root in continuous side-wall posts.
    rail_bottom = z-3.5;
    if (floor_supported)
        for (x=p297_grid_x)
            translate([x,y,rail_bottom/2])
                // 5 x 5 is the permanent slice-gate minimum island area and
                // lands fully on the 8 mm floor-grid rib below.
                cube([5,5,rail_bottom],center=true);
    else {
        // Each outer wall post carries one solid triangular web to two rail
        // stations.  Separate braces formed a thin Y-shaped split where they
        // diverged.  The centre post carries one straight web.
        groups = [[-34.5,[-50,-34.5]], [0,[0]], [34.5,[34.5,50]]];
        for (group=groups) {
            wall_x = group[0];
            rail_xs = group[1];
            side_y = sign(y)*91.3;
            max_run = max([for (x=rail_xs)
                sqrt((side_y-y)*(side_y-y)+(wall_x-x)*(wall_x-x))]);
            root_z = rail_bottom+2.3-max_run;
            hull() {
                // A 5 x 5 square section keeps every oblique slice at or above
                // 25 mm2.  Sink it 0.2 mm into the rail for finite overlap.
                translate([wall_x,side_y,root_z])
                    cube([5,5,5],center=true);
                for (x=rail_xs)
                    translate([x,y,rail_bottom+2.3])
                        cube([5,5,5],center=true);
            }
            // Pads bury the oblique transition inside existing material; the
            // brace leaves each junction at its full 5 x 5 section.
            translate([wall_x,side_y,root_z]) cube([9,9,5],center=true);
            // Both upper brace levels root near the bed.  This short pier
            // covers the triangular web's initial taper until it has fully
            // separated from the wall face.
            translate([wall_x,side_y,7.5]) cube([9,9,15],center=true);
            for (x=rail_xs)
                translate([x,y,rail_bottom+2.3]) cube([7,7,5],center=true);
        }
    }
}

module p297_logic_box() {
    difference() {
        union() {
            p297_open_box(p297_logic_h,true,true);
            // Three independent horizontal carriers, inserted from the open
            // rear.  Mega low, perfboard middle, Pi high; USB extension leads
            // rise through the common rear cable channel to the panel slot.
            for (y=[-58,58]) p297_tray_rail(y,10,true);
            // Raised only enough for the outer 3-D braces to retain a
            // >=45-degree underside from the existing X=+/-34.5 wall posts.
            for (y=[-63,63]) p297_tray_rail(y,38);
            for (y=[-32,32]) p297_tray_rail(y,66);
            // Continuous cable channel, aligned through all occupied floors.
            translate([-58,0,31]) cube([12,22,58],center=true);
        }
        translate([-60,0,32]) cube([10,14,62],center=true);
        // Rails are added outside p297_open_box(), so reopen the female seam
        // after them as well.  This preserves the nominal 0.35 mm lateral and
        // 0.25 mm vertical clearance to the power-box male tongue.
        p297_socket_cavity();
        // The released panel's X=-69..-49 deep rear beam passes through the
        // collar.  Break both side tongues completely across this 20.4 mm
        // interval; retaining only their outer edge would create a constant
        // 1.05 mm wall.  The notch leaves 0.2 mm X/Y clearance to the beam.
        for (side=[-1,1])
            translate([-59,side*p297_seam_side_y,
                       p297_logic_h+p297_seam_lip_h/2+0.2])
                cube([20.4,p297_seam_band,p297_seam_lip_h+0.4],center=true);
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
    // X=+/-37.5 cross-slots extend to +/-41.5.  A 86 mm plate left only
    // 1.50 mm to its edge through the complete 3.2 mm thickness.
    p297_board_carrier([86.4,126.4],
        [[-37.5,-57.5],[37.5,-57.5],[-37.5,57.5],[37.5,57.5]],true,"PERF");
}

module p297_pi5_carrier() {
    p297_board_carrier([92,64],
        [[-29,-24.5],[29,-24.5],[-29,24.5],[29,24.5]],false,"PI5");
}

module p297_collar() {
    assert(p297_insert_depth >= 5.7+1,
        "RX-M3x5.7 collar pocket needs at least 1 mm bottom allowance");
    assert((8-p297_insert_hole)/2 >= 1.6,
        "collar insert column must retain at least 1.6 mm radial wall");
    difference() {
        union() {
            // At only 24 mm high the lattice produced dozens of tiny slicer
            // islands.  Continuous 2.4 mm side walls give the collar a
            // four-line PETG shell and retain the common socket below.
            p297_open_box(p297_collar_h,false,false,false,false,false,false);
            // Panel lands align to the four existing Ø8.6 panel holes.
            // Their Ø16 OD leaves 6 mm radial PETG around the Ø4.0
            // RX-M3x5.7 heat-set pocket (minimum required: 1.6 mm).
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
                    cube([10,40,p297_floor],center=true);
            }
        }
        // Ruthex RX-M3x5.7: Ø4.0 recommended printed bore, 5.7 mm insert.
        // A 6.7 mm blind pocket provides the requested 1 mm bottom allowance;
        // it opens only at the panel face and leaves solid material below.
        for (x=[-58,58],y=[-56,56])
            translate([x,y,p297_collar_h-p297_insert_depth])
                cylinder(d=p297_insert_hole,h=p297_insert_depth+0.1,$fn=32);
        // The existing panel has ribs down to local Z-20.  Collar material is
        // removed against the exact released panel solid, leaving the panel
        // lands in its existing holes but no scene-check overlap.
        for (dx=[-0.15,0.15],dy=[-0.15,0.15],dz=[-0.2,0.2])
            translate([dx,dy,p297_collar_h+dz]) control_panel_v2();
        // The released panel's asymmetric lower rib only grazes this dogleg,
        // otherwise leaving a ~0.004 mm2 detached triangular remnant through
        // Z3.8..15.8.  Clear the tangency instead of asking the slicer to emit
        // an unprintable one-extrusion island.
        translate([-51,75,10]) cube([0.6,0.6,14],center=true);
        // Two more panel-rib tangencies occur only while the socket roof is
        // closing.  A deliberate short service notch avoids 0.02..0.22 mm
        // material webs at Z9.0/9.2.
        for (side=[-1,1])
            translate([-59,side*91.3,9.1]) cube([22,1.2,1.4],center=true);
        // At the panel interface four released-panel corner cutouts otherwise
        // leave 5.4 mm2 tips at the ends of the solid side walls.  Terminate
        // those tips cleanly during the final 0.8 mm instead.
        for (x=[-67,67],y=[-94,94])
            translate([x,y,p297_collar_h-0.4]) cube([6,4,0.8],center=true);
        // Explicit E-stop service keepout through the front crossbar.
        translate([0,-58,p297_collar_h-25]) cube([39,34,54],center=true);
        // Collar floor spokes are added outside p297_open_box(); reopen the
        // continuous female seam through those spokes as well.
        p297_socket_cavity();
    }
}

// P2-99b step 1.  This collar core deliberately does not subtract an expanded
// control_panel_v2() clearance: the released panel and all of its ribs are now
// part of the same manifold body. The engaged socket/cavity dimensions and
// E-stop service volume remain the P2-97 geometry; the obsolete panel lands,
// insert pockets and non-seam floor ring are intentionally absent.
module p299_collar_core() {
    difference() {
        union() {
            // Preserve the engaged three-sided female seam exactly, but omit
            // the old four-sided horizontal floor ring: face-down it became a
            // 2308 mm2 suspended ledge and it has no role in tongue capture.
            p297_socket_band();
            p297_solid_side_walls(p297_collar_h);
            translate([p297_outer[0]/2-1.2,0,p297_collar_h/2])
                cube([2.4,p297_outer[1],p297_collar_h],center=true);

            // In the required face-down print, these 45-degree transitions
            // grow outwards from the collar walls to the complete socket band.
            // They sit strictly above the engaged cavity, so 0.35 mm radial
            // and 0.25 mm vertical seam clearances remain unchanged.
            ramp_h = max((p297_outer[1]/2-2.4)
                         -(p297_seam_side_y-p297_seam_band/2),
                         (p297_outer[0]/2-2.4)
                         -(p297_seam_front_x-p297_seam_band/2));
            for (side=[-1,1]) hull() {
                translate([(p297_seam_front_x-p297_outer[0]/2)/2,
                           side*p297_seam_side_y,
                           p297_seam_socket_h])
                    cube([p297_seam_front_x+p297_outer[0]/2,
                          p297_seam_band,0.02],center=true);
                translate([0,side*(p297_outer[1]/2-1.2),
                           p297_seam_socket_h+ramp_h])
                    cube([p297_outer[0],2.4,0.02],center=true);
            }
            hull() {
                translate([p297_seam_front_x,0,p297_seam_socket_h])
                    cube([p297_seam_band,p297_outer[1],0.02],center=true);
                translate([p297_outer[0]/2-1.2,0,
                           p297_seam_socket_h+ramp_h])
                    cube([2.4,p297_outer[1],0.02],center=true);
            }
        }
        translate([0,-58,p297_collar_h-25]) cube([39,34,54],center=true);
        p297_socket_cavity();
        // Open the two female-groove entrances cleanly through the rear edge.
        // The released P2-97 cavity ended exactly on X=-69; after reversing
        // the whole body for face-down printing that coincident contour was
        // slicer-dependent.  Groove width and the 0.35 mm radial clearance
        // are unchanged everywhere in the engaged seam.
        for (side=[-1,1])
            translate([-68.9,side*p297_seam_side_y,
                       p297_seam_straight_h/2])
                cube([0.4,p297_seam_lip_t+2*p297_seam_clearance,
                      p297_seam_straight_h+0.2],center=true);
        // Retain the released P2-97 closure/tangency clean-ups.  They act in
        // collar material only and therefore cannot alter a panel hole/seat.
        translate([-51,75,10]) cube([0.6,0.6,14],center=true);
        for (side=[-1,1])
            translate([-59,side*91.3,9.1]) cube([22,1.2,1.4],center=true);
        for (x=[-67,67],y=[-94,94])
            translate([x,y,p297_collar_h-0.4]) cube([6,4,0.8],center=true);
    }
}

module p299_panel_collar() {
    difference() {
        union() {
            p299_collar_core();
            // Do not clone or reinterpret any panel feature: reuse the released
            // solid so every control hole, counter-volume, seat and L-beam remains
            // at exactly its released coordinate and diameter.
            translate([0,0,p297_collar_h]) control_panel_v2();
            // Blind Ruthex bosses open on the control face. The deliberately
            // asymmetric fourth point is the pedestal's position key.
            for (p=p299_panel_mounts)
                translate([p[0],p[1],p299_panel_collar_h-7])
                    cylinder(d=12,h=7,$fn=48);
        }
        for (p=p299_panel_mounts)
            translate([p[0],p[1],p299_panel_collar_h-6.7])
                cylinder(d=p297_insert_hole,h=6.8,$fn=40);
        // Captive XH lead: provisional 17 x 10 envelope. The actual housing
        // and overmould must be measured before printing.
        translate([p299_xh_opening_center[0],p299_xh_opening_center[1],
                   p297_collar_h-0.1])
            linear_extrude(height=plate_t+0.2)
                offset(r=2) square([p299_xh_opening[0]-4,
                                    p299_xh_opening[1]-4],center=true);
    }
}

module p299_c1_pedestal_installed() {
    assert(p299_pedestal_h > 10,"P2-99b pedestal height must be positive");
    assert(abs(p299_c1_base_z+p299_c1_scan_offset-419.8) < 0.001,
           "P2-99b nominal scan plane changed");
    assert(p299_c1_plate_t == 4,
           "P2-99f M2.5x8 requires exactly 4 mm plate thickness");
    assert(p299_c1_plate_size[0] >= p299_c1_size[0]
           && p299_c1_plate_size[1] >= p299_c1_size[1],
           "P2-99f plate must cover the complete C1 footprint");
    assert(p299_c1_cheek_inner_x-p299_c1_pattern/2 >= 5,
           "P2-99f cheek inner face must be >=5 mm from every C1 hole axis");
    difference() {
        union() {
            // P2-99f keeps the complete lower interface unchanged: 58 x 58 x 5,
            // the four asymmetric M3 axes and the existing XH passage below.
            translate([0,95,2.5]) cube([58,58,5],center=true);
            // Continuous cheeks: the inner faces are X=±27, i.e. 5.5 mm from
            // each X=±21.5 hole axis, throughout the complete tool-access
            // height. Their outer faces X=±31 are fully carried by the plate.
            for (side=[-1,1])
                translate([side*(p299_c1_cheek_inner_x+p299_c1_cheek_t/2),
                           102.5,(4.6+p299_pedestal_h-p299_c1_plate_t)/2])
                    cube([p299_c1_cheek_t,73,
                          p299_pedestal_h-p299_c1_plate_t-4.6],center=true);
            // The failed four pads become one 4.0 mm plate. 62 x 73 covers
            // the whole 55.6 x 55.6 C1 footprint, both widened cheeks and
            // both service webs, so every vertical wall starts on the bed.
            translate([p299_c1_plate_center[0],p299_c1_plate_center[1],
                       p299_pedestal_h-p299_c1_plate_t/2])
                cube([p299_c1_plate_size[0],p299_c1_plate_size[1],
                      p299_c1_plate_t],center=true);
            // Two continuous service webs replace the old mid-air cross tie
            // and cable ears. In the plate-down print they grow from the bed,
            // split the 58 mm foot roof into short supported spans, and retain
            // a 19 mm-wide central cable chute aligned with the XH opening.
            for (side=[-1,1])
                translate([side*11,95,(5+p299_pedestal_h-p299_c1_plate_t)/2])
                    cube([3,58,p299_pedestal_h-p299_c1_plate_t-5],center=true);
        }
        for (p=p299_panel_mounts)
            translate([p[0],p[1],-0.1]) cylinder(d=p297_m3_clear,h=5.2,$fn=32);
        for (x=[-p299_c1_pattern/2,p299_c1_pattern/2],
             y=[p299_c1_center[1]-p299_c1_pattern/2,
                p299_c1_center[1]+p299_c1_pattern/2])
            translate([x,y,p299_pedestal_h-p299_c1_plate_t-0.1])
                cylinder(d=p299_c1_clear_d,h=p299_c1_plate_t+0.2,$fn=48);
        // SLAMTEC C1 drawing: the captive lead leaves the middle of the side
        // between the negative-Y hole pair, at the sensor mounting plane.
        // This full-thickness slot starts outside the 55.6 mm footprint and
        // opens into the central chute directly above the existing XH hole.
        translate([p299_c1_center[0]-p299_c1_cable_slot[0]/2,
                   p299_c1_plate_center[1]-p299_c1_plate_size[1]/2-0.1,
                   p299_pedestal_h-p299_c1_plate_t-0.1])
            cube([p299_c1_cable_slot[0],p299_c1_cable_slot[1]+0.2,
                  p299_c1_plate_t+0.2],center=false);
        translate([p299_xh_opening_center[0],p299_xh_opening_center[1],-0.1])
            linear_extrude(height=5.2)
                offset(r=2) square([p299_xh_opening[0]-4,
                                    p299_xh_opening[1]-4],center=true);
        // Reusable tie passages through both service webs. Short 3 mm roofs
        // are support-free in the plate-down orientation; no floating ear
        // begins above the bed.
        for (side=[-1,1],z=[22,42])
            translate([side*11,95,z]) rotate([0,90,0])
                cylinder(d=5,h=5,center=true,$fn=24);
    }
}

module p299_c1_access_keepouts() {
    // Exact release gate volume: no printed material may enter these four
    // Ø4.5 vertical head/key corridors below the plate. The lower 0.2 mm is
    // above the unchanged foot, where an L-key can enter from the open side.
    for (x=[-p299_c1_pattern/2,p299_c1_pattern/2],
         y=[p299_c1_center[1]-p299_c1_pattern/2,
            p299_c1_center[1]+p299_c1_pattern/2])
        translate([x,y,5.2]) cylinder(d=p299_c1_head_d,
            h=p299_pedestal_h-p299_c1_plate_t-5.25,$fn=72);
}

module p299_c1_hardware_preview() {
    // Four M2.5 button heads and shafts, shown from below.
    for (x=[-p299_c1_pattern/2,p299_c1_pattern/2],
         y=[p299_c1_center[1]-p299_c1_pattern/2,
            p299_c1_center[1]+p299_c1_pattern/2]) {
        color([0.22,0.24,0.27]) translate([x,y,p299_pedestal_h-5.6])
            cylinder(d=p299_c1_head_d,h=1.6,$fn=48);
        color([0.22,0.24,0.27]) translate([x,y,p299_pedestal_h-4])
            cylinder(d=2.5,h=8,$fn=32);
    }
    // Representative 2 mm L-key engaged in the negative-X/negative-Y head.
    color([0.12,0.13,0.15]) {
        translate([-p299_c1_pattern/2,
                   p299_c1_center[1]-p299_c1_pattern/2,
                   p299_pedestal_h-18]) cylinder(d=2,h=12.5,$fn=6);
        translate([-p299_c1_pattern/2,
                   p299_c1_center[1]-p299_c1_pattern/2,
                   p299_pedestal_h-18]) rotate([0,90,0])
            cylinder(d=2,h=18,$fn=6);
    }
}

module p299_c1_pedestal_bottom_preview() {
    // Cut away the lower 55 mm so the view from below unambiguously shows all
    // four heads, the cable notch and the representative L-key engagement.
    color([0.92,0.63,0.14]) intersection() {
        p299_c1_pedestal_installed();
        translate([0,102.5,61]) cube([90,90,12],center=true);
    }
    p299_c1_hardware_preview();
}

module p299_c1_pedestal_side_preview() {
    // X=0 section: remove the near (+X) half and expose the service chute.
    color([0.92,0.63,0.14]) intersection() {
        p299_c1_pedestal_installed();
        translate([-50,102.5,p299_pedestal_h/2])
            cube([100,100,p299_pedestal_h+2],center=true);
    }
    p299_c1_hardware_preview();
    // Captive lead centreline: side exit at the mounting plane, then straight
    // down the protected central chute to the unchanged XH opening.
    color([0.08,0.24,0.78]) translate([0,82,5])
        cylinder(d=4,h=p299_pedestal_h-p299_c1_plate_t-5,$fn=32);
}

module p299_c1_ghost() {
    color([.12,.18,.22,.45])
        translate([p299_c1_center[0],p299_c1_center[1],
                   p299_pedestal_h+p299_c1_size[2]/2])
            cube(p299_c1_size,center=true);
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
    translate([0,0,p297_collar_z]) p299_panel_collar();
    translate([0,0,p297_panel_z+plate_t]) p299_c1_pedestal_installed();
    if (ghosts) {
        // Four oriented BTS envelopes plus their 25 mm terminal keepouts.
        translate([0,0,p297_power_z])
            for (c=p297_bts_centres) p297_bts_ghost(c[0],c[1]);
        translate([0,0,p297_collar_z]) p297_controls_ghosts();
        translate([0,0,p297_panel_z+plate_t]) p299_c1_ghost();
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
else if (part=="p297_seam_coupon_lower") p297_seam_coupon_lower();
else if (part=="p297_seam_coupon_upper") p297_seam_coupon_upper();
else if (part=="p297_logic_box") p297_logic_box();
else if (part=="p297_collar") p297_collar();
else if (part=="p299_collar_core") p299_collar_core();
else if (part=="p299_panel_collar")
    translate([0,0,p299_panel_collar_h]) rotate([180,0,0]) p299_panel_collar();
else if (part=="p299_c1_pedestal")
    // P2-99f prints upside-down: the 4 mm C1 plate is the complete flat bed
    // face. The widened cheeks and service webs then support the unchanged
    // 58 x 58 panel foot; there is no >50 mm bridge.
    translate([0,139,p299_pedestal_h]) rotate([180,0,0])
        p299_c1_pedestal_installed();
else if (part=="p299_c1_access_keepouts")
    translate([0,139,p299_pedestal_h]) rotate([180,0,0])
        p299_c1_access_keepouts();
else if (part=="p299_c1_bottom_preview")
    p299_c1_pedestal_bottom_preview();
else if (part=="p299_c1_side_preview")
    p299_c1_pedestal_side_preview();
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
       && (p297_seam_band-(p297_seam_lip_t+2*p297_seam_clearance))/2 >= 1.5,
       "P2-97 corner/socket wall is below its minimum radial thickness");
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
assert(p297_bts_air_gap >= 1.99,
       "P2-97 BTS heatsink has less than 2 mm air below it");
assert(p297_bts_pattern[1]/2-p297_bts_heatsink_size[1]/2-p297_m3_clear/2 >= 3,
       "P2-97 M3 path is too close to the measured BTS heatsink");
assert((8-p297_m3_clear)/2 >= 2
       && p297_bts_pattern[1]/2-p297_bts_heatsink_size[1]/2-8/2 >= 1,
       "P2-97 BTS standoff wall or heatsink side clearance is too small");
assert((part!="p297_power_box" && part!="p297_power_fitcheck"
        && part!="p297_assembly")
       || p297_bts_measured_h > 0,
       "P2-97: supply measured total BTS height with -D p297_bts_measured_h=36");
assert(p297_bts_measured_h <= 0
       || abs(p297_bts_measured_h-(p297_bts_heatsink_below
                                   +p297_bts_pcb_t+p297_bts_above)) < 0.6,
       "P2-97 measured total must match 20 mm heatsink + PCB + 15 mm top envelope");
assert(min(p297_lattice,p297_lattice_t) >= 2*p297_extrusion_w,
       "P2-97 lattice is thinner than two 0.62 mm extrusion lines");
assert(p297_seam_clearance >= 0 && p297_seam_clearance <= 0.8,
       "P2-97 seam clearance must be between 0 and 0.8 mm per side");
assert(p297_seam_lip_t >= 2*p297_extrusion_w,
       "P2-97 perimeter lip is thinner than two 0.62 mm extrusion lines");
