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
    // A 1.6 mm ridge satisfies the horizontal buffer gate.  Short 0.2 mm
    // breaks keep each supported downward patch below 40 mm2; the lattice and
    // tongue join the segments into the final single body.
    count = ceil(length/23.5);
    pitch = length/count;
    for (i=[0:count-1])
        translate([(i-(count-1)/2)*pitch,0,0])
            p297_beam_segment_x(pitch-0.2,depth,height);
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
                cube([x == 0 ? 8 : 6,p297_lattice_t,
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

module p297_tray_rail(y,z) {
    // Horizontal rear-service runners.  Each carrier pulls toward -X without
    // disturbing the floors below or removing the control panel.
    translate([0,y,z]) union() {
        front_post_y = sign(y)*47.5;
        front_dogleg = front_post_y-y;
        difference() {
            union() {
                p297_beam_x(112,8,7);
                // The segmented print-safe beam needs one continuous outer
                // top spine so every tray rail is part of the box body.
                translate([0,sign(y)*3,3]) cube([112,2,1],center=true);
            }
            translate([0,-sign(y)*2,2.1]) cube([104,4.2,3.4],center=true);
        }
        // Short stop/bridge overlaps the rail and the X=66.4 front wall.
        // The rear remains open for independent tray withdrawal.
        translate([61,0,0]) p297_beam_x(11,4,4);
        // A 5 x 5 printable corner node removes tapered sub-line gaps where
        // the low rail bridge enters the front wall/socket roof.
        translate([66.4,0,0]) cube([5,5,7],center=true);
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
            // At only 24 mm high the lattice produced dozens of tiny slicer
            // islands.  Continuous 2.4 mm side walls give the collar a
            // four-line PETG shell and retain the common socket below.
            p297_open_box(p297_collar_h,false,false,false,false,false,false);
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
                    cube([10,40,p297_floor],center=true);
            }
        }
        for (x=[-58,58],y=[-56,56]) translate([x,y,p297_collar_h-6])
            cylinder(d=m4_clear,h=8,$fn=32);
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
else if (part=="p297_seam_coupon_lower") p297_seam_coupon_lower();
else if (part=="p297_seam_coupon_upper") p297_seam_coupon_upper();
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
