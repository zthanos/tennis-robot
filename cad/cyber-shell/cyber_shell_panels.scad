// Cyber-Shell body panels — tennis robot "Concept A"
// Coordinate system: chassis centre = origin, +X forward, +Y left, +Z up.
// All dimensions in mm.  Designed for FDM print-bed <= 250x250 mm.
//
// Print guide:
//   front_faceplate_half(1)  / front_faceplate_half(-1)   upright, no supports
//   side_panel_half(true)    / side_panel_half(false)      flat, outer face down
//   rear_panel_half(1)       / rear_panel_half(-1)         flat, outer face down
//   top_cover_half(true)     / top_cover_half(false)       outer face down
//
// Set SHOW to choose what to render:
//   "assembly"     full robot assembly view (slow)
//   "front"        both front halves
//   "sides"        all four side-panel pieces
//   "rear"         both rear halves
//   "top"          both top halves
//   "print_layout" flat exploded view for slicing

SHOW = "assembly";

$fn = 72;

// ─── Robot reference dimensions ──────────────────────────────────────────────
CHASSIS_X  = 920;
CHASSIS_Y  = 580;
CHASSIS_Z  = 14;

PANEL_H    = 440;          // shell height above chassis top surface
HX         = CHASSIS_X / 2;  // 460
HY         = CHASSIS_Y / 2;  // 290

ROLLER_X   = 620;
ROLLER_Z   = 95;
ROLLER_W   = 240;

CAM_Z      = 310;          // camera window base height in panel frame

LIDAR_X    = -80;          // from chassis centre
LIDAR_HOLE_R = 48;

WHEEL_X    = -350;
WHEEL_Y    = 350;
WHEEL_R    = 90;απο ο
WHEEL_W    = 80;

// ─── Panel parameters ─────────────────────────────────────────────────────────
WALL_T     = 4;
LIP_W      = 10;    // assembly lip width
LIP_D      = 8;     // assembly lip depth

// ─── Shared helpers ───────────────────────────────────────────────────────────
module screw_boss(h=12, od=10, id=5.5) {
    difference() {
        cylinder(h=h, d=od);
        translate([0, 0, -0.5]) cylinder(h=h+1, d=id);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  1.  FRONT FACEPLATE  —  "Shark Nose"
//      Each half covers y = 0..+HY (left) or 0..-HY (right).
//      Wedge cross-section: bottom thickness FACE_BT, top raked FACE_RAKE
//      forward.  Trapezoidal roller opening at base.  Camera slot at top.
// ─────────────────────────────────────────────────────────────────────────────
FACE_BT    = 42;   // base (bottom) depth in X
FACE_RAKE  = 26;   // extra X-protrusion of the top edge (shark nose rake)
FACE_TT    = FACE_BT + FACE_RAKE;  // total top depth

// Roller opening cut
RCUT_W_BOT = ROLLER_W + 40;  // 280 mm at base
RCUT_W_TOP = ROLLER_W + 10;  // 250 mm at top of opening
RCUT_H     = 92;             // opening height

// Camera window
CAM_WIN_W  = 120;
CAM_WIN_H  = 40;

// Hull helper: rake wedge for one half (y from 0 to HY in +Y direction).
// After calling, rotate/mirror to place correctly.
module _face_wedge_half_shape() {
    hull() {
        // bottom plate
        translate([0, 0, 0])
            cube([FACE_BT, HY, 1]);
        // top plate, shifted forward by FACE_RAKE
        translate([FACE_RAKE, 0, PANEL_H - 1])
            cube([FACE_BT, HY, 1]);
    }
}

module _face_wedge_inner() {
    hull() {
        translate([WALL_T, WALL_T, WALL_T])
            cube([FACE_BT - WALL_T*2, HY - WALL_T*2, 1]);
        translate([FACE_RAKE + WALL_T, WALL_T, PANEL_H - 1 - WALL_T])
            cube([FACE_BT - WALL_T*2, HY - WALL_T*2, 1]);
    }
}

module _roller_cut() {
    // Trapezoidal prism: wider at z=0, narrower at z=RCUT_H.
    // Centred at y=0, extends through full X depth.
    hull() {
        translate([-1, -(RCUT_W_BOT/2), 0])
            cube([FACE_TT + 2, RCUT_W_BOT, 1]);
        translate([-1, -(RCUT_W_TOP/2), RCUT_H - 1])
            cube([FACE_TT + 2, RCUT_W_TOP, 1]);
    }
}

module front_faceplate_half(side=1) {
    // side=1  -> left (+Y) half
    // side=-1 -> right (-Y) half
    // Placed so that the rear face sits at x=HX (chassis front edge),
    // panel extends toward +X.
    sy = (side > 0) ? 1 : -1;

    difference() {
        union() {
            if (side > 0) {
                _face_wedge_half_shape();
            } else {
                // mirror in Y for the right half
                mirror([0, 1, 0]) _face_wedge_half_shape();
            }
            // Assembly lip on the Y-split edge (inner face)
            translate([0, -LIP_D, WALL_T])
                cube([FACE_TT, LIP_D, PANEL_H - WALL_T*2]);
            // Screw bosses on rear (inner) face
            for (bz = [60, 200, 340])
                translate([WALL_T, sy * 30, bz])
                    rotate([0, 90, 0]) screw_boss(h=14);
        }

        // Hollow interior
        if (side > 0) {
            _face_wedge_inner();
        } else {
            mirror([0, 1, 0]) _face_wedge_inner();
        }

        // Roller trapezoid opening
        _roller_cut();

        // Camera window — centred on full width so it spans both halves;
        // each half just gets its portion naturally
        translate([-1, -(CAM_WIN_W/2), CAM_Z])
            cube([FACE_TT + 2, CAM_WIN_W, CAM_WIN_H]);

        // Hairline split at y=0 to separate halves cleanly
        translate([-1, -0.3, -1])
            cube([FACE_TT + 2, 0.6, PANEL_H + 2]);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  2.  SIDE PANELS  — "Aero Skirts"
//      Flat WALL_T slab, 880 mm long, PANEL_H tall.
//      Split front / rear at x=0 (chassis centre).
//      Character line groove at mid-height.
//      Rear half: 3 pseudo-louvre vents near wheel arch.
// ─────────────────────────────────────────────────────────────────────────────
SIDE_LEN   = 880;
HALF_SIDE  = SIDE_LEN / 2;   // 440 mm each half

module side_panel_half(front=true) {
    // Output placed at x=0 (chassis centre), extending toward +X (front=true)
    // or -X (front=false).  Caller mirrors in Y for the right side.
    xstart = front ? 0 : -HALF_SIDE;
    xlen   = HALF_SIDE;
    arch_x = WHEEL_X - xstart;  // wheel arch x in local coords (only on rear half)

    difference() {
        union() {
            cube([xlen, WALL_T, PANEL_H]);

            // Assembly lip on split edge (at chassis-centre end of each half)
            translate([front ? 0 : (xlen - LIP_D), 0, WALL_T])
                cube([LIP_D, LIP_W, PANEL_H - WALL_T*2]);

            // Top assembly lip
            translate([0, 0, PANEL_H - LIP_W])
                cube([xlen, LIP_D, LIP_W]);

            // Screw bosses on inner face
            for (bx = [xlen*0.25, xlen*0.75])
                for (bz = [80, 240, 380])
                    translate([bx, WALL_T, bz])
                        rotate([-90, 0, 0]) screw_boss(h=10);
        }

        // Character line groove (1/3 from bottom, 6 mm wide, 2.5 mm deep)
        translate([-1, -1, PANEL_H / 3])
            cube([xlen + 2, WALL_T - 1.5, 6]);

        // Rear half: 3 louvre vents (near wheel arch region)
        if (!front) {
            // scoops near wheel-arch region (chassis x ≈ WHEEL_X = -350)
            // local x = WHEEL_X - xstart = -350 - (-440) = 90
            for (i = [0:2])
                translate([80, -1, 100 + i * 20])
                    cube([28, WALL_T + 2, 9]);

            // Wheel arch clearance
            translate([WHEEL_X - xstart, -1, 0])
                scale([1, 1, 1])
                    cylinder(h = WHEEL_R + 20, r = WHEEL_R + 20, $fn=60);
        }
    }
}

// Left side = +Y face; right side mirrors in Y, flipped in X.
module side_panel_left(front=true) {
    xoff = front ? 0 : -HALF_SIDE;
    translate([xoff, HY, 0])
        side_panel_half(front=front);
}

module side_panel_right(front=true) {
    xoff = front ? 0 : -HALF_SIDE;
    translate([xoff, -HY - WALL_T, 0])
        mirror([0, 1, 0]) side_panel_half(front=front);
}

// ─────────────────────────────────────────────────────────────────────────────
//  3.  REAR PANEL  — "Truncated Tail" with fenders
//      U-shape: rear flat face + two short side returns + wheel-arch fenders.
//      Split L/R at y=0.
// ─────────────────────────────────────────────────────────────────────────────
REAR_RETURN = 80;   // how far the U side-legs extend forward
FENDER_R    = WHEEL_R + 18;  // 108 mm fender radius

module rear_panel_half(side=1) {
    // side=1 (+Y / left), side=-1 (-Y / right)
    ydir = (side > 0) ? 1 : -1;

    difference() {
        union() {
            // Rear face (flush with chassis rear x = -HX)
            if (side > 0) {
                translate([-HX, 0, 0])
                    cube([WALL_T, HY, PANEL_H]);
            } else {
                translate([-HX, -HY, 0])
                    cube([WALL_T, HY, PANEL_H]);
            }

            // Side return leg of the U
            translate([-HX, ydir > 0 ? HY : -(HY + WALL_T), 0])
                cube([REAR_RETURN, WALL_T, PANEL_H]);

            // Half-cylinder fender over drive wheel
            translate([WHEEL_X, ydir * (HY + WALL_T + FENDER_R), 0])
                rotate([-90, 0, 0])
                difference() {
                    cylinder(h = WALL_T, r = FENDER_R + WALL_T);
                    translate([0, 0, -0.5]) cylinder(h = WALL_T + 1, r = FENDER_R);
                    // clip to upper half
                    translate([-(FENDER_R + WALL_T + 1), -(FENDER_R + WALL_T + 1), -0.5])
                        cube([(FENDER_R + WALL_T + 1)*2, FENDER_R + WALL_T + 1, WALL_T + 1]);
                }

            // Screw bosses on inner face
            for (bz = [60, 200, 340])
                translate([-HX + WALL_T, ydir * 20, bz])
                    rotate([0, 90, 0]) screw_boss(h=14);
        }

        // Hairline split at y=0
        translate([-HX - 1, -0.3, -1])
            cube([REAR_RETURN + 2, 0.6, PANEL_H + 2]);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  4.  TOP COVER
//      Flat WALL_T plate, split front/rear at x=0.
//      Printed outer-face-down → perfect finish, zero supports.
//      Rear half has LiDAR cutout.
// ─────────────────────────────────────────────────────────────────────────────
module top_cover_half(front=true) {
    xstart = front ? 0 : -HX;
    xlen   = HX;
    z      = PANEL_H;

    difference() {
        union() {
            translate([xstart, -HY, z])
                cube([xlen, CHASSIS_Y, WALL_T]);

            // Assembly lip on split edge
            translate([front ? (xstart - LIP_D) : (xstart + xlen - LIP_D),
                       -HY + WALL_T, z - LIP_W])
                cube([LIP_D, CHASSIS_Y - WALL_T*2, LIP_W]);

            // Corner screw bosses (pointing down)
            for (bx = [xstart + xlen*0.25, xstart + xlen*0.75])
                for (by = [-HY + 20, HY - 20])
                    translate([bx, by, z - 12])
                        screw_boss(h=12);
        }

        // LiDAR hole (rear half only; LIDAR_X = -80 from chassis centre)
        if (!front)
            translate([LIDAR_X, 0, z - 1])
                cylinder(h = WALL_T + 2, r = LIDAR_HOLE_R);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  5.  ASSEMBLY & DISPLAY
// ─────────────────────────────────────────────────────────────────────────────
module ghost_chassis() {
    color([0.60, 0.42, 0.25, 0.12])
        translate([-HX, -HY, -(CHASSIS_Z/2)])
            cube([CHASSIS_X, CHASSIS_Y, CHASSIS_Z]);
    // Wheel ghosts
    color([0.08, 0.08, 0.08, 0.25]) {
        for (wy = [WHEEL_Y, -WHEEL_Y]) {
            translate([WHEEL_X, wy, 0])
                rotate([90, 0, 0])
                    cylinder(h=WHEEL_W, r=WHEEL_R, center=true);
        }
    }
}

module all_front()  {
    translate([HX, 0, 0]) {
        front_faceplate_half(side= 1);
        front_faceplate_half(side=-1);
    }
}

module all_sides() {
    side_panel_left(front=true);
    side_panel_left(front=false);
    side_panel_right(front=true);
    side_panel_right(front=false);
}

module all_rear() {
    rear_panel_half(side= 1);
    rear_panel_half(side=-1);
}

module all_top() {
    top_cover_half(front=true);
    top_cover_half(front=false);
}

module assembly() {
    ghost_chassis();
    color([0.94, 0.94, 0.94]) {
        all_front();
        all_sides();
        all_rear();
        all_top();
    }
}

// Print-layout: all pieces laid flat on XY plane, grouped by part
module print_layout() {
    // Front halves
    translate([  0,   0, 0]) front_faceplate_half(side= 1);
    translate([320,   0, 0]) front_faceplate_half(side=-1);
    // Side panel halves
    translate([  0, 260, 0]) side_panel_half(front=true);
    translate([  0, 280, 0]) side_panel_half(front=false);
    // Rear panel halves
    translate([  0, 560, 0]) rear_panel_half(side= 1);
    translate([320, 560, 0]) rear_panel_half(side=-1);
    // Top cover halves
    translate([  0, 800, 0]) top_cover_half(front=true);
    translate([480, 800, 0]) top_cover_half(front=false);
}

// ─── Render entry point ───────────────────────────────────────────────────────
if (SHOW == "assembly")     assembly();
if (SHOW == "front")        all_front();
if (SHOW == "sides")        all_sides();
if (SHOW == "rear")         all_rear();
if (SHOW == "top")          all_top();
if (SHOW == "print_layout") print_layout();
