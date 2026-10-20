// P2-84 — release geometry for the rear portion of body concept F on rear pod v2.
// STL parts are designed but NOT approved for printing yet.
// Robot coordinates: +X front, +Y left, +Z up.  Select with `part`.

use <body-panels-f.scad>
use <body-panels-v3.scad>
use <../rear-pod/rear-pod-v2.scad>
use <../modular-test-chassis/modular-chassis-v3.scad>
use <../modular-test-chassis/shared-flywheel-intake-launcher-phase1.scad>

part = "assembly";
show_context = true;
door_angle = 0;
$fn = 72;

icr_x = 117;
f_inner_r = 424.5;       // P2-84: protected F datum / inside face
f_t = 2.5;               // grows outward; pod clearance is not consumed
f_outer_r = f_inner_r + f_t;
f_y_end = 171;
f_z0 = 60;
f_z_split0 = 198;
f_z_split1 = 201;        // 3 mm removable-panel seam
f_z_top = 350;
m3_clear_f = 3.4;
m4_clear_f = 4.5;

function rear_x(y,r=f_inner_r) = icr_x-sqrt(r*r-y*y);
hinge_x = rear_x(164)-3;  // radial offset keeps M4 barrel >=3 mm from drawer
hinge_y = 165.5;          // 0.5 mm beyond the exact Y161 drawer boundary

module half_annulus_2d(r0=f_inner_r,r1=f_outer_r) {
    intersection() {
        difference() {
            translate([icr_x,0]) circle(r=r1);
            translate([icr_x,0]) circle(r=r0);
        }
        translate([-400,0]) square([150,f_y_end]);
    }
}

module curved_patch(z0,z1,y0=0,y1=f_y_end,r0=f_inner_r,r1=f_outer_r) {
    intersection() {
        translate([0,0,z0]) linear_extrude(z1-z0) half_annulus_2d(r0,r1);
        translate([-400,y0,z0-1]) cube([150,y1-y0,z1-z0+2]);
    }
}

module x_cylinder(p,d,h=30) {
    translate(p) rotate([0,90,0]) cylinder(d=d,h=h,center=true);
}

// Broad x extrusion of a diagonal YZ bar, clipped back to the curved shell.
module yz_bar(y0,z0,y1,z1,w=4) {
    hull() {
        translate([-292,y0,z0]) cube([70,w,w],center=true);
        translate([-292,y1,z1]) cube([70,w,w],center=true);
    }
}

module rear_lo_positive_raw() {
    difference() {
        union() {
            // Integrated Parkside frame: the opening is cut below.
            curved_patch(f_z0,f_z_split0);

            // External-screw lands bridge the F skin to the frozen base rear
            // face.  They remain part of rear_lo, not of the pod envelope.
            translate([rear_x(45)-1,39,54])
                cube([-269-(rear_x(45)-1),12,10]);
            translate([rear_x(164)-1,161.25,54])
                cube([-269-(rear_x(164)-1),5.75,10]);

            // Captive side of the M4 vertical hinge.  Alternating knuckles
            // let the door lift off after its pin is removed.
            for (zz=[[62,74],[90,110],[140,150]])
                translate([hinge_x,hinge_y,zz[0]])
                    cylinder(d=8,h=zz[1]-zz[0]);

            // Two outward receiver bosses for rear_hi locating tongues.
            for (y=[30,142]) curved_patch(190,200,y-6,y+6,f_inner_r,f_inner_r+7);
        }

        // Bottom-open Parkside aperture: drawer extraction stays pure -X.
        translate([-330,68,59]) cube([80,93.5,87]);
        for (y=[45,164]) x_cylinder([rear_x(y)-2,y,55],m3_clear_f,48);
        for (zz=[[61,75],[89,111],[139,151]])
            translate([hinge_x,hinge_y,zz[0]])
                cylinder(d=m4_clear_f,h=zz[1]-zz[0]);

        // 0.3 mm per-side fit around the two removable upper tongues.
        for (y=[30,142]) curved_patch(193,201,y-4.3,y+4.3,
                                      f_inner_r+1.7,f_inner_r+5.8);
    }
}

module rear_hi_positive_raw() {
    difference() {
        union() {
            difference() {
                curved_patch(f_z_split1,347);
                translate([-330,10,217]) cube([80,150,114]);
            }

            // The frame and ±60-degree grille are one continuous print body.
            intersection() {
                curved_patch(214,334,7,163);
                union() {
                    for (k=[-7:12]) yz_bar(-20+k*20,214,50+k*20,334,4);
                    for (k=[-4:15]) yz_bar(-20+k*20,334,50+k*20,214,4);
                }
            }

            // Tool-free lower location plus captive-nut bosses immediately
            // below the deck.  Four top-access screws release both halves.
            for (y=[30,142]) curved_patch(194,205,y-4,y+4,
                                          f_inner_r+2,f_inner_r+5.5);
            for (y=[35,140])
                translate([rear_x(y)-1.25,y,339]) cylinder(d=12,h=8);
        }
        for (y=[35,140]) {
            translate([rear_x(y)-1.25,y,338]) cylinder(d=m3_clear_f,h=11);
            translate([rear_x(y)-1.25,y,338]) cylinder(d=6.6,h=4.5,$fn=6);
        }
    }
}

module parkside_door_positive_raw() {
    difference() {
        union() {
            curved_patch(63,143,71,158,f_inner_r+0.4,f_outer_r-0.4);
            // Door-side alternating knuckles, joined to the leaf at Y158.
            for (zz=[[75,89],[111,139]])
                union() {
                    translate([hinge_x,hinge_y,zz[0]])
                        cylinder(d=8,h=zz[1]-zz[0]);
                    translate([hinge_x-4,157,zz[0]])
                        cube([8,8,zz[1]-zz[0]]);
                }
            // Outside pull, clear of the drawer volume behind the leaf.
            translate([rear_x(82)-4,82,96]) rotate([0,90,0])
                difference() { cylinder(d=16,h=5); cylinder(d=7,h=7); }
        }
        for (zz=[[74,90],[110,140]])
            translate([hinge_x,hinge_y,zz[0]])
                cylinder(d=m4_clear_f,h=zz[1]-zz[0]);
    }
}

module deck_positive_raw() {
    difference() {
        union() {
            intersection() {
                translate([0,0,347]) linear_extrude(3)
                    difference() {
                        translate([icr_x,0]) circle(r=f_outer_r);
                        translate([icr_x,0]) circle(r=387.5);
                    }
                translate([-320,0,346]) cube([60,f_y_end,5]);
            }
            // Two rear-facing fastening tongues meet only the P2-84 panel
            // lands.  All other deck edges keep a 3 mm service gap.
            translate([-271,74,341]) cube([4.5,12,9]);
        }
        // Conservative control-panel exclusion (intentional tongues excepted
        // by adding them after the cut in deck_positive()).
        translate([-270.5,-1,346]) cube([145,100,6]);
    }
}

module deck_positive() {
    difference() {
        union() {
            deck_positive_raw();
            difference() {
                translate([-271,74,341]) cube([4.5,12,9]);
                x_cylinder([-270,80,345],m3_clear_f,9);
            }
        }
        for (y=[35,140])
            translate([rear_x(y)-1.25,y,346]) cylinder(d=m3_clear_f,h=6);
    }
}

module side_copy(side=1) {
    if (side>0) children(); else mirror([0,1,0]) children();
}

module rear_lo(side=1) { side_copy(side) rear_lo_positive_raw(); }
module rear_hi(side=1) { side_copy(side) rear_hi_positive_raw(); }
module deck_rear(side=1) { side_copy(side) deck_positive(); }

module parkside_door(side=1,angle=0) {
    // Pivot about the vertical hinge line; signs open each leaf away from the
    // drawer's straight -X extraction corridor.
    hx=hinge_x;
    side_copy(side)
        translate([hx,hinge_y,0]) rotate([0,0,angle]) translate([-hx,-hinge_y,0])
            parkside_door_positive_raw();
}

module all_fixed_f_rear() {
    for (s=[-1,1]) { rear_lo(s); rear_hi(s); deck_rear(s); }
}

module all_f_rear(angle=0) {
    all_fixed_f_rear();
    for (s=[-1,1]) parkside_door(s,angle);
}

module drawer_sweep() {
    for (s=[-1,1]) translate([-364,s*115.5,98.5]) cube([472,91,81],center=true);
}

module p284_pod() { installed_p279_structure(); }
module moving_sweep() { for (a=[-10:2:20]) bp_moving(a); }

module rounded_keepout(origin,size,r=3) {
    minkowski() {
        translate(origin) cube(size);
        sphere(r=r,$fn=12);
    }
}

// Conservative exact solids for the 3 mm service-clearance gate.  This is
// deliberately broader than the detailed pod: it encloses the exact base
// footprint, panel, posts/side frames, carrier/electronics and every reserved
// component after a 3 mm expansion, without the very slow whole-assembly
// Minkowski operation.
module pod_clearance_3() {
    translate([0,0,48]) linear_extrude(14)
        offset(delta=3) p276_footprint_2d();
    // posts + side frames and central load path
    rounded_keepout([-263,-82,61],[129,164,286]);
    // carrier, rear electronics and cable bend conservative union box
    rounded_keepout([-258,-157,125],[126,252,162]);
    // Parkside reservations
    for (s=[-1,1])
        rounded_keepout([-269,s*115.5-45.5,58],[141,91,81]);
    // Ugreen and Pi reservations
    rounded_keepout([-255.5,16,232],[121,52,52]);
    rounded_keepout([-261,-37,64],[98,74,46]);
    // ribbed control panel and control-body volume, expanded 3 mm
    rounded_keepout([-267.5,-95,339],[138,190,12]);
}

module f_skin_without_interfaces() {
    for (s=[-1,1]) { rear_hi(s); }
    for (s=[-1,1]) parkside_door(s,0);
    // rear_lo without its small base attachment band
    difference() {
        for (s=[-1,1]) rear_lo(s);
        translate([-330,-180,53]) cube([70,360,12]);
    }
    // deck minus the two intended control-panel tongues
    difference() {
        for (s=[-1,1]) deck_rear(s);
        translate([-273,-90,339]) cube([9,180,13]);
    }
}

module interface_fasteners() {
    for (s=[-1,1],y=[s*45,s*164])
        color("silver") x_cylinder([rear_x(abs(y))-1,y,55],3,42);
    for (s=[-1,1]) color("silver") x_cylinder([-270,s*80,345],3,18);
    for (s=[-1,1]) color("silver")
        translate([hinge_x,s*hinge_y,58]) cylinder(d=4,h=96);
}

// Printable orientation: global Z/Y become bed X/Y; radial depth becomes Z.
module print_orient() { rotate([0,-90,0]) children(); }

module assembly_scene(service=false) {
    if (show_context) {
        color([0.15,0.36,0.62,0.72]) p284_pod();
        color([0.35,0.35,0.38,0.28]) frame_assembly();
        color([0.18,0.18,0.20,0.18]) drive_package_envelopes();
    }
    color([0.12,0.12,0.13]) for (s=[-1,1]) rear_lo(s);
    if (!service) color([0.74,0.40,0.27]) for (s=[-1,1]) rear_hi(s);
    color([0.74,0.40,0.27]) for (s=[-1,1]) deck_rear(s);
    color([0.20,0.20,0.22]) for (s=[-1,1]) parkside_door(s,service?110:door_angle);
    interface_fasteners();
}

// Empty output means PASS for every exact-solid QA selector.
if (part=="rear_lo_left") print_orient() rear_lo(1);
else if (part=="rear_lo_right") print_orient() rear_lo(-1);
else if (part=="rear_hi_left") print_orient() rear_hi(1);
else if (part=="rear_hi_right") print_orient() rear_hi(-1);
else if (part=="deck_rear_left") print_orient() deck_rear(1);
else if (part=="deck_rear_right") print_orient() deck_rear(-1);
else if (part=="parkside_door_left") print_orient() parkside_door(1,0);
else if (part=="parkside_door_right") print_orient() parkside_door(-1,0);
else if (part=="qa_skin_pod_3mm") intersection() {
    f_skin_without_interfaces(); pod_clearance_3();
}
else if (part=="qa_chassis") intersection() { all_f_rear(); frame_assembly(); }
else if (part=="qa_wheels") intersection() { all_f_rear(); drive_package_envelopes(); }
else if (part=="qa_moving") intersection() { all_f_rear(); moving_sweep(); }
else if (part=="qa_drawer_fixed") intersection() { all_fixed_f_rear(); drawer_sweep(); }
else if (part=="qa_drawer_open") intersection() {
    union() { for (s=[-1,1]) parkside_door(s,110); } drawer_sweep();
}
else if (part=="qa_ground") intersection() {
    all_f_rear(); translate([-400,-300,-100]) cube([800,600,115]);
}
else if (part=="service") assembly_scene(true);
else assembly_scene(false);
