// P2-76 printable rear pod v2 — upright electronics carrier.
// Select a printable part with -D 'part="..."'.

include <params-v2.scad>

part = "assembly";
show_ghosts = true;
show_panel = true;
status_led_hole = 8;

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
            // P2-86 rear-skin interfaces: both are full 12 mm lands.  Their
            // forward noses sit at local Z-7.5..2.5 (global Z47.5..57.5),
            // below the Parkside sweep, with >=3 mm around the Ø3.4 bore.
            for (y=[side*45,side*145])
                translate([pod_x_rear,y-6,-7.5]) cube([22,12,10]);
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
        // P2-86b: the rear-panel stay now replaces this local base strip and
        // shares the existing rear-crossbar top/side M4 at Y+/-165.  The
        // stay top is flush at Z57.5 under the Parkside reservation; no
        // chassis drilling and no pod-envelope growth.
        translate([-198,side>0?156.8:-168.5,-4.1]) cube([59,11.7,8.2]);
        for (y=[75,125])
            translate([rear_crossbar_x,side*y,-20]) cylinder(d=m4_clear,h=40);
        // Through-M4 side-frame fasteners; nuts and washers stay accessible
        // below the base, clear of the rear-only stiffening ribs.
        for (x=[panel_center_x-58,panel_center_x+58])
            translate([x,side*54,-20]) cylinder(d=m4_clear,h=40);
        for (x=[-245,-210,-175])
            translate([x,side*145,-20]) linear_extrude(height=40)
                capsule_2d(14,4,true);
        // P2-83: round through-hole plus a 90-degree countersink.  In the
        // Parkside band the local top is Z2.5/global Z57.5, so an M4 flat
        // head finishes flush and remains 0.5 mm below the drawer sweep.
        translate([-134,side*145,-20]) cylinder(d=m4_clear,h=22.6);
        translate([-134,side*145,0.20])
            cylinder(h=2.35,d1=m4_clear,d2=9.2);
        // P2-86: rear-facing M3 at Z52.5 global. Captive nut loads from the
        // open underside; both Y45/Y145 lands retain at least 3 mm material.
        for (y=[side*45,side*145]) {
            translate([pod_x_rear-1,y,-2.5]) rotate([0,90,0])
                cylinder(d=m3_clear,h=24);
            translate([pod_x_rear+14,y,-2.5]) rotate([0,90,0])
                cylinder(d=6.6,h=3.4,$fn=6);
            translate([pod_x_rear+13.8,y-3.3,-8.5]) cube([3.8,6.6,6]);
        }
    }
}

// P2-86b one-piece inside L tie. The P2-83 rail-side datum was not mirrored
// as a through bore in the printed -Y rail. Both ties now share the existing
// rear-crossbar transverse M4 at Y+/-165/Z37 with the P2-86b panel stay:
// tie on the front face, stay on the rear face, one longer through bolt.
// The horizontal flange still bears on the base underside at Z51.
module corner_tie() {
    difference() {
        union() {
            // Underside bridge: installed global Y139..171, Z45..51.
            translate([7,10,19]) cube([13,32,6],center=true);
            // Front-face leg touches the rear crossbar at X-140.
            translate([3,20,11]) cube([6,16,22],center=true);
        }
        translate([6,0,14]) cylinder(d=m4_clear,h=10);
        translate([-2,20,8]) rotate([0,90,0]) cylinder(d=m4_clear,h=20);
    }
}

module corner_tie_installed(side=1) {
    translate([-140,side*145,29])
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
            // P2-86 continuous rear beam.  Four M3 per Z row tie deck and
            // rear_hi independently to a structural member.  It stays inside
            // the existing X/Y panel envelope; the deep leg is rear of all
            // control bodies and forms the deck's L-section support.
            translate([-69,-91,-20]) cube([20,182,24]);
        }
        // Existing four Ø8.6 locating holes; the P2-79 fastener is through-M4.
        for (x=[-58,58],y=[-56,56])
            translate([x,y,-9]) cylinder(d=8.6,h=14);
        translate([0,-58,-9]) cylinder(d=estop_hole,h=14);
        translate([0,-12,-9]) cylinder(d=toggle_hole,h=14);
        translate([0,32,-9]) cylinder(d=arm_hole,h=14);
        translate([-42,70,-9]) linear_extrude(height=14)
            capsule_2d(24,8,true);
        // Additional status LED requested by P2-84, on the +X side of the
        // cable slot.  It misses the X52 collector rib and rear post pocket.
        translate([42,70,-9]) cylinder(d=status_led_hole,h=14);

        // P2-86 through-M3 rows; externally driven screws and +X washers/
        // nylocs. Hole centres are 10 mm apart vertically so each retains
        // at least 3 mm of PETG to the neighbouring bore.
        for (y=[-85,-20,20,85],z=[-13,-3])
            translate([-70,y,z]) rotate([0,90,0]) cylinder(d=m3_clear,h=23);

        // Underside keepouts: actual ghost body plus 1.5 mm per side.
        translate([-19,-74.5,-9]) cube([38,33,10]);
        translate([-15,-21,-9]) cube([30,18,10]);
        translate([0,32,-9]) cylinder(d=18,h=10);
        // P2-79 Ø23 post end / washer access around every mounting land.
        for (x=[-58,58],y=[-56,56])
            // P2-86 extends the same access cylinder down through the new
            // rear beam; this preserves shoulder-only post/panel contact.
            translate([x,y,-21]) cylinder(d=25,h=22);
        // The +Y rear post finishes its obstacle-avoidance dogleg below the
        // shoulder.  Extend only the deep-beam relief to follow that path;
        // the top skin/shoulder datum and the Y85 fastening land remain.
        hull() for (y=[56,65])
            translate([-58,y,-21]) cylinder(d=25,h=12.5);
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

// P2-88 side frame: print two.  The jaw, tongue datum and all upper post
// cross-bolts are frozen.  Both base fasteners now sit outside the lower
// columns, so the head/nut and a 60 mm tool path are physically reachable.
module frame_web_2d() {
    union() {
        // Upper column datums are unchanged.  The lower 28 mm doglegs leave
        // the rear nyloc and front screw head completely exposed.
        for (x=[-58,58]) translate([x-4,30]) square([8,60]);
        polygon(points=[[-45,4],[-37,4],[-37,25],[-54,34],[-62,34],[-45,25]]);
        polygon(points=[[45,4],[53,4],[62,34],[54,34]]);
        polygon(points=[[-58,4],[-50,4],[24,76],[16,84]]);
        polygon(points=[[58,4],[50,4],[15,76],[23,84]]);
        translate([12,75]) square([15,31]);
        translate([-58,4]) square([124.5,7]);
    }
}

module side_frame() {
    difference() {
        union() {
            rotate([90,0,0]) linear_extrude(height=8,center=true) frame_web_2d();
            // Rear pad reuses X-256.5. Front pad moves to X-132, local 66.5.
            for (x=[-58,side_frame_front_x-panel_center_x])
                translate([x,0,2]) cylinder(d=14,h=4,center=true);
            // Low bridge keeps the rear pad integral below the open wrench
            // pocket and overlaps the inward dogleg by 1 mm.
            translate([-49.5,0,2.5]) cube([17,8,5],center=true);
        }
        for (x=[-58,side_frame_front_x-panel_center_x])
            translate([x,0,-1]) cylinder(d=m4_clear,h=8);
        // Ø7 vertical tool chimney above the moved front screw head.
        translate([side_frame_front_x-panel_center_x,0,4]) cylinder(d=7,h=28);
        // Rear nyloc pocket/open-ended wrench space; the rear bolt enters
        // from below through the wedge washer.
        translate([-58,0,4]) cylinder(d=18,h=23);
        // Carrier plate is X17.5..21.5 relative to panel center.
        translate([17.1,-6,74]) cube([4.8,12,34]);
        // M4 transverse clamp screw through jaw and carrier tongue.
        translate([8,0,95]) rotate([0,90,0]) cylinder(d=m4_clear,h=24);
        // Through-M4 cross-bolts for the two upper-panel posts.
        for (x=[-58,58]) translate([x,-6,90]) rotate([-90,0,0])
            cylinder(d=m4_clear,h=12);
    }
}

// Flat-bottom wedge follows the piecewise underside of the printed rear rib
// at X-256.5.  Installed bottom is global Z39; top seats exactly on the rib.
module rear_foot_wedge_washer() {
    difference() {
        polyhedron(
          points=[[-8,-7,0],[8,-7,0],[8,7,0],[-8,7,0],
                  [-8,-7,10.4615],[2.5,-7,4],[8,-7,4],
                  [-8,7,10.4615],[2.5,7,4],[8,7,4]],
          faces=[[0,1,2,3],[0,4,5,1],[1,5,6],[1,6,2],
                 [3,2,6,9],[3,9,8,7],[0,3,7,4],
                 [4,7,8,5],[5,8,9,6]]);
        translate([0,0,-1]) cylinder(d=m4_clear,h=14);
    }
}

module rear_foot_wedge_installed(side=1) {
    translate([side_frame_rear_x,side*side_frame_mount_y,39])
        rear_foot_wedge_washer();
}

// Locates from both original P2-79 Ø4.5 holes/bosses.  The guide tube is at
// X-132 and is the only approved way to drill the already-printed base.
module side_frame_base_drill_jig() {
    difference() {
        union() {
            translate([(side_frame_rear_x+side_frame_front_x)/2,0,4])
                cube([side_frame_front_x-side_frame_rear_x+18,18,8],center=true);
            // The guide nose reaches the base top, 5 mm below the old boss
            // tops on which the jig beam sits.
            translate([side_frame_front_x,0,-5]) cylinder(d=13,h=21);
        }
        for (x=[side_frame_rear_x,panel_center_x+58])
            translate([x,0,-1]) cylinder(d=m4_clear,h=12);
        translate([side_frame_front_x,0,-6]) cylinder(d=m4_clear,h=23);
    }
}

module ruthex_m3_vertical_bore(z_top=0,through=20) {
    // Insert enters from +Z; the smaller continuation opens to the screw
    // below.  Tensile clamp load therefore pulls the insert into the part.
    translate([0,0,z_top-ruthex_m3_depth])
        cylinder(d=ruthex_m3_hole,h=ruthex_m3_depth+0.2);
    translate([0,0,z_top-through])
        cylinder(d=m3_clear,h=through-ruthex_m3_depth+0.2);
}

// Fixed Parkside rail/dock.  It uses two existing base slots at Y±145,
// biased 1.5 mm outward so the screw heads clear the Y142 rear rib.  The
// drawer and battery dimensions remain explicit provisional parameters.
module parkside_dock(side=1) {
    sy=side;
    difference() {
        union() {
            // Outer structural spine and inner guide, joined at +X stop.
            translate([parkside_center_x,sy*151,64]) cube([133,7,13],center=true);
            translate([parkside_center_x,sy*80,64]) cube([133,5,13],center=true);
            translate([-132,sy*115.5,75]) cube([7,76,35],center=true);
            // Electrical contact carrier with two parametric M3 lands.
            translate([-136,sy*115.5,98]) cube([10,42,38],center=true);
            // Rear positive-lock boss, accessible from the open Parkside door.
            translate([-260,sy*153,76]) cube([16,10,16],center=true);
        }
        // Main mounting inserts are installed from above before the drawer.
        for (x=parkside_mount_x)
            translate([x,sy*146.5,70.5])
                ruthex_m3_vertical_bore(0,14);
        // Contact-module adjustment slots (module TBD after measurement).
        for (z=[88,108]) translate([-142,sy*115.5,z]) rotate([0,90,0])
            cylinder(d=m3_clear,h=16);
        // Horizontal Ruthex lock insert; moderate-load retention only.
        translate([-268,sy*153,76]) rotate([0,90,0])
            cylinder(d=ruthex_m3_hole,h=ruthex_m3_depth+0.2);
    }
}

module parkside_drawer(side=1) {
    sy=side;
    difference() {
        union() {
            translate([parkside_center_x,sy*115.5,parkside_drawer_floor_z+2])
                cube([parkside_drawer_outer[0],parkside_drawer_outer[1],4],center=true);
            for (yy=[-1,1])
                translate([parkside_center_x,sy*115.5+yy*sy*32,
                           parkside_drawer_floor_z+33.5])
                    cube([parkside_drawer_outer[0],3,67],center=true);
            // Rear pull/lock ear, kept inside the accepted envelope.
            translate([-264,sy*153,76]) cube([8,10,16],center=true);
        }
        // Rear-to-front M3 lock screw access.
        translate([-273,sy*153,76]) rotate([0,90,0]) cylinder(d=m3_clear,h=22);
        // +X contact opening and cable relief.
        translate([-139,sy*115.5,98]) cube([18,36,34],center=true);
    }
}

// P2-90 centre-corridor UGREEN cradle.  The already-printed posts cannot pass
// a 115 mm device inserted from +Y (about 93 mm clear).  In the centre corridor it
// slides +X from the removable rear_hi opening, beside both rear posts.  Ports
// face -X and the button faces +Y.  One collar on each rear post carries the
// saddle; the rear retainer is removable and the strap is secondary only.
module ugreen_post_holder() {
    difference() {
        union() {
            // Floor stops short of the front-negative post; the device may
            // overhang 2 mm while its +X stop uses the clear centre region.
            translate([-210,ugreen_center[1],233]) cube([113,51,5],center=true);
            for (sy=[-1,1])
                translate([-210,ugreen_center[1]+sy*24,241])
                    cube([113,3,16],center=true);
            translate([-152,42,233]) cube([5,28,5],center=true);
            translate([-150,42,248.5]) cube([3,28,36],center=true);

            // One lower collar on each rear post; wide Y spacing reacts the
            // saddle moment while all connecting arms remain below Z235.5.
            for (s=[-1,1]) let(py=s>0?90:-56) {
                translate([panel_center_x-58,py,225]) cylinder(d=31,h=18,center=true);
                hull() {
                    translate([panel_center_x-58,py-s*12,230]) cube([18,10,8],center=true);
                    translate([panel_center_x-58,ugreen_center[1]+s*25,230])
                        cube([28,5,8],center=true);
                }
            }
            // Rear-retainer insert bosses sit outside the 45 mm device width.
            for (sy=[-1,1])
                translate([-259.5,ugreen_center[1]+sy*31,242])
                    cube([14,11,24],center=true);
        }
        // 0.4 mm assembly clearance on each device face.
        translate(ugreen_center) cube([ugreen_installed_size[0]+0.8,
                                       ugreen_installed_size[1]+0.8,
                                       ugreen_installed_size[2]+0.8],center=true);
        // Entire -X face stays open until the separate retainer is fitted.
        translate([-274,ugreen_center[1],258]) cube([18,47,47],center=true);

        // Rear printed-post seats split outward.  Clamp screws approach from
        // X and terminate in purchased Ruthex RX-M3x5.7 inserts.
        for (s=[-1,1]) let(py=s>0?90:-56) {
            translate([panel_center_x-58,py,225]) cylinder(d=23.6,h=20,center=true);
            translate([panel_center_x-58,py+s*8,225]) cube([7,22,22],center=true);
            translate([panel_center_x-72,py-s*12,230]) rotate([0,90,0])
                cylinder(d=m3_clear,h=28);
            translate([panel_center_x-54,py-s*12,230]) rotate([0,90,0])
                cylinder(d=ruthex_m3_hole,h=ruthex_m3_depth+0.2);
        }
        // M3 rear-retainer screws and two strap passages.  Neither strap is
        // credited as the primary structural attachment.
        for (sy=[-1,1]) {
            translate([-266.4,ugreen_center[1]+sy*31,242])
                rotate([0,90,0]) cylinder(d=ruthex_m3_hole,h=ruthex_m3_depth+0.2);
            translate([-210,ugreen_center[1]+sy*20,233]) cube([12,8,8],center=true);
        }
    }
}

module ugreen_rear_retainer() {
    difference() {
        union() {
            // Low bridge seats on the rear face; ports and side button remain
            // exposed. 0.1 mm axial fit avoids a false coplanar solid.
            translate([-268.1,ugreen_center[1],242]) cube([3,68,10],center=true);
            for (sy=[-1,1])
                translate([-268.1,ugreen_center[1]+sy*31,242])
                    cube([3,10,18],center=true);
        }
        for (sy=[-1,1])
            translate([-273,ugreen_center[1]+sy*31,242])
                rotate([0,90,0]) cylinder(d=m3_clear,h=6);
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
    color([.15,.65,.95,.65]) translate([panel_center_x+42,70,panel_z-6]) cylinder(d=7.8,h=12,center=true);
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

module p288_access_hardware() {
    // Side-frame front: head/washer above, nyloc below, 60 mm Allen path.
    for (side=[-1,1]) {
        color([.75,.75,.78]) {
            translate([side_frame_front_x,side*side_frame_mount_y,68])
                cylinder(d=9,h=4);
            translate([side_frame_front_x,side*side_frame_mount_y,44])
                cylinder(d=8,h=7);
            translate([side_frame_front_x,side*side_frame_mount_y,72])
                cylinder(d=6,h=60);
            // Rear: low head below wedge, nyloc and wrench space above pad.
            translate([side_frame_rear_x,side*side_frame_mount_y,35])
                cylinder(d=9,h=4);
            translate([side_frame_rear_x,side*side_frame_mount_y,68])
                cylinder(d=8,h=7);
            translate([side_frame_rear_x,side*side_frame_mount_y,68])
                cylinder(d=18,h=20);
        }
        // Dock M3 heads/washers below the two existing slots and tool path.
        for (x=parkside_mount_x) {
            translate([x,side*146.5,46]) cylinder(d=6.5,h=4);
            translate([x,side*146.5,-14]) cylinder(d=5,h=60);
        }
        // Positive drawer lock and 60 mm driver path from the rear.
        translate([-273,side*153,76]) rotate([0,90,0]) cylinder(d=6,h=5);
        translate([-333,side*153,76]) rotate([0,90,0]) cylinder(d=5,h=60);
    }
    // UGREEN: two post collars plus two rear-retainer screws, all approached
    // through the removable rear_hi aperture.
    for (p=[[78,230],[-44,230]])
        translate([-332,p[0],p[1]]) rotate([0,90,0]) cylinder(d=5,h=59.9);
    for (p=[[3,242],[65,242]])
        translate([-332,p[0],p[1]]) rotate([0,90,0]) cylinder(d=5,h=62.3);
}

module p288_new_parts_installed() {
    for (side=[-1,1]) {
        rear_foot_wedge_installed(side);
        parkside_dock(side);
        parkside_drawer(side);
    }
    ugreen_post_holder();
    ugreen_rear_retainer();
}

module p288_battery_ghosts() {
    for (side=[-1,1]) color([.2,.7,.2,.45])
        translate([parkside_center_x,side*parkside_center_y,
                   parkside_drawer_floor_z+4+parkside_battery_provisional[2]/2])
            cube(parkside_battery_provisional,center=true);
    color([.15,.2,.3,.55]) translate(ugreen_center)
        cube(ugreen_installed_size,center=true);
}

module qa_p288_reservation_overflow() {
    difference() {
        union() { for (side=[-1,1]) { parkside_dock(side); parkside_drawer(side); } }
        union() { for (side=[-1,1])
            translate([parkside_center_x,side*parkside_center_y,parkside_center_z-0.25])
                cube([parkside_clear[0],parkside_clear[1],parkside_clear[2]+0.5],center=true); }
    }
}

module qa_p288_ugreen_device_collision() {
    intersection() {
        union(){ ugreen_post_holder(); ugreen_rear_retainer(); }
        translate(ugreen_center) cube(ugreen_installed_size,center=true);
    }
}

module p288_preview() {
    assembly();
    color([.75,.38,.08]) p288_new_parts_installed();
    p288_battery_ghosts();
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
    translate([panel_center_x+42,70,panel_z-6]) cylinder(d=7.8,h=12,center=true);
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
    translate([panel_center_x+42,70,panel_z-6-eps/2])
        cylinder(d=7.8,h=12-eps,center=true);
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

module qa_base_parkside_collision() {
    intersection() {
        union() {
            translate([0,0,pod_z_bottom]) base_half_v2(-1);
            translate([0,0,pod_z_bottom]) base_half_v2(1);
        }
        for (side=[-1,1])
            translate([parkside_center_x,side*parkside_center_y,parkside_center_z])
                cube(parkside_clear,center=true);
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
assert(status_led_hole == 8,
       "P2-84 status LED aperture must remain Ø8");
assert(ruthex_m3_length == 5.7 && ruthex_m3_od == 4.6 &&
       ruthex_m3_hole == 4.0 && ruthex_m3_depth >= 6.7,
       "P2-88 purchased Ruthex RX-M3x5.7 dimensions changed");
assert(ruthex_m3_boss_d >= ruthex_m3_od+4,
       "P2-88 needs >=2 mm PETG around every M3 insert");

if (part=="base_left") base_half_v2(1);
else if (part=="base_right") base_half_v2(-1);
else if (part=="pi_shelf") pi_shelf_v2();
else if (part=="vertical_electronics_plate") vertical_electronics_plate();
else if (part=="side_frame") rotate([90,0,0]) side_frame();
else if (part=="rear_foot_wedge_washer") rear_foot_wedge_washer();
else if (part=="side_frame_base_drill_jig") side_frame_base_drill_jig();
else if (part=="parkside_dock_left") rotate([0,0,0]) parkside_dock(1);
else if (part=="parkside_dock_right") rotate([0,0,0]) parkside_dock(-1);
else if (part=="parkside_drawer_left") parkside_drawer(1);
else if (part=="parkside_drawer_right") parkside_drawer(-1);
else if (part=="ugreen_post_holder") ugreen_post_holder();
else if (part=="ugreen_rear_retainer") ugreen_rear_retainer();
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
else if (part=="qa_base_parkside_collision") qa_base_parkside_collision();
else if (part=="qa_control_bodies_collision") qa_control_bodies_collision();
else if (part=="qa_control_ugreen_collision") qa_control_ugreen_collision();
else if (part=="qa_control_posts_collision") qa_control_posts_collision();
else if (part=="qa_control_posts_penetration") qa_control_posts_penetration();
else if (part=="qa_control_carrier_collision") qa_control_carrier_collision();
else if (part=="p281_post_collision_debug") p281_post_collision_debug();
else if (part=="p281_exploded_preview") p281_exploded_preview();
else if (part=="qa_p288_reservation_overflow") qa_p288_reservation_overflow();
else if (part=="qa_p288_ugreen_device_collision") qa_p288_ugreen_device_collision();
else if (part=="p288_preview") p288_preview();
else if (part=="envelope") envelope_ghost();
else if (part=="assembly") assembly();
else { envelope_ghost(); assembly(); }
