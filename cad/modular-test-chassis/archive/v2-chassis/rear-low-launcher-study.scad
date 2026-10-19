// ALTERNATIVE PACKAGING STUDY ONLY; no manufacturing STL or strength approval.
// Existing front intake and accepted 20-degree launcher geometry unchanged.
// mm, +X front. Whole launcher yawed 180 degrees to eject towards the rear.
$fn=32;
use <modular-chassis-v2.scad>
use <height-infrastructure-study.scad>
use <../../../flywheel-launcher-v0/direct-drive-mechanical-definition-study.scad>

view="assembly"; // [assembly,side,top,launcher,qa_battery_hopper,qa_launcher_battery,qa_launcher_hopper,qa_exit_frame,qa_feed_hardware,qa_feed_battery,qa_chute_hardware]
battery_layout="center"; // [center,side_l]
mount_mode="clear"; // [clear,target20] target20 intentionally clashes with fixed brace
show_path=true;
show_context=true;
pitch=20;
nip_x=-70; // keeps both cradle crossbars supported by existing side rails
nip_z=175;
ball_d=66;
feed_local_x=-185; // outside the upper panel edge plus ball-radius clearance
// Old battery packaging envelope [166,198,170], laid on another face ONLY.
// User dimensions confirmed; permitted orientation/terminals still to check.
battery_size=[198,170,166];
battery_mass_kg=5.59;
ground_clearance=20; // underside of strips; NOT verified under full load
strip_bottom=mount_mode=="target20" ? ground_clearance : (battery_layout=="side_l" ? 54 : 43); // clear brace42 or ramp sidewall53 by1mm
strip_t=3; // packaging thickness, NOT a load-rated printed support
battery_center_x=battery_layout=="side_l" ? 275 : 245;
battery_center_y=battery_layout=="side_l" ? 85 : 0;
battery_bottom=strip_bottom+strip_t;
basket_floor_rear=battery_layout=="side_l" ? 145 : battery_bottom+166+5;
basket_floor_front=basket_layout_front();
basket_top=battery_layout=="side_l" ? 300 : 335;
function basket_layout_front()=battery_layout=="side_l" ? 165 : basket_floor_rear+48;
intake_exit_z=35; // existing Option A handoff surface, not ball centre
basket_entry_z=basket_floor_front+3; // upper surface of floor at front
funnel_top=basket_floor_rear+3+(116-80)*(basket_floor_front-basket_floor_rear)/(battery_layout=="side_l" ? 294 : 230);
lift_required=basket_entry_z-intake_exit_z;
function pose(p)=[nip_x-cos(pitch)*p[0]+sin(pitch)*p[2],-p[1],
                  nip_z+sin(pitch)*p[0]+cos(pitch)*p[2]];
feed=pose([feed_local_x,0,0]);
assert(battery_layout=="center" || battery_layout=="side_l");
assert(battery_layout!="center" || basket_floor_rear>=battery_bottom+battery_size[2]+5);
echo("STUDY_ONLY",pitch_deg=pitch,nip_mm=[nip_x,0,nip_z],
     battery_top_mm=battery_bottom+battery_size[2],
     battery_layout=battery_layout,battery_mass_kg=battery_mass_kg,
     strip_bottom_mm=strip_bottom,mount_mode=mount_mode,
     basket_entry_lift_mm=lift_required,feed_ball_center_mm=feed);

module launcher_pose() {
    translate([nip_x,0,nip_z]) rotate([0,0,180]) rotate([0,-pitch,0]) children();
}
module launcher_hardware() {
    // Exact existing panels, bolt axes and envelopes reused, not mirrored parts.
    launcher_pose() {
        color([0.35,0.40,0.46]) {
            launcher_panel_reference(false); launcher_panel_reference(true);
        }
        for (yy=[-129,129]) { wheel(yy); motor_and_shaft(yy); unresolved_hub_zone(yy); }
        for (xx=[-115,115],yy=[-145,145]) color("silver")
            translate([xx,yy,-39]) difference() {
                cylinder(d=12,h=78);
                translate([0,0,-1]) cylinder(d=3.3,h=80);
            }
    }
}
module launcher_cradle() {
    color([0.60,0.35,0.65]) {
        launcher_pose() for (side=[-1,1]) lower_plate_lock_rail(side);
        // Supports below the tilted lower panel, outside the central ball path.
        for (xx=[-100,70]) {
            p=pose([xx,0,-57]);
            assert(p[0]>=-165 && p[0]<=455,"Cradle root outside current chassis");
            translate([p[0],0,p[2]]) rotate([0,0,90]) horizontal_x(440);
            for (yy=[-220,220]) translate([p[0],yy,52]) tube_z(p[2]-52);
        }
    }
}
module battery_envelope() {
    translate([battery_center_x,battery_center_y,battery_bottom+battery_size[2]/2])
        cube(battery_size,center=true);
}
module battery_tray_reference() {
    // Two thin suspended strip reservations; not verified for 5.59 kg.
    color([0.55,0.55,0.55]) for (xx=[battery_center_x-70,battery_center_x+70]) {
        translate([xx,0,strip_bottom+strip_t/2])
            cube([20,380,strip_t],center=true);
        for (yy=[-190,190]) hull() {
            translate([xx,yy,strip_bottom+strip_t/2]) cube([20,10,strip_t],center=true);
            translate([xx,yy>0 ? 210 : -210,50]) cube([20,10,4],center=true);
        }
    }
}
// Battery bay excluded from the entire L footprint, including its upper walls.
l_outer=[[80,-180],[374,-180],[374,-10],[166,-10],[166,180],[80,180]];
l_inner=[[83,-177],[371,-177],[371,-13],[163,-13],[163,177],[83,177]];
module sloped_l_prism(points,bottom,top) {
    n=len(points);
    vertices=concat([for(p=points) [p[0],p[1],bottom+(p[0]-80)*20/294]],
                    [for(p=points) [p[0],p[1],top]]);
    faces=concat([[for(i=[n-1:-1:0]) i]],
        [[for(i=[0:n-1]) i+n]],
        [for(i=[0:n-1]) [i,(i+1)%n,(i+1)%n+n,i+n]]);
    polyhedron(points=vertices,faces=[for(f=faces) [for(j=[len(f)-1:-1:0]) f[j]]]);
}
module hopper_outer() {
    if (battery_layout=="side_l") sloped_l_prism(l_outer,basket_floor_rear,basket_top);
    else hull() {
        translate([195,0,basket_top]) cube([270,370,2],center=true);
        // Tilted floor: rear lower, front higher.
        translate([80,0,basket_floor_rear]) cube([2,190,2],center=true);
        translate([310,0,basket_floor_front]) cube([2,190,2],center=true);
    }
}
module hopper() {
    color([0.25,0.65,0.35,0.55]) difference() {
        hopper_outer();
        if (battery_layout=="side_l") sloped_l_prism(l_inner,basket_floor_rear+3,basket_top+5);
        else hull() {
            translate([195,0,basket_top+4]) cube([264,364,10],center=true);
            translate([83,0,basket_floor_rear+3]) cube([2,184,2],center=true);
            translate([307,0,basket_floor_front+3]) cube([2,184,2],center=true);
        }
        translate([124,0,funnel_top]) cube([78,84,40],center=true);
    }
}
module feed_chute() {
    // Open gravity funnel ending OUTSIDE panel input edge. Needs an escapement.
    color([0.25,0.65,0.35,0.55]) difference() {
        hull() {
            translate([116,0,funnel_top]) cube([90,90,3],center=true);
            translate([feed[0],0,feed[2]+4]) cube([80,80,3],center=true);
        }
        hull() {
            translate([116,0,funnel_top+5]) cube([84,84,14],center=true);
            translate([feed[0],0,feed[2]]) cube([74,74,14],center=true);
        }
    }
    // Short inclined launch-entry guide; uphill by the unchanged 20-degree pitch.
    color([0.25,0.65,0.35]) launcher_pose()
        translate([-159,0,-35]) cube([62,82,4],center=true);
    // BLUE reserve: one-ball escapement/pusher not yet mechanically designed.
    %color([0.1,0.65,0.95]) translate([feed[0],140,feed[2]-20])
        cube([45,90,50],center=true);
}
module feed_swept_balls() {
    hull() {
        translate([116,0,funnel_top+33]) sphere(d=ball_d);
        translate(feed) sphere(d=ball_d);
    }
}
module entry_lift_reference() {
    // ORANGE ghost is a missing active lift, not a solved gravity-only ramp.
    %color([1,0.45,0.08,0.4]) hull() {
        translate([405,battery_layout=="side_l" ? -95 : 0,intake_exit_z+33]) cube([35,76,66],center=true);
        translate([405,battery_layout=="side_l" ? -95 : 0,basket_entry_z+33]) cube([35,76,66],center=true);
    }
}
module exit_keepout() {
    launcher_pose() translate([190,0,0]) rotate([0,90,0]) cylinder(d=76,h=340,center=true);
}
module basket_supports() {
    color([0.15,0.60,0.65]) for (xx=[80,300],yy=[-190,190]) {
        translate([xx,yy,52]) tube_z(basket_top-52);
        hull() {
            translate([xx,yy,52]) cube([20,20,6],center=true);
            translate([xx,yy>0 ? 220 : -220,52]) cube([20,20,6],center=true);
        }
    }
}
module context() {
    frame_structure(); intake_context();
    for (side=[-1,1],xx=[-65,300]) wheel_context(side,xx);
}
module layout() {
    if (show_context) context();
    launcher_hardware(); launcher_cradle(); basket_supports();
    battery_tray_reference(); color([0.15,0.35,0.75,0.5]) battery_envelope();
    hopper(); feed_chute(); entry_lift_reference();
    if (show_path) {
        %color([0.1,0.65,0.95,0.25]) exit_keepout();
        %color([0.1,0.65,0.95,0.25]) feed_swept_balls();
        launcher_pose() color([0.1,0.65,0.95]) sphere(d=ball_d);
    }
}
if (view=="qa_battery_hopper") intersection() { battery_envelope(); hopper_outer(); }
else if (view=="qa_launcher_battery") intersection() { launcher_hardware(); battery_envelope(); }
else if (view=="qa_launcher_hopper") intersection() { launcher_hardware(); hopper(); }
else if (view=="qa_exit_frame") intersection() {
    exit_keepout(); union() { frame_structure(); launcher_cradle(); basket_supports(); battery_tray_reference(); }
}
else if (view=="qa_feed_hardware") intersection() { feed_swept_balls(); launcher_hardware(); }
else if (view=="qa_feed_battery") intersection() { feed_swept_balls(); battery_envelope(); }
else if (view=="qa_chute_hardware") intersection() { feed_chute(); launcher_hardware(); }
else if (view=="qa_battery_frame") intersection() {
    battery_envelope(); union() { frame_structure(); intake_context(); launcher_cradle(); }
}
else if (view=="side") intersection() { layout(); translate([-600,-600,-10]) cube([1200,600,650]); }
else if (view=="launcher") { launcher_hardware(); launcher_cradle(); feed_chute(); }
else layout();
