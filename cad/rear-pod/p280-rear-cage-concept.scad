// P2-80 analysis-only rear cage shared by rear pod v2 and body concept F.
// No printable part selectors and no STL release. Units mm.

use <rear-pod-v2.scad>
use <../modular-test-chassis/modular-chassis-v3.scad>
use <../body-panels/body-panels-v3.scad>
use <../body-panels/body-panels-f.scad>

p280_view = "assembly";
p280_angle = 0;
show_context = true;
$fn = 48;

// Hybrid cage proposal. Four 20x3 aluminium flats fit in the only continuous
// vertical corridors left by the Parkside drawers while staying inside Y+/-167.
cage_x = [-180,-150];
cage_y = 165.5;
cage_z0 = 58;
cage_zt = 343;
al_flat_x = 20;
al_flat_y = 3;
frame_z0 = 343;
frame_t = 4;

module rod_between(a,b,d=4) {
    hull() { translate(a) sphere(d=d); translate(b) sphere(d=d); }
}

module cage_column(x,side) {
    color([.68,.72,.76]) translate([x,side*cage_y,(cage_z0+cage_zt)/2])
        cube([al_flat_x,al_flat_y,cage_zt-cage_z0],center=true);
}

module cage_base_socket(side) {
    // Full-width saddle bears on rear-crossbar top Z52. Existing top M4 at
    // Y75/Y125 clamp it; no new chassis hole. The outboard lug shares the
    // X-135 corner-clamp side M4 with a longer screw.
    color([.90,.53,.10]) difference() {
        union() {
            translate([-165,side*121,54.775]) cube([50,98,5.45],center=true);
            // Stay above the Z52 rail, then turn down only outside its Y245
            // face. This avoids both the Parkside drawer and chassis solids.
            translate([-142.5,side*206.5,54.8]) cube([15,77,5.4],center=true);
            translate([-135,side*248.1,40]) cube([20,5.8,30],center=true);
            for(x=cage_x) translate([x,side*cage_y,57.8])
                cube([al_flat_x+4,al_flat_y+4,0.4],center=true);
        }
        for(y=[75,125]) translate([-165,side*y,50]) cylinder(d=4.5,h=12);
        translate([-135,side*242,37]) rotate([-side*90,0,0]) cylinder(d=4.5,h=20);
    }
}

module cage_top_frame() {
    color([.68,.72,.76]) {
        // Outer rectangular frame stops at X-267, keeping >3 mm inside the
        // F R424.5 skin and within the frozen pod envelope.
        for(side=[-1,1])
            translate([(-267-125)/2,side*cage_y,frame_z0+frame_t/2])
                cube([142,al_flat_y,frame_t],center=true);
        for(x=[-267,-130])
            translate([x,0,frame_z0+frame_t/2]) cube([6,331,frame_t],center=true);

        // Panel mounting rails use its existing X+/-58 M4 positions. Two
        // extra rails flank the control bodies and turn the 4 mm panel into
        // short plate bays without changing the panel or its apertures.
        for(x=[-256.5,-140.5,-222.5,-174.5])
            translate([x,0,frame_z0+frame_t/2]) cube([6,331,frame_t],center=true);
    }
}

module cage_front_x_brace() {
    // Preloaded M4 steel tension rods in the X=-127 plane. They begin at Z145,
    // 6 mm above the Parkside clear envelopes, and stay ahead of Ugreen/BTS.
    color([.25,.27,.30]) {
        rod_between([-127,-cage_y,145],[-127,cage_y,cage_zt],4);
        rod_between([-127,cage_y,145],[-127,-cage_y,cage_zt],4);
        for(side=[-1,1]) rod_between([-150,side*cage_y,145],[-127,side*cage_y,145],6);
    }
}

module p280_cage() {
    for(side=[-1,1]) {
        cage_base_socket(side);
        for(x=cage_x) cage_column(x,side);
    }
    cage_top_frame();
    cage_front_x_brace();
}

module p280_columns() { for(side=[-1,1]) for(x=cage_x) cage_column(x,side); }
module p280_bases() { for(side=[-1,1]) cage_base_socket(side); }

module p280_drawer_sweeps() {
    for(side=[-1,1])
        translate([-364,side*115.5,98.5]) cube([472,91,81],center=true);
}

module p280_contents() {
    reserved_ghosts(); electronics_ghosts(); controls_ghosts();
}

module p280_pod_context() {
    color([.28,.52,.70]) carrier_installed();
    color([.18,.45,.72]) translate([-212,0,64]) pi_shelf_v2();
    color([.18,.45,.72]) translate([-198.5,0,347]) control_panel_v2();
    controls_ghosts(); reserved_ghosts(); electronics_ghosts();
}

module p280_scene() {
    if(show_context) {
        color([.20,.42,.68,.22]) frame_assembly();
        p280_pod_context();
        tub();
    }
    p280_cage();
}

module p280_keep_radius() {
    translate([117,0,0]) cylinder(r=421.5,h=500,$fn=720);
}

if(p280_view=="assembly") p280_scene();
else if(p280_view=="rear") rotate([90,0,0]) p280_scene();
else if(p280_view=="side") rotate([90,0,90]) p280_scene();
else if(p280_view=="top") rotate([0,0,0]) p280_scene();
else if(p280_view=="cage") p280_cage();
else if(p280_view=="qa_chassis") intersection(){ p280_cage(); frame_assembly(); }
else if(p280_view=="qa_chassis_columns") intersection(){ p280_columns(); frame_assembly(); }
else if(p280_view=="qa_chassis_bases") intersection(){ p280_bases(); frame_assembly(); }
else if(p280_view=="qa_chassis_top") intersection(){ cage_top_frame(); frame_assembly(); }
else if(p280_view=="qa_chassis_braces") intersection(){ cage_front_x_brace(); frame_assembly(); }
else if(p280_view=="qa_moving") intersection(){ p280_cage(); union(){ for(a=[-10:1:20]) bp_moving(a); } }
else if(p280_view=="qa_contents") intersection(){ p280_cage(); p280_contents(); }
else if(p280_view=="qa_contents_columns") intersection(){ p280_columns(); p280_contents(); }
else if(p280_view=="qa_contents_bases") intersection(){ p280_bases(); p280_contents(); }
else if(p280_view=="qa_contents_top") intersection(){ cage_top_frame(); p280_contents(); }
else if(p280_view=="qa_contents_braces") intersection(){ cage_front_x_brace(); p280_contents(); }
else if(p280_view=="qa_drawers") intersection(){ p280_cage(); p280_drawer_sweeps(); }
else if(p280_view=="qa_panels_f") intersection(){ p280_cage(); tub_solid(); }
else if(p280_view=="qa_turn") difference(){ p280_cage(); p280_keep_radius(); }
else if(p280_view=="qa_ground") intersection(){ p280_cage(); translate([-400,-300,-10]) cube([800,600,27]); }
else assert(false,str("unknown P2-80 view: ",p280_view));
