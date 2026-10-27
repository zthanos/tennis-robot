// P2-83 exact-solid and review views for the one-piece inner corner tie.
use <rear-pod-v2.scad>
use <../modular-test-chassis/modular-chassis-v3.scad>

qa = "assembly";

module p283_ties() {
    corner_tie_installed(-1);
    corner_tie_installed(1);
}

module p283_bases() {
    translate([0,0,pod_z_bottom]) base_half_v2(-1);
    translate([0,0,pod_z_bottom]) base_half_v2(1);
}

module p283_drawer_sweeps() {
    for(side=[-1,1])
        translate([-364,side*115.5,98.5]) cube([472,91,81],center=true);
}

// Rear pair of the measured/fit-checked drive package envelopes used by the
// Phase-1/P2-78 QA: FIT0403 mount, motor/encoder/cable/outer stack and wheel.
module p283_rear_drive_packages() {
    for(side=[-1,1]) {
        color([.05,.06,.07,.45])
            translate([-65,side*297,85]) rotate([90,0,0])
                difference(){ cylinder(d=170,h=83,center=true); cylinder(d=36,h=85,center=true); }
        color([.72,.74,.78,.55]) {
            translate([-65,side*220,53]) cube([40,40,2],center=true);
            translate([-65,side*239,77.5]) cube([40,2,47],center=true);
        }
        color([.20,.22,.25,.55]) {
            translate([-65,side*226.5,85]) rotate([90,0,0]) cylinder(d=40,h=25,center=true);
            translate([-65,side*193,85]) rotate([90,0,0]) cylinder(d=38,h=42,center=true);
            translate([-65,side*166,85]) rotate([90,0,0]) cylinder(d=40,h=12,center=true);
            translate([-65,side*264,85]) rotate([90,0,0]) cylinder(d=6,h=48,center=true);
            translate([-65,side*280,85]) rotate([90,0,0]) cylinder(d=30,h=24,center=true,$fn=6);
        }
        color([.92,.72,.12,.55]) {
            translate([-65,side*166,61]) cube([18,14,18],center=true);
            hull(){
                translate([-65,side*166,53]) cube([8,8,8],center=true);
                translate([-65,side*220,16]) cube([8,8,8],center=true);
            }
        }
    }
}

module p283_fasteners() {
    for(side=[-1,1]) {
        // M4 90-degree flat head: top is flush at Z57.5.
        translate([-134,side*145,55.15])
            cylinder(h=2.35,d1=4.5,d2=8.8);
        translate([-134,side*145,45]) cylinder(d=4,h=10.2);
        // Shared rail-side M4 head inside; washer/nyloc remain outside.
        translate([-135,side*187,37]) rotate([90,0,0])
            cylinder(d=8,h=4,center=true);
        translate([-135,side*220,37]) rotate([90,0,0])
            cylinder(d=4,h=62,center=true);
        translate([-135,side*194.5,37]) rotate([90,0,0])
            cylinder(d=9,h=1,center=true);
        translate([-135,side*248,37]) rotate([90,0,0])
            cylinder(d=8,h=6,center=true);
    }
}

module p283_assembly() {
    color([.22,.46,.72,.25]) frame_assembly();
    p283_rear_drive_packages();
    color([.82,.56,.12,.75]) p283_bases();
    color([.92,.28,.08]) p283_ties();
    color([.30,.30,.32]) p283_fasteners();
    color([.10,.58,.22,.16]) p283_drawer_sweeps();
}

module p283_detail() {
    region = [-205,125,15];
    size = [100,140,55];
    color([.22,.46,.72,.35]) intersection(){ frame_assembly(); translate(region) cube(size); }
    intersection(){ p283_rear_drive_packages(); translate(region) cube(size); }
    color([.82,.56,.12,.82]) intersection(){ p283_bases(); translate(region) cube(size); }
    color([.92,.28,.08]) corner_tie_installed(1);
    color([.30,.30,.32]) intersection(){ p283_fasteners(); translate(region) cube(size); }
    color([.10,.58,.22,.12]) intersection(){ p283_drawer_sweeps(); translate(region) cube(size); }
}

if(qa=="assembly") p283_assembly();
else if(qa=="detail") p283_detail();
else if(qa=="tie_base") intersection(){ p283_ties(); p283_bases(); }
else if(qa=="tie_chassis") intersection(){ p283_ties(); union(){ frame_assembly(); p283_rear_drive_packages(); } }
else if(qa=="drawers") intersection(){ union(){ p283_ties(); p283_fasteners(); } p283_drawer_sweeps(); }
else if(qa=="ground") intersection(){ p283_ties(); translate([-400,-300,-100]) cube([800,600,115]); }
else assert(false,str("unknown P2-83 QA: ",qa));
