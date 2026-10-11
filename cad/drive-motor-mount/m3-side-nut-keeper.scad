// Removable side-entry M3 nut keeper. Existing housings are unchanged.
// Trial fit only: no positive latch exists in the old channel.
// Print flat, Z=0 on bed. +Y points into the existing nut channel.
$fn=48;
use <../../engineering/chassis-strength-bench/coupon/drive-motor-inline-module.scad>
use <../modular-test-chassis/modular-chassis-v2.scad>

part="sleeve"; // [motor_short,motor_long,sleeve,preview_sleeve,preview_short,preview_long,qa_motor,qa_sleeve,qa_front_sleeve,qa_nut]
nut_af=5.5; // assumed measured metal nut size; adjust after caliper measurement
nut_clearance=0.20; // total clearance across flats, not per face
channel_af=6.2;
channel_corner_d=channel_af/cos(30);
channel_seat_w=channel_corner_d+0.20;
channel_entry_w=channel_corner_d+0.40;
channel_h=3.7;
side_clearance=0.15;
vertical_clearance=0.20;
keeper_h=channel_h-2*vertical_clearance;
tip_before_axis=1.25; // keep the fork thick; no plastic across screw axis
tab_w=9.5;
tab_depth=2.5;

module keeper(distance=11) {
    tip=distance-tip_before_axis;
    function width_at(y)=channel_seat_w+(channel_entry_w-channel_seat_w)*(distance-y)/(distance+1)-2*side_clearance;
    difference() {
        union() {
            linear_extrude(height=keeper_h)
                polygon([[-width_at(0)/2,-0.05],[width_at(0)/2,-0.05],
                         [width_at(tip)/2,tip],[-width_at(tip)/2,tip]]);
            translate([-tab_w/2,-tab_depth,0]) cube([tab_w,tab_depth,keeper_h]);
        }
        // A through-height near-half hex notch: keeper carries no bolt preload.
        translate([0,distance,-0.1])
            cylinder(d=(nut_af+nut_clearance)/cos(30),h=keeper_h+0.2,$fn=6);
    }
}

module motor_keepers() {
    for (xx=[-15,15]) {
        translate([xx,-25,22.3+vertical_clearance]) keeper(11);
        translate([xx,25,22.3+vertical_clearance]) rotate([0,0,180]) keeper(14);
    }
}

module sleeve_keepers() {
    for (xx=[-30,30],side=[-1,1])
        translate([xx,side*21.5,15.3+vertical_clearance])
            rotate([0,0,side>0 ? 180 : 0]) keeper(9);
}

module front_sleeve_keepers() {
    for (yy=[-15,15],side=[-1,1])
        translate([side*21.5,yy,5.3+vertical_clearance])
            rotate([0,0,side*90]) keeper(9);
}

if (part=="motor_short") keeper(11);
else if (part=="motor_long") keeper(14);
else if (part=="sleeve") keeper(9);
else if (part=="preview_sleeve" || part=="preview_short" || part=="preview_long") {
    distance=part=="preview_sleeve" ? 9 : part=="preview_short" ? 11 : 14;
    color([0.15,0.55,0.8]) keeper(distance);
    color([0.65,0.65,0.65]) translate([0,distance,0.45])
        cylinder(d=nut_af/cos(30),h=2.4,$fn=6);
}
else if (part=="qa_motor") intersection() { drive_motor_inline(1); motor_keepers(); }
else if (part=="qa_sleeve") intersection() { straight_sleeve(); sleeve_keepers(); }
else if (part=="qa_front_sleeve") intersection() { ramp_back_sleeve(); front_sleeve_keepers(); }
else if (part=="qa_nut") intersection() {
    keeper(9);
    translate([0,9,-0.1]) cylinder(d=nut_af/cos(30),h=channel_h+0.2,$fn=6);
}
