// P2-74 dependent-only selector-rail study.
// Only the P2-68 selector-nose rail Y positions may change.
// Units mm; +X front, +Y left, +Z up.
use <shared-flywheel-transition-p252.scad>
use <shared-flywheel-p267-tilted-arch.scad>
use <shared-flywheel-p268-upright-wall.scad>

p274_view = "assembly";
p274_angle = 15;
p274_rail_y = 55;
p274_wheel_y = 129; // frozen P2-40: (D200 + 58 mm centre gap) / 2
$fn = 48;

module p274_selector_floor(angle=p274_angle) {
    p268_xz_strip([278,42],p268_mouth_floor(angle),72,5);
}

module p274_selector_rails(angle=p274_angle,rail_y=p274_rail_y) {
    p268_xz_rails_at_y([278,42],p268_mouth_floor(angle),rail_y,3,16);
}

module p274_selector_nose(angle=p274_angle,rail_y=p274_rail_y) {
    p274_selector_floor(angle);
    p274_selector_rails(angle,rail_y);
}

module p274_ski_walls() {
    pts=p267_ski_points(75);
    for(side=[-1,1],i=[0:len(pts)-2]) {
        p0=pts[i]; p1=pts[i+1];
        dx=p1[0]-p0[0]; dz=p1[1]-p0[1]; l=sqrt(dx*dx+dz*dz); a=atan2(dz,dx);
        translate([(p0[0]+p1[0])/2,side*48.5,(p0[1]+p1[1])/2+10])
            rotate([0,-a,0]) cube([l+.8,3,20],center=true);
    }
}

module p274_ski_complete() {
    p267_ski(75);
    p274_ski_walls();
}

module p274_launcher_sweep() {
    union() for(a=[-10:1:20]) p253_frozen_launcher(a);
}

module p274_plate_sweep() {
    union() for(a=[-10:1:20]) module_pose(a) upper_plate();
}

module p274_wheel_sweep() {
    union() for(a=[-10:1:20]) module_pose(a)
        for(yy=[-p274_wheel_y,p274_wheel_y]) wheel_and_motor(yy);
}

module p274_closed_feed() {
    p268_fixed_feed_channel();
    p268_feed_door(0,false);
}

if(p274_view=="assembly") {
    color([.08,.62,.82]) p274_ski_complete();
    color([.82,.58,.12]) p274_selector_nose(p274_angle,p274_rail_y);
}
else if(p274_view=="qa_ski_rails") intersection() {
    p274_ski_complete(); p274_selector_rails(p274_angle,p274_rail_y);
}
else if(p274_view=="qa_ski_floor") intersection() {
    p274_ski_complete(); p274_selector_floor(p274_angle);
}
else if(p274_view=="qa_launcher_rails") intersection() {
    p274_selector_rails(p274_angle,p274_rail_y); p274_launcher_sweep();
}
else if(p274_view=="qa_launcher_floor") intersection() {
    p274_selector_floor(p274_angle); p274_launcher_sweep();
}
else if(p274_view=="qa_plate_rails") intersection() {
    p274_selector_rails(p274_angle,p274_rail_y); p274_plate_sweep();
}
else if(p274_view=="qa_wheels_rails") intersection() {
    p274_selector_rails(p274_angle,p274_rail_y); p274_wheel_sweep();
}
else if(p274_view=="qa_plate_floor") intersection() {
    p274_selector_floor(p274_angle); p274_plate_sweep();
}
else if(p274_view=="qa_wheels_floor") intersection() {
    p274_selector_floor(p274_angle); p274_wheel_sweep();
}
else if(p274_view=="qa_closed_rails") intersection() {
    p274_selector_rails(p274_angle,p274_rail_y); p274_closed_feed();
}
else if(p274_view=="qa_closed_floor") intersection() {
    p274_selector_floor(p274_angle); p274_closed_feed();
}
else if(p274_view=="qa_door_rails") intersection() {
    p274_selector_rails(p274_angle,p274_rail_y); p268_feed_door(0,false);
}
else if(p274_view=="qa_door_floor") intersection() {
    p274_selector_floor(p274_angle); p268_feed_door(0,false);
}
else if(p274_view=="qa_channel_rails") intersection() {
    p274_selector_rails(p274_angle,p274_rail_y); p268_fixed_feed_channel();
}
else if(p274_view=="qa_channel_floor") intersection() {
    p274_selector_floor(p274_angle); p268_fixed_feed_channel();
}
else if(p274_view=="side") rotate([-90,0,0]) {
    color([.08,.62,.82]) p274_ski_complete();
    color([.82,.58,.12]) p274_selector_nose(p274_angle,p274_rail_y);
}
else if(p274_view=="top") {
    color([.08,.62,.82]) p274_ski_complete();
    color([.82,.58,.12]) p274_selector_nose(p274_angle,p274_rail_y);
}
