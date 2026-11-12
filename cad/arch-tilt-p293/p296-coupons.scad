// P2-96 print coupons only. Installed P2-93d/g geometry is unchanged.
use <p293g-tilt-candidate.scad>
use <p293d-arch-candidate.scad>

part = "coupon_foot_seats";

// Local samples of the real X240 and X230 washer seats plus one complete
// Z80 pin clevis. Thin test-only webs keep the coupon a single body.
module coupon_foot_seats() {
  difference() {
    union() {
      // Compact connected carrier; the sampled bores/recesses retain the
      // production diameters and seat stacks.
      translate([0,0,2.5]) cube([42,20,5], center=true);
      for (y=[-8,8]) translate([16,y,12]) cube([10,4,24], center=true);
    }
    // X240 OD9.6 seat and X230 OD14.6 seat.
    translate([-12,0,-1]) cylinder(d=4.6,h=9,$fn=48);
    translate([-12,0,2]) cylinder(d=9.6,h=6,$fn=48);
    translate([4,0,-1]) cylinder(d=4.6,h=9,$fn=48);
    translate([4,0,2]) cylinder(d=14.6,h=6,$fn=48);
    // One real foot pin stack: Ø8.2 sleeve bore and Ø10.6 head recess.
    translate([16,0,14]) rotate([90,0,0]) cylinder(d=8.2,h=24,center=true,$fn=48);
    translate([16,9,14]) rotate([90,0,0]) cylinder(d=10.6,h=4,center=true,$fn=48);
  }
}

// Real knee section containing both through-pin bores at Z80 and Z114.
module coupon_knee_pins() {
  difference() {
    translate([0,0,17]) cube([12,20,46],center=true);
    for(z=[0,34]) translate([0,0,z]) rotate([90,0,0]) cylinder(d=4.6,h=30,center=true,$fn=48);
  }
}

if (part == "coupon_foot_seats") coupon_foot_seats();
else if (part == "coupon_knee_pins") coupon_knee_pins();
else assert(false, str("unknown P2-96 coupon: ", part));
