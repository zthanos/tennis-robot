// P2-90c/P2-90d review views. Not a printable part.
use <rear-pod-v2.scad>

view = "installed";
$fn = 48;
panel_center_x = -198.5;

module lower_posts() {
  intersection() {
    union() {
      panel_post_installed(panel_center_x-58,-1,"rear_negative");
      panel_post_installed(panel_center_x+58,-1,"front_negative");
      panel_post_installed(panel_center_x-58,1,"positive");
      panel_post_installed(panel_center_x+58,1,"positive");
    }
    translate([-290,-130,138]) cube([185,260,82]);
  }
}

module post_bolt_hardware() {
  for (s=[-1,1]) {
    // Rear: head on -X, nyloc on +X.
    color([0.72,0.74,0.77]) {
      translate([-286.5,s*54,154]) rotate([0,90,0]) cylinder(d=4,h=60);
      translate([-281.5,s*54,154]) rotate([0,90,0]) cylinder(d=10,h=2);
      translate([-230.5,s*54,154]) rotate([0,90,0]) cylinder(d=8,h=5,$fn=6);
      // Front: head on +X, nyloc on -X.
      translate([-170.5,s*54,154]) rotate([0,90,0]) cylinder(d=4,h=60);
      translate([-118.5,s*54,154]) rotate([0,90,0]) cylinder(d=10,h=2);
      translate([-171.5,s*54,154]) rotate([0,90,0]) cylinder(d=8,h=5,$fn=6);
    }
  }
}

module installed_review() {
  color([0.92,0.58,0.10]) for (s=[-1,1])
    translate([panel_center_x,s*54,64]) side_frame();
  color([0.16,0.46,0.76,0.72]) lower_posts();
  color([0.38,0.42,0.48,0.22]) carrier_installed();
  post_bolt_hardware();
}

module cup_detail() {
  color([0.92,0.58,0.10]) side_frame();
  // Show the two frozen post bottoms in their installed local pose.
  color([0.16,0.46,0.76,0.72]) intersection() {
    union() {
      translate([-58,0,85]) panel_post("rear_negative");
      translate([58,0,85]) panel_post("front_negative");
    }
    translate([-100,-55,72]) cube([200,110,72]);
  }
  color([0.72,0.74,0.77]) for (x=[-58,58])
    translate([x-30,0,90]) rotate([0,90,0]) cylinder(d=4,h=60);
}

if (view=="detail") cup_detail();
else installed_review();
