// P2-93 report-only views. Not an STL source.
use <p293-arch-tilt-release.scad>
use <../modular-test-chassis/modular-chassis-v3.scad>
use <../modular-test-chassis/shared-flywheel-p271-basket-grid.scad>
use <../modular-test-chassis/shared-flywheel-p273-bolted-ski.scad>
use <../modular-test-chassis/shared-rear-pod-p270.scad>
use <../body-panels/body-panels-f.scad>

view="integration";
$fn=32;

module p271_clipped_report() {
  intersection() {
    p271_basket_exact();
    translate([-250,0,220]) cube([950,500,500],center=true);
  }
  p271_front_grid();
}

module context(alpha=0.18) {
  color([0.45,0.45,0.48,alpha]) frame_assembly();
  color([0.25,0.72,0.35,alpha]) p271_clipped_report();
  color([0.55,0.45,0.32,alpha]) p273_ski_mount_assembly();
  color([0.48,0.36,0.62,alpha]) p270_outer_envelope();
}

if(view=="integration") {
  context(0.22);
  color([0.10,0.42,0.86]) arch_release_assembly();
  color([0.95,0.43,0.08]) tilt_release_assembly(20);
}
else if(view=="f-clearance") {
  context(0.10);
  color([0.80,0.82,0.86,0.20]) all_pieces_qa();
  color([0.10,0.42,0.86]) arch_release_assembly();
  color([0.95,0.43,0.08]) tilt_release_assembly(0);
}
else if(view=="basket-prism") {
  color([0.25,0.72,0.35,0.30]) p271_clipped_report();
  color([0.10,0.42,0.86]) arch_release_assembly();
  color([0.95,0.43,0.08]) tilt_release_assembly(0);
  color([0.90,0.12,0.12,0.18])
    translate([53.5,0,305]) cube([343,344,430],center=true);
}
