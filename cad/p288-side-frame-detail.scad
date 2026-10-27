use <rear-pod/rear-pod-v2.scad>

color([.78,.56,.12]) {
  translate([0,0,55]) { base_half_v2(-1); base_half_v2(1); }
  for (s=[-1,1]) translate([-198.5,s*54,64]) side_frame();
}
color([.95,.35,.05]) for (s=[-1,1]) rear_foot_wedge_installed(s);
color([.82,.82,.85]) p288_access_hardware();
// Exploded drill jig above the +Y base; it is removed before assembly.
color([.15,.55,.85,.72]) translate([0,54,76]) side_frame_base_drill_jig();
