// P2-90b permanent part-installation path gates.
// Every selected top-level result must export EMPTY.  The obstacle set follows
// the documented assembly order: only parts already installed at that step
// are present.  Linear paths are sampled rather than hulled so cavities and
// fork openings remain real during the sweep.

include <rear-pod/params-v2.scad>
use <rear-pod/rear-pod-v2.scad>
use <body-panels/body-panels-f-rear-v2.scad>
use <modular-test-chassis/modular-chassis-v3.scad>

check = "corner_ties";

module bases() {
  translate([0,0,pod_z_bottom]) { base_half_v2(-1); base_half_v2(1); }
}
module ties() { corner_tie_installed(-1); corner_tie_installed(1); }
module plate() { carrier_installed(); }
module frames() {
  for(s=[-1,1]) translate([panel_center_x,s*54,64]) side_frame();
}
module posts() {
  panel_post_installed(panel_center_x-58,-1,"rear_negative");
  panel_post_installed(panel_center_x+58,-1,"front_negative");
  panel_post_installed(panel_center_x-58,1,"positive");
  panel_post_installed(panel_center_x+58,1,"positive");
}
module panel() { translate([panel_center_x,0,panel_z]) control_panel_v2(); }
module docks() { for(s=[-1,1]) parkside_dock(s); }
module stays() { for(s=[-1,1]) corner_stay(s); }
module frame_without_pod_support_contacts() {
  difference() {
    frame_assembly();
    // P2-79 approved bearing/contact volumes: full rear-crossbar saddle and
    // outer corner-clamp faces. They are not insertion obstructions.
    translate([-191,-196,44]) cube([52,392,8.2]);
    for(side=[-1,1]) translate([-146,side>0?244.9:-251,24]) cube([22,6.1,32]);
  }
}

module sweep_ties() {
  // Stop 0.2 mm before the final bearing face; final static contact is
  // covered by the dedicated tie/frame exact-solid gate.
  for(dx=[0.2:5:80.2]) translate([dx,0,0]) ties();
}
module sweep_bases() {
  for(dz=[0.2:5:80.2]) translate([0,0,dz]) bases();
}
module sweep_plate() {
  for(dz=[0:5:100]) translate([0,0,dz]) plate();
}
module sweep_frames() {
  // Frames first enter an unobstructed assembly station 100 mm above the pod.
  for(s=[-1,1],dy=[0:5:100])
    translate([panel_center_x,s*(54+dy),164]) side_frame();
}
module sweep_plate_into_raised_frames() {
  for(dz=[100.2:5:200.2]) translate([0,0,dz]) plate();
}
module sweep_plate_frame_subassembly() {
  for(dz=[0.2:5:100.2]) translate([0,0,dz]) union(){ plate(); frames(); }
}
module sweep_posts() {
  for(dz=[0:5:110]) translate([0,0,dz]) posts();
}
module sweep_panel() {
  for(dz=[0:5:80]) translate([0,0,dz]) panel();
}
module sweep_docks() {
  // Each dock slides from the open rear rather than passing vertically
  // through its side frame.
  for(dx=[0:-5:-150],s=[-1,1]) translate([dx,0,0.1]) parkside_dock(s);
}
module sweep_holder() {
  for(dx=[0:-5:-145]) translate([dx,0,0]) ugreen_post_holder();
}
module sweep_caps() {
  // Tangential caps stay outside the Ø23 post in X and enter directly from
  // the corresponding outer side; no post bend or electronics is crossed.
  for(s=[-1,1]) {
    for(dy=[0:5:100]) translate([0,s*dy,0]) ugreen_clamp_cap_installed(s);
  }
}
module sweep_stays() {
  // Keep the rear cheek 0.1 mm off the crossbar while lowering; the final
  // 0.1 mm seating stroke ends at the verified zero-volume bearing face.
  for(dz=[0.2:5:100.2]) translate([-0.1,0,dz]) stays();
}

module qa_corner_ties() {
  intersection() { sweep_ties(); frame_assembly(); }
}
module qa_bases() {
  intersection() { sweep_bases(); union(){ frame_without_pod_support_contacts(); ties(); } }
}
module qa_plate() {
  union() {
    intersection() { sweep_plate_into_raised_frames(); translate([0,0,100]) frames(); }
    intersection() { sweep_plate_frame_subassembly(); union(){ frame_assembly(); ties(); bases(); } }
  }
}
module qa_side_frames() {
  intersection() { sweep_frames(); union(){ frame_assembly(); ties(); bases(); } }
}
module qa_frames_plate_debug() {
  intersection() {
    for(s=[-1,1],dy=[0:5:100]) translate([panel_center_x,s*(54+dy),164]) side_frame();
    translate([0,0,100]) plate();
  }
}
module qa_plate_into_raised_frames_debug() {
  intersection() {
    sweep_plate_into_raised_frames();
    translate([0,0,100]) frames();
  }
}
module qa_frames_base_debug() { intersection() { sweep_frames(); union(){ bases(); ties(); frame_assembly(); } } }
module qa_posts() {
  intersection() { sweep_posts(); union(){ frame_assembly(); ties(); bases(); plate(); frames(); } }
}
module qa_posts_static_debug() {
  intersection() { posts(); union(){ frame_assembly(); ties(); bases(); plate(); frames(); } }
}
module qa_posts_frames_debug() { intersection() { posts(); frames(); } }
module qa_posts_other_debug() {
  intersection() { posts(); union(){ frame_assembly(); ties(); bases(); plate(); } }
}
module qa_panel() {
  intersection() { sweep_panel(); union(){ frame_assembly(); ties(); bases(); plate(); frames(); posts(); } }
}
module qa_docks() {
  // Docks install before the holder; the frame/panel stack is already fixed.
  intersection() { sweep_docks(); union(){ frame_assembly(); ties(); bases(); plate(); frames(); posts(); panel(); } }
}
module qa_holder() {
  intersection() { sweep_holder(); union(){ frame_assembly(); ties(); bases(); plate(); frames(); posts(); panel(); docks(); } }
}
module qa_holder_pod_debug() {
  intersection() { sweep_holder(); union(){ ties(); bases(); plate(); frames(); panel(); docks(); } }
}
module qa_holder_posts_debug() { intersection() { sweep_holder(); posts(); } }
module qa_holder_posts_static_debug() { intersection() { ugreen_post_holder(); posts(); } }
module qa_holder_chassis_debug() { intersection() { sweep_holder(); frame_assembly(); } }
module qa_clamp_caps() {
  intersection() { sweep_caps(); union(){ frame_assembly(); ties(); bases(); plate(); frames(); posts(); panel(); docks(); ugreen_post_holder(); } }
}
module qa_corner_stays() {
  // The stays enter from behind after the pod structure; skins are later.
  intersection() { sweep_stays(); union(){ frame_assembly(); ties(); bases(); plate(); frames(); posts(); panel(); docks(); ugreen_post_holder();
    ugreen_clamp_cap_installed(-1); ugreen_clamp_cap_installed(1); } }
}

if(check=="corner_ties") qa_corner_ties();
else if(check=="bases") qa_bases();
else if(check=="plate") qa_plate();
else if(check=="side_frames") qa_side_frames();
else if(check=="frames_plate") qa_frames_plate_debug();
else if(check=="plate_into_frames") qa_plate_into_raised_frames_debug();
else if(check=="frames_base") qa_frames_base_debug();
else if(check=="posts") qa_posts();
else if(check=="posts_static") qa_posts_static_debug();
else if(check=="posts_frames") qa_posts_frames_debug();
else if(check=="posts_other") qa_posts_other_debug();
else if(check=="panel") qa_panel();
else if(check=="docks") qa_docks();
else if(check=="holder") qa_holder();
else if(check=="holder_pod") qa_holder_pod_debug();
else if(check=="holder_posts") qa_holder_posts_debug();
else if(check=="holder_posts_static") qa_holder_posts_static_debug();
else if(check=="holder_chassis") qa_holder_chassis_debug();
else if(check=="clamp_caps") qa_clamp_caps();
else if(check=="corner_stays") qa_corner_stays();
else union(){ qa_corner_ties(); qa_bases(); qa_plate(); qa_side_frames();
              qa_posts(); qa_panel(); qa_docks(); qa_holder();
              qa_clamp_caps(); qa_corner_stays(); }
