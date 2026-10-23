// P2-90 permanent removable-object insertion-path gates.
// Every selected result must export EMPTY.  Intentional guide/contact parts
// and parts explicitly removed earlier in the documented service sequence are
// omitted from the corresponding obstacle set.

include <rear-pod/params-v2.scad>
use <rear-pod/rear-pod-v2.scad>
use <body-panels/body-panels-f-rear-v2.scad>
use <modular-test-chassis/modular-chassis-v3.scad>

check = "ugreen";

module f_rear_service_without_hi() {
  for(s=[-1,1]) {
    rear_lo(s);
    deck_rear(s);
    corner_stay(s);
    parkside_door(s,110);
    turn_button_installed(s,true);
  }
  center_splice(92,168,[105,155]);
}

module ugreen_device(x=ugreen_center[0]) {
  translate([x,ugreen_center[1],ugreen_center[2]])
    cube(ugreen_installed_size,center=true);
}

module ugreen_sweep() {
  hull() {
    ugreen_device(ugreen_service_start_x);
    ugreen_device(ugreen_center[0]);
  }
}

module full_pod_without_ugreen_holder() {
  installed_p279_structure();
  control_body_solids();
  for(s=[-1,1]) { parkside_dock(s); parkside_drawer(s); }
  frame_assembly();
  f_rear_service_without_hi();
}

module qa_ugreen_insertion() {
  intersection() { ugreen_sweep(); full_pod_without_ugreen_holder(); }
}
module qa_ugreen_pod_debug() {
  intersection() { ugreen_sweep(); installed_p279_structure(); }
}
module qa_ugreen_panels_debug() {
  intersection() { ugreen_sweep(); f_rear_service_without_hi(); }
}
module qa_ugreen_aux_debug() {
  intersection() { ugreen_sweep(); union(){ control_body_solids();
    for(s=[-1,1]) { parkside_dock(s); parkside_drawer(s); }
    frame_assembly(); } }
}
module qa_ugreen_carrier_debug() {
  intersection() { ugreen_sweep(); carrier_installed(); }
}
module qa_ugreen_electronics_debug() {
  intersection() { ugreen_sweep(); electronics_ghosts(); }
}
module qa_ugreen_posts_debug() {
  intersection() { ugreen_sweep(); union(){
    panel_post_installed(panel_center_x-58,-1,"rear_negative");
    panel_post_installed(panel_center_x+58,-1,"front_negative");
    panel_post_installed(panel_center_x-58,1,"positive");
    panel_post_installed(panel_center_x+58,1,"positive"); } }
}

// Parkside service: the door is open and the moving drawer/battery envelope
// is checked against the fixed pod and skins; its own guide dock is omitted.
module parkside_unit(side=1,x=parkside_center_x) {
  translate([x,side*parkside_center_y,
             parkside_drawer_floor_z+4+parkside_drawer_outer[2]/2])
    cube(parkside_drawer_outer,center=true);
}
module parkside_sweep(side=1) {
  hull() { parkside_unit(side,parkside_center_x); parkside_unit(side,-365); }
}
module parkside_obstacles(side=1) {
  installed_p279_structure();
  control_body_solids();
  parkside_dock(-side);
  frame_assembly();
  f_rear_service_without_hi();
}
module qa_parkside_insertion() {
  for(s=[-1,1]) intersection() { parkside_sweep(s); parkside_obstacles(s); }
}

// Upright-plate service sequence: disconnect electronics; remove rear_hi,
// control panel, posts and side frames; then lift the bare plate.  The plate's
// integral rear bosses cannot sweep through the frozen 4.8 mm jaws, so the
// side frames are an explicit prerequisite rather than a hidden collision.
module carrier_sweep() {
  hull() { carrier_installed(); translate([0,0,230]) carrier_installed(); }
}
module carrier_obstacles_after_panel_removal() {
  translate([0,0,pod_z_bottom]) base_half_v2(-1);
  translate([0,0,pod_z_bottom]) base_half_v2(1);
  translate([pi_center_x,0,pi_shelf_z]) pi_shelf_v2();
  frame_assembly();
  f_rear_service_without_hi();
}
module qa_carrier_removal() {
  intersection() { carrier_sweep(); carrier_obstacles_after_panel_removal(); }
}

// Pi service follows the carrier: with panel/posts/side frames/carrier removed,
// lift the Pi vertically without crossing a skin or fixed chassis member.
module pi_device(z=pi_shelf_z+plate_t+pi_clear[2]/2) {
  translate([pi_center_x,0,z]) cube(pi_clear,center=true);
}
module pi_sweep() {
  hull() { pi_device(); pi_device(430); }
}
module pi_obstacles_after_carrier_removal() {
  translate([0,0,pod_z_bottom]) base_half_v2(-1);
  translate([0,0,pod_z_bottom]) base_half_v2(1);
  frame_assembly();
  f_rear_service_without_hi();
}
module qa_pi_removal() {
  intersection() { pi_sweep(); pi_obstacles_after_carrier_removal(); }
}

if(check=="ugreen") qa_ugreen_insertion();
else if(check=="ugreen_pod") qa_ugreen_pod_debug();
else if(check=="ugreen_panels") qa_ugreen_panels_debug();
else if(check=="ugreen_aux") qa_ugreen_aux_debug();
else if(check=="ugreen_carrier") qa_ugreen_carrier_debug();
else if(check=="ugreen_electronics") qa_ugreen_electronics_debug();
else if(check=="ugreen_posts") qa_ugreen_posts_debug();
else if(check=="parkside") qa_parkside_insertion();
else if(check=="carrier") qa_carrier_removal();
else if(check=="pi") qa_pi_removal();
else union(){ qa_ugreen_insertion(); qa_parkside_insertion();
              qa_carrier_removal(); qa_pi_removal(); }
