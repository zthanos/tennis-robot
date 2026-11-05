"""Installed-pose bodies for the independent collision check.

Each entry is (name, group, header, body).  `header` names source files by
key; the body is the OpenSCAD call that places the part in robot coordinates.
No body is ever trimmed or subtracted: the check measures what will be printed.
"""

SOURCES = {
    "chassis": "use <{cad}/modular-test-chassis/modular-chassis-v3.scad>",
    "ski": "use <{cad}/modular-test-chassis/shared-flywheel-p273-bolted-ski.scad>",
    "pod": "use <{cad}/rear-pod/rear-pod-v2.scad>\ninclude <{cad}/rear-pod/params-v2.scad>",
    "frear": "use <{cad}/body-panels/body-panels-f-rear-v2.scad>",
    "ffront": "use <{cad}/body-panels/body-panels-f.scad>",
    "p293": "use <{cad}/arch-tilt-p293/p293-arch-tilt-release.scad>",
    "basket": "use <{cad}/modular-test-chassis/shared-flywheel-p271-basket-grid.scad>",
    "launcher": "use <{cad}/modular-test-chassis/shared-flywheel-transition-p252.scad>",
    "p293+phase1": "use <{cad}/arch-tilt-p293/p293-arch-tilt-release.scad>\nuse <{cad}/modular-test-chassis/shared-flywheel-intake-launcher-phase1.scad>",
}

PIVOT = "[387,0,55.84]"

PARTS = [
    # Chassis is printed and assembled; one body.
    ("chassis_frame", "chassis", "chassis", "frame_assembly();"),
    # Ski mount (P2-85).
    ("ski", "ski", "ski", "p273_ski(0);"),
    ("ski_bracket_L", "ski", "ski", "p273_mount_bracket(1);"),
    ("ski_bracket_R", "ski", "ski", "p273_mount_bracket(-1);"),
    ("ski_fork_L", "ski", "ski", "p285_fork(1,0);"),
    ("ski_fork_R", "ski", "ski", "p285_fork(-1,0);"),
    # Rear pod v2.
    ("pod_base_L", "pod", "pod", "translate([0,0,pod_z_bottom]) base_half_v2(1);"),
    ("pod_base_R", "pod", "pod", "translate([0,0,pod_z_bottom]) base_half_v2(-1);"),
    ("pod_corner_tie_L", "pod", "pod", "corner_tie_installed(1);"),
    ("pod_corner_tie_R", "pod", "pod", "corner_tie_installed(-1);"),
    ("pod_pi_shelf", "pod", "pod", "translate([pi_center_x,0,pi_shelf_z]) pi_shelf_v2();"),
    ("pod_carrier", "pod", "pod", "carrier_installed();"),
    ("pod_side_frame_L", "pod", "pod", "translate([panel_center_x,54,64]) side_frame();"),
    ("pod_side_frame_R", "pod", "pod", "translate([panel_center_x,-54,64]) side_frame();"),
    ("pod_post_rear_neg", "pod", "pod", "panel_post_installed(panel_center_x-58,-1,\"rear_negative\");"),
    ("pod_post_front_neg", "pod", "pod", "panel_post_installed(panel_center_x+58,-1,\"front_negative\");"),
    ("pod_post_rear_pos", "pod", "pod", "panel_post_installed(panel_center_x-58,1,\"positive\");"),
    ("pod_post_front_pos", "pod", "pod", "panel_post_installed(panel_center_x+58,1,\"positive\");"),
    ("pod_control_panel", "pod", "pod", "translate([panel_center_x,0,panel_z]) control_panel_v2();"),
    ("pod_wedge_L", "pod", "pod", "rear_foot_wedge_installed(1);"),
    ("pod_wedge_R", "pod", "pod", "rear_foot_wedge_installed(-1);"),
    ("pod_parkside_dock_L", "pod", "pod", "parkside_dock(1);"),
    ("pod_parkside_dock_R", "pod", "pod", "parkside_dock(-1);"),
    ("pod_parkside_drawer_L", "pod", "pod", "parkside_drawer(1);"),
    ("pod_parkside_drawer_R", "pod", "pod", "parkside_drawer(-1);"),
    ("pod_ugreen_holder", "pod", "pod", "ugreen_post_holder();"),
    ("pod_ugreen_cap_L", "pod", "pod", "ugreen_clamp_cap_installed(1);"),
    ("pod_ugreen_cap_R", "pod", "pod", "ugreen_clamp_cap_installed(-1);"),
    ("pod_ugreen_retainer", "pod", "pod", "ugreen_rear_retainer();"),
    # F rear skin (P2-86/P2-87).
    ("frear_rear_lo_L", "frear", "frear", "rear_lo(1);"),
    ("frear_rear_lo_R", "frear", "frear", "rear_lo(-1);"),
    ("frear_rear_hi_L", "frear", "frear", "rear_hi(1);"),
    ("frear_rear_hi_R", "frear", "frear", "rear_hi(-1);"),
    ("frear_deck_L", "frear", "frear", "deck_rear(1);"),
    ("frear_deck_R", "frear", "frear", "deck_rear(-1);"),
    ("frear_corner_stay_L", "frear", "frear", "corner_stay(1);"),
    ("frear_corner_stay_R", "frear", "frear", "corner_stay(-1);"),
    ("frear_splice_lo", "frear", "frear", "center_splice(92,168,[105,155]);"),
    ("frear_splice_hi", "frear", "frear", "center_splice(237,318,[250,305]);"),
    ("frear_door_L", "frear", "frear", "parkside_door(1,0);"),
    ("frear_door_R", "frear", "frear", "parkside_door(-1,0);"),
    # F front (concept).
    ("ffront_concept", "ffront", "ffront", "f_side_front_concept();"),
    # Arch + tilt (P2-93).
    ("arch", "p293", "p293", "arch_release_assembly();"),
    ("tilt_fit_carrier", "tilt", "p293", "fit_carrier_installed(); // P2-93b +Y 28 mm"),
    ("tilt_kfl08", "tilt", "p293", "kfl08_installed(); // P2-93b axis station X274 Z112.653, +Y 24 mm"),
    ("tilt_nut_carriage", "tilt", "p293+phase1", "translate(gpoint(0,[50,-205,90])) anti_backlash_carriage();"),
    ("tilt_stop_L", "tilt", "p293+phase1", "translate(gpoint(20,[20,194,115])+[-72,0,-7]) rotate([0,0,180]) upper_stop();"),
    ("tilt_stop_R", "tilt", "p293+phase1", "translate(gpoint(20,[20,-194,115])+[-72,0,-7]) upper_stop();"),
    ("tilt_as5600_pod", "tilt", "p293", f"translate([{PIVOT}[0],244,{PIVOT}[2]]) as5600_pod();"),
    ("tilt_magnet_cap", "tilt", "p293", f"translate([{PIVOT}[0],238,{PIVOT}[2]]) rotate([90,0,0]) magnet_cap();"),
    # Basket (P2-71 frozen, same clip as p293-qa.scad).
    ("basket", "basket", "basket",
     "intersection() { p271_basket_exact(); translate([-250,0,220]) cube([950,500,500],center=true); } p271_front_grid();"),
    # Moving flywheel module at the three operating angles.
    ("launcher_m3", "launcher", "launcher", "p253_frozen_launcher(-3);"),
    ("launcher_0", "launcher", "launcher", "p253_frozen_launcher(0);"),
    ("launcher_p20", "launcher", "launcher", "p253_frozen_launcher(20);"),
]

# Pairs that are never physically present together.
EXCLUSIVE_GROUPS = [{"launcher_m3", "launcher_0", "launcher_p20"}]
