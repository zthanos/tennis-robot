// P2-78 review views; no manufacturing STL.
use <modular-chassis-v3.scad>
use <shared-flywheel-p273-bolted-ski.scad>
use <p277-torsion-node.scad>

view="assembly";

module joint_detail() {
    color([.16,.48,.78]) {
        translate([300,220,22]) front_terminal_v3(1);
        translate([240,0,22]) p278_crossbar_half(50);
        translate([300,0,22]) p278_crossbar_half(20);
    }
    color([.95,.53,.10]) {
        translate([240,130,32]) rotate([0,0,90]) p278_terminal_sleeve(43,13);
        translate([300,130,32]) rotate([0,0,90]) p278_terminal_sleeve(13,13);
    }
    color([.15,.15,.17]) translate([265.1,150,22]) torsion_side_brace();
}

if(view=="joint") joint_detail();
else if(view=="underside") {
    frame_assembly();
    color([.82,.48,.12,.55]) for(side=[-1,1]) p273_mount_bracket(side);
}
else frame_assembly();
