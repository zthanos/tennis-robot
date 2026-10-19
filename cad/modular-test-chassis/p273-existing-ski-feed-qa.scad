// P2-73 first gate: exact-solid check of the already accepted P2-67 ski
// against the already accepted P2-68 launch feed path. No new geometry.
use <shared-flywheel-p267-tilted-arch.scad>
use <shared-flywheel-p268-upright-wall.scad>

view = "intersection"; // intersection | side
angle = 15;            // 15 or 20
target = "all";       // all | floor | rails

module selected_feed(a) {
    mouth = p268_mouth_floor(a);
    if (target == "floor") p268_xz_strip([278,42],mouth,72,5);
    else if (target == "rails") p268_xz_rails([278,42],mouth,72,3,16);
    else p268_feed_channel(a);
}

if (view == "intersection")
    intersection() {
        p267_ski(75);
        selected_feed(angle);
    }
else
    rotate([-90,0,0]) {
        color([.08,.62,.82]) p267_ski(75);
        color([.82,.58,.12]) selected_feed(angle);
        color([.90,.05,.05]) intersection() {
            p267_ski(75);
            selected_feed(angle);
        }
    }
