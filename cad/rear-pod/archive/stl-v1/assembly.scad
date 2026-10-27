// Review assembly for P2-75. Printable geometry is in rear-pod-v1.scad.
use <../../../modular-test-chassis/modular-chassis-v3.scad>
use <rear-pod-v1.scad>

view = "pod"; // pod | robot | top | side
show_envelope_context = true;

if (view=="robot") {
    color([.30,.30,.32,.30]) frame_assembly();
    if (show_envelope_context) envelope_ghost();
    assembly();
    // MPU6050 is fixed to the existing X240/Y+/-30 M4 grid, not to the pod.
    color([.70,.50,.10]) translate([240,0,42]) rotate([0,0,90])
        mpu6050_chassis_mount();
}
else if (view=="top") assembly();
else if (view=="side") rotate([-90,0,0]) assembly();
else assembly();
