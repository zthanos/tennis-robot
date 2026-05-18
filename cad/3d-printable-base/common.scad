$fn = 72;

module rounded_plate(size=[100, 100, 8], r=8) {
    hull() {
        for (x = [r, size[0] - r]) {
            for (y = [r, size[1] - r]) {
                translate([x, y, 0]) cylinder(h=size[2], r=r);
            }
        }
    }
}

module screw_hole(d=5.2, h=20) {
    translate([0, 0, -0.5]) cylinder(h=h + 1, d=d);
}

module countersink(d1=10, d2=5.2, h=4) {
    cylinder(h=h, d1=d1, d2=d2);
}

module rib(length=100, height=18, thickness=5) {
    rotate([90, 0, 0])
        linear_extrude(height=thickness)
            polygon(points=[[0, 0], [length, 0], [length, height], [0, 0]]);
}

module rounded_box(size=[100, 50, 30], r=6) {
    hull() {
        for (x = [r, size[0] - r]) {
            for (y = [r, size[1] - r]) {
                for (z = [r, size[2] - r]) {
                    translate([x, y, z]) sphere(r=r);
                }
            }
        }
    }
}

