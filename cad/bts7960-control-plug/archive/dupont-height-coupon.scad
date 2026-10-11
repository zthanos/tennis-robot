// Quick fit coupon for the 3+3 individual female Dupont ends in plug v2.
// Print TWO identical halves per opening size. No electrical connection.
// The generous x opening isolates the top-to-bottom fit problem first.

inner_y = 6.0;        // test values: 6.0, 6.5, 7.0 mm
inner_x = 12.0;       // deliberately generous; v2 pocket is 10.56 mm
outer_x = 14.5;       // same total x width as the v2 driver nose
minimum_outer_y = 7.8;
minimum_y_wall = 0.8;
depth = 6.0;

outer_y = max(minimum_outer_y, inner_y + 2*minimum_y_wall);

assert(outer_x > inner_x);

// This U-shaped half can be laid against one row of 1P plastic ends while
// their wires remain attached. Rotate a second copy 180 degrees about z to
// check the opposite row. Do not force it onto an energised driver.
difference() {
    translate([-outer_x/2, 0, 0])
        cube([outer_x, outer_y/2, depth]);
    translate([-inner_x/2, -0.02, -0.02])
        cube([inner_x, inner_y/2+0.02, depth+0.04]);
}
