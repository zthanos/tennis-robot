// P2-96b: create a slicer-ready opposite hand from an already-on-bed STL.
input_file = "";
axis = "y";
bed_height = 0;

assert(input_file != "", "input_file is required");
if (axis == "y") mirror([0,1,0]) import(input_file);
else if (axis == "z") translate([0,0,bed_height]) mirror([0,0,1]) import(input_file);
else assert(false, str("unsupported mirror axis: ", axis));
