// Raised 120x80 mm faceplate and separate perimeter skirt for the new board.
// The original study remains printable unchanged. Export with -D part=...
// Values are fit-test dimensions, not a soldering or production drawing.

include <unified-keyed-faceplate-study.scad>

// Height from the perfboard's component face to the actual female header top.
// Measure the purchased header before final assembly; the study uses 8.5 mm.
skirt_h = 8.50;

// The male plug nose is 18.3 mm long. At this port height, 6.0 mm of nose
// travel remains before its wider shoulder touches the top of the collar.
// Pin engagement still needs a physical continuity/retention test.
collar_h = 12.30;

// Keep the proven board outline and M3 mounting locations from the study.
