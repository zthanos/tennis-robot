# Modular open-front test chassis concept

Η τρέχουσα συναρμολόγηση είναι το **`modular-chassis-v2.scad`**.
Το `modular-open-front-test-chassis-concept.scad` είναι παλιό packaging
reference και δείχνει προσωρινά motor pads, όχι τα σημερινά mounts.

Στο v2, το `part` δέχεται `motor_left`, `motor_right` και `motor_section`
για έλεγχο του πραγματικού παραμετρικού mount: επίπεδη κορυφή, ενίσχυση
μέσα στη δοκό, τέσσερις καθοδηγούμενες φωλιές M3 και υποδοχές splice και
στα δύο άκρα. Το v2 καλεί απευθείας το source SCAD των motor modules.

This directory contains an early packaging/interface model for the physical
drivetrain and intake test fixture. It is deliberately not print-ready.

The concept currently shows:

- two 760 mm segmented hollow side rails, centred at Y=+/-210 mm;
- a 34 x 70 mm preliminary closed-box rail envelope whose top remains at the
  existing Z=52 mm intake mounting datum;
- four bed-sized segments per rail (nominally 180, 165, 255 and 160 mm) with
  100 mm internal splice sleeves; every splice is kept away from the
  provisional axle/motor-pad stations;
- rear and X=10 opening-boundary crossbars, with no front crossbar;
- provisional 440 mm wheelbase, 70 x 70 x 8 mm rail-integrated PLA drive
  nodes, and separate 40 x 40 mm steel-mount-foot envelopes on top;
- a removable electronics-tray envelope;
- the real repaired first-prototype Gamma brackets and the corrected
  wheel-first ramp at their existing physical coordinates;
- two provisional no-drill side cradles that reach inward from the rails and
  capture only the ramp's rear/high sidewall regions. They add no transverse
  member across the ball corridor and leave the front lip unsupported/free.
  Their 10 mm X width and X=321 mm centre are the largest tested default that
  retains an empty exact-CAD intersection with both intake-wheel envelopes;
  increasing the width to 12 mm produces interference and is prohibited.

These dimensions are hypotheses used to review the architecture. Before a
structural export, update the drive pads from the physical fit-gauge result,
define fastener/crush-sleeve access, add printable internal ribs, and verify
the real assembled intake/ramp attachment. The shown ramp top caps must become
removable clamp pieces, with measured wall clearance, before release. Do not
slice the assembly file as a production part.
