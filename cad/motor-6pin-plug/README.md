# Motor-side 1×6 pin spacing gauge — v0

The six contacts are in one row at 2.54 mm pitch. Their first-to-last centre
span is 12.70 mm. With a 15.00 mm overall length, the nominal end margin is
1.15 mm on each side.

`pin-spacing-gauge-v0.stl` is a **fit gauge only**, not a usable electrical
connector. Its six square holes are provisionally sized for 0.64 mm square
individual metal pins plus 0.18 mm total clearance. The 4.5 mm width and
3.0 mm thickness are also provisional. The recessed dot marks contact 1 but
cannot prevent reverse insertion.

Before designing the actual cable plug, test this against the *unpowered*
motor socket and confirm:

- actual pin cross-section, protrusion/engagement depth, and retention feature;
- socket opening width, height, depth, and any orientation key;
- pin numbering/polarity and current carried by each contact.

The finished part must positively retain the metal contacts and provide wire
strain relief. It must use contacts rated for the motor current if power runs
through this connector. A printed hole alone is not contact retention.
