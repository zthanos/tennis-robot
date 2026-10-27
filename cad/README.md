# Active CAD status

## Physical print manifest

Το μοναδικό ευρετήριο για τη σημερινή φυσική κατασκευή είναι το
[`ACTIVE-PRINT-SET.md`](ACTIVE-PRINT-SET.md). Περιλαμβάνει ενεργά STL,
παραμετρικές πηγές, P2 προέλευση, βίδες και κατάσταση εκτύπωσης. Αρχεία σε
`archive/` δεν τυπώνονται.

Για το current shared-flywheel build οι ενεργές releases είναι:

- κοινό dual flywheel P2-40 για intake/throwing (`−3° / 0° / +15…+20°`),
  με P2-53/54 FIT0186+Tr8×2 tilt, AS5600 και BTS #3,
- P2-71 four-side mesh basket: μόνο η fail-closed θύρα `80×80 mm` είναι
  συμπαγής feed path· front-wall detail περιμένει mockup,
- F ως μοναδικό body concept (`body-panels/README.md`).

- `modular-test-chassis/modular-chassis-v3.scad` + `stl-v3/` (P2-78),
- `modular-test-chassis/stl-ski-mount/` (P2-85 bracket + wall forks),
- `rear-pod/rear-pod-v2.scad` + `rear-pod/stl-v2/` (P2-86b panel interface),
- `body-panels/stl-f-rear/` (P2-86b σχεδιασμένο, αναμονή — δεν τυπώνεται),
- `body-panels/p286b-perimeter-map.scad` και
  `../docs/design/body-panels/p286b-perimeter-map-el.md` (P2-87 interface map),
- τα test fixtures που αναφέρονται στο manifest.

Το `basket-bin-v2/` και το `collector-intake-v1/option-a/` είναι ιστορικά
predecessors. Δεν είναι active models και δεν πρέπει να χρησιμοποιούνται για
νέα export. Η σημερινή διαδρομή μπάλας/καλαθιού ορίζεται από τα P2-40,
P2-67…P2-74 και το physical basket mockup gate.

The previous `3d-printable-base` and `cyber-shell` models described superseded
single-roller, scoop, launcher, 760 x 430 mm chassis, and cosmetic-shell
concepts. They were removed from the working tree on the
`feat/dual-wheel-intake-concept` branch; Git history remains the archive.

`archive/single-motor-geared-intake/` is also historical: it visualizes the
rejected one-motor gear-train option. The active intake has two independent
motors, one per side-pinch wheel.

Το `flywheel-launcher-v0/` είναι ιστορικό standalone-launcher workspace.

Current mechanical sources of truth:

- `ACTIVE-PRINT-SET.md`
- `modular-test-chassis/modular-chassis-v3.scad`
- `modular-test-chassis/full-robot-v3.scad` (assembly/check only)
- `rear-pod/rear-pod-v2.scad`
- `body-panels/body-panels-f-rear-v2.scad`
- `../docs/current-design-el.md`
- `../docs/mechanism/shared-flywheel-intake-launcher-debug-log-el.md`
