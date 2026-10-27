# Modular test chassis — ενεργό v3 / P2-78

Η ενεργή δομή ορίζεται από `modular-chassis-v3.scad` και τα STL του
`stl-v3/`. Η P2-78 αντικατέστησε T-clamps, μακριές torsion diagonals και τον
P2-77 centre node με ενσωματωμένα terminal sockets, μανίκια 120 mm και δύο
κοντές `torsion_side_brace`.

Συμπληρωματικά ενεργά:

- `stl-p277/mpu6050_rail_side_mount.stl` — το μοναδικό ενεργό MPU mount,
- `stl-ski-mount/` — ο P2-85 βατήρας R75/75°, κλειστά brackets και wall forks,
- `full-robot-v3.scad` — concept/assembly check, όχι STL release.

Το ακριβές print status, ποσότητες και βίδες είναι στο
`../ACTIVE-PRINT-SET.md`. Οι κατάλογοι `archive/` είναι ιστορικοί και
σημασμένοι **ΜΗΝ ΤΥΠΩΣΕΙΣ**.

Το τρέχον robot χρησιμοποιεί ένα κοινό tilting dual flywheel για intake και
throwing (`−3° / 0° / +15…+20°`), P2-85 ski, P2-71 mesh basket και την P2-93
αψίδα/κλίση. Τα παλιότερα fixed/separate launcher studies δεν είναι ενεργά.

Το `modular-chassis-v2.scad` παραμένει μόνο ως QA compatibility dependency
του `../drive-motor-mount/m3-side-nut-keeper.scad`. Δεν είναι ενεργή release.
