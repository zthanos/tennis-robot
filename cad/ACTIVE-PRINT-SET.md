# Ενεργό σύνολο εκτύπωσης — P2-93

Η παρούσα λίστα είναι το μοναδικό ευρετήριο για κομμάτια που είναι ενεργά
στη σημερινή φυσική κατασκευή. Αρχεία κάτω από `archive/` ή με σήμανση
«concept» **δεν τυπώνονται**. Η κατάσταση αποτυπώνει όσα είναι γνωστά στις
2026-10-26 (P2-93)· όταν ένα φυσικό part αλλάξει, ενημερώνεται πρώτα αυτό το
αρχείο.

## Σασί v3 — P2-78 sockets και πλευρικές διαγώνιες

Κύρια πηγή: `modular-test-chassis/modular-chassis-v3.scad`.
Ενεργός κατάλογος: `modular-test-chassis/stl-v3/`.

| Φυσικό κομμάτι | Ενεργό STL (ποσότητα) | P2 | Βίδες / σύνδεση | Κατάσταση |
|---|---|---:|---|---|
| Fit coupon μανικιού | `stl-v3/sleeve_fit_coupon.stl` ×1 | pre-P2-78 fit gate | M4 δοκιμής | Τυπωμένο· μόνο δοκιμή |
| Κύρια μανίκια 50×30 | `stl-v3/clamp_sleeve.stl` ×5 | v3/P2-78 retained | M4×40 top, M4×60 side, ροδέλες/παξιμάδια | Τυπωμένα |
| Πίσω γωνίες | `stl-v3/corner_clamp_left.stl` ×1, `corner_clamp_right.stl` ×1 | P2-78 retained/fixed | M4×40 top, M4×60 side | Τυπωμένα |
| Πίσω δοκοί κινητήρων | `stl-v3/rear_motor_left.stl` ×1, `rear_motor_right.stl` ×1 | v3/P2-78 retained | Hardware FIT0403 + joint M4 | Τυπωμένα |
| Μεσαίες πλευρικές δοκοί | `stl-v3/rail_extension_left.stl` ×1, `rail_extension_right.stl` ×1 | v3/P2-78 retained | Joint M4×40/60 | Τυπωμένα |
| Εμπρός terminal δοκοί με sockets | `stl-v3/front_terminal_left.stl` ×1, `front_terminal_right.stl` ×1 | P2-78 | Joint M4 + socket locks | Τυπωμένα |
| Πίσω τραβέρσα | `stl-v3/rear_crossbar_half.stl` ×2 | v3 corner fix/P2-78 retained | M4×40/60 | Τυπωμένη διορθωμένη έκδοση |
| Τραβέρσα X240 | `stl-v3/x240_crossbar_half.stl` ×2 | P2-78 | 4×M4×30 top + 4×M4×60 transverse συνολικά | Τυπωμένη |
| Κεντρικό μανίκι X240 | `stl-v3/x240_clamp_sleeve.stl` ×1 | v3/P2-78 retained | M4×30/60 | Τυπωμένο |
| Terminal μανίκια X240 | `stl-v3/x240_terminal_sleeve.stl` ×2 | P2-78 | M4 locks Y75/100/125/150 | Τυπωμένα |
| Τραβέρσα X300 | `stl-v3/x300_crossbar_half.stl` ×2 | P2-78 | 2×M4×30 top + 4×M4×30 transverse συνολικά | Τυπωμένη |
| Κεντρικό μανίκι X300 | `stl-v3/x300_clamp_sleeve.stl` ×1 | v3/P2-78 retained | M4×30 | Τυπωμένο |
| Terminal μανίκια X300 | `stl-v3/x300_terminal_sleeve.stl` ×2 | P2-78 | M4 locks Y100/160 | Τυπωμένα |
| Πλευρικές torsion διαγώνιες | `stl-v3/torsion_side_brace.stl` ×2 | P2-78 | ανά brace M4×65 στο X240/Y150 + M4×35 στο X300/Y125, μεγάλες ροδέλες/παξιμάδια | Τυπωμένες |

Το `stl-v3/ballast_tab.stl` δεν είναι ενεργό: ανήκε στην καταργημένη
plywood/metal ballast plate. Τα P2-77 `brace_drill_jig.stl` και
`torsion_centre_node.stl` έχουν αντικατασταθεί πλήρως από τα δύο P2-78
`torsion_side_brace.stl` και αρχειοθετούνται.

### Ενεργό MPU

| Φυσικό κομμάτι | Ενεργό STL | Πηγή | P2 | Βίδες | Κατάσταση |
|---|---|---|---:|---|---|
| Πλευρική βάση MPU6050 στη +Y ράγα | `modular-test-chassis/stl-p277/mpu6050_rail_side_mount.stl` | `modular-test-chassis/p277-torsion-node.scad` | P2-77 | 2×M4×55/60 στις X100/X125, ροδέλες/nyloc· MPU με μη φέρον hardware της πλακέτας | Προς εκτύπωση |

Αυτό είναι το μόνο ενεργό MPU mount. Το `rear-pod/stl-v2/mpu6050_mount.stl`
δεν χρησιμοποιείται.

## Βατήρας R75/75° — P2-85

Πηγή: `modular-test-chassis/shared-flywheel-p273-bolted-ski.scad`.
Ενεργός κατάλογος: `modular-test-chassis/stl-ski-mount/`.

| Φυσικό κομμάτι | Ενεργό STL (ποσότητα) | P2 | Βίδες / σύνδεση | Κατάσταση |
|---|---|---:|---|---|
| Βατήρας R75/75° | `stl-ski-mount/r75-75-ski.stl` ×1 | P2-73, P2-85 χωρίς bosses | 2×M4×30 χαμηλής κεφαλής στο X315 (κεφαλή μέσα) | Ήδη τυπωμένος — ο ενεργός STL είναι πλέον γεωμετρικά ο ίδιος· δεν ξανατυπώνεται |
| Αριστερό/δεξί bracket X300 | `stl-ski-mount/x300-ski-bracket-left.stl` ×1, `x300-ski-bracket-right.stl` ×1 | P2-85 | 4×M4×35 στο X300 Y±75/±100, ροδέλες/nyloc | **Τυπώνονται τώρα**, ξαπλωμένα στην παρειά Y61, χωρίς supports· το P2-73 bracket καταργείται (έσπασε) |
| Διχάλα τοιχώματος βατήρα | `stl-ski-mount/p285-ski-wall-fork-left.stl` ×1, `p285-ski-wall-fork-right.stl` ×1 | P2-85 | Hub στη M4×30 του X315· κλείδωμα 2×M4×30 DIN 933 (κεφαλή στο hex pocket) + ροδέλα Ø12/nyloc | **Τυπώνονται τώρα**, εξωτερική παρειά στο bed, χωρίς supports· αντικαθιστούν τους `ski-boss-spacer` |

Το `ski-boss-spacer.stl` (P2-73 fix 2026-10-19) είναι superseded από το hub
της διχάλας P2-85 και αφαιρέθηκε από το ενεργό release. Το full ski dry-run
αναμένει πλέον 5 STL αντί για 4, όλα ακριβώς ένα σώμα.

## Αψίδα/κλίση — P2-93

Πηγή: `arch-tilt-p293/p293-arch-tilt-release.scad`. Ενεργός κατάλογος:
`arch-tilt-p293/stl/`. Αναλυτική λίστα ποσοτήτων, μάζας, χρόνου, hardware
και σειράς συναρμολόγησης: `arch-tilt-p293/README.md`.

| Υποσύνολο | Ενεργά STL | Κατάσταση |
|---|---|---|
| Πόδια/σκέλη/διαγώνιες | `arch_foot_x240/x300`, `arch_leg_lower/upper`, `arch_brace_x240/x300`, `arch_joint_sleeve` | **Release-ready· τυπώνονται τώρα**. Μόνο υπάρχουσες P2-78 οπές, through-M4 |
| Άνω αψίδα/sensors | `arch_crossbar_center/end`, `lidar_mast`, `lidar_crossbar_half`, `sensor_joint_sleeve`, `oak_universal_plate` | **Release-ready· τυπώνονται τώρα**. Το OAK part είναι universal interface, όχι cradle |
| Φέροντα tilt stops | `tilt_stop` | **Release-ready μετά από dry assembly**· συμμετρική διαδρομή φορτίου προς αψίδα· ενσωματωμένο M3 interface για μελλοντικό striker tab |
| FIT/KFL/Tr8 interfaces | `tilt_fit0186_carrier`, `tilt_kfl08_base`, `tilt_nut_carriage` | **Fit-check μόνο** μέχρι `[ΜΕΤΡΗΣΕΙΣ ΧΡΗΣΤΗ: ...]`· μην ξεκινήσεις τελικό print |
| Encoder | `tilt_as5600_pod/cover`, `tilt_magnet_cap` | **Fit-check μόνο** μέχρι μετρήσεις AS5600· AZ1 εκκρεμεί |

Τα endpoint `tilt_switch_lower/upper` αφαιρέθηκαν από το P2-93. Η cassette
ανήκει στο release της κινούμενης μονάδας και παραμένουν ανοικτά gates:

- ενεργοποίηση roller lever ~1° πριν τα hard stops −3°/+20°,
- υπερδιαδρομή ώστε reaction/impact να περνά αποκλειστικά stop→αψίδα και
  ποτέ από τον microswitch,
- καλώδιο μέσα στο D5065 service loop με strain relief και στα δύο άκρα,
- δύο switches ανά άκρο: Mega sense + NC hardware cutoff R_EN/L_EN.

Basket floor και rear/side grids, καθώς και saddles/guides/locks, **δεν
ανήκουν στο πρώτο κύμα**: περιμένουν τα Δ1–Δ5 του mockup. Η προσωρινή
drive-test console συγκρούεται με το F και αφαιρείται πριν το F.

Το release έχει **20 one-body STL / 33 prints**, planning upper-bound
1.609,7 g PETG και 100,8 h με 0,4/0,28 ή 75,5 h με 0,6/0,30 στον P2S.
Export, P2-90e roofs και exact-solid regression:

```bash
OPENSCAD_BIN=/snap/bin/openscad-nightly bash cad/arch-tilt-p293/export-p293-stls.sh
```

## Rear pod v2 — κατάσταση P2-88

> **ΠΡΟΣΟΧΗ — ΜΗΝ ΤΥΠΩΣΕΙΣ ΠΑΛΙΑ EXPORT:** στο P2-86b άλλαξαν ξανά τα
> `base_left.stl`, `base_right.stl` και `corner_tie.stl`. Τα P2-86 base STL
> είναι superseded. Χρησιμοποίησε μόνο τα τρέχοντα αρχεία του `stl-v2/`.

Κύριες πηγές: `rear-pod/rear-pod-v2.scad`, `rear-pod/params-v2.scad`.
Ενεργός κατάλογος: `rear-pod/stl-v2/`.

| Φυσικό κομμάτι | Ενεργό STL (ποσότητα) | P2 | Βίδες / σύνδεση | Κατάσταση |
|---|---|---:|---|---|
| Ενισχυμένες βάσεις | `stl-v2/base_left.stl` ×1, `base_right.stl` ×1 | P2-79/P2-83/P2-86b | 4×M4×45 προς rear crossbar· 2×M4×20 countersunk προς ties· 4×M3 rear lands· τοπική flush-stay εγκοπή Y±165 | **ΤΥΠΩΜΕΝΕΣ P2-86b — ΚΛΕΙΔΩΜΕΝΕΣ, δεν επανεξήχθησαν στο P2-88** |
| Corner ties | `stl-v2/corner_tie.stl` ×2 | P2-83/P2-86b | 2×M4×20 countersunk προς βάση· κοινή μακρύτερη through-M4 tie→rear crossbar→stay στο `Y±165/Z37` | **ΝΕΟ P2-86b export — προς εκτύπωση** |
| Pi shelf | `stl-v2/pi_shelf.stl` ×1 | P2-76 | 4×M3, αφρός + 2 straps | Προς εκτύπωση |
| Όρθια electronics plate | `stl-v2/vertical_electronics_plate.stl` ×1 | P2-76 | 2×M4×25 στις σιαγόνες, ροδέλες/nyloc | Προς εκτύπωση |
| Side frames | `stl-v2/side_frame.stl` ×2 | P2-88/P2-90d | εμπρός 2×M4×25 στο X−132, πίσω 2×M4×35 με wedge, ροδέλες OD14· ανοικτά post cups 4×M4×60 στον υπάρχοντα άξονα X, ροδέλες/nyloc | **ΤΥΠΩΜΕΝΑ P2-90d (επιβεβαίωση χρήστη 2026-10-26) — συναρμολογημένα με panel posts και control panel**· όρθιο export, 36×24 load spines, ίδιο χαμηλό hardware pocket στα δύο base feet, support-free οροφές 45°· όλα τα προηγούμενα side-frame STL superseded |
| Jig εμπρός οπής βάσης | `stl-v2/side_frame_base_drill_jig.stl` ×1 | P2-88 | εντοπισμός στις δύο παλιές Ø4,5· τρυπάνι Ø4,5 | Προς εκτύπωση· υποχρεωτικό για τις τυπωμένες βάσεις |
| Σφηνοειδείς πίσω ροδέλες | `stl-v2/rear_foot_wedge_washer.stl` ×2 | P2-88 | κάτω από την πίσω M4 | Προς εκτύπωση |
| Parkside docks/drawers | `stl-v2/parkside_dock_left/right.stl`, `parkside_drawer_left/right.stl` ×1 | P2-88 provisional | 4×M3×12 προς υπάρχουσες slots + 2×M3 rear locks, Ruthex RX-M3x5.7 | **Σχεδιασμένο· αναμονή πραγματικής μπαταρίας/dock — ΔΕΝ ΤΥΠΩΝΕΤΑΙ ΑΚΟΜΑ** |
| Θήκη UGREEN + split caps + πίσω retainer | `stl-v2/ugreen_post_holder.stl`, `ugreen_clamp_caps.stl` ×1 print plate / 2 caps, `ugreen_rear_retainer.stl` ×1 | P2-90/P2-90b, πραγματικό 115×45×45 | 4×M3×30 caps + 2×M3×12 retainer, 6× Ruthex RX-M3x5.7, προαιρετικό λουράκι | **ΝΕΟ P2-90b export holder+caps — πλάγια τοποθέτηση caps, κεντρική εισαγωγή συσκευής από −X· κλειστά P2-90 collars superseded** |
| Panel posts | `stl-v2/panel_posts.stl` ×1 print plate / 4 posts | P2-81/P2-90d interface | κάτω/cup 4×M4×60, επάνω 4×M4×20, ροδέλες/nyloc | **ΤΥΠΩΜΕΝΑ — κλειδωμένα, καμία μεταβολή STL στο P2-90d** |
| Νευρωμένο control panel | `stl-v2/control_panel.stl` ×1 | P2-81/P2-84/P2-86 | 4×M4×20, ροδέλες/nyloc· νέα συνεχής rear beam: 4×M3 rear_hi + 4×M3 deck· οπή LED Ø8 | **ΤΥΠΩΜΕΝΟ — κλειδωμένο στο P2-90· συναρμολογημένο στα posts με E-stop, toggle και START/ARM (2026-10-26)**· physical proof-load 30 N πριν τη χρήση |
| Component pins/clips | `stl-v2/component_pins.stl` ×1, `component_clips.stl` ×1 | P2-76 | Τυπωμένα pin/C-clips, χωρίς βίδα | Προς εκτύπωση |
| BTS spacers | `stl-v2/bts_spacers.stl` ×1 | P2-76 | Σε συνδυασμό με pins/C-clips | Προς εκτύπωση |
| Cable bulkhead + feet | `stl-v2/cable_bulkhead.stl` ×1, `bulkhead_foot.stl` ×2 | P2-75 retained in v2 | Υπάρχουσες M4, ροδέλες/παξιμάδια | Προς εκτύπωση |
| Upright jaw coupon | `stl-v2/upright_mount_fit_coupon.stl` ×1 | P2-76 | M4 δοκιμής | **Τυπωμένο, fit PASS**· η σιαγόνα 4,8 mm/γλωττίδα 4 mm/M4 θέση κλειδώνουν |

Τα `rear-pod/stl-v2/mpu6050_mount.stl` και
`rear-pod/stl-v2/chassis_spacers.stl` δεν είναι ενεργά και αρχειοθετούνται.
Δεν υπάρχει M3 self-tap σε φέρον σημείο.

Τα παλιά P2-79 side frames έχουν τυπωθεί αλλά είναι **superseded**: οι τέσσερις
M4 δεν συναρμολογούνται. Δεν χρησιμοποιούνται· τα νέα όρθια P2-90d έχουν
τυπωθεί και φέρουν τα posts και το control panel.

Το P2-83 `corner_tie.stl` είναι ακριβώς ένα σώμα. Βρίσκεται κάτω από τη
βάση (`Z≤51`), στην εσωτερική παρειά της ράγας, και δεν περνά πάνω από τη
ράγα ή μέσα στο Parkside sweep. Τα print plates έχουν δηλωμένο αναμενόμενο
πλήθος: posts 4, pins 27, clips 32, BTS spacers 18 και jaw coupon 2.

## Ενεργά test fixtures

| Fixture / φυσικό κομμάτι | Ενεργό STL ή οδηγία | Πηγή | Βίδες | Κατάσταση |
|---|---|---|---|---|
| TPU friction plate | `test-fixtures/tpu-friction/plate.stl` | `tpu-friction-test.scad` | Ταινία στο τραπέζι | Μόνο δοκιμή· μέτρηση μ εκκρεμεί |
| TPU friction sled | `test-fixtures/tpu-friction/sled.stl` | `tpu-friction-test.scad` | Νήμα/βάρη, χωρίς φέρουσα βίδα | Μόνο δοκιμή |
| TPU pulley + bracket | `test-fixtures/tpu-friction/pulley.stl`, `pulley_bracket.stl` | `tpu-friction-test.scad` | M4×25 + παξιμάδι ως άξονας | Μόνο δοκιμή |
| Ski hand-test, τέσσερις παραλλαγές | `test-fixtures/ski-jump-hand-test/ski_42deg_R75.stl`, `ski_60deg_R75.stl`, `ski_75deg_R75.stl`, `ski_75deg_R110.stl` | `ski-jump-test.scad` | Ταινία/σφιγκτήρας στο τραπέζι | Μόνο χειροκίνητη δοκιμή |
| Basket mockup | Δεν έχει STL· διαστάσεις στο `test-fixtures/basket-mockup/README.md` | P2-71 cardboard fixture | Ταινία/χαρτόνι | Μόνο δοκιμή· pile retention 40/45 μπαλών εκκρεμεί |
| Drive-motor hole gauge | `drive-motor-mount/37mm-mount-hole-gauge.stl` | `drive-motor-mount/37mm-mount-hole-gauge.scad` | Πραγματικές βίδες steel mount μόνο για fit | Τυπωμένο· μόνο δοκιμή, fit-checked σε 1/4 mounts |
| M3 nut-keeper sleeve | `drive-motor-mount/nut-keeper-stl/sleeve.stl` | `drive-motor-mount/m3-side-nut-keeper.scad` | Δεν φέρει preload· συγκρατεί μόνο M3 nut | Μόνο δοκιμή εφαρμογής· physical retention εκκρεμεί |

## Panels F rear — P2-86b, αναμονή / ΔΕΝ ΤΥΠΩΝΕΤΑΙ ΑΚΟΜΑ

Πηγή: `body-panels/body-panels-f-rear-v2.scad`. Release-candidate κατάλογος:
`body-panels/stl-f-rear/`. Τα δεκατρία STL (`rear_lo` L/R, `rear_hi` L/R,
`deck_rear` L/R, Parkside doors L/R, corner stays L/R, δύο εσωτερικά
center splices και turn-button που τυπώνεται ×2) είναι manifold, ένα σώμα και με print footprint έως
250×250 mm, αλλά μένουν σε κατάσταση **σχεδιασμένο, αναμονή** μέχρι το
μελλοντικό βήμα πλαϊνών/εμπρός και το basket mockup. Βίδες και σειρά
συναρμολόγησης βρίσκονται στο
`body-panels/stl-f-rear/README.md`.
Το πραγματικό R60 είναι εφαπτόμενο στο R424,5 και στη Y±244,5, με P2-87
ραφή μετά το fillet στο X−185. Τα `rear_lo`, `rear_hi`, `deck_rear` και
`corner_stay` και `center_splice` STL είναι νέα P2-86b exports. Τα skin
halves είναι 246,85 mm στη μεγάλη print διάσταση και τα deck halves
δίνονται επίπεδα σε 45° orientation (≤185×185 mm). Ο περιμετρικός χάρτης βρίσκεται
στο `docs/design/body-panels/p286b-perimeter-map-el.md`.

## Concepts — F επιλεγμένο, χωρίς ενεργό STL πλαϊνών/εμπρός

- **P2-80 rear cage:** μελλοντικό πλήρως τυπωμένο stackable PETG cage,
  χωρίς αλουμίνιο. Box sections περίπου 35×35 mm δεν χωρούν στη λωρίδα
  `Y161…167`, άρα θέσεις/διατομές κολονών θα ξανακριθούν με τα panels F.
- **Body panels F πλαϊνά/εμπρός:** `body-panels/body-panels-f.scad` και τα
  F renders/QA παραμένουν concept. Το rear subset έχει πλέον ξεχωριστή
  release-candidate γεωμετρία παραπάνω, αλλά δεν έχει εγκριθεί για εκτύπωση.
- **Concepts A–E:** superseded από F στο P2-86 και μεταφέρθηκαν σε
  `body-panels/archive/concepts-a-e/` με σήμανση **ΜΗΝ ΤΥΠΩΣΕΙΣ**.
- **Full robot v3:** `modular-test-chassis/full-robot-v3.scad` είναι assembly
  concept/check, όχι πηγή ενεργών STL.

## Ανοικτές φυσικές δοκιμές

Drive test με PSU, μέτρηση TPU `μ`, basket mockup/pile retention, ski hand
test και τελικό proof-load του P2-81 control panel. Καμία από αυτές τις
εκκρεμότητες δεν μετατρέπει concept CAD σε ενεργό STL.
