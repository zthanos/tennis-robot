# Ενεργό σύνολο εκτύπωσης — P2-82

Η παρούσα λίστα είναι το μοναδικό ευρετήριο για κομμάτια που είναι ενεργά
στη σημερινή φυσική κατασκευή. Αρχεία κάτω από `archive/` ή με σήμανση
«concept» **δεν τυπώνονται**. Η κατάσταση αποτυπώνει όσα είναι γνωστά στις
2026-10-19· όταν ένα φυσικό part αλλάξει, ενημερώνεται πρώτα αυτό το αρχείο.

## Σασί v3 — P2-78 sockets και πλευρικές διαγώνιες

Κύρια πηγή: `modular-test-chassis/modular-chassis-v3.scad`.
Ενεργός κατάλογος: `modular-test-chassis/stl-v3/`.

| Φυσικό κομμάτι | Ενεργό STL (ποσότητα) | P2 | Βίδες / σύνδεση | Κατάσταση |
|---|---|---:|---|---|
| Fit coupon μανικιού | `stl-v3/sleeve_fit_coupon.stl` ×1 | pre-P2-78 fit gate | M4 δοκιμής | Τυπωμένο· μόνο δοκιμή |
| Κύρια μανίκια 50×30 | `stl-v3/clamp_sleeve.stl` ×5 | v3/P2-78 retained | M4×40 top, M4×60 side, ροδέλες/παξιμάδια | Τυπωμένα |
| Πίσω γωνίες | `stl-v3/corner_clamp_left.stl` ×1, `corner_clamp_right.stl` ×1 | P2-78 retained/fixed | M4×40 top, M4×60 side | Προς εκτύπωση |
| Πίσω δοκοί κινητήρων | `stl-v3/rear_motor_left.stl` ×1, `rear_motor_right.stl` ×1 | v3/P2-78 retained | Hardware FIT0403 + joint M4 | Τυπωμένα |
| Μεσαίες πλευρικές δοκοί | `stl-v3/rail_extension_left.stl` ×1, `rail_extension_right.stl` ×1 | v3/P2-78 retained | Joint M4×40/60 | Τυπωμένα |
| Εμπρός terminal δοκοί με sockets | `stl-v3/front_terminal_left.stl` ×1, `front_terminal_right.stl` ×1 | P2-78 | Joint M4 + socket locks | Προς εκτύπωση |
| Πίσω τραβέρσα | `stl-v3/rear_crossbar_half.stl` ×2 | v3 corner fix/P2-78 retained | M4×40/60 | Τυπωμένη παλιότερη έκδοση· διορθωμένη προς εκτύπωση |
| Τραβέρσα X240 | `stl-v3/x240_crossbar_half.stl` ×2 | P2-78 | 4×M4×30 top + 4×M4×60 transverse συνολικά | Προς εκτύπωση |
| Κεντρικό μανίκι X240 | `stl-v3/x240_clamp_sleeve.stl` ×1 | v3/P2-78 retained | M4×30/60 | Τυπωμένο |
| Terminal μανίκια X240 | `stl-v3/x240_terminal_sleeve.stl` ×2 | P2-78 | M4 locks Y75/100/125/150 | Προς εκτύπωση |
| Τραβέρσα X300 | `stl-v3/x300_crossbar_half.stl` ×2 | P2-78 | 2×M4×30 top + 4×M4×30 transverse συνολικά | Προς εκτύπωση |
| Κεντρικό μανίκι X300 | `stl-v3/x300_clamp_sleeve.stl` ×1 | v3/P2-78 retained | M4×30 | Τυπωμένο |
| Terminal μανίκια X300 | `stl-v3/x300_terminal_sleeve.stl` ×2 | P2-78 | M4 locks Y100/160 | Προς εκτύπωση |
| Πλευρικές torsion διαγώνιες | `stl-v3/torsion_side_brace.stl` ×2 | P2-78 | ανά brace M4×65 στο X240/Y150 + M4×35 στο X300/Y125, μεγάλες ροδέλες/παξιμάδια | Προς εκτύπωση |

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

## Βατήρας R75/75° — P2-73 με διόρθωση 2026-10-19

Πηγή: `modular-test-chassis/shared-flywheel-p273-bolted-ski.scad`.
Ενεργός κατάλογος: `modular-test-chassis/stl-ski-mount/`.

| Φυσικό κομμάτι | Ενεργό STL (ποσότητα) | P2 | Βίδες / σύνδεση | Κατάσταση |
|---|---|---:|---|---|
| Βατήρας R75/75° | `stl-ski-mount/r75-75-ski.stl` ×1 | P2-73 + fix 2026-10-19 | Κανονικά 2×M4×25 στους X315 bosses | Ήδη τυπωμένος χωρίς bosses· ενεργό STL είναι η διορθωμένη ενιαία έκδοση με bosses Y49,5…60,5 |
| Αριστερό/δεξί bracket X300 | `stl-ski-mount/x300-ski-bracket-left.stl` ×1, `x300-ski-bracket-right.stl` ×1 | P2-73 | 4×M4×35 στο X300 Y±75/±100, ροδέλες/nyloc | Προς εκτύπωση |
| Retrofit αποστάτες boss | `stl-ski-mount/ski-boss-spacer.stl` ×2 | P2-73 fix 2026-10-19 | 2×M4×30: χαμηλή κεφαλή μέσα, τοίχωμα, spacer, bracket, ροδέλα/nyloc έξω | Προς εκτύπωση για τον ήδη τυπωμένο βατήρα |

Το STL της 2026-10-18 με αιωρούμενα bosses δεν είναι ενεργό. Η τρέχουσα
γεωμετρία είναι ένα σώμα: κάθε boss επικαλύπτει κατά 0,5 mm το sidewall και
αφήνει 0,5 mm προς το bracket.

## Rear pod v2 — κατάσταση P2-81

Κύριες πηγές: `rear-pod/rear-pod-v2.scad`, `rear-pod/params-v2.scad`.
Ενεργός κατάλογος: `rear-pod/stl-v2/`.

| Φυσικό κομμάτι | Ενεργό STL (ποσότητα) | P2 | Βίδες / σύνδεση | Κατάσταση |
|---|---|---:|---|---|
| Ενισχυμένες βάσεις | `stl-v2/base_left.stl` ×1, `base_right.stl` ×1 | P2-79 | 4×M4×45 προς rear crossbar, μεγάλες ροδέλες/nyloc | Προς εκτύπωση |
| Corner ties | `stl-v2/corner_tie.stl` ×2 | P2-79 | 2×M4×65 μαζί με τις side M4 των corner clamps | Προς εκτύπωση |
| Pi shelf | `stl-v2/pi_shelf.stl` ×1 | P2-76 | 4×M3, αφρός + 2 straps | Προς εκτύπωση |
| Όρθια electronics plate | `stl-v2/vertical_electronics_plate.stl` ×1 | P2-76 | 2×M4×25 στις σιαγόνες, ροδέλες/nyloc | Προς εκτύπωση |
| Side frames | `stl-v2/side_frame.stl` ×2 | P2-79 | 4×M4×25 διαμπερώς, ροδέλες/nyloc | Προς εκτύπωση |
| Panel posts | `stl-v2/panel_posts.stl` ×1 print plate / 4 posts | P2-81 | κάτω 4×M4×30, επάνω 4×M4×20, ροδέλες/nyloc | Προς εκτύπωση |
| Νευρωμένο control panel | `stl-v2/control_panel.stl` ×1 | P2-81 | 4×M4×20, ροδέλες/nyloc | Προς εκτύπωση· physical proof-load 30 N πριν τη χρήση |
| Component pins/clips | `stl-v2/component_pins.stl` ×1, `component_clips.stl` ×1 | P2-76 | Τυπωμένα pin/C-clips, χωρίς βίδα | Προς εκτύπωση |
| BTS spacers | `stl-v2/bts_spacers.stl` ×1 | P2-76 | Σε συνδυασμό με pins/C-clips | Προς εκτύπωση |
| Cable bulkhead + feet | `stl-v2/cable_bulkhead.stl` ×1, `bulkhead_foot.stl` ×2 | P2-75 retained in v2 | Υπάρχουσες M4, ροδέλες/παξιμάδια | Προς εκτύπωση |
| Upright jaw coupon | `stl-v2/upright_mount_fit_coupon.stl` ×1 | P2-76 | M4 δοκιμής | Μόνο δοκιμή· τυπώνεται πρώτο |

Τα `rear-pod/stl-v2/mpu6050_mount.stl` και
`rear-pod/stl-v2/chassis_spacers.stl` δεν είναι ενεργά και αρχειοθετούνται.
Δεν υπάρχει M3 self-tap σε φέρον σημείο.

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

## Concepts — παραμένουν, χωρίς ενεργό STL

- **P2-80 rear cage:** μελλοντικό πλήρως τυπωμένο stackable PETG cage,
  χωρίς αλουμίνιο. Box sections περίπου 35×35 mm δεν χωρούν στη λωρίδα
  `Y161…167`, άρα θέσεις/διατομές κολονών θα ξανακριθούν με τα panels F.
- **Body panels F:** `body-panels/body-panels-f.scad` και renders/QA είναι
  concept, όχι print release.
- **Full robot v3:** `modular-test-chassis/full-robot-v3.scad` είναι assembly
  concept/check, όχι πηγή ενεργών STL.

## Ανοικτές φυσικές δοκιμές

Drive test με PSU, μέτρηση TPU `μ`, basket mockup/pile retention, ski hand
test και τελικό proof-load του P2-81 control panel. Καμία από αυτές τις
εκκρεμότητες δεν μετατρέπει concept CAD σε ενεργό STL.
