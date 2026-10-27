# P2-59 — 5d-1 μονή εκτόξευση, τροφοδοσία και break-beam

> **ΙΣΤΟΡΙΚΟ — δεν περιγράφει το τρέχον design.** Διατηρείται ως evidence
> της απόφασης/δοκιμής της εποχής του. Δες
> [το τρέχον design](../current-design-el.md).

> **ΑΝΤΙΚΑΤΑΣΤΑΘΗΚΕ από P2-61 ως προς launch/contact αριθμούς:** το plateau
> 6,2/7,7 m/s προήλθε από το λανθασμένο per-contact compression mapping.
> Break-beam X320, T=100 ms, pins και fail-closed feed baseline παραμένουν·
> το corrected 5d-1 αποτέλεσμα βρίσκεται στο P2-61 report.

## Συμπέρασμα

Το P2-59 κλείνει θετικά το sensing και το μηχανικό bench, αλλά **δεν
αποδεσμεύει launcher 12–14 m/s**. Τα 48/48 Gazebo runs ολοκληρώθηκαν χωρίς
ανεπιθύμητη επαφή και με το όχημα ακίνητο, όμως το ίδιο βαθμονομημένο
P2-51/5c-3 contact model κορέστηκε στα 6,23–6,27 m/s για μ=0,6 και
7,66–7,69 m/s για μ=0,9. Η αύξηση surface speed από 20 σε 24 m/s πρόσθεσε
μόνο 0,0024–0,0052 m/s. Επομένως η απαιτούμενη surface speed για 12 ή
14 m/s είναι μη προσδιορίσιμη· μεγαλύτερες rpm από μόνες τους δεν λύνουν
το όριο επαφής/traction του παρόντος μοντέλου.

Το frozen P2-40 core (pivot X387, τροχοί Ø200×60 στα Y±129, nip 58 mm,
rails Y±220×50) και το P2-51 R75/42° ski-jump/καλάθι δεν άλλαξαν. Δεν έγινε
αλλαγή firmware, STL, commit, push ή PR.

## Break-beam και combined jam detector

Η offline αναπαράσταση του beam ως οριζόντια γραμμή δίνει την ακόλουθη
θέση στο `base_link`:

- κέντρο: **X=320 mm, Z=50 mm**,
- δέσμη κατά Y, από **Y=−140 έως +140 mm**,
- θέση μετά το nip, πάνω στο ski/στην είσοδο του καλαθιού.

Η θέση αναζητήθηκε πάνω στα πραγματικά `observer.csv` και `telemetry.csv`
όλων των διαθέσιμων P2-55/56/58 folders. Και τα οκτώ P2-55 sequential
passes, όλα τα 57 P2-58 passes και ειδικά το `series_gap_0` παράγουν δύο
διακριτούς παλμούς. Το ελάχιστο καθαρό διάστημα μεταξύ των δύο παλμών είναι
**105 ms**, αρκετά μεγαλύτερο από το υπάρχον firmware debounce των 25 ms.

Motor event ορίζεται ως κοινό mean deficit των δύο D5065 **>5 rpm για
≥50 ms**. Στα pass logs ο χρόνος event→beam είναι **30–60 ms**. Επιλέγεται
**T=100 ms**, δηλαδή 25 ms επιπλέον περιθώριο πάνω από το χειρότερο
μετρημένο latency, στρογγυλοποιημένο σε tick 25 ms.

Offline αποτέλεσμα σε 87 cases:

| Φάση | PASS | JAM | REJECT | Combined detector |
|---|---:|---:|---:|---|
| P2-55 | 8 | 4 | 0 | 0/8 false positives, 4/4 jams |
| P2-56 | 0 | 12 | 0 | 12/12 jam-origin diagnostics |
| P2-58 | 57 | 4 | 2 | 0/57 false positives, 4/4 jams |

Συνολικά: **0/65 false positives και 20/20 logged jams detected**. Τα δύο
P2-58 `REJECT` δεν είναι jams και δεν απαιτούν recovery. Ο runtime κανόνας
είναι: ξεκίνα timer όταν επιβεβαιωθεί το rpm event· αν δεν υπάρξει
debounced crossing του downstream beam μέσα σε 100 ms, δήλωσε jam και
επέτρεψε το P2-56 reverse fallback.

Artifact:
`engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5d1_p259/break_beam_offline_calibration.json`.

## Τροφοδοσία και singulator

Το P2-52 selector ανοίγει το launch branch μόνο στα +15…+20°, αλλά δεν
μετρά μία μπάλα. Η προτεινόμενη λύση είναι μονό-servo, spring-return
two-finger escapement μέσα στο ήδη δεσμευμένο envelope 80×100×88 mm:

1. Δύο στρογγυλεμένα, χαμηλής τριβής δάκτυλα απέχουν 72 mm κατά τη ροή.
   Σε ηρεμία το downstream δάκτυλο είναι κλειστό και συγκρατεί την πρώτη
   μπάλα, ενώ το upstream δέχεται την επόμενη.
2. Ένα μικρό servo περιστρέφει έκκεντρο: κλείνει πρώτα το upstream finger,
   μετά ανοίγει το downstream και δίνει περίπου 60 mm controlled handoff
   από τη θύρα X278/Z42 προς το πίσω στόμιο του nip.
3. Ελατήριο επιστροφής και μηχανικό stop κάνουν την απώλεια ισχύος
   **fail-closed**. Το P2-52 collection branch μένει κλειστό στη launch
   θέση.
4. Release επιτρέπεται μόνο με AS5600 μέσα σε 14,5…20,5°, όχημα ακίνητο,
   και τους δύο D5065 ≥95% του setpoint για ≥200 ms. Το upstream beam πρέπει
   να βλέπει διαθέσιμη μπάλα. Το downstream beam πρέπει να επιβεβαιώσει
   crossing· αλλιώς, μετά από T=100 ms, κλείσιμο και jam handling.

Η πλήρης διαδρομή X278→nip δεν υπάρχει ως ξεχωριστό low-feed collision
channel στο σημερινό xacro: η storage floor/ski επιφάνεια καταλαμβάνει αυτό
το abstraction. Γι' αυτό το 5d-1 physics injection αρχίζει στο downstream
singulator handoff, 50 mm πίσω από το nip. Το bench επαληθεύει το nip και
την εκτόξευση, **όχι** τη φυσική κατασκευή/αξιοπιστία του chute ή του
escapement. Αυτά παραμένουν physical design gate.

## Mega pin audit

Το τρέχον integrated sketch είναι
`arduino/motion/motion_intake_mega/motion_intake_mega.ino` και συμφωνεί με
το J_MEGA/J_IR wiring document.

| Λειτουργία | Pin | Κατάσταση |
|---|---|---|
| Feed/entry beam | D36 | ήδη δεσμευμένο και καλωδιωμένο· τοποθέτηση στη θύρα X278, κέντρο περίπου Z75 mm |
| Ski/basket downstream beam | D37 | ήδη δεσμευμένο και καλωδιωμένο· X320/Y±140/Z50 mm |
| AS5600 | D20 SDA, D21 SCL | κοινό I²C, όχι ελεύθερα pins· 0x36 δεν συγκρούεται με MPU6050 0x68/0x69 |
| Servo option | D7 | πραγματικά αχρησιμοποίητο από το integrated Mega sketch και εκτός του τρέχοντος 34-pin J_MEGA harness |

Δεν χρειάζονται νέα pins για τα δύο beams. Για servo, το D7 μπορεί να γίνει
σήμα ή power-enable μόνο μετά από harness update. **Δεν επιτρέπεται naïve
`Servo.h`**: στον Mega δεσμεύει Timer5 και απενεργοποιεί PWM στα D44/D45/D46,
τα οποία χρησιμοποιούν οι intake BTS7960. Προτιμάται PCA9685 στο υπάρχον
I²C για servo pulse, με D7 προαιρετικό fail-safe power enable. Απαιτείται
έλεγχος επιπέδων/pull-ups πριν προστεθεί δεύτερος I²C κλάδος. Firmware δεν
τροποποιήθηκε.

## 5d-1 Gazebo matrix

Τα criteria παγώθηκαν πριν από το πρώτο run, SHA-256
`b3fe914bd63f5570ec4d6752f7d7b93bee8ae1d02f5e29274dc29d9628b8877b`.
Έτρεξαν 2 γωνίες × 2 συντελεστές τριβής × 4 surface speeds × 3 repeats.
Οι τροχοί περιστράφηκαν αντίστροφα από τη συλλογή, το όχημα παρέμεινε
ακίνητο και μία μπάλα μπήκε από πίσω.

| Γωνία | μ | Surface m/s | Exit m/s | Elevation ° | Spin rad/s | RPM drop/τροχό |
|---:|---:|---:|---:|---:|---:|---:|
| 15 | 0,6 | 12 / 16 / 20 / 24 | 6,256 / 6,263 / 6,267 / 6,270 | 10,68 / 10,03 / 9,65 / 9,40 | 0,610 / 0,461 / 0,370 / 0,308 | 28,55 / 29,16 / 30,08 / 31,37 |
| 15 | 0,9 | 12 / 16 / 20 / 24 | 7,679 / 7,685 / 7,688 / 7,691 | 12,37 / 11,74 / 11,38 / 11,14 | 0,713 / 0,541 / 0,434 / 0,362 | 36,23 / 36,95 / 37,89 / 39,11 |
| 20 | 0,6 | 12 / 16 / 20 / 24 | 6,230 / 6,237 / 6,241 / 6,244 | 15,70 / 15,05 / 14,66 / 14,41 | 0,626 / 0,473 / 0,379 / 0,317 | 28,75 / 29,42 / 30,41 / 31,78 |
| 20 | 0,9 | 12 / 16 / 20 / 24 | 7,659 / 7,665 / 7,669 / 7,674 | 17,37 / 16,74 / 16,38 / 16,21 | 0,727 / 0,552 / 0,443 / 0,370 | 36,27 / 36,94 / 37,82 / 38,99 |

Η azimuth απόκλιση και η διασπορά των τριών deterministic repeats είναι
αριθμητικά μηδενικές. Peak spin-up και contact current είναι 20 A ανά
κινητήρα, peak torque 0,62 N·m, και δεν καταγράφηκε καμία επαφή με
cheeks/arch/sensors. Το drop είναι <5% του setpoint, επομένως το
`recovery_to_95%` είναι 0 s ήδη στο τέλος της επαφής. Το αποτέλεσμα 48/48
PASS σημαίνει ότι πέρασαν τα frozen mechanical/safety criteria· δεν σημαίνει
ότι επιτεύχθηκε η ζητούμενη exit speed.

Aggregate:
`engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5d1_p259/summary.json`.

## Static rotor screen

Οι 12/16/20/24 m/s αντιστοιχούν σε 1146/1528/1910/2292 rpm. Το
thin-free-ring screen `σ=ρv²`, με ρ=1200 kg/m³, δίνει hoop stress
0,173/0,307/0,480/0,691 MPa. Η ανεμπόδιστη radial expansion για το εύρος
TPU modulus 10/25/50 MPa είναι αντίστοιχα:

- στα 12 m/s: 1,728 / 0,691 / 0,346 mm,
- στα 24 m/s: 6,912 / 2,765 / 1,382 mm.

Αυτό δεν είναι burst proof: δεν περιλαμβάνει bonding, layers, creep,
θερμοκρασία, imbalance ή hub constraint, και το repository δεν έχει
allowable για το πραγματικό TPU.

Με KV=270 rpm/V, Kt=0,031 N·m/A και R=0,039 Ω, το 24 m/s point θέλει
back-EMF 8,49 V και 9,27 V στα 20 A. Άρα υπάρχει υπολογιστικό voltage
margin 5,73 V στα 15 V και 11,73 V στα 21 V. Οι rpm είναι εφικτές με
electrical margin, όχι όμως πιστοποιημένο continuous thermal margin.

Η πλήμνη **δεν αποδεσμεύεται**: το repository απαγορεύει τυπωμένη
torque-transmitting hub. Το μοντέλο έχει μόνο primitive aluminium clamp-hub
envelope 78,20 g και `I=2,134×10⁻⁵ kg·m²`, όχι ελεγχόμενο σχέδιο. Στα
24 m/s η επιτάχυνση στο χείλος είναι 5760 m/s² και κάθε 1 g imbalance σε
R100 mm φορτίζει ακτινικά με 5,76 N· στο 1,5× overspeed γίνεται 12,96 N/g.
Η διεπαφή πρέπει επίσης να μεταφέρει 0,62 N·m. Χωρίς πραγματικό υλικό,
διαστάσεις, bore, engagement, axial retention, balance και torque rating
δεν μπορεί να υπολογιστεί έντιμα hub stress/expansion. Πριν από φυσική
δοκιμή απαιτούνται controlled drawing, runout/balance, guarded 1,5×
overspeed, thermal soak και post-test inspection.

Artifact:
`engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5d1_p259/rotor_static.json`.

## Recorded οριακές περιπτώσεις και playback

Δεν υπήρξε failed mechanical run. Καταγράφηκαν τα τέσσερα υψηλότερα-rpm
οριακά cases (24 m/s, repeat 1). Το playback ανοίγει paused και δεν αλλάζει
το `5c3_side_gui.config`:

```sh
engineering/shared-flywheel-isolated-bench/replay_5d1_p259_case.sh 15 0.6 24 1
engineering/shared-flywheel-isolated-bench/replay_5d1_p259_case.sh 15 0.9 24 1
engineering/shared-flywheel-isolated-bench/replay_5d1_p259_case.sh 20 0.6 24 1
engineering/shared-flywheel-isolated-bench/replay_5d1_p259_case.sh 20 0.9 24 1
```

## Κατάσταση

`P2_59_BEAM_DETECTOR_PASS_MECHANICAL_RUNS_48_OF_48_LAUNCH_TARGET_NOT_REACHED_HUB_AND_FEED_PHYSICAL_GATES_OPEN`
