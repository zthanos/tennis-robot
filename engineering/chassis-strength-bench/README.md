# Chassis strength bench

Ανεξάρτητο benchmark για την απόφαση αν το αρθρωτό open-front test chassis
μπορεί να τυπωθεί όπως είναι ή χρειάζεται αλλαγή πριν καταναλωθεί υλικό.

Το benchmark χωρίζει επίτηδες δύο διαφορετικά προβλήματα:

1. Το Gazebo υπολογίζει τα δυναμικά φορτία στους τέσσερις άξονες κατά τη
   στροφή επί τόπου. Το μοντέλο είναι rigid-body και **δεν** υπολογίζει τάσεις
   ή παραμόρφωση του πλαστικού.
2. Το structural screen μετατρέπει τα φορτία σε τάση/παραμόρφωση για την
   κλειστή δοκό, το drive node και το splice και συγκρίνει PLA, PETG και
   PETG-CF.

Οι αρχικές τιμές είναι συντηρητικές υποθέσεις και όχι μετρήσεις του τελικού
τυπώματος. Ιδίως η μάζα, η πραγματική τριβή ελαστικού/γηπέδου, η γεωμετρία
οπών και οι ιδιότητες μεταξύ στρώσεων πρέπει να αντικατασταθούν από μετρήσεις.

## Γρήγορη εκτέλεση

Το αναλυτικό screen δεν χρειάζεται ROS ή Gazebo:

```bash
python3 engineering/chassis-strength-bench/analyze.py \
  --output engineering/chassis-strength-bench/results/baseline.json
```

Το Jazzy/Harmonic rigid-body benchmark εκτελείται με:

```bash
engineering/chassis-strength-bench/run_jazzy_harmonic.sh
```

Η πλήρης αρθρωτή v2 συναρμολόγηση, με τέσσερα motor modules, δύο σώματα Γ,
πίσω γωνίες, πίσω τραβέρσα, intake halves και αισθητήρες σε κάθε σύνδεση,
εκτελείται ανεξάρτητα με:

```bash
engineering/chassis-strength-bench/run_modular_integration_bench.sh
CHASSIS_PAYLOAD_MASS_KG=10 \
  engineering/chassis-strength-bench/run_modular_integration_bench.sh
```

Τα δύο runs συνδυάζονται σε design loads με:

```bash
python3 engineering/chassis-strength-bench/summarize_modular_integration.py \
  engineering/chassis-strength-bench/results/modular_integration/payload_0kg/wrench_summary.json \
  engineering/chassis-strength-bench/results/modular_integration/payload_10kg/wrench_summary.json \
  --output engineering/chassis-strength-bench/results/modular_integration/design_loads.json
```

Το integration report χρησιμοποιεί το `p99.5` μετά από χρόνο settle, ώστε
μονοβηματικά spawn/solver impulses να μη μετατρέπονται σε μηχανικό design
load. Τα απόλυτα maxima διατηρούνται στο JSON μόνο για diagnostics.

Μετά το Gazebo run, η ενσωμάτωση των μετρημένων peaks γίνεται με:

```bash
python3 engineering/chassis-strength-bench/analyze.py \
  --wrench-summary engineering/chassis-strength-bench/results/jazzy_harmonic/wrench_summary.json \
  --fea-summary engineering/chassis-strength-bench/results/drive_node_fea.json \
  --output engineering/chassis-strength-bench/results/baseline.json
```

Το script χρησιμοποιεί το system Gazebo Harmonic CLI και δεν εκκινεί τον
πλήρη robot stack. Τα ίδια SDF/config θα χρησιμοποιηθούν αργότερα στο
Lyrical/Jetty compatibility run.

Το τοπικό 3D FEM του drive node εκτελείται με:

```bash
python3 engineering/chassis-strength-bench/run_drive_node_fea.py
```

Χρησιμοποιεί τα Gmsh και CalculiX που διανέμονται μέσα στο εγκατεστημένο
FreeCAD snap, χωρίς GUI και χωρίς να αλλάζει το FreeCAD project του robot.

## Κριτήριο απόφασης

- `PRINT_CANDIDATE`: όλα τα επιμέρους screens έχουν safety factor τουλάχιστον
  2,0 και η προβλεπόμενη παραμόρφωση είναι εντός ορίου.
- `REDESIGN_BEFORE_PRINT`: οποιοδήποτε load path πέφτει κάτω από 2,0 ή λείπει
  κρίσιμη φυσική μέτρηση.
- `BENCHMARK_INCOMPLETE`: δεν έχουν ακόμη εισαχθεί μετρημένα peak loads από
  Gazebo/φυσική δοκιμή.
- `PRINT_LOCAL_COUPON_NOT_FULL_CHASSIS`: το numerical baseline πέρασε, αλλά
  πριν από όλα τα segments απαιτείται ένα μικρό rail+drive-node coupon.

Ακόμη και `PRINT_CANDIDATE` σημαίνει μόνο ότι αξίζει ένα πρωτότυπο χαμηλού
ρίσκου. Δεν αποτελεί πιστοποίηση ασφαλείας ούτε αντικαθιστά δοκιμή μέχρι
αστοχία σε τυπωμένο coupon με τις πραγματικές ρυθμίσεις slicer.

## Material profiles

Τα profiles κρατούν δύο διαφορετικά μεγέθη:

- `reference_*`: δημοσιευμένες/τυπικές τιμές για σύγκριση υλικών.
- `screen_strength_mpa`: μειωμένη χαρακτηριστική αντοχή για το πρώτο design
  screen, πριν υπάρξουν δικά μας coupons.

Το PETG-CF μοντελοποιείται πιο άκαμπτο από το PETG αλλά όχι αυτόματα πιο
ανθεκτικό σε κρούση. Απαιτεί hardened nozzle και ξηρό filament. Για όλα τα
υλικά το τελικό profile πρέπει να προκύψει από coupon τυπωμένο στην ίδια
διεύθυνση με τη δοκό.

## Βάρος φορτίου

Το βάρος χωρίζεται πλέον σε `base_mass_kg` και `payload_mass_kg`. Το πρώτο
περιλαμβάνει το κινούμενο δοκιμαστικό σασί και το δεύτερο ό,τι προστίθεται:
μπαταρία, καλάθι και μπάλες. Το `payload_sweep` του `config/baseline.json`
υπολογίζει αυτόματα 0/2/5/8/10 kg πάνω σε βάση 10 kg.

Το τρέχον sweep υποθέτει κεντραρισμένο φορτίο και ίση κατανομή στους τέσσερις
τροχούς. Η κατακόρυφη δύναμη και η κάμψη αυξάνονται γραμμικά με τη συνολική
μάζα. Η δύναμη στροφής αυξάνεται με την πρόσφυση μέχρι περίπου 20 kg συνολικής
μάζας, όπου γίνεται πλέον περιορισμένη από τη ροπή των μοτέρ. Η θέση του
κέντρου βάρους θα προστεθεί όταν μετρηθούν καλάθι και μπαταρία.

## Απόσταση motor από τη γωνία

Το `corner_proximity_sweep` συνδυάζει την κατακόρυφη κάμψη, τη ροπή yaw και τη στρέψη της δοκού.
Μέχρι να υπάρξει πλήρες corner CAD/FEM, χρησιμοποιεί συντελεστή συγκέντρωσης 2,5. Με το
συντηρητικό turn load και απόσταση 100 mm από το κέντρο του motor μέχρι τη γωνία, η screen τάση είναι
10,71 MPa και οι safety factors είναι 2,80 PLA, 2,43 PETG και 2,62 PETG-CF. Στα 200 mm το PETG και το
PETG-CF πέφτουν κάτω από SF 2. Άρα η μικρή απόσταση δεν είναι από μόνη της δυσμενής· μειώνει τον
μοχλοβραχίονα. Η τοπική γωνία, το μανίκι, οι βίδες και η διεύθυνση των layers παραμένουν το κρίσιμο θέμα.
