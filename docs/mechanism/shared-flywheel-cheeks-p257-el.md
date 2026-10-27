# P2-57 — cheeks/covers για πρόληψη side-by-side

> **ΙΣΤΟΡΙΚΟ — δεν περιγράφει το τρέχον design.** Διατηρείται ως evidence
> της απόφασης/δοκιμής της εποχής του. Δες
> [το τρέχον design](../current-design-el.md).

## Απόφαση

Η P2-57 μήτρα ολοκληρώθηκε, αλλά **καμία παραλλαγή δεν περνά όλα τα
κριτήρια**. Η καλύτερη μετρημένη παραλλαγή είναι η
`p257_stagger_open`: μηδέν σφηνώματα, sequential 8/8, τέσσερις μπάλες στο
καλάθι από τα έξι side-by-side runs και πραγματικό συμμετρικό capture
width ±60 mm. Δεν προτείνεται όμως για release, επειδή κάθε απορριφθείσα
μπάλα παρασύρεται μπροστά από το όχημα και η χειρότερη +X μετατόπιση είναι
4,93918 m, πολύ πάνω από το όριο 0,30 m.

Το συμπέρασμα είναι ότι τα τυπωμένα covers λύνουν το αρχικό wheel-face
jam, αλλά ένα άκαμπτο παθητικό cheek που κινείται με 0,6 m/s δεν αφήνει
την απορριφθείσα μπάλα ελεύθερη στο γήπεδο. Η P2-57 επομένως κλείνει ως
πλήρης συγκριτική δοκιμή, όχι ως αποδεκτό production baseline.

## Παγωμένο scope και παραλλαγές

Δεν άλλαξαν pivot, τροχοί, nip, rails, P2-51 R75/42° ski-jump ή καλάθι.
Το bench διατήρησε το ίδιο P2-51/5c-3 ball/contact model και calibration.
Οι μόνες αλλαγές είναι collision/visual solids των τυπωμένων εμπρός
cheeks/covers, με λεία επιφάνεια `mu=0,05`:

- `p257_shield_m90/m100/m110`: συμμετρικά cheeks με πλήρες καθαρό mouth
  90/100/110 mm, throat 92 mm και covers μπροστά από το εξωτερικό τμήμα
  των wheel faces, X130 mm και Y±46…±190 mm.
- `p257_stagger_open`: mouth 140 mm, ίδιο throat, και +Y cheek από
  `(X180,Y55)` έως `(X250,Y100)` mm. Η εμπρός άκρη έχει ακτίνα 3 mm και
  προηγείται κατά 70 mm.
- `p257_stagger_shield`: ίδιο stagger με τα covers του V1.

Ο ακριβής CAD/bench ορισμός βρίσκεται στα
`shared-flywheel-transition-p252.scad` και
`cad_v3_flywheel_launcher_module.urdf.xacro`. Δεν δημιουργήθηκε STL.

## CAD gate

Το `run_phase2_p57_cheek_clearance.py` έλεγξε κάθε παραλλαγή σε κάθε
ακέραια γωνία −10…+20°. Και οι 155 έλεγχοι exact-solid intersection ήταν
μηδενικοί. Η ελάχιστη απόσταση από το έδαφος είναι 10,733 mm στο −10° για
όλες τις παραλλαγές. Το πλήρες αποτέλεσμα είναι στο
`results/shared_flywheel_phase2/p257_cheek_clearance.json`.

## Headless recorded matrix

Τα criteria γράφτηκαν πριν από το campaign με SHA-256
`8234c62f388ff94ac357633cdeb5dd8d2b09bdf1da2784fc2cff981860dbfefe`.
Έτρεξαν 180/180 deterministic cases: 5 παραλλαγές × 2 tread μ ×
(3 side-by-side + 4 sequential + 11 single offsets). Όλα έτρεξαν headless
μέσω `docker/jazzy/run.sh`, όλα έχουν `replay/state.tlog` και στο τέλος το
`docker ps` ήταν κενό.

Συγκεντρωτικά αποτελέσματα:

| Παραλλαγή | side jams | side balls στο καλάθι | σειρά | capture | max +X rejected |
|---|---:|---:|---:|---:|---:|
| shield m90 | 0/6 | 0/12 | 8/8 | ±20 mm | 4,66579 m |
| shield m100 | 0/6 | 0/12 | 8/8 | ±40 mm | 4,66768 m |
| shield m110 | 0/6 | 0/12 | 8/8 | ±40 mm | 4,66090 m |
| stagger open | 0/6 | 4/12 | 8/8 | ±60 mm | 4,93918 m |
| stagger + shield | 0/6 | 4/12 | 8/8 | ±40 mm | 4,93918 m |

Στο `stagger_open`, και στα δύο μ:

- Y±35: 1/2 στο καλάθι, 1 απορριφθείσα, max +X 4,93918 m.
- Y±50: 1/2 στο καλάθι, 1 απορριφθείσα, max +X
  4,52662/4,52658 m για μ=0,6/0,9.
- Y±100: 0/2 στο καλάθι, 2 απορριφθείσες, max +X 4,73378 m.

Καμία απορριφθείσα side-by-side μπάλα δεν κατέληξε ελεύθερη και ακίνητη
δίπλα στο όχημα. Παρέμεινε στο γήπεδο ως προς Z, αλλά συνέχισε να
κινείται/παρασύρεται μπροστά, επομένως αποτυγχάνει ρητά το λειτουργικό
κριτήριο και δεν πιστώνεται ως ασφαλής απόρριψη. Το peak modeled D5065
current ήταν 20,0 A και η ροπή 0,62 N·m, χωρίς forbidden contact ή
non-finite state.

## Record/replay

Το `replay_5c3_case.sh` δέχεται τα P2-57 cases ως
`<variant>__<case> <mu>` και ανοίγει paused playback μέσω
`docker/jazzy/run.sh`. Το `5c3_side_gui.config` δεν τροποποιήθηκε.
Παραδείγματα:

```sh
engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh p257_stagger_open__side_y35 0.6
engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh p257_stagger_open__side_y50 0.9
engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh p257_stagger_open__single_yp60 0.6
```

Οι ακριβείς 180 εντολές, μία για κάθε recorded case, είναι στο
`results/shared_flywheel_phase2/gazebo_5c3_p257/replay_commands.txt`.
Το πλήρες machine-readable report είναι το διπλανό `summary.json`.

Τελική κατάσταση:
`P2_57_COMPLETE_NO_VARIANT_PASSES_REJECTION_GATE_STAGGER_OPEN_BEST_DIAGNOSTIC`.

## Τελική disposition

Η δοκιμή έγινε δεκτή ως συγκριτική, αλλά οι πέντε παραλλαγές
αρχειοθετούνται και δεν αποτελούν baseline. Τα επίπεδα covers λειτουργούν
σαν λεπίδα εκχιονιστικού· εξωτερική κλίση θα απαιτούσε deflector πέρα από
την εσωτερική ακμή των drive wheels περίπου στο Y±250 mm. Το capture
υποβαθμίστηκε από το baseline P2-40 ±100 mm σε ±20…±60 mm. Baseline
παραμένουν τα P2-40 cheeks και η πρόληψη side-by-side μεταφέρεται στον
planner, με τον P2-56 detector/reverse ως fallback.
