# Φάση 2 — isolated xacro validation και Gazebo 5a

> **ΙΣΤΟΡΙΚΟ — δεν περιγράφει το τρέχον design.** Διατηρείται ως evidence
> της απόφασης/δοκιμής της εποχής του. Δες
> [το τρέχον design](../current-design-el.md).

> **Αναθεώρηση στροφών:** τα straight/braking συμπεράσματα παραμένουν. Τα
> turn results επαναλήφθηκαν με 0,650 kg τροχούς και gyro variant· δείτε
> `shared-flywheel-phase2-turn-diagnostic-el.md`. Το gyro κλείνει το yaw
> rate, αλλά το DART spin μετατοπίζει το κέντρο 0,464–0,566 m. Δεν γίνεται
> ακόμη φυσικό συμπέρασμα στροφής.

## Απόφαση

Η xacro validation πέρασε και στις τρεις περιπτώσεις.  Η καμπάνια 5a
ολοκληρώθηκε για `empty`, `full45`, `ballast` και μ=0,5/0,7/0,9.  Οι ευθείες
0,3/0,6 m/s και το φρενάρισμα πέρασαν σε όλους τους συνδυασμούς, αλλά όλες οι
επιτόπιες και εν κινήσει στροφές απέτυχαν το παγωμένο κριτήριο yaw tracking.
Άρα το `ballast` επιτρέπεται ως πρώτο, χαμηλής ταχύτητας φυσικό drive test για
ευθεία/ήπια καμπύλη, όχι ως τεκμήριο ότι η ανοικτού βρόχου επιτόπια στροφή
είναι έτοιμη.

Παύση εδώ.  Δεν εκτελέστηκαν 5b–5d.

## Περιβάλλον και xacro

- image: `tennis-robot-jazzy@sha256:86e0d8532b005efb99290c8e7875c204ce992fba009b6bd803af8e70587ba077`,
- μόνο `docker/jazzy/run.sh exec`, Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic,
  headless, `ROS_DOMAIN_ID=88`, DART, χρονικό όριο σε κάθε run,
- το `docker ps` ήταν κενό μετά από κάθε validation/build/simulation run,
- δεν χρησιμοποιήθηκε το host `/opt/ros/jazzy`.

Τελικό `check_urdf` και semantic validation:

| Περίπτωση | Μάζα | CG στο zero joint pose [m] | Izz στο CG | SHA-256 URDF | Αποτέλεσμα |
|---|---:|---|---:|---|:---:|
| empty | 18,1155 kg | (0,1655, −0,0010, 0,1092) | 1,5494 kg·m² | `985f6b4f…351e382a9` | PASS |
| full45 | 20,6805 kg | (0,1580, −0,0009, 0,1124) | 1,5996 kg·m² | `011cdc46…0406d9c` | PASS |
| ballast | 18,8579 kg | (0,1565, −0,0010, 0,1164) | 1,3660 kg·m² | `acc9d89b…218f517b2` | PASS |

Ελέγχθηκαν θετική μάζα/θετικά ορισμένες αδράνειες, contract mass/geometry,
collision links, −10°/+20° limits και ακριβής αντιστοίχιση `ros2_control`
interfaces.  Η αριθμητική μάζα κάθε drive wheel μένει 1 g για να μη
διπλομετρηθεί η μάζα που περιέχει το base datum, αλλά κρατήθηκε φυσική
περιστροφική αδράνεια 0,0025 kg·m² περί Υ.

## Παγωμένα κριτήρια και αποτελέσματα

Τα αριθμητικά κριτήρια βρίσκονται στο
`engineering/shared-flywheel-isolated-bench/criteria_5a.json` και γράφτηκαν
πριν από το πρώτο Gazebo run.  Η σειρά κελιών παρακάτω είναι PASS/FAIL για
κάθε περίπτωση × μ × σενάριο.  Το πλήρες αριθμητικό 54-row table, μαζί με
torque, slip, stability και wheel-rise metrics, είναι το
`engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5a/scenario_results.csv`.

| Περίπτωση | μ | ευθεία 0,3 | ευθεία 0,6 | spin 0,5 | spin 1,0 | στροφή 0,4+0,6 | brake 0,6 |
|---|---:|:---:|:---:|:---:|:---:|:---:|:---:|
| empty | 0,5 | PASS | PASS | FAIL | FAIL | FAIL | PASS |
| empty | 0,7 | PASS | PASS | FAIL | FAIL | FAIL | PASS |
| empty | 0,9 | PASS | PASS | FAIL | FAIL | FAIL | PASS |
| full45 | 0,5 | PASS | PASS | FAIL | FAIL | FAIL | PASS |
| full45 | 0,7 | PASS | PASS | FAIL | FAIL | FAIL | PASS |
| full45 | 0,9 | PASS | PASS | FAIL | FAIL | FAIL | PASS |
| ballast | 0,5 | PASS | PASS | FAIL | FAIL | FAIL | PASS |
| ballast | 0,7 | PASS | PASS | FAIL | FAIL | FAIL | PASS |
| ballast | 0,9 | PASS | PASS | FAIL | FAIL | FAIL | PASS |

Σύνολο: 27/54 scenario rows PASS, 27/54 FAIL, και 0/9 πλήρεις συνδυασμοί
περνούν όλα τα σενάρια.

### Τι απέτυχε

- Ευθείες: steady speed error 0,17–2,15%, μη μετρήσιμη πλευρική απόκλιση στο
  ιδανικά συμμετρικό μοντέλο και ουσιαστικά μηδενικό longitudinal slip.
- Φρενάρισμα: 0,10–0,14 s και 26–38 mm.  Η εντολή φτάνει στιγμιαία το όριο
  3,73 N·m, αλλά το saturation fraction μένει κάτω από το παγωμένο 10%.
- Spin 0,5 rad/s: yaw error 52,1–99,9% και drift 1–279 mm.
- Spin 1,0 rad/s: yaw error 36,7–71,2% και drift 352–517 mm.
- Moving turn: linear-speed error 1,8–10,7% περνά, αλλά yaw error
  40,2–88,4% αποτυγχάνει.  Mean/peak wheel slip στις στροφές έφτασε
  0,32/0,93.  Δεν ενεργοποιήθηκε το ειδικό rear-wheel free-spin criterion·
  η αιτία είναι skid-steer scrub/ανεπαρκής ανοικτού βρόχου yaw απόκριση, όχι
  ανεξέλεγκτη υπερταχύτητα πίσω τροχού.
- Το μεγαλύτερο μ δεν βοηθά αναγκαστικά: αυξάνει και την πλευρική αντίσταση
  scrub.  Στο `full45, μ=0,9, spin 0,5` το όχημα σχεδόν δεν στράφηκε.

Το υπάρχον gyro πρέπει να χρησιμοποιηθεί σε εξωτερικό κλειστό βρόχο
yaw-rate πάνω από τους wheel-speed loops.  Αυτό μπορεί να διορθώσει το
tracking και τις ασυμμετρίες, αλλά δεν καταργεί το scrub, την απαίτηση ροπής ή
τα φορτία.  Η παρούσα καμπάνια μένει σκόπιμα η παγωμένη open-yaw baseline·
δεν αλλάχθηκε Arduino/controller code στη μέση της σάρωσης.

## Φορτία προς FEA

Τα μέγιστα της καμπάνιας είναι:

| Μέγεθος | Μέγιστο | Θέση |
|---|---:|---|
| FIT0403 command | 3,73 N·m | transient braking, όλα τα φορτία |
| wheel-joint force norm | 94,66 N | full45, μ=0,9, braking, front-right |
| wheel-joint constraint moment norm | 5,90 N·m | full45, μ=0,9, moving turn, front-left |
| launcher tilt-joint force norm | 59,24 N | empty, μ=0,9, straight 0,6 |
| launcher tilt-joint moment norm | 5,97 N·m | empty, μ=0,9, spin 0,5 |
| p99 inertial force | 39,74 N | full45, μ=0,5 |
| p99 inertial yaw moment | 1,17 N·m | empty, μ=0,9 |
| raw one-step inertial force / yaw moment | 148,91 N / 20,00 N·m | diagnostic peaks |

Το wheel-joint constraint moment δεν είναι motor torque και δεν συγκρίνεται
με το stall 3,73 N·m.  Για FEA χρησιμοποιούνται οι έξι διανυσματικές
συνιστώσες στα per-run CSV, όχι μόνο τα norms.  Τα p99 inertial loads είναι η
προτεινόμενη nominal δυναμική περίπτωση και τα raw peaks sensitivity/impact
case μέχρι να υπάρξει φιλτράρισμα αντίστοιχο του πραγματικού sample rate.

Δεν παρατηρήθηκε γεωμετρικό wheel lift: η μέγιστη άνοδος κέντρου τροχού από
το settled datum ήταν 0,1 μm.  Το joint `Fz` κρατιέται μόνο ως
solver-sensitive diagnostic, επειδή εμφανίζει στιγμιαία μηδενικά ακόμη και
σε ευθεία.  Το rigid Gazebo μοντέλο δεν αποδεικνύει ακαμψία PETG ούτε
συμπεριφορά σε ανώμαλο χώμα.

## Φυσική δοκιμή με έρμα

1. Πρώτα ευθεία 0,3 m/s, έλεγχος ρευμάτων/θερμοκρασίας/απόκλισης και μετά
   0,6 m/s.
2. Φρενάρισμα με ramp ή coast όταν χρησιμοποιείται εργαστηριακό τροφοδοτικό
   που δεν απορροφά αναγέννηση· ενεργό hard brake μόνο με ασφαλή διαδρομή
   ενέργειας/αντίσταση πέδησης.
3. Ήπιες καμπύλες πριν από επιτόπια στροφή.  Για spin, ενεργός gyro yaw-rate
   loop, torque/current limits και καταγραφή gyro + encoders + ρεύματος.
4. Διακοπή αν πλησιάζει 3,73 N·m/το αντίστοιχο μετρημένο current limit, αν
   σηκώνεται τροχός ή αν εμφανίζεται παραμόρφωση/τζόγος στις ενώσεις.

Η προσομοίωση στηρίζει την πρώτη δοκιμή ευθείας/φρεναρίσματος με το τελικό
front terminal και την πλάκα έρματος.  Δεν εγκρίνει ακόμη in-place turn
χωρίς gyro feedback validation και φυσικό έλεγχο πρόσφυσης.

## Περιορισμοί και αρχεία

- Ισότροπο Coulomb μ, όχι βαθμονομημένο πέλμα σε χωμάτινο γήπεδο.
- Ιδανικά άκαμπτο σασί/ενώσεις και συμμετρική επιφάνεια.
- Torque-limited isolated speed servo, όχι production electronics.
- Τα balls είναι fixed payload mass στο 5a, όχι διακριτά κινούμενα σώματα.
- Δεν έγινε strength proof, ball collection, transition ή launch.

Αναπαραγωγή: `run_5a_case.sh`, `analyze_5a_run.py`,
`aggregate_5a_results.py`.  Raw telemetry, world/URDF, logs και summaries
βρίσκονται κάτω από
`engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5a/`.
