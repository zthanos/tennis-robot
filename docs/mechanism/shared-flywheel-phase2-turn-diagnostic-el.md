# Φάση 2 — διαγνωστική επανάληψη στροφών 5a

> **Νεότερη αναθεώρηση P2-31…P2-34:** η μονάδα ευθυγραμμίστηκε με το CAD
> χωρίς lower panel και διορθώθηκε ο joint axis σε `−Y`. Η 18άδα αυτού του
> εγγράφου με `+Y` είναι αντικαταστημένη. Η CAD-correct επανάληψη και ο
> διαχωρισμός base-origin/ICR drift βρίσκονται στο
> `shared-flywheel-phase2-clearance-icr-review-el.md`.

## Απόφαση

Η μάζα 1 g ανά τροχό ήταν μη ρεαλιστική, αλλά **δεν ήταν η μοναδική αιτία**
του αποτυχημένου open-yaw. Με 4×0,650 kg και ακριβή διατήρηση συνολικής
μάζας/CG/αδράνειας, το open-yaw εξακολουθεί να έχει 35,6–88,3% σφάλμα yaw.
Ο εξωτερικός gyro yaw-rate loop μειώνει το σφάλμα σε 3,4–14,1%, όμως οι
επιτόπιες στροφές μετακινούν το κέντρο 0,464–0,566 m και αποτυγχάνουν το
όριο 0,10 m. Άρα το αποτέλεσμα **δεν εγκρίνεται ακόμη ως φυσική πρόβλεψη**.

Τα 9 moving-turn του gyro variant περνούν τα παγωμένα κριτήρια. Τα 18 spin
cases αποτυγχάνουν λόγω translation, όχι λόγω yaw-rate. Συνολικά: 9/54
scenario rows PASS. Δεν εκτελέστηκε 5b–5d.

## Μάζα, validation και επαφές

Οι τροχοί έγιναν 0,650 kg έκαστος. Η base link έγινε 7,404 kg, με
CG=(0,093864;0;0,132272) m και Ixx/Iyy/Izz/Ixz=
(0,386525/0,358776/0,543284/−0,052148) kg·m². Η σύνθεση βάση+τροχοί κρατά
την αρχική μάζα 10,004 kg, CG και aggregate inertia. Και τα τρία xacro
πέρασαν `check_urdf`, positive-definite inertia, collisions, limits και
ros2_control contract. Expanded URDF SHA-256: empty `7912f7d4…399ea`,
full45 `77714792…b8199`, ballast `7c60fca0…a8a5`. Image digest:
`sha256:86e0d8532b005efb99290c8e7875c204ce992fba009b6bd803af8e70587ba077`.

- v3 diagnostic: R=85 mm, width=83 mm, track=594 mm, μ1=μ2=0,5/0,7/0,9,
  χωρίς fdir, DART defaults, torque-limited PI έως 3,73 N·m.
- κύριο robot: 0,250 kg, width=80 mm, track=700 mm, μ1=μ2=1,2, slip=0.
- παλιό modular bench: 0,700 kg, width=80 mm, track=700 mm, μ=0,9,
  αλλά Gazebo DiffDrive velocity plugin. Τα 528°/4 s είναι kinematic
  regression, όχι φυσική σύγκριση ροπής: δεν είχε FIT0403 torque ceiling.

Ευθυγραμμίστηκε μόνο η ύποπτη μάζα. Δεν αντιγράφηκαν το παλιό track ή ο
velocity-imposing controller, διότι θα ακύρωναν τη v3 γεωμετρία και τον
έλεγχο ροπής. Το πλήρες comparison βρίσκεται στο
`engineering/shared-flywheel-isolated-bench/contact_model_comparison.json`.

Στατικό collision audit στο −10°: drive-wheel Z≈0, ελάχιστο non-wheel
διάκενο 3,601 mm στην κάτω πλάκα launcher· base 25 mm, basket 45 mm και
ballast 52 mm. Στα runs η βάση έπεσε το πολύ 0,028 mm, roll/pitch ήταν
αμελητέα και η άρθρωση έμεινε −9,62…−9,64° (στα −9,62° το διάκενο είναι
4,091 mm). Συντηρητικό δυναμικό κάτω όριο: **3,57 mm**. Μόνο οι τέσσερις
τροχοί άγγιξαν το επίπεδο του μοντέλου, αλλά τα 3,57 mm παραμένουν οριακά
για ανοχές/χώμα και χρειάζονται physical fit-check.

## Coulomb screen και αποτελέσματα

Η απαίτηση scrub υπολογίστηκε ως `2 μ m g df dr / L`, το διαθέσιμο traction
yaw ως `μ m g b`, και το motor ceiling ως `4 τmax b/R`. Όλα τα screens
περνούν με available/required 1,646–1,748. Είναι αναγκαίο quasi-static
κριτήριο, όχι πρόβλεψη κίνησης.

| Variant | Case | μ | Coulomb | spin .5 yaw err | spin 1.0 yaw err | moving yaw err | max τmotor | hinge / actuator |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| open | empty | .5/.7/.9 | 1.748 PASS | 50.8/57.0/76.0% | 35.6/43.0/51.9% | 40.3/50.3/64.7% | 2.99 | 2.11/.53 |
| open | full45 | .5/.7/.9 | 1.646–1.712 PASS | 53.1/68.7/88.3% | 38.8/49.2/59.8% | 41.7/59.2/74.1% | 3.21 | 2.10/.52 |
| open | ballast | .5/.7/.9 | 1.705 PASS | 52.1/61.7/82.0% | 36.7/45.2/56.7% | 41.3/57.2/72.5% | 3.07 | n/a |
| gyro | empty | .5/.7/.9 | 1.748 PASS | 3.8/7.3/10.9% | 3.9/6.2/8.8% | 3.5/4.5/9.0% | 3.54 | 2.86/.54 |
| gyro | full45 | .5/.7/.9 | 1.646–1.712 PASS | 5.3/9.5/14.1% | 4.9/7.7/11.3% | 3.6/6.6/12.9% | 3.73 | 2.85/.54 |
| gyro | ballast | .5/.7/.9 | 1.705 PASS | 4.4/8.3/12.3% | 4.4/6.7/9.8% | 3.4/5.1/11.0% | 3.66 | n/a |

Οι ακριβείς 54 γραμμές είναι στο `scenario_results.csv`. Το spin χωρίς
motor saturation δεν παραβιάζει το Coulomb screen: το DART λύνει
κυλινδρικές point/line contacts και lateral scrub constraints, ενώ το screen
υποθέτει κατανεμημένες tyre forces. Το longitudinal torque συνήθως μένει
κάτω από `μ·Nwheel·R`, αλλά το skid-steer απαιτεί lateral skid. Η χειρότερη
open-loop συμπεριφορά σε υψηλότερο μ και η μεγάλη μετάθεση του κέντρου, ενώ
το gyro κλείνει σωστά το yaw rate, δείχνουν contact-model sensitivity και
όχι απλό torque saturation. Απαιτείται φυσική δοκιμή έρματος+IMU.

## Άρθρωση, P16 και terminals

Το προηγούμενο 5,97 N·m δεν επαναλήφθηκε. Το πρώτο νέο attempt αποκάλυψε
ελεύθερη άρθρωση στο +5,84° αντί −10°· κρατήθηκε ως `invalid_free_tilt` και
αντικαταστάθηκε. Με ρητή αρχικοποίηση −10° και PD hold (όριο ±12,6 N·m), το
διορθωμένο μέγιστο joint wrench είναι **2,858 N·m** και η μέγιστη εντολή
συγκράτησης **0,541 N·m**.

Άρα το turn load δεν αυξάνει την απαίτηση actuator πάνω από 5,3 N·m.
Παραμένει 5,3 N·m για τη μετάβαση και προτείνεται design load **8,0 N·m**
για άρθρωση και κάθε terminal (`1,5×5,3`). Στο +20° η launch reaction πάει
στο προφορτισμένο terminal. Στο −10° απαιτείται κάτω seat/terminal 8,0 N·m
ή lock· αλλιώς ο P16 παραλαμβάνει στατικά και κρουστικά φορτία. Το Gazebo
PD result δεν αποδεικνύει αντοχή actuator ή τυπωμένου stop.

## D5065 update

Το ODrive Motor Guide δίνει 32 V, 65 A maximum και 420 g. Οι Parkside
15–21 V είναι εντός τάσης. Το repo είχε 85 A/3 s και 490 g· μέχρι μέτρηση
χρησιμοποιούνται συντηρητικά **65 A και 490 g**. Με Kt=0,031 N·m/A, το
maximum screen είναι 2,015 N·m αντί 2,635 N·m. Το 20 A/0,62 N·m παραμένει
χωριστό provisional operating gate.

Η rotor inertia 9,259·10⁻⁵ kg·m² είναι υπολογισμός για `Red 5065`, όχι
μέτρηση. Continuous torque: 30% maximum χωρίς αέρα, 50% με καλή ροή. Το CAD
κρατά 8 mm radial και 15 mm axial airflow keep-out ανά μοτέρ. Το 50% δεν
πιστώνεται χωρίς fan/cross-flow και temperature test.

## Artifacts και παύση

- `engineering/shared-flywheel-isolated-bench/criteria_turn_diagnostic.json`
- `engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5a_turn_diagnostic/campaign_summary.json`
- ανά run: URDF hash, validation, SDF/world, telemetry, log και summary.

Κατάσταση: `TURN_DIAGNOSTIC_COMPLETE_PAUSED_BEFORE_5B_5C_5D`.
Δεν έγινε commit, push ή PR και δεν άλλαξε robot stack, Arduino ή
ηλεκτρονικά.
