# Intake bench sweep report

Ημερομηνία: 2026-07-10

Σκοπός αυτού του εγγράφου είναι να κρατήσει καθαρά τις μετρήσεις του νέου
deterministic Gazebo bench για το intake. Το ζητούμενο δεν είναι να
μεταφέρουμε τη μπάλα σαν conveyor ανάμεσα σε roller και scoop, αλλά να
προκαλέσουμε σύντομο bite/impulse από το top roller ώστε η μπάλα να
εκτοξευτεί προς τα μέσα και να ανέβει τη ράμπα.

## Μεθοδολογία πειραμάτων

Κάθε sweep πρέπει να κλείνει με ρητή καταγραφή του τι μάθαμε, όχι μόνο με
πίνακα αριθμών. Η βασική φόρμα είναι:

```text
Hypothesis:
  <τι πιστεύουμε ότι ελέγχει η μεταβλητή>

Status:
  Confirmed | Rejected | Inconclusive | Partially confirmed

Evidence:
  <ποια release/contact/pose metrics στηρίζουν το status>

Next:
  <ποια υπόθεση αξίζει να δοκιμαστεί μετά>
```

Παράδειγμα:

```text
Hypothesis:
  lip_x controls release.

Status:
  Rejected for the tested range.

Evidence:
  Transition sweep around lip_x=-0.013 changed contact/force, but release
  criteria stayed failed: no positive inward/vertical release velocity,
  no lip-clear, no crest crossing.
```

Παράδειγμα:

```text
Hypothesis:
  roller_z controls whether roller-ball contact happens.

Status:
  Confirmed.

Evidence:
  roller_z=-0.004 and -0.001 produced no roller contact in the first height
  sweep, while roller_z=-0.003 produced repeatable roller-ball contact.
```

### Κατηγορίες πειραμάτων

Για να μη μπερδεύονται διαφορετικά είδη δοκιμών, κάθε νέο run πρέπει να
χαρακτηρίζεται ως ένα από τα παρακάτω:

```text
Instrumentation experiment
  Στόχος: να αποδείξει ότι η μέτρηση είναι σωστή.
  Παραδείγματα: σωστό drive topic, roller contact sensor, lip contact sensor,
  pose/contact timestamp alignment.

Geometry experiment
  Στόχος: να αλλάξει σχήμα/θέση και να ελεγχθεί μηχανική συμπεριφορά.
  Παραδείγματα: lip_x, lip_height, roller_z, ramp clear zone.

Physics experiment
  Στόχος: να αλλάξει η φυσική επαφής/κίνησης.
  Παραδείγματα: friction, contact stiffness/damping, drive speed, roller speed.

Acceptance validation
  Στόχος: να ελέγξει αν ένα candidate περνάει τα required/preferred release
  criteria.
  Παραδείγματα: 1 geometry με release_criteria.json και πλήρες pass/fail.

Regression test
  Στόχος: να επιβεβαιώσει ότι μια προηγούμενη απόφαση δεν χάλασε.
  Παραδείγματα: rerun known baseline, repeatability 4/5, μετά από αλλαγές σε
  mesh/contact sensors/controller path.
```

### Knowledge gain log

Κάθε ενότητα sweep πρέπει να τελειώνει με ένα μικρό `Knowledge gain` block:

```text
Knowledge gain:
  Hypothesis:
  Status:
  Evidence:
  Decision:
  Next:
```

Αυτό είναι πιο σημαντικό από το να αποθηκεύουμε μόνο raw metrics. Τα raw
metrics λένε τι συνέβη· το knowledge gain λέει ποια απόφαση δικαιολογείται.

### Τρέχουσα αξιολόγηση μεθοδολογίας

```text
Περιοχή                     Πριν   Τώρα
Engineering methodology      6/10   9.5/10
Instrumentation              5/10   9.5/10
Scientific reasoning         6/10   9/10
Experimental design          5/10   9/10
Data-driven decisions        6/10   9.5/10
Documentation quality        7/10   9.5/10
```

### Stop / continue gate

Αν ένα πείραμα δεν δίνει ουσιαστική αλλαγή στο failure mode, δεν συνεχίζουμε
στον ίδιο άξονα απλώς επειδή μπορούμε να τρέξουμε κι άλλο grid. Κάθε sweep
πρέπει να απαντάει και στο:

```text
Continue?
  Yes, same axis | Yes, new axis | No, reconsider concept

Reason:
  <τι άλλαξε ή δεν άλλαξε στο failure mode>
```

Κριτήρια για `No, reconsider concept`:

```text
- 2-3 διαδοχικά sweeps αλλάζουν αριθμούς αλλά όχι failure mode.
- Δεν εμφανίζεται ταυτόχρονα inward + vertical + useful release speed.
- Το force/contact παραμένει σε μη αποδεκτό εύρος παρά τις λογικές αλλαγές.
- Η λύση απαιτεί υπερβολικά στενό sweet spot που δεν είναι πρακτικά repeatable.
```

Για την τρέχουσα φάση, συνεχίζουμε μόνο όσο κάθε νέο sweep απαντάει σε
συγκεκριμένη υπόθεση. Αν το impulse-generation sweep δεν δείξει καθαρή
βελτίωση, το επόμενο βήμα πρέπει να είναι concept review: single top roller vs
dual-wheel / different intake geometry.

## Τι διορθώθηκε πριν τις μετρήσεις

- Το bench πλέον παρακάμπτει perception / `collect_one` και στήνει ελεγχόμενη
  δοκιμή: robot κοντά στη `ball_02`, roller σε σταθερή ταχύτητα, σταθερό
  forward drive.
- Το drive command path διορθώθηκε. Στο ROS 2 Jazzy run ο
  `diff_drive_controller` ακούει στο `/diff_drive_controller/cmd_vel` με
  `geometry_msgs/msg/TwistStamped`, όχι στο παλιό
  `/diff_drive_controller/cmd_vel_unstamped`.
- Το bench καταγράφει:
  - roller contact samples από `/gz/roller_contact_0`
  - roller joint velocity από `/joint_states`
  - drive response από `/diff_drive_controller/odom` και wheel joint velocities
  - Gazebo ground-truth poses από `/world/tennis_court/pose/info`
  - pose-derived closest pass, ball displacement και max ball speed

## Επιβεβαίωση ότι το bench κινείται σωστά

Τελικό καθαρό baseline run:

```text
runtime/intake_sweeps/20260709_224534/lipx_m0p006_liph_0p002_rollerz_m0p003
```

Μετρήσεις:

```text
roller_ready velocity        30.000 rad/s
odom vx                      0.120 m/s
wheel velocity average       1.412 rad/s
roller-ball contact samples  1162
contact duration             4.294 s
max force                    190.29 N
max depth                    0.013 mm
pose radial gap              -1.21 mm
ball displacement            +0.639 m
max ball speed               0.352 m/s
```

Σημαντικό: το contact log δείχνει ρητά επαφή
`tennis_robot::lift_wheel_link...roller_col_collision` με `ball_02::ball::col`,
άρα δεν μετράμε ψευδή επαφή με άλλο κομμάτι του robot.

## Πρώτο sweep ύψους roller

Run:

```text
runtime/intake_sweeps/20260709_225222
```

Παράμετροι:

```text
lip_x       -0.012, -0.006, 0.000 m
lip_height   0.002 m
roller_z    -0.004, -0.001 m
```

Σύνοψη:

```text
case                                      contact_s  gap_mm   ball_dx_m  max_ball_speed_m_s
lipx_0p000_liph_0p002_rollerz_m0p001       0.000     +0.000   +0.748     0.139
lipx_0p000_liph_0p002_rollerz_m0p004       0.000     -0.034   +0.700     0.130
lipx_m0p006_liph_0p002_rollerz_m0p001      0.000     +0.008   +0.858     0.166
lipx_m0p006_liph_0p002_rollerz_m0p004      0.000     -0.061   +0.682     0.125
lipx_m0p012_liph_0p002_rollerz_m0p001      0.000     +0.000   +0.896     0.170
lipx_m0p012_liph_0p002_rollerz_m0p004      0.000     -0.034   +0.688     0.123
```

Συμπέρασμα:

- Τα `roller_z=-0.004` και `roller_z=-0.001` δεν έδωσαν roller contact.
- Η μπάλα μετακινείται και σε αρκετά cases σπάει το intake beam, αλλά δεν
  παίρνει πραγματικό roller bite.
- Το χρήσιμο ύψος φαίνεται πολύ στενό και βρίσκεται γύρω από `roller_z=-0.003`.

## Focused sweep στο ύψος που κάνει contact

Run:

```text
runtime/intake_sweeps/20260709_225726
```

Παράμετροι:

```text
lip_x       -0.012, -0.006, 0.000 m
lip_height   0.002 m
roller_z    -0.003 m
```

Μετρήσεις:

```text
case                                      contact_s  first_t  force_N  depth_mm  gap_mm  ball_dx_m  max_ball_speed_m_s
lipx_0p000_liph_0p002_rollerz_m0p003       4.219    0.853    138.1    0.010     -1.30   +0.535     0.177
lipx_m0p006_liph_0p002_rollerz_m0p003      4.534    0.521    190.3    0.013     -1.19   +0.608     0.405
lipx_m0p012_liph_0p002_rollerz_m0p003      2.941    2.106    485.8    0.034     -0.96   +0.642     0.435
```

Ερμηνεία:

- `lip_x=0.000`: πιο ήπια δύναμη, αλλά χαμηλή ταχύτητα μπάλας και μεγάλη
  διάρκεια επαφής. Μοιάζει περισσότερο με αργό σπρώξιμο.
- `lip_x=-0.006`: αρκετά καλύτερη ταχύτητα μπάλας, αλλά ακόμα πολύ μεγάλη
  διάρκεια επαφής. Είναι χρήσιμο baseline, όχι τελικό sweet spot.
- `lip_x=-0.012`: πιο σύντομη επαφή και η μεγαλύτερη ταχύτητα μπάλας, άρα
  είναι το καλύτερο μέχρι τώρα ως εκτοξευτικό candidate. Όμως η δύναμη είναι
  πολύ υψηλή, άρα μπορεί να είναι υπερβολικά σφιχτό pinch.

## Τρέχον προσωρινό candidate

```text
lip_x      = -0.012 m
lip_height =  0.002 m
roller_z   = -0.003 m
```

Δεν είναι τελικό sweet spot. Είναι το καλύτερο μέχρι τώρα για impulse, αλλά
χρειάζεται refinement για μείωση force / contact duration χωρίς να χαθεί η
ταχύτητα της μπάλας.

## Κριτήρια για το επόμενο sweep

Θέλουμε:

- πραγματικό roller-ball contact
- μικρότερη διάρκεια επαφής από το baseline `-0.006`
- max ball speed κοντά ή πάνω από `0.40 m/s`
- force σαφώς χαμηλότερο από τα `486 N` του `lip_x=-0.012`
- όχι παρατεταμένο σφήνωμα ή conveyor-like μεταφορά

## Επόμενο προτεινόμενο sweep

Λεπτότερο grid γύρω από το candidate:

```text
lip_x     = -0.010 -0.012 -0.014
roller_z  = -0.0025 -0.0030 -0.0035
lip_h     =  0.002
```

Αν το grid δείξει ότι το force μένει πολύ υψηλό, το επόμενο refinement πρέπει
να είναι είτε μικρότερη ταχύτητα εισόδου του robot είτε λίγο πιο ανοιχτό
entry clearance, όχι απλή αύξηση roller speed.

## Fine sweep γύρω από το candidate

Run:

```text
runtime/intake_sweeps/20260710_095508
```

Παράμετροι:

```text
lip_x       -0.010, -0.012, -0.014 m
lip_height   0.002 m
roller_z    -0.0025, -0.0030, -0.0035 m
```

Σημείωση για τις νέες μετρικές:

- `max_inward_base_m_s`: μέγιστη ταχύτητα της μπάλας προς το εσωτερικό του
  robot στο robot/base frame. Είναι πιο χρήσιμη από την absolute world speed,
  επειδή το robot κινείται προς τη μπάλα.
- `max_outward_base_m_s`: αντίθετη κίνηση / rebound προς τα έξω.
- `samples`: αριθμός πραγματικών roller-ball contact samples. Το
  `contact_span_s` μπορεί να είναι μεγάλο ακόμη και για σποραδικές επαφές,
  άρα πρέπει να διαβάζεται μαζί με τα samples.

Μετρήσεις:

```text
case                                        samples  contact_span_s  force_N  depth_mm  inward_m_s  outward_m_s  base_dx_m  ball_dz_m
lipx_m0p010_liph_0p002_rollerz_m0p0025       605       2.972          470.4    0.033     0.400       0.007       -0.178     0.000
lipx_m0p010_liph_0p002_rollerz_m0p0030      1178       4.053          296.6    0.021     0.329       0.008       -0.176     0.000
lipx_m0p010_liph_0p002_rollerz_m0p0035         0       0.000            n/a      n/a     0.133       0.053       +0.174     0.000
lipx_m0p012_liph_0p002_rollerz_m0p0025        21       4.028         2634.3    0.152     0.688       0.592       -0.279     0.014
lipx_m0p012_liph_0p002_rollerz_m0p0030      1662       4.966          485.8    0.034     0.653       0.000       -0.179     0.000
lipx_m0p012_liph_0p002_rollerz_m0p0035         0       0.000            n/a      n/a     0.122       0.053       +0.155     0.000
lipx_m0p014_liph_0p002_rollerz_m0p0025        14       4.106           26.2    0.171     0.679       0.581       -0.252     0.027
lipx_m0p014_liph_0p002_rollerz_m0p0030        16       4.419           24.7    0.103     0.656       0.574       -0.278     0.022
lipx_m0p014_liph_0p002_rollerz_m0p0035         0       0.000            n/a      n/a     0.125       0.053       +0.137     0.000
```

Συμπεράσματα fine sweep:

- `roller_z=-0.0035`: δεν δίνει roller contact σε αυτό το grid. Η μπάλα
  μετακινείται κυρίως από το robot/scoop, όχι από το roller.
- `lip_x=-0.010`: δίνει πιο ήπιο stable bite, αλλά η inward ταχύτητα πέφτει
  στα `0.33-0.40 m/s`. Είναι ασφαλές, όχι τόσο εκτοξευτικό.
- `lip_x=-0.012, roller_z=-0.0030`: παραμένει το καλύτερο stable bite μέχρι
  τώρα. Έχει πολλά samples, inward speed `0.653 m/s`, μηδενικό outward rebound,
  αλλά force `486 N`.
- `lip_x=-0.014`: φαίνεται εντυπωσιακό αν κοιτάξουμε μόνο speed/force, αλλά
  έχει μόλις `14-16` contact samples και πολύ μεγάλο outward rebound
  (`~0.58 m/s`). Αυτό μοιάζει με grazing/bounce, όχι αξιόπιστο grip.
- `lip_x=-0.012, roller_z=-0.0025`: πολύ υψηλή δύναμη (`2634 N`) και μεγάλο
  rebound. Απορρίπτεται.

Νέο προσωρινό συμπέρασμα:

```text
Καλύτερο stable candidate:
  lip_x      = -0.012 m
  lip_height =  0.002 m
  roller_z   = -0.0030 m

Πιο ήπιο αλλά πιο αδύναμο fallback:
  lip_x      = -0.010 m
  lip_height =  0.002 m
  roller_z   = -0.0025 m
```

Το πιθανό sweet spot για χαμηλότερη δύναμη χωρίς bounce βρίσκεται στη
μετάβαση ανάμεσα σε `lip_x=-0.012` και `lip_x=-0.014`, όχι ακριβώς στα άκρα.

## Επόμενο προτεινόμενο sweep μετά το fine sweep

Στόχος: να βρούμε αν υπάρχει σημείο γύρω από `lip_x=-0.013` που κρατάει την
inward ταχύτητα κοντά στο `0.6 m/s`, αλλά χωρίς το rebound του `-0.014` και
χωρίς τα `486 N` του `-0.012`.

```text
lip_x     = -0.0125 -0.0130 -0.0135
roller_z  = -0.0028 -0.0030 -0.0032
lip_h     =  0.002
```

## Release acceptance criteria

Από εδώ και πέρα το sweep πρέπει να κρίνεται με release-oriented κριτήρια, όχι
μόνο με `contact_s`, `force` και ball displacement.

Required:

```text
- confirmed roller-ball contact
- ball crosses ramp-entry plane
- positive inward velocity at release
- positive vertical velocity at release
- roller contact ends before timeout
- no ball-roller contact for at least 200 ms after release
- no ball-front-lip contact for at least 200 ms after release
```

Preferred:

```text
- contact duration < 0.50 s
- post-release speed >= 0.40 m/s
- force_p95 below selected safety threshold
- successful ramp crest crossing
- repeatable success in at least 4/5 runs
```

Ορισμοί που χρησιμοποιεί το bench:

- Στο robot/base frame, το εσωτερικό του robot είναι προς μικρότερο `base_x`.
  Άρα `positive inward velocity` υπολογίζεται ως `-base_vx`.
- `release` ορίζεται ως το τελευταίο roller-ball contact sample.
- `ramp-entry plane` ορίζεται αρχικά στο πραγματικό `lip_x` του case.
- `ramp crest` ορίζεται αρχικά ως `ball_base_z >= 0.138 m`, δηλαδή περίπου
  το πίσω/ψηλό άκρο της ramp με λίγη ανοχή για την ακτίνα/κέντρο της μπάλας.
- Το `no ball-roller contact for 200 ms` μετριέται πλέον από το contact log
  επειδή προστέθηκε `t_wall` στα contact samples.
- Το `no ball-front-lip contact for 200 ms` μετριέται πλέον με pose-based
  classification των `/gz/lip_contact_0` contact points σε
  `front_lip_contact_sample` και `ramp_guide_contact_sample`.
- Το ramp guide contact δεν είναι αυτόματα failure. Κρίνεται μαζί με τη
  release κατεύθυνση/ταχύτητα, επειδή μπορεί να είναι φυσιολογική επαφή με τη
  ράμπα όσο η μπάλα ανεβαίνει.
- Για να αποφεύγεται floating-point θόρυβος, το `positive inward/vertical
  velocity` απαιτεί τουλάχιστον `0.01 m/s`.

Νέο εργαλείο:

```text
scripts/sim_debug/analyze_intake_release_criteria.py
```

Το `run_native_intake_sweep.sh` γράφει πλέον ανά case:

```text
release_criteria.json
release_criteria.pretty.json
```

Σημαντικό: τα παλιά runs δεν έχουν `t_wall` στα contact samples, οπότε δεν
μπορούν να δώσουν αξιόπιστο release alignment. Τα release criteria πρέπει να
διαβαστούν από τα επόμενα runs.

Verification run μετά την προσθήκη `t_wall`:

```text
runtime/intake_sweeps/20260710_100926/lipx_m0p006_liph_0p002_rollerz_m0p003
```

Το παλιό baseline επιβεβαιώνει γιατί χρειαζόμαστε release criteria:

```text
confirmed roller-ball contact          true
ball crosses ramp-entry plane          true
roller contact ends before timeout     true
no roller contact for 200 ms           true
no front-lip contact for 200 ms        false
positive inward velocity at release    false
positive vertical velocity at release  false
ramp crest crossing                    false
post-release speed >= 0.40 m/s         false
```

Δηλαδή ένα run μπορεί να έχει πολλά contact samples (`1148`) και force
measurement, αλλά να μην είναι επιτυχημένη εκτόξευση. Αυτό είναι πλέον το
κύριο φίλτρο για τα επόμενα sweeps.

Νεότερο verification με lip contact sensor:

```text
runtime/intake_sweeps/20260710_101456/lipx_m0p006_liph_0p002_rollerz_m0p003
```

Επιβεβαίωση instrumentation:

```text
roller_contact_sample  1176
lip_contact_sample     1175
```

Το baseline αποτυγχάνει και στο lip-clear requirement, άρα το πρόβλημα δεν
είναι απλώς "δεν εκτοξεύεται", αλλά ότι η μπάλα παραμένει σε παρατεταμένη
επαφή με το scoop/lip αντί να απελευθερωθεί καθαρά.

## Transition sweep με πλήρη release criteria

Run:

```text
runtime/intake_sweeps/20260710_101741
```

Παράμετροι:

```text
lip_x       -0.0125, -0.0130, -0.0135 m
lip_height   0.002 m
roller_z    -0.0028, -0.0030, -0.0032 m
```

Σύνοψη release criteria:

```text
case                                        req   roller  entry  inward  vertical  roller_clear  lip_clear  crest  samples  lip_samples  contact_s  force_p95_N  release_speed
lipx_m0p0125_liph_0p002_rollerz_m0p0028    4/7   true    true   false   false     true          false      false  952      950          3.311      999.7        0.000
lipx_m0p0125_liph_0p002_rollerz_m0p0030    4/7   true    true   false   false     true          false      false  1177     1174         4.564      616.4        0.000
lipx_m0p0125_liph_0p002_rollerz_m0p0032    4/7   true    true   false   false     true          false      false  647      646          2.543      447.3        0.002
lipx_m0p0130_liph_0p002_rollerz_m0p0028    1/6   false   true   false   false     false         n/a        false  0        717          0.000      n/a          n/a
lipx_m0p0130_liph_0p002_rollerz_m0p0030    4/7   true    true   false   false     true          false      false  1191     1185         4.972      1056.6       0.002
lipx_m0p0130_liph_0p002_rollerz_m0p0032    4/7   true    true   false   false     true          false      false  1254     1251         4.625      545.9        0.000
lipx_m0p0135_liph_0p002_rollerz_m0p0028    4/7   true    true   false   false     true          false      false  35       1160         4.166      3997.7       0.314
lipx_m0p0135_liph_0p002_rollerz_m0p0030    4/7   true    true   false   false     true          false      false  1444     1440         4.671      1197.7       0.000
lipx_m0p0135_liph_0p002_rollerz_m0p0032    4/7   true    true   false   false     true          false      false  1468     1464         4.610      753.9        0.000
```

Συμπέρασμα:

- Κανένα case δεν πέρασε τα required.
- Το κοινό failure είναι καθαρό: `positive inward velocity at release=false`,
  `positive vertical velocity at release=false`, `lip_clear=false`,
  `crest=false`.
- Η μπάλα περνάει ramp-entry, αλλά δεν απελευθερώνεται με ταχύτητα από το
  roller/lip. Παραμένει σε παρατεταμένη lip/scoop contact μέχρι να τελειώσει
  το probe ή βγαίνει ως grazing/bounce χωρίς αξιόπιστο release.
- Η απλή μικρορύθμιση `lip_x` γύρω από `-0.013` δεν λύνει το πρόβλημα.

Νέα κατεύθυνση:

Το επόμενο πείραμα πρέπει να αλλάξει το release geometry, όχι απλώς το
`lip_x`. Πιο χρήσιμες δοκιμές:

```text
1. Μείωση lip_height: 0.002 -> 0.001 ή 0.0015
   Στόχος: αρκετή αρχική αντίσταση για bite, αλλά όχι παρατεταμένο lip contact.

2. Μικρότερη drive speed: 0.12 -> 0.06-0.08 m/s
   Στόχος: να ξεχωρίσουμε αν το robot σπρώχνει/κρατάει τη μπάλα πάνω στο lip
   αντί να την αφήνει να απελευθερωθεί.

3. Αλλαγή ramp clear zone μετά το lip
   Στόχος: πιο γρήγορο άνοιγμα της ράμπας αμέσως μετά το bite ώστε να μη
   λειτουργεί σαν δεύτερη επιφάνεια pinch.
```

Knowledge gain:

```text
Hypothesis:
  lip_x controls release.

Status:
  Rejected for the tested range.

Evidence:
  Changing lip_x around -0.013 changed contact samples and force, but all cases
  still failed positive inward/vertical release velocity, lip-clear and crest.

Decision:
  Stop spending sweeps on lip_x-only refinement.

Next:
  Change release geometry, starting with lip_height and then ramp clear zone.
```

## Lip height release-geometry sweep

Experiment type:

```text
Geometry experiment
```

Run:

```text
runtime/intake_sweeps/20260710_103300
```

Hypothesis:

```text
Lower lip_height keeps enough initial resistance for roller bite, but reduces
the prolonged lip/scoop contact that prevents clean release.
```

Παράμετροι:

```text
lip_x       -0.012, -0.013 m
lip_height   0.0010, 0.0015, 0.0020 m
roller_z    -0.0030 m
```

Σύνοψη release criteria:

```text
case                                        req   roller  entry  inward  vertical  roller_clear  lip_clear  crest  samples  lip_samples  contact_s  force_p95_N  release_speed
lipx_m0p012_liph_0p0010_rollerz_m0p0030    4/7   true    true   false   false     true          false      false  15       1243         4.620      26.5         0.336
lipx_m0p012_liph_0p0015_rollerz_m0p0030    4/7   true    true   false   false     true          false      false  1111     1107         4.899      2901.5       0.000
lipx_m0p012_liph_0p0020_rollerz_m0p0030    4/7   true    true   false   false     true          false      false  1416     1414         4.816      485.8        0.000
lipx_m0p013_liph_0p0010_rollerz_m0p0030    6/7   true    true   true    true      true          false      false  16       698          2.867      26.5         0.465
lipx_m0p013_liph_0p0015_rollerz_m0p0030    4/7   true    true   false   false     true          false      false  16       565          2.345      25.7         0.356
lipx_m0p013_liph_0p0020_rollerz_m0p0030    4/7   true    true   false   false     true          false      false  1126     1120         4.920      1038.0       0.000
```

Συμπέρασμα:

- Κανένα case δεν πέρασε όλα τα required, επειδή όλα αποτυγχάνουν στο
  `lip_clear`.
- Το `lip_x=-0.013, lip_height=0.0010, roller_z=-0.0030` είναι η πρώτη
  γεωμετρία που περνάει τα δυναμικά release criteria:
  `positive_inward_velocity_at_release=true`,
  `positive_vertical_velocity_at_release=true`, και
  `release_speed=0.465 m/s`.
- Η ίδια γεωμετρία έχει χαμηλό `force_p95=26.5 N`, αλλά μόνο `16` roller
  contact samples και `698` lip samples. Αυτό δείχνει ότι παίρνουμε launch-like
  impulse, αλλά η μπάλα εξακολουθεί να ακουμπάει στο lip/scoop μετά το release.
- Το `lip_height=0.0015` δεν είναι ενδιάμεση βελτίωση. Για `lip_x=-0.012`
  προκαλεί υπερβολικό force (`2901.5 N`), ενώ για `lip_x=-0.013` κρατάει
  χαμηλό force αλλά χάνει inward/vertical release.
- Το παλιό `lip_height=0.0020` επαναλαμβάνει το γνωστό failure mode:
  παρατεταμένο contact, μηδενική release speed, όχι εκτόξευση.

Knowledge gain:

```text
Hypothesis:
  lip_height controls release quality.

Status:
  Partially confirmed.

Evidence:
  Lowering lip_height to 0.0010 at lip_x=-0.013 produced the first 6/7 required
  result, with positive inward/vertical release velocity and release speed above
  0.40 m/s. However lip_clear stayed false and crest crossing stayed false.

Decision:
  lip_height=0.0010 with lip_x=-0.013 is the new best diagnostic candidate, but
  it is not an accepted design because the ball still remains in lip/scoop
  contact after release.

Next:
  Keep this candidate and change the ramp clear zone immediately after the lip,
  so the lip provides initial resistance but stops acting as a second pinch
  surface after the roller impulse.
```

## Ramp clear-zone slope sweep

Experiment type:

```text
Geometry experiment
```

Run:

```text
runtime/intake_sweeps/20260710_104105
```

Code change before the run:

```text
INTAKE_RAMP_CLEAR_RUN_M
INTAKE_RAMP_CLEAR_Z_M
```

Οι παράμετροι αυτοί ελέγχουν την κλίση αμέσως μετά το lip:

- `RAMP_CLEAR_RUN_M`: σε πόσα μέτρα μετά το lip φτάνει η ράμπα στο clear ύψος.
- `RAMP_CLEAR_Z_M`: σε τι ύψος φτάνει αυτό το πρώτο clear point.

Hypothesis:

```text
A steeper clear-zone slope after the lowered lip will preserve the initial bite,
but reduce post-release lip/scoop contact and improve vertical release.
```

Παράμετροι:

```text
lip_x       -0.013 m
lip_height   0.0010 m
roller_z    -0.0030 m
clear_run    0.010, 0.015, 0.030 m
clear_z      0.004, 0.006 m
```

Σύνοψη release criteria:

```text
case                                                           req   roller  entry  inward  vertical  roller_clear  lip_clear  crest  samples  lip_samples  contact_s  force_p95_N  release_speed
clearrun_0p010_clearz_0p004                                    4/7   true    true   false   false     true          false      false  15       1104         4.150      26.0         0.019
clearrun_0p010_clearz_0p006                                    1/6   false   true   false   false     false         n/a        false  0        0            0.000      n/a          n/a
clearrun_0p015_clearz_0p004                                    4/7   true    true   false   false     true          false      false  11       945          3.353      26.6         0.379
clearrun_0p015_clearz_0p006                                    6/7   true    true   true    true      true          false      false  18       1195         3.982      26.1         0.035
clearrun_0p030_clearz_0p004                                    5/7   true    true   false   false     true          true       false  9        513          1.475      26.5         0.479
clearrun_0p030_clearz_0p006                                    4/7   true    true   false   false     true          false      false  15       1144         4.872      26.4         0.280
```

Συμπέρασμα:

- Η κλίση επηρεάζει καθαρά το release, αλλά δεν έδωσε ακόμη αποδεκτό
  candidate.
- Το `clear_run=0.030, clear_z=0.004` είναι το μόνο case με `lip_clear=true`
  και `release_speed=0.479 m/s`, αλλά αποτυγχάνει `inward=false` και
  `vertical=false`. Δηλαδή μοιάζει με rebound προς λάθος κατεύθυνση, όχι με
  σωστή εκτόξευση προς τη ράμπα.
- Το `clear_run=0.015, clear_z=0.006` περνάει inward/vertical, αλλά με πολύ
  μικρή `release_speed=0.035 m/s` και `lip_clear=false`. Άρα δεν είναι
  εκτόξευση.
- Το πολύ απότομο `clear_run=0.010, clear_z=0.006` έχασε εντελώς roller contact.
- Όλα τα force values που είχαν roller contact έμειναν χαμηλά (`~26 N`), άρα
  το πρόβλημα εδώ δεν είναι υπερβολικό pinch force αλλά κατεύθυνση/clearance
  του release.

Σημαντική παρατήρηση instrumentation:

Το τωρινό `/gz/lip_contact_0` μετράει το collision του `intake_channel_col`,
δηλαδή πρακτικά lip + ramp/scoop μαζί. Για μελλοντική αποδοχή πρέπει να
ξεχωρίσουμε:

```text
front lip contact  -> ανεπιθύμητο μετά το release
ramp guide contact -> μπορεί να είναι φυσιολογικό/επιθυμητό αν η μπάλα ανεβαίνει
```

Άρα το `lip_clear=false` είναι χρήσιμο warning, αλλά δεν αρκεί μόνο του για να
καταδικάσει ένα ramp-guided launch μέχρι να χωρίσουμε τα contact sensors.

Knowledge gain:

```text
Hypothesis:
  clear-zone slope controls clean release.

Status:
  Partially confirmed, but no accepted candidate.

Evidence:
  Changing clear_run/clear_z changed lip_clear, release speed and release
  direction. However the high-speed/lip-clear case rebounded outward/downward,
  while the positive inward/vertical case had too little release speed.

Decision:
  Keep ramp clear-zone slope as a real tuning axis, but do not accept any slope
  candidate from this sweep.

Next:
  Split front-lip contact from ramp-guide contact, then retest a small grid
  around clear_run=0.020-0.030 and clear_z=0.004 with repeatability checks.
```

## Front-lip vs ramp-guide contact split

Experiment type:

```text
Instrumentation experiment
```

Για να κάνουμε πιο στοχευμένες κινήσεις, το aggregate `lip_contact_sample` δεν
αρκεί. Μετρούσε όλο το `intake_channel_col`, δηλαδή front lip και ramp/scoop
μαζί. Αυτό έκανε ένα ramp-guided launch να φαίνεται σαν failure, παρότι η
επαφή με τη ράμπα μπορεί να είναι επιθυμητή.

Νέα λογική:

```text
front_lip_contact_sample
  Επαφή κοντά στο μπροστινό lip. Ανεπιθύμητη μετά το release.

ramp_guide_contact_sample
  Επαφή πίσω από το front lip, πάνω στη ράμπα. Μπορεί να είναι αποδεκτή αν η
  μπάλα κινείται inward/upward.
```

Implementation:

- Το `sim_physics_probe.py` γράφει split samples όταν έχει διαθέσιμα contact
  points.
- Το `analyze_intake_release_criteria.py` κάνει την κύρια ταξινόμηση από
  `points_world` + Gazebo pose log, ώστε να χρησιμοποιεί το true robot pose.
- Το required criterion άλλαξε από:

```text
no_lip_contact_for_release_window
```

σε:

```text
no_front_lip_contact_for_release_window
```

Verification run:

```text
runtime/intake_sweeps/20260710_105328/lipx_m0p013_liph_0p0010_rollerz_m0p0030_clearrun_0p030_clearz_0p004
```

Μετρήσεις μετά το split:

```text
roller_contact_samples       11
lip_contact_samples          651   # legacy aggregate
front_lip_contact_samples    0
ramp_guide_contact_samples   651
legacy_lip_clear             false
front_lip_clear              true
release_inward_velocity      0.023 m/s
release_vertical_velocity    0.001 m/s
release_speed                0.023 m/s
```

Συμπέρασμα:

- Το παλιό `lip_clear=false` ήταν υπερβολικά αυστηρό, επειδή τιμωρούσε και την
  κανονική ramp guide επαφή.
- Το συγκεκριμένο verification run δεν είναι επιτυχημένη εκτόξευση, γιατί η
  release speed είναι πολύ χαμηλή και το vertical release είναι κάτω από το
  threshold.
- Όμως πλέον ξέρουμε ότι το failure δεν είναι post-release front-lip pinch.
  Είναι ανεπαρκές launch impulse / κατεύθυνση μετά το roller.

Knowledge gain:

```text
Hypothesis:
  Aggregate lip contact was hiding whether the ball was stuck on the front lip
  or simply guided by the ramp.

Status:
  Confirmed.

Evidence:
  The verification run had legacy lip_contact_samples=651 and legacy
  lip_clear=false, but after pose-based classification it had
  front_lip_contact_samples=0 and ramp_guide_contact_samples=651.

Decision:
  Use no_front_lip_contact_for_release_window as the required clear criterion.
  Treat ramp_guide_contact as contextual: acceptable only with inward/upward
  release and enough speed.

Next:
  Retest the small clear-zone grid using the new front-lip criterion and judge
  ramp contact together with release direction/speed.
```

## Targeted clear-zone sweep με front-lip criterion

Experiment type:

```text
Geometry experiment
```

Run:

```text
runtime/intake_sweeps/20260710_105753
```

Hypothesis:

```text
With front-lip contact separated from ramp-guide contact, a narrower clear-zone
grid around clear_run=0.020-0.030 can reveal whether ramp geometry is still the
main blocker.
```

Παράμετροι:

```text
lip_x       -0.013 m
lip_height   0.0010 m
roller_z    -0.0030 m
clear_run    0.020, 0.025, 0.030 m
clear_z      0.004 m
front_lip_zone 0.008 m
```

Σύνοψη release criteria:

```text
case                       req   roller  entry  inward  vertical  roller_clear  front_lip_clear  crest  roller_samples  front_lip  ramp_guide  contact_s  force_p95_N  release_speed  legacy_lip_clear
clearrun_0p020_clearz_0p004 5/7  true    true   false   false     true          true             false  12              0          1165        3.900      26.6         0.155          false
clearrun_0p025_clearz_0p004 5/7  true    true   false   false     true          true             false  12              0          834         4.315      26.4         0.317          false
clearrun_0p030_clearz_0p004 6/7  true    true   true    false     true          true             false  14              0          862         3.200      26.3         0.037          false
```

Συμπέρασμα:

- Και τα τρία cases έχουν `front_lip_clear=true` και `front_lip=0`, άρα το
  μπροστινό lip δεν είναι πλέον ο κύριος blocker.
- Το παλιό `legacy_lip_clear=false` συνεχίζει να εμφανίζεται επειδή υπάρχει
  ramp-guide contact. Αυτό πλέον δεν πρέπει να διαβάζεται ως αποτυχία από μόνο
  του.
- Το `clear_run=0.025` δίνει την καλύτερη ταχύτητα από αυτό το μικρό grid
  (`0.317 m/s`), αλλά η κατεύθυνση είναι outward/downward
  (`inward=false`, `vertical=false`).
- Το `clear_run=0.030` δίνει inward release, αλλά με πολύ χαμηλή ταχύτητα
  (`0.037 m/s`) και χωρίς vertical pass.
- Όλα τα cases έχουν χαμηλό force (`~26 N`) και λίγα roller samples
  (`12-14`). Άρα δεν έχουμε πια πρόβλημα υπερβολικού pinch· έχουμε πρόβλημα
  ανεπαρκούς ή λάθος κατευθυνόμενου impulse.

Knowledge gain:

```text
Hypothesis:
  Clear-zone geometry is still the main blocker after separating front-lip and
  ramp-guide contact.

Status:
  Rejected as the primary blocker for this local range.

Evidence:
  All three clear_run cases passed front_lip_clear and had only ramp-guide
  contact, but none achieved both positive vertical release and useful release
  speed. The best speed case moved outward/downward, while the inward case was
  too slow.

Decision:
  Stop tuning clear_run in this narrow range as the next primary axis.

Next:
  Tune impulse generation: either increase effective roller bite/energy or
  reduce approach/drive interaction so the roller launch is not turned into a
  slow ramp push.
```

## Impulse-generation sweep

Experiment type:

```text
Physics experiment
```

Run:

```text
runtime/intake_sweeps/20260710_110434
```

Code change before the run:

```text
INTAKE_SWEEP_DRIVE_SPEEDS
INTAKE_SWEEP_ROLLER_SPEEDS
```

Το `run_native_intake_sweep.sh` μπορεί πλέον να κάνει grid και σε drive/roller
speed, ώστε να δοκιμάζουμε impulse generation χωρίς χειροκίνητα runs.

Hypothesis:

```text
The current geometry is front-lip clear, but the roller impulse is too weak or
is being corrupted by robot drive interaction. Lower drive speed or higher
roller speed should improve release direction/speed if the single top roller is
still viable.
```

Παράμετροι:

```text
lip_x       -0.013 m
lip_height   0.0010 m
roller_z    -0.0030 m
clear_run    0.025 m
clear_z      0.004 m
drive_speed  0.06, 0.12 m/s
roller_speed 30.0, 45.0 rad/s
```

Σύνοψη release criteria:

```text
case                         req   roller  entry  inward  vertical  roller_clear  front_lip_clear  crest  roller_samples  front_lip  ramp_guide  contact_s  force_p95_N  release_speed
drive_0p06_rollerspeed_30p0   0/6   false   false  false   false     false         n/a              false  0               0          0           0.000      n/a          n/a
drive_0p06_rollerspeed_45p0   0/6   false   false  false   false     false         n/a              false  0               0          0           0.000      n/a          n/a
drive_0p12_rollerspeed_30p0   7/7   true    true   true    true      true          true             false  10              0          867         2.976      26.3         0.133
drive_0p12_rollerspeed_45p0   5/7   true    true   false   false     true          true             false  23              0          610         3.663      35.2         0.249
```

Συμπέρασμα:

- Το χαμηλότερο drive speed (`0.06 m/s`) έχασε τελείως roller contact. Άρα η
  κίνηση του robot δεν είναι απλώς "too much push"· φαίνεται να είναι μέρος
  του μηχανισμού που φέρνει τη μπάλα στο bite.
- Το baseline drive με roller `30 rad/s` πέρασε τα required 7/7 με το νέο
  front-lip criterion, αλλά η release speed έμεινε πολύ χαμηλή
  (`0.133 m/s`) και δεν έφτασε crest.
- Η αύξηση roller speed σε `45 rad/s` αύξησε τα roller samples (`23`) και το
  force (`35.2 N`), αλλά έκανε το release outward/downward
  (`inward=false`, `vertical=false`). Δηλαδή περισσότερη ταχύτητα roller δεν
  μεταφράστηκε σε καλύτερη εκτόξευση.
- Η γεωμετρία δεν κρατάει τη μπάλα στο front lip, αλλά η single top-roller
  επαφή δεν δίνει repeatable, σωστά κατευθυνόμενο launch impulse.

Knowledge gain:

```text
Hypothesis:
  Drive speed / roller speed can recover useful launch impulse in the current
  single top-roller geometry.

Status:
  Rejected for this local test.

Evidence:
  Lower drive removed roller contact entirely. Higher roller speed increased
  contact activity but pushed release outward/downward. The only 7/7 required
  case had release speed far below the preferred 0.40 m/s and no crest crossing.

Decision:
  Do not continue same-axis tuning of drive/roller speed as the primary path.

Continue?
  No, reconsider concept.

Reason:
  We now have front-lip clear and ramp-guide separation, but still cannot get
  useful, correctly directed release speed from the single top-roller setup.

Next:
  Run a concept review: compare whether to keep single top roller with a
  redesigned bite geometry, or pivot to a two-wheel / dual-roller intake that
  creates a controlled pinch and launch direction.
```

## Σχετικά εργαλεία

- `scripts/sim_debug/run_native_intake_sweep.sh`
- `scripts/sim_debug/summarize_contact_physics.py`
- `scripts/sim_debug/analyze_intake_bench_poses.py`
- `scripts/sim_debug/analyze_intake_release_criteria.py`
- `scripts/sim_debug/log_gz_poses.py`
- `ros2_ws/src/tennis_robot/tennis_robot/sim_physics_probe.py`
