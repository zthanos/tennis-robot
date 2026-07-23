# Tennis Robot Validation and Implementation Plan

Αυτό είναι το βασικό πλάνο υλοποίησης και validation για το tennis robot.
Η κεντρική στρατηγική είναι simulation-first, με σταδιακή μεταφορά των
αποφάσεων σε μικρό φυσικό prototype και μετά σε πραγματικό γήπεδο.

```text
Webots = architecture + algorithm validation
Real prototype = sensing + mechanics validation
Real court = product-environment validation
```

## 1. Design Direction

Το robot χρησιμοποιεί καθαρό διαχωρισμό ευθυνών ανάμεσα στα sensors:

- **LiDAR** για navigation, obstacle avoidance, mapping και safety.
- **OAK-D / camera** για ball detection, person recognition και semantic perception.
- **Low front proximity / ToF sensors** για τελική επιβεβαίωση κοντά στον συλλέκτη.

Η βασική αρχιτεκτονική απόφαση είναι:

```text
LiDAR = navigation, safety, obstacle map
OAK-D = ball/person recognition
Low front sensors = final collection confirmation
```

Το LiDAR δεν χρησιμοποιείται ως κύριο sensor για αναγνώριση μπαλών. Αυτό
μειώνει την πολυπλοκότητα, κρατάει το simulation ρεαλιστικό και βοηθάει το
prototype να επικεντρωθεί στα σωστά ρίσκα.

## 2. Target Sensor Layout

### Navigation LiDAR

Το LiDAR τοποθετείται περίπου στα **25-35 cm** από το έδαφος.

Χρησιμοποιείται για:

- ανίχνευση εμποδίων,
- αποφυγή ανθρώπων και ποδιών,
- δημιουργία occupancy map ή obstacle sectors,
- επιλογή ασφαλούς διαδρομής,
- υποστήριξη search pattern.

Δεν χρησιμοποιείται ως κύριο sensor για αναγνώριση μπαλών.

### OAK-D Camera

Η OAK-D τοποθετείται περίπου στα **40-60 cm**, με ελαφριά κλίση προς τα κάτω.

Χρησιμοποιείται για:

- αναγνώριση tennis balls,
- εκτίμηση θέσης μπάλας,
- person detection,
- επιβεβαίωση στόχου πριν το robot κινηθεί προς αυτόν.

### Front Low Sensors

Χαμηλοί proximity ή ToF sensors τοποθετούνται μπροστά στον μηχανισμό συλλογής.

Χρησιμοποιούνται για:

- τελική επιβεβαίωση ότι η μπάλα βρίσκεται μπροστά στον συλλέκτη,
- αποφυγή μικρών χαμηλών εμποδίων,
- υποστήριξη του collection mechanism.

## 3. Phase 1 - Webots Simulation

### Goal

Να επιβεβαιωθεί ότι η αρχιτεκτονική, το navigation logic και το search behavior
είναι λειτουργικά πριν περάσουμε σε πραγματικό prototype.

### Scope

Το Webots simulation πρέπει να απαντήσει:

- Μπορεί το robot να ψάξει συστηματικά το γήπεδο;
- Μπορεί να αποφύγει ανθρώπους και εμπόδια;
- Μπορεί να κινηθεί προς detected ball;
- Μπορεί να οργανώσει search pattern;
- Πού πρέπει να μπουν LiDAR, OAK-D και low sensors;

### Simulation Components

Το simulation πρέπει να περιλαμβάνει:

- tennis court environment,
- robot base,
- 2D 360-degree LiDAR στα 25-35 cm,
- RGB ή RGB-D camera ως OAK-D approximation,
- tennis balls ως μικρά objects στο court,
- ανθρώπους ή moving obstacles,
- απλά static obstacles,
- collector placeholder μπροστά στο robot.

### Core Behavior State Machine

```text
Idle
↓
Require / Load Court Knowledge Model
↓
Scan Selected Side Matrix
↓
Plan Collection
↓
Detect Candidate Ball
↓
Validate Path
↓
Navigate to Ball
↓
Final Approach
↓
Collect / Simulated Collect
↓
Resume Plan Or Rescan
```

### Webots Implementation Milestones

1. Confirm current simulated sensor layout and document exact mount heights.
2. Add or validate obstacle sectors from LiDAR.
3. Add Court Knowledge Model storage and require a valid Court Knowledge Model before collection, following `docs/court-knowledge-model-specification.md`.
4. Add matrix-based side scan for the selected collection side.
5. Connect camera detections to matrix cells and planned targets.
6. Add path validation before committing to a planned target.
7. Follow the active continuous-route behavior in `docs/collection-route-rules-el.md`.
8. Add simulated collection event.
9. Add structured telemetry logs for Court Knowledge Model status, matrix scan, planning decisions and failures.
10. Add repeatable smoke/regression scenarios.

### DoD - Webots Simulation

Η φάση θεωρείται ολοκληρωμένη όταν:

1. Το robot δεν ξεκινά collection χωρίς valid Court Knowledge Model για το selected court.
2. Το Court Knowledge Model παράγει court geometry, fence geometry, obstacle map, accessibility map και traversable route information.
3. Το robot μπορεί να εκτελέσει side scan και να ενημερώσει cells με ball observations.
4. Το LiDAR δημιουργεί usable obstacle map ή τουλάχιστον reliable obstacle sectors.
5. Το robot σταματά ή αλλάζει πορεία όταν υπάρχει άνθρωπος/εμπόδιο μπροστά του.
6. Η camera μπορεί να εντοπίσει simulated tennis balls.
7. Το robot μπορεί να παράγει ordered collection plan από matrix data.
8. Το robot μπορεί να κινηθεί προς planned target χωρίς να αγνοεί εμπόδια.
9. Το robot μπορεί να κάνει final approach με μειωμένη ταχύτητα.
10. Το simulation δείχνει καθαρά αν η θέση LiDAR/OAK-D είναι λογική.
11. Υπάρχουν logs για Court Knowledge Model status, matrix scan, planned targets, obstacle events, navigation decisions και failed detections.
12. Υπάρχει documented limitation list για όσα δεν μπορεί να αποδείξει το Webots.

### Suggested Webots Metrics

- detection success rate σε known simulated balls,
- false positive count ανά scenario,
- average time to first ball,
- average time to collect/simulate collect,
- obstacle stop/replan count,
- blocked-target recovery count,
- number of successful full search loops.

## 4. Phase 2 - Small Physical Prototype

### Goal

Να επιβεβαιωθεί ότι τα πραγματικά sensors και η βασική μηχανική συμπεριφορά
δουλεύουν εκτός simulation.

### Scope

Το prototype δεν χρειάζεται να είναι πλήρες προϊόν. Χρειάζεται να αποδείξει:

- ότι η OAK-D βλέπει πραγματικά tennis balls,
- ότι το LiDAR βλέπει αξιόπιστα ανθρώπους και εμπόδια,
- ότι το robot μπορεί να κινηθεί προς στόχο,
- ότι ο μηχανισμός συλλογής έχει ρεαλιστική βάση.

### Prototype Components

- mobile base,
- 2D LiDAR,
- OAK-D camera,
- low front ToF/proximity sensors,
- basic collection mechanism,
- onboard computer ή external control laptop,
- emergency stop.

### Physical Prototype Milestones

1. Bench-test OAK-D ball detection.
2. Bench-test LiDAR obstacle/person sectors.
3. Bench-test low front sensor trigger zone.
4. Drive toward a known fixed target in a controlled indoor area.
5. Drive toward a detected ball in a controlled indoor area.
6. Stop on human/obstacle intrusion.
7. Attempt simple one-ball collection.
8. Feed real failures back into Webots scenarios.

### DoD - Physical Prototype

Η φάση θεωρείται ολοκληρωμένη όταν:

1. Η OAK-D εντοπίζει tennis balls σε διαφορετικές θέσεις και αποστάσεις.
2. Το LiDAR εντοπίζει αξιόπιστα ανθρώπινα πόδια και μεγάλα εμπόδια.
3. Το robot μπορεί να κινηθεί προς detected ball σε ελεγχόμενο χώρο.
4. Το robot σταματά όταν άνθρωπος μπει μπροστά του.
5. Ο μηχανισμός συλλογής μπορεί να συλλέξει τουλάχιστον μία μπάλα σε απλό σενάριο.
6. Έχουν καταγραφεί failure cases για κακό φωτισμό, σκιά, μπάλα κοντά σε γραμμή, μπάλα κοντά σε δίχτυ, άνθρωπο που κινείται και false positive object.
7. Τα πραγματικά failures επιστρέφουν ως input για βελτίωση του simulation.

## 5. Phase 3 - Real Court Tests

### Goal

Να ελεγχθεί αν το robot δουλεύει σε πραγματικό tennis court.

### Test Scenarios

- άδειο γήπεδο με 5-10 μπάλες,
- γήπεδο με άνθρωπο που κινείται,
- μπάλες κοντά σε γραμμές,
- μπάλες κοντά στο δίχτυ,
- μπάλες σε σκιά,
- μπάλες κοντά σε εμπόδια,
- διαφορετικές επιφάνειες γηπέδου,
- διαφορετικές συνθήκες φωτισμού.

### DoD - Real Court Validation

Η φάση θεωρείται ολοκληρωμένη όταν:

1. Το robot μπορεί να βρει και να προσεγγίσει μπάλες σε πραγματικό γήπεδο.
2. Το robot αποφεύγει ανθρώπους και εμπόδια με ασφαλή τρόπο.
3. Το robot μπορεί να συνεχίσει search μετά από failed detection ή blocked path.
4. Το robot δεν βασίζεται αποκλειστικά σε ένα sensor για κρίσιμες αποφάσεις.
5. Υπάρχει καταγραφή detection success rate, false positives, missed balls, average time to find ball, collision/near-collision events και successful collections.
6. Τα failure cases έχουν ταξινομηθεί σε sensing failures, navigation failures, mechanical collection failures και decision logic failures.
7. Το mission dashboard, όπως ορίζεται στο `docs/mission-dashboard-plan-el.md`, δείχνει progress, safety counters και mission timeline χωρίς άνοιγμα raw logs.

## 6. Phase 4 - Improve Simulation Based on Real Failures

### Goal

Το simulation να γίνει πιο χρήσιμο επειδή θα βασίζεται σε πραγματικές αποτυχίες.

### Improvements

Μετά τα real tests, το Webots simulation πρέπει να εμπλουτιστεί με:

- realistic ball positions,
- moving humans,
- blind spots,
- false positive objects,
- lighting approximation όπου είναι εφικτό,
- obstacle scenarios από πραγματικά tests,
- failed approach scenarios,
- collection edge cases.

### DoD - Simulation Feedback Loop

Η φάση θεωρείται ολοκληρωμένη όταν:

1. Κάθε σημαντικό real-world failure έχει αντίστοιχο simulation scenario.
2. Το simulation χρησιμοποιείται για regression testing.
3. Κάθε νέα αλλαγή στο navigation/search logic δοκιμάζεται πρώτα στο Webots.
4. Τα simulation tests μπορούν να δείξουν αν μια αλλαγή βελτιώνει ή χαλάει το behavior.
5. Υπάρχει repeatable test suite με βασικά scenarios.

## 7. Overall Product DoD

Το project μπορεί να θεωρηθεί validated σε concept level όταν:

1. Το Webots αποδεικνύει ότι η αρχιτεκτονική και ο αλγόριθμος έχουν συνοχή.
2. Το physical prototype αποδεικνύει ότι τα sensors δουλεύουν σε πραγματικές συνθήκες.
3. Τα real court tests αποδεικνύουν ότι το robot μπορεί να λειτουργήσει στο φυσικό περιβάλλον του.
4. Το robot μπορεί να ψάχνει το γήπεδο, να εντοπίζει μπάλες, να αποφεύγει ανθρώπους/εμπόδια, να προσεγγίζει μπάλα, να προσπαθεί να τη συλλέξει και να συνεχίζει μετά από αποτυχία.
5. Οι βασικές αβεβαιότητες είναι γνωστές και καταγεγραμμένες.
6. Υπάρχει καθαρός διαχωρισμός ανάμεσα σε simulation validation, sensing validation, mechanical validation και safety validation.

## 8. Phase Gates

Για να αποφύγουμε premature hardware complexity, κάθε phase ανοίγει μόνο όταν το
προηγούμενο έχει καθαρό evidence:

| Gate | Από | Προς | Minimum evidence |
| --- | --- | --- | --- |
| G1 | Webots architecture | Physical prototype | Search, detection, obstacle response και final approach δουλεύουν σε repeatable scenarios. |
| G2 | Physical prototype | Real court tests | Τα πραγματικά sensors βλέπουν μπάλα/άνθρωπο και το robot σταματά αξιόπιστα σε ελεγχόμενο χώρο. |
| G3 | Real court tests | Simulation feedback loop | Υπάρχει categorized failure log από πραγματικό γήπεδο. |
| G4 | Feedback loop | Iterative product work | Τα σημαντικά real failures έχουν γίνει regression scenarios. |

## 9. Immediate Next Steps

1. Κλείδωμα sensor layout στο Webots με documented heights και angles.
2. Σύνδεση του υπάρχοντος camera ball detection με target-selection telemetry.
3. Επιβεβαίωση LiDAR obstacle sectors και stop/replan behavior.
4. Δημιουργία repeatable half-court search smoke test.
5. Καταγραφή limitation list για Webots, ώστε να είναι καθαρό τι πρέπει να αποδειχθεί μόνο στο prototype.
