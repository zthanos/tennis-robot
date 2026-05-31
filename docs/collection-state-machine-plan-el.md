# Tennis Robot Collection State Machine Plan

Αυτό το έγγραφο ορίζει το behavior plan για τη συλλογή μίας μπάλας αφού το
search layer έχει βρει ή προτείνει στόχο. Συμπληρώνει το search strategy και το
mechanical collector plan: το search αποφασίζει πού υπάρχουν στόχοι, ενώ αυτό
το state machine αποφασίζει πώς ένας στόχος γίνεται stored ball.

## Objective

Να υπάρχει repeatable collection flow που:

- επιλέγει την καλύτερη διαθέσιμη μπάλα,
- προσεγγίζει με navigation όσο η μπάλα είναι μακριά,
- ευθυγραμμίζει με ακρίβεια τον collector κοντά στη μπάλα,
- εκτελεί προσπάθεια συλλογής,
- επιβεβαιώνει αν η συλλογή πέτυχε,
- κάνει retry χωρίς να κολλάει,
- επιστρέφει στο search.

## 1. Collection State Machine

```text
BALL_DETECTED
    ↓
TARGET_SELECTED
    ↓
APPROACH_TARGET
    ↓
FINE_ALIGNMENT
    ↓
COLLECTION_ATTEMPT
    ↓
VERIFY_COLLECTION
    ↓
SUCCESS
    ↓
RESUME_SEARCH
```

Failure paths πρέπει να υπάρχουν από `APPROACH_TARGET`, `FINE_ALIGNMENT`,
`COLLECTION_ATTEMPT` και `VERIFY_COLLECTION` προς retry ή search recovery.

## 2. Phase 1 - Target Selection

Δεν συλλέγουμε απαραίτητα την πρώτη μπάλα που βλέπουμε.

Παράδειγμα:

```text
Ball A = 2m
Ball B = 1m
Ball C = 4m
```

Πιθανό scoring:

```text
score = distance
      + obstacle_penalty
      + stale_detection_penalty
      + alignment_penalty
      - confidence_bonus
```

Το χαμηλότερο score είναι ο καλύτερος στόχος.

### DoD

Το robot επιλέγει την καλύτερη μπάλα με βάση απόσταση, εμπόδια, confidence και
ευκολία προσέγγισης.

## 3. Phase 2 - Approach

Όσο η μπάλα είναι μακριά:

```text
distance > 2m
```

χρησιμοποιούμε:

- LiDAR,
- odometry,
- navigation,
- path validation.

Η OAK-D κρατάει τον στόχο ενημερωμένο, αλλά δεν κάνει μόνη της το full
navigation.

### DoD

Το robot μπορεί να φτάσει κοντά στη μπάλα χωρίς να αγνοεί obstacle/safety rules.

## 4. Phase 3 - Fine Alignment

Εδώ γίνεται η περισσότερη δουλειά.

Όταν:

```text
distance < 1m
```

η OAK-D γίνεται ο κύριος sensor για την τελική ευθυγράμμιση.

### Goal

Να φέρουμε τη μπάλα στο κέντρο του collector.

```text
camera center
       |
       v

      O
```

Αν η μπάλα είναι αριστερά, το robot στρίβει αριστερά. Αν είναι δεξιά, στρίβει
δεξιά. Το LiDAR συνεχίζει να λειτουργεί ως safety layer.

### Alignment Signals

```text
ball_bearing_rad
ball_distance_m
ball_confidence
collector_center_offset_px or collector_center_offset_m
```

### DoD

Η μπάλα καταλήγει μπροστά από τον collector με αρκετή ακρίβεια για προσπάθεια
συλλογής.

## 5. Phase 4 - Collection Attempt

Ενεργοποίηση μηχανισμού συλλογής.

### Roller Concept

```text
μπάλα
 ↓
rollers
 ↓
storage
```

### Scoop Concept

```text
μπάλα
 ↓
συλλέκτης
 ↓
κάδος
```

Για το τρέχον Concept A, η πρώτη υλοποίηση είναι funnel + wide intake roller.

### DoD

Το robot εκτελεί προσπάθεια συλλογής με collector command και ελεγχόμενη χαμηλή
ταχύτητα βάσης.

## 6. Phase 5 - Collection Verification

Δεν θεωρούμε ότι συλλέξαμε μπάλα επειδή κινηθήκαμε πάνω της. Χρειαζόμαστε
επιβεβαίωση.

### Option 1 - ToF Sensor

```text
ball_present = true
```

### Option 2 - Beam Break Sensor

```text
ball_crossed_entry = true
```

### Option 3 - Counter

```text
stored_ball_count = 17
```

Για το πρώτο physical MVP, προτιμάται IR break-beam στο throat ή στο hopper
entry, όπως περιγράφεται στο `docs/concept-a-funnel-lift-wheel-plan.md`.

### DoD

Το robot γνωρίζει αν η συλλογή πέτυχε και ενημερώνει το mission/dashboard state.

## 7. Phase 6 - Retry Strategy

Αν αποτύχει:

```text
COLLECT_FAILED
```

τότε:

```text
Reverse 20cm
↓
Rescan
↓
Retry
```

μέχρι:

```text
max_retries = 3
```

Μετά από `max_retries`, ο στόχος γίνεται `missing`, `blocked` ή
`collection_failed`, ανάλογα με το τι είδαν οι sensors.

### DoD

Το robot δεν κολλάει επ' άπειρον σε μία μπάλα.

## 8. Phase 7 - Capacity Management

Αργότερα, το collection state machine πρέπει να γνωρίζει χωρητικότητα.

Παράδειγμα:

```text
Bucket Capacity = 40 balls
```

Όταν:

```text
stored_balls >= 40
```

τότε:

```text
Return To Base
```

### DoD

Το robot γνωρίζει πότε γέμισε και δεν συνεχίζει να προσπαθεί να συλλέξει χωρίς
διαθέσιμο χώρο.

## 9. Phase 8 - Collection Metrics

### Metrics

- Detected Balls,
- Targeted Balls,
- Collection Attempts,
- Successful Collections,
- Failed Collections,
- Retries,
- Average Collection Time.

### Required Fields

```text
balls_detected
targets_selected
collection_attempts
successful_collections
failed_collections
collection_retries
avg_collection_time_s
stored_ball_count
bucket_capacity
```

## 10. Collection Plan DoD

Το collection plan θεωρείται ολοκληρωμένο όταν:

1. Το robot επιλέγει στόχο.
2. Φτάνει κοντά στον στόχο.
3. Ευθυγραμμίζεται με τον collector.
4. Εκτελεί συλλογή.
5. Επιβεβαιώνει το αποτέλεσμα.
6. Κάνει retry αν χρειάζεται.
7. Επιστρέφει στο search.
8. Μετρά τις συλλεγμένες μπάλες.

## 11. Roadmap Integration

Το συνολικό flow γίνεται:

```text
Search Plan
    ↓
Target Selected
    ↓
Collect Plan
    ↓
Ball Stored
    ↓
Resume Search
```

Μετά από αυτά θα χρειαστεί ξεχωριστό Recovery Plan για:

- lost ball,
- stuck robot,
- blocked path,
- full bucket,
- jammed collector,
- stale target,
- unsafe human proximity.

Αυτό το recovery layer είναι πιθανό να κρύβει μεγάλο μέρος της πραγματικής
πολυπλοκότητας του autonomous robot, άρα πρέπει να σχεδιαστεί αφού υπάρχουν τα
πρώτα telemetry δεδομένα από search και collection.

## 12. First Webots Implementation Target

Η πρώτη υλοποίηση στο Webots πρέπει να αποδείξει:

1. Target selection από πολλαπλές visible balls.
2. Approach μέχρι κοντινή απόσταση.
3. Fine alignment με bearing/offset.
4. Simulated collector attempt.
5. Simulated collection verification.
6. Retry μέχρι `max_retries = 3`.
7. Return to search με updated `stored_ball_count`.
