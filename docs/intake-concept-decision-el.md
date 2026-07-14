# Intake concept decision

Ημερομηνία: 2026-07-10

## Απόφαση

Σταματάμε να επενδύουμε στο τρέχον single top-roller + scoop concept ως κύρια
κατεύθυνση για το intake. Η λύση αποδείχθηκε υπερβολικά ευαίσθητη στη
γεωμετρία και στην ταχύτητα επαφής. Αυτό σημαίνει ότι ακόμη κι αν βρεθεί
στενό simulation sweet spot, είναι πιθανό να είναι ευθραυστο στην πραγματική
χρήση.

Η επόμενη κύρια κατεύθυνση είναι dual-wheel / dual-roller intake concept, με
ελεγχόμενο pinch και προβλέψιμη μεταφορά της μπάλας προς τη ράμπα και το
hopper.

Σε αντίθεση με το single top-roller concept, η δημιουργία του pinch, η
προώθηση της μπάλας και η καθοδήγησή της διαχωρίζονται σε ανεξάρτητες
λειτουργίες.

## Evidence

Η απόφαση βασίζεται στο deterministic Gazebo intake bench και στο report:

```text
docs/intake-bench-sweep-report-el.md
```

Κύρια ευρήματα:

- Το `roller_z` ελέγχει αν υπάρχει roller-ball contact, αλλά το χρήσιμο εύρος
  είναι στενό.
- Η μείωση `lip_height` βοήθησε να εμφανιστεί release-like συμπεριφορά, αλλά
  δεν έλυσε τη σταθερότητα.
- Μετά το split `front_lip_contact` / `ramp_guide_contact`, φάνηκε ότι το
  front lip δεν ήταν πλέον ο blocker.
- Τα clear-zone sweeps έδειξαν ότι η ράμπα δεν είναι ο επόμενος κύριος
  περιορισμός.
- Το impulse-generation sweep έδειξε ότι:
  - χαμηλότερο drive speed έχασε τελείως roller contact,
  - υψηλότερο roller speed αύξησε contact activity αλλά έδωσε λάθος
    κατεύθυνση release,
  - το μόνο 7/7 required case είχε πολύ χαμηλή release speed και no crest.

Το failure mode δεν βελτιώθηκε ουσιαστικά μετά από διαδοχικά στοχευμένα
experiments. Αυτό ενεργοποιεί το stop / continue gate:

```text
Continue?
  No, reconsider concept.
```

## Why Not Keep Tuning

Το single top-roller setup ζητάει ταυτόχρονα:

- αρκετή αντίσταση ώστε ο roller να μεταδώσει κίνηση,
- όχι τόσο πολύ pinch ώστε να γίνει conveyor ή jam,
- σωστή release κατεύθυνση,
- αρκετή release speed,
- ανεκτικότητα σε πραγματικές αποκλίσεις μπάλας, foam, τριβής, ταχύτητας και
  επιφάνειας.

Το simulation έδειξε ότι αυτά δεν συνυπάρχουν εύκολα στο τρέχον concept. Αυτό
είναι κόκκινη σημαία για πραγματικό μηχανισμό, όπου οι ανοχές και οι φθορές θα
είναι χειρότερες από το deterministic bench.

Το βασικό πρόβλημα δεν είναι ότι δεν βρέθηκε ακόμη το σωστό configuration,
αλλά ότι δεν εμφανίστηκε αρκετά μεγάλο operating envelope ώστε η λύση να
θεωρείται ανθεκτική σε πραγματικές αποκλίσεις.

Engineering decision:

```text
A concept that only works inside a narrow simulation sweet spot is not
considered an acceptable production direction for this project.
```

## Λειτουργική φιλοσοφία

Το προηγούμενο concept είχε στόχο:

```text
bite -> impulse -> launch -> ramp
```

Το νέο concept περιγράφεται ως:

```text
capture -> transport -> guide -> hopper
```

Η μπάλα δεν εκτοξεύεται· συλλαμβάνεται, μεταφέρεται ενεργά μέσα από το wheel
throat και καθοδηγείται προς το hopper. Κάθε στάδιο αξιολογείται χωριστά.

## Next Concept Direction

Το dual-wheel / dual-roller concept πρέπει να δοκιμαστεί με στόχο:

- controlled pinch ανάμεσα σε δύο ενεργές επιφάνειες,
- προβλέψιμη μεταφορά της μπάλας μέσα από το wheel throat,
- ανεξάρτητη λειτουργία funnel, rollers και ramp,
- δυνατότητα ρύθμισης wheel gap/compression χωρίς αλλαγή της υπόλοιπης
  γεωμετρίας,
- repeatability σε 4/5 runs πριν θεωρηθεί candidate.

Προτεινόμενα πρώτα simulation criteria:

```text
Required:
- confirmed contact with both rollers
- successful capture through the wheel throat
- positive inward transport through the intake
- no stall or jam
- ramp-entry crossing
- hopper-entry or ramp-crest crossing
- repeatable success in at least 4/5 runs

Preferred:
- transport speed >= selected target
- contact duration within expected range
- force_p95 below selected threshold
- successful collection from lateral offsets
- successful collection across drive-speed variations
```

Σημείωση: το παλιό required "positive vertical velocity at release" ήταν
απαίτηση του launch concept και δεν μεταφέρεται στο transport concept. Η
ανύψωση είναι πλέον ευθύνη του guide/ramp σταδίου, όχι του release.

## Initial Dual-Wheel Architecture

Το πρώτο prototype θα ακολουθήσει την απλούστερη δυνατή αρχιτεκτονική.

Mechanism:

- δύο side rollers
- κατακόρυφοι άξονες περιστροφής
- οριζόντιο pinch
- συμμετρικό funnel για centering
- guide ramp προς το hopper

Drive train:

- δύο ίδια μοτέρ
- ένα μοτέρ ανά roller
- ίδια ονομαστική ταχύτητα
- αντίθετη φορά περιστροφής
- ίδιο torque/current limit
- κοινό emergency stop
- coordinated jam handling

Δεν χρησιμοποιείται αρχικά κοινό μοτέρ με γρανάζια.
Ο στόχος είναι πρώτα να αποδειχθεί η μηχανική αρχή και μετά να εξεταστεί
πιθανή απλοποίηση του drivetrain.

## Concept Validation Plan

Το νέο concept θα επικυρωθεί σταδιακά.

```text
Phase 1
  Dual-wheel throat only
  Goal: Validate capture and powered transport.

Phase 2
  Funnel + dual wheels
  Goal: Validate ball centering and off-axis capture.

Phase 3
  Dual wheels + ramp
  Goal: Validate transport onto the ramp.

Phase 4
  Full intake
  Goal: Validate complete collection into the hopper.
```

## Operating Envelope Validation

Η αποδοχή του νέου concept δεν θα βασιστεί σε μία επιτυχημένη δοκιμή.

Το concept πρέπει να αποδείξει ότι λειτουργεί σε εύρος πραγματικών συνθηκών.

Representative variations:

- lateral ball offset
- robot approach speed
- wheel gap tolerance
- ball compression variation
- friction variation

Success criterion:

```text
At least 4 successful collections out of 5 runs for each representative
condition.
```

## Branch Plan

Αυτό το branch κρατάει:

- το deterministic bench,
- τα instrumentation εργαλεία,
- τα release criteria,
- την τεκμηρίωση της απόφασης.

Νέο branch για την επόμενη μηχανική κατεύθυνση:

```text
feat/dual-wheel-intake-concept
```

Το νέο branch πρέπει να ξεκινήσει από το current work, όχι από παλιό main,
ώστε να κληρονομήσει το bench και τα measurement tools.
