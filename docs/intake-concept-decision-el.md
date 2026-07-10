# Intake concept decision

Ημερομηνία: 2026-07-10

## Απόφαση

Σταματάμε να επενδύουμε στο τρέχον single top-roller + scoop concept ως κύρια
κατεύθυνση για το intake. Η λύση αποδείχθηκε υπερβολικά ευαίσθητη στη
γεωμετρία και στην ταχύτητα επαφής. Αυτό σημαίνει ότι ακόμη κι αν βρεθεί
στενό simulation sweet spot, είναι πιθανό να είναι ευθραυστο στην πραγματική
χρήση.

Η επόμενη κύρια κατεύθυνση είναι dual-wheel / dual-roller intake concept, με
ελεγχόμενο pinch και πιο προβλέψιμη κατεύθυνση εκτόξευσης προς τη ράμπα ή το
hopper.

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

## Next Concept Direction

Το dual-wheel / dual-roller concept πρέπει να δοκιμαστεί με στόχο:

- controlled pinch ανάμεσα σε δύο ενεργές επιφάνειες,
- καθαρά ορισμένη launch direction,
- λιγότερη εξάρτηση από το scoop/lip ως αντίσταση,
- δυνατότητα ρύθμισης gap/compression χωρίς να αλλάζει όλη η ράμπα,
- repeatability σε 4/5 runs πριν θεωρηθεί candidate.

Προτεινόμενα πρώτα simulation criteria:

```text
Required:
- confirmed dual-wheel / ball contact
- positive inward velocity at release
- positive vertical velocity at release
- post-release speed >= 0.40 m/s
- no front-lip jam
- ramp or hopper entry crossing

Preferred:
- contact duration < 0.50 s
- force_p95 below selected safety threshold
- repeatable success in at least 4/5 runs
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
