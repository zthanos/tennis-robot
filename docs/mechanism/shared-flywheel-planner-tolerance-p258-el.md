# P2-58 — ανοχή planner για διαγώνια ζεύγη

> **P2-61 corrected-model regression:** το planner boundary επιβεβαιώθηκε
> στα 5 m/s/0,25 ms: dx35 με dy70/100 πέρασε 8/8 και dx0 με dy70/100
> απορρίφθηκε 8/8. Ο κανόνας σειράς παραμένει.

## Απόφαση

Με τα baseline P2-40 cheeks και το αμετάβλητο P2-51/5c-3 setup, ο planner
μπορεί να χρησιμοποιήσει τον ακόλουθο συντηρητικό κανόνα:

> Παρουσίασε το ζεύγος με διαμήκη απόσταση κέντρων `dx≥70 mm`. Στο πλήρες
> δοκιμασμένο envelope `|dy|≤100 mm`, και οι δύο μπάλες πέρασαν για
> μ=0,6/0,9 και για τις δύο φορές της πρώτης μπάλας όπου δοκιμάστηκαν.
> Αν το projected `dx<70 mm`, άλλαξε τη γραμμή προσέγγισης πριν από το intake.

Το χαμηλότερο diagnostic boundary `dx35/dy70…100` πέρασε, αλλά δεν γίνεται
release rule: τα μικρότερα dy στο dx35 είναι μη φυσικά για Ø66 mm μπάλες,
οπότε δεν υπάρχει πλήρες sweep γύρω από αυτό το όριο.

## Setup και φυσικά εφικτό grid

Δεν άλλαξε geometry. Χρησιμοποιήθηκαν `five_c_design:=p244`,
`five_c_cheek_variant:=exposed_shallow`, το P2-51 R75/42° ski/basket και
η υπάρχουσα βαθμονόμηση επαφής. Τα criteria παγώθηκαν πριν από το run με
SHA-256 `1d48fd5e755b661c0323665979b4d41ba0d545123493d6413ba3a70a1a15fdfe`.

Τα dx/dy είναι συνιστώσες κέντρο-κέντρο. Κελιά με
`sqrt(dx²+dy²)<66 mm` σημειώνονται `N/A`, επειδή θα απαιτούσαν αρχική
αλληλοδιείσδυση των δύο bench balls. Έτρεξαν 63 φυσικά εφικτά cases.

Κωδικοί: `P` PASS 2/2 στο καλάθι, `J` JAM, `R` REJECT χωρίς jam signature,
`—` N/A. Οι στήλες είναι dy `0,20,35,50,70,100 mm`.

### μ=0,6 — πρώτη μπάλα στο +dy

```text
dx=  0   — — — — R J
dx= 35   — — — — P P
dx= 70   P P P P P P
dx=100   P P P P P P
dx=150   P P P P P P
```

### μ=0,6 — πρώτη μπάλα στο −dy

```text
dx=  0   — — — — R J
dx= 35   — — — — P P
dx= 70   P P P P P P
dx=100   P P P P P P
dx=150   P P P P P P
```

### μ=0,9 — πρώτη μπάλα στο +dy

```text
dx=  0   — — — — J J
dx= 35   — — — — P P
dx= 70   P P P P P P
dx=100   P P P P P P
dx=150   P P P P P P
```

Σύνολο: 57 PASS, 4 JAM και 2 REJECT. Το αποτέλεσμα είναι κατοπτρικά ίδιο
στο πλήρες μ=0,6 mirror sweep.

## P2-56 detector και fallback

Ο detector ενεργοποιήθηκε στα 4/4 JAM και δεν ενεργοποιήθηκε στα δύο
REJECT (`dx0/dy70`, μ=0,6). Ενεργοποιήθηκε όμως και σε 38/57 PASS. Τα
diagonal/close-sequence περάσματα επομένως μοιάζουν αρκετά με jam στα
D5065-only rpm/current/torque σήματα.

Η αναστροφή τροχών παραμένει fallback, αλλά όχι άμεση αυτόνομη αντίδραση
σε κάθε detector trigger. Ο planner γνωρίζει ότι έχει προγραμματίσει ζεύγος
και πρέπει να κάνει qualify/mask το trigger κατά το αναμενόμενο passage
window· reverse επιτρέπεται αν δεν ολοκληρωθεί η αναμενόμενη διπλή
διέλευση. Δεν άλλαξαν thresholds ή P2-56 control σε αυτό το βήμα.

## Record/replay

Έγιναν recorded headless επαναλήψεις για τα 6 JAM/REJECT και τα 6
edge-adjacent PASS, 12/12 συνολικά. Παράδειγμα:

```sh
engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh p258_dx0_dy100_lead_plus 0.9
engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh p258_dx35_dy100_lead_plus 0.9
```

Οι ακριβείς 12 εντολές είναι στο
`results/shared_flywheel_phase2/gazebo_5c3_p258/replay_commands.txt` και το
machine-readable report στο διπλανό `summary.json`. Το campaign άφησε
κενό `docker ps`. Δεν έγινε STL, commit, push ή PR.
