# P2-93g candidate — ενδιάμεσο, BLOCKED / NOT RELEASED

> **ΙΣΤΟΡΙΚΟ ΕΝΔΙΑΜΕΣΟ.** Η μετατόπιση6,4 mm εγκρίθηκε στη συνέχεια.
> Τρέχουσα παράδοση: [P2-93g-debug.md](P2-93g-debug.md). Οι παρακάτω
> blockers/παλιά candidate-02 αποτελέσματα δεν είναι το τελικό gate.

2026-11-06. Δεν αποτελεί τελική παράδοση ή έγκριση εκτύπωσης/λειτουργίας.

Το candidate-02 χρησιμοποιεί foot με παρειές8 mm, rail pad40 mm και
seatØ14,6, κονσόλα K→B, rocking cradle και nut clevis/arm. Δεν αλλάχθηκαν
τα scene_check.py, scadtree.py, parts.py ή check-current-scene.sh.
Το knee και τα templates δεν επανεξήχθησαν. Το moving yoke concept
απέκτησε τέσσερις πραγματικές through-M4 οπές για θετική στήριξη του arm·
δεν πρόκειται για αφαίρεση όγκου από QA.

## Raw gate candidate-02

Αυτούσιο output: [stdout](p293g-candidate-results/candidate-02-stdout.txt).
Machine-readable: [report](p293g-candidate-results/candidate-02-report.json).

- Γωνίες −10/−3/0/15/20°,110 σώματα ανά γωνία.
- 0 duplicates dropped,0 export failures.
- **86 global collisions +3 contained: raw gate FAIL**, όχι συνολικό PASS.
- Δεν αναφέρθηκαν τομές μεταξύ νέων tilt σωμάτων ή νέου nut arm με αψίδα.
- Παραμένουν τέσσερις νέες τομές με το παλιό upper_stop[side=-1]:
  console_inboard3359,9 mm³ σε όλες τις γωνίες,
  cradle44,7 mm³ στους15/20°, Tr8 shaft40,7 mm³ στους15/20°,
  nut_arm4,2 mm³ στους20°.
- Οι ήδη γνωστές foot×F-concept τομές παραμένουν και αυξάνονται με τις
  εγκεκριμένες παρειές8 mm:3557,1 mm³ ανά πλευρά. Δεν εξαιρέθηκαν από QA.

Η απόσυρση του παλιού upper stop εγκρίθηκε και υλοποιήθηκε μετά το
candidate-02 gate. Χωρίς ενεργά stops η διάταξη είναι μόνο FIT-CHECK,
με απαγόρευση λειτουργίας μέχρι την εγκατάσταση και επαλήθευση των νέων
stops/strikers στο P2-93h. Το παραπάνω output είναι **πριν την απόσυρση**,
όχι επιβεβαίωση της σημερινής σκηνής.

## Νέο όριο F body — ασυμβατότητα στην αμετάβλητη τοποθέτηση knee

Πηγή: `p293d_knee` extrude20, installed centreY228, και foot socket20,8.
Το knee καταλαμβάνει Y218…238. Για πραγματικό socket clearance0,4 ανά
πλευρά και εξωτερική παρειά8 mm, η εξωτερική επιφάνεια είναι
228+10+0,4+8=**246,4 mm**, πάνω από το νέο240 mm.
Με εξωτερική επιφάνεια240, η παρειά8 καταλαμβάνει Y232…240 και
τέμνει το υπάρχον knee στο Y232…238 κατά6 mm (6,4 με το clearance).
Αυτό δεν διορθώνεται με QA exception ή απλό clipping του foot:
το clipping θα άφηνε μόνο1,6 mm εξωτερική παρειά αντί8.

Μια γεωμετρική εναλλακτική που διατηρεί το knee solid20 mm και τις
γραμμές των pin axes X/Z είναι η μετατόπιση του installed knee centre
από |Y|228 σε221,6 (6,4 mm προς τα μέσα), με socket/cheeks αντίστοιχα.
Αυτό **δεν υλοποιήθηκε**: αλλάζει installed poses, crown/crossbar και
console interfaces και χρειάζεται επιλογή πριν από νέους υπολογισμούς.
Η άλλη επιλογή είναι αλλαγή της φέρουσας knee διατομής/σύνδεσης,
όχι απλή αφαίρεση διακοσμητικών προεξοχών. Δεν δηλώνεται strength PASS
για καμία από τις δύο χωρίς νέο αναλυτικό/solid έλεγχο.

Οι τελικοί LC1/LC2 με μετρημένες διατομές/μάζες CAD, εικόνες και πλήρης
έλεγχος συναρμολόγησης δεν έχουν ολοκληρωθεί. Τα ήδη εξαχθέντα STL
είναι ενδιάμεσα FIT-CHECK candidates, όχι τελικό print set. Δεν υπάρχει
φυσικό proof-load PASS. Τα συμφωνημένα αριθμητικά proof limits έχουν
καταγραφεί στο ACTIVE-PRINT-SET.md.
