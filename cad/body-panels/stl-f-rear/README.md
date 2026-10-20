# P2-84 — πίσω panels F για rear pod v2

> **ΔΕΝ ΤΥΠΩΝΕΤΑΙ ΑΚΟΜΑ.** Τα STL είναι release-ready CAD για έλεγχο και
> slicing, αλλά η φυσική εκτύπωση περιμένει την ολοκλήρωση P2-85 και το
> basket mockup/pile-retention test.

Πηγή: `../body-panels-f-rear-v2.scad`. Export:
`../export-f-rear-stls.sh`. Exact-solid suite: `../qa-p284-rear-f.sh`.

## Τι διαφέρει από το αναμενόμενο

Το παλιό concept F δεν μπορούσε να κουμπώσει στο v2: το `rear_lo` περίμενε
τα καταργημένα «πόδια/αυτιά P2-70» και στις πίσω γωνίες έμεναν μόνο 0,6 mm.
Το P2-84 κρατά το τόξο `R424,5` ως **εσωτερικό datum** και χτίζει το PETG
πάχος 2,5 mm προς τα έξω. Έτσι το pod δεν αλλάζει envelope και το πραγματικό
ελάχιστο keepout γίνεται τουλάχιστον 3 mm. Η ορατή εξωτερική επιφάνεια είναι
συνεπώς `R427`, η μόνη αισθητική απόκλιση από το concept.

Οι Parkside frames είναι πλέον ενιαίο μέρος των `rear_lo`. Οι πόρτες είναι
ξεχωριστά, ανοιγόμενα φύλλα με αφαιρούμενο διαμπερή M4 άξονα. Τα
`rear_hi` έχουν ενσωματωμένο παράθυρο με πλέγμα ±60°, δύο κάτω locating
tongues ανά μισό και captive-nut bosses κάτω από το deck. Αφαιρούνται με
τέσσερις βίδες από επάνω, χωρίς λύσιμο pod, και ανοίγουν την πίσω service
όψη Mega/perfboard.

## Κομμάτια

| STL | Ποσότητα | Bounding box εκτύπωσης | Περιγραφή |
|---|---:|---:|---|
| `rear_lo_left.stl`, `rear_lo_right.stl` | 1+1 | 146×171×44,5 mm | κάτω collar, Parkside frame, base lands, σταθερά hinge knuckles |
| `rear_hi_left.stl`, `rear_hi_right.stl` | 1+1 | 153×171×42,1 mm | αφαιρούμενο πλαίσιο/πλέγμα ±60°, locating tongues, nut bosses |
| `deck_rear_left.stl`, `deck_rear_right.stl` | 1+1 | 9×171×50 mm | deck στο Z350 και σύνδεση με control panel |
| `parkside_door_left.stl`, `parkside_door_right.stl` | 1+1 | 80×98,5×30 mm | ανοιγόμενη πόρτα, εξωτερικό pull, κινητά hinge knuckles |

Όλα είναι κάτω από 250×250 mm, manifold και ακριβώς ένα σώμα. Η export
orientation βάζει το μεγάλο `Y/Z` περίγραμμα στο bed και κρατά το καμπύλο
βάθος ως τρίτη διάσταση. PETG, 4 perimeters και brim συνιστώνται για τα
ψηλά καμπύλα frames· δεν απαιτούνται supports στη σχεδιασμένη orientation.

## Βίδες / παξιμάδια

- 2×M3×50, ροδέλα και M3 nut: εσωτερικά base lands `Y±45`.
- 2×M3×25, ροδέλα και M3 nut: εξωτερικά base lands `Y±164`.
- 2×M3×20, ροδέλα και M3 nut: deck προς τα δύο rear lands του control panel.
- 4×M3×12, ροδέλα: deck προς captive M3 nuts των δύο `rear_hi`.
- 2×M4×105 με ροδέλες/nyloc, ή M4 rod κομμένο στο πραγματικό stack-up:
  αφαιρούμενοι κατακόρυφοι άξονες πορτών.

Δεν υπάρχει self-tap ή heat-set insert. Τα M3 nuts της βάσης/control panel
φορτώνονται από κάτω πριν την τοποθέτηση. Τα τέσσερα nuts των `rear_hi`
φορτώνονται στις εξαγωνικές θήκες από την κάτω πλευρά του άνω rail.
Επιβεβαίωσε μήκη στο πραγματικό stack-up πριν από αγορά/κοπή.

## Σειρά συναρμολόγησης και service

1. Βάλε τα captive nuts στις δύο βάσεις και στα δύο control-panel lands.
2. Βίδωσε τα `rear_lo` από έξω προς τις βάσεις.
3. Πέρασε κάθε πόρτα ανάμεσα στα εναλλάξ knuckles και βάλε τον M4 άξονα.
4. Βίδωσε τα δύο deck halves στο control panel.
5. Βάλε τα `rear_hi` πρώτα στα κάτω tongues, κατέβασέ τα κάτω από το deck
   και κλείδωσε τις τέσσερις M3 από επάνω.

Για service Mega/perfboard αφαιρούνται μόνο οι τέσσερις επάνω M3 και τα δύο
`rear_hi`. Οι πόρτες ανοίγουν 110° και τα συρτάρια βγαίνουν ευθεία προς
`−X` χωρίς λύσιμο panel ή pod.

## Έλεγχοι P2-84

Empty/PASS: 3 mm F↔pod keepout, σασί/frame, drive wheels/FIT0403, πλήρες
moving sweep `−10…+20°`, fixed drawer sweep, doors ανοικτές 110° έναντι
drawer sweep, ground `Z<15`, control bodies/νέο LED, Ugreen, carrier, posts
και νέα base lands έναντι Parkside. Το χαμηλότερο F feature είναι CAD
`Z54`, άρα περίπου physical `Z52` με το σασί 2 mm χαμηλότερα.

## P2-85 — δεν σχεδιάστηκε εδώ

Το παλιό F στήριζε το `c2` στο T-clamp `X240`, το οποίο καταργήθηκε στο
P2-78. Τα πλαϊνά/εμπρός χρειάζονται νέο interface στις ενεργές δοκούς και
δεν κλειδώνουν πριν από το cardboard basket mockup και το pile-retention
test. Το P2-84 δεν σχεδιάζει ούτε υπονοεί αυτή τη στήριξη.
