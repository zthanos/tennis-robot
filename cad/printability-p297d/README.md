# τι διαφέρει από το αναμενόμενο

Το `p297_beam_x()` δεν ήταν support-free: τα κενά 0,2 mm έκρυβαν τις
μεγάλες κάτω επιφάνειες από το area gate, αλλά κάθε ενδιάμεσο τμήμα χωρίς
κολόνα ή διαγώνιο άρχιζε ως ανεξάρτητο layer island. Ο ανεξάρτητος έλεγχος
του `c4418f3a` αναπαρήγαγε τα Z65,1 / 46,3 / 6,5 / 32,7 / 58,5 / 77,1.

## P2-97 stack

- Το `p297_beam_x()` παράγει πλέον μία συνεχή δοκό, χωρίς τεχνητά κενά.
- Τα energy/power perimeter beams στηρίζονται από τα πραγματικά lattice
  posts. Τα power portal posts έγιναν 8 mm εξωτερικά και 9 mm κεντρικά,
  ώστε τα τέσσερα ανοίγματα να είναι γέφυρες ακριβώς 50 mm.
- Τα logic rails έγιναν συνεχή, μήκους 100 mm. Το χαμηλό rail πατά σε
  κολόνες 5×5 mm πάνω στο floor grid. Τα δύο ανώτερα πατούν σε ενιαία
  45° triangular webs προς τις υπάρχουσες side-wall κολόνες, με support
  stations έως 34,5 mm μεταξύ τους.
- Τα front rail links είναι πραγματικές γέφυρες δύο άκρων και τα nodes τους
  έχουν 45° κάτω μετάβαση. Δεν χρησιμοποιείται support material.

Τελικό αποτέλεσμα για `base_adapter`, `energy_spacer`, `power_box`,
`logic_box`, `collar` και τους πέντε carriers/coupons: ένα σώμα, 0 floating
islands και PASS στα downward/slice gates. Τα `base_adapter` και `collar`
παρέμειναν καθαρά όπως στον ανεξάρτητο έλεγχο.

## Μόνιμα gates

Το `check-stl-printability.sh` εκτελεί πλέον, χωρίς floating εξαιρέσεις:

1. έλεγχο ενός σώματος,
2. `independent-check/check-floating-islands.py` με απαίτηση 0 islands,
3. downward-face gate,
4. slice thickness/island gate.

Ο floating checker διατηρεί τον ίδιο ορισμό επικάλυψης με το προηγούμενο
layer, αλλά αντικαθιστά το αργό raster 41×41 με ακριβείς ελέγχους κορυφών,
ακμών και εγκλεισμού. Αναπαράγει τα αρχικά logic-box ευρήματα σε περίπου
3 s αντί για λεπτά.

Το downward gate εξαιρεί από το όριο 40 mm² μόνο face για το οποίο το STL
αποδεικνύει υλικό κάτω και στα δύο άκρα ενός άξονα και ελεύθερο span έως
50 mm. Ο έλεγχος γίνεται και στους δύο άξονες, ώστε μία μακριά επιφάνεια που
πατά σε δύο διαμήκεις rails να αναγνωρίζεται ως γέφυρα κατά τον κοντό άξονα.

## Πρόσθετα ευρήματα στα ενεργά print sets

Το νέο gate εκτελέστηκε σε 34 ενεργά output STL: P2-97 (10), common arch
(3), P293d (11), P293g FIT-CHECK (6) και ramp-entry fixtures (4).

- `mast_cap_L/R`, Z17,7: η Ø4,6 οπή έκοβε πλήρως το χαμηλότερο τόξο της
  ελλειπτικής μύτης. Η μύτη επικαλύπτει πλέον το cap κατά 2 mm και δύο
  διαμήκεις 1,6 mm rails ξεκινούν ένα layer νωρίτερα, έξω από την οπή.
- `oak_mount_x1`, Z1,1: δύο συμμετρικές λωρίδες 4 mm² των struts άρχιζαν
  έξω από τα υπάρχοντα bed gussets. Τα gussets περιλαμβάνουν πλέον τα
  πραγματικά footprints X110,5 / Y41±33,3.
- `crown_L/R`, Z14,1: το inboard lug άρχιζε ως περιοχή περίπου 1040 mm²
  πριν εμφανιστεί το crown body. Ένα μόνιμο bed-to-lug rib 4 mm, μετατοπισμένο
  έξω από το Tr8 relief και τερματισμένο στο lug end, το στηρίζει χωρίς να
  εισέρχεται στο crossbar tube.
- Στα τέσσερα ramp-entry fixture STL δεν βρέθηκε πρόσθετο floating island.

Μετά τις διορθώσεις και τα 34 ενεργά STL αναφέρουν
`FLOATING_ISLANDS PASS ...: 0 floating islands`.

## Scene regression

Το τελικό `check-current-scene.sh` αναφέρει 94 bodies ανά γωνία στις
−10/−3/0/15/20°, 0 duplicates, 0 export failures, 0 contained pairs και
**48 collision pairs**. Το crown rib σταματά πριν από το crossbar tube· τα
δύο προσωρινά `p293g_crown × p293g_crossbar_end` pairs δεν υπάρχουν.
