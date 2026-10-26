# P2-93 — πρώτο release αψίδας και μηχανισμού κλίσης

Το release περιέχει **20 manifold, one-body STL / 33 φυσικά prints**. Η
αψίδα είναι release-ready. Τα hardware interfaces του tilt είναι **fit-check
release** με διαστάσεις κατασκευαστή και δεν παίρνουν τελικό πολύωρο print
πριν συμπληρωθεί:

`[ΜΕΤΡΗΣΕΙΣ ΧΡΗΣΤΗ: FIT0186 / KFL08 / Tr8 nut / microswitch / AS5600]`

Τα FIT0186, KFL08, Tr8 nut, GT2 και microswitches έχουν παραληφθεί· οι
πραγματικές μετρήσεις εκκρεμούν. Δεν περιλαμβάνονται basket floor,
rear/side grids ή basket saddles/guides/locks: περιμένουν το mockup και τα
Δ1–Δ5. Δεν περιλαμβάνονται D5065 cradle, τελικό OAK-D cradle, firmware ή
Gazebo.

## Κλειδωμένα interfaces

- Πόδια μόνο στις υπάρχουσες οπές P2-78: X240/X300, Y±150/±160. Καμία νέα
  οπή στο τυπωμένο σασί.
- Η αψίδα βρίσκεται στις Y±228 με hollow 20×26 mm σκέλη, χωρίζεται σε μέλη
  μικρότερα από 250 mm και συνδέεται με εσωτερικά sleeves και διαμπερείς M4.
- Δύο M4 lands ανά επάνω σκέλος για S4 και δύο ανά mast για τα front halves.
- Η κεντρική ζώνη αφαίρεσης καλαθιού X−118…225, Y±172, Z≥90 μένει κενή.
- Το plate OAK-D είναι universal slotted interface, όχι cradle συσκευής.
- FIT0186 nominal Ø37, εξάδα M3 σε PCD 31 και άξονας Ø6 D· carrier με
  απόσταση 57…63 mm, ώστε ο GT2-160/20T–20T να τανύζεται γύρω από 60 mm.
- KFL08 nominal bore 8 και pitch 48 mm. Tr8 nut nominal flange Ø22 / pitch
  16 mm.
- AS5600 pod: μαγνήτης Ø6×2,5 και στόχος gap 1,5 mm, ρυθμιζόμενος με τις
  κατακόρυφες slots του pod/cover.
- Τα δύο +20° stops είναι συμμετρικά και παρακάμπτουν τον actuator:
  moving yoke → shoulder pins → stops → αψίδα → πόδια X240/X300 → σασί.
- Κάθε `tilt_stop` έχει δύο M3 interfaces για μελλοντικό, αντικαταστάσιμο
  striker tab. Το interface είναι έξω από τη φέρουσα shoulder contact και
  δεν αλλάζει τη διαδρομή κρουστικού φορτίου.

## Ανοικτές απαιτήσεις endpoint cassette — release κινούμενης μονάδας

Δεν υπάρχουν `tilt_switch_lower/upper` STL στο P2-93. Η cassette θα
σχεδιαστεί πάνω στο κινούμενο yoke χωρίς αλλαγή της αψίδας:

- ενεργοποίηση roller lever περίπου **1° πριν** από την επαφή με κάθε hard
  stop, δηλαδή στα άκρα −3° και +20°,
- αρκετή υπερδιαδρομή μετά την ηλεκτρική ενεργοποίηση, ώστε reaction/impact
  να περνά πάντα yoke → hard stop → αψίδα και ποτέ από τον microswitch,
- καλώδιο μέσα στο D5065 service loop, με strain relief και στα δύο άκρα,
- δύο switches ανά άκρο: ένα για Mega sense και ένα NC hardware cutoff των
  R_EN/L_EN.

## STL, υλικό και χρόνος P2S

Planning envelope: PETG 1,27 g/cm³. Η μάζα είναι συντηρητικό full-STL-volume
upper bound· η slicer mass αναμένεται χαμηλότερη στα infilled solids. Οι
χρόνοι είναι planning estimates και τελικό gate παραμένει το Bambu Studio.

| STL | Ποσ. | g/τεμ. | h/τεμ. 0,4/0,28 | h/τεμ. 0,6/0,30 | Ρυθμίσεις 0,4 → 0,6 |
|---|---:|---:|---:|---:|---:|
| `arch_foot_x240` | 2 | 55,5 | 3,4 | 2,5 | 5/35% → 4/35% |
| `arch_foot_x300` | 2 | 109,0 | 5,8 | 4,2 | 5/35% → 4/35% |
| `arch_leg_lower` | 2 | 39,2 | 2,6 | 2,0 | 5/35% → 4/35% |
| `arch_leg_upper` | 2 | 39,8 | 2,6 | 2,0 | 5/35% → 4/35% |
| `arch_brace_x240` | 2 | 92,0 | 5,2 | 3,9 | 5/35% → 4/35% |
| `arch_brace_x300` | 2 | 77,5 | 4,5 | 3,3 | 5/35% → 4/35% |
| `arch_joint_sleeve` | 4 | 7,2 | 1,1 | 0,9 | 5/35% → 4/35% |
| `arch_crossbar_center` | 1 | 78,8 | 4,6 | 3,4 | 5/35% → 4/35% |
| `arch_crossbar_end` | 2 | 50,3 | 3,2 | 2,4 | 5/35% → 4/35% |
| `lidar_mast` | 2 | 54,6 | 3,3 | 2,5 | 4/25% → 3/25% |
| `lidar_crossbar_half` | 2 | 57,2 | 3,5 | 2,6 | 4/25% → 3/25% |
| `sensor_joint_sleeve` | 1 | 6,0 | 1,0 | 0,8 | 4/25% → 3/25% |
| `oak_universal_plate` | 1 | 32,0 | 2,3 | 1,7 | 4/25% → 3/25% |
| `tilt_fit0186_carrier` | 1 | 64,8 | 3,8 | 2,8 | 5/35% → 4/35% |
| `tilt_kfl08_base` | 1 | 56,6 | 3,3 | 2,5 | 5/35% → 4/35% |
| `tilt_nut_carriage` | 1 | 49,6 | 3,0 | 2,2 | 5/35% → 4/35% |
| `tilt_stop` | 2 | 56,5 | 3,3 | 2,5 | 5/35% → 4/35% |
| `tilt_as5600_pod` | 1 | 18,3 | 1,6 | 1,2 | 4/25% → 3/25% |
| `tilt_as5600_cover` | 1 | 9,6 | 1,2 | 0,9 | 4/25% → 3/25% |
| `tilt_magnet_cap` | 1 | 2,0 | 0,8 | 0,6 | 4/25% → 3/25% |
| **Πρώτο κύμα** | **33 prints** | **1.609,7 g** | **100,8 h** | **75,5 h** | — |

Οι κύριοι όγκοι είναι τα τέσσερα braces (339 g), τα τέσσερα πόδια
(329 g), το arch top/masts/crossbars (~408 g) και τα tilt parts (~294 g).
Για ολόκληρο το σύνολο κρατείται envelope 3,50 kg: πρώτο κύμα 1,610 kg +
μελλοντική κινούμενη flywheel μονάδα έως 0,85 kg + καλάθι/saddles έως
0,90 kg = **3,360 kg**, δηλαδή **140 g reserve**. Αυτό δεν είναι release
των δύο τελευταίων υποσυνόλων.

Με F≈1,56 kg και 45 μπάλες≈2,60 kg, η τελική εκτίμηση γίνεται **7,520 kg**.
Η πλάκα έρματος της δοκιμής κίνησης ήταν 5,45 kg: το τελικό φορτίο είναι
περίπου **2,07 kg βαρύτερο**. Χρειάζεται δεύτερη επαλήθευση κίνησης στο
τελικό φορτίο ή ισοδύναμο έρμα.

## Σειρά εκτύπωσης και συναρμολόγησης

1. Εκτύπωσε ένα `arch_joint_sleeve` και επιβεβαίωσε ολίσθηση/διαμπερή M4.
   Μετά τύπωσε πόδια, legs, braces, crossbars και mast. Αυτά δεν εξαρτώνται
   από τροχούς, D5065, OAK-D ή mockup.
2. Συναρμολόγησε τα πόδια χαλαρά στο P2-78, πέρασε τα μέλη από ανοικτές
   clevises/sleeves, έλεγξε διαγώνιες και μετά σύσφιξε. Το καλάθι αφαιρείται
   κατακόρυφα χωρίς στοιχείο αψίδας πάνω από το removal prism.
3. Το universal OAK plate και τα LiDAR members τυπώνονται τώρα, αλλά δεν
   ορίζουν τελικό sensor cradle.
4. Για tilt τύπωσε πρώτα fit coupons/ένα interface: FIT carrier, KFL base,
   nut carriage και AS5600 pod/cover/cap. Μέτρησε hardware και ενημέρωσε
   parameters πριν το τελικό print. Τα δύο stops τυπώνονται τώρα μετά από
   dry assembly· τα striker tabs και η moving cassette δεν ανήκουν εδώ.
5. Εγκατέστησε KFL/FIT carrier, Tr8 carriage και yoke πριν κλείσουν τα F
   side/front panels. Pod/μαγνήτης μπαίνουν από +Y και ρυθμίζονται τελευταίοι
   στο gap 1,5 mm.
6. Η drive-test console τέμνει σκόπιμα το F και αφαιρείται υποχρεωτικά πριν
   από οποιαδήποτε τοποθέτηση F.

## Αντοχή και μόνιμα gates

Το analytical screen βάζει όλο το terminal design moment 8 N·m σε ένα
μέλος/stop. Μέγιστη χρησιμοποίηση 0,877: stop root 8,77 MPa έναντι 10 MPa.
Οι across-layer έλεγχοι είναι foot M4 bearing 2,90 MPa και stop M6 bearing
1,89 MPa έναντι 5 MPa. Το μικρό striker boss δεν πιστώνεται στη φέρουσα
διατομή. Proof-load, preload/slip, PETG creep και πραγματικό impact
παραμένουν φυσικά gates.

Το export αποτυγχάνει αν STL δεν είναι ένα σώμα ή αν έχει downward
horizontal roof >40 mm². Το χειρότερο STL είναι `tilt_stop`, 28,73 mm².
Το exact-solid regression ελέγχει σασί, P2-71 basket + removal prism,
P2-85 ski, rear pod, F, moving set −10…+20°, arch↔tilt, υπάρχοντες άξονες
P2-78, printed↔printed M4 και Ø10×60 mm driver paths. Η console/F σύγκρουση
είναι απαιτούμενο `EXPECTED_INTERSECTION`.

```bash
OPENSCAD_BIN=/snap/bin/openscad-nightly bash cad/arch-tilt-p293/export-p293-stls.sh
python3 cad/arch-tilt-p293/p293-strength.py
```

Τελικό αποτέλεσμα P2-93: **20 STL / 33 prints**, όλα one-body,
printability PASS, exact-solid regression PASS και analytical strength PASS.
