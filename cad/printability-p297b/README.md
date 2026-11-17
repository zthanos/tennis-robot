# P2-97b — έλεγχος τομών ενεργών print sets

Το μόνιμο gate είναι το `cad/check-stl-printability.sh`. Για κάθε STL
απαιτεί, με αυτή τη σειρά:

1. ακριβώς ένα connected body,
2. κανένα μη ταξινομημένο downward face πάνω από 40 mm²,
3. τομές ανά 0,20 mm από Z=0,5 με ελάχιστο πάχος 1,60 mm και κανένα
   μη ταξινομημένο νησί κάτω από 25 mm².

Τα παρακάτω PNG/SVG είναι οι αρχικές αποτυχημένες τομές, πριν από τις
διορθώσεις. Η κόκκινη γραμμή στα SVG δείχνει το witness του ελάχιστου
πάχους. Οι εξαιρέσεις του gate περιορίζονται σε ρητά Z intervals
γεωμετρικών ελεύθερων ακμών· αποτυχία έξω από αυτά σταματά την εξαγωγή.

## Κατάταξη και ενέργεια

| STL | Αρχικό εύρημα | Κατάταξη | Ενέργεια |
| --- | --- | --- | --- |
| `foot_L/R` | 0,411 mm στο Z≈23 | **Πραγματικό λεπτό φέρον τοίχωμα**: ligament ανάμεσα στις δύο παλιές X240 εσοχές ροδέλας στη βάση | Οι δύο έδρες ενώθηκαν σε ενιαία obround έδρα. Η canonical εγκατεστημένη γεωμετρία περνά αυστηρά ≥1,6 mm / ≥25 mm² πριν επιτραπεί η rotated support-free εξαγωγή. Το εύρημα **δεν είναι στις παρειές με τις οπές**. Το ήδη τυπωμένο παλιό foot δεν χρησιμοποιείται για proof-load. |
| `crown_L/R` | ≈1,18 mm | **Μυτερή άκρη λοξότμησης/οπής**: tangent στην έξοδο της εγκάρσιας κυλινδρικής οπής | Διατηρήθηκε η λειτουργική οπή. Τα ακριβή Z της μη φέρουσας ελεύθερης ακμής ταξινομήθηκαν· κάθε άλλη αποτυχία παραμένει blocking. |
| `oak_mount_x1` | 0,224 mm | **Μυτερή λοξή ελεύθερη άκρη** από την παλιά print orientation | Άκαμπτη επανατοποθέτηση plate-flat και flip 180°, χωρίς αλλαγή interfaces. Τα ακριβή Z της λοξής ακμής ταξινομήθηκαν. |
| `mast_cap_L/R` | 20,5–21 mm² islands | **Μυτερή καμπύλη ακμή**: οι πρώτες σημειακές τομές του ελλειπτικού spigot, όχι λεπτό φέρον cheek | Η installed γεωμετρία διατηρήθηκε. Τα ακριβή αρχικά/τελικά Z της ελλειπτικής ελεύθερης ακμής ταξινομήθηκαν ως μη φέροντα· διεύρυνση των cheeks απορρίφθηκε επειδή δημιουργούσε δύο νέες `mast × mast_cap` συγκρούσεις. |
| `coupon_foot_seats` | islands 2–6 mm² | **Πραγματικά απομονωμένα test features** | Προστέθηκε ενιαίο U-shaped carrier web. Η νέα έκδοση περνά αυστηρά (μικρότερο island 280,8 mm²). |
| `ramp_entry_launcher_base` | 0,076 mm και τοπικό lip 1,25 mm | **Χαραγμένο κείμενο** για το 0,076 mm, και πραγματικό λεπτό lip δίπλα στις slots | Αφαιρέθηκαν οι χαραγμένοι αριθμοί, διευρύνθηκε η βάση και μετακινήθηκαν οι slots. Η νέα βάση περνά αυστηρά (≥1,6 mm, μικρότερο island 32,4 mm²). |
| `ramp_entry_launcher_pusher` | 0,369 mm | **Μυτερή ακμή** του σφαιρικού cup | Κόπηκε το θεωρητικό tangent point σε επίπεδο πυθμένα. Η υπόλοιπη καμπύλη ελεύθερη ακμή ταξινομήθηκε σε περιορισμένο Z interval. |

## Εικόνες αρχικών αποτυχιών

- [`foot_L`](sections/foot_L.png), [`foot_R`](sections/foot_R.png)
- [`crown_L`](sections/crown_L.png), [`crown_R`](sections/crown_R.png)
- [`oak_mount_x1`](sections/oak_mount_x1.png)
- [`mast_cap_L`](sections/mast_cap_L.png), [`mast_cap_R`](sections/mast_cap_R.png)
- [`coupon_foot_seats`](sections/coupon_foot_seats.png)
- [`ramp_entry_launcher_base`](sections/ramp_entry_launcher_base.png)
- [`ramp_entry_launcher_pusher`](sections/ramp_entry_launcher_pusher.png)

Τα SVG στον ίδιο φάκελο είναι τα machine-generated πρωτότυπα με ακριβές
Z, εμβαδό νησιών και width witness.

## Collision regression

Η σύγκριση με το άμεσο HEAD των 50 pairs ήταν λανθασμένη. Το P2-97c
χρησιμοποιεί το ιστορικό baseline των 48 πριν από τα commits του P2-97 και
αφαιρεί τα δύο νέα P297 overlaps. Βλ.
[`printability-p297c/README.md`](../printability-p297c/README.md).
