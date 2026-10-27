# Rear pod — ενεργό v2 / P2-90d

Η μοναδική ενεργή print release είναι:

- πηγή `rear-pod-v2.scad` + `params-v2.scad`,
- STL στο `stl-v2/`,
- πλήρεις οδηγίες/βίδες στο `stl-v2/README.md`,
- συνολικό manifest στο `../ACTIVE-PRINT-SET.md`.

Το P2-83/P2-86b αντικατέστησε το σπασμένο corner tie και ευθυγράμμισε τα
stays στις πραγματικές υπάρχουσες οπές. Οι P2-86b βάσεις είναι τυπωμένες και
κλειδωμένες. Το P2-81 παραμένει ενεργό για το ribbed control panel και τους
επίπεδους Ø23 ώμους posts. Τα `stl-v2/mpu6050_mount.stl` και
`chassis_spacers.stl` δεν είναι ενεργά και έχουν μεταφερθεί στο `archive/`.

Το P2-84 πρόσθεσε μόνο interface features εντός του ίδιου envelope: δύο
rear-facing M3 captive-nut lands ανά μισή βάση, δύο αντίστοιχα lands κάτω
από το πίσω χείλος του control panel και οπή Ø8 για status LED στο local
`X+42/Y+70`. Οι control οπές και όλες οι θέσεις electronics/chassis έμειναν
αμετάβλητες. Τα panels βρίσκονται στο `../body-panels/stl-f-rear/` και έχουν
σήμανση **ΔΕΝ ΤΥΠΩΝΕΤΑΙ ΑΚΟΜΑ**.

Το P2-80 είναι μόνο μελλοντικό concept για πλήρως τυπωμένο stackable PETG
cage. Δεν έχει STL release· οι θέσεις κολονών θα ξανακριθούν με τα panels F.

Το P2-88 αντικαθιστά τα ήδη τυπωμένα αλλά μη συναρμολογήσιμα side frames.
Το νέο side frame χρησιμοποιεί εμπρός οπή βάσης μόνο μέσω του δηλωμένου jig,
πίσω wedge washer και διαμπερείς M4. Το upright jaw coupon έχει τυπωθεί και
περάσει, επομένως η σιαγόνα 4,8 mm/γλωττίδα 4 mm/M4 θέση δεν αλλάζει.
Η φυσική μικροδιόρθωση P2-90d δίνει και στο εμπρός base foot το ίδιο χαμηλό
18 mm πλευρικό hardware pocket με το πίσω: χωρά πλέον η M4 κεφαλή και η
OD14 ροδέλα, όχι μόνο το Allen μέσα από την Ø7 chimney. Μόνο το
`side_frame.stl` επανεξήχθη. Οι επίπεδες οροφές των pockets αντικαταστάθηκαν
από κλίσεις 45°. Ο έλεγχος εκτυπωσιμότητας της 24/10 κατέβασε επίσης τη
χαμηλή τραβέρσα από Z4 στο bed και έδωσε κλίσεις ≤45° στα cups και στα
washer/nut lands. Έτσι το όρθιο part δεν έχει πλέον μεγάλες κρυφές γέφυρες.

Το P2-90 μέτρησε την πραγματική UGREEN σε `115×45×45 mm`. Η P2-90b θήκη
δέχεται τη συσκευή από −X, με split side-installable caps και ξεχωριστό rear
retainer. Τα P2-90d side frames είναι πλέον τυπωμένα και συναρμολογημένα με
panel posts/control panel· τα παλιότερα side-frame exports είναι superseded.

Οι εκδόσεις v0/v1, τα παλιά review-only proposals και τα ανενεργά STL
βρίσκονται στο `archive/`. **ΜΗΝ ΤΥΠΩΣΕΙΣ** αρχεία από εκεί.
