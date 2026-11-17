# PRINT ARCH — P2-96b

Μόνο τα STL αυτού του φακέλου αποτελούν το ισχύον print set της αψίδας.
Τα `*_L.stl` και `*_R.stl` είναι χωριστά, ήδη κατοπτρισμένα και έτοιμα στο
bed. **Δεν γίνεται mirror, rotate ή drop-to-bed στον slicer.** Τα R παράγονται
ως ακριβή κατοπτρικά των L: print-axis Y για foot/crossbar_end/mast/mast_cap/
lidar_half και print-axis Z με αυτόματο drop-to-bed για knee/crown.

Το `verify-p296b-mirrors.py` απαιτεί ίδιο vertex multiset μετά το mirror και
ίδια bounding extents. Ο exporter περνά κάθε STL από το κοινό gate για ένα
connected body, downward faces και τομές ≥1,6 mm / islands ≥25 mm².

**P2-97b — υποχρεωτική επανεκτύπωση foot:** το παλιό `foot_L/R` είχε
πραγματικό ligament μόλις 0,411 mm ανάμεσα στις δύο X240 εσοχές ροδέλας της
βάσης. Δεν ήταν στις πλευρικές παρειές με τις οπές. Το ήδη τυπωμένο παλιό
foot δεν επιτρέπεται να χρησιμοποιηθεί στο physical proof-load· τύπωσε ξανά
και τα δύο αρχεία αυτού του φακέλου. Ο exporter ελέγχει επιπλέον την canonical
φέρουσα γεωμετρία πριν από τη support-free περιστροφή, ώστε οι λοξές ακμές της
print orientation να μην μπορούν να κρύψουν πραγματικό λεπτό τοίχωμα.

- `coupon_foot_seats`: ελέγχει X240/X230 washer seats/recesses και ένα Z80
  pin clevis. Ενιαίο U-shaped test carrier, όχι φέρον installed part.
- `coupon_knee_pins`: ελέγχει τα πραγματικά knee bores Z80/Z114.

Κάθε coupon είναι ένα σώμα και έχει planning print time κάτω από 1 h· η
τιμή πρέπει να επιβεβαιωθεί από τον πραγματικό slicer πριν την εκτύπωση.
