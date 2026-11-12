# PRINT ARCH — P2-96b

Μόνο τα STL αυτού του φακέλου αποτελούν το ισχύον print set της αψίδας.
Τα `*_L.stl` και `*_R.stl` είναι χωριστά, ήδη κατοπτρισμένα και έτοιμα στο
bed. **Δεν γίνεται mirror, rotate ή drop-to-bed στον slicer.** Τα R παράγονται
ως ακριβή κατοπτρικά των L: print-axis Y για foot/crossbar_end/mast/mast_cap/
lidar_half και print-axis Z με αυτόματο drop-to-bed για knee/crown.

Το `verify-p296b-mirrors.py` απαιτεί ίδιο vertex multiset μετά το mirror και
ίδια bounding extents. Ο exporter ελέγχει επίσης ένα connected body ανά STL.

- `coupon_foot_seats`: ελέγχει X240/X230 washer seats/recesses και ένα Z80
  pin clevis. Test-only webs, όχι φέρον installed part.
- `coupon_knee_pins`: ελέγχει τα πραγματικά knee bores Z80/Z114.

Κάθε coupon είναι ένα σώμα και έχει planning print time κάτω από 1 h· η
τιμή πρέπει να επιβεβαιωθεί από τον πραγματικό slicer πριν την εκτύπωση.
