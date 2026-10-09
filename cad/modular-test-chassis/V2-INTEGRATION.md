# Modular chassis v2 — unified integration assembly

Η v2 συναρμολόγηση χρησιμοποιεί ένα μόνο μηχανικό interface δοκού:
**50 × 30 mm προς 50 × 30 mm**. Το ίδιο ίσιο μανίκι χρησιμοποιείται μεταξύ
motor modules, στην κεντρική σύνδεση της πίσω τραβέρσας και στην είσοδο του
σώματος Γ. Οι πίσω γωνίες χρησιμοποιούν το ίδιο εσωτερικό περίβλημα
43 × 23 mm σε μορφή L.

## Διάταξη

- δύο motor modules μήκους 200 mm ανά πλευρά,
- wheelbase 200 mm για το πρώτο compact integration bench,
- εξωτερική παρειά πλευρικής δοκού `Y=±245 mm`,
- κέντρα δοκών κίνησης `Y=±220 mm`,
- σώματα Γ πλάτους 70 mm με κέντρα `Y=±210 mm`,
- η διαφορά των 20 mm μεγαλώνει μόνο προς το εσωτερικό,
- πίσω διμερής τραβέρσα και ανοικτό εμπρός μέρος,
- πραγματικά reinforced-v2 Γ και πραγματική wheel-first ράμπα ως assembly
  references.

Το σώμα Γ έχει universal socket 65 mm, ασύμμετρη διεύρυνση 30 mm και πέλμα
125 × 70 mm κάτω από τον υπάρχοντα κάνναβο M5 95 × 44 mm.

## Παραγόμενα integration STL

Το `export-v2-stls.sh` παράγει:

- `straight_sleeve.stl`,
- `corner_left.stl` και `corner_right.stl`,
- `crossbar_half.stl` (δύο τεμάχια),
- `gamma_left.stl` και `gamma_right.stl`,
- `ramp_cradle_left.stl` και `ramp_cradle_right.stl`,
- `electronics_tray_left.stl` και `electronics_tray_right.stl`.

Τα δύο μισά του electronics tray έχουν ωφέλιμη ενιαία επιφάνεια
190 × 290 mm και ανεξάρτητους βραχίονες προς τις πλευρικές δοκούς. Οι τελικές
οπές της πλακέτας και των βραχιόνων δεν έχουν ακόμη προστεθεί, επειδή απαιτούν
φυσική μέτρηση της πραγματικής πλακέτας και έλεγχο ότι δεν συμπίπτουν με τα
motor/splice fasteners.

Τα τέσσερα motor modules παραμένουν τα δύο κατοπτρικά αρχεία του
`engineering/chassis-strength-bench/coupon/`, από δύο τεμάχια το καθένα.

## Κατάσταση

Τα STL είναι **integration prototypes, όχι print release**. Το Gazebo θα
ελέγξει ενιαία κίνηση, συγκρούσεις, αποστάσεις και φορτία στις ενώσεις. Δεν
μπορεί να επαληθεύσει τοπική τάση στα τυπωμένα νεύρα. Πριν από πλήρη εκτύπωση
απαιτούνται μικρά coupons για το ίσιο μανίκι, τη γωνία και το socket του Γ.

Εξαγωγή:

```bash
cad/modular-test-chassis/export-v2-stls.sh
```
