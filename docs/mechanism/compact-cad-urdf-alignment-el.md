# Compact packaging: αντιστοίχιση CAD ↔ URDF

**Κατάσταση: ΑΝΑΛΥΣΗ ΜΟΝΟ.** Καμία αλλαγή στο xacro δεν έγινε από αυτό το
έγγραφο. Σκοπός του είναι να πει, με μετρήσεις και όχι με ανάγνωση
παραμέτρων, πού το simulation model διαφέρει από το σχέδιο.

## Ποιο variant είναι η μηχανή

Η πηγή αλήθειας είναι το `cad/flywheel-launcher-v0/compact-packaging-study.scad`.
Αντιστοιχεί στο URDF variant **`compact`** — **όχι** στο `option-a-launch`.

Στο `scripts/generate_robot_urdf.py`:

```python
functional_shift_x = "0.0" if option_a_collect or not compact else "-0.100"
# option_a_collect = variant in {"option-a-collect", "option-a-launch"}
```

Δηλαδή το `option-a-launch` παίρνει `functional_shift_x = 0.0`, ενώ η μελέτη
ορίζει `functional_shift_x = -100`. Το `option-a-launch` **δεν είναι** η compact
διάταξη· είναι η αμετατόπιστη διάταξη με σβησμένο το intake.

Απόφαση: **το `compact` είναι η μηχανή.** Τα `option-a-collect` / `option-a-launch`
μένουν ως έχουν, ως ιστορικά collection-only layouts.

## Μεθοδολογία

Και οι δύο πλευρές **μετρήθηκαν**, δεν διαβάστηκαν:

- **CAD:** export STL ανά εξάρτημα με OpenSCAD (`openscad/openscad:bookworm`),
  απευθείας από τα ίδια τα αρχεία της μελέτης, με το `functional_shift_x = -100`
  εφαρμοσμένο· bounding box από τα vertices του STL. Για τον launcher τα
  `show_guard` / `show_feed_keepout` / `show_reference_balls` γυρίστηκαν σε
  `false` (είναι envelopes ελευθερίας, όχι εξαρτήματα).
- **URDF:** παραγωγή του variant `compact` με τον κανονικό generator· bounding
  box από την πραγματική αλυσίδα joints, collision **και** visual.

Οι αρμοί ανάλυσης αφαιρέθηκαν μετά τη μέτρηση· δεν έμεινε τίποτα στο δέντρο.

## Μετρημένα envelopes (mm, world frame, shift −100 εφαρμοσμένο)

| εξάρτημα | CAD x | CAD y | CAD z | URDF x | URDF y | URDF z |
|---|---|---|---|---|---|---|
| plywood bridge | 280 … 500 | ±235 | **52 … 168** | — | — | **ΑΠΟΝ** |
| cheeks | 455 … 708 | ±208 | 18 … **150** | 346 … 714 | ±212.6 | 18.8 … **281.1** |
| handoff ramp | 319.6 … 360.4 | ±94 | 0 … 53 | — | — | **ΑΠΟΝ** (χωνεμένο στο `funnel_link`) |
| wheel motor pods | 298.3 … 515.7 | ±152 | 4.5 … 269.2 | 355 … 385 | ±103 | 112 … 124 |
| launcher (δομή) | 323.6 … 776.1 | ±229 | **127.1** … 366.7 | 359.2 … 757.3 | **±254** | **23.8** … 359.9 |
| basket (collect) | −112 … 370.6 | ±172 | 19 … 285 | −90 … 372 | ±155 | 20 … 252 |
| basket (launch) | −99 … 370 | ±172 | **129.9 … 444.9** | ίδιο με collect +100 | ±155 | 120 … 352 |

## Τι είναι ήδη ευθυγραμμισμένο

`functional_shift_x` −100 · chassis 920×580 με top στα 52 · drive wheels d170
w80 στα x±330 / y±350 · drive motors d60 l100 στα z85 / y±240 · battery center
x −255 · LiDAR scan plane 498 · launcher origin x 460 (= 560 − 100) · nip 215 ·
flywheel d200 · nip gap 58 · pitch 20° · basket lift travel 100 · intake nip x
370 (0.540 − 0.100 − 0.070) · intake wheel gap 56.

## Οι επτά αποκλίσεις

1. **Η γέφυρα δεν υπάρχει.** `grep -rn bridge ros2_ws/src/tennis_robot/urdf/`
   δεν βρίσκει τίποτα. Στο CAD είναι 18 mm plywood portal, `z[150,168]`, και
   είναι **το επίπεδο διαχωρισμού intake/launcher και ο φορέας του launcher**.
   Στη compact μελέτη δέχεται δύο αφαιρετικά service features (rear notch για
   την εξαγωγή του καλαθιού, motor arches στα ξύλινα πόδια) — ούτε αυτά
   υπάρχουν.

2. **Τα cheeks είναι 131 mm ψηλότερα.** CAD `oa_cheek_top_z = oa_bridge_under_z
   = 150`· URDF `funnel_link` φτάνει `281.1`. Τα cheeks περνούν ίσια μέσα από
   το επίπεδο της γέφυρας, στον όγκο του launcher. **Αυτή είναι η πραγματική
   αιτία των μετρημένων συγκρούσεων intake/launcher** — όχι κάποια μηχανική
   ασυμβατότητα.

3. **Τα cheeks εκτείνονται 109 mm πιο πίσω.** CAD `x[455,708]`· URDF
   `x[346,714]`.

4. **Ο launcher κρέμεται 103 mm χαμηλότερα και είναι 50 mm πιο φαρδύς.**
   CAD cradle = δύο ελάσματα 8 mm (`side_plate_t=8`, `side_plate_y=±43`,
   `cradle_margin=28`) → `z_min 127.1`, `y ±229`. URDF = μία πλάκα
   280×508×35 στα `z = -(wheel_radius + 0.035)` → `z_min 23.8`, `y ±254`.
   Λάθος εξάρτημα, όχι λάθος θέση.

5. **Το πλάτος του flywheel είναι 40 mm αντί 50.** CAD `wheel_width = 50`
   (`launcher-envelope.scad`)· xacro `wheel_width:=0.040`.

6. **Ο handoff ramp δεν είναι διακριτό στοιχείο.** Η compact μελέτη ορίζει
   `compact_handoff_ramp()` — δική της γεωμετρία, `x[319.6,360.4]`,
   `z 1.5 → 35`, πλάτος 180, τοίχωμα 18 — ρητά **όχι** τον `short_handoff_ramp()`
   του Option A, με τον σχολιασμό ότι πρέπει να επικυρωθεί σε Gazebo sweep πριν
   εξαχθεί ως αντικαταστάτης. Στο URDF δεν υπάρχει αντίστοιχο link.

7. **Δεν υπάρχει άξονας tilt.** Η μελέτη ορίζει τη στάση launch ως
   `lift_travel = 100` **ΚΑΙ** `rotate([0, 12, 0])` περί `launch_pivot =
   [470,0,40]` — μετρημένα, το χείλος ανεβαίνει στα `444.9` έναντι `285` στη
   στάση collect. Το URDF μοντελοποιεί μόνο την ανύψωση (`z_max 352`). Αυτό
   είναι ήδη γνωστό ως `MECHANICAL_VALIDATION_PENDING`· καταγράφεται εδώ με τον
   αριθμό του.

## Δευτερεύουσες διαφορές

- intake wheel: CAD `oa_wheel_d = 124` / `oa_wheel_width = 73`· xacro
  `intake_wheel_radius = 0.060` (d=120) / `intake_wheel_height = 0.080`.
- motor pods: το URDF μοντελοποιεί μόνο ένα μικρό carriage
  (`x[355,385] z[112,124]`) έναντι πλήρους pod envelope `x[298,516] z[4.5,269]`.
- basket half-width: CAD ±172 (με flanges/λαβές) έναντι ±155 στο URDF.

## Συνέπεια

Ο ισχυρισμός «intake και launcher δεν συνυπάρχουν» είναι **ιδιότητα του URDF,
όχι της μηχανής**. Οι αποκλίσεις 1, 2, 3 και 4 τον παράγουν από μόνες τους.
Το CAD τα μοντάρει και τα δύο μόνιμα· το `robot-integration.scad` το λέει ρητά:

```text
// Launcher stays mounted in both modes; interlock state changes.
```

Καμία δοκιμή γεωμετρίας ή εκτόξευσης δεν έχει νόημα πριν κλείσουν οι
αποκλίσεις 1–5.
