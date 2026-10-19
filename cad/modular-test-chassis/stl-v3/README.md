# Phase-2 chassis print set v3

Status: frame geometry locked, but print `sleeve_fit_coupon.stl` together with
one `clamp_sleeve.stl` before any long part.  This directory is separate from
`stl-v2`; it removes the old `ramp_back` near X294 and the Gamma foot to X455.
The final rails end at X330.  Masses use the slicer-derived effective density
0.72 g/cm³ and must be replaced by the final slicer estimate.

**2026-10-15 fit-gate fix (physical coupon print).** Two STLs were wrong and
are re-exported:
- `sleeve_fit_coupon`: an internal 14 mm boss filled the rail at the lock hole,
  so the sleeve stopped ~23 mm in and its hole never reached the coupon hole.
  The boss is removed (real rails are hollow there).
- `front_terminal_left/right`: the rear-joint lock hole sat at the part centre
  (65 mm from the X200 joint) instead of 30 mm from it, outside the 60 mm
  sleeve engagement. Moved to 30 mm from the joint.
Probed in OpenSCAD: the sleeve inserts 60 mm without collision into both rear
motor ends and the terminal, and their top holes sit 30 mm from each joint.
Rail extension (centred, holes ±52.5 of 165) and crossbar halves (holes 30 mm
from each end) were checked from the code only, not probed.
Insert the sleeve from the coupon end nearest its hole, 60 mm deep (sleeve
middle flush with the coupon end).

**2026-10-15 through-bolt revision (all 18 STLs re-exported).** The hex nut
pockets inside sleeves, corner clamps and grid towers were enclosed in solid
material and unreachable without a print pause, so they are removed. Every
joint and grid side hole now passes through both walls. Assembly: M4 bolt +
washer, external washer + nut. Lengths: rail/rear-crossbar/corner top M4×40,
side M4×60; X240 top M4×30, side M4×60; X300 top/side M4×30; ballast plate
M4×50 with 10 mm spacers. Οι παλιές οδηγίες T-clamp/rail-through M4×65 και
diagonal centre joins είναι ιστορικές και αντικαθίστανται από την P2-78
ενότητα παρακάτω.
Motor mounts keep their own hardware. Probed: side holes pass through both
walls at every rail joint, the sleeve and the coupon; all STLs manifold.

**2026-10-18 P2-78 — X240/X300 sleeve sockets, όχι T-clamps.** Τα νέα
`front_terminal_left/right` έχουν δύο ενσωματωμένες, κούφιες υποδοχές στην
εσωτερική παρειά: 50×20 στο X240 και 20×20 στο X300, με την κάτω επιφάνεια
στο Z22 όπως η ράγα. Η διεπαφή υποδοχής–τραβέρσας είναι στο Y±130. Κάθε νέο
`x240/x300_terminal_sleeve` έχει συνολικό μήκος 120 mm και μπαίνει 60 mm σε
κάθε μέρος. Οι lock σταθμοί είναι ακριβώς 30 mm από τη διεπαφή, Y±100 και
Y±160. Η Y100 εξυπηρετεί ταυτόχρονα το υπάρχον grid· αυτή η επιλογή διορθώνει
το αρχικό trial Y135, που άφηνε μόνο 0,5 mm ligament ανάμεσα στις οπές Y100
και Y105. Τα grid Y75/100/125/150 περνούν και από το μανίκι, χωρίς εσωτερικό
εμπόδιο. Οι κεντρικές `x240/x300_clamp_sleeve` δεν αλλάζουν.

Τα `x240_t_clamp` και `x300_t_clamp` καταργούνται και μεταφέρθηκαν στο
`../archive/p278-superseded-stl/`. Οι νέες μισές τραβέρσες έχουν μήκος 130 mm. Το C-hook
relief X310/Y±185 βρίσκεται πλέον στην ενσωματωμένη X300 socket και όχι στην
τραβέρσα. Για κάθε άκρο χρησιμοποίησε M4 με ροδέλα και εξωτερικό παξιμάδι:
X240 συνολικά 4× top M4×30 + 4× transverse M4×60. Στο X300 χρειάζονται
2× top M4×30 στα Y±160 και 4× transverse M4×30 στα Y±100/160. Τα top locks
Y±100 μοιράζονται δύο από τις υπάρχουσες 4× M4×35 των αμετάβλητων ski
brackets (οι άλλες δύο μένουν Y±75). Χωρίς brackets, βάλε προσωρινά 2×
M4×30 στα Y±100.

Το torsion deck είναι τώρα 2× `torsion_side_brace`, ένα ανά πλευρά,
X240/Y±150 → X300/Y±125. Κάθε μονοκόμματο 20×15 body γεφυρώνει μόνο το
διάκενο μεταξύ των αντικριστών παρειών X240/X300. Τα end lands είναι 23×23,
με 9,25 mm καθαρό υλικό από την παρειά Ø4,5 έως την άκρη. Η κάτω παρειά είναι
στο CAD Z20,5 (περίπου 18,5 mm στο φυσικό chassis), άρα κανένα νέο frame part
δεν πέφτει κάτω από το όριο φυσικού Z≈15. Το X240 land έχει μόνο ένα τοπικό
άνω inboard relief για τον future basket floor. Στερέωση στις υπάρχουσες
side-grid οπές: ανά brace 1× M4×65 στο X240/Y150 και 1× M4×35 στο X300/Y125,
με μεγάλες ροδέλες και εξωτερικά παξιμάδια.

Τα παλιά `torsion_diagonal_half`, `torsion_joiner` και το απορριφθέν
`torsion_diagonal_onepiece` μεταφέρθηκαν στο `../archive/p278-superseded-stl/`· το P2-77
centre node/drill jig επίσης δεν χρησιμοποιείται με αυτή τη λύση.

FEA 5 mm mesh με τις νέες full-section socket load paths: χωρίς διαγώνιο
0,6454 / 2,3121 / 2,3077 mm, με μία πλήρη 20×15
0,6449 / 1,9746 / 2,0276 mm και με τα επιλεγμένα δύο short braces
0,6427 / 2,1770 / 2,1840 mm (vertical 2g / launch / antisymmetric). Το
FEA-optimal αντίστροφο ζεύγος X240/Y125→X300/Y150 απορρίφθηκε από exact-solid
λόγω τομής 0,08–0,45 mm με τον μελλοντικό πάτο. Το επιλεγμένο ζεύγος είναι
καλύτερο από P2-77 κατά 8,35%/7,23% στα δύο ασύμμετρα cases. Όλα τα τελικά
P2-78 exact-solid collision/alignment exports είναι κενά.

**Ιστορικό, superseded από P2-78 — 2026-10-16 διόρθωση P2-65 (Y150).** Ο βραχίονας 60 mm των `x240_t_clamp` /
`x300_t_clamp` (Y135…195) χτυπούσε στον εσωτερικό πυργίσκο του grid στο Y150
της τραβέρσας και δεν μπορούσε να μπει. Οι `x240_crossbar_half` και
`x300_crossbar_half` ξαναβγήκαν χωρίς πυργίσκο στο Y150· οι τρύπες Y150 (πάνω
και πλάι) μένουν και στηρίζονται από τον συμπαγή βραχίονα. Ελέγχθηκε: βραχίονας
μέσα στην τραβέρσα χωρίς τομή, τρύπες 30 mm και Y150 ευθυγραμμισμένες.

**2026-10-16 διόρθωση πίσω γωνιών.** Η πίσω τραβέρσα έφτανε ως τη μέση της
πλαϊνής δοκού (Y±220) και στη γωνία οι δύο σωλήνες τέμνονταν σε 25×25 mm, άρα
οι τρύπες της γωνίας έπεφταν 25 mm εκτός. Τώρα κάθε `rear_crossbar_half` έχει
μήκος 195 (τελειώνει στην εσωτερική παρειά της δοκού, grid Y75/100/125) και το
`corner_clamp` έχει βραχίονα 60 mm σε κάθε σωλήνα + συμπαγές μπλοκ στη γωνία,
ίδιο ύψος 23 mm ώστε να τυπώνεται επίπεδα. Ελέγχθηκε στο OpenSCAD: καμία τομή
δοκού–τραβέρσας–γωνίας (μόνο επαφή επιφανειών) και οι 4 τρύπες κάθε γωνίας
ευθυγραμμίζονται. **Ξανατυπώνονται:** `corner_clamp_left/right` και
`rear_crossbar_half` ×2. Τα υπόλοιπα τυπωμένα δεν αλλάζουν.

**Ιστορικό, superseded από P2-78 — 2026-10-16 P2-65.** Και οι δύο μισές τραβέρσες είχαν
μήκος 195 mm και σταματούν στην εσωτερική παρειά Y=±195. Σε κάθε άκρο μπαίνει
ξεχωριστό T-clamp κατά 60 mm: χρησιμοποιεί τις υπάρχουσες επάνω/πλάγια M4
της τραβέρσας και ένα στενό tab βιδώνει διαμπερώς στο front terminal. Δεν
αλλάζει κανένα ήδη τυπωμένο κομμάτι. Στα `front_terminal_left/right` ανοίγονται
επί τόπου οπές Ø4,5 mm, άξονας Y, στα **X240/Z46** και **X300/Z46 mm**. Βγάλε
κινητήρες/καλώδια, κάνε dry-assembly με τα T-clamps ως οδηγούς και τρύπησε
κάθε terminal από μέσα προς τα έξω: στο X240 η οπή περνά και από το ήδη
τοποθετημένο longitudinal sleeve, άρα τρυπιούνται μαζί για ευθυγράμμιση.
Χρησιμοποίησε M4×65, χαμηλή κεφαλή+ροδέλα outboard και ροδέλα+παξιμάδι
inboard. Η κεφαλή προεξέχει το πολύ 4 mm και αφήνει 6,5 mm ως τη μετρημένη
εσωτερική παρειά του Baja tyre.

Ο P2-65 exact έλεγχος βρήκε επίσης ότι το M6 C-hook X310/Y±185 της πλάκας
έρματος έτεμνε την άκρη της παλιάς X300. Η νέα X300 έχει συμμετρικό relief
Ø9 mm, με 1,25 mm nominal ακτινικό διάκενο γύρω από το Ø6,5 envelope. Τα
grid Y75/100/125/150 και το endpoint Y150 της διαγωνίου διατηρούνται. Η
ευθεία κάθοδος του εμπρός FIT0403 cable στο X300 επίσης δεν επιτρέπεται:
οδήγησέ το πρώτα πίσω στο X275 πάνω από το deck, κατέβασέ το inboard στο
Y185 και πέρασέ το κάτω από Z22. Το checked envelope κινητήρα/encoder και
αυτή η διαδρομή καλωδίου έχουν μηδενική τομή.

Στον ίδιο οπτικό έλεγχο βρέθηκε ότι το `frame_assembly` τοποθετούσε όλα τα
παλιά centered sleeves/corner clamps κατά 11,5 mm χαμηλότερα από τις δοκούς.
Δεν ήταν σφάλμα των STL και κανένα ήδη τυπωμένο part δεν άλλαξε. Διορθώθηκαν
μόνο οι assembly transforms: rail/rear-corner sleeve centre Z37 και low
X240/X300 sleeve centre Z32. Οι πλάγιες οπές τώρα συμπίπτουν με τα κέντρα των
αντίστοιχων σωλήνων.

P2-52 note: το locked v3 chassis παραμένει ανεξάρτητο. Actuator clevises,
Tr8 carriage/thrust housing, AS5600 pod, indexed stops, selector cam και
front basket/feed-channel panels είναι **DEPENDENT** και δεν ανήκουν σε
αυτό το STL release. Βλ. `docs/mechanism/shared-flywheel-transition-p252-el.md`.

| Printed| STL | Τι είναι | Qty | g/part | g total | Print orientation | State / dependency |
|---:|---|---|---:|---:|---:|---|---|
| &#x2611; | `sleeve_fit_coupon` | Δοκιμαστικό κομμάτι δοκού για έλεγχο εφαρμογής μανικιού και βιδών. Δεν μπαίνει στο ρομπότ. | 1 | 36.9 | 36.9 | rail bottom on bed | LOCKED; first physical fit gate |
| &#x2611; | `clamp_sleeve` | Εσωτερικό μανίκι-σύνδεσμος: μπαίνει 60 mm μέσα σε δύο δοκούς 50×30 και τις ενώνει. 4 στις ενώσεις των πλαϊνών δοκών (X35, X200 ανά πλευρά) + 1 στο κέντρο της πίσω τραβέρσας. | 5 | 41.9 | 209.7 | 43 mm face on bed | LOCKED; print one with coupon first |
| &#x2610; | `corner_clamp_left` | Πίσω αριστερή γωνία: ενώνει την πλαϊνή δοκό με την πίσω τραβέρσα σε 90° (μπαίνει μέσα και στις δύο). **Νέα έκδοση 2026-10-16: ξανατύπωσε.** | 1 | 76.1 | 76.1 | 23 mm broad face on bed | LOCKED; rear-left |
| &#x2610; | `corner_clamp_right` | Πίσω δεξιά γωνία, κατοπτρική της αριστερής. **Νέα έκδοση 2026-10-16: ξανατύπωσε.** | 1 | 76.1 | 76.1 | 23 mm broad face on bed | LOCKED; rear-right |
| &#x2611; | `rear_motor_left` | Πίσω τμήμα αριστερής δοκού (X−165…35) με τη βάση του πίσω αριστερού κινητήρα FIT0403. | 1 | 103.8 | 103.8 | rail bottom on bed | LOCKED; FIT0403 land |
| &#x2611; | `rear_motor_right` | Πίσω τμήμα δεξιάς δοκού με τη βάση του πίσω δεξιού κινητήρα. | 1 | 103.9 | 103.9 | rail bottom on bed | LOCKED; FIT0403 land |
| &#x2611; | `rail_extension_left` | Μεσαίο τμήμα αριστερής δοκού (X35…200), με τρύπες M4 για αξεσουάρ (π.χ. η κονσόλα κουμπιών στα X100/X125). | 1 | 64.9 | 64.9 | rail bottom on bed | LOCKED; M4 grid outside sleeve zones |
| &#x2611; | `rail_extension_right` | Μεσαίο τμήμα δεξιάς δοκού. | 1 | 65.5 | 65.5 | rail bottom on bed | LOCKED; M4 grid outside sleeve zones |
| &#x2610; | `front_terminal_left` | Μπροστινό άκρο αριστερής δοκού με αμετάβλητα X200 sleeve/FIT0403 land και ενσωματωμένες κούφιες υποδοχές X240 50×20 + X300 20×20. **P2-78: ξανατύπωσε.** | 1 | 104.9 | 104.9 | rail bottom on bed | LOCKED; P2-78 socket terminal |
| &#x2610; | `front_terminal_right` | Κατοπτρικό μπροστινό άκρο δεξιάς δοκού με τις δύο P2-78 υποδοχές. **P2-78: ξανατύπωσε.** | 1 | 104.9 | 104.9 | rail bottom on bed | LOCKED; P2-78 socket terminal |
| &#x2611; | `rear_crossbar_half` | Μισό της πίσω τραβέρσας (50×30, στο X−165, μήκος 195). Τα δύο μισά ενώνονται στη μέση με clamp_sleeve και στις άκρες με τις γωνίες. Κλείνει το πλαίσιο πίσω. **Νέα έκδοση 2026-10-16: ξανατύπωσε.** | 2 | 92.8 | 185.6 | long flat bottom on bed | LOCKED; M4 grid Y75–150 |
| &#x2610; | `x240_crossbar_half` | Μισό 130 mm της X240 (50×20), από το κέντρο έως τη διεπαφή Y±130. Τα Y75/100/125 μένουν διαθέσιμα. **P2-78: ξανατύπωσε.** | 2 | 37.5 | 75.0 | long flat bottom on bed | LOCKED; P2-78 socket length |
| &#x2611; | `x240_clamp_sleeve` | Χαμηλό μανίκι που ενώνει τα δύο μισά της X240 στο κέντρο. | 1 | 11.9 | 11.9 | 43 mm face on bed | LOCKED; centre splice |
| &#x2610; | `x240_terminal_sleeve` | Μανίκι 120 mm της ένωσης X240 terminal socket–crossbar, 60 mm ανά πλευρά, με pass-through στα Y75/100/125/150. **Νέο P2-78.** | 2 | 11.7 | 23.4 | 43 mm face on bed | LOCKED; one per end |
| &#x2610; | `x300_crossbar_half` | Μισό 130 mm της X300 (20×20), από το κέντρο έως Y±130. Τα Y75/100 μένουν για τα ski brackets. **P2-78: ξανατύπωσε.** | 2 | 19.5 | 39.1 | long flat bottom on bed | LOCKED; P2-78 socket length |
|&#x2611; | `x300_clamp_sleeve` | Χαμηλό μανίκι που ενώνει τα δύο μισά της X300 στο κέντρο. | 1 | 5.1 | 5.1 | 13 mm face on bed | LOCKED; centre splice |
|&#x2610; | `x300_terminal_sleeve` | Μανίκι 120 mm της ένωσης X300 terminal socket–crossbar, 60 mm ανά πλευρά. **Νέο P2-78.** | 2 | 5.0 | 9.9 | 13 mm face on bed | LOCKED; one per end |
|&#x2610; | `torsion_side_brace` | Κοντή μονοκόμματη πλευρική διαγώνιος 20×15 με 23×23 end lands, X240/Y±150 → X300/Y±125, μέσα στο διάκενο των τραβερσών. **Νέο P2-78.** | 2 | 7.9 | 15.7 | broad face on bed | LOCKED; M4×65 + M4×35, large washers |
|&#x2610; | `ballast_tab` | Προσωρινό πτερύγιο για στερέωση έρματος στο τεστ κίνησης. Όχι στο τελικό ρομπότ. | 4 | 11.8 | 47.3 | 70×30 face on bed | TEMPORARY; plywood/metal ballast deck |

Locked frame mass excluding the coupon and temporary tabs is approximately
1.308 kg.  The complete table including the coupon and four temporary tabs is
approximately 1.382 kg.  All 19 active STL meshes pass the repository manifold
check (`nonmanifold_edge_count=0`).

The M4 grid is on top and outer side faces only, at 25 mm pitch where the
geometry permits it.  It is excluded from unrelated sleeves/insertion zones,
joint-lock holes and motor lands; P2-78 sleeves have explicit pass-throughs at
the retained crossbar grid. A conservative local
net-section calculation gives at most about 5% loss of vertical bending
second moment at a top hole; the surrounding nut tower is not credited.  This
is not fatigue or printed-layer certification.

## Not in the locked STL set

- DEPENDENT: X330 arch/uprights, bearing carriers/guards, moving flywheel
  module, actuator clevis and replaceable endpoint stops.  They wait for
  bearing/runout measurements and the physical P16 linkage check.
- SUPERSEDED: το P2-48 ramp-inside-basket δεν είναι πλέον λειτουργικό
  candidate.  Διατηρείται μόνο ως ιστορικό CAD.
- DEPENDENT: P2-49 separated-path basket, LiDAR mast and camera bracket.  The basket is
  geometrically sized for 45 balls but its mandatory physical gate is 40;
  balls 41–45 are best-effort.  The separate dependent panel CAD is
  `../shared-flywheel-basket-p249-separated.scad`: external central
  collection bay, launch-feed channel below, divider X100, sill Z285 and
  rim Z360.
  It has no STL release until the printed-basket pile/entry/retention/port-flow
  test.  Camera optical and physical mast-vibration checks are also open.
- DEPENDENT: rear quick-change docks for the two Parkside X20V packs.  The
  M4 grid and CAD reserve X=-165/Y=+/-110 mm, but pack, adapter and release
  sweep must be measured before dock STL release.  UGREEN is reserved at
  X=-100 mm on the separate logic-power branch.
- The old Gamma pair is **not required and must not carry drive-test ballast**.
  Its feet were sized around 0.4 N·m, while the old X≈424 mm ballast proposal
  imposed roughly 3 N·m static moment on each foot.  Use the final
  `front_terminal_left/right` plus the X300 crossbar and the removable plate
  below.  The old Gamma STLs remain untouched only as superseded history.

## Drive-test ballast plate (replaces Gamma)

> **2026-10-16, απόφαση χρήστη: ΟΧΙ κόντρα πλακέ.** Το έρμα θα είναι ένας
> δίσκος γυμναστηρίου 5 kg πάνω σε τυπωμένο δίσκο PETG (σχεδίαση σε εξέλιξη).
> Οι πρώτες δοκιμές λειτουργίας γίνονται χωρίς έρμα. Ο στόχος μάζας/CG παρακάτω
> (5,448 kg, X360) ισχύει· η κατασκευή από plywood/μέταλλο ΑΝΤΙΚΑΘΙΣΤΑΤΑΙ.

Fabricate the plate in `../v3-drive-test-ballast-plate.scad` from 12 mm
plywood or an equivalently stiff metal plate: 160×480 mm, centre X=360/Y=0,
spanning X=280…440.  Eight M4 through holes use the actual X300 grid at
Y=±75/100/125/150 mm, with rigid 10 mm spacers from crossbar top Z42 to plate
bottom Z52.  Four M6 holes at X=310/330, Y=±185 mm take the inward legs of
two removable C-hook clamps that reach under the final front terminals.
The clamp stays inboard of the terminals and out of the tyre gap; the new X300
edge relief clears the X310 leg. The ballast clamps do not add terminal holes
and do not share the FIT0403 motor bolts; the P2-78 socket locks are separate
from the motor land. Four M8 tie points at
X=335/385 and Y=±60 secure the ballast in all axes.

The weighed total of plate + fasteners + loose ballast shall be **5.448 kg**
with combined CG at X=360 mm.  This equals the rear-axle moment of the
4.706 kg module at X=427 mm and therefore reproduces the front-axle load
(112.19 N total in the current empty-basket model, including the 95 g P16).
It is not dynamically
identical: the point-mass pitch/yaw term about the rear axle is 13.6% lower,
the full test chassis is 0.742 kg heavier, and its longitudinal CG is X=156.5
mm instead of X=165.6 mm.  Weigh the finished plate and shift the loose mass
until both mass and X-CG meet the targets; do not rely on nominal material
density.

If the real power packs are absent, add separate dummy masses at their actual
locations rather than to the front plate: 0.650 kg at each X=-165/Y=±110 mm
Parkside dock and 0.309 kg at X=-100/Y=0 for UGREEN.  These dummies must be
independently bolted/strapped.  Moving them to X=360 would invalidate the axle
load match.

For the 45-ball basket case, the provisional stored-ball mass is 2.565 kg at
X≈105 mm.  The basket/support estimate is 1.70 kg at X≈120 mm.  Neither is a
print release; use secured dummy masses only for the drive test.

### Dependent basket/electronics pieces (not in `stl-v3`)

- `P2-49 gridded wall/floor panels` — DEPENDENT; wall openings ≤38 mm,
  floor openings ≤18 mm, 4.5°/2° funnel slopes and solid throat fairings.
- `P2-49 divider + external collection ramp` — DEPENDENT; high opening
  sill Z285, clear 80 mm, entirely outside storage.
- `P2-49 launch-feed channel + selector cassette` — DEPENDENT; channel
  below the ramp.  Gate/diverter actuator, interlocks and singulator remain 5d.
- `d5065_controller_shelf_high` ×2 — DEPENDENT on controller choice and cable
  exits; reserved at X=80/Y=±145/Z=245 above the basket.
- `ugreen_retain_and_release` ×1 — DEPENDENT on the physical powerbank/cable
  access; nominal centre remains X=-100/Y=0/Z=145.

Each P2-78 side brace is a pin-jointed tension/compression member. Its holes
reuse the transverse side-grid axes X240/Y±150 and X300/Y±125 through the
aligned terminal sleeves. Use M4×65 at X240 and M4×35 at X300, with a large
washer or thin metal backing plate on both faces of each single-shear endpoint.
