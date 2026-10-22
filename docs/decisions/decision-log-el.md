# Ιστορικό αποφάσεων σχεδιασμού

Κατάσταση: P2-89, 2026-10-22. Οι ημερομηνίες P2 ακολουθούν τη χρονική
εγγραφή του αντίστοιχου βήματος/debug log και όχι την ημερομηνία του
μεταγενέστερου commit που συγκέντρωσε τα αρχεία.

## 2026-06-07 — Webots → Gazebo/ROS 2

- **Πλαίσιο:** το αρχικό prototype χρησιμοποιούσε Webots και Python.
- **Απόφαση:** αφαιρέθηκε ο Webots runtime και η ενεργή simulation/runtime
  στοίβα μεταφέρθηκε σε Gazebo/ROS 2.
- **Γιατί:** **ΤΕΚΜΗΡΙΩΣΗ ΛΕΙΠΕΙ**. Το commit `b55a3f10` αποδεικνύει τη
  μετάβαση, όχι το συγκριτικό rationale του χρήστη.
- **Απορρίφθηκε:** συνέχιση του Webots stack ως κύριου runtime.

## 2026-07-10 — single top roller → dual-wheel intake

- **Πλαίσιο:** το single top-roller+scoop είχε στενό simulation sweet spot.
- **Απόφαση:** controlled dual-wheel pinch και ενεργή μεταφορά προς ramp/hopper.
- **Γιατί:** η αλλαγή ύψους/ταχύτητας έχανε contact ή έδινε λάθος release,
  ενώ το μοναδικό 7/7 case είχε πολύ χαμηλή ταχύτητα και no crest. Δεν
  προέκυψε αρκετά μεγάλο operating envelope για πραγματικές ανοχές.
- **Απορρίφθηκε:** περαιτέρω tuning του single top roller ως production path.
- **Evidence:** [`intake-concept-decision-el.md`](../mechanism/intake-concept-decision-el.md).

## 2026-07-23 — ROS 2 Humble → Jazzy

- **Πλαίσιο:** τα Nav2 parameters είχαν Jazzy plugin names και ο Humble
  container δεν μπορούσε να ενεργοποιήσει τον planner.
- **Απόφαση:** Ubuntu 24.04 + ROS 2 Jazzy έγινε canonical runtime.
- **Γιατί:** στο collection debug #65 ο Humble planner απέτυχε πριν από
  κίνηση, ενώ στο #66 το native Jazzy route ολοκληρώθηκε με 4/4 συλλογές.
- **Απορρίφθηκε:** συντήρηση Humble compatibility στο production path.
- **Evidence:** `collection-route-debug-log-el.md` #65/#66 και
  `docs/process/pi-deployment-plan-el.md`.

## 2026-08-26 — standalone flywheel launcher

- **Πλαίσιο:** χρειαζόταν ελεγχόμενη εκτόξευση ανεξάρτητα από το τότε intake.
- **Απόφαση:** προσωρινό dual direct-drive standalone launcher baseline.
- **Γιατί:** επέτρεψε ξεχωριστά mechanical, contact και capability gates.
  Πλήρες comparative rationale έναντι όλων των launcher alternatives:
  **ΤΕΚΜΗΡΙΩΣΗ ΛΕΙΠΕΙ**.
- **Απορρίφθηκε:** άμεσο manufacturing release χωρίς wheel/hub/traction tests.
- **Evidence:** [`standalone-flywheel-launcher.md`](../mechanism/standalone-flywheel-launcher.md).

## 2026-10-11 — κοινό περιστρεφόμενο flywheel

- **Πλαίσιο:** ξεχωριστό dual-wheel intake και dual-wheel launcher απαιτούσαν
  δύο μηχανισμούς, επιπλέον χώρο, μοτέρ και drivers.
- **Απόφαση χρήστη:** ένα ζεύγος flywheels εξυπηρετεί intake και launch με
  αλλαγή κλίσης όλης της μονάδας.
- **Γιατί:** συμπαγής αρχιτεκτονική και κόστος αγορών. Ένας D5065 αναφέρεται
  περίπου `99 €`· δύο χωριστά ζεύγη θα διπλασίαζαν D5065 και αντίστοιχους
  controllers. Πλήρης ποσοτική σύγκριση κόστους: **ΤΕΚΜΗΡΙΩΣΗ ΛΕΙΠΕΙ**.
- **Απορρίφθηκε:** δύο πλήρως ανεξάρτητοι μηχανισμοί intake/launcher.
- **Evidence:** [`shared-flywheel-intake-launcher-handoff-el.md`](../mechanism/shared-flywheel-intake-launcher-handoff-el.md), Phase 1.

## 2026-10-11 — Ø200 αντί Ø160

- **Πλαίσιο:** συγκρίθηκαν δύο wheel diameters στο ίδιο D5065 baseline.
- **Απόφαση:** Ø200 class ως κύρια γεωμετρία.
- **Γιατί:** 30 m/s απαιτούν 2865 rpm, 82,9% του theoretical 3456 rpm.
  Το Ø160 απαιτεί 3581 rpm, πάνω από το theoretical no-load, και χάνει 20%
  surface speed στο ίδιο rpm.
- **Απορρίφθηκε:** Ø160 ως κύριο launch wheel.
- **Evidence:** shared-flywheel Phase 1, §«Ø200 έναντι Ø160».

## 2026-10-12 — P2-40 παγωμένη γεωμετρία λειτουργίας

- **Πλαίσιο:** P2-37…39 έδειξαν ότι static και moving clearances απαιτούν
  διαφορετικούς κανόνες.
- **Απόφαση:** `−3°` συλλογή, `0°` μεταφορά, `+15…+20°` εκτόξευση.
- **Γιατί:** 9/9 R/h edge-grid cases δίνουν 8 mm, το arch directional screen
  αφήνει ≥11 mm μετά tolerance budget και όλα τα exact sweeps είναι κενά.
- **Απορρίφθηκε:** σταθερό −10° intake endpoint και μη ελεγμένα edge grids.
- **Evidence:** debug log P2-40.

## 2026-10-13 — καλάθι 45 design / 40 mandatory

- **Πλαίσιο:** ο όγκος έπρεπε να καλύπτει full-basket use χωρίς μη
  βαθμονομημένο multi-ball simulation claim.
- **Απόφαση χρήστη:** design για 45, physical acceptance με 40 υποχρεωτικές.
  Οι 41–45 είναι best-effort μέχρι φυσικό mockup/pile retention.
- **Απορρίφθηκε:** claim ότι 45 μπάλες έχουν ήδη επικυρωθεί φυσικά.
- **Evidence:** debug log P2-48/P2-49 και P2-71.

## 2026-10-17 — R75/75, tilted arch και fail-closed front wall

- **Πλαίσιο:** ο R75/42 βατήρας και η παλιά αψίδα δεν έδιναν την απαιτούμενη
  ασφαλή διαδρομή/συγκράτηση.
- **Απόφαση:** R75/75, arch X390/Z245, deflector, upright wall X225/top Z235
  και fail-closed 80×80 door.
- **Γιατί:** P2-67 180/180 PASS, R75/75 90/90 και περισσότερο packaging room
  από 80°. P2-69 confirmation 30/30, χωρίς forbidden contacts.
- **Απορρίφθηκε:** R75/42 baseline, R75/80 ως default και tilted wall X278.
- **Evidence:** debug P2-67/P2-68/P2-69.

## 2026-10-17 — πάγωμα performance και αγορών

- **Πλαίσιο:** οι προσομοιώσεις δεν αποδεικνύουν φυσική κίνηση, tyre friction
  ή πραγματική flywheel απόδοση.
- **Απόφαση χρήστη:** το flywheel performance παγώνει μέχρι να επιβεβαιωθεί
  η φυσική κίνηση. Καμία κύρια αγορά πριν το drive test.
- **Εξαίρεση:** φθηνά δείγματα αποκλειστικά για μέτρηση.
- **Απορρίφθηκε:** αγορά τελικών controllers/actuators/rotating hardware από
  simulation-only evidence.
- **Evidence:** shared-flywheel physical-gate sequence και user decision.

## 2026-10-18 — selector rails Y±55 και R75/75 printable ski

- **Πλαίσιο:** rails Y±52 άφηναν μόνο 0,5 mm και τα πρώτα ski bosses/shells
  συγκρούονταν με rails ή D200 sweep.
- **Απόφαση:** rails Y±55 και ski με ακριβώς το frozen width profile· η
  τελική στήριξη συμπληρώθηκε στο P2-85.
- **Γιατί:** nominal rail-to-ski gap 3,5 mm και exact-solid PASS στο sweep.
- **Απορρίφθηκε:** X327 boss, constant-width shell και broken P2-73 bracket.
- **Evidence:** P2-73/P2-74/P2-85 reports.

## 2026-10-19 — modular v3 / P2-78 sockets

- **Πλαίσιο:** τα T-clamps ήταν εύθραυστα και το P2-77 centre node δεν
  αποτελούσε πρακτικό τελικό torsion deck.
- **Απόφαση:** monolithic terminal sockets, 60 mm sleeve engagement και δύο
  short side braces X240→X300.
- **Γιατί:** 13 exact-solid gates PASS και καλύτερο launch/racking screen από
  το P2-77 single-diagonal baseline.
- **Απορρίφθηκε:** T-clamps και P2-77 centre node.
- **Evidence:** debug P2-78.

## 2026-10-19 — rear pod v1 → v2 upright plate

- **Πλαίσιο:** η πραγματική θέση UGREEN και τα boards δεν χωρούσαν σωστά στο
  v1 snap-stack.
- **Απόφαση:** upright electronics plate 248×146, Pi shelf, side frames και
  panel posts.
- **Γιατί:** κρατά το P2-70 envelope και δίνει serviceable 2×2 BTS layout,
  rear connectors και ξεχωριστό cable corridor.
- **Απορρίφθηκε:** stacked shelves/snap pillars.
- **Evidence:** debug P2-75/P2-76 και μεταγενέστερα P2-79…P2-88.

## 2026-10-19 / 2026-10-20 / 2026-10-22 — χωρίς νέες οπές σασί και μόνιμα QA gates

- **Πλαίσιο:** πραγματικές αστοχίες αποκάλυψαν floating ski bosses, τριμερές
  corner tie, λάθος chassis axes στο corner stay και μη συναρμολογήσιμες
  side-frame M4.
- **Απόφαση:** καμία νέα οπή στο ήδη τυπωμένο σασί. Κάθε active interface
  περνά hole-axis EMPTY probe, STL component-count gate και hardware/tool
  access gate με ≥60 mm driver path.
- **Γιατί:** τα απλά visual/CAD envelope checks δεν εντόπισαν disconnected
  solids, misaligned existing holes ή κλειστές διαδρομές εργαλείων.
- **Απορρίφθηκε:** νέες chassis holes και screw-axis-only QA.
- **Evidence:** P2-83, P2-86b και P2-88.

## 2026-10-20 — body concept F

- **Πλαίσιο:** συγκρίθηκαν A–F και το παλιό v3 shell.
- **Απόφαση:** το F είναι το μόνο ενεργό body concept. Rear panels P2-86b
  είναι release-candidate αλλά δεν τυπώνονται ακόμη.
- **Γιατί:** παρέχει το επιλεγμένο planter form, Parkside doors, service
  window και perimeter interface. Πλήρες comparative scoring A–F:
  **ΤΕΚΜΗΡΙΩΣΗ ΛΕΙΠΕΙ**.
- **Απορρίφθηκε:** A–E και v3, τα οποία αρχειοθετήθηκαν.

## 2026-10-22 — M3 heat-set inserts

- **Πλαίσιο:** τα printed P2-86b bases και ο περιορισμένος nut/tool access
  απαιτούν serviceable μέτριου φορτίου connections.
- **Απόφαση χρήστη:** Ruthex RX-M3x5.7 επιτρέπονται για covers, holders και
  docks. Parametric pilot Ø4,0, depth ≥6,7 mm και wall ≥2 mm.
- **Όριο:** όχι M3 inserts σε κύρια φέροντα σημεία και κανένα M4 insert· οι
  κύριες M4 είναι through-bolts με washer/nyloc.
- **Απορρίφθηκε:** printed threads/self-tap στα φέροντα interfaces.
- **Evidence:** P2-88 και συσκευασία Ruthex του χρήστη.

## 2026-10-22 — AliExpress pneumatic flywheel wheels

- **Πλαίσιο:** χρειαζόταν φθηνό physical sample πριν κλειδώσει wheel/hub CAD.
- **Απόφαση:** παραγγέλθηκαν δύο nominal 200×50 pneumatic wheels της ίδιας
  παρτίδας.
- **Γιατί:** measurement sample μέσα στην εξαίρεση του purchase freeze.
- **Seller data:** Ø192, width 48, mass 760 g· όχι μετρήσεις.
- **Ανοιχτή απόφαση:** Ø192 οδηγεί σε ρυθμιζόμενη απόσταση αξόνων D5065, αλλά
  η τελική διαδρομή/θέση μένει **ΑΝΟΙΧΤΗ μέχρι μέτρηση**.
- **Evidence:** `ordered-parts.md` A1.

## 2026-10-22 — actuator κλίσης και κατανομή BTS7960

- **Πλαίσιο:** τα P2-53/P2-54 έκαναν baseline το υπάρχον FIT0186+Tr8×2 και
  άφησαν το P16 ως compact direct fallback. Η Rev-C εξακολουθεί να είναι
  καλωδιωμένη/προγραμματισμένη για δύο drive και δύο intake κανάλια.
- **Απόφαση χρήστη:** BTS #1/#2 παραμένουν drive left/right για τα ζεύγη
  μοτέρ κάθε πλευράς. Το BTS #3 (`D44/D45`, EN `D40`) γίνεται tilt
  FIT0186+Tr8×2. Το BTS #4 (`D46/D11`, EN `D42`) γίνεται ρεζέρβα.
- **Κατάσταση:** **ΑΠΟΦΑΣΙΣΜΕΝΟ, ΥΛΟΠΟΙΗΣΗ ΕΚΚΡΕΜΕΙ**. Δεν αλλάζει ακόμη
  firmware, Rev-C netlist ή harness· απαιτείται νέα ρητή έγκριση μετά το
  drive test.
- **Απαιτήσεις υλοποίησης:** encoder σε Mega pin-change pins, δύο endpoint
  switches `−3°/+20°` με homing, direction-specific hardware interruption
  του αντίστοιχου R_EN/L_EN, stall detection από R_IS/L_IS, αρχικό PWM cap
  περίπου 60% στα 18–20 V Parkside και μηχανικά emergency stops
  `−12°/+22°`.
- **Pin audit:** `A8…A15` είναι `PCINT16…23`. Τα A10/A11 και A8/A9
  απελευθερώνονται από τα παλιά intake encoders· A10/A11 είναι ήδη στο
  `J_ENC_IL`. Τα A12…A15 είναι firmware-free αλλά όχι στο σημερινό 34-pin
  harness. Η Rev-C γεφυρώνει τα R_EN/L_EN και δεν δρομολογεί IS, επομένως
  το hardware-limit/current-sense κύκλωμα χρειάζεται μελλοντική netlist.
- **Μηχανικό load path:** preload στα συμμετρικά +20° stops και launch
  reaction `yoke→stops→arch`, όχι μέσω actuator/Tr8. Manual transport pin
  μόνο χωρίς τροφοδοσία.
- **Evidence:** debug P2-11/P2-12/P2-53/P2-54, Rev-C wiring και
  `motion_intake_mega.ino` pin audit.
