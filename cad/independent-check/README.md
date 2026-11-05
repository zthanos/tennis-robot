# Full-scene exact-solid print gate — P2-95

Υποχρεωτικό gate είναι πλέον το **αμετάβλητο `scene_check.py`** μαζί με το
`scadtree.py`. Ανακαλύπτει τα σώματα του
`body-panels/current-design-p293.scad:current_robot()` στις −3/0/+20° χωρίς
χειροκίνητο parts manifest. Κάθε νέο/αλλαγμένο STL τοποθετείται στην ίδια τη
σκηνή, μία φορά. Το `collide.py`/`parts.py` διατηρείται για διάγνωση μόνο.

Από τη ρίζα του repo:

```bash
OPENSCAD_BIN=/snap/bin/openscad-nightly bash cad/check-current-scene.sh
```

Ο wrapper κρατά την προηγούμενη cache σε `out/previous-scene.*/scene` και
τρέχει το εργαλείο με φρέσκα exports. Αυτό απαιτείται επειδή το αμετάβλητο
εργαλείο δεν συμπεριλαμβάνει source content/mtime στο cache key. Καλείται
υποχρεωτικά από το `check-active-assemblability.sh` πριν τα επιμέρους gates.
Exit 1, export error, μη αναγνωρισμένα/λανθασμένα σώματα ή εργαλειακό σφάλμα
μπλοκάρουν release. Το εργαλείο εκτυπώνει επίσης `contained` και absent
modules: το ότι δεν τα περιλαμβάνει όλα στο exit status δεν τα καθιστά PASS.

Το αυτούσιο output, οι κατηγορίες και οι περιορισμοί του τρέχοντος run
βρίσκονται στο [P2-95 report](P2-95-report.md).

## Ιστορικό P2-94 — αντικαταστάθηκε από το scene gate

Στο P2-94 το `collide.py` ήταν υποχρεωτικό gate πριν από κάθε νέο ή αλλαγμένο
robot-installed STL. Το script και το `parts.py` ελέγχουν τα raw solids στις
πραγματικές θέσεις συναρμολόγησης. Δεν επιτρέπεται αφαίρεση mating zone,
fastener zone, contact box ή άλλου «declared interface» πριν από την τομή.
Επιφανειακή επαφή σημαίνει μηδενικό όγκο.

## Release command

Από αυτόν τον κατάλογο:

```bash
OPENSCAD_BIN=/snap/bin/openscad-nightly \
  uv run --with manifold3d --with trimesh --with numpy \
  python collide.py --min-vol 0.0001
```

Το `--min-vol 0.0001` είναι το P2-94 numerical floor. Καταγράφει τους
τέσσερις μικρούς θετικούς όγκους `0,00020…0,00030 mm³`, αλλά όχι τέσσερις
τιμές `1,36e-15…7,05e-13 mm³` που προκύπτουν από tessellation σε
συνεπίπεδη επαφή. Το default `0.5` εμφανίζει μόνο 29 από τις 33 τρέχουσες
συγκρούσεις και δεν χρησιμοποιείται για release.

Επιτυχία release είναι μόνο:

```text
0 collisions > 0.0001 mm3 (0 export failures)
```

## Workflow νέου ή αλλαγμένου STL

1. Το STL μπαίνει στο `parts.py` με το πραγματικό installed transform και
   το πραγματικό solid που θα τυπωθεί. Surrogate επιτρέπεται μόνο ως
   προσωρινό, ονομασμένο σώμα και δεν μπορεί να στηρίξει κατάσταση RELEASED.
2. Τρέχει το πλήρες gate και, όπου υπάρχει κινούμενο μέρος, το εφαρμοστέο
   set θέσεων/γωνιών. Η χρήση `--only` επιτρέπεται για διάγνωση, όχι ως
   υποκατάστατο του πλήρους gate.
3. Κάθε θετικός όγκος είναι blocking. Μια σκόπιμη επικάλυψη καταγράφεται με
   αιτιολόγηση, αλλά δεν κρύβεται και δεν μετατρέπεται σε PASS.
4. Τυπώνεται interface coupon. Για part άνω των 4 ωρών, το coupon
   δημιουργείται πρώτα με Cut στο Bambu Studio. Όταν δεν ελέγχεται αντοχή,
   χρησιμοποιούνται 2 τοιχώματα και 10% infill.
5. Το `../ACTIVE-PRINT-SET.md` αλλάζει σε RELEASED μόνο μετά από raw 0 και
   καταγεγραμμένο physical coupon/assembly PASS.

Coupons και bench fixtures δεν είναι robot-installed bodies. Ελέγχονται στο
δικό τους fixture context και παραμένουν FIT-CHECK· δεν προστίθενται σε
τυχαία θέση του robot μόνο για να ικανοποιηθεί τυπικά το `parts.py`.

## P2-94 baseline — 33 blocking intersections

Η κατηγορία περιγράφει την προέλευση, όχι εξαίρεση:

- **πραγματική**: δύο σημερινά printable/candidate solids καταλαμβάνουν τον
  ίδιο χώρο και η διεπαφή χρειάζεται διόρθωση,
- **σκόπιμη**: lap/land/preload που μοντελοποιήθηκε επίτηδες με θετικό όγκο·
  παραμένει blocking μέχρι να γίνει πραγματική έδραση/κενό μηδενικού όγκου,
- **surrogate**: τουλάχιστον ένα σώμα είναι concept/frozen envelope και η
  σύγκρουση μπλοκάρει την προαγωγή του surrogate, όχι ένα ήδη φυσικά
  επαληθευμένο part έξω από το εφαρμοστέο release set.

| # | Ζεύγος | Όγκος mm³ | Κατηγορία | Αιτιολόγηση |
|---:|---|---:|---|---|
| 1 | `frear_rear_lo_L × frear_corner_stay_L` | 12.247,7 | σκόπιμη | Δηλωμένο corner-stay load-path land, αλλά μοντελοποιήθηκε ως co-volume |
| 2 | `frear_rear_lo_R × frear_corner_stay_R` | 12.247,7 | σκόπιμη | Κατοπτρικό corner-stay load-path land, αλλά μοντελοποιήθηκε ως co-volume |
| 3 | `frear_rear_hi_L × frear_deck_L` | 8.620,9 | σκόπιμη | Δηλωμένο deck/rear-hi structural lap με θετικό όγκο |
| 4 | `frear_rear_hi_R × frear_deck_R` | 8.596,7 | σκόπιμη | Δηλωμένο deck/rear-hi structural lap με θετικό όγκο |
| 5 | `pod_parkside_dock_L × pod_parkside_drawer_L` | 3.282,9 | πραγματική | Printable dock/drawer rails αλληλοδιεισδύουν |
| 6 | `pod_parkside_dock_R × pod_parkside_drawer_R` | 3.282,9 | πραγματική | Printable dock/drawer rails αλληλοδιεισδύουν |
| 7 | `ffront_concept × arch` | 1.760,6 | surrogate | Το front-F είναι concept envelope, όχι released STL |
| 8 | `tilt_magnet_cap × launcher_m3` | 1.601,4 | surrogate | Frozen launcher envelope δεν έχει κλειστή magnet-cap διεπαφή |
| 9 | `tilt_magnet_cap × launcher_0` | 1.601,4 | surrogate | Frozen launcher envelope δεν έχει κλειστή magnet-cap διεπαφή |
| 10 | `tilt_magnet_cap × launcher_p20` | 1.601,4 | surrogate | Frozen launcher envelope δεν έχει κλειστή magnet-cap διεπαφή |
| 11 | `pod_side_frame_L × basket` | 1.566,4 | surrogate | Το basket solid περιέχει superseded tail/envelope |
| 12 | `pod_side_frame_R × basket` | 1.566,4 | surrogate | Το basket solid περιέχει superseded tail/envelope |
| 13 | `ski × launcher_p20` | 1.452,7 | surrogate | Frozen launcher envelope έναντι released ski candidate |
| 14 | `pod_base_L × frear_rear_lo_L` | 1.215,9 | πραγματική | Base και printable rear-F skin καταλαμβάνουν ίδιο χώρο |
| 15 | `pod_base_R × frear_rear_lo_R` | 1.215,9 | πραγματική | Base και printable rear-F skin καταλαμβάνουν ίδιο χώρο |
| 16 | `ski × launcher_0` | 1.199,1 | surrogate | Frozen launcher envelope έναντι released ski candidate |
| 17 | `ski × launcher_m3` | 1.175,7 | surrogate | Frozen launcher envelope έναντι released ski candidate |
| 18 | `pod_control_panel × frear_deck_L` | 1.054,1 | πραγματική | Τυπωμένο control panel και printable deck αλληλοδιεισδύουν |
| 19 | `pod_control_panel × frear_deck_R` | 1.054,1 | πραγματική | Τυπωμένο control panel και printable deck αλληλοδιεισδύουν |
| 20 | `frear_rear_lo_L × frear_door_L` | 503,4 | πραγματική | Printable door/skin hinge region αλληλοδιεισδύει |
| 21 | `frear_rear_lo_R × frear_door_R` | 503,4 | πραγματική | Printable door/skin hinge region αλληλοδιεισδύει |
| 22 | `pod_ugreen_holder × pod_ugreen_retainer` | 172,6 | σκόπιμη | Δηλωμένη holder/retainer σύνδεση μοντελοποιήθηκε ως co-volume |
| 23 | `frear_corner_stay_L × ffront_concept` | 123,0 | surrogate | Το front-F είναι concept envelope |
| 24 | `frear_corner_stay_R × ffront_concept` | 123,0 | surrogate | Το front-F είναι concept envelope |
| 25 | `pod_control_panel × frear_rear_hi_L` | 110,9 | πραγματική | Τυπωμένο control panel και printable rear-hi αλληλοδιεισδύουν |
| 26 | `pod_control_panel × frear_rear_hi_R` | 110,9 | πραγματική | Τυπωμένο control panel και printable rear-hi αλληλοδιεισδύουν |
| 27 | `pod_base_L × pod_wedge_L` | 12,5 | σκόπιμη | Προφορτισμένη wedge/foot έδραση μοντελοποιήθηκε με θετικό όγκο |
| 28 | `pod_base_R × pod_wedge_R` | 12,5 | σκόπιμη | Προφορτισμένη wedge/foot έδραση μοντελοποιήθηκε με θετικό όγκο |
| 29 | `frear_rear_hi_R × frear_deck_L` | 1,2 | πραγματική | Cross-seam printable halves τέμνονται εκτός της δηλωμένης ίδιας-πλευράς έδρασης |
| 30 | `frear_rear_lo_L × frear_splice_lo` | 0,000300 | σκόπιμη | Center lap/contact έχει μικρό αλλά θετικό tessellated volume |
| 31 | `frear_rear_lo_R × frear_splice_lo` | 0,000207 | σκόπιμη | Center lap/contact έχει μικρό αλλά θετικό tessellated volume |
| 32 | `frear_rear_hi_L × frear_splice_hi` | 0,000296 | σκόπιμη | Center lap/contact έχει μικρό αλλά θετικό tessellated volume |
| 33 | `frear_rear_hi_R × frear_splice_hi` | 0,000202 | σκόπιμη | Center lap/contact έχει μικρό αλλά θετικό tessellated volume |

Σύνολο: **11 πραγματικές, 11 σκόπιμες, 11 surrogate**. Και οι 33 είναι
blocking για το part/set που τις εισάγει. Δεν υπάρχει whitelist ή αφαίρεση
όγκου.
