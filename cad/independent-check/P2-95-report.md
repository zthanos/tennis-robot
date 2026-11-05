# P2-95 — πλήρης σκηνή CAD και υποχρεωτικό gate

Ημερομηνία: 2026-11-05. Σκηνή: `body-panels/current-design-p293.scad:current_robot()`.
Τελικό run από καθαρή cache, γωνίες −3/0/+20°. **FAIL: 115 collisions,
3 contained pairs, 106 bodies ανά γωνία, 0 duplicates, 0 export failures.**
Καμία printable γεωμετρία ή STL δεν άλλαξε.

## Σύνθεση και installed poses

- Προστέθηκαν docks/drawers Parkside L/R, UGREEN holder/caps/retainer και
  rear-foot wedges με τα υπάρχοντα installed modules τους.
- MPU: `translate([112.5,195,37]) rotate([90,0,0])`, ο υπάρχων P2-77
  installed transform.
- AS5600 cover: ίδιος άξονας με το pod, origin `[387,254,55.84]`.
  Το pod καλύπτει Y244…254 και το cover Y254…258, με μηδενικού όγκου έδραση.
- FIT0186: η μήτρα του carrier και η τοπική οπή `[30,0]` καθορίζουν τον
  πραγματικό άξονα, στη −Y πλευρά. Κέντρο εξόδου περίπου
  `[291.2,-137.4,162.21]`, διεύθυνση σώματος `[0.98,0,0.20]`.
  Το Ø37×70 motor can συμπληρώνεται με Ø37×17 provisional gearbox
  στο station της οπής. **Το μήκος 17 mm είναι envelope παραδοχή από το
  carrier ring, όχι μετρημένη διάσταση hardware**· δεν στηρίζει release.
- Το P2-71 basket κλείνει στο X225 με ονομασμένο assembly
  `p271_basket_current()`. Το παλιό X278 tail αφαιρείται από τον ορισμό
  της σκηνής, όχι από QA intersection. Το front system σχεδιάζεται μία φορά.
- Η `assembly_scene(service,context)` έχει πλέον ρητό context argument.
  Η πλήρης σκηνή καλεί `assembly_scene(service_open,false)`: pod, chassis
  και drive δεν ξανασχεδιάζονται από τα rear panels. Το default του
  standalone rear-F preview διατηρείται.

## Cable bulkhead v1

**Ναι, παραμένει ενεργό**: το `rear_pod_v2_assembly()` εισάγει
`stl-v2/cable_bulkhead.stl` και τα δύο `bulkhead_foot.stl`, ο
`export-v2-stls.sh` τα αντιγράφει από v1, και υπάρχουν στο
`ACTIVE-PRINT-SET.md`. Δεν αφαιρέθηκαν για να καθαρίσει ο έλεγχος.
Το bulkhead τέμνει side frame +Y κατά 5.438,3 mm³, side frame −Y κατά
4.714,2 mm³ και κάθε βάση κατά 5.096,0 mm³.
Το bulkhead και τα feet σημειώθηκαν **CONCEPT / blocking**.

## Gate και περιορισμοί εργαλείου

```bash
OPENSCAD_BIN=/snap/bin/openscad-nightly bash cad/check-current-scene.sh
```

Το wrapper τρέχει τα αμετάβλητα εργαλεία και φυλάσσει την παλιά cache πριν
από fresh export. Καλείται στην αρχή του active assemblability suite.
Το `collide.py` είναι πλέον διαγνωστικό, όχι release gate.

Αναφέρθηκαν/διατηρήθηκαν οι εξής περιορισμοί, χωρίς αλλαγή στα εργαλεία:

1. Το πρώτο run με ανώνυμο basket intersection σταμάτησε με
   `ValueError: No points given` στο convex hull του ball-space.
   Ονομασμένο basket assembly στη σκηνή επιτρέπει την ανακάλυψη και το
   τελικό run ολοκληρώθηκε.
2. Το cache key δεν εξαρτάται από source content/mtime. Ο wrapper διατηρεί
   την παλιά cache και εξαναγκάζει φρέσκα exports.
3. Υποσύνολα χωρίς αναγνωρισμένο STL dispatcher παραμένουν ομαδοποιημένα:
   `moving_module_concept`, `drive_package_envelopes`, `panel_post_installed`,
   front-system και ghosts. Basket και front system μπήκαν σε ξεχωριστές
   κλήσεις, ώστε το ball-space να προκύπτει μόνο από το basket.
   Τα exported basket bounds είναι ακριβώς
   `[-120,-173.5,44.0822]…[225,173.5,320]` και στις τρεις γωνίες.
   Δεν ισχυριζόμαστε ότι ελέγχθηκαν εσωτερικά όλες οι διεπαφές των άλλων ομάδων.
4. Το exit status αγνοεί `contained` και absent-module λίστα. Και τα τρία
   contained ζεύγη καταγράφονται παρακάτω ως blocking surrogate, όχι PASS.
5. Τα 33 absent modules αφορούν όλο το μη αρχειοθετημένο CAD tree:
   περιλαμβάνουν παλιά μη ενεργά μοντέλα, test coupons, print-plate wrappers
   και functions (`leg_split`, `seg_len`) που ο dispatcher ανιχνεύει ως
   parts. Τα caps/posts υπάρχουν installed ως επιμέρους bodies παρότι
   τα print-set wrappers αναφέρονται absent. `bts_spacer_plate`,
   `component_pin_plate`, `component_clip_plate` δεν έχουν τοποθετηθεί:
   λείπει τεκμηριωμένο installed placement από τη σημερινή σκηνή· είναι
   πρόσθετη εκκρεμότητα κάλυψης, όχι μηδενική σύγκρουση.
6. Το output στρογγυλεύει όγκους σε ένα δεκαδικό. Γραμμές `0.0 mm3`
   παρακάτω έχουν θετικό όγκο πάνω από το threshold· ο πίνακας δίνει
   τέσσερα δεκαδικά όπως το report.json.
7. Το FIT0186 gearbox είναι provisional· οι προτεινόμενες hardware
   διαστάσεις/συναρμολόγηση χρειάζονται φυσική επιβεβαίωση.

SHA-256 αμετάβλητων αρχείων χρήστη:

```text
d1c81ee21f93c441982becdacbffa140e2e654c86683f0ee958f65342998c4df  scene_check.py
ae7b86ce78723bddd7c708b433bbbbd28c4533c1be186c23c1d48d75d683630f  scadtree.py
```

## Πίνακας συγκρούσεων ανά κατηγορία

Οι κατηγορίες περιγράφουν την αιτία, δεν εξαιρούν όγκο. Το «σκόπιμη»
αφορά δηλωμένη μηχανική διεπαφή και όχι πιστοποίηση ότι η διείσδυση είναι
αποδεκτή. Οι τομές μεταξύ διακριτών arch parts καταγράφονται πραγματικές.
Το «surrogate» περιλαμβάνει και ball-space/ghost/συγκεντρωτικά envelopes,
με αβεβαιότητα για το επιμέρους hardware. **Δεν δίνεται release PASS.**

Σύνολο με τα contained: 45 πραγματικές,
28 σκόπιμες, 45 surrogate.

### πραγματική

| Ζεύγος | Max mm³ | Γωνίες | Είδος εργαλείου | Αιτιολόγηση |
|---|---:|---|---|---|
| `arch_brace[side=1]#2` × `arch_leg_upper[side=1]` | 10865.9058 | -3/0/20 | collision | Διακριτά printable μέλη της αψίδας αλληλοδιεισδύουν· καμία ενιαία union δεν δικαιολογεί την τομή. |
| `arch_brace[side=-1]#2` × `arch_leg_upper[side=-1]` | 10865.9055 | -3/0/20 | collision | Διακριτά printable μέλη της αψίδας αλληλοδιεισδύουν· καμία ενιαία union δεν δικαιολογεί την τομή. |
| `arch_brace[side=-1]` × `arch_brace[side=-1]#2` | 10381.6871 | -3/0/20 | collision | Διακριτά printable μέλη της αψίδας αλληλοδιεισδύουν· καμία ενιαία union δεν δικαιολογεί την τομή. |
| `arch_brace[side=1]` × `arch_brace[side=1]#2` | 10381.6871 | -3/0/20 | collision | Διακριτά printable μέλη της αψίδας αλληλοδιεισδύουν· καμία ενιαία union δεν δικαιολογεί την τομή. |
| `arch_brace[side=-1]#2` × `arch_leg_lower[side=-1]` | 5833.2173 | -3/0/20 | collision | Διακριτά printable μέλη της αψίδας αλληλοδιεισδύουν· καμία ενιαία union δεν δικαιολογεί την τομή. |
| `arch_brace[side=1]#2` × `arch_leg_lower[side=1]` | 5833.2173 | -3/0/20 | collision | Διακριτά printable μέλη της αψίδας αλληλοδιεισδύουν· καμία ενιαία union δεν δικαιολογεί την τομή. |
| `arch_brace[side=-1]` × `arch_leg_upper[side=-1]` | 5614.9973 | -3/0/20 | collision | Διακριτά printable μέλη της αψίδας αλληλοδιεισδύουν· καμία ενιαία union δεν δικαιολογεί την τομή. |
| `arch_brace[side=1]` × `arch_leg_upper[side=1]` | 5614.9970 | -3/0/20 | collision | Διακριτά printable μέλη της αψίδας αλληλοδιεισδύουν· καμία ενιαία union δεν δικαιολογεί την τομή. |
| `cable_bulkhead` × `side_frame[side=1]` | 5438.2891 | -3/0/20 | collision | Διακριτά printable solids/εγκατεστημένες διεπαφές καταλαμβάνουν τον ίδιο χώρο. |
| `base_half_v2` × `cable_bulkhead` | 5096.0013 | -3/0/20 | collision | Διακριτά printable solids/εγκατεστημένες διεπαφές καταλαμβάνουν τον ίδιο χώρο. |
| `base_half_v2#2` × `cable_bulkhead` | 5096.0013 | -3/0/20 | collision | Διακριτά printable solids/εγκατεστημένες διεπαφές καταλαμβάνουν τον ίδιο χώρο. |
| `cable_bulkhead` × `side_frame[side=-1]` | 4714.1825 | -3/0/20 | collision | Διακριτά printable solids/εγκατεστημένες διεπαφές καταλαμβάνουν τον ίδιο χώρο. |
| `parkside_dock[side=-1]` × `parkside_drawer[side=-1]` | 3298.1026 | -3/0/20 | collision | Διακριτά printable solids/εγκατεστημένες διεπαφές καταλαμβάνουν τον ίδιο χώρο. |
| `parkside_dock[side=1]` × `parkside_drawer[side=1]` | 3298.1026 | -3/0/20 | collision | Διακριτά printable solids/εγκατεστημένες διεπαφές καταλαμβάνουν τον ίδιο χώρο. |
| `arch_crossbar_end[side=-1]` × `lidar_mast[side=-1]` | 2134.6621 | -3/0/20 | collision | Διακριτά printable μέλη της αψίδας αλληλοδιεισδύουν· καμία ενιαία union δεν δικαιολογεί την τομή. |
| `arch_crossbar_end[side=1]` × `lidar_mast[side=1]` | 2134.6621 | -3/0/20 | collision | Διακριτά printable μέλη της αψίδας αλληλοδιεισδύουν· καμία ενιαία union δεν δικαιολογεί την τομή. |
| `lidar_crossbar_half[side=-1]` × `lidar_mast[side=-1]` | 2034.1874 | -3/0/20 | collision | Διακριτά printable μέλη της αψίδας αλληλοδιεισδύουν· καμία ενιαία union δεν δικαιολογεί την τομή. |
| `lidar_crossbar_half[side=1]` × `lidar_mast[side=1]` | 2034.1874 | -3/0/20 | collision | Διακριτά printable μέλη της αψίδας αλληλοδιεισδύουν· καμία ενιαία union δεν δικαιολογεί την τομή. |
| `base_half_v2` × `rear_lo[s=-1]` | 1231.3376 | -3/0/20 | collision | Διακριτά printable solids/εγκατεστημένες διεπαφές καταλαμβάνουν τον ίδιο χώρο. |
| `base_half_v2#2` × `rear_lo[s=1]` | 1231.3376 | -3/0/20 | collision | Διακριτά printable solids/εγκατεστημένες διεπαφές καταλαμβάνουν τον ίδιο χώρο. |
| `arch_brace[side=-1]#2` × `joint_sleeve_x[side=-1]` | 1151.9122 | -3/0/20 | collision | Διακριτά printable μέλη της αψίδας αλληλοδιεισδύουν· καμία ενιαία union δεν δικαιολογεί την τομή. |
| `arch_brace[side=1]#2` × `joint_sleeve_x[side=1]` | 1151.9122 | -3/0/20 | collision | Διακριτά printable μέλη της αψίδας αλληλοδιεισδύουν· καμία ενιαία union δεν δικαιολογεί την τομή. |
| `control_panel_v2` × `deck_rear[s=-1]` | 1054.3587 | -3/0/20 | collision | Διακριτά printable solids/εγκατεστημένες διεπαφές καταλαμβάνουν τον ίδιο χώρο. |
| `control_panel_v2` × `deck_rear[s=1]` | 1054.3587 | -3/0/20 | collision | Διακριτά printable solids/εγκατεστημένες διεπαφές καταλαμβάνουν τον ίδιο χώρο. |
| `arch_brace[side=-1]#2` × `arch_crossbar_end[side=-1]` | 971.5463 | -3/0/20 | collision | Διακριτά printable μέλη της αψίδας αλληλοδιεισδύουν· καμία ενιαία union δεν δικαιολογεί την τομή. |
| `arch_brace[side=1]#2` × `arch_crossbar_end[side=1]` | 971.5461 | -3/0/20 | collision | Διακριτά printable μέλη της αψίδας αλληλοδιεισδύουν· καμία ενιαία union δεν δικαιολογεί την τομή. |
| `arch_brace[side=1]` × `arch_crossbar_end[side=1]` | 903.9128 | -3/0/20 | collision | Διακριτά printable μέλη της αψίδας αλληλοδιεισδύουν· καμία ενιαία union δεν δικαιολογεί την τομή. |
| `arch_brace[side=-1]` × `arch_crossbar_end[side=-1]` | 903.9126 | -3/0/20 | collision | Διακριτά printable μέλη της αψίδας αλληλοδιεισδύουν· καμία ενιαία union δεν δικαιολογεί την τομή. |
| `arch_crossbar_end[side=-1]` × `arch_leg_upper[side=-1]` | 844.7510 | -3/0/20 | collision | Διακριτά printable μέλη της αψίδας αλληλοδιεισδύουν· καμία ενιαία union δεν δικαιολογεί την τομή. |
| `arch_crossbar_end[side=1]` × `arch_leg_upper[side=1]` | 844.7510 | -3/0/20 | collision | Διακριτά printable μέλη της αψίδας αλληλοδιεισδύουν· καμία ενιαία union δεν δικαιολογεί την τομή. |
| `arch_brace[side=-1]` × `arch_foot_x240[side=-1]` | 761.0413 | -3/0/20 | collision | Διακριτά printable μέλη της αψίδας αλληλοδιεισδύουν· καμία ενιαία union δεν δικαιολογεί την τομή. |
| `arch_brace[side=1]` × `arch_foot_x240[side=1]` | 761.0413 | -3/0/20 | collision | Διακριτά printable μέλη της αψίδας αλληλοδιεισδύουν· καμία ενιαία union δεν δικαιολογεί την τομή. |
| `parkside_door[s=1]` × `rear_lo[s=1]` | 503.4318 | -3/0/20 | collision | Διακριτά printable solids/εγκατεστημένες διεπαφές καταλαμβάνουν τον ίδιο χώρο. |
| `parkside_door[s=-1]` × `rear_lo[s=-1]` | 503.4314 | -3/0/20 | collision | Διακριτά printable solids/εγκατεστημένες διεπαφές καταλαμβάνουν τον ίδιο χώρο. |
| `lidar_crossbar_half[side=1]` × `oak_universal_plate` | 364.6862 | -3/0/20 | collision | Διακριτά printable solids/εγκατεστημένες διεπαφές καταλαμβάνουν τον ίδιο χώρο. |
| `lidar_crossbar_half[side=-1]` × `oak_universal_plate` | 364.6861 | -3/0/20 | collision | Διακριτά printable solids/εγκατεστημένες διεπαφές καταλαμβάνουν τον ίδιο χώρο. |
| `panel_post_installed` × `side_frame[side=-1]` | 317.1197 | -3/0/20 | collision | Διακριτά printable solids/εγκατεστημένες διεπαφές καταλαμβάνουν τον ίδιο χώρο. |
| `panel_post_installed` × `side_frame[side=1]` | 233.1726 | -3/0/20 | collision | Διακριτά printable solids/εγκατεστημένες διεπαφές καταλαμβάνουν τον ίδιο χώρο. |
| `arch_foot_x300[side=-1]` × `arch_leg_lower[side=-1]` | 211.4395 | -3/0/20 | collision | Διακριτά printable μέλη της αψίδας αλληλοδιεισδύουν· καμία ενιαία union δεν δικαιολογεί την τομή. |
| `arch_foot_x300[side=1]` × `arch_leg_lower[side=1]` | 211.4395 | -3/0/20 | collision | Διακριτά printable μέλη της αψίδας αλληλοδιεισδύουν· καμία ενιαία union δεν δικαιολογεί την τομή. |
| `control_panel_v2` × `rear_hi[s=-1]` | 112.4915 | -3/0/20 | collision | Διακριτά printable solids/εγκατεστημένες διεπαφές καταλαμβάνουν τον ίδιο χώρο. |
| `control_panel_v2` × `rear_hi[s=1]` | 112.4915 | -3/0/20 | collision | Διακριτά printable solids/εγκατεστημένες διεπαφές καταλαμβάνουν τον ίδιο χώρο. |
| `arch_brace[side=-1]#2` × `arch_foot_x300[side=-1]` | 106.9271 | -3/0/20 | collision | Διακριτά printable μέλη της αψίδας αλληλοδιεισδύουν· καμία ενιαία union δεν δικαιολογεί την τομή. |
| `arch_brace[side=1]#2` × `arch_foot_x300[side=1]` | 106.9271 | -3/0/20 | collision | Διακριτά printable μέλη της αψίδας αλληλοδιεισδύουν· καμία ενιαία union δεν δικαιολογεί την τομή. |
| `deck_rear[s=1]` × `rear_hi[s=-1]` | 1.2100 | -3/0/20 | collision | Διακριτά printable solids/εγκατεστημένες διεπαφές καταλαμβάνουν τον ίδιο χώρο. |

### σκόπιμη

| Ζεύγος | Max mm³ | Γωνίες | Είδος εργαλείου | Αιτιολόγηση |
|---|---:|---|---|---|
| `corner_stay[s=-1]` × `rear_lo[s=-1]` | 12247.7240 | -3/0/20 | collision | Δηλωμένο structural land stay/skin· απαιτεί πραγματική έδραση. |
| `corner_stay[s=1]` × `rear_lo[s=1]` | 12247.7240 | -3/0/20 | collision | Δηλωμένο structural land stay/skin· απαιτεί πραγματική έδραση. |
| `deck_rear[s=1]` × `rear_hi[s=1]` | 8620.8933 | -3/0/20 | collision | Δηλωμένο deck/rear-hi structural lap στην ίδια πλευρά· blocking. |
| `deck_rear[s=-1]` × `rear_hi[s=-1]` | 8596.6572 | -3/0/20 | collision | Δηλωμένο deck/rear-hi structural lap στην ίδια πλευρά· blocking. |
| `door_turn_button[s=-1]` × `rear_lo[s=-1]` | 424.1735 | -3/0/20 | collision | Κλείστρο/έδρα turn-button· καταγράφεται η σχεδιαστική πρόθεση, απαιτεί clearance. |
| `door_turn_button[s=1]` × `rear_lo[s=1]` | 424.1734 | -3/0/20 | collision | Κλείστρο/έδρα turn-button· καταγράφεται η σχεδιαστική πρόθεση, απαιτεί clearance. |
| `interface_fasteners` × `rear_lo[s=-1]` | 256.6535 | -3/0/20 | collision | Μοντελοποιημένο fastener/interface contact· το θετικό αποτέλεσμα διατηρείται, δεν αποτελεί εξαίρεση. |
| `interface_fasteners` × `rear_lo[s=1]` | 256.6535 | -3/0/20 | collision | Μοντελοποιημένο fastener/interface contact· το θετικό αποτέλεσμα διατηρείται, δεν αποτελεί εξαίρεση. |
| `ugreen_post_holder` × `ugreen_rear_retainer` | 172.5920 | -3/0/20 | collision | Δηλωμένη συγκράτηση holder/retainer· η πρόθεση δεν νομιμοποιεί co-volume. |
| `clamp_sleeve_x[side=-1xx=35]` × `rail_extension_v3[side=-1]` | 96.8321 | -3/0/20 | collision | Δηλωμένη σύνδεση μανικιού/δοκού στο ήδη τυπωμένο σασί· θετικός mesh όγκος, όχι νέο PASS. |
| `clamp_sleeve_x[side=1xx=35]` × `rail_extension_v3[side=1]` | 96.8321 | -3/0/20 | collision | Δηλωμένη σύνδεση μανικιού/δοκού στο ήδη τυπωμένο σασί· θετικός mesh όγκος, όχι νέο PASS. |
| `base_half_v2` × `rear_foot_wedge_washer[side=-1]` | 12.5044 | -3/0/20 | collision | Δηλωμένη wedge έδραση στην κάτω επιφάνεια της βάσης· απαιτεί μηδενικό όγκο. |
| `base_half_v2#2` × `rear_foot_wedge_washer[side=1]` | 12.5044 | -3/0/20 | collision | Δηλωμένη wedge έδραση στην κάτω επιφάνεια της βάσης· απαιτεί μηδενικό όγκο. |
| `door_turn_button[s=-1]` × `parkside_door[s=-1]` | 5.5132 | -3/0/20 | collision | Κλείστρο/έδρα turn-button· καταγράφεται η σχεδιαστική πρόθεση, απαιτεί clearance. |
| `door_turn_button[s=1]` × `parkside_door[s=1]` | 5.5132 | -3/0/20 | collision | Κλείστρο/έδρα turn-button· καταγράφεται η σχεδιαστική πρόθεση, απαιτεί clearance. |
| `corner_clamp#2` × `motor_module_v3[side=-1]` | 1.4712 | -3/0/20 | collision | Δηλωμένη σύνδεση μανικιού/δοκού στο ήδη τυπωμένο σασί· θετικός mesh όγκος, όχι νέο PASS. |
| `corner_clamp` × `motor_module_v3[side=1]` | 1.4712 | -3/0/20 | collision | Δηλωμένη σύνδεση μανικιού/δοκού στο ήδη τυπωμένο σασί· θετικός mesh όγκος, όχι νέο PASS. |
| `interface_fasteners` × `rear_hi[s=-1]` | 0.0326 | -3/0/20 | collision | Μοντελοποιημένο fastener/interface contact· το θετικό αποτέλεσμα διατηρείται, δεν αποτελεί εξαίρεση. |
| `interface_fasteners` × `rear_hi[s=1]` | 0.0326 | -3/0/20 | collision | Μοντελοποιημένο fastener/interface contact· το θετικό αποτέλεσμα διατηρείται, δεν αποτελεί εξαίρεση. |
| `control_panel_v2` × `interface_fasteners` | 0.0304 | -3/0/20 | collision | Μοντελοποιημένο fastener/interface contact· το θετικό αποτέλεσμα διατηρείται, δεν αποτελεί εξαίρεση. |
| `deck_rear[s=-1]` × `interface_fasteners` | 0.0217 | -3/0/20 | collision | Μοντελοποιημένο fastener/interface contact· το θετικό αποτέλεσμα διατηρείται, δεν αποτελεί εξαίρεση. |
| `deck_rear[s=1]` × `interface_fasteners` | 0.0217 | -3/0/20 | collision | Μοντελοποιημένο fastener/interface contact· το θετικό αποτέλεσμα διατηρείται, δεν αποτελεί εξαίρεση. |
| `base_half_v2` × `interface_fasteners` | 0.0109 | -3/0/20 | collision | Μοντελοποιημένο fastener/interface contact· το θετικό αποτέλεσμα διατηρείται, δεν αποτελεί εξαίρεση. |
| `base_half_v2#2` × `interface_fasteners` | 0.0109 | -3/0/20 | collision | Μοντελοποιημένο fastener/interface contact· το θετικό αποτέλεσμα διατηρείται, δεν αποτελεί εξαίρεση. |
| `center_splice` × `rear_lo[s=1]` | 0.0003 | -3/0/20 | collision | Δηλωμένη center-lap σύνδεση· μικρός θετικός όγκος, παραμένει blocking. |
| `center_splice#2` × `rear_hi[s=1]` | 0.0003 | -3/0/20 | collision | Δηλωμένη center-lap σύνδεση· μικρός θετικός όγκος, παραμένει blocking. |
| `center_splice` × `rear_lo[s=-1]` | 0.0002 | -3/0/20 | collision | Δηλωμένη center-lap σύνδεση· μικρός θετικός όγκος, παραμένει blocking. |
| `center_splice#2` × `rear_hi[s=-1]` | 0.0002 | -3/0/20 | collision | Δηλωμένη center-lap σύνδεση· μικρός θετικός όγκος, παραμένει blocking. |

### surrogate

| Ζεύγος | Max mm³ | Γωνίες | Είδος εργαλείου | Αιτιολόγηση |
|---|---:|---|---|---|
| `parkside_dock[side=-1]` × `reserved_ghosts` | 53298.9657 | -3/0/20 | collision | Concept/keep-out/συγκεντρωτικό hardware ή basket/feed envelope· απαιτεί έλεγχο της πραγματικής διεπαφής. |
| `parkside_dock[side=1]` × `reserved_ghosts` | 53298.9657 | -3/0/20 | collision | Concept/keep-out/συγκεντρωτικό hardware ή basket/feed envelope· απαιτεί έλεγχο της πραγματικής διεπαφής. |
| `basket_ball_space` × `drive_package_envelopes` | 31928.3366 | -3/0/20 | collision | Concept/keep-out/συγκεντρωτικό hardware ή basket/feed envelope· απαιτεί έλεγχο της πραγματικής διεπαφής. |
| `reserved_ghosts` × `ugreen_post_holder` | 25223.2229 | -3/0/20 | collision | Concept/keep-out/συγκεντρωτικό hardware ή basket/feed envelope· απαιτεί έλεγχο της πραγματικής διεπαφής. |
| `arch_foot_x300[side=-1]` × `drive_package_envelopes` | 17207.2326 | -3/0/20 | collision | Concept/keep-out/συγκεντρωτικό hardware ή basket/feed envelope· απαιτεί έλεγχο της πραγματικής διεπαφής. |
| `arch_foot_x300[side=1]` × `drive_package_envelopes` | 17207.2326 | -3/0/20 | collision | Concept/keep-out/συγκεντρωτικό hardware ή basket/feed envelope· απαιτεί έλεγχο της πραγματικής διεπαφής. |
| `drive_package_envelopes` × `kfl08_base` | 7497.3197 | -3/0/20 | collision | Concept/keep-out/συγκεντρωτικό hardware ή basket/feed envelope· απαιτεί έλεγχο της πραγματικής διεπαφής. |
| `arch_brace[side=-1]#2` × `drive_package_envelopes` | 7435.1969 | -3/0/20 | collision | Concept/keep-out/συγκεντρωτικό hardware ή basket/feed envelope· απαιτεί έλεγχο της πραγματικής διεπαφής. |
| `arch_brace[side=1]#2` × `drive_package_envelopes` | 7435.1968 | -3/0/20 | collision | Concept/keep-out/συγκεντρωτικό hardware ή basket/feed envelope· απαιτεί έλεγχο της πραγματικής διεπαφής. |
| `basket_ball_space` × `side_frame[side=-1]` | 5027.7642 | -3/0/20 | collision | Concept/keep-out/συγκεντρωτικό hardware ή basket/feed envelope· απαιτεί έλεγχο της πραγματικής διεπαφής. |
| `basket_ball_space` × `side_frame[side=1]` | 5027.7642 | -3/0/20 | collision | Concept/keep-out/συγκεντρωτικό hardware ή basket/feed envelope· απαιτεί έλεγχο της πραγματικής διεπαφής. |
| `arch_leg_lower[side=-1]` × `drive_package_envelopes` | 2946.6879 | -3/0/20 | collision | Concept/keep-out/συγκεντρωτικό hardware ή basket/feed envelope· απαιτεί έλεγχο της πραγματικής διεπαφής. |
| `arch_leg_lower[side=1]` × `drive_package_envelopes` | 2946.6879 | -3/0/20 | collision | Concept/keep-out/συγκεντρωτικό hardware ή basket/feed envelope· απαιτεί έλεγχο της πραγματικής διεπαφής. |
| `drive_package_envelopes` × `p271_basket_current` | 2722.8923 | -3/0/20 | collision | Concept/keep-out/συγκεντρωτικό hardware ή basket/feed envelope· απαιτεί έλεγχο της πραγματικής διεπαφής. |
| `as5600_cover` × `drive_package_envelopes` | 2005.5790 | -3/0/20 | collision | Concept/keep-out/συγκεντρωτικό hardware ή basket/feed envelope· απαιτεί έλεγχο της πραγματικής διεπαφής. |
| `p271_basket_current` × `p271_front_system` | 1835.3616 | -3/0/20 | collision | Συγκεντρωτικό front feed/door σύστημα· απαιτεί έλεγχο της πραγματικής διεπαφής. |
| `reserved_ghosts` × `side_frame[side=-1]` | 1691.1755 | -3/0/20 | collision | Concept/keep-out/συγκεντρωτικό hardware ή basket/feed envelope· απαιτεί έλεγχο της πραγματικής διεπαφής. |
| `reserved_ghosts` × `vertical_electronics_plate` | 1592.9289 | -3/0/20 | collision | Concept/keep-out/συγκεντρωτικό hardware ή basket/feed envelope· απαιτεί έλεγχο της πραγματικής διεπαφής. |
| `drive_package_envelopes` × `fit0186_carrier` | 1572.5375 | -3/0/20 | collision | Concept/keep-out/συγκεντρωτικό hardware ή basket/feed envelope· απαιτεί έλεγχο της πραγματικής διεπαφής. |
| `p271_basket_current` × `side_frame[side=-1]` | 1566.3693 | -3/0/20 | collision | Concept/keep-out/συγκεντρωτικό hardware ή basket/feed envelope· απαιτεί έλεγχο της πραγματικής διεπαφής. |
| `p271_basket_current` × `side_frame[side=1]` | 1566.3693 | -3/0/20 | collision | Concept/keep-out/συγκεντρωτικό hardware ή basket/feed envelope· απαιτεί έλεγχο της πραγματικής διεπαφής. |
| `p271_front_system` × `p278_crossbar_half[side=-1]#2` | 1508.8078 | 20 | collision | Συγκεντρωτικό front feed/door σύστημα· απαιτεί έλεγχο της πραγματικής διεπαφής. |
| `p271_front_system` × `p278_crossbar_half[side=1]#2` | 1508.8078 | 20 | collision | Συγκεντρωτικό front feed/door σύστημα· απαιτεί έλεγχο της πραγματικής διεπαφής. |
| `reserved_ghosts` × `ugreen_rear_retainer` | 1478.9969 | -3/0/20 | collision | Concept/keep-out/συγκεντρωτικό hardware ή basket/feed envelope· απαιτεί έλεγχο της πραγματικής διεπαφής. |
| `moving_module_concept` × `p273_ski_mount_assembly` | 1452.6904 | -3/0/20 | collision | Concept/keep-out/συγκεντρωτικό hardware ή basket/feed envelope· απαιτεί έλεγχο της πραγματικής διεπαφής. |
| `drive_package_envelopes` × `front_terminal_v3[side=-1]` | 1275.2847 | -3/0/20 | collision | Concept/keep-out/συγκεντρωτικό hardware ή basket/feed envelope· απαιτεί έλεγχο της πραγματικής διεπαφής. |
| `drive_package_envelopes` × `front_terminal_v3[side=1]` | 1275.2847 | -3/0/20 | collision | Concept/keep-out/συγκεντρωτικό hardware ή basket/feed envelope· απαιτεί έλεγχο της πραγματικής διεπαφής. |
| `low_clamp_sleeve#2` × `p271_front_system` | 1207.4978 | 20 | collision | Συγκεντρωτικό front feed/door σύστημα· απαιτεί έλεγχο της πραγματικής διεπαφής. |
| `drive_package_envelopes` × `motor_module_v3[side=-1]` | 1090.7581 | -3/0/20 | collision | Concept/keep-out/συγκεντρωτικό hardware ή basket/feed envelope· απαιτεί έλεγχο της πραγματικής διεπαφής. |
| `drive_package_envelopes` × `motor_module_v3[side=1]` | 1090.7580 | -3/0/20 | collision | Concept/keep-out/συγκεντρωτικό hardware ή basket/feed envelope· απαιτεί έλεγχο της πραγματικής διεπαφής. |
| `arch_brace[side=1]` × `f_side_front_concept` | 880.2863 | -3/0/20 | collision | Concept/keep-out/συγκεντρωτικό hardware ή basket/feed envelope· απαιτεί έλεγχο της πραγματικής διεπαφής. |
| `arch_brace[side=-1]` × `f_side_front_concept` | 880.2860 | -3/0/20 | collision | Concept/keep-out/συγκεντρωτικό hardware ή basket/feed envelope· απαιτεί έλεγχο της πραγματικής διεπαφής. |
| `reserved_ghosts` × `side_frame[side=1]` | 702.1744 | -3/0/20 | collision | Concept/keep-out/συγκεντρωτικό hardware ή basket/feed envelope· απαιτεί έλεγχο της πραγματικής διεπαφής. |
| `p271_front_system` × `p278_crossbar_half[side=-1]` | 229.9246 | -3/0/20 | collision | Συγκεντρωτικό front feed/door σύστημα· απαιτεί έλεγχο της πραγματικής διεπαφής. |
| `p271_front_system` × `p278_crossbar_half[side=1]` | 229.9246 | -3/0/20 | collision | Συγκεντρωτικό front feed/door σύστημα· απαιτεί έλεγχο της πραγματικής διεπαφής. |
| `pi_shelf_v2` × `reserved_ghosts` | 136.5928 | -3/0/20 | collision | Concept/keep-out/συγκεντρωτικό hardware ή basket/feed envelope· απαιτεί έλεγχο της πραγματικής διεπαφής. |
| `corner_stay[s=-1]` × `f_side_front_concept` | 123.0000 | -3/0/20 | collision | Concept/keep-out/συγκεντρωτικό hardware ή basket/feed envelope· απαιτεί έλεγχο της πραγματικής διεπαφής. |
| `corner_stay[s=1]` × `f_side_front_concept` | 123.0000 | -3/0/20 | collision | Concept/keep-out/συγκεντρωτικό hardware ή basket/feed envelope· απαιτεί έλεγχο της πραγματικής διεπαφής. |
| `drive_package_envelopes` × `p278_terminal_sleeve[side=-1]#2` | 121.9610 | -3/0/20 | collision | Concept/keep-out/συγκεντρωτικό hardware ή basket/feed envelope· απαιτεί έλεγχο της πραγματικής διεπαφής. |
| `drive_package_envelopes` × `p278_terminal_sleeve[side=1]#2` | 121.9610 | -3/0/20 | collision | Concept/keep-out/συγκεντρωτικό hardware ή basket/feed envelope· απαιτεί έλεγχο της πραγματικής διεπαφής. |
| `fit0186_installed_hardware` × `moving_module_concept` | 71.5115 | 20 | collision | Concept/keep-out/συγκεντρωτικό hardware ή basket/feed envelope· απαιτεί έλεγχο της πραγματικής διεπαφής. |
| `panel_post_installed` × `reserved_ghosts` | 61.4524 | -3/0/20 | collision | Concept/keep-out/συγκεντρωτικό hardware ή basket/feed envelope· απαιτεί έλεγχο της πραγματικής διεπαφής. |
| `parkside_drawer[side=-1]` × `reserved_ghosts` | 87057.9325 | -3/0/20 | contained | Concept/keep-out/συγκεντρωτικό hardware ή basket/feed envelope· απαιτεί έλεγχο της πραγματικής διεπαφής. |
| `parkside_drawer[side=1]` × `reserved_ghosts` | 87057.9325 | -3/0/20 | contained | Concept/keep-out/συγκεντρωτικό hardware ή basket/feed envelope· απαιτεί έλεγχο της πραγματικής διεπαφής. |
| `magnet_cap` × `moving_module_concept` | 1601.3974 | -3/0/20 | contained | Concept/keep-out/συγκεντρωτικό hardware ή basket/feed envelope· απαιτεί έλεγχο της πραγματικής διεπαφής. |

## Αυτούσιο τελικό stdout του scene_check.py

Exit status: **1**. Τα παρακάτω είναι ακριβές stdout του εργαλείου.

```text
bodies per angle: -3°=106, 0°=106, 20°=106
duplicates dropped: 0   export failures: 0
STL part modules absent from the scene (33): bts_spacer_plate, carry_handle, center_lip, component_clip_plate, component_pin_plate, corner_guard, curved_cheek, drop_strut, floor_tile, front_motor_gamma_preview, ir_drop_bracket, joiner_plate, keeper, leg_split, management_tray_tile, panel_posts_print_set, plate_print, pulley, pulley_bracket, qa_plate, rail_extension, rear_flange_segment, rear_wall_tile, receiving_chute_tile, seg_len, short_handoff_ramp, side_flange_segment, side_frame_base_drill_jig, side_wall_tile, sled, sleeve_fit_coupon, ugreen_clamp_caps_print_set, upright_mount_fit_coupon

3 pairs where one body lies wholly inside another (drawn twice by a context scene, or a part inside a surrogate envelope):
      87057.9 mm3  parkside_drawer[side=-1] inside/with reserved_ghosts
      87057.9 mm3  parkside_drawer[side=1] inside/with reserved_ghosts
       1601.4 mm3  magnet_cap inside/with moving_module_concept

115 collisions > 0.0001 mm3
      53299.0 mm3  parkside_dock[side=-1]             x reserved_ghosts                    [-3,0,20]  bbox [-268.0, -158.0, 58.0, -128.5, -77.5, 117.0]
      53299.0 mm3  parkside_dock[side=1]              x reserved_ghosts                    [-3,0,20]  bbox [-268.0, 77.5, 58.0, -128.5, 158.0, 117.0]
      31928.3 mm3  basket_ball_space                  x drive_package_envelopes            [-3,0,20]  bbox [-85.0, -173.5, 67.7, -45.0, 173.5, 104.9]
      25223.2 mm3  reserved_ghosts                    x ugreen_post_holder                 [-3,0,20]  bbox [-269.5, 8.5, 232.5, -148.5, 59.5, 266.5]
      17207.2 mm3  arch_foot_x300[side=-1]            x drive_package_envelopes            [-3,0,20]  bbox [280.0, -240.0, 52.0, 320.0, -159.0, 100.1]
      17207.2 mm3  arch_foot_x300[side=1]             x drive_package_envelopes            [-3,0,20]  bbox [280.0, 159.0, 52.0, 320.0, 240.0, 100.1]
      12247.7 mm3  corner_stay[s=-1]                  x rear_lo[s=-1]                      [-3,0,20]  bbox [-196.0, -245.0, 60.0, -185.0, -234.5, 194.0]
      12247.7 mm3  corner_stay[s=1]                   x rear_lo[s=1]                       [-3,0,20]  bbox [-196.0, 234.5, 60.0, -185.0, 245.0, 194.0]
      10865.9 mm3  arch_brace[side=1]#2               x arch_leg_upper[side=1]             [-3,0,20]  bbox [343.4, 218.0, 159.1, 399.7, 241.0, 249.1]
      10865.9 mm3  arch_brace[side=-1]#2              x arch_leg_upper[side=-1]            [-3,0,20]  bbox [343.4, -241.0, 159.1, 399.7, -218.0, 249.1]
      10381.7 mm3  arch_brace[side=-1]                x arch_brace[side=-1]#2              [-3,0,20]  bbox [336.1, -241.0, 167.7, 398.2, -215.0, 250.2]
      10381.7 mm3  arch_brace[side=1]                 x arch_brace[side=1]#2               [-3,0,20]  bbox [336.1, 215.0, 167.7, 398.2, 241.0, 250.2]
       8620.9 mm3  deck_rear[s=1]                     x rear_hi[s=1]                       [-3,0,20]  bbox [-308.0, 0.2, 339.0, -185.0, 247.0, 347.0]
       8596.7 mm3  deck_rear[s=-1]                    x rear_hi[s=-1]                      [-3,0,20]  bbox [-308.0, -247.0, 339.0, -185.0, -0.2, 347.0]
       7497.3 mm3  drive_package_envelopes            x kfl08_base                         [-3,0,20]  bbox [280.0, -214.9, 82.3, 297.4, -160.0, 104.5]
       7435.2 mm3  arch_brace[side=-1]#2              x drive_package_envelopes            [-3,0,20]  bbox [290.3, -241.0, 71.8, 320.0, -215.0, 104.8]
       7435.2 mm3  arch_brace[side=1]#2               x drive_package_envelopes            [-3,0,20]  bbox [290.3, 215.0, 71.8, 320.0, 241.0, 104.8]
       5833.2 mm3  arch_brace[side=-1]#2              x arch_leg_lower[side=-1]            [-3,0,20]  bbox [306.6, -238.0, 71.8, 356.5, -218.0, 164.4]
       5833.2 mm3  arch_brace[side=1]#2               x arch_leg_lower[side=1]             [-3,0,20]  bbox [306.6, 218.0, 71.8, 356.5, 238.0, 164.4]
       5615.0 mm3  arch_brace[side=-1]                x arch_leg_upper[side=-1]            [-3,0,20]  bbox [351.9, -241.0, 185.5, 398.2, -218.0, 249.4]
       5615.0 mm3  arch_brace[side=1]                 x arch_leg_upper[side=1]             [-3,0,20]  bbox [351.9, 218.0, 185.5, 398.2, 241.0, 249.4]
       5438.3 mm3  cable_bulkhead                     x side_frame[side=1]                 [-3,0,20]  bbox [-135.3, 36.5, 64.0, -125.0, 66.0, 155.0]
       5096.0 mm3  base_half_v2                       x cable_bulkhead                     [-3,0,20]  bbox [-149.0, -67.5, 55.0, -125.0, -12.5, 59.0]
       5096.0 mm3  base_half_v2#2                     x cable_bulkhead                     [-3,0,20]  bbox [-149.0, 12.5, 55.0, -125.0, 67.5, 59.0]
       5027.8 mm3  basket_ball_space                  x side_frame[side=-1]                [-3,0,20]  bbox [-120.0, -66.0, 72.2, -114.0, -42.0, 163.0]
       5027.8 mm3  basket_ball_space                  x side_frame[side=1]                 [-3,0,20]  bbox [-120.0, 42.0, 72.2, -114.0, 66.0, 163.0]
       4714.2 mm3  cable_bulkhead                     x side_frame[side=-1]                [-3,0,20]  bbox [-135.3, -66.0, 64.0, -125.0, -36.5, 155.0]
       3298.1 mm3  parkside_dock[side=-1]             x parkside_drawer[side=-1]           [-3,0,20]  bbox [-268.0, -158.0, 68.0, -132.0, -82.0, 92.5]
       3298.1 mm3  parkside_dock[side=1]              x parkside_drawer[side=1]            [-3,0,20]  bbox [-268.0, 82.0, 68.0, -132.0, 158.0, 92.5]
       2946.7 mm3  arch_leg_lower[side=-1]            x drive_package_envelopes            [-3,0,20]  bbox [305.9, -238.0, 66.2, 320.0, -218.0, 101.0]
       2946.7 mm3  arch_leg_lower[side=1]             x drive_package_envelopes            [-3,0,20]  bbox [305.9, 218.0, 66.2, 320.0, 238.0, 101.0]
       2722.9 mm3  drive_package_envelopes            x p271_basket_current                [-3,0,20]  bbox [-85.0, -173.5, 70.0, -45.0, 173.5, 104.9]
       2134.7 mm3  arch_crossbar_end[side=-1]         x lidar_mast[side=-1]                [-3,0,20]  bbox [380.0, -162.0, 245.0, 400.0, -138.0, 258.0]
       2134.7 mm3  arch_crossbar_end[side=1]          x lidar_mast[side=1]                 [-3,0,20]  bbox [380.0, 138.0, 245.0, 400.0, 162.0, 258.0]
       2034.2 mm3  lidar_crossbar_half[side=-1]       x lidar_mast[side=-1]                [-3,0,20]  bbox [378.0, -160.0, 344.0, 402.0, -138.0, 354.0]
       2034.2 mm3  lidar_crossbar_half[side=1]        x lidar_mast[side=1]                 [-3,0,20]  bbox [378.0, 138.0, 344.0, 402.0, 160.0, 354.0]
       2005.6 mm3  as5600_cover                       x drive_package_envelopes            [-3,0,20]  bbox [364.0, 255.5, 52.8, 385.0, 258.0, 98.8]
       1835.4 mm3  p271_basket_current                x p271_front_system                  [-3,0,20]  bbox [208.0, -173.5, 46.1, 225.0, 172.0, 223.5]
       1691.2 mm3  reserved_ghosts                    x side_frame[side=-1]                [-3,0,20]  bbox [-255.6, -63.0, 147.0, -133.1, -33.3, 170.0]
       1592.9 mm3  reserved_ghosts                    x vertical_electronics_plate         [-3,0,20]  bbox [-191.0, -155.0, 151.0, -177.0, 81.0, 281.0]
       1572.5 mm3  drive_package_envelopes            x fit0186_carrier                    [-3,0,20]  bbox [302.9, -194.6, 91.6, 315.4, -160.0, 104.7]
       1566.4 mm3  p271_basket_current                x side_frame[side=-1]                [-3,0,20]  bbox [-120.0, -66.0, 70.2, -114.0, -42.0, 163.0]
       1566.4 mm3  p271_basket_current                x side_frame[side=1]                 [-3,0,20]  bbox [-120.0, 42.0, 70.2, -114.0, 66.0, 163.0]
       1508.8 mm3  p271_front_system                  x p278_crossbar_half[side=-1]#2      [20]  bbox [290.0, -56.5, 26.8, 310.0, 0.0, 42.0]
       1508.8 mm3  p271_front_system                  x p278_crossbar_half[side=1]#2       [20]  bbox [290.0, 0.0, 26.8, 310.0, 56.5, 42.0]
       1479.0 mm3  reserved_ghosts                    x ugreen_rear_retainer               [-3,0,20]  bbox [-269.5, 8.5, 237.0, -266.6, 59.5, 247.0]
       1452.7 mm3  moving_module_concept              x p273_ski_mount_assembly            [-3,0,20]  bbox [339.9, -46.1, 16.5, 352.2, 46.1, 24.9]
       1275.3 mm3  drive_package_envelopes            x front_terminal_v3[side=-1]         [-3,0,20]  bbox [280.0, -236.1, 22.0, 320.0, -172.2, 52.0]
       1275.3 mm3  drive_package_envelopes            x front_terminal_v3[side=1]          [-3,0,20]  bbox [280.0, 172.2, 22.0, 320.0, 236.1, 52.0]
       1231.3 mm3  base_half_v2                       x rear_lo[s=-1]                      [-3,0,20]  bbox [-272.0, -163.5, 47.5, -267.0, -39.0, 59.0]
       1231.3 mm3  base_half_v2#2                     x rear_lo[s=1]                       [-3,0,20]  bbox [-272.0, 39.0, 47.5, -267.0, 163.5, 59.0]
       1207.5 mm3  low_clamp_sleeve#2                 x p271_front_system                  [20]  bbox [293.5, -56.5, 28.2, 306.5, 56.5, 38.5]
       1151.9 mm3  arch_brace[side=-1]#2              x joint_sleeve_x[side=-1]            [-3,0,20]  bbox [341.2, -234.5, 137.9, 367.9, -221.5, 182.3]
       1151.9 mm3  arch_brace[side=1]#2               x joint_sleeve_x[side=1]             [-3,0,20]  bbox [341.2, 221.5, 137.9, 367.9, 234.5, 182.3]
       1090.8 mm3  drive_package_envelopes            x motor_module_v3[side=-1]           [-3,0,20]  bbox [-85.0, -235.9, 22.0, -45.0, -195.0, 52.0]
       1090.8 mm3  drive_package_envelopes            x motor_module_v3[side=1]            [-3,0,20]  bbox [-85.0, 195.0, 22.0, -45.0, 233.3, 52.0]
       1054.4 mm3  control_panel_v2                   x deck_rear[s=-1]                    [-3,0,20]  bbox [-267.5, -95.0, 339.0, -257.3, -14.0, 351.0]
       1054.4 mm3  control_panel_v2                   x deck_rear[s=1]                     [-3,0,20]  bbox [-267.5, 14.0, 339.0, -257.3, 95.0, 351.0]
        971.5 mm3  arch_brace[side=-1]#2              x arch_crossbar_end[side=-1]         [-3,0,20]  bbox [380.0, -228.0, 232.0, 399.7, -215.0, 250.2]
        971.5 mm3  arch_brace[side=1]#2               x arch_crossbar_end[side=1]          [-3,0,20]  bbox [380.0, 215.0, 232.0, 399.7, 228.0, 250.2]
        903.9 mm3  arch_brace[side=1]                 x arch_crossbar_end[side=1]          [-3,0,20]  bbox [380.0, 215.0, 232.0, 398.2, 228.0, 252.3]
        903.9 mm3  arch_brace[side=-1]                x arch_crossbar_end[side=-1]         [-3,0,20]  bbox [380.0, -228.0, 232.0, 398.2, -215.0, 252.3]
        880.3 mm3  arch_brace[side=1]                 x f_side_front_concept               [-3,0,20]  bbox [334.1, 236.5, 195.3, 357.5, 241.0, 225.0]
        880.3 mm3  arch_brace[side=-1]                x f_side_front_concept               [-3,0,20]  bbox [334.1, -241.0, 195.3, 357.5, -236.5, 225.0]
        844.8 mm3  arch_crossbar_end[side=-1]         x arch_leg_upper[side=-1]            [-3,0,20]  bbox [380.0, -228.0, 232.0, 400.0, -218.0, 249.0]
        844.8 mm3  arch_crossbar_end[side=1]          x arch_leg_upper[side=1]             [-3,0,20]  bbox [380.0, 218.0, 232.0, 400.0, 228.0, 249.0]
        761.0 mm3  arch_brace[side=-1]                x arch_foot_x240[side=-1]            [-3,0,20]  bbox [240.2, -239.0, 68.7, 253.0, -215.0, 75.8]
        761.0 mm3  arch_brace[side=1]                 x arch_foot_x240[side=1]             [-3,0,20]  bbox [240.2, 215.0, 68.7, 253.0, 239.0, 75.8]
        702.2 mm3  reserved_ghosts                    x side_frame[side=1]                 [-3,0,20]  bbox [-186.5, 50.0, 151.0, -181.4, 58.0, 170.0]
        503.4 mm3  parkside_door[s=1]                 x rear_lo[s=1]                       [-3,0,20]  bbox [-277.0, 161.5, 75.0, -273.5, 167.9, 139.0]
        503.4 mm3  parkside_door[s=-1]                x rear_lo[s=-1]                      [-3,0,20]  bbox [-277.0, -167.9, 75.0, -273.5, -161.5, 139.0]
        424.2 mm3  door_turn_button[s=-1]             x rear_lo[s=-1]                      [-3,0,20]  bbox [-305.3, -68.0, 99.0, -302.2, -45.0, 110.9]
        424.2 mm3  door_turn_button[s=1]              x rear_lo[s=1]                       [-3,0,20]  bbox [-305.3, 45.0, 99.0, -302.2, 68.0, 110.9]
        364.7 mm3  lidar_crossbar_half[side=1]        x oak_universal_plate                [-3,0,20]  bbox [378.0, 0.0, 344.0, 383.1, 24.0, 350.1]
        364.7 mm3  lidar_crossbar_half[side=-1]       x oak_universal_plate                [-3,0,20]  bbox [378.0, -24.0, 344.0, 383.1, 0.0, 350.1]
        317.1 mm3  panel_post_installed               x side_frame[side=-1]                [-3,0,20]  bbox [-268.0, -60.8, 148.8, -129.0, -34.3, 158.5]
        256.7 mm3  interface_fasteners                x rear_lo[s=-1]                      [-3,0,20]  bbox [-306.1, -247.0, 53.7, -188.0, -44.1, 177.9]
        256.7 mm3  interface_fasteners                x rear_lo[s=1]                       [-3,0,20]  bbox [-306.1, 44.1, 53.7, -188.0, 247.0, 177.9]
        233.2 mm3  panel_post_installed               x side_frame[side=1]                 [-3,0,20]  bbox [-268.0, 47.2, 148.8, -129.0, 66.0, 158.5]
        229.9 mm3  p271_front_system                  x p278_crossbar_half[side=-1]        [-3,0,20]  bbox [253.0, -40.0, 41.0, 265.0, 0.0, 42.0]
        229.9 mm3  p271_front_system                  x p278_crossbar_half[side=1]         [-3,0,20]  bbox [253.0, 0.0, 41.0, 265.0, 40.0, 42.0]
        211.4 mm3  arch_foot_x300[side=-1]            x arch_leg_lower[side=-1]            [-3,0,20]  bbox [322.9, -238.0, 62.1, 331.2, -218.0, 65.0]
        211.4 mm3  arch_foot_x300[side=1]             x arch_leg_lower[side=1]             [-3,0,20]  bbox [322.9, 218.0, 62.1, 331.2, 238.0, 65.0]
        172.6 mm3  ugreen_post_holder                 x ugreen_rear_retainer               [-3,0,20]  bbox [-269.6, -2.0, 233.0, -266.6, 70.0, 239.5]
        136.6 mm3  pi_shelf_v2                        x reserved_ghosts                    [-3,0,20]  bbox [-261.0, -37.0, 68.0, -163.0, 37.0, 72.0]
        123.0 mm3  corner_stay[s=-1]                  x f_side_front_concept               [-3,0,20]  bbox [-185.0, -244.5, 60.0, -184.0, -242.0, 194.0]
        123.0 mm3  corner_stay[s=1]                   x f_side_front_concept               [-3,0,20]  bbox [-185.0, 242.0, 60.0, -184.0, 244.5, 194.0]
        122.0 mm3  drive_package_envelopes            x p278_terminal_sleeve[side=-1]#2    [-3,0,20]  bbox [296.0, -190.0, 37.2, 304.0, -177.3, 38.5]
        122.0 mm3  drive_package_envelopes            x p278_terminal_sleeve[side=1]#2     [-3,0,20]  bbox [296.0, 177.3, 37.2, 304.0, 190.0, 38.5]
        112.5 mm3  control_panel_v2                   x rear_hi[s=-1]                      [-3,0,20]  bbox [-267.5, -91.0, 329.0, -267.0, -14.0, 339.0]
        112.5 mm3  control_panel_v2                   x rear_hi[s=1]                       [-3,0,20]  bbox [-267.5, 14.0, 329.0, -267.0, 91.0, 339.0]
        106.9 mm3  arch_brace[side=-1]#2              x arch_foot_x300[side=-1]            [-3,0,20]  bbox [300.4, -240.0, 71.8, 312.4, -215.0, 76.8]
        106.9 mm3  arch_brace[side=1]#2               x arch_foot_x300[side=1]             [-3,0,20]  bbox [300.4, 215.0, 71.8, 312.4, 240.0, 76.8]
         96.8 mm3  clamp_sleeve_x[side=-1xx=35]       x rail_extension_v3[side=-1]         [-3,0,20]  bbox [93.6, -241.5, 25.5, 95.0, -198.5, 48.5]
         96.8 mm3  clamp_sleeve_x[side=1xx=35]        x rail_extension_v3[side=1]          [-3,0,20]  bbox [93.6, 198.5, 25.5, 95.0, 241.5, 48.5]
         71.5 mm3  fit0186_installed_hardware         x moving_module_concept              [20]  bbox [376.9, -138.8, 161.6, 380.1, -123.5, 168.0]
         61.5 mm3  panel_post_installed               x reserved_ghosts                    [-3,0,20]  bbox [-147.8, -52.1, 175.0, -135.0, -49.0, 281.0]
         12.5 mm3  base_half_v2                       x rear_foot_wedge_washer[side=-1]    [-3,0,20]  bbox [-264.5, -57.0, 43.0, -254.0, -47.0, 49.5]
         12.5 mm3  base_half_v2#2                     x rear_foot_wedge_washer[side=1]     [-3,0,20]  bbox [-264.5, 47.0, 43.0, -254.0, 57.0, 49.5]
          5.5 mm3  door_turn_button[s=-1]             x parkside_door[s=-1]                [-3,0,20]  bbox [-303.5, -78.7, 99.3, -302.2, -74.8, 103.2]
          5.5 mm3  door_turn_button[s=1]              x parkside_door[s=1]                 [-3,0,20]  bbox [-303.5, 74.8, 99.3, -302.2, 78.7, 103.2]
          1.5 mm3  corner_clamp#2                     x motor_module_v3[side=-1]           [-3,0,20]  bbox [-165.0, -245.0, 25.5, -165.0, -195.0, 48.5]
          1.5 mm3  corner_clamp                       x motor_module_v3[side=1]            [-3,0,20]  bbox [-165.0, 195.0, 25.5, -165.0, 245.0, 48.5]
          1.2 mm3  deck_rear[s=1]                     x rear_hi[s=-1]                      [-3,0,20]  bbox [-307.2, -5.0, 342.0, -307.0, -2.9, 347.0]
          0.0 mm3  interface_fasteners                x rear_hi[s=-1]                      [-3,0,20]  bbox [-282.0, -85.9, 335.2, -267.0, -19.1, 335.2]
          0.0 mm3  interface_fasteners                x rear_hi[s=1]                       [-3,0,20]  bbox [-282.0, 19.1, 335.2, -267.0, 85.9, 335.2]
          0.0 mm3  control_panel_v2                   x interface_fasteners                [-3,0,20]  bbox [-267.5, -85.9, 335.2, -264.0, 85.9, 345.2]
          0.0 mm3  deck_rear[s=-1]                    x interface_fasteners                [-3,0,20]  bbox [-277.0, -85.9, 345.2, -267.0, -19.1, 345.2]
          0.0 mm3  deck_rear[s=1]                     x interface_fasteners                [-3,0,20]  bbox [-277.0, 19.1, 345.2, -267.0, 85.9, 345.2]
          0.0 mm3  base_half_v2                       x interface_fasteners                [-3,0,20]  bbox [-272.0, -145.9, 53.7, -262.0, -144.1, 53.7]
          0.0 mm3  base_half_v2#2                     x interface_fasteners                [-3,0,20]  bbox [-272.0, 144.1, 53.7, -262.0, 145.9, 53.7]
          0.0 mm3  center_splice                      x rear_lo[s=1]                       [-3,0,20]  bbox [-299.2, 1.0, 99.0, -297.9, 13.0, 161.0]
          0.0 mm3  center_splice#2                    x rear_hi[s=1]                       [-3,0,20]  bbox [-299.2, 1.0, 244.0, -297.9, 13.0, 311.0]
          0.0 mm3  center_splice                      x rear_lo[s=-1]                      [-3,0,20]  bbox [-299.2, -13.0, 99.0, -297.9, -1.0, 161.0]
          0.0 mm3  center_splice#2                    x rear_hi[s=-1]                      [-3,0,20]  bbox [-299.2, -13.0, 244.0, -297.9, -1.0, 311.0]
```
