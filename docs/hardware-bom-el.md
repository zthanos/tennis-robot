# Hardware BOM — On-Board Compute (EL)

> Απόφαση: **Raspberry Pi 5 (16GB)** ως on-board computer του ρομπότ,
> **αντικαθιστά** το παλιό candidate Lenovo i5 mini-PC. Το βαρύ vision
> (stereo depth + ανίχνευση) γίνεται offload στην **OAK-D** (Myriad X VPU),
> οπότε η CPU του Pi μένει ελεύθερη για slam_toolbox + Nav2 + classical CV.
>
> Σχετικά: `prototype-purchase-list-el.md` §6, `telemetry-architecture-el.md`.

---

## 1. Απόφαση: Pi 5 vs i5 mini-PC

Το ρομπότ είναι **μπαταριοκίνητο και κινούμενο**, οπότε η τιμή αγοράς είναι
δευτερεύουσα — κρίνουν τα watt, τα ampere και τα γραμμάρια.

| Κριτήριο | Pi 5 16GB | i5 mini-PC | Νικητής |
|---|---|---|---|
| Κατανάλωση | ~5-10W | ~25-45W | **Pi** (3-5x λιγότερο) |
| Τροφοδοσία | 5V, απευθείας από buck | 12-19V, ειδικό DC-DC | **Pi** |
| Μέγεθος/βάρος | μικροσκοπικό | μεγαλύτερο | **Pi** |
| Σύνδεση MCU (Mega/Nano) | USB serial | USB serial | ισοπαλία (USB CDC και στα δύο) |
| Θερμότητα σε κλειστό σασί | εύκολη ψύξη | δύσκολη | **Pi** |
| Compute headroom | αρκετό (+OAK-D offload) | περισσότερο | i5 |
| Gazebo onboard | όχι (περιττό onboard) | ναι | i5 (μόνο bench) |

Το μόνο πλεονέκτημα του i5 (headroom) ακυρώνεται από το OAK-D offload. Επιπλέον
ο i5 με 30-40W αντί 8W σημαίνει 3-5x μεγαλύτερη/βαρύτερη μπαταρία για ίδια
αυτονομία — μόνιμος φόρτος που κουβαλιέται για πάντα.

**Πότε θα γυρίζαμε στον i5:** μόνο αν το ρομπότ ήταν tethered (σταθερή τροφοδοσία)
ή αν τρέχαμε βαρύ 3D perception / onboard learning ΧΩΡΙΣ offload. Κανένα δεν ισχύει.

---

## 2. Compute budget — γιατί το Pi 5 αρκεί

| Φορτίο | Πού τρέχει | Βάρος στο Pi |
|---|---|---|
| 2D LiDAR `/scan` | host | αμελητέο (KB/s) |
| slam_toolbox (map→base_link TF) | host | μέτριο, OK σε Pi (τρέχει & σε Pi 4) |
| Survey FSM + extraction | host | ελαφρύ (5Hz, max 1500 σημεία) |
| Nav2 local costmap + avoidance | host | μέτριο — χρήση **DWB/Regulated Pure Pursuit**, ΟΧΙ MPPI |
| Stereo depth (`/camera/depth`) | **OAK-D VPU** | offloaded (μηδέν στο host) |
| Classical CV (HSV/Canny/Hough) | host | ελαφρύ· throttle στα ~10fps |
| Telemetry writes (5Hz JSON) | host + NVMe | ελαφρύ (γι' αυτό NVMe, όχι SD) |

Obstacle avoidance (π.χ. άνθρωπος στο court): reactive LiDAR e-stop + Nav2 local
costmap με ελαφρύ controller — μέσα στο budget του Pi 5.

---

## 3. BOM — Compute stack

| # | Είδος | Σημειώσεις | Ενδεικτικό κόστος |
|---|---|---|---|
| 1 | **db-tronic Raspberry Pi 5 16GB PCIe M.2 NVMe Set — 64GB Edition** | Επιλεγμένο στο Amazon basket. Περιλαμβάνει Pi 5 16GB, PCIe M.2 NVMe board, active cooler, metal housing και 27W PSU. Η «64GB Edition» αφορά το αποθηκευτικό μέσο του kit· να επιβεβαιωθεί το ακριβές περιεχόμενο κατά την παραλαβή. | **€385,54** |
| 2 | **Silicon Power P34A60 256GB NVMe M.2 PCIe Gen3x4 2280 SSD** (`SP256GBP34A60M28AY`) | Επιλεγμένο ξεχωριστά ως NVMe boot drive. Δεν είναι ο επίσημος Raspberry Pi SSD· απαιτείται recognition/boot/stress test με το M.2 board πριν θεωρηθεί validated για το robot. | **€50,99** |
| 3 | 12V→5V buck converter 5A+ | On-robot τροφοδοσία Pi από LiFePO4 (ήδη στη βασική λίστα) | ~€8-12 |
| — | (microSD του kit) | Μόνο για flashing/εφεδρεία — **όχι** ως boot drive | περιλαμβάνεται |
| — | (27W PSU του kit) | Μόνο για bench/development — όχι on-robot | περιλαμβάνεται |
| **Σύνολο επιλεγμένου Amazon basket** | | | **€436,53** |
| **Σύνολο compute με on-robot buck** | | | **~€444,53-448,53** |

> Τιμές από το Amazon shopping basket της 28/06/2026. Τα είδη είναι
> **επιλεγμένα αλλά όχι ακόμη καταγεγραμμένα ως παραγγελμένα**.

---

## 4. NVMe SSD — συμβατότητα (κρίσιμο)

Το Pi 5 έχει γνωστά θέματα recognition με κάποιους NVMe controllers. **Μη πάρεις
τυχαίο δίσκο.**

**Top (χαμηλότερο ρίσκο):** Επίσημος Raspberry Pi SSD (256/512GB) — φτιαγμένος για το M.2 HAT+.

**Τρέχουσα επιλογή καλαθιού:** Silicon Power P34A60 256GB
(`SP256GBP34A60M28AY`). Δεν υπάρχει ακόμη project-specific validation. Πριν
χρησιμοποιηθεί ως κύριο boot drive:

1. επιβεβαίωση ότι εμφανίζεται στα `lspci` / `lsblk`,
2. cold-boot και reboot δοκιμές,
3. έλεγχος SMART,
4. συνεχόμενο read/write stress test,
5. επανάληψη με το 27W PSU και αργότερα με το on-robot 5V/5A buck.

**Γενικά ασφαλείς:** Samsung 980 / 990 EVO, Crucial P3.

**Απόφυγε (reported I/O errors / no recognition):** Kingston NV2 / NV3,
WD Black SN770 / SN850X. Τα WD γενικά είναι hit-or-miss στο Pi 5.

Setup σημειώσεις:
- Μόνο **M.2 NVMe Key-M** (όχι SATA, όχι Key-B).
- Το HAT δέχεται μεγέθη **2230 / 2242 / 2280**.
- Νέος δίσκος θέλει partition + format πριν χρησιμοποιηθεί.
- Αν δεν αναγνωρίζεται, βάλε `dtparam=pciex1_gen=3` στο `config.txt`.
- **5V/5A σταθερό** είναι απαραίτητο για σταθερό NVMe — υποτροφοδοσία → throttle/αστάθεια.

---

## 5. Σύνδεση με MCU (Mega / Nano / launcher)

Ο Pi μιλάει στους Arduino μέσω **USB serial (CDC, 115200 baud)** — host-agnostic,
ίδιο με PC. Ο IMU (MPU6050) είναι στο **I2C του Arduino Mega** (μέσω perfboard),
**όχι** στα GPIO του Pi. Άρα ο Pi χρειάζεται κυρίως **USB πρόσβαση**, όχι GPIO.

Η ασφάλεια/real-time loop ζει στους Arduino (Mega boots DISARMED, command timeout,
encoder sanity), ανεξάρτητα από το Pi. Ο Pi στέλνει μόνο high-level εντολές — δεν
κλείνει real-time βρόχο πάνω από serial.

**USB port budget (κρίσιμο):**

| Συσκευή | Θύρα |
|---|---|
| OAK-D S2 | USB3 |
| RPLIDAR C1 | USB (ttyUSB0) |
| Arduino Mega (driving) | USB (ttyACM) |
| Arduino Nano (collector) | USB (ttyACM) |

= **4 συσκευές / 4 θύρες** στο Pi 5 (2×USB3 + 2×USB2). Γεμίζουν όλες.
**Με το flywheel/launcher MCU (Phase 2) → χρειάζεται powered USB hub.**

## 6. Σημεία προσοχής στην ενσωμάτωση

- **udev rules (απαραίτητο).** Σε Linux ο Mega/Nano εναλλάσσουν `ttyACM0`/`ttyACM1`
  σε κάθε boot. Κλείδωσέ τους σε σταθερά ονόματα (π.χ. `/dev/ttyMEGA`, `/dev/ttyNANO`)
  βάσει USB serial number, αλλιώς το driving node μπορεί να ανοίξει τον collector.
- **USB3 ↔ Wi-Fi interference.** Το USB3 της OAK-D εκπέμπει θόρυβο στα 2.4GHz που
  χτυπά το onboard Wi-Fi (το telemetry link). Κοντό/θωρακισμένο USB3 καλώδιο,
  κεραία μακριά, ή 5GHz Wi-Fi.
- **USB access στο metal case.** Το case του kit πρέπει να αφήνει πρόσβαση στις 4
  θύρες USB. Αν όχι → βγάζουμε τη board από το case για on-robot mount, με standoffs.
- **Τροφοδοσία on-robot:** Pi από LiFePO4 → 12V→5V buck 5A+. Το 27W PSU μένει για bench.
  Πρόσεξε το συνολικό 5V budget: Pi + OAK-D (USB-powered, λαίμαργη) + LiDAR + Arduinos.
  Powered USB hub βοηθά και εδώ.
- **NVMe boot, όχι SD:** οι microSD κάρτες κάνουν corruption σε Pi με συχνά μικρά
  writes + απότομες διακοπές ρεύματος — ακριβώς το προφίλ του ρομπότ (5Hz telemetry
  + battery). Γι' αυτό boot από NVMe.
- **Vibration:** rubber isolation/standoffs αν χρειαστεί.
- **Wi-Fi:** για το telemetry link· external antenna/USB αν το onboard δεν φτάνει.

## 7. Flywheel / launcher (Phase 2)

Ξεχωριστό module (dual flywheels + ESCs), προς το παρόν deferred. Control:

- **Ξεχωριστός MCU (προτεινόμενο):** απομονώνει το safety-critical launcher από το
  drive loop → +1 USB συσκευή → **powered USB hub**.
- **Spare PWM pins του Mega:** τα ESCs ελέγχονται με servo-style PWM και ο Mega έχει
  άφθονα PWM pins → κανένα επιπλέον USB, αλλά μπλέκει launcher με drive MCU.

---

## 8. Τι ΔΕΝ αλλάζει

Το υπόλοιπο σύστημα (μπαταρία LiFePO4, motor drivers BTS7960, Arduino Mega motion
MCU, buck converters, OAK-D, LiDAR) παραμένει ως έχει στο `prototype-purchase-list-el.md`.
Μόνο το on-board compute αλλάζει από i5 mini-PC σε Pi 5 16GB.
