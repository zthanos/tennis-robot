# Intake bench web panel στο Raspberry Pi

Το `scripts/intake_bench_panel.py` σερβίρει το
`scripts/intake_bench_panel.html` στη θύρα `8082` και ελέγχει αποκλειστικά το
firmware `arduino/collector/07_dual_intake_mega_bench`.

## Safety contract

- Δεν υπάρχει πεδίο για αυθαίρετες serial εντολές. Ο server δέχεται μόνο το
  σταθερό allow-list σεναρίων του αρχείου Python.
- Σε κάθε σύνδεση ή επανασύνδεση USB, οι πρώτες εντολές είναι `STOP`,
  `DISARM`, `STATUS`.
- Κάθε χρονισμένη δοκιμή τελειώνει με `STOP`, `DISARM`, `STATUS` ακόμη και αν
  το firmware έχει ήδη σταματήσει από timeout.
- Η θέση `OUT ON/OFF` του εργαστηριακού τροφοδοτικού δεν είναι ηλεκτρονικά
  ορατή στο Pi. Ο χειριστής πρέπει να επιβεβαιώνει το checklist της κάρτας.
- Το `AUTO_BENCH` έχει επιπλέον watchdog 30 s από τον server.
- Το μεγάλο κόκκινο `STOP + DISARM` δεν ζητά επιβεβαίωση.
- Η σειριακή θύρα ανοίγει αποκλειστικά. Όσο τρέχει η υπηρεσία, δεν τρέχουμε
  παράλληλα `run_intake_ir_cycle.py`, Arduino Serial Monitor ή χειροκίνητο
  pyserial script. Για χειροκίνητη χρήση προηγείται
  `sudo systemctl stop tennis-robot-intake-bench.service`.

Το panel δεν αντικαθιστά φυσικό E-stop, σωστή ασφάλεια τροφοδοσίας ή σταθερές
κλέμες. Δεν χρησιμοποιείται με χαλαρά κροκοδειλάκια.

## Εγκατάσταση στο Pi

Στο repository του Pi:

```bash
sudo apt update
sudo apt install python3-serial
sudo TENNIS_ROBOT_SERVICE_USER=thanos \
  ./scripts/install_pi_intake_bench_service.sh
```

Η υπηρεσία ξεκινά αυτόματα σε κάθε boot:

```bash
systemctl status tennis-robot-intake-bench.service
journalctl -u tennis-robot-intake-bench.service -f
```

Άνοιγμα από client στο hotspot του Pi:

```text
http://10.42.0.1:8082
```

Στο οικιακό LAN χρησιμοποιείται η τρέχουσα διεύθυνση του Pi, π.χ.
`http://tennisserver.local:8082`.

## Σημερινή αντιστοίχιση

- Φυσικός δεξιός: firmware `L`, `RPWM=D44`, `LPWM=D45`, enable `D40`, inward
  παλμός `PULSE L -60 250`.
- Φυσικός αριστερός: firmware `R`, `RPWM=D46`, `LPWM=D11`, enable `D42`, inward
  παλμός `PULSE R 60 250`.

Η ονομασία firmware είναι προσωρινή και το panel εμφανίζει πάντα πρώτα τη
φυσική πλευρά για να αποφεύγεται σύγχυση.

## Πρώτη επόμενη δοκιμή

Η κάρτα «ύποπτο μοτέρ στον καλό driver» εκτελεί `PULSE R 60 250`, αλλά μόνο
αφού ο χειριστής έχει:

1. αποσυνδέσει ρεύμα και USB,
2. αφαιρέσει/μονώσει το κανονικό μοτέρ του καλού driver,
3. συνδέσει μόνο το ύποπτο μοτέρ στα `M+ / M-` του καλού driver,
4. αφήσει τις εξόδους του ύποπτου driver κενές.

Αν το μοτέρ κινηθεί, μοτέρ και καλώδιο ισχύος είναι καλά και η βλάβη μένει
στον ύποπτο BTS7960 ή στην κλέμα/κόλλησή του.
