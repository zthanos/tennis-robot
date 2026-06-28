# Collector Arduino Bring-Up Sketches

Αυτός ο φάκελος έχει τα Arduino sketches για το πρώτο bench bring-up του
collector:

- TB6612FNG motor driver
- GB37Y3530 encoder motor
- 2x Adafruit IR break beam sensors

Άνοιξε κάθε `.ino` από το Arduino IDE μέσα από τον αντίστοιχο φάκελο.

## Pin Map

| Arduino Nano Pin | Χρήση |
|---|---|
| D2 | Encoder A |
| D3 | TB6612 PWMA |
| D4 | TB6612 AIN1 |
| D5 | TB6612 AIN2 |
| D6 | TB6612 STBY |
| D7 | Encoder B |
| D9 | IR Break Beam #1 receiver signal |
| D10 | IR Break Beam #2 receiver signal |
| 5V | TB6612 VCC, encoder VCC, IR sensors VCC |
| GND | Common ground |

## GB37Y3530 Motor Cable Colors

| Χρώμα καλωδίου | Αντιστοιχεί σε | Σύνδεση σε αυτό το prototype |
|---|---|---|
| Κόκκινο | Motor lead | TB6612 `AO1` |
| Μαύρο | Motor lead | TB6612 `AO2` |
| Μπλε | Encoder A | Arduino Nano `D2` |
| Πράσινο | Encoder B | Arduino Nano `D7` |
| Κίτρινο | Encoder VCC | `5V` |
| Λευκό | Encoder GND | `GND` |

Αν ο roller γυρίζει ανάποδα, άλλαξε τη φορά στο software ή αντάλλαξε τα δύο
motor leads `AO1`/`AO2`. Μην αλλάξεις τα καλώδια του encoder για να διορθώσεις
τη φορά του μοτέρ.

## Προτεινόμενη Σειρά Δοκιμών

1. `01_ir_beam_test`
   - Επιβεβαιώνει ότι τα 2 IR receivers διαβάζονται σωστά.
   - Δεν χρειάζεται 12V στο `VM`.

2. `02_encoder_hand_test`
   - Επιβεβαιώνει ότι ο encoder δίνει pulses όταν γυρίζεις τον άξονα με το χέρι.
   - Δεν χρειάζεται να κινείται το μοτέρ από τον driver.

3. `03_motor_tb6612_test`
   - Επιβεβαιώνει ότι ο TB6612 γυρίζει το μοτέρ εμπρός/πίσω.
   - Χρειάζεται `+12V -> VM` και κοινό `GND`.
   - Ξεκινά με χαμηλό PWM για να προστατεύσει τον driver.

4. `04_collector_smoke_test`
   - Τρέχει motor + encoder + 2 IR beams μαζί.
   - Είναι το πρώτο ολοκληρωμένο smoke test του collector.

5. `05_wiring_diagnostic`
   - Δείχνει στο Serial Monitor αν φαίνονται σωστά IR, encoder και TB6612 pin map.
   - Έχει εντολή `p` για σύντομο pulse 300 ms στο τρέχον PWM, ώστε να ελέγξεις `VM`, κοινό `GND`, `AO1`/`AO2`, `PWMA`, `AIN1`, `AIN2` και `STBY`.
   - Η hardware δοκιμή που πέρασε στις 28/06/2026 χρησιμοποίησε Serial `9600 baud`, `START_PWM=255` και `MAX_SAFE_PWM=255`.
   - Ξεκίνα με `i`, μετά σπάσε κάθε IR beam, γύρισε τον άξονα με το χέρι, και τέλος δοκίμασε `p`.

6. `06_motor_driver_wiring_check`
   - Ελέγχει μόνο Arduino Nano -> TB6612FNG -> motor, χωρίς IR και encoder.
   - Ξεκίνα με την εντολή `v` και μέτρα `VCC=5V`, `VM=12V`, κοινό `GND`.
   - Μετά δοκίμασε `p` για ένα πολύ σύντομο χαμηλό-PWM pulse.

## Πριν Βάλεις 12V Στο VM

Μέτρα με πολύμετρο πάνω στα pins του TB6612:

| Μέτρηση | Αναμενόμενο |
|---|---|
| `VCC` προς `GND` | περίπου 5V |
| `VM` προς `GND` | περίπου 12V |
| `Arduino GND` προς `TB6612 GND` | συνέχεια / σχεδόν 0 ohm |
| `AO1` και `AO2` | μόνο προς τα δύο καλώδια του μοτέρ |

Αν δεις 12V στο `VCC`, σταμάτα. Το `VCC` είναι μόνο για 5V λογικής.

## Serial Monitor

Για τα sketches `01` έως `05`:

```text
Baud rate: 115200
Line ending: No line ending
```

Για το hardware-validated `06_motor_driver_wiring_check`:

```text
Baud rate: 9600
Line ending: No line ending
```

## IR Logic

Τα Adafruit IR break beam receivers είναι open-collector και τα sketches
χρησιμοποιούν:

```cpp
pinMode(pin, INPUT_PULLUP);
```

Με αυτό το wiring:

```text
LOW  = beam broken / blocked
HIGH = beam unbroken / receiver sees emitter
```

## Motor Driver Προσοχή

Ο TB6612FNG είναι μικρός driver για το GB37Y3530, ειδικά αν ο roller κολλήσει.
Τα motor tests χρησιμοποιούν χαμηλό PWM. Αν ο driver ζεσταθεί, κάνει reset, ή το
μοτέρ τραβήξει απότομα, σταμάτα το test.
