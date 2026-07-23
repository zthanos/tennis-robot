# Hardware Glossary

## VM (Motor Voltage)

Τροφοδοσία ισχύος του driver.

* Συνήθως 6–15V (ανάλογα με τον driver)
* Πηγαίνει στα μοτέρ
* Στον collector: **12V**

---

## VCC (Logic Voltage)

Τροφοδοσία λογικής του chip.

* Συνήθως 5V ή 3.3V
* Δεν τροφοδοτεί το μοτέρ
* Τροφοδοτεί τα εσωτερικά κυκλώματα του driver

Στον TB6612:

**VCC = 5V**

---

## GND (Ground)

Κοινή αναφορά τάσης.

Όλες οι συσκευές πρέπει να μοιράζονται το ίδιο GND.

```text
Arduino GND
TB6612 GND
12V PSU -
Buck GND
IR GND
```

Όλα συνδέονται μαζί.

---

## PWM (Pulse Width Modulation)

Έλεγχος ισχύος κινητήρα.

0
→ Motor OFF

255
→ Full Speed

Δεν αλλάζει την τάση.
Αλλάζει τον χρόνο που η τάση είναι ενεργή.

---

## STBY (Standby)

Enable pin του TB6612.

LOW
→ Driver OFF

HIGH
→ Driver ON

---

## AIN1 / AIN2

Direction pins.

HIGH / LOW
→ Forward

LOW / HIGH
→ Reverse

LOW / LOW
→ Coast

HIGH / HIGH
→ Brake

---

## AO1 / AO2

Έξοδος προς το μοτέρ.

Δεν συνδέεται τίποτε άλλο εδώ.

---

## Buck Converter

Μετατρέπει υψηλότερη τάση σε χαμηλότερη.

Παράδειγμα:

12V

↓

5V

Χρησιμοποιείται για Arduino και αισθητήρες.

---

## Perfboard

Διάτρητη πλακέτα για μόνιμη συναρμολόγηση.

Πιο αξιόπιστη από breadboard.

---

## Breadboard

Πλακέτα δοκιμών.

Δεν απαιτεί κόλληση.

Ιδανική για prototype.

---

## H-Bridge

Κύκλωμα που επιτρέπει:

* Forward
* Reverse
* Brake
* PWM

Ο TB6612 είναι H-Bridge.

---

## Driver

Το κύκλωμα που οδηγεί ένα φορτίο.

Παράδειγμα:

Arduino

↓

TB6612 (Driver)

↓

Motor

---

## Controller

Το firmware που ελέγχει το hardware.

Στο project:

Arduino = Controller

Python = High-Level Controller

---

## MCU (Microcontroller Unit)

Μικρός υπολογιστής πραγματικού χρόνου.

Παράδειγμα:

Arduino Nano

ESP32

STM32

---

## Logic Power

Η τροφοδοσία των ηλεκτρονικών.

Συνήθως:

5V

---

## Motor Power

Η τροφοδοσία των κινητήρων.

Στο robot:

12V
