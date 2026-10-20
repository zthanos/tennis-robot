# Rear pod — ενεργό v2 / P2-84

Η μοναδική ενεργή print release είναι:

- πηγή `rear-pod-v2.scad` + `params-v2.scad`,
- STL στο `stl-v2/`,
- πλήρεις οδηγίες/βίδες στο `stl-v2/README.md`,
- συνολικό manifest στο `../ACTIVE-PRINT-SET.md`.

Το P2-83 αντικατέστησε το σπασμένο corner tie με μονοκόμματη εσωτερική
L-γωνία κάτω από τη βάση, με flush φρεζάτη M4 και την υπάρχουσα πλευρική
M4 του corner clamp. Το P2-81 παραμένει ενεργό για το ribbed control panel
και τους επίπεδους Ø23 ώμους posts. Τα `stl-v2/mpu6050_mount.stl` και
`chassis_spacers.stl` δεν είναι ενεργά και έχουν μεταφερθεί στο `archive/`.

Το P2-84 πρόσθεσε μόνο interface features εντός του ίδιου envelope: δύο
rear-facing M3 captive-nut lands ανά μισή βάση, δύο αντίστοιχα lands κάτω
από το πίσω χείλος του control panel και οπή Ø8 για status LED στο local
`X+42/Y+70`. Οι control οπές και όλες οι θέσεις electronics/chassis έμειναν
αμετάβλητες. Τα panels βρίσκονται στο `../body-panels/stl-f-rear/` και έχουν
σήμανση **ΔΕΝ ΤΥΠΩΝΕΤΑΙ ΑΚΟΜΑ**.

Το P2-80 είναι μόνο μελλοντικό concept για πλήρως τυπωμένο stackable PETG
cage. Δεν έχει STL release· οι θέσεις κολονών θα ξανακριθούν με τα panels F.

Οι εκδόσεις v0/v1, τα παλιά review-only proposals και τα ανενεργά STL
βρίσκονται στο `archive/`. **ΜΗΝ ΤΥΠΩΣΕΙΣ** αρχεία από εκεί.
