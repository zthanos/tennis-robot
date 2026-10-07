# BTS7960 control plug v2 — individual M–F jumper wires

This **fit prototype** replaces the single soldered 2×4 female PCB header of
v1 with six separate female ends of 15 cm male-to-female jumper wires. Their
male ends remain free to plug into female sockets soldered on the perfboard.
The printed two-piece driver shell packs the female ends into a 2×4 grid,
uses a rear comb to keep each wire in its intended column, and closes with
the same two M3 bolts and external nuts as v1. Two through-bored printed
dummy pieces occupy the unused current-sense positions so the real female
ends cannot shift into them.

Assumed orientation, **viewed from the mating face with the `VCC` mark on the
upper shell**:

| Row | Col 0 | Col 1 | Col 2 | Col 3 |
| --- | --- | --- | --- | --- |
| VCC-marked | VCC | R_IS dummy | R_EN | RPWM |
| Other | GND | L_IS dummy | L_EN | LPWM |

The pin labels/orientation must be checked against the silkscreen of the
*actual* driver board before wiring or powering it. The v2 shell still has
**no mechanical anti-reversal key** on the driver: its bare 2×4 pin grid is
symmetric. The `VCC` text is an assembly aid, not reverse-polarity protection.

## What changes electrically

There are now **six jumpers per driver**, not the previous five-wire soldered
harness. `R_EN` and `L_EN` are independent cables in the driver plug. Their
two male ends must land on two female perfboard sockets connected to the
**same EN control net**. Do not connect either EN socket to 5 V directly.
The other four jumpers land at VCC, GND, RPWM and LPWM. For four drivers that
means 24 jumpers, or three packs of ten if using the linked 10-piece pack.

## Required fit checks

The seller's listing does not give the plastic dimensions of the individual
female ends. The CAD uses a provisional **2.54 × 2.54 × 14 mm** body and
0.40 mm total pack clearance. `wire_channel_x=1.50 mm` and the rear cable
opening are also provisional. Do not print four complete plugs before
checking one actual jumper end against one printed shell.

1. Print one `plug-lower-v2.stl`, one `plug-upper-v2.stl` and **two copies**
   of `plug-dummy-v2.stl`. The original v1 driver-pin gauge already fit; the
   v2 `plug-fit-gauge-v2.stl` is supplied only for an optional confirmation.
2. With the driver fully disconnected from power, place each female plastic
   end in its assigned pocket and lay its preterminated wire into the comb
   channel. The two printed dummies go in column 1, one in each row. The
   male ends do not need to be threaded through the housing because the
   shell opens along the cable path.
3. Close the halves without forcing them, secure with M3 bolts, and provide
   cable strain relief. Check that no female end can retreat, rotate or swap
   columns when handling the cable bundle. If it can, revise the pocket
   dimensions; do not solve it by jamming the shell onto the driver pins.
4. Verify all six circuits, the two EN perfboard sockets' common connection,
   and isolation of both IS positions with a continuity meter. Only then
   consider connecting the Mega and motor power.

This is for the low-current **logic header only**, never the driver's motor
power terminals. Loose Dupont ends and a printed shell have not been
vibration-qualified. Use a proven locking connector for a permanent robot if
the handling test does not securely retain the individual ends.
