# P2-93h — PARKED generated reference tables

**PARKED — depends on the real moving module.** ANALYSIS ONLY; proposed contact/sections, NOT a new-solid gate PASS or authorization to design/print/order.

For future screens using the existing 8 N·m and 10/5 MPa bounds, margin >=1.0 is PASS. Do not add a higher target without evidence; borderline cases go to physical test.

| Existing element | LC1 with200g fixed budget | Proposed actual-contact LC2 margin |
|---|---:|---:|
| knee | 1.506 | 1.689 |
| M4_Z80_114_thread_8_8 | 1.144 | 1.385 |
| foot_cheek_bearing | 1.008 | 1.221 |
| foot_cheek_edge | 1.686 | 2.238 |
| knee_bearing | 1.210 | 1.465 |
| rail_pad_pitch | 1.039 | 1.322 |
| foot_root_roll | 2.669 | 5.782 |
| foot_bridge_torsion | 4.922 | 10.662 |
| base_M4_8_8 | 11.412 | 15.359 |
| PETG_rail_OD14 | 1.321 | 1.645 |
| PETG_inboard_OD9 | 1.784 | 2.092 |
| PETG_pin_head_OD10 | 1.286 | 1.286 |
| PETG_pin_nut_OD12 | 3.014 | 3.014 |
| PETG_crown_washer | 2.350 | 0.930 |
| pitch_contact | 1.122 | 1.483 |
| console_stem | 5.578 | 93.550 |
| cradle_B_bearing | 3.818 | 46.845 |
| cradle_bridge_spine | 1.365 | 17.403 |
| cradle_B_boss_net | 1.111 | 14.131 |
| console_B_ear_bearing | 1.113 | 13.663 |
| console_fork_leg | 2.443 | 29.978 |
| nut_clevis_bearing | 1.622 | 89.543 |
| nut_arm_riser | 1.241 | 48.806 |
| nut_arm_riser_torsion | 2.141 | 101.754 |
| nut_arm_shelf | 1.828 | 71.898 |
| nut_arm_shelf_torsion_U | 6.636 | 260.490 |
| nut_arm_shelf_torsion_Y | 3.215 | 152.821 |
| nut_arm_fork_legs | 1.174 | 50.566 |
| nut_arm_bight | 2.660 | 104.571 |
| nut_arm_bight_torsion | 1.477 | 70.202 |
| nut_trunnion_plate | 2.204 | 121.665 |
| arm_yoke_OD10_washer | 1.217 | 3.073 |
| arm_yoke_M4_8_8 | 32.449 | 6744.833 |
| arm_base_plate | 1.197 | 212.942 |
| cradle_front_plate | 2.557 | 58.615 |
| crown_inboard_lug_bearing | 6.589 | 1.572 |
| crown_lug_local_biaxial | 4.259 | 1.016 |
| crown_pin_thread_10_9 | 4.220 | 1.007 |

| Proposed local section | Margin |
|---|---:|
| striker_40x12_net_M4_root | 1.674 |
| striker_torsion | 3.917 |
| console_20x24_net_root | 3.045 |
| pad_effective_12x2_CONDITIONAL | 2.791 |
| striker_mount_OD12 | 1.943 |
| crown_seat_20x20 | 46.519 |
| larger_crown_OD12_seat_axial_bound | 1.909 |

Crown partial-thread8.8 alternative: measured smooth grip>=22mm, d>=3.8, combined bending/shear +FULL q/8.78 axial bound: margin 1.202. NOT a generic fully-threaded8.8 swap.

## Print planning (slicing failed; no G-code estimate)

All nine models100% solid for conservative procurement; volumes exclude supports/purge/brim. Effective4–7mm³/s plus12min per separate print, NOT printer prediction.

| STL | Qty | Solid g/each | Planning h/each |
|---|---:|---:|---:|
| p293g_console_inboard.stl | 1 | 106.0 | 3.51–6.00 |
| p293g_console_outboard.stl | 1 | 65.5 | 2.25–3.78 |
| p293g_cradle.stl | 1 | 106.7 | 3.53–6.03 |
| p293g_crossbar_end.stl | 2 | 44.7 | 1.60–2.65 |
| p293g_crown.stl | 2 | 61.4 | 2.12–3.56 |
| p293g_float_cap.stl | 1 | 5.5 | 0.37–0.50 |
| p293g_foot.stl | 2 | 159.7 | 5.19–8.93 |
| p293g_nut_arm.stl | 1 | 88.7 | 2.97–5.05 |
| p293g_nut_trunnion.stl | 1 | 11.7 | 0.57–0.84 |

Total 915.6g, 31.01–52.46h / 12 separate prints; consumables/supports additional.
