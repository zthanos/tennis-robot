# Collector Motor Driver Test Cheat Sheet

Validated on `28/06/2026` with:

```text
Arduino Nano -> TB6612FNG -> 12V collector motor
D3 -> PWMA
D4 -> AIN1
D5 -> AIN2
D6 -> STBY
5V -> VCC
12V -> VM
common GND between Arduino, TB6612 and 12V PSU
```

Upload:

```text
arduino/collector/06_motor_driver_wiring_check/06_motor_driver_wiring_check.ino
```

Serial Monitor:

```text
9600 baud
No line ending
```

Tested sketch parameters:

```text
START_PWM = 255
MAX_SAFE_PWM = 255
PULSE_MS = 300
```

Commands:

| Command | Action |
|---|---|
| `v` | Print voltage/wiring checklist |
| `t` | Toggle TB6612 standby |
| `p` | Forward pulse for 300 ms, then coast |
| `f` | Forward at current PWM |
| `r` | Reverse at current PWM |
| `s` | Stop/coast |
| `b` | Brake command |
| `+` / `-` | Adjust PWM by 10 |
| `?` | Print help |

Validated scope:

- Nano serial command handling
- TB6612 `PWMA`, `AIN1`, `AIN2`, and `STBY` control path
- 12V motor output through `AO1/AO2`
- common-ground wiring

Not validated by this sketch:

- encoder
- IR break-beam sensors
- closed-loop collector behavior
