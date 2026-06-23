# Motion MCU (Arduino Mega) — drive firmware

`motion_mega/motion_mega.ino` — real-time / safety layer for the 4WD skid-steer
drive. Two BTS7960 drivers (one per side), four encoders. The host (PC now, Pi
later) sends high-level per-side duty over USB serial; the Mega handles arming,
ramp, timeout and E-stop. USB serial is host-agnostic, so what you validate on
the PC transfers unchanged to the Pi.

## Bench test (motors → PC)

1. **Power:** drive motors need a strong 12V supply (or the battery) — **NOT**
   the 12V/2A collector PSU (4 motors stall ~28A). USB only powers Arduino logic.
2. **Common ground:** Arduino GND + both BTS7960 GND + 12V PSU GND tied together.
3. **Wheels OFF the ground** for first runs.
4. Flash `motion_mega.ino` to the Mega.
5. Run the host test:
   ```bash
   uv run python scripts/test_motion_serial.py --port /dev/ttyACM0   # or COM5 on Windows
   ```
6. `arm` → `f 0.2` (forward, low duty) → watch encoder counts move. `stop`,
   then test `b`, `l`, `r`. Confirm each side's direction + encoder sign.
   Do **one side at a time** first: `m 0.2 0` then `m 0 0.2`.

## Serial protocol (115200, `\n`-terminated)

| Host → Mega | Meaning |
|---|---|
| `ARM` | enable drivers (only if E-stop ok) |
| `DISARM` | disable + stop |
| `M <left> <right>` | per-side duty, floats `[-1.0, 1.0]` (also a heartbeat) |
| `STOP` | target duty 0, stays armed (also a heartbeat) |
| `PING` | heartbeat only → replies `PONG` |

| Mega → Host | Meaning |
|---|---|
| `T <state> <Lduty> <Rduty> <encLF> <encLR> <encRF> <encRR> <estop>` | telemetry @10Hz |
| `OK ARM` / `OK DISARM <reason>` / `ERR <reason>` / `PONG` / `READY ...` | acks |

state: `0`=DISARMED `1`=ARMED `2`=ESTOP · estop: `1`=tripped `0`=ok

## Safety behaviour

- Boots **DISARMED** (drivers off, duty 0).
- `ARM` refused while E-stop tripped; a trip mid-run forces DISARM.
- **Command timeout** (`CMD_TIMEOUT_MS`, 300 ms): no heartbeat → duty ramps to 0
  (stays armed). The host script streams `M`/`PING` at 20 Hz.
- **PWM ramp** (`RAMP_PER_TICK`): ~0.5 s from 0→full, limiting current spikes
  that would otherwise trip the battery BMS in skid-steer turns.

## Pin map

Matches `docs/motion-perfboard-wiring-el.md §4`:

| Function | Mega pin |
|---|---|
| LEFT_RPWM / LPWM / EN | D5 / D6 / D30 |
| RIGHT_RPWM / LPWM / EN | D9 / D10 / D31 |
| Encoders A (interrupt) | LF=D2, LR=D3, RF=D18, RR=D19 |
| Encoders B | LF=D22, LR=D23, RF=D24, RR=D25 |
| START_ARM / ESTOP_STATUS | D32 / D33 (active-low, pullup) |
| IMU I2C (not used in v1) | SDA=D20, SCL=D21 |

## Notes / next steps

- v1 counts encoders on the A-edge only (direction from B). Fine for bench
  direction/odometry sanity; full 4× quadrature can come later if needed.
- The IMU (MPU6050) is wired to the Mega I2C but not read in v1.
- Later, a host node converts ROS `/cmd_vel` (Twist) → per-side duty and sends
  `M left right` — keeps the Mega dumb + safe. See `motion_controller.py`.
