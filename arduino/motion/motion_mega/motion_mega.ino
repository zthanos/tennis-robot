/*
 * motion_mega.ino - Tennis robot DRIVE motion MCU (Arduino Mega 2560)
 * ---------------------------------------------------------------------------
 * 4WD skid-steer. Two BTS7960 drivers (one per side), four encoders.
 * The Mega is the real-time / safety layer; the host (PC now, Pi later) sends
 * high-level per-side duty over USB serial. Same protocol works on PC and Pi
 * (USB CDC is host-agnostic).
 *
 * SAFETY MODEL
 *   - Boots DISARMED: drivers disabled (EN low), duty 0.
 *   - ARM only if E-stop status not tripped.
 *   - Command timeout: if no command/heartbeat within CMD_TIMEOUT_MS, duty -> 0
 *     (stays armed, but motors stop). Host must stream heartbeats.
 *   - PWM ramp (slew-rate limit) toward target -> avoids current spikes that
 *     would trip the battery BMS during skid-steer turns.
 *   - E-stop status (active-low, optional aux contact) forces DISARM.
 *
 * SERIAL PROTOCOL  (115200 baud, '\n'-terminated ASCII lines)
 *   Host -> Mega:
 *     ARM                 arm the drivers (if E-stop ok)
 *     DISARM              disarm + stop
 *     M <left> <right>    per-side duty, floats in [-1.0, 1.0]  (also a heartbeat)
 *     STOP                target duty 0 (stays armed)           (also a heartbeat)
 *     PING                heartbeat only; Mega replies "PONG"
 *   Mega -> Host (telemetry, every TELEM_MS):
 *     T <state> <Lduty> <Rduty> <encLF> <encLR> <encRF> <encRR> <estop>
 *       state: 0=DISARMED 1=ARMED 2=ESTOP
 *       estop: 1=tripped 0=ok
 *   Mega -> Host (acks): "OK ARM", "OK DISARM", "ERR <reason>", "PONG"
 *
 * PIN MAP — see docs/motion-perfboard-wiring-el.md §4.
 */

#include <Arduino.h>

// ---- BTS7960 control pins -------------------------------------------------
const uint8_t LEFT_RPWM  = 5;   // D5  PWM  (left forward)
const uint8_t LEFT_LPWM  = 6;   // D6  PWM  (left reverse)
const uint8_t LEFT_EN    = 30;  // D30      (left R_EN + L_EN tied)
const uint8_t RIGHT_RPWM = 9;   // D9  PWM  (right forward)
const uint8_t RIGHT_LPWM = 10;  // D10 PWM  (right reverse)
const uint8_t RIGHT_EN   = 31;  // D31      (right R_EN + L_EN tied)

// ---- Encoder pins (A on external-interrupt pins) --------------------------
const uint8_t ENC_LF_A = 2,  ENC_LF_B = 22;
const uint8_t ENC_LR_A = 3,  ENC_LR_B = 23;
const uint8_t ENC_RF_A = 18, ENC_RF_B = 24;
const uint8_t ENC_RR_A = 19, ENC_RR_B = 25;

// ---- Safety / user inputs (active-low, INPUT_PULLUP) ----------------------
const uint8_t START_ARM_PIN    = 32;  // optional arm button
const uint8_t ESTOP_STATUS_PIN = 33;  // optional E-stop aux contact

// ---- Tuning ---------------------------------------------------------------
const unsigned long BAUD          = 115200;
const unsigned long CMD_TIMEOUT_MS = 300;   // no heartbeat -> stop
const unsigned long CONTROL_MS     = 10;    // control loop period
const unsigned long TELEM_MS       = 100;   // telemetry period
const float  RAMP_PER_TICK = 0.02f;         // duty step per CONTROL_MS (~0.5s 0->full)
const float  DEADBAND      = 0.02f;         // below this, treat as zero

// ---- State ----------------------------------------------------------------
enum State { DISARMED = 0, ARMED = 1, ESTOPPED = 2 };
State state = DISARMED;

float targetLeft = 0.0f, targetRight = 0.0f;   // requested [-1,1]
float curLeft = 0.0f, curRight = 0.0f;         // ramped actual [-1,1]
unsigned long lastCmdMs = 0, lastControlMs = 0, lastTelemMs = 0;

volatile long encLF = 0, encLR = 0, encRF = 0, encRR = 0;

char rxBuf[48];
uint8_t rxLen = 0;

// ---- Encoder ISRs (count on A edge, direction from B) ---------------------
void isrLF() { encLF += (digitalRead(ENC_LF_B) ? 1 : -1); }
void isrLR() { encLR += (digitalRead(ENC_LR_B) ? 1 : -1); }
void isrRF() { encRF += (digitalRead(ENC_RF_B) ? 1 : -1); }
void isrRR() { encRR += (digitalRead(ENC_RR_B) ? 1 : -1); }

// ---- Helpers --------------------------------------------------------------
bool estopTripped() { return digitalRead(ESTOP_STATUS_PIN) == LOW; }

void setDriverEnable(bool en) {
  digitalWrite(LEFT_EN,  en ? HIGH : LOW);
  digitalWrite(RIGHT_EN, en ? HIGH : LOW);
}

void applySide(uint8_t rpwm, uint8_t lpwm, float duty) {
  if (duty > DEADBAND) {
    analogWrite(rpwm, (uint8_t)(duty * 255.0f));
    analogWrite(lpwm, 0);
  } else if (duty < -DEADBAND) {
    analogWrite(rpwm, 0);
    analogWrite(lpwm, (uint8_t)(-duty * 255.0f));
  } else {
    analogWrite(rpwm, 0);
    analogWrite(lpwm, 0);
  }
}

void stopOutputs() {
  curLeft = curRight = 0.0f;
  targetLeft = targetRight = 0.0f;
  applySide(LEFT_RPWM, LEFT_LPWM, 0.0f);
  applySide(RIGHT_RPWM, RIGHT_LPWM, 0.0f);
}

void doDisarm(const char* reason) {
  stopOutputs();
  setDriverEnable(false);
  state = (estopTripped() ? ESTOPPED : DISARMED);
  Serial.print(F("OK DISARM "));
  Serial.println(reason);
}

void doArm() {
  if (estopTripped()) { state = ESTOPPED; Serial.println(F("ERR estop")); return; }
  stopOutputs();
  setDriverEnable(true);
  state = ARMED;
  lastCmdMs = millis();   // grace period for first heartbeat
  Serial.println(F("OK ARM"));
}

float clampUnit(float v) { return v < -1.0f ? -1.0f : (v > 1.0f ? 1.0f : v); }

float rampToward(float cur, float tgt) {
  if (cur < tgt) { cur += RAMP_PER_TICK; if (cur > tgt) cur = tgt; }
  else if (cur > tgt) { cur -= RAMP_PER_TICK; if (cur < tgt) cur = tgt; }
  return cur;
}

// ---- Serial command parser ------------------------------------------------
void handleLine(char* line) {
  // tokenize
  if (strncmp(line, "M ", 2) == 0) {
    float l = 0, r = 0;
    if (sscanf(line + 2, "%f %f", &l, &r) == 2) {
      targetLeft  = clampUnit(l);
      targetRight = clampUnit(r);
      lastCmdMs = millis();
    } else {
      Serial.println(F("ERR parse"));
    }
  } else if (strcmp(line, "STOP") == 0) {
    targetLeft = targetRight = 0.0f;
    lastCmdMs = millis();
  } else if (strcmp(line, "ARM") == 0) {
    doArm();
  } else if (strcmp(line, "DISARM") == 0) {
    doDisarm("host");
  } else if (strcmp(line, "PING") == 0) {
    lastCmdMs = millis();
    Serial.println(F("PONG"));
  } else if (line[0] != '\0') {
    Serial.println(F("ERR cmd"));
  }
}

void pollSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxLen > 0) { rxBuf[rxLen] = '\0'; handleLine(rxBuf); rxLen = 0; }
    } else if (rxLen < sizeof(rxBuf) - 1) {
      rxBuf[rxLen++] = c;
    } else {
      rxLen = 0;  // overflow -> drop line
    }
  }
}

void sendTelemetry() {
  noInterrupts();
  long lf = encLF, lr = encLR, rf = encRF, rr = encRR;
  interrupts();
  Serial.print(F("T "));
  Serial.print((int)state);          Serial.print(' ');
  Serial.print(curLeft, 3);          Serial.print(' ');
  Serial.print(curRight, 3);         Serial.print(' ');
  Serial.print(lf); Serial.print(' ');
  Serial.print(lr); Serial.print(' ');
  Serial.print(rf); Serial.print(' ');
  Serial.print(rr); Serial.print(' ');
  Serial.println(estopTripped() ? 1 : 0);
}

void setup() {
  Serial.begin(BAUD);

  pinMode(LEFT_RPWM, OUTPUT);  pinMode(LEFT_LPWM, OUTPUT);  pinMode(LEFT_EN, OUTPUT);
  pinMode(RIGHT_RPWM, OUTPUT); pinMode(RIGHT_LPWM, OUTPUT); pinMode(RIGHT_EN, OUTPUT);

  pinMode(ENC_LF_A, INPUT_PULLUP); pinMode(ENC_LF_B, INPUT_PULLUP);
  pinMode(ENC_LR_A, INPUT_PULLUP); pinMode(ENC_LR_B, INPUT_PULLUP);
  pinMode(ENC_RF_A, INPUT_PULLUP); pinMode(ENC_RF_B, INPUT_PULLUP);
  pinMode(ENC_RR_A, INPUT_PULLUP); pinMode(ENC_RR_B, INPUT_PULLUP);

  pinMode(START_ARM_PIN, INPUT_PULLUP);
  pinMode(ESTOP_STATUS_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_LF_A), isrLF, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_LR_A), isrLR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RF_A), isrRF, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RR_A), isrRR, RISING);

  // Boot DISARMED: drivers off, outputs zero.
  setDriverEnable(false);
  stopOutputs();
  state = DISARMED;
  Serial.println(F("READY motion_mega DISARMED"));
}

void loop() {
  pollSerial();

  unsigned long now = millis();

  // E-stop has absolute priority.
  if (estopTripped() && state != ESTOPPED) {
    doDisarm("estop");
    state = ESTOPPED;
  }
  // Recover from ESTOPPED to DISARMED once contact clears (still needs ARM).
  if (state == ESTOPPED && !estopTripped()) {
    state = DISARMED;
  }

  // Control loop: timeout + ramp + output.
  if (now - lastControlMs >= CONTROL_MS) {
    lastControlMs = now;

    if (state == ARMED) {
      // Command timeout -> coast target to zero (stay armed).
      if (now - lastCmdMs > CMD_TIMEOUT_MS) {
        targetLeft = targetRight = 0.0f;
      }
      curLeft  = rampToward(curLeft,  targetLeft);
      curRight = rampToward(curRight, targetRight);
      applySide(LEFT_RPWM,  LEFT_LPWM,  curLeft);
      applySide(RIGHT_RPWM, RIGHT_LPWM, curRight);
    } else {
      // Not armed: ensure outputs are zero and drivers disabled.
      stopOutputs();
      setDriverEnable(false);
    }
  }

  if (now - lastTelemMs >= TELEM_MS) {
    lastTelemMs = now;
    sendTelemetry();
  }
}
