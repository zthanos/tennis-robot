/*
 * 07_dual_intake_mega_bench.ino
 * --------------------------------
 * Static bench bring-up for:
 *   Arduino Mega 2560 -> 2x BTS7960 -> 2x DFRobot FIT0186 intake motors
 *   2x motor encoders and optional entry/exit IR break beams
 *
 * This sketch deliberately does NOT drive the 4WD base. Disconnect or lift the
 * drive system before uploading it. The selected pins do not overlap the
 * motion pin map, so they can later be merged into motion_mega firmware.
 *
 * Each BTS7960 drives ONE FIT0186. Motor 12 V and current stay off the Mega.
 * This sketch caps PWM and burst duration, but software cannot limit motor
 * current. Use a current-limited 12 V bench supply and a physical E-stop.
 * Do not perform stall/jam endurance testing.
 *
 * Serial: 115200 baud, newline terminated
 *   HELP
 *   ARM
 *   DISARM
 *   STOP
 *   ZERO
 *   STATUS
 *   DIST <beam_spacing_mm>
 *   PULSE L <signed_pwm> <duration_ms>
 *   PULSE R <signed_pwm> <duration_ms>
 *   RUN <pwm> <duration_ms>       (both wheels inward)
 *   RUN_BENCH <pwm> <duration_ms> (both inward; encoder check disabled)
 *   AUTO <pwm> [timeout_ms]       (one ball cycle using IR beams)
 *   AUTO_BENCH <pwm> [timeout_ms] (IR/timeout test; ignores encoders)
 *
 * Limits: |pwm| <= 90, manual duration <= 1500 ms, AUTO <= 4000 ms.
 */

#include <Arduino.h>
#include <avr/interrupt.h>

// Intake-only pins; drive uses D5/D6/D9/D10 and D30/D31.
const uint8_t LEFT_RPWM_PIN = 44;
const uint8_t LEFT_LPWM_PIN = 45;
const uint8_t LEFT_EN_PIN = 40;   // both R_EN and L_EN on left BTS7960
const uint8_t RIGHT_RPWM_PIN = 46;
const uint8_t RIGHT_LPWM_PIN = 11;
const uint8_t RIGHT_EN_PIN = 42;  // both R_EN and L_EN on right BTS7960

// FIT0186 encoders. A8-A11 are ATmega2560 PCINT pins (port K).
// As wired: left on A10/A11, right on A8/A9.
const uint8_t LEFT_ENC_A_PIN = A10;   // PK2 / PCINT18
const uint8_t LEFT_ENC_B_PIN = A11;   // PK3
const uint8_t RIGHT_ENC_A_PIN = A8;   // PK0 / PCINT16
const uint8_t RIGHT_ENC_B_PIN = A9;   // PK1

// Optional Adafruit-style open-collector IR break beams, active LOW.
const uint8_t IR_ENTRY_PIN = 36;
const uint8_t IR_EXIT_PIN = 37;

// Reuse the motion safety nets when the common Mega harness is fitted.
const uint8_t ESTOP_STATUS_PIN = 33;  // active LOW, INPUT_PULLUP
const uint8_t ARMED_LED_PIN = 34;

const unsigned long BAUD = 115200;
const uint8_t MAX_INTAKE_TEST_PWM = 90;
const uint8_t MIN_JAM_CHECK_PWM = 35;
const unsigned long MAX_BURST_MS = 1500;
const unsigned long DEFAULT_AUTO_BALL_MS = 4000;
const unsigned long MAX_AUTO_BALL_MS = 4000;
const unsigned long STARTUP_GRACE_MS = 300;
const unsigned long NO_ENCODER_PROGRESS_MS = 250;
const unsigned long TELEMETRY_MS = 100;
const unsigned long IR_DEBOUNCE_MS = 25;
const float ENCODER_COUNTS_PER_OUTPUT_REV = 700.8f;

// Change either sign only after a wheel-off direction check.
const int8_t LEFT_INWARD_SIGN = -1;
const int8_t RIGHT_INWARD_SIGN = 1;

enum BenchState : uint8_t {
  DISARMED = 0,
  ARMED_IDLE = 1,
  RUNNING = 2,
  WAITING_FOR_BALL = 3,
  FAULTED = 4
};

enum RunMode : uint8_t {
  RUN_NONE = 0,
  RUN_TIMED = 1,
  RUN_AUTO_BALL = 2,
  RUN_AUTO_BENCH = 3
};

struct DebouncedBeam {
  uint8_t pin;
  bool rawBroken;
  bool stableBroken;
  unsigned long rawChangedMs;
};

volatile int32_t leftEncoderCount = 0;
volatile int32_t rightEncoderCount = 0;
volatile bool previousLeftA = false;
volatile bool previousRightA = false;

BenchState state = DISARMED;
RunMode runMode = RUN_NONE;
DebouncedBeam entryBeam = {IR_ENTRY_PIN, false, false, 0};
DebouncedBeam exitBeam = {IR_EXIT_PIN, false, false, 0};

int16_t commandedLeftPwm = 0;
int16_t commandedRightPwm = 0;
uint8_t autoPwm = 0;
unsigned long autoTimeoutMs = DEFAULT_AUTO_BALL_MS;
unsigned long runStartedMs = 0;
unsigned long runDurationMs = 0;
unsigned long lastTelemetryMs = 0;
unsigned long lastProgressCheckMs = 0;
unsigned long ballEntryMs = 0;
bool exitWasReached = false;
float beamSpacingMm = 0.0f;
int32_t lastProgressLeftCount = 0;
int32_t lastProgressRightCount = 0;
int32_t lastTelemetryLeftCount = 0;
int32_t lastTelemetryRightCount = 0;

char rxBuffer[72];
uint8_t rxLength = 0;

ISR(PCINT2_vect) {
  const uint8_t pins = PINK;
  const bool leftA = (pins & _BV(2)) != 0;
  const bool rightA = (pins & _BV(0)) != 0;

  // Count only rising A edges. Read B for direction. This matches the
  // manufacturer's approximately 700 counts/output-revolution convention.
  if (leftA && !previousLeftA) {
    leftEncoderCount += ((pins & _BV(3)) != 0) ? 1 : -1;
  }
  if (rightA && !previousRightA) {
    rightEncoderCount += ((pins & _BV(1)) != 0) ? 1 : -1;
  }

  previousLeftA = leftA;
  previousRightA = rightA;
}

bool estopTripped() {
  return digitalRead(ESTOP_STATUS_PIN) == LOW;
}

bool readBeamBroken(uint8_t pin) {
  return digitalRead(pin) == LOW;
}

bool updateBeam(DebouncedBeam &beam, unsigned long now) {
  const bool raw = readBeamBroken(beam.pin);
  if (raw != beam.rawBroken) {
    beam.rawBroken = raw;
    beam.rawChangedMs = now;
  }
  if (beam.stableBroken != beam.rawBroken &&
      now - beam.rawChangedMs >= IR_DEBOUNCE_MS) {
    beam.stableBroken = beam.rawBroken;
    return true;
  }
  return false;
}

void readEncoderCounts(int32_t &left, int32_t &right) {
  noInterrupts();
  left = leftEncoderCount;
  right = rightEncoderCount;
  interrupts();
}

int16_t clampTestPwm(int value) {
  if (value > MAX_INTAKE_TEST_PWM) return MAX_INTAKE_TEST_PWM;
  if (value < -MAX_INTAKE_TEST_PWM) return -MAX_INTAKE_TEST_PWM;
  return value;
}

void applyMotor(uint8_t enablePin, uint8_t rpwm, uint8_t lpwm, int16_t pwm) {
  digitalWrite(enablePin, LOW);
  analogWrite(rpwm, 0);
  analogWrite(lpwm, 0);
  if (pwm > 0) {
    analogWrite(rpwm, (uint8_t)pwm);
  } else if (pwm < 0) {
    analogWrite(lpwm, (uint8_t)(-pwm));
  }
  if (pwm != 0) digitalWrite(enablePin, HIGH);
}

void stopOutputs() {
  commandedLeftPwm = 0;
  commandedRightPwm = 0;
  applyMotor(LEFT_EN_PIN, LEFT_RPWM_PIN, LEFT_LPWM_PIN, 0);
  applyMotor(RIGHT_EN_PIN, RIGHT_RPWM_PIN, RIGHT_LPWM_PIN, 0);
  runMode = RUN_NONE;
}

void printEvent(const __FlashStringHelper *name) {
  Serial.print(F("EVENT,"));
  Serial.print(millis());
  Serial.print(',');
  Serial.println(name);
}

void faultStop(const __FlashStringHelper *reason) {
  stopOutputs();
  state = FAULTED;
  digitalWrite(ARMED_LED_PIN, LOW);
  Serial.print(F("FAULT,"));
  Serial.print(millis());
  Serial.print(',');
  Serial.println(reason);
}

void armBench() {
  if (estopTripped()) {
    Serial.println(F("ERR,ESTOP_ACTIVE"));
    return;
  }
  stopOutputs();
  state = ARMED_IDLE;
  digitalWrite(ARMED_LED_PIN, HIGH);
  Serial.println(F("OK,ARMED"));
}

void disarmBench(const __FlashStringHelper *reason) {
  stopOutputs();
  state = estopTripped() ? FAULTED : DISARMED;
  digitalWrite(ARMED_LED_PIN, LOW);
  Serial.print(F("OK,DISARMED,"));
  Serial.println(reason);
}

bool canStartRun() {
  if (state != ARMED_IDLE) {
    Serial.println(F("ERR,NOT_ARMED_OR_BUSY"));
    return false;
  }
  if (estopTripped()) {
    faultStop(F("ESTOP"));
    return false;
  }
  return true;
}

void beginRun(int16_t leftPwm, int16_t rightPwm,
              unsigned long durationMs, RunMode mode) {
  int32_t leftCount, rightCount;
  readEncoderCounts(leftCount, rightCount);

  commandedLeftPwm = clampTestPwm(leftPwm);
  commandedRightPwm = clampTestPwm(rightPwm);
  const unsigned long maxDurationMs =
      (mode == RUN_AUTO_BALL || mode == RUN_AUTO_BENCH)
          ? MAX_AUTO_BALL_MS : MAX_BURST_MS;
  runDurationMs = min(durationMs, maxDurationMs);
  runStartedMs = millis();
  lastProgressCheckMs = runStartedMs;
  lastProgressLeftCount = leftCount;
  lastProgressRightCount = rightCount;
  runMode = mode;
  state = RUNNING;

  applyMotor(LEFT_EN_PIN, LEFT_RPWM_PIN, LEFT_LPWM_PIN, commandedLeftPwm);
  applyMotor(RIGHT_EN_PIN, RIGHT_RPWM_PIN, RIGHT_LPWM_PIN, commandedRightPwm);
  printEvent(F("RUN_STARTED"));
}

void startInwardRun(uint8_t pwm, unsigned long durationMs, RunMode mode) {
  beginRun(LEFT_INWARD_SIGN * pwm, RIGHT_INWARD_SIGN * pwm, durationMs, mode);
}

void finishRun(const __FlashStringHelper *reason) {
  stopOutputs();
  state = ARMED_IDLE;
  Serial.print(F("EVENT,"));
  Serial.print(millis());
  Serial.print(F(",RUN_STOPPED,"));
  Serial.println(reason);
}

void zeroEncoders() {
  noInterrupts();
  leftEncoderCount = 0;
  rightEncoderCount = 0;
  interrupts();
  lastTelemetryLeftCount = 0;
  lastTelemetryRightCount = 0;
  Serial.println(F("OK,ENCODERS_ZEROED"));
}

void printHelp() {
  Serial.println(F("CMD,ARM|DISARM|STOP|ZERO|STATUS"));
  Serial.println(F("CMD,DIST beam_spacing_mm"));
  Serial.println(F("CMD,PULSE L|R signed_pwm duration_ms"));
  Serial.println(F("CMD,RUN pwm duration_ms"));
  Serial.println(F("CMD,RUN_BENCH pwm duration_ms (ENCODER CHECK DISABLED)"));
  Serial.println(F("CMD,AUTO pwm [timeout_ms]"));
  Serial.println(F("CMD,AUTO_BENCH pwm [timeout_ms] (ENCODER CHECK DISABLED)"));
  Serial.println(F("LIMIT,pwm=90,manual_ms=1500,auto_ms=4000,BTS7960"));
}

void sendStatus() {
  int32_t leftCount, rightCount;
  readEncoderCounts(leftCount, rightCount);
  Serial.print(F("STATUS,"));
  Serial.print(millis()); Serial.print(',');
  Serial.print((int)state); Serial.print(',');
  Serial.print(commandedLeftPwm); Serial.print(',');
  Serial.print(commandedRightPwm); Serial.print(',');
  Serial.print(leftCount); Serial.print(',');
  Serial.print(rightCount); Serial.print(',');
  Serial.print(entryBeam.stableBroken ? 1 : 0); Serial.print(',');
  Serial.print(exitBeam.stableBroken ? 1 : 0); Serial.print(',');
  Serial.print(estopTripped() ? 1 : 0); Serial.print(',');
  Serial.println(beamSpacingMm, 1);
}

void handleLine(char *line) {
  if (strcmp(line, "HELP") == 0) {
    printHelp();
  } else if (strcmp(line, "ARM") == 0) {
    armBench();
  } else if (strcmp(line, "DISARM") == 0) {
    disarmBench(F("HOST"));
  } else if (strcmp(line, "STOP") == 0) {
    if (state == RUNNING || state == WAITING_FOR_BALL) finishRun(F("HOST"));
    else stopOutputs();
  } else if (strcmp(line, "ZERO") == 0) {
    zeroEncoders();
  } else if (strcmp(line, "STATUS") == 0) {
    sendStatus();
  } else if (strncmp(line, "DIST ", 5) == 0) {
    float distanceMm = 0.0f;
    if (sscanf(line + 5, "%f", &distanceMm) != 1 ||
        distanceMm < 20.0f || distanceMm > 1000.0f || state == RUNNING) {
      Serial.println(F("ERR,DIST_ARGS_OR_BUSY"));
    } else {
      beamSpacingMm = distanceMm;
      Serial.print(F("OK,DIST_MM,"));
      Serial.println(beamSpacingMm, 1);
    }
  } else if (strncmp(line, "PULSE ", 6) == 0) {
    char side = 0;
    int pwm = 0;
    unsigned long durationMs = 0;
    if (sscanf(line + 6, "%c %d %lu", &side, &pwm, &durationMs) != 3 ||
        (side != 'L' && side != 'R') || pwm == 0 || durationMs == 0 ||
        abs(pwm) > MAX_INTAKE_TEST_PWM || durationMs > MAX_BURST_MS) {
      Serial.println(F("ERR,PULSE_ARGS"));
    } else if (canStartRun()) {
      beginRun(side == 'L' ? pwm : 0, side == 'R' ? pwm : 0,
               durationMs, RUN_TIMED);
    }
  } else if (strncmp(line, "RUN_BENCH ", 10) == 0) {
    int pwm = 0;
    unsigned long durationMs = 0;
    if (sscanf(line + 10, "%d %lu", &pwm, &durationMs) != 2 || pwm <= 0 ||
        pwm > MAX_INTAKE_TEST_PWM || durationMs == 0 ||
        durationMs > MAX_AUTO_BALL_MS) {
      Serial.println(F("ERR,RUN_BENCH_ARGS"));
    } else if (canStartRun()) {
      startInwardRun((uint8_t)pwm, durationMs, RUN_AUTO_BENCH);
    }
  } else if (strncmp(line, "RUN ", 4) == 0) {
    int pwm = 0;
    unsigned long durationMs = 0;
    if (sscanf(line + 4, "%d %lu", &pwm, &durationMs) != 2 || pwm <= 0 ||
        pwm > MAX_INTAKE_TEST_PWM || durationMs == 0 ||
        durationMs > MAX_BURST_MS) {
      Serial.println(F("ERR,RUN_ARGS"));
    } else if (canStartRun()) {
      startInwardRun((uint8_t)pwm, durationMs, RUN_TIMED);
    }
  } else if (strncmp(line, "AUTO ", 5) == 0 ||
             strncmp(line, "AUTO_BENCH ", 11) == 0) {
    const bool benchWithoutEncoders = strncmp(line, "AUTO_BENCH ", 11) == 0;
    const char *args = line + (benchWithoutEncoders ? 11 : 5);
    int pwm = 0;
    unsigned long timeoutMs = DEFAULT_AUTO_BALL_MS;
    const int parsed = sscanf(args, "%d %lu", &pwm, &timeoutMs);
    if (parsed < 1 || pwm <= 0 || pwm > MAX_INTAKE_TEST_PWM ||
        timeoutMs == 0 || timeoutMs > MAX_AUTO_BALL_MS) {
      Serial.println(F("ERR,AUTO_ARGS"));
    } else if (canStartRun() &&
               (entryBeam.stableBroken || exitBeam.stableBroken)) {
      Serial.println(F("ERR,BEAMS_NOT_CLEAR"));
    } else if (state == ARMED_IDLE) {
      autoPwm = (uint8_t)pwm;
      autoTimeoutMs = timeoutMs;
      ballEntryMs = 0;
      exitWasReached = false;
      state = WAITING_FOR_BALL;
      runMode = benchWithoutEncoders ? RUN_AUTO_BENCH : RUN_AUTO_BALL;
      Serial.print(F("OK,WAITING_FOR_BALL,"));
      Serial.print(autoTimeoutMs);
      Serial.println(benchWithoutEncoders ? F(",BENCH_NO_ENCODER") : F(",ENCODER_SAFE"));
    }
  } else if (line[0] != '\0') {
    Serial.println(F("ERR,UNKNOWN_COMMAND"));
  }
}

void pollSerial() {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (rxLength > 0) {
        rxBuffer[rxLength] = '\0';
        handleLine(rxBuffer);
        rxLength = 0;
      }
    } else if (rxLength < sizeof(rxBuffer) - 1) {
      rxBuffer[rxLength++] = c;
    } else {
      rxLength = 0;
      Serial.println(F("ERR,LINE_TOO_LONG"));
    }
  }
}

void updateBallCycle(unsigned long now, bool entryChanged, bool exitChanged) {
  if (state == WAITING_FOR_BALL && entryChanged && entryBeam.stableBroken) {
    ballEntryMs = now;
    exitWasReached = false;
    Serial.print(F("BALL_ENTRY,"));
    Serial.println(now);
    startInwardRun(autoPwm, autoTimeoutMs,
                   runMode == RUN_AUTO_BENCH ? RUN_AUTO_BENCH : RUN_AUTO_BALL);
  }

  if (state == RUNNING &&
      (runMode == RUN_AUTO_BALL || runMode == RUN_AUTO_BENCH)) {
    if (exitChanged && exitBeam.stableBroken && !exitWasReached) {
      exitWasReached = true;
      Serial.print(F("BALL_EXIT_REACHED,"));
      Serial.print(now);
      Serial.print(',');
      const unsigned long transitMs = now - ballEntryMs;
      Serial.print(transitMs);
      Serial.print(',');
      if (beamSpacingMm > 0.0f && transitMs > 0) {
        // mm/ms is numerically equal to m/s.
        Serial.println(beamSpacingMm / transitMs, 3);
      } else {
        Serial.println(F("NA"));
      }
      Serial.print(F("BALL_CYCLE_COMPLETE,"));
      Serial.print(now);
      Serial.print(',');
      Serial.println(now - ballEntryMs);
      finishRun(F("BALL_EXIT"));
    }
  }
}

void checkRunSafety(unsigned long now) {
  if (state != RUNNING) return;

  if (now - runStartedMs >= runDurationMs) {
    finishRun((runMode == RUN_AUTO_BALL || runMode == RUN_AUTO_BENCH)
                  ? F("BALL_TIMEOUT") : F("TIME_LIMIT"));
    return;
  }

  // Explicit bench-only mode still enforces the hard run timeout above, but
  // allows IR/timeout testing before the encoder harness is commissioned.
  if (runMode == RUN_AUTO_BENCH) return;

  if (now - runStartedMs < STARTUP_GRACE_MS ||
      now - lastProgressCheckMs < NO_ENCODER_PROGRESS_MS) return;

  int32_t leftCount, rightCount;
  readEncoderCounts(leftCount, rightCount);
  const bool leftExpected = abs(commandedLeftPwm) >= MIN_JAM_CHECK_PWM;
  const bool rightExpected = abs(commandedRightPwm) >= MIN_JAM_CHECK_PWM;
  const bool leftStopped = leftExpected && leftCount == lastProgressLeftCount;
  const bool rightStopped = rightExpected && rightCount == lastProgressRightCount;

  lastProgressLeftCount = leftCount;
  lastProgressRightCount = rightCount;
  lastProgressCheckMs = now;

  if (leftStopped || rightStopped) {
    if (leftStopped && rightStopped) faultStop(F("NO_ENCODER_BOTH"));
    else if (leftStopped) faultStop(F("NO_ENCODER_LEFT"));
    else faultStop(F("NO_ENCODER_RIGHT"));
  }
}

void sendTelemetry(unsigned long now) {
  const unsigned long elapsed = now - lastTelemetryMs;
  if (elapsed < TELEMETRY_MS) return;

  int32_t leftCount, rightCount;
  readEncoderCounts(leftCount, rightCount);
  const float leftRpm = (leftCount - lastTelemetryLeftCount) * 60000.0f /
                        (elapsed * ENCODER_COUNTS_PER_OUTPUT_REV);
  const float rightRpm = (rightCount - lastTelemetryRightCount) * 60000.0f /
                         (elapsed * ENCODER_COUNTS_PER_OUTPUT_REV);

  Serial.print(F("DATA,"));
  Serial.print(now); Serial.print(',');
  Serial.print((int)state); Serial.print(',');
  Serial.print(commandedLeftPwm); Serial.print(',');
  Serial.print(commandedRightPwm); Serial.print(',');
  Serial.print(leftCount); Serial.print(',');
  Serial.print(rightCount); Serial.print(',');
  Serial.print(leftRpm, 1); Serial.print(',');
  Serial.print(rightRpm, 1); Serial.print(',');
  Serial.print(entryBeam.stableBroken ? 1 : 0); Serial.print(',');
  Serial.println(exitBeam.stableBroken ? 1 : 0);

  lastTelemetryLeftCount = leftCount;
  lastTelemetryRightCount = rightCount;
  lastTelemetryMs = now;
}

void setupPinChangeInterrupts() {
  noInterrupts();
  previousLeftA = (PINK & _BV(2)) != 0;
  previousRightA = (PINK & _BV(0)) != 0;
  PCIFR = _BV(PCIF2);                   // clear pending port-K interrupt
  PCMSK2 |= _BV(PCINT16) | _BV(PCINT18);  // enable A channels only
  PCICR |= _BV(PCIE2);
  interrupts();
}

void setup() {
  Serial.begin(BAUD);

  pinMode(LEFT_EN_PIN, OUTPUT);
  pinMode(LEFT_RPWM_PIN, OUTPUT);
  pinMode(LEFT_LPWM_PIN, OUTPUT);
  pinMode(RIGHT_EN_PIN, OUTPUT);
  pinMode(RIGHT_RPWM_PIN, OUTPUT);
  pinMode(RIGHT_LPWM_PIN, OUTPUT);
  pinMode(LEFT_ENC_A_PIN, INPUT_PULLUP);
  pinMode(LEFT_ENC_B_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B_PIN, INPUT_PULLUP);
  pinMode(IR_ENTRY_PIN, INPUT_PULLUP);
  pinMode(IR_EXIT_PIN, INPUT_PULLUP);
  pinMode(ESTOP_STATUS_PIN, INPUT_PULLUP);
  pinMode(ARMED_LED_PIN, OUTPUT);

  stopOutputs();
  digitalWrite(ARMED_LED_PIN, LOW);
  entryBeam.rawBroken = entryBeam.stableBroken = readBeamBroken(IR_ENTRY_PIN);
  exitBeam.rawBroken = exitBeam.stableBroken = readBeamBroken(IR_EXIT_PIN);
  setupPinChangeInterrupts();

  const unsigned long now = millis();
  lastTelemetryMs = now;
  Serial.println(F("READY,DUAL_INTAKE_MEGA_BENCH,DISARMED"));
  Serial.println(F("FEATURE,AUTO_TIMEOUT_MS,4000,STOP_ON_EXIT_BROKEN"));
  Serial.println(F("FEATURE,AUTO_BENCH_NO_ENCODER,HARD_TIMEOUT_MS,4000"));
  Serial.println(F("HEADER,ms,state,left_pwm,right_pwm,left_count,right_count,left_rpm,right_rpm,entry,exit"));
  printHelp();
}

void loop() {
  pollSerial();
  const unsigned long now = millis();

  if (estopTripped() && state != FAULTED) {
    faultStop(F("ESTOP"));
  }

  const bool entryChanged = updateBeam(entryBeam, now);
  const bool exitChanged = updateBeam(exitBeam, now);
  updateBallCycle(now, entryChanged, exitChanged);
  checkRunSafety(now);
  sendTelemetry(now);
}
