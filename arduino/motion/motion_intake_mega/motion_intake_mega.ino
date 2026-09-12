/*
 * motion_intake_mega.ino - unified Mega 2560 bring-up firmware
 * --------------------------------------------------------------------------
 * Hardware covered by the common 120x80 mm perfboard:
 *   - 2x BTS7960 drive channels, 4x drive encoders
 *   - dual intake H-bridge control, 2x intake encoders
 *   - entry/exit IR break beams
 *   - START, E-stop status, armed LED
 *   - GY-521 / MPU6050 over I2C
 *
 * This sketch is a safe integration/bring-up target. The L298N intake driver
 * remains bench-only: FIT0186 stall current exceeds its channel rating.
 * Motor current and 12V never pass through the perfboard.
 *
 * Serial, 115200 baud, newline terminated:
 *   ARM
 *   DISARM
 *   STOP
 *   PING
 *   M <left> <right>                  drive duty [-1.0, 1.0]
 *   I <left_pwm> <right_pwm> <ms>     intake, each [-90, 90], max 1500 ms
 *   INTAKE_STOP
 *   AUTO <pwm> <timeout_ms>           wait for entry, stop at exit/max 4000 ms
 *   ZERO
 *   STATUS
 *   IMU
 *
 * A fresh M/PING/command is required at least every 300 ms while armed.
 */

#include <Arduino.h>
#include <Wire.h>
#include <avr/interrupt.h>

// Drive BTS7960 control.
const uint8_t DRIVE_LEFT_RPWM_PIN = 5;
const uint8_t DRIVE_LEFT_LPWM_PIN = 6;
const uint8_t DRIVE_LEFT_EN_PIN = 30;
const uint8_t DRIVE_RIGHT_RPWM_PIN = 9;
const uint8_t DRIVE_RIGHT_LPWM_PIN = 10;
const uint8_t DRIVE_RIGHT_EN_PIN = 31;

// Drive encoders: A uses external interrupts, B supplies direction.
const uint8_t DRIVE_LF_ENC_A_PIN = 2;
const uint8_t DRIVE_LF_ENC_B_PIN = 22;
const uint8_t DRIVE_LR_ENC_A_PIN = 3;
const uint8_t DRIVE_LR_ENC_B_PIN = 23;
const uint8_t DRIVE_RF_ENC_A_PIN = 18;
const uint8_t DRIVE_RF_ENC_B_PIN = 24;
const uint8_t DRIVE_RR_ENC_A_PIN = 19;
const uint8_t DRIVE_RR_ENC_B_PIN = 25;

// Intake H-bridge control.
const uint8_t INTAKE_LEFT_EN_PIN = 44;
const uint8_t INTAKE_LEFT_IN1_PIN = 40;
const uint8_t INTAKE_LEFT_IN2_PIN = 41;
const uint8_t INTAKE_RIGHT_EN_PIN = 45;
const uint8_t INTAKE_RIGHT_IN3_PIN = 42;
const uint8_t INTAKE_RIGHT_IN4_PIN = 43;

// Intake encoders on Mega port K pin-change interrupts.
const uint8_t INTAKE_LEFT_ENC_A_PIN = A8;
const uint8_t INTAKE_LEFT_ENC_B_PIN = A9;
const uint8_t INTAKE_RIGHT_ENC_A_PIN = A10;
const uint8_t INTAKE_RIGHT_ENC_B_PIN = A11;

const uint8_t IR_ENTRY_PIN = 36;
const uint8_t IR_EXIT_PIN = 37;
const uint8_t START_ARM_PIN = 32;
const uint8_t ESTOP_STATUS_PIN = 33;
const uint8_t ARMED_LED_PIN = 34;
const uint8_t IMU_SDA_PIN = 20;  // fixed Mega I2C pin, documented for pin audit
const uint8_t IMU_SCL_PIN = 21;  // fixed Mega I2C pin, documented for pin audit

const unsigned long BAUD = 115200;
const unsigned long HOST_TIMEOUT_MS = 300;
const unsigned long CONTROL_PERIOD_MS = 10;
const unsigned long TELEMETRY_PERIOD_MS = 100;
const unsigned long IMU_PERIOD_MS = 50;
const unsigned long BUTTON_DEBOUNCE_MS = 40;
const unsigned long IR_DEBOUNCE_MS = 25;
const unsigned long INTAKE_STARTUP_GRACE_MS = 300;
const unsigned long INTAKE_PROGRESS_PERIOD_MS = 250;
const unsigned long MAX_INTAKE_RUN_MS = 1500;
const unsigned long MAX_AUTO_RUN_MS = 4000;
const uint8_t MAX_L298N_TEST_PWM = 90;
const uint8_t MIN_INTAKE_PROGRESS_PWM = 35;
const int8_t INTAKE_LEFT_INWARD_SIGN = 1;
const int8_t INTAKE_RIGHT_INWARD_SIGN = -1;
const float DRIVE_RAMP_PER_TICK = 0.02f;
const float DRIVE_DEADBAND = 0.02f;

enum SystemState : uint8_t { DISARMED = 0, ARMED = 1, ESTOPPED = 2, FAULTED = 3 };
enum IntakeMode : uint8_t { INTAKE_IDLE = 0, INTAKE_TIMED = 1, INTAKE_WAIT_ENTRY = 2, INTAKE_AUTO = 3 };

struct DebouncedInput {
  uint8_t pin;
  bool raw;
  bool stable;
  unsigned long changedMs;
};

struct ImuSample {
  int16_t ax, ay, az;
  int16_t temperature;
  int16_t gx, gy, gz;
};

SystemState state = DISARMED;
IntakeMode intakeMode = INTAKE_IDLE;

float driveTargetLeft = 0.0f;
float driveTargetRight = 0.0f;
float driveCurrentLeft = 0.0f;
float driveCurrentRight = 0.0f;

int16_t intakeLeftPwm = 0;
int16_t intakeRightPwm = 0;
uint8_t autoPwm = 0;
unsigned long intakeStartedMs = 0;
unsigned long intakeDeadlineMs = 0;
unsigned long intakeLastProgressMs = 0;
int32_t intakeLastLeftCount = 0;
int32_t intakeLastRightCount = 0;

volatile int32_t driveLfCount = 0;
volatile int32_t driveLrCount = 0;
volatile int32_t driveRfCount = 0;
volatile int32_t driveRrCount = 0;
volatile int32_t intakeLeftCount = 0;
volatile int32_t intakeRightCount = 0;
volatile bool intakePreviousLeftA = false;
volatile bool intakePreviousRightA = false;

DebouncedInput entryBeam = {IR_ENTRY_PIN, HIGH, HIGH, 0};
DebouncedInput exitBeam = {IR_EXIT_PIN, HIGH, HIGH, 0};
bool startRaw = HIGH;
bool startStable = HIGH;
unsigned long startChangedMs = 0;

bool imuPresent = false;
uint8_t imuAddress = 0x68;
ImuSample imu = {0, 0, 0, 0, 0, 0, 0};

unsigned long lastHostMs = 0;
unsigned long lastControlMs = 0;
unsigned long lastTelemetryMs = 0;
unsigned long lastImuMs = 0;
char rxBuffer[72];
uint8_t rxLength = 0;

void isrDriveLf() { driveLfCount += digitalRead(DRIVE_LF_ENC_B_PIN) ? 1 : -1; }
void isrDriveLr() { driveLrCount += digitalRead(DRIVE_LR_ENC_B_PIN) ? 1 : -1; }
void isrDriveRf() { driveRfCount += digitalRead(DRIVE_RF_ENC_B_PIN) ? 1 : -1; }
void isrDriveRr() { driveRrCount += digitalRead(DRIVE_RR_ENC_B_PIN) ? 1 : -1; }

ISR(PCINT2_vect) {
  const uint8_t portK = PINK;
  const bool leftA = (portK & _BV(PK0)) != 0;
  const bool rightA = (portK & _BV(PK2)) != 0;
  if (leftA && !intakePreviousLeftA) {
    intakeLeftCount += (portK & _BV(PK1)) ? 1 : -1;
  }
  if (rightA && !intakePreviousRightA) {
    intakeRightCount += (portK & _BV(PK3)) ? 1 : -1;
  }
  intakePreviousLeftA = leftA;
  intakePreviousRightA = rightA;
}

bool estopTripped() { return digitalRead(ESTOP_STATUS_PIN) == LOW; }
bool beamBroken(const DebouncedInput &beam) { return beam.stable == LOW; }

float clampDuty(float value) {
  if (value > 1.0f) return 1.0f;
  if (value < -1.0f) return -1.0f;
  return value;
}

float rampToward(float current, float target) {
  if (current < target) {
    current += DRIVE_RAMP_PER_TICK;
    if (current > target) current = target;
  } else if (current > target) {
    current -= DRIVE_RAMP_PER_TICK;
    if (current < target) current = target;
  }
  return current;
}

void applyDriveSide(uint8_t rpwm, uint8_t lpwm, float duty) {
  if (duty > DRIVE_DEADBAND) {
    analogWrite(rpwm, (uint8_t)(duty * 255.0f));
    analogWrite(lpwm, 0);
  } else if (duty < -DRIVE_DEADBAND) {
    analogWrite(rpwm, 0);
    analogWrite(lpwm, (uint8_t)(-duty * 255.0f));
  } else {
    analogWrite(rpwm, 0);
    analogWrite(lpwm, 0);
  }
}

void setDriveEnable(bool enabled) {
  digitalWrite(DRIVE_LEFT_EN_PIN, enabled ? HIGH : LOW);
  digitalWrite(DRIVE_RIGHT_EN_PIN, enabled ? HIGH : LOW);
}

void stopDrive() {
  driveTargetLeft = driveTargetRight = 0.0f;
  driveCurrentLeft = driveCurrentRight = 0.0f;
  applyDriveSide(DRIVE_LEFT_RPWM_PIN, DRIVE_LEFT_LPWM_PIN, 0.0f);
  applyDriveSide(DRIVE_RIGHT_RPWM_PIN, DRIVE_RIGHT_LPWM_PIN, 0.0f);
}

void applyIntakeMotor(uint8_t en, uint8_t in1, uint8_t in2, int16_t pwm) {
  analogWrite(en, 0);
  if (pwm > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(en, (uint8_t)pwm);
  } else if (pwm < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(en, (uint8_t)(-pwm));
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void stopIntake() {
  intakeLeftPwm = intakeRightPwm = 0;
  applyIntakeMotor(INTAKE_LEFT_EN_PIN, INTAKE_LEFT_IN1_PIN, INTAKE_LEFT_IN2_PIN, 0);
  applyIntakeMotor(INTAKE_RIGHT_EN_PIN, INTAKE_RIGHT_IN3_PIN, INTAKE_RIGHT_IN4_PIN, 0);
  intakeMode = INTAKE_IDLE;
}

void stopAll() {
  stopDrive();
  stopIntake();
}

void setArmedLed() { digitalWrite(ARMED_LED_PIN, state == ARMED ? HIGH : LOW); }

void disarmSystem(const __FlashStringHelper *reason) {
  stopAll();
  setDriveEnable(false);
  state = estopTripped() ? ESTOPPED : DISARMED;
  setArmedLed();
  Serial.print(F("OK,DISARMED,"));
  Serial.println(reason);
}

void faultSystem(const __FlashStringHelper *reason) {
  stopAll();
  setDriveEnable(false);
  state = FAULTED;
  setArmedLed();
  Serial.print(F("FAULT,"));
  Serial.println(reason);
}

void armSystem() {
  if (estopTripped()) {
    Serial.println(F("ERR,ESTOP_ACTIVE"));
    return;
  }
  stopAll();
  state = ARMED;
  setDriveEnable(true);
  setArmedLed();
  lastHostMs = millis();
  Serial.println(F("OK,ARMED"));
}

void readAllCounts(int32_t &lf, int32_t &lr, int32_t &rf, int32_t &rr,
                   int32_t &intakeLeft, int32_t &intakeRight) {
  noInterrupts();
  lf = driveLfCount; lr = driveLrCount;
  rf = driveRfCount; rr = driveRrCount;
  intakeLeft = intakeLeftCount; intakeRight = intakeRightCount;
  interrupts();
}

void startIntake(int16_t leftPwm, int16_t rightPwm,
                 unsigned long durationMs, IntakeMode mode) {
  int32_t lf, lr, rf, rr;
  readAllCounts(lf, lr, rf, rr, intakeLastLeftCount, intakeLastRightCount);
  intakeLeftPwm = leftPwm;
  intakeRightPwm = rightPwm;
  intakeStartedMs = millis();
  intakeDeadlineMs = intakeStartedMs + durationMs;
  intakeLastProgressMs = intakeStartedMs;
  intakeMode = mode;
  applyIntakeMotor(INTAKE_LEFT_EN_PIN, INTAKE_LEFT_IN1_PIN, INTAKE_LEFT_IN2_PIN, leftPwm);
  applyIntakeMotor(INTAKE_RIGHT_EN_PIN, INTAKE_RIGHT_IN3_PIN, INTAKE_RIGHT_IN4_PIN, rightPwm);
}

bool updateDebounced(DebouncedInput &input, unsigned long now) {
  const bool raw = digitalRead(input.pin);
  if (raw != input.raw) {
    input.raw = raw;
    input.changedMs = now;
  }
  if (input.stable != raw && now - input.changedMs >= IR_DEBOUNCE_MS) {
    input.stable = raw;
    return true;
  }
  return false;
}

void pollStartButton(unsigned long now) {
  const bool raw = digitalRead(START_ARM_PIN);
  if (raw != startRaw) {
    startRaw = raw;
    startChangedMs = now;
  }
  if (raw != startStable && now - startChangedMs >= BUTTON_DEBOUNCE_MS) {
    startStable = raw;
    if (startStable == LOW && state == DISARMED) armSystem();
  }
}

bool imuWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(imuAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool imuRead(uint8_t reg, uint8_t *data, uint8_t length) {
  Wire.beginTransmission(imuAddress);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  const uint8_t received = Wire.requestFrom(imuAddress, length);
  if (received != length) return false;
  for (uint8_t i = 0; i < length; ++i) data[i] = Wire.read();
  return true;
}

bool probeImuAddress(uint8_t address) {
  imuAddress = address;
  uint8_t who = 0;
  return imuRead(0x75, &who, 1) && (who == 0x68 || who == 0x69);
}

void setupImu() {
  Wire.begin();
  Wire.setClock(100000UL);  // conservative for a cabled module near motor wiring
  imuPresent = probeImuAddress(0x68) || probeImuAddress(0x69);
  if (imuPresent) {
    imuPresent = imuWrite(0x6B, 0x00) &&  // wake, internal 8 MHz clock initially
                 imuWrite(0x1B, 0x00) &&  // gyro +/-250 deg/s
                 imuWrite(0x1C, 0x00);    // accelerometer +/-2g
  }
}

void updateImu(unsigned long now) {
  if (!imuPresent || now - lastImuMs < IMU_PERIOD_MS) return;
  lastImuMs = now;
  uint8_t data[14];
  if (!imuRead(0x3B, data, sizeof(data))) {
    imuPresent = false;
    return;
  }
  int16_t *values[] = {&imu.ax, &imu.ay, &imu.az, &imu.temperature,
                       &imu.gx, &imu.gy, &imu.gz};
  for (uint8_t i = 0; i < 7; ++i) {
    *values[i] = (int16_t)((uint16_t)data[2 * i] << 8 | data[2 * i + 1]);
  }
}

void printImu() {
  Serial.print(F("IMU,"));
  Serial.print(imuPresent ? 1 : 0); Serial.print(',');
  Serial.print(imuAddress, HEX); Serial.print(',');
  Serial.print(imu.ax); Serial.print(','); Serial.print(imu.ay); Serial.print(',');
  Serial.print(imu.az); Serial.print(','); Serial.print(imu.gx); Serial.print(',');
  Serial.print(imu.gy); Serial.print(','); Serial.println(imu.gz);
}

void zeroEncoders() {
  noInterrupts();
  driveLfCount = driveLrCount = driveRfCount = driveRrCount = 0;
  intakeLeftCount = intakeRightCount = 0;
  interrupts();
  Serial.println(F("OK,ENCODERS_ZEROED"));
}

void sendStatus(unsigned long now) {
  int32_t lf, lr, rf, rr, il, ir;
  readAllCounts(lf, lr, rf, rr, il, ir);
  Serial.print(F("T,")); Serial.print(now); Serial.print(',');
  Serial.print((uint8_t)state); Serial.print(',');
  Serial.print(driveCurrentLeft, 3); Serial.print(',');
  Serial.print(driveCurrentRight, 3); Serial.print(',');
  Serial.print(intakeLeftPwm); Serial.print(','); Serial.print(intakeRightPwm); Serial.print(',');
  Serial.print(lf); Serial.print(','); Serial.print(lr); Serial.print(',');
  Serial.print(rf); Serial.print(','); Serial.print(rr); Serial.print(',');
  Serial.print(il); Serial.print(','); Serial.print(ir); Serial.print(',');
  Serial.print(beamBroken(entryBeam) ? 1 : 0); Serial.print(',');
  Serial.print(beamBroken(exitBeam) ? 1 : 0); Serial.print(',');
  Serial.print(estopTripped() ? 1 : 0); Serial.print(',');
  Serial.print(imuPresent ? 1 : 0); Serial.print(',');
  Serial.println(imu.gz);
}

void printHelp() {
  Serial.println(F("CMD,ARM|DISARM|STOP|PING|ZERO|STATUS|IMU|INTAKE_STOP"));
  Serial.println(F("CMD,M left_duty right_duty"));
  Serial.println(F("CMD,I left_pwm right_pwm duration_ms"));
  Serial.println(F("CMD,AUTO pwm timeout_ms"));
  Serial.println(F("LIMIT,drive=1.0,intake_pwm=90,intake_ms=1500,auto_ms=4000"));
}

void handleLine(char *line) {
  if (strcmp(line, "HELP") == 0) {
    printHelp();
  } else if (strcmp(line, "ARM") == 0) {
    armSystem();
  } else if (strcmp(line, "DISARM") == 0) {
    disarmSystem(F("HOST"));
  } else if (strcmp(line, "STOP") == 0) {
    stopAll();
    lastHostMs = millis();
    Serial.println(F("OK,STOPPED"));
  } else if (strcmp(line, "INTAKE_STOP") == 0) {
    stopIntake();
    lastHostMs = millis();
    Serial.println(F("OK,INTAKE_STOPPED"));
  } else if (strcmp(line, "PING") == 0) {
    lastHostMs = millis();
    Serial.println(F("PONG"));
  } else if (strcmp(line, "ZERO") == 0) {
    zeroEncoders();
  } else if (strcmp(line, "STATUS") == 0) {
    sendStatus(millis());
  } else if (strcmp(line, "IMU") == 0) {
    printImu();
  } else if (strncmp(line, "M ", 2) == 0) {
    float left = 0.0f, right = 0.0f;
    if (state != ARMED) {
      Serial.println(F("ERR,NOT_ARMED"));
    } else if (sscanf(line + 2, "%f %f", &left, &right) != 2 ||
               left < -1.0f || left > 1.0f || right < -1.0f || right > 1.0f) {
      Serial.println(F("ERR,M_ARGS"));
    } else {
      driveTargetLeft = clampDuty(left);
      driveTargetRight = clampDuty(right);
      lastHostMs = millis();
    }
  } else if (strncmp(line, "I ", 2) == 0) {
    int left = 0, right = 0;
    unsigned long duration = 0;
    if (state != ARMED) {
      Serial.println(F("ERR,NOT_ARMED"));
    } else if (sscanf(line + 2, "%d %d %lu", &left, &right, &duration) != 3 ||
               abs(left) > MAX_L298N_TEST_PWM || abs(right) > MAX_L298N_TEST_PWM ||
               duration == 0 || duration > MAX_INTAKE_RUN_MS) {
      Serial.println(F("ERR,I_ARGS"));
    } else {
      startIntake(left, right, duration, INTAKE_TIMED);
      lastHostMs = millis();
      Serial.println(F("OK,INTAKE_STARTED"));
    }
  } else if (strncmp(line, "AUTO ", 5) == 0) {
    int pwm = 0;
    unsigned long timeoutMs = 0;
    if (state != ARMED) {
      Serial.println(F("ERR,NOT_ARMED"));
    } else if (sscanf(line + 5, "%d %lu", &pwm, &timeoutMs) != 2 ||
               pwm <= 0 || pwm > MAX_L298N_TEST_PWM || timeoutMs == 0 ||
               timeoutMs > MAX_AUTO_RUN_MS) {
      Serial.println(F("ERR,AUTO_ARGS"));
    } else if (beamBroken(entryBeam) || beamBroken(exitBeam)) {
      Serial.println(F("ERR,BEAMS_NOT_CLEAR"));
    } else {
      autoPwm = (uint8_t)pwm;
      intakeDeadlineMs = millis() + timeoutMs;
      intakeMode = INTAKE_WAIT_ENTRY;
      lastHostMs = millis();
      Serial.println(F("OK,WAITING_FOR_ENTRY"));
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

void updateIntake(unsigned long now, bool entryChanged, bool exitChanged) {
  if (intakeMode == INTAKE_WAIT_ENTRY) {
    if ((long)(now - intakeDeadlineMs) >= 0) {
      stopIntake();
      Serial.println(F("EVENT,AUTO_WAIT_TIMEOUT"));
    } else if (entryChanged && beamBroken(entryBeam)) {
      const unsigned long runMs = intakeDeadlineMs - now;
      startIntake(INTAKE_LEFT_INWARD_SIGN * autoPwm,
                  INTAKE_RIGHT_INWARD_SIGN * autoPwm,
                  runMs, INTAKE_AUTO);
      Serial.println(F("EVENT,BALL_ENTRY"));
    }
    return;
  }

  if (intakeMode == INTAKE_IDLE) return;
  if (intakeMode == INTAKE_AUTO && exitChanged && beamBroken(exitBeam)) {
    stopIntake();
    Serial.println(F("EVENT,BALL_EXIT_INTAKE_STOPPED"));
    return;
  }
  if ((long)(now - intakeDeadlineMs) >= 0) {
    stopIntake();
    Serial.println(F("EVENT,INTAKE_TIME_LIMIT"));
    return;
  }
  if (now - intakeStartedMs < INTAKE_STARTUP_GRACE_MS ||
      now - intakeLastProgressMs < INTAKE_PROGRESS_PERIOD_MS) return;

  int32_t lf, lr, rf, rr, leftCount, rightCount;
  readAllCounts(lf, lr, rf, rr, leftCount, rightCount);
  const bool leftExpected = abs(intakeLeftPwm) >= MIN_INTAKE_PROGRESS_PWM;
  const bool rightExpected = abs(intakeRightPwm) >= MIN_INTAKE_PROGRESS_PWM;
  const bool leftStopped = leftExpected && leftCount == intakeLastLeftCount;
  const bool rightStopped = rightExpected && rightCount == intakeLastRightCount;
  intakeLastLeftCount = leftCount;
  intakeLastRightCount = rightCount;
  intakeLastProgressMs = now;
  if (leftStopped || rightStopped) {
    faultSystem(leftStopped && rightStopped ? F("INTAKE_NO_ENCODER_BOTH") :
                leftStopped ? F("INTAKE_NO_ENCODER_LEFT") : F("INTAKE_NO_ENCODER_RIGHT"));
  }
}

void setupIntakePinChangeInterrupts() {
  noInterrupts();
  intakePreviousLeftA = (PINK & _BV(PK0)) != 0;
  intakePreviousRightA = (PINK & _BV(PK2)) != 0;
  PCIFR = _BV(PCIF2);
  PCMSK2 |= _BV(PCINT16) | _BV(PCINT18);
  PCICR |= _BV(PCIE2);
  interrupts();
}

void setup() {
  Serial.begin(BAUD);

  pinMode(DRIVE_LEFT_RPWM_PIN, OUTPUT); pinMode(DRIVE_LEFT_LPWM_PIN, OUTPUT);
  pinMode(DRIVE_LEFT_EN_PIN, OUTPUT); pinMode(DRIVE_RIGHT_RPWM_PIN, OUTPUT);
  pinMode(DRIVE_RIGHT_LPWM_PIN, OUTPUT); pinMode(DRIVE_RIGHT_EN_PIN, OUTPUT);
  pinMode(DRIVE_LF_ENC_A_PIN, INPUT_PULLUP); pinMode(DRIVE_LF_ENC_B_PIN, INPUT_PULLUP);
  pinMode(DRIVE_LR_ENC_A_PIN, INPUT_PULLUP); pinMode(DRIVE_LR_ENC_B_PIN, INPUT_PULLUP);
  pinMode(DRIVE_RF_ENC_A_PIN, INPUT_PULLUP); pinMode(DRIVE_RF_ENC_B_PIN, INPUT_PULLUP);
  pinMode(DRIVE_RR_ENC_A_PIN, INPUT_PULLUP); pinMode(DRIVE_RR_ENC_B_PIN, INPUT_PULLUP);

  pinMode(INTAKE_LEFT_EN_PIN, OUTPUT); pinMode(INTAKE_LEFT_IN1_PIN, OUTPUT);
  pinMode(INTAKE_LEFT_IN2_PIN, OUTPUT); pinMode(INTAKE_RIGHT_EN_PIN, OUTPUT);
  pinMode(INTAKE_RIGHT_IN3_PIN, OUTPUT); pinMode(INTAKE_RIGHT_IN4_PIN, OUTPUT);
  pinMode(INTAKE_LEFT_ENC_A_PIN, INPUT_PULLUP); pinMode(INTAKE_LEFT_ENC_B_PIN, INPUT_PULLUP);
  pinMode(INTAKE_RIGHT_ENC_A_PIN, INPUT_PULLUP); pinMode(INTAKE_RIGHT_ENC_B_PIN, INPUT_PULLUP);

  pinMode(IR_ENTRY_PIN, INPUT_PULLUP); pinMode(IR_EXIT_PIN, INPUT_PULLUP);
  pinMode(START_ARM_PIN, INPUT_PULLUP); pinMode(ESTOP_STATUS_PIN, INPUT_PULLUP);
  pinMode(ARMED_LED_PIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(DRIVE_LF_ENC_A_PIN), isrDriveLf, RISING);
  attachInterrupt(digitalPinToInterrupt(DRIVE_LR_ENC_A_PIN), isrDriveLr, RISING);
  attachInterrupt(digitalPinToInterrupt(DRIVE_RF_ENC_A_PIN), isrDriveRf, RISING);
  attachInterrupt(digitalPinToInterrupt(DRIVE_RR_ENC_A_PIN), isrDriveRr, RISING);
  setupIntakePinChangeInterrupts();

  entryBeam.raw = entryBeam.stable = digitalRead(IR_ENTRY_PIN);
  exitBeam.raw = exitBeam.stable = digitalRead(IR_EXIT_PIN);
  stopAll();
  setDriveEnable(false);
  setArmedLed();
  setupImu();

  const unsigned long now = millis();
  lastHostMs = lastControlMs = lastTelemetryMs = lastImuMs = now;
  Serial.print(F("READY,MOTION_INTAKE_MEGA,DISARMED,IMU="));
  Serial.println(imuPresent ? F("OK") : F("NOT_FOUND"));
  printHelp();
}

void loop() {
  pollSerial();
  const unsigned long now = millis();
  pollStartButton(now);

  if (estopTripped() && state != ESTOPPED) {
    stopAll();
    setDriveEnable(false);
    state = ESTOPPED;
    setArmedLed();
    Serial.println(F("FAULT,ESTOP"));
  } else if (state == ESTOPPED && !estopTripped()) {
    state = DISARMED;
    setArmedLed();
    Serial.println(F("EVENT,ESTOP_CLEARED_STILL_DISARMED"));
  }

  const bool entryChanged = updateDebounced(entryBeam, now);
  const bool exitChanged = updateDebounced(exitBeam, now);

  if (state == ARMED && now - lastHostMs > HOST_TIMEOUT_MS) {
    stopAll();
  }

  if (now - lastControlMs >= CONTROL_PERIOD_MS) {
    lastControlMs = now;
    if (state == ARMED) {
      driveCurrentLeft = rampToward(driveCurrentLeft, driveTargetLeft);
      driveCurrentRight = rampToward(driveCurrentRight, driveTargetRight);
      applyDriveSide(DRIVE_LEFT_RPWM_PIN, DRIVE_LEFT_LPWM_PIN, driveCurrentLeft);
      applyDriveSide(DRIVE_RIGHT_RPWM_PIN, DRIVE_RIGHT_LPWM_PIN, driveCurrentRight);
    } else {
      stopAll();
      setDriveEnable(false);
    }
  }

  if (state == ARMED) updateIntake(now, entryChanged, exitChanged);
  updateImu(now);
  if (now - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = now;
    sendStatus(now);
  }
}
