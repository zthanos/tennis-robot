/*
 * P2-64 isolated physical-drive firmware for Arduino Mega 2560.
 * Existing robot sketches are intentionally untouched.
 *
 * The BTS7960 is a voltage/PWM actuator.  Wheel control therefore uses a
 * no-load-speed feed-forward plus a PI voltage-duty correction.  NOLOAD must
 * be measured during lifted-wheel bring-up and supplied in rad/s.
 *
 * E-stop input is fail-safe: an NC switch connects D33 to GND.  LOW means OK;
 * HIGH means pressed, disconnected, or broken wire and disables the bridges.
 */

#include <Arduino.h>
#include <Wire.h>

// BTS7960 pins.  Each bridge powers the two motors on one side in parallel.
const uint8_t LEFT_RPWM_PIN = 5;
const uint8_t LEFT_LPWM_PIN = 6;
const uint8_t LEFT_ENABLE_PIN = 30;
const uint8_t RIGHT_RPWM_PIN = 9;
const uint8_t RIGHT_LPWM_PIN = 10;
const uint8_t RIGHT_ENABLE_PIN = 31;

// Encoder A pins have external interrupts; B pins determine direction.
const uint8_t LEFT_FRONT_A_PIN = 2;
const uint8_t LEFT_FRONT_B_PIN = 22;
const uint8_t LEFT_REAR_A_PIN = 3;
const uint8_t LEFT_REAR_B_PIN = 23;
const uint8_t RIGHT_FRONT_A_PIN = 18;
const uint8_t RIGHT_FRONT_B_PIN = 24;
const uint8_t RIGHT_REAR_A_PIN = 19;
const uint8_t RIGHT_REAR_B_PIN = 25;

const uint8_t START_PIN = 32;
const uint8_t ESTOP_PIN = 33;
const uint8_t STATUS_LED_PIN = 34;

// Timing and transport.
const unsigned long SERIAL_BAUD = 500000UL;
const unsigned long HOST_TIMEOUT_MS = 300UL;
const unsigned long CONTROL_PERIOD_US = 10000UL;
const unsigned long SPEED_FILTER_PERIOD_US = 40000UL;
const unsigned long TELEMETRY_PERIOD_MS = 20UL;
// START/ARM button: local arming on a debounced press.  It never commands
// motion; the host must keep sending traffic or HOST_TIMEOUT disarms again.
const unsigned long START_DEBOUNCE_MS = 50UL;

const float TWO_PI_F = 6.28318530718f;
const float DEG_TO_RAD_F = 0.01745329252f;

// Starting gains for physical bring-up.  They require lifted-wheel tuning.
const float SPEED_KP_DUTY_PER_RAD_S = 0.2144772f;
const float SPEED_KI_DUTY_PER_RAD = 0.0938338f;
const float ANTI_WINDUP_GAIN = 4.0f;
const float YAW_KP = 2.0f;
const float YAW_KI = 0.8f;
const float YAW_CORRECTION_LIMIT_RAD_S = 1.5f;

enum ControllerState : uint8_t {
  DISARMED = 0,
  ARMED = 1,
  ESTOPPED = 2,
  FAULTED = 3
};

enum DriveMode : uint8_t {
  OPEN = 0,
  GYRO = 1
};

ControllerState controllerState = DISARMED;
DriveMode driveMode = OPEN;

volatile int32_t countLeftFront = 0;
volatile int32_t countLeftRear = 0;
volatile int32_t countRightFront = 0;
volatile int32_t countRightRear = 0;

int8_t signLeftFront = 1;
int8_t signLeftRear = 1;
int8_t signRightFront = 1;
int8_t signRightRear = 1;

float countsPerWheelRevolution = 0.0f;
float wheelRadiusM = 0.085f;
float trackM = 0.585f;
float pwmCeiling = 1.0f;
float noLoadSpeedRadS = 122.0f * TWO_PI_F / 60.0f;
float accelerationLimitMps2 = 0.35f;
float yawAccelerationLimitRadS2 = 0.8f;
float maximumRunSeconds = 15.0f;
float maximumRunDistanceM = 4.0f;

float commandV = 0.0f;
float commandW = 0.0f;
float rampV = 0.0f;
float rampW = 0.0f;
float dutyLeft = 0.0f;
float dutyRight = 0.0f;
float speedIntegralLeft = 0.0f;
float speedIntegralRight = 0.0f;
float yawIntegral = 0.0f;

float speedLeftFront = 0.0f;
float speedLeftRear = 0.0f;
float speedRightFront = 0.0f;
float speedRightRear = 0.0f;
float gyroZRadS = 0.0f;
float gyroBiasRadS = 0.0f;
float integratedGyroYawRad = 0.0f;
float odomX = 0.0f;
float odomY = 0.0f;
float odomYaw = 0.0f;

int16_t rawGyroZ = 0;
bool imuPresent = false;
uint8_t imuAddress = 0x68;

int32_t previousFilterLeftFront = 0;
int32_t previousFilterLeftRear = 0;
int32_t previousFilterRightFront = 0;
int32_t previousFilterRightRear = 0;

unsigned long lastHostMs = 0;
unsigned long lastControlUs = 0;
unsigned long lastSpeedFilterUs = 0;
unsigned long lastTelemetryMs = 0;
unsigned long lastImuUs = 0;
unsigned long motionStartMs = 0;
unsigned long previousLoopUs = 0;
unsigned long maximumLoopGapUs = 0;
unsigned long maximumTelemetryWriteUs = 0;

bool startRawLast = false;
bool startStablePressed = false;
unsigned long startChangeMs = 0;

float motionDistanceM = 0.0f;
char commandBuffer[112];
uint8_t commandLength = 0;

void encoderLeftFrontIsr() {
  countLeftFront += digitalRead(LEFT_FRONT_B_PIN) ? signLeftFront : -signLeftFront;
}

void encoderLeftRearIsr() {
  countLeftRear += digitalRead(LEFT_REAR_B_PIN) ? signLeftRear : -signLeftRear;
}

void encoderRightFrontIsr() {
  countRightFront += digitalRead(RIGHT_FRONT_B_PIN) ? signRightFront : -signRightFront;
}

void encoderRightRearIsr() {
  countRightRear += digitalRead(RIGHT_REAR_B_PIN) ? signRightRear : -signRightRear;
}

bool estopHealthy() {
  return digitalRead(ESTOP_PIN) == LOW;
}

float clampFloat(float value, float lower, float upper) {
  return value < lower ? lower : (value > upper ? upper : value);
}

float approach(float value, float target, float maximumStep) {
  if (value < target) {
    return min(value + maximumStep, target);
  }
  return max(value - maximumStep, target);
}

void applySide(uint8_t rpwmPin, uint8_t lpwmPin, float duty) {
  duty = clampFloat(duty, -pwmCeiling, pwmCeiling);
  uint8_t pwm = static_cast<uint8_t>(fabs(duty) * 255.0f);

  if (duty > 0.015f) {
    analogWrite(rpwmPin, pwm);
    analogWrite(lpwmPin, 0);
  } else if (duty < -0.015f) {
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, pwm);
  } else {
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, 0);
  }
}

void outputsOff() {
  dutyLeft = 0.0f;
  dutyRight = 0.0f;
  applySide(LEFT_RPWM_PIN, LEFT_LPWM_PIN, 0.0f);
  applySide(RIGHT_RPWM_PIN, RIGHT_LPWM_PIN, 0.0f);
}

void stopMotion() {
  commandV = 0.0f;
  commandW = 0.0f;
  rampV = 0.0f;
  rampW = 0.0f;
  speedIntegralLeft = 0.0f;
  speedIntegralRight = 0.0f;
  yawIntegral = 0.0f;
  motionStartMs = 0;
  motionDistanceM = 0.0f;
  outputsOff();
}

void disableBridges() {
  digitalWrite(LEFT_ENABLE_PIN, LOW);
  digitalWrite(RIGHT_ENABLE_PIN, LOW);
}

void disarm(const __FlashStringHelper *reason) {
  stopMotion();
  disableBridges();
  controllerState = estopHealthy() ? DISARMED : ESTOPPED;
  digitalWrite(STATUS_LED_PIN, LOW);
  Serial.print(F("OK,DISARMED,"));
  Serial.println(reason);
}

void fault(const __FlashStringHelper *reason) {
  stopMotion();
  disableBridges();
  controllerState = FAULTED;
  digitalWrite(STATUS_LED_PIN, LOW);
  Serial.print(F("FAULT,"));
  Serial.println(reason);
}

void enterEstop(const __FlashStringHelper *reason) {
  stopMotion();
  disableBridges();
  controllerState = ESTOPPED;
  digitalWrite(STATUS_LED_PIN, LOW);
  Serial.print(F("ESTOP,"));
  Serial.println(reason);
}

void armController(const __FlashStringHelper *source) {
  if (!estopHealthy()) {
    enterEstop(F("OPEN_CIRCUIT"));
    return;
  }
  if (countsPerWheelRevolution <= 0.0f) {
    Serial.println(F("ERR,CPR_REQUIRED"));
    return;
  }

  stopMotion();
  controllerState = ARMED;
  digitalWrite(LEFT_ENABLE_PIN, HIGH);
  digitalWrite(RIGHT_ENABLE_PIN, HIGH);
  digitalWrite(STATUS_LED_PIN, HIGH);
  lastHostMs = millis();
  Serial.print(F("OK,ARMED,"));
  Serial.println(source);
}

bool imuRead(uint8_t registerAddress, uint8_t *data, uint8_t length) {
  Wire.beginTransmission(imuAddress);
  Wire.write(registerAddress);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(imuAddress, length) != length) {
    return false;
  }
  for (uint8_t index = 0; index < length; ++index) {
    data[index] = Wire.read();
  }
  return true;
}

bool imuWrite(uint8_t registerAddress, uint8_t value) {
  Wire.beginTransmission(imuAddress);
  Wire.write(registerAddress);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

void initializeImu() {
  Wire.begin();
  Wire.setClock(100000);

  uint8_t whoAmI = 0;
  imuPresent = imuRead(0x75, &whoAmI, 1) && (whoAmI == 0x68 || whoAmI == 0x69);
  if (!imuPresent) {
    imuAddress = 0x69;
    imuPresent = imuRead(0x75, &whoAmI, 1) && (whoAmI == 0x68 || whoAmI == 0x69);
  }
  if (imuPresent) {
    imuPresent = imuWrite(0x6B, 0) && imuWrite(0x1B, 0);
  }
}

void updateImu(unsigned long nowUs) {
  if (!imuPresent || nowUs - lastImuUs < 5000UL) {
    return;
  }
  lastImuUs = nowUs;

  uint8_t data[2];
  if (!imuRead(0x47, data, 2)) {
    imuPresent = false;
    return;
  }

  rawGyroZ = static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
  gyroZRadS = (static_cast<float>(rawGyroZ) / 131.0f) * DEG_TO_RAD_F - gyroBiasRadS;
}

void snapshotCounts(int32_t &leftFront, int32_t &leftRear,
                    int32_t &rightFront, int32_t &rightRear) {
  noInterrupts();
  leftFront = countLeftFront;
  leftRear = countLeftRear;
  rightFront = countRightFront;
  rightRear = countRightRear;
  interrupts();
}

void updateFilteredSpeeds(unsigned long nowUs) {
  if (nowUs - lastSpeedFilterUs < SPEED_FILTER_PERIOD_US) {
    return;
  }

  float elapsedSeconds = (nowUs - lastSpeedFilterUs) * 1.0e-6f;
  lastSpeedFilterUs = nowUs;

  int32_t leftFront;
  int32_t leftRear;
  int32_t rightFront;
  int32_t rightRear;
  snapshotCounts(leftFront, leftRear, rightFront, rightRear);

  if (countsPerWheelRevolution > 0.0f && elapsedSeconds > 0.0f) {
    float scale = TWO_PI_F / (countsPerWheelRevolution * elapsedSeconds);
    speedLeftFront = (leftFront - previousFilterLeftFront) * scale;
    speedLeftRear = (leftRear - previousFilterLeftRear) * scale;
    speedRightFront = (rightFront - previousFilterRightFront) * scale;
    speedRightRear = (rightRear - previousFilterRightRear) * scale;
  }

  previousFilterLeftFront = leftFront;
  previousFilterLeftRear = leftRear;
  previousFilterRightFront = rightFront;
  previousFilterRightRear = rightRear;
}

float calculateWheelDuty(float targetRadS, float measuredRadS,
                         float &integralState, float dt) {
  float feedForward = targetRadS / max(noLoadSpeedRadS, 0.1f);
  float error = targetRadS - measuredRadS;
  float unsaturated = feedForward + SPEED_KP_DUTY_PER_RAD_S * error
                      + SPEED_KI_DUTY_PER_RAD * integralState;
  float saturated = clampFloat(unsaturated, -pwmCeiling, pwmCeiling);

  // Back-calculation unwinds the integrator whenever the voltage command clips.
  float antiWindup = ANTI_WINDUP_GAIN * (saturated - unsaturated)
                     / SPEED_KI_DUTY_PER_RAD;
  integralState += (error + antiWindup) * dt;

  // The integral may use the complete configured duty range.
  float integralLimit = pwmCeiling / SPEED_KI_DUTY_PER_RAD;
  integralState = clampFloat(integralState, -integralLimit, integralLimit);

  unsaturated = feedForward + SPEED_KP_DUTY_PER_RAD_S * error
                + SPEED_KI_DUTY_PER_RAD * integralState;
  return clampFloat(unsaturated, -pwmCeiling, pwmCeiling);
}

void controlTick(unsigned long nowUs) {
  float dt = (nowUs - lastControlUs) * 1.0e-6f;
  if (lastControlUs == 0 || dt <= 0.0f || dt > 0.1f) {
    dt = 0.01f;
  }
  lastControlUs = nowUs;

  float leftSpeed = 0.5f * (speedLeftFront + speedLeftRear);
  float rightSpeed = 0.5f * (speedRightFront + speedRightRear);
  float distanceStep = wheelRadiusM * 0.5f * (leftSpeed + rightSpeed) * dt;
  float yawStep = wheelRadiusM * (rightSpeed - leftSpeed) / trackM * dt;

  odomYaw += yawStep;
  odomX += distanceStep * cos(odomYaw);
  odomY += distanceStep * sin(odomYaw);
  motionDistanceM += fabs(distanceStep);
  integratedGyroYawRad += gyroZRadS * dt;

  if (controllerState != ARMED) {
    outputsOff();
    return;
  }

  rampV = approach(rampV, commandV, accelerationLimitMps2 * dt);
  rampW = approach(rampW, commandW, yawAccelerationLimitRadS2 * dt);

  float effectiveW = rampW;
  if (driveMode == GYRO && fabs(rampW) > 1.0e-4f) {
    float yawError = rampW - gyroZRadS;
    yawIntegral = clampFloat(yawIntegral + yawError * dt, -1.5f, 1.5f);
    float correction = YAW_KP * yawError + YAW_KI * yawIntegral;
    effectiveW += clampFloat(correction, -YAW_CORRECTION_LIMIT_RAD_S,
                            YAW_CORRECTION_LIMIT_RAD_S);
  } else {
    yawIntegral = 0.0f;
  }

  float leftTarget = (rampV - effectiveW * trackM * 0.5f) / wheelRadiusM;
  float rightTarget = (rampV + effectiveW * trackM * 0.5f) / wheelRadiusM;
  dutyLeft = calculateWheelDuty(leftTarget, leftSpeed, speedIntegralLeft, dt);
  dutyRight = calculateWheelDuty(rightTarget, rightSpeed, speedIntegralRight, dt);

  applySide(LEFT_RPWM_PIN, LEFT_LPWM_PIN, dutyLeft);
  applySide(RIGHT_RPWM_PIN, RIGHT_LPWM_PIN, dutyRight);

  if ((fabs(commandV) > 1.0e-4f || fabs(commandW) > 1.0e-4f)
      && motionStartMs == 0) {
    motionStartMs = millis();
  }

  bool timeLimitReached = maximumRunSeconds > 0.0f && motionStartMs != 0
                          && millis() - motionStartMs > maximumRunSeconds * 1000.0f;
  bool distanceLimitReached = maximumRunDistanceM > 0.0f
                              && motionDistanceM > maximumRunDistanceM;
  if (timeLimitReached || distanceLimitReached) {
    fault(F("RUN_LIMIT"));
  }
}

void printTelemetry(unsigned long nowMs) {
  unsigned long writeStartUs = micros();
  int32_t leftFront;
  int32_t leftRear;
  int32_t rightFront;
  int32_t rightRear;
  snapshotCounts(leftFront, leftRear, rightFront, rightRear);

  Serial.print(F("T,"));
  Serial.print(nowMs);
  Serial.print(',');
  Serial.print(static_cast<int>(controllerState));
  Serial.print(',');
  Serial.print(driveMode == GYRO ? F("GYRO") : F("OPEN"));

  const float firstValues[] = {
    commandV, commandW, rampV, rampW, dutyLeft, dutyRight
  };
  for (float value : firstValues) {
    Serial.print(',');
    Serial.print(value, 6);
  }

  Serial.print(',');
  Serial.print(leftFront);
  Serial.print(',');
  Serial.print(leftRear);
  Serial.print(',');
  Serial.print(rightFront);
  Serial.print(',');
  Serial.print(rightRear);

  const float secondValues[] = {
    speedLeftFront, speedLeftRear, speedRightFront, speedRightRear,
    gyroZRadS, integratedGyroYawRad, odomX, odomY
  };
  for (float value : secondValues) {
    Serial.print(',');
    Serial.print(value, 6);
  }

  Serial.print(',');
  Serial.print(estopHealthy() ? 0 : 1);
  Serial.print(',');
  Serial.print(maximumLoopGapUs);
  Serial.print(',');
  Serial.println(maximumTelemetryWriteUs);

  unsigned long writeDurationUs = micros() - writeStartUs;
  maximumTelemetryWriteUs = max(maximumTelemetryWriteUs, writeDurationUs);
}

void zeroOdometry() {
  noInterrupts();
  countLeftFront = 0;
  countLeftRear = 0;
  countRightFront = 0;
  countRightRear = 0;
  interrupts();

  previousFilterLeftFront = 0;
  previousFilterLeftRear = 0;
  previousFilterRightFront = 0;
  previousFilterRightRear = 0;
  odomX = 0.0f;
  odomY = 0.0f;
  odomYaw = 0.0f;
}

void handleCommand(char *command) {
  lastHostMs = millis();

  if (strcmp(command, "PING") == 0) {
    Serial.println(F("PONG"));
    return;
  }
  if (strcmp(command, "ARM") == 0) {
    armController(F("HOST"));
    return;
  }
  if (strcmp(command, "DISARM") == 0) {
    disarm(F("HOST"));
    return;
  }
  if (strcmp(command, "STOP") == 0) {
    stopMotion();
    Serial.println(F("OK,STOP"));
    return;
  }
  if (strcmp(command, "ZERO") == 0) {
    zeroOdometry();
    Serial.println(F("OK,ZERO"));
    return;
  }
  if (strcmp(command, "ZERO_GYRO") == 0) {
    gyroBiasRadS = (static_cast<float>(rawGyroZ) / 131.0f) * DEG_TO_RAD_F;
    integratedGyroYawRad = 0.0f;
    Serial.println(F("OK,ZERO_GYRO"));
    return;
  }

  char word[12];
  float first;
  float second;
  if (sscanf(command, "CMD %f %f", &first, &second) == 2) {
    if (controllerState != ARMED) {
      Serial.println(F("ERR,NOT_ARMED"));
      return;
    }
    commandV = clampFloat(first, -0.8f, 0.8f);
    commandW = clampFloat(second, -1.5f, 1.5f);
    Serial.println(F("OK,CMD"));
    return;
  }
  if (sscanf(command, "MODE %11s", word) == 1) {
    if (strcmp(word, "OPEN") == 0) {
      driveMode = OPEN;
    } else if (strcmp(word, "GYRO") == 0 && imuPresent) {
      driveMode = GYRO;
    } else {
      Serial.println(F("ERR,MODE"));
      return;
    }
    Serial.println(F("OK,MODE"));
    return;
  }
  if (sscanf(command, "CFG CPR %f", &first) == 1) {
    countsPerWheelRevolution = max(0.0f, first);
    Serial.println(F("OK,CPR"));
    return;
  }
  if (sscanf(command, "CFG RADIUS %f", &first) == 1) {
    wheelRadiusM = clampFloat(first, 0.05f, 0.12f);
    Serial.println(F("OK,RADIUS"));
    return;
  }
  if (sscanf(command, "CFG TRACK %f", &first) == 1) {
    trackM = clampFloat(first, 0.3f, 0.9f);
    Serial.println(F("OK,TRACK"));
    return;
  }
  if (sscanf(command, "CFG PWM %f", &first) == 1) {
    pwmCeiling = clampFloat(first, 0.0f, 1.0f);
    Serial.println(F("OK,PWM"));
    return;
  }
  if (sscanf(command, "CFG NOLOAD %f", &first) == 1) {
    noLoadSpeedRadS = clampFloat(first, 0.1f, 100.0f);
    Serial.println(F("OK,NOLOAD"));
    return;
  }
  if (sscanf(command, "CFG ACCEL %f %f", &first, &second) == 2) {
    accelerationLimitMps2 = clampFloat(first, 0.02f, 2.0f);
    yawAccelerationLimitRadS2 = clampFloat(second, 0.05f, 4.0f);
    Serial.println(F("OK,ACCEL"));
    return;
  }
  if (sscanf(command, "LIMIT %f %f", &first, &second) == 2) {
    maximumRunSeconds = max(0.0f, first);
    maximumRunDistanceM = max(0.0f, second);
    Serial.println(F("OK,LIMIT"));
    return;
  }

  Serial.println(F("ERR,UNKNOWN"));
}

void serviceSerial() {
  while (Serial.available() > 0) {
    char character = Serial.read();
    if (character == '\n' || character == '\r') {
      if (commandLength > 0) {
        commandBuffer[commandLength] = '\0';
        handleCommand(commandBuffer);
        commandLength = 0;
      }
    } else if (commandLength < sizeof(commandBuffer) - 1) {
      commandBuffer[commandLength++] = character;
    } else {
      commandLength = 0;
    }
  }
}

// Debounced START press arms locally (same checks as host ARM: E-stop
// closed and CPR configured) and reports it to the host.  Holding the button
// or pressing it while armed does nothing more.
void serviceStartButton(unsigned long nowMs) {
  bool pressed = digitalRead(START_PIN) == LOW;
  if (pressed != startRawLast) {
    startRawLast = pressed;
    startChangeMs = nowMs;
  }
  if (nowMs - startChangeMs < START_DEBOUNCE_MS || pressed == startStablePressed) {
    return;
  }
  startStablePressed = pressed;
  if (!pressed) {
    return;
  }
  Serial.println(F("EVENT,START_PRESSED"));
  if (controllerState != ARMED) {
    armController(F("BUTTON"));
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);

  pinMode(LEFT_RPWM_PIN, OUTPUT);
  pinMode(LEFT_LPWM_PIN, OUTPUT);
  pinMode(RIGHT_RPWM_PIN, OUTPUT);
  pinMode(RIGHT_LPWM_PIN, OUTPUT);
  pinMode(LEFT_ENABLE_PIN, OUTPUT);
  pinMode(RIGHT_ENABLE_PIN, OUTPUT);
  pinMode(LEFT_FRONT_A_PIN, INPUT_PULLUP);
  pinMode(LEFT_FRONT_B_PIN, INPUT_PULLUP);
  pinMode(LEFT_REAR_A_PIN, INPUT_PULLUP);
  pinMode(LEFT_REAR_B_PIN, INPUT_PULLUP);
  pinMode(RIGHT_FRONT_A_PIN, INPUT_PULLUP);
  pinMode(RIGHT_FRONT_B_PIN, INPUT_PULLUP);
  pinMode(RIGHT_REAR_A_PIN, INPUT_PULLUP);
  pinMode(RIGHT_REAR_B_PIN, INPUT_PULLUP);
  pinMode(START_PIN, INPUT_PULLUP);
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  pinMode(STATUS_LED_PIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(LEFT_FRONT_A_PIN), encoderLeftFrontIsr, RISING);
  attachInterrupt(digitalPinToInterrupt(LEFT_REAR_A_PIN), encoderLeftRearIsr, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_FRONT_A_PIN), encoderRightFrontIsr, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_REAR_A_PIN), encoderRightRearIsr, RISING);

  disableBridges();
  outputsOff();
  initializeImu();

  lastSpeedFilterUs = micros();
  previousLoopUs = micros();
  if (!estopHealthy()) {
    controllerState = ESTOPPED;
  }
  // A button held during boot is not treated as a press.
  startRawLast = digitalRead(START_PIN) == LOW;
  startStablePressed = startRawLast;
  startChangeMs = millis();

  Serial.print(F("READY,DRIVE_TEST_MEGA,CPR_REQUIRED,ESTOP="));
  Serial.print(estopHealthy() ? F("OK") : F("OPEN"));
  Serial.print(F(",IMU="));
  Serial.println(imuPresent ? F("OK") : F("MISSING"));
}

void loop() {
  unsigned long nowUs = micros();
  unsigned long nowMs = millis();
  unsigned long loopGapUs = nowUs - previousLoopUs;
  previousLoopUs = nowUs;
  maximumLoopGapUs = max(maximumLoopGapUs, loopGapUs);

  // Service host commands first.  At 500 kbaud a ~220-character telemetry line
  // occupies about 4.4 ms on the wire, keeping STOP below the 20 ms requirement.
  serviceSerial();
  serviceStartButton(nowMs);

  if (!estopHealthy() && controllerState != ESTOPPED) {
    enterEstop(F("OPEN_CIRCUIT"));
  }
  if (controllerState == ARMED && nowMs - lastHostMs > HOST_TIMEOUT_MS) {
    fault(F("HOST_TIMEOUT"));
  }

  updateImu(nowUs);
  updateFilteredSpeeds(nowUs);
  if (nowUs - lastControlUs >= CONTROL_PERIOD_US) {
    controlTick(nowUs);
  }

  if (nowMs - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = nowMs;
    printTelemetry(nowMs);
  }
}
