/*
  Collector runtime driver

  Preserves the hardware-validated 06_motor_driver_wiring_check contract:
    9600 baud, no line ending, f/r/s/+/- single-byte commands.
    D3=PWMA, D4=AIN1, D5=AIN2, D6=STBY.
*/
const byte MOTOR_PWM_PIN = 3;
const byte MOTOR_AIN1_PIN = 4;
const byte MOTOR_AIN2_PIN = 5;
const byte MOTOR_STBY_PIN = 6;
const byte IR_ENTRY_PIN = 9;
const byte IR_EXIT_PIN = 10;
const byte PWM_STEP = 10;
const unsigned long COMMAND_TIMEOUT_MS = 1000;
const unsigned long IR_DEBOUNCE_MS = 25;
const unsigned long IR_HEARTBEAT_MS = 500;
const unsigned long AUTOMATIC_CYCLE_TIMEOUT_MS = 5000;

// Safe bring-up default. Increase gradually from the UI after confirming that
// the TB6612 and 12 V supply remain stable under continuous load.
byte motorPwm = 75;
char motorMode = 's';
unsigned long lastCommandMs = 0;

enum CollectionCycleState {
  WAITING_FOR_ENTRY,
  MOVING_TO_EXIT,
  WAITING_FOR_EXIT_CLEAR,
  WAITING_FOR_ENTRY_CLEAR,
  CYCLE_TIMED_OUT
};

CollectionCycleState collectionState = WAITING_FOR_ENTRY;
bool automaticCycleActive = false;
unsigned long automaticCycleStartedMs = 0;

struct DebouncedBeam {
  byte pin;
  bool rawBroken;
  bool stableBroken;
  unsigned long rawChangedMs;
};

DebouncedBeam entryBeam = {IR_ENTRY_PIN, false, false, 0};
DebouncedBeam exitBeam = {IR_EXIT_PIN, false, false, 0};
unsigned long lastIrHeartbeatMs = 0;

const char *collectionStateName() {
  switch (collectionState) {
    case WAITING_FOR_ENTRY:
      return "waiting_entry";
    case MOVING_TO_EXIT:
      return "moving_to_exit";
    case WAITING_FOR_EXIT_CLEAR:
      return "waiting_exit_clear";
    case WAITING_FOR_ENTRY_CLEAR:
      return "waiting_entry_clear";
    case CYCLE_TIMED_OUT:
      return "timed_out";
  }
  return "unknown";
}

bool readBeamBroken(byte pin) {
  // Adafruit break-beam receivers are open-collector and active LOW.
  return digitalRead(pin) == LOW;
}

bool updateBeam(DebouncedBeam &beam, unsigned long now) {
  bool rawBroken = readBeamBroken(beam.pin);
  if (rawBroken != beam.rawBroken) {
    beam.rawBroken = rawBroken;
    beam.rawChangedMs = now;
  }
  if (beam.stableBroken != beam.rawBroken &&
      now - beam.rawChangedMs >= IR_DEBOUNCE_MS) {
    beam.stableBroken = beam.rawBroken;
    return true;
  }
  return false;
}

void publishIrStatus() {
  // Machine-readable line consumed by SerialCollectorDriver.
  Serial.print("ir:");
  Serial.print(entryBeam.stableBroken ? 1 : 0);
  Serial.print(",");
  Serial.print(exitBeam.stableBroken ? 1 : 0);
  Serial.print(",");
  Serial.println(collectionStateName());
}

void motorCoast() {
  analogWrite(MOTOR_PWM_PIN, 0);
  digitalWrite(MOTOR_AIN1_PIN, LOW);
  digitalWrite(MOTOR_AIN2_PIN, LOW);
  motorMode = 's';
}

void applyMotor() {
  digitalWrite(MOTOR_STBY_PIN, HIGH);
  if (motorMode == 'f') {
    digitalWrite(MOTOR_AIN1_PIN, HIGH);
    digitalWrite(MOTOR_AIN2_PIN, LOW);
  } else if (motorMode == 'r') {
    digitalWrite(MOTOR_AIN1_PIN, LOW);
    digitalWrite(MOTOR_AIN2_PIN, HIGH);
  } else {
    motorCoast();
    return;
  }
  analogWrite(MOTOR_PWM_PIN, motorPwm);
}

void updateAutomaticCollection() {
  switch (collectionState) {
    case WAITING_FOR_ENTRY:
      if (entryBeam.stableBroken) {
        motorMode = 'f';
        automaticCycleActive = true;
        automaticCycleStartedMs = millis();
        applyMotor();
        collectionState = exitBeam.stableBroken
          ? WAITING_FOR_EXIT_CLEAR
          : MOVING_TO_EXIT;
      }
      break;

    case MOVING_TO_EXIT:
      if (exitBeam.stableBroken) {
        collectionState = WAITING_FOR_EXIT_CLEAR;
      }
      break;

    case WAITING_FOR_EXIT_CLEAR:
      if (!exitBeam.stableBroken) {
        motorCoast();
        automaticCycleActive = false;
        collectionState = WAITING_FOR_ENTRY_CLEAR;
      }
      break;

    case WAITING_FOR_ENTRY_CLEAR:
      if (!entryBeam.stableBroken) {
        collectionState = WAITING_FOR_ENTRY;
      }
      break;

    case CYCLE_TIMED_OUT:
      if (!entryBeam.stableBroken && !exitBeam.stableBroken) {
        collectionState = WAITING_FOR_ENTRY;
      }
      break;
  }

  if (automaticCycleActive &&
      millis() - automaticCycleStartedMs >= AUTOMATIC_CYCLE_TIMEOUT_MS) {
    motorCoast();
    automaticCycleActive = false;
    collectionState = CYCLE_TIMED_OUT;
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_AIN1_PIN, OUTPUT);
  pinMode(MOTOR_AIN2_PIN, OUTPUT);
  pinMode(MOTOR_STBY_PIN, OUTPUT);
  pinMode(IR_ENTRY_PIN, INPUT_PULLUP);
  pinMode(IR_EXIT_PIN, INPUT_PULLUP);
  digitalWrite(MOTOR_STBY_PIN, LOW);
  motorCoast();
  entryBeam.rawBroken = entryBeam.stableBroken = readBeamBroken(IR_ENTRY_PIN);
  exitBeam.rawBroken = exitBeam.stableBroken = readBeamBroken(IR_EXIT_PIN);
  publishIrStatus();
}

void loop() {
  while (Serial.available() > 0) {
    char command = Serial.read();
    lastCommandMs = millis();
    if (command == 'f' || command == 'r') {
      motorMode = command;
      applyMotor();
    } else if (command == 's') {
      motorCoast();
      automaticCycleActive = false;
      collectionState = entryBeam.stableBroken
        ? WAITING_FOR_ENTRY_CLEAR
        : WAITING_FOR_ENTRY;
    } else if (command == '+') {
      motorPwm = min(255, motorPwm + PWM_STEP);
      applyMotor();
    } else if (command == '-') {
      motorPwm = max(0, motorPwm - PWM_STEP);
      applyMotor();
    }
  }
  if (!automaticCycleActive &&
      motorMode != 's' &&
      millis() - lastCommandMs > COMMAND_TIMEOUT_MS) {
    motorCoast();
  }

  unsigned long now = millis();
  bool irChanged = updateBeam(entryBeam, now);
  irChanged = updateBeam(exitBeam, now) || irChanged;
  CollectionCycleState previousState = collectionState;
  updateAutomaticCollection();
  bool cycleChanged = collectionState != previousState;
  if (irChanged || cycleChanged || now - lastIrHeartbeatMs >= IR_HEARTBEAT_MS) {
    publishIrStatus();
    lastIrHeartbeatMs = now;
  }
}
