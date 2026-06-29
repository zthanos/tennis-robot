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
const byte PWM_STEP = 10;
const unsigned long COMMAND_TIMEOUT_MS = 1000;

// Safe bring-up default. Increase gradually from the UI after confirming that
// the TB6612 and 12 V supply remain stable under continuous load.
byte motorPwm = 75;
char motorMode = 's';
unsigned long lastCommandMs = 0;

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

void setup() {
  Serial.begin(9600);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_AIN1_PIN, OUTPUT);
  pinMode(MOTOR_AIN2_PIN, OUTPUT);
  pinMode(MOTOR_STBY_PIN, OUTPUT);
  digitalWrite(MOTOR_STBY_PIN, LOW);
  motorCoast();
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
    } else if (command == '+') {
      motorPwm = min(255, motorPwm + PWM_STEP);
      applyMotor();
    } else if (command == '-') {
      motorPwm = max(0, motorPwm - PWM_STEP);
      applyMotor();
    }
  }
  if (motorMode != 's' && millis() - lastCommandMs > COMMAND_TIMEOUT_MS) {
    motorCoast();
  }
}
