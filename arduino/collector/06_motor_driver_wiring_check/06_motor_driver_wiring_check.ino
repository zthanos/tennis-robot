/*
  Collector Motor Driver Wiring Check

  This sketch checks only:
    Arduino Nano -> TB6612FNG -> 12V motor

  It ignores IR sensors and encoder on purpose.

  Wiring:
    Arduino D3  -> TB6612 PWMA
    Arduino D4  -> TB6612 AIN1
    Arduino D5  -> TB6612 AIN2
    Arduino D6  -> TB6612 STBY
    Arduino 5V  -> TB6612 VCC
    Arduino GND -> TB6612 GND
    12V PSU -   -> TB6612 GND / Arduino GND
    12V PSU +   -> TB6612 VM
    Motor lead 1 -> TB6612 AO1
    Motor lead 2 -> TB6612 AO2

  Important:
    Do not connect 12V to VCC.
    Do not connect 12V directly to the Arduino.
    Connect the 12V PSU minus to the common GND.

  Serial Monitor:
    Baud rate: 115200
    Line ending: No line ending

  Commands:
    v = print voltage checklist
    t = toggle TB6612 STBY enable/disable
    p = one short forward pulse, then stop
    f = forward low PWM
    r = reverse low PWM
    s = stop/coast
    b = brake
    + = increase PWM by 10, max 120
    - = decrease PWM by 10
    ? = help
*/

const byte MOTOR_PWM_PIN = 3;
const byte MOTOR_AIN1_PIN = 4;
const byte MOTOR_AIN2_PIN = 5;
const byte MOTOR_STBY_PIN = 6;

const byte START_PWM = 60;
const byte MAX_SAFE_PWM = 120;
const unsigned int PULSE_MS = 300;

byte motorPwm = START_PWM;
bool driverEnabled = false;
char motorMode = 's';

void writeStandby() {
  digitalWrite(MOTOR_STBY_PIN, driverEnabled ? HIGH : LOW);
}

void motorCoast() {
  analogWrite(MOTOR_PWM_PIN, 0);
  digitalWrite(MOTOR_AIN1_PIN, LOW);
  digitalWrite(MOTOR_AIN2_PIN, LOW);
  motorMode = 's';
}

void motorBrake() {
  analogWrite(MOTOR_PWM_PIN, 0);
  digitalWrite(MOTOR_AIN1_PIN, HIGH);
  digitalWrite(MOTOR_AIN2_PIN, HIGH);
  motorMode = 'b';
}

void motorForward() {
  driverEnabled = true;
  writeStandby();
  digitalWrite(MOTOR_AIN1_PIN, HIGH);
  digitalWrite(MOTOR_AIN2_PIN, LOW);
  analogWrite(MOTOR_PWM_PIN, motorPwm);
  motorMode = 'f';
}

void motorReverse() {
  driverEnabled = true;
  writeStandby();
  digitalWrite(MOTOR_AIN1_PIN, LOW);
  digitalWrite(MOTOR_AIN2_PIN, HIGH);
  analogWrite(MOTOR_PWM_PIN, motorPwm);
  motorMode = 'r';
}

void applyCurrentMode() {
  if (motorMode == 'f') {
    motorForward();
  } else if (motorMode == 'r') {
    motorReverse();
  } else if (motorMode == 'b') {
    motorBrake();
  } else {
    motorCoast();
  }
}

void printVoltageChecklist() {
  Serial.println();
  Serial.println("Measure these with a multimeter:");
  Serial.println("  1. TB6612 VCC -> TB6612 GND = about 5V");
  Serial.println("  2. TB6612 VM  -> TB6612 GND = about 12V");
  Serial.println("  3. Arduino GND -> TB6612 GND = continuity / almost 0 ohm");
  Serial.println("  4. 12V PSU - -> Arduino GND = continuity / almost 0 ohm");
  Serial.println("  5. Motor wires go only to AO1 and AO2");
  Serial.println();
  Serial.println("Expected motor driver wiring:");
  Serial.println("  Arduino 5V  -> TB6612 VCC");
  Serial.println("  Arduino GND -> TB6612 GND -> 12V PSU -");
  Serial.println("  12V PSU +   -> TB6612 VM");
  Serial.println("  Motor       -> TB6612 AO1/AO2");
  Serial.println();
}

void printStatus() {
  Serial.print("status: STBY=");
  Serial.print(driverEnabled ? "HIGH/enabled" : "LOW/disabled");
  Serial.print(" mode=");
  Serial.print(motorMode);
  Serial.print(" pwm=");
  Serial.print(motorPwm);
  Serial.print(" pins: D3/PWMA pwm ");
  Serial.print(motorMode == 'f' || motorMode == 'r' ? motorPwm : 0);
  Serial.print(" D4/AIN1=");
  Serial.print(digitalRead(MOTOR_AIN1_PIN));
  Serial.print(" D5/AIN2=");
  Serial.print(digitalRead(MOTOR_AIN2_PIN));
  Serial.print(" D6/STBY=");
  Serial.println(digitalRead(MOTOR_STBY_PIN));
}

void printHelp() {
  Serial.println();
  Serial.println("Collector motor driver wiring check commands:");
  Serial.println("  v = voltage checklist");
  Serial.println("  t = toggle STBY enable/disable");
  Serial.println("  p = one short forward pulse, then stop");
  Serial.println("  f = forward low PWM");
  Serial.println("  r = reverse low PWM");
  Serial.println("  s = stop/coast");
  Serial.println("  b = brake");
  Serial.println("  + = PWM +10, max 120");
  Serial.println("  - = PWM -10");
  Serial.println("  ? = help");
  Serial.println();
}

void pulseMotor() {
  Serial.print("pulse: forward pwm=");
  Serial.print(motorPwm);
  Serial.print(" for ");
  Serial.print(PULSE_MS);
  Serial.println("ms");

  motorForward();
  delay(PULSE_MS);
  motorCoast();

  Serial.println("pulse done, motor stopped");
}

void toggleStandby() {
  driverEnabled = !driverEnabled;
  if (!driverEnabled) {
    motorCoast();
  }
  writeStandby();
  Serial.print("STBY is now ");
  Serial.println(driverEnabled ? "HIGH/enabled" : "LOW/disabled");
}

void handleSerial() {
  while (Serial.available() > 0) {
    char cmd = Serial.read();

    if (cmd == 'v') {
      printVoltageChecklist();
    } else if (cmd == 't') {
      toggleStandby();
    } else if (cmd == 'p') {
      pulseMotor();
    } else if (cmd == 'f') {
      motorForward();
      Serial.println("cmd: forward");
    } else if (cmd == 'r') {
      motorReverse();
      Serial.println("cmd: reverse");
    } else if (cmd == 's') {
      motorCoast();
      Serial.println("cmd: stop/coast");
    } else if (cmd == 'b') {
      motorBrake();
      Serial.println("cmd: brake");
    } else if (cmd == '+') {
      motorPwm = min(MAX_SAFE_PWM, motorPwm + 10);
      applyCurrentMode();
      Serial.print("cmd: pwm=");
      Serial.println(motorPwm);
    } else if (cmd == '-') {
      motorPwm = max(0, motorPwm - 10);
      applyCurrentMode();
      Serial.print("cmd: pwm=");
      Serial.println(motorPwm);
    } else if (cmd == '?') {
      printHelp();
    }

    printStatus();
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_AIN1_PIN, OUTPUT);
  pinMode(MOTOR_AIN2_PIN, OUTPUT);
  pinMode(MOTOR_STBY_PIN, OUTPUT);

  driverEnabled = false;
  writeStandby();
  motorCoast();

  Serial.println("Collector motor driver wiring check ready");
  Serial.println("This sketch checks only Arduino -> TB6612 -> motor.");
  Serial.println("Send '?' for commands. Start with 'v' and measure voltages.");
  printVoltageChecklist();
  printStatus();
}

void loop() {
  static unsigned long lastPrintMs = 0;

  handleSerial();

  unsigned long now = millis();
  if (now - lastPrintMs >= 2000) {
    printStatus();
    lastPrintMs = now;
  }
}
