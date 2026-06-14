/*
  Collector Wiring Diagnostic

  Purpose:
    Shows whether the first collector wiring is alive before running full logic.

  Pin map:
    D2  -> Encoder A
    D3  -> TB6612 PWMA
    D4  -> TB6612 AIN1
    D5  -> TB6612 AIN2
    D6  -> TB6612 STBY
    D7  -> Encoder B
    D9  -> IR #1 entry receiver signal
    D10 -> IR #2 exit receiver signal

  Power:
    Arduino 5V  -> TB6612 VCC, encoder VCC, IR VCC
    Arduino GND -> TB6612 GND -> 12V PSU -
    12V PSU +   -> TB6612 VM
    Motor red   -> TB6612 AO1
    Motor black -> TB6612 AO2

  Serial Monitor:
    Baud rate: 115200
    Line ending: No line ending

  Commands:
    i = print wiring status once
    p = short low-PWM forward pulse, then stop
    f = forward low PWM
    r = reverse low PWM
    s = stop/coast
    b = brake
    + = increase PWM by 10, max 120
    - = decrease PWM by 10
    ? = help

  Expected IR logic for Adafruit break beam receivers with INPUT_PULLUP:
    LOW  = beam broken / blocked
    HIGH = beam clear / unbroken
*/

const byte ENCODER_A_PIN = 2;
const byte MOTOR_PWM_PIN = 3;
const byte MOTOR_AIN1_PIN = 4;
const byte MOTOR_AIN2_PIN = 5;
const byte MOTOR_STBY_PIN = 6;
const byte ENCODER_B_PIN = 7;
const byte IR_ENTRY_PIN = 9;
const byte IR_EXIT_PIN = 10;

const byte START_PWM = 60;
const byte MAX_SAFE_PWM = 120;
const unsigned int PULSE_MS = 300;

volatile long encoderCount = 0;
byte motorPwm = START_PWM;
char motorMode = 's';

void onEncoderAChange() {
  bool a = digitalRead(ENCODER_A_PIN);
  bool b = digitalRead(ENCODER_B_PIN);

  if (a == b) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

bool beamBroken(byte pin) {
  return digitalRead(pin) == LOW;
}

const char *beamText(byte pin) {
  return beamBroken(pin) ? "BROKEN" : "CLEAR";
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
  digitalWrite(MOTOR_STBY_PIN, HIGH);
  digitalWrite(MOTOR_AIN1_PIN, HIGH);
  digitalWrite(MOTOR_AIN2_PIN, LOW);
  analogWrite(MOTOR_PWM_PIN, motorPwm);
  motorMode = 'f';
}

void motorReverse() {
  digitalWrite(MOTOR_STBY_PIN, HIGH);
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

void printHelp() {
  Serial.println();
  Serial.println("Collector wiring diagnostic commands:");
  Serial.println("  i = print wiring status once");
  Serial.println("  p = short low-PWM forward pulse, then stop");
  Serial.println("  f = forward low PWM");
  Serial.println("  r = reverse low PWM");
  Serial.println("  s = stop/coast");
  Serial.println("  b = brake");
  Serial.println("  + = PWM +10, max 120");
  Serial.println("  - = PWM -10");
  Serial.println("  ? = help");
  Serial.println();
}

void printStatus() {
  noInterrupts();
  long count = encoderCount;
  interrupts();

  Serial.println("----- wiring status -----");
  Serial.print("TB6612 control pins: PWMA=D");
  Serial.print(MOTOR_PWM_PIN);
  Serial.print(" AIN1=D");
  Serial.print(MOTOR_AIN1_PIN);
  Serial.print(" AIN2=D");
  Serial.print(MOTOR_AIN2_PIN);
  Serial.print(" STBY=D");
  Serial.println(MOTOR_STBY_PIN);

  Serial.print("motor_mode=");
  Serial.print(motorMode);
  Serial.print(" pwm=");
  Serial.println(motorPwm);

  Serial.print("encoder: A raw=");
  Serial.print(digitalRead(ENCODER_A_PIN));
  Serial.print(" B raw=");
  Serial.print(digitalRead(ENCODER_B_PIN));
  Serial.print(" count=");
  Serial.println(count);

  Serial.print("IR entry D9 raw=");
  Serial.print(digitalRead(IR_ENTRY_PIN));
  Serial.print(" state=");
  Serial.println(beamText(IR_ENTRY_PIN));

  Serial.print("IR exit  D10 raw=");
  Serial.print(digitalRead(IR_EXIT_PIN));
  Serial.print(" state=");
  Serial.println(beamText(IR_EXIT_PIN));

  Serial.println("Power check with multimeter:");
  Serial.println("  TB6612 VCC->GND should be about 5V");
  Serial.println("  TB6612 VM->GND should be about 12V");
  Serial.println("  Arduino GND, TB6612 GND, and 12V PSU - must be common");
  Serial.println("-------------------------");
}

void pulseMotor() {
  Serial.print("short forward pulse pwm=");
  Serial.print(motorPwm);
  Serial.print(" for ");
  Serial.print(PULSE_MS);
  Serial.println("ms");

  motorForward();
  delay(PULSE_MS);
  motorCoast();

  Serial.println("pulse done, motor stopped");
}

void handleSerial() {
  while (Serial.available() > 0) {
    char cmd = Serial.read();

    if (cmd == 'i') {
      printStatus();
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
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  pinMode(IR_ENTRY_PIN, INPUT_PULLUP);
  pinMode(IR_EXIT_PIN, INPUT_PULLUP);

  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_AIN1_PIN, OUTPUT);
  pinMode(MOTOR_AIN2_PIN, OUTPUT);
  pinMode(MOTOR_STBY_PIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), onEncoderAChange, CHANGE);

  digitalWrite(MOTOR_STBY_PIN, HIGH);
  motorCoast();

  Serial.println("Collector wiring diagnostic ready");
  Serial.println("Open Serial Monitor at 115200 baud. Send '?' for commands.");
  Serial.println("Start with 'i'. Then block each IR beam, rotate the shaft by hand, and try 'p'.");
  printStatus();
}

void loop() {
  static unsigned long lastPrintMs = 0;
  static long lastCount = 0;

  handleSerial();

  unsigned long now = millis();
  if (now - lastPrintMs >= 1000) {
    noInterrupts();
    long count = encoderCount;
    interrupts();

    long delta = count - lastCount;

    Serial.print("live: mode=");
    Serial.print(motorMode);
    Serial.print(" pwm=");
    Serial.print(motorPwm);
    Serial.print(" enc_delta_1s=");
    Serial.print(delta);
    Serial.print(" enc_count=");
    Serial.print(count);
    Serial.print(" entry=");
    Serial.print(beamText(IR_ENTRY_PIN));
    Serial.print(" exit=");
    Serial.println(beamText(IR_EXIT_PIN));

    lastCount = count;
    lastPrintMs = now;
  }
}
