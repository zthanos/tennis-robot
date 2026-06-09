/*
  Collector Smoke Test

  Tests motor + encoder + 2 IR beams together.

  Wiring:
    D2  -> Encoder A
    D3  -> TB6612 PWMA
    D4  -> TB6612 AIN1
    D5  -> TB6612 AIN2
    D6  -> TB6612 STBY
    D7  -> Encoder B
    D9  -> IR #1 entry receiver signal
    D10 -> IR #2 exit receiver signal

  GB37Y3530 motor colors:
    Red    -> TB6612 AO1
    Black  -> TB6612 AO2
    Blue   -> Encoder A / D2
    Green  -> Encoder B / D7
    Yellow -> Encoder VCC / 5V
    White  -> Encoder GND / GND

  Serial commands:
    f = forward low PWM
    r = reverse low PWM
    s = stop/coast
    b = brake
    + = increase PWM by 10
    - = decrease PWM by 10
*/

const byte ENCODER_A_PIN = 2;
const byte MOTOR_PWM_PIN = 3;
const byte MOTOR_AIN1_PIN = 4;
const byte MOTOR_AIN2_PIN = 5;
const byte MOTOR_STBY_PIN = 6;
const byte ENCODER_B_PIN = 7;
const byte IR_ENTRY_PIN = 9;
const byte IR_EXIT_PIN = 10;

volatile long encoderCount = 0;

byte motorPwm = 70;
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
  return digitalRead(pin) == HIGH;
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

void handleSerial() {
  while (Serial.available() > 0) {
    char cmd = Serial.read();

    if (cmd == 'f') {
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
      motorPwm = min(120, motorPwm + 10);
      applyCurrentMode();
      Serial.print("cmd: pwm=");
      Serial.println(motorPwm);
    } else if (cmd == '-') {
      motorPwm = max(0, motorPwm - 10);
      applyCurrentMode();
      Serial.print("cmd: pwm=");
      Serial.println(motorPwm);
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

  Serial.println("Collector smoke test");
  Serial.println("Commands: f=forward r=reverse s=stop b=brake +=pwm up -=pwm down");
  Serial.println("PWM is capped at 120 for TB6612 bench safety.");
}

void loop() {
  static unsigned long lastPrintMs = 0;
  static long lastCount = 0;

  handleSerial();

  unsigned long now = millis();
  if (now - lastPrintMs >= 500) {
    noInterrupts();
    long count = encoderCount;
    interrupts();

    long delta = count - lastCount;

    Serial.print("mode=");
    Serial.print(motorMode);
    Serial.print(" pwm=");
    Serial.print(motorPwm);
    Serial.print(" encoder_count=");
    Serial.print(count);
    Serial.print(" encoder_delta_500ms=");
    Serial.print(delta);
    Serial.print(" entry=");
    Serial.print(beamBroken(IR_ENTRY_PIN) ? "BROKEN" : "CLEAR");
    Serial.print(" exit=");
    Serial.println(beamBroken(IR_EXIT_PIN) ? "BROKEN" : "CLEAR");

    lastCount = count;
    lastPrintMs = now;
  }
}
