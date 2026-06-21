/*
  Collector TB6612 Motor Test

  Wiring:
    D3 -> PWMA
    D4 -> AIN1
    D5 -> AIN2
    D6 -> STBY
    TB6612 VCC -> Arduino 5V
    TB6612 GND -> Arduino GND and 12V PSU GND
    TB6612 VM  -> +12V
    Motor red   -> AO1
    Motor black -> AO2

  This sketch runs low-PWM forward/reverse pulses.
  Stop immediately if the driver gets hot or the motor stalls.
*/

const byte MOTOR_PWM_PIN = 3;
const byte MOTOR_AIN1_PIN = 4;
const byte MOTOR_AIN2_PIN = 5;
const byte MOTOR_STBY_PIN = 6;

const byte TEST_PWM = 70;

void motorCoast() {
  analogWrite(MOTOR_PWM_PIN, 0);
  digitalWrite(MOTOR_AIN1_PIN, LOW);
  digitalWrite(MOTOR_AIN2_PIN, LOW);
}

void motorBrake() {
  analogWrite(MOTOR_PWM_PIN, 0);
  digitalWrite(MOTOR_AIN1_PIN, HIGH);
  digitalWrite(MOTOR_AIN2_PIN, HIGH);
}

void motorForward(byte pwm) {
  digitalWrite(MOTOR_STBY_PIN, HIGH);
  digitalWrite(MOTOR_AIN1_PIN, HIGH);
  digitalWrite(MOTOR_AIN2_PIN, LOW);
  analogWrite(MOTOR_PWM_PIN, pwm);
}

void motorReverse(byte pwm) {
  digitalWrite(MOTOR_STBY_PIN, HIGH);
  digitalWrite(MOTOR_AIN1_PIN, LOW);
  digitalWrite(MOTOR_AIN2_PIN, HIGH);
  analogWrite(MOTOR_PWM_PIN, pwm);
}

void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_AIN1_PIN, OUTPUT);
  pinMode(MOTOR_AIN2_PIN, OUTPUT);
  pinMode(MOTOR_STBY_PIN, OUTPUT);

  digitalWrite(MOTOR_STBY_PIN, LOW);
  motorCoast();

  Serial.println("Collector TB6612 motor test");
  Serial.println("Make sure TB6612 VM has 12V and all grounds are common.");
  delay(2000);

  digitalWrite(MOTOR_STBY_PIN, HIGH);
}

void loop() {
  Serial.print("forward pwm=");
  Serial.println(TEST_PWM);
  motorForward(TEST_PWM);
  delay(1500);

  Serial.println("coast");
  motorCoast();
  delay(1000);

  Serial.print("reverse pwm=");
  Serial.println(TEST_PWM);
  motorReverse(TEST_PWM);
  delay(1500);

  Serial.println("brake");
  motorBrake();
  delay(500);

  Serial.println("standby off");
  digitalWrite(MOTOR_STBY_PIN, LOW);
  delay(2000);

  digitalWrite(MOTOR_STBY_PIN, HIGH);
}
