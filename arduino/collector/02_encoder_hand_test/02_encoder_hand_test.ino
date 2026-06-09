/*
  Collector Encoder Hand Test

  Wiring:
    Encoder A blue   -> D2
    Encoder B green  -> D7
    Encoder VCC yellow -> 5V
    Encoder GND white  -> GND

  Rotate the shaft by hand. The count should change.
*/

const byte ENCODER_A_PIN = 2;
const byte ENCODER_B_PIN = 7;

volatile long encoderCount = 0;

void onEncoderAChange() {
  bool a = digitalRead(ENCODER_A_PIN);
  bool b = digitalRead(ENCODER_B_PIN);

  if (a == b) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), onEncoderAChange, CHANGE);

  Serial.println("Collector encoder hand test");
  Serial.println("Rotate the shaft by hand and watch count/delta.");
}

void loop() {
  static unsigned long lastPrintMs = 0;
  static long lastCount = 0;

  unsigned long now = millis();
  if (now - lastPrintMs >= 250) {
    noInterrupts();
    long count = encoderCount;
    interrupts();

    long delta = count - lastCount;

    Serial.print("count=");
    Serial.print(count);
    Serial.print(" delta_250ms=");
    Serial.println(delta);

    lastCount = count;
    lastPrintMs = now;
  }
}
