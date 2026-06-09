/*
  Collector IR Beam Test

  Wiring:
    IR #1 receiver white/yellow signal -> D9
    IR #2 receiver white/yellow signal -> D10
    Receiver red -> 5V
    Receiver black -> GND

  Adafruit break beam receivers are open-collector:
    LOW  = beam unbroken
    HIGH = beam broken
*/

const byte IR_ENTRY_PIN = 9;
const byte IR_EXIT_PIN = 10;

bool beamBroken(byte pin) {
  return digitalRead(pin) == HIGH;
}

void printBeamState(const char *name, byte pin) {
  Serial.print(name);
  Serial.print(": raw=");
  Serial.print(digitalRead(pin));
  Serial.print(" state=");
  Serial.print(beamBroken(pin) ? "BROKEN" : "CLEAR");
}

void setup() {
  Serial.begin(115200);

  pinMode(IR_ENTRY_PIN, INPUT_PULLUP);
  pinMode(IR_EXIT_PIN, INPUT_PULLUP);

  Serial.println("Collector IR beam test");
  Serial.println("Expected: CLEAR when aligned, BROKEN when blocked.");
}

void loop() {
  printBeamState("entry", IR_ENTRY_PIN);
  Serial.print(" | ");
  printBeamState("exit", IR_EXIT_PIN);
  Serial.println();

  delay(250);
}
