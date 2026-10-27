/*
 * 08_mpu6050_mega_bench.ino
 * --------------------------------------------------------------------------
 * Standalone test for the ordered GY-521 / MPU6050 on Arduino Mega 2560.
 *
 * Wiring through the common perfboard GYRO header:
 *   VCC -> logic 5V rail (this exact Grobotronics GY-521 is specified 3-5V)
 *   GND -> logic GND rail
 *   SDA -> Mega D20 / SDA
 *   SCL -> Mega D21 / SCL
 *
 * XDA, XCL, AD0 and INT remain disconnected for this test. The sketch checks
 * both normal I2C addresses, wakes the sensor and prints raw 6-axis samples.
 */

#include <Arduino.h>
#include <Wire.h>

const unsigned long BAUD = 115200;
const unsigned long SAMPLE_PERIOD_MS = 100;

uint8_t imuAddress = 0x68;
bool imuReady = false;
unsigned long lastSampleMs = 0;

bool readRegisters(uint8_t reg, uint8_t *data, uint8_t length) {
  Wire.beginTransmission(imuAddress);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(imuAddress, length) != length) return false;
  for (uint8_t i = 0; i < length; ++i) data[i] = Wire.read();
  return true;
}

bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(imuAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool probe(uint8_t address) {
  imuAddress = address;
  uint8_t whoAmI = 0;
  if (!readRegisters(0x75, &whoAmI, 1)) return false;
  return whoAmI == 0x68 || whoAmI == 0x69;
}

int16_t signedWord(const uint8_t *data) {
  return (int16_t)((uint16_t)data[0] << 8 | data[1]);
}

void setup() {
  Serial.begin(BAUD);
  Wire.begin();
  Wire.setClock(100000UL);  // conservative for the perfboard harness

  imuReady = probe(0x68) || probe(0x69);
  if (imuReady) {
    imuReady = writeRegister(0x6B, 0x00) &&  // wake
               writeRegister(0x1B, 0x00) &&  // gyro +/-250 deg/s
               writeRegister(0x1C, 0x00);    // accelerometer +/-2g
  }

  if (!imuReady) {
    Serial.println(F("ERROR,MPU6050_NOT_FOUND"));
    Serial.println(F("CHECK,5V,GND,SDA_D20,SCL_D21"));
    return;
  }

  Serial.print(F("READY,MPU6050,ADDRESS=0x"));
  Serial.println(imuAddress, HEX);
  Serial.println(F("HEADER,ms,ax_raw,ay_raw,az_raw,temp_raw,gx_raw,gy_raw,gz_raw"));
}

void loop() {
  if (!imuReady) return;
  const unsigned long now = millis();
  if (now - lastSampleMs < SAMPLE_PERIOD_MS) return;
  lastSampleMs = now;

  uint8_t data[14];
  if (!readRegisters(0x3B, data, sizeof(data))) {
    imuReady = false;
    Serial.println(F("ERROR,MPU6050_READ_FAILED"));
    return;
  }

  Serial.print(F("DATA,")); Serial.print(now); Serial.print(',');
  for (uint8_t i = 0; i < 7; ++i) {
    Serial.print(signedWord(&data[2 * i]));
    Serial.print(i == 6 ? '\n' : ',');
  }
}
