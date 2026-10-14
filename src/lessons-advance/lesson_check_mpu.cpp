// ============================================================================
//  CHECK: MPU6050 / MPU6500 — sensor check before lesson 20
// ----------------------------------------------------------------------------
//  Goal: make sure the sensor is connected and alive BEFORE mixing in
//  Wi-Fi/MQTT (lesson 20). No network and no library — only I2C and Serial.
//
//  IMPORTANT about the chip: an MPU6500 (WHO_AM_I = 0x70) or MPU9250 (0x71) is
//  often sold under the name "MPU6050". Adafruit_MPU6050 strictly checks
//  WHO_AM_I == 0x68 and rejects such chips. So here we read registers DIRECTLY — the
//  base map (accelerometer/gyroscope/temperature/PWR_MGMT) is the same across the
//  whole family, so this sketch works equally with the MPU6050 and the MPU6500/9250.
//
//  What it does:
//    • scans the I2C bus and prints the addresses found (expect 0x68, sometimes 0x69);
//    • prints WHO_AM_I so the real chip model is visible;
//    • wakes the sensor (PWR_MGMT_1 = 0) and reads 14 data bytes every 500 ms;
//    • prints accelerations (g), gyro (°/s), temperature and pitch/roll angles.
//
//  How to read the output:
//    • sensor lying flat on the table -> az ≈ +1.0 g, ax/ay ≈ 0, pitch/roll ≈ 0;
//    • tilt it sideways               -> roll changes;
//    • tilt it forward/back           -> pitch changes;
//    • gx/gy/gz ≈ 0 at rest, grow when rotating.
//
//  Build/flash:
//    pio run -e lesson_check_mpu -t upload
//    pio device monitor
//
//  Wiring (see include/pins.h): SDA -> GPIO32, SCL -> GPIO33, VCC -> 3.3V,
//  GND -> GND. AD0 -> GND gives address 0x68 (default), AD0 -> 3.3V -> 0x69.
// ============================================================================

#include <Arduino.h>
#include <Wire.h>
#include "pins.h"

// ---- Address and registers (same for MPU6050 / MPU6500 / MPU9250) ----------
const uint8_t MPU_ADDR    = 0x68;   // AD0 -> GND
const uint8_t REG_WHOAMI  = 0x75;
const uint8_t REG_PWRMGMT = 0x6B;   // bit 6 = SLEEP; write 0 -> wake the sensor
const uint8_t REG_ACCEL_X = 0x3B;   // from 0x3B come 14 bytes: accel(6)+temp(2)+gyro(6)

// Sensitivity at the default ranges (±2g, ±250°/s):
const float ACCEL_LSB = 16384.0f;   // LSB per 1 g
const float GYRO_LSB  = 131.0f;     // LSB per 1 °/s

// ---------------------------------------------------------------------------
//  Write one byte to a sensor register.
// ---------------------------------------------------------------------------
void writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// ---------------------------------------------------------------------------
//  Read one register (for WHO_AM_I).
// ---------------------------------------------------------------------------
uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);       // repeated start, don't release the bus
  Wire.requestFrom((int)MPU_ADDR, 1);
  return Wire.available() ? Wire.read() : 0xFF;
}

// ---------------------------------------------------------------------------
//  I2C bus scanner: if nothing at all is found here — the problem is in the
//  wires/power, not in the code.
// ---------------------------------------------------------------------------
void scanI2C() {
  Serial.println("[I2C] Сканирование шины...");
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("[I2C]   найдено устройство: 0x%02X\n", addr);
      found++;
    }
  }
  if (found == 0) {
    Serial.println("[I2C]   НИЧЕГО не найдено — проверь SDA/SCL, питание 3.3V и GND.");
  } else {
    Serial.printf("[I2C]   всего устройств: %u\n", found);
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== CHECK MPU (raw I2C) ===");

  Wire.begin(MPU_SDA, MPU_SCL);

  scanI2C();

  // WHO_AM_I: 0x68 = MPU6050, 0x70 = MPU6500, 0x71 = MPU9250.
  uint8_t who = readReg(REG_WHOAMI);
  Serial.printf("[MPU] WHO_AM_I (0x75) = 0x%02X ", who);
  switch (who) {
    case 0x68: Serial.println("-> MPU6050");  break;
    case 0x70: Serial.println("-> MPU6500");  break;
    case 0x71: Serial.println("-> MPU9250");  break;
    default:   Serial.println("-> неизвестный чип"); break;
  }

  // Wake the sensor: PWR_MGMT_1 = 0 (clear the SLEEP bit).
  writeReg(REG_PWRMGMT, 0x00);
  delay(100);

  Serial.println("[MPU] Готово. Печатаю показания раз в 500 мс.\n");
}

void loop() {
  // Read 14 bytes starting at 0x3B in one request:
  //   ax,ay,az (2 bytes each), temp (2), gx,gy,gz (2 bytes each).
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_X);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU_ADDR, 14);

  if (Wire.available() < 14) {
    Serial.println("[MPU] нет данных — датчик не отвечает.");
    delay(500);
    return;
  }

  // Each value is signed 16-bit (big-endian: high byte first).
  int16_t rawAx = (Wire.read() << 8) | Wire.read();
  int16_t rawAy = (Wire.read() << 8) | Wire.read();
  int16_t rawAz = (Wire.read() << 8) | Wire.read();
  int16_t rawT  = (Wire.read() << 8) | Wire.read();
  int16_t rawGx = (Wire.read() << 8) | Wire.read();
  int16_t rawGy = (Wire.read() << 8) | Wire.read();
  int16_t rawGz = (Wire.read() << 8) | Wire.read();

  // Convert to physical units.
  float ax = rawAx / ACCEL_LSB;   // g
  float ay = rawAy / ACCEL_LSB;
  float az = rawAz / ACCEL_LSB;
  float gx = rawGx / GYRO_LSB;    // °/s
  float gy = rawGy / GYRO_LSB;
  float gz = rawGz / GYRO_LSB;
  // Temperature: MPU6500 formula (slightly different on the MPU6050, doesn't matter for a check).
  float tempC = rawT / 340.0f + 36.53f;

  // Tilt angles from the gravity vector (in degrees).
  float roll  = atan2f(ay, az) * 180.0f / PI;
  float pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI;

  Serial.printf("acc[g]     x=%6.2f  y=%6.2f  z=%6.2f\n", ax, ay, az);
  Serial.printf("gyro[dps]  x=%6.1f  y=%6.1f  z=%6.1f\n", gx, gy, gz);
  Serial.printf("temp       %.1f C\n", tempC);
  Serial.printf("angle      pitch=%6.1f  roll=%6.1f\n", pitch, roll);
  Serial.println("----------------------------------------");

  delay(500);
}
