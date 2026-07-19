// ============================================================================
//  CHECK: MPU6050 / MPU6500 — проверка датчика перед уроком 20
// ----------------------------------------------------------------------------
//  Цель: убедиться, что датчик подключён и живой, ДО того как подмешивать
//  Wi-Fi/MQTT (урок 20). Никакой сети и никакой библиотеки — только I2C и Serial.
//
//  ВАЖНО про чип: под именем «MPU6050» часто продают MPU6500 (WHO_AM_I = 0x70)
//  или MPU9250 (0x71). Adafruit_MPU6050 строго проверяет WHO_AM_I == 0x68 и
//  такие чипы отвергает. Поэтому здесь читаем регистры НАПРЯМУЮ — базовая карта
//  (акселерометр/гироскоп/температура/PWR_MGMT) у всего семейства одинаковая,
//  так что этот скетч одинаково работает и с MPU6050, и с MPU6500/9250.
//
//  Что делает:
//    • сканирует шину I2C и печатает найденные адреса (ждём 0x68, иногда 0x69);
//    • печатает WHO_AM_I, чтобы было видно реальную модель чипа;
//    • будит датчик (PWR_MGMT_1 = 0) и раз в 500 мс читает 14 байт данных;
//    • печатает ускорения (g), гироскоп (°/с), температуру и углы pitch/roll.
//
//  Как читать вывод:
//    • датчик лежит ровно на столе -> az ≈ +1.0 g, ax/ay ≈ 0, pitch/roll ≈ 0;
//    • наклоняешь вбок           -> меняется roll;
//    • наклоняешь вперёд/назад   -> меняется pitch;
//    • gx/gy/gz ≈ 0 в покое, растут при вращении.
//
//  Сборка/прошивка:
//    pio run -e lesson_check_mpu -t upload
//    pio device monitor
//
//  Подключение (см. include/pins.h): SDA -> GPIO32, SCL -> GPIO33, VCC -> 3.3V,
//  GND -> GND. AD0 -> GND даёт адрес 0x68 (по умолчанию), AD0 -> 3.3V -> 0x69.
// ============================================================================

#include <Arduino.h>
#include <Wire.h>
#include "pins.h"

// ---- Адрес и регистры (одинаковы для MPU6050 / MPU6500 / MPU9250) ----------
const uint8_t MPU_ADDR    = 0x68;   // AD0 -> GND
const uint8_t REG_WHOAMI  = 0x75;
const uint8_t REG_PWRMGMT = 0x6B;   // бит 6 = SLEEP; пишем 0 -> будим датчик
const uint8_t REG_ACCEL_X = 0x3B;   // с 0x3B идут 14 байт: accel(6)+temp(2)+gyro(6)

// Чувствительность при диапазонах по умолчанию (±2g, ±250°/с):
const float ACCEL_LSB = 16384.0f;   // LSB на 1 g
const float GYRO_LSB  = 131.0f;     // LSB на 1 °/с

// ---------------------------------------------------------------------------
//  Записать один байт в регистр датчика.
// ---------------------------------------------------------------------------
void writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// ---------------------------------------------------------------------------
//  Прочитать один регистр (для WHO_AM_I).
// ---------------------------------------------------------------------------
uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);       // repeated start, шину не отпускаем
  Wire.requestFrom((int)MPU_ADDR, 1);
  return Wire.available() ? Wire.read() : 0xFF;
}

// ---------------------------------------------------------------------------
//  Сканер шины I2C: если тут вообще ничего не найдено — проблема в проводах/
//  питании, а не в коде.
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

  // Будим датчик: PWR_MGMT_1 = 0 (снимаем бит SLEEP).
  writeReg(REG_PWRMGMT, 0x00);
  delay(100);

  Serial.println("[MPU] Готово. Печатаю показания раз в 500 мс.\n");
}

void loop() {
  // Одним запросом читаем 14 байт начиная с 0x3B:
  //   ax,ay,az (по 2 байта), temp (2), gx,gy,gz (по 2 байта).
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_X);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU_ADDR, 14);

  if (Wire.available() < 14) {
    Serial.println("[MPU] нет данных — датчик не отвечает.");
    delay(500);
    return;
  }

  // Каждое значение — знаковое 16-битное (big-endian: сначала старший байт).
  int16_t rawAx = (Wire.read() << 8) | Wire.read();
  int16_t rawAy = (Wire.read() << 8) | Wire.read();
  int16_t rawAz = (Wire.read() << 8) | Wire.read();
  int16_t rawT  = (Wire.read() << 8) | Wire.read();
  int16_t rawGx = (Wire.read() << 8) | Wire.read();
  int16_t rawGy = (Wire.read() << 8) | Wire.read();
  int16_t rawGz = (Wire.read() << 8) | Wire.read();

  // Переводим в физические единицы.
  float ax = rawAx / ACCEL_LSB;   // g
  float ay = rawAy / ACCEL_LSB;
  float az = rawAz / ACCEL_LSB;
  float gx = rawGx / GYRO_LSB;    // °/с
  float gy = rawGy / GYRO_LSB;
  float gz = rawGz / GYRO_LSB;
  // Температура: формула MPU6500 (у MPU6050 чуть иначе, для проверки не важно).
  float tempC = rawT / 340.0f + 36.53f;

  // Углы наклона из вектора силы тяжести (в градусах).
  float roll  = atan2f(ay, az) * 180.0f / PI;
  float pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI;

  Serial.printf("acc[g]     x=%6.2f  y=%6.2f  z=%6.2f\n", ax, ay, az);
  Serial.printf("gyro[dps]  x=%6.1f  y=%6.1f  z=%6.1f\n", gx, gy, gz);
  Serial.printf("temp       %.1f C\n", tempC);
  Serial.printf("angle      pitch=%6.1f  roll=%6.1f\n", pitch, roll);
  Serial.println("----------------------------------------");

  delay(500);
}
