#include <Arduino.h>
#include <DHT.h>
#include "pins.h"

// Lesson 12: DHT11 — температура и влажность.
// Схема: VCC -> 3.3V, GND -> GND, DATA -> GPIO16 (DHT_PIN).
// Требуется библиотека adafruit/DHT sensor library (см. lib_deps в platformio.ini).

#define DHTTYPE DHT11
DHT dht(DHT_PIN, DHTTYPE);

void setup() {
  Serial.begin(115200);
  dht.begin();
}

void loop() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (isnan(t) || isnan(h)) {
    Serial.println("Ошибка чтения DHT");
  } else {
    Serial.printf("T=%.1f C  H=%.1f %%\n", t, h);
  }
  delay(2000);   // DHT11 нельзя опрашивать чаще ~1 раза в секунду
}
