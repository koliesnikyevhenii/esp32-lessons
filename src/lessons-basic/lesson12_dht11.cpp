#include <Arduino.h>
#include <DHT.h>
#include "pins.h"

// Lesson 12: DHT11 — temperature and humidity.
// Wiring: VCC -> 3.3V, GND -> GND, DATA -> GPIO16 (DHT_PIN).
// Requires the adafruit/DHT sensor library (see lib_deps in platformio.ini).

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
  delay(2000);   // DHT11 must not be polled more often than ~once per second
}
