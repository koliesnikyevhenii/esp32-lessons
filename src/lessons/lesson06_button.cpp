#include <Arduino.h>
#include "pins.h"

// Lesson 6: чтение кнопки с антидребезгом, вывод в Serial.

static int lastReading = HIGH;
static unsigned long lastChange = 0;

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON, INPUT_PULLUP); // не нажата = HIGH, нажата = LOW
}

void loop() {
  int reading = digitalRead(BUTTON);
  if (reading != lastReading && millis() - lastChange > 50) { // антидребезг 50 мс
    lastChange = millis();
    if (reading == LOW) {
      Serial.println("Кнопка нажата");
    }
    lastReading = reading;
  }
}
