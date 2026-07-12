#include <Arduino.h>
#include "pins.h"

// Lesson 7: кнопка переключает светодиод (toggle по фронту нажатия).

static int lastReading = HIGH;
static unsigned long lastChange = 0;
static bool ledOn = false;

void setup() {
  pinMode(LED_RED2, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
}

void loop() {
  int reading = digitalRead(BUTTON);
  if (reading == LOW && lastReading == HIGH && millis() - lastChange > 10) {
    ledOn = !ledOn;                              // переключаем состояние
    digitalWrite(LED_RED2, ledOn ? HIGH : LOW);
    lastChange = millis();
  }
  lastReading = reading;
}
