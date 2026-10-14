#include <Arduino.h>
#include "pins.h"

// Lesson 6: reading a button with debounce, output to Serial.

static int lastReading = HIGH;
static unsigned long lastChange = 0;

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON, INPUT_PULLUP); // not pressed = HIGH, pressed = LOW
}

void loop() {
  int reading = digitalRead(BUTTON);
  if (reading != lastReading && millis() - lastChange > 50) { // 50 ms debounce
    lastChange = millis();
    if (reading == LOW) {
      Serial.println("Кнопка нажата");
    }
    lastReading = reading;
  }
}
