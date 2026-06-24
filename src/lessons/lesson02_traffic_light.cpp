#include <Arduino.h>
#include "pins.h"

// Lesson 2: неблокирующий светофор на millis() (без delay).

static unsigned long previousMillis = 0;
static const long interval = 3000;
static int phase = 0;

void setup() {
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
}

void loop() {
  unsigned long currentMillis = millis();

  if (phase == 0 && currentMillis - previousMillis >= interval) {
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, LOW);
    previousMillis = currentMillis;
    phase = 1;
  }

  if (phase == 1 && currentMillis - previousMillis >= interval) {
    digitalWrite(LED_YELLOW, HIGH);
    digitalWrite(LED_RED, LOW);
    previousMillis = currentMillis;
    phase = 2;
  }

  if (phase == 2 && currentMillis - previousMillis >= interval) {
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_YELLOW, LOW);
    previousMillis = currentMillis;
    phase = 0;
  }
}
