#include <Arduino.h>
#include "pins.h"

// Lesson 4: простой бипер — включаем/выключаем пищалку через digitalWrite.

void setup() {
  pinMode(BUZZER, OUTPUT);
}

void loop() {
  digitalWrite(BUZZER, HIGH); delay(200);
  digitalWrite(BUZZER, LOW);  delay(800);
}
