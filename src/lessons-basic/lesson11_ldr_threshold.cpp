#include <Arduino.h>
#include "pins.h"

// Lesson 11: фоторезистор (LDR) включает LED, когда темно.
// Схема: модуль фоторезистора VCC -> 3.3V, GND -> GND, A0 -> GPIO35 (LDR).
// LED на GPIO23 (LED_RED).
//
// Калибровка: закрой датчик ладонью и посмотри число в мониторе,
// затем освети и посмотри снова; поставь threshold между этими значениями.
//
// Если на модуле нет пина A0, а только D0 — это цифровой вход (как в Уроке 9),
// порог крутится потенциометром на самом модуле.

int threshold = 2000;   // подбери под свою комнату по выводу в мониторе

void setup() {
  Serial.begin(115200);
  pinMode(LED_RED, OUTPUT);
}

void loop() {
  int light = analogRead(LDR);
  Serial.println(light);
  if (light < threshold) {          // темно
    digitalWrite(LED_RED, HIGH);
  } else {
    digitalWrite(LED_RED, LOW);
  }
  delay(100);
}
