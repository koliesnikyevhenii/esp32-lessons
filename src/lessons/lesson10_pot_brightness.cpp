#include <Arduino.h>
#include "pins.h"

// Lesson 10: потенциометр управляет яркостью LED.
// Схема: крайние выводы потенциометра -> 3.3V и GND, средний -> GPIO34 (POT).
// Красный LED на GPIO23 (LED_RED), управляется через PWM (канал CH_R).

void setup() {
  Serial.begin(115200);
  ledcSetup(CH_R, 5000, 8);        // 5 кГц, 8 бит (0..255)
  ledcAttachPin(LED_RED, CH_R);
}

void loop() {
  int raw = analogRead(POT);        // 0..4095 (12-битный ADC)
  int duty = map(raw, 0, 4095, 0, 255);
  ledcWrite(CH_R, duty);
  Serial.println(raw);
  delay(50);
}
