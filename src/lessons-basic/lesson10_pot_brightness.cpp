#include <Arduino.h>
#include "pins.h"

// Lesson 10: a potentiometer controls LED brightness.
// Wiring: outer potentiometer pins -> 3.3V and GND, middle -> GPIO34 (POT).
// Red LED on GPIO23 (LED_RED), driven via PWM (channel CH_R).

void setup() {
  Serial.begin(115200);
  ledcSetup(CH_R, 5000, 8);        // 5 kHz, 8 bit (0..255)
  ledcAttachPin(LED_RED, CH_R);
}

void loop() {
  int raw = analogRead(POT);        // 0..4095 (12-bit ADC)
  int duty = map(raw, 0, 4095, 0, 255);
  ledcWrite(CH_R, duty);
  Serial.println(raw);
  delay(50);
}
