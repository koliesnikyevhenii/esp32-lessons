#include <Arduino.h>
#include "pins.h"

// Lesson 11: a photoresistor (LDR) turns the LED on when it is dark.
// Wiring: photoresistor module VCC -> 3.3V, GND -> GND, A0 -> GPIO35 (LDR).
// LED on GPIO23 (LED_RED).
//
// Calibration: cover the sensor with your palm and read the number in the monitor,
// then light it up and read again; set threshold between these values.
//
// If the module has no A0 pin, only D0 — that is a digital input (as in Lesson 9),
// and the threshold is set by the potentiometer on the module itself.

int threshold = 2000;   // tune it to your room using the monitor output

void setup() {
  Serial.begin(115200);
  pinMode(LED_RED, OUTPUT);
}

void loop() {
  int light = analogRead(LDR);
  Serial.println(light);
  if (light < threshold) {          // dark
    digitalWrite(LED_RED, HIGH);
  } else {
    digitalWrite(LED_RED, LOW);
  }
  delay(100);
}
