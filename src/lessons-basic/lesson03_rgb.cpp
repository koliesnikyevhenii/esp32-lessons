#include <Arduino.h>
#include "pins.h"

// Lesson 3: RGB LED, cycling through colours via PWM (LEDC).

static void setColor(int r, int g, int b) {
  ledcWrite(CH_R, r);
  ledcWrite(CH_G, g);
  ledcWrite(CH_B, b);
}

void setup() {
  // 5000 Hz, 8-bit resolution -> values 0..255
  ledcSetup(CH_R, 5000, 8);
  ledcSetup(CH_G, 5000, 8);
  ledcSetup(CH_B, 5000, 8);

  ledcAttachPin(PIN_R, CH_R);
  ledcAttachPin(PIN_G, CH_G);
  ledcAttachPin(PIN_B, CH_B);
}

void loop() {
  setColor(255, 0, 0);     delay(700); // red
  setColor(0, 255, 0);     delay(700); // green
  setColor(0, 0, 255);     delay(700); // blue
  setColor(255, 255, 0);   delay(700); // yellow
  setColor(0, 255, 255);   delay(700); // cyan
  setColor(255, 0, 255);   delay(700); // magenta
  setColor(255, 255, 255); delay(700); // white
  setColor(0, 0, 0);       delay(700); // off
}
