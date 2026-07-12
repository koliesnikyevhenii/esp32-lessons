#include <Arduino.h>
#include "pins.h"

// Lesson 5: мелодия (гамма) через ledcWriteTone.

static int melody[] = {262, 294, 330, 349, 392, 440, 494, 523}; // до ре ми фа соль ля си до

void setup() {
  ledcSetup(CH_BUZZER, 2000, 8);
  ledcAttachPin(BUZZER, CH_BUZZER);
}

void loop() {
  for (int i = 0; i < 8; i++) {
    ledcWriteTone(CH_BUZZER, melody[i]);
    delay(300);
  }
  ledcWriteTone(CH_BUZZER, 0); // тишина
  delay(1500);
}
