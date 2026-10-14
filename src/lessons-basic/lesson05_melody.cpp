#include <Arduino.h>
#include "pins.h"

// Lesson 5: a melody (a scale) via ledcWriteTone.

static int melody[] = {262, 294, 330, 349, 392, 440, 494, 523}; // do re mi fa sol la ti do

void setup() {
  ledcSetup(CH_BUZZER, 2000, 8);
  ledcAttachPin(BUZZER, CH_BUZZER);
}

void loop() {
  for (int i = 0; i < 8; i++) {
    ledcWriteTone(CH_BUZZER, melody[i]);
    delay(300);
  }
  ledcWriteTone(CH_BUZZER, 0); // silence
  delay(1500);
}
