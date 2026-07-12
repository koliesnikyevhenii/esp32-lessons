#include <Arduino.h>
#define PIR 27
#define LED_RED 32
#define BUZZER 25

void setup() {
  Serial.begin(115200);
  pinMode(PIR, INPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(BUZZER, OUTPUT);
}

void loop() {
  if (digitalRead(PIR) == HIGH) {
    Serial.println("Movement detected!");
    digitalWrite(LED_RED, HIGH);
    digitalWrite(BUZZER, HIGH);
     delay(300);
     digitalWrite(BUZZER, LOW);
  } else {
    digitalWrite(LED_RED, LOW);
    digitalWrite(BUZZER, LOW);
  }
    delay(4000);
}