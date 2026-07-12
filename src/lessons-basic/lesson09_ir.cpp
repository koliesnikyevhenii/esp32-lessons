#include <Arduino.h>
#define IR 33
#define LED_GREEN 13

void setup() {
  Serial.begin(115200);
  pinMode(IR, INPUT);
  pinMode(LED_GREEN, OUTPUT);
}

void loop() {
  // у большинства модулей: LOW = препятствие обнаружено
  if (digitalRead(IR) == LOW) {
    digitalWrite(LED_GREEN, HIGH);
    Serial.println("Препятствие");
  } else {
    digitalWrite(LED_GREEN, LOW);
     Serial.println("Нет препятствия");
  }
  delay(100);
}