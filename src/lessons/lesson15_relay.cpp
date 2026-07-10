#include <Arduino.h>

// Двухканальное реле (набор ESP32)
#define RELAY_2CH_1 32
#define RELAY_2CH_2 33
// Одноканальное реле (набор Arduino Uno)
#define RELAY_1CH   25

// Активный LOW: помощники, чтобы не путаться с уровнями
inline void relayOn(uint8_t pin)  { digitalWrite(pin, LOW);  } // LOW = включено
inline void relayOff(uint8_t pin) { digitalWrite(pin, HIGH); } // HIGH = выключено

const uint8_t RELAYS[3] = { RELAY_2CH_1, RELAY_2CH_2, RELAY_1CH };
const char*   NAMES[3]  = { "Канал 1 (2ch)", "Канал 2 (2ch)", "Реле (1ch)" };

const unsigned long STEP_MS = 1500; // длительность одного шага
unsigned long lastStep = 0;
int stage = 0;

void allOff() {
  for (int i = 0; i < 3; i++) relayOff(RELAYS[i]);
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 3; i++) pinMode(RELAYS[i], OUTPUT);
  allOff();                     // при старте всё выключено
  Serial.println("Реле готовы");
}

void loop() {
  if (millis() - lastStep >= STEP_MS) {
    lastStep = millis();
    allOff();                   // гасим предыдущий канал
    relayOn(RELAYS[stage]);     // включаем текущий
    Serial.printf("ON: %s\n", NAMES[stage]);
    stage = (stage + 1) % 3;    // по кругу: 0 → 1 → 2 → 0
  }
  // loop() не заблокирован delay() — сюда можно добавить опрос кнопки,
  // Wi-Fi, датчиков. Это задел на робота из Этапа 2.
}
