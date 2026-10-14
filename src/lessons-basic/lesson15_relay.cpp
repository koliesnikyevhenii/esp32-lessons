#include <Arduino.h>

// Two-channel relay (ESP32 kit)
#define RELAY_2CH_1 32
#define RELAY_2CH_2 33
// Single-channel relay (Arduino Uno kit)
#define RELAY_1CH   25

// Active LOW: helpers so the levels don't get mixed up
inline void relayOn(uint8_t pin)  { digitalWrite(pin, LOW);  } // LOW = on
inline void relayOff(uint8_t pin) { digitalWrite(pin, HIGH); } // HIGH = off

const uint8_t RELAYS[3] = { RELAY_2CH_1, RELAY_2CH_2, RELAY_1CH };
const char*   NAMES[3]  = { "Канал 1 (2ch)", "Канал 2 (2ch)", "Реле (1ch)" };

const unsigned long STEP_MS = 1500; // duration of one step
unsigned long lastStep = 0;
int stage = 0;

void allOff() {
  for (int i = 0; i < 3; i++) relayOff(RELAYS[i]);
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 3; i++) pinMode(RELAYS[i], OUTPUT);
  allOff();                     // everything off at startup
  Serial.println("Реле готовы");
}

void loop() {
  if (millis() - lastStep >= STEP_MS) {
    lastStep = millis();
    allOff();                   // turn off the previous channel
    relayOn(RELAYS[stage]);     // turn on the current one
    Serial.printf("ON: %s\n", NAMES[stage]);
    stage = (stage + 1) % 3;    // round robin: 0 → 1 → 2 → 0
  }
  // loop() is not blocked by delay() — button polling, Wi-Fi and sensors
  // can be added here. This is groundwork for the Stage 2 robot.
}
