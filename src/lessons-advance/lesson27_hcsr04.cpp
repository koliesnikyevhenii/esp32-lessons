// ============================================================================
//  Stage 7. Autopilot v1 — lesson 27: HC-SR04, ultrasonic rangefinder
// ----------------------------------------------------------------------------
//  Goal: learn to measure the distance to an obstacle "like a bat" — send a
//  burst of ultrasound and time how long the echo takes to come back. No Wi-Fi,
//  no MQTT, no motors: hardware and physics first, then the servo scanner
//  (lesson 28) and the autopilot (lesson 29).
//
//  The sensor does NOT measure distance. It measures TIME. We compute distance
//  ourselves by plugging in the speed of sound — and that depends on the air
//  temperature. That is where almost all of the HC-SR04 "oddities" come from,
//  and where half of this lesson comes from too.
//
//  How the module works:
//    1) we pull TRIG HIGH for exactly 10 µs;
//    2) the module itself emits 8 cycles at 40 kHz (a "burst") and raises ECHO;
//    3) as soon as the echo returns, it drops ECHO.
//    Duration of HIGH on ECHO = round-trip flight time.
//
//        distance[cm] = time[µs] * speed_of_sound[cm/µs] / 2
//
//    At 20 °C sound travels 343 m/s = 0.0343 cm/µs, so 1 cm "costs" 29 µs one
//    way and 58 µs round trip. Hence the famous "/58" formula.
//
//  What is new compared to lessons 8/9 (PIR and IR obstacle sensor):
//    • those sensors answer "yes/no", this one answers with a number in cm;
//    • a measurement takes TIME (up to 30 ms), and that caps the polling rate —
//      you cannot get around physics;
//    • a measurement has a "no answer" outcome (timeout). That is a THIRD state,
//      and mistaking it for "road is clear" is the autopilot's main error (see below).
//
//  Wiring (see include/pins.h and documentation/lesson27_hcsr04.md):
//    VCC  -> 5V (board VIN), GND -> GND,
//    TRIG -> GPIO12,
//    ECHO -> 1k/2k DIVIDER -> GPIO39.  ESP32 pins are not 5 V tolerant!
//
//  Build:  pio run -e lesson27_hcsr04 -t upload && pio device monitor
// ============================================================================

#include <Arduino.h>
#include "pins.h"

// ---- Physics ---------------------------------------------------------------
// Speed of sound in air: v = 331.4 + 0.606*T (m/s, T in °C). The difference
// between 0 °C and +30 °C is about 5 %: at 2 m that is 10 cm of error. For an
// autopilot with a "wall closer than 20 cm" threshold it doesn't matter; for a
// tape measure it is critical.
// Later T can be taken from the MPU6050 (it has a built-in thermometer) or a DHT11.
const float AIR_TEMP_C = 20.0f;

// ---- Measurement limits ----------------------------------------------------
// Below 2 cm the sensor is physically blind: a burst of 8 cycles at 40 kHz lasts
// 200 µs, in which time sound travels ~7 cm (i.e. ~3 cm to the reflector), plus
// the piezo transmitter keeps "ringing". While it rings, the receiver is deaf.
const float MIN_VALID_CM = 2.0f;
const float MAX_VALID_CM = 400.0f;   // datasheet ceiling; beyond it the echo is too weak

// Echo wait timeout. 400 cm = 400*58 = 23200 µs, we take it with margin.
// If there is no echo at all, pulseIn returns 0, and that is NOT "nothing ahead"
// but "don't know".
const uint32_t ECHO_TIMEOUT_US = 30000UL;

// Pause between "pings". The datasheet requires no more than once per 60 ms:
// otherwise the echo of the PREVIOUS ping, reflected off a far wall, lands in the
// next measurement and the sensor shows garbage (usually implausibly close).
const uint32_t PING_PERIOD_MS = 60;

// Autopilot threshold from Stage 7 — here only for an illustrative Serial label.
const float STOP_CM = 20.0f;

// Sliding window of the latest measurements: a median of 3 kills single outliers
// (a re-reflection off a corner, interference from the motors) yet adds almost no
// delay — unlike averaging, which "smears" a real approach to a wall.
const uint8_t WINDOW = 3;
float window[WINDOW];
uint8_t windowPos = 0;
bool windowFilled = false;

uint32_t lastPing = 0;
uint32_t lastPrint = 0;
const uint32_t PRINT_PERIOD_MS = 500;

// Counters — they show right away how well the sensor is mounted.
uint32_t pings = 0, timeouts = 0;

// ---------------------------------------------------------------------------
//  Speed of sound in cm/µs at the current temperature.
// ---------------------------------------------------------------------------
float soundCmPerUs() {
  return (331.4f + 0.606f * AIR_TEMP_C) / 10000.0f;   // m/s -> cm/µs
}

// ---------------------------------------------------------------------------
//  One measurement. Returns the distance in cm, or NAN if there is no answer.
//
//  pulseIn is BLOCKING: it honestly spins a loop until the pulse ends or the
//  timeout hits, i.e. in the worst case it eats all 30 ms. In this lesson there
//  are no motors and it doesn't matter; in the autopilot you can't do this any
//  more — there you need either a short timeout (20 cm threshold = 1160 µs, ~10 ms
//  covers 1.7 m) or measurement by interrupt/via RMT. Discussed in the lesson doc.
// ---------------------------------------------------------------------------
float measureCm() {
  // Guarantee a clean edge: LOW first, then exactly 10 µs HIGH.
  digitalWrite(SONAR_TRIG, LOW);
  delayMicroseconds(4);
  digitalWrite(SONAR_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(SONAR_TRIG, LOW);

  uint32_t us = pulseIn(SONAR_ECHO, HIGH, ECHO_TIMEOUT_US);
  pings++;

  if (us == 0) {           // timeout: the echo did not come back
    timeouts++;
    return NAN;
  }

  float cm = us * soundCmPerUs() / 2.0f;   // /2 — sound travelled THERE AND BACK

  // Readings outside the datasheet range can't be trusted: closer than 2 cm the
  // sensor is blind, beyond 4 m the echo drowns in noise. Return "don't know",
  // not a pretty number.
  if (cm < MIN_VALID_CM || cm > MAX_VALID_CM) return NAN;
  return cm;
}

// ---------------------------------------------------------------------------
//  Window median. NANs (timeouts) are left out; if there are fewer than two
//  valid values, we consider the distance unknown.
// ---------------------------------------------------------------------------
float medianCm() {
  float v[WINDOW];
  uint8_t n = 0;
  uint8_t count = windowFilled ? WINDOW : windowPos;
  for (uint8_t i = 0; i < count; i++) {
    if (!isnan(window[i])) v[n++] = window[i];
  }
  if (n < 2) return NAN;

  // Insertion sort — on three elements it is cheaper than any library.
  for (uint8_t i = 1; i < n; i++) {
    float key = v[i];
    int8_t j = i - 1;
    while (j >= 0 && v[j] > key) { v[j + 1] = v[j]; j--; }
    v[j + 1] = key;
  }
  return v[n / 2];
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(SONAR_TRIG, OUTPUT);
  digitalWrite(SONAR_TRIG, LOW);
  // GPIO39 is input-only and has NO internal pulls; INPUT is the only valid
  // mode here, the line level is held by the divider from ECHO.
  pinMode(SONAR_ECHO, INPUT);

  for (uint8_t i = 0; i < WINDOW; i++) window[i] = NAN;

  Serial.println("\n=== Урок 27. HC-SR04 ===");
  Serial.printf("Температура принята за %.1f C -> скорость звука %.1f м/с (%.0f мкс на см туда-обратно)\n",
                AIR_TEMP_C, 331.4f + 0.606f * AIR_TEMP_C, 2.0f / soundCmPerUs());
  Serial.printf("Рабочий диапазон %.0f..%.0f см, опрос раз в %lu мс.\n",
                MIN_VALID_CM, MAX_VALID_CM, (unsigned long)PING_PERIOD_MS);
  Serial.println("Поднеси ладонь и отводи её — смотри, где показания станут рваными.\n");
}

void loop() {
  uint32_t now = millis();

  // "Ping" no more often than the datasheet allows.
  if (now - lastPing >= PING_PERIOD_MS) {
    lastPing = now;
    window[windowPos] = measureCm();
    windowPos = (windowPos + 1) % WINDOW;
    if (windowPos == 0) windowFilled = true;
  }

  // Print less often than we measure: 16 lines per second are unreadable.
  if (now - lastPrint >= PRINT_PERIOD_MS) {
    lastPrint = now;

    uint8_t lastIdx = (windowPos + WINDOW - 1) % WINDOW;
    float raw = window[lastIdx];
    float med = medianCm();

    if (isnan(med)) {
      // The key point of the lesson: this is NOT "clear". The echo did not come
      // back, and there are three reasons: it's far (>4 m), the surface reflected
      // the sound sideways (a wall at an angle), or the material ate the sound
      // (a curtain, foam, a cat). Meanwhile the wall may be 30 cm right in front.
      Serial.printf("расстояние: НЕТ ЭХА  (тайм-аутов %lu из %lu)\n",
                    (unsigned long)timeouts, (unsigned long)pings);
    } else {
      char rawText[16];
      if (isnan(raw)) strcpy(rawText, "нет эха");
      else            snprintf(rawText, sizeof(rawText), "%.1f", raw);

      Serial.printf("расстояние: %6.1f см   (последнее измерение: %s)  %s\n",
                    med, rawText,
                    med < STOP_CM ? "<-- ближе порога, автопилот бы тормозил" : "");
    }
  }
}
