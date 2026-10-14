// ============================================================================
//  Stage 7. Autopilot v1 — lesson 28: SG90 + HC-SR04, scanning sonar
// ----------------------------------------------------------------------------
//  Goal: turn the single number from lesson 27 into a SECTOR — a little radar.
//  The servo turns the sensor step by step, at each step we measure the distance,
//  and at the end we have a map of "what is ahead and at which angle". That is
//  exactly the autopilot's input: not "clear/blocked", but "a wall 30 cm to the
//  left, a 2-metre corridor to the right".
//
//  Two new concepts:
//    1) A SERVO is not a "motor". Inside there is a motor + gearbox + potentiometer
//       + a tiny controller that turns the shaft by itself until the voltage from
//       the potentiometer matches the target. The feedback loop is closed INSIDE:
//       we say "go to 90°" and cannot ask whether it actually got there.
//    2) 50 Hz PWM where the PULSE WIDTH matters, not the duty cycle: 500 µs ≈ 0°,
//       1500 µs ≈ 90° (centre), 2400 µs ≈ 180°. The period is always 20 ms. The
//       standard was inherited from 1960s radio-controlled models, where the pulse
//       was literally a slot in the radio frame.
//
//  The Servo library is DELIBERATELY not used here: the whole servo is
//  ledcSetup(channel, 50, 16) + converting microseconds to duty. That way it is
//  clear that "servo control" and "PWM from lesson 3" are one and the same.
//
//  The order at each step is strictly: TURN -> WAIT -> MEASURE.
//  You can't measure while moving: the sensor is travelling, the cone is creeping,
//  and on top of that the servo motor creates interference and a supply sag. Hence
//  the main price of scanning — TIME (a full pass ≈ 2 seconds, see Serial output).
//
//  Wiring (see include/pins.h and documentation/lesson28_servo_sonar.md):
//    HC-SR04: VCC -> 5V, GND -> GND, TRIG -> GPIO12,
//             ECHO -> 1k/2k DIVIDER -> GPIO39;
//    SG90:    red -> 5V (SEPARATE rail, not 3V3!), brown -> GND,
//             orange (signal) -> GPIO19.
//
//  Build:  pio run -e lesson28_servo_sonar -t upload && pio device monitor
// ============================================================================

#include <Arduino.h>
#include "pins.h"

// ---- Servo -----------------------------------------------------------------
const uint32_t SERVO_FREQ_HZ = 50;      // 20 ms period — the hobby servo standard
const uint8_t  SERVO_BITS    = 16;      // LEDC resolution: 65535 steps per 20 ms ≈ 0.3 µs
const uint16_t SERVO_MIN_US  = 500;     // ~0°   (SG90 datasheet: 500..2400)
const uint16_t SERVO_MAX_US  = 2400;    // ~180°

// Real SG90s vary from batch to batch: for one, 0° is 480 µs, for another 600.
// Better not to go right up to the mechanical end — the gear will hit the stop,
// buzz and heat up. So we scan not 0..180 but with a margin.
const uint8_t SCAN_MIN_DEG  = 30;
const uint8_t SCAN_MAX_DEG  = 150;
const uint8_t SCAN_STEP_DEG = 15;                                     // 9 points
const uint8_t SCAN_POINTS   = (SCAN_MAX_DEG - SCAN_MIN_DEG) / SCAN_STEP_DEG + 1;

// How long to wait after a turn command. SG90 moves ~0.1 s per 60°, so a 15° step
// is ~25 ms of travel plus the shaft wobbling around the new point. 120 ms with margin.
const uint32_t SETTLE_MS = 120;

// ---- Sonar (same physics as lesson 27) -------------------------------------
const float    AIR_TEMP_C      = 20.0f;    // v = 331.4 + 0.606*T (m/s)
const float    MIN_VALID_CM    = 2.0f;
const float    MAX_VALID_CM    = 400.0f;
const uint32_t ECHO_TIMEOUT_US = 30000UL;
const uint32_t PING_GAP_MS     = 60;       // datasheet requirement between "pings"
const uint8_t  PINGS_PER_POINT = 3;        // median of 3 at each angle
const float    STOP_CM         = 20.0f;    // Stage 7 autopilot threshold

// Result of the last pass: NAN = "there was no echo", NOT "clear".
float scan[SCAN_POINTS];

bool sweepForward = true;   // "ping-pong": alternate the pass direction

float soundCmPerUs() {
  return (331.4f + 0.606f * AIR_TEMP_C) / 10000.0f;
}

// ---------------------------------------------------------------------------
//  Angle -> pulse width -> LEDC channel duty.
//  duty = width / period * (2^bits - 1), the period at 50 Hz = 20000 µs.
// ---------------------------------------------------------------------------
uint32_t usToDuty(uint32_t us) {
  return (uint32_t)(((uint64_t)us * ((1UL << SERVO_BITS) - 1)) / 20000UL);
}

void servoWriteDeg(float deg) {
  deg = constrain(deg, 0.0f, 180.0f);
  uint32_t us = SERVO_MIN_US + (uint32_t)((SERVO_MAX_US - SERVO_MIN_US) * deg / 180.0f);
  ledcWrite(CH_SERVO, usToDuty(us));
}

// ---------------------------------------------------------------------------
//  Remove the pulses from the channel. The servo loses holding torque and
//  "lets go" of the shaft: it stops buzzing, humming and drawing current. A good
//  habit when the position isn't needed for a while; on a battery robot it also
//  saves power.
// ---------------------------------------------------------------------------
void servoRelax() {
  ledcWrite(CH_SERVO, 0);
}

// ---------------------------------------------------------------------------
//  One sonar measurement. NAN = no answer (see lesson 27: this is a THIRD state,
//  not "far").
// ---------------------------------------------------------------------------
float pingCm() {
  digitalWrite(SONAR_TRIG, LOW);
  delayMicroseconds(4);
  digitalWrite(SONAR_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(SONAR_TRIG, LOW);

  uint32_t us = pulseIn(SONAR_ECHO, HIGH, ECHO_TIMEOUT_US);
  if (us == 0) return NAN;

  float cm = us * soundCmPerUs() / 2.0f;
  if (cm < MIN_VALID_CM || cm > MAX_VALID_CM) return NAN;
  return cm;
}

// ---------------------------------------------------------------------------
//  Median of several "pings" at one angle. Timeouts are dropped; if fewer than
//  two are valid, the point stays "don't know".
// ---------------------------------------------------------------------------
float measurePointCm() {
  float v[PINGS_PER_POINT];
  uint8_t n = 0;

  for (uint8_t i = 0; i < PINGS_PER_POINT; i++) {
    if (i) delay(PING_GAP_MS);       // no more often than once per 60 ms
    float cm = pingCm();
    if (!isnan(cm)) v[n++] = cm;
  }
  if (n < 2) return NAN;

  for (uint8_t i = 1; i < n; i++) {  // insertion sort, only a handful of elements
    float key = v[i];
    int8_t j = i - 1;
    while (j >= 0 && v[j] > key) { v[j + 1] = v[j]; j--; }
    v[j + 1] = key;
  }
  return v[n / 2];
}

uint8_t angleAt(uint8_t index) {
  return SCAN_MIN_DEG + index * SCAN_STEP_DEG;
}

// ---------------------------------------------------------------------------
//  A full pass over the sector. Blocking — and that is deliberate: in this lesson
//  nothing but scanning happens. In the autopilot you can't do this any more:
//  during a pass (~2 s) the robot handles neither MQTT nor the motor failsafe
//  (700 ms). There the scan is either unrolled into a state machine (one step per
//  loop()), or the robot first STOPS and only then looks around —
//  "stop and look", as every cheap robot vacuum does.
// ---------------------------------------------------------------------------
void sweep() {
  for (uint8_t k = 0; k < SCAN_POINTS; k++) {
    // Ping-pong: on the way back we go through the same angles in reverse order,
    // so the servo isn't driven across the whole sector for nothing.
    uint8_t i = sweepForward ? k : (SCAN_POINTS - 1 - k);

    servoWriteDeg(angleAt(i));
    delay(SETTLE_MS);                // TURN -> WAIT -> MEASURE
    scan[i] = measurePointCm();
  }
  sweepForward = !sweepForward;
}

// ---------------------------------------------------------------------------
//  Print the sector as a "radar": a bar with length proportional to distance.
// ---------------------------------------------------------------------------
void printScan(uint32_t sweepMs) {
  float best = NAN;   uint8_t bestDeg = 0;    // farthest — where to go
  float near_ = NAN;  uint8_t nearDeg = 0;    // nearest — what to fear
  uint8_t unknown = 0;

  Serial.println("---------------------------------------------");
  for (uint8_t i = 0; i < SCAN_POINTS; i++) {
    uint8_t deg = angleAt(i);
    float cm = scan[i];

    if (isnan(cm)) {
      unknown++;
      Serial.printf("%4u°  |  нет эха   |\n", deg);
      continue;
    }

    // Scale: 10 cm per character, at most 20 characters (200 cm and beyond).
    uint8_t bars = (uint8_t)constrain((int)(cm / 10.0f), 1, 20);
    char bar[21];
    memset(bar, '#', bars);
    bar[bars] = '\0';

    Serial.printf("%4u°  | %6.1f см  |%s%s\n", deg, cm, bar,
                  cm < STOP_CM ? "  <-- близко" : "");

    if (isnan(best)  || cm > best)  { best  = cm; bestDeg = deg; }
    if (isnan(near_) || cm < near_) { near_ = cm; nearDeg = deg; }
  }
  Serial.println("---------------------------------------------");

  if (isnan(best)) {
    // Not a single valid point in the whole pass. For the autopilot this is NOT
    // "all clear" but "I've gone blind": a wall at an angle and a detached sensor
    // both look like this.
    Serial.println("весь сектор без эха — двигаться вслепую НЕЛЬЗЯ");
  } else {
    Serial.printf("свободнее всего: %u° (%.0f см)   ближайшее: %u° (%.0f см)\n",
                  bestDeg, best, nearDeg, near_);
    if (near_ < STOP_CM)
      Serial.println("автопилот бы тормозил и поворачивал в сторону свободного угла");
  }
  if (unknown)
    Serial.printf("точек без ответа: %u из %u — это «не знаю», а не «свободно»\n",
                  unknown, SCAN_POINTS);
  Serial.printf("проход занял %lu мс\n\n", (unsigned long)sweepMs);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  // Sonar — as in lesson 27.
  pinMode(SONAR_TRIG, OUTPUT);
  digitalWrite(SONAR_TRIG, LOW);
  pinMode(SONAR_ECHO, INPUT);          // GPIO39 — input-only, no pulls

  // Servo: 50 Hz PWM, 16 bit. Channel 4 (timer 2 — ch/2 in core 2.x),
  // so as not to collide with RGB/buzzer on channels 0..2.
  ledcSetup(CH_SERVO, SERVO_FREQ_HZ, SERVO_BITS);
  ledcAttachPin(SERVO_PIN, CH_SERVO);

  for (uint8_t i = 0; i < SCAN_POINTS; i++) scan[i] = NAN;

  Serial.println("\n=== Урок 28. Сканирующий сонар (SG90 + HC-SR04) ===");
  Serial.printf("сектор %u..%u° шагом %u° = %u точек, по %u «пика» на точку\n",
                SCAN_MIN_DEG, SCAN_MAX_DEG, SCAN_STEP_DEG, SCAN_POINTS, PINGS_PER_POINT);
  Serial.printf("импульс %u..%u мкс при %lu Гц (центр ~1500 мкс = 90°)\n",
                SERVO_MIN_US, SERVO_MAX_US, (unsigned long)SERVO_FREQ_HZ);

  // Go to the centre and let the servo get there calmly: on power-up the shaft
  // jumps to the target position with a jerk, and that is the biggest current spike.
  servoWriteDeg(90);
  delay(600);
  Serial.println("центр 90° — проверь, что датчик смотрит строго вперёд.\n");
}

void loop() {
  uint32_t t0 = millis();
  sweep();
  uint32_t sweepMs = millis() - t0;

  printScan(sweepMs);

  // Release the shaft between passes: the buzzing stops, current drops almost to zero.
  // You can also hear the difference — that is what holding the position sounds like.
  servoRelax();
  delay(1000);
}
