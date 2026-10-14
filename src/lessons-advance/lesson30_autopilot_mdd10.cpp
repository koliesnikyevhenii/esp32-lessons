// ============================================================================
//  Stage 7. Autopilot v1 on the Cytron MDD10A — lesson 30
// ----------------------------------------------------------------------------
//  This is lesson 29 with NO CHANGES to the logic, but with a different motor driver:
//  instead of the TB6612FNG there is a Cytron MDD10A (2 channels, up to 10 A per
//  channel, 5..30 V). Exactly one layer changed — motor()/driveStop()/driveTank() and
//  setup(). Autopilot, sonar, servo, tilt guard, MQTT topics are byte-for-byte the
//  same, so the backend, the dashboard and the FPV camera (lesson 26) notice nothing.
//
//  HOW THE MDD10A DIFFERS FROM THE TB6612 (for the code):
//    • TWO wires per channel, not three: DIR (direction) and PWM (speed) —
//      sign-magnitude mode. There are no IN1/IN2 pins any more;
//    • PWM = 0 -> motor outputs are shorted to GND, this is a BRAKE (on the TB6612 with
//      IN1=IN2=LOW the motor coasted). The car stops harder —
//      which only helps the autopilot (STOP_CM);
//    • PWM up to 20 kHz. analogWrite in core 2.x gives 1 kHz — the motor whines on it.
//      Hence explicit LEDC at 20 kHz here (inaudible) — see the MOTORS section.
//
//  WIRING (pins — the same wires that went to the TB6612, see include/pins.h):
//    MDD_DIR1 13 / MDD_PWM1 25 — channel 1 = LEFT motor (formerly AIN1 / PWMA)
//    MDD_DIR2 26 / MDD_PWM2 33 — channel 2 = RIGHT motor (formerly BIN1 / PWMB)
//    Driver GND -> ESP32 GND (MANDATORY), power input -> battery directly.
//    The wires from GPIO14 and GPIO27 (AIN2/BIN2) are simply removed — no longer needed.
//    MDD10A logic understands 3.3 V, no level shifter needed.
//  Details: src/lessons-advance/documentation/lesson30_autopilot_mdd10.md
//
//  What follows is the description of lesson 29, it fully applies.
// ----------------------------------------------------------------------------
//  Goal: combine lesson 22 (telemetry + MQTT control) and lesson 28
//  (scanning sonar) into one firmware where the robot can drive without a human:
//
//      if the way ahead is clear      -> drive forward
//      if a wall is closer than limit -> stop, look around, turn
//      if there is no echo at all     -> treat it as DANGER, not as a free road
//
//  What is new compared to 22 and 28 — three things, all three fundamental:
//
//  1) SCANNING NO LONGER BLOCKS loop(). In lesson 28 sweep() honestly sat in delay()
//     for ~2 seconds — nothing else was happening there anyway. Here, during those
//     2 seconds, the MQTT keep-alive would drop, the motor failsafe (700 ms) would
//     fire, and the tilt guard would "sleep through". So the sweep is unrolled into a
//     STATE MACHINE: one pass of loop() does exactly one small step
//     (turn the horn / fire one "ping"), and between steps the robot keeps living.
//
//  2) TWO MODES AND WHO IS IN CHARGE. There is a second command topic —
//     commands/esp32/mode with body auto|manual. The autopilot is switched on FROM THE
//     BROWSER and switched off in five ways: the manual command, ANY drive command
//     (the human grabbed the wheel — the autopilot silently steps aside), loss of the
//     broker, three failed looks in a row and, of course, the button in the dashboard.
//     The commands/esp32/drive contract is NOT touched: the FPV page from lesson 26
//     knows nothing about the autopilot and keeps working as it did.
//
//  3) THE TURN IS CLOSED ON THE GYRO. We have no encoders, "spin for 600 ms" is a
//     lottery (battery charge, carpet, parquet). But since lesson 20 we have yaw, and
//     the turn goes on until |yaw - yaw at start| accumulates the required degrees.
//     The sign of yaw does not matter (see serviceTurn) — the direction is set by the
//     motors, and the gyro only counts how much has been turned so far. A timeout in
//     case the wheels slip and yaw does not grow.
//
//  Telemetry adds three metrics to lesson 22 (same contract — a bare number):
//     sensors/esp32/distance — distance ahead, cm; -1 = "no echo" (NOT "far"!)
//     sensors/esp32/mode     — 0 = manual, 1 = autopilot (publish on change)
//     sensors/esp32/bearing  — the angle the autopilot chose on its last look,
//                              in servo coordinates: 90 = straight ahead (on change)
//
//  Pins: MPU (32/17), MDD10A motors (13/25 and 26/33), sonar (12/39), servo (19).
//  Four peripherals AT THE SAME TIME — this is exactly why the groups in pins.h don't overlap.
//
//  Libraries: knolleary/PubSubClient + tockn/MPU6050_tockn
//  Build: pio run -e lesson30_autopilot_mdd10 -t upload && pio device monitor
// ============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <MPU6050_tockn.h>
#include "pins.h"

// ---- Wi-Fi ----------------------------------------------------------------
const char* SSID = "Vektor_04";
const char* PASS = "uteam2020";

// ---- MQTT broker (RabbitMQ + mqtt plugin) ---------------------------------
const char* MQTT_BROKER = "192.168.0.7";   // <-- your PC's LAN IP, NOT localhost
const uint16_t MQTT_PORT = 1883;
const char* MQTT_USER = "esp";
const char* MQTT_PASS = "esp-pass";

const char* DEVICE_ID = "esp32";
const char* CLIENT_ID = "esp32-lesson30";

// ---- Topics ----------------------------------------------------------------
const char* TOPIC_PITCH   = "sensors/esp32/pitch";
const char* TOPIC_ROLL    = "sensors/esp32/roll";
const char* TOPIC_YAW     = "sensors/esp32/yaw";
const char* TOPIC_GUARD   = "sensors/esp32/guard";     // 0/1 — tilt guard (lesson 22)
const char* TOPIC_DIST    = "sensors/esp32/distance";  // cm ahead, -1 = no echo
const char* TOPIC_MODE    = "sensors/esp32/mode";      // 0 manual / 1 autopilot
const char* TOPIC_BEARING = "sensors/esp32/bearing";   // chosen angle, 90 = straight ahead
const char* TOPIC_DRIVE   = "commands/esp32/drive";    // forward|back|left|right|stop
const char* TOPIC_CMDMODE = "commands/esp32/mode";     // auto|manual  <-- NEW

// ---- Sensor ----------------------------------------------------------------
MPU6050 mpu(Wire);

// ---- Motors ----------------------------------------------------------------
// PWM for the MDD10A: 20 kHz (datasheet maximum, above hearing), 8 bit — so that
// speeds stay in the familiar 0..255, as with analogWrite in lessons 21/22/29.
const uint32_t MOTOR_PWM_HZ   = 20000;
const uint8_t  MOTOR_PWM_BITS = 8;

// Motor spinning the wrong way? With the TB6612 we swapped the motor wires —
// you can do that here too (M1A <-> M1B), or simply set true.
const bool MOTOR_LEFT_INVERT  = false;
const bool MOTOR_RIGHT_INVERT = true;

// The MDD10A delivers up to 10 A, and an abrupt "forward -> back" reversal at full speed
// produces a current spike that small motors and the battery feel. Before a direction
// change the motor gets 0 (brake) for these milliseconds, and only then does DIR
// change. The wait does NOT block loop() — see motorService().
const uint32_t REVERSE_PAUSE_MS = 30;

const int SPEED_MANUAL = 200;                // as in lesson 22
// The autopilot drives SLOWER than a human, and this is not politeness but arithmetic
// from lesson 27: the sonar gives ~14 readings per second, so between two "pings" the
// robot drives blind. At 200 that is a noticeable chunk of braking distance, at 160 — half that.
const int SPEED_AUTO   = 160;
const unsigned long FAILSAFE_MS = 700;       // manual mode: no commands for longer -> stop

// ---- Tilt guard (entirely from lesson 22) ----------------------------------
const float TILT_LIMIT   = 45.0f;
const float TILT_RELEASE = 35.0f;
const unsigned long TILT_DEBOUNCE_MS = 300;
const float ACCEL_TRUST_BAND = 0.25f;

// ---- Servo (from lesson 28) ------------------------------------------------
const uint32_t SERVO_FREQ_HZ = 50;
const uint8_t  SERVO_BITS    = 16;
const uint16_t SERVO_MIN_US  = 500;
const uint16_t SERVO_MAX_US  = 2400;

const uint8_t SCAN_MIN_DEG  = 30;
const uint8_t SCAN_MAX_DEG  = 150;
const uint8_t SCAN_STEP_DEG = 15;
const uint8_t SCAN_POINTS   = (SCAN_MAX_DEG - SCAN_MIN_DEG) / SCAN_STEP_DEG + 1;  // 9
const uint8_t SERVO_FORWARD_DEG = 90;        // "straight ahead"

// Where the horn points at angles GREATER than 90°. Depends on which way round the
// bracket is mounted: if on a look the robot confidently turns away from the free
// space — change true to false here, and nothing else.
const bool SERVO_HIGH_IS_LEFT = true;

const uint32_t SETTLE_MS       = 120;        // 15° step: ~25 ms of travel + settling
const uint32_t SETTLE_FAR_MS   = 450;        // move across the whole sector (90° -> 30°)

// ---- Sonar (from lesson 27) ------------------------------------------------
const float AIR_TEMP_C   = 20.0f;            // v = 331.4 + 0.606*T
const float MIN_VALID_CM = 2.0f;
const float MAX_VALID_CM = 400.0f;

// In lesson 27 the timeout was 30000 µs (≈5 m) — and pulseIn honestly ate 30 ms.
// Here that is unacceptable: in 30 ms we service neither MQTT nor the failsafe. 15000 µs
// is ~2.5 m — beyond that the autopilot doesn't care, "free is free".
const uint32_t ECHO_TIMEOUT_US = 15000UL;
const uint32_t PING_GAP_MS     = 70;         // 60 per datasheet + margin
const uint8_t  PINGS_PER_POINT = 3;          // median of 3 (a single outlier won't move it)

// ---- Autopilot thresholds --------------------------------------------------
// We brake NOT at the 20 cm from the stage wording, but earlier: while driving the sonar
// is silent for ~70 ms, plus the car itself coasts. 30 cm is the margin for both effects.
const float STOP_CM  = 30.0f;
// And a direction counts as free only from 45 cm: 31 cm of "freedom" is not a passage,
// it is the next stop half a second later.
const float CLEAR_CM = 45.0f;

// How many readings in a row without an echo count as an obstacle. One "no echo" in a
// room is normal (the wall is further than 2.5 m). Four in a row at speed is already 0.3
// seconds of silence, and per lesson 27 that is "don't know", i.e. a reason to stop and look.
const uint8_t UNKNOWN_LIMIT = 4;

const int16_t  TURN_MIN_DEG   = 25;          // turning less than this is pointless
const int16_t  TURN_DEADZONE  = 10;          // chosen angle almost straight -> just drive
const uint32_t BACK_MS        = 600;         // how long to back up when everything is blocked
const uint8_t  BLIND_LIMIT    = 3;           // this many fruitless looks -> give up

// ---- Timings ---------------------------------------------------------------
const unsigned long PUBLISH_INTERVAL = 200;  // 5 times per second, as in lesson 22
const unsigned long DIST_STALE_MS    = 1500; // older -> publish -1 ("don't know")

// ---- State: network, sensor, guard (lesson 22) -----------------------------
unsigned long lastCommandMs = 0;
unsigned long lastPublish   = 0;
bool moving      = false;
bool tiltBlocked = false;
unsigned long overLimitSince = 0;
int  guardSent = -1;
float pitch = 0, roll = 0, yaw = 0;

WiFiClient   net;
PubSubClient mqtt(net);

// ---- State: servo ----------------------------------------------------------
uint8_t  servoAimDeg  = SERVO_FORWARD_DEG;   // where the horn was SENT
uint32_t servoReadyAt = 0;                   // when it can be trusted (arrived and settled)

// ---- State: "ahead" sonar --------------------------------------------------
float    fwdRing[PINGS_PER_POINT];           // latest readings at 90°
uint8_t  fwdCount   = 0;
uint32_t nextPingAt = 0;
uint8_t  unknownRun = 0;                     // how many "no echo" in a row
float    lastFwdCm  = NAN;                   // last TRUSTED distance ahead
uint32_t lastFwdAt  = 0;

// ---- State: autopilot ------------------------------------------------------
enum AutoState : uint8_t {
  AUTO_OFF,     // a human is steering
  AUTO_DRIVE,   // driving forward and "pinging" ahead
  AUTO_SCAN,    // standing and looking over the sector (stop and look)
  AUTO_TURN,    // turning by the gyro
  AUTO_BACK     // backing up: everything ahead is blocked
};
AutoState autoState = AUTO_OFF;
int modeSent = -1;                           // what the backend already knows about the mode
int bearingSent = -1000;                     // -1000 = nothing sent yet
bool autoPaused = false;                     // paused by the tilt guard
uint8_t blindCount = 0;                      // looks in a row with no free direction

// Sector sweep as a state machine: one step per loop().
enum ScanPhase : uint8_t { SCAN_SETTLE, SCAN_PING };
float    scanBuf[SCAN_POINTS];
float    pingBuf[PINGS_PER_POINT];
uint8_t  scanIndex = 0, scanPing = 0, pingCount = 0;
ScanPhase scanPhase = SCAN_SETTLE;
uint32_t scanNextAt = 0, scanStartedAt = 0;

// Turn by the gyro.
float    turnStartYaw = 0;
int16_t  turnGoalDeg  = 0;
uint32_t turnDeadline = 0;

// ---------------------------------------------------------------------------
//  MOTORS — Cytron MDD10A. The ONLY section rewritten relative to lesson 29.
//
//  Sign-magnitude mode: DIR = direction (HIGH/LOW), PWM = speed (duty cycle).
//  PWM = 0 — brake (both channel outputs to GND), not free coasting.
//
//  ABOUT LEDC. There is NO analogWrite here: it gives 1 kHz, which is a loud whine in
//  the motors. Explicit ledcSetup at 20 kHz, channels CH_MOTOR_L/CH_MOTOR_R = 8/9.
//  Channels 8..15 in core 2.x are a separate LEDC group (low-speed) with their own
//  timers, so they don't share a timer with the servo (CH_SERVO 4 -> timer 2 of the
//  high-speed group, 50 Hz) and don't knock each other's frequencies off. And sharing a
//  timer between 20 kHz / 50 Hz is not allowed: whoever called ledcSetup last set the
//  frequency for both.
//
//  The external interface — driveStop()/driveTank(left, right, speed) and the moving
//  flag — is the same as in lesson 29, so the whole autopilot below is untouched.
// ---------------------------------------------------------------------------
struct Motor {
  uint8_t  dirPin;
  uint8_t  channel;
  bool     invert;
  int8_t   wantDir;     // what was requested: -1 / 0 / +1
  uint8_t  wantSpeed;
  int8_t   runDir;      // which way the motor is spinning RIGHT NOW (0 = brake)
  int8_t   lastDir;     // the last direction it actually spun in
  uint32_t brakedAt;    // when it was last stopped
};

Motor motorL = { MDD_DIR1, CH_MOTOR_L, MOTOR_LEFT_INVERT,  0, 0, 0, 0, 0 };
Motor motorR = { MDD_DIR2, CH_MOTOR_R, MOTOR_RIGHT_INVERT, 0, 0, 0, 0, 0 };

// Bring the motor to what was requested. Called both from driveTank() and on every
// loop() from motorService() — this is how the pre-reverse pause works without delay().
void motorUpdate(Motor& m) {
  if (m.wantDir == 0) {                       // stop = brake
    if (m.runDir != 0) { m.runDir = 0; m.brakedAt = millis(); }
    ledcWrite(m.channel, 0);
    return;
  }
  if (m.runDir == m.wantDir) {                // same direction — change only the speed
    ledcWrite(m.channel, m.wantSpeed);
    return;
  }
  if (m.runDir != 0) {                        // reversing on the move: brake first
    m.runDir   = 0;
    m.brakedAt = millis();
    ledcWrite(m.channel, 0);
    return;                                   // DIR is changed once the pause is over
  }
  // Standing. If we last spun the OTHER way and recently — wait.
  if (m.lastDir != 0 && m.lastDir != m.wantDir &&
      millis() - m.brakedAt < REVERSE_PAUSE_MS) return;

  // DIR is changed only at PWM = 0 — then the motor won't jerk at full speed.
  bool high = (m.wantDir > 0) != m.invert;
  digitalWrite(m.dirPin, high ? HIGH : LOW);
  ledcWrite(m.channel, m.wantSpeed);
  m.runDir  = m.wantDir;
  m.lastDir = m.wantDir;
}

void motorSet(Motor& m, int dir, int speed) {
  m.wantDir   = (dir > 0) - (dir < 0);
  m.wantSpeed = (uint8_t)constrain(speed, 0, 255);
  motorUpdate(m);
}

// Every loop(): carries a deferred reversal through to the end.
void motorService() {
  motorUpdate(motorL);
  motorUpdate(motorR);
}

void driveStop() {
  motorSet(motorL, 0, 0);
  motorSet(motorR, 0, 0);
  moving = false;
}

void driveTank(int left, int right, int speed) {
  motorSet(motorL, left,  speed);
  motorSet(motorR, right, speed);
  moving = (left != 0 || right != 0);
}

// ---------------------------------------------------------------------------
//  SERVO (lesson 28). servoRelax() is DELIBERATELY not used here:
//  the sonar must always face straight ahead, and on a moving car a released
//  horn gets dragged off by vibration. We pay for it with holding current (~10 mA) and buzzing.
// ---------------------------------------------------------------------------
uint32_t usToDuty(uint32_t us) {
  return (uint32_t)(((uint64_t)us * ((1UL << SERVO_BITS) - 1)) / 20000UL);
}

void servoWriteDeg(float deg) {
  deg = constrain(deg, 0.0f, 180.0f);
  uint32_t us = SERVO_MIN_US + (uint32_t)((SERVO_MAX_US - SERVO_MIN_US) * deg / 180.0f);
  ledcWrite(CH_SERVO, usToDuty(us));
}

// Aim the horn and remember when it can be trusted again: TURN -> WAIT -> MEASURE.
void servoAim(uint8_t deg, uint32_t settleMs) {
  servoWriteDeg(deg);
  servoAimDeg  = deg;
  servoReadyAt = millis() + settleMs;
}

bool servoSettled() {
  return (int32_t)(millis() - servoReadyAt) >= 0;
}

// ---------------------------------------------------------------------------
//  SONAR (lesson 27). NAN = no response. This is a THIRD state, not "far":
//  an angled wall, a curtain and a sensor that lost power all look the same.
// ---------------------------------------------------------------------------
float pingCm() {
  digitalWrite(SONAR_TRIG, LOW);
  delayMicroseconds(4);
  digitalWrite(SONAR_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(SONAR_TRIG, LOW);

  uint32_t us = pulseIn(SONAR_ECHO, HIGH, ECHO_TIMEOUT_US);
  if (us == 0) return NAN;

  float cm = us * ((331.4f + 0.606f * AIR_TEMP_C) / 10000.0f) / 2.0f;
  if (cm < MIN_VALID_CM || cm > MAX_VALID_CM) return NAN;
  return cm;
}

// Median; NAN if fewer than two are valid — "one reading out of three" isn't worth trusting.
float medianOf(float* v, uint8_t n) {
  if (n < 2) return NAN;
  for (uint8_t i = 1; i < n; i++) {          // insertion sort, only a handful of elements
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
//  TELEMETRY (lesson 19 contract: body = bare number)
// ---------------------------------------------------------------------------
void publishNumber(const char* topic, float value) {
  char buf[16];
  dtostrf(value, 0, 1, buf);
  mqtt.publish(topic, buf);
}

// guard/mode/bearing are EVENTS, not a stream of measurements: sent only on change.
void publishGuardIfChanged() {
  int state = tiltBlocked ? 1 : 0;
  if (state == guardSent) return;
  guardSent = state;
  mqtt.publish(TOPIC_GUARD, state ? "1" : "0");
  Serial.printf("[MQTT] -> %s = %d\n", TOPIC_GUARD, state);
}

void publishModeIfChanged() {
  int state = (autoState != AUTO_OFF) ? 1 : 0;
  if (state == modeSent) return;
  modeSent = state;
  mqtt.publish(TOPIC_MODE, state ? "1" : "0");
  Serial.printf("[MQTT] -> %s = %d\n", TOPIC_MODE, state);
}

void publishBearing(int16_t deg) {
  if (deg == bearingSent) return;
  bearingSent = deg;
  char buf[8];
  itoa(deg, buf, 10);
  mqtt.publish(TOPIC_BEARING, buf);
  Serial.printf("[MQTT] -> %s = %d\n", TOPIC_BEARING, deg);
}

// Distance ahead. -1 means "don't know": either there is no echo, or the reading is stale
// (a look is in progress, the horn faces sideways). The dashboard must draw -1 as "no echo",
// not as zero centimetres and not as "free".
void publishDistance() {
  bool fresh = !isnan(lastFwdCm) && (millis() - lastFwdAt) <= DIST_STALE_MS;
  publishNumber(TOPIC_DIST, fresh ? lastFwdCm : -1.0f);
}

// ---------------------------------------------------------------------------
//  TILT GUARD — unchanged from lesson 22 (two filters, both needed:
//  details are there, in the function's comment).
// ---------------------------------------------------------------------------
void updateTiltGuard(float accelG) {
  if (fabsf(accelG - 1.0f) > ACCEL_TRUST_BAND) return;

  float tilt = max(fabsf(pitch), fabsf(roll));
  unsigned long now = millis();

  if (!tiltBlocked) {
    if (tilt >= TILT_LIMIT) {
      if (overLimitSince == 0) overLimitSince = now;
      if (now - overLimitSince >= TILT_DEBOUNCE_MS) {
        tiltBlocked = true;
        overLimitSince = 0;
        Serial.printf("[GUARD] наклон %.0f град >= %.0f держится %lu мс — СТОП\n",
                      tilt, TILT_LIMIT, TILT_DEBOUNCE_MS);
        driveStop();
      }
    } else {
      overLimitSince = 0;
    }
  } else if (tilt <= TILT_RELEASE) {
    tiltBlocked = false;
    Serial.printf("[GUARD] наклон %.0f град <= %.0f — снова можно ехать\n", tilt, TILT_RELEASE);
  }
}

// ---------------------------------------------------------------------------
//  AUTOPILOT: on/off
// ---------------------------------------------------------------------------
void beginScan();   // below

void setAutopilot(bool on, const char* why) {
  if (on == (autoState != AUTO_OFF)) return;

  driveStop();
  if (on) {
    // ALWAYS start with a look, not with driving: we have no idea what is in front
    // of our nose, and "let's go and figure it out" is exactly the autopilot that crashes.
    blindCount = 0;
    autoPaused = false;
    autoState  = AUTO_SCAN;
    beginScan();
    Serial.printf("\n[AUTO] ВКЛЮЧЁН (%s) — сначала осмотреться\n", why);
  } else {
    autoState = AUTO_OFF;
    servoAim(SERVO_FORWARD_DEG, SETTLE_FAR_MS);   // point the sonar back ahead
    Serial.printf("[AUTO] выключен (%s)\n", why);
  }
  publishModeIfChanged();
}

// ---------------------------------------------------------------------------
//  AUTOPILOT: sector look, one step per loop()
//
//  This is the main difference from lesson 28. There sweep() sat in delay() for two
//  one-second stretches; here each call does the MINIMUM of work and immediately returns
//  control, and "how much longer to wait" lives in scanNextAt. The only thing that
//  still blocks is pulseIn itself (up to 15 ms), and that is physics, not code.
// ---------------------------------------------------------------------------
void beginScan() {
  for (uint8_t i = 0; i < SCAN_POINTS; i++) scanBuf[i] = NAN;
  scanIndex = 0;
  scanPing  = 0;
  pingCount = 0;
  scanPhase = SCAN_SETTLE;
  scanStartedAt = millis();
  servoAim(angleAt(0), SETTLE_FAR_MS);   // from 90° to 30° is a long move, not a step
  scanNextAt = servoReadyAt;
}

// true = sweep finished, result in scanBuf.
bool serviceScan() {
  uint32_t now = millis();
  if ((int32_t)(now - scanNextAt) < 0) return false;

  if (scanPhase == SCAN_SETTLE) {
    // The horn has arrived and settled — start a series of "pings" at this angle.
    scanPhase = SCAN_PING;
    scanPing  = 0;
    pingCount = 0;
  }

  // Exactly ONE "ping" per pass of loop().
  float cm = pingCm();
  if (!isnan(cm)) pingBuf[pingCount++] = cm;
  scanPing++;

  // The 90° point — also a fresh distance ahead for telemetry.
  if (!isnan(cm) && angleAt(scanIndex) == SERVO_FORWARD_DEG) {
    lastFwdCm = cm;
    lastFwdAt = now;
  }

  if (scanPing < PINGS_PER_POINT) {
    scanNextAt = now + PING_GAP_MS;      // datasheet: no more often than once per 60 ms
    return false;
  }

  scanBuf[scanIndex] = medianOf(pingBuf, pingCount);
  scanIndex++;
  if (scanIndex >= SCAN_POINTS) return true;

  servoAim(angleAt(scanIndex), SETTLE_MS);
  scanPhase  = SCAN_SETTLE;
  scanNextAt = servoReadyAt;
  return false;
}

// Print the sector as a "radar" — same as in lesson 28, but one line per point.
void printScan() {
  Serial.println("--------- осмотр ---------------------------");
  for (uint8_t i = 0; i < SCAN_POINTS; i++) {
    if (isnan(scanBuf[i])) { Serial.printf("%4u°  |  нет эха\n", angleAt(i)); continue; }
    uint8_t bars = (uint8_t)constrain((int)(scanBuf[i] / 10.0f), 1, 20);
    char bar[21];
    memset(bar, '#', bars);
    bar[bars] = '\0';
    Serial.printf("%4u°  | %6.1f см  |%s\n", angleAt(i), scanBuf[i], bar);
  }
  Serial.printf("проход занял %lu мс\n", (unsigned long)(millis() - scanStartedAt));
}

// ---------------------------------------------------------------------------
//  AUTOPILOT: turn by the gyro
//
//  The sign of yaw does NOT matter: the direction is set by the motors, and the gyro
//  only measures how much has been turned so far — so we take the absolute difference.
//  This also removes the question "which way does the Z axis count on this particular board".
//  The timeout is for when yaw doesn't grow: a wheel slips, the robot is stuck,
//  the sensor dropped out. Then we simply stop and go look around again.
// ---------------------------------------------------------------------------
void beginTurn(int16_t deg, bool left) {
  turnGoalDeg   = deg;
  turnStartYaw  = yaw;
  turnDeadline  = millis() + 400 + (uint32_t)deg * 25;   // with a large margin
  servoAim(SERVO_FORWARD_DEG, SETTLE_MS);                // sonar — straight ahead
  driveTank(left ? -1 : +1, left ? +1 : -1, SPEED_AUTO); // tank-style spin in place
  autoState = AUTO_TURN;
  Serial.printf("[AUTO] поворот %s на %d град\n", left ? "ВЛЕВО" : "ВПРАВО", deg);
}

void serviceTurn() {
  float turned = fabsf(yaw - turnStartYaw);
  bool done    = turned >= turnGoalDeg;
  bool timeout = (int32_t)(millis() - turnDeadline) >= 0;
  if (!done && !timeout) return;

  driveStop();
  if (timeout && !done)
    Serial.printf("[AUTO] поворот по таймауту: накрутили %.0f из %d град "
                  "(буксуем? упёрлись?)\n", turned, turnGoalDeg);
  // After a turn — look again. "Turned and drove off" would mean driving on
  // data taken before the turn, i.e. on the old world.
  autoState = AUTO_SCAN;
  beginScan();
}

// ---------------------------------------------------------------------------
//  AUTOPILOT: decision based on the look
// ---------------------------------------------------------------------------
void decideAfterScan() {
  printScan();

  int8_t best = -1;
  for (uint8_t i = 0; i < SCAN_POINTS; i++) {
    if (isnan(scanBuf[i]) || scanBuf[i] < CLEAR_CM) continue;
    if (best < 0 || scanBuf[i] > scanBuf[best]) best = i;
  }

  if (best < 0) {
    // Not a single free direction. This is NOT "all clear around" — it is either a corner,
    // or we are standing nose-first in a sofa the sonar can't hear. Back up and look again.
    blindCount++;
    Serial.printf("[AUTO] свободных направлений нет (попытка %u из %u) — назад\n",
                  blindCount, BLIND_LIMIT);
    if (blindCount >= BLIND_LIMIT) {
      // Give up honestly: shut down and report mode=0 to the backend, so the toggle in the
      // dashboard switches off by itself. A robot that "will get out somehow" gets out through the skirting board.
      setAutopilot(false, "три осмотра подряд без свободного направления");
      return;
    }
    driveTank(-1, -1, SPEED_AUTO);
    turnDeadline = millis() + BACK_MS;   // reused as "until when to back up"
    autoState = AUTO_BACK;
    return;
  }

  blindCount = 0;
  int16_t bearingDeg = angleAt(best);
  publishBearing(bearingDeg);

  int16_t delta = bearingDeg - (int16_t)SERVO_FORWARD_DEG;   // >0 — towards the "larger" angles
  Serial.printf("[AUTO] свободнее всего %d° (%.0f см)\n", bearingDeg, scanBuf[best]);

  if (abs(delta) <= TURN_DEADZONE) {
    servoAim(SERVO_FORWARD_DEG, SETTLE_MS);
    unknownRun = 0;
    fwdCount   = 0;
    nextPingAt = servoReadyAt;
    driveTank(+1, +1, SPEED_AUTO);
    autoState = AUTO_DRIVE;
    Serial.println("[AUTO] прямо по курсу свободно — поехали");
    return;
  }

  bool left = SERVO_HIGH_IS_LEFT ? (delta > 0) : (delta < 0);
  beginTurn(max((int16_t)abs(delta), TURN_MIN_DEG), left);
}

// ---------------------------------------------------------------------------
//  AUTOPILOT: driving forward with "pings" ahead
// ---------------------------------------------------------------------------
void serviceDrive() {
  uint32_t now = millis();
  if (!servoSettled() || (int32_t)(now - nextPingAt) < 0) return;
  nextPingAt = now + PING_GAP_MS;

  float cm = pingCm();
  if (isnan(cm)) {
    // "No echo" is not "free". But it is not panic either: a single silence in a room
    // merely means "further than 2.5 m". Count CONSECUTIVE silences.
    if (++unknownRun >= UNKNOWN_LIMIT) {
      Serial.printf("[AUTO] %u измерений подряд без эха — это «не знаю», тормозим\n", unknownRun);
      driveStop();
      autoState = AUTO_SCAN;
      beginScan();
    }
    return;
  }

  unknownRun = 0;
  lastFwdCm  = cm;
  lastFwdAt  = now;

  if (fwdCount < PINGS_PER_POINT) fwdRing[fwdCount++] = cm;
  else { fwdRing[0] = fwdRing[1]; fwdRing[1] = fwdRing[2]; fwdRing[2] = cm; }

  float buf[PINGS_PER_POINT];
  memcpy(buf, fwdRing, sizeof(float) * fwdCount);
  float median = medianOf(buf, fwdCount);
  if (isnan(median)) return;                 // haven't collected three yet — keep driving

  if (median <= STOP_CM) {
    Serial.printf("[AUTO] стена в %.0f см (порог %.0f) — стоп, осмотреться\n", median, STOP_CM);
    driveStop();
    autoState = AUTO_SCAN;
    beginScan();
  }
}

// ---------------------------------------------------------------------------
//  AUTOPILOT: one state-machine step per pass of loop()
// ---------------------------------------------------------------------------
void serviceAutopilot() {
  if (autoState == AUTO_OFF) return;

  // The tilt guard outranks the autopilot: while the car is tipped up — stand still.
  if (tiltBlocked) {
    if (moving) driveStop();
    autoPaused = true;
    return;
  }
  if (autoPaused) {
    // Released. The world may have changed meanwhile (we may, for example, have been lifted
    // off the obstacle and set down facing elsewhere) — so we don't resume, we look again.
    autoPaused = false;
    autoState  = AUTO_SCAN;
    beginScan();
    Serial.println("[AUTO] наклон отпустило — осматриваемся заново");
    return;
  }

  switch (autoState) {
    case AUTO_DRIVE:
      serviceDrive();
      break;

    case AUTO_SCAN:
      if (serviceScan()) decideAfterScan();
      break;

    case AUTO_TURN:
      serviceTurn();
      break;

    case AUTO_BACK:
      if ((int32_t)(millis() - turnDeadline) >= 0) {
        driveStop();
        autoState = AUTO_SCAN;
        beginScan();
      }
      break;

    default:
      break;
  }
}

// ---------------------------------------------------------------------------
//  COMMANDS
// ---------------------------------------------------------------------------
void applyCommand(const String& cmd) {
  // First PARSE the command, and only then do anything. The order matters: below, a
  // recognised command turns the autopilot off, and garbage in the topic must not get
  // that right — a "stop" from a typo in someone else's client is not a stop, it's an accident.
  int left, right;
  if      (cmd == "stop")    { left =  0; right =  0; }
  else if (cmd == "forward") { left = +1; right = +1; }
  else if (cmd == "back")    { left = -1; right = -1; }
  else if (cmd == "left")    { left = -1; right = +1; }
  else if (cmd == "right")   { left = +1; right = -1; }
  else { Serial.printf("[DRIVE] неизвестная команда '%s'\n", cmd.c_str()); return; }

  // The human grabbed the wheel -> the autopilot silently steps aside. This is not an
  // "error" but the normal way to switch it off: the dashboard button, a gamepad,
  // and the D-pad on the FPV page from lesson 26, which knows nothing about the autopilot, all work.
  if (autoState != AUTO_OFF) setAutopilot(false, "ручная команда — человек взялся за руль");

  // "Stop" is ALWAYS executed, even with the guard active: stop is safe by definition.
  if (left == 0 && right == 0) { driveStop(); return; }

  if (tiltBlocked) {
    Serial.printf("[GUARD] наклон %.0f/%.0f град — команда '%s' проигнорирована\n",
                  pitch, roll, cmd.c_str());
    return;
  }

  driveTank(left, right, SPEED_MANUAL);
}

void applyMode(const String& mode) {
  if      (mode == "auto")   setAutopilot(true,  "команда из браузера");
  else if (mode == "manual") setAutopilot(false, "команда из браузера");
  else Serial.printf("[MODE] неизвестный режим '%s' (жду auto|manual)\n", mode.c_str());
}

void onMessage(char* topic, byte* payload, unsigned int length) {
  String body;
  for (unsigned int i = 0; i < length; i++) body += (char)payload[i];
  body.trim();
  body.toLowerCase();

  Serial.printf("[MQTT] <- %s = %s\n", topic, body.c_str());

  if (strcmp(topic, TOPIC_CMDMODE) == 0) { applyMode(body); return; }

  lastCommandMs = millis();   // manual-mode failsafe
  applyCommand(body);
}

void reconnect() {
  while (!mqtt.connected()) {
    // The autopilot must NOT drive without a connection. Not because it needs commands — it
    // is self-sufficient precisely — but because without the broker there is nothing to stop it
    // with: neither the dashboard button nor telemetry shows what it is doing.
    if (autoState != AUTO_OFF) setAutopilot(false, "потеряна связь с брокером");

    Serial.print("[MQTT] Подключение к брокеру... ");
    if (mqtt.connect(CLIENT_ID, MQTT_USER, MQTT_PASS)) {
      Serial.println("OK");
      mqtt.subscribe(TOPIC_DRIVE, 1);      // QoS 1 — commands matter more than telemetry
      mqtt.subscribe(TOPIC_CMDMODE, 1);
      Serial.printf("[MQTT] Подписки: '%s', '%s' (QoS 1)\n", TOPIC_DRIVE, TOPIC_CMDMODE);
      // After reconnecting, report the state of all "event" metrics to the backend again.
      guardSent   = -1;
      modeSent    = -1;
      bearingSent = -1000;
    } else {
      Serial.printf("ошибка, state=%d, повтор через 3 c\n", mqtt.state());
      driveStop();
      delay(3000);
    }
  }
}

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  // 1) MOTORS — stop first of all: three seconds of gyro calibration follow.
  //    Order: first the PWM channel at zero, then DIR — so that there is never, even
  //    for an instant, "speed present, direction random".
  ledcSetup(CH_MOTOR_L, MOTOR_PWM_HZ, MOTOR_PWM_BITS);
  ledcSetup(CH_MOTOR_R, MOTOR_PWM_HZ, MOTOR_PWM_BITS);
  ledcWrite(CH_MOTOR_L, 0);
  ledcWrite(CH_MOTOR_R, 0);
  ledcAttachPin(MDD_PWM1, CH_MOTOR_L);
  ledcAttachPin(MDD_PWM2, CH_MOTOR_R);
  pinMode(MDD_DIR1, OUTPUT);
  pinMode(MDD_DIR2, OUTPUT);
  digitalWrite(MDD_DIR1, LOW);
  digitalWrite(MDD_DIR2, LOW);
  driveStop();

  // 2) SONAR (lesson 27)
  pinMode(SONAR_TRIG, OUTPUT);
  digitalWrite(SONAR_TRIG, LOW);
  pinMode(SONAR_ECHO, INPUT);          // GPIO39 — input-only, the divider holds the line

  // 3) SERVO (lesson 28): 50 Hz, 16 bit, channel 4 (timer 2).
  ledcSetup(CH_SERVO, SERVO_FREQ_HZ, SERVO_BITS);
  ledcAttachPin(SERVO_PIN, CH_SERVO);
  servoAim(SERVO_FORWARD_DEG, SETTLE_FAR_MS);
  delay(600);                           // blocking is still fine in setup()
  Serial.println("\n=== Урок 30. Автопилот v1 на Cytron MDD10A ===");
  Serial.println("сонар смотрит на 90° — проверь, что он смотрит СТРОГО ВПЕРЁД");

  // 4) I2C + MPU6050 (lessons 20/22)
  Wire.begin(MPU_SDA, MPU_SCL);
  Wire.setClock(400000);
  Serial.print("[MPU] Инициализация... ");
  mpu.begin();
  Serial.println("калибровка гироскопа, не трогай робота ~3 c...");
  mpu.calcGyroOffsets(true);
  Serial.println("[MPU] OK");

  // 5) Wi-Fi
  WiFi.begin(SSID, PASS);
  Serial.print("[WiFi] Подключение");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.printf("\n[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());

  // 6) MQTT
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setCallback(onMessage);

  lastCommandMs = millis();
  nextPingAt    = millis();
  Serial.printf("[AUTO] порог торможения %.0f см, свободным считаем от %.0f см\n",
                STOP_CM, CLEAR_CM);
  Serial.println("[AUTO] включается командой auto в commands/esp32/mode\n");
}

void loop() {
  // 1) Connection. Without mqtt.loop() neither subscriptions nor keep-alive work.
  if (!mqtt.connected()) reconnect();
  mqtt.loop();

  // 2) Sensor — every iteration: this makes the gyro integral more accurate, and the
  //    autopilot's turn relies on it.
  mpu.update();
  float ax = mpu.getAccX(), ay = mpu.getAccY(), az = mpu.getAccZ();
  roll  = atan2f(ay, az) * 180.0f / PI;
  pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI;
  yaw   = mpu.getAngleZ();

  updateTiltGuard(sqrtf(ax * ax + ay * ay + az * az));

  // 3) Autopilot — exactly one state-machine step. Nothing blocking except pulseIn.
  serviceAutopilot();

  // 3a) Motors: complete a deferred reversal (REVERSE_PAUSE_MS pause without delay).
  motorService();

  // 4) "Ahead" sonar in MANUAL mode. Needed not by the autopilot but by the human: the
  //    dashboard shows the distance and warns about a wall while steering by hand.
  //    In AUTO_DRIVE serviceDrive() takes care of this; during a look the horn faces
  //    sideways — then there is nothing to measure "ahead" with.
  if (autoState == AUTO_OFF && servoSettled() && (int32_t)(millis() - nextPingAt) >= 0) {
    nextPingAt = millis() + PING_GAP_MS;
    float cm = pingCm();
    if (!isnan(cm)) { lastFwdCm = cm; lastFwdAt = millis(); }
  }

  // 5) Failsafe from lessons 21/22 — ONLY in manual mode. In autopilot there are no commands
  //    by definition, and this timer would kill it 700 ms after the start.
  if (autoState == AUTO_OFF && moving && millis() - lastCommandMs > FAILSAFE_MS) {
    Serial.println("[DRIVE] failsafe: команд нет, стоп");
    driveStop();
  }

  // 6) Telemetry once per PUBLISH_INTERVAL ms, without a blocking delay.
  unsigned long now = millis();
  if (now - lastPublish >= PUBLISH_INTERVAL) {
    lastPublish = now;
    publishNumber(TOPIC_PITCH, pitch);
    publishNumber(TOPIC_ROLL,  roll);
    publishNumber(TOPIC_YAW,   yaw);
    publishDistance();
    publishGuardIfChanged();
    publishModeIfChanged();
  }
}
