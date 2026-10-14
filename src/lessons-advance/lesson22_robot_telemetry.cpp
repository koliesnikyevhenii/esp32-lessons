// ============================================================================
//  Stage 3. Robot — lesson 22: telemetry + control in ONE firmware
// ----------------------------------------------------------------------------
//  Goal: merge lesson 20 (tilt angle from the MPU6050) and lesson 21 (motors via
//  the TB6612FNG). Now a single firmware simultaneously:
//    • PUBLISHES the tilt angle    — sensors/esp32/pitch|roll|yaw   (as in lesson 20);
//    • is SUBSCRIBED to commands   — commands/esp32/drive           (as in lesson 21).
//
//   Browser --HTTP--> ASP.NET --AMQP--> RabbitMQ --MQTT--> ESP32 --> motors
//   Browser <--SignalR-- ASP.NET <--AMQP-- RabbitMQ <--MQTT-- ESP32 <-- MPU6050
//
//  So in the dashboard we STEER the car and in the same second we SEE how it
//  tilts: both directions of exchange live on one MQTT connection.
//
//  Since lesson 26 this firmware has a SECOND remote: the ESP32-CAM publishes
//  button presses into the same topic commands/esp32/drive, and listens to the
//  guard metric (below) to show the lockout on the FPV page. Nothing had to change
//  here — the robot listens to a TOPIC, not to a specific client; you can steer from
//  the dashboard and from the camera at the same time. The second board with the
//  camera sits on the car next to this one.
//
//  What is new compared to lessons 20 and 21 (this is not just "glueing"):
//    • one PubSubClient handles both publish and subscribe — it is important not to
//      block loop() (no delay), otherwise commands start arriving late;
//    • mpu.update() is called on EVERY loop() iteration, while publish is once per
//      200 ms: tockn integrates the gyro over time inside update(), so the more often
//      you call it, the less yaw lies (in lesson 20 update() ran once per 200 ms);
//    • TILT GUARD — a "sensor -> motors" feedback loop: if the car tips up steeper
//      than TILT_LIMIT (drove onto an obstacle, stood on its end), the firmware
//      itself kills the motors and ignores drive commands until the tilt returns
//      below TILT_RELEASE (hysteresis, so it does not chatter).
//      A single sample past the limit is NOT enough for it: the motors' starting
//      jolt used to kill itself, because the angle is computed from the acceleration
//      vector and lies by tens of degrees during a jolt — details in updateTiltGuard();
//    • the state of this guard goes to the backend as an ORDINARY metric guard (0/1),
//      but is published ONLY ON CHANGE — "publish on change" instead of
//      "publish by timer": events are rare, no point writing to the DB every 200 ms.
//
//  Pins: the MPU sits on I2C (32/17), the motors on 13/14/25 and 26/27/33. The groups
//  do not overlap (see include/pins.h) — this is a mandatory condition of lesson 22,
//  since here, unlike 20 and 21, both devices are connected AT THE SAME TIME.
//
//  Libraries: knolleary/PubSubClient + tockn/MPU6050_tockn
//  (see platformio.ini, env lesson22_robot_telemetry).
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
// LAN IP of the computer running docker compose (NOT localhost: the ESP32 is a separate device).
const char* MQTT_BROKER = "192.168.0.7";   // <-- replace with your PC's IP
const uint16_t MQTT_PORT = 1883;

// guest only works from localhost — the ESP32 needs a separate user.
const char* MQTT_USER = "esp";
const char* MQTT_PASS = "esp-pass";

const char* DEVICE_ID = "esp32";
const char* CLIENT_ID = "esp32-lesson22";

// ---- Topics ----------------------------------------------------------------
// The contracts are the same as in lessons 20/21 — the backend does NOT need changes:
//   telemetry   sensors/<device>/<metric>   body = bare number ("-3.4")
//   commands    commands/<device>/drive     body = word (forward|back|left|right|stop)
const char* TOPIC_PITCH = "sensors/esp32/pitch";
const char* TOPIC_ROLL  = "sensors/esp32/roll";
const char* TOPIC_YAW   = "sensors/esp32/yaw";
const char* TOPIC_GUARD = "sensors/esp32/guard";   // 0 = OK to drive, 1 = blocked by tilt
const char* TOPIC_DRIVE = "commands/esp32/drive";

// ---- Sensor ----------------------------------------------------------------
// tockn does not check WHO_AM_I, so it works with both the MPU6050 and the MPU6500 (0x70).
MPU6050 mpu(Wire);

// ---- Motors (TB6612FNG) ----------------------------------------------------
// A = left motor, B = right. The driver's STBY is on 3V3 (always enabled).
const int SPEED = 200;                       // 0..255 — PWM speed
const unsigned long FAILSAFE_MS = 700;       // no commands for longer -> stop

// ---- Tilt guard ------------------------------------------------------------
// Hysteresis: lock at TILT_LIMIT, unlock only below TILT_RELEASE.
// A single threshold for both events would chatter at the boundary (on/off/on...).
const float TILT_LIMIT   = 45.0f;            // degrees — steeper than this, stop
const float TILT_RELEASE = 35.0f;            // degrees — below this, OK again

// Two filters against FALSE trips. Why they are needed at all: without them the
// guard tripped on EVERY motor start: the log showed "pitch = 1.8", right after
// the forward command — "tilt 47 deg >= 45 — STOP", a couple hundred milliseconds
// later the release. The car twitched and stood still. Measured over six events:
// the longest false trip lived 228 ms, the shortest — 47 ms.
const unsigned long TILT_DEBOUNCE_MS = 300;  // how long the tilt must HOLD before we believe it
const float ACCEL_TRUST_BAND = 0.25f;        // |a| outside 1.0 +- this (g) — sample is not trusted

// ---- Timings ---------------------------------------------------------------
const unsigned long PUBLISH_INTERVAL = 200;  // 200 ms = 5 times per second

// ---- State -----------------------------------------------------------------
unsigned long lastCommandMs = 0;   // when we last heard a command
unsigned long lastPublish   = 0;   // when we last published the angles
bool moving      = false;          // motors are currently spinning
bool tiltBlocked = false;          // the tilt guard has tripped
unsigned long overLimitSince = 0;  // when the tilt went past the limit (0 = currently normal)
int  guardSent   = -1;             // what the backend already knows about guard (-1 = nothing yet)
float pitch = 0, roll = 0, yaw = 0;

// ---- Network objects -------------------------------------------------------
WiFiClient   net;
PubSubClient mqtt(net);

// ---------------------------------------------------------------------------
//  MOTORS (as in lesson 21)
//  One motor: dir>0 — forward, dir<0 — backward, dir==0 — stop.
// ---------------------------------------------------------------------------
void motor(int in1, int in2, int pwm, int dir) {
  digitalWrite(in1, dir > 0 ? HIGH : LOW);
  digitalWrite(in2, dir < 0 ? HIGH : LOW);
  analogWrite(pwm, dir == 0 ? 0 : SPEED);
}

void driveStop() {
  motor(MOTOR_AIN1, MOTOR_AIN2, MOTOR_PWMA, 0);
  motor(MOTOR_BIN1, MOTOR_BIN2, MOTOR_PWMB, 0);
  moving = false;
}

// Tank-style control: left/right — spin in place (motors in opposite directions).
void applyCommand(const String& cmd) {
  // "stop" is ALWAYS executed, even with the guard active — stop is safe by definition.
  if (cmd == "stop") { driveStop(); return; }

  // Tilt guard: drive commands are ignored while the car is tipped up.
  if (tiltBlocked) {
    Serial.printf("[GUARD] наклон %.0f/%.0f град — команда '%s' проигнорирована\n",
                  pitch, roll, cmd.c_str());
    return;
  }

  int left, right;
  if      (cmd == "forward") { left = +1; right = +1; }
  else if (cmd == "back")    { left = -1; right = -1; }
  else if (cmd == "left")    { left = -1; right = +1; }
  else if (cmd == "right")   { left = +1; right = -1; }
  else { Serial.printf("[DRIVE] неизвестная команда '%s'\n", cmd.c_str()); return; }

  motor(MOTOR_AIN1, MOTOR_AIN2, MOTOR_PWMA, left);
  motor(MOTOR_BIN1, MOTOR_BIN2, MOTOR_PWMB, right);
  moving = true;
}

// ---------------------------------------------------------------------------
//  TELEMETRY
//  Publish one number to a topic. The body is a bare string — exactly what the
//  .NET consumer parses (TelemetryConsumer.ParseReading).
// ---------------------------------------------------------------------------
void publishNumber(const char* topic, float value) {
  char buf[16];
  dtostrf(value, 0, 1, buf);          // e.g. "-3.4"
  mqtt.publish(topic, buf);
  Serial.printf("[MQTT] -> %s = %s\n", topic, buf);
}

// guard is sent ONLY when the state has changed (publish on change): it is an event,
// not a stream of measurements — no point pouring 5 identical rows per second into the DB.
void publishGuardIfChanged() {
  int state = tiltBlocked ? 1 : 0;
  if (state == guardSent) return;
  guardSent = state;
  mqtt.publish(TOPIC_GUARD, state ? "1" : "0");
  Serial.printf("[MQTT] -> %s = %d\n", TOPIC_GUARD, state);
}

// ---------------------------------------------------------------------------
//  The "sensor -> motors" feedback loop: the very reason to merge 20 and 21.
//  Take the tilt from the larger of the two axes and compare it with the thresholds.
//
//  IMPORTANT ABOUT FALSE TRIPS. pitch/roll above are derived from the ACCELERATION
//  VECTOR (atan2 over ax/ay/az), and the accelerometer measures gravity PLUS the
//  robot's own acceleration. While the car stands still these are the same and the
//  angle is honest; on the motors' starting jolt a horizontal component appears,
//  and atan2 reports 45-48 degrees on a level floor. A naive guard immediately killed
//  the motors — i.e. the jolt killed itself, and the robot twitched instead of driving.
//
//  So a single sample past the limit is no longer trusted, and there are TWO filters:
//    1) SAMPLE TRUST — at rest |a| = 1 g; if the magnitude drifted by more than
//       ACCEL_TRUST_BAND, the robot is accelerating/braking/bouncing, and the angle
//       from such a sample is garbage. Such a sample is skipped ENTIRELY: it does not
//       arm the guard, but it does not release it or reset the countdown either;
//    2) DEBOUNCE — arm only if the tilt stays past TILT_LIMIT continuously for
//       TILT_DEBOUNCE_MS. A real rollover holds the tilt for seconds; the measured
//       false trips lived 47-228 ms, i.e. all shorter than 300.
//  Each filter alone is weaker: |a| without debounce won't save you from prolonged
//  shaking, and debounce without |a| simply waits out the jolt instead of not
//  counting it at all.
// ---------------------------------------------------------------------------
void updateTiltGuard(float accelG) {
  // Filter 1: a sample with a distorted acceleration magnitude is not trusted.
  if (fabsf(accelG - 1.0f) > ACCEL_TRUST_BAND) return;

  float tilt = max(fabsf(pitch), fabsf(roll));
  unsigned long now = millis();

  if (!tiltBlocked) {
    if (tilt >= TILT_LIMIT) {
      // Filter 2: note the first trusted sample past the limit and wait.
      if (overLimitSince == 0) overLimitSince = now;
      if (now - overLimitSince >= TILT_DEBOUNCE_MS) {
        tiltBlocked = true;
        overLimitSince = 0;
        Serial.printf("[GUARD] наклон %.0f град >= %.0f держится %lu мс — СТОП\n",
                      tilt, TILT_LIMIT, TILT_DEBOUNCE_MS);
        driveStop();                 // kill right away, without waiting for a command
      }
    } else {
      overLimitSince = 0;            // back to normal — restart the countdown
    }
  } else if (tilt <= TILT_RELEASE) {
    // Release — on the very first TRUSTED sample below the threshold, no debounce:
    // keeping the robot blocked longer than needed is worse than releasing it
    // early, and the TILT_RELEASE hysteresis already prevents chatter at the boundary.
    tiltBlocked = false;
    Serial.printf("[GUARD] наклон %.0f град <= %.0f — снова можно ехать\n", tilt, TILT_RELEASE);
    // Motors are NOT touched: while the button is held, the browser will resend a command within ~300 ms.
  }
}

// ---------------------------------------------------------------------------
//  SUBSCRIBE: callback for an incoming command. payload has no trailing zero -> length.
// ---------------------------------------------------------------------------
void onMessage(char* topic, byte* payload, unsigned int length) {
  String cmd;
  for (unsigned int i = 0; i < length; i++) cmd += (char)payload[i];
  cmd.trim();

  Serial.printf("[MQTT] <- %s = %s\n", topic, cmd.c_str());

  lastCommandMs = millis();   // saw a command -> reset the failsafe timer
  applyCommand(cmd);
}

void reconnect() {
  while (!mqtt.connected()) {
    Serial.print("[MQTT] Подключение к брокеру... ");
    if (mqtt.connect(CLIENT_ID, MQTT_USER, MQTT_PASS)) {
      Serial.println("OK");
      // QoS 1 — commands matter more than telemetry, better "deliver at least once".
      mqtt.subscribe(TOPIC_DRIVE, 1);
      Serial.printf("[MQTT] Подписка на '%s' (QoS 1)\n", TOPIC_DRIVE);
      guardSent = -1;   // after reconnecting, report the guard state to the backend again
    } else {
      // state(): -2 = network, 4 = login/password, 5 = not authorized.
      Serial.printf("ошибка, state=%d, повтор через 3 c\n", mqtt.state());
      // Motors must stand still while reconnecting: there are no commands anyway.
      driveStop();
      delay(3000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  // 1) MOTORS — first of all: pins to OUTPUT and stop right away. It matters that the
  //    robot doesn't twitch while we spend three seconds calibrating the gyro and joining Wi-Fi.
  int pins[] = { MOTOR_AIN1, MOTOR_AIN2, MOTOR_PWMA, MOTOR_BIN1, MOTOR_BIN2, MOTOR_PWMB };
  for (int p : pins) pinMode(p, OUTPUT);
  driveStop();

  // 2) I2C + sensor. Wire.begin(SDA, SCL) — pins are set explicitly.
  Wire.begin(MPU_SDA, MPU_SCL);
  Wire.setClock(400000);        // 400 kHz: fast enough to poll the sensor on every loop()
  Serial.print("[MPU] Инициализация... ");
  mpu.begin();

  // Gyro zero calibration: the robot must stand STILL and LEVEL for ~3 seconds.
  // The motors are already stopped (step 1), so there is no vibration — otherwise yaw drifts off at once.
  Serial.println("калибровка гироскопа, не трогай робота ~3 c...");
  mpu.calcGyroOffsets(true);
  Serial.println("[MPU] OK");

  // 3) Wi-Fi
  WiFi.begin(SSID, PASS);
  Serial.print("[WiFi] Подключение");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.printf("\n[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());

  // 4) MQTT: broker address + callback (in lesson 20 the callback wasn't needed, here it is mandatory).
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setCallback(onMessage);

  lastCommandMs = millis();
}

void loop() {
  // 1) Keep the connection alive. MANDATORY on every pass: without mqtt.loop()
  //    neither subscriptions nor keep-alive work.
  if (!mqtt.connected()) reconnect();
  mqtt.loop();

  // 2) The sensor is polled on every iteration — this makes the gyro integral (yaw) more accurate.
  mpu.update();
  float ax = mpu.getAccX();
  float ay = mpu.getAccY();
  float az = mpu.getAccZ();

  // Tilt angle from the gravity vector (in degrees), as in lesson 20:
  //   roll  — "sideways" roll (around X), pitch — "fore-aft" pitch (around Y).
  roll  = atan2f(ay, az) * 180.0f / PI;
  pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI;
  // Heading: integral of the gyro around Z. There is no reference "north" -> it DRIFTS
  // over time, and drifts noticeably faster while moving — the motors shake the sensor.
  yaw = mpu.getAngleZ();

  // 3) Feedback: the tilt may kill the motors by itself. Pass the magnitude of the
  //    acceleration vector — the guard uses it to decide whether to trust the angle at all.
  updateTiltGuard(sqrtf(ax * ax + ay * ay + az * az));

  // 4) Failsafe from lesson 21: we're moving but no commands for a while -> stop.
  //    Tab closed / Wi-Fi lost — the robot doesn't "run away".
  if (moving && millis() - lastCommandMs > FAILSAFE_MS) {
    Serial.println("[DRIVE] failsafe: команд нет, стоп");
    driveStop();
  }

  // 5) Telemetry once per PUBLISH_INTERVAL ms (without a blocking delay).
  unsigned long now = millis();
  if (now - lastPublish >= PUBLISH_INTERVAL) {
    lastPublish = now;
    publishNumber(TOPIC_PITCH, pitch);
    publishNumber(TOPIC_ROLL,  roll);
    publishNumber(TOPIC_YAW,   yaw);
    publishGuardIfChanged();
  }
}
