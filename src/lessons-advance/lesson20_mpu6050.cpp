// ============================================================================
//  Stage 2. IoT Foundation — lesson 20: MPU6050 -> tilt angle (pitch/roll)
// ----------------------------------------------------------------------------
//  Goal: read X/Y/Z accelerations from the MPU6050 gyro-accelerometer,
//  compute the tilt angle (pitch/roll) and send it to the .NET backend over MQTT.
//
//      ESP32 --MQTT(1883)--> RabbitMQ --AMQP--> .NET Worker --> PostgreSQL
//                                                    |
//                                              SignalR -> React dashboard (pitch/roll/yaw)
//
//  A continuation of lesson 19. The MQTT part is the same (publish a bare number),
//  only the data source changes: instead of the DHT11 — an MPU6050 sensor on the I2C bus.
//
//  What's new compared to lesson 19:
//    • I2C — we read the sensor over two wires (SDA/SCL), address 0x68;
//    • accelerometer — accelerations ax/ay/az (in m/s^2), from which we get the tilt angle;
//    • pitch/roll — the angle via atan2 formulas (tilt "forward-back" and "sideways");
//    • we publish more often (5 times per second) — the dashboard draws "almost in real time".
//
//  Library: adafruit/Adafruit MPU6050 (see platformio.ini, env lesson20_mpu6050).
// ============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <MPU6050_tockn.h>
#include "pins.h"

// ---- Wi-Fi ----------------------------------------------------------------
const char* SSID = "Koleso";
const char* PASS = "12345zxc";

// ---- MQTT broker (RabbitMQ + mqtt plugin) ---------------------------------
// LAN IP of the computer running docker compose (NOT localhost: the ESP32 is a separate device).
const char* MQTT_BROKER = "192.168.1.164";   // <-- replace with your PC's IP
const uint16_t MQTT_PORT = 1883;

// RabbitMQ account. guest works only from localhost — for the ESP32 create
// a separate user esp / esp-pass (see lesson 19 / the telemetry README).
const char* MQTT_USER = "esp";
const char* MQTT_PASS = "esp-pass";

// <device> in topic sensors/<device>/<metric>.
const char* DEVICE_ID = "esp32";
const char* CLIENT_ID = "esp32-lesson20";

// ---- Topics ----------------------------------------------------------------
// One number per metric. The RabbitMQ MQTT plugin turns slashes into dots:
//   sensors/esp32/pitch -> routing key sensors.esp32.pitch,
// and the .NET consumer (ParseReading) reads it as sensors.<device>.<metric>.
// pitch, roll and yaw are just "metrics", the backend needs no changes for them.
const char* TOPIC_PITCH = "sensors/esp32/pitch";
const char* TOPIC_ROLL  = "sensors/esp32/roll";
const char* TOPIC_YAW   = "sensors/esp32/yaw";

// ---- Sensor ----------------------------------------------------------------
// The tockn library doesn't check WHO_AM_I, so it works with both the MPU6050 and
// the MPU6500 (our chip, WHO_AM_I=0x70). The constructor takes the I2C bus.
MPU6050 mpu(Wire);

// ---- Network objects -------------------------------------------------------
WiFiClient   net;
PubSubClient mqtt(net);

unsigned long lastPublish = 0;
const unsigned long PUBLISH_INTERVAL = 200;   // 200 ms = 5 times per second

// ---------------------------------------------------------------------------
//  Reconnecting to the broker. Here we only publish, there are no subscriptions,
//  so nothing extra needs to be set up after connect.
// ---------------------------------------------------------------------------
void reconnect() {
  while (!mqtt.connected()) {
    Serial.print("[MQTT] Подключение к брокеру... ");
    if (mqtt.connect(CLIENT_ID, MQTT_USER, MQTT_PASS)) {
      Serial.println("OK");
    } else {
      // state(): -2 = network, 4 = login/password, 5 = not authorized.
      Serial.printf("ошибка, state=%d, повтор через 3 c\n", mqtt.state());
      delay(3000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  // 1) I2C + sensor. Wire.begin(SDA, SCL) — we set the pins explicitly.
  Wire.begin(MPU_SDA, MPU_SCL);
  Serial.print("[MPU] Инициализация... ");
  mpu.begin();   // tockn::begin() returns nothing and doesn't check WHO_AM_I

  // Gyro zero calibration: the sensor must lie STILL for ~3 seconds.
  // The library averages the drift and subtracts it afterwards (otherwise the angles "wander").
  Serial.println("калибровка гироскопа, не трогай датчик...");
  mpu.calcGyroOffsets(true);   // true = print progress to Serial
  Serial.println("[MPU] OK");

  // 2) Wi-Fi
  WiFi.begin(SSID, PASS);
  Serial.print("[WiFi] Подключение");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.printf("\n[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());

  // 3) MQTT: only the broker address (no callback needed — we don't subscribe).
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
}

void loop() {
  // Keep the MQTT connection alive.
  if (!mqtt.connected()) reconnect();
  mqtt.loop();

  // PUBLISH every PUBLISH_INTERVAL ms (without a blocking delay).
  unsigned long now = millis();
  if (now - lastPublish >= PUBLISH_INTERVAL) {
    lastPublish = now;

    // Update the sensor readings (required before every read).
    mpu.update();

    // Accelerations along X/Y/Z (tockn returns them in g — units don't matter for atan2).
    float ax = mpu.getAccX();
    float ay = mpu.getAccY();
    float az = mpu.getAccZ();

    // Tilt angle from the gravity vector (in degrees):
    //   roll  — "sideways" bank         (rotation around the X axis);
    //   pitch — "forward-back" pitch    (rotation around the Y axis).
    // atan2 handles the signs and quadrants by itself.
    float roll  = atan2f(ay, az) * 180.0f / PI;
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI;

    // Heading (yaw, rotation around the vertical) can't be measured by the accelerometer:
    // gravity doesn't change under such a rotation. We take the integrated Z gyro,
    // which tockn tracks itself (getAngleZ; calcGyroOffsets subtracted the drift in setup).
    // There is no reference "north", so the value DRIFTS over time — that's normal
    // for gyro-only without a magnetometer.
    float yaw = mpu.getAngleZ();

    // Message body is a bare number (exactly what the .NET consumer parses).
    char buf[16];

    dtostrf(pitch, 0, 1, buf);            // e.g. "-3.4"
    mqtt.publish(TOPIC_PITCH, buf);
    Serial.printf("[MQTT] -> %s = %s\n", TOPIC_PITCH, buf);

    dtostrf(roll, 0, 1, buf);             // e.g. "12.1"
    mqtt.publish(TOPIC_ROLL, buf);
    Serial.printf("[MQTT] -> %s = %s\n", TOPIC_ROLL, buf);

    dtostrf(yaw, 0, 1, buf);              // e.g. "-45.0" (drifts over time)
    mqtt.publish(TOPIC_YAW, buf);
    Serial.printf("[MQTT] -> %s = %s\n", TOPIC_YAW, buf);
  }
}
