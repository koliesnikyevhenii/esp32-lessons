// ============================================================================
//  Stage 2. IoT Foundation — lesson 19: MQTT telemetry
// ----------------------------------------------------------------------------
//  Goal: connect the ESP32 and the .NET backend over MQTT.
//
//      ESP32 --MQTT(1883)--> RabbitMQ --AMQP--> .NET Worker --> PostgreSQL
//
//  This lesson covers 4 basic MQTT concepts:
//    • publish   — we publish telemetry (temperature, uptime);
//    • subscribe — we subscribe to a command topic and react with an LED;
//    • topics    — hierarchical names sensors/<device>/<metric> + wildcard '#';
//    • QoS       — delivery levels 0/1 and the limitations of the PubSubClient library.
//
//  Library: knolleary/PubSubClient (see platformio.ini, env lesson19_mqtt).
// ============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include "pins.h"

// ---- Wi-Fi ----------------------------------------------------------------
const char* SSID = "Vektor_04";
const char* PASS = "uteam2020";

// ---- MQTT broker (RabbitMQ + mqtt plugin) ---------------------------------
// IMPORTANT: put the LAN IP of the computer running docker compose (not localhost!
// The ESP32 is a separate device on the network). Port 1883 is exposed in docker-compose.yml.
const char* MQTT_BROKER = "192.168.0.3";   // <-- replace with your PC's IP
const uint16_t MQTT_PORT = 1883;

// RabbitMQ user. The guest account works ONLY from localhost, so for an
// ESP32 on the network create a separate user (Admin -> Add user), e.g.
// esp / esp-pass, and grant it permissions on vhost '/'. See the telemetry README.
const char* MQTT_USER = "esp";
const char* MQTT_PASS = "esp-pass";

// Device identifier. It is also the <device> in topic sensors/<device>/<metric>.
const char* DEVICE_ID  = "esp32";
const char* CLIENT_ID  = "esp32-lesson19";

// ---- Topics ----------------------------------------------------------------
// Publish: one number per metric. The RabbitMQ MQTT plugin turns
// slashes into dots: sensors/esp32/temperature  ->  routing key sensors.esp32.temperature,
// and the .NET consumer (ParseReading) parses it as sensors.<device>.<metric>.
const char* TOPIC_TEMPERATURE = "sensors/esp32/temperature";
const char* TOPIC_UPTIME      = "sensors/esp32/uptime";

// Subscribe: command topic with wildcard '#' (all subtopics commands/esp32/...).
// Example command: publish to commands/esp32/led  with body "on" / "off".
const char* TOPIC_COMMANDS = "commands/esp32/#";

// ---- Sensor ----------------------------------------------------------------
#define DHTPIN  33
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// ---- Network objects -------------------------------------------------------
WiFiClient   net;
PubSubClient mqtt(net);

unsigned long lastPublish = 0;
const unsigned long PUBLISH_INTERVAL = 5000;   // publish every 5 seconds

// ---------------------------------------------------------------------------
//  SUBSCRIBE: the callback is called when a message arrives on a subscribed topic.
//  payload is NOT a null-terminated string, so we use length.
// ---------------------------------------------------------------------------
void onMessage(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];

  Serial.printf("[MQTT] <- топик '%s', тело '%s'\n", topic, msg.c_str());

  // topics: react to commands/esp32/led. Body "on"/"off" switches the LED.
  if (String(topic).endsWith("/led")) {
    if (msg == "on")  digitalWrite(LED_RED2, HIGH);
    if (msg == "off") digitalWrite(LED_RED2, LOW);
  }
}

// ---------------------------------------------------------------------------
//  Reconnecting to the broker. The MQTT connection can drop — we keep it
//  alive in loop(). After connect the subscriptions MUST be set up again.
// ---------------------------------------------------------------------------
void reconnect() {
  while (!mqtt.connected()) {
    Serial.print("[MQTT] Подключение к брокеру... ");
    if (mqtt.connect(CLIENT_ID, MQTT_USER, MQTT_PASS)) {
      Serial.println("OK");

      // QoS on subscribe: PubSubClient supports 0 and 1.
      //   QoS 0 — "fire and forget", no acknowledgement (may be lost);
      //   QoS 1 — "at least once", the broker keeps sending until it gets a PUBACK
      //           (duplicates possible). We ask for QoS 1 for commands — they matter more.
      mqtt.subscribe(TOPIC_COMMANDS, 1);
      Serial.printf("[MQTT] Подписка на '%s' (QoS 1)\n", TOPIC_COMMANDS);
    } else {
      // Error codes: mqtt.state()  (-4..5). -2 = network, 4 = login/password, 5 = not authorized.
      Serial.printf("ошибка, state=%d, повтор через 3 c\n", mqtt.state());
      delay(3000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_RED2, OUTPUT);
  dht.begin();

  // 1) Wi-Fi
  WiFi.begin(SSID, PASS);
  Serial.print("[WiFi] Подключение");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.printf("\n[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());

  // 2) MQTT: broker address + callback for incoming messages
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setCallback(onMessage);
}

void loop() {
  // Keep the MQTT connection alive and process incoming packets.
  if (!mqtt.connected()) reconnect();
  mqtt.loop();   // REQUIRED: without calling loop() subscriptions don't work

  // PUBLISH every PUBLISH_INTERVAL ms (without a blocking delay).
  unsigned long now = millis();
  if (now - lastPublish >= PUBLISH_INTERVAL) {
    lastPublish = now;

    float temperature = dht.readTemperature();     // °C
    long  uptime      = millis() / 1000;            // seconds since power-on

    if (isnan(temperature)) {
      Serial.println("[DHT] Ошибка чтения температуры, пропуск публикации");
    } else {
      // Message body is a bare number (exactly what the current .NET consumer parses).
      char buf[16];

      dtostrf(temperature, 0, 1, buf);              // "25.0"
      // publish: QoS 0 (PubSubClient can only publish at QoS 0),
      // retained = false. Returns true if the packet went into the socket.
      mqtt.publish(TOPIC_TEMPERATURE, buf);
      Serial.printf("[MQTT] -> %s = %s\n", TOPIC_TEMPERATURE, buf);

      snprintf(buf, sizeof(buf), "%ld", uptime);    // "3600"
      mqtt.publish(TOPIC_UPTIME, buf);
      Serial.printf("[MQTT] -> %s = %s\n", TOPIC_UPTIME, buf);
    }
  }
}
