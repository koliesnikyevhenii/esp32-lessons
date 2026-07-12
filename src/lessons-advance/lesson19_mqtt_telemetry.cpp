// ============================================================================
//  Этап 2. IoT Foundation — урок 19: MQTT-телеметрия
// ----------------------------------------------------------------------------
//  Цель: связать ESP32 и .NET-бэкенд через MQTT.
//
//      ESP32 --MQTT(1883)--> RabbitMQ --AMQP--> .NET Worker --> PostgreSQL
//
//  В этом уроке разбираем 4 базовых понятия MQTT:
//    • publish   — публикуем телеметрию (temperature, uptime);
//    • subscribe — подписываемся на топик команд и реагируем светодиодом;
//    • topics    — иерархические имена sensors/<device>/<metric> + wildcard '#';
//    • QoS       — уровни доставки 0/1 и ограничения библиотеки PubSubClient.
//
//  Библиотека: knolleary/PubSubClient (см. platformio.ini, env lesson19_mqtt).
// ============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include "pins.h"

// ---- Wi-Fi ----------------------------------------------------------------
const char* SSID = "Vektor_04";
const char* PASS = "uteam2020";

// ---- MQTT-брокер (RabbitMQ + mqtt-плагин) ---------------------------------
// ВАЖНО: укажи LAN-IP компьютера, на котором поднят docker compose (не localhost!
// ESP32 — отдельное устройство в сети). Порт 1883 проброшен в docker-compose.yml.
const char* MQTT_BROKER = "192.168.0.100";   // <-- заменить на IP своего ПК
const uint16_t MQTT_PORT = 1883;

// Пользователь RabbitMQ. Учётка guest работает ТОЛЬКО с localhost, поэтому
// для ESP32 в сети создай отдельного пользователя (Admin -> Add user), например
// esp / esp-pass, и дай ему права на vhost '/'. См. README телеметрии.
const char* MQTT_USER = "esp";
const char* MQTT_PASS = "esp-pass";

// Идентификатор устройства. Он же <device> в топике sensors/<device>/<metric>.
const char* DEVICE_ID  = "esp32";
const char* CLIENT_ID  = "esp32-lesson19";

// ---- Топики ----------------------------------------------------------------
// Publish: по одному числу на метрику. MQTT-плагин RabbitMQ превращает
// слэши в точки: sensors/esp32/temperature  ->  routing key sensors.esp32.temperature,
// а .NET-консьюмер (ParseReading) разбирает его как sensors.<device>.<metric>.
const char* TOPIC_TEMPERATURE = "sensors/esp32/temperature";
const char* TOPIC_UPTIME      = "sensors/esp32/uptime";

// Subscribe: топик команд с wildcard '#' (все подтопики commands/esp32/...).
// Пример команды: publish в commands/esp32/led  с телом "on" / "off".
const char* TOPIC_COMMANDS = "commands/esp32/#";

// ---- Датчик ----------------------------------------------------------------
#define DHTPIN  33
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// ---- Объекты сети ----------------------------------------------------------
WiFiClient   net;
PubSubClient mqtt(net);

unsigned long lastPublish = 0;
const unsigned long PUBLISH_INTERVAL = 5000;   // публикуем раз в 5 секунд

// ---------------------------------------------------------------------------
//  SUBSCRIBE: колбэк вызывается при приходе сообщения в подписанный топик.
//  payload — НЕ строка с нулём на конце, поэтому используем length.
// ---------------------------------------------------------------------------
void onMessage(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];

  Serial.printf("[MQTT] <- топик '%s', тело '%s'\n", topic, msg.c_str());

  // topics: реагируем на commands/esp32/led. Тело "on"/"off" зажигает светодиод.
  if (String(topic).endsWith("/led")) {
    if (msg == "on")  digitalWrite(LED_RED2, HIGH);
    if (msg == "off") digitalWrite(LED_RED2, LOW);
  }
}

// ---------------------------------------------------------------------------
//  Переподключение к брокеру. MQTT-соединение может рваться — держим его
//  живым в loop(). После connect обязательно заново оформляем подписки.
// ---------------------------------------------------------------------------
void reconnect() {
  while (!mqtt.connected()) {
    Serial.print("[MQTT] Подключение к брокеру... ");
    if (mqtt.connect(CLIENT_ID, MQTT_USER, MQTT_PASS)) {
      Serial.println("OK");

      // QoS при подписке: PubSubClient поддерживает 0 и 1.
      //   QoS 0 — "fire and forget", без подтверждения (может потеряться);
      //   QoS 1 — "at least once", брокер шлёт, пока не получит PUBACK
      //           (возможны дубли). Просим QoS 1 для команд — они важнее.
      mqtt.subscribe(TOPIC_COMMANDS, 1);
      Serial.printf("[MQTT] Подписка на '%s' (QoS 1)\n", TOPIC_COMMANDS);
    } else {
      // Коды ошибок: mqtt.state()  (-4..5). -2 = сеть, 4 = логин/пароль, 5 = не авторизован.
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

  // 2) MQTT: адрес брокера + колбэк для входящих сообщений
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setCallback(onMessage);
}

void loop() {
  // Держим MQTT-соединение живым и обрабатываем входящие пакеты.
  if (!mqtt.connected()) reconnect();
  mqtt.loop();   // ОБЯЗАТЕЛЬНО: без вызова loop() подписки не работают

  // PUBLISH раз в PUBLISH_INTERVAL мс (без блокирующего delay).
  unsigned long now = millis();
  if (now - lastPublish >= PUBLISH_INTERVAL) {
    lastPublish = now;

    float temperature = dht.readTemperature();     // °C
    long  uptime      = millis() / 1000;            // секунды с включения

    if (isnan(temperature)) {
      Serial.println("[DHT] Ошибка чтения температуры, пропуск публикации");
    } else {
      // Тело сообщения — голое число (именно это парсит текущий .NET-консьюмер).
      char buf[16];

      dtostrf(temperature, 0, 1, buf);              // "25.0"
      // publish: QoS 0 (PubSubClient умеет публиковать только на QoS 0),
      // retained = false. Возвращает true, если пакет ушёл в сокет.
      mqtt.publish(TOPIC_TEMPERATURE, buf);
      Serial.printf("[MQTT] -> %s = %s\n", TOPIC_TEMPERATURE, buf);

      snprintf(buf, sizeof(buf), "%ld", uptime);    // "3600"
      mqtt.publish(TOPIC_UPTIME, buf);
      Serial.printf("[MQTT] -> %s = %s\n", TOPIC_UPTIME, buf);
    }
  }
}
