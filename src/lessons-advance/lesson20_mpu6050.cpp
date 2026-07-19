// ============================================================================
//  Этап 2. IoT Foundation — урок 20: MPU6050 -> угол наклона (pitch/roll)
// ----------------------------------------------------------------------------
//  Цель: снять с гироскопа-акселерометра MPU6050 ускорения по осям X/Y/Z,
//  вычислить угол наклона (pitch/roll) и отправить его на .NET-бэкенд по MQTT.
//
//      ESP32 --MQTT(1883)--> RabbitMQ --AMQP--> .NET Worker --> PostgreSQL
//                                                    |
//                                              SignalR -> React-дашборд (pitch/roll/yaw)
//
//  Продолжение урока 19. MQTT-часть та же (publish бодрым числом), меняется
//  только источник данных: вместо DHT11 — датчик MPU6050 по шине I2C.
//
//  Что нового по сравнению с уроком 19:
//    • I2C — читаем датчик по двум проводам (SDA/SCL), адрес 0x68;
//    • акселерометр — ускорения ax/ay/az (в m/s^2), из них — угол наклона;
//    • pitch/roll — угол по формулам atan2 (наклон «вперёд-назад» и «вбок»);
//    • публикуем чаще (5 раз в секунду) — дашборд рисует «почти в реальном времени».
//
//  Библиотека: adafruit/Adafruit MPU6050 (см. platformio.ini, env lesson20_mpu6050).
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

// ---- MQTT-брокер (RabbitMQ + mqtt-плагин) ---------------------------------
// LAN-IP компьютера с docker compose (НЕ localhost: ESP32 — отдельное устройство).
const char* MQTT_BROKER = "192.168.1.164";   // <-- заменить на IP своего ПК
const uint16_t MQTT_PORT = 1883;

// Учётка RabbitMQ. guest работает только с localhost — для ESP32 создай
// отдельного пользователя esp / esp-pass (см. урок 19 / README телеметрии).
const char* MQTT_USER = "esp";
const char* MQTT_PASS = "esp-pass";

// <device> в топике sensors/<device>/<metric>.
const char* DEVICE_ID = "esp32";
const char* CLIENT_ID = "esp32-lesson20";

// ---- Топики ----------------------------------------------------------------
// По одному числу на метрику. MQTT-плагин RabbitMQ превращает слэши в точки:
//   sensors/esp32/pitch -> routing key sensors.esp32.pitch,
// а .NET-консьюмер (ParseReading) читает его как sensors.<device>.<metric>.
// pitch, roll и yaw — это просто «метрики», бэкенд их менять не надо.
const char* TOPIC_PITCH = "sensors/esp32/pitch";
const char* TOPIC_ROLL  = "sensors/esp32/roll";
const char* TOPIC_YAW   = "sensors/esp32/yaw";

// ---- Датчик ----------------------------------------------------------------
// tockn-библиотека не проверяет WHO_AM_I, поэтому работает и с MPU6050, и с
// MPU6500 (нашим чипом, WHO_AM_I=0x70). Конструктору передаём шину I2C.
MPU6050 mpu(Wire);

// ---- Объекты сети ----------------------------------------------------------
WiFiClient   net;
PubSubClient mqtt(net);

unsigned long lastPublish = 0;
const unsigned long PUBLISH_INTERVAL = 200;   // 200 мс = 5 раз в секунду

// ---------------------------------------------------------------------------
//  Переподключение к брокеру. Здесь мы только публикуем, подписок нет,
//  поэтому после connect ничего дополнительно оформлять не нужно.
// ---------------------------------------------------------------------------
void reconnect() {
  while (!mqtt.connected()) {
    Serial.print("[MQTT] Подключение к брокеру... ");
    if (mqtt.connect(CLIENT_ID, MQTT_USER, MQTT_PASS)) {
      Serial.println("OK");
    } else {
      // state(): -2 = сеть, 4 = логин/пароль, 5 = не авторизован.
      Serial.printf("ошибка, state=%d, повтор через 3 c\n", mqtt.state());
      delay(3000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  // 1) I2C + датчик. Wire.begin(SDA, SCL) — задаём пины явно.
  Wire.begin(MPU_SDA, MPU_SCL);
  Serial.print("[MPU] Инициализация... ");
  mpu.begin();   // tockn::begin() ничего не возвращает и WHO_AM_I не проверяет

  // Калибровка нуля гироскопа: датчик должен лежать НЕПОДВИЖНО ~3 секунды.
  // Библиотека усредняет дрейф и вычитает его дальше (иначе углы «уплывают»).
  Serial.println("калибровка гироскопа, не трогай датчик...");
  mpu.calcGyroOffsets(true);   // true = печатать прогресс в Serial
  Serial.println("[MPU] OK");

  // 2) Wi-Fi
  WiFi.begin(SSID, PASS);
  Serial.print("[WiFi] Подключение");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.printf("\n[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());

  // 3) MQTT: только адрес брокера (колбэк не нужен — мы не подписываемся).
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
}

void loop() {
  // Держим MQTT-соединение живым.
  if (!mqtt.connected()) reconnect();
  mqtt.loop();

  // PUBLISH раз в PUBLISH_INTERVAL мс (без блокирующего delay).
  unsigned long now = millis();
  if (now - lastPublish >= PUBLISH_INTERVAL) {
    lastPublish = now;

    // Обновляем показания датчика (обязательно перед каждым чтением).
    mpu.update();

    // Ускорения по X/Y/Z (tockn отдаёт их в g — для atan2 единицы не важны).
    float ax = mpu.getAccX();
    float ay = mpu.getAccY();
    float az = mpu.getAccZ();

    // Угол наклона из вектора силы тяжести (в градусах):
    //   roll  — крен «вбок»    (поворот вокруг оси X);
    //   pitch — тангаж «вперёд-назад» (поворот вокруг оси Y).
    // atan2 сам разбирается со знаками и квадрантами.
    float roll  = atan2f(ay, az) * 180.0f / PI;
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI;

    // Курс (рыскание, поворот вокруг вертикали) — акселерометром не измеряется:
    // сила тяжести при таком повороте не меняется. Берём интеграл гироскопа по Z,
    // который tockn ведёт сама (getAngleZ, calcGyroOffsets вычел дрейф в setup).
    // Опорного «севера» нет, поэтому со временем значение ДРЕЙФУЕТ — это нормально
    // для гиро-only без магнитометра.
    float yaw = mpu.getAngleZ();

    // Тело сообщения — голое число (именно это парсит .NET-консьюмер).
    char buf[16];

    dtostrf(pitch, 0, 1, buf);            // например "-3.4"
    mqtt.publish(TOPIC_PITCH, buf);
    Serial.printf("[MQTT] -> %s = %s\n", TOPIC_PITCH, buf);

    dtostrf(roll, 0, 1, buf);             // например "12.1"
    mqtt.publish(TOPIC_ROLL, buf);
    Serial.printf("[MQTT] -> %s = %s\n", TOPIC_ROLL, buf);

    dtostrf(yaw, 0, 1, buf);              // например "-45.0" (дрейфует со временем)
    mqtt.publish(TOPIC_YAW, buf);
    Serial.printf("[MQTT] -> %s = %s\n", TOPIC_YAW, buf);
  }
}
