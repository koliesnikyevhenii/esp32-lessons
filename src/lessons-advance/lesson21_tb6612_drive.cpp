// ============================================================================
//  Этап 3. Робот — урок 21: управление моторами через TB6612FNG по MQTT
// ----------------------------------------------------------------------------
//  Цель: получать команды движения из браузера и крутить два мотора — вперёд,
//  назад, влево, вправо, стоп. Без автопилота, чистое ручное управление.
//
//   Браузер --HTTP--> ASP.NET --AMQP--> RabbitMQ --MQTT(1883)--> ESP32 --> TB6612FNG --> моторы
//
//  Это ОБРАТНОЕ направление к урокам 19–20: там ESP32 публиковал телеметрию,
//  здесь ESP32 ПОДПИСЫВАЕТСЯ на команды. .NET публикует routing key
//  commands.esp32.drive, а MQTT-плагин RabbitMQ отдаёт его как топик
//  commands/esp32/drive (тело — одно слово: forward|back|left|right|stop).
//
//  TB6612FNG — драйвер двух моторов. На каждый мотор: два входа направления
//  (xIN1/xIN2) и вход скорости PWM (PWMx):
//     IN1=1, IN2=0 -> вперёд;  IN1=0, IN2=1 -> назад;  IN1=IN2 -> стоп.
//  Вход STBY включает драйвер — в этой схеме он заведён на 3V3 (всегда ON).
//
//  БЕЗОПАСНОСТЬ (failsafe): если команды перестали приходить (закрыли вкладку,
//  пропал Wi-Fi) дольше FAILSAFE_MS — сами глушим моторы, чтобы робот не «убежал».
//  Поэтому браузер, пока кнопка зажата, повторяет команду каждые ~300 мс.
//
//  Библиотека: knolleary/PubSubClient (см. platformio.ini, env lesson21_motors).
// ============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "pins.h"

// ---- Wi-Fi ----------------------------------------------------------------
const char* SSID = "Koleso";
const char* PASS = "12345zxc";

// ---- MQTT-брокер (RabbitMQ + mqtt-плагин) ---------------------------------
const char* MQTT_BROKER = "192.168.1.164";   // <-- заменить на IP своего ПК
const uint16_t MQTT_PORT = 1883;
const char* MQTT_USER = "esp";
const char* MQTT_PASS = "esp-pass";

const char* DEVICE_ID = "esp32";
const char* CLIENT_ID = "esp32-lesson21";

// Топик команд движения. .NET -> commands.esp32.drive -> сюда.
const char* TOPIC_DRIVE = "commands/esp32/drive";

// ---- Моторы (TB6612FNG) ----------------------------------------------------
// A = левый мотор, B = правый. STBY драйвера — на 3V3.
const int SPEED = 200;                    // 0..255 — скорость по ШИМ (analogWrite)
const unsigned long FAILSAFE_MS = 700;    // нет команд дольше -> стоп

unsigned long lastCommandMs = 0;
bool moving = false;

// ---- Объекты сети ----------------------------------------------------------
WiFiClient   net;
PubSubClient mqtt(net);

// Один мотор: dir>0 — вперёд, dir<0 — назад, dir==0 — стоп.
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

// Танковое управление: left/right — разворот на месте (моторы в разные стороны).
// Если мотор крутится не туда — поменяй местами его провода или знаки dir.
void applyCommand(const String& cmd) {
  int left, right;
  if      (cmd == "forward") { left = +1; right = +1; }
  else if (cmd == "back")    { left = -1; right = -1; }
  else if (cmd == "left")    { left = -1; right = +1; }
  else if (cmd == "right")   { left = +1; right = -1; }
  else if (cmd == "stop")    { driveStop(); return; }
  else { Serial.printf("[DRIVE] неизвестная команда '%s'\n", cmd.c_str()); return; }

  motor(MOTOR_AIN1, MOTOR_AIN2, MOTOR_PWMA, left);
  motor(MOTOR_BIN1, MOTOR_BIN2, MOTOR_PWMB, right);
  moving = true;
}

// ---------------------------------------------------------------------------
//  SUBSCRIBE: колбэк на входящую команду. payload без нуля на конце -> length.
// ---------------------------------------------------------------------------
void onMessage(char* topic, byte* payload, unsigned int length) {
  String cmd;
  for (unsigned int i = 0; i < length; i++) cmd += (char)payload[i];
  cmd.trim();

  Serial.printf("[MQTT] <- %s = %s\n", topic, cmd.c_str());

  lastCommandMs = millis();   // видели команду -> сбрасываем failsafe-таймер
  applyCommand(cmd);
}

void reconnect() {
  while (!mqtt.connected()) {
    Serial.print("[MQTT] Подключение к брокеру... ");
    if (mqtt.connect(CLIENT_ID, MQTT_USER, MQTT_PASS)) {
      Serial.println("OK");
      // QoS 1 — команды важнее телеметрии, лучше «доставить хотя бы раз».
      mqtt.subscribe(TOPIC_DRIVE, 1);
      Serial.printf("[MQTT] Подписка на '%s' (QoS 1)\n", TOPIC_DRIVE);
    } else {
      Serial.printf("ошибка, state=%d, повтор через 3 c\n", mqtt.state());
      delay(3000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Пины моторов — выходы; сразу глушим (важно: при старте робот стоит).
  int pins[] = { MOTOR_AIN1, MOTOR_AIN2, MOTOR_PWMA, MOTOR_BIN1, MOTOR_BIN2, MOTOR_PWMB };
  for (int p : pins) pinMode(p, OUTPUT);
  driveStop();

  WiFi.begin(SSID, PASS);
  Serial.print("[WiFi] Подключение");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.printf("\n[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());

  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setCallback(onMessage);
}

void loop() {
  if (!mqtt.connected()) reconnect();
  mqtt.loop();   // ОБЯЗАТЕЛЬНО: без loop() подписки не работают

  // Failsafe: едем, но команд давно нет -> стоп.
  if (moving && millis() - lastCommandMs > FAILSAFE_MS) {
    Serial.println("[DRIVE] failsafe: команд нет, стоп");
    driveStop();
  }
}
