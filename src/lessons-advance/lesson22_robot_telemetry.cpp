// ============================================================================
//  Этап 3. Робот — урок 22: телеметрия + управление в ОДНОЙ прошивке
// ----------------------------------------------------------------------------
//  Цель: объединить урок 20 (угол наклона с MPU6050) и урок 21 (моторы через
//  TB6612FNG). Теперь одна прошивка одновременно:
//    • ПУБЛИКУЕТ угол наклона   — sensors/esp32/pitch|roll|yaw   (как в уроке 20);
//    • ПОДПИСАНА на команды     — commands/esp32/drive           (как в уроке 21).
//
//   Браузер --HTTP--> ASP.NET --AMQP--> RabbitMQ --MQTT--> ESP32 --> моторы
//   Браузер <--SignalR-- ASP.NET <--AMQP-- RabbitMQ <--MQTT-- ESP32 <-- MPU6050
//
//  То есть в дашборде мы РУЛИМ машинкой и в ту же секунду ВИДИМ, как она
//  наклоняется: два направления обмена живут на одном MQTT-соединении.
//
//  С урока 26 у этой прошивки появился ВТОРОЙ пульт: ESP32-CAM публикует нажатия
//  кнопок в тот же топик commands/esp32/drive, а метрику guard (ниже) слушает,
//  чтобы показать блокировку на FPV-странице. Менять здесь ничего не пришлось —
//  робот слушает ТОПИК, а не конкретного клиента; рулить можно из дашборда и с
//  камеры одновременно. Вторая плата с камерой стоит на машинке рядом с этой.
//
//  Что нового по сравнению с уроками 20 и 21 (это не просто «склейка»):
//    • один PubSubClient обслуживает и publish, и subscribe — важно не блокировать
//      loop() (никаких delay), иначе команды начнут опаздывать;
//    • mpu.update() вызывается КАЖДУЮ итерацию loop(), а publish — раз в 200 мс:
//      tockn интегрирует гироскоп по времени внутри update(), поэтому чем чаще
//      его дёргаешь, тем меньше врёт yaw (в уроке 20 update() был раз в 200 мс);
//    • ЗАЩИТА ПО НАКЛОНУ (tilt guard) — обратная связь «датчик -> моторы»: если
//      машинка задралась круче TILT_LIMIT (наехала на препятствие, встала на
//      попа), прошивка сама глушит моторы и игнорирует команды движения, пока
//      наклон не вернётся ниже TILT_RELEASE (гистерезис, чтобы не дребезжало).
//      Одного сэмпла за порогом ей НЕ достаточно: пусковой рывок моторов сам
//      себя глушил, потому что угол считается из вектора ускорения и на рывке
//      врёт на десятки градусов — подробности в updateTiltGuard();
//    • состояние этой защиты уезжает в бэкенд как ОБЫЧНАЯ метрика guard (0/1),
//      но публикуется ТОЛЬКО ПРИ ИЗМЕНЕНИИ — «publish on change» вместо
//      «publish by timer»: событий мало, незачем каждые 200 мс писать в БД.
//
//  Пины: MPU сидит на I2C (32/17), моторы — на 13/14/25 и 26/27/33. Группы не
//  пересекаются (см. include/pins.h) — это обязательное условие урока 22, ведь
//  здесь, в отличие от 20 и 21, оба устройства подключены ОДНОВРЕМЕННО.
//
//  Библиотеки: knolleary/PubSubClient + tockn/MPU6050_tockn
//  (см. platformio.ini, env lesson22_robot_telemetry).
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

// ---- MQTT-брокер (RabbitMQ + mqtt-плагин) ---------------------------------
// LAN-IP компьютера с docker compose (НЕ localhost: ESP32 — отдельное устройство).
const char* MQTT_BROKER = "192.168.0.7";   // <-- заменить на IP своего ПК
const uint16_t MQTT_PORT = 1883;

// guest работает только с localhost — для ESP32 нужен отдельный пользователь.
const char* MQTT_USER = "esp";
const char* MQTT_PASS = "esp-pass";

const char* DEVICE_ID = "esp32";
const char* CLIENT_ID = "esp32-lesson22";

// ---- Топики ----------------------------------------------------------------
// Контракты те же, что в уроках 20/21 — бэкенд менять НЕ надо:
//   телеметрия  sensors/<device>/<metric>   тело = голое число ("-3.4")
//   команды     commands/<device>/drive     тело = слово (forward|back|left|right|stop)
const char* TOPIC_PITCH = "sensors/esp32/pitch";
const char* TOPIC_ROLL  = "sensors/esp32/roll";
const char* TOPIC_YAW   = "sensors/esp32/yaw";
const char* TOPIC_GUARD = "sensors/esp32/guard";   // 0 = ехать можно, 1 = блок по наклону
const char* TOPIC_DRIVE = "commands/esp32/drive";

// ---- Датчик ----------------------------------------------------------------
// tockn не проверяет WHO_AM_I, поэтому работает и с MPU6050, и с MPU6500 (0x70).
MPU6050 mpu(Wire);

// ---- Моторы (TB6612FNG) ----------------------------------------------------
// A = левый мотор, B = правый. STBY драйвера — на 3V3 (всегда включён).
const int SPEED = 200;                       // 0..255 — скорость по ШИМ
const unsigned long FAILSAFE_MS = 700;       // нет команд дольше -> стоп

// ---- Защита по наклону -----------------------------------------------------
// Гистерезис: блокируемся на TILT_LIMIT, разблокируемся только ниже TILT_RELEASE.
// Один порог на оба события давал бы дребезг на границе (вкл/выкл/вкл...).
const float TILT_LIMIT   = 45.0f;            // градусов — круче этого стоп
const float TILT_RELEASE = 35.0f;            // градусов — ниже этого снова можно

// Два фильтра против ЛОЖНЫХ срабатываний. Почему они вообще нужны: без них
// защита срабатывала на КАЖДОМ старте моторов: в логе стояло «pitch = 1.8»,
// сразу за командой forward — «наклон 47 град >= 45 — СТОП», через две сотни
// миллисекунд отпуск. Машинка дёргалась и стояла. Замер по шести событиям:
// самое длинное ложное жило 228 мс, самое короткое — 47 мс.
const unsigned long TILT_DEBOUNCE_MS = 300;  // столько наклон должен ДЕРЖАТЬСЯ, чтобы поверить
const float ACCEL_TRUST_BAND = 0.25f;        // |a| вне 1.0 +- этого (g) — сэмплу не верим

// ---- Тайминги --------------------------------------------------------------
const unsigned long PUBLISH_INTERVAL = 200;  // 200 мс = 5 раз в секунду

// ---- Состояние -------------------------------------------------------------
unsigned long lastCommandMs = 0;   // когда последний раз слышали команду
unsigned long lastPublish   = 0;   // когда последний раз публиковали углы
bool moving      = false;          // моторы сейчас крутятся
bool tiltBlocked = false;          // защита по наклону сработала
unsigned long overLimitSince = 0;  // когда наклон ушёл за порог (0 = сейчас в норме)
int  guardSent   = -1;             // что про guard уже знает бэкенд (-1 = ещё ничего)
float pitch = 0, roll = 0, yaw = 0;

// ---- Объекты сети ----------------------------------------------------------
WiFiClient   net;
PubSubClient mqtt(net);

// ---------------------------------------------------------------------------
//  МОТОРЫ (как в уроке 21)
//  Один мотор: dir>0 — вперёд, dir<0 — назад, dir==0 — стоп.
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

// Танковое управление: left/right — разворот на месте (моторы в разные стороны).
void applyCommand(const String& cmd) {
  // "stop" исполняем ВСЕГДА, даже при активной защите — стоп безопасен по определению.
  if (cmd == "stop") { driveStop(); return; }

  // Защита по наклону: команды движения игнорируем, пока машинка задрана.
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
//  ТЕЛЕМЕТРИЯ
//  Публикуем одно число в топик. Тело — голая строка, именно это парсит
//  .NET-консьюмер (TelemetryConsumer.ParseReading).
// ---------------------------------------------------------------------------
void publishNumber(const char* topic, float value) {
  char buf[16];
  dtostrf(value, 0, 1, buf);          // например "-3.4"
  mqtt.publish(topic, buf);
  Serial.printf("[MQTT] -> %s = %s\n", topic, buf);
}

// guard шлём ТОЛЬКО когда состояние изменилось (publish on change): это событие,
// а не поток измерений, — в БД незачем лить 5 одинаковых строк в секунду.
void publishGuardIfChanged() {
  int state = tiltBlocked ? 1 : 0;
  if (state == guardSent) return;
  guardSent = state;
  mqtt.publish(TOPIC_GUARD, state ? "1" : "0");
  Serial.printf("[MQTT] -> %s = %d\n", TOPIC_GUARD, state);
}

// ---------------------------------------------------------------------------
//  Обратная связь «датчик -> моторы»: та самая причина объединять 20 и 21.
//  Считаем наклон по большей из двух осей и сравниваем с порогами.
//
//  ВАЖНОЕ ПРО ЛОЖНЫЕ СРАБАТЫВАНИЯ. pitch/roll выше получены из ВЕКТОРА
//  УСКОРЕНИЯ (atan2 по ax/ay/az), а акселерометр меряет гравитацию ПЛЮС
//  ускорение самого робота. Пока машинка стоит, это одно и то же и угол
//  честный; на пусковом рывке моторов появляется горизонтальная составляющая,
//  и atan2 выдаёт 45-48 градусов на ровном полу. Наивная защита тут же глушила
//  моторы — то есть рывок глушил сам себя, и робот дёргался вместо езды.
//
//  Поэтому одному сэмплу за порогом больше не верим, и фильтров ДВА:
//    1) ДОВЕРИЕ К СЭМПЛУ — в покое |a| = 1 g; если модуль уехал больше чем на
//       ACCEL_TRUST_BAND, робот разгоняется/тормозит/подпрыгивает, и угол из
//       такого сэмпла — мусор. Такой сэмпл пропускаем ЦЕЛИКОМ: он не взводит
//       защиту, но и не снимает её и не сбрасывает отсчёт;
//    2) ДЕБАУНС — взводимся, только если наклон держится за TILT_LIMIT
//       непрерывно TILT_DEBOUNCE_MS. Настоящий переворот держит наклон
//       секундами; замеренные ложные жили 47-228 мс, то есть все короче 300.
//  По отдельности каждый фильтр слабее: |a| без дебаунса не спасёт от долгой
//  тряски, а дебаунс без |a| просто дождётся конца рывка вместо того, чтобы
//  не считать его вовсе.
// ---------------------------------------------------------------------------
void updateTiltGuard(float accelG) {
  // Фильтр 1: сэмплу с искажённым модулем ускорения веры нет.
  if (fabsf(accelG - 1.0f) > ACCEL_TRUST_BAND) return;

  float tilt = max(fabsf(pitch), fabsf(roll));
  unsigned long now = millis();

  if (!tiltBlocked) {
    if (tilt >= TILT_LIMIT) {
      // Фильтр 2: засекаем первый достоверный сэмпл за порогом и ждём.
      if (overLimitSince == 0) overLimitSince = now;
      if (now - overLimitSince >= TILT_DEBOUNCE_MS) {
        tiltBlocked = true;
        overLimitSince = 0;
        Serial.printf("[GUARD] наклон %.0f град >= %.0f держится %lu мс — СТОП\n",
                      tilt, TILT_LIMIT, TILT_DEBOUNCE_MS);
        driveStop();                 // глушим сразу, не дожидаясь команды
      }
    } else {
      overLimitSince = 0;            // вернулся в норму — отсчёт заново
    }
  } else if (tilt <= TILT_RELEASE) {
    // Снятие — по первому же ДОСТОВЕРНОМУ сэмплу ниже порога, без дебаунса:
    // держать робота заблокированным дольше нужного хуже, чем отпустить
    // раньше, а от дребезга на границе уже спасает гистерезис TILT_RELEASE.
    tiltBlocked = false;
    Serial.printf("[GUARD] наклон %.0f град <= %.0f — снова можно ехать\n", tilt, TILT_RELEASE);
    // Моторы НЕ трогаем: браузер, пока кнопка зажата, сам пришлёт команду через ~300 мс.
  }
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
      guardSent = -1;   // после реконнекта заново сообщим бэкенду состояние защиты
    } else {
      // state(): -2 = сеть, 4 = логин/пароль, 5 = не авторизован.
      Serial.printf("ошибка, state=%d, повтор через 3 c\n", mqtt.state());
      // Моторы на время реконнекта должны стоять: команд-то всё равно нет.
      driveStop();
      delay(3000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  // 1) МОТОРЫ — первым делом: пины в OUTPUT и сразу стоп. Важно, чтобы робот
  //    не дёрнулся, пока мы три секунды калибруем гироскоп и лезем в Wi-Fi.
  int pins[] = { MOTOR_AIN1, MOTOR_AIN2, MOTOR_PWMA, MOTOR_BIN1, MOTOR_BIN2, MOTOR_PWMB };
  for (int p : pins) pinMode(p, OUTPUT);
  driveStop();

  // 2) I2C + датчик. Wire.begin(SDA, SCL) — задаём пины явно.
  Wire.begin(MPU_SDA, MPU_SCL);
  Wire.setClock(400000);        // 400 кГц: успеваем опрашивать датчик в каждом loop()
  Serial.print("[MPU] Инициализация... ");
  mpu.begin();

  // Калибровка нуля гироскопа: робот должен стоять НЕПОДВИЖНО и РОВНО ~3 секунды.
  // Моторы уже заглушены (шаг 1), так что вибрации нет — иначе yaw сразу уплывёт.
  Serial.println("калибровка гироскопа, не трогай робота ~3 c...");
  mpu.calcGyroOffsets(true);
  Serial.println("[MPU] OK");

  // 3) Wi-Fi
  WiFi.begin(SSID, PASS);
  Serial.print("[WiFi] Подключение");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.printf("\n[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());

  // 4) MQTT: адрес брокера + колбэк (в уроке 20 колбэк был не нужен, здесь — обязателен).
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setCallback(onMessage);

  lastCommandMs = millis();
}

void loop() {
  // 1) Держим соединение живым. ОБЯЗАТЕЛЬНО каждый проход: без mqtt.loop()
  //    не работают ни подписки, ни keep-alive.
  if (!mqtt.connected()) reconnect();
  mqtt.loop();

  // 2) Датчик опрашиваем на каждой итерации — так точнее интеграл гироскопа (yaw).
  mpu.update();
  float ax = mpu.getAccX();
  float ay = mpu.getAccY();
  float az = mpu.getAccZ();

  // Угол наклона из вектора силы тяжести (в градусах), как в уроке 20:
  //   roll  — крен «вбок» (вокруг X), pitch — тангаж «вперёд-назад» (вокруг Y).
  roll  = atan2f(ay, az) * 180.0f / PI;
  pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI;
  // Курс: интеграл гироскопа по Z. Опорного «севера» нет -> со временем ДРЕЙФУЕТ,
  // а на ходу дрейфует заметно быстрее — моторы трясут датчик.
  yaw = mpu.getAngleZ();

  // 3) Обратная связь: наклон может сам заглушить моторы. Передаём модуль
  //    вектора ускорения — по нему защита решает, верить ли углу вообще.
  updateTiltGuard(sqrtf(ax * ax + ay * ay + az * az));

  // 4) Failsafe из урока 21: едем, но команд давно нет -> стоп.
  //    Закрыли вкладку / пропал Wi-Fi — робот не «убегает».
  if (moving && millis() - lastCommandMs > FAILSAFE_MS) {
    Serial.println("[DRIVE] failsafe: команд нет, стоп");
    driveStop();
  }

  // 5) Телеметрия раз в PUBLISH_INTERVAL мс (без блокирующего delay).
  unsigned long now = millis();
  if (now - lastPublish >= PUBLISH_INTERVAL) {
    lastPublish = now;
    publishNumber(TOPIC_PITCH, pitch);
    publishNumber(TOPIC_ROLL,  roll);
    publishNumber(TOPIC_YAW,   yaw);
    publishGuardIfChanged();
  }
}
