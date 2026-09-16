// ============================================================================
//  Этап 7. Автопилот v1 — урок 29: робот едет САМ
// ----------------------------------------------------------------------------
//  Цель: соединить урок 22 (телеметрия + управление по MQTT) и урок 28
//  (сканирующий сонар) в одну прошивку, где робот умеет ехать без человека:
//
//      если впереди чисто        -> ехать вперёд
//      если стена ближе порога   -> остановиться, осмотреться, повернуть
//      если ответа нет вообще    -> считать это ОПАСНОСТЬЮ, а не свободной дорогой
//
//  Что нового по сравнению с 22 и 28 — три вещи, и все три принципиальные:
//
//  1) СКАНИРОВАНИЕ БОЛЬШЕ НЕ БЛОКИРУЕТ loop(). В уроке 28 sweep() честно стоял
//     ~2 секунды в delay() — там больше ничего и не происходило. Здесь за эти
//     2 секунды успели бы: отвалиться keep-alive MQTT, сработать failsafe моторов
//     (700 мс) и «проспаться» защита по наклону. Поэтому проход развёрнут в
//     АВТОМАТ СОСТОЯНИЙ: за один проход loop() делается ровно один маленький шаг
//     (повернуть вал / сделать один «пик»), а между шагами робот продолжает жить.
//
//  2) ДВА РЕЖИМА И КТО ГЛАВНЫЙ. Появился второй топик команд —
//     commands/esp32/mode с телом auto|manual. Автопилот включается ИЗ БРАУЗЕРА
//     и выключается пятью способами: командой manual, ЛЮБОЙ командой движения
//     (человек взялся за руль — автопилот молча уходит), потерей брокера,
//     тремя подряд неудачными осмотрами и, конечно, кнопкой в дашборде.
//     Контракт commands/esp32/drive НЕ тронут: FPV-страница из урока 26 про
//     автопилот ничего не знает и продолжает работать как была.
//
//  3) ПОВОРОТ ЗАКРЫТ ПО ГИРОСКОПУ. Энкодеров у нас нет, «крутиться 600 мс» —
//     это лотерея (заряд батареи, ковёр, паркет). Зато с урока 20 есть yaw, и
//     поворот идёт до тех пор, пока |yaw - yaw на старте| не наберёт нужные
//     градусы. Знак yaw при этом не важен (см. serviceTurn) — направление задают
//     моторы, а гироскоп только считает, сколько уже накрутили. Таймаут на случай,
//     если колёса буксуют и yaw не растёт.
//
//  Телеметрия к уроку 22 добавляет три метрики (контракт тот же — голое число):
//     sensors/esp32/distance — расстояние по курсу, см; -1 = «эха нет» (НЕ «далеко»!)
//     sensors/esp32/mode     — 0 = ручной, 1 = автопилот (publish on change)
//     sensors/esp32/bearing  — угол, который автопилот выбрал последним осмотром,
//                              в координатах сервопривода: 90 = прямо (on change)
//
//  Пины: MPU (32/17), моторы (13/14/25 и 26/27/33), сонар (12/39), сервопривод (19).
//  Четыре периферии ОДНОВРЕМЕННО — именно ради этого группы в pins.h не пересекаются.
//
//  Библиотеки: knolleary/PubSubClient + tockn/MPU6050_tockn
//  Сборка: pio run -e lesson29_autopilot -t upload && pio device monitor
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
const char* MQTT_BROKER = "192.168.0.7";   // <-- LAN-IP своего ПК, НЕ localhost
const uint16_t MQTT_PORT = 1883;
const char* MQTT_USER = "esp";
const char* MQTT_PASS = "esp-pass";

const char* DEVICE_ID = "esp32";
const char* CLIENT_ID = "esp32-lesson29";

// ---- Топики ----------------------------------------------------------------
const char* TOPIC_PITCH   = "sensors/esp32/pitch";
const char* TOPIC_ROLL    = "sensors/esp32/roll";
const char* TOPIC_YAW     = "sensors/esp32/yaw";
const char* TOPIC_GUARD   = "sensors/esp32/guard";     // 0/1 — защита по наклону (урок 22)
const char* TOPIC_DIST    = "sensors/esp32/distance";  // см по курсу, -1 = нет эха
const char* TOPIC_MODE    = "sensors/esp32/mode";      // 0 ручной / 1 автопилот
const char* TOPIC_BEARING = "sensors/esp32/bearing";   // выбранный угол, 90 = прямо
const char* TOPIC_DRIVE   = "commands/esp32/drive";    // forward|back|left|right|stop
const char* TOPIC_CMDMODE = "commands/esp32/mode";     // auto|manual  <-- НОВОЕ

// ---- Датчик ----------------------------------------------------------------
MPU6050 mpu(Wire);

// ---- Моторы ----------------------------------------------------------------
const int SPEED_MANUAL = 200;                // как в уроке 22
// Автопилот ездит МЕДЛЕННЕЕ человека, и это не вежливость, а арифметика из урока 27:
// сонар даёт ~14 измерений в секунду, то есть между двумя «пиками» робот проезжает
// вслепую. На 200 это заметный кусок тормозного пути, на 160 — вдвое меньше.
const int SPEED_AUTO   = 160;
const unsigned long FAILSAFE_MS = 700;       // ручной режим: нет команд дольше -> стоп

// ---- Защита по наклону (целиком из урока 22) -------------------------------
const float TILT_LIMIT   = 45.0f;
const float TILT_RELEASE = 35.0f;
const unsigned long TILT_DEBOUNCE_MS = 300;
const float ACCEL_TRUST_BAND = 0.25f;

// ---- Сервопривод (из урока 28) ---------------------------------------------
const uint32_t SERVO_FREQ_HZ = 50;
const uint8_t  SERVO_BITS    = 16;
const uint16_t SERVO_MIN_US  = 500;
const uint16_t SERVO_MAX_US  = 2400;

const uint8_t SCAN_MIN_DEG  = 30;
const uint8_t SCAN_MAX_DEG  = 150;
const uint8_t SCAN_STEP_DEG = 15;
const uint8_t SCAN_POINTS   = (SCAN_MAX_DEG - SCAN_MIN_DEG) / SCAN_STEP_DEG + 1;  // 9
const uint8_t SERVO_FORWARD_DEG = 90;        // «прямо по курсу»

// Куда смотрит вал на углах БОЛЬШЕ 90°. Зависит от того, какой стороной надет
// кронштейн: если робот на осмотре уверенно сворачивает в противоположную сторону
// от свободного места — поменяй здесь true на false, и больше ничего.
const bool SERVO_HIGH_IS_LEFT = true;

const uint32_t SETTLE_MS       = 120;        // шаг 15°: ~25 мс хода + успокоение
const uint32_t SETTLE_FAR_MS   = 450;        // переезд через весь сектор (90° -> 30°)

// ---- Сонар (из урока 27) ---------------------------------------------------
const float AIR_TEMP_C   = 20.0f;            // v = 331.4 + 0.606*T
const float MIN_VALID_CM = 2.0f;
const float MAX_VALID_CM = 400.0f;

// В уроке 27 тайм-аут был 30000 мкс (≈5 м) — и pulseIn честно съедал 30 мс.
// Здесь это недопустимо: за 30 мс мы не обслужим ни MQTT, ни failsafe. 15000 мкс
// это ~2.5 м — дальше автопилоту всё равно, «свободно и свободно».
const uint32_t ECHO_TIMEOUT_US = 15000UL;
const uint32_t PING_GAP_MS     = 70;         // 60 по даташиту + запас
const uint8_t  PINGS_PER_POINT = 3;          // медиана из 3 (одиночный выброс не сдвинет)

// ---- Пороги автопилота -----------------------------------------------------
// Тормозим НЕ на 20 см из формулировки этапа, а раньше: пока едем, сонар молчит
// ~70 мс, плюс сама машинка катится по инерции. 30 см — это запас на оба эффекта.
const float STOP_CM  = 30.0f;
// А направление считаем свободным только с 45 см: 31 см «свободы» — это не проезд,
// это следующая остановка через полсекунды.
const float CLEAR_CM = 45.0f;

// Сколько подряд измерений без эха считать препятствием. Одно «нет эха» в комнате —
// обычное дело (стена дальше 2.5 м). Четыре подряд на скорости — это уже 0.3 секунды
// молчания, и по уроку 27 это «не знаю», то есть повод остановиться и осмотреться.
const uint8_t UNKNOWN_LIMIT = 4;

const int16_t  TURN_MIN_DEG   = 25;          // меньше поворачивать бессмысленно
const int16_t  TURN_DEADZONE  = 10;          // выбранный угол почти прямо -> просто ехать
const uint32_t BACK_MS        = 600;         // сколько пятиться, когда всё занято
const uint8_t  BLIND_LIMIT    = 3;           // столько безрезультатных осмотров -> сдаёмся

// ---- Тайминги --------------------------------------------------------------
const unsigned long PUBLISH_INTERVAL = 200;  // 5 раз в секунду, как в уроке 22
const unsigned long DIST_STALE_MS    = 1500; // старее -> публикуем -1 («не знаю»)

// ---- Состояние: сеть, датчик, защита (урок 22) -----------------------------
unsigned long lastCommandMs = 0;
unsigned long lastPublish   = 0;
bool moving      = false;
bool tiltBlocked = false;
unsigned long overLimitSince = 0;
int  guardSent = -1;
float pitch = 0, roll = 0, yaw = 0;

WiFiClient   net;
PubSubClient mqtt(net);

// ---- Состояние: сервопривод ------------------------------------------------
uint8_t  servoAimDeg  = SERVO_FORWARD_DEG;   // куда ПОСЛАН вал
uint32_t servoReadyAt = 0;                   // когда ему можно верить (доехал и успокоился)

// ---- Состояние: сонар «по курсу» -------------------------------------------
float    fwdRing[PINGS_PER_POINT];           // последние измерения на 90°
uint8_t  fwdCount   = 0;
uint32_t nextPingAt = 0;
uint8_t  unknownRun = 0;                     // сколько подряд «нет эха»
float    lastFwdCm  = NAN;                   // последняя ДОСТОВЕРНАЯ дистанция по курсу
uint32_t lastFwdAt  = 0;

// ---- Состояние: автопилот --------------------------------------------------
enum AutoState : uint8_t {
  AUTO_OFF,     // рулит человек
  AUTO_DRIVE,   // едем вперёд и «пикаем» по курсу
  AUTO_SCAN,    // стоим и осматриваем сектор (stop and look)
  AUTO_TURN,    // доворачиваем по гироскопу
  AUTO_BACK     // пятимся: впереди всё занято
};
AutoState autoState = AUTO_OFF;
int modeSent = -1;                           // что про режим уже знает бэкенд
int bearingSent = -1000;                     // -1000 = ещё ничего не отправляли
bool autoPaused = false;                     // приостановлены защитой по наклону
uint8_t blindCount = 0;                      // осмотров подряд без свободного направления

// Проход сектора как автомат: за один loop() — один шаг.
enum ScanPhase : uint8_t { SCAN_SETTLE, SCAN_PING };
float    scanBuf[SCAN_POINTS];
float    pingBuf[PINGS_PER_POINT];
uint8_t  scanIndex = 0, scanPing = 0, pingCount = 0;
ScanPhase scanPhase = SCAN_SETTLE;
uint32_t scanNextAt = 0, scanStartedAt = 0;

// Поворот по гироскопу.
float    turnStartYaw = 0;
int16_t  turnGoalDeg  = 0;
uint32_t turnDeadline = 0;

// ---------------------------------------------------------------------------
//  МОТОРЫ. Отличие от урока 22 — скорость стала параметром: автопилот ездит
//  медленнее человека (см. SPEED_AUTO выше).
//
//  ПРО analogWrite И LEDC. Моторы идут через analogWrite, а сервопривод — через
//  явный ledcSetup(CH_SERVO=4). В ядре 2.x analogWrite раздаёт каналы сам, по
//  порядку с нулевого: два ШИМ-пина моторов займут каналы 0 и 1 (таймер 0), а
//  канал 4 — это таймер 2. Пересечения нет, и именно поэтому CH_SERVO не 0..2.
// ---------------------------------------------------------------------------
void motor(int in1, int in2, int pwm, int dir, int speed) {
  digitalWrite(in1, dir > 0 ? HIGH : LOW);
  digitalWrite(in2, dir < 0 ? HIGH : LOW);
  analogWrite(pwm, dir == 0 ? 0 : speed);
}

void driveStop() {
  motor(MOTOR_AIN1, MOTOR_AIN2, MOTOR_PWMA, 0, 0);
  motor(MOTOR_BIN1, MOTOR_BIN2, MOTOR_PWMB, 0, 0);
  moving = false;
}

void driveTank(int left, int right, int speed) {
  motor(MOTOR_AIN1, MOTOR_AIN2, MOTOR_PWMA, left,  speed);
  motor(MOTOR_BIN1, MOTOR_BIN2, MOTOR_PWMB, right, speed);
  moving = (left != 0 || right != 0);
}

// ---------------------------------------------------------------------------
//  СЕРВОПРИВОД (урок 28). servoRelax() здесь СОЗНАТЕЛЬНО не используется:
//  сонар должен всё время смотреть строго вперёд, а на едущей машинке отпущенный
//  вал утащит вибрацией. Платим за это током удержания (~10 мА) и жужжанием.
// ---------------------------------------------------------------------------
uint32_t usToDuty(uint32_t us) {
  return (uint32_t)(((uint64_t)us * ((1UL << SERVO_BITS) - 1)) / 20000UL);
}

void servoWriteDeg(float deg) {
  deg = constrain(deg, 0.0f, 180.0f);
  uint32_t us = SERVO_MIN_US + (uint32_t)((SERVO_MAX_US - SERVO_MIN_US) * deg / 180.0f);
  ledcWrite(CH_SERVO, usToDuty(us));
}

// Навести вал и запомнить, когда ему снова можно верить: ПОВЕРНУТЬ -> ПОДОЖДАТЬ -> МЕРИТЬ.
void servoAim(uint8_t deg, uint32_t settleMs) {
  servoWriteDeg(deg);
  servoAimDeg  = deg;
  servoReadyAt = millis() + settleMs;
}

bool servoSettled() {
  return (int32_t)(millis() - servoReadyAt) >= 0;
}

// ---------------------------------------------------------------------------
//  СОНАР (урок 27). NAN = ответа нет. Это ТРЕТЬЕ состояние, а не «далеко»:
//  так же выглядит стена под углом, штора и отвалившийся по питанию датчик.
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

// Медиана; NAN, если валидных меньше двух — «одно измерение из трёх» доверия не стоит.
float medianOf(float* v, uint8_t n) {
  if (n < 2) return NAN;
  for (uint8_t i = 1; i < n; i++) {          // сортировка вставками, элементов единицы
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
//  ТЕЛЕМЕТРИЯ (контракт урока 19: тело = голое число)
// ---------------------------------------------------------------------------
void publishNumber(const char* topic, float value) {
  char buf[16];
  dtostrf(value, 0, 1, buf);
  mqtt.publish(topic, buf);
}

// guard/mode/bearing — СОБЫТИЯ, а не поток измерений: шлём только при изменении.
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

// Дистанция по курсу. -1 — это «не знаю»: либо эха нет, либо измерение протухло
// (идёт осмотр, вал смотрит вбок). Дашборд обязан рисовать -1 как «нет эха», а не
// как ноль сантиметров и не как «свободно».
void publishDistance() {
  bool fresh = !isnan(lastFwdCm) && (millis() - lastFwdAt) <= DIST_STALE_MS;
  publishNumber(TOPIC_DIST, fresh ? lastFwdCm : -1.0f);
}

// ---------------------------------------------------------------------------
//  ЗАЩИТА ПО НАКЛОНУ — без изменений из урока 22 (два фильтра, оба нужны:
//  подробности там же, в комментарии к функции).
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
//  АВТОПИЛОТ: включение/выключение
// ---------------------------------------------------------------------------
void beginScan();   // ниже

void setAutopilot(bool on, const char* why) {
  if (on == (autoState != AUTO_OFF)) return;

  driveStop();
  if (on) {
    // Начинаем ВСЕГДА с осмотра, а не с езды: мы понятия не имеем, что перед
    // носом, а «поехали и разберёмся» — это ровно тот автопилот, который врезается.
    blindCount = 0;
    autoPaused = false;
    autoState  = AUTO_SCAN;
    beginScan();
    Serial.printf("\n[AUTO] ВКЛЮЧЁН (%s) — сначала осмотреться\n", why);
  } else {
    autoState = AUTO_OFF;
    servoAim(SERVO_FORWARD_DEG, SETTLE_FAR_MS);   // вернуть сонар по курсу
    Serial.printf("[AUTO] выключен (%s)\n", why);
  }
  publishModeIfChanged();
}

// ---------------------------------------------------------------------------
//  АВТОПИЛОТ: осмотр сектора одним шагом за loop()
//
//  Это и есть главное отличие от урока 28. Там sweep() стоял в delay() два
//  раза по секунде; здесь каждый вызов делает МИНИМУМ работы и сразу возвращает
//  управление, а «сколько ещё ждать» живёт в scanNextAt. Единственное, что всё
//  ещё блокирует, — сам pulseIn (до 15 мс), и это физика, а не код.
// ---------------------------------------------------------------------------
void beginScan() {
  for (uint8_t i = 0; i < SCAN_POINTS; i++) scanBuf[i] = NAN;
  scanIndex = 0;
  scanPing  = 0;
  pingCount = 0;
  scanPhase = SCAN_SETTLE;
  scanStartedAt = millis();
  servoAim(angleAt(0), SETTLE_FAR_MS);   // с 90° на 30° — это переезд, не шаг
  scanNextAt = servoReadyAt;
}

// true = проход закончен, результат в scanBuf.
bool serviceScan() {
  uint32_t now = millis();
  if ((int32_t)(now - scanNextAt) < 0) return false;

  if (scanPhase == SCAN_SETTLE) {
    // Вал доехал и успокоился — начинаем серию «пиков» на этом угле.
    scanPhase = SCAN_PING;
    scanPing  = 0;
    pingCount = 0;
  }

  // Ровно ОДИН «пик» за проход loop().
  float cm = pingCm();
  if (!isnan(cm)) pingBuf[pingCount++] = cm;
  scanPing++;

  // Точка 90° — заодно свежая дистанция по курсу для телеметрии.
  if (!isnan(cm) && angleAt(scanIndex) == SERVO_FORWARD_DEG) {
    lastFwdCm = cm;
    lastFwdAt = now;
  }

  if (scanPing < PINGS_PER_POINT) {
    scanNextAt = now + PING_GAP_MS;      // даташит: не чаще раза в 60 мс
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

// Печать сектора «радаром» — то же, что в уроке 28, но одной строкой на точку.
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
//  АВТОПИЛОТ: поворот по гироскопу
//
//  Знак yaw НЕ важен: направление задают моторы, а гироскоп только меряет, сколько
//  уже накрутили — поэтому берём модуль разности. Это заодно избавляет от вопроса
//  «в какую сторону считает ось Z у конкретно этой платы».
//  Таймаут — на случай, когда yaw не растёт: колесо буксует, робот упёрся,
//  датчик отвалился. Тогда просто прекращаем и идём осматриваться заново.
// ---------------------------------------------------------------------------
void beginTurn(int16_t deg, bool left) {
  turnGoalDeg   = deg;
  turnStartYaw  = yaw;
  turnDeadline  = millis() + 400 + (uint32_t)deg * 25;   // с большим запасом
  servoAim(SERVO_FORWARD_DEG, SETTLE_MS);                // сонар — по курсу
  driveTank(left ? -1 : +1, left ? +1 : -1, SPEED_AUTO); // танковый разворот на месте
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
  // После поворота — снова осмотр. «Повернулся и поехал» означало бы ехать по
  // данным, снятым до поворота, то есть по старому миру.
  autoState = AUTO_SCAN;
  beginScan();
}

// ---------------------------------------------------------------------------
//  АВТОПИЛОТ: решение по итогам осмотра
// ---------------------------------------------------------------------------
void decideAfterScan() {
  printScan();

  int8_t best = -1;
  for (uint8_t i = 0; i < SCAN_POINTS; i++) {
    if (isnan(scanBuf[i]) || scanBuf[i] < CLEAR_CM) continue;
    if (best < 0 || scanBuf[i] > scanBuf[best]) best = i;
  }

  if (best < 0) {
    // Ни одного свободного направления. Это НЕ «кругом чисто» — это либо угол,
    // либо мы стоим носом в диван, который сонар не слышит. Пятимся и смотрим снова.
    blindCount++;
    Serial.printf("[AUTO] свободных направлений нет (попытка %u из %u) — назад\n",
                  blindCount, BLIND_LIMIT);
    if (blindCount >= BLIND_LIMIT) {
      // Сдаёмся честно: глохнем и сообщаем бэкенду mode=0, чтобы в дашборде тумблер
      // сам погас. Робот, который «как-то выберется», выберется через плинтус.
      setAutopilot(false, "три осмотра подряд без свободного направления");
      return;
    }
    driveTank(-1, -1, SPEED_AUTO);
    turnDeadline = millis() + BACK_MS;   // переиспользуем как «до каких пор пятиться»
    autoState = AUTO_BACK;
    return;
  }

  blindCount = 0;
  int16_t bearingDeg = angleAt(best);
  publishBearing(bearingDeg);

  int16_t delta = bearingDeg - (int16_t)SERVO_FORWARD_DEG;   // >0 — в сторону «больших» углов
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
//  АВТОПИЛОТ: езда вперёд с «пиками» по курсу
// ---------------------------------------------------------------------------
void serviceDrive() {
  uint32_t now = millis();
  if (!servoSettled() || (int32_t)(now - nextPingAt) < 0) return;
  nextPingAt = now + PING_GAP_MS;

  float cm = pingCm();
  if (isnan(cm)) {
    // «Нет эха» — это не «свободно». Но и не паника: одно молчание в комнате
    // означает всего лишь «дальше 2.5 м». Считаем ПОДРЯД ИДУЩИЕ молчания.
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
  if (isnan(median)) return;                 // ещё не набрали три — едем дальше

  if (median <= STOP_CM) {
    Serial.printf("[AUTO] стена в %.0f см (порог %.0f) — стоп, осмотреться\n", median, STOP_CM);
    driveStop();
    autoState = AUTO_SCAN;
    beginScan();
  }
}

// ---------------------------------------------------------------------------
//  АВТОПИЛОТ: один шаг автомата за проход loop()
// ---------------------------------------------------------------------------
void serviceAutopilot() {
  if (autoState == AUTO_OFF) return;

  // Защита по наклону главнее автопилота: пока машинка задрана — стоим.
  if (tiltBlocked) {
    if (moving) driveStop();
    autoPaused = true;
    return;
  }
  if (autoPaused) {
    // Отпустило. Мир за это время мог измениться (нас, например, сняли с препятствия
    // и поставили в другую сторону) — поэтому не продолжаем, а смотрим заново.
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
//  КОМАНДЫ
// ---------------------------------------------------------------------------
void applyCommand(const String& cmd) {
  // Сначала РАЗБИРАЕМ команду и только потом что-то делаем. Порядок важен: ниже
  // распознанная команда выключает автопилот, и мусор в топике не должен получить
  // такое право — «стоп» по опечатке в чужом клиенте это не остановка, а авария.
  int left, right;
  if      (cmd == "stop")    { left =  0; right =  0; }
  else if (cmd == "forward") { left = +1; right = +1; }
  else if (cmd == "back")    { left = -1; right = -1; }
  else if (cmd == "left")    { left = -1; right = +1; }
  else if (cmd == "right")   { left = +1; right = -1; }
  else { Serial.printf("[DRIVE] неизвестная команда '%s'\n", cmd.c_str()); return; }

  // Человек взялся за руль -> автопилот молча уходит. Это не «ошибка», а
  // нормальный способ его выключить: сработает и кнопка в дашборде, и геймпад,
  // и D-pad на FPV-странице из урока 26, которая про автопилот ничего не знает.
  if (autoState != AUTO_OFF) setAutopilot(false, "ручная команда — человек взялся за руль");

  // «Стоп» исполняем ВСЕГДА, даже при активной защите: стоп безопасен по определению.
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

  lastCommandMs = millis();   // failsafe ручного режима
  applyCommand(body);
}

void reconnect() {
  while (!mqtt.connected()) {
    // Автопилот БЕЗ связи ездить не должен. Не потому, что ему нужны команды — он
    // как раз самостоятельный, — а потому, что без брокера его нечем остановить:
    // ни кнопкой в дашборде, ни телеметрией не видно, что он делает.
    if (autoState != AUTO_OFF) setAutopilot(false, "потеряна связь с брокером");

    Serial.print("[MQTT] Подключение к брокеру... ");
    if (mqtt.connect(CLIENT_ID, MQTT_USER, MQTT_PASS)) {
      Serial.println("OK");
      mqtt.subscribe(TOPIC_DRIVE, 1);      // QoS 1 — команды важнее телеметрии
      mqtt.subscribe(TOPIC_CMDMODE, 1);
      Serial.printf("[MQTT] Подписки: '%s', '%s' (QoS 1)\n", TOPIC_DRIVE, TOPIC_CMDMODE);
      // После реконнекта заново сообщим бэкенду состояние всех «событийных» метрик.
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

  // 1) МОТОРЫ — первым делом стоп: дальше три секунды калибровки гироскопа.
  int pins[] = { MOTOR_AIN1, MOTOR_AIN2, MOTOR_PWMA, MOTOR_BIN1, MOTOR_BIN2, MOTOR_PWMB };
  for (int p : pins) pinMode(p, OUTPUT);
  driveStop();

  // 2) СОНАР (урок 27)
  pinMode(SONAR_TRIG, OUTPUT);
  digitalWrite(SONAR_TRIG, LOW);
  pinMode(SONAR_ECHO, INPUT);          // GPIO39 — вход-only, линию держит делитель

  // 3) СЕРВОПРИВОД (урок 28): 50 Гц, 16 бит, канал 4 (таймер 2).
  ledcSetup(CH_SERVO, SERVO_FREQ_HZ, SERVO_BITS);
  ledcAttachPin(SERVO_PIN, CH_SERVO);
  servoAim(SERVO_FORWARD_DEG, SETTLE_FAR_MS);
  delay(600);                           // в setup() блокировать ещё можно
  Serial.println("\n=== Урок 29. Автопилот v1 ===");
  Serial.println("сонар смотрит на 90° — проверь, что он смотрит СТРОГО ВПЕРЁД");

  // 4) I2C + MPU6050 (уроки 20/22)
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
  // 1) Связь. Без mqtt.loop() не работают ни подписки, ни keep-alive.
  if (!mqtt.connected()) reconnect();
  mqtt.loop();

  // 2) Датчик — каждую итерацию: так точнее интеграл гироскопа, а на нём держится
  //    поворот автопилота.
  mpu.update();
  float ax = mpu.getAccX(), ay = mpu.getAccY(), az = mpu.getAccZ();
  roll  = atan2f(ay, az) * 180.0f / PI;
  pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI;
  yaw   = mpu.getAngleZ();

  updateTiltGuard(sqrtf(ax * ax + ay * ay + az * az));

  // 3) Автопилот — ровно один шаг автомата. Ничего блокирующего, кроме pulseIn.
  serviceAutopilot();

  // 4) Сонар «по курсу» в РУЧНОМ режиме. Нужен не автопилоту, а человеку: дашборд
  //    показывает дистанцию и предупреждает о стене, пока рулят руками.
  //    В AUTO_DRIVE этим занимается serviceDrive(), во время осмотра вал смотрит
  //    вбок — тогда мерить «по курсу» нечем.
  if (autoState == AUTO_OFF && servoSettled() && (int32_t)(millis() - nextPingAt) >= 0) {
    nextPingAt = millis() + PING_GAP_MS;
    float cm = pingCm();
    if (!isnan(cm)) { lastFwdCm = cm; lastFwdAt = millis(); }
  }

  // 5) Failsafe урока 21/22 — ТОЛЬКО в ручном режиме. В автопилоте команд нет по
  //    определению, и этот таймер заглушил бы его через 700 мс после старта.
  if (autoState == AUTO_OFF && moving && millis() - lastCommandMs > FAILSAFE_MS) {
    Serial.println("[DRIVE] failsafe: команд нет, стоп");
    driveStop();
  }

  // 6) Телеметрия раз в PUBLISH_INTERVAL мс, без блокирующего delay.
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
