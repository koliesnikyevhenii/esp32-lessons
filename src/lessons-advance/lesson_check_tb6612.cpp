// ============================================================================
//  CHECK: TB6612FNG — проверка драйвера и моторов перед уроком 21
// ----------------------------------------------------------------------------
//  Цель: убедиться, что драйвер запитан, провода на месте и каждый мотор
//  крутится В НУЖНУЮ сторону — ДО того как подмешивать Wi-Fi/MQTT (урок 21).
//  Никакой сети и никаких библиотек — только GPIO, analogWrite и Serial.
//
//  TB6612FNG — драйвер двух моторов. На каждый мотор: два входа направления
//  (xIN1/xIN2) и вход скорости PWM (PWMx):
//     IN1=1, IN2=0 -> вперёд;  IN1=0, IN2=1 -> назад;  IN1=IN2 -> стоп.
//  Вход STBY включает драйвер — в этой схеме он заведён на 3V3 (всегда ON);
//  если моторы вообще не крутятся — первым делом проверь STBY и питание VM.
//
//  Что делает: сначала (если DIAG_B) прогоняет фазу изоляции стороны B, потом
//  гоняет по кругу тестовую последовательность (раз в STEP_MS), печатая в
//  Serial каждый шаг. Крутит сначала КАЖДЫЙ мотор по отдельности (чтобы понять,
//  где A, а где B, и куда каждый крутится), потом оба вместе.
//
//  Как читать результат (мотор A = левый, B = правый):
//    • «A forward» -> левое колесо должно ехать ВПЕРЁД. Если назад — поменяй
//      местами два провода этого мотора ИЛИ MOTOR_AIN1/MOTOR_AIN2 в pins.h.
//    • «B forward» -> то же для правого колеса.
//    • если крутится не тот мотор, что назван — перепутаны выходы A/B драйвера.
//    • если мотор молчит на одном направлении, но не на другом — плохой контакт
//      на его IN-пине.
//
//  БЕЗОПАСНОСТЬ: подними колёса над столом (робот «на домкрате»), иначе он
//  уедет во время теста. Стартуем со стопа, между шагами тоже пауза со стопом.
//
//  Сборка/прошивка:
//    pio run -e lesson_check_tb6612 -t upload
//    pio device monitor
//
//  Подключение (см. include/pins.h): STBY -> 3V3, VM -> питание моторов,
//  VCC -> 3.3V логики, GND -> общий GND с ESP32.
// ============================================================================

#include <Arduino.h>
#include "pins.h"

const int SPEED = 200;                 // 0..255 — скорость по ШИМ (analogWrite)
const unsigned long STEP_MS = 1500;    // длительность одного шага теста
const unsigned long PAUSE_MS = 600;    // пауза-стоп между шагами

// Один мотор: dir>0 — вперёд, dir<0 — назад, dir==0 — стоп.
void motor(int in1, int in2, int pwm, int dir) {
  digitalWrite(in1, dir > 0 ? HIGH : LOW);
  digitalWrite(in2, dir < 0 ? HIGH : LOW);
  analogWrite(pwm, dir == 0 ? 0 : SPEED);
}

void motorA(int dir) { motor(MOTOR_AIN1, MOTOR_AIN2, MOTOR_PWMA, dir); }
void motorB(int dir) { motor(MOTOR_BIN1, MOTOR_BIN2, MOTOR_PWMB, dir); }

void bothStop() {
  motorA(0);
  motorB(0);
}

// Один шаг теста: печатаем что делаем, крутим STEP_MS, потом стоп + пауза.
void step(const char* label, int dirA, int dirB) {
  Serial.printf("[TEST] %s  (A=%+d, B=%+d)\n", label, dirA, dirB);
  motorA(dirA);
  motorB(dirB);
  delay(STEP_MS);
  bothStop();
  delay(PAUSE_MS);
}

// ============================================================================
//  ФАЗА ИЗОЛЯЦИИ СТОРОНЫ B (DIAG_B)
// ----------------------------------------------------------------------------
//  Прогоняется один раз в setup(). Сужает «сторона B не работает» до
//  конкретного звена цепочки: GPIO -> провод -> драйвер -> мотор.
//  Код для A и B полностью симметричен (analogWrite выдаёт каждому PWM-пину
//  свой канал LEDC), поэтому причина почти наверняка в железе — и её надо
//  локализовать, а не угадывать.
//
//  ВАЖНО ПРО ПОРЯДОК: analogWrite на ESP32 привязывает пин к матрице LEDC, и
//  после этого digitalWrite на ЭТОМ ЖЕ пине больше ничего не меняет. Поэтому
//  «сырые» шаги 1 и 2 обязаны идти ДО любого analogWrite — в том числе до
//  bothStop(), который внутри делает analogWrite(pwm, 0).
// ============================================================================
const bool DIAG_B = true;              // false -> сразу обычный цикл теста

// Шаг 1. Есть ли вообще сигнал на пинах B — медленное мигание сырым
// digitalWrite. Мерь мультиметром (или светодиодом через резистор ~330 Ом)
// ДВА раза: сначала на ножке ESP32, потом на соответствующем входе драйвера.
//   • нет ~3.3 V на ножке ESP32          -> дело в GPIO/коде, перевесь пин;
//   • есть на ESP32, нет на драйвере     -> обрыв провода / плохой контакт;
//   • есть на обоих концах всех трёх     -> логика доезжает, иди на шаг 2.
void diagPinLevels() {
  struct { const char* name; int pin; } bpins[] = {
    { "MOTOR_BIN1", MOTOR_BIN1 },
    { "MOTOR_BIN2", MOTOR_BIN2 },
    { "MOTOR_PWMB", MOTOR_PWMB },
  };

  Serial.println("--- ШАГ 1: уровни на пинах B (сырой digitalWrite, без ШИМ) ---");
  for (auto& b : bpins) {
    Serial.printf("[DIAG] %s = GPIO%d: 6 раз по 0.5 c HIGH/LOW — мерь сейчас\n", b.name, b.pin);
    for (int i = 0; i < 6; i++) {
      digitalWrite(b.pin, HIGH);
      delay(500);
      digitalWrite(b.pin, LOW);
      delay(500);
    }
  }
}

// Шаг 2. Мотор B на полной мощности вообще без ШИМ: PWMB зажат в HIGH обычным
// digitalWrite. Самый «тупой» и самый сильный режим, какой возможен.
//   • закрутился здесь, но не на analogWrite -> дело в ШИМ/скорости (шаг 3);
//   • не крутится даже так                   -> механика/драйвер/питание (шаг 4).
void diagFullOn() {
  Serial.println("--- ШАГ 2: мотор B на 100% без ШИМ (PWMB зажат HIGH) ---");

  digitalWrite(MOTOR_PWMB, HIGH);       // полный газ, мимо LEDC
  Serial.println("[DIAG] B forward, 100%");
  digitalWrite(MOTOR_BIN1, HIGH);
  digitalWrite(MOTOR_BIN2, LOW);
  delay(STEP_MS);

  Serial.println("[DIAG] B back, 100%");
  digitalWrite(MOTOR_BIN1, LOW);
  digitalWrite(MOTOR_BIN2, HIGH);
  delay(STEP_MS);

  digitalWrite(MOTOR_BIN1, LOW);        // стоп: IN1 == IN2
  digitalWrite(MOTOR_BIN2, LOW);
  digitalWrite(MOTOR_PWMB, LOW);
  delay(PAUSE_MS);
}

// Шаг 3. С этого места PWMB уходит в LEDC. Гоним рампу по дути: если мотор
// оживает только на 220-255, ему просто не хватает SPEED=200 чтобы стронуться
// (тугая механика или просаживающееся питание) — это не «не работает».
void diagPwmRamp() {
  Serial.println("--- ШАГ 3: мотор B через analogWrite, рампа дути ---");
  digitalWrite(MOTOR_BIN1, HIGH);
  digitalWrite(MOTOR_BIN2, LOW);
  for (int duty : { 100, 140, 180, 220, 255 }) {
    analogWrite(MOTOR_PWMB, duty);
    Serial.printf("[DIAG] B forward, duty=%d (LEDC канал %d)\n",
                  duty, analogGetChannel(MOTOR_PWMB));
    delay(1200);
  }
  analogWrite(MOTOR_PWMB, 0);
  digitalWrite(MOTOR_BIN1, LOW);
  delay(PAUSE_MS);
}

// Шаг 4 — руками, кодом это не сделать: печатаем инструкцию в Serial.
void diagManualHints() {
  Serial.println("--- ШАГ 4 (руками), если шаги 1-3 причину не выявили ---");
  Serial.println("  a) перекинь провода моторов на ВЫХОДАХ драйвера: A01/A02 <-> B01/B02.");
  Serial.println("     бывший B закрутился на выходах A -> сдох канал B драйвера;");
  Serial.println("     мотор молчит и на выходах A      -> дело в моторе или его проводах.");
  Serial.println("  b) проверь VM под нагрузкой: батарея просаживается -> драйвер уходит в защиту.");
  Serial.println("  c) GND ESP32 и GND драйвера должны быть соединены — без общего GND уровни плавают.");
  Serial.printf ("  d) PWMB сейчас на GPIO%d — если шаг 1 не дал 3.3 V, попробуй другой пин.\n", MOTOR_PWMB);
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== CHECK TB6612FNG (raw GPIO) ===");
  Serial.println("Подними колёса над столом! Тест начнётся через 2 c.\n");

  int pins[] = { MOTOR_AIN1, MOTOR_AIN2, MOTOR_PWMA, MOTOR_BIN1, MOTOR_BIN2, MOTOR_PWMB };
  for (int p : pins) pinMode(p, OUTPUT);
  for (int p : pins) digitalWrite(p, LOW);   // стоп БЕЗ analogWrite: LEDC пока не трогаем

  delay(2000);

  // Изоляция стороны B: шаги 1-2 сырые, поэтому строго до первого analogWrite.
  if (DIAG_B) {
    Serial.println("=== DIAG B: изоляция стороны B ===\n");
    diagPinLevels();
    diagFullOn();
    diagPwmRamp();
    diagManualHints();
    Serial.println("=== DIAG B завершён, дальше обычный цикл теста ===\n");
  }

  bothStop();
  delay(PAUSE_MS);
}

void loop() {
  // 1) Каждый мотор по отдельности — так видно, кто A/кто B и куда крутится.
  step("A forward", +1,  0);
  step("A back",    -1,  0);
  step("B forward",  0, +1);
  step("B back",     0, -1);

  // 2) Оба вместе — как в уроке 21 (танковое управление).
  step("forward",   +1, +1);
  step("back",      -1, -1);
  step("left",      -1, +1);   // разворот на месте
  step("right",     +1, -1);

  Serial.println("[TEST] круг завершён, повтор через 2 c\n");
  delay(2000);
}
