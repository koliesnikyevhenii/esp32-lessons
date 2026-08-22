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
//  Что делает: гоняет по кругу тестовую последовательность (раз в STEP_MS),
//  печатая в Serial каждый шаг. Крутит сначала КАЖДЫЙ мотор по отдельности
//  (чтобы понять, где A, а где B, и куда каждый крутится), потом оба вместе.
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

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== CHECK TB6612FNG (raw GPIO) ===");
  Serial.println("Подними колёса над столом! Тест начнётся через 2 c.\n");

  int pins[] = { MOTOR_AIN1, MOTOR_AIN2, MOTOR_PWMA, MOTOR_BIN1, MOTOR_BIN2, MOTOR_PWMB };
  for (int p : pins) pinMode(p, OUTPUT);
  bothStop();

  delay(2000);
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
