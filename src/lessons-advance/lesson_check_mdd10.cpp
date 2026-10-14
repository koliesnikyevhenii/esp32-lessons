// ============================================================================
//  CHECK: Cytron MDD10A — проверка драйвера и моторов перед уроком 30
// ----------------------------------------------------------------------------
//  То же, что lesson_check_tb6612, но для нового драйвера. Никакой сети и
//  никаких библиотек — только GPIO, LEDC и Serial.
//
//  MDD10A, режим sign-magnitude: на канал два входа —
//     DIR  -> направление (HIGH/LOW),
//     PWM  -> скорость (скважность, до 20 кГц); PWM = 0 — ТОРМОЗ.
//
//  ДО ПРОШИВКИ: на самой плате MDD10A есть кнопки теста (M1A/M1B/M2A/M2B) —
//  нажми их при поданном питании моторов. Колёса крутятся -> силовая часть и
//  моторы исправны, дальше ищем только в проводах от ESP32.
//
//  Как читать результат (канал 1 = левый, канал 2 = правый):
//    • «L forward» -> левое колесо едет ВПЕРЁД. Если назад — поставь
//      MOTOR_LEFT_INVERT = true (здесь и в уроке 30) или поменяй M1A/M1B.
//    • крутится не то колесо -> перепутаны каналы 1/2 (или провода моторов).
//    • рампа: колесо стартует только с высокой скважности -> тугая механика или
//      просаживается батарея; SPEED_MANUAL/SPEED_AUTO в уроке 30 подними.
//
//  БЕЗОПАСНОСТЬ: колёса над столом, иначе робот уедет во время теста.
//
//  Сборка: pio run -e lesson_check_mdd10 -t upload && pio device monitor
// ============================================================================

#include <Arduino.h>
#include "pins.h"

const uint32_t PWM_HZ   = 20000;      // как в уроке 30
const uint8_t  PWM_BITS = 8;
const int SPEED = 200;
const unsigned long STEP_MS  = 1500;
const unsigned long PAUSE_MS = 600;   // стоп между шагами — заодно пауза перед реверсом

const bool MOTOR_LEFT_INVERT  = false;
const bool MOTOR_RIGHT_INVERT = true;

void motor(uint8_t dirPin, uint8_t ch, bool invert, int dir) {
  if (dir == 0) { ledcWrite(ch, 0); return; }        // тормоз
  ledcWrite(ch, 0);                                  // DIR меняем при нулевом PWM
  digitalWrite(dirPin, ((dir > 0) != invert) ? HIGH : LOW);
  ledcWrite(ch, SPEED);
}

void motorL(int dir) { motor(MDD_DIR1, CH_MOTOR_L, MOTOR_LEFT_INVERT,  dir); }
void motorR(int dir) { motor(MDD_DIR2, CH_MOTOR_R, MOTOR_RIGHT_INVERT, dir); }

void step(const char* label, int l, int r) {
  Serial.printf("[TEST] %-10s (L=%+d, R=%+d)\n", label, l, r);
  motorL(l);
  motorR(r);
  delay(STEP_MS);
  motorL(0);
  motorR(0);
  delay(PAUSE_MS);
}

// Рампа скважности на одном канале: с какого значения колесо трогается.
void ramp(const char* name, uint8_t dirPin, uint8_t ch) {
  digitalWrite(dirPin, HIGH);
  for (int duty : { 60, 100, 140, 180, 220, 255 }) {
    ledcWrite(ch, duty);
    Serial.printf("[RAMP] %s duty=%d\n", name, duty);
    delay(1000);
  }
  ledcWrite(ch, 0);
  delay(PAUSE_MS);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== CHECK Cytron MDD10A ===");
  Serial.println("Подними колёса над столом! Тест начнётся через 2 c.\n");

  ledcSetup(CH_MOTOR_L, PWM_HZ, PWM_BITS);
  ledcSetup(CH_MOTOR_R, PWM_HZ, PWM_BITS);
  ledcWrite(CH_MOTOR_L, 0);
  ledcWrite(CH_MOTOR_R, 0);
  ledcAttachPin(MDD_PWM1, CH_MOTOR_L);
  ledcAttachPin(MDD_PWM2, CH_MOTOR_R);
  pinMode(MDD_DIR1, OUTPUT);
  pinMode(MDD_DIR2, OUTPUT);
  digitalWrite(MDD_DIR1, LOW);
  digitalWrite(MDD_DIR2, LOW);

  delay(2000);
  ramp("L", MDD_DIR1, CH_MOTOR_L);
  ramp("R", MDD_DIR2, CH_MOTOR_R);
}

void loop() {
  step("L forward", +1,  0);
  step("L back",    -1,  0);
  step("R forward",  0, +1);
  step("R back",     0, -1);

  step("forward",   +1, +1);
  step("back",      -1, -1);
  step("left",      -1, +1);   // разворот на месте
  step("right",     +1, -1);

  Serial.println("[TEST] круг завершён, повтор через 2 c\n");
  delay(2000);
}
