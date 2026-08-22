// ============================================================================
//  CHECK: TB6612FNG — самый простой тест: ОБА мотора ВПЕРЁД одновременно
// ----------------------------------------------------------------------------
//  Никаких шагов, циклов и диагностики: включились и едем вперёд, пока есть
//  питание. Нужен, чтобы одним взглядом увидеть — крутятся ли оба колеса и в
//  одну ли сторону.
//
//  Направление на TB6612FNG: IN1=1, IN2=0 -> вперёд (IN1=IN2 -> стоп).
//  STBY заведён на 3V3 (всегда включён), скорость — ШИМ на PWMx.
//
//  Что смотреть:
//    • крутятся оба и вперёд          -> проводка в порядке;
//    • одно колесо назад              -> поменяй два провода этого мотора
//                                        (или xIN1/xIN2 в pins.h);
//    • одно колесо молчит             -> гоняй lesson_check_tb6612 (там
//                                        пошаговая изоляция стороны B).
//
//  БЕЗОПАСНОСТЬ: подними колёса над столом — робот поедет СРАЗУ после старта
//  и не остановится сам.
//
//  Сборка/прошивка:
//    pio run -e lesson_check_tb6612_forward -t upload
//    pio device monitor
// ============================================================================

#include <Arduino.h>
#include "pins.h"

const int SPEED = 200;   // 0..255 — если мотор не стартует, подними до 255

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== TB6612FNG: оба мотора ВПЕРЁД ===");
  Serial.println("Подними колёса над столом! Старт через 2 c.");

  int pins[] = { MOTOR_AIN1, MOTOR_AIN2, MOTOR_PWMA, MOTOR_BIN1, MOTOR_BIN2, MOTOR_PWMB };
  for (int p : pins) pinMode(p, OUTPUT);

  delay(2000);

  // Мотор A (левый) — вперёд.
  digitalWrite(MOTOR_AIN1, HIGH);
  digitalWrite(MOTOR_AIN2, LOW);
  analogWrite(MOTOR_PWMA, SPEED);

  // Мотор B (правый) — вперёд.
  digitalWrite(MOTOR_BIN1, HIGH);
  digitalWrite(MOTOR_BIN2, LOW);
  analogWrite(MOTOR_PWMB, SPEED);

  Serial.printf("A: IN1=%d IN2=%d PWM=%d (duty %d)\n", MOTOR_AIN1, MOTOR_AIN2, MOTOR_PWMA, SPEED);
  Serial.printf("B: IN1=%d IN2=%d PWM=%d (duty %d)\n", MOTOR_BIN1, MOTOR_BIN2, MOTOR_PWMB, SPEED);
  Serial.println("Поехали. Чтобы остановить — сними питание моторов (VM).");
}

void loop() {
  // Ничего не делаем: моторы уже крутятся, состояние держится само.
  delay(1000);
}
