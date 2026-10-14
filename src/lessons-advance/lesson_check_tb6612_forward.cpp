// ============================================================================
//  CHECK: TB6612FNG — the simplest test: BOTH motors FORWARD at the same time
// ----------------------------------------------------------------------------
//  No steps, loops or diagnostics: power on and drive forward for as long as
//  there is power. Needed to see at a glance whether both wheels turn and
//  whether they turn the same way.
//
//  Direction on the TB6612FNG: IN1=1, IN2=0 -> forward (IN1=IN2 -> stop).
//  STBY is wired to 3V3 (always enabled), speed is PWM on PWMx.
//
//  What to look for:
//    • both turn, and forward         -> wiring is fine;
//    • one wheel turns backwards      -> swap that motor's two wires
//                                        (or xIN1/xIN2 in pins.h);
//    • one wheel is silent            -> run lesson_check_tb6612 (it has
//                                        step-by-step isolation of side B).
//
//  SAFETY: lift the wheels off the table — the robot moves RIGHT after startup
//  and won't stop by itself.
//
//  Build/flash:
//    pio run -e lesson_check_tb6612_forward -t upload
//    pio device monitor
// ============================================================================

#include <Arduino.h>
#include "pins.h"

const int SPEED = 200;   // 0..255 — if the motor doesn't start, raise to 255

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== TB6612FNG: оба мотора ВПЕРЁД ===");
  Serial.println("Подними колёса над столом! Старт через 2 c.");

  int pins[] = { MOTOR_AIN1, MOTOR_AIN2, MOTOR_PWMA, MOTOR_BIN1, MOTOR_BIN2, MOTOR_PWMB };
  for (int p : pins) pinMode(p, OUTPUT);

  delay(2000);

  // Motor A (left) — forward.
  digitalWrite(MOTOR_AIN1, HIGH);
  digitalWrite(MOTOR_AIN2, LOW);
  analogWrite(MOTOR_PWMA, SPEED);

  // Motor B (right) — forward.
  digitalWrite(MOTOR_BIN1, HIGH);
  digitalWrite(MOTOR_BIN2, LOW);
  analogWrite(MOTOR_PWMB, SPEED);

  Serial.printf("A: IN1=%d IN2=%d PWM=%d (duty %d)\n", MOTOR_AIN1, MOTOR_AIN2, MOTOR_PWMA, SPEED);
  Serial.printf("B: IN1=%d IN2=%d PWM=%d (duty %d)\n", MOTOR_BIN1, MOTOR_BIN2, MOTOR_PWMB, SPEED);
  Serial.println("Поехали. Чтобы остановить — сними питание моторов (VM).");
}

void loop() {
  // Do nothing: the motors are already spinning, the state holds by itself.
  delay(1000);
}
