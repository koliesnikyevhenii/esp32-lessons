// ============================================================================
//  CHECK: Cytron MDD10A — driver and motor check before lesson 30
// ----------------------------------------------------------------------------
//  Same as lesson_check_tb6612, but for the new driver. No network and
//  no libraries — only GPIO, LEDC and Serial.
//
//  MDD10A, sign-magnitude mode: two inputs per channel —
//     DIR  -> direction (HIGH/LOW),
//     PWM  -> speed (duty cycle, up to 20 kHz); PWM = 0 is a BRAKE.
//
//  BEFORE FLASHING: the MDD10A board itself has test buttons (M1A/M1B/M2A/M2B) —
//  press them with motor power applied. Wheels turn -> the power stage and
//  motors are fine, from here on only look at the wires from the ESP32.
//
//  How to read the result (channel 1 = left, channel 2 = right):
//    • "L forward" -> the left wheel drives FORWARD. If backwards — set
//      MOTOR_LEFT_INVERT = true (here and in lesson 30) or swap M1A/M1B.
//    • the wrong wheel turns -> channels 1/2 are swapped (or the motor wires).
//    • ramp: the wheel starts only at a high duty cycle -> stiff mechanics or
//      a sagging battery; raise SPEED_MANUAL/SPEED_AUTO in lesson 30.
//
//  SAFETY: wheels off the table, otherwise the robot drives off during the test.
//
//  Build: pio run -e lesson_check_mdd10 -t upload && pio device monitor
// ============================================================================

#include <Arduino.h>
#include "pins.h"

const uint32_t PWM_HZ   = 20000;      // as in lesson 30
const uint8_t  PWM_BITS = 8;
const int SPEED = 200;
const unsigned long STEP_MS  = 1500;
const unsigned long PAUSE_MS = 600;   // stop between steps — doubles as the pause before reversing

const bool MOTOR_LEFT_INVERT  = false;
const bool MOTOR_RIGHT_INVERT = true;

void motor(uint8_t dirPin, uint8_t ch, bool invert, int dir) {
  if (dir == 0) { ledcWrite(ch, 0); return; }        // brake
  ledcWrite(ch, 0);                                  // change DIR at zero PWM
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

// Duty-cycle ramp on one channel: at what value the wheel starts moving.
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
  step("left",      -1, +1);   // spin in place
  step("right",     +1, -1);

  Serial.println("[TEST] круг завершён, повтор через 2 c\n");
  delay(2000);
}
