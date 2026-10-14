// ============================================================================
//  CHECK: TB6612FNG — driver and motor check before lesson 21
// ----------------------------------------------------------------------------
//  Goal: make sure the driver is powered, the wires are in place and each motor
//  turns the RIGHT way — BEFORE mixing in Wi-Fi/MQTT (lesson 21).
//  No network and no libraries — only GPIO, analogWrite and Serial.
//
//  TB6612FNG is a dual motor driver. Per motor: two direction inputs
//  (xIN1/xIN2) and a PWM speed input (PWMx):
//     IN1=1, IN2=0 -> forward;  IN1=0, IN2=1 -> back;  IN1=IN2 -> stop.
//  The STBY input enables the driver — in this circuit it is wired to 3V3 (always ON);
//  if the motors don't turn at all — first check STBY and the VM supply.
//
//  What it does: first (if DIAG_B) runs the side-B isolation phase, then
//  loops through a test sequence (once per STEP_MS), printing each step to
//  Serial. It first turns EACH motor separately (to tell which is A and which
//  is B, and which way each one turns), then both together.
//
//  How to read the result (motor A = left, B = right):
//    • "A forward" -> the left wheel should drive FORWARD. If backwards — swap
//      this motor's two wires OR MOTOR_AIN1/MOTOR_AIN2 in pins.h.
//    • "B forward" -> same for the right wheel.
//    • if a motor other than the named one turns — the driver's A/B outputs are swapped.
//    • if a motor is silent in one direction but not the other — bad contact
//      on its IN pin.
//
//  SAFETY: lift the wheels off the table (robot "on a jack"), otherwise it
//  drives off during the test. We start from a stop, and there is a stop pause between steps too.
//
//  Build/flash:
//    pio run -e lesson_check_tb6612 -t upload
//    pio device monitor
//
//  Wiring (see include/pins.h): STBY -> 3V3, VM -> motor supply,
//  VCC -> 3.3V logic, GND -> common GND with the ESP32.
// ============================================================================

#include <Arduino.h>
#include "pins.h"

const int SPEED = 200;                 // 0..255 — PWM speed (analogWrite)
const unsigned long STEP_MS = 1500;    // duration of one test step
const unsigned long PAUSE_MS = 600;    // stop pause between steps

// One motor: dir>0 — forward, dir<0 — back, dir==0 — stop.
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

// One test step: print what we're doing, run for STEP_MS, then stop + pause.
void step(const char* label, int dirA, int dirB) {
  Serial.printf("[TEST] %s  (A=%+d, B=%+d)\n", label, dirA, dirB);
  motorA(dirA);
  motorB(dirB);
  delay(STEP_MS);
  bothStop();
  delay(PAUSE_MS);
}

// ============================================================================
//  SIDE B ISOLATION PHASE (DIAG_B)
// ----------------------------------------------------------------------------
//  Runs once in setup(). Narrows "side B doesn't work" down to a
//  specific link of the chain: GPIO -> wire -> driver -> motor.
//  The code for A and B is fully symmetric (analogWrite gives each PWM pin
//  its own LEDC channel), so the cause is almost certainly in the hardware — and it
//  must be localised, not guessed.
//
//  IMPORTANT ABOUT ORDER: on the ESP32 analogWrite attaches the pin to the LEDC matrix,
//  and after that digitalWrite on THAT SAME pin no longer changes anything. So
//  the "raw" steps 1 and 2 must come BEFORE any analogWrite — including before
//  bothStop(), which internally does analogWrite(pwm, 0).
// ============================================================================
const bool DIAG_B = true;              // false -> go straight to the normal test loop

// Step 1. Is there any signal on the B pins at all — slow blinking with raw
// digitalWrite. Measure with a multimeter (or an LED through a ~330 Ohm resistor)
// TWICE: first at the ESP32 pin, then at the corresponding driver input.
//   • no ~3.3 V at the ESP32 pin         -> GPIO/code issue, move the pin;
//   • present at ESP32, not at driver    -> broken wire / bad contact;
//   • present at both ends of all three  -> logic gets through, go to step 2.
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

// Step 2. Motor B at full power with no PWM at all: PWMB held HIGH by plain
// digitalWrite. The "dumbest" and strongest mode possible.
//   • turned here, but not with analogWrite -> it's the PWM/speed (step 3);
//   • doesn't turn even like this           -> mechanics/driver/power (step 4).
void diagFullOn() {
  Serial.println("--- ШАГ 2: мотор B на 100% без ШИМ (PWMB зажат HIGH) ---");

  digitalWrite(MOTOR_PWMB, HIGH);       // full throttle, bypassing LEDC
  Serial.println("[DIAG] B forward, 100%");
  digitalWrite(MOTOR_BIN1, HIGH);
  digitalWrite(MOTOR_BIN2, LOW);
  delay(STEP_MS);

  Serial.println("[DIAG] B back, 100%");
  digitalWrite(MOTOR_BIN1, LOW);
  digitalWrite(MOTOR_BIN2, HIGH);
  delay(STEP_MS);

  digitalWrite(MOTOR_BIN1, LOW);        // stop: IN1 == IN2
  digitalWrite(MOTOR_BIN2, LOW);
  digitalWrite(MOTOR_PWMB, LOW);
  delay(PAUSE_MS);
}

// Step 3. From here on PWMB goes to LEDC. Run a duty ramp: if the motor
// comes alive only at 220-255, SPEED=200 is simply not enough to get it moving
// (stiff mechanics or sagging power) — that is not "doesn't work".
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

// Step 4 — by hand, code can't do it: we print instructions to Serial.
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
  for (int p : pins) digitalWrite(p, LOW);   // stop WITHOUT analogWrite: don't touch LEDC yet

  delay(2000);

  // Side B isolation: steps 1-2 are raw, so strictly before the first analogWrite.
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
  // 1) Each motor separately — this shows which is A/which is B and which way it turns.
  step("A forward", +1,  0);
  step("A back",    -1,  0);
  step("B forward",  0, +1);
  step("B back",     0, -1);

  // 2) Both together — as in lesson 21 (tank-style control).
  step("forward",   +1, +1);
  step("back",      -1, -1);
  step("left",      -1, +1);   // spin in place
  step("right",     +1, -1);

  Serial.println("[TEST] круг завершён, повтор через 2 c\n");
  delay(2000);
}
