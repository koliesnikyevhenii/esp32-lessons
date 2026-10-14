#pragma once
#include <Arduino.h>

// Shared hardware map for all lessons. Each lesson includes this and
// initialises only the peripherals it actually uses.

// Traffic-light LEDs
#define LED_RED     23
#define LED_YELLOW  18
#define LED_GREEN   19
#define LED_RED2     2   // standalone LED toggled by the button

// RGB LED pins
#define PIN_R 22
#define PIN_G 21
#define PIN_B 15

// LEDC PWM channels (ESP32 Arduino core 2.x channel-based API)
#define CH_R       0
#define CH_G       1
#define CH_B       2
#define CH_BUZZER  0   // buzzer tone channel

#define BUZZER 5
#define BUTTON 4       // INPUT_PULLUP: idle = HIGH, pressed = LOW

// Analog sensors (ADC1, available while Wi-Fi is active)
#define POT     34     // potentiometer: middle pin -> GPIO34
#define LDR     35     // photoresistor module: A0 -> GPIO35
#define DHT_PIN 16     // DHT11 DATA -> GPIO16

// I2C (lessons 20 and 22, MPU6050/6500). The ESP32 bus is remappable — we take free
// pins so as not to overlap with RGB (21/22) and the other peripherals in this file.
// IMPORTANT: in lesson 22 the sensor and the motors run AT THE SAME TIME, so this pin
// pair and the MOTOR_* group below must not overlap (SCL 17 <-> PWMB 33 were once
// the other way round — 33 turned out to be more convenient for motor B's PWM).
#define MPU_SDA 32     // MPU6050 SDA -> GPIO32
#define MPU_SCL 17     // MPU6050 SCL -> GPIO17

// TB6612FNG — dual motor driver (lessons 21 and 22). A = left motor, B = right.
// The driver's STBY is wired to 3V3 (always enabled), so no GPIO is needed for it.
// Free pins were taken, not overlapping with the peripherals above in this file.
#define MOTOR_AIN1 13  // motor A, direction 1 orange
#define MOTOR_AIN2 14  // motor A, direction 2 yellow
#define MOTOR_PWMA 25  // motor A, speed (PWM) green
#define MOTOR_BIN1 26  // motor B, direction 1 brown
#define MOTOR_BIN2 27  // motor B, direction 2 violet
#define MOTOR_PWMB 33  // motor B, speed (PWM) white

// Cytron MDD10A — dual motor driver replacing the TB6612 (lesson 30 and check sketch).
// Two wires per channel: DIR (direction) + PWM (speed, up to 20 kHz). The numbers
// are a SUBSET of the TB6612 pins above: the same wires move over to the new board,
// and AIN2 (14) and BIN2 (27) are freed. Channel 1 = left motor, channel 2 = right.
// MDD10A logic accepts 3.3 V. Driver and ESP32 GND are common — mandatory.
#define MDD_DIR1 13    // channel 1 (left), direction   (was MOTOR_AIN1) orange
#define MDD_PWM1 25    // channel 1 (left), speed       (was MOTOR_PWMA) green
#define MDD_DIR2 26    // channel 2 (right), direction  (was MOTOR_BIN1) brown
#define MDD_PWM2 33    // channel 2 (right), speed      (was MOTOR_PWMB) white
// LEDC channels for the 20 kHz PWM. 8..15 are a separate low-speed LEDC group with
// its own timers: they don't share a timer with the servo (CH_SERVO 4, 50 Hz).
#define CH_MOTOR_L 8
#define CH_MOTOR_R 9

// HC-SR04 — ultrasonic rangefinder (lesson 27, then the Stage 7 autopilot).
// The sensor is powered from 5V (the board's VIN), and ECHO outputs 5-volt logic —
// ESP32 pins are NOT 5-volt tolerant, so ECHO goes through a 1k/2k divider
// (see documentation/lesson27_hcsr04.md). TRIG is an ordinary 3.3 V output, the
// sensor understands it.
//
// The pins are chosen so as not to overlap with MPU_* and MOTOR_*: on the robot all
// three peripherals run AT THE SAME TIME.
//   GPIO39 — INPUT-only (VN), it physically cannot become an output, and that is
//            exactly what ECHO needs. 34..39 have no pulls — the divider always
//            holds the line, so the input never "floats".
//   GPIO12 — strapping pin (MTDI): if it is HIGH at reset, the chip selects
//            1.8 V for flash and won't boot. GPIO12 has its own internal pull-DOWN
//            at reset, and the HC-SR04's TRIG is a module input without a
//            pull-up, so the combination works. If the board suddenly stops
//            booting with the sensor attached — move TRIG to any free
//            output (e.g. 16, if the DHT11 isn't connected).
#define SONAR_TRIG 12  // output: 10 µs pulse -> "ping"
#define  SONAR_ECHO 14  // input (through the divider!): duration = time of flight

// SG90 — servo for the sonar (lesson 28: scanning rangefinder).
// A servo is driven by 50 Hz PWM (20 ms period), where the VALUE is the
// pulse width 500..2400 µs, not the duty cycle in the usual sense.
//
// POWER — ONLY from 5 V (the MP1584EN rail on the robot), NEVER from the board's 3V3:
// SG90 inrush current reaches ~700 mA, the DevKit regulator can't supply that — you'll
// get a brownout reset exactly at the moment of rotation. Common GND is mandatory.
//
// There are no free pins left on the DevKit (0/1/3 — boot and UART0, 6..11 — flash,
// 36/39 — input only), so we take a number from the basic lessons: GPIO19 is the
// traffic-light LED_GREEN, which the robot doesn't have. What matters is that the
// pin doesn't overlap with the MPU_*, MOTOR_* and SONAR_* groups — on the robot they run together.
#define SERVO_PIN 19   // SG90 signal (orange/yellow) wire
#define CH_SERVO   4   // LEDC channel: not 0/1/2 (RGB and buzzer) and not 7 (camera flash)

// ===========================================================================
//  CAMERA — lessons 23-26. WARNING: THIS IS A DIFFERENT BOARD AND A DIFFERENT CHIP!
// ---------------------------------------------------------------------------
//  Everything above applies to the ESP32-WROOM-32 DevKit (board = esp32dev). The
//  camera lessons are built for the ESP32-S3 (board = esp32-s3-devkitc-1, see
//  platformio.ini) — a physically separate module: the DevKit stays the robot
//  (lesson 22), the camera board is its "eyes". Both live on the car and talk over
//  MQTT, not wires.
//
//  We do NOT choose the numbers below: the sensor ribbon is hard-wired on the board.
//  That's why some numbers coincide with names above (e.g. CAM_Y7 = 18 and
//  LED_YELLOW = 18) — there is no conflict, these are different boards, different
//  chips and different firmware.
//
//  The pins define the BOARD, not the sensor. Which sensor is on the ribbon is a
//  separate question, and it changes the code: on our board it is a GC2145 (PID
//  0x2145), it has NO hardware JPEG, so lessons 23-26 capture RGB565 and compress
//  the frame in software (frame2jpg). Checked with a single run of lesson_check_cam.
//
//  CHOOSE EXACTLY ONE model. Ready-made maps for other boards live in the core:
//  packages/framework-arduinoespressif32/libraries/ESP32/examples/Camera/
//  CameraWebServer/camera_pins.h
// ===========================================================================
//  The model can also be set from OUTSIDE — with a -DCAM_MODEL_* flag in
//  platformio.ini. Then the default below is not applied, and there is no need to
//  edit this file for a board that gets connected once every six months (that's how
//  the lesson_check_cam_aithinker env does it). Without a flag — the S3 board stays,
//  as before.
#if !defined(CAM_MODEL_ESP32S3_CAM) && \
    !defined(CAM_MODEL_XIAO_ESP32S3) && \
    !defined(CAM_MODEL_AI_THINKER)
#define CAM_MODEL_ESP32S3_CAM        // HW-679 / Goouuu ESP32-S3-CAM (pinout as ESP32-S3-EYE)
#endif
// #define CAM_MODEL_XIAO_ESP32S3    // Seeed XIAO ESP32S3 Sense
// #define CAM_MODEL_AI_THINKER      // classic ESP32-CAM — a PLAIN ESP32, not S3!

#if defined(CAM_MODEL_ESP32S3_CAM)
// --- HW-679 ESP32-S3-CAM (ESP32-S3-WROOM-1 N16R8) --------------------------
// Same map as CAMERA_MODEL_ESP32S3_EYE in the core: these boards are wired identically.
#define CAM_PWDN   -1   // no power-down line
#define CAM_RESET  -1   // no reset line
#define CAM_XCLK   15   // sensor clock (20 MHz, generated via LEDC)
#define CAM_SIOD    4   // SCCB (the sensor's I2C) data
#define CAM_SIOC    5   // SCCB clock

// 8-bit parallel data bus D0..D7 (Y2..Y9 in sensor datasheets)
#define CAM_Y9     16   // D7
#define CAM_Y8     17   // D6
#define CAM_Y7     18   // D5
#define CAM_Y6     12   // D4
#define CAM_Y5     10   // D3
#define CAM_Y4      8   // D2
#define CAM_Y3      9   // D1
#define CAM_Y2     11   // D0
#define CAM_VSYNC   6   // frame strobe
#define CAM_HREF    7   // line strobe
#define CAM_PCLK   13   // pixel clock

// Flash/indicator. On the AI-Thinker the white LED is hard-wired to GPIO4, but on S3
// boards the wiring varies from batch to batch, so it is OFF by default (-1):
// all the flash code in the lessons is wrapped in #if and simply doesn't compile.
// To find your pin: pio run -e lesson_check_cam -t upload -> the "LED hunt" mode
// goes through the safe GPIOs and prints which one is blinking right now.
#define CAM_FLASH_LED  -1
#define CAM_STATUS_LED -1
#define CAM_STATUS_LED_ACTIVE_LOW 1   // on most boards LOW = lit

// Free GPIOs on the HW-679 (lesson 26 variant B — motors on the camera itself):
//   1, 2, 3, 14, 21, 47, 48  — free;
//   38, 39, 40               — microSD (free if the card isn't needed);
//   DO NOT touch: 19/20 (native USB), 43/44 (UART0), 0/45/46 (strapping),
//   26..37 (flash + PSRAM of the N16R8 module).
// That's ~7-10 lines in total: plenty for a TB6612FNG driver (6 of them) — this is
// the fundamental difference from the AI-Thinker, which had almost no free pins.

#elif defined(CAM_MODEL_XIAO_ESP32S3)
// --- Seeed XIAO ESP32S3 Sense ---------------------------------------------
#define CAM_PWDN   -1
#define CAM_RESET  -1
#define CAM_XCLK   10
#define CAM_SIOD   40
#define CAM_SIOC   39
#define CAM_Y9     48
#define CAM_Y8     11
#define CAM_Y7     12
#define CAM_Y6     14
#define CAM_Y5     16
#define CAM_Y4     18
#define CAM_Y3     17
#define CAM_Y2     15
#define CAM_VSYNC  38
#define CAM_HREF   47
#define CAM_PCLK   13
#define CAM_FLASH_LED  -1
#define CAM_STATUS_LED 21    // yellow LED on the board, INVERTED
#define CAM_STATUS_LED_ACTIVE_LOW 1

#elif defined(CAM_MODEL_AI_THINKER)
// --- AI-Thinker ESP32-CAM (plain ESP32) ---------------------------------
// Kept for reference. If you take this board — in platformio.ini the camera envs
// will have board = esp32cam instead of the S3 settings, and nothing else.
#define CAM_PWDN   32
#define CAM_RESET  -1
#define CAM_XCLK    0   // also the boot pin: pulled to GND when flashing
#define CAM_SIOD   26
#define CAM_SIOC   27
#define CAM_Y9     35
#define CAM_Y8     34
#define CAM_Y7     39
#define CAM_Y6     36
#define CAM_Y5     21
#define CAM_Y4     19
#define CAM_Y3     18
#define CAM_Y2      5
#define CAM_VSYNC  25
#define CAM_HREF   23
#define CAM_PCLK   22
#define CAM_FLASH_LED   4   // powerful white LED (also SD DATA1) — VERY bright
#define CAM_STATUS_LED 33   // red LED by the antenna, INVERTED
#define CAM_STATUS_LED_ACTIVE_LOW 1

#else
#error "Выбери модель камеры в include/pins.h (CAM_MODEL_*)"
#endif

// LEDC channel for the flash. The camera itself occupies LEDC_CHANNEL_0 + LEDC_TIMER_0
// (XCLK is generated from them), and ledcSetup(ch,...) in core 2.x binds a channel
// to timer ch/2. So we take channel 7 (timer 3) — guaranteed not the camera's.
#define CH_CAM_FLASH 7
