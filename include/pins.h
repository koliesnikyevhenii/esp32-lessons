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

// Analog sensors (ADC1, доступен при активном Wi-Fi)
#define POT     34     // потенциометр: средний вывод -> GPIO34
#define LDR     35     // модуль фоторезистора: A0 -> GPIO35
#define DHT_PIN 16     // DHT11 DATA -> GPIO16

// I2C (уроки 20 и 22, MPU6050/6500). Шина ESP32 переназначаемая — берём свободные
// пины, чтобы не пересекаться с RGB (21/22) и остальной периферией из этого файла.
// ВАЖНО: в уроке 22 датчик и моторы работают ОДНОВРЕМЕННО, поэтому эта пара пинов
// и группа MOTOR_* ниже не должны пересекаться (SCL 17 <-> PWMB 33 когда-то были
// наоборот — 33 оказался удобнее под ШИМ мотора B).
#define MPU_SDA 32     // MPU6050 SDA -> GPIO32
#define MPU_SCL 17     // MPU6050 SCL -> GPIO17

// TB6612FNG — драйвер двух моторов (уроки 21 и 22). A = левый мотор, B = правый.
// STBY драйвера заведён на 3V3 (всегда включён), поэтому GPIO под него не нужен.
// Взяты свободные пины, не пересекающиеся с периферией выше в этом файле.
#define MOTOR_AIN1 13  // мотор A, направление 1 orange
#define MOTOR_AIN2 14  // мотор A, направление 2 yellow
#define MOTOR_PWMA 25  // мотор A, скорость (ШИМ) green
#define MOTOR_BIN1 26  // мотор B, направление 1 brown
#define MOTOR_BIN2 27  // мотор B, направление 2 violet
#define MOTOR_PWMB 33  // мотор B, скорость (ШИМ) white

// ===========================================================================
//  КАМЕРА — уроки 23-26. ВНИМАНИЕ: ЭТО ДРУГАЯ ПЛАТА И ДРУГОЙ ЧИП!
// ---------------------------------------------------------------------------
//  Всё выше относится к ESP32-WROOM-32 DevKit (board = esp32dev). Камерные уроки
//  собираются под ESP32-S3 (board = esp32-s3-devkitc-1, см. platformio.ini) —
//  это физически отдельный модуль: DevKit остаётся роботом (урок 22), плата с
//  камерой — «глазами». Обе живут на машинке и общаются через MQTT, не проводами.
//
//  Номера ниже мы НЕ выбираем: шлейф сенсора разведён на плате жёстко. Поэтому
//  часть чисел совпадает с именами выше (например CAM_Y7 = 18 и LED_YELLOW = 18)
//  — конфликта нет, это разные платы, разные чипы и разные прошивки.
//
//  Пины задают ПЛАТУ, а не сенсор. Какой именно сенсор на шлейфе — отдельный
//  вопрос, и он меняет код: на нашей плате это GC2145 (PID 0x2145), у него НЕТ
//  аппаратного JPEG, поэтому уроки 23-26 снимают RGB565 и жмут кадр программно
//  (frame2jpg). Проверяется одним запуском lesson_check_cam.
//
//  ВЫБЕРИ РОВНО ОДНУ модель. Готовые карты для остальных плат лежат в ядре:
//  packages/framework-arduinoespressif32/libraries/ESP32/examples/Camera/
//  CameraWebServer/camera_pins.h
// ===========================================================================
#define CAM_MODEL_ESP32S3_CAM        // HW-679 / Goouuu ESP32-S3-CAM (пинаут как ESP32-S3-EYE)
// #define CAM_MODEL_XIAO_ESP32S3    // Seeed XIAO ESP32S3 Sense
// #define CAM_MODEL_AI_THINKER      // классическая ESP32-CAM — ОБЫЧНЫЙ ESP32, не S3!

#if defined(CAM_MODEL_ESP32S3_CAM)
// --- HW-679 ESP32-S3-CAM (ESP32-S3-WROOM-1 N16R8) --------------------------
// Та же карта, что CAMERA_MODEL_ESP32S3_EYE в ядре: эти платы разведены одинаково.
#define CAM_PWDN   -1   // линии power-down нет
#define CAM_RESET  -1   // линии reset нет
#define CAM_XCLK   15   // такт для сенсора (20 МГц, генерируется через LEDC)
#define CAM_SIOD    4   // SCCB (это I2C сенсора) data
#define CAM_SIOC    5   // SCCB clock

// 8-битная параллельная шина данных D0..D7 (в даташитах сенсоров — Y2..Y9)
#define CAM_Y9     16   // D7
#define CAM_Y8     17   // D6
#define CAM_Y7     18   // D5
#define CAM_Y6     12   // D4
#define CAM_Y5     10   // D3
#define CAM_Y4      8   // D2
#define CAM_Y3      9   // D1
#define CAM_Y2     11   // D0
#define CAM_VSYNC   6   // строб кадра
#define CAM_HREF    7   // строб строки
#define CAM_PCLK   13   // пиксельный клок

// Подсветка/индикация. У AI-Thinker белый LED жёстко на GPIO4, а у S3-плат
// разводка гуляет от партии к партии, поэтому по умолчанию ВЫКЛЮЧЕНО (-1):
// весь код подсветки в уроках обёрнут в #if и просто не компилируется.
// Найти свой пин: pio run -e lesson_check_cam -t upload -> режим «охота за LED»
// перебирает безопасные GPIO и печатает, какой мигает прямо сейчас.
#define CAM_FLASH_LED  -1
#define CAM_STATUS_LED -1
#define CAM_STATUS_LED_ACTIVE_LOW 1   // у большинства плат LOW = горит

// Свободные GPIO на HW-679 (вариант B урока 26 — моторы на самой камере):
//   1, 2, 3, 14, 21, 47, 48  — свободны;
//   38, 39, 40               — microSD (свободны, если карта не нужна);
//   НЕ трогать: 19/20 (нативный USB), 43/44 (UART0), 0/45/46 (strapping),
//   26..37 (flash + PSRAM модуля N16R8).
// Итого ~7-10 линий: на драйвер TB6612FNG (6 штук) хватает с запасом — это
// принципиальное отличие от AI-Thinker, где свободных пинов почти не было.

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
#define CAM_STATUS_LED 21    // жёлтый LED на плате, ИНВЕРТИРОВАН
#define CAM_STATUS_LED_ACTIVE_LOW 1

#elif defined(CAM_MODEL_AI_THINKER)
// --- AI-Thinker ESP32-CAM (обычный ESP32) ---------------------------------
// Оставлено для справки. Если возьмёшь эту плату — в platformio.ini у камерных
// env вместо S3-настроек будет board = esp32cam, и больше ничего.
#define CAM_PWDN   32
#define CAM_RESET  -1
#define CAM_XCLK    0   // он же boot-пин: при прошивке тянется в GND
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
#define CAM_FLASH_LED   4   // мощный белый LED (он же SD DATA1) — ОЧЕНЬ яркий
#define CAM_STATUS_LED 33   // красный LED у антенны, ИНВЕРТИРОВАН
#define CAM_STATUS_LED_ACTIVE_LOW 1

#else
#error "Выбери модель камеры в include/pins.h (CAM_MODEL_*)"
#endif

// LEDC-канал для подсветки. Камера сама занимает LEDC_CHANNEL_0 + LEDC_TIMER_0
// (из них генерируется XCLK), а ledcSetup(ch,...) в ядре 2.x привязывает канал
// к таймеру ch/2. Поэтому берём канал 7 (таймер 3) — гарантированно не камерин.
#define CH_CAM_FLASH 7
