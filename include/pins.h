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

// I2C (урок 20, MPU6050). Шина ESP32 переназначаемая — берём свободные пины,
// чтобы не пересекаться с RGB (21/22) и остальной периферией из этого файла.
#define MPU_SDA 32     // MPU6050 SDA -> GPIO32
#define MPU_SCL 33     // MPU6050 SCL -> GPIO33
