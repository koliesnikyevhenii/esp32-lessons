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
