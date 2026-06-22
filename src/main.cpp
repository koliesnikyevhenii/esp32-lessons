#include <Arduino.h>

#define LED_RED 23
#define LED_YELLOW 18
#define LED_GREEN 19
#define LED_RED2 02

// Track the time of the last event
unsigned long previousMillis = 0; 

// Set the desired interval (e.g., 3000 ms)
const long interval = 3000;   
int phase = 0;

#define PIN_R 22
#define PIN_G 21
#define PIN_B 15

// LEDC PWM channels (ESP32 core 2.x is channel-based)
#define CH_R 0
#define CH_G 1
#define CH_B 2

#define CH 0

#define BUZZER 05

#define BUTTON 04

int lastReading = HIGH;
unsigned long lastChange = 0;
bool ledOn = false;

int melody[] = {262, 294, 330, 349, 392, 440, 494, 523}; // до ре ми фа соль ля 

void setColor(int r, int g, int b) {
  ledcWrite(CH_R, r);
  ledcWrite(CH_G, g);
  ledcWrite(CH_B, b);
}

void lesson2()
{
    // Capture the current time
  unsigned long currentMillis = millis(); 
   // Check if the interval has passed
  if (phase == 0 && currentMillis - previousMillis >= interval) {
    digitalWrite(LED_RED, HIGH);
    // Save the last time you blinked the LED
    previousMillis = currentMillis;
    phase = 1;
    digitalWrite(LED_GREEN, LOW);
   
  }

  if (phase == 1 && currentMillis - previousMillis >= interval) {
    digitalWrite(LED_YELLOW, HIGH);
    // Save the last time you blinked the LED
    previousMillis = currentMillis;
    phase = 2;
      digitalWrite(LED_RED, LOW);
  }

    if (phase == 2 && currentMillis - previousMillis >= interval) {
    digitalWrite(LED_GREEN, HIGH);
    // Save the last time you blinked the LED
    previousMillis = currentMillis;
    phase = 0;
     digitalWrite(LED_YELLOW, LOW);
  }
}

void lesson3()
{
  setColor(255, 0, 0);     // красный
  delay(700);

  setColor(0, 255, 0);     // зелёный
  delay(700);

  setColor(0, 0, 255);     // синий
  delay(700);

  setColor(255, 255, 0);   // жёлтый
  delay(700);

  setColor(0, 255, 255);   // голубой
  delay(700);

  setColor(255, 0, 255);   // фиолетовый
  delay(700);

  setColor(255, 255, 255); // белый
  delay(700);

  setColor(0, 0, 0);       // выключить
  delay(700);
}
void lesson4()
{
    digitalWrite(BUZZER, HIGH); delay(200);
    digitalWrite(BUZZER, LOW);  delay(800);
}
void lesson5()
{
   for (int i = 0; i < 8; i++) {
    ledcWriteTone(CH, melody[i]);
    delay(300);
  }
  ledcWriteTone(CH, 0); // тишина
  delay(1500);
}
void lesson6()
{
  int reading = digitalRead(BUTTON);
  if (reading != lastReading && millis() - lastChange > 50) { // антидребезг 50 мс
    lastChange = millis();
    if (reading == LOW) {
      Serial.println("Кнопка нажата");
    }
    lastReading = reading;
  }
}
void lesson7()
{
   int reading = digitalRead(BUTTON);
  if (reading == LOW && lastReading == HIGH && millis() - lastChange > 10) {
    ledOn = !ledOn;                              // переключаем состояние
    digitalWrite(LED_RED2, ledOn ? HIGH : LOW);
    lastChange = millis();
  }
  lastReading = reading;
}
void setup() {
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  // ESP32 core 2.x LEDC API: configure each channel (5000 Hz, 8-bit
  // resolution -> values 0..255), then bind a pin to it. setColor()
  // then writes by channel.
  ledcSetup(CH_R, 5000, 8);
  ledcSetup(CH_G, 5000, 8);
  ledcSetup(CH_B, 5000, 8);

  ledcAttachPin(PIN_R, CH_R);
  ledcAttachPin(PIN_G, CH_G);
  ledcAttachPin(PIN_B, CH_B);

  pinMode(BUZZER, OUTPUT);

  ledcSetup(CH, 2000, 8);
  ledcAttachPin(BUZZER, CH);

    pinMode(LED_RED2, OUTPUT);

Serial.begin(115200);
  pinMode(BUTTON, INPUT_PULLUP); // не нажата = HIGH, нажата = LOW
}

void loop() {
// lesson2();
// lesson3();  
//   lesson5();
//   lesson6();
lesson7();
}

