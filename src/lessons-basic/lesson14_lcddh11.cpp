#include <Arduino.h>
#include <LiquidCrystal.h>
#include <DHT.h>

LiquidCrystal lcd(13, 12, 27, 26, 25, 33);

String text = "   Hello Subscribers! Welcome to my channel!   ";

#define DHTPIN 32
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

unsigned long lastDhtRead = 0;
float t = 0, h = 0;

void setup() {
    lcd.begin(16, 2);
    dht.begin();
    lcd.setCursor(0, 1);
    lcd.print("Time:");
      Serial.begin(115200);
}

void loop() {
    // Бегущая строка каждые 250 мс
    static int pos = 0;
    lcd.setCursor(0, 0);
    lcd.print(text.substring(pos, pos + 16));
    pos = (pos + 1) % (text.length() - 15);

    // Читаем DHT раз в 2 секунды
    if (millis() - lastDhtRead >= 2000) {
        lastDhtRead = millis();

        t = dht.readTemperature();
        h = dht.readHumidity();
    }
    Serial.print("Temperature: ");
    Serial.print(t);    
    Serial.print(" Humidity: ");
    Serial.print(h);
    Serial.println();

    // Отображаем последние полученные значения
    lcd.setCursor(0, 1);
    if (!isnan(t) && !isnan(h)) {
        lcd.print("T:");
        lcd.print(t, 1);
        lcd.print((char)223);
        lcd.print(" H:");
        lcd.print(h, 0);
        lcd.print("%  ");
    }

    delay(500);
}