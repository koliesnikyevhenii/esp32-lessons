#include <Arduino.h>
#include <LiquidCrystal.h>

LiquidCrystal lcd(13, 12, 27, 26, 25, 33);

String text = "   Hello Subscribers! Welcome to my channel!   ";

void setup() {
    lcd.begin(16, 2);

    lcd.setCursor(0, 1);
    lcd.print("Time:");
}

void loop() {
    static int pos = 0;

    lcd.setCursor(0, 0);
    lcd.print(text.substring(pos, pos + 16));

    lcd.setCursor(6, 1);
    lcd.print(millis() / 1000);
    lcd.print("   "); // затираем старые цифры

    pos++;
    if (pos > text.length() - 16)
        pos = 0;

    delay(400);
}