#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LiquidCrystal.h>
#include <DHT.h>

const char* SSID = "Vektor_04";
const char* PASS = "uteam2020";

// Все выводы — на одной стороне платы (левая колонка DevKit V1).
// GPIO12 (strapping-пин) не используем; LDR на GPIO35 = ADC1, работает при активном Wi-Fi.
//              RS  E   D4  D5  D6  D7
LiquidCrystal lcd(13, 14, 25, 26, 27, 32);

#define DHTPIN 33
#define DHTTYPE DHT11
#define LDR 35

DHT dht(DHTPIN, DHTTYPE);
WebServer server(80);

bool page = false;   // false = датчики, true = IP

void handleData() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  int light = analogRead(LDR);
  String json = "{";
  json += "\"temperature\":" + String(t, 1) + ",";
  json += "\"humidity\":"    + String(h, 0) + ",";
  json += "\"light\":"       + String(light);
  json += "}";
  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);
  lcd.begin(16, 2);
  dht.begin();

  WiFi.begin(SSID, PASS);
  while (WiFi.status() != WL_CONNECTED) { delay(500); }
  Serial.println(WiFi.localIP());

  server.on("/data", handleData);
  server.begin();
}

void loop() {
  server.handleClient();

  float t = dht.readTemperature();
  float h = dht.readHumidity();
  int light = analogRead(LDR);

  lcd.clear();
  if (!page) {
    // страница 1: T / H / свет
    lcd.setCursor(0, 0);
    lcd.print("T:"); lcd.print(t, 1);
    lcd.print(" H:"); lcd.print(h, 0); lcd.print("%");
    lcd.setCursor(0, 1);
    lcd.print("Light: "); lcd.print(light);
  } else {
    // страница 2: IP-адрес
    lcd.setCursor(0, 0);
    lcd.print("IP address:");
    lcd.setCursor(0, 1);
    lcd.print(WiFi.localIP());   // IPAddress печатается напрямую
  }
  page = !page;

  delay(2000);
}