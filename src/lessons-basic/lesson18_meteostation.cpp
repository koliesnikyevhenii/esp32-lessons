#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LiquidCrystal.h>
#include <DHT.h>

const char* SSID = "Vektor_04";
const char* PASS = "uteam2020";

// All pins are on one side of the board (left column of the DevKit V1).
// GPIO12 (a strapping pin) is not used; LDR on GPIO35 = ADC1, works while Wi-Fi is active.
//              RS  E   D4  D5  D6  D7
LiquidCrystal lcd(13, 14, 25, 26, 27, 32);

#define DHTPIN 33
#define DHTTYPE DHT11
#define LDR 35

DHT dht(DHTPIN, DHTTYPE);
WebServer server(80);

bool page = false;   // false = sensors, true = IP

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
    // page 1: T / H / light
    lcd.setCursor(0, 0);
    lcd.print("T:"); lcd.print(t, 1);
    lcd.print(" H:"); lcd.print(h, 0); lcd.print("%");
    lcd.setCursor(0, 1);
    lcd.print("Light: "); lcd.print(light);
  } else {
    // page 2: IP address
    lcd.setCursor(0, 0);
    lcd.print("IP address:");
    lcd.setCursor(0, 1);
    lcd.print(WiFi.localIP());   // IPAddress prints directly
  }
  page = !page;

  delay(2000);
}