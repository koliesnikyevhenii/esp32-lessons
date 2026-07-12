#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DHT.h>

const char* SSID = "Vektor_04";
const char* PASS = "uteam2020";

#define DHTPIN 33
#define DHTTYPE DHT11
#define RELAY1 32

DHT dht(DHTPIN, DHTTYPE);
WebServer server(80);

void handleRoot() {
  float t = dht.readTemperature();
  String html = "<h1>ESP32</h1>";
  html += "<p>Температура: " + String(t, 1) + " C</p>";
  html += "<a href='/on'>Включить реле</a><br><a href='/off'>Выключить реле</a>";
  server.send(200, "text/html; charset=utf-8", html);
}

void setup() {
  Serial.begin(115200);
  pinMode(RELAY1, OUTPUT);
  digitalWrite(RELAY1, HIGH);
  dht.begin();

  WiFi.begin(SSID, PASS);
  while (WiFi.status() != WL_CONNECTED) { delay(500); }
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.on("/on",  []() { digitalWrite(RELAY1, LOW);  server.send(200, "text/plain", "ON"); });
  server.on("/off", []() { digitalWrite(RELAY1, HIGH); server.send(200, "text/plain", "OFF"); });
  server.begin();
}

void loop() {
  server.handleClient();
}