#include <Arduino.h>
#include <WiFi.h>

const char* SSID = "Vektor_04";
const char* PASS = "uteam2020";

void setup() {
  Serial.begin(115200);
  WiFi.begin(SSID, PASS);
  Serial.print("Подключение");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("IP-адрес: ");
  Serial.println(WiFi.localIP());
}

void loop() {}