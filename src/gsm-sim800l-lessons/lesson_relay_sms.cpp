#include <Arduino.h>
#include "gsm_at.h"

#define RELAY 32                                // same GPIO as RELAY_2CH_1 in lesson 15
const char* ALLOWED[] = { "+380XXXXXXXXX" };    // whitelist, international format as in +CMGL

bool relayOn = false;
bool modemReady = false;

bool isAllowed(const String& num) {
  for (auto a : ALLOWED) if (num == a) return true;
  return false;
}

void setRelay(bool on) {
  relayOn = on;
  digitalWrite(RELAY, on ? LOW : HIGH);         // active LOW
}

void onSms(const String& from, const String& textRaw) {
  Serial.printf("📩 %s: %s\n", from.c_str(), textRaw.c_str());
  if (!isAllowed(from)) { Serial.println("⛔ чужой номер — игнор"); return; }

  String cmd = textRaw;
  cmd.trim();
  cmd.toUpperCase();

  if (cmd == "RELAY ON")       { setRelay(true);  sendSMS(from, "Relay: ON"); }
  else if (cmd == "RELAY OFF") { setRelay(false); sendSMS(from, "Relay: OFF"); }
  else if (cmd == "STATUS") {
    String s = "Relay: " + String(relayOn ? "ON" : "OFF") +
               "\nRSSI: " + String(signalDbm()) + " dBm" +
               "\nVBAT: " + String(batteryMv()) + " mV" +
               "\nUptime: " + String(millis() / 1000) + " s";
    sendSMS(from, s);
  }
  else sendSMS(from, "Unknown. Use: RELAY ON | RELAY OFF | STATUS");
}

void setup() {
  Serial.begin(115200);
  // Level first, then output: otherwise the pin starts LOW and an active-LOW
  // relay clicks on for a moment at every boot.
  setRelay(false);
  pinMode(RELAY, OUTPUT);
  if (!simBegin() || !waitNetwork()) { Serial.println("❌ модем/сеть"); return; }
  atOk("AT+CMGF=1");
  atOk("AT+CNMI=2,1,0,0,0");   // method B: the modem stores SMS, we poll
  modemReady = true;
  Serial.println("✅ Жду команды");
}

void loop() {
  if (!modemReady) return;     // no modem — nothing to poll, AT calls would just time out
  static uint32_t t = 0;
  if (millis() - t > 3000) {   // poll every 3 s
    t = millis();
    pollSms(onSms);
  }
}
