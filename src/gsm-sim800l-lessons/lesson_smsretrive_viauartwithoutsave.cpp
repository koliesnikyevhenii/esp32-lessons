#include <Arduino.h>
#include "gsm_at.h"

void setup() {
  Serial.begin(115200);
  if (!simBegin() || !waitNetwork()) { Serial.println("❌ модем/сеть"); return; }
  atOk("AT+CMGF=1");
  atOk("AT+CNMI=2,2,0,0,0");     // incoming SMS → straight to UART
  atOk("AT+CMGD=1,4");           // clear old SMS from SIM memory
  Serial.println("✅ Жду SMS...");
}

void loop() {
  static String from;
  String line;
  if (simReadLine(line)) {
    if (line.startsWith("+CMT:")) {
      from = quoted(line, 0);                 // sender's number
    } else if (from.length()) {               // the next line is the text
      Serial.printf("📩 От %s: %s\n", from.c_str(), line.c_str());
      from = "";
    }
  }
}
