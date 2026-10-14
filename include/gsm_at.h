#pragma once
#include <Arduino.h>

// Shared helpers for SIM800L AT commands (lessons in src/gsm-sim800l-lessons/).
// The modem sits on Serial1; same pins as in lesson_uart_gsm_manual_command.

#ifndef SIM_RX
#define SIM_RX 27  // ESP32 RX  ← GSM TX
#endif
#ifndef SIM_TX
#define SIM_TX 26  // ESP32 TX  → GSM RX
#endif
#define SIM_BAUD 9600  // SIM800L autobaud locks on most reliably at 9600

// Final result codes arrive on a line of their own. Matching a bare "OK"/"ERROR"
// anywhere would also hit an SMS body in an AT+CMGL listing ("all OK") and cut
// the reply short, losing every message after it.
inline bool atHasOk(const String& r) { return r.indexOf("\r\nOK\r\n") >= 0; }
inline bool atHasError(const String& r) {
  return r.indexOf("\r\nERROR\r\n") >= 0 || r.indexOf("+CMS ERROR:") >= 0
      || r.indexOf("+CME ERROR:") >= 0;
}

// Sends a command and accumulates the reply until expect, an error or the timeout.
// expect = "OK" means the final OK line; anything else (e.g. ">") is a plain substring.
// The reply is returned whole — parsing it is the caller's job.
inline String sendAT(const char* cmd, const char* expect = "OK", uint32_t timeoutMs = 1000) {
  bool wantOk = strcmp(expect, "OK") == 0;
  while (Serial1.available()) Serial1.read();  // discard old garbage
  Serial1.print(cmd);
  Serial1.print("\r");
  String r;
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    while (Serial1.available()) r += (char)Serial1.read();
    if ((wantOk ? atHasOk(r) : r.indexOf(expect) >= 0) || atHasError(r)) break;
  }
  return r;
}

// true if the modem replied OK. On failure prints the command and the reply.
inline bool atOk(const char* cmd, uint32_t timeoutMs = 1000) {
  String r = sendAT(cmd, "OK", timeoutMs);
  bool ok = atHasOk(r);
  if (!ok) {
    r.trim();
    Serial.printf("⚠️ %s → %s\n", cmd, r.length() ? r.c_str() : "тишина");
  }
  return ok;
}

// Brings up the UART and waits until the modem answers AT. ATE0 turns echo off,
// otherwise every command we send would come back as a line in simReadLine().
inline bool simBegin() {
  Serial1.begin(SIM_BAUD, SERIAL_8N1, SIM_RX, SIM_TX);
  delay(100);
  for (int i = 0; i < 10; i++) {
    if (atHasOk(sendAT("AT", "OK", 300))) {
      atOk("ATE0");
      return true;
    }
    delay(200);
  }
  return false;
}

// Waits for network registration: +CREG: x,1 (home) or x,5 (roaming).
inline bool waitNetwork(uint32_t timeoutMs = 30000) {
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    String r = sendAT("AT+CREG?");
    if (r.indexOf(",1") >= 0 || r.indexOf(",5") >= 0) return true;
    delay(1000);
  }
  return false;
}

// Non-blocking line read from the modem. true — a non-empty line is complete.
// Empty lines (the modem likes \r\n\r\n) are skipped.
inline bool simReadLine(String& out) {
  static String buf;
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    if (c == '\n') {
      buf.trim();
      if (buf.length()) {
        out = buf;
        buf = "";
        return true;
      }
    } else {
      buf += c;
    }
  }
  return false;
}

// The n-th (zero-based) quoted string: quoted("+CMT: \"+380...\",\"\",...", 0) → "+380...".
inline String quoted(const String& s, int n) {
  int from = 0;
  for (int i = 0; ; i++) {
    int a = s.indexOf('"', from);
    if (a < 0) return "";
    int b = s.indexOf('"', a + 1);
    if (b < 0) return "";
    if (i == n) return s.substring(a + 1, b);
    from = b + 1;
  }
}

// Polling instead of +CMT: reads unread SMS via AT+CMGL, passes each one to
// handler, then deletes exactly that one by index. Works when CNMI stores SMS in
// memory (AT+CNMI=2,1,...).
// Mode 1 in AT+CMGL leaves the status alone: an SMS that did not make it into this
// reply (timeout, cut-off listing) stays UNREAD and is picked up on the next poll.
// "Mark all read, then AT+CMGD=1,1" would have deleted it unseen.
typedef void (*SmsHandler)(const String& from, const String& text);

inline void pollSms(SmsHandler handler) {
  String r = sendAT("AT+CMGL=\"REC UNREAD\",1", "OK", 5000);
  int pos = 0;
  while ((pos = r.indexOf("+CMGL:", pos)) >= 0) {
    int eol  = r.indexOf('\n', pos);
    int next = r.indexOf("\r\n", eol + 1);
    if (eol < 0 || next < 0) break;
    String header = r.substring(pos, eol);            // +CMGL: 1,"REC UNREAD","+380...",...
    String text   = r.substring(eol + 1, next);
    text.trim();
    int index = header.substring(6).toInt();          // right after "+CMGL:"
    handler(quoted(header, 1), text);                 // 0 = status, 1 = number
    atOk(("AT+CMGD=" + String(index)).c_str());       // delete only what was handled
    pos = next;
  }
}

// Text-mode SMS (needs AT+CMGF=1), GSM charset — ASCII only. Cyrillic needs UCS2,
// see lesson_uart_gsm_manual_command. Blocks until the network accepts the message
// (usually 3-10 s, up to 60 s).
inline bool sendSMS(const String& to, const String& text) {
  String cmd = "AT+CMGS=\"" + to + "\"";
  if (sendAT(cmd.c_str(), ">", 5000).indexOf('>') < 0) {
    Serial.println("⚠️ CMGS: нет приглашения >");
    return false;
  }
  Serial1.print(text);
  Serial1.write(0x1A);                                // Ctrl+Z = send
  String r;
  uint32_t t0 = millis();
  while (millis() - t0 < 60000) {
    while (Serial1.available()) r += (char)Serial1.read();
    if (r.indexOf("+CMGS:") >= 0 && atHasOk(r)) return true;  // +CMGS: <mr> = accepted
    if (atHasError(r)) break;
  }
  r.trim();
  Serial.printf("⚠️ SMS на %s не ушло: %s\n", to.c_str(), r.length() ? r.c_str() : "таймаут");
  return false;
}

// AT+CSQ → "+CSQ: <rssi>,<ber>"; rssi 0..31 maps to -113..-51 dBm.
// 99 (no signal / unknown) and a missing reply both return 0.
inline int signalDbm() {
  String r = sendAT("AT+CSQ");
  int p = r.indexOf("+CSQ:");
  if (p < 0) return 0;
  int rssi = r.substring(p + 5).toInt();
  return rssi == 99 ? 0 : -113 + 2 * rssi;
}

// AT+CBC → "+CBC: <charging>,<percent>,<mV>": module supply voltage in mV, 0 if no reply.
inline int batteryMv() {
  String r = sendAT("AT+CBC");
  int p = r.indexOf("+CBC:");
  if (p < 0) return 0;
  int c1 = r.indexOf(',', p);
  int c2 = c1 < 0 ? -1 : r.indexOf(',', c1 + 1);
  return c2 < 0 ? 0 : r.substring(c2 + 1).toInt();
}
