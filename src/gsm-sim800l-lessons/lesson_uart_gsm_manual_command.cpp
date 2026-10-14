#include <Arduino.h>
#define MY_PHONE  "+380994285796"
#define SIM_RX 27  // ESP32 RX  ← GSM TX
#define SIM_TX 26  // ESP32 TX  → GSM RX

// Строка "777" в мониторе отправляет это SMS на MY_PHONE (вместо пересылки в модуль).

#define SMS_TEXT  "С Днём учителя! Тест GSM прошёл успешно."
#define SMS_TRIGGER "777"

// SIM800L по умолчанию в режиме автобода: скорость он "ловит" по первым
// символам "AT". Поэтому перебираем типичные скорости и шлём AT несколько раз.
const uint32_t BAUDS[] = {9600, 115200, 57600, 38400, 19200};

// Шлёт "AT\r" и ждёт "OK" в ответе. Всё, что пришло, печатаем — полезно видеть мусор.
bool probe(uint32_t baud) {
    Serial1.end();
    Serial1.begin(baud, SERIAL_8N1, SIM_RX, SIM_TX);
    delay(50);
    while (Serial1.available()) Serial1.read();

    String resp;
    for (int i = 0; i < 3; i++) {
        Serial1.print("AT\r");
        uint32_t t0 = millis();
        while (millis() - t0 < 300) {
            while (Serial1.available()) resp += (char)Serial1.read();
        }
        if (resp.indexOf("OK") >= 0) break;
    }

    Serial.printf("  %6u бод: ", baud);
    if (resp.length() == 0) {
        Serial.println("тишина");
    } else {
        resp.replace("\r", "\\r");
        resp.replace("\n", "\\n");
        Serial.println(resp);
    }
    return resp.indexOf("OK") >= 0;
}

// Ждёт в ответе модуля строку expect (или ERROR), всё пришедшее печатает в монитор.
bool waitFor(const char* expect, uint32_t timeoutMs) {
    String resp;
    uint32_t t0 = millis();
    while (millis() - t0 < timeoutMs) {
        while (Serial1.available()) {
            char c = Serial1.read();
            Serial.write(c);
            resp += c;
        }
        if (resp.indexOf(expect) >= 0) return true;
        if (resp.indexOf("ERROR") >= 0) return false;
    }
    return false;
}

bool atCmd(const String& cmd, const char* expect = "OK", uint32_t timeoutMs = 2000) {
    Serial1.print(cmd + "\r");
    return waitFor(expect, timeoutMs);
}

// UTF-8 → UCS2 в hex ("С" → "0421"). Кириллица в SMS идёт только так:
// в режиме "GSM" модуль понимает лишь латиницу.
String toUcs2Hex(const char* s) {
    String out;
    char buf[5];
    const uint8_t* p = (const uint8_t*)s;
    while (*p) {
        uint16_t cp;
        if (*p < 0x80) {
            cp = *p++;
        } else if ((*p & 0xE0) == 0xC0) {
            cp = ((p[0] & 0x1F) << 6) | (p[1] & 0x3F);
            p += 2;
        } else {
            cp = ((p[0] & 0x0F) << 12) | ((p[1] & 0x3F) << 6) | (p[2] & 0x3F);
            p += 3;
        }
        snprintf(buf, sizeof(buf), "%04X", cp);
        out += buf;
    }
    return out;
}

void sendSms(const char* phone, const char* text) {
    Serial.printf("\n>>> Отправляю SMS на %s...\n", phone);

    // UCS2: и номер, и текст передаются hex-строкой UTF-16; dcs=8 в CSMP = UCS2.
    bool ok = atCmd("AT+CMGF=1")
           && atCmd("AT+CSCS=\"UCS2\"")
           && atCmd("AT+CSMP=17,167,0,8")
           && atCmd("AT+CMGS=\"" + toUcs2Hex(phone) + "\"", ">", 5000);

    if (ok) {
        Serial1.print(toUcs2Hex(text));
        Serial1.write(0x1A);
        // Ответ сети при слабом сигнале может идти до минуты.
        ok = waitFor("+CMGS:", 60000);
    } else {
        Serial1.write(0x1B);  // ESC: выйти из ожидания текста, если застряли на ">"
    }

    // Возвращаем режим для ручного ввода латиницей.
    atCmd("AT+CSCS=\"GSM\"");
    atCmd("AT+CSMP=17,167,0,0");

    Serial.println(ok ? "\n>>> SMS отправлено" : "\n>>> SMS НЕ отправлено, см. ответ модуля выше");
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    // После подачи питания модулю нужно ~3 с на загрузку, прежде чем он ответит.
    delay(3000);

    Serial.println("Ищу SIM800L...");
    uint32_t found = 0;
    for (uint32_t b : BAUDS) {
        if (probe(b)) { found = b; break; }
    }

    if (found) {
        Serial.printf("Модуль отвечает на %u бод.\n", found);
    } else {
        // Оставляем 9600 — на ней автобод срабатывает надёжнее всего.
        Serial1.end();
        Serial1.begin(9600, SERIAL_8N1, SIM_RX, SIM_TX);
        Serial.println("Ответа нет ни на одной скорости: проверь питание (3.7-4.2 В, 2 А),");
        Serial.println("общий GND и перекрёст TX/RX. Мост оставлен на 9600.");
    }

    Serial.println("Bridge ready. '~' = Ctrl+Z, '" SMS_TRIGGER "' = тестовое SMS");
}

// Строка из монитора копится целиком, чтобы успеть узнать в ней SMS_TRIGGER
// до того, как она уйдёт в модуль. send_on_enter и так шлёт строку разом.
String line;

void loop() {

    // ПК → ESP32 → GSM
    while (Serial.available()) {
        char c = Serial.read();
        line += c;
        if (c != '\n') continue;

        String cmd = line;
        cmd.trim();
        if (cmd == SMS_TRIGGER) {
            sendSms(MY_PHONE, SMS_TEXT);
        } else {
            // Для отправки SMS '~' превращаем в Ctrl+Z
            for (char ch : line) Serial1.write(ch == '~' ? 0x1A : ch);
        }
        line = "";
    }

    // GSM → ESP32 → ПК
    while (Serial1.available()) {
        Serial.write(Serial1.read());
    }
}
