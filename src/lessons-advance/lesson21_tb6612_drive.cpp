// ============================================================================
//  Stage 3. Robot — lesson 21: motor control via TB6612FNG over MQTT
// ----------------------------------------------------------------------------
//  Goal: receive drive commands from the browser and turn two motors — forward,
//  back, left, right, stop. No autopilot, pure manual control.
//
//   Browser --HTTP--> ASP.NET --AMQP--> RabbitMQ --MQTT(1883)--> ESP32 --> TB6612FNG --> motors
//
//  This is the REVERSE direction to lessons 19–20: there the ESP32 published
//  telemetry, here the ESP32 SUBSCRIBES to commands. .NET publishes the routing key
//  commands.esp32.drive, and the RabbitMQ MQTT plugin delivers it as the topic
//  commands/esp32/drive (body is one word: forward|back|left|right|stop).
//
//  TB6612FNG is a dual motor driver. Per motor: two direction inputs
//  (xIN1/xIN2) and a PWM speed input (PWMx):
//     IN1=1, IN2=0 -> forward;  IN1=0, IN2=1 -> back;  IN1=IN2 -> stop.
//  The STBY input enables the driver — in this circuit it is wired to 3V3 (always ON).
//
//  SAFETY (failsafe): if commands stop arriving (tab closed, Wi-Fi lost) for
//  longer than FAILSAFE_MS — we cut the motors ourselves so the robot doesn't "run away".
//  That's why the browser repeats the command every ~300 ms while a button is held.
//
//  Library: knolleary/PubSubClient (see platformio.ini, env lesson21_motors).
// ============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "pins.h"

// ---- Wi-Fi ----------------------------------------------------------------
const char* SSID = "Koleso";
const char* PASS = "12345zxc";

// ---- MQTT broker (RabbitMQ + mqtt plugin) ---------------------------------
const char* MQTT_BROKER = "192.168.1.164";   // <-- replace with your PC's IP
const uint16_t MQTT_PORT = 1883;
const char* MQTT_USER = "esp";
const char* MQTT_PASS = "esp-pass";

const char* DEVICE_ID = "esp32";
const char* CLIENT_ID = "esp32-lesson21";

// Drive command topic. .NET -> commands.esp32.drive -> here.
const char* TOPIC_DRIVE = "commands/esp32/drive";

// ---- Motors (TB6612FNG) ----------------------------------------------------
// A = left motor, B = right. The driver's STBY is on 3V3.
const int SPEED = 200;                    // 0..255 — PWM speed (analogWrite)
const unsigned long FAILSAFE_MS = 700;    // no commands for longer -> stop

unsigned long lastCommandMs = 0;
bool moving = false;

// ---- Network objects -------------------------------------------------------
WiFiClient   net;
PubSubClient mqtt(net);

// One motor: dir>0 — forward, dir<0 — back, dir==0 — stop.
void motor(int in1, int in2, int pwm, int dir) {
  digitalWrite(in1, dir > 0 ? HIGH : LOW);
  digitalWrite(in2, dir < 0 ? HIGH : LOW);
  analogWrite(pwm, dir == 0 ? 0 : SPEED);
}

void driveStop() {
  motor(MOTOR_AIN1, MOTOR_AIN2, MOTOR_PWMA, 0);
  motor(MOTOR_BIN1, MOTOR_BIN2, MOTOR_PWMB, 0);
  moving = false;
}

// Tank-style control: left/right — spin in place (motors in opposite directions).
// If a motor turns the wrong way — swap its wires or the dir signs.
void applyCommand(const String& cmd) {
  int left, right;
  if      (cmd == "forward") { left = +1; right = +1; }
  else if (cmd == "back")    { left = -1; right = -1; }
  else if (cmd == "left")    { left = -1; right = +1; }
  else if (cmd == "right")   { left = +1; right = -1; }
  else if (cmd == "stop")    { driveStop(); return; }
  else { Serial.printf("[DRIVE] неизвестная команда '%s'\n", cmd.c_str()); return; }

  motor(MOTOR_AIN1, MOTOR_AIN2, MOTOR_PWMA, left);
  motor(MOTOR_BIN1, MOTOR_BIN2, MOTOR_PWMB, right);
  moving = true;
}

// ---------------------------------------------------------------------------
//  SUBSCRIBE: callback for an incoming command. payload is not null-terminated -> length.
// ---------------------------------------------------------------------------
void onMessage(char* topic, byte* payload, unsigned int length) {
  String cmd;
  for (unsigned int i = 0; i < length; i++) cmd += (char)payload[i];
  cmd.trim();

  Serial.printf("[MQTT] <- %s = %s\n", topic, cmd.c_str());

  lastCommandMs = millis();   // saw a command -> reset the failsafe timer
  applyCommand(cmd);
}

void reconnect() {
  while (!mqtt.connected()) {
    Serial.print("[MQTT] Подключение к брокеру... ");
    if (mqtt.connect(CLIENT_ID, MQTT_USER, MQTT_PASS)) {
      Serial.println("OK");
      // QoS 1 — commands matter more than telemetry, better "deliver at least once".
      mqtt.subscribe(TOPIC_DRIVE, 1);
      Serial.printf("[MQTT] Подписка на '%s' (QoS 1)\n", TOPIC_DRIVE);
    } else {
      Serial.printf("ошибка, state=%d, повтор через 3 c\n", mqtt.state());
      delay(3000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Motor pins are outputs; cut them right away (important: the robot stands still at startup).
  int pins[] = { MOTOR_AIN1, MOTOR_AIN2, MOTOR_PWMA, MOTOR_BIN1, MOTOR_BIN2, MOTOR_PWMB };
  for (int p : pins) pinMode(p, OUTPUT);
  driveStop();

  WiFi.begin(SSID, PASS);
  Serial.print("[WiFi] Подключение");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.printf("\n[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());

  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setCallback(onMessage);
}

void loop() {
  if (!mqtt.connected()) reconnect();
  mqtt.loop();   // MANDATORY: subscriptions don't work without loop()

  // Failsafe: we are moving, but no commands for a long time -> stop.
  if (moving && millis() - lastCommandMs > FAILSAFE_MS) {
    Serial.println("[DRIVE] failsafe: команд нет, стоп");
    driveStop();
  }
}
