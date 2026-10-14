// ============================================================================
//  Stage 6. Camera — lesson 26: FPV car (video + remote on one page)
// ----------------------------------------------------------------------------
//  Goal: build what all of Stage 6 was for — open a page, see the picture
//  "through the robot's eyes" and drive it while looking at that picture (FPV = First
//  Person View). This is exactly how drones are flown.
//
//  ARCHITECTURE (important to understand before flashing!). There are TWO boards:
//
//    Browser ──GET :81/stream──> ESP32-CAM ──── video
//            ──GET /drive?cmd=─> ESP32-CAM ──MQTT──> RabbitMQ ──MQTT──> ESP32 DevKit
//                                                                        └─> TB6612FNG ─> motors
//                                              RabbitMQ <──MQTT── DevKit (pitch/roll/yaw/guard)
//
//  The ESP32-CAM does NOT drive the motors. It does two things: serves the video and
//  REPUBLISHES button presses into the same topic commands/esp32/drive that
//  lesson 22 already listens to. Therefore:
//    - the robot firmware (lesson 22) does NOT need changes — the topic contract is the same;
//    - the backend and the React dashboard do NOT need changes — they keep recording telemetry;
//    - you can drive BOTH from the dashboard AND from the FPV page, even at the same time.
//  Why this way and not "everything on one board": the ESP32-S3 has just enough free GPIOs
//  for a motor driver (unlike the old AI-Thinker), so the "camera drives by itself"
//  option is TECHNICALLY possible here. But the DevKit already has the motors and the
//  MPU6050 wired, and runs lesson 22's tilt guard — moving all of that onto the camera
//  means throwing away what's done and losing the tilt guard. So by default
//  there are two boards; variant B (one board) with the S3 pin map is covered in
//  documentation/lesson26_fpv_robot.md.
//
//  What we study:
//    - TWO TASKS, ONE MQTT CLIENT. The /drive handler lives in the
//      esp_http_server task, and PubSubClient is NOT thread-safe. So the handler
//      only puts the command into a FreeRTOS queue (xQueueSend), and loop() publishes
//      it (xQueueReceive). This is the standard "producer -> queue -> consumer" pattern;
//    - the ESP32-CAM as a FULL MQTT device: it both publishes (telemetry
//      sensors/esp32cam/fps|rssi) and subscribes (listens to sensors/esp32/guard,
//      to show on the page that the robot has blocked itself due to tilt);
//    - <device> in the topic contract is not a project constant: the camera has its own
//      device-id esp32cam, and the backend writes its metrics as a separate row in the DB;
//    - repeating the command while a button is held: the robot has a 700 ms failsafe (lesson 21),
//      so the page sends the command every 300 ms and "stop" on release;
//    - control from the keyboard (WASD / arrows) and from a phone (touch).
//
//  Endpoints:
//    :80/                 FPV page: video + remote
//    :80/drive?cmd=NAME   forward|back|left|right|stop -> queue -> MQTT
//    :80/stats            JSON: fps, kbps, rssi, mqtt, guard, sent
//    :81/stream           MJPEG stream (as in lesson 24)
//
//  Before flashing:
//    1) SSID / PASS — your 2.4 GHz network;
//    2) MQTT_BROKER — LAN IP of the computer running docker (NOT localhost);
//    3) MQTT_USER / MQTT_PASS — not guest (guest is only allowed from localhost);
//    4) the DevKit must be flashed with lesson 22 (or 21) and be on the network.
//
//  Build/flash:
//    pio run -e lesson26_fpv_robot -t upload
//    pio device monitor
//
//  Board: HW-679 ESP32-S3-CAM (board = esp32-s3-devkitc-1 + memory_type = qio_opi).
//  Pins — include/pins.h (CAM_* block).
// ============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "esp_camera.h"
#include "img_converters.h"   // frame2jpg(): software RGB565 -> JPEG compression
#include "esp_http_server.h"
#include "esp_timer.h"
#include "pins.h"

// ---- Wi-Fi ----------------------------------------------------------------
const char* SSID = "Vektor_04";
const char* PASS = "uteam2020";

// Frame compression is done in software (GC2145 has no hardware JPEG, see lesson 23):
// the frame2jpg scale is 0..100, HIGHER = better picture and more bytes.
const int JPEG_QUALITY = 80;

// ---- MQTT broker (RabbitMQ + mqtt plugin) ---------------------------------
const char* MQTT_BROKER = "192.168.0.7";   // <-- your PC's LAN IP
const uint16_t MQTT_PORT = 1883;
const char* MQTT_USER = "esp";
const char* MQTT_PASS = "esp-pass";
const char* CLIENT_ID = "esp32cam-lesson26";

// ---- Topics ----------------------------------------------------------------
// Commands go to the ROBOT (device = esp32) — the very topic from lessons 21/22.
const char* TOPIC_DRIVE = "commands/esp32/drive";
// The camera publishes its own telemetry under ITS OWN device-id.
const char* TOPIC_FPS   = "sensors/esp32cam/fps";
const char* TOPIC_RSSI  = "sensors/esp32cam/rssi";
// And this is a ROBOT metric: we listen to its tilt guard to highlight it on the remote.
const char* TOPIC_GUARD = "sensors/esp32/guard";

// ---- MJPEG -----------------------------------------------------------------
#define PART_BOUNDARY "esp32camframe"
static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY     = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART         = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t webServer    = NULL;
httpd_handle_t streamServer = NULL;

WiFiClient   net;
PubSubClient mqtt(net);

// ---- Command queue: httpd task -> loop() -----------------------------------
// PubSubClient is single-threaded: publishing from two tasks is not allowed. The
// HTTP request handler puts the command here, and loop() does the publish.
typedef struct { char text[12]; } cmd_msg_t;
QueueHandle_t cmdQueue = NULL;

// ---- State -----------------------------------------------------------------
volatile float streamFps  = 0;
volatile float streamKbps = 0;
// Frame time broken down by stage (ms, averaged over a second). Needed so we
// don't have to guess why the video lags: capture, compression and sending slow down
// for completely different reasons and are fixed differently too.
volatile float streamMsGrab = 0;   // waiting for a ready frame from the sensor
volatile float streamMsJpeg = 0;   // frame2jpg() — CPU compression
volatile float streamMsSend = 0;   // socket write = bottlenecked on Wi-Fi
volatile uint32_t cmdSent = 0;      // how many commands went out to the broker
volatile int robotGuard   = -1;     // -1 = not heard yet, 0 = ok, 1 = robot blocked
unsigned long lastTelemetry = 0;
const unsigned long TELEMETRY_INTERVAL = 2000;

// ---- Flash LED and indicator (pins aren't standardised on S3 boards -> -1 in pins.h) ----
void flashInit() {
#if CAM_FLASH_LED >= 0
  ledcSetup(CH_CAM_FLASH, 5000, 8);
  ledcAttachPin(CAM_FLASH_LED, CH_CAM_FLASH);
  ledcWrite(CH_CAM_FLASH, 0);
#endif
}

void statusLed(bool on) {
#if CAM_STATUS_LED >= 0
  pinMode(CAM_STATUS_LED, OUTPUT);
  digitalWrite(CAM_STATUS_LED, CAM_STATUS_LED_ACTIVE_LOW ? !on : on);
#else
  (void)on;
#endif
}

// ---------------------------------------------------------------------------
//  Camera. For FPV the resolution is deliberately NOT the maximum: low latency
//  and steady fps matter more than detail. QVGA/VGA is the practical choice.
// ---------------------------------------------------------------------------
bool initCamera() {
  camera_config_t cfg = {};

  cfg.pin_pwdn     = CAM_PWDN;
  cfg.pin_reset    = CAM_RESET;
  cfg.pin_xclk     = CAM_XCLK;
  cfg.pin_sccb_sda = CAM_SIOD;
  cfg.pin_sccb_scl = CAM_SIOC;
  cfg.pin_d7 = CAM_Y9;  cfg.pin_d6 = CAM_Y8;  cfg.pin_d5 = CAM_Y7;  cfg.pin_d4 = CAM_Y6;
  cfg.pin_d3 = CAM_Y5;  cfg.pin_d2 = CAM_Y4;  cfg.pin_d1 = CAM_Y3;  cfg.pin_d0 = CAM_Y2;
  cfg.pin_vsync = CAM_VSYNC;
  cfg.pin_href  = CAM_HREF;
  cfg.pin_pclk  = CAM_PCLK;

  cfg.xclk_freq_hz = 20000000;
  cfg.ledc_timer   = LEDC_TIMER_0;
  cfg.ledc_channel = LEDC_CHANNEL_0;

  cfg.pixel_format = PIXFORMAT_RGB565;   // GC2145 doesn't output JPEG — we compress in software
  cfg.jpeg_quality = 12;                 // unused with RGB565

  if (psramFound()) {
    // For FPV latency matters more than detail, and our compression is done by the CPU
    // (see lesson 23) — hence QVGA, not VGA.
    cfg.frame_size  = FRAMESIZE_QVGA;     // 320x240
    cfg.fb_count    = 2;
    cfg.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    cfg.frame_size  = FRAMESIZE_QVGA;
    cfg.fb_count    = 1;
    cfg.fb_location = CAMERA_FB_IN_DRAM;
  }
  cfg.grab_mode = CAMERA_GRAB_LATEST;      // critical for FPV: only the freshest frame

  esp_err_t err = esp_camera_init(&cfg);
  if (err != ESP_OK) {
    Serial.printf("[CAM] init FAILED: 0x%x (%s)\n", err, esp_err_to_name(err));
    return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
//  FPV page. The remote repeats the command every REPEAT ms while a button or
//  key is held, and sends stop on release — exactly the behaviour that
//  lesson 21's failsafe was built for (no commands for 700 ms -> motors stop).
// ---------------------------------------------------------------------------
static const char PAGE_HTML[] = R"HTML(
<!doctype html><html lang="ru"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Урок 26 — FPV-машинка</title>
<style>
 body{font-family:system-ui,sans-serif;background:#111;color:#eee;margin:0;padding:14px}
 h1{font-size:18px;margin:0 0 10px}
 img{width:100%;max-width:640px;border:1px solid #444;background:#000;display:block}
 #pad{display:grid;grid-template-columns:repeat(3,72px);grid-template-rows:repeat(3,60px);
      gap:8px;margin:14px 0;touch-action:none;user-select:none}
 #pad button{font-size:20px;border-radius:10px;border:1px solid #555;background:#222;color:#eee}
 #pad button:active,#pad button.on{background:#2a6;color:#000}
 .f{grid-column:2;grid-row:1} .l{grid-column:1;grid-row:2}
 .s{grid-column:2;grid-row:2} .r{grid-column:3;grid-row:2} .b{grid-column:2;grid-row:3}
 #stats{color:#8cf;font-variant-numeric:tabular-nums}
 #guard{font-weight:600}
 .warn{color:#f66}
</style></head><body>
<h1>Урок 26 — FPV: видео + пульт</h1>
<img id="cam" alt="поток">
<div id="pad">
  <button class="f" data-c="forward">&#9650;</button>
  <button class="l" data-c="left">&#9664;</button>
  <button class="s" data-c="stop">&#9632;</button>
  <button class="r" data-c="right">&#9654;</button>
  <button class="b" data-c="back">&#9660;</button>
</div>
<div id="stats">-</div>
<div id="guard"></div>
<p style="color:#888;font-size:13px">Клавиши: W/A/S/D или стрелки. Пока кнопка нажата,
команда уходит каждые 300 мс; на отпускание уходит stop.</p>
<script>
const REPEAT=300;                       // ms; the robot's failsafe is 700 ms
let timer=null, current=null;
const cam=document.getElementById('cam');
function open_(){ cam.src='http://'+location.hostname+':81/stream?t='+Date.now(); }
function send(c){ fetch('/drive?cmd='+c).catch(()=>{}); }

function start(c){
  if(current===c) return;
  stopRepeat();
  current=c; send(c);
  if(c!=='stop') timer=setInterval(()=>send(c),REPEAT);
  mark(c);
}
function stopRepeat(){ if(timer){clearInterval(timer);timer=null;} }
function release(){ stopRepeat(); if(current&&current!=='stop') send('stop'); current=null; mark(null); }
function mark(c){ document.querySelectorAll('#pad button').forEach(b=>
  b.classList.toggle('on', b.dataset.c===c)); }

document.querySelectorAll('#pad button').forEach(b=>{
  const c=b.dataset.c;
  b.addEventListener('pointerdown',e=>{e.preventDefault();start(c);});
  b.addEventListener('pointerup',release);
  b.addEventListener('pointerleave',()=>{ if(current===c) release(); });
  b.addEventListener('pointercancel',release);
});
const KEYS={ArrowUp:'forward',ArrowDown:'back',ArrowLeft:'left',ArrowRight:'right',
            w:'forward',s:'back',a:'left',d:'right',' ':'stop'};
addEventListener('keydown',e=>{ const c=KEYS[e.key]; if(c){e.preventDefault();start(c);} });
addEventListener('keyup',  e=>{ if(KEYS[e.key]) release(); });
// Left the page with a button held — the robot must not drive off.
addEventListener('blur',release);

setInterval(async()=>{
  try{
    const s=await (await fetch('/stats')).json();
    document.getElementById('stats').textContent =
      s.fps.toFixed(1)+' к/с, '+s.kbps.toFixed(0)+' кбит/с, RSSI '+s.rssi+
      ' dBm, MQTT: '+(s.mqtt?'ok':'нет')+', команд отправлено: '+s.sent+
      ' | съёмка '+s.grab+' мс, сжатие '+s.jpeg+' мс, отправка '+s.send+' мс';
    const g=document.getElementById('guard');
    g.textContent = s.guard===1 ? 'РОБОТ ЗАБЛОКИРОВАН ПО НАКЛОНУ — команды движения игнорируются'
                  : s.guard===0 ? '' : 'состояние робота ещё не получено';
    g.className = s.guard===1 ? 'warn' : '';
  }catch(e){}
},1000);
open_();
</script></body></html>
)HTML";

static esp_err_t indexHandler(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  return httpd_resp_send(req, PAGE_HTML, HTTPD_RESP_USE_STRLEN);
}

// ---------------------------------------------------------------------------
//  Stream — as in lesson 24.
// ---------------------------------------------------------------------------
static esp_err_t streamHandler(httpd_req_t* req) {
  esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  char partBuf[72];
  int64_t windowStart = esp_timer_get_time();
  uint32_t windowFrames = 0, windowBytes = 0;
  int64_t accGrab = 0, accJpeg = 0, accSend = 0;

  while (true) {
    int64_t t0 = esp_timer_get_time();
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) { res = ESP_FAIL; break; }
    int64_t t1 = esp_timer_get_time();

    // The GC2145 sensor outputs raw RGB565, it has no ready JPEG — we compress it ourselves
    // (the same frame2jpg as in lesson 23). This is the main fps limiter.
    uint8_t* jpg = nullptr;
    size_t   jpg_len = 0;
    bool ok = frame2jpg(fb, JPEG_QUALITY, &jpg, &jpg_len);
    esp_camera_fb_return(fb);          // the driver no longer needs the raw frame
    if (!ok) { res = ESP_FAIL; break; }
    int64_t t2 = esp_timer_get_time();

    res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res == ESP_OK) {
      size_t hlen = snprintf(partBuf, sizeof(partBuf), STREAM_PART, (unsigned)jpg_len);
      res = httpd_resp_send_chunk(req, partBuf, hlen);
    }
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char*)jpg, jpg_len);
    int64_t t3 = esp_timer_get_time();

    windowFrames++;
    windowBytes += jpg_len;
    accGrab += t1 - t0;                // waited for the sensor
    accJpeg += t2 - t1;                // burned CPU
    accSend += t3 - t2;                // waited for the network
    free(jpg);                         // the frame2jpg buffer is ours
    if (res != ESP_OK) break;

    int64_t now = esp_timer_get_time();
    if (now - windowStart >= 1000000) {
      float sec = (now - windowStart) / 1000000.0f;
      streamFps  = windowFrames / sec;
      streamKbps = (windowBytes * 8.0f / 1000.0f) / sec;
      streamMsGrab = accGrab / 1000.0f / windowFrames;
      streamMsJpeg = accJpeg / 1000.0f / windowFrames;
      streamMsSend = accSend / 1000.0f / windowFrames;
      Serial.printf("[STREAM] %.1f к/с | съёмка %.0f мс | сжатие %.0f мс | отправка %.0f мс | кадр %u Б\n",
                    streamFps, streamMsGrab, streamMsJpeg, streamMsSend,
                    (unsigned)(windowBytes / windowFrames));
      windowStart = now; windowFrames = 0; windowBytes = 0;
      accGrab = accJpeg = accSend = 0;
    }
  }

  streamFps = 0; streamKbps = 0;
  streamMsGrab = streamMsJpeg = streamMsSend = 0;
  return res;
}

// ---------------------------------------------------------------------------
//  /drive?cmd=... — the only "steering" endpoint.
//  NOTE: we do NOT publish to MQTT here. This code runs in the
//  esp_http_server task, and PubSubClient is not thread-safe — publishing from two
//  tasks corrupts its buffer. We put the command in a queue; loop() does the publish.
//
//  The allow-list duplicates the backend's (RobotEndpoints) and the robot firmware's:
//  better to reject garbage here than to push it through the broker.
// ---------------------------------------------------------------------------
static bool isAllowed(const char* cmd) {
  return !strcmp(cmd, "forward") || !strcmp(cmd, "back") ||
         !strcmp(cmd, "left")    || !strcmp(cmd, "right") ||
         !strcmp(cmd, "stop");
}

static esp_err_t driveHandler(httpd_req_t* req) {
  char query[64];
  cmd_msg_t msg = {};

  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
      httpd_query_key_value(query, "cmd", msg.text, sizeof(msg.text)) != ESP_OK) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "нужен cmd");
    return ESP_FAIL;
  }

  if (!isAllowed(msg.text)) {
    Serial.printf("[DRIVE] отбито: '%s'\n", msg.text);
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "неизвестная команда");
    return ESP_FAIL;
  }

  // Timeout 0: if the queue is full (the broker dropped off), it's better to throw
  // the command away than to hold up the HTTP handler and the whole stream.
  bool queued = cmdQueue && xQueueSend(cmdQueue, &msg, 0) == pdTRUE;

  httpd_resp_set_type(req, "text/plain");
  // 202 means the same as on the backend: the command was ACCEPTED, but the robot isn't us,
  // and it is entitled not to execute it (e.g. while the tilt guard is active).
  httpd_resp_set_status(req, queued ? "202 Accepted" : "503 Service Unavailable");
  return httpd_resp_send(req, queued ? "queued" : "queue full", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t statsHandler(httpd_req_t* req) {
  char json[320];
  snprintf(json, sizeof(json),
           "{\"fps\":%.1f,\"kbps\":%.0f,\"rssi\":%d,\"mqtt\":%d,\"guard\":%d,\"sent\":%u,\"heap\":%u,"
           "\"grab\":%.0f,\"jpeg\":%.0f,\"send\":%.0f}",
           streamFps, streamKbps, WiFi.RSSI(), mqtt.connected() ? 1 : 0,
           robotGuard, (unsigned)cmdSent, (unsigned)ESP.getFreeHeap(),
           streamMsGrab, streamMsJpeg, streamMsSend);
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

void startServers() {
  httpd_config_t web = HTTPD_DEFAULT_CONFIG();
  web.server_port = 80;
  web.ctrl_port   = 32768;
  web.max_uri_handlers = 8;

  httpd_uri_t uriIndex = { "/",      HTTP_GET, indexHandler, NULL };
  httpd_uri_t uriDrive = { "/drive", HTTP_GET, driveHandler, NULL };
  httpd_uri_t uriStats = { "/stats", HTTP_GET, statsHandler, NULL };

  if (httpd_start(&webServer, &web) == ESP_OK) {
    httpd_register_uri_handler(webServer, &uriIndex);
    httpd_register_uri_handler(webServer, &uriDrive);
    httpd_register_uri_handler(webServer, &uriStats);
    Serial.println("[WEB] FPV-страница на :80");
  }

  httpd_config_t stream = HTTPD_DEFAULT_CONFIG();
  stream.server_port = 81;
  stream.ctrl_port   = 32769;
  httpd_uri_t uriStream = { "/stream", HTTP_GET, streamHandler, NULL };

  if (httpd_start(&streamServer, &stream) == ESP_OK) {
    httpd_register_uri_handler(streamServer, &uriStream);
    Serial.println("[WEB] поток на :81/stream");
  }
}

// ---------------------------------------------------------------------------
//  MQTT. The camera is the same kind of bus participant as the robot: both publish and subscribe.
// ---------------------------------------------------------------------------
void onMessage(char* topic, byte* payload, unsigned int length) {
  String body;
  for (unsigned int i = 0; i < length; i++) body += (char)payload[i];
  body.trim();

  // The only subscription — the robot's tilt guard from lesson 22.
  if (!strcmp(topic, TOPIC_GUARD)) {
    robotGuard = body.toInt();
    Serial.printf("[MQTT] <- guard = %d\n", robotGuard);
  }
}

void mqttReconnect() {
  if (mqtt.connected()) return;
  Serial.print("[MQTT] Подключение к брокеру... ");
  if (mqtt.connect(CLIENT_ID, MQTT_USER, MQTT_PASS)) {
    Serial.println("OK");
    mqtt.subscribe(TOPIC_GUARD, 1);
  } else {
    // state(): -2 = network, 4 = login/password, 5 = not authorized.
    Serial.printf("ошибка, state=%d\n", mqtt.state());
    // No delay(3000): here it would only slow down loop(), but why bother —
    // the stream lives in its own task, whereas the telemetry would get delayed.
  }
}

void publishNumber(const char* topic, float value) {
  char buf[16];
  dtostrf(value, 0, 1, buf);
  mqtt.publish(topic, buf);
}

void setup() {
  Serial.begin(115200);
  delay(1500);      // native USB needs time to come up, otherwise the first lines are lost
  Serial.println("\n=== Урок 26. FPV-машинка (ESP32-S3-CAM + робот из урока 22) ===");

  statusLed(true);
  flashInit();

  // Queue for 8 commands: the remote sends at most ~3 per second, plenty of headroom.
  cmdQueue = xQueueCreate(8, sizeof(cmd_msg_t));

  if (!initCamera()) {
    Serial.println("[CAM] камера не поднялась — прогони lesson_check_cam.");
    return;
  }
  sensor_t* s = esp_camera_sensor_get();
  if (s) { s->set_brightness(s, 1); s->set_saturation(s, 1); }

  WiFi.begin(SSID, PASS);
  Serial.print("[WiFi] Подключение");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.printf("\n[WiFi] IP: %s (RSSI %d dBm)\n",
                WiFi.localIP().toString().c_str(), WiFi.RSSI());
  WiFi.setSleep(false);

  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setCallback(onMessage);

  startServers();
  Serial.printf("[WEB] FPV: http://%s/\n", WiFi.localIP().toString().c_str());

  statusLed(false);
}

void loop() {
  // 1) Keep MQTT alive. Without mqtt.loop() neither subscriptions nor keep-alive work.
  mqttReconnect();
  mqtt.loop();

  // 2) Drain the command queue from the HTTP task. Publish ONLY here —
  //    one thread, one PubSubClient.
  cmd_msg_t msg;
  while (cmdQueue && xQueueReceive(cmdQueue, &msg, 0) == pdTRUE) {
    if (mqtt.connected() && mqtt.publish(TOPIC_DRIVE, msg.text)) {
      cmdSent++;
      Serial.printf("[MQTT] -> %s = %s\n", TOPIC_DRIVE, msg.text);
    } else {
      Serial.printf("[MQTT] команда '%s' ПОТЕРЯНА (брокер недоступен)\n", msg.text);
    }
  }

  // 3) Own telemetry every 2 s: the same bare numbers, the same pipeline as the
  //    robot, only device = esp32cam. The backend didn't need changes.
  unsigned long now = millis();
  if (now - lastTelemetry >= TELEMETRY_INTERVAL) {
    lastTelemetry = now;
    if (mqtt.connected()) {
      publishNumber(TOPIC_FPS,  streamFps);
      publishNumber(TOPIC_RSSI, WiFi.RSSI());
    }
  }

  delay(10);   // don't spin loop() idle at full speed
}
