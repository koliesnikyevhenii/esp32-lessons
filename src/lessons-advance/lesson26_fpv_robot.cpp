// ============================================================================
//  Этап 6. Камера — урок 26: FPV-машинка (видео + пульт в одной странице)
// ----------------------------------------------------------------------------
//  Цель: собрать то, ради чего был весь Этап 6 — открыть страницу, видеть
//  картинку "из глаз робота" и рулить им, глядя в эту картинку (FPV = First
//  Person View). Именно так летают на дронах.
//
//  АРХИТЕКТУРА (важно понять до прошивки!). Плат ДВЕ:
//
//    Браузер ──GET :81/stream──> ESP32-CAM ──── видео
//            ──GET /drive?cmd=─> ESP32-CAM ──MQTT──> RabbitMQ ──MQTT──> ESP32 DevKit
//                                                                        └─> TB6612FNG ─> моторы
//                                              RabbitMQ <──MQTT── DevKit (pitch/roll/yaw/guard)
//
//  ESP32-CAM НЕ крутит моторы. Она делает две вещи: отдаёт видео и
//  ПЕРЕПУБЛИКОВЫВАЕТ нажатия кнопок в тот же топик commands/esp32/drive, который
//  урок 22 уже слушает. Поэтому:
//    - прошивку робота (урок 22) менять НЕ надо — контракт топиков тот же;
//    - бэкенд и React-дашборд менять НЕ надо — они продолжают писать телеметрию;
//    - рулить можно И из дашборда, И из FPV-страницы, хоть одновременно.
//  Почему так, а не "всё на одной плате": у ESP32-S3 свободных GPIO как раз
//  хватает на драйвер моторов (в отличие от старой AI-Thinker), так что вариант
//  "камера сама рулит" здесь ТЕХНИЧЕСКИ возможен. Но на DevKit уже разведены и
//  моторы, и MPU6050, и работает защита по наклону из урока 22 — переносить всё
//  это на камеру значит выбросить готовое и потерять tilt guard. Поэтому по
//  умолчанию две платы; вариант B (одна плата) с картой пинов S3 разобран в
//  documentation/lesson26_fpv_robot.md.
//
//  Что изучаем:
//    - ДВЕ ЗАДАЧИ, ОДИН MQTT-КЛИЕНТ. Обработчик /drive живёт в задаче
//      esp_http_server, а PubSubClient НЕ потокобезопасен. Поэтому обработчик
//      только кладёт команду в очередь FreeRTOS (xQueueSend), а публикует её
//      loop() (xQueueReceive). Это стандартный приём "producer -> queue -> consumer";
//    - ESP32-CAM как ПОЛНОЦЕННОЕ MQTT-устройство: она и publish (телеметрия
//      sensors/esp32cam/fps|rssi), и subscribe (слушает sensors/esp32/guard,
//      чтобы показать на странице, что робот заблокировал себя по наклону);
//    - <device> в контракте топиков — это не константа проекта: у камеры свой
//      device-id esp32cam, и бэкенд пишет её метрики отдельной строкой в БД;
//    - повтор команды пока кнопка нажата: у робота failsafe 700 мс (урок 21),
//      поэтому страница шлёт команду каждые 300 мс и "stop" на отпускание;
//    - управление с клавиатуры (WASD / стрелки) и с телефона (touch).
//
//  Эндпоинты:
//    :80/                 FPV-страница: видео + пульт
//    :80/drive?cmd=NAME   forward|back|left|right|stop -> в очередь -> MQTT
//    :80/stats            JSON: fps, kbps, rssi, mqtt, guard, sent
//    :81/stream           MJPEG-поток (как в уроке 24)
//
//  Перед прошивкой:
//    1) SSID / PASS — твоя сеть 2.4 ГГц;
//    2) MQTT_BROKER — LAN-IP компьютера с docker (НЕ localhost);
//    3) MQTT_USER / MQTT_PASS — не guest (guest пускают только с localhost);
//    4) на DevKit должен быть залит урок 22 (или 21) и он должен быть в сети.
//
//  Сборка/прошивка:
//    pio run -e lesson26_fpv_robot -t upload
//    pio device monitor
//
//  Плата: HW-679 ESP32-S3-CAM (board = esp32-s3-devkitc-1 + memory_type = qio_opi).
//  Пины — include/pins.h (блок CAM_*).
// ============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "pins.h"

// ---- Wi-Fi ----------------------------------------------------------------
const char* SSID = "Vektor_04";
const char* PASS = "uteam2020";

// ---- MQTT-брокер (RabbitMQ + mqtt-плагин) ---------------------------------
const char* MQTT_BROKER = "192.168.0.7";   // <-- LAN-IP своего ПК
const uint16_t MQTT_PORT = 1883;
const char* MQTT_USER = "esp";
const char* MQTT_PASS = "esp-pass";
const char* CLIENT_ID = "esp32cam-lesson26";

// ---- Топики ----------------------------------------------------------------
// Команды шлём РОБОТУ (device = esp32) — тот самый топик из уроков 21/22.
const char* TOPIC_DRIVE = "commands/esp32/drive";
// Свою телеметрию камера публикует под СВОИМ device-id.
const char* TOPIC_FPS   = "sensors/esp32cam/fps";
const char* TOPIC_RSSI  = "sensors/esp32cam/rssi";
// А это метрика РОБОТА: слушаем его защиту по наклону, чтобы подсветить на пульте.
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

// ---- Очередь команд: httpd-задача -> loop() --------------------------------
// PubSubClient однопоточный: публиковать из двух задач нельзя. Обработчик
// HTTP-запроса кладёт команду сюда, а publish делает loop().
typedef struct { char text[12]; } cmd_msg_t;
QueueHandle_t cmdQueue = NULL;

// ---- Состояние -------------------------------------------------------------
volatile float streamFps  = 0;
volatile float streamKbps = 0;
volatile uint32_t cmdSent = 0;      // сколько команд уехало в брокер
volatile int robotGuard   = -1;     // -1 = ещё не слышали, 0 = ок, 1 = робот заблокирован
unsigned long lastTelemetry = 0;
const unsigned long TELEMETRY_INTERVAL = 2000;

// ---- Подсветка и индикатор (на S3-платах пины не стандартизованы -> в pins.h -1) ----
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
//  Камера. Для FPV разрешение специально НЕ максимальное: важнее низкая
//  задержка и стабильные к/с, чем детализация. QVGA/VGA — рабочий выбор.
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

  cfg.pixel_format = PIXFORMAT_JPEG;
  cfg.jpeg_quality = 12;

  if (psramFound()) {
    cfg.frame_size  = FRAMESIZE_VGA;      // 640x480 — компромисс задержки и картинки
    cfg.fb_count    = 2;
    cfg.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    cfg.frame_size  = FRAMESIZE_QVGA;
    cfg.fb_count    = 1;
    cfg.fb_location = CAMERA_FB_IN_DRAM;
  }
  cfg.grab_mode = CAMERA_GRAB_LATEST;      // для FPV критично: только свежий кадр

  esp_err_t err = esp_camera_init(&cfg);
  if (err != ESP_OK) {
    Serial.printf("[CAM] init FAILED: 0x%x (%s)\n", err, esp_err_to_name(err));
    return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
//  FPV-страница. Пульт повторяет команду каждые REPEAT мс, пока кнопка или
//  клавиша удерживается, и шлёт stop на отпускание — это ровно то поведение,
//  под которое в уроке 21 сделан failsafe (нет команд 700 мс -> моторы стоп).
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
const REPEAT=300;                       // мс; на роботе failsafe 700 мс
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
// Ушли со страницы с зажатой кнопкой — робот не должен уехать.
addEventListener('blur',release);

setInterval(async()=>{
  try{
    const s=await (await fetch('/stats')).json();
    document.getElementById('stats').textContent =
      s.fps.toFixed(1)+' к/с, '+s.kbps.toFixed(0)+' кбит/с, RSSI '+s.rssi+
      ' dBm, MQTT: '+(s.mqtt?'ok':'нет')+', команд отправлено: '+s.sent;
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
//  Стрим — как в уроке 24.
// ---------------------------------------------------------------------------
static esp_err_t streamHandler(httpd_req_t* req) {
  esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  char partBuf[72];
  int64_t windowStart = esp_timer_get_time();
  uint32_t windowFrames = 0, windowBytes = 0;

  while (true) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) { res = ESP_FAIL; break; }

    res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res == ESP_OK) {
      size_t hlen = snprintf(partBuf, sizeof(partBuf), STREAM_PART, (unsigned)fb->len);
      res = httpd_resp_send_chunk(req, partBuf, hlen);
    }
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);

    windowFrames++;
    windowBytes += fb->len;
    esp_camera_fb_return(fb);
    if (res != ESP_OK) break;

    int64_t now = esp_timer_get_time();
    if (now - windowStart >= 1000000) {
      float sec = (now - windowStart) / 1000000.0f;
      streamFps  = windowFrames / sec;
      streamKbps = (windowBytes * 8.0f / 1000.0f) / sec;
      windowStart = now; windowFrames = 0; windowBytes = 0;
    }
  }

  streamFps = 0; streamKbps = 0;
  return res;
}

// ---------------------------------------------------------------------------
//  /drive?cmd=... — единственная "рулевая" ручка.
//  ВНИМАНИЕ: здесь мы НЕ публикуем в MQTT. Этот код исполняется в задаче
//  esp_http_server, а PubSubClient не потокобезопасен — publish из двух задач
//  ломает его буфер. Кладём команду в очередь; publish сделает loop().
//
//  Аллоу-лист дублирует бэкендовый (RobotEndpoints) и прошивку робота: лучше
//  отбить мусор здесь, чем гонять его через брокер.
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

  // Таймаут 0: если очередь переполнена (брокер отвалился), команду лучше
  // выбросить, чем задержать HTTP-обработчик и весь стрим.
  bool queued = cmdQueue && xQueueSend(cmdQueue, &msg, 0) == pdTRUE;

  httpd_resp_set_type(req, "text/plain");
  // 202 по смыслу как у бэкенда: команда ПРИНЯТА, но робот — не мы, и он
  // вправе её не исполнить (например при активной защите по наклону).
  httpd_resp_set_status(req, queued ? "202 Accepted" : "503 Service Unavailable");
  return httpd_resp_send(req, queued ? "queued" : "queue full", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t statsHandler(httpd_req_t* req) {
  char json[224];
  snprintf(json, sizeof(json),
           "{\"fps\":%.1f,\"kbps\":%.0f,\"rssi\":%d,\"mqtt\":%d,\"guard\":%d,\"sent\":%u,\"heap\":%u}",
           streamFps, streamKbps, WiFi.RSSI(), mqtt.connected() ? 1 : 0,
           robotGuard, (unsigned)cmdSent, (unsigned)ESP.getFreeHeap());
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
//  MQTT. Камера — такой же участник шины, как робот: и publish, и subscribe.
// ---------------------------------------------------------------------------
void onMessage(char* topic, byte* payload, unsigned int length) {
  String body;
  for (unsigned int i = 0; i < length; i++) body += (char)payload[i];
  body.trim();

  // Единственная подписка — защита по наклону робота из урока 22.
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
    // state(): -2 = сеть, 4 = логин/пароль, 5 = не авторизован.
    Serial.printf("ошибка, state=%d\n", mqtt.state());
    // Никакого delay(3000): здесь он затормозил бы только loop(), но зачем —
    // стрим живёт в своей задаче, а вот телеметрию мы бы задержали.
  }
}

void publishNumber(const char* topic, float value) {
  char buf[16];
  dtostrf(value, 0, 1, buf);
  mqtt.publish(topic, buf);
}

void setup() {
  Serial.begin(115200);
  delay(1500);      // нативному USB нужно время подняться, иначе первые строки теряются
  Serial.println("\n=== Урок 26. FPV-машинка (ESP32-S3-CAM + робот из урока 22) ===");

  statusLed(true);
  flashInit();

  // Очередь на 8 команд: пульт шлёт максимум ~3 в секунду, запас достаточный.
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
  // 1) Держим MQTT живым. Без mqtt.loop() не работают ни подписки, ни keep-alive.
  mqttReconnect();
  mqtt.loop();

  // 2) Разгружаем очередь команд из HTTP-задачи. Публикуем ТОЛЬКО здесь —
  //    один поток, один PubSubClient.
  cmd_msg_t msg;
  while (cmdQueue && xQueueReceive(cmdQueue, &msg, 0) == pdTRUE) {
    if (mqtt.connected() && mqtt.publish(TOPIC_DRIVE, msg.text)) {
      cmdSent++;
      Serial.printf("[MQTT] -> %s = %s\n", TOPIC_DRIVE, msg.text);
    } else {
      Serial.printf("[MQTT] команда '%s' ПОТЕРЯНА (брокер недоступен)\n", msg.text);
    }
  }

  // 3) Своя телеметрия раз в 2 с: те же голые числа, тот же конвейер, что у
  //    робота, только device = esp32cam. Бэкенд менять не пришлось.
  unsigned long now = millis();
  if (now - lastTelemetry >= TELEMETRY_INTERVAL) {
    lastTelemetry = now;
    if (mqtt.connected()) {
      publishNumber(TOPIC_FPS,  streamFps);
      publishNumber(TOPIC_RSSI, WiFi.RSSI());
    }
  }

  delay(10);   // не крутим loop() вхолостую на полной скорости
}
