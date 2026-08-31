// ============================================================================
//  Этап 6. Камера — урок 24: видеопоток MJPEG в браузере
// ----------------------------------------------------------------------------
//  Цель: превратить "фотоаппарат" из урока 23 в ВИДЕО. Никакого H.264 и WebRTC:
//  ESP32 отдаёт поток MJPEG — бесконечную последовательность обычных JPEG-кадров
//  в одном HTTP-ответе. Браузер умеет показывать такой ответ в теге <img> сам,
//  без единой строчки видео-кода.
//
//     Браузер --GET :81/stream--> ESP32-CAM ...кадр...кадр...кадр... (не закрывая ответ)
//
//  Что изучаем:
//    - multipart/x-mixed-replace — "заменяй предыдущую часть следующей". Это и есть
//      весь секрет MJPEG: один ответ, много частей, каждая часть = целый JPEG;
//    - chunked-передача: httpd_resp_send_chunk() в цикле, ответ не завершается;
//    - почему WebServer из уроков 17 и 23 здесь не годится и мы берём
//      esp_http_server: обработчик стрима НИКОГДА не возвращает управление, а
//      синхронный WebServer живёт внутри loop() — он бы заблокировал всё разом;
//    - ДВА сервера на двух портах (80 — страница и кнопки, 81 — стрим): пока
//      воркер занят бесконечным стримом, обычные запросы на этот же сервер
//      будут ждать в очереди. Ровно так сделано в примере CameraWebServer;
//    - цена разрешения: сравни к/с на QVGA / VGA / SVGA. На нашей плате стоит
//      сенсор GC2145 без аппаратного JPEG, поэтому кадр жмёт процессор
//      (frame2jpg, как в уроке 23) — упрёмся мы именно в сжатие, а не в Wi-Fi;
//    - CAMERA_GRAB_LATEST — отдавать самый свежий кадр, чтобы видео не отставало.
//
//  Эндпоинты:
//    :80/            HTML: <img> со стримом, к/с, выбор разрешения, вспышка
//    :80/stats       JSON: {"fps":..,"kbps":..,"clients":..,"rssi":..,"heap":..}
//    :80/size?v=N    разрешение (5=QVGA, 8=VGA, 9=SVGA, 13=UXGA)
//    :80/flash?v=0..255  вспышка
//    :81/stream      сам MJPEG-поток
//
//  Сборка/прошивка:
//    pio run -e lesson24_cam_stream -t upload
//    pio device monitor        # напечатает IP -> открой http://IP/ в браузере
//
//  Плата: HW-679 ESP32-S3-CAM (board = esp32-s3-devkitc-1 + memory_type = qio_opi).
//  Пины — include/pins.h (блок CAM_*).
// ============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include "esp_camera.h"
#include "img_converters.h"   // frame2jpg(): программное сжатие RGB565 -> JPEG
#include "esp_http_server.h"
#include "esp_timer.h"
#include "pins.h"

// ---- Wi-Fi ----------------------------------------------------------------
const char* SSID = "Vektor_04";
const char* PASS = "uteam2020";

// GC2145 (сенсор на нашей плате) аппаратного JPEG не умеет, кадр жмётся
// программно через frame2jpg — а у неё шкала 0..100 и БОЛЬШЕ = лучше.
const int JPEG_QUALITY = 80;

// ---- Границы MJPEG ---------------------------------------------------------
// Разделитель может быть любой строкой, которой точно нет в бинарных данных.
#define PART_BOUNDARY "esp32camframe"
static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY     = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART         = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// ---- Два сервера: страница и поток ----------------------------------------
httpd_handle_t webServer    = NULL;   // порт 80
httpd_handle_t streamServer = NULL;   // порт 81

// ---- Статистика (её считает сам стрим-обработчик) --------------------------
volatile float    streamFps   = 0;    // кадров в секунду
volatile float    streamKbps  = 0;    // килобит в секунду
volatile int      streamClients = 0;  // сколько стримов открыто прямо сейчас

// ---------------------------------------------------------------------------
//  Подсветка и индикатор (на S3-платах пины не стандартизованы -> в pins.h -1).
// ---------------------------------------------------------------------------
void flashInit() {
#if CAM_FLASH_LED >= 0
  ledcSetup(CH_CAM_FLASH, 5000, 8);
  ledcAttachPin(CAM_FLASH_LED, CH_CAM_FLASH);
  ledcWrite(CH_CAM_FLASH, 0);
#endif
}

void flashSet(int value) {
#if CAM_FLASH_LED >= 0
  ledcWrite(CH_CAM_FLASH, constrain(value, 0, 255));
#else
  (void)value;
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
//  Инициализация камеры. Отличие от урока 23 — grab_mode.
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

  cfg.pixel_format = PIXFORMAT_RGB565;   // сырой кадр: сжимать будем сами
  cfg.jpeg_quality = 12;                 // при RGB565 не используется

  if (psramFound()) {
    // На OV2640 здесь стояла бы VGA. У нас сжатие делает процессор, а его цена
    // растёт с площадью кадра — на VGA выходит 1-2 к/с. Стартуем с QVGA,
    // разрешение можно поднять кнопкой на странице и сравнить к/с.
    cfg.frame_size  = FRAMESIZE_QVGA;
    cfg.fb_count    = 2;
    cfg.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    cfg.frame_size  = FRAMESIZE_QVGA;
    cfg.fb_count    = 1;
    cfg.fb_location = CAMERA_FB_IN_DRAM;
  }

  // ВОТ ЭТО важно для видео: с CAMERA_GRAB_WHEN_EMPTY браузер получает кадры из
  // очереди буферов и картинка отстаёт от реальности на fb_count кадров.
  // CAMERA_GRAB_LATEST всегда берёт самый свежий — задержка меньше.
  cfg.grab_mode = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&cfg);
  if (err != ESP_OK) {
    Serial.printf("[CAM] init FAILED: 0x%x (%s)\n", err, esp_err_to_name(err));
    return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
//  СТРАНИЦА. Стрим лежит на другом порту, поэтому адрес собираем в браузере из
//  location.hostname — так один и тот же HTML работает с любым IP.
// ---------------------------------------------------------------------------
static const char PAGE_HTML[] = R"HTML(
<!doctype html><html lang="ru"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Урок 24 — видеопоток ESP32-CAM</title>
<style>
 body{font-family:system-ui,sans-serif;background:#111;color:#eee;margin:0;padding:16px}
 h1{font-size:18px;margin:0 0 12px}
 img{max-width:100%;border:1px solid #444;background:#000;display:block}
 .row{display:flex;gap:10px;align-items:center;flex-wrap:wrap;margin:12px 0}
 button{padding:7px 12px;font-size:15px;cursor:pointer}
 #stats{font-variant-numeric:tabular-nums;color:#8cf}
</style></head><body>
<h1>Урок 24 — MJPEG-поток</h1>
<img id="cam" alt="поток">
<div class="row">
  разрешение:
  <button onclick="size(5)">QVGA 320x240</button>
  <button onclick="size(8)">VGA 640x480</button>
  <button onclick="size(9)">SVGA 800x600</button>
  <button onclick="size(13)">UXGA 1600x1200</button>
</div>
<div class="row">вспышка: <input type="range" min="0" max="255" value="0" oninput="flash(this.value)"></div>
<div class="row" id="stats">-</div>
<script>
const cam=document.getElementById('cam');
// Один и тот же URL стрима браузер может взять из кеша — добавляем метку времени.
function open_(){ cam.src='http://'+location.hostname+':81/stream?t='+Date.now(); }
function size(v){ fetch('/size?v='+v).then(open_); }   // после смены размера поток надо переоткрыть
function flash(v){ fetch('/flash?v='+v); }
setInterval(async()=>{
  try{
    const s=await (await fetch('/stats')).json();
    document.getElementById('stats').textContent =
      s.fps.toFixed(1)+' к/с, '+s.kbps.toFixed(0)+' кбит/с, клиентов: '+s.clients+
      ', RSSI '+s.rssi+' dBm, heap '+s.heap;
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
//  СТРИМ. Обработчик не возвращается, пока клиент не отвалится: он крутит
//  бесконечный while и подкладывает в тот же ответ новые кадры.
//
//  Каждый кадр отдаётся тремя чанками:
//     "\r\n--boundary\r\n"                          <- граница части
//     "Content-Type: image/jpeg\r\nContent-Length: N\r\n\r\n"
//     <N байт JPEG>
//  Как только httpd_resp_send_chunk() вернёт ошибку — значит браузер закрыл
//  вкладку, и мы просто выходим из цикла.
// ---------------------------------------------------------------------------
static esp_err_t streamHandler(httpd_req_t* req) {
  esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;

  // Разрешаем встраивать поток в страницу с другого origin (например в React-дашборд
  // на localhost:5173). Для тега <img> это не обязательно, но пригодится, если
  // однажды захочется читать кадры из JS/canvas.
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  streamClients++;
  Serial.printf("[STREAM] клиент подключился (всего %d)\n", streamClients);

  char partBuf[72];
  int64_t windowStart = esp_timer_get_time();
  uint32_t windowFrames = 0;
  uint32_t windowBytes  = 0;

  while (true) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) { Serial.println("[STREAM] fb_get NULL"); res = ESP_FAIL; break; }

    // Сенсор GC2145 отдаёт сырой RGB565, готового JPEG у него нет — жмём сами
    // (тот же frame2jpg, что в уроке 23). Это и есть главный ограничитель к/с.
    uint8_t* jpg = nullptr;
    size_t   jpg_len = 0;
    bool ok = frame2jpg(fb, JPEG_QUALITY, &jpg, &jpg_len);
    esp_camera_fb_return(fb);          // сырой кадр драйверу больше не нужен
    if (!ok) { res = ESP_FAIL; break; }

    res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res == ESP_OK) {
      size_t hlen = snprintf(partBuf, sizeof(partBuf), STREAM_PART, (unsigned)jpg_len);
      res = httpd_resp_send_chunk(req, partBuf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char*)jpg, jpg_len);
    }

    windowFrames++;
    windowBytes += jpg_len;
    free(jpg);                         // буфер от frame2jpg наш — освободить ОБЯЗАТЕЛЬНО

    if (res != ESP_OK) break;          // клиент ушёл

    // Раз в секунду пересчитываем к/с и битрейт — их читает /stats.
    int64_t now = esp_timer_get_time();
    if (now - windowStart >= 1000000) {
      float sec = (now - windowStart) / 1000000.0f;
      streamFps  = windowFrames / sec;
      streamKbps = (windowBytes * 8.0f / 1000.0f) / sec;
      Serial.printf("[STREAM] %.1f к/с, %.0f кбит/с\n", streamFps, streamKbps);
      windowStart = now; windowFrames = 0; windowBytes = 0;
    }
  }

  streamClients--;
  streamFps = 0; streamKbps = 0;
  Serial.printf("[STREAM] клиент отключился (осталось %d)\n", streamClients);
  return res;
}

// ---------------------------------------------------------------------------
//  Мелкие обработчики: статистика, разрешение, вспышка.
// ---------------------------------------------------------------------------
static esp_err_t statsHandler(httpd_req_t* req) {
  char json[192];
  snprintf(json, sizeof(json),
           "{\"fps\":%.1f,\"kbps\":%.0f,\"clients\":%d,\"rssi\":%d,\"heap\":%u}",
           streamFps, streamKbps, streamClients, WiFi.RSSI(), (unsigned)ESP.getFreeHeap());
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

// Достать числовой query-параметр вида ?v=123
static int queryInt(httpd_req_t* req, const char* key, int def) {
  char query[64];
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) return def;
  char value[16];
  if (httpd_query_key_value(query, key, value, sizeof(value)) != ESP_OK) return def;
  return atoi(value);
}

static esp_err_t sizeHandler(httpd_req_t* req) {
  int v = queryInt(req, "v", FRAMESIZE_VGA);
  v = constrain(v, (int)FRAMESIZE_96X96, (int)FRAMESIZE_UXGA);   // OV2640 больше UXGA не умеет

  sensor_t* s = esp_camera_sensor_get();
  int rc = s ? s->set_framesize(s, (framesize_t)v) : -1;
  Serial.printf("[CAM] framesize -> #%d (rc=%d)\n", v, rc);

  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_send(req, rc == 0 ? "ok" : "fail", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t flashHandler(httpd_req_t* req) {
  int v = constrain(queryInt(req, "v", 0), 0, 255);
  flashSet(v);
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_send(req, "ok", HTTPD_RESP_USE_STRLEN);
}

// ---------------------------------------------------------------------------
//  Поднимаем два независимых httpd. Второму серверу НУЖНО поменять не только
//  server_port, но и ctrl_port: это внутренний UDP-порт управления, и на общем
//  ctrl_port второй httpd_start() просто не запустится.
// ---------------------------------------------------------------------------
void startServers() {
  httpd_config_t web = HTTPD_DEFAULT_CONFIG();
  web.server_port = 80;
  web.ctrl_port   = 32768;
  web.max_uri_handlers = 8;

  httpd_uri_t uriIndex = { "/",      HTTP_GET, indexHandler, NULL };
  httpd_uri_t uriStats = { "/stats", HTTP_GET, statsHandler, NULL };
  httpd_uri_t uriSize  = { "/size",  HTTP_GET, sizeHandler,  NULL };
  httpd_uri_t uriFlash = { "/flash", HTTP_GET, flashHandler, NULL };

  if (httpd_start(&webServer, &web) == ESP_OK) {
    httpd_register_uri_handler(webServer, &uriIndex);
    httpd_register_uri_handler(webServer, &uriStats);
    httpd_register_uri_handler(webServer, &uriSize);
    httpd_register_uri_handler(webServer, &uriFlash);
    Serial.println("[WEB] сервер страницы на :80");
  } else {
    Serial.println("[WEB] не удалось поднять сервер на :80");
  }

  httpd_config_t stream = HTTPD_DEFAULT_CONFIG();
  stream.server_port = 81;
  stream.ctrl_port   = 32769;      // ОБЯЗАТЕЛЬНО другой, иначе конфликт с первым
  httpd_uri_t uriStream = { "/stream", HTTP_GET, streamHandler, NULL };

  if (httpd_start(&streamServer, &stream) == ESP_OK) {
    httpd_register_uri_handler(streamServer, &uriStream);
    Serial.println("[WEB] сервер потока на :81/stream");
  } else {
    Serial.println("[WEB] не удалось поднять сервер на :81");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);      // нативному USB нужно время подняться, иначе первые строки теряются
  Serial.println("\n=== Урок 24. MJPEG-поток с ESP32-S3-CAM ===");

  statusLed(true);
  flashInit();

  if (!initCamera()) {
    Serial.println("[CAM] камера не поднялась — прогони lesson_check_cam.");
    return;
  }
  Serial.printf("[CAM] init OK, PSRAM: %s\n", psramFound() ? "да" : "нет");

  WiFi.begin(SSID, PASS);
  Serial.print("[WiFi] Подключение");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.printf("\n[WiFi] IP: %s (RSSI %d dBm)\n",
                WiFi.localIP().toString().c_str(), WiFi.RSSI());

  // Wi-Fi спит между пакетами и режет пропускную способность — для видео это
  // заметно. Отключаем power save: к/с растут, потребление тоже.
  WiFi.setSleep(false);

  startServers();
  Serial.printf("[WEB] открой http://%s/\n", WiFi.localIP().toString().c_str());

  statusLed(false);
}

void loop() {
  // Здесь ПУСТО, и это главное отличие от урока 23: esp_http_server живёт в
  // своих собственных задачах FreeRTOS, а не внутри loop(). Поэтому бесконечный
  // стрим ничего не блокирует, и loop() свободен под что угодно — в уроке 26
  // мы займём его MQTT-клиентом.
  delay(1000);
}
