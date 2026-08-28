// ============================================================================
//  Этап 6. Камера — урок 23: один кадр по HTTP (фотоаппарат в браузере)
// ----------------------------------------------------------------------------
//  Цель: получить с камеры ОДИН кадр и показать его в браузере. Это тот же
//  веб-сервер, что в уроке 17 (WebServer + server.on), только вместо HTML со
//  температурой мы отдаём JPEG-байты.
//
//     Браузер --HTTP GET /jpg--> ESP32-CAM --> esp_camera_fb_get() --> JPEG
//
//  Почему начинаем с кадра, а не сразу со стрима: пока картинка одна, видно
//  весь цикл целиком — снять, отдать, вернуть буфер. Стрим (урок 24) — это
//  просто тот же цикл в бесконечном while.
//
//  Что изучаем:
//    - esp_camera_fb_get() / esp_camera_fb_return() — жизненный цикл кадра;
//    - Content-Type: image/jpeg и setContentLength() — как отдать БИНАРНЫЕ данные
//      (server.send() умеет только текст, поэтому пишем в клиента напрямую);
//    - кеш браузера: без ?t=<время> вторая картинка приедет из кеша, а не с камеры;
//    - "видео из фотографий": страница может дёргать /jpg по кругу — получится
//      2-4 кадра в секунду. Больше не выйдет, и в уроке 24 разберём почему.
//
//  Страница отдаёт:
//    /            HTML: картинка, кнопка "Снять", галочка "непрерывно", вспышка
//    /jpg         один кадр JPEG
//    /flash?v=0..255  яркость вспышки (PWM)
//    /info        текстовая статистика (разрешение, размер кадра, RAM, RSSI)
//
//  Сборка/прошивка (сначала прогони lesson_check_cam!):
//    pio run -e lesson23_cam_snapshot -t upload
//    pio device monitor        # в мониторе напечатается IP -> открой в браузере
//
//  Плата: HW-679 ESP32-S3-CAM (board = esp32-s3-devkitc-1 + memory_type = qio_opi),
//  НЕ DevKit. Пины — include/pins.h (блок CAM_*).
// ============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "esp_camera.h"
#include "pins.h"

// ---- Wi-Fi ----------------------------------------------------------------
const char* SSID = "Vektor_04";
const char* PASS = "uteam2020";

const int JPEG_QUALITY = 12;          // 0..63, меньше = лучше картинка и больше байт

WebServer server(80);
bool cameraReady = false;
unsigned long shots = 0;

// ---------------------------------------------------------------------------
//  Подсветка и индикатор. На S3-платах их пины не стандартизованы, поэтому в
//  pins.h они по умолчанию -1 и весь этот код просто не компилируется
//  (см. «охоту за LED» в lesson_check_cam).
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
//  Инициализация камеры — тот же код, что в lesson_check_cam (уроки в этом
//  проекте самодостаточны, поэтому копия, а не общий модуль).
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
  cfg.jpeg_quality = JPEG_QUALITY;

  if (psramFound()) {
    cfg.frame_size  = FRAMESIZE_VGA;      // 640x480
    cfg.fb_count    = 2;
    cfg.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    cfg.frame_size  = FRAMESIZE_QVGA;     // 320x240
    cfg.fb_count    = 1;
    cfg.fb_location = CAMERA_FB_IN_DRAM;
  }
  cfg.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

  esp_err_t err = esp_camera_init(&cfg);
  if (err != ESP_OK) {
    Serial.printf("[CAM] init FAILED: 0x%x (%s)\n", err, esp_err_to_name(err));
    return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
//  Страница. Держим её в R"HTML(...)HTML" — так можно писать вёрстку как есть,
//  без экранирования кавычек.
//
//  Ключевая деталь — ?t=Date.now() в src. Браузер кеширует картинки по URL;
//  без уникального параметра он покажет ту же самую первую фотографию и никуда
//  не пойдёт. Тот же приём понадобится всюду, где URL один, а данные меняются.
// ---------------------------------------------------------------------------
const char PAGE_HTML[] PROGMEM = R"HTML(
<!doctype html><html lang="ru"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Урок 23 — кадр с ESP32-CAM</title>
<style>
 body{font-family:system-ui,sans-serif;background:#111;color:#eee;margin:0;padding:16px}
 h1{font-size:18px;margin:0 0 12px}
 img{max-width:100%;border:1px solid #444;background:#000;display:block}
 .row{display:flex;gap:12px;align-items:center;flex-wrap:wrap;margin:12px 0}
 button{padding:8px 14px;font-size:15px;cursor:pointer}
 code{color:#8cf}
 #fps{font-variant-numeric:tabular-nums}
</style></head><body>
<h1>Урок 23 — один кадр по HTTP</h1>
<img id="shot" src="/jpg" alt="кадр">
<div class="row">
  <button onclick="shot()">Снять кадр</button>
  <label><input type="checkbox" id="loop" onchange="if(this.checked)shot()"> непрерывно</label>
  <span id="fps">-</span>
</div>
<div class="row">
  вспышка: <input type="range" min="0" max="255" value="0" oninput="flash(this.value)">
</div>
<p><code>/jpg</code> — кадр, <code>/info</code> — статистика, <code>/flash?v=0..255</code> — свет.</p>
<script>
const img=document.getElementById('shot'), fpsEl=document.getElementById('fps');
let t0=0, n=0, tStart=0;
function shot(){ t0=performance.now(); if(!tStart)tStart=t0; img.src='/jpg?t='+Date.now(); }
img.onload=()=>{
  const dt=performance.now()-t0; n++;
  const avg=n/((performance.now()-tStart)/1000);
  fpsEl.textContent='кадр за '+dt.toFixed(0)+' мс, средний темп '+avg.toFixed(1)+' к/с';
  if(document.getElementById('loop').checked) shot();
};
img.onerror=()=>{ fpsEl.textContent='ошибка запроса'; };
function flash(v){ fetch('/flash?v='+v); }
</script></body></html>
)HTML";

void handleRoot() {
  server.send_P(200, "text/html; charset=utf-8", PAGE_HTML);
}

// ---------------------------------------------------------------------------
//  Главный обработчик: снять кадр и отдать его как JPEG.
//
//  server.send() кладёт в тело СТРОКУ, а в JPEG есть нулевые байты — строкой
//  его не передать. Поэтому: объявляем длину, отправляем пустое тело с нужными
//  заголовками, а байты пишем в TCP-сокет напрямую через client.write().
// ---------------------------------------------------------------------------
void handleJpg() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    server.send(503, "text/plain", "no frame");
    Serial.println("[CAM] fb_get вернул NULL");
    return;
  }

  shots++;
  // Запрещаем кеширование и на стороне сервера — плюс к ?t= на клиенте.
  server.sendHeader("Cache-Control", "no-store");
  server.setContentLength(fb->len);
  server.send(200, "image/jpeg", "");

  WiFiClient client = server.client();
  size_t sent = client.write(fb->buf, fb->len);

  Serial.printf("[HTTP] кадр #%lu: %ux%u, %u байт (отправлено %u)\n",
                shots, (unsigned)fb->width, (unsigned)fb->height,
                (unsigned)fb->len, (unsigned)sent);

  // Вернуть буфер драйверу ОБЯЗАТЕЛЬНО и ОБЯЗАТЕЛЬНО после отправки.
  esp_camera_fb_return(fb);
}

// ---------------------------------------------------------------------------
//  Вспышка. PWM, а не digitalWrite: светодиод очень яркий и заметно греется.
//  Если пин не задан (CAM_FLASH_LED = -1), эндпоинт остаётся, но ничего не делает.
// ---------------------------------------------------------------------------
void handleFlash() {
  int v = server.hasArg("v") ? server.arg("v").toInt() : 0;
  v = constrain(v, 0, 255);
  flashSet(v);
  server.send(200, "text/plain", String(v));
  Serial.printf("[LED] вспышка = %d\n", v);
}

void handleInfo() {
  sensor_t* s = esp_camera_sensor_get();
  String out = "ESP32-CAM, урок 23\n";
  out += "IP:            " + WiFi.localIP().toString() + "\n";
  out += "RSSI:          " + String(WiFi.RSSI()) + " dBm\n";
  out += "framesize #:   " + String(s ? s->status.framesize : -1) + "\n";
  out += "quality:       " + String(s ? s->status.quality : -1) + "\n";
  out += "PSRAM:         " + String(psramFound() ? "да" : "нет") + "\n";
  out += "free heap:     " + String((unsigned)ESP.getFreeHeap()) + "\n";
  out += "кадров отдано: " + String(shots) + "\n";
  server.send(200, "text/plain; charset=utf-8", out);
}

void setup() {
  Serial.begin(115200);
  delay(1500);      // нативному USB нужно время подняться, иначе первые строки теряются
  Serial.println("\n=== Урок 23. Кадр с ESP32-S3-CAM по HTTP ===");

  statusLed(true);
  flashInit();

  cameraReady = initCamera();
  if (!cameraReady) {
    Serial.println("[CAM] камера не поднялась — прогони lesson_check_cam.");
    return;
  }
  Serial.printf("[CAM] init OK, PSRAM: %s\n", psramFound() ? "да" : "нет");

  WiFi.begin(SSID, PASS);
  Serial.print("[WiFi] Подключение");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.printf("\n[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());
  Serial.printf("[WEB] открой в браузере: http://%s/\n", WiFi.localIP().toString().c_str());

  server.on("/",      handleRoot);
  server.on("/jpg",   handleJpg);
  server.on("/flash", handleFlash);
  server.on("/info",  handleInfo);
  server.begin();

  statusLed(false);                           // погасили: всё поднялось
}

void loop() {
  // WebServer синхронный: он обрабатывает запрос ровно здесь, внутри loop().
  // Пока handleJpg() снимает и отправляет кадр, ничего другого не происходит —
  // именно это ограничение и заставит нас в уроке 24 перейти на esp_http_server.
  server.handleClient();
}
