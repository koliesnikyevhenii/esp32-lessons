// ============================================================================
//  Stage 6. Camera — lesson 23: a single frame over HTTP (a camera in the browser)
// ----------------------------------------------------------------------------
//  Goal: grab ONE frame from the camera and show it in the browser. This is the
//  same web server as in lesson 17 (WebServer + server.on), except that instead
//  of HTML with a temperature we send JPEG bytes.
//
//     Browser --HTTP GET /jpg--> ESP32-CAM --> esp_camera_fb_get() --> JPEG
//
//  Why start with a single frame rather than a stream right away: with one
//  picture the whole cycle is visible — capture, send, return the buffer. A stream
//  (lesson 24) is just the same cycle inside an endless while.
//
//  What we learn:
//    - esp_camera_fb_get() / esp_camera_fb_return() — the frame lifecycle;
//    - frame2jpg() — software compression: our board carries a GC2145 sensor,
//      and unlike the OV2640 it cannot do JPEG itself (checked by lesson_check_cam:
//      "sensor PID 0x2145 (GC2145), JPEG: no");
//    - Content-Type: image/jpeg and setContentLength() — how to send BINARY data
//      (server.send() only handles text, so we write to the client directly);
//    - browser cache: without ?t=<time> the second picture comes from the cache, not the camera;
//    - "video from photos": the page can poll /jpg in a loop — that gives
//      2-4 frames per second. No more than that, and lesson 24 explains why.
//
//  The page serves:
//    /            HTML: picture, "Capture" button, "continuous" checkbox, flash
//    /jpg         a single JPEG frame
//    /flash?v=0..255  flash brightness (PWM)
//    /info        text statistics (resolution, frame size, RAM, RSSI)
//
//  Build/flash (run lesson_check_cam first!):
//    pio run -e lesson23_cam_snapshot -t upload
//    pio device monitor        # the monitor prints the IP -> open it in the browser
//
//  Board: HW-679 ESP32-S3-CAM (board = esp32-s3-devkitc-1 + memory_type = qio_opi),
//  NOT the DevKit. Pins — include/pins.h (the CAM_* block).
// ============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "esp_camera.h"
#include "img_converters.h"   // frame2jpg(): software RGB565 -> JPEG compression
#include "pins.h"

// ---- Wi-Fi ----------------------------------------------------------------
const char* SSID = "Vektor_04";
const char* PASS = "uteam2020";

// The sensor on this board is a GC2145, and it has NO hardware JPEG (unlike the
// OV2640): esp_camera_init with PIXFORMAT_JPEG fails with ESP_ERR_NOT_SUPPORTED.
// So we capture raw RGB565 and compress it to JPEG ourselves, on the CPU.
// Careful: frame2jpg quality is 0..100 and HIGHER = better (the sensor's is the reverse).
const int JPEG_QUALITY = 80;          // 0..100, higher = better picture and more bytes

WebServer server(80);
bool cameraReady = false;
unsigned long shots = 0;

// ---------------------------------------------------------------------------
//  Flash light and status indicator. On S3 boards their pins are not standardised,
//  so in pins.h they default to -1 and all of this code simply doesn't compile
//  (see the "LED hunt" in lesson_check_cam).
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
//  Camera initialisation — the same code as in lesson_check_cam (lessons in this
//  project are self-contained, hence a copy rather than a shared module).
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

  cfg.pixel_format = PIXFORMAT_RGB565;   // raw frame: 2 bytes per pixel
  cfg.jpeg_quality = 12;                 // unused with RGB565, but the field must be valid

  if (psramFound()) {
    cfg.frame_size  = FRAMESIZE_VGA;      // 640x480 = 614 KB per buffer — PSRAM only
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
//  The page. We keep it in R"HTML(...)HTML" — that way the markup can be written
//  as is, without escaping quotes.
//
//  The key detail is ?t=Date.now() in src. The browser caches images by URL;
//  without a unique parameter it shows the very same first photo and never makes
//  a request. The same trick is needed wherever the URL stays fixed but the data changes.
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
//  Main handler: capture a frame and send it as JPEG.
//
//  server.send() puts a STRING in the body, and JPEG contains zero bytes — it can't
//  be sent as a string. So: declare the length, send an empty body with the right
//  headers, and write the bytes straight into the TCP socket via client.write().
//
//  The frame arrives raw (RGB565), so a third step appears between "capture" and
//  "send" — frame2jpg(). It allocates the JPEG buffer itself, and that buffer MUST be
//  free()d. This is the price of having no hardware JPEG: compression eats CPU.
// ---------------------------------------------------------------------------
void handleJpg() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    server.send(503, "text/plain", "no frame");
    Serial.println("[CAM] fb_get вернул NULL");
    return;
  }

  uint8_t* jpg     = nullptr;
  size_t   jpg_len = 0;
  unsigned int w = fb->width, h = fb->height;
  unsigned long t0 = millis();
  bool ok = frame2jpg(fb, JPEG_QUALITY, &jpg, &jpg_len);
  unsigned long enc_ms = millis() - t0;

  // Return the frame to the driver right away: the JPEG is already in a separate buffer.
  esp_camera_fb_return(fb);

  if (!ok) {
    server.send(500, "text/plain", "jpeg encode failed");
    Serial.println("[CAM] frame2jpg не смогла сжать кадр");
    return;
  }

  shots++;
  // Forbid caching on the server side too — in addition to ?t= on the client.
  server.sendHeader("Cache-Control", "no-store");
  server.setContentLength(jpg_len);
  server.send(200, "image/jpeg", "");

  WiFiClient client = server.client();
  size_t sent = client.write(jpg, jpg_len);

  Serial.printf("[HTTP] кадр #%lu: %ux%u, JPEG %u байт (сжатие %lu мс, отправлено %u)\n",
                shots, w, h, (unsigned)jpg_len, enc_ms, (unsigned)sent);

  // The frame2jpg buffer is ours — freeing it is MANDATORY, otherwise after a dozen
  // frames the heap runs out and the server silently stops responding.
  free(jpg);
}

// ---------------------------------------------------------------------------
//  Flash. PWM rather than digitalWrite: the LED is very bright and gets noticeably hot.
//  If the pin is not set (CAM_FLASH_LED = -1), the endpoint stays but does nothing.
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
  delay(1500);      // native USB needs time to come up, otherwise the first lines are lost
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

  statusLed(false);                           // turned it off: everything is up
}

void loop() {
  // WebServer is synchronous: it handles a request right here, inside loop().
  // While handleJpg() captures and sends a frame, nothing else happens —
  // exactly this limitation is what makes us switch to esp_http_server in lesson 24.
  server.handleClient();
}
