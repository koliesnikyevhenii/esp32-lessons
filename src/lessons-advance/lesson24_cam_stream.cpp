// ============================================================================
//  Stage 6. Camera — lesson 24: an MJPEG video stream in the browser
// ----------------------------------------------------------------------------
//  Goal: turn the "camera" from lesson 23 into VIDEO. No H.264 and no WebRTC:
//  the ESP32 serves an MJPEG stream — an endless sequence of ordinary JPEG frames
//  in a single HTTP response. The browser can show such a response in an <img> tag
//  on its own, without a single line of video code.
//
//     Browser --GET :81/stream--> ESP32-CAM ...frame...frame...frame... (never closing the response)
//
//  What we learn:
//    - multipart/x-mixed-replace — "replace the previous part with the next one". That is
//      the whole secret of MJPEG: one response, many parts, each part = a whole JPEG;
//    - chunked transfer: httpd_resp_send_chunk() in a loop, the response never completes;
//    - why the WebServer from lessons 17 and 23 won't do here and we use
//      esp_http_server: the stream handler NEVER returns control, and the
//      synchronous WebServer lives inside loop() — it would block everything at once;
//    - TWO servers on two ports (80 — page and buttons, 81 — stream): while a
//      worker is busy with the endless stream, ordinary requests to the same server
//      would wait in a queue. This is exactly how the CameraWebServer example does it;
//    - the cost of resolution: compare fps at QVGA / VGA / SVGA. Our board carries a
//      GC2145 sensor without hardware JPEG, so the CPU compresses each frame
//      (frame2jpg, as in lesson 23) — the bottleneck is compression, not Wi-Fi;
//    - CAMERA_GRAB_LATEST — serve the freshest frame so the video doesn't lag;
//    - WHY resolution is changed by re-initialising the camera: frame buffers are
//      allocated once, in esp_camera_init(), for cfg.frame_size, while
//      set_framesize() only touches the sensor registers. With RGB565 this shows
//      immediately — the frame doesn't fit the old buffer and breaks into stripes.
//
//  Endpoints:
//    :80/            HTML: <img> with the stream, fps, resolution choice, flash
//    :80/stats       JSON: {"fps":..,"kbps":..,"clients":..,"rssi":..,"heap":..}
//    :80/size?v=N    resolution (5=QVGA, 8=VGA, 9=SVGA, 13=UXGA) — with re-initialisation
//    :80/flash?v=0..255  flash
//    :81/stream      the MJPEG stream itself
//
//  Build/flash:
//    pio run -e lesson24_cam_stream -t upload
//    pio device monitor        # prints the IP -> open http://IP/ in the browser
//
//  Board: HW-679 ESP32-S3-CAM (board = esp32-s3-devkitc-1 + memory_type = qio_opi).
//  Pins — include/pins.h (the CAM_* block).
// ============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include "esp_camera.h"
#include "img_converters.h"   // frame2jpg(): software RGB565 -> JPEG compression
#include "esp_http_server.h"
#include "esp_timer.h"
#include "pins.h"

// ---- Wi-Fi ----------------------------------------------------------------
const char* SSID = "Vektor_04";
const char* PASS = "uteam2020";

// GC2145 (the sensor on our board) can't do hardware JPEG, the frame is compressed
// in software via frame2jpg — and its scale is 0..100 where HIGHER = better.
const int JPEG_QUALITY = 80;

// ---- MJPEG boundaries -------------------------------------------------------
// The separator can be any string that is guaranteed not to occur in the binary data.
#define PART_BOUNDARY "esp32camframe"
static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY     = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART         = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// ---- Two servers: page and stream -----------------------------------------
httpd_handle_t webServer    = NULL;   // port 80
httpd_handle_t streamServer = NULL;   // port 81

// ---- Statistics (computed by the stream handler itself) --------------------
volatile float    streamFps   = 0;    // frames per second
volatile float    streamKbps  = 0;    // kilobits per second
volatile int      streamClients = 0;  // how many streams are open right now

// ---- Resolution: why it can't be changed "on the fly" -----------------------
// The esp32-camera driver computes the frame geometry ONCE — in esp_camera_init():
// cam_config() stores width/height and allocates buffers for exactly that size.
// And sensor_t::set_framesize() is a SENSOR driver function: it writes GC2145
// registers and knows nothing about the buffers. In JPEG mode we'd still get away
// with it (JPEG is self-contained, the size is inside the frame), but we use RGB565:
// frame2jpg() reads fb->width/height, which stayed the same, while the sensor already
// emits rows of a different length. The result is exactly those stripes and a black bottom.
// So we change resolution by RE-INITIALISING the camera: deinit + init.
static framesize_t       camSize = FRAMESIZE_QVGA;   // what is set right now
static SemaphoreHandle_t camLock = NULL;             // the camera has one owner at a time

// RGB565 is 2 bytes per pixel, and the buffer is allocated in full:
//   QVGA 150 KB | VGA 600 KB | SVGA 938 KB | XGA 1.5 MB | UXGA 3.7 MB.
// Two UXGA buffers (7.3 MB) no longer fit in 8 MB of PSRAM alongside Wi-Fi and the
// frame2jpg output buffer, so on large frames we use single buffering.
static size_t fbCountFor(framesize_t fs) {
  return (fs <= FRAMESIZE_SVGA) ? 2 : 1;
}

// ---------------------------------------------------------------------------
//  Flash light and status indicator (pins aren't standardised on S3 boards -> -1 in pins.h).
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
//  Camera initialisation. The difference from lesson 23 is grab_mode.
// ---------------------------------------------------------------------------
bool initCamera(framesize_t fs) {
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

  // XCLK depends on the frame size, and that is not fine-tuning but a requirement.
  // On large frames the camera bus can't keep up: DMA can't push the pixel stream into
  // PSRAM, cam_hal starts spewing "EV-EOF-OVF", and fb_get() returns NULL —
  // the camera is dead until the next re-initialisation. Measured on this
  // board: UXGA at 20 MHz = an OVF storm and not a single frame, at 10 MHz = 0.5 fps and
  // a normal picture. SXGA at 10 MHz is even faster (0.7 vs 0.4),
  // so this is not just a cure for OVF.
  cfg.xclk_freq_hz = (fs >= FRAMESIZE_XGA) ? 10000000 : 20000000;
  cfg.ledc_timer   = LEDC_TIMER_0;
  cfg.ledc_channel = LEDC_CHANNEL_0;

  cfg.pixel_format = PIXFORMAT_RGB565;   // raw frame: we'll compress it ourselves
  cfg.jpeg_quality = 12;                 // unused with RGB565

  // Compression is done by the CPU, and its cost grows with frame area: we start
  // at QVGA, and the buttons on the page re-initialise the camera to a bigger frame
  // so fps can be compared.
  if (psramFound()) {
    cfg.frame_size  = fs;
    cfg.fb_count    = fbCountFor(fs);
    cfg.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    // Without PSRAM there is simply nowhere to put more than QVGA: internal DRAM is ~300 KB.
    cfg.frame_size  = FRAMESIZE_QVGA;
    cfg.fb_count    = 1;
    cfg.fb_location = CAMERA_FB_IN_DRAM;
  }

  // THIS matters for video: with CAMERA_GRAB_WHEN_EMPTY the browser gets frames from
  // the buffer queue and the picture lags behind reality by fb_count frames.
  // CAMERA_GRAB_LATEST always takes the freshest one — less latency.
  cfg.grab_mode = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&cfg);
  if (err != ESP_OK) {
    Serial.printf("[CAM] init FAILED: 0x%x (%s)\n", err, esp_err_to_name(err));
    return false;
  }
  camSize = cfg.frame_size;   // remembered: applyFramesize() compares against this
  Serial.printf("[CAM] кадр #%d, буферов %u\n", (int)camSize, (unsigned)cfg.fb_count);
  return true;
}
// ---------------------------------------------------------------------------
//  Changing resolution = deinit + init (why — see the comment at camSize).
//  It is called by the handlers on :80, while frames are being pulled by the stream
//  task on :81 — so both take camLock: without it the stream could get an fb
//  from an already destroyed driver.
// ---------------------------------------------------------------------------
static bool applyFramesize(framesize_t fs) {
  if (!psramFound() && fs > FRAMESIZE_QVGA) return false;   // won't fit in DRAM
  if (fs == camSize) return true;

  xSemaphoreTake(camLock, portMAX_DELAY);
  esp_camera_deinit();
  bool ok = initCamera(fs);
  if (!ok) {                                  // didn't come up — roll back
    Serial.println("[CAM] не удалось, откат на QVGA");
    ok = initCamera(FRAMESIZE_QVGA);
  }
  xSemaphoreGive(camLock);
  return ok;
}


// ---------------------------------------------------------------------------
//  THE PAGE. The stream lives on another port, so the browser builds its address from
//  location.hostname — that way the same HTML works with any IP.
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
// The browser may take the same stream URL from the cache — add a timestamp.
function open_(){ cam.src='http://'+location.hostname+':81/stream?t='+Date.now(); }
function size(v){ fetch('/size?v='+v).then(open_); }   // after a size change the stream must be reopened
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
//  THE STREAM. The handler doesn't return until the client drops off: it spins
//  an endless while and feeds new frames into the same response.
//
//  Each frame is sent as three chunks:
//     "\r\n--boundary\r\n"                          <- part boundary
//     "Content-Type: image/jpeg\r\nContent-Length: N\r\n\r\n"
//     <N bytes of JPEG>
//  As soon as httpd_resp_send_chunk() returns an error, the browser has closed the
//  tab, and we simply leave the loop.
// ---------------------------------------------------------------------------
static esp_err_t streamHandler(httpd_req_t* req) {
  esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;

  // Allow embedding the stream in a page from another origin (e.g. a React dashboard
  // on localhost:5173). For an <img> tag this isn't required, but it will help if
  // one day we want to read frames from JS/canvas.
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  streamClients++;
  Serial.printf("[STREAM] клиент подключился (всего %d)\n", streamClients);

  char partBuf[72];
  int64_t windowStart = esp_timer_get_time();
  uint32_t windowFrames = 0;
  uint32_t windowBytes  = 0;

  while (true) {
    // Take the frame under camLock: while we hold fb, nobody is allowed to
    // deinit the camera from the /size handler. The lock is released BEFORE sending —
    // the network is slow, and there's no reason to hold the camera during it.
    xSemaphoreTake(camLock, portMAX_DELAY);
    camera_fb_t* fb = esp_camera_fb_get();
    // The GC2145 sensor outputs raw RGB565, it has no ready JPEG — we compress ourselves
    // (the same frame2jpg as in lesson 23). This is the main limit on fps.
    uint8_t* jpg = nullptr;
    size_t   jpg_len = 0;
    bool ok = fb && frame2jpg(fb, JPEG_QUALITY, &jpg, &jpg_len);
    if (fb) esp_camera_fb_return(fb);  // the driver no longer needs the raw frame
    xSemaphoreGive(camLock);

    if (!fb) { Serial.println("[STREAM] fb_get NULL"); res = ESP_FAIL; break; }
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
    free(jpg);                         // the frame2jpg buffer is ours — freeing it is MANDATORY

    if (res != ESP_OK) break;          // the client left

    // Once a second recompute fps and bitrate — /stats reads them.
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
//  Small handlers: statistics, resolution, flash.
// ---------------------------------------------------------------------------
static esp_err_t statsHandler(httpd_req_t* req) {
  char json[192];
  snprintf(json, sizeof(json),
           "{\"fps\":%.1f,\"kbps\":%.0f,\"clients\":%d,\"rssi\":%d,\"heap\":%u}",
           streamFps, streamKbps, streamClients, WiFi.RSSI(), (unsigned)ESP.getFreeHeap());
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

// Fetch a numeric query parameter like ?v=123
static int queryInt(httpd_req_t* req, const char* key, int def) {
  char query[64];
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) return def;
  char value[16];
  if (httpd_query_key_value(query, key, value, sizeof(value)) != ESP_OK) return def;
  return atoi(value);
}

static esp_err_t sizeHandler(httpd_req_t* req) {
  int v = queryInt(req, "v", (int)FRAMESIZE_QVGA);
  v = constrain(v, (int)FRAMESIZE_96X96, (int)FRAMESIZE_UXGA);   // GC2145 can't go beyond UXGA

  bool ok = applyFramesize((framesize_t)v);
  Serial.printf("[CAM] framesize -> #%d (%s)\n", v, ok ? "ok" : "fail");

  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_send(req, ok ? "ok" : "fail", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t flashHandler(httpd_req_t* req) {
  int v = constrain(queryInt(req, "v", 0), 0, 255);
  flashSet(v);
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_send(req, "ok", HTTPD_RESP_USE_STRLEN);
}

// ---------------------------------------------------------------------------
//  Start two independent httpd instances. The second server MUST change not only
//  server_port but also ctrl_port: that's the internal UDP control port, and with a shared
//  ctrl_port the second httpd_start() simply won't start.
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
  stream.ctrl_port   = 32769;      // MUST differ, otherwise it conflicts with the first one
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
  delay(1500);      // native USB needs time to come up, otherwise the first lines are lost
  Serial.println("\n=== Урок 24. MJPEG-поток с ESP32-S3-CAM ===");

  statusLed(true);
  flashInit();

  camLock = xSemaphoreCreateMutex();

  if (!initCamera(FRAMESIZE_QVGA)) {
    Serial.println("[CAM] камера не поднялась — прогони lesson_check_cam.");
    return;
  }
  Serial.printf("[CAM] init OK, PSRAM: %s\n", psramFound() ? "да" : "нет");

  WiFi.begin(SSID, PASS);
  Serial.print("[WiFi] Подключение");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.printf("\n[WiFi] IP: %s (RSSI %d dBm)\n",
                WiFi.localIP().toString().c_str(), WiFi.RSSI());

  // Wi-Fi sleeps between packets and cuts throughput — noticeable for video.
  // Disable power save: fps goes up, and so does power consumption.
  WiFi.setSleep(false);

  startServers();
  Serial.printf("[WEB] открой http://%s/\n", WiFi.localIP().toString().c_str());

  statusLed(false);
}

void loop() {
  // EMPTY here, and that's the main difference from lesson 23: esp_http_server lives in
  // its own FreeRTOS tasks, not inside loop(). So the endless stream blocks
  // nothing, and loop() is free for anything — in lesson 26
  // we'll fill it with an MQTT client.
  delay(1000);
}
