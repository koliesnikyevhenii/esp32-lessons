// ============================================================================
//  Stage 6. Camera — lesson 25: every sensor capability in one panel
// ----------------------------------------------------------------------------
//  Goal: get a feel for WHAT the sensor can do besides "producing frames". Lesson 24
//  gave us a stream; here the stream gets a control panel that tweaks camera
//  settings on live video — and you immediately see how each of them affects the
//  picture and the number of frames per second.
//
//  The main idea of the lesson: the settings live not in our code but IN THE SENSOR ITSELF.
//  The driver exposes a sensor_t struct with function pointers:
//
//      sensor_t* s = esp_camera_sensor_get();
//      s->set_brightness(s, 1);        // <- writes sensor registers over SCCB
//      s->status.brightness;           // <- current value
//
//  So set_* is not a "filter in the firmware" but a command to the camera chip. That's
//  why it is free in CPU terms: the ESP32 doesn't process pixels at all.
//
//  Exactly one exception — quality. Our board carries a GC2145, which cannot
//  output JPEG, so the frame is compressed in software (frame2jpg, lesson 23). Hence
//  quality here is a parameter of OUR codec, not a sensor register: scale 0..100,
//  higher = better (the OV2640's hardware JPEG was the reverse, 0..63, lower =
//  better). It also shows how "hardware settings" differ from "code settings":
//  none of the other sliders load the CPU, but this one does.
//
//  On the GC2145 some set_* return -1 ("unsupported") — that's expected, every
//  sensor has its own register set. The panel honestly shows this as a status.
//
//  What we learn (and what we watch with our own eyes):
//    - SIZE AND QUALITY:     framesize (sensor) and quality (our codec) -> fps and bitrate;
//    - BRIGHTNESS/COLOUR:    brightness, contrast, saturation;
//    - EFFECTS:              special_effect (negative, b/w, sepia, tints);
//    - GEOMETRY:             hmirror, vflip (handy when the camera on the robot
//                            ends up upside down);
//    - AUTOMATICS:           awb/awb_gain/wb_mode (white balance),
//                            aec/aec2/ae_level/aec_value (exposure),
//                            agc/agc_gain/gainceiling (gain -> noise in the dark);
//    - CORRECTIONS:          bpc, wpc, raw_gma, lenc, dcw;
//    - DIAGNOSTICS:          colorbar (test bars instead of the picture — if they
//                            show up but the image doesn't, the problem is optics/light,
//                            not the data bus).
//
//  Endpoints:
//    :80/                          HTML panel + stream
//    :80/status                    JSON with ALL current settings
//    :80/control?var=NAME&val=N    change one setting
//    :80/stats                     JSON: fps/kbps/clients/rssi/heap
//    :81/stream                    MJPEG stream (as in lesson 24)
//
//  Build/flash:
//    pio run -e lesson25_cam_controls -t upload
//    pio device monitor
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

#define PART_BOUNDARY "esp32camframe"
static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY     = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART         = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t webServer    = NULL;
httpd_handle_t streamServer = NULL;

volatile float streamFps     = 0;
volatile float streamKbps    = 0;
volatile int   streamClients = 0;
int flashLevel = 0;                      // remembered so /status can report it

// quality here is NOT a sensor register. The GC2145 on our board can't do JPEG,
// the frame is compressed by frame2jpg (see lesson 23), and its scale differs: 0..100, higher = better.
// So the only "non-hardware" control on the page is this variable.
int jpegQuality = 80;
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


// ---- Flash light and status indicator (pins aren't standardised on S3 boards -> -1 in pins.h) ----
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

  cfg.pixel_format = PIXFORMAT_RGB565;   // GC2145 doesn't output JPEG — we compress in software
  cfg.jpeg_quality = 12;                 // unused with RGB565

  // Compression runs on the CPU, its cost grows with frame area: we start at QVGA,
  // and the framesize list lets you go higher and watch the fps drop.
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
//  THE PANEL. The list of controls is described as an array in JS and drawn in a loop —
//  otherwise markup for 25 settings wouldn't fit into the flash string.
//  t: 'r' = slider (range), 's' = list (select), 'c' = checkbox.
// ---------------------------------------------------------------------------
static const char PAGE_HTML[] = R"HTML(
<!doctype html><html lang="ru"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Урок 25 — возможности OV2640</title>
<style>
 body{font-family:system-ui,sans-serif;background:#111;color:#eee;margin:0;padding:14px}
 h1{font-size:18px;margin:0 0 10px}
 h2{font-size:14px;color:#8cf;margin:16px 0 6px;border-bottom:1px solid #333}
 .wrap{display:flex;gap:18px;flex-wrap:wrap;align-items:flex-start}
 .panel{min-width:280px;max-width:340px}
 img{max-width:100%;border:1px solid #444;background:#000;display:block}
 .c{display:flex;justify-content:space-between;align-items:center;gap:8px;margin:5px 0;font-size:14px}
 .c label{flex:1}
 .c input[type=range]{flex:1}
 .c output{width:44px;text-align:right;font-variant-numeric:tabular-nums;color:#8cf}
 #stats{color:#8cf;font-variant-numeric:tabular-nums;margin-top:8px}
 button{padding:6px 10px;cursor:pointer}
 .st{font-size:11px;color:#f88;white-space:nowrap}
 .c.bad label{color:#777;text-decoration:line-through}
</style></head><body>
<h1>Урок 25 — что умеет OV2640</h1>
<div class="wrap">
  <div>
    <img id="cam" alt="поток">
    <div id="stats">-</div>
    <div style="margin-top:8px"><button onclick="open_()">переоткрыть поток</button>
    <button onclick="load()">перечитать /status</button></div>
  </div>
  <div class="panel" id="panel"></div>
</div>
<script>
const GROUPS=[
 ["Размер и качество",[
  {v:'framesize',t:'s',o:[[5,'QVGA 320x240'],[6,'CIF 400x296'],[7,'HVGA 480x320'],[8,'VGA 640x480'],[9,'SVGA 800x600'],[10,'XGA 1024x768'],[11,'HD 1280x720'],[12,'SXGA 1280x1024'],[13,'UXGA 1600x1200']],l:'разрешение'},
  {v:'quality',t:'r',min:10,max:95,l:'quality (программное сжатие, больше=лучше)'},
 ]],
 ["Яркость и цвет",[
  {v:'brightness',t:'r',min:-2,max:2,l:'brightness'},
  {v:'contrast',t:'r',min:-2,max:2,l:'contrast'},
  {v:'saturation',t:'r',min:-2,max:2,l:'saturation'},
  {v:'special_effect',t:'s',o:[[0,'без эффекта'],[1,'негатив'],[2,'ч/б'],[3,'красный'],[4,'зелёный'],[5,'синий'],[6,'сепия']],l:'special_effect'},
 ]],
 ["Геометрия",[
  {v:'hmirror',t:'c',l:'hmirror (зеркало по X)'},
  {v:'vflip',t:'c',l:'vflip (переворот по Y)'},
  {v:'dcw',t:'c',l:'dcw (масштабирование)'},
 ]],
 ["Баланс белого",[
  {v:'awb',t:'c',l:'awb (авто ББ)'},
  {v:'awb_gain',t:'c',l:'awb_gain'},
  {v:'wb_mode',t:'s',o:[[0,'auto'],[1,'солнце'],[2,'облачно'],[3,'офис'],[4,'дом']],l:'wb_mode (при awb_gain=1)'},
 ]],
 ["Экспозиция",[
  {v:'aec',t:'c',l:'aec (авто экспозиция)'},
  {v:'aec2',t:'c',l:'aec2 (ночной алгоритм)'},
  {v:'ae_level',t:'r',min:-2,max:2,l:'ae_level'},
  {v:'aec_value',t:'r',min:0,max:1200,l:'aec_value (при aec=0)'},
 ]],
 ["Усиление (шум в темноте)",[
  {v:'agc',t:'c',l:'agc (авто усиление)'},
  {v:'agc_gain',t:'r',min:0,max:30,l:'agc_gain (при agc=0)'},
  {v:'gainceiling',t:'r',min:0,max:6,l:'gainceiling (2x..128x)'},
 ]],
 ["Коррекции и диагностика",[
  {v:'bpc',t:'c',l:'bpc (битые пиксели)'},
  {v:'wpc',t:'c',l:'wpc (белые пиксели)'},
  {v:'raw_gma',t:'c',l:'raw_gma (гамма)'},
  {v:'lenc',t:'c',l:'lenc (коррекция объектива)'},
  {v:'colorbar',t:'c',l:'colorbar (тестовые полосы)'},
  {v:'flash',t:'r',min:0,max:255,l:'вспышка GPIO4'},
 ]],
];
const cam=document.getElementById('cam');
function open_(){ cam.src='http://'+location.hostname+':81/stream?t='+Date.now(); }

async function set(v,val){
  const el=document.getElementById('f_'+v);
  const row=el?el.closest('.c'):null;
  try{
    const t=await (await fetch('/control?var='+v+'&val='+val)).text();
    if(!row) return;
    const bad=(t!=='ok');
    row.classList.toggle('bad',bad);
    row.querySelector('.st').textContent=bad?'не поддерживается сенсором':'';
  }catch(e){}
}

const panel=document.getElementById('panel');
for(const [title,items] of GROUPS){
  const h=document.createElement('h2'); h.textContent=title; panel.appendChild(h);
  for(const it of items){
    const row=document.createElement('div'); row.className='c';
    const lab=document.createElement('label'); lab.textContent=it.l; row.appendChild(lab);
    let el;
    if(it.t==='r'){
      el=document.createElement('input'); el.type='range'; el.min=it.min; el.max=it.max;
      const out=document.createElement('output');
      el.oninput=()=>{ out.textContent=el.value; set(it.v,el.value); };
      row.appendChild(el); row.appendChild(out); el.dataset.out='1'; el._out=out;
    }else if(it.t==='c'){
      el=document.createElement('input'); el.type='checkbox';
      el.onchange=()=>set(it.v,el.checked?1:0);
      row.appendChild(el);
    }else{
      el=document.createElement('select');
      for(const [val,text] of it.o){ const o=document.createElement('option'); o.value=val; o.textContent=text; el.appendChild(o); }
      // framesize changes the frame size -> the stream must be reopened, otherwise <img> freezes
      el.onchange=()=>{ set(it.v,el.value); if(it.v==='framesize') setTimeout(open_,300); };
      row.appendChild(el);
    }
    el.id='f_'+it.v;
    const st=document.createElement('span'); st.className='st'; row.appendChild(st);
    panel.appendChild(row);
  }
}
async function load(){
  const s=await (await fetch('/status')).json();
  for(const k in s){
    const el=document.getElementById('f_'+k); if(!el) continue;
    if(el.type==='checkbox') el.checked=!!s[k];
    else { el.value=s[k]; if(el._out) el._out.textContent=s[k]; }
  }
}
setInterval(async()=>{
  try{ const s=await (await fetch('/stats')).json();
    document.getElementById('stats').textContent=
      s.fps.toFixed(1)+' к/с, '+s.kbps.toFixed(0)+' кбит/с, RSSI '+s.rssi+' dBm, heap '+s.heap;
  }catch(e){}
},1000);
load(); open_();
</script></body></html>
)HTML";

static esp_err_t indexHandler(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  return httpd_resp_send(req, PAGE_HTML, HTTPD_RESP_USE_STRLEN);
}

// ---------------------------------------------------------------------------
//  Stream — the same as in lesson 24.
// ---------------------------------------------------------------------------
static esp_err_t streamHandler(httpd_req_t* req) {
  esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  streamClients++;
  char partBuf[72];
  int64_t windowStart = esp_timer_get_time();
  uint32_t windowFrames = 0, windowBytes = 0;

  while (true) {
    // Take the frame under camLock: while we hold fb, nobody is allowed to
    // deinit the camera from the /control?var=framesize handler.
    xSemaphoreTake(camLock, portMAX_DELAY);
    camera_fb_t* fb = esp_camera_fb_get();
    // The sensor outputs raw RGB565 — we compress ourselves (see lesson 23).
    uint8_t* jpg = nullptr;
    size_t   jpg_len = 0;
    bool ok = fb && frame2jpg(fb, jpegQuality, &jpg, &jpg_len);
    if (fb) esp_camera_fb_return(fb);  // the driver no longer needs the raw frame
    xSemaphoreGive(camLock);

    if (!fb || !ok) { res = ESP_FAIL; break; }

    res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res == ESP_OK) {
      size_t hlen = snprintf(partBuf, sizeof(partBuf), STREAM_PART, (unsigned)jpg_len);
      res = httpd_resp_send_chunk(req, partBuf, hlen);
    }
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char*)jpg, jpg_len);

    windowFrames++;
    windowBytes += jpg_len;
    free(jpg);                         // the frame2jpg buffer is ours
    if (res != ESP_OK) break;

    int64_t now = esp_timer_get_time();
    if (now - windowStart >= 1000000) {
      float sec = (now - windowStart) / 1000000.0f;
      streamFps  = windowFrames / sec;
      streamKbps = (windowBytes * 8.0f / 1000.0f) / sec;
      windowStart = now; windowFrames = 0; windowBytes = 0;
    }
  }

  streamClients--;
  streamFps = 0; streamKbps = 0;
  return res;
}

// ---------------------------------------------------------------------------
//  /control?var=NAME&val=N — a single entry point for ALL sensor settings.
//  Each branch is a call to a sensor_t method, i.e. a write to OV2640 registers.
//  Return 0 = ok, -1 = the sensor doesn't support it (e.g. sharpness on the OV2640).
// ---------------------------------------------------------------------------
static int applyControl(const char* var, int val) {
  // The flash isn't a sensor setting but our own LED, yet the panel finds a
  // single endpoint more convenient. If the pin isn't found (CAM_FLASH_LED = -1), the
  // slider simply has no effect.
  if (!strcmp(var, "flash")) {
    flashLevel = constrain(val, 0, 255);
    flashSet(flashLevel);
    return 0;
  }

  // quality is handled BEFORE the sensor: for us it's a software codec parameter,
  // not a register. s->set_quality() on an RGB565 sensor would just return -1.
  if (!strcmp(var, "quality")) {
    jpegQuality = constrain(val, 10, 95);
    return 0;
  }

  // framesize is also handled BEFORE the sensor, for a completely different reason than
  // quality: set_framesize() doesn't recreate the frame buffers, and with RGB565 their size
  // must match the frame. Hence a full camera re-initialisation.
  if (!strcmp(var, "framesize")) {
    framesize_t fs = (framesize_t)constrain(val, (int)FRAMESIZE_96X96, (int)FRAMESIZE_UXGA);
    return applyFramesize(fs) ? 0 : -1;
  }

  sensor_t* s = esp_camera_sensor_get();
  if (!s) return -1;

  if (!strcmp(var, "brightness"))     return s->set_brightness(s, val);
  if (!strcmp(var, "contrast"))       return s->set_contrast(s, val);
  if (!strcmp(var, "saturation"))     return s->set_saturation(s, val);
  if (!strcmp(var, "sharpness"))      return s->set_sharpness(s, val);   // OV2640: not supported, returns -1
  if (!strcmp(var, "special_effect")) return s->set_special_effect(s, val);
  if (!strcmp(var, "hmirror"))        return s->set_hmirror(s, val);
  if (!strcmp(var, "vflip"))          return s->set_vflip(s, val);
  if (!strcmp(var, "dcw"))            return s->set_dcw(s, val);
  if (!strcmp(var, "awb"))            return s->set_whitebal(s, val);
  if (!strcmp(var, "awb_gain"))       return s->set_awb_gain(s, val);
  if (!strcmp(var, "wb_mode"))        return s->set_wb_mode(s, val);
  if (!strcmp(var, "aec"))            return s->set_exposure_ctrl(s, val);
  if (!strcmp(var, "aec2"))           return s->set_aec2(s, val);
  if (!strcmp(var, "ae_level"))       return s->set_ae_level(s, val);
  if (!strcmp(var, "aec_value"))      return s->set_aec_value(s, constrain(val, 0, 1200));
  if (!strcmp(var, "agc"))            return s->set_gain_ctrl(s, val);
  if (!strcmp(var, "agc_gain"))       return s->set_agc_gain(s, constrain(val, 0, 30));
  if (!strcmp(var, "gainceiling"))    return s->set_gainceiling(s, (gainceiling_t)constrain(val, 0, 6));
  if (!strcmp(var, "bpc"))            return s->set_bpc(s, val);
  if (!strcmp(var, "wpc"))            return s->set_wpc(s, val);
  if (!strcmp(var, "raw_gma"))        return s->set_raw_gma(s, val);
  if (!strcmp(var, "lenc"))           return s->set_lenc(s, val);
  if (!strcmp(var, "colorbar"))       return s->set_colorbar(s, val);
  return -1;
}

static esp_err_t controlHandler(httpd_req_t* req) {
  char query[96], var[24], val[12];
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
      httpd_query_key_value(query, "var", var, sizeof(var)) != ESP_OK ||
      httpd_query_key_value(query, "val", val, sizeof(val)) != ESP_OK) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "нужны var и val");
    return ESP_FAIL;
  }

  int v  = atoi(val);
  int rc = applyControl(var, v);
  Serial.printf("[CTRL] %s = %d -> rc=%d%s\n", var, v, rc,
                rc ? "  (сенсор не поддерживает)" : "");

  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_send(req, rc == 0 ? "ok" : "unsupported", HTTPD_RESP_USE_STRLEN);
}

// ---------------------------------------------------------------------------
//  /status — a snapshot of ALL settings. We read not our own variables but s->status:
//  that's where the driver keeps what was actually written to the sensor.
// ---------------------------------------------------------------------------
static esp_err_t statusHandler(httpd_req_t* req) {
  sensor_t* s = esp_camera_sensor_get();
  if (!s) { httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no sensor"); return ESP_FAIL; }

  char json[640];
  snprintf(json, sizeof(json),
    "{\"framesize\":%u,\"quality\":%u,\"brightness\":%d,\"contrast\":%d,\"saturation\":%d,"
    "\"special_effect\":%u,\"hmirror\":%u,\"vflip\":%u,\"dcw\":%u,"
    "\"awb\":%u,\"awb_gain\":%u,\"wb_mode\":%u,"
    "\"aec\":%u,\"aec2\":%u,\"ae_level\":%d,\"aec_value\":%u,"
    "\"agc\":%u,\"agc_gain\":%u,\"gainceiling\":%u,"
    "\"bpc\":%u,\"wpc\":%u,\"raw_gma\":%u,\"lenc\":%u,\"colorbar\":%u,\"flash\":%d}",
    s->status.framesize, jpegQuality, s->status.brightness, s->status.contrast,
    s->status.saturation, s->status.special_effect, s->status.hmirror, s->status.vflip,
    s->status.dcw, s->status.awb, s->status.awb_gain, s->status.wb_mode,
    s->status.aec, s->status.aec2, s->status.ae_level, s->status.aec_value,
    s->status.agc, s->status.agc_gain, s->status.gainceiling,
    s->status.bpc, s->status.wpc, s->status.raw_gma, s->status.lenc,
    s->status.colorbar, flashLevel);

  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t statsHandler(httpd_req_t* req) {
  char json[192];
  snprintf(json, sizeof(json),
           "{\"fps\":%.1f,\"kbps\":%.0f,\"clients\":%d,\"rssi\":%d,\"heap\":%u}",
           streamFps, streamKbps, streamClients, WiFi.RSSI(), (unsigned)ESP.getFreeHeap());
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

void startServers() {
  httpd_config_t web = HTTPD_DEFAULT_CONFIG();
  web.server_port = 80;
  web.ctrl_port   = 32768;
  web.max_uri_handlers = 8;

  httpd_uri_t uriIndex   = { "/",        HTTP_GET, indexHandler,   NULL };
  httpd_uri_t uriControl = { "/control", HTTP_GET, controlHandler, NULL };
  httpd_uri_t uriStatus  = { "/status",  HTTP_GET, statusHandler,  NULL };
  httpd_uri_t uriStats   = { "/stats",   HTTP_GET, statsHandler,   NULL };

  if (httpd_start(&webServer, &web) == ESP_OK) {
    httpd_register_uri_handler(webServer, &uriIndex);
    httpd_register_uri_handler(webServer, &uriControl);
    httpd_register_uri_handler(webServer, &uriStatus);
    httpd_register_uri_handler(webServer, &uriStats);
    Serial.println("[WEB] панель на :80");
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

void setup() {
  Serial.begin(115200);
  delay(1500);      // native USB needs time to come up, otherwise the first lines are lost
  Serial.println("\n=== Урок 25. Возможности OV2640 ===");

  statusLed(true);
  flashInit();

  camLock = xSemaphoreCreateMutex();

  if (!initCamera(FRAMESIZE_QVGA)) {
    Serial.println("[CAM] камера не поднялась — прогони lesson_check_cam.");
    return;
  }

  // Initial "default" correction, as in the CameraWebServer example:
  // out of the box the sensor gives a noticeably dull, undersaturated picture.
  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    s->set_brightness(s, 1);
    s->set_saturation(s, 1);
    // The camera on a robot is often mounted upside down — then uncomment:
    // s->set_vflip(s, 1);
    // s->set_hmirror(s, 1);
    camera_sensor_info_t* info = esp_camera_sensor_get_info(&s->id);
    Serial.printf("[CAM] сенсор %s (PID 0x%04x)\n", info ? info->name : "?", s->id.PID);
  }

  WiFi.begin(SSID, PASS);
  Serial.print("[WiFi] Подключение");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.printf("\n[WiFi] IP: %s (RSSI %d dBm)\n",
                WiFi.localIP().toString().c_str(), WiFi.RSSI());
  WiFi.setSleep(false);

  startServers();
  Serial.printf("[WEB] открой http://%s/\n", WiFi.localIP().toString().c_str());

  statusLed(false);
}

void loop() {
  delay(1000);   // all the work happens in esp_http_server tasks
}
