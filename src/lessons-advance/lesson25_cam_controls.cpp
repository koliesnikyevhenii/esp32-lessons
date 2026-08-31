// ============================================================================
//  Этап 6. Камера — урок 25: все возможности сенсора в одной панели
// ----------------------------------------------------------------------------
//  Цель: пощупать, ЧТО вообще умеет сенсор, кроме "выдавать кадры". Урок 24 дал
//  поток; здесь к потоку добавляется панель управления, которая крутит настройки
//  камеры на живом видео — и сразу видно, как каждая из них влияет на картинку
//  и на количество кадров в секунду.
//
//  Главная идея урока: настройки живут не в нашем коде, а В САМОМ СЕНСОРЕ.
//  Драйвер отдаёт структуру sensor_t с указателями на функции:
//
//      sensor_t* s = esp_camera_sensor_get();
//      s->set_brightness(s, 1);        // <- пишем регистры сенсора по SCCB
//      s->status.brightness;           // <- текущее значение
//
//  То есть set_* — это не "фильтр в прошивке", а команда чипу камеры. Поэтому
//  оно бесплатно по процессору: ESP32 не обрабатывает пиксели вообще.
//
//  Ровно одно исключение — quality. На нашей плате стоит GC2145, он не умеет
//  отдавать JPEG, и кадр сжимается программно (frame2jpg, урок 23). Значит
//  quality здесь — параметр НАШЕГО кодека, а не регистр сенсора: шкала 0..100,
//  больше = лучше (у аппаратного JPEG в OV2640 было наоборот, 0..63, меньше =
//  лучше). Заодно видно, чем «настройка железа» отличается от «настройки кода»:
//  все остальные ползунки процессор не грузят, а этот — грузит.
//
//  У GC2145 часть set_* вернёт -1 («unsupported») — так и должно быть, набор
//  регистров у каждого сенсора свой. Панель честно покажет это статусом.
//
//  Что изучаем (и что смотрим глазами):
//    - РАЗМЕР И КАЧЕСТВО:    framesize (сенсор) и quality (наш кодек) -> к/с и битрейт;
//    - ЯРКОСТЬ/ЦВЕТ:         brightness, contrast, saturation;
//    - ЭФФЕКТЫ:              special_effect (негатив, ч/б, сепия, тонировки);
//    - ГЕОМЕТРИЯ:            hmirror, vflip (пригодится, когда камера на роботе
//                            окажется перевёрнутой);
//    - АВТОМАТИКА:           awb/awb_gain/wb_mode (баланс белого),
//                            aec/aec2/ae_level/aec_value (экспозиция),
//                            agc/agc_gain/gainceiling (усиление -> шум в темноте);
//    - КОРРЕКЦИИ:            bpc, wpc, raw_gma, lenc, dcw;
//    - ДИАГНОСТИКА:          colorbar (тестовые полосы вместо картинки — если они
//                            есть, а изображения нет, значит проблема в оптике/свете,
//                            а не в шине данных).
//
//  Эндпоинты:
//    :80/                          HTML-панель + стрим
//    :80/status                    JSON со ВСЕМИ текущими настройками
//    :80/control?var=NAME&val=N    поменять одну настройку
//    :80/stats                     JSON: fps/kbps/clients/rssi/heap
//    :81/stream                    MJPEG-поток (как в уроке 24)
//
//  Сборка/прошивка:
//    pio run -e lesson25_cam_controls -t upload
//    pio device monitor
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

#define PART_BOUNDARY "esp32camframe"
static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY     = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART         = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t webServer    = NULL;
httpd_handle_t streamServer = NULL;

volatile float streamFps     = 0;
volatile float streamKbps    = 0;
volatile int   streamClients = 0;
int flashLevel = 0;                      // помним, чтобы отдавать в /status

// quality здесь — НЕ регистр сенсора. GC2145 на нашей плате не умеет JPEG,
// кадр жмёт frame2jpg (см. урок 23), и её шкала другая: 0..100, больше = лучше.
// Поэтому единственный «нежелезный» контрол на странице — вот эта переменная.
int jpegQuality = 80;

// ---- Подсветка и индикатор (на S3-платах пины не стандартизованы -> в pins.h -1) ----
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

  cfg.pixel_format = PIXFORMAT_RGB565;   // GC2145 не отдаёт JPEG — жмём программно
  cfg.jpeg_quality = 12;                 // при RGB565 не используется

  if (psramFound()) {
    // Сжатие процессорное, его цена растёт с площадью кадра: стартуем с QVGA,
    // а ползунком framesize можно поднять и увидеть, как проседают к/с.
    cfg.frame_size  = FRAMESIZE_QVGA;
    cfg.fb_count    = 2;
    cfg.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
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
  return true;
}

// ---------------------------------------------------------------------------
//  ПАНЕЛЬ. Список контролов описан массивом в JS и рисуется циклом — иначе
//  вёрстка на 25 настроек не влезла бы во флеш-строку.
//  t: 'r' = слайдер (range), 's' = список (select), 'c' = галочка (checkbox).
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
function set(v,val){ fetch('/control?var='+v+'&val='+val); }

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
      // framesize меняет размер кадра -> поток надо переоткрыть, иначе <img> залипает
      el.onchange=()=>{ set(it.v,el.value); if(it.v==='framesize') setTimeout(open_,300); };
      row.appendChild(el);
    }
    el.id='f_'+it.v;
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
//  Стрим — тот же, что в уроке 24.
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
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) { res = ESP_FAIL; break; }

    // Сенсор отдаёт сырой RGB565 — сжимаем сами (см. урок 23).
    uint8_t* jpg = nullptr;
    size_t   jpg_len = 0;
    bool ok = frame2jpg(fb, jpegQuality, &jpg, &jpg_len);
    esp_camera_fb_return(fb);          // сырой кадр драйверу больше не нужен
    if (!ok) { res = ESP_FAIL; break; }

    res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res == ESP_OK) {
      size_t hlen = snprintf(partBuf, sizeof(partBuf), STREAM_PART, (unsigned)jpg_len);
      res = httpd_resp_send_chunk(req, partBuf, hlen);
    }
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char*)jpg, jpg_len);

    windowFrames++;
    windowBytes += jpg_len;
    free(jpg);                         // буфер от frame2jpg наш
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
//  /control?var=NAME&val=N — единая точка входа для ВСЕХ настроек сенсора.
//  Каждая ветка — вызов метода из sensor_t, то есть запись в регистры OV2640.
//  Возврат 0 = ок, -1 = сенсор не поддерживает (например sharpness у OV2640).
// ---------------------------------------------------------------------------
static int applyControl(const char* var, int val) {
  // Вспышка — не настройка сенсора, а наш светодиод, но панели удобнее иметь
  // единый эндпоинт. Если пин не найден (CAM_FLASH_LED = -1) — ползунок просто
  // ни на что не влияет.
  if (!strcmp(var, "flash")) {
    flashLevel = constrain(val, 0, 255);
    flashSet(flashLevel);
    return 0;
  }

  // quality обрабатываем ДО сенсора: у нас это параметр программного кодека,
  // а не регистр. s->set_quality() на RGB565-сенсоре просто вернул бы -1.
  if (!strcmp(var, "quality")) {
    jpegQuality = constrain(val, 10, 95);
    return 0;
  }

  sensor_t* s = esp_camera_sensor_get();
  if (!s) return -1;

  if (!strcmp(var, "framesize"))      return s->set_framesize(s, (framesize_t)constrain(val, 0, (int)FRAMESIZE_UXGA));
  if (!strcmp(var, "brightness"))     return s->set_brightness(s, val);
  if (!strcmp(var, "contrast"))       return s->set_contrast(s, val);
  if (!strcmp(var, "saturation"))     return s->set_saturation(s, val);
  if (!strcmp(var, "sharpness"))      return s->set_sharpness(s, val);   // OV2640: не поддерживается, вернёт -1
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
//  /status — снимок ВСЕХ настроек. Читаем не свои переменные, а s->status:
//  драйвер хранит там то, что реально записано в сенсор.
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
  delay(1500);      // нативному USB нужно время подняться, иначе первые строки теряются
  Serial.println("\n=== Урок 25. Возможности OV2640 ===");

  statusLed(true);
  flashInit();

  if (!initCamera()) {
    Serial.println("[CAM] камера не поднялась — прогони lesson_check_cam.");
    return;
  }

  // Стартовая коррекция "по умолчанию", как в примере CameraWebServer:
  // сенсор из коробки даёт заметно тусклую и малонасыщенную картинку.
  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    s->set_brightness(s, 1);
    s->set_saturation(s, 1);
    // Камера на роботе часто стоит вверх ногами — тогда раскомментируй:
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
  delay(1000);   // вся работа — в задачах esp_http_server
}
