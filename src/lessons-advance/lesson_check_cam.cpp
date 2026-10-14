// ============================================================================
//  CHECK: ESP32-S3-CAM (OV2640) — camera check before lesson 23
// ----------------------------------------------------------------------------
//  Goal: make sure the camera module is alive and frames are actually captured
//  BEFORE mixing in Wi-Fi and a web server (lessons 23-26). No network — just
//  esp_camera + Serial. Same principle as lesson_check_mpu and
//  lesson_check_tb6612: hardware first, protocols later.
//
//  IMPORTANT: this is a DIFFERENT BOARD AND A DIFFERENT CHIP. Lessons 02-22 were
//  built for the ESP32-WROOM-32 DevKit (board = esp32dev), the camera ones for the
//  HW-679 ESP32-S3-CAM (ESP32-S3-WROOM-1 N16R8, board = esp32-s3-devkitc-1 +
//  memory_type = qio_opi). On the S3 the USB-C is NATIVE: no GPIO0-to-GND jumper
//  is needed, flashing and the monitor go over one cable. If upload doesn't
//  start — hold BOOT, briefly press RST.
//  Details: documentation/esp32cam_hardware.md
//
//  What it does:
//    - prints the chip model, PSRAM size and type (the resolution depends on it);
//    - initialises the camera (esp_camera_init) and prints the sensor model;
//    - once a second captures a frame and prints the resolution + JPEG size in bytes
//      (and if the sensor has no hardware JPEG — also the software compression time);
//    - if the flash LED pin is unknown (CAM_FLASH_LED = -1), runs an "LED
//      hunt": blinks the safe GPIOs one by one and prints which one. Watch the
//      board — whichever one blinked, put that into include/pins.h.
//
//  How to read the output:
//    - "[SYS] PSRAM: найдена, 8192 КБ" -> memory_type in platformio.ini is right;
//    - "[SYS] PSRAM: НЕ найдена" on an N16R8 module -> wrong memory_type
//      (octal PSRAM needs qio_opi, quad needs qio_qspi);
//    - "[CAM] init OK" + the line "сенсор PID ..." -> the ribbon is seated correctly.
//      PID 0x0026 = OV2640 (has hardware JPEG), PID 0x2145 = GC2145 (doesn't —
//      then the sketch switches to RGB565 + frame2jpg by itself, and the line
//      "[CAM] режим:" will tell exactly how it works);
//    - "кадр 640x480, 23456 байт" -> the camera is capturing;
//    - JPEG size "breathes" when the lighting changes -> that's normal: the
//      compression ratio depends on the picture, the more detail and noise, the more bytes;
//    - init FAILED 0x20001 (ESP_ERR_CAMERA_NOT_DETECTED) -> the ribbon isn't latched
//      all the way, is inserted the wrong way round, or the wrong CAM_MODEL_* is
//      selected in pins.h;
//    - the monitor is completely empty -> the board has a UART bridge, not native USB:
//      set -DARDUINO_USB_CDC_ON_BOOT=0 in platformio.ini (see the comment there).
//
//  Build/flash:
//    pio run -e lesson_check_cam -t upload
//    pio device monitor
//
//  Camera pins are in include/pins.h (CAM_* block, model chosen via CAM_MODEL_*).
//  We do NOT choose them: the sensor ribbon is hard-wired on the board.
// ============================================================================

#include <Arduino.h>
#include "esp_camera.h"
#include "img_converters.h"   // frame2jpg(): RGB565 -> JPEG compression for sensors without hardware JPEG
#include "pins.h"

// ---- Capture settings ------------------------------------------------------
// jpeg_quality: 0..63, LOWER = better picture and MORE bytes (it's a measure
// of compression coarseness, not a percentage). 10-12 is the usual working range.
const int JPEG_QUALITY = 12;
const unsigned long SHOT_INTERVAL = 1000;   // ms between frames in this test

// "LED hunt": only makes sense while the flash LED pin hasn't been found.
// The list is GPIOs known to be free on the HW-679. Camera, USB (19/20), UART0 (43/44),
// strapping (0/45/46) and flash/PSRAM lines (26..37) are left out ON PURPOSE.
const int LED_CANDIDATES[] = { 1, 2, 3, 14, 21, 47, 48 };

unsigned long lastShot       = 0;
unsigned long frames         = 0;   // total frames captured
unsigned long framesInWindow = 0;   // frames captured in the current 10-second window
unsigned long lastFpsPrint   = 0;
bool cameraReady = false;
// Whether the sensor can output a ready JPEG. OV2640 — yes, GC2145 (our board) — no,
// and then the camera runs in RGB565 and the CPU compresses the frame. See initCamera().
bool sensorHasJpeg = false;

// ---------------------------------------------------------------------------
//  Flash LED. On the AI-Thinker the white LED is hard-wired to GPIO4, but S3 boards
//  vary in wiring, so by default CAM_FLASH_LED = -1 and all this code simply
//  doesn't compile. Once you find your pin — put it into pins.h and it comes alive by itself.
// ---------------------------------------------------------------------------
void flashInit() {
#if CAM_FLASH_LED >= 0
  ledcSetup(CH_CAM_FLASH, 5000, 8);            // 5 kHz, 8 bit (0..255)
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
//  Going through the candidates: blink each one 3 times and print which one. Watch the
//  board. The pins are taken only from the free ones, so toggling them is safe.
// ---------------------------------------------------------------------------
void huntForLed() {
  Serial.println("\n[LED] Пин подсветки не задан (CAM_FLASH_LED = -1).");
  Serial.println("[LED] Сейчас по очереди мигну свободными GPIO — смотри на плату.");
  Serial.println("[LED] Что мигнёт, то и впиши в include/pins.h.\n");

  for (int pin : LED_CANDIDATES) {
    Serial.printf("[LED] GPIO %d ...\n", pin);
    pinMode(pin, OUTPUT);
    for (int i = 0; i < 3; i++) {
      digitalWrite(pin, HIGH); delay(180);
      digitalWrite(pin, LOW);  delay(180);
    }
    // Return the pin to high impedance — in case it belongs to something after all.
    pinMode(pin, INPUT);
    delay(500);
  }
  Serial.println("[LED] Перебор закончен. Ничего не мигало — значит подсветки на");
  Serial.println("      плате нет (частый случай), просто оставь -1.\n");
}

// ---------------------------------------------------------------------------
//  Filling in the camera configuration struct.
//  No magic: we list which GPIOs the sensor is connected to, and in what form
//  we want to receive frames.
// ---------------------------------------------------------------------------
bool initCamera() {
  camera_config_t cfg = {};      // first zero the whole struct

  // 1) Pins. The camera's data bus is PARALLEL (8 bits D0..D7 plus three strobes),
  //    which is why there are so many pins — this is not a two-wire I2C sensor.
  cfg.pin_pwdn     = CAM_PWDN;     // -1 = the board has no such line
  cfg.pin_reset    = CAM_RESET;
  cfg.pin_xclk     = CAM_XCLK;
  cfg.pin_sccb_sda = CAM_SIOD;     // SCCB = "almost I2C", the sensor is configured over it
  cfg.pin_sccb_scl = CAM_SIOC;
  cfg.pin_d7 = CAM_Y9;  cfg.pin_d6 = CAM_Y8;  cfg.pin_d5 = CAM_Y7;  cfg.pin_d4 = CAM_Y6;
  cfg.pin_d3 = CAM_Y5;  cfg.pin_d2 = CAM_Y4;  cfg.pin_d1 = CAM_Y3;  cfg.pin_d0 = CAM_Y2;
  cfg.pin_vsync = CAM_VSYNC;       // strobe "a new frame has started"
  cfg.pin_href  = CAM_HREF;        // strobe "a line is in progress"
  cfg.pin_pclk  = CAM_PCLK;        // strobe "the next pixel is ready"

  // 2) Sensor clock. XCLK 20 MHz is the standard for the OV2640. It is generated
  //    by the ESP32 itself via LEDC (yes, the same PWM module that drove the RGB in lesson 3):
  //    the camera takes LEDC_TIMER_0 + LEDC_CHANNEL_0, so the flash LED above
  //    goes on channel 7 (CH_CAM_FLASH), so as not to take the timer from the camera.
  //    On the S3 the driver supports an EDMA mode at 16 MHz (faster, but marked as
  //    experimental) — if you want to try it, set 16000000.
  cfg.xclk_freq_hz = 20000000;
  cfg.ledc_timer   = LEDC_TIMER_0;
  cfg.ledc_channel = LEDC_CHANNEL_0;

  // 3) Frame format. PIXFORMAT_JPEG = the sensor outputs an ALREADY COMPRESSED frame. This
  //    is fundamental: VGA in RGB565 is 640*480*2 = 614 400 bytes, while in JPEG it's
  //    20-40 KB. Without compression a stream to the browser would fit in neither RAM nor Wi-Fi.
  //
  //    BUT not every sensor has hardware JPEG: the OV2640 does, the GC2145 (fitted
  //    on our board) doesn't, and esp_camera_init with PIXFORMAT_JPEG fails on it
  //    with ESP_ERR_NOT_SUPPORTED (0x106). So we try JPEG, and on refusal
  //    re-initialise in RGB565 — after that frame2jpg compresses the frame (lesson 23).
  cfg.pixel_format = PIXFORMAT_JPEG;
  cfg.jpeg_quality = JPEG_QUALITY;

  // 4) Resolution and buffers depend on whether PSRAM is present. The N16R8 has 8 MB —
  //    we can safely take VGA and two buffers (and in fact up to UXGA, see lesson 25).
  //    Without PSRAM the frame must fit in internal RAM — QVGA only.
  if (psramFound()) {
    cfg.frame_size  = FRAMESIZE_VGA;         // 640x480
    cfg.fb_count    = 2;                     // double buffering: capture and hand off in parallel
    cfg.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    cfg.frame_size  = FRAMESIZE_QVGA;        // 320x240
    cfg.fb_count    = 1;
    cfg.fb_location = CAMERA_FB_IN_DRAM;
  }

  // CAMERA_GRAB_WHEN_EMPTY — fill a buffer when it has been freed (economical).
  // CAMERA_GRAB_LATEST — always return the freshest frame; for a stream this
  // matters more (lesson 24), otherwise the video lags by fb_count frames.
  cfg.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

  esp_err_t err = esp_camera_init(&cfg);
  if (err == ESP_OK) {
    sensorHasJpeg = true;
    return true;
  }

  if (err == ESP_ERR_NOT_SUPPORTED) {
    // The sensor was found but can't do JPEG. Not a hardware fault — just a different chip.
    Serial.println("[CAM] сенсор не умеет аппаратный JPEG — переключаюсь на RGB565");
    cfg.pixel_format = PIXFORMAT_RGB565;
    if (psramFound()) {
      cfg.frame_size = FRAMESIZE_QVGA;    // RGB565 is many times heavier than JPEG
    }
    err = esp_camera_init(&cfg);
    if (err == ESP_OK) {
      sensorHasJpeg = false;
      return true;
    }
  }

  Serial.printf("[CAM] init FAILED: 0x%x (%s)\n", err, esp_err_to_name(err));
  Serial.println("      0x20001 = сенсор не найден: проверь шлейф, защёлку разъёма");
  Serial.println("                и выбранную CAM_MODEL_* в include/pins.h;");
  Serial.println("      0x106   = сенсор без аппаратного JPEG, и RGB565 тоже не завёлся;");
  Serial.println("      0x105   = не хватило памяти: уменьши frame_size / fb_count.");
  return false;
}

// ---------------------------------------------------------------------------
//  Who exactly is connected. The core has a sensor table: using the id from
//  esp_camera_sensor_get() we get the name and the maximum resolution.
// ---------------------------------------------------------------------------
void printSensorInfo() {
  sensor_t* s = esp_camera_sensor_get();
  if (!s) { Serial.println("[CAM] sensor_get вернул NULL"); return; }

  camera_sensor_info_t* info = esp_camera_sensor_get_info(&s->id);
  Serial.printf("[CAM] сенсор PID 0x%04x", s->id.PID);
  if (info) {
    Serial.printf(" (%s), JPEG: %s, макс. framesize #%d",
                  info->name, info->support_jpeg ? "да" : "нет", (int)info->max_size);
  }
  Serial.println();
  Serial.printf("[CAM] адрес по SCCB 0x%02x, XCLK %d Гц\n", s->slv_addr, s->xclk_freq_hz);
  Serial.printf("[CAM] режим: %s\n", sensorHasJpeg
                ? "аппаратный JPEG (кадр приходит уже сжатым)"
                : "RGB565 + программное сжатие frame2jpg (так работают уроки 23-26)");
}

void setup() {
  Serial.begin(115200);
  // With native USB the port doesn't come up instantly: without a pause the first lines
  // are lost and the monitor looks empty. On a UART bridge this delay does no harm.
  delay(1500);
  Serial.println("\n=== CHECK ESP32-S3-CAM ===");

  statusLed(true);        // on while initialisation runs (if there is an LED at all)
  flashInit();

  // Chip and memory. PSRAM is the main question: without it there's neither VGA nor two buffers.
  Serial.printf("[SYS] чип %s, ядер %d, ревизия %d\n",
                ESP.getChipModel(), ESP.getChipCores(), ESP.getChipRevision());
  if (psramFound()) {
    Serial.printf("[SYS] PSRAM: найдена, %u КБ (свободно %u КБ)\n",
                  (unsigned)(ESP.getPsramSize() / 1024),
                  (unsigned)(ESP.getFreePsram() / 1024));
  } else {
    Serial.println("[SYS] PSRAM: НЕ найдена!");
    Serial.println("      На модуле N16R8 её 8 МБ — значит дело в настройке:");
    Serial.println("      board_build.arduino.memory_type должен быть qio_opi (octal).");
    Serial.println("      Для модулей N8R2 (quad PSRAM) — qio_qspi.");
  }
  Serial.printf("[SYS] свободная внутренняя RAM: %u КБ\n",
                (unsigned)(ESP.getFreeHeap() / 1024));

  cameraReady = initCamera();
  if (!cameraReady) {
    Serial.println("[CAM] дальше идти некуда — камера не поднялась.");
    return;
  }
  Serial.println("[CAM] init OK");
  printSensorInfo();

#if CAM_FLASH_LED >= 0
  // Quick flash LED check: ~20% brightness for 200 ms.
  Serial.printf("[CAM] проверка подсветки (GPIO %d)...\n", CAM_FLASH_LED);
  flashSet(50);
  delay(200);
  flashSet(0);
#else
  huntForLed();
#endif

  statusLed(false);
  Serial.println("[CAM] снимаю по кадру в секунду. Води рукой перед объективом —");
  Serial.println("      размер JPEG должен заметно меняться.");
}

void loop() {
  if (!cameraReady) { delay(1000); return; }
  if (millis() - lastShot < SHOT_INTERVAL) return;
  lastShot = millis();

  // Capture a frame. What's returned is a POINTER to a buffer inside the driver, not a copy.
  unsigned long t0 = millis();
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("[CAM] fb_get вернул NULL (кадра нет). Питание? Шлейф?");
    return;
  }

  frames++;
  framesInWindow++;
  unsigned long grab_ms = millis() - t0;
  unsigned w = fb->width, h = fb->height, raw_len = fb->len;

  if (sensorHasJpeg) {
    Serial.printf("[CAM] кадр #%lu: %ux%u, JPEG %u байт, снят за %lu мс\n",
                  frames, w, h, raw_len, grab_ms);
    // You MUST return the buffer to the driver. Forget it — and after fb_count frames
    // esp_camera_fb_get() starts returning NULL, and the camera "breaks".
    esp_camera_fb_return(fb);
  } else {
    // We compress the raw frame ourselves — and measure what it costs along the way.
    uint8_t* jpg = nullptr;
    size_t   jpg_len = 0;
    unsigned long t1 = millis();
    bool ok = frame2jpg(fb, 80, &jpg, &jpg_len);
    unsigned long enc_ms = millis() - t1;
    esp_camera_fb_return(fb);           // the raw frame is no longer needed

    if (!ok) {
      Serial.println("[CAM] frame2jpg не смогла сжать кадр (мало памяти?)");
      return;
    }
    Serial.printf("[CAM] кадр #%lu: %ux%u, RGB565 %u байт -> JPEG %u байт "
                  "(снят за %lu мс, сжат за %lu мс)\n",
                  frames, w, h, raw_len, (unsigned)jpg_len, grab_ms, enc_ms);
    free(jpg);                          // the frame2jpg buffer is ours
  }

  if (millis() - lastFpsPrint >= 10000) {
    Serial.printf("[CAM] за 10 c снято %lu кадров (здесь мы сами тормозим таймером,\n"
                  "      реальный предел скорости замерим в уроке 24)\n", framesInWindow);
    framesInWindow = 0;
    lastFpsPrint = millis();
  }
}
