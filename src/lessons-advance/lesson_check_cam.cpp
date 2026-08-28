// ============================================================================
//  CHECK: ESP32-S3-CAM (OV2640) — проверка камеры перед уроком 23
// ----------------------------------------------------------------------------
//  Цель: убедиться, что модуль камеры живой и кадры реально снимаются, ДО того
//  как подмешивать Wi-Fi и веб-сервер (уроки 23-26). Никакой сети — только
//  esp_camera + Serial. Тот же принцип, что у lesson_check_mpu и
//  lesson_check_tb6612: сначала железо, потом протоколы.
//
//  ВАЖНО: это ДРУГАЯ ПЛАТА И ДРУГОЙ ЧИП. Уроки 02-22 собирались под
//  ESP32-WROOM-32 DevKit (board = esp32dev), камерные — под HW-679 ESP32-S3-CAM
//  (ESP32-S3-WROOM-1 N16R8, board = esp32-s3-devkitc-1 + memory_type = qio_opi).
//  У S3 USB-C НАТИВНЫЙ: перемычка GPIO0 на GND не нужна, прошивка и монитор идут
//  по одному кабелю. Если upload не стартует — зажать BOOT, коротко нажать RST.
//  Подробности: documentation/esp32cam_hardware.md
//
//  Что делает:
//    - печатает модель чипа, объём и тип PSRAM (от неё зависит разрешение);
//    - инициализирует камеру (esp_camera_init) и печатает модель сенсора;
//    - раз в секунду снимает кадр и печатает разрешение + размер JPEG в байтах;
//    - если пин подсветки неизвестен (CAM_FLASH_LED = -1), включает «охоту за
//      LED»: по очереди мигает безопасными GPIO и пишет, каким именно. Смотри на
//      плату — какой мигнул, тот и вписывай в include/pins.h.
//
//  Как читать вывод:
//    - "[SYS] PSRAM: найдена, 8192 КБ" -> memory_type в platformio.ini угадан верно;
//    - "[SYS] PSRAM: НЕ найдена" на модуле N16R8 -> неверный memory_type
//      (для octal-PSRAM нужен qio_opi, для quad — qio_qspi);
//    - "[CAM] init OK" + "сенсор PID 0x0026 (OV2640)" -> шлейф вставлен верно;
//    - "кадр 640x480, 23456 байт" -> камера снимает;
//    - размер JPEG "дышит" при смене освещения -> это нормально: степень сжатия
//      зависит от картинки, чем больше деталей и шума, тем больше байт;
//    - init FAILED 0x20001 (ESP_ERR_CAMERA_NOT_DETECTED) -> шлейф не защёлкнут
//      до конца, вставлен другой стороной, или выбрана не та CAM_MODEL_* в pins.h;
//    - монитор вообще пустой -> на плате UART-мост, а не нативный USB: поставь
//      -DARDUINO_USB_CDC_ON_BOOT=0 в platformio.ini (см. комментарий там).
//
//  Сборка/прошивка:
//    pio run -e lesson_check_cam -t upload
//    pio device monitor
//
//  Пины камеры — в include/pins.h (блок CAM_*, выбор модели через CAM_MODEL_*).
//  Мы их НЕ выбираем: шлейф OV2640 разведён на плате жёстко.
// ============================================================================

#include <Arduino.h>
#include "esp_camera.h"
#include "pins.h"

// ---- Настройки съёмки ------------------------------------------------------
// jpeg_quality: 0..63, МЕНЬШЕ = лучше картинка и БОЛЬШЕ байт (это счётчик
// грубости сжатия, а не проценты). 10-12 — обычный рабочий диапазон.
const int JPEG_QUALITY = 12;
const unsigned long SHOT_INTERVAL = 1000;   // мс между кадрами в этом тесте

// «Охота за LED»: имеет смысл только пока пин подсветки не найден.
// Список — заведомо свободные GPIO HW-679. Камерные, USB (19/20), UART0 (43/44),
// strapping (0/45/46) и линии flash/PSRAM (26..37) сюда не попадают СПЕЦИАЛЬНО.
const int LED_CANDIDATES[] = { 1, 2, 3, 14, 21, 47, 48 };

unsigned long lastShot       = 0;
unsigned long frames         = 0;   // сколько кадров снято всего
unsigned long framesInWindow = 0;   // сколько снято за текущее 10-секундное окно
unsigned long lastFpsPrint   = 0;
bool cameraReady = false;

// ---------------------------------------------------------------------------
//  Подсветка. У AI-Thinker белый LED жёстко на GPIO4, а у S3-плат разводка
//  гуляет, поэтому по умолчанию CAM_FLASH_LED = -1 и весь этот код просто не
//  компилируется. Найдёшь свой пин — впиши в pins.h, и он оживёт сам.
// ---------------------------------------------------------------------------
void flashInit() {
#if CAM_FLASH_LED >= 0
  ledcSetup(CH_CAM_FLASH, 5000, 8);            // 5 кГц, 8 бит (0..255)
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
//  Перебор кандидатов: мигаем каждым по 3 раза и пишем, каким именно. Смотри на
//  плату. Пины взяты только из свободных, так что «дёрнуть» их безопасно.
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
    // Возвращаем пин в высокоимпедансное состояние — вдруг он всё-таки чей-то.
    pinMode(pin, INPUT);
    delay(500);
  }
  Serial.println("[LED] Перебор закончен. Ничего не мигало — значит подсветки на");
  Serial.println("      плате нет (частый случай), просто оставь -1.\n");
}

// ---------------------------------------------------------------------------
//  Заполняем структуру конфигурации камеры.
//  Никакой магии: перечисляем, к каким GPIO подключён сенсор, и в каком виде
//  хотим получать кадры.
// ---------------------------------------------------------------------------
bool initCamera() {
  camera_config_t cfg = {};      // сначала обнуляем структуру целиком

  // 1) Пины. Шина данных у камеры ПАРАЛЛЕЛЬНАЯ (8 бит D0..D7 плюс три строба),
  //    поэтому пинов так много — это не I2C-датчик на двух проводах.
  cfg.pin_pwdn     = CAM_PWDN;     // -1 = линии на плате нет
  cfg.pin_reset    = CAM_RESET;
  cfg.pin_xclk     = CAM_XCLK;
  cfg.pin_sccb_sda = CAM_SIOD;     // SCCB = "почти I2C", по нему настраивается сенсор
  cfg.pin_sccb_scl = CAM_SIOC;
  cfg.pin_d7 = CAM_Y9;  cfg.pin_d6 = CAM_Y8;  cfg.pin_d5 = CAM_Y7;  cfg.pin_d4 = CAM_Y6;
  cfg.pin_d3 = CAM_Y5;  cfg.pin_d2 = CAM_Y4;  cfg.pin_d1 = CAM_Y3;  cfg.pin_d0 = CAM_Y2;
  cfg.pin_vsync = CAM_VSYNC;       // строб "начался новый кадр"
  cfg.pin_href  = CAM_HREF;        // строб "идёт строка"
  cfg.pin_pclk  = CAM_PCLK;        // строб "готов очередной пиксель"

  // 2) Тактирование сенсора. XCLK 20 МГц — стандарт для OV2640. Генерирует его
  //    сам ESP32 через LEDC (да, тот же PWM-модуль, что крутил RGB в уроке 3):
  //    камера занимает LEDC_TIMER_0 + LEDC_CHANNEL_0, поэтому подсветку выше
  //    вешаем на канал 7 (CH_CAM_FLASH), чтобы не отобрать у камеры таймер.
  //    У S3 драйвер умеет режим EDMA на 16 МГц (быстрее, но помечен как
  //    экспериментальный) — если захочешь попробовать, поставь 16000000.
  cfg.xclk_freq_hz = 20000000;
  cfg.ledc_timer   = LEDC_TIMER_0;
  cfg.ledc_channel = LEDC_CHANNEL_0;

  // 3) Формат кадра. PIXFORMAT_JPEG = сенсор отдаёт УЖЕ СЖАТЫЙ кадр. Это
  //    принципиально: VGA в RGB565 — это 640*480*2 = 614 400 байт, а в JPEG —
  //    20-40 КБ. Без сжатия поток в браузер не влез бы ни в RAM, ни в Wi-Fi.
  cfg.pixel_format = PIXFORMAT_JPEG;
  cfg.jpeg_quality = JPEG_QUALITY;

  // 4) Разрешение и буферы зависят от наличия PSRAM. У N16R8 её 8 МБ — можно
  //    смело брать VGA и два буфера (и вообще вплоть до UXGA, см. урок 25).
  //    Без PSRAM кадр должен уместиться во внутреннюю RAM — только QVGA.
  if (psramFound()) {
    cfg.frame_size  = FRAMESIZE_VGA;         // 640x480
    cfg.fb_count    = 2;                     // двойная буферизация: снимаем и отдаём параллельно
    cfg.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    cfg.frame_size  = FRAMESIZE_QVGA;        // 320x240
    cfg.fb_count    = 1;
    cfg.fb_location = CAMERA_FB_IN_DRAM;
  }

  // CAMERA_GRAB_WHEN_EMPTY — заполнять буфер, когда он освободился (экономно).
  // CAMERA_GRAB_LATEST — всегда отдавать самый свежий кадр; для стрима это
  // важнее (урок 24), иначе видео отстаёт на fb_count кадров.
  cfg.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

  esp_err_t err = esp_camera_init(&cfg);
  if (err != ESP_OK) {
    Serial.printf("[CAM] init FAILED: 0x%x (%s)\n", err, esp_err_to_name(err));
    Serial.println("      0x20001 = сенсор не найден: проверь шлейф, защёлку разъёма");
    Serial.println("                и выбранную CAM_MODEL_* в include/pins.h;");
    Serial.println("      0x105   = не хватило памяти: уменьши frame_size / fb_count.");
    return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
//  Кто именно к нам подключён. В ядре есть таблица сенсоров: по id из
//  esp_camera_sensor_get() достаём имя и максимальное разрешение.
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
}

void setup() {
  Serial.begin(115200);
  // У нативного USB порт поднимается не мгновенно: без паузы первые строки
  // теряются, и монитор кажется пустым. На UART-мосте эта задержка не мешает.
  delay(1500);
  Serial.println("\n=== CHECK ESP32-S3-CAM ===");

  statusLed(true);        // горит, пока идёт инициализация (если LED вообще есть)
  flashInit();

  // Чип и память. PSRAM — главный вопрос: без неё нет ни VGA, ни двух буферов.
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
  // Короткая проверка подсветки: ~20% яркости на 200 мс.
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

  // Снять кадр. Возвращается УКАЗАТЕЛЬ на буфер внутри драйвера, а не копия.
  unsigned long t0 = millis();
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("[CAM] fb_get вернул NULL (кадра нет). Питание? Шлейф?");
    return;
  }

  frames++;
  framesInWindow++;
  Serial.printf("[CAM] кадр #%lu: %ux%u, %u байт, снят за %lu мс\n",
                frames, (unsigned)fb->width, (unsigned)fb->height,
                (unsigned)fb->len, millis() - t0);

  // ОБЯЗАТЕЛЬНО вернуть буфер драйверу. Забудешь — через fb_count кадров
  // esp_camera_fb_get() начнёт возвращать NULL, и камера "сломается".
  esp_camera_fb_return(fb);

  if (millis() - lastFpsPrint >= 10000) {
    Serial.printf("[CAM] за 10 c снято %lu кадров (здесь мы сами тормозим таймером,\n"
                  "      реальный предел скорости замерим в уроке 24)\n", framesInWindow);
    framesInWindow = 0;
    lastFpsPrint = millis();
  }
}
