# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A PlatformIO project of incremental ESP32 learning exercises (Arduino framework). Each lesson is a **self-contained sketch** in `src/lessons-basic/lessonNN_name.cpp` with its own `setup()` and `loop()`. Only one lesson is compiled per build, selected by a PlatformIO env (see below) — this avoids the duplicate `setup()`/`loop()` and global-variable collisions you'd get if they all compiled together. The shared pin/channel map lives in `include/pins.h`.

Default board: `esp32dev` (ESP32-WROOM-32 DevKit) — **except the camera lessons 23–26**, which
run on an ESP32-**S3**-CAM and override the board (and PSRAM/USB settings) via `extends = cam_s3`
in their env (see below). Serial monitor baud: `115200`. Code comments are in Russian; pin/lesson labels and identifiers are English.

## Layout

```
include/pins.h                     # shared pin + LEDC channel #defines (the single source of truth for wiring)
src/lessons-basic/lessonNN_*.cpp   # standalone basic sketches (lessons 02–18)
src/lessons-advance/lessonNN_*.cpp # advanced IoT/robot/camera sketches (lessons 19–26, + lesson_check_* diagnostics)
src/lessons-advance/documentation/ # per-lesson notes for the advanced track (19, 20, 22, 23–26 + esp32cam_hardware.md)
src/documentation/                 # notes/docs (ESP32 guide, driver install)
platformio.ini                     # one [env:lessonNN_*] per lesson, each with build_src_filter
```

## Commands

Each lesson is a separate PlatformIO environment named after its file. Select it with `-e`:

```bash
pio run -e lesson07_toggle -t upload   # build + flash a specific lesson
pio run -e lesson05_melody             # build only
pio device monitor                     # serial monitor (115200 baud)
pio run                                # builds default_envs (set in platformio.ini)
```

`default_envs` in `platformio.ini` controls what bare `pio run` builds — point it at whatever lesson you're currently working on.

## Adding a lesson

1. Create `src/lessons-basic/lessonNN_name.cpp` (or `src/lessons-advance/…` for IoT/robot
   lessons) with its own `#include "pins.h"`, `setup()`, and `loop()`. Add any new pins to
   `include/pins.h` rather than redefining them locally.
2. Add a matching env to `platformio.ini`, pointing `build_src_filter` at that file and
   declaring any `lib_deps` it needs:
   ```ini
   [env:lessonNN_name]
   build_src_filter = +<lessons-basic/lessonNN_name.cpp>
   ```
   The `[env]` base section already supplies platform/board/framework/baud. A lesson that runs
   on a **different board** (the camera lessons) pulls its overrides from a plain section instead:
   `extends = cam_s3`.

There is no test suite or linter despite the empty `test/` dir.

## Advanced lessons — IoT + robot (Этап 2–3)

`src/lessons-advance/` talks to the **.NET backend** in the sibling repo
`../ArduinoEducation` (RabbitMQ + MQTT plugin on port 1883). Keep both sides in sync.

- **lesson19_mqtt_telemetry** — MQTT basics with `knolleary/PubSubClient`: publishes
  `sensors/esp32/<metric>` (body = bare number) and subscribes to `commands/esp32/#`.
- **lesson20_mpu6050** — reads the IMU with `tockn/MPU6050_tockn` and publishes
  `pitch`/`roll` (from the accelerometer via `atan2`) and `yaw` (from `getAngleZ()`, i.e.
  integrated gyro — **drifts** over time; no magnetometer). The real chip is an MPU**6500**
  (WHO_AM_I `0x70`); tockn works because it doesn't check WHO_AM_I (Adafruit's lib would reject it).
  `lesson_check_mpu` is a no-library I2C diagnostic to run first.
- **lesson21_tb6612_drive** — subscribes to `commands/esp32/drive` and drives two motors via a
  **TB6612FNG** (`forward`/`back`/`left`/`right`/`stop`; tank-style turns). Speed via
  `analogWrite` (no LEDC channels here). Has a **failsafe**: stops the motors if no command
  arrives for `FAILSAFE_MS` (700 ms), so the browser resends every ~300 ms while a key/button
  is held. `lesson_check_tb6612` / `lesson_check_tb6612_forward` are no-network diagnostics.
- **lesson22_robot_telemetry** — lessons 20 + 21 in **one** sketch: the same `PubSubClient`
  both publishes `pitch`/`roll`/`yaw` and subscribes to `commands/esp32/drive`, so the IMU and
  the motor driver are wired **at the same time** (their pins must not overlap — see below).
  Beyond merging: `mpu.update()` runs every `loop()` (publish stays at 5 Hz) for a less drifty
  integrated `yaw`; a **tilt guard** stops the motors and ignores drive commands while
  |pitch| or |roll| exceeds `TILT_LIMIT` 45° (released below `TILT_RELEASE` 35° — hysteresis),
  and reports itself as an ordinary metric `sensors/esp32/guard` (0/1) published **only on
  change**. That guard does **not** trust a single sample, and must not be simplified back to
  one: `pitch`/`roll` come from the acceleration vector, so the motors' own starting jolt read
  as 45–48° on a level floor and the guard cut the motors it had just started — the jolt
  silenced itself. Two filters fix it, both needed: a sample counts only while
  |a| ≈ 1 g (`ACCEL_TRUST_BAND`, an untrusted sample is skipped whole — it neither arms nor
  releases nor resets), and arming needs the tilt to hold past the limit for
  `TILT_DEBOUNCE_MS` 300 ms (measured false trips lived 47–228 ms). Releasing has no debounce. Keep `loop()` non-blocking: a `delay` there delays incoming `stop` commands.

**Topic contracts (must match the .NET side):** telemetry `sensors/<device>/<metric>`,
commands `commands/<device>/drive`. `<device>` is `esp32` for the robot and `esp32cam` for the
camera's own telemetry (lesson 26). Before flashing, set `SSID`/`PASS`, `MQTT_BROKER` (the PC's
LAN IP, **not** localhost), and use a non-`guest` RabbitMQ user (`guest` only works from localhost).

## Camera lessons — ESP32-S3-CAM (Этап 6)

Lessons 23–26 + `lesson_check_cam` run on a **HW-679 ESP32-S3-CAM** (ESP32-S3-WROOM-1-N16R8:
16 MB flash, 8 MB **octal** PSRAM, USB-C) — a different board **and a different chip**
from the DevKit. This is the single most important thing to keep in mind when touching them:

- their envs `extends = cam_s3`, a plain section in `platformio.ini` holding
  `board = esp32-s3-devkitc-1`, `board_build.arduino.memory_type = qio_opi`,
  `partitions = default_16MB.csv`, `flash_size = 16MB` and
  `-DBOARD_HAS_PSRAM -DARDUINO_USB_MODE=1 -DARDUINO_USB_CDC_ON_BOOT=1`. `extends` **does**
  override the `board` from `[env]` (verified);
- **the sensor is a GC2145, not an OV2640** (verified on hardware: PID `0x2145`, `esp_camera_sensor_get_info()` reports `support_jpeg = false`). It has **no hardware JPEG**, so
  `esp_camera_init()` with `PIXFORMAT_JPEG` fails with `ESP_ERR_NOT_SUPPORTED` (`0x106`) and
  the driver logs `JPEG format is not supported on this sensor`. Every camera sketch therefore
  runs the sensor in **`PIXFORMAT_RGB565`** and compresses each frame in software with
  **`frame2jpg()`** from `img_converters.h` (also in the core, no `lib_deps`), then **`free()`s**
  that buffer — the raw `fb` goes back via `esp_camera_fb_return()` as usual, so there are now
  **two** buffers to release per frame. Consequences to keep in mind when editing:
  quality is the `frame2jpg` scale (**0..100, higher = better**), not the sensor's 0..63
  lower-is-better; the streaming lessons default to **QVGA** because compression, not Wi-Fi,
  is the bottleneck; and in lesson 25 `quality` is the one control that writes a firmware
  variable instead of a sensor register. `lesson_check_cam` tries JPEG, falls back to RGB565 and
  prints which mode it got — if a board turns out to carry an OV2640, `PIXFORMAT_JPEG` +
  `fb->buf` can go back and will be much faster;
- **because of RGB565, `framesize` is fixed at `esp_camera_init()` and is NOT settable at
  runtime.** `cam_config()` derives `width`/`height` and allocates the frame buffers once from
  `cfg.frame_size`; `sensor_t::set_framesize()` belongs to the *sensor* driver and only writes
  SCCB registers — there is no re-allocating wrapper (`nm` on `libesp32-camera.a` shows
  `cam_config` as the only frame-geometry symbol, and it is called from init alone). In JPEG mode
  the mismatch is survivable, since each JPEG carries its own dimensions; with RGB565 it is not —
  `frame2jpg()` reads the stale `fb->width/height` while the sensor already emits rows of a
  different length, and the frame breaks into **diagonal stripes with a black remainder** (the
  symptom: "everything except the init framesize is broken"). Lessons 24/25 therefore switch
  resolution by **`esp_camera_deinit()` + `initCamera(fs)`** in `applyFramesize()`, serialised
  against the stream task by a `camLock` mutex (the stream holds it only around
  `fb_get`→`frame2jpg`→`fb_return`, not during the network write), and choose `fb_count` by area
  in `fbCountFor()`: 2 up to SVGA, 1 above — UXGA RGB565 is 3.7 MB per buffer, so two of them
  will not fit in 8 MB beside Wi-Fi and `frame2jpg`'s output. Never "init small, `set_framesize`
  big" here, and never init at the max and scale down either — both directions break RGB565;
- **`xclk_freq_hz` must drop to 10 MHz at XGA and above** — `cfg.xclk_freq_hz = (fs >=
  FRAMESIZE_XGA) ? 10000000 : 20000000` in lessons 24/25. At 20 MHz the camera bus outruns the
  DMA path into PSRAM on big frames: `cam_hal` spams **`EV-EOF-OVF`** and `esp_camera_fb_get()`
  returns NULL *forever* — the camera is dead until the next re-init, and the OVF interrupt storm
  starves Wi-Fi too (the board drops off the network). Measured on this board: UXGA at 20 MHz =
  OVF storm, zero frames; at 10 MHz = 0.5 fps and a clean image. SXGA is actually *faster* at
  10 MHz (0.7 vs 0.4 fps), so this is not purely a workaround. Full measured ladder (RGB565 +
  `frame2jpg`, quality 80): QVGA **10.2** fps | VGA **2.9** | SVGA **1.6** | XGA **0.9** |
  SXGA **0.7** | UXGA **0.5**. Every framesize up to UXGA works; the ceiling is CPU, not Wi-Fi;
- **`memory_type` is the fragile bit:** `qio_opi` for octal PSRAM (N16R8 / N8R8), `qio_qspi` for
  quad (N8R2). Wrong value → `psramFound()` is false → QVGA only. `lesson_check_cam` prints the
  PSRAM size in KB precisely so this is a one-run check;
- USB-C on S3 is *usually* the chip's **native** USB — but **this board has a CH340 bridge**
  (`pio device list` → `1A86:7523`), so `[cam_s3]` carries `-DARDUINO_USB_CDC_ON_BOOT=0` and
  `Serial` goes to UART0. With `=1` the monitor stays **completely empty** while flashing still
  works. `pio device list` VID/PID tells which board is which (`303A:1001` = native USB → then
  `=1` is right). The `delay(1500)` at the top of each camera `setup()` is a leftover of the
  native-USB case and harmless here;
- **the serial monitor holds this board in reset** unless DTR/RTS are left alone: CH340 drives
  the auto-reset circuit, so an open monitor means no output *and* a dead Wi-Fi server (the
  symptom is "the page was working, then it wasn't"). `[env]` therefore sets `monitor_dtr = 0`
  and `monitor_rts = 0`; uploading is unaffected. Do not remove them;
- `esp_camera.h` ships **inside** the arduino-esp32 core — no `lib_deps` for it (verified on core
  2.0.17 / platform 7.0.1). Only lesson 26 needs a library (`PubSubClient`);
- the `CAM_*` pins in `pins.h` are **not chosen by us** — the sensor ribbon is hard-wired on the
  module. `pins.h` carries a `CAM_MODEL_*` switch (`ESP32S3_CAM` default, `XIAO_ESP32S3`,
  `AI_THINKER`); numbers colliding with DevKit names (`CAM_Y7 18` vs `LED_YELLOW 18`) are not a
  conflict — different boards, different firmware;
- `CAM_FLASH_LED` / `CAM_STATUS_LED` default to **−1** because S3 boards don't standardise them;
  every LED call in the sketches sits behind `#if CAM_FLASH_LED >= 0` in local `flashInit()` /
  `flashSet()` / `statusLed()` helpers. `lesson_check_cam` has a **"LED hunt"** that pulses the
  safe free GPIOs one by one so the pin can be found by eye. Keep the guards when editing;
- the camera occupies `LEDC_CHANNEL_0` + `LEDC_TIMER_0` for XCLK, so the flash LED uses
  `CH_CAM_FLASH 7` (channel→timer is `ch/2` in core 2.x). **Never** put anything on channel 0/1
  in a camera sketch — and prefer explicit `ledcSetup` over `analogWrite` there, since
  `analogWrite` picks channels on its own;
- hardware, flashing (native USB, no GPIO0 jumper), power and failure modes:
  `src/lessons-advance/documentation/esp32cam_hardware.md`.

- **lesson_check_cam** — no Wi-Fi diagnostic: PSRAM, `esp_camera_init` (JPEG, else RGB565),
  sensor PID + whether it does hardware JPEG, one frame per second to Serial (with the software
  compression time when there is no hardware JPEG), flash-LED blink. Run it first, like
  `lesson_check_mpu`.
- **lesson23_cam_snapshot** — single JPEG frame over HTTP with the familiar sync `WebServer`
  (`/jpg`). Teaches the frame lifecycle (`fb_get` → **`frame2jpg`** → send → `fb_return` +
  `free`) and why binary bodies need `setContentLength()` + `client.write()`.
- **lesson24_cam_stream** — MJPEG (`multipart/x-mixed-replace`) via **`esp_http_server`**, because
  a stream handler never returns and would block the sync `WebServer`'s `loop()` forever. **Two
  servers:** page on `:80`, stream on `:81` — and the second one needs a distinct `ctrl_port`
  (32769) or `httpd_start` fails. Uses `CAMERA_GRAB_LATEST` so the video does not lag.
  `:80/size?v=N` re-inits the camera rather than calling `set_framesize` — see the RGB565
  framesize bullet above.
- **lesson25_cam_controls** — the whole `sensor_t` control surface (`framesize`, `quality`,
  brightness/contrast/saturation, `special_effect`, mirror/flip, awb/aec/agc, bpc/wpc/lenc,
  `colorbar`) behind `/control?var=&val=` + `/status`. The point of the lesson: `set_*` writes
  sensor registers over SCCB, it is not a filter in our firmware. **`quality` is the exception**
  — it feeds `frame2jpg`, so it is the one slider that costs CPU and moves the fps. **On this
  board almost nothing else in `sensor_t` is real:** `gc2145_init()` in the prebuilt
  `libesp32-camera.a` wires up only `set_pixformat`, `set_framesize`, `set_hmirror`, `set_vflip`,
  `set_colorbar`, `set_reg`/`get_reg` — every other setter points at `set_dummy`, a two-instruction
  `return -1` (verified with `objdump -r` on `gc2145.c.obj`: the `.literal.gc2145_init` pool holds
  `set_dummy` once and it is stored into ~20 struct slots). So brightness/contrast/saturation,
  `special_effect`, `dcw`, awb/aec/agc, `ae_level`, `aec_value`, `agc_gain`, `gainceiling`,
  bpc/wpc/`raw_gma`/`lenc` all answer `unsupported` and change nothing — this is the driver, not a
  bug in the sketch, and not something a different `val` will fix. Working controls on GC2145:
  **framesize, quality, hmirror, vflip, colorbar** (plus `flash`, which does nothing while
  `CAM_FLASH_LED` is −1). The panel marks the dead rows struck-through from the `/control`
  response — keep that feedback, it is what stops the page from looking broken. Reviving the rest
  means writing GC2145 registers by hand through `set_reg`, not calling `set_*`. **`framesize` is
  the other exception:** `applyControl` intercepts it *before* touching `sensor_t` and re-inits the
  camera (see the RGB565 framesize bullet above), so it is the one control that is not an SCCB
  register write at all.
- **lesson26_fpv_robot** — FPV: stream + on-page D-pad. The camera **does not drive motors**; it
  republishes button presses into `commands/esp32/drive`, which lesson 22 already consumes, so the
  robot firmware, the backend and the dashboard need **no changes**. Two boards, one broker.
  (On S3 there *are* enough free GPIOs for a one-board robot — the doc's "variant B" maps them —
  but that means re-wiring lesson 22's motors and IMU, so it stays a documented alternative.)
  It also publishes `sensors/esp32cam/fps|rssi` and subscribes to `sensors/esp32/guard`.
  `/drive` runs in the httpd task and `PubSubClient` is **not** thread-safe, so the handler only
  `xQueueSend`s and `loop()` publishes — keep that split if you touch this file.

## Pin / channel conventions

All hardware wiring lives as `#define`s in `include/pins.h` — check there before assuming a pin. Notable points:

- `LED_RED2 02`, `LED_RED 23`, `LED_YELLOW 18`, `LED_GREEN 19` — traffic-light LEDs.
- RGB LED on `PIN_R/G/B` (22/21/15), driven via PWM. `setColor(r,g,b)` writes to LEDC **channels** `CH_R/CH_G/CH_B` (0/1/2), not pins.
- `BUZZER 05` shares LEDC channel `CH 0` with the RGB red channel — both demos aren't meant to run simultaneously.
- `BUTTON 04` uses `INPUT_PULLUP`: not-pressed = HIGH, pressed = LOW. Button lessons debounce manually via `millis()`.
- `MPU_SDA 32` / `MPU_SCL 17` — I2C bus for the MPU6050/6500 (lessons 20, 22).
- **TB6612FNG motors (lessons 21, 22):** `MOTOR_AIN1 13`, `MOTOR_AIN2 14`, `MOTOR_PWMA 25` (motor A = left);
  `MOTOR_BIN1 26`, `MOTOR_BIN2 27`, `MOTOR_PWMB 33` (motor B = right). (`MPU_SCL` and `MOTOR_PWMB`
  were once swapped — 17 for PWM, 33 for SCL; `pins.h` is the truth, not older notes.) The driver's `STBY` is wired
  to `3V3` (always enabled), so no GPIO is reserved for it. `IN1=1,IN2=0`→forward; `IN1=0,IN2=1`→back;
  `IN1=IN2`→stop. If a motor spins the wrong way, swap its two output leads (or the `dir` signs).

- **ESP32-S3-CAM (lessons 23–26) — a different board and chip:** the `CAM_*` block at the bottom
  of `pins.h`, selected by `CAM_MODEL_*`. For the default HW-679: XCLK 15, SIOD/SIOC 4/5,
  D0–D7 on 11/9/8/10/12/18/17/16, VSYNC/HREF/PCLK 6/7/13, PWDN/RESET −1, plus `CH_CAM_FLASH 7`
  and `CAM_FLASH_LED`/`CAM_STATUS_LED` at −1 (see the camera section above). Free there: 1, 2, 3,
  14, 21, 47, 48 and 38/39/40 (microSD). **Never** take 19/20 (native USB), 43/44 (UART0),
  0/45/46 (strapping) or 26–37 (flash + PSRAM).

Because only one lesson compiles at a time (`build_src_filter`), the same GPIO can legitimately
appear under different names across unrelated lessons — but keep new advanced-lesson pins distinct
from the peripherals above to avoid confusion when several are wired at once. **Lesson 22 makes this
a hard requirement, not a nicety:** it drives the motors and reads the IMU in the same sketch, so the
`MPU_*` and `MOTOR_*` groups must stay disjoint.

## LEDC PWM API note (important)

This project uses the **ESP32 Arduino core 2.x channel-based LEDC API**:
`ledcSetup(channel, freq, resolution)` + `ledcAttachPin(pin, channel)`, then write with `ledcWrite(channel, value)` / `ledcWriteTone(channel, freq)`.

Core 3.x renamed these to pin-based calls (`ledcAttach(pin, freq, res)`, `ledcWrite(pin, value)`). Do not mix the two APIs. If a build fails on `ledcSetup`/`ledcAttachPin` being undefined, the installed core is 3.x — keep the 2.x API and pin the platform version rather than rewriting, unless asked otherwise.
