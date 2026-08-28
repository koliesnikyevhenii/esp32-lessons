# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A PlatformIO project of incremental ESP32 learning exercises (Arduino framework). Each lesson is a **self-contained sketch** in `src/lessons-basic/lessonNN_name.cpp` with its own `setup()` and `loop()`. Only one lesson is compiled per build, selected by a PlatformIO env (see below) — this avoids the duplicate `setup()`/`loop()` and global-variable collisions you'd get if they all compiled together. The shared pin/channel map lives in `include/pins.h`.

Default board: `esp32dev` (ESP32-WROOM-32 DevKit) — **except the camera lessons 23–26**, which
are a different physical board and override `board = esp32cam` in their env (see below). Serial
monitor baud: `115200`. Code comments are in Russian; pin/lesson labels and identifiers are English.

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
   on a **different board** (the camera lessons) overrides just that one key: `board = esp32cam`.

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
  change**. Keep `loop()` non-blocking: a `delay` there delays incoming `stop` commands.

**Topic contracts (must match the .NET side):** telemetry `sensors/<device>/<metric>`,
commands `commands/<device>/drive`. `<device>` is `esp32` for the robot and `esp32cam` for the
camera's own telemetry (lesson 26). Before flashing, set `SSID`/`PASS`, `MQTT_BROKER` (the PC's
LAN IP, **not** localhost), and use a non-`guest` RabbitMQ user (`guest` only works from localhost).

## Camera lessons — ESP32-CAM (Этап 6)

Lessons 23–26 + `lesson_check_cam` run on an **AI-Thinker ESP32-CAM**, not the DevKit. This is
the single most important thing to keep in mind when touching them:

- their envs set `board = esp32cam`, which also brings `-DBOARD_HAS_PSRAM
  -mfix-esp32-psram-cache-issue` and `partitions = huge_app.csv` (the camera firmware does not
  fit the default partition table);
- `esp_camera.h` ships **inside** the arduino-esp32 core — no `lib_deps` for it (verified on core
  2.0.17). Only lesson 26 needs a library (`PubSubClient`);
- the `CAM_*` pins in `pins.h` are **not chosen by us** — the OV2640 ribbon is hard-wired on the
  module, which is why some numbers collide with DevKit names (`CAM_Y4 19` vs `LED_GREEN 19`).
  Different boards, different firmware: not a conflict;
- the camera occupies `LEDC_CHANNEL_0` + `LEDC_TIMER_0` for XCLK, so the flash LED uses
  `CH_CAM_FLASH 7` (channel→timer is `ch/2` in core 2.x). **Never** put anything on channel 0/1
  in a camera sketch;
- hardware, flashing (no USB, GPIO0→GND), power and failure modes:
  `src/lessons-advance/documentation/esp32cam_hardware.md`.

- **lesson_check_cam** — no Wi-Fi diagnostic: PSRAM, `esp_camera_init`, sensor PID, one frame per
  second to Serial, flash-LED blink. Run it first, like `lesson_check_mpu`.
- **lesson23_cam_snapshot** — single JPEG frame over HTTP with the familiar sync `WebServer`
  (`/jpg`). Teaches the frame lifecycle (`fb_get` → send → **`fb_return`**) and why binary bodies
  need `setContentLength()` + `client.write()`.
- **lesson24_cam_stream** — MJPEG (`multipart/x-mixed-replace`) via **`esp_http_server`**, because
  a stream handler never returns and would block the sync `WebServer`'s `loop()` forever. **Two
  servers:** page on `:80`, stream on `:81` — and the second one needs a distinct `ctrl_port`
  (32769) or `httpd_start` fails. Uses `CAMERA_GRAB_LATEST` so the video does not lag.
- **lesson25_cam_controls** — the whole `sensor_t` control surface (`framesize`, `quality`,
  brightness/contrast/saturation, `special_effect`, mirror/flip, awb/aec/agc, bpc/wpc/lenc,
  `colorbar`) behind `/control?var=&val=` + `/status`. The point of the lesson: `set_*` writes
  OV2640 registers over SCCB, it is not a filter in our firmware.
- **lesson26_fpv_robot** — FPV: stream + on-page D-pad. The camera **does not drive motors**; it
  republishes button presses into `commands/esp32/drive`, which lesson 22 already consumes, so the
  robot firmware, the backend and the dashboard need **no changes**. Two boards, one broker.
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

- **ESP32-CAM (lessons 23–26) — a different board:** the `CAM_*` block at the bottom of `pins.h`
  (PWDN 32, XCLK 0, SIOD/SIOC 26/27, D0–D7 on 5/18/19/21/36/39/34/35, VSYNC/HREF/PCLK 25/23/22),
  plus `CAM_FLASH_LED 4` (very bright, shares SD DATA1), `CAM_STATUS_LED 33` (**inverted**: LOW =
  on) and `CH_CAM_FLASH 7`. Free GPIOs there: 12/13/14/15 (SD), 2, 4; `GPIO 0` is boot **and**
  XCLK — never repurpose it.

Because only one lesson compiles at a time (`build_src_filter`), the same GPIO can legitimately
appear under different names across unrelated lessons — but keep new advanced-lesson pins distinct
from the peripherals above to avoid confusion when several are wired at once. **Lesson 22 makes this
a hard requirement, not a nicety:** it drives the motors and reads the IMU in the same sketch, so the
`MPU_*` and `MOTOR_*` groups must stay disjoint.

## LEDC PWM API note (important)

This project uses the **ESP32 Arduino core 2.x channel-based LEDC API**:
`ledcSetup(channel, freq, resolution)` + `ledcAttachPin(pin, channel)`, then write with `ledcWrite(channel, value)` / `ledcWriteTone(channel, freq)`.

Core 3.x renamed these to pin-based calls (`ledcAttach(pin, freq, res)`, `ledcWrite(pin, value)`). Do not mix the two APIs. If a build fails on `ledcSetup`/`ledcAttachPin` being undefined, the installed core is 3.x — keep the 2.x API and pin the platform version rather than rewriting, unless asked otherwise.
