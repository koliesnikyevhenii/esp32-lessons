# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A PlatformIO project of incremental ESP32 learning exercises (Arduino framework). Each lesson is a **self-contained sketch** in `src/lessons-basic/lessonNN_name.cpp` with its own `setup()` and `loop()`. Only one lesson is compiled per build, selected by a PlatformIO env (see below) — this avoids the duplicate `setup()`/`loop()` and global-variable collisions you'd get if they all compiled together. The shared pin/channel map lives in `include/pins.h`.

Target board: `esp32dev`. Serial monitor baud: `115200`. Code comments are in Russian; pin/lesson labels and identifiers are English.

## Layout

```
include/pins.h                     # shared pin + LEDC channel #defines (the single source of truth for wiring)
src/lessons-basic/lessonNN_*.cpp   # standalone basic sketches (lessons 02–18)
src/lessons-advance/lessonNN_*.cpp # advanced IoT/robot sketches (lessons 19–22, + lesson_check_* diagnostics)
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
   The `[env]` base section already supplies platform/board/framework/baud.

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
commands `commands/<device>/drive`. `<device>` is `esp32`. Before flashing, set `SSID`/`PASS`,
`MQTT_BROKER` (the PC's LAN IP, **not** localhost), and use a non-`guest` RabbitMQ user
(`guest` only works from localhost).

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

Because only one lesson compiles at a time (`build_src_filter`), the same GPIO can legitimately
appear under different names across unrelated lessons — but keep new advanced-lesson pins distinct
from the peripherals above to avoid confusion when several are wired at once. **Lesson 22 makes this
a hard requirement, not a nicety:** it drives the motors and reads the IMU in the same sketch, so the
`MPU_*` and `MOTOR_*` groups must stay disjoint.

## LEDC PWM API note (important)

This project uses the **ESP32 Arduino core 2.x channel-based LEDC API**:
`ledcSetup(channel, freq, resolution)` + `ledcAttachPin(pin, channel)`, then write with `ledcWrite(channel, value)` / `ledcWriteTone(channel, freq)`.

Core 3.x renamed these to pin-based calls (`ledcAttach(pin, freq, res)`, `ledcWrite(pin, value)`). Do not mix the two APIs. If a build fails on `ledcSetup`/`ledcAttachPin` being undefined, the installed core is 3.x — keep the 2.x API and pin the platform version rather than rewriting, unless asked otherwise.
