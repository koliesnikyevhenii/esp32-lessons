# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A PlatformIO project of incremental ESP32 learning exercises (Arduino framework). Each lesson is a **self-contained sketch** in `src/lessons/lessonNN_name.cpp` with its own `setup()` and `loop()`. Only one lesson is compiled per build, selected by a PlatformIO env (see below) — this avoids the duplicate `setup()`/`loop()` and global-variable collisions you'd get if they all compiled together. The shared pin/channel map lives in `include/pins.h`.

Target board: `esp32dev`. Serial monitor baud: `115200`. Code comments are in Russian; pin/lesson labels and identifiers are English.

## Layout

```
include/pins.h            # shared pin + LEDC channel #defines (the single source of truth for wiring)
src/lessons/lessonNN_*.cpp # one standalone sketch per lesson
platformio.ini            # one [env:lessonNN_*] per lesson, each with build_src_filter
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

1. Create `src/lessons/lessonNN_name.cpp` with its own `#include "pins.h"`, `setup()`, and `loop()`. Add any new pins to `include/pins.h` rather than redefining them locally.
2. Add a matching env to `platformio.ini`:
   ```ini
   [env:lessonNN_name]
   build_src_filter = +<lessons/lessonNN_name.cpp>
   ```
   The `[env]` base section already supplies platform/board/framework/baud.

There is no test suite or linter despite the empty `test/` dir.

## Pin / channel conventions

All hardware wiring lives as `#define`s in `include/pins.h` — check there before assuming a pin. Notable points:

- `LED_RED2 02`, `LED_RED 23`, `LED_YELLOW 18`, `LED_GREEN 19` — traffic-light LEDs.
- RGB LED on `PIN_R/G/B` (22/21/15), driven via PWM. `setColor(r,g,b)` writes to LEDC **channels** `CH_R/CH_G/CH_B` (0/1/2), not pins.
- `BUZZER 05` shares LEDC channel `CH 0` with the RGB red channel — both demos aren't meant to run simultaneously.
- `BUTTON 04` uses `INPUT_PULLUP`: not-pressed = HIGH, pressed = LOW. Button lessons debounce manually via `millis()`.

## LEDC PWM API note (important)

This project uses the **ESP32 Arduino core 2.x channel-based LEDC API**:
`ledcSetup(channel, freq, resolution)` + `ledcAttachPin(pin, channel)`, then write with `ledcWrite(channel, value)` / `ledcWriteTone(channel, freq)`.

Core 3.x renamed these to pin-based calls (`ledcAttach(pin, freq, res)`, `ledcWrite(pin, value)`). Do not mix the two APIs. If a build fails on `ledcSetup`/`ledcAttachPin` being undefined, the installed core is 3.x — keep the 2.x API and pin the platform version rather than rewriting, unless asked otherwise.
