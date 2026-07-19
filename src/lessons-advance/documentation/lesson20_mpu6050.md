# Урок 20. MPU6050 → угол наклона (Этап 2. IoT Foundation)

Продолжение урока 19. MQTT-часть та же, меняется источник данных: вместо DHT11 —
акселерометр/гироскоп **MPU6050** по шине **I2C**. Считаем **угол наклона**
(pitch/roll) и шлём его на бэкенд «почти в реальном времени».

```
ESP32 --MQTT(1883)--> RabbitMQ --AMQP--> .NET Worker --> PostgreSQL
                                              |
                                        SignalR -> React-дашборд (pitch/roll)
```

Бэкенд лежит в `C:\Projects\arduino\ArduinoEducation` (проект TelemetryApi).

## Что изучаем

| Понятие          | Где в коде                                                        |
|------------------|-------------------------------------------------------------------|
| **I2C**          | `Wire.begin(MPU_SDA, MPU_SCL)` + `mpu.begin()` (адрес `0x68`)      |
| **акселерометр** | `mpu.getEvent(&a, …)` → `a.acceleration.x/y/z` (m/s²)              |
| **угол наклона** | `atan2` от вектора силы тяжести → `pitch` / `roll`                 |
| **publish**      | `sensors/esp32/pitch`, `sensors/esp32/roll` — по одному числу      |

## Что такое pitch и roll

MPU6050 в покое чувствует только силу тяжести. По тому, как вектор `g`
распределён между осями X/Y/Z, восстанавливаем наклон платы (в градусах):

```
roll  = atan2(ay, az)                    · 180/π   // крен «вбок»
pitch = atan2(-ax, √(ay² + az²))         · 180/π   // тангаж «вперёд-назад»
```

`atan2` сам разбирается со знаками и квадрантами. Такой расчёт **только по
акселерометру** отлично работает для статического наклона; при быстрых движениях
его дополняют гироскопом (комплементарный фильтр / Madgwick) — это следующий шаг.

## Почему pitch/roll — просто новые «метрики»

MQTT-плагин RabbitMQ заменяет слэши на точки:

```
MQTT topic:   sensors/esp32/pitch
routing key:  sensors.esp32.pitch
```

`.NET`-консьюмер (`TelemetryConsumer.ParseReading`) читает ключ как
`sensors.<device>.<metric>`, тело — **голое число** (`"-3.4"`). Поэтому на бэкенде
**ничего менять не нужно**: `pitch` и `roll` проходят по тому же конвейеру, что и
`temperature` в уроке 19.

## Подключение (I2C)

| MPU6050 | ESP32           |
|---------|-----------------|
| VCC     | 3V3             |
| GND     | GND             |
| SDA     | GPIO21 (`MPU_SDA`) |
| SCL     | GPIO22 (`MPU_SCL`) |

> Модуль GY-521 работает от 3.3 В (на плате есть стабилизатор, но питай от 3V3).
> Адрес по умолчанию `0x68`; если пин AD0 подтянут к VCC — станет `0x69`.

## Настройка перед прошивкой

В `lesson20_mpu6050.cpp` (как в уроке 19):

1. `MQTT_BROKER` — **LAN-IP компьютера** с docker (`ipconfig` → IPv4), не `localhost`.
2. `MQTT_USER` / `MQTT_PASS` — пользователь RabbitMQ (`esp` / `esp-pass`).
3. `SSID` / `PASS` — Wi-Fi (ESP32 и ПК в одной сети).
4. `PUBLISH_INTERVAL` = 200 мс (5 Гц) — чаще, чем в уроке 19, для «реального времени».

## Запуск

```bash
# 1) бэкенд (в папке ArduinoEducation)
docker compose up -d          # RabbitMQ + PostgreSQL
dotnet run                    # API + консьюмер + SignalR

# 2) прошивка урока
pio run -e lesson20_mpu6050 -t upload
pio device monitor            # 115200 — видно [MQTT] -> sensors/esp32/pitch = ...
```

## Проверка end-to-end

- Наклоняй плату — в мониторе меняются `pitch`/`roll`.
- Management UI RabbitMQ: http://localhost:15672 — очередь `telemetry.ingest`.
- API: `GET http://localhost:<port>/api/telemetry/latest?device=esp32` — последние
  записи `pitch`/`roll`.
- React-дашборд (`dashboard/`, порт 5173) рисует pitch/roll живой линией.

## Дальше

- Добавить гироскоп + комплементарный фильтр (устойчивее при движении).
- Слать также yaw (нужен магнитометр, напр. MPU9250).
- Перейти на JSON-полезную нагрузку (меняется только `ParseReading` на бэкенде).
