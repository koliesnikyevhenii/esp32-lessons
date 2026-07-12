# Урок 19. MQTT-телеметрия (Этап 2. IoT Foundation)

Первый урок продвинутого трека. Цель этапа — **связать ESP32 и .NET** через MQTT.

```
ESP32 --MQTT(1883)--> RabbitMQ --AMQP--> .NET Worker --> PostgreSQL
```

Бэкенд лежит в `C:\Projects\arduino\ArduinoEducation` (проект TelemetryApi).

## Что изучаем (4 понятия MQTT)

| Понятие      | Где в коде                                             |
|--------------|--------------------------------------------------------|
| **publish**  | `mqtt.publish(topic, body)` — шлём temperature и uptime |
| **subscribe**| `mqtt.subscribe(TOPIC_COMMANDS, 1)` + колбэк `onMessage` |
| **topics**   | иерархия `sensors/<device>/<metric>`, wildcard `#`      |
| **QoS**      | подписка на QoS 1, публикация на QoS 0 (см. ниже)       |

## Топики и трансляция в RabbitMQ

MQTT-плагин RabbitMQ публикует сообщения в обменник `amq.topic`, **заменяя слэши на точки**:

```
MQTT topic:   sensors/esp32/temperature
routing key:  sensors.esp32.temperature
```

`.NET`-консьюмер (`TelemetryConsumer.ParseReading`) разбирает ключ как
`sensors.<device>.<metric>`, а **тело сообщения — голое число** (`"25.0"`).
Поэтому в уроке мы шлём по одному числу на топик, а не JSON.

> Пример из описания этапа — `{"temperature": 25, "uptime": 3600}` — это
> следующий шаг: перейти на JSON. Тогда меняется только `ParseReading`
> на бэкенде (в README телеметрии это указано как точка расширения).

Wildcard `#` в `commands/esp32/#` подписывает на **все** подтопики:
`commands/esp32/led`, `commands/esp32/reboot` и т.д. (`+` — ровно один уровень).

## QoS — важное ограничение библиотеки

`PubSubClient` (knolleary):

- **publish** — только **QoS 0** ("fire and forget", без подтверждения).
- **subscribe** — QoS 0 или **QoS 1** ("at least once", возможны дубли).

Если нужен QoS 1/2 на публикации — берут другую библиотеку (например
`256dpi/arduino-mqtt` или `AsyncMqttClient`). Для телеметрии, где потеря
одного замера некритична, QoS 0 обычно достаточно.

## Настройка перед прошивкой

В `lesson19_mqtt_telemetry.cpp`:

1. `MQTT_BROKER` — **LAN-IP компьютера** с docker (НЕ `localhost`: ESP32 —
   отдельное устройство). Узнать: `ipconfig` -> IPv4 адрес.
2. `MQTT_USER` / `MQTT_PASS` — учётка RabbitMQ. `guest` работает только с
   localhost, поэтому создай пользователя `esp` / `esp-pass`
   (Management UI -> Admin -> Add user, права на vhost `/`).
3. `SSID` / `PASS` — Wi-Fi (ESP32 и ПК в одной сети).
4. DHT11 на `GPIO33` (как в уроке 18). Uptime считается из `millis()`.

## Запуск

```bash
# 1) бэкенд (в папке ArduinoEducation)
docker compose up -d          # RabbitMQ + PostgreSQL
dotnet run                    # API + консьюмер

# 2) прошивка урока
pio run -e lesson19_mqtt -t upload
pio device monitor            # 115200 — видно [MQTT] -> ...
```

## Проверка end-to-end

- Management UI RabbitMQ: http://localhost:15672 (guest/guest) — вкладка
  Queues -> `telemetry.ingest` показывает поток сообщений.
- API: `GET http://localhost:<port>/api/telemetry/latest?device=esp32`
  вернёт последние записи `temperature` и `uptime`.

## Проверка subscribe (команды)

Опубликуй команду любым MQTT-клиентом (или через Management UI ->
Exchanges -> `amq.topic` -> Publish, routing key `commands.esp32.led`):

```
topic:  commands/esp32/led
body:   on      # зажигает встроенный светодиод (LED_RED2, GPIO2)
body:   off     # гасит
```

В мониторе появится `[MQTT] <- топик 'commands/esp32/led', тело 'on'`.

## Дальше

- Перейти на JSON-полезную нагрузку (изменить `ParseReading` на бэкенде).
- Добавить `qos`/`retained` для «последнего известного» значения.
- Батчить запись в PostgreSQL при большом потоке (см. README телеметрии).
