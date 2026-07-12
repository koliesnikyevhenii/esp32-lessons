# ✅ Установка CP210x драйвера (Windows)

### 🔧 Способ 1 — через Device Manager (правильный)

1. Подключи ESP32 к USB
2. Открой:  
    **Device Manager (Диспетчер устройств)**
3. Найди:
    - “Unknown device” или
    - “USB Serial” с жёлтым значком
4. ПКМ → **Update driver**
5. Выбери:  
    👉 **Browse my computer for drivers** (C:\Projects\arduino\CP210x_Universal_Windows_Driver)
6. Укажи папку:
    
    ```
    CP210x_Universal_Windows_Driver
    ```
    
7. Включи:  
    ✔ “Include subfolders”
8. Нажми Next → Install


ВТОРОЙ ДРАЙВЕР: 
# ⚡ Как устанавливать (самый простой способ)

1. Скачай `CH341SER.EXE`
2. Запусти **от имени администратора**
3. Нажми:
    - INSTALL
4. Переподключи ESP32

---

# 💡 Как понять, что всё ок

После установки в Windows появится:

- “USB-SERIAL CH340 (COMx)”  
    или
- “CH341 USB-to-Serial”

---

# 🧠 Важный момент для твоего набора

У тебя ESP32 Starter Kit → там **может быть CP2102 или CH340**

👉 Поэтому ты правильно делаешь, что ставишь **оба драйвера (CP210x + CH341)**