# DIYRobotLawnMower Headquarter

An ESP32-based **perimeter wire sender** for a DIY robotic lawn mower — with a built-in web dashboard, WhatsApp notifications, MQTT integration, an OLED status display, and wireless (OTA) firmware updates.

The sender continuously transmits a signal code through a wire buried around the lawn. The mower's onboard sensor picks up this signal to detect the boundary and stay inside it — the same principle used by classic robotic mowers (e.g. Ardumower-compatible signal codes).

> 🔰 **New to ESP32 / Arduino?** The `.ino` sketch is heavily commented for beginners — every function and non-obvious line explains *what* it does and *why*. It's a great way to learn how interrupts, FreeRTOS tasks, a tiny web server, and MQTT work together on a microcontroller.

---

## ✨ Features

- **Perimeter signal generation** via a hardware timer interrupt (ISR) for precise, jitter-free timing, with two independent sender loops (A and B)
- **Web dashboard** (`/`) — live voltage/current readings, uptime, one-click toggles for both sender loops and auto-mode
- **Settings page** (`/settings`) — change the perimeter threshold, edit signal codes, and (if enabled) configure MQTT and WhatsApp credentials, all from the browser, saved permanently to flash
- **WhatsApp notifications** (via [Whatabot](https://api.whatabot.net)) when the mower leaves, returns, or times out — sent from a background task so it never blocks the rest of the sketch
- **MQTT** publish/subscribe support for home automation systems (Home Assistant, Node-RED, ...)
- **OLED status display** (SSD1306/SH1106) showing live sensor values and sender status
- **OTA (Over-The-Air) firmware updates** — after the initial USB upload, you can update wirelessly
- **Automatic mode**: starts/stops the perimeter signal based on the measured charging current, with a configurable safety timeout
- **Wire-cut detection**: alerts if the perimeter current drops unexpectedly while a sender is active
- No hardcoded WiFi credentials — first-time setup goes through a [WiFiManager](https://github.com/tzapu/WiFiManager) captive portal

---

## 🌍 Available languages

This repository ships **three fully translated, functionally identical** versions of the firmware. Pick the folder for your preferred language — the underlying code and behavior are exactly the same, only comments and on-screen/log text differ:

| Folder | Language |
|--------|----------|
| [`/EN`](./EN) | English |
| [`/DE`](./DE) | German (Deutsch) |
| [`/FR`](./FR) | French (Français) |

Each folder contains everything needed for that language version:
- `ESP32_WIFI_Sender.ino` — the firmware
- `index.html`, `settings.html` — the web dashboard and settings page (go in the `data` folder for LittleFS upload, see below)
- `config.json` — an example/starting configuration file

---

## 🛠 Hardware

| Component | Notes |
|---|---|
| ESP32 dev board | Any standard ESP32 (WROOM-32 etc.) |
| L298N (or compatible) H-bridge driver ×2 | One per sender loop (A/B) |
| INA226 current/voltage sensor ×2 | One for the perimeter loop, one for the charging current |
| OLED display (SSD1306 or SH1106, I2C, 128×64) | Optional but supported out of the box |
| 2-color status LED (common cathode) | Green = ready/charged, Red = charging |

### Pin assignments (default)

| Signal | ESP32 GPIO | Connects to |
|---|---|---|
| I2C SDA | 21 | INA226 ×2 + OLED |
| I2C SCL | 22 | INA226 ×2 + OLED |
| Sender A – IN1 | 12 | L298N #1 – IN1 |
| Sender A – IN2 | 13 | L298N #1 – IN2 |
| Sender A – Enable | 23 | L298N #1 – ENA |
| Sender B – IN3 | 14 | L298N #2 – IN3 |
| Sender B – IN4 | 18 | L298N #2 – IN4 |
| Sender B – Enable | 19 | L298N #2 – ENA |
| Green LED | 25 | Status LED (ready/charged) |
| Red LED | 26 | Status LED (charging) |

All pins are `#define`d near the top of the sketch, so they're easy to change if your wiring differs.

---

## ⚙️ Getting started

### 1. Arduino IDE setup

1. Install the **ESP32 board package** in the Arduino IDE (Boards Manager).
2. Install the required libraries via the Library Manager:
   - `ArduinoJson` (by Benoit Blanchon)
   - `PubSubClient` (by Nick O'Leary)
   - `WiFiManager` (by tzapu)
   - `UrlEncode` (by Masayuki Sugahara)
   - `INA226_WE` (by Wolfgang Ewald)
   - `U8g2` (by oliver / olikraus) — provides `U8x8lib.h`
   - `ArduinoOTA`, `LittleFS`, `FS`, `HTTPClient`, `WiFiClient` are part of the ESP32 core and don't need separate installation.
3. Install the **[arduino-littlefs-upload](https://github.com/earlephilhower/arduino-littlefs-upload)** plugin — this lets you upload `index.html`, `settings.html` and `config.json` to the ESP32's flash file system (LittleFS) directly from the IDE via `Ctrl+Shift+P`.
4. Put `index.html`, `settings.html` and `config.json` (from the language folder you chose) into a subfolder named **`data`**, right next to your `.ino` file.

### 2. Board settings — **important!**

In **Tools**, set:

- **Board:** an ESP32 board matching your hardware (e.g. "ESP32 Dev Module")
- **Partition Scheme:** `Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS)`

This partition scheme is **required** and is *not* the default. It reserves space for two firmware copies (needed for OTA updates) **and** a file area for LittleFS (where the web pages and settings live). With the default scheme, either LittleFS won't fit or OTA won't work correctly.

### 3. The first upload must be done via USB

This only applies to the very first flash — after that, firmware updates can go wirelessly via OTA.

1. Connect the ESP32 via USB.
2. Upload the sketch normally (**Sketch → Upload**).
3. Immediately after, press **Ctrl+Shift+P** and upload the `data` folder via LittleFS.

Two reasons the very first upload can't be wireless:
- OTA only works once *this sketch itself* — which contains the OTA code — is already running on the ESP32.
- Clicking "Upload" only flashes the `.ino` file; the web pages need the separate LittleFS upload step, which currently only works over USB.

### 4. First boot — WiFi setup

On first boot (or after a factory reset), the ESP32 starts an access point called **`MowerSender_AP`** (password: `12345678`). Connect to it with your phone/laptop; a captive portal will let you select your home WiFi network and enter its password. After that, the ESP32 remembers the credentials and connects automatically on every boot (via DHCP — no static IP needed).

---

## 🌐 Using the web interface

Once connected to your WiFi, find the ESP32's IP address (check your router's device list, or the Serial Monitor at 115200 baud during boot) and open it in a browser.

- **`/`** — Dashboard: live voltage/current readings, uptime, current signal code, sender status, and quick toggles.
- **`/settings`** — Change the `PeriOnOffThreshold`, edit any of the 5 signal codes, and (if the corresponding feature is enabled in the sketch) configure MQTT and WhatsApp. Settings are saved to `config.json` in flash and survive a reboot.
- **`/reset`** — Factory reset: wipes WiFi credentials and saved settings, then restarts.
- **`/download-settings`** — Downloads the current `config.json`.

---

## 📡 MQTT

Enable with `#define MQTT 1` near the top of the sketch. Once enabled, the broker address, port, client ID, username/password, and all topics become editable on the `/settings` page (no need to hardcode them).

**Default topics:**

| Topic | Direction | Payload |
|---|---|---|
| `teensysender/input` | subscribe (incoming commands) | see command list below |
| `teensysender/chargecurrent` | publish | charging current (mA) |
| `teensysender/pericurrent` | publish | perimeter wire current (mA) |
| `teensysender/chargevoltage` | publish | charging voltage (V) |

**Supported incoming commands** (published to the subscribe topic):

`AutoMode0`, `AutoMode1`, `A0`, `A1`, `B0`, `B1`, `sigDuration50`, `sigDuration104`, `sigCode0` … `sigCode4`

---

## 💬 WhatsApp notifications

Enable with `#define WhatsApp_messages 1` near the top of the sketch. Notifications are sent via the free [Whatabot](https://api.whatabot.net) WhatsApp API.

1. Add the Whatabot contact on your phone: `+54 9 2364205798`
2. Send it the message: `I allow whatabot to send me messages`
3. Whatabot will reply with your phone number (in the required format) and an API key.
4. Enter both on the `/settings` page in the browser — they're saved to `config.json`, not hardcoded in the sketch.

You'll get a message when: the ESP32 comes online, the mower leaves the station, the mower returns, or the safety timeout triggers.

---

## 🔒 A note on credentials

WiFi, MQTT, and WhatsApp credentials are **not** stored in the source code. WiFi is configured via the WiFiManager captive portal; MQTT and WhatsApp credentials are entered through the `/settings` web page and saved to `config.json` on the device's flash storage. This means the `.ino` file itself is safe to publish/share without leaking any personal data — just don't share your device's `config.json`.

---

## 📁 Repository structure

```
.
├── EN/
│   ├── ESP32_WIFI_Sender.ino
│   ├── index.html
│   ├── settings.html
│   └── config.json
├── DE/
│   └── ... (same files, German)
├── FR/
│   └── ... (same files, French)
└── README.md
```

To use a language version: copy the four files from that folder into your sketch folder, then move `index.html`, `settings.html`, and `config.json` into a `data` subfolder before doing the LittleFS upload (see [Getting started](#️-getting-started) above).

---

## 🙏 Credits

- Perimeter signal codes based on the [Ardumower](http://grauonline.de/alexwww/ardumower/filter/filter.html) project.
- Built with: [ArduinoJson](https://arduinojson.org/), [PubSubClient](https://github.com/knolleary/pubsubclient), [WiFiManager](https://github.com/tzapu/WiFiManager), [UrlEncode](https://github.com/plageoj/urlencode), [INA226_WE](https://github.com/wollewald/INA226_WE), [U8g2](https://github.com/olikraus/u8g2), and the ESP32 Arduino core.

---

## ⚠️ Disclaimer

This is a DIY / hobby project. Perimeter wire signals, motor drivers, and battery charging circuits involve real electrical current — build and wire everything carefully, and use this project at your own risk.
