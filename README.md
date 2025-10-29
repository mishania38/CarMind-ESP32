# 🚗 CarMind ESP32 — Remote Vehicle Control via MQTT

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![Platform: ESP32](https://img.shields.io/badge/Platform-ESP32-green.svg)](https://www.espressif.com/en/products/socs/esp32)
[![MQTT Enabled](https://img.shields.io/badge/MQTT-Secure%20TLS-orange.svg)](https://mqtt.org/)
[![Build: Manual](https://img.shields.io/badge/Build-Manual-lightgrey.svg)]()

**CarMind ESP32** is a firmware for the ESP32 microcontroller that enables secure remote control of vehicle systems using MQTT. It supports engine start/stop, door locking/unlocking, engine heating, and real-time telemetry. Designed for reliability, modularity, and secure communication, it integrates FreeRTOS multitasking and watchdog protection.

---

## 🔧 Features

- 🔑 Remote engine start and stop
- 🔒 Door lock and unlock control
- 🔥 Engine heater activation
- 📡 Wi-Fi failover with multi-network support
- 🔐 TLS-secured MQTT communication via `WiFiClientSecure`
- ⏱️ Remote start timer with countdown and auto shutdown
- 📊 Telemetry: battery voltage, cabin temperature, RPM
- 🧵 FreeRTOS-based multitasking with watchdog integration
- 📁 Certificate and resource management via LittleFS

---

## 📡 MQTT Topics

| Topic                  | Description                          |
|------------------------|--------------------------------------|
| `fiat/startenginecom`  | Start or stop engine command         |
| `fiat/alarmoncom`      | Lock or unlock vehicle               |
| `fiat/heatenginecom`   | Toggle engine heater                 |
| `fiat/startperiodcom`  | Set engine run duration (minutes)    |
| `fiat/starttimer`      | Countdown timer for remote start     |
| `fiat/batteryvolt`     | Battery voltage reading              |
| `fiat/cartemp`         | Cabin temperature reading            |
| `fiat/rpminfo`         | Engine RPM value                     |
| `fiat/totalerrorcount` | Network error counter                |

---

## ⚙️ Hardware Requirements

- ESP32 DevKit v1
- ULN2003 relay module
- DS18B20 temperature sensor
- Start/Stop button (digital input)
- Tachometer and battery voltage analog inputs

---

## 📂 Project Structure
CarMind
- CarMind_ESP32.ino # Core logic and task scheduling
- mqtt_manager.cpp # MQTT client setup and communication
- config.h # Broker credentials and constants
- certs_ar.h # Embedded TLS certificates (binary format)
- wifi_manager.cpp # Wi-Fi connection and failover


---

## 🚀 Getting Started

1. Flash the firmware to your ESP32 using Arduino IDE or PlatformIO
2. Configure your MQTT broker credentials in `config.h`
3. Connect the ESP32 to your vehicle’s control circuits
4. Monitor and control the system via MQTT dashboard or client

---

## 📜 License

This project is licensed under the MIT License — feel free to use, modify, and distribute.
