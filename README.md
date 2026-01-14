# ESP32 Fuel-Cell Breathalyzer

A compact **ESP32-S3–based breathalyzer prototype** combining **analog front-end design**, **embedded firmware**, and **wireless data visualization**.

The project explores how professional fuel-cell sensing can be integrated into a modern, battery-powered embedded system with real-time feedback and logging.

---

## Overview

- **Fuel-cell ethanol sensor** with custom analog signal conditioning  
- **Blow detection** using an electret microphone  
- **ESP32-S3** handling signal processing, UI, WiFi, and storage  
- **OLED display** for local readout  
- **WiFi Access Point** with browser-based dashboard  
- **Measurement logging** with timestamps and multi-user support  
- **Battery-powered** with deep sleep and wake-on-button  

---

## How It Works

1. A valid blow is detected via the microphone  
2. The fuel-cell output is sampled and filtered  
3. The system waits for a stable peak response  
4. Sensor voltage is converted into estimated **BAC / promille**  
5. Results are shown on the OLED and streamed live to the dashboard  

Temperature compensation is applied to improve repeatability.

---

## Hardware

- ESP32-S3 microcontroller  
- Fuel-cell alcohol sensor (ethanol)  
- Op-amp based analog front-end  
- Electret microphone + amplifier  
- 128×64 OLED display (I²C)  
- RGB button, buzzer, vibration motor  
- Li-ion battery power  
- Two custom PCBs (analog + digital)  
- 3D-printed enclosure  

---

## Key Learnings

- Analog signal conditioning for low-level sensor outputs  
- Sensor stabilization, filtering, and peak detection  
- Power-efficient embedded design (deep sleep, battery monitoring)  
- Web-based visualization for embedded systems  
- Practical PCB bring-up, debugging, and mechanical integration  

---

## Status

This project is a **functional prototype** intended for technical exploration and learning.  
It is **not a certified measuring instrument**.

---

## Team

- Edvin Kajén  
- Simon Nilsson  
- Sebastian Atterhall  
- Linus Enstedt  
