# (WIP) ESP32 Breathalyzer
An alcohol breath measurement system built around the ESP32-S3.

This project began as a group assignment in a electronic project course.  
The objective was to design a system combining both analog and digital electronics.

We chose to build a breathalyzer, since it integrates these domains well:
- A fuel-cell ethanol sensor requiring analog amplification and conditioning  
- A microphone airflow detector 
- An ESP32-S3 microcontroller handling UI, measurements, WiFi, digital signals, and data storage  

The final system is a battery-powered, WiFi-enabled breathalyzer with OLED display, Web dashboard, and data logging.

---

## Features
- SPEC 110-202 fuel-cell ethanol sensor enabling measurements up to ~4 promille 
- Analog amplifier with filtering   
- Airflow detection using electret microphone and amplifier  
- Adafruit ESP32-S3 Feather  
  - WebSocket real-time communication  
  - Deep sleep and wake-on-button logic  
  - battery monitoring  
  - Local file storage via LittleFS  
- 1,3" 128×64 OLED display  
- Vibration motor and PWM-controlled buzzer for feedback  
- 3D-printed enclosure
- Abillity to save users withing the dashboard
- Browser-based dashboard with real-time graphing  
- Timestamp synchronization from the client device  

---

## System Overview
The device continuously monitors airflow via a microphone.  
When blowing is detected the following happens:

1. A 5 second timer starts
2. The buzzer activates to indicate blowing  
3. After 5 seconds has passed the measurement begins
4. the user is instructed to stop blowing  
5. Sensor values are sampled and filtered  
6. The sensor values are monitored to ensure the sensored has stabilized
7. Ethanol concentration is measured  
8. The device vibrates three times to indicate that the procedure is done  
10. The result is displayed on OLED and streamed live to the dashboard
11. A cooldown procedure is inisialized to senure the sensor returns to baseline between mesurements  

---

