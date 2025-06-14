# Fire Detection Using IoT and AI

An intelligent fire detection system that combines ESP32 hardware with physical sensors (temperature, smoke, flame) and AI (YOLOv8) for real-time fire detection from camera images with instant alerts.

## System Architecture

```
ESP32 (Sensor Hub)
 ├── DHT11 (Temperature & Humidity)
 ├── MQ-2 Gas Sensor (CO detection)
 ├── Flame Sensor (IR flame detection) 
 ├── Buzzer + LED Alert System
 └── MQTT Communication ↔ PC Server
                          ↕
PC Server (AI Processing)
 ├── ESP32-CAM Video Stream
 ├── YOLOv8 Model (Fire/Smoke Detection)
 └── MQTT Alert Signal to ESP32
```

## Key Features

- **Multi-sensor Detection**: Temperature, humidity, gas, and flame sensors
- **AI-Powered Vision**: YOLOv8 model for fire and smoke recognition
- **Real-time Monitoring**: Blynk app integration for remote monitoring
- **Instant Alerts**: Buzzer and LED warnings with auto-reset
- **MQTT Communication**: Reliable IoT messaging between devices

## Components

### 1. ESP32 Firmware (Arduino/PlatformIO)
**Sensor Data Collection:**
- Temperature and humidity (DHT11)
- CO levels in ppm (MQ-2)
- Flame detection (IR flame sensor)

**Communication:**
- Receives AI detection signals via MQTT (`esp32/fire_detection` channel)
- Sends sensor data to Blynk app for remote monitoring

**Alert System:**
- Buzzer activation
- LED warning lights

### 2. AI Server (Python + YOLOv8)
- Processes video stream from ESP32-CAM
- Uses YOLOv8 model (`best_320.pt`) for fire/smoke detection
- Sends MQTT alerts to ESP32:
  - `"1"` when fire detected
  - `"0"` when safe

## Blynk Dashboard

| Virtual Pin | Data Display |
|-------------|-------------|
| V0 | Temperature (°C) |
| V1 | Humidity (%) |
| V3 | Flame Detection Status |
| V4 | Alert Status (1 = Alert, 0 = Safe) |
| V5 | CO Concentration (ppm) |

## Installation & Setup

### ESP32 Setup
1. **Upload Firmware**
   ```bash
   # Upload Fire_Detection.ino to ESP32
   ```

2. **Configuration**
   - WiFi SSID/Password
   - MQTT broker IP (PC server IP)
   - Blynk authentication token

3. **Hardware Connections**
   - Connect sensors according to pin diagram in code

### Python AI Server Setup
1. **Install Dependencies**
   ```bash
   pip install ultralytics opencv-python paho-mqtt
   ```

2. **Run Server**
   ```bash
   python fire_detection_server.py
   ```

3. **Verify Camera Stream**
   - Ensure ESP32-CAM is accessible from PC
   - Test stream URL: `http://<esp32-cam-ip>/stream`

## YOLOv8 Model Details

- **Model File**: `best_320.pt`
- **Classes**: Fire and Smoke detection
- **Framework**: Ultralytics YOLOv8
- **Input Resolution**: 320x320 pixels
- **Optimized for**: Real-time detection on standard hardware

## Hardware Requirements

### ESP32 Setup
- ESP32 Development Board
- DHT11 Temperature/Humidity Sensor
- MQ-2 Gas Sensor
- IR Flame Sensor
- Buzzer
- LED indicators
- ESP32-CAM module

⭐ **Star this repository if you found it helpful!** ⭐
