# ESP32-S3 VL53L0X ToF Sensor Implementation
### High-Speed Distance Sensing for Adafruit Feather V2 (ESP32-S3)

This repository contains a high-performance implementation of the VL53L0X Time-of-Flight (ToF) sensor using the **ESP-IDF v6.1** framework. This project was developed as part of a robotics integration task, focusing on high-speed ranging and custom hardware power management.

## 🚀 Key Engineering Challenges Solved

### 1. Feather V2 Power Management (I2C Gates)
The Adafruit Feather V2 (ESP32-S3) uses specific GPIO "gates" to manage power for the I2C/Stemma-QT rail to conserve battery. Communication will fail (Timeout) unless these are explicitly enabled:
* **GPIO 7 (I2C_PWR):** Supplies 3.3V to the I2C rail.
* **GPIO 8 (I2C_PULL):** Enables the onboard I2C pull-up resistors.

### 2. Bypassing Universal Driver "Model ID" Errors
Common "universal" libraries (like `espp`) often perform a hard-coded Model ID check. The VL53L0X (ID: `0x0F`) is frequently rejected by drivers expecting the VL53L1X (ID: `0xEA/EB`). This project utilizes a manual component link to a specialized VL53L0X driver to bypass these checks and perform the necessary **RefSpad** and **RefCalibration** sequences.

### 3. High-Speed Tuning
The implementation is tuned for robotics/drone applications:
* **Update Rate:** ~40Hz (25ms polling).
* **Timing Budget:** 20ms (High-Speed Mode).
* **I2C Clock:** 400kHz (Fast Mode).

## 🛠 Hardware Setup

| VL53L0X Pin | Feather V2 Pin | Function |
| :--- | :--- | :--- |
| VIN | 3V | 3.3V Power |
| GND | GND | Ground |
| SCL | SCL (GPIO 20) | I2C Clock |
| SDA | SDA (GPIO 22) | I2C Data |

*Note: XSHUT is pulled HIGH internally on most breakout boards. For this implementation, it is left disconnected.*

## 📂 Project Structure

```text
vl53l0x_test/
├── CMakeLists.txt        # Root config (defines EXTRA_COMPONENT_DIRS)
├── main/
│   ├── main.cpp          # Application logic & High-speed loop
│   └── CMakeLists.txt    # Component requirements (Requires vl53l0x)
└── components/
    └── vl53l0x/          # Manual driver component (ST API based)
