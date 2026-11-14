# Hexapod firmware build

This repository contains the ESP32-CAM firmware source and its PlatformIO
configuration.

## Requirements

- VS Code with the PlatformIO extension, or PlatformIO Core.
- Internet access for PlatformIO to install the ESP32 platform and libraries on
  the first build.

The project targets the `esp32cam` board with the Arduino framework. Library
versions are pinned in [`platformio.ini`](platformio.ini):

- ArduinoJson 7.4.2
- Adafruit ADS1X15 2.5.0
- Adafruit PWM Servo Driver Library 3.0.2
- Adafruit PCF8574 1.1.1
- MPU6050_light 1.2.1
- PID_v2 2.0.1
- Adafruit BusIO 1.17.4

## Build

Run from the repository root:

```powershell
pio run -e esp32
```

The firmware image is written to `.pio/build/esp32/firmware.bin`.

## Upload and serial monitor

Connect the ESP32-CAM using the appropriate USB-to-serial adapter and set its
port if PlatformIO does not detect it automatically. Then run:

```powershell
pio run -e esp32 -t upload
pio device monitor -e esp32
```

The monitor uses 115200 baud and LF line endings. Application Serial output is
disabled by default in the build flags.

## PlatformIO configuration example

The repository stores firmware sources at its root, so `src_dir = .` tells
PlatformIO where to find them.

```ini
[platformio]
default_envs = esp32
src_dir = .

[env:esp32]
platform = espressif32@5.0.0
board = esp32cam
framework = arduino
lib_deps =
    https://github.com/bblanchon/ArduinoJson.git#v7.4.2
    https://github.com/adafruit/Adafruit_ADS1X15.git#2.5.0
    https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library.git#3.0.2
    https://github.com/adafruit/Adafruit_PCF8574.git#1.1.1
    https://github.com/rfetick/MPU6050_light.git#v1.2.1
    https://github.com/gelraen/Arduino-PID-Library.git#v2.0.1
    https://github.com/adafruit/Adafruit_BusIO.git#1.17.4
```
