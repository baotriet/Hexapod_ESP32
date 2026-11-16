# Hexapod ESP32

ESP32-CAM firmware for a six-legged robot. It provides servo-based leg motion,
a browser controller over Wi-Fi, camera streaming, and optional sensor-assisted
stability and foot-contact features.
https://youtube.com/playlist?list=PLChueof2w0Pg&si=ozY5G9eBqP_QdN1l

## Features

- 18-servo hexapod motion and gait control through two PCA9685 PWM drivers.
- Browser-based robot control and camera streaming.
- MPU6050 orientation correction and foot-contact sensing.
- Battery voltage monitoring, buzzer feedback, and browser-based firmware updates.
- Network credentials provisioned over Serial and stored in ESP32 non-volatile
  storage rather than embedded in the source.

The firmware project and its PlatformIO configuration are in [`src/`](src/).
Mechanical models are in [`CAD/`](CAD/) and board schematics are in
[`schematic/`](schematic/).

## Requirements

- ESP32-CAM using the AI Thinker camera pin layout.
- [PlatformIO Core](https://platformio.org/install/cli) or VS Code with the
  PlatformIO extension.
- USB-to-serial adapter for initial firmware upload and Serial provisioning.

PlatformIO uses the `esp32cam` board, Arduino framework, and pinned library
versions defined in [`src/platformio.ini`](src/platformio.ini). PlatformIO
downloads the platform and libraries on the first build.

## Build

From the repository root, enter the firmware project directory and build:

```powershell
Set-Location src
pio run -e esp32
```

The resulting firmware image is `.pio/build/esp32/firmware.bin` (relative to
`src/`).

## Upload and monitor

Connect the ESP32-CAM using a compatible USB-to-serial adapter. If PlatformIO
does not detect the serial port, configure the port in the PlatformIO project.

```powershell
pio run -e esp32 -t upload
pio device monitor -e esp32
```

The Serial monitor is configured for 115200 baud with LF line endings.

## Network setup and control

The firmware does not start robot motion automatically at boot. Provision
network settings using the Serial monitor while the robot is disabled:

```text
NET {"ssid":"your-router","password":"your-router-password"}
HOTSPOT {"ssid":"your-hotspot-name","password":"your-hotspot-password"}
```

Each password must be 8–63 bytes; each SSID must be 1–32 bytes. Reset the robot
after saving settings for them to take effect. A hotspot can be configured
without router credentials. On startup, the Serial output reports the hotspot
address, or the router control-page address when connected. Open that address
in a browser to use the controller.

**Security:** Network settings are stored on the device. Firmware updates over
the browser are not password-protected, so use the controller only on a trusted
network and do not expose it to the public internet.

## Hardware references

- [ESP32 shield schematic](schematic/ESP32_shield.png)
- [PCA9685 schematic](schematic/PCA9685.png)
- [CAD models](CAD/)
- [Gerber files](Gerber/)
Check wiring, power requirements, and servo limits for your build before
operating the robot.
