# LCT Industrial Server

Arduino IDE firmware for the **Waveshare ESP32-S3-POE-ETH-8DI-8DO** industrial controller board.

This sketch turns the ESP32-S3 board into a small industrial web, Ethernet, Modbus, MQTT, SD logging, and OTA update server. It includes direct drivers for the onboard TCA9554PWR digital-output expander and PCF85063 RTC, so no Waveshare-specific Arduino support library is required for those devices.

## Features

- WiFi fallback access point with generated SSID/password
- WiFi STA client configuration page
- W5500 Ethernet support with DHCP or static IP
- WiFi web interface on port `80`
- Ethernet lightweight web interface on port `80`
- WebSocket live I/O status server on port `81`
- Modbus TCP server on Ethernet port `502`
- Modbus TCP host/client polling
- Modbus RTU slave/server over RS485
- Modbus RTU master polling over RS485
- 8 digital inputs and 8 digital outputs
- Configurable I/O names and invert logic
- DO1 optional WiFi connection indicator
- SD card browser with upload, download, and delete
- CSV I/O logging using RTC timestamps
- Modbus CSV logging with register descriptions and formulas
- I/O event duration logging
- MQTT status publishing and command subscription
- NTP-to-RTC synchronization over WiFi or Ethernet
- Web OTA firmware upload over WiFi
- JSON REST API
- CAN/TWAI configuration and frame monitor
- Persistent configuration using ESP32 Preferences/NVS

## Target Hardware

This firmware is written for:

**Waveshare ESP32-S3-POE-ETH-8DI-8DO**

### Pin Summary

| Function | Pin / Interface |
|---|---|
| DI1-DI8 | GPIO4-GPIO11 |
| DO1-DO8 | TCA9554PWR EXIO1-EXIO8 over I2C |
| I2C SDA | GPIO42 |
| I2C SCL | GPIO41 |
| RTC INT | GPIO40 |
| W5500 INT | GPIO12 |
| W5500 MOSI | GPIO13 |
| W5500 MISO | GPIO14 |
| W5500 SCLK | GPIO15 |
| W5500 CS | GPIO16 |
| W5500 RST | GPIO39 |
| RS485 TX | GPIO17 |
| RS485 RX | GPIO18 |
| RS485 RTS/DE | GPIO21 |
| SD_MMC CLK | GPIO48 |
| SD_MMC CMD | GPIO47 |
| SD_MMC D0 | GPIO45 |
| Buzzer | GPIO46 |
| RGB control | GPIO38 |

## Required Arduino Environment

Install the following before compiling:

- Arduino IDE 2.x recommended
- ESP32 board package 3.x
- **Ethernet** by Arduino
- **WebSockets** by Markus Sattler
- **PubSubClient** by Nick O'Leary

The sketch also uses ESP32 core libraries including `WiFi`, `WebServer`, `Preferences`, `SD_MMC`, `Update`, and the ESP32 `twai` driver.

## Arduino IDE Board Settings

Recommended starting point:

| Setting | Value |
|---|---|
| Board | ESP32S3 Dev Module or the matching Waveshare ESP32-S3 board profile if installed |
| USB CDC On Boot | Enabled, if serial monitor over USB is desired |
| Flash Size | Match your board, commonly 16MB |
| Partition Scheme | Use a scheme with enough OTA/app space for the compiled firmware |
| Upload Speed | 921600 or lower if uploads are unstable |
| PSRAM | Enable if your selected board profile supports it |

## First Boot

On first boot, the board creates a fallback WiFi access point.

Default access point format:

```text
SSID:     ESP32-XXXX
Password: LCT-XXXX
IP:       192.168.4.1
```

`XXXX` is generated from the board MAC suffix.

Connect to the hotspot, then open:

```text
http://192.168.4.1/
```

The home page displays WiFi, Ethernet, RTC, SD, MQTT, Modbus, and I/O status.

## Web Pages

| Page | Purpose |
|---|---|
| `/` | Main status page |
| `/wifi` | WiFi AP/STA configuration |
| `/ethernet` | Ethernet and Modbus TCP configuration |
| `/rs485` | RS485 and Modbus RTU configuration |
| `/io` | Digital input/output naming, logic, and manual output control |
| `/sd` | SD file browser, upload, download, and delete |
| `/can` | CAN/TWAI configuration and frame monitor |
| `/mqtt` | MQTT broker and topic configuration |
| `/time` | RTC and NTP configuration |
| `/ota` | WiFi OTA firmware upload |
| `/json` | JSON status output |

The WiFi interface uses the standard ESP32 `WebServer`. The Ethernet interface uses a separate lightweight W5500 HTTP router so the same device can be managed from either network path.

## WiFi Behavior

The firmware supports two WiFi modes:

- **Hotspot/AP Only**
- **AP + Connect to WiFi**

When STA mode is configured, the board attempts to connect to the saved WiFi network. The fallback access point is advertised while STA is disconnected. Once STA WiFi connects, the fallback hotspot is shut off.

If enabled, **DO1** acts as a WiFi connection indicator:

- Blinks once per second while waiting for STA connection
- Turns on solid after STA connects

## Ethernet

The onboard W5500 Ethernet interface supports:

- DHCP
- Static IP
- Ethernet web interface
- Modbus TCP server
- Modbus TCP host/client polling
- MQTT over Ethernet when selected
- NTP over Ethernet when selected

The Ethernet MAC is generated in firmware from the ESP32 MAC information.

## Modbus

### Holding Registers

| Register | Meaning |
|---:|---|
| `0` | Digital input bitmask |
| `1` | Digital output bitmask |
| `2` | TCA9554PWR detected |
| `3` | PCF85063 RTC detected |
| `4` | SD mounted |
| `5` | WiFi STA connected |
| `6` | Ethernet started |
| `7` | Free heap low word |

Writing holding register `1` controls all digital outputs by bitmask.

### Modbus TCP

- Server listens on Ethernet port `502`
- Host/client polling can read a configured remote register range
- Register monitor supports custom display descriptions and formulas
- Register list can be uploaded by CSV

CSV upload format:

```csv
register address,register description,math formula
40001,Tank Temperature,x/10
40002,Temperature F,x*1.8+32
```

The formula column is optional. If omitted, `x` is used.

### Modbus RTU

The RS485 port supports:

- RTU slave/server mode
- RTU master polling mode
- Configurable baud rate, slave ID, target ID, start register, count, and poll interval

## MQTT

MQTT support includes:

- Configurable broker host, port, username, and password
- Selectable network interface: WiFi, Ethernet, or Auto
- Periodic JSON status publishing
- Digital input/output bitmask publishing
- Command subscriptions for output control

Default base topic:

```text
lct/ESP32-XXXX
```

Published topics include:

```text
<base>/status
<base>/interface
<base>/json
<base>/di_mask
<base>/do_mask
```

Subscribed command topics include:

```text
<base>/cmd/output
<base>/cmd/output_mask
```

## REST API

| Endpoint | Method | Description |
|---|---|---|
| `/json` | GET | Full status JSON |
| `/api/status` | GET | Full status JSON |
| `/api/io` | GET | Alias for status JSON |
| `/api/output?ch=N&state=0|1` | GET | Set one output channel |
| `/api/output_mask?mask=N` | GET | Set all outputs by bitmask |
| `/api/registers` | GET | Return all 128 holding registers |

Output channels are numbered starting at `1` in the web/API interface.

Example:

```text
http://192.168.4.1/api/output?ch=1&state=1
http://192.168.4.1/api/output_mask?mask=255
http://192.168.4.1/api/registers
```

## WebSocket

Live I/O updates are broadcast on port `81`.

Example AP WebSocket URL:

```text
ws://192.168.4.1:81
```

## SD Card and Logging

The firmware mounts the SD card using `SD_MMC` and supports browser-based file management.

Logging features:

- I/O CSV logging
- Modbus CSV logging to `/modbus_log.csv`
- I/O event duration logging

RTC timestamps are provided by the PCF85063 RTC. The RTC can be synchronized from NTP using WiFi or Ethernet.

## OTA Firmware Update

OTA updates are handled from the WiFi web interface only.

Open:

```text
http://192.168.4.1/ota
```

or use the board STA WiFi IP after it connects to your network.

To create the firmware file in Arduino IDE:

1. Open the sketch.
2. Select the correct ESP32-S3 board and settings.
3. Click **Sketch > Export Compiled Binary**.
4. Upload the generated `.bin` file through the OTA page.

Do not remove power during an OTA update.

## CAN/TWAI

The firmware includes ESP32 TWAI/CAN support with configurable TX/RX pins, bitrate, and listen-only mode. CAN status and recent frames are available from the `/can` page.

Use an appropriate external CAN transceiver. The ESP32-S3 TWAI peripheral does not connect directly to a CAN bus without transceiver hardware.

## Persistent Configuration

Configuration is saved in ESP32 NVS using the `Preferences` library. Saved settings survive reboot and power loss.

Stored settings include:

- WiFi mode and STA credentials
- Ethernet mode and static network settings
- Modbus TCP and RTU settings
- MQTT settings
- NTP settings
- CSV logging intervals
- I/O names and invert logic
- CAN settings
- Modbus monitor register list

## Safety Notes

This firmware can control physical outputs. Test with disconnected loads first.

Before connecting real equipment:

- Confirm the board output polarity
- Confirm DO invert settings
- Confirm Modbus write behavior
- Confirm safe startup output states
- Verify relay/load ratings
- Use proper fusing and isolation
