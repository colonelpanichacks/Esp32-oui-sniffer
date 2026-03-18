# BLE/WiFi OUI Detection + Meshtastic

Configurable BLE and WiFi surveillance detection firmware with Meshtastic mesh network output. Configure target devices via web UI, detect them over BLE and WiFi, and receive alerts on your Meshtastic mesh.

Part of the [mesh-detect](https://github.com/colonelpanichacks/mesh-detect) hardware project.

## How It Works

1. **Boot** — Device starts in config mode, creates a WiFi access point
2. **Configure** — Connect to the AP, open the web UI, add target OUI prefixes / MAC addresses / BLE device names
3. **Scan** — Device shuts down the AP and begins dual BLE + WiFi scanning
4. **Alert** — When a target is detected, a plain-text message is sent over UART to your Meshtastic radio, which broadcasts it across the mesh

```
ESP32-S3 (GPIO4 TX) ──Serial 115200──> Meshtastic Device ──LoRa──> Mesh Network
```

## Detection Methods

| Method | What It Matches | Example |
|--------|----------------|---------|
| **OUI Prefix** | First 3 bytes of MAC (manufacturer) | `58:8e:81` |
| **Full MAC** | Exact device MAC address | `58:8e:81:ab:cd:ef` |
| **Device Name** | BLE advertised name (case-insensitive substring) | `Flock` |
| **WiFi Probe** | Source MAC from WiFi management frames (promiscuous mode) | Channels 1-13 |

## Meshtastic Messages

Plain text over Serial1 UART at 115200 baud:

```
Device Detected: 58:8e:81:ab:cd:ef
Redetection: Device 58:8e:81:ab:cd:ef
```

Configure your Meshtastic node: Serial module enabled, 115200 baud, TextMessage protocol, RX/TX pins 19/20.

## Web UI

Green-on-black terminal interface at `http://192.168.4.1`:

- OUI prefixes, full MAC addresses, BLE device names
- AP SSID/password config
- Live detection feed
- Burn-in mode (permanently lock settings, skip config on boot)

Default AP: `mesh-detect` / `meshdetect1`

## Hardware

| Component | Pin | Function |
|-----------|-----|----------|
| Serial1 TX | GPIO4 (D4) | UART to Meshtastic RX |
| Serial1 RX | GPIO5 (D5) | UART from Meshtastic TX |
| NeoPixel | GPIO1 (D0) | Green breathing = scanning, red/white flash = detection |
| Onboard LED | GPIO21 | Status (inverted logic) |
| Boot Button | GPIO0 | Hold 1.5s to force config mode |

**Target**: Seeed Studio XIAO ESP32-S3 + Heltec LoRa V3 (or any Meshtastic device with serial module)

## Build & Flash

### PlatformIO

```bash
pio run                    # Build
pio run -t upload          # Flash
pio device monitor -b 115200   # Monitor
```

### Pre-built Binary

```bash
pip install esptool pyserial
python flash.py            # Auto-detect and flash
python flash.py --erase    # Erase flash first (clears saved config)
python flash.py --single   # Flash one board and exit
```

Batch flashing supported — plug in boards one after another.

## Dependencies

- [NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino) 2.x
- [ESP Async WebServer](https://github.com/mathieucarbou/ESPAsyncWebServer) 3.0.6+
- [Adafruit NeoPixel](https://github.com/adafruit/Adafruit_NeoPixel) 1.12+

## Author

[colonelpanichacks](https://github.com/colonelpanichacks)
