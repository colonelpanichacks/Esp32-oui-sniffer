/*
 * MESH-DETECT - BLE/WiFi Surveillance Detection + Meshtastic Mesh Output
 * colonelpanichacks
 *
 * Configurable BLE and WiFi OUI/MAC/name detection with web UI.
 * Sends detection alerts to Meshtastic device over UART Serial1.
 * No buzzer. NeoPixel for visual feedback.
 *
 * Target: Seeed Studio Xiao ESP32-S3
 * Meshtastic: Heltec LoRa V3 (or similar) via Serial1
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <nvs_flash.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <NimBLEDevice.h>
#include <esp_log.h>
#include <vector>
#include <algorithm>
#include <Adafruit_NeoPixel.h>

// ================================
// Pin Definitions - Xiao ESP32 S3
// ================================
#define SERIAL1_TX_PIN  4    // GPIO4 (D4) -> Meshtastic RX
#define SERIAL1_RX_PIN  5    // GPIO5 (D5) -> Meshtastic TX
#define NEOPIXEL_PIN    1    // GPIO1 (D0) - moved from GPIO4 to avoid Serial1 conflict
#define NEOPIXEL_COUNT  1
#define NEOPIXEL_BRIGHTNESS 50
#define NEOPIXEL_DETECTION_BRIGHTNESS 200
#define LED_PIN         21   // Onboard LED (inverted logic: LOW=ON, HIGH=OFF)

// ================================
// Timing Constants
// ================================
#define REDETECTION_COOLDOWN  30000  // 30 seconds before re-alerting same device
#define SHORT_COOLDOWN        3000   // 3 second short cooldown
#define CONFIG_TIMEOUT        20000  // 20s before auto-starting scan if filters exist
#define BLE_SCAN_INTERVAL     1349
#define BLE_SCAN_WINDOW       449
#define WIFI_CHANNEL_HOP_MS   2000   // Channel hop interval for WiFi promiscuous
#define DEVICE_SAVE_INTERVAL  10000  // Auto-save devices every 10s
#define MAX_DEVICES           100

// ================================
// Data Structures
// ================================
struct DeviceInfo {
    String macAddress;
    int rssi;
    unsigned long firstSeen;
    unsigned long lastSeen;
    bool inCooldown;
    unsigned long cooldownUntil;
    String filterDescription;
};

struct TargetFilter {
    String identifier;
    bool isFullMAC;
    String description;
};

// ================================
// Operating Modes
// ================================
enum OperatingMode {
    CONFIG_MODE,
    SCANNING_MODE
};

// ================================
// Global Variables
// ================================
static OperatingMode currentMode = CONFIG_MODE;
static AsyncWebServer server(80);
static Preferences preferences;

// Target lists
static std::vector<TargetFilter> targetFilters;
static std::vector<String> nameFilters;
static std::vector<DeviceInfo> devices;

// AP config
static String apSSID = "mesh-detect";
static String apPassword = "meshdetect1";

// NeoPixel
static Adafruit_NeoPixel strip(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
static bool detectionFlashActive = false;
static unsigned long detectionFlashStart = 0;

// Scheduling
static unsigned long configStartTime = 0;
static unsigned long lastConfigActivity = 0;
static unsigned long modeSwitchScheduled = 0;

// WiFi promiscuous
static int currentWiFiChannel = 1;
static unsigned long lastChannelHop = 0;

// Device save timer
static unsigned long lastDeviceSave = 0;

// Lock/burn-in
static bool configLocked = false;

// Forward declarations
static void startScanningMode();
static void startConfigMode();
static void sendMeshtasticAlert(const String& mac, bool isRedetection);
static void startDetectionFlash();

// ================================
// Serial Functions
// ================================
static void initializeSerial() {
    Serial.begin(115200);
    delay(100);
    Serial.println("[MESH-DETECT] USB Serial started");

    Serial1.begin(115200, SERIAL_8N1, SERIAL1_RX_PIN, SERIAL1_TX_PIN);
    Serial.println("[MESH-DETECT] Serial1 (Meshtastic) started on TX=GPIO4, RX=GPIO5");
}

static bool isSerialConnected() {
    return Serial;
}

// ================================
// LED Control (inverted logic)
// ================================
static void ledOn() {
    digitalWrite(LED_PIN, LOW);
}

static void ledOff() {
    digitalWrite(LED_PIN, HIGH);
}

// ================================
// MAC Address Randomization
// ================================
static void randomizeMAC() {
    uint8_t mac[6];
    uint32_t r1 = esp_random();
    uint32_t r2 = esp_random();
    mac[0] = (r1 >> 0) & 0xFF;
    mac[1] = (r1 >> 8) & 0xFF;
    mac[2] = (r1 >> 16) & 0xFF;
    mac[3] = (r1 >> 24) & 0xFF;
    mac[4] = (r2 >> 0) & 0xFF;
    mac[5] = (r2 >> 8) & 0xFF;
    mac[0] = (mac[0] | 0x02) & 0xFE; // Locally administered, unicast

    esp_wifi_set_mac(WIFI_IF_AP, mac);
    Serial.printf("[MESH-DETECT] Randomized MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// ================================
// NeoPixel Functions
// ================================
static uint32_t hsvToRgb(uint16_t h, uint8_t s, uint8_t v) {
    uint8_t r, g, b;
    if (s == 0) {
        r = g = b = v;
    } else {
        uint8_t region = h / 43;
        uint8_t remainder = (h - (region * 43)) * 6;
        uint8_t p = (v * (255 - s)) >> 8;
        uint8_t q = (v * (255 - ((s * remainder) >> 8))) >> 8;
        uint8_t t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;
        switch (region) {
            case 0: r = v; g = t; b = p; break;
            case 1: r = q; g = v; b = p; break;
            case 2: r = p; g = v; b = t; break;
            case 3: r = p; g = q; b = v; break;
            case 4: r = t; g = p; b = v; break;
            default: r = v; g = p; b = q; break;
        }
    }
    return strip.Color(r, g, b);
}

static void initializeNeoPixel() {
    strip.begin();
    strip.setBrightness(NEOPIXEL_BRIGHTNESS);
    strip.clear();
    strip.show();
}

// Green breathing animation (scanning active)
static void normalBreathingAnimation() {
    static unsigned long lastUpdate = 0;
    static float brightness = 0.0;
    static bool increasing = true;

    unsigned long now = millis();
    if (now - lastUpdate >= 20) {
        lastUpdate = now;
        if (increasing) {
            brightness += 0.02;
            if (brightness >= 1.0) { brightness = 1.0; increasing = false; }
        } else {
            brightness -= 0.02;
            if (brightness <= 0.1) { brightness = 0.1; increasing = true; }
        }
        // Green hue (85 in 0-255 HSV) for cyberpunk terminal theme
        uint32_t color = hsvToRgb(85, 255, (uint8_t)(NEOPIXEL_BRIGHTNESS * brightness));
        strip.setPixelColor(0, color);
        strip.show();
    }
}

// Detection flash animation (red/white rapid flash)
static void detectionFlashAnimation() {
    unsigned long elapsed = millis() - detectionFlashStart;
    int flashPhase = (elapsed / 150) % 6; // 6 phases, 150ms each

    uint32_t color;
    if (flashPhase % 2 == 0) {
        color = strip.Color(NEOPIXEL_DETECTION_BRIGHTNESS, 0, 0); // Red
    } else {
        color = strip.Color(NEOPIXEL_DETECTION_BRIGHTNESS, NEOPIXEL_DETECTION_BRIGHTNESS, NEOPIXEL_DETECTION_BRIGHTNESS); // White
    }
    strip.setPixelColor(0, color);
    strip.show();

    if (elapsed >= 900) { // 6 x 150ms
        detectionFlashActive = false;
    }
}

static void updateNeoPixelAnimation() {
    if (detectionFlashActive) {
        detectionFlashAnimation();
    } else if (currentMode == SCANNING_MODE) {
        normalBreathingAnimation();
    }
}

static void startDetectionFlash() {
    detectionFlashActive = true;
    detectionFlashStart = millis();
}

// ================================
// Meshtastic Serial Output
// ================================
static void sendMeshtasticAlert(const String& mac, bool isRedetection) {
    if (isRedetection) {
        Serial1.print("Redetection: Device ");
    } else {
        Serial1.print("Device Detected: ");
    }
    Serial1.println(mac.c_str());
    Serial1.flush();

    if (isSerialConnected()) {
        Serial.printf("[MESH] %s sent: %s\n",
            isRedetection ? "Redetection" : "Detection", mac.c_str());
    }
}

// ================================
// MAC Address Utility Functions
// ================================
static void normalizeMACAddress(String& mac) {
    mac.toLowerCase();
    mac.replace("-", ":");
    mac.replace(" ", "");
}

static bool isValidMAC(const String& mac) {
    String normalized = mac;
    normalizeMACAddress(normalized);

    if (normalized.length() != 8 && normalized.length() != 17) {
        return false;
    }
    for (int i = 0; i < (int)normalized.length(); i++) {
        char c = normalized.charAt(i);
        if (i % 3 == 2) {
            if (c != ':') return false;
        } else {
            if (!isxdigit(c)) return false;
        }
    }
    return true;
}

static bool matchesTargetFilter(const String& deviceMAC, String& matchedDescription) {
    String normalizedDeviceMAC = deviceMAC;
    normalizeMACAddress(normalizedDeviceMAC);

    for (const TargetFilter& filter : targetFilters) {
        String filterID = filter.identifier;
        normalizeMACAddress(filterID);

        if (filter.isFullMAC) {
            if (normalizedDeviceMAC.equals(filterID)) {
                matchedDescription = filter.description;
                return true;
            }
        } else {
            if (normalizedDeviceMAC.startsWith(filterID)) {
                matchedDescription = filter.description;
                return true;
            }
        }
    }
    return false;
}

static bool matchesNameFilter(const String& deviceName) {
    if (deviceName.length() == 0) return false;
    String lowerName = deviceName;
    lowerName.toLowerCase();

    for (const String& pattern : nameFilters) {
        String lowerPattern = pattern;
        lowerPattern.toLowerCase();
        if (lowerName.indexOf(lowerPattern) >= 0) {
            return true;
        }
    }
    return false;
}

// ================================
// Handle Detection (shared by BLE + WiFi)
// ================================
static void handleDetection(const String& mac, int rssi, const String& filterDesc, const String& source) {
    unsigned long now = millis();
    bool known = false;

    for (auto& dev : devices) {
        if (dev.macAddress == mac) {
            known = true;

            if (dev.inCooldown && now < dev.cooldownUntil) {
                return; // Still in cooldown
            }
            dev.inCooldown = false;

            unsigned long timeSince = now - dev.lastSeen;

            if (timeSince >= REDETECTION_COOLDOWN) {
                // Redetection after 30s
                sendMeshtasticAlert(mac, true);
                startDetectionFlash();
                ledOn();
                dev.inCooldown = true;
                dev.cooldownUntil = now + 10000;

                if (isSerialConnected()) {
                    Serial.printf("[%s] RE-30s | %s | RSSI: %d | %s\n",
                        source.c_str(), mac.c_str(), rssi, filterDesc.c_str());
                }
            } else if (timeSince >= SHORT_COOLDOWN) {
                // Short redetection
                sendMeshtasticAlert(mac, true);
                startDetectionFlash();
                dev.inCooldown = true;
                dev.cooldownUntil = now + SHORT_COOLDOWN;

                if (isSerialConnected()) {
                    Serial.printf("[%s] RE-3s | %s | RSSI: %d | %s\n",
                        source.c_str(), mac.c_str(), rssi, filterDesc.c_str());
                }
            }

            dev.lastSeen = now;
            dev.rssi = rssi;
            break;
        }
    }

    if (!known) {
        DeviceInfo newDev;
        newDev.macAddress = mac;
        newDev.rssi = rssi;
        newDev.firstSeen = now;
        newDev.lastSeen = now;
        newDev.inCooldown = true;
        newDev.cooldownUntil = now + SHORT_COOLDOWN;
        newDev.filterDescription = filterDesc;

        if (devices.size() >= MAX_DEVICES) {
            devices.erase(devices.begin()); // Remove oldest
        }
        devices.push_back(newDev);

        sendMeshtasticAlert(mac, false);
        startDetectionFlash();
        ledOn();

        if (isSerialConnected()) {
            Serial.printf("[%s] NEW | %s | RSSI: %d | %s\n",
                source.c_str(), mac.c_str(), rssi, filterDesc.c_str());
        }
    }
}

// ================================
// NVS Configuration Storage
// ================================
static void saveConfiguration() {
    preferences.begin("meshdetect", false);
    preferences.putInt("filterCount", targetFilters.size());

    for (int i = 0; i < (int)targetFilters.size(); i++) {
        String keyId = "id_" + String(i);
        String keyMAC = "mac_" + String(i);
        String keyDesc = "desc_" + String(i);
        preferences.putString(keyId.c_str(), targetFilters[i].identifier);
        preferences.putBool(keyMAC.c_str(), targetFilters[i].isFullMAC);
        preferences.putString(keyDesc.c_str(), targetFilters[i].description);
    }

    // Save name filters
    preferences.putInt("nameCount", nameFilters.size());
    for (int i = 0; i < (int)nameFilters.size(); i++) {
        String key = "name_" + String(i);
        preferences.putString(key.c_str(), nameFilters[i]);
    }

    preferences.end();
    if (isSerialConnected()) {
        Serial.printf("[NVS] Saved %d filters, %d name patterns\n",
            targetFilters.size(), nameFilters.size());
    }
}

static void loadConfiguration() {
    preferences.begin("meshdetect", true);
    int filterCount = preferences.getInt("filterCount", 0);

    targetFilters.clear();
    for (int i = 0; i < filterCount; i++) {
        String keyId = "id_" + String(i);
        String keyMAC = "mac_" + String(i);
        String keyDesc = "desc_" + String(i);

        TargetFilter filter;
        filter.identifier = preferences.getString(keyId.c_str(), "");
        filter.isFullMAC = preferences.getBool(keyMAC.c_str(), false);
        filter.description = preferences.getString(keyDesc.c_str(), "");
        if (filter.identifier.length() > 0) {
            targetFilters.push_back(filter);
        }
    }

    // Load name filters
    int nameCount = preferences.getInt("nameCount", 0);
    nameFilters.clear();
    for (int i = 0; i < nameCount; i++) {
        String key = "name_" + String(i);
        String name = preferences.getString(key.c_str(), "");
        if (name.length() > 0) {
            nameFilters.push_back(name);
        }
    }

    preferences.end();
    if (isSerialConnected()) {
        Serial.printf("[NVS] Loaded %d filters, %d name patterns\n",
            targetFilters.size(), nameFilters.size());
    }
}

static void saveAPConfig() {
    preferences.begin("meshdetect", false);
    preferences.putString("ap_ssid", apSSID);
    preferences.putString("ap_pass", apPassword);
    preferences.end();
}

static void loadAPConfig() {
    preferences.begin("meshdetect", true);
    apSSID = preferences.getString("ap_ssid", "mesh-detect");
    apPassword = preferences.getString("ap_pass", "meshdetect1");
    preferences.end();
}

static void saveDetectedDevices() {
    preferences.begin("meshdetect", false);
    int count = min((int)devices.size(), MAX_DEVICES);
    preferences.putInt("deviceCount", count);

    for (int i = 0; i < count; i++) {
        preferences.putString(("dm_" + String(i)).c_str(), devices[i].macAddress);
        preferences.putInt(("dr_" + String(i)).c_str(), devices[i].rssi);
        preferences.putULong(("dt_" + String(i)).c_str(), devices[i].lastSeen);
        preferences.putString(("df_" + String(i)).c_str(), devices[i].filterDescription);
    }
    preferences.end();
}

static void loadDetectedDevices() {
    preferences.begin("meshdetect", true);
    int count = preferences.getInt("deviceCount", 0);

    devices.clear();
    for (int i = 0; i < count; i++) {
        DeviceInfo dev;
        dev.macAddress = preferences.getString(("dm_" + String(i)).c_str(), "");
        dev.rssi = preferences.getInt(("dr_" + String(i)).c_str(), 0);
        dev.lastSeen = preferences.getULong(("dt_" + String(i)).c_str(), 0);
        dev.filterDescription = preferences.getString(("df_" + String(i)).c_str(), "");
        dev.firstSeen = dev.lastSeen;
        dev.inCooldown = false;
        dev.cooldownUntil = 0;
        if (dev.macAddress.length() > 0) {
            devices.push_back(dev);
        }
    }
    preferences.end();

    if (isSerialConnected()) {
        Serial.printf("[NVS] Loaded %d detected devices\n", devices.size());
    }
}

static void clearDetectedDevices() {
    devices.clear();
    preferences.begin("meshdetect", false);
    preferences.putInt("deviceCount", 0);
    preferences.end();
}

// ================================
// BLE Scan Callbacks (NimBLE 2.x)
// ================================
class ScanCallbacks : public NimBLEScanCallbacks {
public:
    void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override {
        if (currentMode != SCANNING_MODE) return;

        std::string macStd = advertisedDevice->getAddress().toString();
        String mac = String(macStd.c_str());
        int rssi = advertisedDevice->getRSSI();

        // Check MAC/OUI filters
        String matchedDescription;
        if (matchesTargetFilter(mac, matchedDescription)) {
            handleDetection(mac, rssi, matchedDescription, "BLE");
            return;
        }

        // Check name filters
        std::string nameStd = advertisedDevice->getName();
        String name = String(nameStd.c_str());
        if (name.length() > 0 && matchesNameFilter(name)) {
            String desc = "Name: " + name;
            handleDetection(mac, rssi, desc, "BLE");
        }
    }

    void onScanEnd(const NimBLEScanResults& results, int reason) override {
        if (isSerialConnected()) {
            Serial.printf("[BLE] Scan cycle ended (reason: %d, found: %d) - restarting\n",
                reason, results.getCount());
        }
        NimBLEDevice::getScan()->start(0); // Restart continuous scan
    }
};

// ================================
// WiFi Promiscuous Mode Callback
// ================================
static void wifiSnifferCallback(void* buf, wifi_promiscuous_pkt_type_t type) {
    if (type != WIFI_PKT_MGMT) return;
    if (currentMode != SCANNING_MODE) return;

    wifi_promiscuous_pkt_t* pkt = (wifi_promiscuous_pkt_t*)buf;
    uint8_t* payload = pkt->payload;

    // Extract source MAC (bytes 10-15 of 802.11 frame header)
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
             payload[10], payload[11], payload[12],
             payload[13], payload[14], payload[15]);

    String mac = String(macStr);
    int rssi = pkt->rx_ctrl.rssi;

    String matchedDescription;
    if (matchesTargetFilter(mac, matchedDescription)) {
        handleDetection(mac, rssi, matchedDescription, "WiFi");
    }
}

// ================================
// Text Line Parser Helper
// ================================
static void parseTextareaLines(const String& data, std::vector<String>& out) {
    int start = 0;
    int end = data.indexOf('\n');
    while (start < (int)data.length()) {
        String line;
        if (end == -1) {
            line = data.substring(start);
            start = data.length();
        } else {
            line = data.substring(start, end);
            start = end + 1;
            end = data.indexOf('\n', start);
        }
        line.trim();
        line.replace("\r", "");
        if (line.length() > 0) {
            out.push_back(line);
        }
    }
}

// ================================
// Web UI HTML
// ================================
static const char CONFIG_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>MESH-DETECT</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#000;color:#0f0;font-family:monospace;padding:8px;min-height:100vh}
.t{border:2px solid #0f0;padding:10px;min-height:calc(100vh - 16px)}
.h{text-align:center;padding-bottom:6px;margin-bottom:8px;border-bottom:1px solid #0f0}
.ti{font-size:22px;font-weight:bold;letter-spacing:3px}
.sub{font-size:9px;opacity:.7;margin-top:2px}
.mesh-bar{background:#001a00;border:1px solid #0f0;padding:6px 10px;margin-bottom:10px;font-size:11px;display:flex;justify-content:space-between;align-items:center}
.mesh-ok{color:#0f0}.mesh-dot{display:inline-block;width:8px;height:8px;border-radius:50%;background:#0f0;margin-right:6px;animation:pulse 2s infinite}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.3}}
.sec{margin-bottom:10px;border:1px solid #0a0;padding:8px}
.sec-title{font-size:13px;font-weight:bold;margin-bottom:6px;color:#0f0;letter-spacing:1px}
.sec-help{font-size:9px;opacity:.5;margin-top:4px}
textarea{width:100%;min-height:60px;background:#001a00;color:#0f0;border:1px solid #0a0;font-family:monospace;font-size:12px;padding:6px;resize:vertical}
textarea:focus{outline:none;border-color:#0f0}
input[type=text]{width:100%;padding:6px;background:#001a00;color:#0f0;border:1px solid #0a0;font-family:monospace;font-size:12px;margin-top:4px}
input[type=text]:focus{outline:none;border-color:#0f0}
.btn-row{display:flex;gap:6px;margin-top:10px}
.btn{flex:1;padding:10px;background:#0f0;color:#000;border:none;font-family:monospace;font-size:13px;font-weight:bold;cursor:pointer;text-align:center;letter-spacing:1px}
.btn:active{background:#fff}
.btn-danger{background:#300;color:#f00;border:1px solid #f00}
.btn-danger:active{background:#f00;color:#000}
.btn-warn{background:#1a0a00;color:#f80;border:1px solid #f80}
.btn-warn:active{background:#f80;color:#000}
#feed{max-height:200px;overflow-y:auto;font-size:11px;line-height:1.6}
.dev{padding:4px 0;border-bottom:1px solid #030}
.dev-mac{color:#0ff}.dev-rssi{color:#888;margin-left:8px}.dev-time{color:#555;margin-left:8px;font-style:italic}
.dev-filt{color:#0a0;margin-left:8px;font-size:10px}
.dev-recent{color:#0f0}
.f{margin-top:8px;font-size:7px;text-align:center;opacity:.4}
.burn-sec{border-color:#800;margin-top:10px}
.burn-sec .sec-title{color:#f44}
.burn-warn{background:#1a0a0a;color:#faa;padding:8px;border:1px solid #800;margin:6px 0;font-size:11px;line-height:1.5}
</style></head><body>
<div class="t">
<div class="h">
<div class="ti">MESH-DETECT</div>
<div class="sub">BLE/WiFi Surveillance Detection + Meshtastic Mesh Output</div>
</div>

<div class="mesh-bar">
<span><span class="mesh-dot"></span><span class="mesh-ok">MESHTASTIC: SERIAL1 @ 115200</span></span>
<span style="opacity:.5">TX:GPIO4 RX:GPIO5</span>
</div>

<form id="configForm" method="POST" action="/save">

<div class="sec">
<div class="sec-title">> OUI PREFIXES</div>
<textarea name="ouis" id="ouis" placeholder="XX:XX:XX (one per line)">%OUI_VALUES%</textarea>
<div class="sec-help">Match all devices from a manufacturer (first 3 bytes of MAC)</div>
</div>

<div class="sec">
<div class="sec-title">> MAC ADDRESSES</div>
<textarea name="macs" id="macs" placeholder="XX:XX:XX:XX:XX:XX (one per line)">%MAC_VALUES%</textarea>
<div class="sec-help">Match specific devices by full MAC address</div>
</div>

<div class="sec">
<div class="sec-title">> DEVICE NAMES (BLE)</div>
<textarea name="names" id="names" placeholder="Device name substring (one per line)">%NAME_VALUES%</textarea>
<div class="sec-help">Match BLE devices by name (case-insensitive substring match)</div>
</div>

<div class="sec">
<div class="sec-title">> AP SETTINGS</div>
<input type="text" name="ap_ssid" id="ap_ssid" placeholder="SSID" maxlength="32" value="%SSID%">
<input type="text" name="ap_pass" id="ap_pass" placeholder="PASSWORD (8+ chars)" maxlength="63" value="%PASS%">
</div>

<div class="sec">
<div class="sec-title">> DETECTION FEED</div>
<div id="feed">No detections yet.</div>
</div>

<div class="btn-row">
<button type="submit" class="btn">[SAVE & START SCANNING]</button>
</div>
<div class="btn-row">
<button type="button" class="btn btn-danger" onclick="clearFilters()">[CLEAR FILTERS]</button>
<button type="button" class="btn btn-danger" onclick="clearDevices()">[CLEAR DEVICES]</button>
</div>

<div class="sec burn-sec">
<div class="sec-title">> BURN IN SETTINGS</div>
<div class="burn-warn">
<strong>WARNING:</strong> Permanently locks all current settings. Boots directly to scanning mode. Requires flash erase + reflash to unlock.
</div>
<div class="btn-row">
<button type="button" class="btn btn-warn" onclick="burnIn()">[LOCK CONFIGURATION]</button>
</div>
</div>

</form>

<div class="f">colonelpanichacks // mesh-detect // hold BOOT 2s for menu</div>
</div>

<script>
function loadFeed(){
fetch('/api/devices').then(r=>r.json()).then(d=>{
var f=document.getElementById('feed');
if(d.devices&&d.devices.length>0){
var h='';
d.devices.forEach(function(dev){
var ts=dev.timeSince||0;
var tStr=ts<60000?'Just now':ts<3600000?Math.floor(ts/60000)+'m ago':Math.floor(ts/3600000)+'h ago';
var cls=ts<60000?' dev-recent':'';
h+='<div class="dev"><span class="dev-mac">'+dev.mac+'</span>';
h+='<span class="dev-rssi">'+dev.rssi+' dBm</span>';
h+='<span class="dev-time'+cls+'">'+tStr+'</span>';
if(dev.filter)h+='<span class="dev-filt">'+dev.filter+'</span>';
h+='</div>';
});
f.innerHTML=h;
}
}).catch(function(){});
}
setInterval(loadFeed,3000);
window.addEventListener('DOMContentLoaded',loadFeed);

function clearFilters(){
if(confirm('Clear all filters?')){
fetch('/clear',{method:'POST'}).then(function(){location.reload();});
}}
function clearDevices(){
if(confirm('Clear all detected devices?')){
fetch('/api/clear-devices',{method:'POST'}).then(function(){location.reload();});
}}
function burnIn(){
if(!confirm('PERMANENT LOCK\n\nThis will permanently lock all settings.\nDevice boots directly to scanning.\nRequires flash erase to unlock.\n\nProceed?'))return;
var fd=new URLSearchParams(new FormData(document.getElementById('configForm')));
fetch('/api/lock-config',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:fd.toString()})
.then(r=>r.text()).then(d=>{document.open();document.write(d);document.close();})
.catch(e=>{alert('Error: '+e);});
}
</script>
</body></html>
)rawliteral";

// ================================
// Generate Config HTML with Values
// ================================
static String generateConfigHTML() {
    String html = FPSTR(CONFIG_HTML);

    String ouiValues = "";
    String macValues = "";
    for (const TargetFilter& filter : targetFilters) {
        if (filter.isFullMAC) {
            if (macValues.length() > 0) macValues += "\n";
            macValues += filter.identifier;
        } else {
            if (ouiValues.length() > 0) ouiValues += "\n";
            ouiValues += filter.identifier;
        }
    }

    String nameValues = "";
    for (const String& name : nameFilters) {
        if (nameValues.length() > 0) nameValues += "\n";
        nameValues += name;
    }

    html.replace("%OUI_VALUES%", ouiValues);
    html.replace("%MAC_VALUES%", macValues);
    html.replace("%NAME_VALUES%", nameValues);
    html.replace("%SSID%", apSSID);
    html.replace("%PASS%", apPassword);

    return html;
}

// ================================
// Process Form Submission (shared by /save and /api/lock-config)
// ================================
static void processFormSubmission(AsyncWebServerRequest *request) {
    targetFilters.clear();
    nameFilters.clear();

    // Process OUI entries
    if (request->hasParam("ouis", true)) {
        String ouiData = request->getParam("ouis", true)->value();
        ouiData.trim();
        if (ouiData.length() > 0) {
            std::vector<String> lines;
            parseTextareaLines(ouiData, lines);
            for (const String& oui : lines) {
                if (isValidMAC(oui)) {
                    TargetFilter filter;
                    filter.identifier = oui;
                    filter.description = "OUI: " + oui;
                    filter.isFullMAC = false;
                    targetFilters.push_back(filter);
                }
            }
        }
    }

    // Process MAC entries
    if (request->hasParam("macs", true)) {
        String macData = request->getParam("macs", true)->value();
        macData.trim();
        if (macData.length() > 0) {
            std::vector<String> lines;
            parseTextareaLines(macData, lines);
            for (const String& mac : lines) {
                if (isValidMAC(mac)) {
                    TargetFilter filter;
                    filter.identifier = mac;
                    filter.description = "MAC: " + mac;
                    filter.isFullMAC = true;
                    targetFilters.push_back(filter);
                }
            }
        }
    }

    // Process name entries
    if (request->hasParam("names", true)) {
        String nameData = request->getParam("names", true)->value();
        nameData.trim();
        if (nameData.length() > 0) {
            parseTextareaLines(nameData, nameFilters);
        }
    }

    // Process AP settings
    if (request->hasParam("ap_ssid", true)) {
        String newSSID = request->getParam("ap_ssid", true)->value();
        newSSID.trim();
        if (newSSID.length() > 0 && newSSID.length() <= 32) {
            apSSID = newSSID;
        }
    }
    if (request->hasParam("ap_pass", true)) {
        String newPass = request->getParam("ap_pass", true)->value();
        newPass.trim();
        if (newPass.length() == 0 || (newPass.length() >= 8 && newPass.length() <= 63)) {
            apPassword = newPass;
        }
    }

    saveAPConfig();
    saveConfiguration();
}

// ================================
// Start Config Mode (WiFi AP + Web Server)
// ================================
static void startConfigMode() {
    currentMode = CONFIG_MODE;

    Serial.println("\n========================================");
    Serial.println("  MESH-DETECT - Configuration Mode");
    Serial.printf("  Connect to WiFi: %s\n", apSSID.c_str());
    Serial.printf("  Password: %s\n", apPassword.c_str());
    Serial.println("  Open: http://192.168.4.1");
    Serial.println("========================================\n");

    WiFi.mode(WIFI_OFF);
    delay(500);
    WiFi.persistent(false);
    WiFi.mode(WIFI_AP);
    delay(200);

    randomizeMAC();

    bool apStarted;
    if (apPassword.length() >= 8) {
        apStarted = WiFi.softAP(apSSID.c_str(), apPassword.c_str());
    } else {
        apStarted = WiFi.softAP(apSSID.c_str());
    }

    Serial.printf("[CONFIG] AP started: %s | IP: %s\n",
        apStarted ? "OK" : "FAIL", WiFi.softAPIP().toString().c_str());

    delay(1000);

    configStartTime = millis();
    lastConfigActivity = millis();

    // Web server routes
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        lastConfigActivity = millis();
        request->send(200, "text/html", generateConfigHTML());
    });

    server.on("/save", HTTP_POST, [](AsyncWebServerRequest *request) {
        lastConfigActivity = millis();
        processFormSubmission(request);

        if (targetFilters.size() > 0 || nameFilters.size() > 0) {
            Serial.printf("[CONFIG] Saved %d MAC/OUI filters, %d name filters\n",
                targetFilters.size(), nameFilters.size());

            String resp = R"(<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>body{background:#000;color:#0f0;font-family:monospace;text-align:center;padding:40px}
.box{border:2px solid #0f0;padding:30px;max-width:400px;margin:0 auto}
h1{font-size:18px;margin-bottom:15px}
@keyframes b{0%,50%{opacity:1}51%,100%{opacity:0}}.blink{animation:b 1s infinite}
</style></head><body><div class="box">
<h1>CONFIGURATION SAVED</h1>
<p>)" + String(targetFilters.size() + nameFilters.size()) + R"( filters active</p>
<p style="margin-top:15px">Switching to scanning mode<span class="blink">_</span></p>
</div></body></html>)";

            request->send(200, "text/html", resp);
            modeSwitchScheduled = millis() + 3000;
        } else {
            request->send(400, "text/html",
                "<html><body style='background:#000;color:#f00;font-family:monospace;text-align:center;padding:40px'>"
                "<h1>ERROR: No valid filters</h1><p><a href='/' style='color:#0f0'>Go back</a></p></body></html>");
        }
    });

    server.on("/clear", HTTP_POST, [](AsyncWebServerRequest *request) {
        lastConfigActivity = millis();
        targetFilters.clear();
        nameFilters.clear();
        saveConfiguration();
        request->send(200, "text/plain", "Filters cleared");
    });

    server.on("/api/devices", HTTP_GET, [](AsyncWebServerRequest *request) {
        lastConfigActivity = millis();
        String json = "{\"devices\":[";
        unsigned long now = millis();

        for (size_t i = 0; i < devices.size(); i++) {
            if (i > 0) json += ",";
            unsigned long timeSince = (now >= devices[i].lastSeen) ? (now - devices[i].lastSeen) : 0;
            json += "{\"mac\":\"" + devices[i].macAddress + "\",";
            json += "\"rssi\":" + String(devices[i].rssi) + ",";
            json += "\"filter\":\"" + devices[i].filterDescription + "\",";
            json += "\"timeSince\":" + String(timeSince) + "}";
        }
        json += "]}";
        request->send(200, "application/json", json);
    });

    server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request) {
        String json = "{\"mode\":\"" + String(currentMode == CONFIG_MODE ? "config" : "scanning") + "\",";
        json += "\"filters\":" + String(targetFilters.size()) + ",";
        json += "\"names\":" + String(nameFilters.size()) + ",";
        json += "\"devices\":" + String(devices.size()) + ",";
        json += "\"meshtastic\":true}";
        request->send(200, "application/json", json);
    });

    server.on("/api/clear-devices", HTTP_POST, [](AsyncWebServerRequest *request) {
        lastConfigActivity = millis();
        clearDetectedDevices();
        request->send(200, "application/json", "{\"success\":true}");
    });

    // Burn-in / lock configuration
    server.on("/api/lock-config", HTTP_POST, [](AsyncWebServerRequest *request) {
        lastConfigActivity = millis();
        processFormSubmission(request);

        preferences.begin("meshdetect", false);
        preferences.putBool("locked", true);
        preferences.end();

        Serial.println("[CONFIG] *** CONFIGURATION LOCKED ***");

        String resp = R"(<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>body{background:#000;color:#f80;font-family:monospace;text-align:center;padding:40px}
.box{border:2px solid #f80;padding:30px;max-width:500px;margin:0 auto}
h1{font-size:18px;margin-bottom:15px;color:#f44}
p{line-height:1.6;margin:10px 0}
</style><script>setTimeout(function(){window.location.href='about:blank';},3000);</script>
</head><body><div class="box">
<h1>CONFIGURATION LOCKED</h1>
<p>Settings permanently saved. WiFi AP disabled on future boots.</p>
<p>Device will restart and begin scanning in 3 seconds...</p>
<p style="font-size:11px;color:#888;margin-top:20px">Unlock: flash erase + firmware reflash required</p>
</div></body></html>)";

        request->send(200, "text/html", resp);
        modeSwitchScheduled = millis() + 3000;
    });

    // Return to menu (reboot to config)
    server.on("/menu", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/plain", "Rebooting to config...");
        delay(500);
        ESP.restart();
    });

    server.begin();
    Serial.println("[CONFIG] Web server started");
}

// ================================
// Start Scanning Mode (BLE + WiFi Promiscuous)
// ================================
static void startScanningMode() {
    currentMode = SCANNING_MODE;
    modeSwitchScheduled = 0;

    // Stop web server and AP
    server.end();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(500);

    Serial.println("\n========================================");
    Serial.println("  MESH-DETECT - Scanning Mode");
    Serial.printf("  Filters: %d MAC/OUI, %d names\n",
        targetFilters.size(), nameFilters.size());
    Serial.println("  Meshtastic: Serial1 TX=GPIO4 @ 115200");
    Serial.println("========================================\n");

    for (const TargetFilter& f : targetFilters) {
        Serial.printf("  [%s] %s\n", f.isFullMAC ? "MAC" : "OUI", f.identifier.c_str());
    }
    for (const String& n : nameFilters) {
        Serial.printf("  [NAME] %s\n", n.c_str());
    }

    // Initialize BLE
    NimBLEDevice::init("");
    delay(500);

    NimBLEScan* pScan = NimBLEDevice::getScan();
    pScan->setScanCallbacks(new ScanCallbacks());
    pScan->setActiveScan(true);
    pScan->setInterval(BLE_SCAN_INTERVAL);
    pScan->setWindow(BLE_SCAN_WINDOW);

    // Start WiFi promiscuous mode for WiFi OUI detection
    WiFi.mode(WIFI_STA);
    delay(200);
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(&wifiSnifferCallback);
    esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);
    currentWiFiChannel = 6;
    lastChannelHop = millis();

    Serial.println("[SCAN] WiFi promiscuous mode enabled (channel hopping 1-13)");

    // Start BLE continuous scan
    delay(500);
    if (pScan->start(0)) {
        Serial.println("[SCAN] BLE continuous scan started");
    } else {
        Serial.println("[SCAN] ERROR: BLE scan failed to start!");
    }

    // Green flash to indicate scan start
    strip.setPixelColor(0, strip.Color(0, NEOPIXEL_DETECTION_BRIGHTNESS, 0));
    strip.show();
    delay(300);
    strip.clear();
    strip.show();

    Serial.println("[SCAN] *** SCANNING ACTIVE ***\n");
}

// ================================
// Boot Button Detection (GPIO0)
// ================================
#define BOOT_BUTTON_PIN 0
#define BOOT_HOLD_TIME  1500

static bool checkBootButton() {
    pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
    if (digitalRead(BOOT_BUTTON_PIN) == HIGH) return false;

    Serial.println("[BOOT] Button pressed - hold to force config mode...");
    unsigned long start = millis();
    while (millis() - start < BOOT_HOLD_TIME) {
        if (digitalRead(BOOT_BUTTON_PIN) == HIGH) return false;
        delay(10);
    }

    Serial.println("[BOOT] *** BUTTON HELD *** -> Forcing config mode");
    return true;
}

// Boot button check in loop (hold to reboot to config)
static unsigned long bootBtnStart = 0;
static bool bootBtnActive = false;

static void checkBootButtonLoop() {
    if (digitalRead(BOOT_BUTTON_PIN) == LOW) {
        if (!bootBtnActive) {
            bootBtnActive = true;
            bootBtnStart = millis();
        } else if (millis() - bootBtnStart >= BOOT_HOLD_TIME) {
            Serial.println("\n[BOOT] *** BUTTON HELD -> REBOOTING TO CONFIG ***");
            // Flash LED rapidly to confirm
            for (int i = 0; i < 5; i++) {
                ledOn(); delay(50);
                ledOff(); delay(50);
            }
            delay(200);
            ESP.restart();
        }
    } else {
        bootBtnActive = false;
    }
}

// ================================
// Setup
// ================================
void setup() {
    delay(2000);

    // Initialize serial ports
    initializeSerial();

    Serial.println("\n========================================");
    Serial.println("  MESH-DETECT v1.0");
    Serial.println("  BLE/WiFi Detection + Meshtastic");
    Serial.println("  colonelpanichacks");
    Serial.println("========================================\n");

    // Check boot button
    bool forceConfig = checkBootButton();

    // Initialize hardware
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH); // LED off

    // WiFi factory reset (clear stale AP config)
    WiFi.mode(WIFI_AP_STA);
    delay(100);
    esp_wifi_restore();
    WiFi.softAPdisconnect(true);
    WiFi.disconnect(true, true);
    WiFi.mode(WIFI_OFF);
    delay(100);

    // Silence IDF logs
    esp_log_level_set("*", ESP_LOG_NONE);

    // Initialize NeoPixel
    initializeNeoPixel();
    strip.setPixelColor(0, strip.Color(0, NEOPIXEL_BRIGHTNESS, 0)); // Green boot flash
    strip.show();
    delay(500);
    strip.clear();
    strip.show();

    // Load config from NVS
    loadAPConfig();
    loadConfiguration();
    loadDetectedDevices();

    // Check if config is locked
    preferences.begin("meshdetect", true);
    configLocked = preferences.getBool("locked", false);
    bool factoryReset = preferences.getBool("factoryReset", false);
    preferences.end();

    if (factoryReset) {
        Serial.println("[BOOT] Factory reset flag detected - clearing all data");
        preferences.begin("meshdetect", false);
        preferences.clear();
        preferences.end();
        targetFilters.clear();
        nameFilters.clear();
        devices.clear();
        configLocked = false;
    }

    if (forceConfig) {
        Serial.println("[BOOT] Boot button override -> config mode");
        startConfigMode();
    } else if (configLocked && (targetFilters.size() > 0 || nameFilters.size() > 0)) {
        Serial.println("[BOOT] Config locked -> scanning mode");
        startScanningMode();
    } else {
        Serial.println("[BOOT] Starting config mode");
        startConfigMode();
    }

    Serial.println("[BOOT] Setup complete\n");
}

// ================================
// Loop
// ================================
void loop() {
    unsigned long now = millis();

    // Always check boot button
    checkBootButtonLoop();

    if (currentMode == CONFIG_MODE) {
        // Check scheduled mode switch
        if (modeSwitchScheduled > 0 && now >= modeSwitchScheduled) {
            startScanningMode();
            return;
        }

        // Auto-timeout: if filters exist and no web activity within timeout
        if ((targetFilters.size() > 0 || nameFilters.size() > 0) &&
            now - configStartTime > CONFIG_TIMEOUT &&
            lastConfigActivity == configStartTime) {
            Serial.println("[CONFIG] Timeout with saved filters -> scanning mode");
            startScanningMode();
            return;
        }

        // LED blink in config mode
        static unsigned long lastLed = 0;
        static bool ledState = false;
        if (now - lastLed > 1000) {
            ledState = !ledState;
            digitalWrite(LED_PIN, ledState ? LOW : HIGH);
            lastLed = now;
        }

        delay(10);
    }

    if (currentMode == SCANNING_MODE) {
        // WiFi channel hopping
        if (now - lastChannelHop >= WIFI_CHANNEL_HOP_MS) {
            currentWiFiChannel = (currentWiFiChannel % 13) + 1;
            esp_wifi_set_channel(currentWiFiChannel, WIFI_SECOND_CHAN_NONE);
            lastChannelHop = now;
        }

        // Auto-save devices periodically
        if (now - lastDeviceSave >= DEVICE_SAVE_INTERVAL) {
            saveDetectedDevices();
            lastDeviceSave = now;
        }

        // Turn off onboard LED after detection flash (500ms)
        static unsigned long ledOnTime = 0;
        if (digitalRead(LED_PIN) == LOW) {
            if (ledOnTime == 0) ledOnTime = now;
            if (now - ledOnTime > 500) {
                ledOff();
                ledOnTime = 0;
            }
        } else {
            ledOnTime = 0;
        }

        delay(10);
    }

    // Always update NeoPixel
    updateNeoPixelAnimation();
}
