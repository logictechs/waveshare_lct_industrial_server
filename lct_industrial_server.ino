/*
  LCT Industrial Server
  Waveshare ESP32-S3-POE-ETH-8DI-8DO
  Single-file Arduino IDE sketch.

  Implemented:
  - Initial AP: ESP32-XXXX, password LCT-XXXX
  - WiFi AP / STA configuration
  - W5500 Ethernet configuration
  - Modbus TCP server on Ethernet port 502
  - Modbus TCP host/client polling
  - Modbus RTU slave/server on RS485
  - Modbus RTU master polling
  - SD file browser with upload, download, delete
  - CSV IO logging using PCF85063 RTC timestamps
  - Web OTA firmware update
  - MQTT status publishing and command subscription
  - JSON REST API
  - WebSocket live IO updates on port 81
  - NTP to RTC synchronization
  - Persistent IO naming/configuration
  - TCA9554PWR DO1-DO8 direct driver, no Waveshare external library
  - PCF85063 RTC direct driver, no Waveshare external library

  Required Arduino libraries:
  - ESP32 board package 3.x
  - Ethernet by Arduino
  - WebSockets by Markus Sattler
  - PubSubClient by Nick O'Leary

  Board pin summary:
  DI1-DI8: GPIO4-GPIO11
  DO1-DO8: TCA9554PWR EXIO1-EXIO8 over I2C
  I2C SDA: GPIO42
  I2C SCL: GPIO41
  RTC INT: GPIO40
  W5500 INT GPIO12, MOSI GPIO13, MISO GPIO14, SCLK GPIO15, CS GPIO16, RST GPIO39
  RS485 TX GPIO17, RX GPIO18, RTS/DE GPIO21
  SD_MMC CLK GPIO48, CMD GPIO47, D0 GPIO45
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Wire.h>
#include <SD_MMC.h>
#include <FS.h>
#include <SPI.h>
#include <Ethernet.h>
#include <Dns.h>
#include <WebSocketsServer.h>
#include <PubSubClient.h>
#include <Update.h>
#include <EthernetUdp.h>
#include <time.h>
#include <esp_mac.h>
#include "driver/twai.h"

// ---------------- Board Pins ----------------

#define I2C_SDA       42
#define I2C_SCL       41
#define RTC_INT       40

#define W5500_INT     12
#define W5500_MOSI    13
#define W5500_MISO    14
#define W5500_SCLK    15
#define W5500_CS      16
#define W5500_RST     39

#define RS485_TX      17
#define RS485_RX      18
#define RS485_RTS     21

#define SD_D0         45
#define SD_CMD        47
#define SD_CLK        48

#define BUZZER_PIN    46
#define RGB_CTRL      38

const int DI_PINS[8] = {4, 5, 6, 7, 8, 9, 10, 11};

// ---------------- TCA9554PWR ----------------

#define TCA9554_ADDR        0x20
#define TCA_INPUT_REG       0x00
#define TCA_OUTPUT_REG      0x01
#define TCA_POLARITY_REG    0x02
#define TCA_CONFIG_REG      0x03

// ---------------- PCF85063 RTC ----------------

#define PCF85063_ADDR       0x51
#define RTC_CTRL_1_ADDR     0x00
#define RTC_CTRL_2_ADDR     0x01
#define RTC_SECOND_ADDR     0x04
#define RTC_MINUTE_ADDR     0x05
#define RTC_HOUR_ADDR       0x06
#define RTC_DAY_ADDR        0x07
#define RTC_WDAY_ADDR       0x08
#define RTC_MONTH_ADDR      0x09
#define RTC_YEAR_ADDR       0x0A

typedef struct {
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t weekday;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
} LCT_DateTime;

// ---------------- Global Services ----------------

WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);
Preferences prefs;
HardwareSerial RS485Serial(1);

WiFiClient wifiNetClient;
EthernetClient mqttEthClient;
PubSubClient mqttClient(wifiNetClient);
EthernetUDP ethUdp;

EthernetServer ethHttpServer(80);
EthernetServer modbusTcpServer(502);
EthernetClient modbusTcpClient;

bool ethernetStarted = false;
bool modbusTcpStarted = false;

byte ethMacAddr[6] = {0};

unsigned long lastWifiStaRetry = 0;
unsigned long lastWifiIndicatorBlink = 0;
bool wifiIndicatorBlinkState = false;
const uint32_t WIFI_STA_RETRY_MS = 300000UL;
const uint32_t WIFI_INDICATOR_BLINK_MS = 1000UL;

// ---------------- Persistent Config ----------------

String wifiMode;
String staSsid;
String staPass;
bool wifiIndicatorEnable;

bool diInvertLogic;
bool doInvertLogic;

String ethMode;
String ethIP;
String ethGW;
String ethMask;
String ethDNS;

bool ethModbusServerEnable;
bool ethModbusHostEnable;
String mbTcpHostIP;
uint16_t mbTcpHostPort;
uint8_t mbTcpHostUnit;
uint16_t mbTcpHostReg;
uint16_t mbTcpHostCount;
uint32_t mbTcpPollMs;

bool rs485ModbusSlaveEnable;
bool rs485ModbusMasterEnable;
uint32_t rs485Baud;
uint8_t rs485Id;
uint8_t rtuMasterId;
uint16_t rtuMasterReg;
uint16_t rtuMasterCount;
uint32_t rtuPollMs;

bool mqttEnable;
String mqttIface;
String mqttHost;
uint16_t mqttPort;
String mqttUser;
String mqttPass;
String mqttBaseTopic;

bool ntpEnable;
String ntpIface;
String ntpServer;
long gmtOffsetSec;
int daylightOffsetSec;

bool csvLogEnable;
uint32_t csvLogMs;

bool mbCsvLogEnable;
uint32_t mbCsvLogMs;

bool ioEventLogEnable;

String inputNames[8];
String outputNames[8];

bool canEnable;
int canTxPin;
int canRxPin;
uint32_t canBitrate;
bool canListenOnly;

// ---------------- Runtime State ----------------

uint8_t tcaOutputState = 0x00;
bool tcaOK = false;
bool rtcOK = false;
bool sdOK = false;
bool outState[8] = {false, false, false, false, false, false, false, false};
bool inState[8]  = {false, false, false, false, false, false, false, false};

uint16_t holdingRegs[128];

#define MB_MON_MAX 40
uint16_t mbMonAddr[MB_MON_MAX];
String mbMonDesc[MB_MON_MAX];
String mbMonFormula[MB_MON_MAX];
uint8_t mbMonCount = 0;

#define MB_REMOTE_MAX 125
uint16_t mbRemoteStart = 0;
uint16_t mbRemoteCount = 0;
uint16_t mbRemoteRegs[MB_REMOTE_MAX];
bool mbRemoteValid[MB_REMOTE_MAX];

unsigned long lastWebSocketPush = 0;
unsigned long lastCsvLog = 0;
unsigned long lastMbCsvLog = 0;

bool ioEventInitialized = false;
bool ioEventState[16];
unsigned long ioEventStartMillis[16];
String ioEventStartTime[16];
uint32_t ioEventOnSeconds[16];
uint32_t ioEventOffSeconds[16];
String ioEventCurrentDate = "";
unsigned long lastMqttPublish = 0;
unsigned long lastMqttReconnect = 0;
unsigned long lastNtpSync = 0;
unsigned long lastTcpHostPoll = 0;
unsigned long lastRtuMasterPoll = 0;

bool canStarted = false;
uint32_t canRxCount = 0;
uint32_t canTxCount = 0;
uint32_t canErrCount = 0;
String canLastFrame = "";

#define CAN_RX_HISTORY_MAX 20
String canRxHistory[CAN_RX_HISTORY_MAX];
uint8_t canRxHistoryIndex = 0;
uint8_t canRxHistoryCount = 0;

String activeMqttIface = "";

// ---------------- Utility ----------------

uint8_t decToBcd(uint8_t val) { return ((val / 10) << 4) | (val % 10); }
uint8_t bcdToDec(uint8_t val) { return ((val >> 4) * 10) + (val & 0x0F); }

String chipSuffix() {
  /*
    Use factory WiFi STA MAC bytes in normal display order.
    Example:
      STA MAC E0:72:A1:F1:D0:10 -> suffix D010
  */
  uint8_t mac[6] = {0};
  esp_read_mac(mac, ESP_MAC_WIFI_STA);

  char buf[5];
  snprintf(buf, sizeof(buf), "%02X%02X", mac[4], mac[5]);
  return String(buf);
}

String apName() {
  return "ESP32-" + chipSuffix();
}

String apPassword() {
  return "LCT-" + chipSuffix();
}


bool wifiApRunning() {
  wifi_mode_t mode = WiFi.getMode();
  return mode == WIFI_AP || mode == WIFI_AP_STA;
}

String wifiApIpString() {
  return wifiApRunning() ? WiFi.softAPIP().toString() : String("Disabled");
}


String esc(String v) {
  v.replace("&", "&amp;");
  v.replace("<", "&lt;");
  v.replace(">", "&gt;");
  v.replace("\"", "&quot;");
  v.replace("'", "&#39;");
  return v;
}

String urlDecode(String input) {
  String s = input;
  s.replace("+", " ");
  String out = "";
  for (uint16_t i = 0; i < s.length(); i++) {
    if (s[i] == '%' && i + 2 < s.length()) {
      char h[3] = {s[i + 1], s[i + 2], 0};
      out += char(strtol(h, NULL, 16));
      i += 2;
    } else {
      out += s[i];
    }
  }
  return out;
}

IPAddress parseIP(String s, IPAddress fallback) {
  IPAddress ip;
  if (ip.fromString(s)) return ip;
  return fallback;
}

String ipToString(IPAddress ip) {
  return String(ip[0]) + "." + String(ip[1]) + "." + String(ip[2]) + "." + String(ip[3]);
}

String macToString(const uint8_t *mac) {
  char buf[18];
  snprintf(
    buf,
    sizeof(buf),
    "%02X:%02X:%02X:%02X:%02X:%02X",
    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]
  );
  return String(buf);
}

String wifiApMacString() {
  return WiFi.softAPmacAddress();
}

String wifiStaMacString() {
  String m = WiFi.macAddress();
  if (m == "00:00:00:00:00:00") {
    return "Not active";
  }
  return m;
}

String ethMacString() {
  return macToString(ethMacAddr);
}

String htmlStart(const String &title) {
  String s;
  s += "<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>";
  s += "<title>" + esc(title) + "</title>";
  s += "<style>";
  s += "body{font-family:Arial;background:#111;color:#eee;margin:0;padding:18px;}";
  s += ".card{background:#1d1d1d;border:1px solid #444;border-radius:12px;padding:16px;margin:12px 0;}";
  s += "a,button,input,select{font-size:16px;margin:4px;padding:9px;border-radius:8px;}";
  s += "a.btn,button{background:#ff9900;color:#000;text-decoration:none;border:0;font-weight:bold;display:inline-block;}";
  s += "input,select{background:#222;color:#fff;border:1px solid #666;max-width:95%;}";
  s += "table{border-collapse:collapse;width:100%;margin-top:8px;}";
  s += "td,th{border:1px solid #555;padding:8px;text-align:left;}";
  s += ".on{color:#55ff55;font-weight:bold;}.off{color:#ff5555;font-weight:bold;}.warn{color:#ffff66;font-weight:bold;}";
  s += "</style></head><body>";
  s += "<h2>" + esc(title) + "</h2><p>";
  s += "<a class='btn' href='/'>Home</a><a class='btn' href='/wifi'>WiFi</a><a class='btn' href='/ethernet'>Ethernet</a>";
  s += "<a class='btn' href='/rs485'>RS485</a><a class='btn' href='/sd'>SD</a><a class='btn' href='/io'>IO</a><a class='btn' href='/can'>CAN</a>";
  s += "<a class='btn' href='/mqtt'>MQTT</a><a class='btn' href='/time'>Time/NTP</a><a class='btn' href='/ota'>WiFi OTA</a>";
  s += "<a class='btn' href='/api/status'>JSON</a></p>";
  return s;
}

String htmlEnd() { return "</body></html>"; }

// ---------------- I2C / TCA / RTC ----------------

bool i2cWriteReg(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool i2cReadReg(uint8_t addr, uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, (uint8_t)1) != 1) return false;
  value = Wire.read();
  return true;
}

bool tca9554Init(uint8_t pinModeMask = 0x00, uint8_t outputState = 0x00) {
  if (!i2cWriteReg(TCA9554_ADDR, TCA_CONFIG_REG, pinModeMask)) return false;
  if (!i2cWriteReg(TCA9554_ADDR, TCA_POLARITY_REG, 0x00)) return false;
  tcaOutputState = outputState;
  return i2cWriteReg(TCA9554_ADDR, TCA_OUTPUT_REG, tcaOutputState);
}

bool setEXIO(uint8_t pin, bool level) {
  if (pin < 1 || pin > 8) return false;
  uint8_t bit = pin - 1;
  if (level) tcaOutputState |= (1 << bit);
  else tcaOutputState &= ~(1 << bit);
  return i2cWriteReg(TCA9554_ADDR, TCA_OUTPUT_REG, tcaOutputState);
}

bool readEXIOOutput(uint8_t pin, bool &level) {
  if (pin < 1 || pin > 8) return false;
  uint8_t value = 0;
  if (!i2cReadReg(TCA9554_ADDR, TCA_OUTPUT_REG, value)) return false;
  level = value & (1 << (pin - 1));
  return true;
}

bool pcf85063Init() {
  bool ok1 = i2cWriteReg(PCF85063_ADDR, RTC_CTRL_1_ADDR, 0x00);
  bool ok2 = i2cWriteReg(PCF85063_ADDR, RTC_CTRL_2_ADDR, 0x00);
  return ok1 && ok2;
}

bool pcf85063ReadTime(LCT_DateTime &dt) {
  Wire.beginTransmission(PCF85063_ADDR);
  Wire.write(RTC_SECOND_ADDR);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(PCF85063_ADDR, (uint8_t)7) != 7) return false;

  uint8_t secRaw = Wire.read();
  uint8_t minRaw = Wire.read();
  uint8_t hourRaw = Wire.read();
  uint8_t dayRaw = Wire.read();
  uint8_t wdayRaw = Wire.read();
  uint8_t monthRaw = Wire.read();
  uint8_t yearRaw = Wire.read();

  dt.second = bcdToDec(secRaw & 0x7F);
  dt.minute = bcdToDec(minRaw & 0x7F);
  dt.hour = bcdToDec(hourRaw & 0x3F);
  dt.day = bcdToDec(dayRaw & 0x3F);
  dt.weekday = bcdToDec(wdayRaw & 0x07);
  dt.month = bcdToDec(monthRaw & 0x1F);
  dt.year = 1970 + bcdToDec(yearRaw);
  return true;
}

bool pcf85063SetTime(const LCT_DateTime &dt) {
  uint8_t yearOffset = dt.year >= 1970 ? dt.year - 1970 : 0;
  Wire.beginTransmission(PCF85063_ADDR);
  Wire.write(RTC_SECOND_ADDR);
  Wire.write(decToBcd(dt.second));
  Wire.write(decToBcd(dt.minute));
  Wire.write(decToBcd(dt.hour));
  Wire.write(decToBcd(dt.day));
  Wire.write(decToBcd(dt.weekday));
  Wire.write(decToBcd(dt.month));
  Wire.write(decToBcd(yearOffset));
  return Wire.endTransmission() == 0;
}

String rtcString() {
  LCT_DateTime now;
  if (!pcf85063ReadTime(now)) return "RTC read failed";
  char buf[32];
  snprintf(buf, sizeof(buf), "%04u-%02u-%02u %02u:%02u:%02u", now.year, now.month, now.day, now.hour, now.minute, now.second);
  return String(buf);
}

bool syncRtcFromWiFiNtp() {
  if (WiFi.status() != WL_CONNECTED) return false;
  configTime(gmtOffsetSec, daylightOffsetSec, ntpServer.c_str());
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 10000)) return false;

  LCT_DateTime dt;
  dt.year = timeinfo.tm_year + 1900;
  dt.month = timeinfo.tm_mon + 1;
  dt.day = timeinfo.tm_mday;
  dt.weekday = timeinfo.tm_wday;
  dt.hour = timeinfo.tm_hour;
  dt.minute = timeinfo.tm_min;
  dt.second = timeinfo.tm_sec;
  return pcf85063SetTime(dt);
}

bool syncRtcFromEthernetNtp() {
  if (!ethernetStarted) return false;

  IPAddress ntpIP;
  DNSClient dns;
  IPAddress dnsServer = parseIP(ethDNS, IPAddress(8, 8, 8, 8));
  dns.begin(dnsServer);
  if (dns.getHostByName(ntpServer.c_str(), ntpIP) != 1) return false;

  const int NTP_PACKET_SIZE = 48;
  uint8_t packetBuffer[NTP_PACKET_SIZE];
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  packetBuffer[0] = 0b11100011;
  packetBuffer[1] = 0;
  packetBuffer[2] = 6;
  packetBuffer[3] = 0xEC;
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;

  ethUdp.stop();
  if (!ethUdp.begin(2390)) return false;
  ethUdp.beginPacket(ntpIP, 123);
  ethUdp.write(packetBuffer, NTP_PACKET_SIZE);
  ethUdp.endPacket();

  unsigned long start = millis();
  while (millis() - start < 2000) {
    int size = ethUdp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      ethUdp.read(packetBuffer, NTP_PACKET_SIZE);
      uint32_t highWord = word(packetBuffer[40], packetBuffer[41]);
      uint32_t lowWord = word(packetBuffer[42], packetBuffer[43]);
      uint32_t secsSince1900 = (highWord << 16) | lowWord;
      const uint32_t seventyYears = 2208988800UL;
      time_t unixTime = (time_t)(secsSince1900 - seventyYears + gmtOffsetSec + daylightOffsetSec);

      struct tm timeinfo;
      gmtime_r(&unixTime, &timeinfo);

      LCT_DateTime dt;
      dt.year = timeinfo.tm_year + 1900;
      dt.month = timeinfo.tm_mon + 1;
      dt.day = timeinfo.tm_mday;
      dt.weekday = timeinfo.tm_wday;
      dt.hour = timeinfo.tm_hour;
      dt.minute = timeinfo.tm_min;
      dt.second = timeinfo.tm_sec;
      return pcf85063SetTime(dt);
    }
    delay(10);
  }

  return false;
}

bool syncRtcFromNtp() {
  if (ntpIface == "WIFI") return syncRtcFromWiFiNtp();
  if (ntpIface == "ETH") return syncRtcFromEthernetNtp();

  // AUTO: prefer WiFi STA when connected, otherwise try W5500 Ethernet.
  if (WiFi.status() == WL_CONNECTED && syncRtcFromWiFiNtp()) return true;
  if (ethernetStarted && syncRtcFromEthernetNtp()) return true;
  return false;
}


// ---------------- Modbus Monitor Table / Formula Support ----------------
//
// Formula syntax:
//   x
//   x*1.8+32
//   x/10
//   (x-4000)*0.01
// Supports: x, numbers, +, -, *, /, and parentheses.

class LCTFormulaParser {
public:
  String s;
  int p;
  double x;

  LCTFormulaParser(String expr, double raw) {
    s = expr;
    p = 0;
    x = raw;
    s.replace(" ", "");
  }

  char peek() {
    if (p >= (int)s.length()) return '\0';
    return s[p];
  }

  char get() {
    if (p >= (int)s.length()) return '\0';
    return s[p++];
  }

  double parseNumber() {
    int start = p;
    if (peek() == '-') p++;
    while (isDigit(peek()) || peek() == '.') p++;
    return s.substring(start, p).toDouble();
  }

  double parseFactor() {
    char c = peek();

    if (c == '+') {
      get();
      return parseFactor();
    }

    if (c == '-') {
      get();
      return -parseFactor();
    }

    if (c == '(') {
      get();
      double v = parseExpression();
      if (peek() == ')') get();
      return v;
    }

    if (c == 'x' || c == 'X') {
      get();
      return x;
    }

    if (isDigit(c) || c == '.') {
      return parseNumber();
    }

    return 0.0;
  }

  double parseTerm() {
    double v = parseFactor();

    while (true) {
      char c = peek();

      if (c == '*') {
        get();
        v *= parseFactor();
      } else if (c == '/') {
        get();
        double d = parseFactor();
        if (d != 0.0) v /= d;
      } else {
        break;
      }
    }

    return v;
  }

  double parseExpression() {
    double v = parseTerm();

    while (true) {
      char c = peek();

      if (c == '+') {
        get();
        v += parseTerm();
      } else if (c == '-') {
        get();
        v -= parseTerm();
      } else {
        break;
      }
    }

    return v;
  }
};

double computeFormula(String formula, double raw) {
  formula.trim();
  if (formula.length() == 0) return raw;
  LCTFormulaParser parser(formula, raw);
  return parser.parseExpression();
}

String fmtDouble(double v) {
  char buf[32];
  snprintf(buf, sizeof(buf), "%.4f", v);
  String s = String(buf);
  while (s.endsWith("0")) s.remove(s.length() - 1);
  if (s.endsWith(".")) s.remove(s.length() - 1);
  return s;
}

void loadModbusMonitorConfig() {
  prefs.begin("mbmon", false);

  mbMonCount = prefs.getUChar("count", 0);
  if (mbMonCount > MB_MON_MAX) mbMonCount = MB_MON_MAX;

  for (int i = 0; i < MB_MON_MAX; i++) {
    mbMonAddr[i] = prefs.getUShort(("addr" + String(i)).c_str(), i);
    mbMonDesc[i] = prefs.getString(("desc" + String(i)).c_str(), "");
    mbMonFormula[i] = prefs.getString(("form" + String(i)).c_str(), "x");
  }

  if (mbMonCount == 0) {
    mbMonCount = 8;
    mbMonAddr[0] = 0; mbMonDesc[0] = "DI bitmask"; mbMonFormula[0] = "x";
    mbMonAddr[1] = 1; mbMonDesc[1] = "DO bitmask"; mbMonFormula[1] = "x";
    mbMonAddr[2] = 2; mbMonDesc[2] = "TCA9554 OK"; mbMonFormula[2] = "x";
    mbMonAddr[3] = 3; mbMonDesc[3] = "RTC OK"; mbMonFormula[3] = "x";
    mbMonAddr[4] = 4; mbMonDesc[4] = "SD mounted"; mbMonFormula[4] = "x";
    mbMonAddr[5] = 5; mbMonDesc[5] = "WiFi OK"; mbMonFormula[5] = "x";
    mbMonAddr[6] = 6; mbMonDesc[6] = "Ethernet OK"; mbMonFormula[6] = "x";
    mbMonAddr[7] = 7; mbMonDesc[7] = "Free heap low word"; mbMonFormula[7] = "x";
  }

  prefs.end();
}

void saveModbusMonitorConfig() {
  prefs.begin("mbmon", false);
  prefs.putUChar("count", mbMonCount);

  for (int i = 0; i < MB_MON_MAX; i++) {
    prefs.putUShort(("addr" + String(i)).c_str(), mbMonAddr[i]);
    prefs.putString(("desc" + String(i)).c_str(), mbMonDesc[i]);
    prefs.putString(("form" + String(i)).c_str(), mbMonFormula[i]);
  }

  prefs.end();
}

void addModbusMonitorRegister(uint16_t addr, String desc, String formula) {
  if (mbMonCount >= MB_MON_MAX) return;

  desc.trim();
  formula.trim();
  if (formula.length() == 0) formula = "x";

  mbMonAddr[mbMonCount] = addr;
  mbMonDesc[mbMonCount] = desc;
  mbMonFormula[mbMonCount] = formula;
  mbMonCount++;

  saveModbusMonitorConfig();
}

void deleteModbusMonitorRegister(uint8_t index) {
  if (index >= mbMonCount) return;

  for (int i = index; i < mbMonCount - 1; i++) {
    mbMonAddr[i] = mbMonAddr[i + 1];
    mbMonDesc[i] = mbMonDesc[i + 1];
    mbMonFormula[i] = mbMonFormula[i + 1];
  }

  mbMonCount--;
  saveModbusMonitorConfig();
}

void deleteAllModbusMonitorRegisters() {
  mbMonCount = 0;
  for (int i = 0; i < MB_MON_MAX; i++) {
    mbMonAddr[i] = 0;
    mbMonDesc[i] = "";
    mbMonFormula[i] = "x";
  }
  saveModbusMonitorConfig();
}


uint16_t getModbusMonitorRawValue(uint16_t displayAddr) {
  // First preference: values returned by Modbus TCP/RTU host polling.
  // If your device documentation lists 40002 and the host start register is 40000,
  // the code maps 40002 to polled offset 2.
  if (mbRemoteCount > 0 && displayAddr >= mbRemoteStart) {
    uint16_t idx = displayAddr - mbRemoteStart;
    if (idx < mbRemoteCount && idx < MB_REMOTE_MAX && mbRemoteValid[idx]) {
      return mbRemoteRegs[idx];
    }
  }

  // Common Modbus notation: 40001 means holding-register offset 0.
  if (displayAddr >= 40001 && mbRemoteCount > 0) {
    uint16_t idx = displayAddr - 40001;
    if (idx < mbRemoteCount && idx < MB_REMOTE_MAX && mbRemoteValid[idx]) {
      return mbRemoteRegs[idx];
    }
  }

  // Local internal status registers.
  if (displayAddr < 128) {
    return holdingRegs[displayAddr];
  }

  return 0;
}


String modbusAutoRefreshScript() {
  uint32_t refreshMs = mbTcpPollMs;

  if (refreshMs < 1000) refreshMs = 1000;

  // Add a small delay so the Modbus poll has time to complete before browser refresh.
  refreshMs += 750;

  String s;
  s += "<script>";
  s += "let mbRefreshMs=" + String(refreshMs) + ";";
  s += "let mbLeft=Math.ceil(mbRefreshMs/1000);";
  s += "function mbTick(){";
  s += "  let el=document.getElementById('mb_refresh_countdown');";
  s += "  if(el){el.textContent=mbLeft;}";
  s += "  mbLeft--;";
  s += "  if(mbLeft<0){location.reload();}";
  s += "}";
  s += "setInterval(mbTick,1000);";
  s += "mbTick();";
  s += "</script>";
  return s;
}

String modbusRefreshInfoHtml() {
  uint32_t refreshMs = mbTcpPollMs;
  if (refreshMs < 1000) refreshMs = 1000;
  refreshMs += 750;

  String s;
  s += "<div class='card'>";
  s += "<b>Auto Refresh:</b> enabled<br>";
  s += "<b>Configured Modbus TCP poll interval:</b> " + String(mbTcpPollMs) + " ms<br>";
  s += "<b>Page refresh interval:</b> " + String(refreshMs) + " ms<br>";
  s += "Refreshing in <b><span id='mb_refresh_countdown'></span></b> seconds.";
  s += "</div>";
  s += modbusAutoRefreshScript();
  return s;
}

String modbusMonitorTableHtml(bool includeActions) {
  refreshInputsOutputs();

  String s;

  if (includeActions) {
    s += "<form method='post' action='/mbmon/delete_selected'>";
    s += "<button name='action' value='delete_selected' onclick='return confirm(\"Delete selected Modbus monitor rows?\")'>Delete Selected</button>";
    s += "<button name='action' value='delete_all' onclick='return confirm(\"Delete ALL Modbus monitor rows?\")'>Delete All</button>";
  }

  s += "<table>";
  s += "<tr>";
  if (includeActions) s += "<th>Select</th>";
  s += "<th>#</th><th>Register Address</th><th>Description</th><th>Raw Value</th><th>Math Formula</th><th>Computed Value</th>";
  if (includeActions) s += "<th>Quick Action</th>";
  s += "</tr>";

  for (int i = 0; i < mbMonCount; i++) {
    uint16_t addr = mbMonAddr[i];
    uint16_t raw = getModbusMonitorRawValue(addr);
    double computed = computeFormula(mbMonFormula[i], raw);

    s += "<tr>";
    if (includeActions) s += "<td><input type='checkbox' name='sel" + String(i) + "' value='1'></td>";
    s += "<td>" + String(i + 1) + "</td>";
    s += "<td>" + String(addr) + "</td>";
    s += "<td>" + esc(mbMonDesc[i]) + "</td>";
    s += "<td>" + String(raw) + "</td>";
    s += "<td>" + esc(mbMonFormula[i]) + "</td>";
    s += "<td>" + fmtDouble(computed) + "</td>";

    if (includeActions) {
      s += "<td><a class='btn' href='/mbmon/delete?i=" + String(i) + "' onclick='return confirm(\"Remove this register?\")'>Delete</a></td>";
    }

    s += "</tr>";
  }

  if (mbMonCount == 0) {
    s += "<tr><td colspan='" + String(includeActions ? 8 : 6) + "'>No Modbus monitor registers configured.</td></tr>";
  }

  s += "</table>";

  if (includeActions) {
    s += "</form>";
  }

  return s;
}

// ---------------- Config ----------------

void loadConfig() {
  prefs.begin("cfg", false);

  wifiMode = prefs.getString("wifiMode", "AP");
  staSsid = prefs.getString("staSsid", "");
  staPass = prefs.getString("staPass", "");
  wifiIndicatorEnable = prefs.getBool("wifiInd", false);

  diInvertLogic = prefs.getBool("diInv", false);
  // The relay/DO LED circuit on this board appears to be active-low.
  // Default true keeps DO LEDs/relays OFF at boot by writing HIGH to the expander.
  doInvertLogic = prefs.getBool("doInv", true);

  ethMode = prefs.getString("ethMode", "DHCP");
  ethIP = prefs.getString("ethIP", "192.168.1.222");
  ethGW = prefs.getString("ethGW", "192.168.1.1");
  ethMask = prefs.getString("ethMask", "255.255.255.0");
  ethDNS = prefs.getString("ethDNS", "8.8.8.8");

  ethModbusServerEnable = prefs.getBool("ethMbs", true);
  ethModbusHostEnable = prefs.getBool("ethMbc", false);
  mbTcpHostIP = prefs.getString("mbTcpIP", "192.168.1.100");
  mbTcpHostPort = prefs.getUShort("mbTcpPort", 502);
  mbTcpHostUnit = prefs.getUChar("mbTcpUnit", 1);
  mbTcpHostReg = prefs.getUShort("mbTcpReg", 0);
  mbTcpHostCount = prefs.getUShort("mbTcpCnt", 8);
  mbTcpPollMs = prefs.getUInt("mbTcpMs", 5000);

  rs485Baud = prefs.getUInt("rbaud", 9600);
  rs485Id = prefs.getUChar("rid", 1);
  rs485ModbusSlaveEnable = prefs.getBool("rMbs", true);
  rs485ModbusMasterEnable = prefs.getBool("rMbc", false);
  rtuMasterId = prefs.getUChar("rtuMid", 1);
  rtuMasterReg = prefs.getUShort("rtuMReg", 0);
  rtuMasterCount = prefs.getUShort("rtuMCnt", 8);
  rtuPollMs = prefs.getUInt("rtuMs", 5000);

  mqttEnable = prefs.getBool("mqttEn", false);
  mqttIface = prefs.getString("mqttIf", "AUTO");
  mqttHost = prefs.getString("mqttHost", "192.168.1.10");
  mqttPort = prefs.getUShort("mqttPort", 1883);
  mqttUser = prefs.getString("mqttUser", "");
  mqttPass = prefs.getString("mqttPass", "");
  mqttBaseTopic = prefs.getString("mqttTopic", "lct/" + apName());

  ntpEnable = prefs.getBool("ntpEn", false);
  ntpIface = prefs.getString("ntpIf", "AUTO");
  ntpServer = prefs.getString("ntpSrv", "pool.ntp.org");
  gmtOffsetSec = prefs.getLong("gmtOff", -21600);
  daylightOffsetSec = prefs.getInt("dstOff", 3600);

  csvLogEnable = prefs.getBool("csvEn", true);
  csvLogMs = prefs.getUInt("csvMs", 5000);

  mbCsvLogEnable = prefs.getBool("mbCsvEn", true);
  mbCsvLogMs = prefs.getUInt("mbCsvMs", 5000);

  ioEventLogEnable = prefs.getBool("ioEvtEn", true);

  canEnable = prefs.getBool("canEn", false);
  canTxPin = prefs.getInt("canTx", 36);
  canRxPin = prefs.getInt("canRx", 37);
  canBitrate = prefs.getUInt("canBaud", 500000);
  canListenOnly = prefs.getBool("canListen", false);

  for (int i = 0; i < 8; i++) {
    inputNames[i] = prefs.getString(("in" + String(i + 1)).c_str(), "DI" + String(i + 1));
    outputNames[i] = prefs.getString(("out" + String(i + 1)).c_str(), "DO" + String(i + 1));
  }

  prefs.end();
}

void saveIOConfigFromWeb() {
  prefs.begin("cfg", false);
  for (int i = 0; i < 8; i++) {
    String ik = "in" + String(i + 1);
    String ok = "out" + String(i + 1);
    String iv = server.arg(ik);
    String ov = server.arg(ok);
    prefs.putString(ik.c_str(), iv);
    prefs.putString(ok.c_str(), ov);
    inputNames[i] = iv;
    outputNames[i] = ov;
  }
  prefs.putBool("csvEn", server.hasArg("csvEn"));
  prefs.putUInt("csvMs", server.arg("csvMs").toInt());
  prefs.putBool("ioEvtEn", server.hasArg("ioEvtEn"));
  prefs.putBool("diInv", server.hasArg("diInv"));
  prefs.putBool("doInv", server.hasArg("doInv"));
  csvLogEnable = server.hasArg("csvEn");
  csvLogMs = server.arg("csvMs").toInt();
  ioEventLogEnable = server.hasArg("ioEvtEn");
  diInvertLogic = server.hasArg("diInv");
  doInvertLogic = server.hasArg("doInv");
  if (tcaOK) {
    tca9554Init(0x00, outputsOffRawByte());
    for (int i = 0; i < 8; i++) outState[i] = false;
  }
  prefs.end();
}

// ---------------- IO / Registers / JSON ----------------

bool logicalInputFromRaw(bool rawLevel) {
  return diInvertLogic ? !rawLevel : rawLevel;
}

bool logicalOutputFromRaw(bool rawLevel) {
  return doInvertLogic ? !rawLevel : rawLevel;
}

bool rawOutputFromLogical(bool logicalOn) {
  return doInvertLogic ? !logicalOn : logicalOn;
}

uint8_t outputsOffRawByte() {
  return doInvertLogic ? 0xFF : 0x00;
}

void setOutput(uint8_t ch, bool on) {
  if (ch < 1 || ch > 8) return;
  if (tcaOK) setEXIO(ch, rawOutputFromLogical(on));
  outState[ch - 1] = on;
}

void refreshInputsOutputs() {
  for (int i = 0; i < 8; i++) {
    bool raw = digitalRead(DI_PINS[i]);
    inState[i] = logicalInputFromRaw(raw);
  }

  if (tcaOK) {
    for (int i = 0; i < 8; i++) {
      bool rawActual = rawOutputFromLogical(outState[i]);
      if (readEXIOOutput(i + 1, rawActual)) outState[i] = logicalOutputFromRaw(rawActual);
    }
  }

  uint16_t diMask = 0;
  uint16_t doMask = 0;
  for (int i = 0; i < 8; i++) {
    if (inState[i]) diMask |= (1 << i);
    if (outState[i]) doMask |= (1 << i);
  }

  holdingRegs[0] = diMask;
  holdingRegs[1] = doMask;
  holdingRegs[2] = tcaOK ? 1 : 0;
  holdingRegs[3] = rtcOK ? 1 : 0;
  holdingRegs[4] = sdOK ? 1 : 0;
  holdingRegs[5] = WiFi.status() == WL_CONNECTED ? 1 : 0;
  holdingRegs[6] = ethernetStarted ? 1 : 0;
  holdingRegs[7] = ESP.getFreeHeap() & 0xFFFF;
}

void applyOutputMask(uint16_t mask) {
  for (int i = 0; i < 8; i++) setOutput(i + 1, mask & (1 << i));
}

String statusJson() {
  refreshInputsOutputs();
  String s;
  s += "{";
  s += "\"device\":\"" + apName() + "\",";
  s += "\"ap_ip\":\"" + wifiApIpString() + "\",";
  s += "\"sta_ip\":\"" + WiFi.localIP().toString() + "\",";
  s += "\"wifi_connected\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false") + ",";
  s += "\"wifi_indicator_enable\":" + String(wifiIndicatorEnable ? "true" : "false") + ",";
  s += "\"di_invert\":" + String(diInvertLogic ? "true" : "false") + ",";
  s += "\"do_invert\":" + String(doInvertLogic ? "true" : "false") + ",";
  s += "\"ethernet_started\":" + String(ethernetStarted ? "true" : "false") + ",";
  s += "\"ethernet_ip\":\"" + (ethernetStarted ? ipToString(Ethernet.localIP()) : String("0.0.0.0")) + "\",";
  s += "\"rtc\":\"" + rtcString() + "\",";
  s += "\"tca_ok\":" + String(tcaOK ? "true" : "false") + ",";
  s += "\"rtc_ok\":" + String(rtcOK ? "true" : "false") + ",";
  s += "\"sd_ok\":" + String(sdOK ? "true" : "false") + ",";
  s += "\"can_started\":" + String(canStarted ? "true" : "false") + ",";
  s += "\"can_rx_count\":" + String(canRxCount) + ",";
  s += "\"can_tx_count\":" + String(canTxCount) + ",";
  s += "\"ntp_interface\":\"" + ntpIface + "\",";
  s += "\"mqtt_interface\":\"" + mqttIface + "\",";
  s += "\"mqtt_active_interface\":\"" + activeMqttIface + "\",";
  s += "\"free_heap\":" + String(ESP.getFreeHeap()) + ",";
  s += "\"di_mask\":" + String(holdingRegs[0]) + ",";
  s += "\"do_mask\":" + String(holdingRegs[1]) + ",";
  s += "\"inputs\":[";
  for (int i = 0; i < 8; i++) {
    if (i) s += ",";
    s += "{\"ch\":" + String(i + 1) + ",\"name\":\"" + esc(inputNames[i]) + "\",\"state\":" + String(inState[i] ? "true" : "false") + "}";
  }
  s += "],\"outputs\":[";
  for (int i = 0; i < 8; i++) {
    if (i) s += ",";
    s += "{\"ch\":" + String(i + 1) + ",\"name\":\"" + esc(outputNames[i]) + "\",\"state\":" + String(outState[i] ? "true" : "false") + "}";
  }
  s += "]}";
  return s;
}

// ---------------- WiFi / Ethernet / SD ----------------

void setupWiFi() {
  WiFi.persistent(false);
  WiFi.disconnect(true, true);
  WiFi.softAPdisconnect(true);
  delay(250);

  bool shouldTrySta = (wifiMode == "STA" || wifiIndicatorEnable) && staSsid.length() > 0;

  if (shouldTrySta) {
    /*
      Try STA first while temporarily enabling fallback AP.
      If STA connects, shut fallback AP off.
      If STA fails, leave fallback AP running so the device is reachable.
    */
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(apName().c_str(), apPassword().c_str());
    WiFi.setHostname(apName().c_str());

    Serial.println("Connecting STA WiFi to SSID: " + staSsid);
    WiFi.begin(staSsid.c_str(), staPass.c_str());

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
      delay(250);
      Serial.print(".");
    }

    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("STA WiFi connected. Disabling fallback AP/hotspot.");
      WiFi.softAPdisconnect(true);
      WiFi.mode(WIFI_STA);
    } else {
      Serial.println("STA WiFi connection failed or timed out. Keeping fallback AP/hotspot active.");
      WiFi.mode(WIFI_AP_STA);
      WiFi.softAP(apName().c_str(), apPassword().c_str());
    }

    lastWifiStaRetry = millis();
  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(apName().c_str(), apPassword().c_str());
  }
}

void retryStaWifiIfNeeded() {
  if (staSsid.length() == 0) return;

  if (WiFi.status() == WL_CONNECTED) {
    // Once connected to configured WiFi, keep ESP32 fallback AP/hotspot off.
    if (WiFi.getMode() != WIFI_STA) {
      WiFi.softAPdisconnect(true);
      WiFi.mode(WIFI_STA);
    }
    return;
  }

  if (millis() - lastWifiStaRetry < WIFI_STA_RETRY_MS) return;

  lastWifiStaRetry = millis();

  Serial.println("Retrying STA WiFi connection to SSID: " + staSsid);

  // Not connected, so keep fallback AP alive while retrying STA.
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(apName().c_str(), apPassword().c_str());

  WiFi.disconnect(false, false);
  delay(100);
  WiFi.begin(staSsid.c_str(), staPass.c_str());
}

void handleWifiConnectIndicator() {
  if (!wifiIndicatorEnable) return;

  bool hasTargetAp = staSsid.length() > 0;
  bool staConnected = (WiFi.status() == WL_CONNECTED);

  if (hasTargetAp && staConnected) {
    // Connected indicator: DO1 held ON.
    if (!outState[0]) setOutput(1, true);
    return;
  }

  // Not connected indicator: blink DO1 once per second.
  if (millis() - lastWifiIndicatorBlink >= WIFI_INDICATOR_BLINK_MS) {
    lastWifiIndicatorBlink = millis();
    wifiIndicatorBlinkState = !wifiIndicatorBlinkState;
    setOutput(1, wifiIndicatorBlinkState);
  }
}


void setupEthernet() {
  pinMode(W5500_RST, OUTPUT);
  digitalWrite(W5500_RST, LOW);
  delay(100);
  digitalWrite(W5500_RST, HIGH);
  delay(300);

  SPI.begin(W5500_SCLK, W5500_MISO, W5500_MOSI, W5500_CS);
  Ethernet.init(W5500_CS);

  byte mac[6];
  uint8_t wifiStaMac[6] = {0};
  esp_read_mac(wifiStaMac, ESP_MAC_WIFI_STA);

  // Locally administered Ethernet MAC based on factory WiFi STA MAC.
  mac[0] = 0x02;
  mac[1] = wifiStaMac[1];
  mac[2] = wifiStaMac[2];
  mac[3] = wifiStaMac[3];
  mac[4] = wifiStaMac[4];
  mac[5] = wifiStaMac[5];

  memcpy(ethMacAddr, mac, 6);

  if (ethMode == "STATIC") {
    IPAddress ip = parseIP(ethIP, IPAddress(192,168,1,222));
    IPAddress dns = parseIP(ethDNS, IPAddress(8,8,8,8));
    IPAddress gw = parseIP(ethGW, IPAddress(192,168,1,1));
    IPAddress mask = parseIP(ethMask, IPAddress(255,255,255,0));
    Ethernet.begin(mac, ip, dns, gw, mask);
    ethernetStarted = true;
  } else {
    ethernetStarted = Ethernet.begin(mac) != 0;
  }

  if (ethernetStarted && ethModbusServerEnable) {
    modbusTcpServer.begin();
    modbusTcpStarted = true;
  }

  if (ethernetStarted) {
    ethHttpServer.begin();
  }
}

void setupSD() {
  SD_MMC.setPins(SD_CLK, SD_CMD, SD_D0);
  sdOK = SD_MMC.begin("/sdcard", true);
  if (sdOK && !SD_MMC.exists("/io_log.csv")) {
    File f = SD_MMC.open("/io_log.csv", FILE_WRITE);
    if (f) {
      f.println("timestamp,di_mask,do_mask,di1,di2,di3,di4,di5,di6,di7,di8,do1,do2,do3,do4,do5,do6,do7,do8");
      f.close();
    }
  }

  if (sdOK && !SD_MMC.exists("/modbus_log.csv")) {
    File f = SD_MMC.open("/modbus_log.csv", FILE_WRITE);
    if (f) {
      f.println("timestamp,register_address,register_description,raw_value,math_formula,computed_value");
      f.close();
    }
  }
}


// ---------------- IO Event Logging ----------------
//
// Creates one daily file:
//   /io_events_YYYY-MM-DD.csv
//
// Event rows are appended when an input/output changes state.
// The duration row records how long the PREVIOUS state lasted.
// At day rollover, active intervals are closed and a TOTALIZER section
// is appended to the previous day's file.

String rtcDateString() {
  LCT_DateTime now;
  if (!pcf85063ReadTime(now)) return "unknown-date";

  char buf[16];
  snprintf(buf, sizeof(buf), "%04u-%02u-%02u", now.year, now.month, now.day);
  return String(buf);
}

String ioEventFilePath(String dateStr) {
  return "/io_events_" + dateStr + ".csv";
}

String secondsToHMS(uint32_t seconds) {
  uint32_t h = seconds / 3600UL;
  uint32_t m = (seconds % 3600UL) / 60UL;
  uint32_t s = seconds % 60UL;

  char buf[24];
  snprintf(buf, sizeof(buf), "%lu:%02lu:%02lu", (unsigned long)h, (unsigned long)m, (unsigned long)s);
  return String(buf);
}

String ioEventType(uint8_t idx) {
  return idx < 8 ? "DI" : "DO";
}

uint8_t ioEventNumber(uint8_t idx) {
  return idx < 8 ? idx + 1 : idx - 7;
}

String ioEventName(uint8_t idx) {
  return idx < 8 ? inputNames[idx] : outputNames[idx - 8];
}

bool ioEventCurrentState(uint8_t idx) {
  return idx < 8 ? inState[idx] : outState[idx - 8];
}

void ensureIoEventFile(String dateStr) {
  if (!sdOK) return;

  String path = ioEventFilePath(dateStr);

  if (!SD_MMC.exists(path)) {
    File f = SD_MMC.open(path, FILE_WRITE);
    if (f) {
      f.println("record_type,date,pin_type,pin_number,pin_name,state,start_time,end_time,duration_seconds,duration_hms,total_on_seconds,total_off_seconds,total_on_hms,total_off_hms");
      f.close();
    }
  }
}

void appendIoEventRow(uint8_t idx, bool state, String startTime, String endTime, uint32_t durationSeconds, String dateStr) {
  if (!ioEventLogEnable || !sdOK) return;

  ensureIoEventFile(dateStr);

  File f = SD_MMC.open(ioEventFilePath(dateStr), FILE_APPEND);
  if (!f) return;

  f.print("EVENT");
  f.print(",");
  f.print(dateStr);
  f.print(",");
  f.print(ioEventType(idx));
  f.print(",");
  f.print(ioEventNumber(idx));
  f.print(",");
  f.print(csvEscape(ioEventName(idx)));
  f.print(",");
  f.print(state ? "ON" : "OFF");
  f.print(",");
  f.print(csvEscape(startTime));
  f.print(",");
  f.print(csvEscape(endTime));
  f.print(",");
  f.print(durationSeconds);
  f.print(",");
  f.print(csvEscape(secondsToHMS(durationSeconds)));
  f.print(",,,,");
  f.println();

  f.close();
}

void appendIoEventTotals(String dateStr) {
  if (!ioEventLogEnable || !sdOK) return;

  ensureIoEventFile(dateStr);

  File f = SD_MMC.open(ioEventFilePath(dateStr), FILE_APPEND);
  if (!f) return;

  f.println();
  f.println("TOTALIZER");
  f.println("record_type,date,pin_type,pin_number,pin_name,state,start_time,end_time,duration_seconds,duration_hms,total_on_seconds,total_off_seconds,total_on_hms,total_off_hms");

  for (int i = 0; i < 16; i++) {
    f.print("TOTAL");
    f.print(",");
    f.print(dateStr);
    f.print(",");
    f.print(ioEventType(i));
    f.print(",");
    f.print(ioEventNumber(i));
    f.print(",");
    f.print(csvEscape(ioEventName(i)));
    f.print(",,,,,,");
    f.print(ioEventOnSeconds[i]);
    f.print(",");
    f.print(ioEventOffSeconds[i]);
    f.print(",");
    f.print(csvEscape(secondsToHMS(ioEventOnSeconds[i])));
    f.print(",");
    f.println(csvEscape(secondsToHMS(ioEventOffSeconds[i])));
  }

  f.close();
}

void resetIoEventTotals() {
  for (int i = 0; i < 16; i++) {
    ioEventOnSeconds[i] = 0;
    ioEventOffSeconds[i] = 0;
  }
}

void initIoEventLogger() {
  refreshInputsOutputs();

  ioEventCurrentDate = rtcDateString();
  ensureIoEventFile(ioEventCurrentDate);

  String now = rtcString();

  for (int i = 0; i < 16; i++) {
    ioEventState[i] = ioEventCurrentState(i);
    ioEventStartMillis[i] = millis();
    ioEventStartTime[i] = now;
    ioEventOnSeconds[i] = 0;
    ioEventOffSeconds[i] = 0;
  }

  ioEventInitialized = true;
}

void closeIoEventIntervalsForDay(String closingDate, String closingTime) {
  unsigned long nowMs = millis();

  for (int i = 0; i < 16; i++) {
    uint32_t durationSeconds = (nowMs - ioEventStartMillis[i]) / 1000UL;

    if (ioEventState[i]) {
      ioEventOnSeconds[i] += durationSeconds;
    } else {
      ioEventOffSeconds[i] += durationSeconds;
    }

    appendIoEventRow(i, ioEventState[i], ioEventStartTime[i], closingTime, durationSeconds, closingDate);
  }

  appendIoEventTotals(closingDate);
}

void handleIoEventLogging() {
  if (!ioEventLogEnable || !sdOK) return;

  if (!ioEventInitialized) {
    initIoEventLogger();
    return;
  }

  refreshInputsOutputs();

  String dateNow = rtcDateString();
  String timeNow = rtcString();
  unsigned long nowMs = millis();

  if (dateNow != ioEventCurrentDate && dateNow != "unknown-date") {
    closeIoEventIntervalsForDay(ioEventCurrentDate, timeNow);

    resetIoEventTotals();
    ioEventCurrentDate = dateNow;
    ensureIoEventFile(ioEventCurrentDate);

    for (int i = 0; i < 16; i++) {
      ioEventState[i] = ioEventCurrentState(i);
      ioEventStartMillis[i] = nowMs;
      ioEventStartTime[i] = timeNow;
    }

    return;
  }

  for (int i = 0; i < 16; i++) {
    bool current = ioEventCurrentState(i);

    if (current != ioEventState[i]) {
      uint32_t durationSeconds = (nowMs - ioEventStartMillis[i]) / 1000UL;

      if (ioEventState[i]) {
        ioEventOnSeconds[i] += durationSeconds;
      } else {
        ioEventOffSeconds[i] += durationSeconds;
      }

      appendIoEventRow(i, ioEventState[i], ioEventStartTime[i], timeNow, durationSeconds, ioEventCurrentDate);

      ioEventState[i] = current;
      ioEventStartMillis[i] = nowMs;
      ioEventStartTime[i] = timeNow;
    }
  }
}

// ---------------- CSV Logging ----------------

void csvLogIO() {
  if (!csvLogEnable || !sdOK) return;
  if (millis() - lastCsvLog < csvLogMs) return;
  lastCsvLog = millis();

  refreshInputsOutputs();

  File f = SD_MMC.open("/io_log.csv", FILE_APPEND);
  if (!f) return;

  f.print(rtcString());
  f.print(",");
  f.print(holdingRegs[0]);
  f.print(",");
  f.print(holdingRegs[1]);

  for (int i = 0; i < 8; i++) {
    f.print(",");
    f.print(inState[i] ? 1 : 0);
  }

  for (int i = 0; i < 8; i++) {
    f.print(",");
    f.print(outState[i] ? 1 : 0);
  }

  f.println();
  f.close();
}


String csvEscape(String value) {
  value.replace("\"", "\"\"");
  if (value.indexOf(',') >= 0 || value.indexOf('"') >= 0 || value.indexOf('\n') >= 0 || value.indexOf('\r') >= 0) {
    value = "\"" + value + "\"";
  }
  return value;
}

uint16_t getDisplayedModbusRawValue(uint16_t addr) {
  // Internal status/register mirror
  if (addr < 128) {
    return holdingRegs[addr];
  }

  // Common Modbus holding register display ranges:
  // 40001 maps to internal polled offset 0.
  // 40000 maps to internal polled offset 0 for devices/documentation that start at 40000.
  if (addr >= 40001 && addr <= 40128) {
    uint16_t offset = addr - 40001;
    if (offset < 128) return holdingRegs[offset];
  }

  if (addr >= 40000 && addr <= 40127) {
    uint16_t offset = addr - 40000;
    if (offset < 128) return holdingRegs[offset];
  }

  return 0;
}

void csvLogModbus() {
  if (!mbCsvLogEnable || !sdOK) return;
  if (millis() - lastMbCsvLog < mbCsvLogMs) return;
  lastMbCsvLog = millis();

  refreshInputsOutputs();

  File f = SD_MMC.open("/modbus_log.csv", FILE_APPEND);
  if (!f) return;

  String ts = rtcString();

  for (int i = 0; i < mbMonCount; i++) {
    uint16_t addr = mbMonAddr[i];
    uint16_t raw = getDisplayedModbusRawValue(addr);
    double computed = computeFormula(mbMonFormula[i], raw);

    f.print(csvEscape(ts));
    f.print(",");
    f.print(addr);
    f.print(",");
    f.print(csvEscape(mbMonDesc[i]));
    f.print(",");
    f.print(raw);
    f.print(",");
    f.print(csvEscape(mbMonFormula[i]));
    f.print(",");
    f.println(fmtDouble(computed));
  }

  f.close();
}

// ---------------- Modbus CRC / Core ----------------

uint16_t modbusCRC(const uint8_t *buf, int len) {
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];
    for (int i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

uint16_t getReg(uint16_t addr) {
  if (addr < 128) return holdingRegs[addr];
  return 0;
}

void setReg(uint16_t addr, uint16_t val) {
  if (addr >= 128) return;
  holdingRegs[addr] = val;
  if (addr == 1) applyOutputMask(val);
}

int buildModbusPDU(const uint8_t *req, int reqLen, uint8_t *resp) {
  if (reqLen < 5) return 0;

  uint8_t fc = req[0];

  if (fc == 3 || fc == 4) {
    if (reqLen < 5) return 0;
    uint16_t start = ((uint16_t)req[1] << 8) | req[2];
    uint16_t qty = ((uint16_t)req[3] << 8) | req[4];
    if (qty < 1 || qty > 60 || start + qty > 128) {
      resp[0] = fc | 0x80;
      resp[1] = 0x02;
      return 2;
    }
    resp[0] = fc;
    resp[1] = qty * 2;
    int idx = 2;
    for (uint16_t i = 0; i < qty; i++) {
      uint16_t v = getReg(start + i);
      resp[idx++] = v >> 8;
      resp[idx++] = v & 0xFF;
    }
    return idx;
  }

  if (fc == 6) {
    uint16_t addr = ((uint16_t)req[1] << 8) | req[2];
    uint16_t val = ((uint16_t)req[3] << 8) | req[4];
    if (addr >= 128) {
      resp[0] = fc | 0x80;
      resp[1] = 0x02;
      return 2;
    }
    setReg(addr, val);
    memcpy(resp, req, 5);
    return 5;
  }

  if (fc == 16) {
    if (reqLen < 6) return 0;
    uint16_t start = ((uint16_t)req[1] << 8) | req[2];
    uint16_t qty = ((uint16_t)req[3] << 8) | req[4];
    uint8_t byteCount = req[5];
    if (qty < 1 || qty > 60 || byteCount != qty * 2 || start + qty > 128 || reqLen < 6 + byteCount) {
      resp[0] = fc | 0x80;
      resp[1] = 0x02;
      return 2;
    }
    int idx = 6;
    for (uint16_t i = 0; i < qty; i++) {
      uint16_t val = ((uint16_t)req[idx] << 8) | req[idx + 1];
      setReg(start + i, val);
      idx += 2;
    }
    resp[0] = fc;
    resp[1] = start >> 8;
    resp[2] = start & 0xFF;
    resp[3] = qty >> 8;
    resp[4] = qty & 0xFF;
    return 5;
  }

  resp[0] = fc | 0x80;
  resp[1] = 0x01;
  return 2;
}

// ---------------- Modbus TCP Server ----------------

void handleModbusTcpServer() {
  if (!ethernetStarted || !ethModbusServerEnable) return;

  EthernetClient newClient = modbusTcpServer.available();
  if (newClient) {
    if (!modbusTcpClient || !modbusTcpClient.connected()) {
      modbusTcpClient = newClient;
    } else {
      newClient.stop();
    }
  }

  if (!modbusTcpClient || !modbusTcpClient.connected()) return;
  if (modbusTcpClient.available() < 8) return;

  uint8_t header[7];
  modbusTcpClient.read(header, 7);
  uint16_t trans = ((uint16_t)header[0] << 8) | header[1];
  uint16_t proto = ((uint16_t)header[2] << 8) | header[3];
  uint16_t len = ((uint16_t)header[4] << 8) | header[5];
  uint8_t unit = header[6];

  if (proto != 0 || len < 2 || len > 253) {
    modbusTcpClient.stop();
    return;
  }

  int pduLen = len - 1;
  uint8_t pdu[253];
  int got = 0;
  unsigned long start = millis();
  while (got < pduLen && millis() - start < 500) {
    if (modbusTcpClient.available()) pdu[got++] = modbusTcpClient.read();
  }
  if (got != pduLen) return;

  refreshInputsOutputs();

  uint8_t respPdu[253];
  int respLen = buildModbusPDU(pdu, pduLen, respPdu);
  if (respLen <= 0) return;

  uint8_t out[260];
  out[0] = trans >> 8;
  out[1] = trans & 0xFF;
  out[2] = 0;
  out[3] = 0;
  out[4] = ((respLen + 1) >> 8) & 0xFF;
  out[5] = (respLen + 1) & 0xFF;
  out[6] = unit;
  memcpy(out + 7, respPdu, respLen);
  modbusTcpClient.write(out, respLen + 7);
}

// ---------------- Modbus RTU Slave ----------------

void rs485TxMode(bool tx) {
  digitalWrite(RS485_RTS, tx ? HIGH : LOW);
  delayMicroseconds(80);
}

void handleModbusRtuSlave() {
  if (!rs485ModbusSlaveEnable) return;
  static uint8_t buf[256];
  static int len = 0;
  static unsigned long lastByte = 0;

  while (RS485Serial.available()) {
    if (len < 256) buf[len++] = RS485Serial.read();
    lastByte = millis();
  }

  if (len == 0 || millis() - lastByte < 8) return;

  if (len >= 8) {
    uint16_t rxCrc = ((uint16_t)buf[len - 1] << 8) | buf[len - 2];
    uint16_t calc = modbusCRC(buf, len - 2);

    if (rxCrc == calc && (buf[0] == rs485Id || buf[0] == 0)) {
      refreshInputsOutputs();

      uint8_t respPdu[253];
      int respPduLen = buildModbusPDU(buf + 1, len - 3, respPdu);

      if (respPduLen > 0 && buf[0] != 0) {
        uint8_t out[260];
        out[0] = rs485Id;
        memcpy(out + 1, respPdu, respPduLen);
        uint16_t crc = modbusCRC(out, respPduLen + 1);
        out[respPduLen + 1] = crc & 0xFF;
        out[respPduLen + 2] = crc >> 8;

        rs485TxMode(true);
        RS485Serial.write(out, respPduLen + 3);
        RS485Serial.flush();
        rs485TxMode(false);
      }
    }
  }

  len = 0;
}

// ---------------- Modbus Host Polling ----------------

bool modbusTcpReadHolding(String host, uint16_t port, uint8_t unit, uint16_t reg, uint16_t count) {
  if (!ethernetStarted || !ethModbusHostEnable) return false;
  if (count < 1) return false;
  if (count > MB_REMOTE_MAX) count = MB_REMOTE_MAX;

  IPAddress ip = parseIP(host, IPAddress(192,168,1,100));
  EthernetClient c;
  if (!c.connect(ip, port)) return false;

  uint8_t req[12];
  static uint16_t tid = 1;
  req[0] = tid >> 8; req[1] = tid & 0xFF; tid++;
  req[2] = 0; req[3] = 0;
  req[4] = 0; req[5] = 6;
  req[6] = unit;
  req[7] = 3;
  req[8] = reg >> 8; req[9] = reg & 0xFF;
  req[10] = count >> 8; req[11] = count & 0xFF;
  c.write(req, 12);

  unsigned long start = millis();
  while (c.available() < 9 && millis() - start < 1500) delay(1);
  if (c.available() < 9) { c.stop(); return false; }

  uint8_t hdr[9];
  c.read(hdr, 9);

  uint16_t mbapLen = ((uint16_t)hdr[4] << 8) | hdr[5];
  if (hdr[7] != 3 || mbapLen < 3) { c.stop(); return false; }

  uint8_t bytes = hdr[8];
  if (bytes < 2) { c.stop(); return false; }

  start = millis();
  while (c.available() < bytes && millis() - start < 1500) delay(1);
  if (c.available() < bytes) { c.stop(); return false; }

  uint16_t regsReceived = bytes / 2;
  if (regsReceived > MB_REMOTE_MAX) regsReceived = MB_REMOTE_MAX;

  mbRemoteStart = reg;
  mbRemoteCount = regsReceived;
  memset(mbRemoteValid, 0, sizeof(mbRemoteValid));

  for (uint16_t i = 0; i < regsReceived; i++) {
    uint8_t hi = c.read();
    uint8_t lo = c.read();
    mbRemoteRegs[i] = ((uint16_t)hi << 8) | lo;
    mbRemoteValid[i] = true;
  }

  // Drain any extra bytes.
  while (c.available()) c.read();

  c.stop();
  return true;
}

bool modbusRtuReadHolding(uint8_t unit, uint16_t reg, uint16_t count) {
  if (!rs485ModbusMasterEnable) return false;

  uint8_t req[8];
  req[0] = unit;
  req[1] = 3;
  req[2] = reg >> 8;
  req[3] = reg & 0xFF;
  req[4] = count >> 8;
  req[5] = count & 0xFF;
  uint16_t crc = modbusCRC(req, 6);
  req[6] = crc & 0xFF;
  req[7] = crc >> 8;

  while (RS485Serial.available()) RS485Serial.read();

  rs485TxMode(true);
  RS485Serial.write(req, 8);
  RS485Serial.flush();
  rs485TxMode(false);

  uint8_t resp[256];
  int len = 0;
  unsigned long start = millis();

  while (millis() - start < 1000 && len < 256) {
    if (RS485Serial.available()) resp[len++] = RS485Serial.read();
  }

  if (len < 5) return false;
  uint16_t rxCrc = ((uint16_t)resp[len - 1] << 8) | resp[len - 2];
  if (rxCrc != modbusCRC(resp, len - 2)) return false;
  if (resp[0] != unit || resp[1] != 3) return false;

  uint8_t bytes = resp[2];
  uint16_t regsReceived = bytes / 2;
  if (regsReceived > MB_REMOTE_MAX) regsReceived = MB_REMOTE_MAX;

  mbRemoteStart = reg;
  mbRemoteCount = regsReceived;
  memset(mbRemoteValid, 0, sizeof(mbRemoteValid));

  int pos = 3;
  for (uint16_t i = 0; i < regsReceived; i++) {
    mbRemoteRegs[i] = ((uint16_t)resp[pos] << 8) | resp[pos + 1];
    mbRemoteValid[i] = true;
    pos += 2;
  }

  return true;
}

void handleModbusHostPolling() {
  if (ethModbusHostEnable && millis() - lastTcpHostPoll > mbTcpPollMs) {
    lastTcpHostPoll = millis();
    modbusTcpReadHolding(mbTcpHostIP, mbTcpHostPort, mbTcpHostUnit, mbTcpHostReg, mbTcpHostCount);
  }

  if (rs485ModbusMasterEnable && millis() - lastRtuMasterPoll > rtuPollMs) {
    lastRtuMasterPoll = millis();
    modbusRtuReadHolding(rtuMasterId, rtuMasterReg, rtuMasterCount);
  }
}

// ---------------- MQTT ----------------

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String t = topic;
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];

  String base = mqttBaseTopic;
  if (t == base + "/cmd/output") {
    int comma = msg.indexOf(',');
    if (comma > 0) {
      int ch = msg.substring(0, comma).toInt();
      int st = msg.substring(comma + 1).toInt();
      setOutput(ch, st == 1);
    }
  }

  if (t == base + "/cmd/output_mask") {
    uint16_t mask = msg.toInt();
    applyOutputMask(mask);
  }
}

String chooseMqttInterface() {
  if (mqttIface == "WIFI") return "WIFI";
  if (mqttIface == "ETH") return "ETH";
  if (WiFi.status() == WL_CONNECTED) return "WIFI";
  if (ethernetStarted) return "ETH";
  return "NONE";
}

bool mqttNetworkReady(const String &iface) {
  if (iface == "WIFI") return WiFi.status() == WL_CONNECTED;
  if (iface == "ETH") return ethernetStarted;
  return false;
}

void setupMqtt() {
  activeMqttIface = chooseMqttInterface();
  if (activeMqttIface == "ETH") mqttClient.setClient(mqttEthClient);
  else mqttClient.setClient(wifiNetClient);
  mqttClient.setServer(mqttHost.c_str(), mqttPort);
  mqttClient.setCallback(mqttCallback);
}

void handleMqtt() {
  if (!mqttEnable) return;

  String desiredIface = chooseMqttInterface();
  if (desiredIface == "NONE") return;

  if (desiredIface != activeMqttIface) {
    if (mqttClient.connected()) mqttClient.disconnect();
    activeMqttIface = desiredIface;
    if (activeMqttIface == "ETH") mqttClient.setClient(mqttEthClient);
    else mqttClient.setClient(wifiNetClient);
    mqttClient.setServer(mqttHost.c_str(), mqttPort);
  }

  if (!mqttNetworkReady(activeMqttIface)) return;

  if (!mqttClient.connected() && millis() - lastMqttReconnect > 5000) {
    lastMqttReconnect = millis();
    String clientId = apName() + "-" + activeMqttIface;
    bool ok;
    if (mqttUser.length()) ok = mqttClient.connect(clientId.c_str(), mqttUser.c_str(), mqttPass.c_str());
    else ok = mqttClient.connect(clientId.c_str());

    if (ok) {
      mqttClient.subscribe((mqttBaseTopic + "/cmd/output").c_str());
      mqttClient.subscribe((mqttBaseTopic + "/cmd/output_mask").c_str());
      mqttClient.publish((mqttBaseTopic + "/status").c_str(), "online", true);
      mqttClient.publish((mqttBaseTopic + "/interface").c_str(), activeMqttIface.c_str(), true);
    }
  }

  mqttClient.loop();

  if (mqttClient.connected() && millis() - lastMqttPublish > 5000) {
    lastMqttPublish = millis();
    String js = statusJson();
    mqttClient.publish((mqttBaseTopic + "/json").c_str(), js.c_str(), true);
    mqttClient.publish((mqttBaseTopic + "/di_mask").c_str(), String(holdingRegs[0]).c_str(), true);
    mqttClient.publish((mqttBaseTopic + "/do_mask").c_str(), String(holdingRegs[1]).c_str(), true);
    mqttClient.publish((mqttBaseTopic + "/interface").c_str(), activeMqttIface.c_str(), true);
  }
}

// ---------------- WebSocket ----------------

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_TEXT) {
    String msg = String((char*)payload);
    if (msg.startsWith("OUT,")) {
      int c1 = msg.indexOf(',');
      int c2 = msg.indexOf(',', c1 + 1);
      int ch = msg.substring(c1 + 1, c2).toInt();
      int st = msg.substring(c2 + 1).toInt();
      setOutput(ch, st == 1);
    }
  }
}

void handleWebSocketPush() {
  webSocket.loop();
  if (millis() - lastWebSocketPush > 1000) {
    lastWebSocketPush = millis();
    String wsPayload = statusJson();
    webSocket.broadcastTXT(wsPayload);
  }
}



String canFrameToString(const twai_message_t &msg) {
  String s = "ID=0x" + String(msg.identifier, HEX);
  s.toUpperCase();
  s += msg.extd ? " EXT" : " STD";
  s += msg.rtr ? " RTR" : " DATA";
  s += " DLC=" + String(msg.data_length_code) + " DATA=";

  uint64_t unsignedValue = 0;
  int32_t signedValue = 0;

  for (int i = 0; i < msg.data_length_code; i++) {
    if (msg.data[i] < 16) s += "0";
    s += String(msg.data[i], HEX);
    if (i < msg.data_length_code - 1) s += " ";
    unsignedValue = (unsignedValue << 8) | msg.data[i];
  }

  if (msg.data_length_code > 0 && msg.data_length_code <= 4) {
    signedValue = (int32_t)unsignedValue;
    int bits = msg.data_length_code * 8;
    if (bits < 32 && (unsignedValue & (1UL << (bits - 1)))) {
      signedValue = (int32_t)(unsignedValue | (~0UL << bits));
    }
    s += " UINT=" + String((uint32_t)unsignedValue);
    s += " INT=" + String(signedValue);
  }

  return s;
}

void addCanHistory(String frame) {
  canRxHistory[canRxHistoryIndex] = frame;
  canRxHistoryIndex = (canRxHistoryIndex + 1) % CAN_RX_HISTORY_MAX;
  if (canRxHistoryCount < CAN_RX_HISTORY_MAX) canRxHistoryCount++;
}

String canHistoryTableHtml() {
  String s;
  s += "<table><tr><th>Newest First</th><th>Received CAN Frame / Decoded Value</th></tr>";
  for (int n = 0; n < canRxHistoryCount; n++) {
    int idx = (int)canRxHistoryIndex - 1 - n;
    if (idx < 0) idx += CAN_RX_HISTORY_MAX;
    s += "<tr><td>" + String(n + 1) + "</td><td>" + esc(canRxHistory[idx]) + "</td></tr>";
  }
  if (canRxHistoryCount == 0) {
    s += "<tr><td colspan='2'>No CAN frames received yet.</td></tr>";
  }
  s += "</table>";
  return s;
}

// ---------------- CAN Bus / ESP32 TWAI ----------------

bool setupCAN() {
  if (!canEnable) return false;

  twai_timing_config_t t_config;
  if (canBitrate == 250000) t_config = TWAI_TIMING_CONFIG_250KBITS();
  else if (canBitrate == 125000) t_config = TWAI_TIMING_CONFIG_125KBITS();
  else if (canBitrate == 100000) t_config = TWAI_TIMING_CONFIG_100KBITS();
  else if (canBitrate == 800000) t_config = TWAI_TIMING_CONFIG_800KBITS();
  else if (canBitrate == 1000000) t_config = TWAI_TIMING_CONFIG_1MBITS();
  else t_config = TWAI_TIMING_CONFIG_500KBITS();

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)canTxPin, (gpio_num_t)canRxPin, canListenOnly ? TWAI_MODE_LISTEN_ONLY : TWAI_MODE_NORMAL);
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) return false;
  if (twai_start() != ESP_OK) return false;

  canStarted = true;
  return true;
}

void handleCANRuntime() {
  if (!canStarted) return;

  twai_message_t msg;
  while (twai_receive(&msg, 0) == ESP_OK) {
    canRxCount++;
    canLastFrame = canFrameToString(msg);
    addCanHistory(canLastFrame);
  }
}

bool canSendFrame(uint32_t id, const uint8_t *data, uint8_t len, bool ext) {
  if (!canStarted || canListenOnly) return false;
  if (len > 8) len = 8;

  twai_message_t msg = {};
  msg.identifier = id;
  msg.extd = ext ? 1 : 0;
  msg.rtr = 0;
  msg.data_length_code = len;
  for (int i = 0; i < len; i++) msg.data[i] = data[i];

  if (twai_transmit(&msg, pdMS_TO_TICKS(1000)) == ESP_OK) {
    canTxCount++;
    return true;
  }
  canErrCount++;
  return false;
}

uint8_t parseHexByte(String v) {
  v.trim();
  if (v.startsWith("0x") || v.startsWith("0X")) v = v.substring(2);
  return (uint8_t)strtoul(v.c_str(), NULL, 16);
}

void handleCANPage() {
  String sentMsg = "";

  if (server.hasArg("sendid") && server.hasArg("data")) {
    uint32_t id = strtoul(server.arg("sendid").c_str(), NULL, 16);
    String dataStr = server.arg("data");
    dataStr.replace(",", " ");
    uint8_t data[8];
    uint8_t len = 0;
    while (dataStr.length() && len < 8) {
      dataStr.trim();
      int sp = dataStr.indexOf(' ');
      String tok = sp >= 0 ? dataStr.substring(0, sp) : dataStr;
      if (tok.length()) data[len++] = parseHexByte(tok);
      if (sp < 0) break;
      dataStr = dataStr.substring(sp + 1);
    }
    bool ok = canSendFrame(id, data, len, server.hasArg("ext"));
    sentMsg = ok ? "<span class='on'>Frame sent.</span>" : "<span class='off'>Frame failed or CAN not started/listen-only.</span>";
  }

  String s = htmlStart("CAN Bus / TWAI");
  s += "<div class='card'>";
  s += "<b>Status:</b> " + String(canStarted ? "<span class='on'>Started</span>" : "<span class='off'>Stopped</span>") + "<br>";
  s += "<b>RX Count:</b> " + String(canRxCount) + "<br>";
  s += "<b>TX Count:</b> " + String(canTxCount) + "<br>";
  s += "<b>Error Count:</b> " + String(canErrCount) + "<br>";
  s += "<b>Last RX:</b> " + esc(canLastFrame) + "<br>";
  s += sentMsg;
  s += "</div>";

  s += "<div class='card'>";
  s += "<h3>Received CAN Frames</h3>";
  s += "This table updates when the page is refreshed. Use Listen Only mode if you only want to receive values from the connected CAN device.<br>";
  s += canHistoryTableHtml();
  s += "</div>";

  s += "<form class='card' method='post' action='/savecan'>";
  s += "<label><input type='checkbox' name='en' " + String(canEnable ? "checked" : "") + "> Enable CAN/TWAI</label><br>";
  s += "TX GPIO <input name='tx' value='" + String(canTxPin) + "'><br>";
  s += "RX GPIO <input name='rx' value='" + String(canRxPin) + "'><br>";
  s += "Bitrate <select name='baud'>";
  uint32_t rates[6] = {100000,125000,250000,500000,800000,1000000};
  for (int i = 0; i < 6; i++) s += "<option value='" + String(rates[i]) + "' " + String(canBitrate == rates[i] ? "selected" : "") + ">" + String(rates[i]) + "</option>";
  s += "</select><br>";
  s += "<label><input type='checkbox' name='listen' " + String(canListenOnly ? "checked" : "") + "> Listen Only</label><br>";
  s += "<button>Save CAN and Reboot</button></form>";

  s += "<form class='card' method='get' action='/can'>";
  s += "<h3>Send CAN Frame</h3>";
  s += "CAN ID hex <input name='sendid' value='123'><br>";
  s += "Data bytes hex <input name='data' value='01 02 03 04'><br>";
  s += "<label><input type='checkbox' name='ext'> Extended 29-bit ID</label><br>";
  s += "<button>Send Frame</button></form>";

  s += "<div class='card'><span class='warn'>ESP32-S3 has a TWAI/CAN controller, but you still need an external CAN transceiver wired to the selected TX/RX pins.</span></div>";
  s += htmlEnd();
  server.send(200, "text/html", s);
}

void saveCANPage() {
  prefs.begin("cfg", false);
  prefs.putBool("canEn", server.hasArg("en"));
  prefs.putInt("canTx", server.arg("tx").toInt());
  prefs.putInt("canRx", server.arg("rx").toInt());
  prefs.putUInt("canBaud", server.arg("baud").toInt());
  prefs.putBool("canListen", server.hasArg("listen"));
  prefs.end();
  server.send(200, "text/html", htmlStart("Saved") + "<div class='card'>CAN config saved. Rebooting...</div>" + htmlEnd());
  delay(800);
  ESP.restart();
}

// ---------------- HTTP Pages ----------------

void handleRoot() {
  String s = htmlStart("ESP32-S3-POE-ETH-8DI-8DO Industrial Server");
  s += "<div class='card'>";
  s += "<b>AP SSID:</b> " + apName() + "<br>";
  s += "<b>AP Password:</b> " + apPassword() + "<br>";
  s += "<b>AP IP:</b> " + wifiApIpString() + "<br>";
  s += "<b>STA IP:</b> " + WiFi.localIP().toString() + "<br>";
  s += "<b>WiFi AP MAC:</b> " + wifiApMacString() + "<br>";
  s += "<b>WiFi STA MAC:</b> " + wifiStaMacString() + "<br>";
  s += "<b>Ethernet IP:</b> " + String(ethernetStarted ? ipToString(Ethernet.localIP()) : "Not started") + "<br>";
  s += "<b>Ethernet MAC:</b> " + ethMacString() + "<br>";
  s += "<b>Ethernet Web:</b> " + String(ethernetStarted ? ("http://" + ipToString(Ethernet.localIP())) : "Not available") + "<br>";
  s += "<b>RTC:</b> " + rtcString() + "<br>";
  s += "<b>TCA9554PWR:</b> " + String(tcaOK ? "<span class='on'>OK</span>" : "<span class='off'>Not found</span>") + "<br>";
  s += "<b>PCF85063:</b> " + String(rtcOK ? "<span class='on'>OK</span>" : "<span class='off'>Not found</span>") + "<br>";
  s += "<b>SD:</b> " + String(sdOK ? "<span class='on'>Mounted</span>" : "<span class='off'>Not mounted</span>") + "<br>";
  s += "<b>MQTT:</b> " + String(mqttClient.connected() ? "<span class='on'>Connected</span>" : "<span class='off'>Disconnected</span>") + " via " + activeMqttIface + "<br>";
  s += "<b>DO1 WiFi Indicator:</b> " + String(wifiIndicatorEnable ? "Enabled" : "Disabled") + "<br>";
  s += "<b>DI Invert:</b> " + String(diInvertLogic ? "Enabled" : "Disabled") + "<br>";
  s += "<b>DO Invert:</b> " + String(doInvertLogic ? "Enabled" : "Disabled") + "<br>";
  s += "<b>WebSocket:</b> ws://" + wifiApIpString() + ":81<br>";
  s += "</div>";
  s += "<div class='card'><b>Modbus Holding Registers</b><br>";
  s += "0 = DI bitmask, 1 = DO bitmask, 2 = TCA OK, 3 = RTC OK, 4 = SD OK, 5 = WiFi OK, 6 = Ethernet OK, 7 = Free heap low word.<br>";
  s += "Write holding register 1 to control all outputs by bitmask.</div>";
  s += htmlEnd();
  server.send(200, "text/html", s);
}

void handleWifiPage() {
  String s = htmlStart("WiFi Configuration");
  s += "<form class='card' method='post' action='/savewifi'>";
  s += "Mode <select name='mode'><option value='AP' " + String(wifiMode == "AP" ? "selected" : "") + ">Hotspot/AP Only</option><option value='STA' " + String(wifiMode == "STA" ? "selected" : "") + ">AP + Connect to WiFi</option></select><br>";
  s += "STA SSID <input name='ssid' value='" + esc(staSsid) + "' autocomplete='off'><br>";
  s += "STA Password <input name='pass' type='password' value='" + esc(staPass) + "' autocomplete='new-password'><br>";
  s += "<label><input type='checkbox' name='wifiInd' " + String(wifiIndicatorEnable ? "checked" : "") + "> Use DO1 as WiFi STA connection indicator</label><br>";
  s += "<div class='card'>When enabled: DO1 blinks once per second until the board connects to the saved STA access point. DO1 stays ON after STA WiFi connects. The ESP32 fallback hotspot is only advertised while STA is disconnected; once STA connects, the fallback hotspot is shut off. STA reconnect is retried every 5 minutes.</div>";
  s += "<button>Save WiFi and Reboot</button></form>";
  s += "<div class='card'>";
  s += "<b>Currently saved STA SSID:</b> " + esc(staSsid) + "<br>";
  s += "<b>Current WiFi mode:</b> " + esc(wifiMode) + "<br>";
  s += "<b>STA connection status:</b> " + String(WiFi.status() == WL_CONNECTED ? "Connected" : "Not connected") + "<br>";
  s += "<b>STA IP:</b> " + WiFi.localIP().toString() + "<br>";
  s += "<b>Fallback AP running:</b> " + String(wifiApRunning() ? "Yes" : "No") + "<br>";
  s += "<b>Fallback AP SSID:</b> " + String(wifiApRunning() ? apName() : "Disabled") + "<br>";
  s += "<b>DO1 WiFi Indicator:</b> " + String(wifiIndicatorEnable ? "Enabled" : "Disabled") + "<br>";
  s += "</div>";
  s += htmlEnd();
  server.send(200, "text/html", s);
}

void saveWifi() {
  String newMode = server.arg("mode");
  String newSsid = server.arg("ssid");
  String newPass = server.arg("pass");

  newMode.trim();
  newSsid.trim();

  if (newMode != "AP" && newMode != "STA") newMode = "AP";

  prefs.begin("cfg", false);
  prefs.putString("wifiMode", newMode);
  prefs.putString("staSsid", newSsid);
  prefs.putString("staPass", newPass);
  prefs.putBool("wifiInd", server.hasArg("wifiInd"));
  prefs.end();

  String s = htmlStart("WiFi Saved");
  s += "<div class='card'>";
  s += "Saved WiFi mode: <b>" + esc(newMode) + "</b><br>";
  s += "Saved STA SSID: <b>" + esc(newSsid) + "</b><br>";
  s += "DO1 WiFi Indicator: <b>" + String(server.hasArg("wifiInd") ? "Enabled" : "Disabled") + "</b><br>";
  s += "Rebooting now so the new STA SSID is used...";
  s += "</div>";
  s += htmlEnd();

  server.send(200, "text/html", s);
  delay(1200);
  ESP.restart();
}

void handleEthernetPage() {
  String s = htmlStart("Ethernet / Modbus TCP");

  s += "<form class='card' method='post' action='/saveethernet'>";
  s += "<h3>Ethernet Network Configuration</h3>";
  s += "IP Mode <select name='mode'><option value='DHCP' " + String(ethMode == "DHCP" ? "selected" : "") + ">DHCP</option><option value='STATIC' " + String(ethMode == "STATIC" ? "selected" : "") + ">Static</option></select><br>";
  s += "IP <input name='ip' value='" + esc(ethIP) + "'><br>";
  s += "Gateway <input name='gw' value='" + esc(ethGW) + "'><br>";
  s += "Subnet Mask <input name='mask' value='" + esc(ethMask) + "'><br>";
  s += "DNS <input name='dns' value='" + esc(ethDNS) + "'><br>";
  s += "<button>Save Ethernet and Reboot</button>";
  s += "</form>";

  s += "<form class='card' method='post' action='/saveethernet'>";
  s += "<h3>Modbus TCP Server / Host Configuration</h3>";
  s += "<label><input type='checkbox' name='mbs' " + String(ethModbusServerEnable ? "checked" : "") + "> Enable Modbus TCP Server port 502</label><br>";
  s += "<label><input type='checkbox' name='mbc' " + String(ethModbusHostEnable ? "checked" : "") + "> Enable Modbus TCP Host/Client Polling</label><br>";
  s += "<input type='hidden' name='mode' value='" + esc(ethMode) + "'>";
  s += "<input type='hidden' name='ip' value='" + esc(ethIP) + "'>";
  s += "<input type='hidden' name='gw' value='" + esc(ethGW) + "'>";
  s += "<input type='hidden' name='mask' value='" + esc(ethMask) + "'>";
  s += "<input type='hidden' name='dns' value='" + esc(ethDNS) + "'>";
  s += "Host IP <input name='hip' value='" + esc(mbTcpHostIP) + "'><br>";
  s += "Host Port <input name='hport' value='" + String(mbTcpHostPort) + "'><br>";
  s += "Unit ID <input name='hunit' value='" + String(mbTcpHostUnit) + "'><br>";
  s += "Start Register <input name='hreg' value='" + String(mbTcpHostReg) + "'><br>";
  s += "Register Count <input name='hcnt' value='" + String(mbTcpHostCount) + "'><br>";
  s += "Poll ms <input name='hms' value='" + String(mbTcpPollMs) + "'><br>";
  s += "<button>Save Modbus TCP and Reboot</button>";
  s += "</form>";

  s += modbusRefreshInfoHtml();

  s += "<div class='card'>";
  s += "<h3>Modbus Register Monitor</h3>";
  s += "Active Ethernet IP: " + String(ethernetStarted ? ipToString(Ethernet.localIP()) : "Not started") + "<br>";
  s += "Formula examples: <b>x</b>, <b>x/10</b>, <b>x*1.8+32</b>, <b>(x-4000)*0.01</b><br>";
  s += "<b>Modbus CSV Log:</b> /modbus_log.csv<br>";
  s += modbusMonitorTableHtml(true);
  s += "</div>";

  s += "<form class='card' method='post' action='/savemblog'>";
  s += "<h3>Modbus CSV Logging</h3>";
  s += "<label><input type='checkbox' name='en' " + String(mbCsvLogEnable ? "checked" : "") + "> Enable Modbus CSV logging to /modbus_log.csv</label><br>";
  s += "Log interval ms <input name='ms' value='" + String(mbCsvLogMs) + "'><br>";
  s += "<a class='btn' href='/sd/download?file=/modbus_log.csv'>Download Modbus Log</a>";
  s += "<button>Save Modbus Logging</button>";
  s += "</form>";

  s += "<form class='card' method='post' action='/mbmon/add'>";
  s += "<h3>Manually Add Register To Monitor</h3>";
  s += "Register Address <input name='addr' required><br>";
  s += "Register Description <input name='desc' required><br>";
  s += "Math Formula <input name='formula' value='x'><br>";
  s += "<button>Add Register</button>";
  s += "</form>";

  s += "<form class='card' method='post' action='/mbmon/upload' enctype='multipart/form-data'>";
  s += "<h3>Upload Register CSV</h3>";
  s += "CSV format: <b>register address, register description</b><br>";
  s += "Optional third column: <b>math formula</b><br>";
  s += "<input type='file' name='upload' accept='.csv' required><br>";
  s += "<button>Upload CSV Register List</button>";
  s += "</form>";

  s += htmlEnd();
  server.send(200, "text/html", s);
}

void saveEthernet() {
  prefs.begin("cfg", false);
  prefs.putString("ethMode", server.arg("mode"));
  prefs.putString("ethIP", server.arg("ip"));
  prefs.putString("ethGW", server.arg("gw"));
  prefs.putString("ethMask", server.arg("mask"));
  prefs.putString("ethDNS", server.arg("dns"));
  prefs.putBool("ethMbs", server.hasArg("mbs"));
  prefs.putBool("ethMbc", server.hasArg("mbc"));
  prefs.putString("mbTcpIP", server.arg("hip"));
  prefs.putUShort("mbTcpPort", server.arg("hport").toInt());
  prefs.putUChar("mbTcpUnit", server.arg("hunit").toInt());
  prefs.putUShort("mbTcpReg", server.arg("hreg").toInt());
  prefs.putUShort("mbTcpCnt", server.arg("hcnt").toInt());
  prefs.putUInt("mbTcpMs", server.arg("hms").toInt());
  prefs.end();
  server.send(200, "text/html", htmlStart("Saved") + "<div class='card'>Ethernet saved. Rebooting...</div>" + htmlEnd());
  delay(800);
  ESP.restart();
}

void handleRS485Page() {
  String s = htmlStart("RS485 / Modbus RTU");
  s += "<form class='card' method='post' action='/savers485'>";
  s += "Baud <input name='baud' value='" + String(rs485Baud) + "'><br>";
  s += "Slave ID <input name='id' value='" + String(rs485Id) + "'><br>";
  s += "<label><input type='checkbox' name='mbs' " + String(rs485ModbusSlaveEnable ? "checked" : "") + "> Enable RTU Slave/Server</label><br>";
  s += "<label><input type='checkbox' name='mbc' " + String(rs485ModbusMasterEnable ? "checked" : "") + "> Enable RTU Master/Host Polling</label><br>";
  s += "Master Target ID <input name='mid' value='" + String(rtuMasterId) + "'><br>";
  s += "Start Register <input name='mreg' value='" + String(rtuMasterReg) + "'><br>";
  s += "Register Count <input name='mcnt' value='" + String(rtuMasterCount) + "'><br>";
  s += "Poll ms <input name='mms' value='" + String(rtuPollMs) + "'><br>";
  s += "<button>Save RS485 and Reboot</button></form>";
  s += htmlEnd();
  server.send(200, "text/html", s);
}

void saveRS485() {
  prefs.begin("cfg", false);
  prefs.putUInt("rbaud", server.arg("baud").toInt());
  prefs.putUChar("rid", server.arg("id").toInt());
  prefs.putBool("rMbs", server.hasArg("mbs"));
  prefs.putBool("rMbc", server.hasArg("mbc"));
  prefs.putUChar("rtuMid", server.arg("mid").toInt());
  prefs.putUShort("rtuMReg", server.arg("mreg").toInt());
  prefs.putUShort("rtuMCnt", server.arg("mcnt").toInt());
  prefs.putUInt("rtuMs", server.arg("mms").toInt());
  prefs.end();
  server.send(200, "text/html", htmlStart("Saved") + "<div class='card'>RS485 saved. Rebooting...</div>" + htmlEnd());
  delay(800);
  ESP.restart();
}

void handleIOPage() {
  if (server.hasArg("ch") && server.hasArg("state")) setOutput(server.arg("ch").toInt(), server.arg("state") == "1");
  refreshInputsOutputs();

  String s = htmlStart("IO Status / Names / Logging");
  s += "<h3>Live Inputs</h3><table><tr><th>Input</th><th>Name</th><th>GPIO</th><th>Raw GPIO</th><th>Logical Status</th></tr>";
  for (int i = 0; i < 8; i++) {
    bool raw = digitalRead(DI_PINS[i]);
    s += "<tr><td>DI" + String(i + 1) + "</td><td>" + esc(inputNames[i]) + "</td><td>GPIO" + String(DI_PINS[i]) + "</td><td>" + String(raw ? "HIGH" : "LOW") + "</td><td>" + String(inState[i] ? "<span class='on'>ON/HIGH</span>" : "<span class='off'>OFF/LOW</span>") + "</td></tr>";
  }
  s += "</table><h3>Outputs</h3><table><tr><th>Output</th><th>Name</th><th>Status</th><th>Control</th></tr>";
  for (int i = 0; i < 8; i++) {
    s += "<tr><td>DO" + String(i + 1) + "</td><td>" + esc(outputNames[i]) + "</td><td>" + String(outState[i] ? "<span class='on'>ON</span>" : "<span class='off'>OFF</span>") + "</td><td>";
    s += "<a class='btn' href='/io?ch=" + String(i + 1) + "&state=1'>ON</a><a class='btn' href='/io?ch=" + String(i + 1) + "&state=0'>OFF</a></td></tr>";
  }
  s += "</table>";

  s += "<form class='card' method='post' action='/saveio'><h3>Persistent IO Names and CSV Logging</h3>";
  for (int i = 0; i < 8; i++) {
    s += "DI" + String(i + 1) + " Name <input name='in" + String(i + 1) + "' value='" + esc(inputNames[i]) + "'> ";
    s += "DO" + String(i + 1) + " Name <input name='out" + String(i + 1) + "' value='" + esc(outputNames[i]) + "'><br>";
  }
  s += "<label><input type='checkbox' name='csvEn' " + String(csvLogEnable ? "checked" : "") + "> Enable snapshot CSV logging to /io_log.csv</label><br>";
  s += "Snapshot log interval ms <input name='csvMs' value='" + String(csvLogMs) + "'><br>";
  s += "<label><input type='checkbox' name='ioEvtEn' " + String(ioEventLogEnable ? "checked" : "") + "> Enable IO event logging to daily /io_events_YYYY-MM-DD.csv files</label><br>";
  s += "<h3>IO Polarity / Reverse Logic</h3>";
  s += "<label><input type='checkbox' name='diInv' " + String(diInvertLogic ? "checked" : "") + "> Invert DI logic for display/logging/Modbus</label><br>";
  s += "<label><input type='checkbox' name='doInv' " + String(doInvertLogic ? "checked" : "") + "> Invert DO logic for TCA9554 outputs / relay LEDs</label><br>";
  s += "<div class='card'>Your observed DO LEDs being ON while the web table says OFF indicates active-low output logic. This firmware defaults DO invert ON so OFF writes HIGH to the TCA9554 expander. DI8 staying LOW while DI1-DI7 stay HIGH may be field wiring, input pull state, or active-low input hardware; use DI invert if the field logic is backwards.</div>";
  s += "<div class='card'>IO event logging records ON and OFF interval durations for DI1-DI8 and DO1-DO8. A TOTALIZER section is appended when the day rolls over.</div>";
  s += "<button>Save IO Config</button></form>";

  s += "<div class='card'><h3>WebSocket Live JSON</h3>";
  s += "<pre id='ws'>Connecting...</pre><script>";
  s += "let ws=new WebSocket('ws://'+location.hostname+':81');";
  s += "ws.onmessage=e=>{document.getElementById('ws').textContent=e.data;};";
  s += "</script></div>";
  s += htmlEnd();
  server.send(200, "text/html", s);
}

void saveIOPage() {
  saveIOConfigFromWeb();
  server.send(200, "text/html", htmlStart("Saved") + "<div class='card'>IO names and logging settings saved. <a class='btn' href='/io'>Back</a></div>" + htmlEnd());
}

void handleSDPage() {
  String s = htmlStart("SD File Manager");
  s += "<div class='card'>SD status: " + String(sdOK ? "<span class='on'>Mounted</span>" : "<span class='off'>Not mounted</span>") + "</div>";
  s += "<form class='card' method='post' action='/sd/upload' enctype='multipart/form-data'>";
  s += "<input type='file' name='upload'><button>Upload</button></form>";

  if (sdOK) {
    File root = SD_MMC.open("/");
    s += "<table><tr><th>Name</th><th>Type</th><th>Size</th><th>Actions</th></tr>";
    File f = root.openNextFile();
    while (f) {
      String name = String(f.name());
      String p = name.startsWith("/") ? name : "/" + name;
      s += "<tr><td>" + esc(p) + "</td><td>" + String(f.isDirectory() ? "DIR" : "FILE") + "</td><td>" + String(f.size()) + "</td><td>";
      if (!f.isDirectory()) {
        s += "<a class='btn' href='/sd/download?file=" + esc(p) + "'>Download</a>";
        s += "<a class='btn' href='/sd/delete?file=" + esc(p) + "' onclick='return confirm(\"Delete file?\")'>Delete</a>";
      }
      s += "</td></tr>";
      f = root.openNextFile();
    }
    s += "</table>";
  }

  s += htmlEnd();
  server.send(200, "text/html", s);
}

File uploadFile;

void handleSDUpload() {
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    String filename = "/" + upload.filename;
    uploadFile = SD_MMC.open(filename, FILE_WRITE);
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (uploadFile) uploadFile.write(upload.buf, upload.currentSize);
  } else if (upload.status == UPLOAD_FILE_END) {
    if (uploadFile) uploadFile.close();
  }
}

void handleSDUploadDone() {
  server.send(200, "text/html", htmlStart("Uploaded") + "<div class='card'>Upload complete. <a class='btn' href='/sd'>Back</a></div>" + htmlEnd());
}

void handleSDDownload() {
  if (!sdOK || !server.hasArg("file")) {
    server.send(404, "text/plain", "File not found");
    return;
  }
  String file = urlDecode(server.arg("file"));
  if (!file.startsWith("/")) file = "/" + file;
  if (!SD_MMC.exists(file)) {
    server.send(404, "text/plain", "File not found");
    return;
  }
  File f = SD_MMC.open(file, FILE_READ);
  server.streamFile(f, "application/octet-stream");
  f.close();
}

void handleSDDelete() {
  if (sdOK && server.hasArg("file")) {
    String file = urlDecode(server.arg("file"));
    if (!file.startsWith("/")) file = "/" + file;
    SD_MMC.remove(file);
  }
  server.sendHeader("Location", "/sd");
  server.send(302, "text/plain", "");
}

void handleMqttPage() {
  String s = htmlStart("MQTT Configuration");
  s += "<form class='card' method='post' action='/savemqtt'>";
  s += "<label><input type='checkbox' name='en' " + String(mqttEnable ? "checked" : "") + "> Enable MQTT</label><br>";
  s += "Network Interface <select name='iface'>";
  s += "<option value='AUTO' " + String(mqttIface == "AUTO" ? "selected" : "") + ">Auto</option>";
  s += "<option value='WIFI' " + String(mqttIface == "WIFI" ? "selected" : "") + ">WiFi</option>";
  s += "<option value='ETH' " + String(mqttIface == "ETH" ? "selected" : "") + ">Ethernet/W5500</option>";
  s += "</select><br>";
  s += "Broker Host <input name='host' value='" + esc(mqttHost) + "'><br>";
  s += "Broker Port <input name='port' value='" + String(mqttPort) + "'><br>";
  s += "Username <input name='user' value='" + esc(mqttUser) + "'><br>";
  s += "Password <input name='pass' type='password' value='" + esc(mqttPass) + "'><br>";
  s += "Base Topic <input name='topic' value='" + esc(mqttBaseTopic) + "'><br>";
  s += "<button>Save MQTT and Reboot</button></form>";
  s += "<div class='card'>Commands: " + esc(mqttBaseTopic) + "/cmd/output payload <b>ch,state</b>, or /cmd/output_mask payload decimal mask.</div>";
  s += htmlEnd();
  server.send(200, "text/html", s);
}

void saveMqtt() {
  prefs.begin("cfg", false);
  prefs.putBool("mqttEn", server.hasArg("en"));
  prefs.putString("mqttIf", server.arg("iface"));
  prefs.putString("mqttHost", server.arg("host"));
  prefs.putUShort("mqttPort", server.arg("port").toInt());
  prefs.putString("mqttUser", server.arg("user"));
  prefs.putString("mqttPass", server.arg("pass"));
  prefs.putString("mqttTopic", server.arg("topic"));
  prefs.end();
  server.send(200, "text/html", htmlStart("Saved") + "<div class='card'>MQTT saved. Rebooting...</div>" + htmlEnd());
  delay(800);
  ESP.restart();
}

void handleTimePage() {
  String s = htmlStart("Time / NTP / RTC");
  s += "<div class='card'><b>RTC Time:</b> " + rtcString() + "</div>";
  s += "<form class='card' method='post' action='/savetime'>";
  s += "<label><input type='checkbox' name='ntp' " + String(ntpEnable ? "checked" : "") + "> Enable NTP to RTC sync</label><br>";
  s += "Network Interface <select name='iface'>";
  s += "<option value='AUTO' " + String(ntpIface == "AUTO" ? "selected" : "") + ">Auto</option>";
  s += "<option value='WIFI' " + String(ntpIface == "WIFI" ? "selected" : "") + ">WiFi</option>";
  s += "<option value='ETH' " + String(ntpIface == "ETH" ? "selected" : "") + ">Ethernet/W5500</option>";
  s += "</select><br>";
  s += "NTP Server <input name='srv' value='" + esc(ntpServer) + "'><br>";
  s += "GMT Offset Seconds <input name='gmt' value='" + String(gmtOffsetSec) + "'><br>";
  s += "DST Offset Seconds <input name='dst' value='" + String(daylightOffsetSec) + "'><br>";
  s += "<button>Save Time Config</button></form>";
  s += "<form class='card' method='post' action='/ntpsync'><button>Sync RTC From NTP Now</button></form>";
  s += htmlEnd();
  server.send(200, "text/html", s);
}

void saveTimePage() {
  prefs.begin("cfg", false);
  prefs.putBool("ntpEn", server.hasArg("ntp"));
  prefs.putString("ntpIf", server.arg("iface"));
  prefs.putString("ntpSrv", server.arg("srv"));
  prefs.putLong("gmtOff", server.arg("gmt").toInt());
  prefs.putInt("dstOff", server.arg("dst").toInt());
  prefs.end();
  server.send(200, "text/html", htmlStart("Saved") + "<div class='card'>Time config saved. Rebooting...</div>" + htmlEnd());
  delay(800);
  ESP.restart();
}

void handleNtpSyncNow() {
  bool ok = syncRtcFromNtp();
  server.send(200, "text/html", htmlStart("NTP Sync") + "<div class='card'>NTP sync: " + String(ok ? "<span class='on'>OK</span>" : "<span class='off'>FAILED</span>") + "<br>RTC: " + rtcString() + "</div>" + htmlEnd());
}

void handleOTAPage() {
  String s = htmlStart("WiFi OTA Firmware Update");

  s += "<div class='card'>";
  s += "<b>This OTA updater is served through the ESP32 WiFi web server only.</b><br>";
  s += "Use it from the board hotspot or from the STA WiFi IP after the board connects to your WiFi network.<br><br>";
  s += "<b>Hotspot SSID:</b> " + apName() + "<br>";
  s += "<b>Hotspot Password:</b> " + apPassword() + "<br>";
  s += "<b>Hotspot OTA URL:</b> http://" + wifiApIpString() + "/ota<br>";
  s += "<b>STA OTA URL:</b> http://" + WiFi.localIP().toString() + "/ota<br>";
  s += "</div>";

  s += "<form class='card' method='post' action='/ota/upload' enctype='multipart/form-data'>";
  s += "<h3>Upload New Firmware .bin</h3>";
  s += "<input type='file' name='firmware' accept='.bin' required><br>";
  s += "<button onclick='return confirm(\"Upload firmware and reboot device?\")'>Upload Firmware Over WiFi</button>";
  s += "</form>";

  s += "<div class='card'>";
  s += "<span class='warn'>Use Arduino IDE: Sketch &gt; Export Compiled Binary, then upload the generated .bin file here.</span><br>";
  s += "Do not remove power during the upload.";
  s += "</div>";

  s += htmlEnd();
  server.send(200, "text/html", s);
}

void handleOTAUpload() {
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    Update.begin(UPDATE_SIZE_UNKNOWN);
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    Update.write(upload.buf, upload.currentSize);
  } else if (upload.status == UPLOAD_FILE_END) {
    Update.end(true);
  }
}

void handleOTADone() {
  bool ok = !Update.hasError();

  String s = htmlStart("WiFi OTA Result");
  s += "<div class='card'>";

  if (ok) {
    s += "<span class='on'>OTA firmware upload complete.</span><br>";
    s += "Device is rebooting now.";
  } else {
    s += "<span class='off'>OTA firmware upload failed.</span><br>";
    s += "Update error code: " + String(Update.getError()) + "<br>";
    s += "<a class='btn' href='/ota'>Back to OTA Page</a>";
  }

  s += "</div>";
  s += htmlEnd();

  server.send(200, "text/html", s);

  if (ok) {
    delay(1000);
    ESP.restart();
  }
}

// ---------------- REST API ----------------

void apiStatus() {
  server.send(200, "application/json", statusJson());
}

void apiOutput() {
  if (server.hasArg("ch") && server.hasArg("state")) setOutput(server.arg("ch").toInt(), server.arg("state") == "1");
  server.send(200, "application/json", statusJson());
}

void apiOutputMask() {
  if (server.hasArg("mask")) applyOutputMask(server.arg("mask").toInt());
  server.send(200, "application/json", statusJson());
}

void apiRegs() {
  refreshInputsOutputs();
  String s = "{\"holding\":[";
  for (int i = 0; i < 128; i++) {
    if (i) s += ",";
    s += String(holdingRegs[i]);
  }
  s += "]}";
  server.send(200, "application/json", s);
}


// ---------------- Ethernet Lightweight Web Interface ----------------
//
// NOTE:
// The Arduino WebServer object used by the WiFi/AP side cannot directly serve
// W5500 EthernetClient sockets. This section is a separate lightweight HTTP
// router for the W5500 side, with matching pages for every menu route.

EthernetServer ethWebServer(80);

String ethHtmlStart(const String &title) {
  String s;
  s += "<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>";
  s += "<title>" + esc(title) + "</title>";
  s += "<style>";
  s += "body{font-family:Arial;background:#111;color:#eee;margin:0;padding:18px;}";
  s += ".card{background:#1d1d1d;border:1px solid #444;border-radius:12px;padding:16px;margin:12px 0;}";
  s += "a,button,input,select{font-size:16px;margin:4px;padding:9px;border-radius:8px;}";
  s += "a.btn,button{background:#ff9900;color:#000;text-decoration:none;border:0;font-weight:bold;display:inline-block;}";
  s += "input,select{background:#222;color:#fff;border:1px solid #666;max-width:95%;}";
  s += "table{border-collapse:collapse;width:100%;margin-top:8px;}";
  s += "td,th{border:1px solid #555;padding:8px;text-align:left;}";
  s += ".on{color:#55ff55;font-weight:bold;}.off{color:#ff5555;font-weight:bold;}.warn{color:#ffff66;font-weight:bold;}";
  s += "</style></head><body>";
  s += "<h2>" + esc(title) + "</h2><p>";
  s += "<a class='btn' href='/'>Home</a>";
  s += "<a class='btn' href='/wifi'>WiFi</a>";
  s += "<a class='btn' href='/ethernet'>Ethernet</a>";
  s += "<a class='btn' href='/rs485'>RS485</a>";
  s += "<a class='btn' href='/sd'>SD</a>";
  s += "<a class='btn' href='/io'>IO</a>";
  s += "<a class='btn' href='/can'>CAN</a>";
  s += "<a class='btn' href='/mqtt'>MQTT</a>";
  s += "<a class='btn' href='/time'>Time/NTP</a>";
  s += "<a class='btn' href='/ota'>WiFi OTA</a>";
  s += "<a class='btn' href='/json'>JSON</a>";
  s += "</p>";
  return s;
}

String ethHtmlEnd() {
  return "</body></html>";
}

String ethBanner() {
  String s;
  s += "<div class='card'>";
  s += "<b>This page is being served by the W5500 Ethernet interface.</b><br>";
  s += "<b>Ethernet IP:</b> " + String(ethernetStarted ? ipToString(Ethernet.localIP()) : "Not started") + "<br>";
  s += "<b>Ethernet MAC:</b> " + ethMacString() + "<br>";
  s += "<b>WiFi AP:</b> " + apName() + " / " + apPassword() + "<br>";
  s += "<b>WiFi AP MAC:</b> " + wifiApMacString() + "<br>";
  s += "<b>WiFi STA MAC:</b> " + wifiStaMacString() + "<br>";
  s += "<b>WiFi AP URL:</b> http://" + wifiApIpString() + "/<br>";
  s += "<b>STA IP:</b> " + WiFi.localIP().toString() + "<br>";
  s += "<b>RTC:</b> " + rtcString() + "<br>";
  s += "</div>";
  return s;
}

String ethHomePage() {
  refreshInputsOutputs();
  String s = ethHtmlStart("Ethernet Home Interface");
  s += ethBanner();
  s += "<div class='card'>";
  s += "<b>System Status</b><br>";
  s += "The Ethernet web server has its own route handler and each menu button now maps to its matching Ethernet page.<br>";
  s += "For full firmware upload, use the WiFi OTA page because browser file uploads are safest through the ESP32 WiFi WebServer parser.";
  s += "</div>";
  s += "<table><tr><th>Feature</th><th>Status</th></tr>";
  s += "<tr><td>Modbus TCP Server</td><td>" + String(ethModbusServerEnable ? "Enabled" : "Disabled") + "</td></tr>";
  s += "<tr><td>Modbus TCP Host</td><td>" + String(ethModbusHostEnable ? "Enabled" : "Disabled") + "</td></tr>";
  s += "<tr><td>RS485 RTU Slave</td><td>" + String(rs485ModbusSlaveEnable ? "Enabled" : "Disabled") + "</td></tr>";
  s += "<tr><td>RS485 RTU Master</td><td>" + String(rs485ModbusMasterEnable ? "Enabled" : "Disabled") + "</td></tr>";
  s += "<tr><td>MQTT</td><td>" + String(mqttEnable ? "Enabled" : "Disabled") + "</td></tr>";
  s += "<tr><td>NTP</td><td>" + String(ntpEnable ? "Enabled" : "Disabled") + "</td></tr>";
  s += "<tr><td>SD</td><td>" + String(sdOK ? "Mounted" : "Not mounted") + "</td></tr>";
  s += "</table>";
  s += ethHtmlEnd();
  return s;
}

String ethWifiPage() {
  String s = ethHtmlStart("Ethernet WiFi Configuration View");
  s += ethBanner();
  s += "<div class='card'>";
  s += "<b>Current WiFi Settings</b><br>";
  s += "Mode: " + esc(wifiMode) + "<br>";
  s += "STA SSID: " + esc(staSsid) + "<br>";
  s += "STA IP: " + WiFi.localIP().toString() + "<br>";
  s += "AP SSID: " + apName() + "<br>";
  s += "AP Password: " + apPassword() + "<br><br>";
  s += "<span class='warn'>Use the WiFi web interface for editing WiFi settings so you do not accidentally lock yourself out over Ethernet.</span>";
  s += "</div>";
  s += ethHtmlEnd();
  return s;
}

String ethEthernetPage() {
  String s = ethHtmlStart("Ethernet Configuration Interface");
  s += ethBanner();

  s += "<div class='card'>";
  s += "<h3>Ethernet Network Configuration</h3>";
  s += "Mode: " + esc(ethMode) + "<br>";
  s += "Configured IP: " + esc(ethIP) + "<br>";
  s += "Gateway: " + esc(ethGW) + "<br>";
  s += "Subnet: " + esc(ethMask) + "<br>";
  s += "DNS: " + esc(ethDNS) + "<br>";
  s += "Active Ethernet IP: " + String(ethernetStarted ? ipToString(Ethernet.localIP()) : "Not started") + "<br>";
  s += "</div>";

  s += "<div class='card'>";
  s += "<h3>Modbus TCP Server / Host Configuration</h3>";
  s += "<table><tr><th>Function</th><th>Status / Value</th></tr>";
  s += "<tr><td>Modbus TCP Server Port 502</td><td>" + String(ethModbusServerEnable ? "Enabled" : "Disabled") + "</td></tr>";
  s += "<tr><td>Modbus TCP Host Polling</td><td>" + String(ethModbusHostEnable ? "Enabled" : "Disabled") + "</td></tr>";
  s += "<tr><td>TCP Host IP</td><td>" + esc(mbTcpHostIP) + "</td></tr>";
  s += "<tr><td>TCP Host Port</td><td>" + String(mbTcpHostPort) + "</td></tr>";
  s += "<tr><td>Unit ID</td><td>" + String(mbTcpHostUnit) + "</td></tr>";
  s += "<tr><td>Start Register</td><td>" + String(mbTcpHostReg) + "</td></tr>";
  s += "<tr><td>Register Count</td><td>" + String(mbTcpHostCount) + "</td></tr>";
  s += "</table>";
  s += "</div>";

  s += modbusRefreshInfoHtml();

  s += "<div class='card'>";
  s += "<h3>Modbus Register Monitor</h3>";
  s += modbusMonitorTableHtml(false);
  s += "<br><span class='warn'>Manual register edits and CSV upload are available from the WiFi-served Ethernet page.</span>";
  s += "</div>";

  s += ethHtmlEnd();
  return s;
}

String ethRS485Page() {
  String s = ethHtmlStart("Ethernet RS485 Interface");
  s += ethBanner();
  s += "<div class='card'>";
  s += "<b>RS485 Settings</b><br>";
  s += "TX GPIO17, RX GPIO18, RTS/DE GPIO21<br>";
  s += "Baud: " + String(rs485Baud) + "<br>";
  s += "Local Slave ID: " + String(rs485Id) + "<br>";
  s += "RTU Slave/Server: " + String(rs485ModbusSlaveEnable ? "Enabled" : "Disabled") + "<br>";
  s += "RTU Master/Host: " + String(rs485ModbusMasterEnable ? "Enabled" : "Disabled") + "<br>";
  s += "Master Target ID: " + String(rtuMasterId) + "<br>";
  s += "Master Start Register: " + String(rtuMasterReg) + "<br>";
  s += "Master Count: " + String(rtuMasterCount) + "<br>";
  s += "Master Poll ms: " + String(rtuPollMs) + "<br>";
  s += "</div>";
  s += ethHtmlEnd();
  return s;
}

String ethSDPage() {
  String s = ethHtmlStart("Ethernet SD Interface");
  s += ethBanner();
  s += "<div class='card'>SD status: " + String(sdOK ? "<span class='on'>Mounted</span>" : "<span class='off'>Not mounted</span>") + "</div>";
  if (sdOK) {
    File root = SD_MMC.open("/");
    s += "<table><tr><th>Name</th><th>Type</th><th>Size</th></tr>";
    File f = root.openNextFile();
    while (f) {
      String name = String(f.name());
      String p = name.startsWith("/") ? name : "/" + name;
      s += "<tr><td>" + esc(p) + "</td><td>" + String(f.isDirectory() ? "DIR" : "FILE") + "</td><td>" + String(f.size()) + "</td></tr>";
      f = root.openNextFile();
    }
    s += "</table>";
  }
  s += "<div class='card'><span class='warn'>Upload, download, and delete are available on the WiFi web interface.</span></div>";
  s += ethHtmlEnd();
  return s;
}

String ethIOPage() {
  refreshInputsOutputs();
  String s = ethHtmlStart("Ethernet IO Interface");
  s += ethBanner();

  s += "<h3>Inputs</h3><table><tr><th>Input</th><th>Name</th><th>Status</th></tr>";
  for (int i = 0; i < 8; i++) {
    s += "<tr><td>DI" + String(i + 1) + "</td><td>" + esc(inputNames[i]) + "</td><td>" + String(inState[i] ? "<span class='on'>HIGH</span>" : "<span class='off'>LOW</span>") + "</td></tr>";
  }
  s += "</table>";

  s += "<h3>Outputs</h3><table><tr><th>Output</th><th>Name</th><th>Status</th><th>Control</th></tr>";
  for (int i = 0; i < 8; i++) {
    s += "<tr><td>DO" + String(i + 1) + "</td><td>" + esc(outputNames[i]) + "</td><td>" + String(outState[i] ? "<span class='on'>ON</span>" : "<span class='off'>OFF</span>") + "</td><td>";
    s += "<a class='btn' href='/eth/output?ch=" + String(i + 1) + "&state=1'>ON</a>";
    s += "<a class='btn' href='/eth/output?ch=" + String(i + 1) + "&state=0'>OFF</a>";
    s += "</td></tr>";
  }
  s += "</table>";
  s += ethHtmlEnd();
  return s;
}

String ethCANPage() {
  String s = ethHtmlStart("Ethernet CAN Interface");
  s += ethBanner();
  s += "<div class='card'>";
  s += "<b>CAN / TWAI Status</b><br>";
  s += "This page is the Ethernet view of the CAN configuration/status page.<br>";
  s += "ESP32-S3 TWAI requires an external CAN transceiver connected to the configured TX/RX pins.<br>";
  s += "Use the WiFi page for editable CAN settings if enabled in this build.";
  s += "</div>";
  s += ethHtmlEnd();
  return s;
}

String ethMQTTPage() {
  String s = ethHtmlStart("Ethernet MQTT Interface");
  s += ethBanner();
  s += "<div class='card'>";
  s += "<b>MQTT Settings</b><br>";
  s += "Enabled: " + String(mqttEnable ? "Yes" : "No") + "<br>";
  s += "Broker: " + esc(mqttHost) + ":" + String(mqttPort) + "<br>";
  s += "Base Topic: " + esc(mqttBaseTopic) + "<br>";
  s += "Connected: " + String(mqttClient.connected() ? "<span class='on'>Yes</span>" : "<span class='off'>No</span>") + "<br>";
  s += "</div>";
  s += ethHtmlEnd();
  return s;
}

String ethTimePage() {
  String s = ethHtmlStart("Ethernet Time/NTP Interface");
  s += ethBanner();
  s += "<div class='card'>";
  s += "<b>Time/NTP Settings</b><br>";
  s += "RTC: " + rtcString() + "<br>";
  s += "NTP Enabled: " + String(ntpEnable ? "Yes" : "No") + "<br>";
  s += "NTP Server: " + esc(ntpServer) + "<br>";
  s += "GMT Offset Seconds: " + String(gmtOffsetSec) + "<br>";
  s += "DST Offset Seconds: " + String(daylightOffsetSec) + "<br>";
  s += "</div>";
  s += ethHtmlEnd();
  return s;
}

String ethOTAPage() {
  String s = ethHtmlStart("Ethernet WiFi OTA Interface");
  s += ethBanner();
  s += "<div class='card'>";
  s += "<b>OTA is intentionally handled by the WiFi WebServer route.</b><br>";
  s += "Connect to the board hotspot or STA WiFi IP and open:<br>";
  s += "<b>http://" + wifiApIpString() + "/ota</b><br>";
  s += "<span class='warn'>The W5500 lightweight server does not process multipart firmware uploads.</span>";
  s += "</div>";
  s += ethHtmlEnd();
  return s;
}

String ethJSONPage() {
  String s = ethHtmlStart("Ethernet JSON Interface");
  s += ethBanner();
  s += "<div class='card'>";
  s += "Status JSON: <b>/api/status</b><br>";
  s += "Output Control: <b>/api/output?ch=1&state=1</b><br>";
  s += "Output Mask: <b>/api/output_mask?mask=255</b><br>";
  s += "</div><pre class='card'>";
  s += esc(statusJson());
  s += "</pre>";
  s += ethHtmlEnd();
  return s;
}

String getPathOnly(String requestTarget) {
  int sp = requestTarget.indexOf(' ');
  if (sp >= 0) requestTarget = requestTarget.substring(0, sp);
  int q = requestTarget.indexOf('?');
  if (q >= 0) return requestTarget.substring(0, q);
  return requestTarget;
}

String getQueryValue(String target, const String &key) {
  int q = target.indexOf('?');
  if (q < 0) return "";
  String query = target.substring(q + 1);
  int start = 0;
  while (start < query.length()) {
    int amp = query.indexOf('&', start);
    if (amp < 0) amp = query.length();
    String pair = query.substring(start, amp);
    int eq = pair.indexOf('=');
    if (eq > 0) {
      String k = pair.substring(0, eq);
      String v = pair.substring(eq + 1);
      if (k == key) return urlDecode(v);
    }
    start = amp + 1;
  }
  return "";
}

void sendEthHttp(EthernetClient &c, const String &type, const String &body) {
  c.println("HTTP/1.1 200 OK");
  c.println("Connection: close");
  c.println("Cache-Control: no-store");
  c.print("Content-Type: ");
  c.println(type);
  c.print("Content-Length: ");
  c.println(body.length());
  c.println();

  const size_t chunk = 512;
  for (size_t i = 0; i < body.length(); i += chunk) {
    c.print(body.substring(i, min(i + chunk, (size_t)body.length())));
  }
}

void sendEthRedirect(EthernetClient &c, const String &location) {
  c.println("HTTP/1.1 302 Found");
  c.println("Connection: close");
  c.print("Location: ");
  c.println(location);
  c.println();
}

void handleEthernetWebClient() {
  if (!ethernetStarted) return;

  EthernetClient c = ethWebServer.available();
  if (!c) return;

  String reqLine = "";
  unsigned long start = millis();

  while (c.connected() && millis() - start < 1000) {
    if (c.available()) {
      char ch = c.read();
      if (ch == '\n') break;
      if (ch != '\r') reqLine += ch;
    }
  }

  while (c.available()) c.read();

  // Expected request line: GET /path?x=y HTTP/1.1
  String target = "/";
  if (reqLine.startsWith("GET ")) {
    int s1 = reqLine.indexOf(' ');
    int s2 = reqLine.indexOf(' ', s1 + 1);
    if (s1 >= 0 && s2 > s1) target = reqLine.substring(s1 + 1, s2);
  }

  String path = getPathOnly(target);

  if (path == "/api/status" || path == "/api/io") {
    sendEthHttp(c, "application/json", statusJson());

  } else if (path == "/api/registers") {
    refreshInputsOutputs();
    String js = "{\"holding\":[";
    for (int i = 0; i < 128; i++) {
      if (i) js += ",";
      js += String(holdingRegs[i]);
    }
    js += "]}";
    sendEthHttp(c, "application/json", js);

  } else if (path == "/eth/output" || path == "/api/output") {
    int ch = getQueryValue(target, "ch").toInt();
    int state = getQueryValue(target, "state").toInt();
    if (ch >= 1 && ch <= 8) setOutput(ch, state == 1);
    if (path == "/api/output") sendEthHttp(c, "application/json", statusJson());
    else sendEthRedirect(c, "/io");

  } else if (path == "/api/output_mask") {
    int mask = getQueryValue(target, "mask").toInt();
    applyOutputMask(mask);
    sendEthHttp(c, "application/json", statusJson());

  } else if (path == "/" || path == "/home") {
    sendEthHttp(c, "text/html", ethHomePage());

  } else if (path == "/wifi") {
    sendEthHttp(c, "text/html", ethWifiPage());

  } else if (path == "/ethernet") {
    sendEthHttp(c, "text/html", ethEthernetPage());

  } else if (path == "/rs485") {
    sendEthHttp(c, "text/html", ethRS485Page());

  } else if (path == "/sd") {
    sendEthHttp(c, "text/html", ethSDPage());

  } else if (path == "/io") {
    sendEthHttp(c, "text/html", ethIOPage());

  } else if (path == "/can") {
    sendEthHttp(c, "text/html", ethCANPage());

  } else if (path == "/mqtt") {
    sendEthHttp(c, "text/html", ethMQTTPage());

  } else if (path == "/time" || path == "/ntp") {
    sendEthHttp(c, "text/html", ethTimePage());

  } else if (path == "/ota" || path == "/wifiota") {
    sendEthHttp(c, "text/html", ethOTAPage());

  } else if (path == "/json") {
    sendEthHttp(c, "text/html", ethJSONPage());

  } else {
    sendEthHttp(c, "text/html", ethHomePage());
  }

  delay(2);
  c.stop();
}

void setupEthernetWebServer() {
  if (ethernetStarted) {
    ethWebServer.begin();
  }
}



// ---------------- Modbus Monitor Web Handlers ----------------

File mbCsvUploadFile;

void handleMbMonAdd() {
  uint16_t addr = server.arg("addr").toInt();
  String desc = server.arg("desc");
  String formula = server.arg("formula");

  addModbusMonitorRegister(addr, desc, formula);

  server.sendHeader("Location", "/ethernet");
  server.send(302, "text/plain", "");
}

void handleMbMonDelete() {
  uint8_t index = server.arg("i").toInt();
  deleteModbusMonitorRegister(index);

  server.sendHeader("Location", "/ethernet");
  server.send(302, "text/plain", "");
}

void handleMbMonDeleteSelected() {
  String action = server.arg("action");

  if (action == "delete_all") {
    deleteAllModbusMonitorRegisters();
  } else {
    for (int i = mbMonCount - 1; i >= 0; i--) {
      if (server.hasArg("sel" + String(i))) {
        deleteModbusMonitorRegister(i);
      }
    }
    saveModbusMonitorConfig();
  }

  server.sendHeader("Location", "/ethernet");
  server.send(302, "text/plain", "");
}

String cleanCsvField(String v) {
  v.trim();

  // Remove UTF-8 BOM if the CSV came from Excel or Windows.
  while (v.length() && ((uint8_t)v[0] == 0xEF || (uint8_t)v[0] == 0xBB || (uint8_t)v[0] == 0xBF)) {
    v.remove(0, 1);
  }

  // Remove common Unicode BOM when represented as one code point.
  if (v.length() && v[0] == 0xFEFF) {
    v.remove(0, 1);
  }

  if (v.startsWith("\"") && v.endsWith("\"") && v.length() >= 2) {
    v.remove(0, 1);
    v.remove(v.length() - 1);
  }

  v.replace("\"\"", "\"");
  v.trim();
  return v;
}

String csvField(String line, int fieldIndex, char delim) {
  int current = 0;
  bool quoted = false;
  String field = "";

  for (int i = 0; i < line.length(); i++) {
    char ch = line[i];

    if (ch == '"') {
      quoted = !quoted;
      field += ch;
    } else if (ch == delim && !quoted) {
      if (current == fieldIndex) return cleanCsvField(field);
      field = "";
      current++;
    } else {
      field += ch;
    }
  }

  if (current == fieldIndex) return cleanCsvField(field);
  return "";
}

bool csvFirstFieldIsNumericAddress(String f) {
  f = cleanCsvField(f);

  // Strip non-numeric prefix such as BOM artifacts.
  while (f.length() && !isDigit(f[0])) {
    f.remove(0, 1);
  }

  if (f.length() == 0) return false;
  for (int i = 0; i < f.length(); i++) {
    if (!isDigit(f[i])) return false;
  }
  return true;
}

void processMbMonCsvLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  char delim = ',';
  if (line.indexOf(',') < 0 && line.indexOf(';') >= 0) delim = ';';
  if (line.indexOf(',') < 0 && line.indexOf(';') < 0 && line.indexOf('\t') >= 0) delim = '\t';

  String addrStr = csvField(line, 0, delim);
  String desc = csvField(line, 1, delim);
  String formula = csvField(line, 2, delim);

  // Skip any header row or malformed row. This handles:
  // register address,register description
  // "Register Address","Description","Formula"
  // address,description
  if (!csvFirstFieldIsNumericAddress(addrStr)) return;

  // Strip any remaining leading junk before the numeric address.
  while (addrStr.length() && !isDigit(addrStr[0])) {
    addrStr.remove(0, 1);
  }

  if (formula.length() == 0) formula = "x";
  if (desc.length() == 0) desc = "Register " + addrStr;

  addModbusMonitorRegister(addrStr.toInt(), desc, formula);
}

void handleMbMonCsvUpload() {
  HTTPUpload& upload = server.upload();

  if (upload.status == UPLOAD_FILE_START) {
    // CSV upload replaces the current monitor list.
    mbMonCount = 0;
    saveModbusMonitorConfig();

    if (sdOK) {
      mbCsvUploadFile = SD_MMC.open("/mb_register_upload.csv", FILE_WRITE);
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (sdOK && mbCsvUploadFile) {
      mbCsvUploadFile.write(upload.buf, upload.currentSize);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (sdOK && mbCsvUploadFile) {
      mbCsvUploadFile.close();

      File f = SD_MMC.open("/mb_register_upload.csv", FILE_READ);
      if (f) {
        String line = "";
        while (f.available()) {
          char ch = f.read();
          if (ch == '\n') {
            processMbMonCsvLine(line);
            line = "";
          } else if (ch != '\r') {
            line += ch;
          }
        }
        if (line.length()) processMbMonCsvLine(line);
        f.close();
      }
    }
  }
}

void handleMbMonCsvDone() {
  saveModbusMonitorConfig();
  server.sendHeader("Location", "/ethernet");
  server.send(302, "text/plain", "");
}


void handleSaveMbLog() {
  prefs.begin("cfg", false);
  prefs.putBool("mbCsvEn", server.hasArg("en"));
  prefs.putUInt("mbCsvMs", server.arg("ms").toInt());
  prefs.end();

  mbCsvLogEnable = server.hasArg("en");
  mbCsvLogMs = server.arg("ms").toInt();
  if (mbCsvLogMs < 1000) mbCsvLogMs = 1000;

  server.sendHeader("Location", "/ethernet");
  server.send(302, "text/plain", "");
}

// ---------------- WiFi WebServer Routes ----------------

void setupRoutes() {
  server.on("/", handleRoot);

  server.on("/wifi", handleWifiPage);
  server.on("/savewifi", HTTP_POST, saveWifi);

  server.on("/ethernet", handleEthernetPage);
  server.on("/saveethernet", HTTP_POST, saveEthernet);

  server.on("/rs485", handleRS485Page);
  server.on("/savers485", HTTP_POST, saveRS485);

  server.on("/io", handleIOPage);
  server.on("/saveio", HTTP_POST, saveIOPage);

  server.on("/sd", handleSDPage);
  server.on("/sd/upload", HTTP_POST, handleSDUploadDone, handleSDUpload);
  server.on("/sd/download", HTTP_GET, handleSDDownload);
  server.on("/sd/delete", HTTP_GET, handleSDDelete);

  server.on("/can", handleCANPage);
  server.on("/savecan", HTTP_POST, saveCANPage);

  server.on("/mqtt", handleMqttPage);
  server.on("/savemqtt", HTTP_POST, saveMqtt);

  server.on("/time", handleTimePage);
  server.on("/savetime", HTTP_POST, saveTimePage);
  server.on("/ntpsync", HTTP_POST, handleNtpSyncNow);

  server.on("/ota", handleOTAPage);
  server.on("/wifiota", handleOTAPage);
  server.on("/ota/upload", HTTP_POST, handleOTADone, handleOTAUpload);

  server.on("/savemblog", HTTP_POST, handleSaveMbLog);

  server.on("/mbmon/add", HTTP_POST, handleMbMonAdd);
  server.on("/mbmon/delete", HTTP_GET, handleMbMonDelete);
  server.on("/mbmon/delete_selected", HTTP_POST, handleMbMonDeleteSelected);
  server.on("/mbmon/upload", HTTP_POST, handleMbMonCsvDone, handleMbMonCsvUpload);

  server.on("/json", HTTP_GET, apiStatus);
  server.on("/api/status", HTTP_GET, apiStatus);
  server.on("/api/io", HTTP_GET, apiStatus);
  server.on("/api/output", HTTP_GET, apiOutput);
  server.on("/api/output_mask", HTTP_GET, apiOutputMask);
  server.on("/api/registers", HTTP_GET, apiRegs);

  server.begin();
}

// ---------------- Periodic Time Sync ----------------

void handleNtpPeriodic() {
  if (!ntpEnable) return;
  if (lastNtpSync != 0 && millis() - lastNtpSync < 3600000UL) return;
  lastNtpSync = millis();
  syncRtcFromNtp();
}

// ---------------- Setup / Loop ----------------

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("Booting LCT Industrial Server...");

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(RGB_CTRL, OUTPUT);
  digitalWrite(RGB_CTRL, LOW);
  pinMode(RTC_INT, INPUT_PULLUP);

  for (int i = 0; i < 8; i++) pinMode(DI_PINS[i], INPUT);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  // Safe default: write all TCA outputs HIGH first. The board output LEDs/relays appear active-low.
  tcaOK = tca9554Init(0x00, 0xFF);
  Serial.println(tcaOK ? "TCA9554PWR OK" : "TCA9554PWR NOT FOUND");

  rtcOK = pcf85063Init();
  Serial.println(rtcOK ? "PCF85063 RTC OK" : "PCF85063 RTC NOT FOUND");

  memset(holdingRegs, 0, sizeof(holdingRegs));
  memset(mbRemoteRegs, 0, sizeof(mbRemoteRegs));
  memset(mbRemoteValid, 0, sizeof(mbRemoteValid));

  loadConfig();
  if (tcaOK) {
    tca9554Init(0x00, outputsOffRawByte());
    for (int i = 0; i < 8; i++) outState[i] = false;
  }
  loadModbusMonitorConfig();
  setupWiFi();
  setupSD();
  setupEthernet();
  setupEthernetWebServer();
  initIoEventLogger();

  pinMode(RS485_RTS, OUTPUT);
  rs485TxMode(false);
  RS485Serial.begin(rs485Baud, SERIAL_8N1, RS485_RX, RS485_TX);

  setupMqtt();
  setupCAN();
  setupRoutes();

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  if (ntpEnable) syncRtcFromNtp();

  Serial.println("Started.");
  Serial.println("AP SSID: " + apName());
  Serial.println("AP Password: " + apPassword());
  Serial.println("AP IP: " + wifiApIpString());
  Serial.println("STA IP: " + WiFi.localIP().toString());
  Serial.println("WiFi AP MAC: " + wifiApMacString());
  Serial.println("WiFi STA MAC: " + wifiStaMacString());
  Serial.println("DO1 WiFi Indicator: " + String(wifiIndicatorEnable ? "Enabled" : "Disabled"));
  Serial.println("DI Invert: " + String(diInvertLogic ? "Enabled" : "Disabled"));
  Serial.println("DO Invert: " + String(doInvertLogic ? "Enabled" : "Disabled"));
  Serial.println("ETH IP: " + String(ethernetStarted ? ipToString(Ethernet.localIP()) : "Not started"));
  Serial.println("ETH MAC: " + ethMacString());
  Serial.println("HTTP: http://192.168.4.1");
  Serial.println("WebSocket: ws://192.168.4.1:81");
}

void loop() {
  server.handleClient();

  refreshInputsOutputs();
  retryStaWifiIfNeeded();
  handleWifiConnectIndicator();

  handleWebSocketPush();
  handleMqtt();
  handleEthernetWebClient();
  handleModbusTcpServer();
  handleModbusRtuSlave();
  handleModbusHostPolling();
  handleCANRuntime();
  handleNtpPeriodic();
  csvLogIO();
  csvLogModbus();
  handleIoEventLogging();

  if (ethernetStarted) Ethernet.maintain();
}
