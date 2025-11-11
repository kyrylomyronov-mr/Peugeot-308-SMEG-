/*
 * PSA Comfort CAN Remote — с дисплеем (Arduino_GFX для ESP32-C6)
 * Waveshare ESP32-C6 1.47inch Display
 * CAN: 125/500 кбит/с, TX=GPIO20, RX=GPIO23
 * SN65HVD230 RS (standby) pin: set CAN_STANDBY_PIN accordingly
 * 
 * Установите библиотеку: Arduino_GFX by moononournation
 */

#include <WiFi.h>
#include <WebServer.h>
#include "driver/twai.h"
#include <esp_err.h>
#include <Arduino_GFX_Library.h>
#include <Preferences.h>
#include <string.h>
#include <stdio.h>
#include <sys/time.h>
#include <time.h>

// --------- CAN pins (ESP32-C6) ----------
static constexpr gpio_num_t CAN_TX = GPIO_NUM_20;
static constexpr gpio_num_t CAN_RX = GPIO_NUM_23;

// SN65HVD230 transceiver standby (RS) pin. Set to -1 if RS is tied to GND.
static constexpr int CAN_STANDBY_PIN = -1;

// --------- Display pins (Waveshare ESP32-C6 1.47") ----------
#define TFT_MOSI  6
#define TFT_SCK   7
#define TFT_CS    14
#define TFT_DC    15
#define TFT_RST   21
#define TFT_BL    22

// --------- Wi-Fi AP ----------
const char* AP_SSID = "ESP32-CAN-Remote";
const char* AP_PASS = "12345678";
WebServer server(80);

// --------- Display ----------
Arduino_DataBus *bus = new Arduino_HWSPI(TFT_DC, TFT_CS);
Arduino_GFX *gfx = new Arduino_ST7789(bus, TFT_RST, 0 /* rotation */, true /* IPS */, 172, 320, 34, 0, 0, 0);

// --------- Known button indices ----------
static constexpr uint8_t BTN_SETTINGS = 17;
static constexpr uint8_t BTN_MAPS     = 18;
static constexpr uint8_t BTN_RADIO    = 19;
static constexpr uint8_t BTN_PHONE    = 20;
static constexpr uint8_t BTN_APPS     = 29;
static constexpr uint8_t BTN_TRIP     = 30;
static constexpr uint8_t BTN_AC       = 32;

// --------- CAN data ----------
enum class CanBitrate : uint8_t { k125k = 0 };

static bool canStarted = false;
static CanBitrate currentBitrate = CanBitrate::k125k;
static float engineTemp = -99.0f;
static float batteryVolt = 0.0f;
static unsigned long lastCanRx = 0;
static unsigned long lastDisplayUpdate = 0;
static unsigned long lastBsi260 = 0;
static unsigned long lastBsi276 = 0;
// --------- Display cache / brightness ----------
static bool uiInitialized = false;
static String lastTempText;
static String lastVoltText;
static String lastCanText;
static uint16_t lastTempColor = 0;
static uint16_t lastVoltColor = 0;
static uint16_t lastCanColor = 0;
static uint8_t brightness = 153; // 0-255 (≈60%)
static Preferences prefs;
static constexpr const char* kPrefsNamespace = "psacan";
static constexpr const char* kPrefBrightness = "brightness";
static constexpr const char* kPrefLang5 = "lang5";
static constexpr const char* kPrefIs24 = "is24h";
static constexpr const char* kPrefIsC = "isc";
static constexpr const char* kPrefRaw260 = "raw260";
static constexpr const char* kPrefEpoch = "epoch";
static constexpr const char* kPrefEpochTs = "epoch_ts";

// --------- BSI emulator state ----------
static constexpr uint32_t CAN_ID_260 = 0x260; // BSI broadcast settings
static constexpr uint32_t CAN_ID_15B = 0x15B; // SMEG -> write settings
static constexpr uint32_t CAN_ID_39B = 0x39B; // SMEG -> write date/time
static constexpr uint32_t CAN_ID_276 = 0x276; // BSI broadcast date/time
static constexpr unsigned long BSI_PERIOD_260_MS = 500;
static constexpr unsigned long BSI_PERIOD_276_MS = 1000;

static uint8_t bsiState260[8] = {0};
static bool bsiHave260 = false;
static uint8_t bsiLang5 = 0b01110; // default Russian
static bool bsiIs24h = true;
static bool bsiIsCelsius = true;
static constexpr uint8_t kBsiCamByte = 4;   // byte index for rear camera presence flag
static constexpr uint8_t kBsiCamMask = 0x01; // bit mask for rear camera presence flag
static time_t bsiPersistedEpoch = 0;
static uint32_t bsiPersistedEpochMillis = 0;

static void bsiEnsureBaseline();
static void bsiApplyLangUnits();
static void bsiApplyFeatureFlags();
static void bsiSaveState();
static void bsiLoadState();
static void bsiPersistTime(time_t epoch);
static void bsiHandle15B(const uint8_t *data, uint8_t len);
static void bsiHandle39B(const uint8_t *data, uint8_t len);
static void bsiSend260();
static void bsiSend276();
static void bsiTick();

static String iso8601Now();
static bool parseIso8601(const String &iso, time_t &outEpoch);

static void invalidateDisplayCache();

static void applyBrightness(uint8_t value) {
  brightness = value;
  analogWrite(TFT_BL, brightness);
}

// --------- CAN Init ----------
static const char* bitrateName(CanBitrate /*b*/) { return "125k"; }

static twai_timing_config_t timingFor(CanBitrate /*b*/) {
  return TWAI_TIMING_CONFIG_125KBITS();
}

static bool twaiStart() {
  if (canStarted) return true;

  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
  g.tx_queue_len = 8;
  g.rx_queue_len = 16;
  g.clkout_divider = 0;
  g.alerts_enabled = 0;

  twai_timing_config_t t = timingFor(currentBitrate);
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g, &t, &f) != ESP_OK) { Serial.println("[CAN] driver install failed"); return false; }
  if (twai_start() != ESP_OK)                    { Serial.println("[CAN] start failed");          return false; }
  canStarted = true;
  Serial.printf("[CAN] started @%s (TX=%d RX=%d)\n", bitrateName(currentBitrate), (int)CAN_TX, (int)CAN_RX);
  lastBsi260 = lastBsi276 = millis();
  bsiSend260();
  bsiSend276();
  return true;
}

static bool twaiReconfigure(CanBitrate b) {
  if (currentBitrate == b && canStarted) return true;

  if (canStarted) {
    esp_err_t err = twai_stop();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) Serial.printf("[CAN] stop error: %d\n", (int)err);
    err = twai_driver_uninstall();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) Serial.printf("[CAN] uninstall error: %d\n", (int)err);
    canStarted = false;
    delay(50);
  }

  currentBitrate = b;
  lastCanRx = 0;
  engineTemp = -99.0f;
  batteryVolt = 0.0f;
  invalidateDisplayCache();
  unsigned long now = millis();
  lastBsi260 = now;
  lastBsi276 = now;
  return twaiStart();
}

// --------- CAN TX ----------
static bool canSend(uint32_t id, const uint8_t* d, uint8_t dlc) {
  if (!canStarted || dlc > 8) return false;
  twai_message_t m = {};
  m.identifier = id;
  m.flags = 0;
  m.data_length_code = dlc;
  if (dlc && d) memcpy(m.data, d, dlc);
  return (twai_transmit(&m, pdMS_TO_TICKS(20)) == ESP_OK);
}

static bool canSend122(const uint8_t* d, uint8_t dlc) {
  return canSend(0x122, d, dlc);
}

static inline void sendIdle122() { uint8_t z[8] = {0}; canSend122(z, 8); }

static void pressIndex(uint8_t idx, uint16_t press_ms=75) {
  if (idx < 1 || idx > 40) return;
  uint8_t z = (uint8_t)(idx - 1);
  uint8_t byteIdx = z / 8;
  uint8_t bitIdx  = z % 8;

  uint8_t d[8] = {0};
  d[byteIdx] = (uint8_t)(1u << bitIdx);
  canSend122(d, 8);
  delay(press_ms);
  sendIdle122();
  delay(120);
}

// --------- BSI Helpers ----------
static inline void bsiSetBit(uint8_t &byte, uint8_t mask, bool value) {
  if (value) byte |= mask; else byte &= static_cast<uint8_t>(~mask);
}

static void bsiApplyFeatureFlags() {
  // Byte 5 bit 0 announces the presence of a reversing camera to the SMEG head unit.
  // Force the bit to zero to disable camera announcements.
  bsiSetBit(bsiState260[kBsiCamByte], kBsiCamMask, false);
}

static void bsiEnsureBaseline() {
  if (bsiHave260) return;
  memset(bsiState260, 0, sizeof(bsiState260));
  bsiState260[0] |= 0x80; // menu active bit
  bsiHave260 = true;
  bsiApplyFeatureFlags();
}

static void bsiApplyLangUnits() {
  bsiEnsureBaseline();
  uint8_t low2 = bsiState260[0] & 0x03u;
  bsiState260[0] = static_cast<uint8_t>(((bsiLang5 & 0x1Fu) << 2) | low2);
  bsiState260[0] |= 0x80;
  bsiSetBit(bsiState260[1], 0x40u, bsiIsCelsius);
  bsiApplyFeatureFlags();
}

static void bsiSaveState() {
  bsiApplyLangUnits();
  prefs.putUChar(kPrefLang5, bsiLang5);
  prefs.putBool(kPrefIs24, bsiIs24h);
  prefs.putBool(kPrefIsC, bsiIsCelsius);
  prefs.putBytes(kPrefRaw260, bsiState260, sizeof(bsiState260));
}

static void bsiLoadState() {
  size_t sz = prefs.getBytesLength(kPrefRaw260);
  if (sz >= sizeof(bsiState260)) {
    prefs.getBytes(kPrefRaw260, bsiState260, sizeof(bsiState260));
    bsiHave260 = true;
  } else {
    bsiHave260 = false;
  }

  bsiLang5 = prefs.getUChar(kPrefLang5, bsiLang5);
  bsiIs24h = prefs.getBool(kPrefIs24, bsiIs24h);
  bsiIsCelsius = prefs.getBool(kPrefIsC, bsiIsCelsius);

  bsiEnsureBaseline();
  bsiApplyLangUnits();

  bsiPersistedEpoch = static_cast<time_t>(prefs.getLong(kPrefEpoch, 0));
  bsiPersistedEpochMillis = prefs.getUInt(kPrefEpochTs, 0);
  if (bsiPersistedEpoch > 0) {
    uint32_t nowMs = millis();
    uint32_t baseMs = bsiPersistedEpochMillis;
    uint32_t delta = (baseMs > 0 && nowMs >= baseMs) ? (nowMs - baseMs) : 0;
    time_t epoch = bsiPersistedEpoch + static_cast<time_t>(delta / 1000u);
    struct timeval tv = { .tv_sec = epoch, .tv_usec = 0 };
    settimeofday(&tv, nullptr);
  }
}

static void bsiPersistTime(time_t epoch) {
  bsiPersistedEpoch = epoch;
  bsiPersistedEpochMillis = millis();
  prefs.putLong(kPrefEpoch, static_cast<long>(epoch));
  prefs.putUInt(kPrefEpochTs, bsiPersistedEpochMillis);
}

static void bsiHandle15B(const uint8_t *data, uint8_t len) {
  if (!data || len == 0) return;
  if (len > 8) len = 8;
  uint8_t buf[8] = {0};
  memcpy(buf, data, len);

  bsiEnsureBaseline();

  bool useLangUnits = (buf[0] & 0x80u) != 0;
  bool useTail = (len > 1) && ((buf[1] & 0x04u) != 0);

  if (useTail) {
    for (uint8_t i = 1; i < len && i < sizeof(bsiState260); ++i) {
      bsiState260[i] = buf[i];
    }
  }

  if (useLangUnits) {
    bsiLang5 = static_cast<uint8_t>((buf[0] >> 2) & 0x1Fu);
    bsiIsCelsius = (buf[1] & 0x40u) != 0;
  }

  bsiApplyLangUnits();
  bsiApplyFeatureFlags();
  bsiSaveState();
  bsiSend260();
}

static void bsiHandle39B(const uint8_t *data, uint8_t len) {
  if (!data || len < 5) return;

  struct {
    bool is24;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
  } ct;

  ct.is24 = (data[0] & 0x80u) != 0;
  ct.year = 2000u + static_cast<uint16_t>(data[0] & 0x7Fu);
  ct.month = data[1] & 0x0Fu;
  ct.day = data[2] & 0x1Fu;
  ct.hour = data[3] & 0x1Fu;
  ct.minute = data[4] & 0x3Fu;

  bsiIs24h = ct.is24;
  bsiSaveState();

  struct tm tmv = {};
  tmv.tm_year = static_cast<int>(ct.year) - 1900;
  tmv.tm_mon = (ct.month > 0) ? (ct.month - 1) : 0;
  tmv.tm_mday = (ct.day > 0) ? ct.day : 1;
  tmv.tm_hour = ct.hour;
  tmv.tm_min = ct.minute;
  tmv.tm_sec = 0;
  time_t epoch = mktime(&tmv);
  if (epoch != static_cast<time_t>(-1)) {
    struct timeval tv = { .tv_sec = epoch, .tv_usec = 0 };
    settimeofday(&tv, nullptr);
    bsiPersistTime(epoch);
  }

  bsiSend276();
}

static void bsiBuild276(uint8_t out[7]) {
  time_t nowEpoch = time(nullptr);
  struct tm *t = localtime(&nowEpoch);
  uint16_t year = t ? static_cast<uint16_t>(t->tm_year + 1900) : 2000u;
  uint8_t mon = t ? static_cast<uint8_t>(t->tm_mon + 1) : 1u;
  uint8_t day = t ? static_cast<uint8_t>(t->tm_mday) : 1u;
  uint8_t hour = t ? static_cast<uint8_t>(t->tm_hour) : 0u;
  uint8_t min = t ? static_cast<uint8_t>(t->tm_min) : 0u;

  out[0] = (bsiIs24h ? 0x80u : 0x00u) | static_cast<uint8_t>((year >= 2000u) ? (year - 2000u) : 0u);
  out[1] = mon & 0x0Fu;
  out[2] = day & 0x1Fu;
  out[3] = hour & 0x1Fu;
  out[4] = min & 0x3Fu;
  out[5] = 0x3Fu;
  out[6] = 0xFEu;
}

static void bsiSend260() {
  if (!canStarted) return;
  bsiApplyLangUnits();
  if (canSend(CAN_ID_260, bsiState260, sizeof(bsiState260))) {
    lastBsi260 = millis();
  }
}

static void bsiSend276() {
  if (!canStarted) return;
  uint8_t payload[7];
  bsiBuild276(payload);
  if (canSend(CAN_ID_276, payload, sizeof(payload))) {
    lastBsi276 = millis();
  }
}

static void bsiTick() {
  if (!canStarted) return;
  unsigned long now = millis();
  if (now - lastBsi260 >= BSI_PERIOD_260_MS) {
    bsiSend260();
  }
  if (now - lastBsi276 >= BSI_PERIOD_276_MS) {
    bsiSend276();
  }
}

static String iso8601FromTm(const struct tm &tmv) {
  char buf[32];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02d",
           tmv.tm_year + 1900,
           tmv.tm_mon + 1,
           tmv.tm_mday,
           tmv.tm_hour,
           tmv.tm_min,
           tmv.tm_sec);
  return String(buf);
}

static String iso8601Now() {
  time_t nowEpoch = time(nullptr);
  struct tm tmv;
  if (!localtime_r(&nowEpoch, &tmv)) {
    return String(F("1970-01-01T00:00:00"));
  }
  return iso8601FromTm(tmv);
}

static bool parseIso8601(const String &iso, time_t &outEpoch) {
  int year = 0, month = 0, day = 0, hour = 0, minute = 0, second = 0;
  int matched = sscanf(iso.c_str(), "%d-%d-%dT%d:%d:%d",
                       &year, &month, &day, &hour, &minute, &second);
  if (matched < 5) {
    return false;
  }
  if (matched < 6) {
    second = 0;
  }
  if (year < 1970 || month < 1 || month > 12 || day < 1 || day > 31 ||
      hour < 0 || hour > 23 || minute < 0 || minute > 59 || second < 0 || second > 59) {
    return false;
  }

  struct tm tmv = {};
  tmv.tm_year = year - 1900;
  tmv.tm_mon = month - 1;
  tmv.tm_mday = day;
  tmv.tm_hour = hour;
  tmv.tm_min = minute;
  tmv.tm_sec = second;
  tmv.tm_isdst = -1;

  time_t epoch = mktime(&tmv);
  if (epoch == static_cast<time_t>(-1)) {
    return false;
  }

  outEpoch = epoch;
  return true;
}

// --------- CAN RX ----------
static void readCanMessages() {
  twai_message_t msg;
  while (twai_receive(&msg, 0) == ESP_OK) {
    lastCanRx = millis();
    
    // Peugeot 307 (2006) Comfort CAN:
    // 0x0F6 - Температура двигателя (D0: -40°C offset)
    if (msg.identifier == 0x0F6 && msg.data_length_code >= 1) {
      engineTemp = (float)msg.data[0] - 40.0f;
    }

    // 0x128 - Напряжение (D5: * 10)
    if (msg.identifier == 0x128 && msg.data_length_code >= 6) {
      batteryVolt = (float)msg.data[5] / 10.0f;
    }

    if (msg.identifier == CAN_ID_15B && msg.data_length_code > 0) {
      bsiHandle15B(msg.data, msg.data_length_code);
    }

    if (msg.identifier == CAN_ID_39B && msg.data_length_code >= 5) {
      bsiHandle39B(msg.data, msg.data_length_code);
    }
  }
}

// --------- Display Update ----------
static constexpr int TEMP_X = 60;
static constexpr int TEMP_Y = 50;
static constexpr int TEMP_W = 112;
static constexpr int TEMP_H = 24;
static constexpr int VOLT_X = 60;
static constexpr int VOLT_Y = 90;
static constexpr int VOLT_W = 112;
static constexpr int VOLT_H = 24;
static constexpr int CAN_X  = 60;
static constexpr int CAN_Y  = 130;
static constexpr int CAN_W  = 112;
static constexpr int CAN_H  = 10;

static void invalidateDisplayCache() {
  uiInitialized = false;
  lastTempText = "";
  lastVoltText = "";
  lastCanText = "";
  lastTempColor = 0;
  lastVoltColor = 0;
  lastCanColor = 0;
}

static void drawStaticUi() {
  if (uiInitialized) return;

  gfx->fillScreen(BLACK);

  gfx->setTextColor(WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(10, 10);
  gfx->println("PSA CAN");

  gfx->setTextSize(3);
  gfx->setTextColor(CYAN);
  gfx->setCursor(10, 50);
  gfx->print("T:");

  gfx->setTextColor(ORANGE);
  gfx->setCursor(10, 90);
  gfx->print("V:");

  gfx->setTextSize(1);
  gfx->setTextColor(WHITE);
  gfx->setCursor(10, 130);
  gfx->print("CAN:");

  uiInitialized = true;
}

static void drawField(int x, int y, int w, int h, uint8_t textSize,
                      const String &text, uint16_t color,
                      String &lastText, uint16_t &lastColor) {
  if (text == lastText && color == lastColor) return;

  gfx->fillRect(x, y, w, h, BLACK);
  gfx->setTextSize(textSize);
  gfx->setCursor(x, y);
  gfx->setTextColor(color);
  gfx->print(text);

  lastText = text;
  lastColor = color;
}

static void updateDisplay() {
  unsigned long now = millis();
  if (now - lastDisplayUpdate < 150) return;
  lastDisplayUpdate = now;

  bool canOk = (now - lastCanRx < 2000);

  drawStaticUi();

  char buf[16];

  String tempText;
  uint16_t tempColor;
  if (engineTemp > -90.0f) {
    snprintf(buf, sizeof(buf), "%.0fC", engineTemp);
    tempText = String(buf);
    if (engineTemp < 80.0f) tempColor = GREEN;
    else if (engineTemp < 95.0f) tempColor = YELLOW;
    else tempColor = RED;
  } else {
    tempText = F("---");
    tempColor = DARKGREY;
  }
  drawField(TEMP_X, TEMP_Y, TEMP_W, TEMP_H, 3, tempText, tempColor,
            lastTempText, lastTempColor);

  String voltText;
  uint16_t voltColor;
  if (batteryVolt > 5.0f) {
    snprintf(buf, sizeof(buf), "%.1fV", batteryVolt);
    voltText = String(buf);
    if (batteryVolt >= 12.5f) voltColor = GREEN;
    else if (batteryVolt >= 12.0f) voltColor = YELLOW;
    else voltColor = RED;
  } else {
    voltText = F("---");
    voltColor = DARKGREY;
  }
  drawField(VOLT_X, VOLT_Y, VOLT_W, VOLT_H, 3, voltText, voltColor,
            lastVoltText, lastVoltColor);

  String canText;
  uint16_t canColor = canOk ? GREEN : RED;
  if (canOk) {
    canText = String("OK @") + bitrateName(currentBitrate);
  } else {
    canText = String("No data @") + bitrateName(currentBitrate);
  }
  drawField(CAN_X, CAN_Y, CAN_W, CAN_H, 1, canText, canColor,
            lastCanText, lastCanColor);
}

// --------- HTML ----------
static String page() {
  String s; s.reserve(6500);
  s += F("<!doctype html><html lang='ru'><head><meta charset='utf-8'/>"
         "<meta name='viewport' content='width=device-width,initial-scale=1'/>"
         "<title>PSA CAN Remote</title>"
         "<style>body{font-family:'Inter',system-ui,Arial,sans-serif;margin:0;padding:24px;background:#0f172a;color:#f8fafc;}"
         "main{max-width:960px;margin:0 auto;display:flex;flex-direction:column;gap:20px;}"
         "h2{margin:0 0 8px;font-size:26px;}"
         "h3{margin:0 0 12px;font-size:18px;color:#60a5fa;}"
         ".card{background:#16213b;border-radius:16px;padding:22px;box-shadow:0 12px 40px rgba(8,15,35,0.45);}"
         ".card p{margin:0 0 12px;}"
         ".metrics{display:grid;grid-template-columns:repeat(auto-fit,minmax(160px,1fr));gap:18px;}"
         ".metric .label{font-size:12px;color:#94a3b8;text-transform:uppercase;letter-spacing:.08em;margin-bottom:6px;}"
         ".metric .value{font-size:26px;font-weight:600;color:#f1f5f9;}"
         ".badge{display:inline-flex;align-items:center;padding:6px 14px;border-radius:999px;font-size:13px;font-weight:600;}"
         ".badge.ok{background:rgba(34,197,94,0.18);color:#4ade80;}"
         ".badge.err{background:rgba(248,113,113,0.18);color:#f87171;}"
         ".range-row{display:flex;flex-direction:column;gap:12px;}"
         ".toggle{display:flex;align-items:center;gap:10px;margin-top:10px;}"
         ".toggle label{color:#cbd5f5;font-size:15px;}"
         "input[type=range]{width:100%;accent-color:#2563eb;}"
         "input[type=datetime-local]{padding:10px 14px;border-radius:10px;border:1px solid #334155;background:#0f172a;color:#e2e8f0;font-size:15px;}"
         ".controls{display:flex;flex-wrap:wrap;gap:12px;}"
         "button{padding:12px 18px;border:none;border-radius:10px;background:#1f2937;color:#f8fafc;font-size:15px;cursor:pointer;transition:background .2s,transform .2s;}"
         "button:hover{background:#273548;transform:translateY(-1px);}"
         "button:active{transform:scale(0.98);}"
         "button.primary{background:#2563eb;}"
         "button.primary:hover{background:#1d4ed8;}"
         ".small{color:#94a3b8;font-size:13px;}"
         ".row{display:flex;flex-wrap:wrap;gap:12px;margin-top:12px;}"
         ".row button{flex:1 1 140px;}"
         "footer{color:#64748b;font-size:12px;text-align:center;padding:16px 0;}</style>"
         "<script>let blSlider=null,blLock=false,prefLock=false,timeInput=null;"
         "function updateBrightnessLabel(v){document.getElementById('brightnessPct').innerText=Math.round(v/255*100)+'%';}"
         "function formatUptime(secs){secs=Number(secs||0);if(secs<0)secs=0;const h=Math.floor(secs/3600);const m=Math.floor((secs%3600)/60);const s=Math.floor(secs%60);let parts=[];if(h)parts.push(h+' ч');if(m||h)parts.push(m+' м');parts.push(s+' с');return parts.join(' ');}"
         "async function refresh(){try{let r=await fetch('/data');if(!r.ok)return;let d=await r.json();const tempVal=(typeof d.temp==='number')?d.temp.toFixed(1):d.temp;document.getElementById('temp').innerText=tempVal+(d.isC?'°C':'°F');"
         "const voltVal=(typeof d.volt==='number')?d.volt.toFixed(1):d.volt;document.getElementById('volt').innerText=voltVal+'V';document.getElementById('speed').innerText=d.speed||'---';document.getElementById('ip').innerText=d.ip||'—';document.getElementById('timeNow').innerText=(d.time||'--').replace('T',' ');document.getElementById('uptime').innerText=formatUptime(d.uptime);const badge=document.getElementById('canBadge');if(badge){badge.innerText=d.canOk?'CAN онлайн':'CAN нет данных';badge.className='badge '+(d.canOk?'ok':'err');}if(blSlider){if(!blLock){blSlider.value=d.brightness;}updateBrightnessLabel(Number(blSlider.value));}if(!prefLock){const is24=document.getElementById('is24');const isc=document.getElementById('isc');if(is24)is24.checked=!!d.is24;if(isc)isc.checked=!!d.isC;}if(timeInput&&document.activeElement!==timeInput&&d.time){let v=d.time.length>=16?d.time.substring(0,16):d.time;timeInput.value=v;}}catch(e){console.error(e);}}"
         "function init(){blSlider=document.getElementById('bl');if(blSlider){blSlider.addEventListener('input',()=>{blLock=true;updateBrightnessLabel(Number(blSlider.value));});blSlider.addEventListener('change',async()=>{let v=blSlider.value;await setBrightness(v);blLock=false;});}timeInput=document.getElementById('timeInput');const is24=document.getElementById('is24');const isc=document.getElementById('isc');if(is24){is24.addEventListener('change',()=>updatePref('is24',is24.checked));}if(isc){isc.addEventListener('change',()=>updatePref('isc',isc.checked));}refresh();setInterval(refresh,1500);}"
         "async function setBrightness(v){try{let r=await fetch('/brightness?value='+v);if(!r.ok)throw new Error();}catch(e){alert('Не удалось обновить подсветку');}}"
         "async function updatePref(name,val){prefLock=true;try{let r=await fetch('/prefs?'+name+'='+(val?'1':'0'));if(!r.ok)throw new Error();}catch(e){alert('Не удалось сохранить настройки');}finally{prefLock=false;}}"
         "function pad2(n){return n.toString().padStart(2,'0');}"
         "function isoLocalNow(){const d=new Date();return d.getFullYear()+'-'+pad2(d.getMonth()+1)+'-'+pad2(d.getDate())+'T'+pad2(d.getHours())+':'+pad2(d.getMinutes())+':'+pad2(d.getSeconds());}"
         "async function syncTime(){const iso=isoLocalNow();try{let r=await fetch('/time?iso='+encodeURIComponent(iso));if(!r.ok)throw new Error();refresh();}catch(e){alert('Не удалось синхронизировать время');}}"
         "async function setTimeFromInput(){if(!timeInput||!timeInput.value){alert('Введите дату и время');return;}let iso=timeInput.value;if(iso.length===16){iso+=':00';}try{let r=await fetch('/time?iso='+encodeURIComponent(iso));if(!r.ok)throw new Error();refresh();}catch(e){alert('Не удалось установить время');}}"
         "async function btn(n){await fetch('/btn?n='+n);}"
         "document.addEventListener('DOMContentLoaded',init);</script></head><body><main>");

  s += F("<div class='card'>"
         "<h2>🚗 PSA CAN Remote</h2>"
         "<p class='small'>ESP32-C6 Comfort CAN контроллер. IP точки доступа: <strong id='ip'>--</strong></p>"
         "<div class='metrics'>"
         "<div class='metric'><div class='label'>Температура двигателя</div><div class='value' id='temp'>--°C</div></div>"
         "<div class='metric'><div class='label'>Напряжение</div><div class='value' id='volt'>--V</div></div>"
         "<div class='metric'><div class='label'>CAN скорость</div><div class='value' id='speed'>---</div></div>"
         "<div class='metric'><div class='label'>Аптайм</div><div class='value' id='uptime'>--</div></div>"
         "</div>"
         "<span class='badge err' id='canBadge'>CAN нет данных</span>"
         "</div>");

  s += F("<div class='card'>"
         "<h3>Подсветка дисплея</h3>"
         "<div class='range-row'>"
         "<label for='bl'>Яркость: <span id='brightnessPct'>--%</span></label>"
         "<input type='range' id='bl' min='0' max='255' value='153'/>"
         "</div>"
         "<p class='small'>Настройка применяется сразу и сохраняется во встроенной памяти.</p>"
         "</div>");

  s += F("<div class='card'>"
         "<h3>Настройки отображения</h3>"
         "<div class='toggle'><input type='checkbox' id='is24'/><label for='is24'>24-часовой формат времени</label></div>"
         "<div class='toggle'><input type='checkbox' id='isc'/><label for='isc'>Температура в градусах Цельсия</label></div>"
         "<p class='small'>Параметры мгновенно применяются к эмуляции BSI и сохраняются в памяти.</p>"
         "</div>");

  s += F("<div class='card'>"
         "<h3>Дата и время</h3>"
         "<div class='metrics'>"
         "<div class='metric'><div class='label'>Встроенные часы</div><div class='value' id='timeNow'>--</div></div>"
         "</div>"
         "<div class='controls'>"
         "<input type='datetime-local' id='timeInput'/>"
         "<button class='primary' onclick='setTimeFromInput()'>Установить вручную</button>"
         "<button onclick='syncTime()'>Синхронизировать с браузером</button>"
         "</div>"
         "<p class='small'>Используйте для быстрой синхронизации часов SMEG без подключения к диагностике.</p>"
         "</div>");

  s += F("<div class='card'>"
         "<h3>Быстрые команды</h3>"
         "<p class='small'>Имитация нажатий физических кнопок на панели SMEG.</p>"
         "<div class='row'>"
         "<button onclick='btn(17)'>⚙️ Settings</button>"
         "<button onclick='btn(18)'>🗺️ Maps</button>"
         "<button onclick='btn(19)'>📻 Radio</button>"
         "<button onclick='btn(20)'>📞 Phone</button>"
         "<button onclick='btn(29)'>📱 Apps</button>"
         "<button onclick='btn(30)'>🛣️ Trip</button>"
         "<button onclick='btn(32)'>❄️ AC</button>"
         "</div>"
         "</div>");

  s += F("</main><footer>Comfort CAN remote • ESP32-C6</footer></body></html>");
  return s;
}

// --------- HTTP ----------
static void handleRoot() { 
  server.send(200, "text/html; charset=utf-8", page()); 
}

static void handleData() {
  String json = "{\"temp\":";
  if (engineTemp > -90.0f) {
    float value = engineTemp;
    if (!bsiIsCelsius) {
      value = value * 9.0f / 5.0f + 32.0f;
    }
    json += String(value, 1);
  } else {
    json += "\"---\"";
  }
  json += ",\"volt\":";
  json += (batteryVolt > 5.0f) ? String(batteryVolt, 1) : "\"---\"";
  json += ",\"speed\":\"";
  json += bitrateName(currentBitrate);
  json += "\"";
  json += ",\"brightness\":";
  json += String((int)brightness);
  json += ",\"is24\":";
  json += bsiIs24h ? "true" : "false";
  json += ",\"isC\":";
  json += bsiIsCelsius ? "true" : "false";
  json += ",\"time\":\"";
  json += iso8601Now();
  json += "\"";
  json += ",\"ip\":\"";
  json += WiFi.softAPIP().toString();
  json += "\"";
  json += ",\"canOk\":";
  bool canOk = (millis() - lastCanRx) < 2000u;
  json += canOk ? "true" : "false";
  json += ",\"uptime\":";
  json += String((uint32_t)(millis() / 1000u));
  json += "}";
  server.send(200, "application/json", json);
}

static void handleBtn() {
  if (!server.hasArg("n")) {
    server.send(400, "text/plain", "Missing param");
    return;
  }
  int n = server.arg("n").toInt();
  if (!(n==BTN_SETTINGS || n==BTN_MAPS || n==BTN_RADIO || n==BTN_PHONE ||
        n==BTN_APPS || n==BTN_TRIP || n==BTN_AC)) {
    server.send(400, "text/plain", "Invalid index");
    return;
  }
  pressIndex((uint8_t)n, 75);
  server.send(200, "text/plain", "OK");
}

static void handleBrightness() {
  if (!server.hasArg("value")) {
    server.send(400, "text/plain", "Missing value");
    return;
  }

  int v = server.arg("value").toInt();
  if (v < 0) v = 0;
  if (v > 255) v = 255;

  uint8_t newBrightness = (uint8_t)v;
  uint8_t previousBrightness = brightness;
  applyBrightness(newBrightness);
  if (newBrightness != previousBrightness) {
    prefs.putUChar(kPrefBrightness, brightness);
  }
  server.send(200, "text/plain", "OK");
}

static void handleCfg() {
  if (!server.hasArg("speed")) {
    server.send(400, "text/plain", "Missing speed");
    return;
  }

  String v = server.arg("speed");
  if (v != "125") {
    server.send(400, "text/plain", "Unknown speed");
    return;
  }

  if (!twaiReconfigure(CanBitrate::k125k)) {
    server.send(500, "text/plain", "CAN reconfigure failed");
    return;
  }

  server.send(200, "text/plain", "OK");
}

static void handlePrefs() {
  bool changed = false;
  bool need260 = false;
  bool need276 = false;

  if (server.hasArg("is24")) {
    bool newVal = server.arg("is24") != "0";
    if (newVal != bsiIs24h) {
      bsiIs24h = newVal;
      changed = true;
      need276 = true;
    }
  }

  if (server.hasArg("isc")) {
    bool newVal = server.arg("isc") != "0";
    if (newVal != bsiIsCelsius) {
      bsiIsCelsius = newVal;
      changed = true;
      need260 = true;
    }
  }

  if (changed) {
    bsiSaveState();
    if (need260) {
      bsiSend260();
    }
    if (need276) {
      bsiSend276();
    }
  }

  server.send(200, "text/plain", "OK");
}

static void handleTime() {
  time_t epoch = 0;
  bool ok = false;

  if (server.hasArg("epoch")) {
    long val = server.arg("epoch").toInt();
    if (val > 0) {
      epoch = static_cast<time_t>(val);
      ok = true;
    }
  } else if (server.hasArg("iso")) {
    ok = parseIso8601(server.arg("iso"), epoch);
  }

  if (!ok) {
    server.send(400, "text/plain", "Invalid time");
    return;
  }

  struct timeval tv = { .tv_sec = epoch, .tv_usec = 0 };
  settimeofday(&tv, nullptr);
  bsiPersistTime(epoch);
  bsiSend276();

  server.send(200, "text/plain", "OK");
}

// --------- Setup/Loop ----------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[PSA CAN Monitor] ESP32-C6");

  // Настройка SPI для дисплея
  SPI.begin(TFT_SCK, -1, TFT_MOSI, TFT_CS);
  
  // Display
  pinMode(TFT_BL, OUTPUT);
  analogWriteResolution(TFT_BL, 8);      // match the previous LEDC configuration
  analogWriteFrequency(TFT_BL, 5000);

  prefs.begin(kPrefsNamespace, false);
  uint8_t savedBrightness = prefs.getUChar(kPrefBrightness, brightness);
  applyBrightness(savedBrightness);
  bsiLoadState();

  if (CAN_STANDBY_PIN >= 0) {
    pinMode(CAN_STANDBY_PIN, OUTPUT);
    digitalWrite(CAN_STANDBY_PIN, LOW);
  }

  gfx->begin();
  gfx->fillScreen(BLACK);
  gfx->setTextColor(WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(20, 60);
  gfx->println("Starting...");

  // Wi-Fi
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.printf("[WiFi] AP: %s, IP: %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());

  // CAN
  if (!twaiStart()) Serial.println("[CAN] init failed");

  // HTTP
  server.on("/", HTTP_GET, handleRoot);
  server.on("/btn", HTTP_GET, handleBtn);
  server.on("/data", HTTP_GET, handleData);
  server.on("/brightness", HTTP_GET, handleBrightness);
  server.on("/cfg", HTTP_GET, handleCfg);
  server.on("/prefs", HTTP_GET, handlePrefs);
  server.on("/time", HTTP_GET, handleTime);
  server.begin();
  Serial.println("[HTTP] Server started on :80");

  delay(1000);
  gfx->fillScreen(BLACK);
  invalidateDisplayCache();
}

void loop() {
  server.handleClient();
  readCanMessages();
  bsiTick();
  updateDisplay();
}
