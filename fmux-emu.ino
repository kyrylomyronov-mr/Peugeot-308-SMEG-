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
static time_t bsiPersistedEpoch = 0;
static uint32_t bsiPersistedEpochMillis = 0;

static void bsiEnsureBaseline();
static void bsiApplyLangUnits();
static void bsiSaveState();
static void bsiLoadState();
static void bsiPersistTime(time_t epoch);
static void bsiHandle15B(const uint8_t *data, uint8_t len);
static void bsiHandle39B(const uint8_t *data, uint8_t len);
static void bsiSend260();
static void bsiSend276();
static void bsiTick();

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

static void bsiEnsureBaseline() {
  if (bsiHave260) return;
  memset(bsiState260, 0, sizeof(bsiState260));
  bsiState260[0] |= 0x80; // menu active bit
  bsiHave260 = true;
}

static void bsiApplyLangUnits() {
  bsiEnsureBaseline();
  uint8_t low2 = bsiState260[0] & 0x03u;
  bsiState260[0] = static_cast<uint8_t>(((bsiLang5 & 0x1Fu) << 2) | low2);
  bsiState260[0] |= 0x80;
  bsiSetBit(bsiState260[1], 0x40u, bsiIsCelsius);
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
  String s; s.reserve(4500);
  s += F("<!doctype html><html lang='ru'><head><meta charset='utf-8'/>"
         "<meta name='viewport' content='width=device-width,initial-scale=1'/>"
         "<title>PSA CAN Remote</title>"
         "<style>body{font-family:system-ui,Arial,sans-serif;margin:24px;background:#1a1a1a;color:#fff}"
         "h2{margin:0 0 12px;color:#0af} .row{display:flex;flex-wrap:wrap;gap:10px;margin-bottom:20px}"
         "button{padding:12px 16px;border:1px solid #555;border-radius:10px;background:#2a2a2a;color:#fff;cursor:pointer}"
         "button:active{background:#3a3a3a} .info{background:#2a2a2a;padding:15px;border-radius:8px;margin-bottom:15px}"
         ".muted{color:#888;font-size:12px;margin-top:8px} .tag{display:inline-block;padding:4px 8px;border-radius:6px;background:#2a2a2a;margin-top:8px;font-size:12px;color:#0af}"
         ".tag span{color:#fff} input[type=range]{width:100%;margin-top:12px}</style>"
         "<script>let blSlider=null,blLock=false;function updateBrightnessLabel(v){document.getElementById('brightnessPct').innerText=Math.round(v/255*100)+'%';}"
         "async function refresh(){let r=await fetch('/data');let d=await r.json();"
         "document.getElementById('temp').innerText=d.temp+'°C';"
         "document.getElementById('volt').innerText=d.volt+'V';"
         "document.getElementById('speed').innerText=d.speed;"
         "if(blSlider){if(!blLock){blSlider.value=d.brightness;}updateBrightnessLabel(Number(blSlider.value));}else{updateBrightnessLabel(d.brightness);}}"
         "window.addEventListener('DOMContentLoaded',()=>{blSlider=document.getElementById('bl');if(blSlider){blSlider.addEventListener('input',()=>{blLock=true;updateBrightnessLabel(Number(blSlider.value));});"
         "blSlider.addEventListener('change',async()=>{let v=blSlider.value;await setBrightness(v);blLock=false;});}"
         "refresh();setInterval(refresh,1000);});"
         "async function setBrightness(v){let r=await fetch('/brightness?value='+v);if(!r.ok)alert('Brightness update failed');}"
         "async function btn(n){await fetch('/btn?n='+n);}</script></head><body>");

  s += F("<h2>🚗 PSA CAN Remote</h2>"
         "<div class='info'><b>Температура:</b> <span id='temp'>---</span> &nbsp;|&nbsp; "
         "<b>Напряжение:</b> <span id='volt'>---</span><br/><span class='tag'>CAN <span id='speed'>---</span></span></div>"
         "<h3>Подсветка</h3>"
         "<div class='info'><label for='bl'>Яркость подсветки: <span id='brightnessPct'>--%</span></label><input type='range' id='bl' min='0' max='255' value='153'/></div>"
         "<h3>Управление</h3>"
         "<div class='row'>"
         "<button onclick='btn(17)'>⚙️ Settings</button>"
         "<button onclick='btn(18)'>🗺️ Maps</button>"
         "<button onclick='btn(19)'>📻 Radio</button>"
         "<button onclick='btn(20)'>📞 Phone</button>"
         "<button onclick='btn(29)'>📱 Apps</button>"
         "<button onclick='btn(30)'>🛣️ Trip</button>"
         "<button onclick='btn(32)'>❄️ AC</button>"
         "</div>"
         "<h3>CAN скорость</h3>"
         "<div class='info'>125 кбит/с (Comfort CAN). SMEG поддерживает только эту скорость.</div>"
         "<div class='muted'>Peugeot 307 (2006) full CAN • SMEG+ 2009</div>"
         "</body></html>");
  return s;
}

// --------- HTTP ----------
static void handleRoot() { 
  server.send(200, "text/html; charset=utf-8", page()); 
}

static void handleData() {
  String json = "{\"temp\":";
  json += (engineTemp > -90.0f) ? String(engineTemp, 1) : "\"---\"";
  json += ",\"volt\":";
  json += (batteryVolt > 5.0f) ? String(batteryVolt, 1) : "\"---\"";
  json += ",\"speed\":\"";
  json += bitrateName(currentBitrate);
  json += "\",\"brightness\":";
  json += String((int)brightness);
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
