/*
 * PSA Comfort CAN Remote — ESP32 (без дисплея)
 * Аппаратные подключения:
 *   ESP32 TXCAN (GPIO22) -> MCP2551 TXD
 *   ESP32 RXCAN (GPIO23) <- MCP2551 RXD
 * CAN шина: 125 кбит/с
 * SN65HVD230/MCP2551 RS (standby) pin: установите CAN_STANDBY_PIN при необходимости
 */

#include <WiFi.h>
#include <WebServer.h>
#include "driver/twai.h"
#include <esp_err.h>
#include <Preferences.h>
#include <string.h>
#include <stdio.h>
#include <sys/time.h>
#include <time.h>

// --------- CAN pins (ESP32) ----------
static constexpr gpio_num_t CAN_TX = GPIO_NUM_22;
static constexpr gpio_num_t CAN_RX = GPIO_NUM_23;

// SN65HVD230 / MCP2551 transceiver standby (RS) pin. Set to -1 if RS is tied to GND.
static constexpr int CAN_STANDBY_PIN = -1;

// --------- Wi-Fi AP ----------
const char* AP_SSID = "ESP32-CAN-Remote";
const char* AP_PASS = "12345678";
WebServer server(80);

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
static float intakeAirTemp = -99.0f;
static float batteryVolt = 0.0f;
static unsigned long lastCanRx = 0;
static unsigned long lastBsi260 = 0;
static unsigned long lastBsi276 = 0;
static bool wifiActive = false;
static String wifiIp;
static bool ignitionOn = true;
static bool lampsOn = false;
static unsigned long lastIgnitionFrame = 0;
static unsigned long lastLampsFrame = 0;
static Preferences prefs;
static constexpr const char* kPrefsNamespace = "psacan";
static constexpr const char* kPrefLang5 = "lang5";
static constexpr const char* kPrefIs24 = "is24h";
static constexpr const char* kPrefIsC = "isc";
static constexpr const char* kPrefRaw260 = "raw260";
static constexpr const char* kPrefEpoch = "epoch";
static constexpr const char* kPrefEpochTs = "epoch_ts";

static constexpr unsigned long IGNITION_TIMEOUT_MS = 2000;
static constexpr unsigned long LAMPS_TIMEOUT_MS = 3000;
static constexpr float INVALID_TEMP = -90.0f;
static constexpr float MIN_VALID_VOLT = 5.0f;
static constexpr unsigned long CAN_STALE_TIMEOUT_MS = 2000u;

// --------- BSI emulator state ----------
static constexpr uint32_t CAN_ID_260 = 0x260; // BSI broadcast settings
static constexpr uint32_t CAN_ID_15B = 0x15B; // SMEG -> write settings
static constexpr uint32_t CAN_ID_39B = 0x39B; // SMEG -> write date/time
static constexpr uint32_t CAN_ID_276 = 0x276; // BSI broadcast date/time
static constexpr unsigned long BSI_PERIOD_260_MS = 500;
static constexpr unsigned long BSI_PERIOD_276_MS = 1000;

// Peugeot 307 (full CAN) BSI broadcast 0x260 baseline.
static constexpr uint8_t kBsi260DefaultPeugeot307[8] = {
  0xB8, // Byte0: Lang=RU (0b01110), меню активно, остальные биты как в стоке
  0x44, // Byte1: метрическая система + конфигурационные флаги
  0x22, // Byte2: тип кузова/адаптации
  0x90, // Byte3: конфигурация приборов и кнопок на руле
  0x21, // Byte4: тип мультимедиа/телематики
  0x08, // Byte5: наличие ESP/ABS и датчиков парковки
  0x00, // Byte6: зарезервировано
  0x00  // Byte7: зарезервировано
};

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
static void bsiBuild276(uint8_t out[7]);
static void bsiSend260();
static void bsiSend276();
static void bsiTick();
static String frameHex(const uint8_t *data, size_t len);

static String iso8601Now();
static bool parseIso8601(const String &iso, time_t &outEpoch);

static void setIgnition(bool value);
static void setLamps(bool value);

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
  intakeAirTemp = -99.0f;
  batteryVolt = 0.0f;
  unsigned long now = millis();
  lastBsi260 = now;
  lastBsi276 = now;
  return twaiStart();
}

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

static inline void bsiSetBit(uint8_t &byte, uint8_t mask, bool value) {
  if (value) byte |= mask; else byte &= static_cast<uint8_t>(~mask);
}

static void bsiEnsureBaseline() {
  if (bsiHave260) return;
  memcpy(bsiState260, kBsi260DefaultPeugeot307, sizeof(bsiState260));
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
    bool allZero = true;
    for (size_t i = 0; i < sizeof(bsiState260); ++i) {
      if (bsiState260[i] != 0) { allZero = false; break; }
    }
    if (allZero) {
      memcpy(bsiState260, kBsi260DefaultPeugeot307, sizeof(bsiState260));
    }
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

static String frameHex(const uint8_t *data, size_t len) {
  String out;
  if (!data || len == 0) {
    return out;
  }
  out.reserve(len * 3 - 1);
  for (size_t i = 0; i < len; ++i) {
    if (i) out += ' ';
    char buf[3];
    snprintf(buf, sizeof(buf), "%02X", data[i]);
    out += buf;
  }
  return out;
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

static void setIgnition(bool value) {
  if (value != ignitionOn) {
    ignitionOn = value;
    Serial.printf("[POWER] Ignition %s\n", ignitionOn ? "ON" : "OFF");
  }
}

static void setLamps(bool value) {
  if (value != lampsOn) {
    lampsOn = value;
    Serial.printf("[POWER] Lamps %s\n", lampsOn ? "ON" : "OFF");
  }
}

static void readCanMessages() {
  twai_message_t msg;
  while (twai_receive(&msg, 0) == ESP_OK) {
    lastCanRx = millis();

    // Peugeot 308 Comfort CAN (AEE2004):
    // 0x0F6 - Температура воздуха на впуске и ОЖ (байты 0 и 1 со смещением -39°C)
    if (msg.identifier == 0x0F6) {
      if (msg.data_length_code >= 1) {
        intakeAirTemp = (float)msg.data[0] - 39.0f;
      }
      if (msg.data_length_code >= 2) {
        engineTemp = (float)msg.data[1] - 39.0f;
      }
    }

    // 0x0E6 - Напряжение АКБ (байт 5 в децивольтах, иногда байт 6)
    if (msg.identifier == 0x0E6) {
      float rawDecivolt = -1.0f;
      if (msg.data_length_code >= 6) {
        rawDecivolt = static_cast<float>(msg.data[5]);
      }
      if ((rawDecivolt <= 0.0f || rawDecivolt == 255.0f) && msg.data_length_code >= 7) {
        rawDecivolt = static_cast<float>(msg.data[6]);
      }
      if (rawDecivolt >= 0.0f) {
        batteryVolt = rawDecivolt / 10.0f;
      }
    }

    // 0x128 - Статусы (зажигание/+ACC в битах 6-7 первого байта)
    if (msg.identifier == 0x128) {
      if (msg.data_length_code >= 1) {
        bool ign = (msg.data[0] & 0xC0u) != 0;
        lastIgnitionFrame = millis();
        setIgnition(ign);
      }
    }

    if (msg.identifier == 0x0A8 && msg.data_length_code >= 3) {
      bool ign = (msg.data[2] & 0xC0u) != 0;
      lastIgnitionFrame = millis();
      setIgnition(ign);
    }

    if (msg.identifier == 0x1A8 && msg.data_length_code >= 1) {
      bool lamps = (msg.data[0] & 0x14u) != 0; // bit2=parking, bit4=low beam
      lastLampsFrame = millis();
      setLamps(lamps);
    }

    if (msg.identifier == CAN_ID_15B && msg.data_length_code > 0) {
      bsiHandle15B(msg.data, msg.data_length_code);
    }

    if (msg.identifier == CAN_ID_39B && msg.data_length_code >= 5) {
      bsiHandle39B(msg.data, msg.data_length_code);
    }
  }
}

static bool hasValidTemp(float value) { return value > INVALID_TEMP; }
static bool hasValidVoltage(float value) { return value > MIN_VALID_VOLT; }

static float displayTemp(float value) {
  return bsiIsCelsius ? value : (value * 9.0f / 5.0f + 32.0f);
}

static void appendTempJson(String &json, const char *key, float value) {
  json += '\"';
  json += key;
  json += '\"';
  json += ':';
  if (hasValidTemp(value)) {
    json += String(displayTemp(value), 1);
  } else {
    json += F("\"---\"");
  }
}

static String page() {
  String s; s.reserve(5200);
  s += F("<!doctype html><html lang='ru'><head><meta charset='utf-8'/><meta name='viewport' content='width=device-width,initial-scale=1'/><title>PSA CAN Remote</title>");
  s += F("<style>body{font-family:'Inter',system-ui,Arial,sans-serif;margin:0;padding:24px;background:#0f172a;color:#f8fafc;}main{max-width:960px;margin:0 auto;display:flex;flex-direction:column;gap:20px;}h2{margin:0 0 8px;font-size:26px;}h3{margin:0 0 12px;font-size:18px;color:#60a5fa;}.card{background:#16213b;border-radius:16px;padding:22px;box-shadow:0 12px 40px rgba(8,15,35,0.45);}.card p{margin:0 0 12px;}.metrics{display:grid;grid-template-columns:repeat(auto-fit,minmax(160px,1fr));gap:18px;}.metric .label{font-size:12px;color:#94a3b8;text-transform:uppercase;letter-spacing:.08em;margin-bottom:6px;}.metric .value{font-size:26px;font-weight:600;color:#f1f5f9;}.row{display:flex;flex-wrap:wrap;gap:10px;}.row button{flex:1 1 120px;padding:12px 16px;border-radius:10px;border:0;background:#1e293b;color:#e2e8f0;font-size:16px;cursor:pointer;transition:transform .12s,background .12s;}.row button:hover{transform:translateY(-1px);background:#334155;}.primary{background:#2563eb;color:#f8fafc;}.primary:hover{background:#1d4ed8;}.status{display:grid;grid-template-columns:repeat(auto-fit,minmax(160px,1fr));gap:16px;}.status .metric{padding:16px;border-radius:12px;background:#1e293b;}.small{font-size:14px;color:#94a3b8;margin-top:12px;}</style></head><body><main>");

  s += F("<div class='card'><h2>PSA Comfort CAN Remote</h2><p>ESP32 без дисплея. Управление CAN кнопками SMEG через Wi-Fi и мониторинг датчиков.</p><div class='status'>");
  s += F("<div class='metric'><div class='label'>CAN</div><div class='value' id='canState'>--</div></div>");
  s += F("<div class='metric'><div class='label'>Температура ОЖ</div><div class='value' id='temp'>--</div></div>");
  s += F("<div class='metric'><div class='label'>Температура воздуха</div><div class='value' id='iat'>--</div></div>");
  s += F("<div class='metric'><div class='label'>Напряжение АКБ</div><div class='value' id='volt'>--</div></div>");
  s += F("<div class='metric'><div class='label'>Зажигание</div><div class='value' id='ignition'>--</div></div>");
  s += F("<div class='metric'><div class='label'>AP IP</div><div class='value' id='ip'>--</div></div></div></div>");

  s += F("<div class='card'><h3>CAN скорость</h3><p class='small'>Комфортная шина AEE2004 работает на 125 кбит/с.</p><div class='row'><button class='primary' onclick=\"setSpeed('125')\">125 кбит/с</button></div></div>");

  s += F("<div class='card'><h3>Дата и время</h3><div class='metrics'><div class='metric'><div class='label'>Встроенные часы</div><div class='value' id='timeNow'>--</div></div></div><div class='row'><input type='datetime-local' id='timeInput'/><button class='primary' onclick='setTimeFromInput()'>Установить вручную</button><button onclick='syncTime()'>Синхронизировать с браузером</button></div><p class='small'>Используйте для быстрой синхронизации часов SMEG.</p></div>");

  s += F("<div class='card'><h3>Быстрые команды</h3><p class='small'>Имитация нажатий физических кнопок на панели SMEG.</p><div class='row'><button onclick='btn(17)'>⚙️ Settings</button><button onclick='btn(18)'>🗺️ Maps</button><button onclick='btn(19)'>📻 Radio</button><button onclick='btn(20)'>📞 Phone</button><button onclick='btn(29)'>📱 Apps</button><button onclick='btn(30)'>🛣️ Trip</button><button onclick='btn(32)'>❄️ AC</button></div></div>");

  s += F("</main><footer style='text-align:center;color:#475569;font-size:12px;margin-top:24px;'>Comfort CAN remote • ESP32</footer><script>");
  s += F("async function fetchData(){const res=await fetch('/data');if(!res.ok) return;const j=await res.json();document.getElementById('temp').textContent=j.temp;document.getElementById('iat').textContent=j.iat;document.getElementById('volt').textContent=j.volt;document.getElementById('ignition').textContent=j.ignition?'ON':'OFF';document.getElementById('ip').textContent=j.ip;document.getElementById('timeNow').textContent=j.time;document.getElementById('canState').textContent=j.canOk?('OK @'+j.speed):('Нет данных @'+j.speed);}async function btn(n){await fetch('/btn?n='+n);}async function setSpeed(v){await fetch('/cfg?speed='+v);}async function syncTime(){const now=new Date();const iso=now.toISOString().slice(0,19);await fetch('/time?iso='+iso);}async function setTimeFromInput(){const el=document.getElementById('timeInput');if(!el.value) return;await fetch('/time?iso='+el.value+':00');}setInterval(fetchData,1200);fetchData();</script></body></html>");
  return s;
}

static void handleRoot() {
  server.send(200, "text/html; charset=utf-8", page());
}

static void handleData() {
  String json(F("{"));
  json.reserve(360);

  appendTempJson(json, "temp", engineTemp);
  json += ',';
  appendTempJson(json, "iat", intakeAirTemp);

  json += F(",\"volt\":");
  if (hasValidVoltage(batteryVolt)) {
    json += String(batteryVolt, 1);
  } else {
    json += F("\"---\"");
  }

  json += F(",\"speed\":\"");
  json += bitrateName(currentBitrate);
  json += '\"';

  json += F(",\"is24\":");
  json += bsiIs24h ? F("true") : F("false");

  json += F(",\"isC\":");
  json += bsiIsCelsius ? F("true") : F("false");

  json += F(",\"time\":\"");
  json += iso8601Now();
  json += '\"';

  json += F(",\"ip\":\"");
  if (wifiActive && wifiIp.length()) {
    json += wifiIp;
  } else {
    json += F("--");
  }
  json += '\"';

  const bool canOk = (millis() - lastCanRx) < CAN_STALE_TIMEOUT_MS;
  json += F(",\"canOk\":");
  json += canOk ? F("true") : F("false");

  json += F(",\"ignition\":");
  json += ignitionOn ? F("true") : F("false");

  json += F(",\"uptime\":");
  json += String(static_cast<uint32_t>(millis() / 1000u));

  bsiEnsureBaseline();
  bsiApplyLangUnits();
  String bsi260Hex = frameHex(bsiState260, sizeof(bsiState260));
  uint8_t bsi276Payload[7];
  bsiBuild276(bsi276Payload);
  String bsi276Hex = frameHex(bsi276Payload, sizeof(bsi276Payload));

  json += F(",\"bsi260\":\"");
  json += bsi260Hex;
  json += '\"';

  json += F(",\"bsi276\":\"");
  json += bsi276Hex;
  json += '\"';

  json += '}';
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

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[PSA CAN Remote] ESP32 headless");

  prefs.begin(kPrefsNamespace, false);
  bsiLoadState();

  if (CAN_STANDBY_PIN >= 0) {
    pinMode(CAN_STANDBY_PIN, OUTPUT);
    digitalWrite(CAN_STANDBY_PIN, LOW);
  }

  WiFi.mode(WIFI_AP);
  if (WiFi.softAP(AP_SSID, AP_PASS)) {
    wifiActive = true;
    wifiIp = WiFi.softAPIP().toString();
    Serial.printf("[WiFi] AP started: %s (%s)\n", AP_SSID, wifiIp.c_str());
  } else {
    Serial.println("[WiFi] Failed to start AP");
  }

  if (!twaiStart()) Serial.println("[CAN] init failed");

  server.on("/", HTTP_GET, handleRoot);
  server.on("/btn", HTTP_GET, handleBtn);
  server.on("/data", HTTP_GET, handleData);
  server.on("/cfg", HTTP_GET, handleCfg);
  server.on("/prefs", HTTP_GET, handlePrefs);
  server.on("/time", HTTP_GET, handleTime);
  server.begin();
  Serial.println("[HTTP] Server started on :80");
}

void loop() {
  server.handleClient();
  readCanMessages();
  unsigned long now = millis();
  if (now - lastIgnitionFrame > IGNITION_TIMEOUT_MS) {
    setIgnition(false);
  }
  if (lampsOn && (now - lastLampsFrame > LAMPS_TIMEOUT_MS)) {
    setLamps(false);
  }
  bsiTick();
}

