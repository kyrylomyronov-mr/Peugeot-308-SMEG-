/*
 * PSA Comfort CAN Remote — BSI Emulator
 * Board  : Waveshare ESP32-C6-Zero
 * CAN    : 125 кбит/с, TX=GP22, RX=GP21
 * RTC    : DS3231 via I2C, SDA=GP19, SCL=GP20
 * LED    : Onboard WS2812 RGB (GP8) — status indicator
 * OTA    : Wi-Fi OTA (ArduinoOTA) hostname: psa-can
 * SN65HVD230 RS (standby) pin: set CAN_STANDBY_PIN accordingly
 */

#include "driver/twai.h"
#include <Adafruit_NeoPixel.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <RTClib.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>
#include "page_html.h"
#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_sleep.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

// --------- CAN0 pins (Car Side) ----------
static constexpr gpio_num_t CAN0_TX = GPIO_NUM_22;
static constexpr gpio_num_t CAN0_RX = GPIO_NUM_21;

// --------- CAN1 pins (Radio Side) ----------
static constexpr gpio_num_t CAN1_TX = GPIO_NUM_19;
static constexpr gpio_num_t CAN1_RX = GPIO_NUM_20;

static constexpr int CAN_STANDBY_PIN = -1;

static twai_handle_t can0_handle = NULL;
static twai_handle_t can1_handle = NULL;

// --------- Onboard WS2812 RGB LED (GP8) ----------
static constexpr int LED_PIN = 8;
static constexpr uint8_t LED_BRIGHT = 30; // 0-255
static Adafruit_NeoPixel
    led(1, LED_PIN, NEO_RGB + NEO_KHZ800); // Changed GRB to RGB due to swap

// --------- RTC (DS3231) ----------
static constexpr int RTC_SDA_PIN = 15;
static constexpr int RTC_SCL_PIN = 18;
static RTC_DS3231 rtc;
static bool rtcOk = false;
static bool rtcSynced = false; // true once valid time is known

// --------- Wi-Fi AP ----------
const char *AP_SSID = "ESP32-CAN-Remote";
const char *AP_PASS = "12345678";
WebServer server(80);

// --------- Known button indices ----------
static constexpr uint8_t BTN_SETTINGS = 17;
static constexpr uint8_t BTN_MAPS = 18;
static constexpr uint8_t BTN_RADIO = 19;
static constexpr uint8_t BTN_PHONE = 20;
static constexpr uint8_t BTN_APPS = 29;
static constexpr uint8_t BTN_TRIP = 30;
static constexpr uint8_t BTN_AC = 32;

// --------- CAN state ----------
static bool can0Started = false;
static bool can1Started = false;
static unsigned long lastCanRx = 0;
static bool canEverReceived = false;
static uint32_t can0ValidFrames = 0;
static uint32_t can1ValidFrames = 0;
static bool wifiActive = false;
static String wifiIp;
static bool ignitionOn = false; // false until confirmed by CAN
static unsigned long lastIgnitionFrame = 0;
static unsigned long lastCar260 = 0; // Track last car 0x260 to inhibit synthetic ticks
static Preferences prefs;

static constexpr const char *kPrefsNamespace = "psacan";
static constexpr const char *kPrefLang5 = "lang5";
static constexpr const char *kPrefIs24 = "is24h";
static constexpr const char *kPrefIsC = "isc";
static constexpr const char *kPrefRaw260 = "raw260";
static constexpr const char *kPrefP260 = "p260";
static constexpr const char *kPrefP276 = "p276";

static constexpr unsigned long IGNITION_TIMEOUT_MS = 2000;
static constexpr unsigned long CAN_STALE_TIMEOUT_MS = 2000u;
static constexpr unsigned long SLEEP_TIMEOUT_MS =
    60000u; // 60 seconds of CAN inactivity

// --------- BSI emulator state ----------
static constexpr uint32_t CAN_ID_260 = 0x260;     // BSI broadcast settings
static constexpr uint32_t CAN_ID_15B = 0x15B;     // SMEG -> write settings
static constexpr uint32_t CAN_ID_39B = 0x39B;     // SMEG -> write date/time
static constexpr uint32_t CAN_ID_276 = 0x276;     // BSI broadcast date/time
static constexpr uint32_t ID_EMF_BUTTONS = 0x21F; // EMF-A menu remote buttons
static unsigned long bsiPeriod260 = 500;
static unsigned long bsiPeriod276 = 1000;
static unsigned long lastBsi260 = 0;
static unsigned long lastBsi276 = 0;

// Peugeot 307 Full CAN BSI 0x260 baseline
static constexpr uint8_t kBsi260DefaultPeugeot307[8] = {
    0xB8, // Byte0: Lang=RU (0b01110), menu active
    0x44, // Byte1: metric system + config flags
    0x22, // Byte2: body/adaptation type
    0x90, // Byte3: instrument/steering buttons config
    0x21, // Byte4: multimedia/telematics type
    0x08, // Byte5: ESP/ABS/parking sensors
    0x00, // Byte6: reserved
    0x00  // Byte7: reserved
};

static uint8_t bsiState260[8] = {0};
static bool bsiHave260 = false;
static uint8_t bsiLang5 = 0b01110; // default Russian
static bool bsiIs24h = true;
static bool bsiIsCelsius = true;

struct GatewayConfig {
  bool swapDoors = false;
  bool parkingFront = true;
  bool parkingRear = true;
  bool drl = true;
  bool moodLight = true;
} gwConfig;

// --------- Forward declarations ----------
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
static void setWifiActive(bool on);
static void setIgnition(bool value);
static void rtcInit();
static void rtcWriteTime(time_t epoch);
static void updateLed();
static void enterLightSleep();

// --------- LED status ----------
// Green       = CAN OK + RTC synced    (everything good)
// Dim green   = CAN OK, RTC unknown
// Red         = no CAN data
// Yellow blink= RTC not synced (time unknown)
// Orange blink= no CAN + RTC not synced
static void updateLed() {
  static unsigned long lastLedUpdate = 0;
  static bool blinkState = false;
  unsigned long now = millis();
  if (now - lastLedUpdate < 500)
    return;
  lastLedUpdate = now;
  blinkState = !blinkState;

  // canOk requires >= 3 received frames to filter out pin-noise false-positives
  bool canOk =
      ((can0ValidFrames >= 3) || (can1ValidFrames >= 3)) && 
      ((now - lastCanRx) < CAN_STALE_TIMEOUT_MS);

  uint32_t colour;
  if (canOk && rtcSynced) {
    // All good — solid green
    colour = led.Color(0, LED_BRIGHT, 0);
  } else if (canOk && !rtcSynced) {
    // CAN OK but time unknown — yellow blink (R+G)
    colour = blinkState ? led.Color(LED_BRIGHT, LED_BRIGHT, 0) : 0;
  } else if (!canOk && rtcSynced) {
    // No CAN data — solid red
    colour = led.Color(LED_BRIGHT, 0, 0);
  } else {
    // Both bad — "orange" blink (R + tiny G)
    colour = blinkState ? led.Color(LED_BRIGHT, LED_BRIGHT / 8, 0) : 0;
  }

  led.setPixelColor(0, colour);
  led.show();
}

// --------- RTC ----------
static void rtcInit() {
  Wire.begin(RTC_SDA_PIN, RTC_SCL_PIN);
  if (!rtc.begin(&Wire)) {
    Serial.println("[RTC] DS3231 not found! Check wiring (SDA=GP15, SCL=GP18)");
    rtcOk = false;
    return;
  }
  rtcOk = true;
  if (rtc.lostPower()) {
    Serial.println("[RTC] power was lost, time not set — set via web or CAN");
    return; // rtcSynced stays false
  }
  // Sync ESP32 system clock from DS3231
  DateTime now = rtc.now();
  // Validate: year must be plausible (DS3231 uninitialized shows 2000-01-01)
  if (now.year() < 2020 || now.year() > 2099) {
    Serial.printf("[RTC] implausible year %d, treating as not synced\n",
                  now.year());
    return; // rtcSynced stays false
  }
  time_t epoch = now.unixtime();
  struct timeval tv = {.tv_sec = epoch, .tv_usec = 0};
  settimeofday(&tv, nullptr);
  rtcSynced = true;
  Serial.printf("[RTC] time loaded: %04d-%02d-%02dT%02d:%02d:%02d\n",
                now.year(), now.month(), now.day(), now.hour(), now.minute(),
                now.second());
}

static void rtcWriteTime(time_t epoch) {
  if (!rtcOk)
    return;
  DateTime dt(static_cast<uint32_t>(epoch));
  rtc.adjust(dt);
  rtcSynced = true; // valid time is now known
  Serial.printf("[RTC] time written: %04d-%02d-%02dT%02d:%02d:%02d\n",
                dt.year(), dt.month(), dt.day(), dt.hour(), dt.minute(),
                dt.second());
}

// --------- Wi-Fi ----------
static void setWifiActive(bool on) {
  if (on) {
    if (!wifiActive) {
      WiFi.mode(WIFI_AP);
      WiFi.softAP(AP_SSID, AP_PASS);
      wifiActive = true;
      wifiIp = WiFi.softAPIP().toString();
      Serial.printf("[WiFi] AP: %s, IP: %s\n", AP_SSID, wifiIp.c_str());
    }
  } else {
    if (wifiActive) {
      WiFi.softAPdisconnect(true);
      WiFi.mode(WIFI_OFF);
      wifiActive = false;
      wifiIp = String();
      Serial.println("[WiFi] AP disabled");
    }
  }
}

static void setIgnition(bool value) {
  if (value != ignitionOn) {
    ignitionOn = value;
    Serial.printf("[POWER] Ignition %s\n", ignitionOn ? "ON" : "OFF");
  }
}

// --------- Sleep ----------
static void enterLightSleep() {
  Serial.println("[SLEEP] Entering Light Sleep due to CAN inactivity...");

  // Turn off LED
  led.setPixelColor(0, 0);
  led.show();

  // Disable Wi-Fi to save power
  setWifiActive(false);

  // Put SN65HVD230 to standby (if pin is defined)
  if (CAN_STANDBY_PIN >= 0) {
    digitalWrite(CAN_STANDBY_PIN, HIGH);
  }

  // Stop TWAI so we can use CAN0_RX/CAN1_RX as standard GPIO wakeups
  if (can0Started && can0_handle) {
    twai_stop_v2(can0_handle);
    twai_driver_uninstall_v2(can0_handle);
    can0Started = false;
  }
  if (can1Started && can1_handle) {
    twai_stop_v2(can1_handle);
    twai_driver_uninstall_v2(can1_handle);
    can1Started = false;
  }

  // Wake up when either CAN_RX goes LOW (dominant state)
  gpio_reset_pin(CAN0_RX);
  gpio_reset_pin(CAN1_RX);
  gpio_set_direction(CAN0_RX, GPIO_MODE_INPUT);
  gpio_set_direction(CAN1_RX, GPIO_MODE_INPUT);
  gpio_set_pull_mode(CAN0_RX, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode(CAN1_RX, GPIO_PULLUP_ONLY);
  gpio_wakeup_enable(CAN0_RX, GPIO_INTR_LOW_LEVEL);
  gpio_wakeup_enable(CAN1_RX, GPIO_INTR_LOW_LEVEL);
  esp_sleep_enable_gpio_wakeup();

  // Flush serial log before sleeping
  Serial.flush();

  // Enter light sleep
  esp_light_sleep_start();

  // Debounce wakeup and wait for transceiver
  delay(10);

  // === WOKEN UP ===
  Serial.println("[SLEEP] Woke up from CAN activity!");

  // Wake up transceiver
  if (CAN_STANDBY_PIN >= 0) {
    digitalWrite(CAN_STANDBY_PIN, LOW);
  }

  // Reinitialize interfaces
  twaiStart();
  setWifiActive(true);

  // Reset timeout so we don't sleep immediately again
  lastCanRx = millis();
}


// --------- CAN Init ----------
static bool twaiStart() {
  twai_timing_config_t t = TWAI_TIMING_CONFIG_125KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Настройка Car Side (CAN0)
  if (!can0Started) {
    twai_general_config_t g0 =
        TWAI_GENERAL_CONFIG_DEFAULT(CAN0_TX, CAN0_RX, TWAI_MODE_NORMAL);
    g0.controller_id = 0;
    g0.tx_queue_len = 16;
    g0.rx_queue_len = 16;

    if (twai_driver_install_v2(&g0, &t, &f, &can0_handle) == ESP_OK) {
      twai_start_v2(can0_handle);
      can0Started = true;
      Serial.println("[CAN] Car side (CAN0) started @125k");
    } else {
      Serial.println("[CAN] Car side install failed");
    }
  }

  // Настройка Radio Side (CAN1)
  if (!can1Started) {
    twai_general_config_t g1 =
        TWAI_GENERAL_CONFIG_DEFAULT(CAN1_TX, CAN1_RX, TWAI_MODE_NORMAL);
    g1.controller_id = 1;
    g1.tx_queue_len = 16;
    g1.rx_queue_len = 16;

    if (twai_driver_install_v2(&g1, &t, &f, &can1_handle) == ESP_OK) {
      twai_start_v2(can1_handle);
      can1Started = true;
      Serial.println("[CAN] Radio side (CAN1) started @125k");
    } else {
      Serial.println("[CAN] Radio side install failed");
    }
  }

  return can0Started && can1Started;
}

// Removing obsolete twaiReconfigure as we only use 125k now
// --------- CAN TX ----------
static bool canSend(twai_handle_t handle, uint32_t id, const uint8_t *d,
                    uint8_t dlc) {
  if (!handle || dlc > 8)
    return false;
  twai_message_t m = {};
  m.identifier = id;
  m.flags = 0;
  m.data_length_code = dlc;
  if (dlc && d)
    memcpy(m.data, d, dlc);
  return (twai_transmit_v2(handle, &m, pdMS_TO_TICKS(20)) == ESP_OK);
}

static bool canSendRadio(uint32_t id, const uint8_t *d, uint8_t dlc) {
  return canSend(can1_handle, id, d, dlc);
}

static bool canSendCar(uint32_t id, const uint8_t *d, uint8_t dlc) {
  return canSend(can0_handle, id, d, dlc);
}

static bool canSend122(const uint8_t *d, uint8_t dlc) {
  return canSendRadio(0x122, d, dlc); // Buttons go to Radio
}


static inline void sendIdle122() {
  uint8_t z[8] = {0};
  canSend122(z, 8);
}

// Forward declaration to allow pressIndex to read CAN during delays
static void processBus(twai_handle_t src, twai_handle_t dst, bool isCarSide);

// Safe delay that yields to CAN processing to prevent RX buffer overflow
static void safeDelay(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    if (can0Started && can1Started) {
      processBus(can0_handle, can1_handle, true);
      processBus(can1_handle, can0_handle, false);
    }
    yield();
  }
}

static void pressIndex(uint8_t idx, uint16_t press_ms = 75) {
  if (idx < 1 || idx > 40)
    return;
  uint8_t z = (uint8_t)(idx - 1);
  uint8_t byteIdx = z / 8;
  uint8_t bitIdx = z % 8;

  uint8_t d[8] = {0};
  d[byteIdx] = (uint8_t)(1u << bitIdx);
  canSend122(d, 8);
  safeDelay(press_ms); // Non-blocking
  sendIdle122();
  safeDelay(120); // Non-blocking
}

// --------- BSI Helpers ----------
static inline void bsiSetBit(uint8_t &byte, uint8_t mask, bool value) {
  if (value)
    byte |= mask;
  else
    byte &= static_cast<uint8_t>(~mask);
}

static void bsiEnsureBaseline() {
  if (bsiHave260)
    return;
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
  prefs.putULong(kPrefP260, bsiPeriod260);
  prefs.putULong(kPrefP276, bsiPeriod276);
  prefs.putBool("swpD", gwConfig.swapDoors); // Added persistence
  prefs.putBytes(kPrefRaw260, bsiState260, sizeof(bsiState260));
}

static void bsiLoadState() {
  size_t sz = prefs.getBytesLength(kPrefRaw260);
  if (sz >= sizeof(bsiState260)) {
    prefs.getBytes(kPrefRaw260, bsiState260, sizeof(bsiState260));
    bool allZero = true;
    for (size_t i = 0; i < sizeof(bsiState260); ++i) {
      if (bsiState260[i] != 0) {
        allZero = false;
        break;
      }
    }
    // Fix: Validate against partial-write or corrupt zero buffers.
    // If it is 0x00 or has bad placeholders, fall back.
    if (allZero || bsiState260[0] == 0) {
      memcpy(bsiState260, kBsi260DefaultPeugeot307, sizeof(bsiState260));
    }
    bsiHave260 = true;
  } else {
    bsiHave260 = false;
  }

  bsiLang5 = prefs.getUChar(kPrefLang5, bsiLang5);
  bsiIs24h = prefs.getBool(kPrefIs24, bsiIs24h);
  bsiIsCelsius = prefs.getBool(kPrefIsC, bsiIsCelsius);
  bsiPeriod260 = prefs.getULong(kPrefP260, 500);
  bsiPeriod276 = prefs.getULong(kPrefP276, 1000);
  gwConfig.swapDoors = prefs.getBool("swpD", false); // Added loading

  bsiEnsureBaseline();
  bsiApplyLangUnits();
  // Time is handled by rtcInit() — not stored in NVS
}

static void bsiPersistTime(time_t epoch) {
  // Write to DS3231 hardware RTC (battery-backed, no NVS needed)
  rtcWriteTime(epoch);
}

static void bsiHandle15B(const uint8_t *data, uint8_t len) {
  if (!data || len == 0)
    return;
  if (len > 8)
    len = 8;
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
  if (!data || len < 5)
    return;

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
  if (ct.month == 0 || ct.day == 0) return;

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
    struct timeval tv = {.tv_sec = epoch, .tv_usec = 0};
    settimeofday(&tv, nullptr);
    bsiPersistTime(epoch);
  }

  bsiSend276();
}

static void bsiBuild276(uint8_t out[7]) {
  time_t nowEpoch = time(nullptr);
  struct tm tmv;
  struct tm *t =
      localtime_r(&nowEpoch, &tmv); // reentrant — safe in ISR/callbacks
  uint16_t year = t ? static_cast<uint16_t>(t->tm_year + 1900) : 2000u;
  uint8_t mon = t ? static_cast<uint8_t>(t->tm_mon + 1) : 1u;
  uint8_t day = t ? static_cast<uint8_t>(t->tm_mday) : 1u;
  uint8_t hour = t ? static_cast<uint8_t>(t->tm_hour) : 0u;
  uint8_t min = t ? static_cast<uint8_t>(t->tm_min) : 0u;

  out[0] = (bsiIs24h ? 0x80u : 0x00u) |
           static_cast<uint8_t>((year >= 2000u) ? (year - 2000u) : 0u);
  out[1] = mon & 0x0Fu;
  out[2] = day & 0x1Fu;
  out[3] = hour & 0x1Fu;
  out[4] = min & 0x3Fu;
  out[5] = 0x3Fu;
  out[6] = 0xFEu;
}

static void bsiSend260() {
  if (!can1Started)
    return;
  bsiApplyLangUnits();
  if (canSendRadio(CAN_ID_260, bsiState260, sizeof(bsiState260))) {
    lastBsi260 = millis();
  }
}

static void bsiSend276() {
  if (!can1Started)
    return;
  uint8_t payload[7];
  bsiBuild276(payload);
  if (canSendRadio(CAN_ID_276, payload, sizeof(payload))) {
    lastBsi276 = millis();
  }
}

static void bsiTick() {
  if (!can1Started)
    return;
  unsigned long now = millis();
  if (now - lastBsi260 >= bsiPeriod260) {
    if (now - lastCar260 > 2000) {
      bsiSend260();
    }
  }
  if (now - lastBsi276 >= bsiPeriod276)
    bsiSend276();
}

static String frameHex(const uint8_t *data, size_t len) {
  String out;
  if (!data || len == 0)
    return out;
  out.reserve(len * 3 - 1);
  for (size_t i = 0; i < len; ++i) {
    if (i)
      out += ' ';
    char buf[3];
    snprintf(buf, sizeof(buf), "%02X", data[i]);
    out += buf;
  }
  return out;
}

static String iso8601FromTm(const struct tm &tmv) {
  char buf[32];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02d",
           tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday, tmv.tm_hour,
           tmv.tm_min, tmv.tm_sec);
  return String(buf);
}

static String iso8601Now() {
  time_t nowEpoch = time(nullptr);
  struct tm tmv;
  if (!localtime_r(&nowEpoch, &tmv))
    return String(F("1970-01-01T00:00:00"));
  return iso8601FromTm(tmv);
}

static bool parseIso8601(const String &iso, time_t &outEpoch) {
  int year = 0, month = 0, day = 0, hour = 0, minute = 0, second = 0;
  int matched = sscanf(iso.c_str(), "%d-%d-%dT%d:%d:%d", &year, &month, &day,
                       &hour, &minute, &second);
  if (matched < 5)
    return false;
  if (matched < 6)
    second = 0;
  if (year < 1970 || month < 1 || month > 12 || day < 1 || day > 31 ||
      hour < 0 || hour > 23 || minute < 0 || minute > 59 || second < 0 ||
      second > 59)
    return false;

  struct tm tmv = {};
  tmv.tm_year = year - 1900;
  tmv.tm_mon = month - 1;
  tmv.tm_mday = day;
  tmv.tm_hour = hour;
  tmv.tm_min = minute;
  tmv.tm_sec = second;
  tmv.tm_isdst = -1;

  time_t epoch = mktime(&tmv);
  if (epoch == static_cast<time_t>(-1))
    return false;
  outEpoch = epoch;
  return true;
}

// --------- CAN RX ----------
// --------- BRIDGE LOGIC ----------
static void processBus(twai_handle_t src, twai_handle_t dst, bool isCarSide) {
  twai_message_t msg;
  while (twai_receive_v2(src, &msg, 0) == ESP_OK) {
    if (msg.rtr)
      continue;
    lastCanRx = millis();
    canEverReceived = true;
    bool forward = true; // Added forwarding flag

    if (isCarSide) {
      if (can0ValidFrames < UINT32_MAX)
        can0ValidFrames++;

      if (msg.identifier == 0x276)
        continue; // Block BSI clock broadcast

      // Overrides 0x260 Menu Unlock
      if (msg.identifier == 0x260 && msg.data_length_code >= 5) {
        lastCar260 = millis(); // Track active car 0x260
        msg.data[0] = (msg.data[0] & 0x03u) | ((bsiLang5 & 0x1Fu) << 2) | 0x80u;
        if (gwConfig.parkingRear)
          msg.data[1] |= 0x80;
        if (gwConfig.parkingFront)
          msg.data[1] |= 0x40;
        if (gwConfig.drl)
          msg.data[2] |= 0x10;
        if (gwConfig.moodLight)
          msg.data[2] |= 0x08;
      }

      // Door Fix 0x220
      if (msg.identifier == 0x220 && gwConfig.swapDoors &&
          msg.data_length_code >= 3) {
        uint8_t d = msg.data[2];
        uint8_t newD = d & 0x0F;
        if (d & 0x10) newD |= 0x20;
        if (d & 0x20) newD |= 0x10;
        if (d & 0x40) newD |= 0x80;
        if (d & 0x80) newD |= 0x40;
        msg.data[2] = newD;
      }

      // Ignition Trigger OR check
      bool ignOn = false;
      bool hasIgnFrame = false;
      if (msg.identifier == 0x128 && msg.data_length_code >= 1) {
        if ((msg.data[0] & 0xC0u) != 0) ignOn = true;
        hasIgnFrame = true;
      }
      if (msg.identifier == 0x0A8 && msg.data_length_code >= 3) {
        if ((msg.data[2] & 0xC0u) != 0) ignOn = true;
        hasIgnFrame = true;
      }
      if (hasIgnFrame) {
        lastIgnitionFrame = millis();
        setIgnition(ignOn);
      }

      // Trip Computer Translation 0x221 -> 0x33B
      if (msg.identifier == 0x221 && msg.data_length_code >= 5) {
        uint8_t bc[8] = {0};
        bc[0] = msg.data[0];
        bc[1] = msg.data[1];
        bc[2] = msg.data[2];
        bc[4] = msg.data[4];
        canSend(dst, 0x33B, bc, 8);
        forward = false; // Block downstream prop of 0x221
      }
    } else {
      if (can1ValidFrames < UINT32_MAX)
        can1ValidFrames++;

      // Intercept 0x15B Settings sync
      if (msg.identifier == 0x15B && msg.data_length_code >= 4 &&
          msg.data[0] == 0x01) {
        bool newDrl = (msg.data[2] & 0x10) != 0;
        if (newDrl != gwConfig.drl) {
          gwConfig.drl = newDrl;
        }
        // Вызываем полный обработчик (для языков и парктроников)
        bsiHandle15B(msg.data, msg.data_length_code);
      }

      // SMEG Time Sync 0x39B
      if (msg.identifier == 0x39B && msg.data_length_code >= 5) {
        bsiHandle39B(
            msg.data,
            msg.data_length_code); // Utilizes old working logic to sync RTC
      }
    }

    if (forward) {
      canSend(dst, msg.identifier, msg.data, msg.data_length_code);
    }
  }
}

// --------- HTTP ----------
static void handleRoot() {
  server.send_P(200, "text/html; charset=utf-8", page_html);
}

static void handleData() {
  String json(F("{"));
  json.reserve(300);

  json += F("\"is24\":");
  json += bsiIs24h ? F("true") : F("false");

  json += F(",\"isC\":");
  json += bsiIsCelsius ? F("true") : F("false");

  json += F(",\"time\":\"");
  json += iso8601Now();
  json += '"';

  json += F(",\"ip\":\"");
  if (wifiActive && wifiIp.length()) {
    json += wifiIp;
  } else {
    json += F("--");
  }
  json += '"';

  // canOk is false until at least one frame arrives, then uses stale timeout
  const bool canOk =
      canEverReceived && ((millis() - lastCanRx) < CAN_STALE_TIMEOUT_MS);
  json += F(",\"canOk\":");
  json += canOk ? F("true") : F("false");

  json += F(",\"frames0\":");
  json += String(can0ValidFrames);
  json += F(",\"frames1\":");
  json += String(can1ValidFrames);

  json += F(",\"ignition\":");
  json += ignitionOn ? F("true") : F("false");

  json += F(",\"uptime\":");
  json += String(static_cast<uint32_t>(millis() / 1000u));

  json += F(",\"lang\":");
  json += String(bsiLang5);

  json += F(",\"p260\":");
  json += String(bsiPeriod260);
  json += F(",\"p276\":");
  json += String(bsiPeriod276);

  bsiEnsureBaseline();
  bsiApplyLangUnits();
  String bsi260Hex = frameHex(bsiState260, sizeof(bsiState260));
  uint8_t bsi276Payload[7];
  bsiBuild276(bsi276Payload);
  String bsi276Hex = frameHex(bsi276Payload, sizeof(bsi276Payload));

  json += F(",\"bsi260\":\"");
  json += bsi260Hex;
  json += '"';

  json += F(",\"bsi276\":\"");
  json += bsi276Hex;
  json += '"';

  json += '}';
  server.send(200, "application/json", json);
}

static void handleBtn() {
  if (!server.hasArg("n")) {
    server.send(400, "text/plain", "Missing param");
    return;
  }
  int n = server.arg("n").toInt();
  // Fix: Validate bounds BEFORE casting to uint8_t to prevent negative cast
  // bypass
  if (n < 1 || n > 40) {
    server.send(400, "text/plain", "Invalid index");
    return;
  }
  if (!(n == BTN_SETTINGS || n == BTN_MAPS || n == BTN_RADIO ||
        n == BTN_PHONE || n == BTN_APPS || n == BTN_TRIP || n == BTN_AC)) {
    server.send(400, "text/plain", "Unavailable button");
    return;
  }
  pressIndex((uint8_t)n, 75);
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

  if (server.hasArg("lang")) {
    uint8_t newVal = static_cast<uint8_t>(server.arg("lang").toInt() & 0x1F);
    if (newVal != bsiLang5) {
      bsiLang5 = newVal;
      changed = true;
      need260 = true;
    }
  }

  if (server.hasArg("p260")) {
    unsigned long val = server.arg("p260").toInt();
    if (val >= 10 && val <= 10000 && val != bsiPeriod260) {
      bsiPeriod260 = val;
      changed = true;
    }
  }

  if (server.hasArg("p276")) {
    unsigned long val = server.arg("p276").toInt();
    if (val >= 10 && val <= 10000 && val != bsiPeriod276) {
      bsiPeriod276 = val;
      changed = true;
    }
  }

  if (changed) {
    bsiSaveState();
    if (need260)
      bsiSend260();
    if (need276)
      bsiSend276();
  }
  server.send(200, "text/plain", "OK");
}

static void handleEmf() {
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  if (!server.hasArg("c")) {
    server.send(400, "text/plain", "Err");
    return;
  }
  int c = server.arg("c").toInt();
  uint8_t d[8] = {0};

  switch (c) {
  case 1:
    d[0] = 0x01;
    break; // Menu
  case 2:
    d[0] = 0x02;
    break; // OK
  case 3:
    d[0] = 0x04;
    break; // ESC
  case 4:
    d[2] = 0x01;
    break; // Left
  case 5:
    d[2] = 0x02;
    break; // Up
  case 6:
    d[2] = 0x04;
    break; // Down
  case 7:
    d[2] = 0x08;
    break; // Right
  }

  canSendCar(ID_EMF_BUTTONS, d, 8);
  server.send(200, "text/plain", "OK"); // Send first
  safeDelay(100);
  uint8_t idle[8] = {0};
  canSendCar(ID_EMF_BUTTONS, idle, 8);
}

static void handleConf() {
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  gwConfig.swapDoors = !gwConfig.swapDoors; // Toggle
  bsiSaveState(); // Persist setting
  server.sendHeader("Location", "/");
  server.send(303);
}

static void handleTime() {
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
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

  struct timeval tv = {.tv_sec = epoch, .tv_usec = 0};
  settimeofday(&tv, nullptr);
  bsiPersistTime(epoch);
  bsiSend276();
  server.send(200, "text/plain", "OK");
}

// --------- Setup / Loop ----------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[PSA CAN] ESP32-C6 BSI Emulator");

  led.begin();
  led.setPixelColor(0, led.Color(0, 0, LED_BRIGHT)); // blue = booting
  led.show();

  prefs.begin(kPrefsNamespace, false);
  bsiLoadState();
  rtcInit(); // sync system clock from DS3231 (must be after Wire is available)
  setWifiActive(true);

  if (CAN_STANDBY_PIN >= 0) {
    pinMode(CAN_STANDBY_PIN, OUTPUT);
    digitalWrite(CAN_STANDBY_PIN, LOW);
  }

  if (!twaiStart()) {
    Serial.println("[CAN] init failed");
  } else {
    // Prevent immediate sleep on boot
    lastCanRx = millis();
  }

  server.on("/", HTTP_GET, handleRoot);
  server.on("/btn", HTTP_GET, handleBtn);
  server.on("/data", HTTP_GET, handleData);
  server.on("/prefs", HTTP_GET, handlePrefs);
  server.on("/time", HTTP_GET, handleTime);
  server.on("/emf", HTTP_GET, handleEmf);
  server.on("/conf", HTTP_GET, handleConf);
  server.begin();
  Serial.println("[HTTP] Server started on :80");

  // OTA firmware update over Wi-Fi
  ArduinoOTA.setHostname("psa-can");
  ArduinoOTA.onStart([]() {
    Serial.println("[OTA] start");
    led.setPixelColor(0, led.Color(0, 0, LED_BRIGHT));
    led.show(); // blue during OTA
  });
  ArduinoOTA.onProgress([](unsigned int done, unsigned int total) {
    // pulse white proportional to progress
    uint8_t v = (uint8_t)(LED_BRIGHT * done / total);
    led.setPixelColor(0, led.Color(v, v, v));
    led.show();
  });
  ArduinoOTA.onEnd([]() { Serial.println("[OTA] done"); });
  ArduinoOTA.onError(
      [](ota_error_t e) { Serial.printf("[OTA] error %u\n", e); });
  ArduinoOTA.begin();
  Serial.println("[OTA] ready — hostname: psa-can");
}

void loop() {
  ArduinoOTA.handle();
  server.handleClient();

  if (can0Started && can1Started) {
    processBus(can0_handle, can1_handle, true);  // Car -> Radio
    processBus(can1_handle, can0_handle, false); // Radio -> Car
  }

  if (millis() - lastIgnitionFrame > IGNITION_TIMEOUT_MS) {
    setIgnition(false);
  }

  // BSI ticking removed for passive gateway mode, or kept for synthetic 0x276?
  // bsiTick(); // Obsolete if we are just bridging?
  // We keep it if we still want to force 0x276/0x260 on timeouts.
  bsiTick();
  updateLed();

  if (millis() - lastCanRx > SLEEP_TIMEOUT_MS) {
    enterLightSleep();
  }
}
