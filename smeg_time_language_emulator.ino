#include <Arduino.h>
#include "driver/twai.h"
#include <Preferences.h>
#include <sys/time.h>
#include <time.h>

// --------------------------- Hardware configuration ---------------------------
static constexpr gpio_num_t CAN_TX = GPIO_NUM_20;
static constexpr gpio_num_t CAN_RX = GPIO_NUM_23;
static constexpr int CAN_STBY = -1; // set to pin number if transceiver standby must be toggled

// ------------------------------ Frame parameters ------------------------------
// Keep the language/settings frame frequent to avoid the SMEG UI blinking
static constexpr uint32_t PERIOD_260_MS = 100;
static constexpr uint32_t PERIOD_276_MS = 500;

static constexpr uint32_t ID_260 = 0x260; // BSI -> broadcast current settings
static constexpr uint32_t ID_15B = 0x15B; // SMEG -> write settings
static constexpr uint32_t ID_39B = 0x39B; // SMEG -> write date/time
static constexpr uint32_t ID_276 = 0x276; // BSI -> broadcast current date/time

// ------------------------------ Persistent state ------------------------------
Preferences prefs;
static constexpr const char *PREF_NAMESPACE = "smeg";
static constexpr const char *KEY_LANG5 = "lang5";
static constexpr const char *KEY_IS24 = "is24h";
static constexpr const char *KEY_ISC = "isC";
static constexpr const char *KEY_RAW260 = "raw260";
static constexpr const char *KEY_EPOCH = "epoch";
static constexpr const char *KEY_EPOCH_TS = "epoch_ts";

uint8_t state260[8] = {0};
bool have260Baseline = false;
uint8_t lang5 = 0b01110; // default Russian
bool is24h = true;
bool isCelsius = true;

// Time persistence helpers
static time_t persistedEpoch = 0;            // epoch stored in NVS
static uint32_t persistedEpochMillis = 0;    // millis() snapshot when epoch was written

// ------------------------------ Runtime state --------------------------------
uint32_t last260 = 0;
uint32_t last276 = 0;

// ------------------------------ CAN helpers ----------------------------------
static bool canInit125k() {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
  g_config.tx_queue_len = 10;
  g_config.rx_queue_len = 20;
  g_config.clkout_divider = 0;
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_125KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) return false;
  if (twai_start() != ESP_OK) return false;
  return true;
}

static bool canSend(uint32_t id, const uint8_t *data, uint8_t len) {
  if (len > 8) return false;
  twai_message_t m = {};
  m.identifier = id;
  m.data_length_code = len;
  m.rtr = 0;
  m.extd = 0;
  memcpy(m.data, data, len);
  return (twai_transmit(&m, pdMS_TO_TICKS(5)) == ESP_OK);
}

static bool canReceive(twai_message_t &m, uint32_t timeout_ms = 10) {
  return (twai_receive(&m, pdMS_TO_TICKS(timeout_ms)) == ESP_OK);
}

// ------------------------------ Bit helpers ----------------------------------
static inline void setBit(uint8_t &byte, uint8_t mask, bool value) {
  if (value) byte |= mask; else byte &= static_cast<uint8_t>(~mask);
}

static void applyLangAndUnitsTo260() {
  // Preserve the lower two bits in D0
  uint8_t low2 = state260[0] & 0x03;
  state260[0] = static_cast<uint8_t>((lang5 & 0x1F) << 2) | low2;

  // Force menu active bit
  state260[0] |= 0x80;

  // Celsius/Fahrenheit bit in D1[6]
  setBit(state260[1], 0x40, isCelsius);
}

// ------------------------------ NVS helpers ----------------------------------
static void persistTimeToNvs(time_t epoch) {
  uint32_t ts = millis();
  prefs.begin(PREF_NAMESPACE, false);
  prefs.putLong(KEY_EPOCH, static_cast<long>(epoch));
  prefs.putUInt(KEY_EPOCH_TS, ts);
  prefs.end();
  persistedEpoch = epoch;
  persistedEpochMillis = ts;
}

static void saveStateToNvs() {
  prefs.begin(PREF_NAMESPACE, false);
  prefs.putUChar(KEY_LANG5, lang5);
  prefs.putBool(KEY_IS24, is24h);
  prefs.putBool(KEY_ISC, isCelsius);
  prefs.putBytes(KEY_RAW260, state260, sizeof(state260));
  prefs.end();
}

static void loadStateFromNvs() {
  prefs.begin(PREF_NAMESPACE, true);
  lang5 = prefs.getUChar(KEY_LANG5, lang5);
  is24h = prefs.getBool(KEY_IS24, is24h);
  isCelsius = prefs.getBool(KEY_ISC, isCelsius);
  size_t sz = prefs.getBytesLength(KEY_RAW260);
  if (sz >= sizeof(state260)) {
    prefs.getBytes(KEY_RAW260, state260, sizeof(state260));
    have260Baseline = true;
  }
  persistedEpoch = static_cast<time_t>(prefs.getLong(KEY_EPOCH, 0));
  persistedEpochMillis = prefs.getUInt(KEY_EPOCH_TS, 0);
  prefs.end();

  if (!have260Baseline) {
    memset(state260, 0, sizeof(state260));
    state260[0] |= 0x80; // ensure menu active by default
    have260Baseline = true;
  }

  applyLangAndUnitsTo260();

  if (persistedEpoch > 0) {
    time_t nowEpoch = persistedEpoch;
    uint32_t nowMs = millis();
    if (persistedEpochMillis > 0 && nowMs >= persistedEpochMillis) {
      uint32_t delta = nowMs - persistedEpochMillis;
      nowEpoch += delta / 1000;
    }
    struct timeval tv = { .tv_sec = nowEpoch, .tv_usec = 0 };
    settimeofday(&tv, nullptr);
  }
}

// ------------------------------ Settings logic -------------------------------
static void apply15BtoState(const uint8_t msg[8], uint8_t len) {
  if (!have260Baseline) return;

  bool useLangUnits = (msg[0] & 0x80u) != 0; // D0[7]
  bool useTail = (len > 1) && ((msg[1] & 0x04u) != 0); // D1[2]

  if (useTail) {
    for (uint8_t i = 1; i < len && i < sizeof(state260); ++i) {
      state260[i] = msg[i];
    }
  }

  if (useLangUnits) {
    lang5 = static_cast<uint8_t>((msg[0] >> 2) & 0x1F);
    isCelsius = (msg[1] & 0x40u) != 0;
  }

  applyLangAndUnitsTo260();
  saveStateToNvs();
  Serial.printf("[260] applied lang=%u C=%d tail=%d\n", lang5, (int)isCelsius, (int)useTail);
}

// ------------------------------ Time handling --------------------------------
struct CarTime {
  bool is24;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
};

static void handle39B(const uint8_t *data, uint8_t len) {
  if (len < 5) return;
  CarTime ct;
  ct.is24 = (data[0] & 0x80u) != 0;
  ct.year = 2000u + static_cast<uint16_t>(data[0] & 0x7Fu);
  ct.month = data[1] & 0x0Fu;
  ct.day = data[2] & 0x1Fu;
  ct.hour = data[3] & 0x1Fu;
  ct.minute = data[4] & 0x3Fu;

  is24h = ct.is24;
  saveStateToNvs();

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
    persistTimeToNvs(epoch);
  }

  Serial.printf("[TIME] set %04u-%02u-%02u %02u:%02u 24h=%d\n",
                ct.year, ct.month, ct.day, ct.hour, ct.minute, (int)ct.is24);
}

static void build276(uint8_t out[7]) {
  time_t nowEpoch = time(nullptr);
  struct tm *t = localtime(&nowEpoch);
  uint16_t year = t ? static_cast<uint16_t>(t->tm_year + 1900) : 2000;
  uint8_t mon = t ? static_cast<uint8_t>(t->tm_mon + 1) : 1;
  uint8_t day = t ? static_cast<uint8_t>(t->tm_mday) : 1;
  uint8_t hour = t ? static_cast<uint8_t>(t->tm_hour) : 0;
  uint8_t min = t ? static_cast<uint8_t>(t->tm_min) : 0;

  out[0] = (is24h ? 0x80 : 0x00) | static_cast<uint8_t>((year >= 2000) ? (year - 2000) : 0);
  out[1] = mon & 0x0F;
  out[2] = day & 0x1F;
  out[3] = hour & 0x1F;
  out[4] = min & 0x3F;
  out[5] = 0x3F; // forward unknown/reserved bits as typical filler
  out[6] = 0xFE; // marker (no seconds information)
}

// ------------------------------ Periodic TX ----------------------------------
static void send260() {
  applyLangAndUnitsTo260();
  canSend(ID_260, state260, sizeof(state260));
}

static void send276() {
  uint8_t payload[7];
  build276(payload);
  canSend(ID_276, payload, sizeof(payload));
}

// ------------------------------ Setup/loop -----------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("SMEG Time/Language Emulator @125k");

  if (CAN_STBY >= 0) {
    pinMode(CAN_STBY, OUTPUT);
    digitalWrite(CAN_STBY, LOW);
  }

  loadStateFromNvs();

  if (!canInit125k()) {
    Serial.println("[CAN] init failed");
    while (true) delay(1000);
  }

  last260 = millis();
  last276 = millis();
}

void loop() {
  twai_message_t msg;
  while (canReceive(msg, 0)) {
    if (msg.rtr || msg.extd) continue;

    if (msg.identifier == ID_15B && msg.data_length_code > 0) {
      uint8_t buf[8] = {0};
      uint8_t len = msg.data_length_code;
      if (len > 8) len = 8;
      memcpy(buf, msg.data, len);
      apply15BtoState(buf, len);
      send260();
    } else if (msg.identifier == ID_39B && msg.data_length_code >= 5) {
      handle39B(msg.data, msg.data_length_code);
      send276();
    }
  }

  uint32_t now = millis();
  if (now - last260 >= PERIOD_260_MS) {
    last260 = now;
    send260();
  }
  if (now - last276 >= PERIOD_276_MS) {
    last276 = now;
    send276();
  }
}
