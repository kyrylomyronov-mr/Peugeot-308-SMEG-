#include "driver/twai.h"
#include "page_html.h"
#include <Adafruit_NeoPixel.h>
#include <Preferences.h>
#include <RTClib.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>

// --------- Onboard WS2812 RGB LED (GP8) ----------
static constexpr int LED_PIN = 8;
static constexpr uint8_t LED_BRIGHT = 30;
static Adafruit_NeoPixel led(1, LED_PIN, NEO_RGB + NEO_KHZ800);

// --------- CAN0 pins (Car Side) ----------
static constexpr gpio_num_t CAN0_RX = GPIO_NUM_19;
static constexpr gpio_num_t CAN0_TX = GPIO_NUM_20;

// --------- CAN1 pins (Radio Side) ----------
static constexpr gpio_num_t CAN1_RX = GPIO_NUM_21;
static constexpr gpio_num_t CAN1_TX = GPIO_NUM_22;

static twai_handle_t can0_handle = NULL;
static twai_handle_t can1_handle = NULL;

const char *AP_SSID = "PSA-Bridge";
const char *AP_PASS = "12345678";
WebServer server(80);

// --------- CAN frame IDs ----------
static constexpr uint32_t CAN_ID_BSI_INFO =
    0x260; // Language/Units/Status (BSI -> Radio)
static constexpr uint32_t CAN_ID_OPTIONS =
    0x361; // Menu tab permissions (BSI -> Radio)
static constexpr uint32_t CAN_ID_SMEG_TIME =
    0x39B; // Time sync from SMEG (Radio -> BSI)
static constexpr uint32_t CAN_ID_RADIO_BTNS =
    0x122; // Physical radio buttons (FMUX -> Radio)
static constexpr uint32_t CAN_ID_EMF_REMOTE =
    0x3E5; // EMF Display remote commands
static constexpr uint32_t CAN_ID_BSI_TIME =
    0x276; // Time sync set to SMEG (BSI -> Radio)
static constexpr uint32_t CAN_ID_PARK_SENSORS =
    0x0E1; // AAS / Parking sensors data (AEE2004)
static constexpr uint32_t CAN_ID_BSI_SLOW =
    0x0F6; // BSI Slow (Reverse, Economy, Brightness)
static constexpr uint32_t CAN_ID_HMI_CMD =
    0x1A9; // HMI Buttons from screen/steering
static constexpr uint32_t CAN_ID_CONF_SYNC =
    0x15B; // SMEG Configuration Sync request

// --------- State & Emulation Variables ----------
static unsigned long lastCan0Rx = 0; // Timestamp of last CAR message
static unsigned long lastCan1Rx = 0; // Timestamp of last RADIO message
static uint32_t canDroppedFrames = 0;

static uint8_t bsiLang = 14;      // Default 14 (Russian)
static bool bsiIsCelsius = true;  // Temperature unit status
static bool bsiDRL = true;        // Daytime Running Lights status (0x260 D2[1])
static bool bsiParkAssist = true; // Parking Assistance status (0x260 D4[7])
static bool bsiGrilleLight = false; // Grille Illumination status (0x260 D6[2])
static bool carReverse = false;     // Real vehicle reverse status
static bool lastBtn1A9 = false;     // State of button on 1A9 command
static Preferences prefs;

static uint8_t mod122[8] = {0};  // Virtual buttons for 0x122 (Radio buttons)
static uint8_t last122[8] = {0}; // Last real 0x122 message from FMUX
static uint8_t last0F6[8] = {0, 0, 0, 0,
                             0, 0, 0, 0x65}; // Last BSI_SLOW for brightness

static bool forceParking = false; // Manual parking emulator activation
static bool ecoBypass =
    false; // Bypass car's Economy mode (prevent radio shutdown)
static bool ecoForce =
    false; // Force radio into Economy Mode (remote power off)
static uint8_t parkDist[6] = {7, 7, 7, 7, 7, 7}; // Distance for 6 sensors (0-7)

// --------- RTC (DS3231) ----------
static constexpr int RTC_SDA_PIN = 15;
static constexpr int RTC_SCL_PIN = 18;
static RTC_DS3231 rtc;
static bool rtcOk = false;
static unsigned long lastClockTick = 0;

static unsigned long btnReleaseAt = 0;
static unsigned long emfReleaseAt = 0;

struct CanLogEntry {
  uint32_t timestamp;
  uint32_t id;
  uint8_t dlc;
  uint8_t data[8];
  bool isCar;
};
static constexpr int LOG_MAX = 10;
static CanLogEntry canLogCache[LOG_MAX];
static int canLogHead = 0;
static int canLogCount = 0;

static bool twaiStart() {
  twai_general_config_t g0 =
      TWAI_GENERAL_CONFIG_DEFAULT(CAN0_TX, CAN0_RX, TWAI_MODE_NORMAL);
  g0.controller_id = 0;
  g0.tx_queue_len = 64;
  g0.rx_queue_len = 64;
  twai_general_config_t g1 =
      TWAI_GENERAL_CONFIG_DEFAULT(CAN1_TX, CAN1_RX, TWAI_MODE_NORMAL);
  g1.controller_id = 1;
  g1.tx_queue_len = 64;
  g1.rx_queue_len = 64;
  twai_timing_config_t t = TWAI_TIMING_CONFIG_125KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  esp_err_t err0 = twai_driver_install_v2(&g0, &t, &f, &can0_handle);
  esp_err_t err1 = twai_driver_install_v2(&g1, &t, &f, &can1_handle);
  bool ok0 = (err0 == ESP_OK && twai_start_v2(can0_handle) == ESP_OK);
  bool ok1 = (err1 == ESP_OK && twai_start_v2(can1_handle) == ESP_OK);
  return ok0 && ok1;
}

static void recoverTwai() {
  if (can0_handle) {
    twai_stop_v2(can0_handle);
    twai_driver_uninstall_v2(can0_handle);
    can0_handle = NULL;
  }
  if (can1_handle) {
    twai_stop_v2(can1_handle);
    twai_driver_uninstall_v2(can1_handle);
    can1_handle = NULL;
  }
  delay(100);
  twaiStart();
}

static bool canSend(twai_handle_t h, uint32_t id, const uint8_t *d,
                    uint8_t dlc) {
  if (!h)
    return false;
  twai_message_t m = {};
  m.identifier = id;
  m.data_length_code = dlc;
  if (dlc > 0 && d)
    memcpy(m.data, d, dlc);
  return (twai_transmit_v2(h, &m, 0) == ESP_OK);
}

static void processBus(twai_handle_t src, twai_handle_t dst, bool isCarSide) {
  if (!src || !dst)
    return;
  twai_message_t msg;
  int maxReads = 100;
  while (twai_receive_v2(src, &msg, 0) == ESP_OK && maxReads-- > 0) {
    // Store raw data to canLogCache for Web UI debugging
    CanLogEntry &e = canLogCache[canLogHead];
    e.timestamp = millis();
    e.id = msg.identifier;
    e.dlc = msg.data_length_code;
    e.isCar = isCarSide;
    memcpy(e.data, msg.data,
           msg.data_length_code > 8 ? 8 : msg.data_length_code);
    canLogHead = (canLogHead + 1) % LOG_MAX;
    if (canLogCount < LOG_MAX)
      canLogCount++;

    if (isCarSide) {
      lastCan0Rx = millis();
      // Block Car side Status/Permissions (Bridge acts as authority)
      if (msg.identifier == CAN_ID_BSI_INFO || msg.identifier == CAN_ID_OPTIONS)
        continue;

      // Monitor BSI_SLOW for Reverse status and Economy mode flags
      if (msg.identifier == CAN_ID_BSI_SLOW && msg.data_length_code >= 8) {
        memcpy(last0F6, msg.data,
               8); // Save real frame baseline (brightness, etc.)
        carReverse = (msg.data[7] & 0x80) != 0;

        // Economy Mode Control (D5[0]): 1=Eco, 0=Normal
        if (ecoBypass)
          msg.data[5] &= ~0x01; // Force Normal to keep radio ON
        if (ecoForce)
          msg.data[5] |= 0x01; // Force Economy to turn radio OFF

        if (forceParking)
          continue; // Block real reverse to allow emulation
      }
      if (msg.identifier == CAN_ID_RADIO_BTNS && msg.data_length_code >= 8) {
        memcpy(last122, msg.data, 8);
        for (int i = 0; i < 8; i++)
          msg.data[i] |= mod122[i];
      }
    } else {
      lastCan1Rx = millis();
      if (msg.identifier == CAN_ID_HMI_CMD && msg.data_length_code >= 7) {
        bool btn = ((msg.data[3] & 0x04) != 0) || ((msg.data[5] & 0x04) != 0);
        if (btn && !lastBtn1A9)
          forceParking = !forceParking;
        lastBtn1A9 = btn;
      }
      if (msg.identifier == CAN_ID_CONF_SYNC && msg.data_length_code >= 7) {
        if (msg.data[1] == 0x04) {
          Serial.printf("[SMEG] 15B: D2=%02X D4=%02X D6=%02X\n", msg.data[2],
                        msg.data[4], msg.data[6]);
          // Handling "Three Parameters" synchronization from SMEG screen
          bool nDRL = (msg.data[2] & 0x02) != 0;
          if (nDRL != bsiDRL) {
            bsiDRL = nDRL;
            prefs.putBool("drl", bsiDRL);
            Serial.println("[NVS] DRL Upd");
          }

          // AAS (Park Assist) button on screen triggers the emulator
          bool nPrk = (msg.data[4] & 0x80) != 0;
          if (nPrk != bsiParkAssist) {
            bsiParkAssist = nPrk;
            forceParking =
                nPrk; // Activate visualization immediately if user turns it ON
            prefs.putBool("park", bsiParkAssist);
            Serial.println("[NVS] AAS Upd");
          }

          // Grille Light (CAL): SMEG sends 0x10 (Bit 4) to toggle it
          bool nGrl = (msg.data[6] & 0x10) != 0;
          if (nGrl != bsiGrilleLight) {
            bsiGrilleLight = nGrl;
            prefs.putBool("grille", bsiGrilleLight);
            Serial.println("[NVS] CAL Upd");
          }
        }
      }
      if (msg.identifier == CAN_ID_SMEG_TIME && msg.data_length_code >= 5 &&
          rtcOk) {
        int y = 2000 + (msg.data[0] & 0x7F);
        if (y >= 2000 && msg.data[1] >= 1 && msg.data[1] <= 12) {
          rtc.adjust(DateTime(y, msg.data[1], msg.data[2], msg.data[3],
                              msg.data[4], 0));
          prefs.putUInt("uxt", rtc.now().unixtime());
        }
      }
    }
    if (twai_transmit_v2(dst, &msg, 0) != ESP_OK)
      canDroppedFrames++;
  }
}

// Periodically sends BSI configuration and permissions to the radio
static void runBsiHeartbeat() {
  static unsigned long lastBsiHb = 0;
  unsigned long now = millis();
  if (now - lastBsiHb < 500)
    return;
  lastBsiHb = now;
  if (!can1_handle)
    return;

  // --- 0x260: BSI_INFO (Status & Config) ---
  uint8_t d260[8] = {0};
  d260[0] = 0x80 | ((bsiLang & 0x1F) << 2);
  d260[1] = 0x04; // Default brightness/status
  if (!bsiIsCelsius)
    d260[1] |= (1 << 6);

  if (bsiDRL)
    d260[2] |= 0x02;
  if (bsiParkAssist)
    d260[4] |= 0x80;

  // Grille Light Status: Radio expects confirmation on Bit 2 (0x04)
  if (bsiGrilleLight)
    d260[6] |= 0x04;
  canSend(can1_handle, CAN_ID_BSI_INFO, d260, 8);

  // --- 0x361: BSI_OPTIONS (Menu Permissions) ---
  // Bitmask: [D0: Menu/DRL, D1: AAS, D3: Cruise, D4: Grille]
  uint8_t d361[8] = {0x81, 0x80, 0x00, 0x10, 0x10, 0x00, 0x00, 0x00};
  canSend(can1_handle, CAN_ID_OPTIONS, d361, 8);
}

static void runParkingEmulation() {
  static unsigned long lastPrk = 0;
  unsigned long now = millis();
  if (!(carReverse || forceParking) || (now - lastPrk < 100))
    return;
  lastPrk = now;
  if (!can1_handle)
    return;

  if (forceParking) {
    uint8_t rs[8];
    memcpy(rs, last0F6, 8); // Use real frame baseline to keep brightness
    rs[7] |= 0x80; // Spoof Reverse flag (forces radio to parking screen)
    canSend(can1_handle, CAN_ID_BSI_SLOW, rs, 8);
  }

  // AEE2004 style bit-packing for AAS sensors (D3-D5)
  uint8_t dp[8] = {0};
  dp[3] = (parkDist[0] << 5) | (parkDist[1] << 2);
  dp[4] = (parkDist[2] << 5) | (parkDist[3] << 2);
  dp[5] = (parkDist[4] << 5) | (parkDist[5] << 2) | 0x02; // AAS Active status
  canSend(can1_handle, CAN_ID_PARK_SENSORS, dp, 8);
}

static void pressIndex(uint8_t idx, uint16_t duration = 100) {
  if (idx < 1 || idx > 40)
    return;
  mod122[(idx - 1) / 8] |= (1 << ((idx - 1) % 8));
  uint8_t d[8];
  memcpy(d, last122, 8);
  for (int i = 0; i < 8; i++)
    d[i] |= mod122[i];
  canSend(can1_handle, CAN_ID_RADIO_BTNS, d, 8);
  btnReleaseAt = millis() + duration;
}

static void pressEmf(uint8_t b) {
  if (!can0_handle)
    return;
  uint8_t d[6] = {0};
  switch (b) {
  case 1:
    d[0] = 0x40;
    break;
  case 2:
    d[1] = 0x40;
    break;
  case 3:
    d[1] = 0x10;
    break;
  case 4:
    d[2] = 0x40;
    break;
  case 5:
    d[2] = 0x10;
    break;
  case 6:
    d[5] = 0x40;
    break;
  case 7:
    d[5] = 0x10;
    break;
  case 8:
    d[5] = 0x04;
    break;
  case 9:
    d[5] = 0x01;
    break;
  }
  canSend(can0_handle, CAN_ID_EMF_REMOTE, d, 6);
  emfReleaseAt = millis() + 150;
}

static void rtcInit() {
  Wire.begin(RTC_SDA_PIN, RTC_SCL_PIN);
  if (rtc.begin(&Wire)) {
    rtcOk = true;
    if (rtc.lostPower())
      rtc.adjust(DateTime(prefs.getUInt("uxt", 1577836800)));
  }
}

static void runClockEmulation() {
  if (!rtcOk || !can1_handle || millis() - lastClockTick < 3000)
    return;
  lastClockTick = millis();
  DateTime now = rtc.now();
  uint8_t d[7] = {(uint8_t)(0x80 | (now.year() - 2000)),
                  (uint8_t)now.month(),
                  (uint8_t)now.day(),
                  (uint8_t)now.hour(),
                  (uint8_t)now.minute(),
                  0x3F,
                  0xFE};
  canSend(can1_handle, CAN_ID_BSI_TIME, d, 7);
}

static void handleLog() {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");
  server.sendContent("[");
  for (int i = 0; i < canLogCount; i++) {
    int idx = (canLogHead - canLogCount + i + LOG_MAX) % LOG_MAX;
    char hex[32] = {0};
    int cp = 0;
    for (int b = 0; b < canLogCache[idx].dlc; b++)
      cp += snprintf(hex + cp, 32 - cp, "%02X ", canLogCache[idx].data[b]);
    char buf[256];
    snprintf(buf, 256, "{\"t\":%u,\"id\":%u,\"dlc\":%u,\"c\":%d,\"d\":\"%s\"}",
             canLogCache[idx].timestamp, canLogCache[idx].id,
             canLogCache[idx].dlc, canLogCache[idx].isCar ? 1 : 0, hex);
    if (i > 0)
      server.sendContent(",");
    server.sendContent(buf);
  }
  server.sendContent("]");
}

void setup() {
  Serial.begin(115200);
  led.begin();
  led.setPixelColor(0, led.Color(0, 0, LED_BRIGHT));
  led.show();
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(IPAddress(192, 168, 8, 1), IPAddress(192, 168, 8, 1),
                    IPAddress(255, 255, 255, 0));
  prefs.begin("psabrg", false);
  bsiLang = prefs.getUChar("lang", 14);
  bsiIsCelsius = prefs.getBool("temp", true);
  bsiDRL = prefs.getBool("drl", true);
  bsiParkAssist = prefs.getBool("park", true);
  bsiGrilleLight = prefs.getBool("grille", false);
  ecoBypass = prefs.getBool("ecob", false);
  WiFi.softAP(AP_SSID, AP_PASS);
  rtcInit();
  twaiStart();
  lastCan0Rx = lastCan1Rx = millis();
  server.on("/", []() { server.send_P(200, "text/html", page_html); });
  server.on("/log", handleLog);
  server.on("/ctrl", []() {
    if (server.hasArg("idx"))
      pressIndex(server.arg("idx").toInt(),
                 server.hasArg("t") ? server.arg("t").toInt() : 100);
    server.send(200, "text/plain", "OK");
  });
  server.on("/emf", []() {
    if (server.hasArg("id"))
      pressEmf(server.arg("id").toInt());
    server.send(200, "text/plain", "OK");
  });
  server.on("/clock", []() {
    if (server.hasArg("y")) {
      if (rtcOk)
        rtc.adjust(DateTime(server.arg("y").toInt(), server.arg("m").toInt(),
                            server.arg("d").toInt(), server.arg("h").toInt(),
                            server.arg("min").toInt(), 0));
      server.send(200, "text/plain", "OK");
    } else {
      if (!rtcOk)
        server.send(200, "text/plain", "RTC_ERR");
      else {
        DateTime n = rtc.now();
        char buf[16];
        snprintf(buf, 16, "%02d:%02d:%02d", n.hour(), n.minute(), n.second());
        server.send(200, "text/plain", buf);
      }
    }
  });
  server.on("/config", []() {
    if (server.hasArg("lang")) {
      bsiLang = server.arg("lang").toInt();
      prefs.putUChar("lang", bsiLang);
      bsiIsCelsius = (server.arg("temp").toInt() == 1);
      prefs.putBool("temp", bsiIsCelsius);
      if (server.hasArg("drl")) {
        bsiDRL = (server.arg("drl").toInt() == 1);
        prefs.putBool("drl", bsiDRL);
      }
      if (server.hasArg("park")) {
        bsiParkAssist = (server.arg("park").toInt() == 1);
        prefs.putBool("park", bsiParkAssist);
      }
      if (server.hasArg("grille")) {
        bsiGrilleLight = (server.arg("grille").toInt() == 1);
        prefs.putBool("grille", bsiGrilleLight);
      }
      server.send(200, "text/plain", "OK");
    } else {
      String j = "{\"lang\":" + String(bsiLang) +
                 ",\"temp\":" + String(bsiIsCelsius ? 1 : 0) +
                 ",\"drl\":" + String(bsiDRL ? 1 : 0) +
                 ",\"park\":" + String(bsiParkAssist ? 1 : 0) +
                 ",\"grille\":" + String(bsiGrilleLight ? 1 : 0) +
                 ",\"eco_b\":" + String(ecoBypass ? 1 : 0) + "}";
      server.send(200, "application/json", j);
    }
  });
  server.on("/sim", []() {
    if (server.hasArg("park"))
      forceParking = (server.arg("park").toInt() == 1);
    if (server.hasArg("eco_b")) {
      ecoBypass = (server.arg("eco_b").toInt() == 1);
      prefs.putBool("ecob", ecoBypass);
    }
    if (server.hasArg("eco_f"))
      ecoForce = (server.arg("eco_f").toInt() == 1);
    for (int i = 0; i < 6; i++) {
      String pi = "p" + String(i);
      if (server.hasArg(pi))
        parkDist[i] = server.arg(pi).toInt();
    }
    server.send(200, "text/plain", "OK");
  });
  server.begin();
}

void loop() {
  server.handleClient();
  unsigned long now = millis();
  static unsigned long lastLed = 0;
  if (now - lastLed >= 500) {
    lastLed = now;
    bool c0 = (now - lastCan0Rx < 2000), c1 = (now - lastCan1Rx < 2000);
    if (!c0 && !c1)
      led.setPixelColor(0, led.Color(LED_BRIGHT, 0, 0));
    else if (!c0 || !c1)
      led.setPixelColor(0, led.Color(LED_BRIGHT, LED_BRIGHT / 2, 0));
    else
      led.setPixelColor(0, led.Color(0, 0, LED_BRIGHT));
    led.show();
  }
  if (can0_handle) {
    twai_status_info_t s;
    if (twai_get_status_info_v2(can0_handle, &s) == ESP_OK &&
        s.state == TWAI_STATE_BUS_OFF)
      recoverTwai();
  }
  if (can1_handle) {
    twai_status_info_t s;
    if (twai_get_status_info_v2(can1_handle, &s) == ESP_OK &&
        s.state == TWAI_STATE_BUS_OFF)
      recoverTwai();
  }
  if (can0_handle && can1_handle) {
    processBus(can0_handle, can1_handle, true);
    processBus(can1_handle, can0_handle, false);
  }
  runClockEmulation();
  runBsiHeartbeat();
  runParkingEmulation();
  if (btnReleaseAt && now > btnReleaseAt) {
    memset(mod122, 0, 8);
    canSend(can1_handle, CAN_ID_RADIO_BTNS, last122, 8);
    btnReleaseAt = 0;
  }
  if (emfReleaseAt && now > emfReleaseAt) {
    uint8_t d[6] = {0};
    canSend(can0_handle, CAN_ID_EMF_REMOTE, d, 6);
    emfReleaseAt = 0;
  }
}
