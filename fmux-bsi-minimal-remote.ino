/*
 * PSA SMEG Minimal Remote — ONLY Buttons & Time Sync
 * Board  : Waveshare ESP32-C6-Zero
 * CAN    : 125 кбит/с, TX=GP20, RX=GP23 (Radio Side)
 */

#include <Adafruit_NeoPixel.h>
#include <WebServer.h>
#include <WiFi.h>
#include "driver/twai.h"

// --------- CAN Pins (Original Single Bus) ----------
static constexpr gpio_num_t CAN_TX = GPIO_NUM_22;
static constexpr gpio_num_t CAN_RX = GPIO_NUM_21;

// --------- GPIO & LED ----------
static constexpr int LED_PIN = 8;
static Adafruit_NeoPixel led(1, LED_PIN, NEO_RGB + NEO_KHZ800);

// --------- WiFi AP ----------
const char *AP_SSID = "SMEG-Remote-Minimal";
const char *AP_PASS = "12345678";
WebServer server(80);

// --------- State ----------
static bool canStarted = false;

// --------- TWAI Init ----------
static bool twaiStart() {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t t = TWAI_TIMING_CONFIG_125KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g, &t, &f) == ESP_OK) {
    twai_start();
    canStarted = true;
    return true;
  }
  return false;
}

static bool canSend(uint32_t id, const uint8_t *d, uint8_t dlc) {
  if (!canStarted) return false;
  twai_message_t m = {};
  m.identifier = id;
  m.data_length_code = dlc;
  if (dlc && d) memcpy(m.data, d, dlc);
  return twai_transmit(&m, pdMS_TO_TICKS(10)) == ESP_OK;
}

static void pressIndex(uint8_t idx) {
  uint8_t z = idx - 1;
  uint8_t d[8] = {0};
  d[z / 8] = (1u << (z % 8));
  if (canSend(0x122, d, 8)) {
    delay(80);
    uint8_t empty[8] = {0};
    canSend(0x122, empty, 8);
  }
}

// --------- Web Handlers ----------
static void handleRoot() {
  String s = F("<!DOCTYPE html><html><head><meta charset='utf-8'/><meta name='viewport' content='width=device-width,initial-scale=1'/>"
               "<title>Minimal Remote</title><style>"
               "body{font-family:sans-serif;background:#121212;color:#eee;text-align:center;padding:20px;}"
               "button{padding:15px;margin:5px;width:140px;border-radius:8px;border:none;background:#334;color:#fff;font-size:14px;cursor:pointer;}"
               "button:active{background:#557;}"
               ".sync{background:#252;width:95%}"
               "</style></head><body>"
               "<h2>SMEG Minimal Remote</h2>"
               "<button class='sync' onclick='fetch(\"/sync\")'>🕒 Sync Time from Browser</button><br/>"
               "<div style='display:flex;flex-wrap:wrap;justify-content:center;margin-top:20px;'>"
               "<button onclick='fetch(\"/btn?n=17\")'>⚙️ Settings</button>"
               "<button onclick='fetch(\"/btn?n=18\")'>🗺️ Maps</button>"
               "<button onclick='fetch(\"/btn?n=19\")'>📻 Radio</button>"
               "<button onclick='fetch(\"/btn?n=20\")'>📞 Phone</button>"
               "<button onclick='fetch(\"/btn?n=29\")'>📱 Apps</button>"
               "<button onclick='fetch(\"/btn?n=30\")'>🚗 Trip</button>"
               "<button onclick='fetch(\"/btn?n=32\")'>❄️ AC</button>"
               "</div></body></html>");
  server.send(200, "text/html", s);
}

void setup() {
  Serial.begin(115200);
  led.begin();
  led.setPixelColor(0, led.Color(20, 0, 0)); led.show();

  WiFi.softAP(AP_SSID, AP_PASS);
  twaiStart();

  server.on("/", handleRoot);
  server.on("/btn", []() {
    if (server.hasArg("n")) pressIndex(server.arg("n").toInt());
    server.send(200, "text/plain", "OK");
  });
  server.on("/sync", []() {
    uint8_t d[8] = {0x80, 0x19, 0x01, 0x0C, 0x00, 0x3F, 0xFE, 0x00}; // Default 12:00, 24h, 2025
    canSend(0x276, d, 8);
    server.send(200, "text/plain", "OK sync");
  });
  server.begin();
  led.setPixelColor(0, led.Color(0, 20, 0)); led.show();
}

void loop() {
  server.handleClient();
}
