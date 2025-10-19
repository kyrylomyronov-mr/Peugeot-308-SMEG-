/*
 * PSA Comfort CAN Remote — с дисплеем (Arduino_GFX для ESP32-C6)
 * Waveshare ESP32-C6 1.47inch Display
 * CAN: 125 кбит/с, TX=GPIO20, RX=GPIO23
 * 
 * Установите библиотеку: Arduino_GFX by moononournation
 */

#include <WiFi.h>
#include <WebServer.h>
#include "driver/twai.h"
#include <Arduino_GFX_Library.h>
#include <string.h>

// --------- CAN pins (ESP32-C6) ----------
static constexpr gpio_num_t CAN_TX = GPIO_NUM_20;
static constexpr gpio_num_t CAN_RX = GPIO_NUM_23;

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
static bool canStarted = false;
static float engineTemp = -99.0f;
static float batteryVolt = 0.0f;
static unsigned long lastCanRx = 0;

// --------- Display brightness ----------
static uint8_t brightness = 255; // 0-255

// --------- CAN Init ----------
static bool twaiStart125k() {
  if (canStarted) return true;
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
  g.tx_queue_len = 8;
  g.rx_queue_len = 16;
  g.clkout_divider = 0;
  g.alerts_enabled = 0;

  twai_timing_config_t t = TWAI_TIMING_CONFIG_125KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g, &t, &f) != ESP_OK) { Serial.println("[CAN] driver install failed"); return false; }
  if (twai_start() != ESP_OK)                    { Serial.println("[CAN] start failed");          return false; }
  canStarted = true;
  Serial.printf("[CAN] started @125k (TX=%d RX=%d)\n", (int)CAN_TX, (int)CAN_RX);
  return true;
}

// --------- CAN TX ----------
static bool canSend122(const uint8_t* d, uint8_t dlc) {
  if (!canStarted || dlc > 8) return false;
  twai_message_t m = {};
  m.identifier = 0x122;
  m.flags = 0;
  m.data_length_code = dlc;
  if (dlc && d) memcpy(m.data, d, dlc);
  return (twai_transmit(&m, pdMS_TO_TICKS(20)) == ESP_OK);
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
  }
}

// --------- Display Update ----------
static unsigned long lastDisplayUpdate = 0;
static float prevEngineTemp = -999.0f;
static float prevBatteryVolt = 0.0f;
static bool prevCanStatus = false;

static void updateDisplay() {
  if (millis() - lastDisplayUpdate < 500) return;
  lastDisplayUpdate = millis();
  
  bool canOk = (millis() - lastCanRx < 2000);
  
  // Обновляем только если значения изменились
  bool needUpdate = false;
  if (abs(engineTemp - prevEngineTemp) > 0.5f) needUpdate = true;
  if (abs(batteryVolt - prevBatteryVolt) > 0.1f) needUpdate = true;
  if (canOk != prevCanStatus) needUpdate = true;
  
  // Первый запуск - всегда рисуем
  if (prevEngineTemp < -900.0f) needUpdate = true;
  
  if (!needUpdate) return;
  
  prevEngineTemp = engineTemp;
  prevBatteryVolt = batteryVolt;
  prevCanStatus = canOk;
  
  // Рисуем только изменившиеся области
  
  // Заголовок (один раз)
  static bool headerDrawn = false;
  if (!headerDrawn) {
    gfx->fillRect(0, 0, 172, 40, BLACK);
    gfx->setTextColor(WHITE);
    gfx->setTextSize(2);
    gfx->setCursor(10, 10);
    gfx->println("PSA CAN");
    headerDrawn = true;
  }
  
  // Температура
  gfx->fillRect(0, 40, 172, 40, BLACK);
  gfx->setTextSize(3);
  gfx->setCursor(10, 50);
  gfx->setTextColor(CYAN);
  gfx->print("T:");
  
  if (engineTemp > -90.0f) {
    if (engineTemp < 80) gfx->setTextColor(GREEN);
    else if (engineTemp < 95) gfx->setTextColor(YELLOW);
    else gfx->setTextColor(RED);
    gfx->printf("%.0fC", engineTemp);
  } else {
    gfx->setTextColor(DARKGREY);
    gfx->print("---");
  }
  
  // Напряжение
  gfx->fillRect(0, 80, 172, 40, BLACK);
  gfx->setTextSize(3);
  gfx->setCursor(10, 90);
  gfx->setTextColor(ORANGE);
  gfx->print("V:");
  
  if (batteryVolt > 5.0f) {
    if (batteryVolt >= 12.5f) gfx->setTextColor(GREEN);
    else if (batteryVolt >= 12.0f) gfx->setTextColor(YELLOW);
    else gfx->setTextColor(RED);
    gfx->printf("%.1fV", batteryVolt);
  } else {
    gfx->setTextColor(DARKGREY);
    gfx->print("---");
  }
  
  // Статус CAN
  gfx->fillRect(0, 120, 172, 20, BLACK);
  gfx->setTextSize(1);
  gfx->setCursor(10, 130);
  if (canOk) {
    gfx->setTextColor(GREEN);
    gfx->print("CAN: OK");
  } else {
    gfx->setTextColor(RED);
    gfx->print("CAN: No data");
  }
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
         ".muted{color:#888;font-size:12px;margin-top:8px}</style>"
         "<script>setInterval(async()=>{let r=await fetch('/data');let d=await r.json();"
         "document.getElementById('temp').innerText=d.temp+'°C';"
         "document.getElementById('volt').innerText=d.volt+'V'},1000)</script></head><body>");

  s += F("<h2>🚗 PSA CAN Remote</h2>"
         "<div class='info'><b>Температура:</b> <span id='temp'>---</span> &nbsp;|&nbsp; "
         "<b>Напряжение:</b> <span id='volt'>---</span></div>"
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
         "<div class='muted'>Peugeot 307 2006</div>"
         "<script>async function btn(n){await fetch('/btn?n='+n)}</script>"
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

// --------- Setup/Loop ----------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[PSA CAN Monitor] ESP32-C6");

  // Настройка SPI для дисплея
  SPI.begin(TFT_SCK, -1, TFT_MOSI, TFT_CS);
  
  // Display
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  
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
  if (!twaiStart125k()) Serial.println("[CAN] init failed");

  // HTTP
  server.on("/", HTTP_GET, handleRoot);
  server.on("/btn", HTTP_GET, handleBtn);
  server.on("/data", HTTP_GET, handleData);
  server.begin();
  Serial.println("[HTTP] Server started on :80");

  delay(1000);
  gfx->fillScreen(BLACK);
}

void loop() {
  server.handleClient();
  readCanMessages();
  updateDisplay();
}
