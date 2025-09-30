#include <Arduino.h>
#include "driver/twai.h"

// ----------------- CAN Pins -----------------
#define CAN_TX GPIO_NUM_22    // CAN TX pin
#define CAN_RX GPIO_NUM_23    // CAN RX pin

// ----------------- Button Panel Pin -----------------
#define BTN_ADC_PIN 4         // GPIO4 (ADC2_CH0)

// ----------------- Settings -----------------
#define LONG_PRESS_MS 800     // Long press threshold (ms)
#define DEBOUNCE_MS 80        // Debounce delay (ms)
#define ADC_TOL 100           // Allowed ADC tolerance

// ----------------- Button Mapping -----------------
// Each button has an average ADC value (measured), a name, and CAN commands
struct Button {
  const char* name;
  int adcValue;        // Reference ADC reading (0–4095)
  uint8_t canCmdShort; // Short press CAN command
  uint8_t canCmdLong;  // Long press CAN command
};

Button buttons[] = {
  { "Settings",          2780, 0x17, 0x27 },  // Right button
  { "Maps",              3210, 0x18, 0x28 },  // Center button
  { "Radio",                0, 0x19, 0x29 },  // Upper button
  { "Phone",             1488, 0x20, 0x30 },  // Left button
  { "Peugeot Connects",  2000, 0x29, 0x39 },  // Example placeholder
  { "Trip Info",         2259, 0x30, 0x40 },  // Lower button
  { "Air Conditioning",  3500, 0x32, 0x42 },  // Example placeholder
};
const int BTN_COUNT = sizeof(buttons) / sizeof(buttons[0]);

// ----------------- State Variables -----------------
int lastBtn = -1;
unsigned long pressStart = 0;
bool longSent = false;

// ----------------- CAN Initialization -----------------
void canInit() {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_125KBITS(); // 125 kbps CAN speed
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK &&
      twai_start() == ESP_OK) {
    Serial.println("CAN started @125 kbps");
  } else {
    Serial.println("CAN start FAILED");
  }
}

// ----------------- Send CAN Command -----------------
void sendCanCmd(uint8_t cmd) {
  twai_message_t msg;
  msg.identifier = 0x122;       // SMEG+ FMUX frame ID
  msg.extd = 0;
  msg.rtr = 0;
  msg.data_length_code = 8;
  memset(msg.data, 0, 8);

  msg.data[0] = cmd;            // Put command in first byte

  if (twai_transmit(&msg, pdMS_TO_TICKS(100)) == ESP_OK) {
    Serial.printf("CAN TX: ID=0x122, CMD=0x%02X\n", cmd);
  } else {
    Serial.println("CAN TX ERROR");
  }
}

// ----------------- Detect Button by ADC -----------------
int detectButton(int adc) {
  for (int i = 0; i < BTN_COUNT; i++) {
    if (abs(adc - buttons[i].adcValue) < ADC_TOL) {
      return i;  // Found button
    }
  }
  return -1;     // None
}

// ----------------- Setup -----------------
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=== Button Panel Test ===");

  analogReadResolution(12);       // 0..4095
  analogSetAttenuation(ADC_11db); // ~0..3.6 V

  canInit();
}

// ----------------- Loop -----------------
void loop() {
  int adc = analogRead(BTN_ADC_PIN);
  int btn = detectButton(adc);

  if (btn != -1) {
    if (lastBtn != btn) {
      pressStart = millis();
      longSent = false;
      lastBtn = btn;
      Serial.printf("Pressed: %s (ADC=%d)\n", buttons[btn].name, adc);
    } else {
      if (!longSent && millis() - pressStart > LONG_PRESS_MS) {
        sendCanCmd(buttons[btn].canCmdLong);
        longSent = true;
      }
    }
  } else {
    if (lastBtn != -1) {
      if (!longSent) {
        sendCanCmd(buttons[lastBtn].canCmdShort);
      }
      Serial.printf("Released: %s\n", buttons[lastBtn].name);
      lastBtn = -1;
    }
  }

  delay(DEBOUNCE_MS);
}
