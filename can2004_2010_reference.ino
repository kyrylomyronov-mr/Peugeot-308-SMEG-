/*
Copyright 2019-2022, Ludwig V. <https://github.com/ludwig-v>
Copyright 2021, Nick V. (V3nn3tj3) <https://github.com/v3nn3tj3>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License at <http://www.gnu.org/licenses/> for
more details.

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
*/

/////////////////////
//    Libraries    //
/////////////////////

#include <EEPROM.h>
#include <SPI.h>
#include <Time.h>
#include <TimeLib.h>
#include <Wire.h>
#include <DS1307RTC.h> // https://github.com/PaulStoffregen/DS1307RTC
#include <mcp2515.h> // https://github.com/autowp/arduino-mcp2515 + https://github.com/watterott/Arduino-Libs/tree/master/digitalWriteFast

/////////////////////
//  Configuration  //
/////////////////////

#define CS_PIN_CAN0 10
#define CS_PIN_CAN1 9
#define SERIAL_SPEED 115200
#define CAN_SPEED CAN_125KBPS // Entertainment CAN bus - Low speed
#define CAN_FREQ MCP_16MHZ // Switch to 8MHZ if you have a 8Mhz module

///////////////////////
// Private functions //
///////////////////////
byte checksumm_0E6(const byte* frame);

////////////////////
// Initialization //
////////////////////

MCP2515 CAN0(CS_PIN_CAN0); // CAN-BUS Shield N°1
MCP2515 CAN1(CS_PIN_CAN1); // CAN-BUS Shield N°2

////////////////////
//   Variables    //
////////////////////

// My variables
bool debugGeneral = false; // Get some debug informations on Serial
bool debugCAN0 = false; // Read data sent by ECUs from the car to Entertainment CAN bus using https://github.com/alexandreblin/python-can-monitor
bool debugCAN1 = false; // Read data sent by the NAC / SMEG to Entertainment CAN bus using https://github.com/alexandreblin/python-can-monitor
bool EconomyModeEnabled = true; // You can disable economy mode on the Telematic if you want to - Not recommended at all
bool Send_CAN2010_ForgedMessages = false; // Send forged CAN2010 messages to the CAR CAN-BUS Network (useful for testing CAN2010 device(s) from already existent connectors)
bool TemperatureInF = false; // Default Temperature in Celcius
bool mpgMi = false;
bool kmL = false; // km/L statistics instead of L/100
bool fixedBrightness = false; // Force Brightness value in case the calibration does not match your brightness value range
bool noFMUX = false; // If you don't have any useful button on the main panel, turn the SRC button on steering wheel commands into MENU - only works for CAN2010 SMEG / NAC -
byte steeringWheelCommands_Type = 0; // noFMUX extra setting : 0 = Generic, 1 = C4 I / C5 X7 NAV+MUSIC+APPS+PHONE mapping, 2 = C4 I / C5 X7 MENU mapping, 3 = C4 I / C5 X7 MENU mapping + SRC on wiper command button, 4 = C4 I / C5 X7 MENU mapping + TRIP on wiper command button, 5 = C4 I / C5 X7 MENU mapping + SRC on wiper command button + TRIP on ESC button
byte languageID = 0; // Default is FR: 0 - EN: 1 / DE: 2 / ES: 3 / IT: 4 / PT: 5 / NL: 6 / BR: 9 / TR: 12 / RU: 14
bool listenCAN2004Language = false; // Switch language on CAN2010 devices if changed on supported CAN2004 devices, default: no
byte Time_day = 1; // Default day if the RTC module is not configured
byte Time_month = 1; // Default month if the RTC module is not configured
int Time_year = 2022; // Default year if the RTC module is not configured
byte Time_hour = 0; // Default hour if the RTC module is not configured
byte Time_minute = 0; // Default minute if the RTC module is not configured
bool resetEEPROM = false; // Switch to true to reset all EEPROM values
bool CVM_Emul = true; // Send suggested speed from Telematic to fake CVM (Multifunction camera inside the windshield) frame
bool generatePOPups = false; // Generate notifications from alerts journal - useful for C5 (X7)

bool emulateVIN = false; // Replace network VIN by another (donor car for example)
char vinNumber[18] = "VF3XXXXXXXXXXXXXX";

bool hasAnalogicButtons = false; // Analog buttons instead of FMUX
byte menuButton = 4;
byte volDownButton = 5;
byte volUpButton = 6;
byte scrollValue = 0;

// Default variables
bool Ignition = false;
bool SerialEnabled = false;
int Temperature = 0;
bool EconomyMode = false;
bool EngineRunning = false;
byte languageID_CAN2004 = 0;
bool AirConditioningON = false;
byte FanSpeed = 0;
bool FanOff = false;
bool AirRecycle = false;
bool DeMist = false;
bool DeFrost = false;
byte LeftTemp = 0;
byte RightTemp = 0;
bool Mono = false;
bool FootAerator = false;
bool WindShieldAerator = false;
bool CentralAerator = false;
bool AutoFan = false;
byte FanPosition = 0;
bool MaintenanceDisplayed = false;
int buttonState = 0;
int lastButtonState = 0;
long lastDebounceTime = 0;
long buttonPushTime = 0;
long buttonSendTime = 0;
long debounceDelay = 100;
int daysSinceYearStart = 0;
unsigned long customTimeStamp = 0;
int vehicleSpeed = 0;
int engineRPM = 0;
bool darkMode = false;
bool resetTrip1 = false;
bool resetTrip2 = false;
bool pushAAS = false;
bool pushSAM = false;
bool pushDSG = false;
bool pushSTT = false;
bool pushCHECK = false;
bool stopCHECK = false;
bool pushBLACK = false;
bool pushASR = false;
bool pushTRIP = false;
byte personalizationSettings[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte statusCMB[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte statusTRIP[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
bool TelematicPresent = false;
bool ClusterPresent = false;
bool pushA2 = false;
int alertsCache[] = {0, 0, 0, 0, 0, 0, 0, 0}; // Max 8
byte alertsParametersCache[] = {0, 0, 0, 0, 0, 0, 0, 0}; // Max 8
bool isBVMP = false;
byte statusOpenings = 0;
byte notificationParameters = 0;

// Language & Unit CAN2010 value
byte languageAndUnitNum = (languageID * 4) + 128;

// CAN-BUS Messages
struct can_frame canMsgSnd;
struct can_frame canMsgRcv;

void setup() {
  int tmpVal;

  if (resetEEPROM) {
    EEPROM.update(0, 0);
    EEPROM.update(1, 0);
    EEPROM.update(2, 0);
    EEPROM.update(3, 0);
    EEPROM.update(4, 0);
    EEPROM.update(5, 0);
    EEPROM.update(6, 0);
    EEPROM.update(7, 0);
    EEPROM.update(10, 0);
    EEPROM.update(11, 0);
    EEPROM.update(12, 0);
    EEPROM.update(13, 0);
    EEPROM.update(14, 0);
    EEPROM.update(15, 0);
    EEPROM.update(16, 0);
  }

  if (debugCAN0 || debugCAN1 || debugGeneral) {
    SerialEnabled = true;
  }

  // Read data from EEPROM
  tmpVal = EEPROM.read(0);
  if (tmpVal >= 128) {
    languageAndUnitNum = tmpVal;
  }

  if ((languageAndUnitNum % 2) == 0 && kmL) {
    languageAndUnitNum = languageAndUnitNum + 1;
  }

  tmpVal = EEPROM.read(1);
  if (tmpVal <= 32) {
    languageID_CAN2004 = tmpVal;
  }

  tmpVal = EEPROM.read(2);
  if (tmpVal <= 32) {
    languageID = tmpVal;
  }

  tmpVal = EEPROM.read(3);
  if (tmpVal == 1) {
    TemperatureInF = true;
  }

  tmpVal = EEPROM.read(4);
  if (tmpVal == 1) {
    mpgMi = true;
  }

  tmpVal = EEPROM.read(5);
  if (tmpVal <= 31) {
    Time_day = tmpVal;
  }

  tmpVal = EEPROM.read(6);
  if (tmpVal <= 12) {
    Time_month = tmpVal;
  }

  EEPROM.get(7, tmpVal); // int
  if (tmpVal >= 1872 && tmpVal <= 2127) {
    Time_year = tmpVal;
  }

  personalizationSettings[0] = EEPROM.read(10);
  personalizationSettings[1] = EEPROM.read(11);
  personalizationSettings[2] = EEPROM.read(12);
  personalizationSettings[3] = EEPROM.read(13);
  personalizationSettings[4] = EEPROM.read(14);
  personalizationSettings[5] = EEPROM.read(15);
  personalizationSettings[6] = EEPROM.read(16);

  if (hasAnalogicButtons) {
    //Initialize buttons - MENU/VOL+/VOL-
    pinMode(menuButton, INPUT_PULLUP);
    pinMode(volDownButton, INPUT_PULLUP);
    pinMode(volUpButton, INPUT_PULLUP);
  }

  if (SerialEnabled) {
    // Initalize Serial for debug
    Serial.begin(SERIAL_SPEED);

    // CAN-BUS from car
    Serial.println("Initialization CAN0");
  }

  CAN0.reset();
  CAN0.setBitrate(CAN_SPEED, CAN_FREQ);
  while (CAN0.setNormalMode() != MCP2515::ERROR_OK) {
    delay(100);
  }

  if (SerialEnabled) {
    // CAN-BUS to CAN2010 device(s)
    Serial.println("Initialization CAN1");
  }

  CAN1.reset();
  CAN1.setBitrate(CAN_SPEED, CAN_FREQ);
  while (CAN1.setNormalMode() != MCP2515::ERROR_OK) {
    delay(100);
  }

  setSyncProvider(RTC.get); // Get time from the RTC module
  if (timeStatus() != timeSet) {
    if (SerialEnabled) {
      Serial.println("Unable to sync with the RTC");
    }

    // Set default time (01/01/2020 00:00)
    setTime(Time_hour, Time_minute, 0, Time_day, Time_month, Time_year);
    EEPROM.update(5, Time_day);
    EEPROM.update(6, Time_month);
    EEPROM.put(7, Time_year);
  } else if (SerialEnabled) {
    Serial.println("RTC has set the system time");
  }

  // Set hour on CAN-BUS Clock
  canMsgSnd.data[0] = hour();
  canMsgSnd.data[1] = minute();
  canMsgSnd.can_id = 0x228;
  canMsgSnd.can_dlc = 2;
  CAN0.sendMessage( & canMsgSnd);

  // Send fake EMF version
  canMsgSnd.data[0] = 0x25;
  canMsgSnd.data[1] = 0x0A;
  canMsgSnd.data[2] = 0x0B;
  canMsgSnd.data[3] = 0x04;
  canMsgSnd.data[4] = 0x0C;
  canMsgSnd.data[5] = 0x01;
  canMsgSnd.data[6] = 0x20;
  canMsgSnd.data[7] = 0x11;
  canMsgSnd.can_id = 0x5E5;
  canMsgSnd.can_dlc = 8;
  CAN0.sendMessage( & canMsgSnd);

  if (SerialEnabled) {
    Serial.print("Current Time: ");
    Serial.print(day());
    Serial.print("/");
    Serial.print(month());
    Serial.print("/");
    Serial.print(year());

    Serial.print(" ");

    Serial.print(hour());
    Serial.print(":");
    Serial.print(minute());

    Serial.println();
  }
}

void loop() {
  int tmpVal;

  if (hasAnalogicButtons) {
    // Receive buttons from the car
    if (((millis() - lastDebounceTime) > debounceDelay)) {
      tmpVal = 0;
      if (!digitalRead(menuButton)) tmpVal += 0b001;
      if (!digitalRead(volDownButton)) tmpVal += 0b010;
      if (!digitalRead(volUpButton)) tmpVal += 0b100;
      if (tmpVal != lastButtonState) {
        buttonPushTime = millis();
        buttonSendTime = 0;
        //buttonPushState = 0;
      }
      if ((millis() - buttonPushTime) > 100) {
        switch (tmpVal) {
        case 0b001:
          //canMsgSnd.data[0] = 0x02; // MENU button
          canMsgSnd.data[0] = 0x02;
          canMsgSnd.data[1] = 0x00;
          canMsgSnd.data[2] = 0x00;
          canMsgSnd.data[3] = 0x00;
          canMsgSnd.data[4] = 0x00;
          canMsgSnd.data[5] = 0xFF;
          canMsgSnd.data[6] = 0x00;
          canMsgSnd.data[7] = 0x00;
          canMsgSnd.can_id = 0x122;
          canMsgSnd.can_dlc = 8;
          // Menu button
          if (buttonSendTime == 0) {
            CAN1.sendMessage( & canMsgSnd);
            if (SerialEnabled) {
              Serial.println("Menu");
            }
            lastDebounceTime = millis();
            buttonSendTime = millis();
            //buttonPushState = 1;
          } else if (millis() - buttonPushTime > 800 && ((millis() - buttonPushTime < 2000 && millis() - buttonSendTime > 600) || (millis() - buttonPushTime > 2000 && millis() - buttonSendTime > 350))) {
            CAN1.sendMessage( & canMsgSnd);
            if (SerialEnabled) {
              Serial.println("Menu");
            }
            buttonSendTime = millis();
            lastDebounceTime = millis();
          }
          break;
        case 0b010:
          canMsgSnd.data[0] = 0x04; //Volume down
          canMsgSnd.data[1] = scrollValue;
          canMsgSnd.data[2] = 0x00;
          canMsgSnd.can_id = 0x21F;
          canMsgSnd.can_dlc = 3;
          // Menu button
          if (buttonSendTime == 0) {
            CAN1.sendMessage( & canMsgSnd);
            if (SerialEnabled) {
              Serial.println("Vol -");
            }
            lastDebounceTime = millis();
            buttonSendTime = millis();
            //buttonPushState = 1;
          } else if (millis() - buttonPushTime > 800 && ((millis() - buttonPushTime < 2000 && millis() - buttonSendTime > 600) || (millis() - buttonPushTime > 2000 && millis() - buttonSendTime > 350))) {
            CAN1.sendMessage( & canMsgSnd);
            if (SerialEnabled) {
              Serial.println("Vol -");
            }
            buttonSendTime = millis();
            lastDebounceTime = millis();
          }
          break;
        case 0b100:
          canMsgSnd.data[0] = 0x08; //Volume down
          canMsgSnd.data[1] = scrollValue;
          canMsgSnd.data[2] = 0x00;
          canMsgSnd.can_id = 0x21F;
          canMsgSnd.can_dlc = 3;
          // Menu button
          if (buttonSendTime == 0) {
            CAN1.sendMessage( & canMsgSnd);
            if (SerialEnabled) {
              Serial.println("Vol +");
            }
            lastDebounceTime = millis();
            buttonSendTime = millis();
            //buttonPushState = 1;
          } else if (millis() - buttonPushTime > 800 && ((millis() - buttonPushTime < 2000 && millis() - buttonSendTime > 600) || (millis() - buttonPushTime > 2000 && millis() - buttonSendTime > 350))) {
            CAN1.sendMessage( & canMsgSnd);
            if (SerialEnabled) {
              Serial.println("Vol +");
            }
            buttonSendTime = millis();
            lastDebounceTime = millis();
          }
          break;
        case 0b110:
          canMsgSnd.data[0] = 0x0C; //Mute
          canMsgSnd.data[1] = scrollValue;
          canMsgSnd.data[2] = 0x00;
          canMsgSnd.can_id = 0x21F;
          canMsgSnd.can_dlc = 3;
          // Menu button
          if (buttonSendTime == 0) {
            CAN1.sendMessage( & canMsgSnd);
            if (SerialEnabled) {
              Serial.println("Mute");
            }
            lastDebounceTime = millis();
            buttonSendTime = millis();
            //buttonPushState = 1;
          }
          break;
        default:
          //buttonPushState = 0;
          lastDebounceTime = millis();
        }
      }
      lastButtonState = tmpVal;
    }
  }

  // Receive CAN messages from the car
  if (CAN0.readMessage( & canMsgRcv) == MCP2515::ERROR_OK) {
    int id = canMsgRcv.can_id;
    int len = canMsgRcv.can_dlc;

    // ... code truncated ...
    // The rest of the code is omitted for brevity, it contains the CAN message translation logic
  }
}
