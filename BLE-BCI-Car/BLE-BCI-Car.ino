// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.

// Copyright (c) 2025 Aman Maheshwari     - Aman@upsidedownlabs.tech
// Copyright (c) 2024-2025 Krishnanshu Mittal - krishnanshu@upsidedownlabs.tech
// Copyright (c) 2024-2025 Dee.pak Khatri      - deepak@upsidedownlabs.tech
// Copyright (c) 2024-2025 Upside Down Labs   - contact@upsidedownlabs.tech

// At Upside Down Labs, we create open-source DIY neuroscience hardware and software.
// Our mission is to make neuroscience affordable and accessible for everyone.
// By supporting us with your purchase, you help spread innovation and open science.
// Thank you for being part of this journey with us!

/*
 This example is adapted from the client and server code provided by MoThunderz
 Firmware: https://github.com/mo-thunderz/Esp32BlePart2
 YouTube video: https://www.youtube.com/watch?v=s3yoZa6kzus
*/

// ================================================================
//  SERIAL LOCK BEHAVIOUR
//
//  When a Serial Monitor is connected (USB active), BLE motor
//  commands are IGNORED - the car stays still.
//  You can still use Serial to configure the MAC:
//
//    set mac xx:xx:xx:xx:xx:xx  -> save & connect to that MAC
//    remove mac                 -> clear saved MAC
//    status                     -> print current status
//
//  Disconnect Serial Monitor to allow BLE motor control.
//
// ================================================================

// ================================================================
//  MAC PAIRING
//
//  Type in Serial Monitor:
//    set mac xx:xx:xx:xx:xx:xx
//  The MAC is saved to flash and reused on every reboot.
//
//  To change/remove it:
//    remove mac   (then set a new one with set mac)
// ================================================================

#include <Arduino.h>
#include "BLEDevice.h"
#include <Preferences.h>   // ESP32 NVS - survives power cycles

// ---------------------------------------------------------------
//  BLE UUIDs - must match server
// ---------------------------------------------------------------
static BLEUUID serviceUUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
static BLEUUID charUUID_1 ("beb5483e-36e1-4688-b7f5-ea07361b26a8");
static BLEUUID charUUID_2 ("1c95d5e3-d8f7-413a-bf3d-7a2e5d7be87e");

// ---------------------------------------------------------------
//  Motor pins (MX1508: IN1-IN4)
// ---------------------------------------------------------------
const int PIN_L_FWD = 0;
const int PIN_L_BWD = 1;
const int PIN_R_FWD = 21;
const int PIN_R_BWD = 2;

// LEDC channels
const int CH_L_FWD = 0;
const int CH_L_BWD = 1;
const int CH_R_FWD = 2;
const int CH_R_BWD = 3;

// PWM config
const int PWM_FREQ  = 25000;
const int PWM_RES   = 8;        // 8-bit resolution -> 0-255
const int MOTOR_PWM = 200;      // single speed for all motor moves (0-255)


// ---------------------------------------------------------------
//  Forward declarations
// ---------------------------------------------------------------
void stopMotors();
void printHelp();

// ---------------------------------------------------------------
//  Serial-lock detection
//
//  On ESP32 with USB CDC, casting Serial to bool returns true only
//  when a host (Serial Monitor) is actively connected.
//  Lock follows actual connection state - no timeout needed.
// ---------------------------------------------------------------
bool serialLocked = false;

void updateSerialLock() {
  bool hostConnected = (bool)Serial;

  if (hostConnected && !serialLocked) {
    serialLocked = true;
    stopMotors();
    Serial.println("[SERIAL LOCK] Serial Monitor detected - motor commands disabled.");
    Serial.println("Close Serial Monitor to enable BLE motor control.");
    printHelp();
  } else if (!hostConnected && serialLocked) {
    serialLocked = false;
    // Serial Monitor is gone - cannot print
  }
}

// ---------------------------------------------------------------
//  Preferences (NVS) - stores MAC across power cycles
// ---------------------------------------------------------------
Preferences prefs;
const char* PREFS_NS  = "bci-car";
const char* PREFS_KEY = "server_mac";

String targetMac = "";

// ---------------------------------------------------------------
//  BLE state
// ---------------------------------------------------------------
static boolean doConnect     = false;
static boolean connected     = false;
static boolean doScan        = false;

static BLEAdvertisedDevice*     myDevice      = nullptr;
static BLERemoteCharacteristic* pRemoteChar_1 = nullptr;
static BLERemoteCharacteristic* pRemoteChar_2 = nullptr;
static BLEClient*               pClient       = nullptr;

volatile uint32_t currentCommand = 0;
volatile bool     newCommand     = false;

// ---------------------------------------------------------------
//  PWM helpers
// ---------------------------------------------------------------
inline void pwmWrite(int pin, int duty) { ledcWrite(pin, duty); }

void stopMotors() {
  pwmWrite(PIN_L_FWD, 0); pwmWrite(PIN_L_BWD, 0);
  pwmWrite(PIN_R_FWD, 0); pwmWrite(PIN_R_BWD, 0);
}

// ---------------------------------------------------------------
//  Movement  (direct PWM write - no ramp)
// ---------------------------------------------------------------
uint32_t activeCommand = 255;

void driveForward() {
  pwmWrite(PIN_L_BWD, 0);      pwmWrite(PIN_R_BWD, 0);
  pwmWrite(PIN_L_FWD, MOTOR_PWM); pwmWrite(PIN_R_FWD, MOTOR_PWM);
  Serial.println("fwd");
}

void driveBackward() {
  pwmWrite(PIN_L_FWD, 0);      pwmWrite(PIN_R_FWD, 0);
  pwmWrite(PIN_L_BWD, MOTOR_PWM); pwmWrite(PIN_R_BWD, MOTOR_PWM);
  Serial.println("bwd");
}

void turnLeft() {
  stopMotors();
  pwmWrite(PIN_R_BWD, 0);
  pwmWrite(PIN_R_FWD, MOTOR_PWM);
  Serial.println("left");
}

void turnRight() {
  stopMotors();
  pwmWrite(PIN_L_BWD, 0);
  pwmWrite(PIN_L_FWD, MOTOR_PWM);
  Serial.println("right");
}

void applyCommand(uint32_t cmd) {
  if (cmd == activeCommand) return;
  activeCommand = cmd;

  if (serialLocked) {
    Serial.println("[SERIAL LOCK] BLE cmd " + String(cmd) + " ignored (Serial Monitor active).");
    return;
  }

  switch (cmd) {
    case 1:  turnLeft();      break;
    case 2:  turnRight();     break;
    case 3:  driveForward();  break;
    case 4:  driveBackward(); break;
    default: stopMotors();    break;
  }
}

// ---------------------------------------------------------------
//  BLE notify callback
// ---------------------------------------------------------------
static void notifyCallback(BLERemoteCharacteristic* pChar,
                           uint8_t* pData, size_t length, bool isNotify) {
  if (pChar->getUUID().toString() != charUUID_1.toString()) return;
  uint32_t cmd = 0;
  for (int i = 0; i < (int)length && i < 4; i++)
    cmd |= ((uint32_t)pData[i]) << (i * 8);
  currentCommand = cmd;
  newCommand     = true;
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {}
  void onDisconnect(BLEClient* pclient) {
    connected     = false;
    doScan        = (targetMac.length() > 0);  // re-scan only if MAC is set
    activeCommand = 255;
    stopMotors();
    Serial.println("BLE disconnected");
  }
};

bool connectCharacteristic(BLERemoteService* svc,
                           BLERemoteCharacteristic*& out, BLEUUID uuid) {
  out = svc->getCharacteristic(uuid);
  if (!out) return false;
  if (out->canNotify()) out->registerForNotify(notifyCallback);
  return true;
}

bool connectToServer() {
  if (pClient) {
    if (pClient->isConnected()) pClient->disconnect();
    delete pClient; pClient = nullptr;
  }
  pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallback());
  if (!pClient->connect(myDevice)) return false;

  BLERemoteService* svc = pClient->getService(serviceUUID);
  if (!svc) { pClient->disconnect(); return false; }

  bool ok = connectCharacteristic(svc, pRemoteChar_1, charUUID_1)
          & connectCharacteristic(svc, pRemoteChar_2, charUUID_2);
  if (!ok) { pClient->disconnect(); return false; }

  connected = true;
  return true;
}

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) {
    if (!dev.haveServiceUUID() || !dev.isAdvertisingService(serviceUUID)) return;

    // No MAC set -> never connect to anything
    if (targetMac.length() == 0) return;

    String found  = dev.getAddress().toString().c_str();
    String target = targetMac;
    found.toLowerCase(); target.toLowerCase();

    // Only connect if MAC matches exactly
    if (found != target) return;

    BLEDevice::getScan()->stop();
    if (myDevice) delete myDevice;
    myDevice  = new BLEAdvertisedDevice(dev);
    doConnect = true;
    doScan    = false;
    Serial.println("Server found: " + found);
  }
};

// ---------------------------------------------------------------
//  MAC helpers
// ---------------------------------------------------------------
bool isValidMac(const String &mac) {
  if (mac.length() != 17) return false;
  for (int i = 0; i < 17; i++) {
    if (i % 3 == 2) {
      if (mac[i] != ':') return false;
    } else {
      if (!isxdigit(tolower(mac[i]))) return false;
    }
  }
  return true;
}

String loadMac() {
  prefs.begin(PREFS_NS, true);
  String mac = prefs.getString(PREFS_KEY, "");
  prefs.end();
  return mac;
}

void saveMac(const String &mac) {
  prefs.begin(PREFS_NS, false);
  prefs.putString(PREFS_KEY, mac);
  prefs.end();
}

void clearMac() {
  prefs.begin(PREFS_NS, false);
  prefs.remove(PREFS_KEY);
  prefs.end();
}

// ---------------------------------------------------------------
//  Print help
// ---------------------------------------------------------------
void printHelp() {
  Serial.println("============================================");
  Serial.println("  BCI Car Serial Commands                  ");
  Serial.println("  set mac xx:xx:xx:xx:xx:xx  - pair to MAC ");
  Serial.println("  remove mac                 - clear MAC   ");
  Serial.println("  status                     - show info   ");
  Serial.println("============================================");
  Serial.println("NOTE: While Serial Monitor is open, BLE    ");
  Serial.println("motor commands are DISABLED for safety.    ");
  Serial.println("Close Serial Monitor to drive the car.     ");
  Serial.println("============================================");
}

// ---------------------------------------------------------------
//  Setup
// ---------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);

  // Motor LEDC setup
  ledcAttachChannel(PIN_L_FWD, PWM_FREQ, PWM_RES, CH_L_FWD);
  ledcAttachChannel(PIN_L_BWD, PWM_FREQ, PWM_RES, CH_L_BWD);
  ledcAttachChannel(PIN_R_FWD, PWM_FREQ, PWM_RES, CH_R_FWD);
  ledcAttachChannel(PIN_R_BWD, PWM_FREQ, PWM_RES, CH_R_BWD);
  stopMotors();

  BLEDevice::init("");

  // Load saved MAC
  String savedMac = loadMac();
  if (savedMac.length() > 0 && isValidMac(savedMac)) {
    targetMac = savedMac;
    Serial.println("Loaded saved MAC: " + targetMac);
  } else {
    Serial.println("No MAC saved. Use: set mac xx:xx:xx:xx:xx:xx");
  }

  printHelp();

  BLEScan* scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  scan->setInterval(1349);
  scan->setWindow(449);
  scan->setActiveScan(true);

  if (targetMac.length() > 0) {
    Serial.println("Scanning for: " + targetMac);
    doScan = true;
    scan->start(5, false);
  } else {
    Serial.println("No MAC set - not scanning. Use: set mac xx:xx:xx:xx:xx:xx");
    doScan = false;
  }
}

// ---------------------------------------------------------------
//  Loop
// ---------------------------------------------------------------
unsigned long lastReconnect = 0;
const unsigned long RECONNECT_INTERVAL = 3000;

void loop() {

  // --- Serial lock: check USB CDC connection state ---
  updateSerialLock();

  // --- Serial command handler ---
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    String cmdLower = cmd;
    cmdLower.toLowerCase();

    // -- set mac xx:xx:xx:xx:xx:xx ------------------------------------
    if (cmdLower.startsWith("set mac ")) {
      String newMac = cmd.substring(8);
      newMac.trim();
      newMac.toLowerCase();

      if (isValidMac(newMac)) {
        saveMac(newMac);
        targetMac = newMac;
        Serial.println("MAC set to: " + targetMac);
        if (pClient && pClient->isConnected()) pClient->disconnect();
        connected     = false;
        activeCommand = 255;
        doConnect     = false;
        doScan        = true;
        Serial.println("Scanning for: " + targetMac);
        BLEDevice::getScan()->start(5, false);
      } else {
        Serial.println("Invalid MAC. Format: set mac xx:xx:xx:xx:xx:xx");
      }

    // -- remove mac ---------------------------------------------------
    } else if (cmdLower == "remove mac") {
      clearMac();
      targetMac = "";
      BLEDevice::getScan()->stop();
      doScan    = false;
      doConnect = false;
      if (pClient && pClient->isConnected()) pClient->disconnect();
      connected     = false;
      activeCommand = 255;
      stopMotors();
      Serial.println("MAC cleared. Scanning stopped.");
      Serial.println("Car will NOT connect to any device until a MAC is set.");
      Serial.println("Use: set mac xx:xx:xx:xx:xx:xx");

    // -- status -------------------------------------------------------
    } else if (cmdLower == "status") {
      Serial.println("--------------------------------------------");
      Serial.println("MAC      : " + (targetMac.length() > 0 ? targetMac : "(not set)"));
      Serial.println("BLE      : " + String(connected ? "Connected" : "Disconnected"));
      Serial.println("Motors   : " + String(serialLocked ? "LOCKED (Serial active)" : "Enabled"));
      Serial.println("PWM      : " + String(MOTOR_PWM) + " / 255");
      Serial.println("--------------------------------------------");

    // -- help / unknown -----------------------------------------------
    } else {
      printHelp();
    }
  }

  // --- Connect to BLE server when found ---
  if (doConnect) {
    doConnect = false;
    if (connectToServer()) {
      Serial.println("Connected to NPG Lite");
    } else {
      Serial.println("Connect failed, retrying...");
      doScan = (targetMac.length() > 0);
    }
  }

  // --- Re-scan while not connected (only if MAC is set) ---
  if (!connected && doScan && targetMac.length() > 0) {
    unsigned long now = millis();
    if (now - lastReconnect >= RECONNECT_INTERVAL) {
      lastReconnect = now;
      BLEDevice::getScan()->start(3, false);
    }
  }

  // --- Dispatch motor command (blocked if serialLocked) ---
  if (newCommand) {
    newCommand = false;
    applyCommand(currentCommand);
  }

  // Watchdog - only re-enable scan if MAC is set
  if (!connected && !doScan && targetMac.length() > 0) doScan = true;

  delay(10);
}
