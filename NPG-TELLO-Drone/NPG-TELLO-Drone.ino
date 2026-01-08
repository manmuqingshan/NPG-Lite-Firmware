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

// Copyright (c) 2025 Aman Maheshwari - Aman@upsidedownlabs.tech
// Copyright (c) 2024 - 2025 Deepak Khatri - deepak@upsidedownlabs.tech
// Copyright (c) 2024 - 2025 Upside Down Labs - contact@upsidedownlabs.tech

// At Upside Down Labs, we create open-source DIY neuroscience hardware and software.
// Our mission is to make neuroscience affordable and accessible for everyone.
// By supporting us with your purchase, you help spread innovation and open science.
// Thank you for being part of this journey with us!

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include "esp_dsp.h"
#include <vector>
#include <WiFi.h>
#include <WiFiUdp.h>

typedef struct
{
  float delta, theta, alpha, beta, gamma, total;
} BandpowerResults;

// constants won't change. They're used here to set pin numbers:
const char *tello_pass = "";  // Tello has no password

WiFiUDP udp;
const char *TELLO_IP = "192.168.10.1";
const int TELLO_PORT = 8889;

// UDP connection status
bool udpConnected = false;
unsigned long lastConnectionCheck = 0;
const unsigned long CONNECTION_CHECK_INTERVAL = 5000;  // Check every 5 seconds

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastCmdTime = 0;
const unsigned long cmdCooldown = 200;  // 0.2 sec between commands

void sendCommandSafe(const String &cmd) {
  unsigned long now = millis();

  // enforce cooldown
  if (now - lastCmdTime < cmdCooldown)
    return;

  // Send
  udp.beginPacket(TELLO_IP, TELLO_PORT);
  udp.print(cmd);
  udp.endPacket();
  Serial.print("Sent: ");
  Serial.println(cmd);

  lastCmdTime = now;
}

// Some variables to keep track on device connected
Adafruit_NeoPixel pixel(6, 15, NEO_GRB + NEO_KHZ800);
uint32_t betaThreshold = 8;

// ----------------- USER CONFIGURATION -----------------
#define SAMPLE_RATE 512  // samples per second
#define FFT_SIZE 512     // must be a power of two
#define BAUD_RATE 115200
#define INPUT_PIN1 A0  // EEG input pin
#define INPUT_PIN2 A1  // Left hand EMG input pin
#define INPUT_PIN3 A2  // Right hand EMG input pin
#define BOOT_BUTTON 9  // NPG-Lite BOOT button (active LOW)
#define BUZZER_PIN 8   // Buzzer pin
#define LED_MOTOR_PIN 7

// EEG bands (Hz)
#define DELTA_LOW 0.5f
#define DELTA_HIGH 4.0f
#define THETA_LOW 4.0f
#define THETA_HIGH 8.0f
#define ALPHA_LOW 8.0f
#define ALPHA_HIGH 13.0f
#define BETA_LOW 13.0f
#define BETA_HIGH 30.0f
#define GAMMA_LOW 30.0f
#define GAMMA_HIGH 45.0f

#define SMOOTHING_FACTOR 0.63f
#define EPS 1e-7f
#define NOTE_C4 262
#define NOTE_G3 196
#define NOTE_A3 220
#define NOTE_B3 247

// ----------------- USER CONFIGURATION -----------------
// Add this after existing defines:
#define ENVELOPE_WINDOW_MS 100  // Smoothing window in milliseconds
#define ENVELOPE_WINDOW_SIZE ((ENVELOPE_WINDOW_MS * SAMPLE_RATE) / 1000)

const unsigned long BLINK_DEBOUNCE_MS = 250;  // minimal spacing between individual blinks
const unsigned long DOUBLE_BLINK_MS = 800;    // max time between the two blinks
const unsigned long TRIPLE_BLINK_MS = 1000;   // max time between all three blinks

// ===== JAW CLENCH CONFIGURATION =====
const unsigned long JAW_DEBOUNCE_MS = 500;        // ignore new clench triggers for this many ms after a valid clench
const unsigned long JAW_BLOCK_DURATION_MS = 500;  // Block other detections for 500ms after jaw clench
const float JAW_ON_THRESHOLD = 60.0;              // Jaw clench detection threshold (adjust based on your signal)
const float JAW_OFF_THRESHOLD = 50.0;             // hysteresis: must fall below this to re-arm

unsigned long lastBlinkTime = 0;   // time of most recent blink
unsigned long firstBlinkTime = 0;  // time of the first blink in a pair
int blinkCount = 0;                // how many valid blinks so far (0–2)
bool is_rotation_mode = LOW;       // current is_rotation_mode state (LOW = vertical movement, HIGH = rotation)

// Jaw clench variables
unsigned long lastJawDetectionTime = 0;  // Time when jaw clench was last detected
unsigned long lastJawClenchTime = 0;     // last time a clench was accepted
bool jawState = false;                   // true = currently considered in a clench

float BlinkThreshold = 50.0;

// Emergency stop variables
bool emergencyStop = false;                    // Global emergency stop flag
unsigned long lastButtonPressTime = 0;         // For button debouncing
const unsigned long BUTTON_DEBOUNCE_MS = 500;  // Button debounce time
unsigned long lastLandCommandTime = 0;         // For continuous land commands

// Police LED and buzzer timing
unsigned long lastPoliceFlashTime = 0;   // For police-style LED flashing
unsigned long lastBuzzerToggleTime = 0;  // For buzzer beeping
bool policeRedState = true;              // Toggle between red and blue
bool buzzerState = false;                // Current buzzer state

// notes in the melody:
int melody[] = {
  NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4
};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {
  4, 8, 8, 4, 4, 4, 4, 4
};

// Add these after existing buffers:
float envelopeBuffer[ENVELOPE_WINDOW_SIZE] = { 0 };
int envelopeIndex = 0;
float envelopeSum = 0;
float currentEEGEnvelope = 0;

// Separate envelope for jaw clench detection (different frequency characteristics)
float jawEnvelopeBuffer[ENVELOPE_WINDOW_SIZE] = { 0 };
int jawEnvelopeIndex = 0;
float jawEnvelopeSum = 0;
float currentJawEnvelope = 0;
static bool firstCommandSent = false;
static bool secondCommandScheduled = false;
// ----------------- BUFFERS & TYPES -----------------
float inputBuffer[FFT_SIZE];
float powerSpectrum[FFT_SIZE / 2];

// For two-real FFT trick
__attribute__((aligned(16))) float y_cf[FFT_SIZE * 2];
float *y1_cf = &y_cf[0];

BandpowerResults smoothedPowers = { 0, 0, 0, 0, 0, 0 };

// ----------------- NOTCH FILTER CLASSES -----------------
// For 50Hz AC noise removal
// Band-Stop Butterworth IIR digital filter, generated using filter_gen.py.
// Sampling rate: 500.0 Hz, frequency: [48.0, 52.0] Hz.
// Filter is order 2, implemented as second-order sections (biquads).
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
class NotchFilter {
private:
  struct BiquadState {
    float z1 = 0;
    float z2 = 0;
  };

  BiquadState state1;
  BiquadState state2;

public:
  float process(float input) {
    float output = input;

    // First biquad section
    float x = output - (-1.56858163f * state1.z1) - (0.96424138f * state1.z2);
    output = 0.96508099f * x + (-1.56202714f * state1.z1) + (0.96508099f * state1.z2);
    state1.z2 = state1.z1;
    state1.z1 = x;

    // Second biquad section
    x = output - (-1.61100358f * state2.z1) - (0.96592171f * state2.z2);
    output = 1.00000000f * x + (-1.61854514f * state2.z1) + (1.00000000f * state2.z2);
    state2.z2 = state2.z1;
    state2.z1 = x;

    return output;
  }

  void reset() {
    state1.z1 = state1.z2 = 0;
    state2.z1 = state2.z2 = 0;
  }
};

// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
float highpass(float input) {
  float output = input;
  {
    static float z1 = 0, z2 = 0;  // filter section state - initialized to 0
    float x = output - -1.91327599 * z1 - 0.91688335 * z2;
    output = 0.95753983 * x + -1.91507967 * z1 + 0.95753983 * z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}

// ----------------- EMG FILTER CLASSES -----------------
// High-Pass Butterworth IIR digital filter, generated using filter_gen.py.
// Sampling rate: 500.0 Hz, frequency: 70.0 Hz.
// Filter is order 2, implemented as second-order sections (biquads).
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
class EMGHighPassFilter {
private:
  // Filter state for a single channel
  double z1 = 0.0;
  double z2 = 0.0;

public:
  // Process a single sample
  double process(double input) {
    const double x = input - -0.82523238 * z1 - 0.29463653 * z2;
    const double output = 0.52996723 * x + -1.05993445 * z1 + 0.52996723 * z2;

    // Update state
    z2 = z1;
    z1 = x;

    return output;
  }

  // Reset filter state
  void reset() {
    z1 = 0.0;
    z2 = 0.0;
  }
};

// Class to calculate EMG Envelope
class EnvelopeFilter {
private:
  std::vector<double> circularBuffer;
  double sum = 0.0;
  int dataIndex = 0;
  const int bufferSize;

public:
  EnvelopeFilter(int bufferSize)
    : bufferSize(bufferSize) {
    circularBuffer.resize(bufferSize, 0.0);
  }

  double getEnvelope(double absEmg) {
    sum -= circularBuffer[dataIndex];
    sum += absEmg;
    circularBuffer[dataIndex] = absEmg;
    dataIndex = (dataIndex + 1) % bufferSize;
    return (sum / bufferSize);
  }
};

// NEW: Low-pass filter for EEG signals
float EEGFilter(float input) {
  float output = input;
  {
    static float z1 = 0, z2 = 0;
    float x = output - -1.22465158 * z1 - 0.45044543 * z2;
    output = 0.05644846 * x + 0.11289692 * z1 + 0.05644846 * z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}

// NEW: Band-pass filter for jaw clench detection (10-50Hz)
float jawClenchFilter(float input) {
  float output = input;
  {
    static float z1 = 0, z2 = 0;  // filter section state - initialized to 0
    float x = output - -0.85080258 * z1 - 0.30256882 * z2;
    output = 0.53834285 * x + -1.07668570 * z1 + 0.53834285 * z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}

// ----------------- ENVELOPE FUNCTION -----------------
float updateEEGEnvelope(float sample) {
  float absSample = fabs(sample);  // Rectify EEG signal

  // Update circular buffer and running sum
  envelopeSum -= envelopeBuffer[envelopeIndex];
  envelopeSum += absSample;
  envelopeBuffer[envelopeIndex] = absSample;
  envelopeIndex = (envelopeIndex + 1) % ENVELOPE_WINDOW_SIZE;

  return envelopeSum / ENVELOPE_WINDOW_SIZE;  // Return moving average
}

float updateJawEnvelope(float sample) {
  float absSample = fabs(sample);  // Rectify jaw signal

  // Update circular buffer and running sum
  jawEnvelopeSum -= jawEnvelopeBuffer[jawEnvelopeIndex];
  jawEnvelopeSum += absSample;
  jawEnvelopeBuffer[jawEnvelopeIndex] = absSample;
  jawEnvelopeIndex = (jawEnvelopeIndex + 1) % ENVELOPE_WINDOW_SIZE;

  return jawEnvelopeSum / ENVELOPE_WINDOW_SIZE;  // Return moving average
}

NotchFilter notchFilters[3];         // Notch filters for all 3 input channels
EMGHighPassFilter emgFilters[3];     // Fixed: Changed from 2 to 3 elements for all channels
EnvelopeFilter envelopeFilter1(16);  // Envelope detector for left EMG
EnvelopeFilter envelopeFilter2(16);  // Envelope detector for right EMG

// ----------------- BANDPOWER & SMOOTHING -----------------
BandpowerResults calculateBandpower(float *ps, float binRes, int halfSize) {
  BandpowerResults r = { 0, 0, 0, 0, 0, 0 };
  for (int i = 1; i < halfSize; i++) {
    float freq = i * binRes;
    float p = ps[i];
    r.total += p;
    if (freq >= DELTA_LOW && freq < DELTA_HIGH)
      r.delta += p;
    else if (freq >= THETA_LOW && freq < THETA_HIGH)
      r.theta += p;
    else if (freq >= ALPHA_LOW && freq < ALPHA_HIGH)
      r.alpha += p;
    else if (freq >= BETA_LOW && freq < BETA_HIGH)
      r.beta += p;
    else if (freq >= GAMMA_LOW && freq < GAMMA_HIGH)
      r.gamma += p;
  }
  return r;
}

void smoothBandpower(const BandpowerResults *raw, BandpowerResults *s) {
  s->delta = SMOOTHING_FACTOR * raw->delta + (1 - SMOOTHING_FACTOR) * s->delta;
  s->theta = SMOOTHING_FACTOR * raw->theta + (1 - SMOOTHING_FACTOR) * s->theta;
  s->alpha = SMOOTHING_FACTOR * raw->alpha + (1 - SMOOTHING_FACTOR) * s->alpha;
  s->beta = SMOOTHING_FACTOR * raw->beta + (1 - SMOOTHING_FACTOR) * s->beta;
  s->gamma = SMOOTHING_FACTOR * raw->gamma + (1 - SMOOTHING_FACTOR) * s->gamma;
  s->total = SMOOTHING_FACTOR * raw->total + (1 - SMOOTHING_FACTOR) * s->total;
}

// ----------------- DSP FFT SETUP -----------------
void initFFT() {
  // initialize esp-dsp real-FFT (two-real trick)
  esp_err_t err = dsps_fft2r_init_fc32(NULL, FFT_SIZE);
  if (err != ESP_OK) {
    Serial.println("FFT init failed");
    while (1)
      delay(10);
  }
}

// ----------------- FFT + BANDPOWER + PEAK -----------------
void processFFT() {
  // pack real→complex: real=inputBuffer, imag=0
  for (int i = 0; i < FFT_SIZE; i++) {
    y_cf[2 * i] = inputBuffer[i];
    y_cf[2 * i + 1] = 0.0f;
  }

  // FFT
  dsps_fft2r_fc32(y_cf, FFT_SIZE);
  dsps_bit_rev_fc32(y_cf, FFT_SIZE);
  dsps_cplx2reC_fc32(y_cf, FFT_SIZE);

  // magnitude² spectrum
  int half = FFT_SIZE / 2;
  for (int i = 0; i < half; i++) {
    float re = y1_cf[2 * i];
    float im = y1_cf[2 * i + 1];
    powerSpectrum[i] = re * re + im * im;
  }

  // detect peak bin (skip i=0)
  int maxIdx = 1;
  float maxP = powerSpectrum[1];
  for (int i = 2; i < half; i++) {
    if (powerSpectrum[i] > maxP) {
      maxP = powerSpectrum[i];
      maxIdx = i;
    }
  }
  float binRes = float(SAMPLE_RATE) / FFT_SIZE;
  float peakHz = maxIdx * binRes;

  // bandpower & smoothing
  BandpowerResults raw = calculateBandpower(powerSpectrum, binRes, half);
  smoothBandpower(&raw, &smoothedPowers);
  float T = smoothedPowers.total + EPS;
  Serial.print("Beta percentage: ");
  Serial.println((smoothedPowers.beta / T) * 100);  // for debugging purpose only

  // Only send takeoff command if not in emergency stop mode
  if (!emergencyStop && ((smoothedPowers.beta / T) * 100) > betaThreshold) {
    sendCommandSafe("takeoff");
  }
}

// Function to find and connect to any Tello drone
bool connectToAnyTello() {
  Serial.println("Scanning for Tello drones...");

  // Scan for available WiFi networks
  int n = WiFi.scanNetworks();
  Serial.print("Found ");
  Serial.print(n);
  Serial.println(" networks");

  String telloSSID = "";

  // Look for networks with "TELLO-" prefix
  for (int i = 0; i < n; i++) {
    String ssid = WiFi.SSID(i);
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(ssid);
    Serial.print(" (");
    Serial.print(WiFi.RSSI(i));
    Serial.println(" dBm)");

    if (ssid.startsWith("TELLO-")) {
      telloSSID = ssid;
      Serial.print("Found Tello drone: ");
      Serial.println(telloSSID);
      break;
    }
  }

  WiFi.scanDelete();  // Clear scan results

  if (telloSSID.length() == 0) {
    Serial.println("No Tello drone found!");
    return false;
  }

  // Connect to the found Tello
  Serial.print("Connecting to ");
  Serial.println(telloSSID);

  // If already connected to a different network, disconnect first
  if (WiFi.status() == WL_CONNECTED) {
    WiFi.disconnect();
    delay(100);
  }

  WiFi.begin(telloSSID.c_str(), tello_pass);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to Tello!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    return true;
  } else {
    Serial.println("\nFailed to connect to Tello!");
    return false;
  }
}

// Function to set all LEDs to police-style flashing
void setPoliceLEDs() {
  unsigned long now = millis();
  static byte flashCount = 0;    // Count flashes: 0-3 (red, red, blue, blue)
  static bool ledState = false;  // true = ON, false = OFF for blinking

  // Pattern: Red flashes twice, then Blue flashes twice
  // Each flash is ON for 100ms, OFF for 100ms

  if (flashCount < 2) {
    // Red flashing phase (first 2 flashes)
    if (now - lastPoliceFlashTime >= 100) {
      lastPoliceFlashTime = now;
      ledState = !ledState;

      if (ledState) {
        // Red LEDs ON
        pixel.setPixelColor(0, pixel.Color(255, 0, 0));  // Red
        pixel.setPixelColor(1, pixel.Color(255, 0, 0));  // Red
        pixel.setPixelColor(2, pixel.Color(255, 0, 0));  // Red
        pixel.setPixelColor(3, pixel.Color(0, 0, 0));    // Off
        pixel.setPixelColor(4, pixel.Color(0, 0, 0));    // Off
        pixel.setPixelColor(5, pixel.Color(0, 0, 0));    // Off
        Serial.println("Red flash ON");
      } else {
        // All LEDs OFF
        for (int i = 0; i < 6; i++) {
          pixel.setPixelColor(i, pixel.Color(0, 0, 0));
        }
        Serial.println("OFF between flashes");

        // Increment flash count when turning OFF (completes one flash)
        flashCount++;

        // If we've completed 2 red flashes, move to blue
        if (flashCount == 2) {
          ledState = false;  // Start with OFF state for blue
          Serial.println("Moving to blue flashes");
        }
      }
      pixel.show();
    }
  } else if (flashCount < 4) {
    // Blue flashing phase (next 2 flashes)
    if (now - lastPoliceFlashTime >= 100) {
      lastPoliceFlashTime = now;
      ledState = !ledState;

      if (ledState) {
        // Blue LEDs ON
        pixel.setPixelColor(0, pixel.Color(0, 0, 0));    // Off
        pixel.setPixelColor(1, pixel.Color(0, 0, 0));    // Off
        pixel.setPixelColor(2, pixel.Color(0, 0, 0));    // Off
        pixel.setPixelColor(3, pixel.Color(0, 0, 255));  // Blue
        pixel.setPixelColor(4, pixel.Color(0, 0, 255));  // Blue
        pixel.setPixelColor(5, pixel.Color(0, 0, 255));  // Blue
        Serial.println("Blue flash ON");
      } else {
        // All LEDs OFF
        for (int i = 0; i < 6; i++) {
          pixel.setPixelColor(i, pixel.Color(0, 0, 0));
        }
        Serial.println("OFF between flashes");

        // Increment flash count when turning OFF
        flashCount++;

        // Reset for next cycle when all 4 flashes are done
        if (flashCount == 4) {
          flashCount = 0;
          Serial.println("Restarting pattern");
        }
      }
      pixel.show();
    }
  }
}

// Function to control buzzer in emergency mode
void controlEmergencyBuzzer() {
  unsigned long now = millis();

  // Toggle buzzer on/off every 300ms for beeping effect
  if (now - lastBuzzerToggleTime >= 300) {
    lastBuzzerToggleTime = now;
    buzzerState = !buzzerState;

    if (buzzerState) {
      tone(BUZZER_PIN, 1000, 200);  // 1kHz tone for 200ms
    } else {
      noTone(BUZZER_PIN);
    }
  }
}

// Function to check connection status
void checkConnectionStatus() {
  unsigned long now = millis();

  // Only check periodically
  if (now - lastConnectionCheck >= CONNECTION_CHECK_INTERVAL) {
    lastConnectionCheck = now;

    if (WiFi.status() == WL_CONNECTED) {
      if (!udpConnected) {
        udpConnected = true;
        Serial.println("WiFi connected - UDP ready!");
        pixel.setPixelColor(5, pixel.Color(0, 255, 0));  // Green when connected
        pixel.show();
        
        // Initialize UDP if not already done
        if (udp.begin(8889) == 1) {
          Serial.println("UDP started successfully");
        }
      }
    } else {
      if (udpConnected) {
        udpConnected = false;
        Serial.println("WiFi disconnected! Attempting to reconnect...");
        pixel.setPixelColor(5, pixel.Color(255, 0, 0));  // Red when disconnected
        pixel.show();
        
        // Try to reconnect
        bool reconnected = connectToAnyTello();
        if (reconnected) {
          Serial.println("Successfully reconnected to Tello!");
          udp.begin(8889);
          udpConnected = true;
          pixel.setPixelColor(5, pixel.Color(0, 255, 0));  // Green when reconnected
          pixel.show();
        }
      }
    }
  }
}

// Function to set normal LED operation
void setNormalLEDs() {
  // Clear all LEDs
  for (int i = 0; i < 6; i++) {
    pixel.setPixelColor(i, pixel.Color(0, 0, 0));
  }

  // Set status LED based on UDP connection status
  if (udpConnected) {
    pixel.setPixelColor(5, pixel.Color(0, 255, 0));  // Green when UDP connected
  } else {
    pixel.setPixelColor(5, pixel.Color(255, 0, 0));  // Red when UDP not connected
  }
  pixel.show();
}

// Function to send continuous land commands in emergency mode
void sendContinuousLandCommands() {
  unsigned long now = millis();

  // Send first land command immediately when function is first called
  if (!firstCommandSent) {
    sendCommandSafe("land");
    lastLandCommandTime = now;
    firstCommandSent = true;
    secondCommandScheduled = true;
    Serial.println("Sending first land command...");
  }

  // Send second land command after 5 seconds
  if (secondCommandScheduled && (now - lastLandCommandTime >= 5000)) {
    sendCommandSafe("land");
    lastLandCommandTime = now;
    secondCommandScheduled = false;
    Serial.println("Sending second land command (after 5 seconds)...");
  }
}

// Function to handle emergency stop toggle
void handleEmergencyStop() {
  unsigned long now = millis();

  // Check button press with debouncing
  if (digitalRead(BOOT_BUTTON) == LOW && (now - lastButtonPressTime) > BUTTON_DEBOUNCE_MS) {
    lastButtonPressTime = now;

    // Toggle emergency stop
    emergencyStop = !emergencyStop;
    firstCommandSent = false;
    secondCommandScheduled = false;
    if (emergencyStop) {
      // Emergency stop activated
      Serial.println("EMERGENCY STOP ACTIVATED!");

      // Send initial land command immediately
      sendCommandSafe("land");
      lastLandCommandTime = now;

      // Start buzzer
      tone(BUZZER_PIN, 1000, 500);  // Initial long beep
      buzzerState = true;
      lastBuzzerToggleTime = now;

      Serial.println("All controls disabled. Press BOOT button again to resume.");
      Serial.println("Sending continuous land commands...");
    } else {
      // Emergency stop deactivated
      Serial.println("EMERGENCY STOP DEACTIVATED - Resuming normal operation");

      // Stop buzzer
      noTone(BUZZER_PIN);
      buzzerState = false;

      // Set LEDs back to normal
      setNormalLEDs();

      // Reset blink detection state
      blinkCount = 0;
      firstBlinkTime = 0;

      Serial.println("Normal operation resumed.");
    }
  }
}

void setup() {
  // drone
  Serial.begin(115200);

  // Initialize NeoPixel LED to RED on startup
  pixel.begin();
  pixel.setPixelColor(5, pixel.Color(255, 0, 0));  // Red on startup
  pixel.show();
  pinMode(LED_MOTOR_PIN, OUTPUT);

  // iterate over the notes of the melody:
  for (int thisNote = 0; thisNote < 8; thisNote++) {

    // to calculate the note duration, take one second divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(BUZZER_PIN, melody[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    digitalWrite(LED_MOTOR_PIN, HIGH);
    delay(pauseBetweenNotes / 2);
    digitalWrite(LED_MOTOR_PIN, LOW);
    delay(pauseBetweenNotes / 2);

    // stop the tone playing:
    noTone(BUZZER_PIN);
  }

  delay(500);

  // Try to connect to any Tello drone
  bool connected = false;
  int attempts = 0;
  
  while (!connected && attempts < 3) {
    Serial.print("Trying to connect to Tello (attempt ");
    Serial.print(attempts + 1);
    Serial.println(" of 3)...");
    
    connected = connectToAnyTello();
    attempts++;
    
    if (!connected && attempts <3) {
      Serial.println("Connection failed, retrying in 2 seconds...");
      delay(2000);
    }
  }

  if (!connected) {
    Serial.println("Could not connect to any Tello drone after 3 attempts.");
    Serial.println("Continuing without drone - device will wait for connection.");
    Serial.println("You can restart the device when Tello is ready.");
    
    // Set LED to indicate waiting for connection
    pixel.setPixelColor(5, pixel.Color(255, 165, 0));  // Orange - waiting
    pixel.show();
    
    // Continue setup without drone connection
  } else {
    // Only initialize UDP if WiFi is connected
    udp.begin(8889);
    udpConnected = true;

    // Enter SDK mode
    sendCommandSafe("command");
    delay(1000);
    sendCommandSafe("streamoff");
    delay(1000);
  }

  pinMode(INPUT_PIN1, INPUT);
  pinMode(INPUT_PIN2, INPUT);
  pinMode(INPUT_PIN3, INPUT);
  pinMode(BOOT_BUTTON, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);  // Ensure buzzer is off initially

  initFFT();

  Serial.println("System ready. Press BOOT button for emergency stop.");
}

void loop() {
  // Check connection status
  checkConnectionStatus();

  // Handle emergency stop toggle (always check button)
  handleEmergencyStop();

  // If emergency stop is active
  if (emergencyStop) {
    // Send continuous land commands
    sendContinuousLandCommands();

    // Control police-style LED flashing
    setPoliceLEDs();

    // Control buzzer beeping
    controlEmergencyBuzzer();

    // Don't process any EEG/EMG data while in emergency stop
    return;
  }

  // Normal operation below this point
  static uint16_t idx = 0;
  static unsigned long lastMicros = micros();
  unsigned long now = micros(), dt = now - lastMicros;
  lastMicros = now;

  // Update NeoPixel based on UDP connection status
  if (udpConnected) {
    pixel.setPixelColor(5, pixel.Color(0, 255, 0));  // Green when connected
  } else {
    pixel.setPixelColor(5, pixel.Color(255, 0, 0));  // Red when disconnected
  }
  pixel.show();

  static long timer = 0;
  timer -= dt;
  if (timer <= 0) {
    timer += 1000000L / SAMPLE_RATE;
    int raw1 = analogRead(INPUT_PIN1);
    int raw2 = analogRead(INPUT_PIN2);
    int raw3 = analogRead(INPUT_PIN3);

    // Fixed: Use existing filter objects instead of creating new ones
    float notchFiltered1 = notchFilters[0].process(raw1);
    float notchFiltered2 = notchFilters[1].process(raw2);
    float notchFiltered3 = notchFilters[2].process(raw3);

    float filteredEEG = EEGFilter(notchFiltered1);
    float filtered = highpass(filteredEEG);
    currentEEGEnvelope = updateEEGEnvelope(filtered);

    // NEW: Process jaw clench from same channel
    float jawSignal = jawClenchFilter(notchFiltered1);
    currentJawEnvelope = updateJawEnvelope(jawSignal);

    float filteredEMG2 = emgFilters[1].process(notchFiltered2);
    float filteredEMG3 = emgFilters[2].process(notchFiltered3);
    inputBuffer[idx++] = filteredEEG;

    float env1 = envelopeFilter1.getEnvelope(abs(filteredEMG2));  //ch1 - right EMG
    float env2 = envelopeFilter2.getEnvelope(abs(filteredEMG3));  //ch2 - left EMG

    // Normal running state processing
    unsigned long nowMs = millis();

    // ========== JAW CLENCH BLOCKING CHECK ==========
    bool jawBlockActive = (nowMs - lastJawDetectionTime) < JAW_BLOCK_DURATION_MS;

    // ========== JAW CLENCH DETECTION LOGIC ==========
    if (!jawState) {
      // Not currently clenching - look for rising edge + debounce
      if (currentJawEnvelope > JAW_ON_THRESHOLD && (nowMs - lastJawClenchTime) >= JAW_DEBOUNCE_MS) {
        jawState = true;
        lastJawClenchTime = nowMs;
        lastJawDetectionTime = nowMs;  // Start blocking period
        Serial.println("JAW CLENCH DETECTED!");

        is_rotation_mode = !is_rotation_mode;  // Toggle the is_rotation_mode state
        if (is_rotation_mode) {
          Serial.println("is_rotation_mode ON - EMG controls set to rotation");
          pixel.setPixelColor(0, pixel.Color(255, 255, 0));  // Yellow indicating running
        } else {
          Serial.println("is_rotation_mode OFF - EMG controls set to vertical movement");
          pixel.setPixelColor(0, pixel.Color(255, 0, 0));  // RED indicating running
        }
      }
    } else {
      // Currently in clench state - wait for signal to fall below OFF threshold
      if (currentJawEnvelope < JAW_OFF_THRESHOLD) {
        lastJawClenchTime = nowMs;
        jawState = false;
        Serial.println("Jaw clench released");
      }
    }

    // ========== BLINK DETECTION LOGIC ==========
    // Only process blinks if jaw clench isn't blocking
    if (!jawBlockActive) {
      // 1) Did we cross threshold and respect per‑blink debounce?
      if (currentEEGEnvelope > BlinkThreshold && (nowMs - lastBlinkTime) >= BLINK_DEBOUNCE_MS) {
        lastBlinkTime = nowMs;  // mark this blink

        if (blinkCount == 0) {
          // first blink of the sequence
          firstBlinkTime = nowMs;
          blinkCount = 1;
          Serial.println("First blink detected");
          pixel.setPixelColor(2, pixel.Color(0, 0, 0));
        } else if (blinkCount == 1 && (nowMs - firstBlinkTime) <= DOUBLE_BLINK_MS) {
          // second blink detected within double blink time window
          blinkCount = 2;
          Serial.println("Second blink - double blink detected");
        } else if (blinkCount == 2 && (nowMs - firstBlinkTime) <= TRIPLE_BLINK_MS) {
          // third blink detected within triple blink time window
          Serial.println("Third blink - TRIPLE BLINK DETECTED!");

          // Handle triple blink action
          sendCommandSafe("flip r");
          pixel.setPixelColor(2, pixel.Color(0, 0, 255));  // Blue color


          blinkCount = 0;  // Reset for next sequence
        } else {
          // Too slow or extra blink - restart sequence
          firstBlinkTime = nowMs;
          blinkCount = 1;
          Serial.println("Sequence timeout - restarting");
        }
      }

      // 2) Handle double blink timeout (when second blink wasn't followed by third)
      if (blinkCount == 2 && (nowMs - firstBlinkTime) > TRIPLE_BLINK_MS) {
        // This is a confirmed DOUBLE BLINK
        Serial.println("DOUBLE BLINK CONFIRMED - sending forward/backward commands");
        blinkCount = 0;  // Reset for next sequence
      }

      // 3) Handle single blink timeout (when no second blink arrives)
      if (blinkCount == 1 && (nowMs - firstBlinkTime) > DOUBLE_BLINK_MS) {
        // This was just a single blink, not part of a sequence
        Serial.println("Single blink - no action");
        blinkCount = 0;  // Reset for next sequence
      }
    }

    // ========== EMG CONTROLS ==========
    // Only process EMG controls if jaw clench isn't blocking
    if (!jawBlockActive) {
      if (env1 > 150) {
        if (is_rotation_mode)  // is_rotation_mode ON (rotation mode)
        {
          sendCommandSafe("forward 50");  // Move forward
          Serial.println("is_rotation_mode ON - EMG1 > 150 - Sent: forward 50");
        } else  // is_rotation_mode OFF (vertical movement mode)
        {
          sendCommandSafe("up 50");  // Move up
          Serial.println("is_rotation_mode OFF - EMG1 > 150 - Sent: up 50");
        }
      } else if (env2 > 150) {
        if (is_rotation_mode)  // is_rotation_mode ON (rotation mode)
        {
          sendCommandSafe("cw 50");  // Clockwise rotation
          Serial.println("is_rotation_mode ON - EMG2 > 150 - Sent: cw 50");
        } else  // is_rotation_mode OFF (vertical movement mode)
        {
          sendCommandSafe("down 50");  // Move down
          Serial.println("is_rotation_mode OFF - EMG2 > 150 - Sent: down 50");
        }
      }
    }
    // ========== END EMG CONTROLS ==========

    // Debug output (optional - uncomment to see envelope values)
    // Serial.print("Blink Envelope: ");
    // Serial.print(currentEEGEnvelope);
    // Serial.print(" | Jaw Envelope: ");
    // Serial.println(currentJawEnvelope);

    if (idx >= FFT_SIZE) {
      processFFT();
      idx = 0;
    }
  }
}