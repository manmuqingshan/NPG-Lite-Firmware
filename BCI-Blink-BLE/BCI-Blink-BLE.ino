// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.

// Copyright (c) 2025 Aman Maheshwari - Aman@upsidedownlabs.tech
// Copyright (c) 2025 Krishnanshu Mittal - krishnanshu@upsidedownlabs.tech
// Copyright (c) 2025 Deepak Khatri - deepak@upsidedownlabs.tech
// Copyright (c) 2025 Upside Down Labs - contact@upsidedownlabs.tech

// At Upside Down Labs, we create open-source DIY neuroscience hardware and software.
// Our mission is to make neuroscience affordable and accessible for everyone.
// By supporting us with your purchase, you help spread innovation and open science.
// Thank you for being part of this journey with us!

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "esp_dsp.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

// --- Config ---
#define SAMPLE_RATE         512
#define FFT_SIZE            512
#define BAUD_RATE           115200
#define INPUT_PIN           A0
#define PIXEL_PIN           15
#define PIXEL_COUNT         6
#define VIBRATOR_PIN        7

#define BLINK_SERVICE_UUID  "6910123a-eb0d-4c35-9a60-bebe1dcb549d"
#define BLINK_CHAR_UUID     "5f4f1107-7fc1-43b2-a540-0aa1a9f1ce78"

// BLE commands (1 byte each)
#define CMD_SINGLE_BLINK    1
#define CMD_DOUBLE_BLINK    2
#define CMD_TRIPLE_BLINK    3
#define CMD_JAW_CLENCH      4
#define CMD_FOCUS           5

// EEG frequency bands (Hz)
#define DELTA_LOW   0.5f
#define DELTA_HIGH  4.0f
#define THETA_LOW   4.0f
#define THETA_HIGH  8.0f
#define ALPHA_LOW   8.0f
#define ALPHA_HIGH  13.0f
#define BETA_LOW    13.0f
#define BETA_HIGH   30.0f
#define GAMMA_LOW   30.0f
#define GAMMA_HIGH  45.0f

#define SMOOTHING_FACTOR    0.63f
#define EPS                 1e-7f

typedef struct { float delta, theta, alpha, beta, gamma, total; } BandpowerResults;

// Envelope smoothing window
#define ENVELOPE_WINDOW_MS   100
#define ENVELOPE_WINDOW_SIZE ((ENVELOPE_WINDOW_MS * SAMPLE_RATE) / 1000)

// Blink detection timings
const float         BLINK_THRESHOLD   = 50.0f;
const unsigned long BLINK_DEBOUNCE_MS = 250;   // min gap between blinks
const unsigned long DOUBLE_BLINK_MS   = 800;   // max window from 1st to 2nd blink
const unsigned long TRIPLE_WINDOW_MS  = 600;   // max window from 2nd to 3rd blink

// Jaw clench detection
const float         JAW_ON_THRESHOLD      = 60.0f;
const float         JAW_OFF_THRESHOLD     = 50.0f;
const unsigned long JAW_DEBOUNCE_MS       = 500;
const unsigned long JAW_BLOCK_DURATION_MS = 500;  // suppress blinks after a clench

// Focus (beta band power)
const float         FOCUS_BETA_PCT    = 8.0f;   // beta % threshold to consider as focus
const unsigned long FOCUS_INTERVAL_MS = 1000;   // send at most once per second

Adafruit_NeoPixel pixels(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);


enum VibratePhase { V_IDLE, V_ON, V_GAP };
VibratePhase  vibratePhase     = V_IDLE;
int           vibrateRemaining = 0;
unsigned long vibrateUntil     = 0;

void startVibrate(int pulses)
{
  vibrateRemaining = pulses;
  vibratePhase     = V_ON;
  vibrateUntil     = millis() + 200;
  digitalWrite(VIBRATOR_PIN, HIGH);
}

void handleVibration()
{
  if (vibratePhase == V_IDLE) return;
  unsigned long now = millis();
  if (now < vibrateUntil) return;
  if (vibratePhase == V_ON) {
    digitalWrite(VIBRATOR_PIN, LOW);
    if (--vibrateRemaining > 0) { vibratePhase = V_GAP; vibrateUntil = now + 150; }
    else                          vibratePhase = V_IDLE;
  } else {
    digitalWrite(VIBRATOR_PIN, HIGH);
    vibratePhase = V_ON;
    vibrateUntil = now + 200;
  }
}

BLEServer*         pBleServer    = nullptr;
BLEService*        pBlinkService = nullptr;
BLECharacteristic* pBlinkChar    = nullptr;
static bool        clientConnected = false;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    clientConnected = true;
    Serial.println("BLE client connected");
    pixels.setPixelColor(0, pixels.Color(0, 20, 0));
    pixels.show();
    startVibrate(1);
  }
  void onDisconnect(BLEServer* pServer) override {
    clientConnected = false;
    Serial.println("BLE client disconnected");
    pixels.setPixelColor(0, pixels.Color(20, 0, 0));
    pixels.show();
    startVibrate(2);
    pServer->getAdvertising()->start();
  }
};

unsigned long lastBlinkTime   = 0;
unsigned long firstBlinkTime  = 0;
unsigned long secondBlinkTime = 0;
int           blinkCount      = 0;

unsigned long lastJawClenchTime    = 0;
unsigned long lastJawDetectionTime = 0;
bool          jawState             = false;

unsigned long lastFocusSentTime = 0;

// EEG envelope (blink detection)
float envelopeBuffer[ENVELOPE_WINDOW_SIZE] = {0};
int   envelopeIndex = 0;
float envelopeSum   = 0.0f;
float currentEEGEnvelope = 0.0f;

// Jaw envelope (jaw clench detection)
float jawEnvelopeBuffer[ENVELOPE_WINDOW_SIZE] = {0};
int   jawEnvelopeIndex = 0;
float jawEnvelopeSum   = 0.0f;
float currentJawEnvelope = 0.0f;

// FFT (focus / beta band)
BandpowerResults smoothedPowers = {0};
float inputBuffer[FFT_SIZE];
float powerSpectrum[FFT_SIZE / 2];
__attribute__((aligned(16))) float y_cf[FFT_SIZE * 2];
float *y1_cf = &y_cf[0];
uint16_t fftIdx = 0;

// --- Filters ---
// High-Pass Butterworth IIR digital filter, generated using filter_gen.py.
// Sampling rate: 512.0 Hz, frequency: 5.0 Hz.
// Filter is order 2, implemented as second-order sections (biquads).
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
float highpass(float input)
{
  float output = input;
  {
    static float z1, z2;
    float x = output - -1.91327599f*z1 - 0.91688335f*z2;
    output = 0.95753983f*x + -1.91507967f*z1 + 0.95753983f*z2;
    z2 = z1; z1 = x;
  }
  return output;
}

// Band-Stop Butterworth IIR digital filter, generated using filter_gen.py.
// Sampling rate: 512.0 Hz, frequency: [48.0, 52.0] Hz.
// Filter is order 2, implemented as second-order sections (biquads).
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
float Notch(float input)
{
  float output = input;
  {
    static float z1, z2;
    float x = output - -1.58696045f*z1 - 0.96505858f*z2;
    output = 0.96588529f*x + -1.57986211f*z1 + 0.96588529f*z2;
    z2 = z1; z1 = x;
  }
  {
    static float z1, z2;
    float x = output - -1.62761184f*z1 - 0.96671306f*z2;
    output = 1.00000000f*x + -1.63566226f*z1 + 1.00000000f*z2;
    z2 = z1; z1 = x;
  }
  return output;
}

// Low-Pass Butterworth IIR digital filter, generated using filter_gen.py.
// Sampling rate: 512.0 Hz, frequency: 45.0 Hz.
// Filter is order 2, implemented as second-order sections (biquads).
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
float EEGFilter(float input)
{
  float output = input;
  {
    static float z1, z2;
    float x = output - -1.24200128f*z1 - 0.45885207f*z2;
    output = 0.05421270f*x + 0.10842539f*z1 + 0.05421270f*z2;
    z2 = z1; z1 = x;
  }
  return output;
}

// High-Pass Butterworth IIR digital filter
// Sampling rate: 512.0 Hz, frequency: 70.0 Hz
// Filter is order 2, implemented as second-order sections (biquads)
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
class jawClenchFilter {
private:
    struct BiquadState { float z1 = 0, z2 = 0; };
    BiquadState state0;

public:
    float process(float input) {
        float output = input;

        // Biquad section 0
        float x0 = output - (-0.85080258f * state0.z1) - (0.30256882f * state0.z2);
        output = 0.53834285f * x0 + -1.07668570f * state0.z1 + 0.53834285f * state0.z2;
        state0.z2 = state0.z1;
        state0.z1 = x0;

        return output;
    }

    void reset() {
        state0.z1 = state0.z2 = 0;
    }
};

// --- Envelope ---
float updateEEGEnvelope(float sample)
{
  float a = fabs(sample);
  envelopeSum -= envelopeBuffer[envelopeIndex];
  envelopeSum += a;
  envelopeBuffer[envelopeIndex] = a;
  envelopeIndex = (envelopeIndex + 1) % ENVELOPE_WINDOW_SIZE;
  return envelopeSum / ENVELOPE_WINDOW_SIZE;
}

float updateJawEnvelope(float sample)
{
  float a = fabs(sample);
  jawEnvelopeSum -= jawEnvelopeBuffer[jawEnvelopeIndex];
  jawEnvelopeSum += a;
  jawEnvelopeBuffer[jawEnvelopeIndex] = a;
  jawEnvelopeIndex = (jawEnvelopeIndex + 1) % ENVELOPE_WINDOW_SIZE;
  return jawEnvelopeSum / ENVELOPE_WINDOW_SIZE;
}

// --- FFT / Band Power ---
BandpowerResults calculateBandpower(float *ps, float binRes, int halfSize)
{
  BandpowerResults r = {0};
  for (int i = 1; i < halfSize; i++) {
    float freq = i * binRes, p = ps[i];
    r.total += p;
    if      (freq >= DELTA_LOW && freq < DELTA_HIGH) r.delta += p;
    else if (freq >= THETA_LOW && freq < THETA_HIGH) r.theta += p;
    else if (freq >= ALPHA_LOW && freq < ALPHA_HIGH) r.alpha += p;
    else if (freq >= BETA_LOW  && freq < BETA_HIGH)  r.beta  += p;
    else if (freq >= GAMMA_LOW && freq < GAMMA_HIGH) r.gamma += p;
  }
  return r;
}

void smoothBandpower(const BandpowerResults *raw, BandpowerResults *s)
{
  float a = SMOOTHING_FACTOR, b = 1.0f - a;
  s->delta = a*raw->delta + b*s->delta;
  s->theta = a*raw->theta + b*s->theta;
  s->alpha = a*raw->alpha + b*s->alpha;
  s->beta  = a*raw->beta  + b*s->beta;
  s->gamma = a*raw->gamma + b*s->gamma;
  s->total = a*raw->total + b*s->total;
}

void processFFT()
{
  for (int i = 0; i < FFT_SIZE; i++) { y_cf[2*i] = inputBuffer[i]; y_cf[2*i+1] = 0.0f; }
  dsps_fft2r_fc32(y_cf, FFT_SIZE);
  dsps_bit_rev_fc32(y_cf, FFT_SIZE);
  dsps_cplx2reC_fc32(y_cf, FFT_SIZE);
  int half = FFT_SIZE / 2;
  for (int i = 0; i < half; i++) {
    float re = y1_cf[2*i], im = y1_cf[2*i+1];
    powerSpectrum[i] = re*re + im*im;
  }
  BandpowerResults raw = calculateBandpower(powerSpectrum, float(SAMPLE_RATE)/FFT_SIZE, half);
  smoothBandpower(&raw, &smoothedPowers);
}

// --- BLE ---
void sendCommand(uint8_t cmd)
{
  if (clientConnected) {
    pBlinkChar->setValue(&cmd, 1);
    pBlinkChar->notify();
  }
  Serial.print("CMD: "); Serial.println(cmd);
}

// --- Setup ---
void setup()
{
  Serial.begin(BAUD_RATE);
  delay(100);
  pinMode(INPUT_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(300);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(VIBRATOR_PIN, OUTPUT);
  digitalWrite(VIBRATOR_PIN, LOW);

  pixels.begin();
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(20, 0, 0));  // red = disconnected
  pixels.show();

  esp_err_t err = dsps_fft2r_init_fc32(NULL, FFT_SIZE);
  if (err != ESP_OK) { Serial.println("FFT init failed"); while(1) delay(10); }

  BLEDevice::init("ESP32C6_EEG");
  pBleServer = BLEDevice::createServer();
  pBleServer->setCallbacks(new MyServerCallbacks());
  pBlinkService = pBleServer->createService(BLINK_SERVICE_UUID);
  pBlinkChar = pBlinkService->createCharacteristic(
    BLINK_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pBlinkChar->addDescriptor(new BLE2902());
  pBlinkService->start();
  pBleServer->getAdvertising()->start();
  Serial.println(">> BLE Advertising started");
}

// --- Loop ---
void loop()
{
  static unsigned long lastMicros = micros();
  unsigned long now = micros(), dt = now - lastMicros;
  lastMicros = now;

  static long timer = 0;
  timer -= dt;
  if (timer <= 0) {
    timer += 1000000L / SAMPLE_RATE;
    int raw        = analogRead(INPUT_PIN);
    float notched  = Notch(raw);
    float eegLow   = EEGFilter(notched);
    float eeg      = highpass(eegLow);
    currentEEGEnvelope = updateEEGEnvelope(eeg);
    currentJawEnvelope = updateJawEnvelope(jawClenchFilter(notched));
    inputBuffer[fftIdx] = eegLow;
    if (++fftIdx >= FFT_SIZE) { processFFT(); fftIdx = 0; }
  }

  unsigned long nowMs = millis();
  bool jawBlockActive = (nowMs - lastJawDetectionTime) < JAW_BLOCK_DURATION_MS;

  handleVibration();

  // jaw clench
  if (!jawState) {
    if (currentJawEnvelope > JAW_ON_THRESHOLD && (nowMs - lastJawClenchTime) >= JAW_DEBOUNCE_MS) {
      jawState             = true;
      lastJawClenchTime    = nowMs;
      lastJawDetectionTime = nowMs;
      Serial.println("JAW CLENCH");
      sendCommand(CMD_JAW_CLENCH);
    }
  } else if (currentJawEnvelope < JAW_OFF_THRESHOLD) {
    jawState          = false;
    lastJawClenchTime = nowMs;
  }

  // blink detection - blocked right after a jaw clench
  if (!jawBlockActive) {
    if (currentEEGEnvelope > BLINK_THRESHOLD && (nowMs - lastBlinkTime) >= BLINK_DEBOUNCE_MS) {
      lastBlinkTime = nowMs;
      if (blinkCount == 0) {
        firstBlinkTime = nowMs;
        blinkCount = 1;
      } else if (blinkCount == 1 && (nowMs - firstBlinkTime) <= DOUBLE_BLINK_MS) {
        secondBlinkTime = nowMs;
        blinkCount = 2;
      } else if (blinkCount == 2 && (nowMs - secondBlinkTime) <= TRIPLE_WINDOW_MS) {
        Serial.println("TRIPLE BLINK");
        sendCommand(CMD_TRIPLE_BLINK);
        blinkCount = 0;
      } else {
        firstBlinkTime = nowMs;
        blinkCount = 1;
      }
    }

    // Confirmed double blink — no 3rd blink arrived in time
    if (blinkCount == 2 && (nowMs - secondBlinkTime) > TRIPLE_WINDOW_MS) {
      Serial.println("DOUBLE BLINK");
      sendCommand(CMD_DOUBLE_BLINK);
      blinkCount = 0;
    }

    // Confirmed single blink — no 2nd blink arrived in time
    if (blinkCount == 1 && (nowMs - firstBlinkTime) > DOUBLE_BLINK_MS) {
      Serial.println("SINGLE BLINK");
      sendCommand(CMD_SINGLE_BLINK);
      blinkCount = 0;
    }
  }

  // focus - send every second while beta is above threshold
  float betaPct = (smoothedPowers.beta / (smoothedPowers.total + EPS)) * 100.0f;
  if (betaPct > FOCUS_BETA_PCT && (nowMs - lastFocusSentTime) >= FOCUS_INTERVAL_MS) {
    lastFocusSentTime = nowMs;
    Serial.print("FOCUS beta%: "); Serial.println(betaPct);
    sendCommand(CMD_FOCUS);
  }
}
