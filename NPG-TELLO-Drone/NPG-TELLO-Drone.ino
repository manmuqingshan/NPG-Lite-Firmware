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
const char *tello_ssid = "TELLO-95780E"; // Replace with your Tello WiFi name
const char *tello_pass = "";             // Tello has no password

WiFiUDP udp;
const char *TELLO_IP = "192.168.10.1";
const int TELLO_PORT = 8889;

// Variables will change:
uint32_t buttonState;      // the current reading from the input pin
int lastButtonState = LOW; // the previous reading from the input pin
uint32_t bci_val = 0;      // EEG-based control value (0=stop, 3=forward)
uint32_t emg1_val1 = 0;    // Left EMG control value (0=inactive, 1=left turn)
uint32_t emg2_val2 = 0;    // Right EMG control value (0=inactive, 2=right turn)
uint32_t bootback_val = 4; // Button toggle value (4=toggle state)

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastCmdTime = 0;
const unsigned long cmdCooldown = 200; // 0.2 sec between commands


void sendCommandSafe(const String &cmd)
{
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
bool deviceConnected = false;
bool oldDeviceConnected = false;
Adafruit_NeoPixel pixel(6, 15, NEO_GRB + NEO_KHZ800);

// Variable that will continuously be increased and written to the client
uint32_t value = 0;
uint32_t betaThreshold = 4;

// ----------------- USER CONFIGURATION -----------------
#define SAMPLE_RATE 512 // samples per second
#define FFT_SIZE 512    // must be a power of two
#define BAUD_RATE 115200
#define INPUT_PIN1 A0 // EEG input pin
#define INPUT_PIN2 A1 // Left hand EMG input pin
#define INPUT_PIN3 A2 // Right hand EMG input pin

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

// ----------------- USER CONFIGURATION -----------------
// Add this after existing defines:
#define ENVELOPE_WINDOW_MS 100  // Smoothing window in milliseconds
#define ENVELOPE_WINDOW_SIZE ((ENVELOPE_WINDOW_MS * SAMPLE_RATE) / 1000)

const unsigned long BLINK_DEBOUNCE_MS   = 250;   // minimal spacing between individual blinks
const unsigned long DOUBLE_BLINK_MS     = 800;   // max time between the two blinks
const unsigned long TRIPLE_BLINK_MS     = 1000;  // max time between all three blinks

unsigned long lastBlinkTime     = 0;             // time of most recent blink
unsigned long firstBlinkTime    = 0;             // time of the first blink in a pair
int         blinkCount         = 0;             // how many valid blinks so far (0–2)
bool        menu           = LOW;            // current menu state (LOW = vertical movement, HIGH = rotation)

float BlinkThreshold = 50.0;

// Add these after existing buffers:
float envelopeBuffer[ENVELOPE_WINDOW_SIZE] = {0};
int envelopeIndex = 0;
float envelopeSum = 0;
float currentEEGEnvelope = 0;

// ----------------- BUFFERS & TYPES -----------------
float inputBuffer[FFT_SIZE];
float powerSpectrum[FFT_SIZE / 2];

// For two-real FFT trick
__attribute__((aligned(16))) float y_cf[FFT_SIZE * 2];
float *y1_cf = &y_cf[0];

BandpowerResults smoothedPowers = {0, 0, 0, 0, 0, 0};

// ----------------- NOTCH FILTER CLASSES -----------------
// For 50Hz AC noise removal
// Band-Stop Butterworth IIR digital filter, generated using filter_gen.py.
// Sampling rate: 500.0 Hz, frequency: [48.0, 52.0] Hz.
// Filter is order 2, implemented as second-order sections (biquads).
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
class NotchFilter
{
private:
    struct BiquadState
    {
        float z1 = 0;
        float z2 = 0;
    };

    BiquadState state1;
    BiquadState state2;

public:
    float process(float input)
    {
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

    void reset()
    {
        state1.z1 = state1.z2 = 0;
        state2.z1 = state2.z2 = 0;
    }
};

// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
float highpass(float input)
{
  float output = input;
  {
    static float z1, z2; // filter section state
    float x = output - -1.91327599*z1 - 0.91688335*z2;
    output = 0.95753983*x + -1.91507967*z1 + 0.95753983*z2;
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
class EMGHighPassFilter
{
private:
    // Filter state for a single channel
    double z1 = 0.0;
    double z2 = 0.0;

public:
    // Process a single sample
    double process(double input)
    {
        const double x = input - -0.82523238 * z1 - 0.29463653 * z2;
        const double output = 0.52996723 * x + -1.05993445 * z1 + 0.52996723 * z2;

        // Update state
        z2 = z1;
        z1 = x;

        return output;
    }

    // Reset filter state
    void reset()
    {
        z1 = 0.0;
        z2 = 0.0;
    }
};

// Class to calculate EMG Envelope
class EnvelopeFilter
{
private:
    std::vector<double> circularBuffer;
    double sum = 0.0;
    int dataIndex = 0;
    const int bufferSize;

public:
    EnvelopeFilter(int bufferSize)
        : bufferSize(bufferSize)
    {
        circularBuffer.resize(bufferSize, 0.0);
    }

    double getEnvelope(double absEmg)
    {
        sum -= circularBuffer[dataIndex];
        sum += absEmg;
        circularBuffer[dataIndex] = absEmg;
        dataIndex = (dataIndex + 1) % bufferSize;
        return (sum / bufferSize);
    }
};

// NEW: Low-pass filter for EEG signals
float EEGFilter(float input)
{
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

// ----------------- ENVELOPE FUNCTION -----------------
float updateEEGEnvelope(float sample)
{
    float absSample = fabs(sample); // Rectify EEG signal

    // Update circular buffer and running sum
    envelopeSum -= envelopeBuffer[envelopeIndex];
    envelopeSum += absSample;
    envelopeBuffer[envelopeIndex] = absSample;
    envelopeIndex = (envelopeIndex + 1) % ENVELOPE_WINDOW_SIZE;

    return envelopeSum / ENVELOPE_WINDOW_SIZE; // Return moving average
}

NotchFilter filters[3];             // Notch filters for all 3 input channels
EMGHighPassFilter emgfilters[3];    // Fixed: Changed from 2 to 3 elements for all channels
EnvelopeFilter Envelopefilter1(16); // Envelope detector for left EMG
EnvelopeFilter Envelopefilter2(16); // Envelope detector for right EMG

// ----------------- BANDPOWER & SMOOTHING -----------------
BandpowerResults calculateBandpower(float *ps, float binRes, int halfSize)
{
    BandpowerResults r = {0, 0, 0, 0, 0, 0};
    for (int i = 1; i < halfSize; i++)
    {
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

void smoothBandpower(const BandpowerResults *raw, BandpowerResults *s)
{
    s->delta = SMOOTHING_FACTOR * raw->delta + (1 - SMOOTHING_FACTOR) * s->delta;
    s->theta = SMOOTHING_FACTOR * raw->theta + (1 - SMOOTHING_FACTOR) * s->theta;
    s->alpha = SMOOTHING_FACTOR * raw->alpha + (1 - SMOOTHING_FACTOR) * s->alpha;
    s->beta = SMOOTHING_FACTOR * raw->beta + (1 - SMOOTHING_FACTOR) * s->beta;
    s->gamma = SMOOTHING_FACTOR * raw->gamma + (1 - SMOOTHING_FACTOR) * s->gamma;
    s->total = SMOOTHING_FACTOR * raw->total + (1 - SMOOTHING_FACTOR) * s->total;
}

// ----------------- DSP FFT SETUP -----------------
void initFFT()
{
    // initialize esp-dsp real-FFT (two-real trick)
    esp_err_t err = dsps_fft2r_init_fc32(NULL, FFT_SIZE);
    if (err != ESP_OK)
    {
        Serial.println("FFT init failed");
        while (1)
            delay(10);
    }
}

// ----------------- FFT + BANDPOWER + PEAK -----------------
void processFFT()
{
    // pack real→complex: real=inputBuffer, imag=0
    for (int i = 0; i < FFT_SIZE; i++)
    {
        y_cf[2 * i] = inputBuffer[i];
        y_cf[2 * i + 1] = 0.0f;
    }

    // FFT
    dsps_fft2r_fc32(y_cf, FFT_SIZE);
    dsps_bit_rev_fc32(y_cf, FFT_SIZE);
    dsps_cplx2reC_fc32(y_cf, FFT_SIZE);

    // magnitude² spectrum
    int half = FFT_SIZE / 2;
    for (int i = 0; i < half; i++)
    {
        float re = y1_cf[2 * i];
        float im = y1_cf[2 * i + 1];
        powerSpectrum[i] = re * re + im * im;
    }

    // detect peak bin (skip i=0)
    int maxIdx = 1;
    float maxP = powerSpectrum[1];
    for (int i = 2; i < half; i++)
    {
        if (powerSpectrum[i] > maxP)
        {
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
    Serial.println((smoothedPowers.beta / T) * 100); // for debugging purpose only

    // If the power exceeds the threshold (set as 2% of the total power), the threshold value can be adjusted based on your beta parameters.
    if (((smoothedPowers.beta / T) * 100) > betaThreshold)
    {
        sendCommandSafe("takeoff");
    }
}

void setup()
{
    // drone
    Serial.begin(115200);

    delay(500);

    Serial.println("Connecting to Tello WiFi...");
    WiFi.begin(tello_ssid, tello_pass);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(200);
        Serial.print(".");
    }
    Serial.println("\nConnected to Tello!");

    udp.begin(8889);

    // Enter SDK mode
    sendCommandSafe("command");
    delay(1000);
    
    // ----- Initialize Neopixel LED -----
    pixel.begin();
    // Set the Neopixel to red (indicating device turned on)
    pixel.setPixelColor(0, pixel.Color(0, 0, 0));
    pixel.setPixelColor(2, pixel.Color(0, 0, 0));
    pixel.setPixelColor(5, pixel.Color(0, 0, 0));
    pixel.show();
    
    pinMode(INPUT_PIN1, INPUT);
    pinMode(INPUT_PIN2, INPUT);
    pinMode(INPUT_PIN3, INPUT);

    initFFT();
}

void loop()
{
    static uint16_t idx = 0;
    static unsigned long lastMicros = micros();
    unsigned long now = micros(), dt = now - lastMicros;
    lastMicros = now;
    
    // Update NeoPixel based on connection status
    if (deviceConnected)
    {
        pixel.setPixelColor(5, pixel.Color(0, 255, 0)); // Green when connected
    }
    else
    {
        pixel.setPixelColor(5, pixel.Color(255, 0, 0)); // Red when disconnected
    }
    pixel.show();

    static long timer = 0;
    timer -= dt;
    if (timer <= 0)
    {
        timer += 1000000L / SAMPLE_RATE;
        int raw1 = analogRead(INPUT_PIN1);
        int raw2 = analogRead(INPUT_PIN2);
        int raw3 = analogRead(INPUT_PIN3);

        // Fixed: Use existing filter objects instead of creating new ones
        float notchFiltered1 = filters[0].process(raw1);
        float notchFiltered2 = filters[1].process(raw2);
        float notchFiltered3 = filters[2].process(raw3);
        
        float filt = EEGFilter(notchFiltered1);
        float filtered = highpass(filt);
        currentEEGEnvelope = updateEEGEnvelope(filtered);

        float filteeg = EEGFilter(notchFiltered1);
        float filtemg2 = emgfilters[1].process(notchFiltered2);
        float filtemg3 = emgfilters[2].process(notchFiltered3);
        inputBuffer[idx++] = filteeg;

        float env1 = Envelopefilter1.getEnvelope(abs(filtemg2));
        float env2 = Envelopefilter2.getEnvelope(abs(filtemg3));

        // Normal running state processing
        unsigned long nowMs = millis();

        // ========== BLINK DETECTION LOGIC ==========
        // 1) Did we cross threshold and respect per‑blink debounce?
        if (currentEEGEnvelope > BlinkThreshold && (nowMs - lastBlinkTime) >= BLINK_DEBOUNCE_MS)
        {
            lastBlinkTime = nowMs; // mark this blink
            
            if (blinkCount == 0)
            {
                // first blink of the sequence
                firstBlinkTime = nowMs;
                blinkCount = 1;
                Serial.println("First blink detected");
                pixel.setPixelColor(2, pixel.Color(0, 0, 0));
            }
            else if (blinkCount == 1 && (nowMs - firstBlinkTime) <= DOUBLE_BLINK_MS)
            {
                // second blink detected within double blink time window
                blinkCount = 2;
                Serial.println("Second blink - double blink detected");
            }
            else if (blinkCount == 2 && (nowMs - firstBlinkTime) <= TRIPLE_BLINK_MS)
            {
                // third blink detected within triple blink time window
                Serial.println("Third blink - TRIPLE BLINK DETECTED!");
                
                // Handle triple blink action
                menu = !menu;  // Toggle the menu state
                if (menu)
                {
                    Serial.println("Menu ON - EMG controls set to rotation");
                    pixel.setPixelColor(0, pixel.Color(255, 255, 0)); // Yellow indicating running

                }
                else
                {
                    Serial.println("Menu OFF - EMG controls set to vertical movement");
                    pixel.setPixelColor(0, pixel.Color(255, 0, 0)); // RED indicating running

                }
                
                blinkCount = 0; // Reset for next sequence
            }
            else
            {
                // Too slow or extra blink - restart sequence
                firstBlinkTime = nowMs;
                blinkCount = 1;
                Serial.println("Sequence timeout - restarting");
            }
        }

        // 2) Handle double blink timeout (when second blink wasn't followed by third)
        if (blinkCount == 2 && (nowMs - firstBlinkTime) > TRIPLE_BLINK_MS)
        {
            // This is a confirmed DOUBLE BLINK
            Serial.println("DOUBLE BLINK CONFIRMED - sending forward/backward commands");
            sendCommandSafe("flip r");
            sendCommandSafe("flip r");
            sendCommandSafe("flip r");
            sendCommandSafe("flip r");
            pixel.setPixelColor(2, pixel.Color(0, 0, 255)); // Blue color 
            
            blinkCount = 0; // Reset for next sequence
            // pixel.setPixelColor(4, pixel.Color(0, 0, 0));
        }

        // 3) Handle single blink timeout (when no second blink arrives)
        if (blinkCount == 1 && (nowMs - firstBlinkTime) > DOUBLE_BLINK_MS)
        {
            // This was just a single blink, not part of a sequence
            Serial.println("Single blink - no action");
            blinkCount = 0; // Reset for next sequence
        }

        // ========== EMG CONTROLS ==========
        // These work continuously based on the current menu state
        // (triggered by triple blink to toggle between modes)
        if (env1 > 150)
        {
            if (menu) // Menu ON (rotation mode)
            {
                sendCommandSafe("forward 50"); // Counter-clockwise rotation
                Serial.println("Menu ON - EMG1 > 150 - Sent: forward 50");
            }
            else // Menu OFF (vertical movement mode)
            {
                sendCommandSafe("up 50"); // Move up
                Serial.println("Menu OFF - EMG1 > 150 - Sent: up 50");
            }
        }
        else if (env2 > 150)
        {
            if (menu) // Menu ON (rotation mode)
            {
                sendCommandSafe("cw 50"); // Clockwise rotation
                Serial.println("Menu ON - EMG2 > 150 - Sent: cw 50");
            }
            else // Menu OFF (vertical movement mode)
            {
                sendCommandSafe("land"); // Move down
                Serial.println("Menu OFF - EMG2 > 150 - Sent: land");
            }
        }
        // ========== END EMG CONTROLS ==========

        if (idx >= FFT_SIZE)
        {
            processFFT();
            idx = 0;
        }

        // Disconnecting
        if (!deviceConnected && oldDeviceConnected)
        {
            delay(500); // give the bluetooth stack the chance to get things ready
            Serial.println("start advertising");
            oldDeviceConnected = deviceConnected;
        }
        // Connecting
        if (deviceConnected && !oldDeviceConnected)
        {
            // do stuff here on connecting
            oldDeviceConnected = deviceConnected;
        }
    }
}