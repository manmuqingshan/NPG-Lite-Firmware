# NPG-Mouse: Neuro Playground Lite Mouse Control

This project turns the Neuro Playground Lite (NPG) into a hands-free mouse controller using a headband. It combines EEG/Jaw clench/EOG blink detection and head movement sensing for intuitive computer control.

## **[How to Use](#how-to-use)**

## What It Does
- **Head Movement → Mouse Movement:**
  - The BMI270 sensor (gyro + accelerometer) is attached to the headband and connected to NPG via the QwiKK port.
  - Moving your head up/down or left/right moves the mouse cursor on your computer.
- **Blink Detection & jaw clench → Mouse Clicks:**
  - NPG reads single-channel EOG data.
  - jaw clench triggers a left mouse click.
  - Triple blinks triggers a right mouse click.

## How It Works
- **Sensors Used:**
  - **BMI270:** Detects head tilt and orientation for cursor movement.
  - **EEG/Jaw clench/EOG Input:** Detects blinks for mouse clicks.
- **Mouse Control:**
  - The code processes head tilt angles and translates them into smooth mouse movements.
  - Sensitivity, deadzone, and acceleration are adjustable for comfort and precision.
- **Blink Detection:**
  - The EOG signal is filtered and analyzed to detect blinks.
- **Calibration:**
  - The headband calibrates itself for neutral position and movement directions using vibration feedback.
- **BLE Connection:**
  - NPG acts as a Bluetooth mouse and keyboard, allowing wireless control.

## How To Use
1. Attach the NPG and BMI270 to a headband.
2. Connect the BMI270 to NPG via the Qwiic port.
3. Download the ESP32 BLE Combo library from [ESP32-BLE-Combo](https://github.com/upsidedownlabs/ESP32-BLE-Combo).
4. Click **`Code` → `Download ZIP`** in the repository.
5. Open Arduino IDE and go to **`Sketch` → `Include Library` → `Add .ZIP Library...`**.
6. Select the downloaded ZIP file and install it.
7. This is required because the sketch uses the `BleCombo.h` library, which is not included with Arduino IDE by default.
8. Compile and upload the sketch to your NPG Lite board.
9. Wear the headband and power on NPG.
10. Calibrate by following the vibration feedback.
11. Move your head to control the mouse cursor.
12. Jaw clench for a left click and blink three times for a right click.

---

**Made by Upside Down Labs.**

Open-source, affordable neuroscience for everyone!
