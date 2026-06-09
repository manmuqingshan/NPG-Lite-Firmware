# NPG-Mouse: Neuro Playground Lite Mouse Control

This project turns the Neuro Playground Lite (NPG) into a hands-free mouse controller using a headband. It combines EEG/EOG blink detection and head movement sensing for intuitive computer control.

## **[How to Use](#how-to-use)**

## What It Does
- **Head Movement → Mouse Movement:**
  - The MPU6050 sensor (gyro + accelerometer) is attached to the headband and connected to NPG via the Qwiic port.
  - Moving your head up/down or left/right moves the mouse cursor on your computer.
- **Blink Detection → Mouse Clicks:**
  - NPG reads single-channel EOG data.
  - Double blinks trigger a left mouse click.
  - Triple blinks trigger a right mouse click.

## How It Works
- **Sensors Used:**
  - **MPU6050:** Detects head tilt and orientation for cursor movement.
  - **EEG/EOG Input:** Detects blinks for mouse clicks.
- **Mouse Control:**
  - The code processes head tilt angles and translates them into smooth mouse movements.
  - Sensitivity, deadzone, and acceleration are adjustable for comfort and precision.
- **Blink Detection:**
  - The EOG signal is filtered and analyzed to detect blinks.
  - Timing logic distinguishes between double and triple blinks for different mouse clicks.
- **Calibration:**
  - The headband calibrates itself for neutral position and movement directions using vibration feedback.
- **BLE Connection:**
  - NPG acts as a Bluetooth mouse and keyboard, allowing wireless control.

## How To Use
1. Attach the NPG and MPU6050 to a headband.
2. Connect the MPU6050 to NPG via the Qwiic port.
3. Download the ESP32 BLE Combo library as a `.zip` file from [ESP32-BLE-Combo](https://github.com/upsidedownlabs/ESP32-BLE-Combo).
4. When you click the link, you will be taken to the GitHub repository of ESP32-BLE-Combo. You will see a green button which says `<> Code`. Press that button. A dropdown window appears. At the bottom of that window, there is a button which says `Download ZIP`. Press that button. Now all the files in that repository are downloaded as a `.zip` file.
5. Open Arduino IDE. Now at the top there is a horizontal bar. Click on **`Sketch` → `Include Library` → `Add .ZIP Library...`**.
6. Select the downloaded ZIP file and install it.
7. This is required because the sketch uses the `BleCombo.h` library, which is not included with Arduino IDE by default.
8. Compile and upload the sketch to your NPG Lite board.
9. Wear the headband and power on NPG.
10. Calibrate by following the vibration feedback.
11. Move your head to control the mouse cursor.
12. Blink twice for a left click and three times for a right click.

---

**Made by Upside Down Labs.**

Open-source, affordable neuroscience for everyone!
