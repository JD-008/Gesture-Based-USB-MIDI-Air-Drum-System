# Gesture-Based USB MIDI Air Drum System

A real-time air drumming system that turns hand gestures into MIDI drum hits using a Seeed XIAO nRF52840 Sense and its onboard LSM6DS3 IMU. The firmware estimates pitch, roll, and yaw from accelerometer and gyroscope data, fuses the yaw estimate with magnetometer data to reduce drift, and sends MIDI note messages over USB.

This project is designed for gesture-based percussion control, where a stick or hand motion is interpreted as a drum hit based on acceleration magnitude and orientation.

## Features

- USB MIDI output for drum triggering
- XIAO nRF52840 Sense control board
- IMU-based gesture sensing using accelerometer and gyroscope
- Magnetometer-assisted yaw correction to reduce heading drift
- Kalman filtering for stable pitch and roll estimation
- Real-time hit detection with note velocity proportional to impact magnitude
- 5-note drum mapping based on orientation

## Hardware

- Seeed Studio XIAO nRF52840 Sense
- LSM6DS3 IMU (integrated on the board)
- TLV493D magnetometer for yaw stabilization
- USB cable for power and MIDI USB communication
- Optional: drumstick, controller housing, or custom enclosure

## MIDI note mapping

The system maps the detected orientation to MIDI drum notes:

- Yaw < -54° → Kick (MIDI note 36)
- Yaw < -18° → Snare (MIDI note 38)
- Yaw < 18° → Hi-Hat (MIDI note 42)
- Yaw < 54° → Tom (MIDI note 45)
- Otherwise → Crash (MIDI note 49)

## How it works

The firmware performs the following steps in real time:

1. Reads IMU acceleration and gyroscope data.
2. Uses a two-state Kalman filter to estimate pitch and roll.
3. Computes a tilt-compensated magnetic heading from the magnetometer.
4. Fuses gyro yaw with the magnetometer heading using a complementary filter.
5. Detects a hit when acceleration exceeds a set threshold.
6. Converts the gesture orientation into a MIDI note and sends it over USB.

This prevents the heading from drifting over time while still keeping motion responsive.

## Repository contents

- `code.ino` — main Arduino firmware for the air drum controller
- `README.md` — project overview and usage notes
- `ES_Special.docx` — project documentation and related material

## Setup

### 1. Install Arduino IDE

Install the latest Arduino IDE and configure the XIAO nRF52840 board package.

### 2. Install required libraries

From the Arduino Library Manager, install:

- `LSM6DS3`
- `MIDI Library`
- `Adafruit TinyUSB Library`
- `TLV493D` / magnetometer support library

### 3. Open the firmware

Open `code.ino` in the Arduino IDE and upload it to the XIAO nRF52840 Sense.

### 4. Calibrate the device

On boot, the system performs gyro bias calibration while the device is stationary. It then prompts for a magnetometer calibration routine where the stick is rotated through multiple orientations to estimate hard-iron offsets.

## Usage

- Hold the controller still during startup calibration.
- Rotate the device through multiple orientations during magnetometer calibration.
- Strike or move the device with sufficient acceleration to trigger a note.
- The MIDI signal can be received by DAWs, virtual MIDI apps, or MIDI routing software over USB.

## Notes

This project is a strong example of embedded sensor fusion, MIDI generation, and real-time gesture recognition using low-power microcontroller hardware.

## License

This project is provided as-is for educational and experimental use. Please check the repository for any additional licensing or attribution requirements.
