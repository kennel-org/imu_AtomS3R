# M5AtomS3R Simple Compass

A simple compass application for the M5AtomS3R device using its built-in 9DoF IMU (BMI270 + BMM150).

## Warning
Do not use this unit in situations involving safety to life.

## Features
- Displays a compass with heading in degrees
- Shows cardinal directions (N, E, S, W)
- Supports calibration via button press
- Uses the M5AtomS3R's built-in 9DoF IMU

## Requirements
- M5AtomS3R device
- PlatformIO development environment

## Installation
1. Clone this repository
2. Open the project in PlatformIO
3. Build and upload to your M5AtomS3R device

## Usage
1. Power on your M5AtomS3R
2. The compass will display automatically
3. To calibrate the magnetometer:
   - Press the button on the M5AtomS3R
   - Wait for the 3-second countdown
   - Rotate the device in all directions for best calibration

## Development Notes
- This project is for PlatformIO only (not Arduino IDE)
- Uses M5Unified and M5GFX libraries
- Requires the custom BMI270 and BMM150 libraries included in this repo

## References
- https://github.com/boschsensortec/BMI270_SensorDriver
- https://github.com/boschsensortec/BMM150_SensorDriver
- https://github.com/m5stack/M5Unified
- https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/

## License
- MIT License
- Components have their own licenses:
  - M5Unified - MIT by M5Stack
  - BMI270 - BSD-3-Clause license by BoschSensortec
  - BMM150 - BSD-3-Clause license by BoschSensortec