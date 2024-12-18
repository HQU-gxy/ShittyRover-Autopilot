# NooBot-Driver

This is a simple autopilot for the ShittyRover project. It has an IMU and magnetometer for heading and tilt information.  
It uses a simple PID controller to control the steering and throttle of the rover,
There's also an RK3588 board running ROS2 for AI functions, but it's not implemented yet.

## Hardware
* MCU: STM32G431BT6
* IMU: BMI088
* Magnetometer: QMC5883L
* Power: 6S LiPo
* Motors: 2x brushed DC motors with quadrature encoders
* Screen: ST7735 160x80 TFT
* A few buttons and LEDs

## Build the firmware

This is a PlatformIO project, so you need to install PlatformIO first.  
Pin configuration (except for the screen) is in [include/config.h](include/config.h),
the screen is configured directly in [platformio.ini](platformio.ini)

0. Clone this repo:
```bash
git clone https://github.com/HQU-gxy/ShittyRover-Autopilot/ --recursive --depth=1
```
1. Open the project in PlatformIO
2. Set the `debug_tool` and `upload_protocol` in [platformio.ini](platformio.ini) to fit your debugger
3. Build and upload the project
