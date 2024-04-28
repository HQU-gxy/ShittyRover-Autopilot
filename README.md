# ShittyRover-Autopilot

This is a simple autopilot for the ShittyRover project. It has an IMU and magnetometer for heading and tilt information.  
It uses a simple PID controller to control the steering and throttle of the rover,
which is [this repo](https://github.com/redstonee/ShittyRoverIO).  
There's also an RK3588 board for AI functions, but it's not implemented yet.

## Hardware
* FMU MCU: STM32G431CBU6
* IO MCU: STM32G030C8T6 (for motor control)
* IMU: BMI088
* Magnetometer: QMC5883L
* Power: 6S LiPo

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
