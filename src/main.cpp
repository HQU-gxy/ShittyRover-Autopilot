
#include <Arduino.h>
#include <TFT_eSPI.h>
#include <BMI088.h>
#include <STM32FreeRTOS.h>
#include <CMSIS_DSP.h>
#include <ulog.h>
#include <utility>

#include "SensorCollector.h"
#include "Magnetometer.h"
#include "IMU.h"
#include "Motor.h"
#include "UpLink.h"
#include "config.h"

/**
 * @brief Convert the linear and angular speed of the car to the speed of the left and right motors
 *
 * @param linear Linear speed of the car in m/s
 * @param angular Angular speed of the car in rad/s
 *
 * @return A pair of left and right motor speed in m/s
 */
std::pair<float, float> carSpeedToMotorSpeed(float linear, float angular)
{
  float leftSpeed = linear - angular * WHEEL_DISTANCE / 2;
  float rightSpeed = linear + angular * WHEEL_DISTANCE / 2;

  return {leftSpeed, rightSpeed};
}

/**
 * @brief Convert the speed of the left and right motors to the linear and angular speed of the car
 *
 * @param left Speed of the left motor in m/s
 * @param right Speed of the right motor in m/s
 *
 * @return A pair of linear and angular speed of the car in m/s and rad/s
 */
std::pair<float, float> motorSpeedToCarSpeed(float left, float right)
{
  float linear = (left + right) / 2;
  float angular = (right - left) / WHEEL_DISTANCE;

  return {linear, angular};
}

TFT_eSPI tft;
void app_main(void *)
{

  bool button1Pressed = false;
  attachInterrupt(BUTTON1_PIN, [&button1Pressed]()
                  { button1Pressed = true; }, FALLING);

  Motor leftMotor(MOTOR1_IN1_PIN, MOTOR1_IN2_PIN, MOTOR1_EN_PIN, MOTOR1_ENC_CFG);
  Motor rightMotor(MOTOR2_IN1_PIN, MOTOR2_IN2_PIN, MOTOR2_EN_PIN, MOTOR2_ENC_CFG);

#ifndef PID_TUNING
  std::pair<float, float> upLinkTargetSpeed = {0, 0}; // Linear and angular speed from up-link in m/s and rad/s

  auto getStatus = [&leftMotor, &rightMotor]()
  {
    IMU::IMUData imuData;
    IMU::getData(&imuData);
    auto [linear, angular] = motorSpeedToCarSpeed(leftMotor.getSpeed(), rightMotor.getSpeed());
    return {linear, angular, imuData};
  };

  auto onCommand = [&upLinkTargetSpeed](float linear, float angular)
  {
    upLinkTargetSpeed = {linear, angular};
  };

  UpLink::setGetStatusFunc(getStatus);
  UpLink::setOnCmdCallback(onCommand);
  UpLink::begin();

#endif // !PID_TUNING

  auto ledTimer = xTimerCreate("Alive LED", pdMS_TO_TICKS(1000), pdTRUE, nullptr, [](TimerHandle_t)
                               { digitalToggle(LED1_PIN); });
  xTimerStart(ledTimer, 0);

  while (1)
  {

#ifdef PID_TUNING
    struct __attribute__((packed)) PIDConfig
    {
      uint8_t header;
      float kp;
      float ki;
      float kd;
      char shit[4];
    };

    struct __attribute__((packed)) SpeedSet
    {
      uint8_t header;
      float speed;
      char shit[4];
    };

    static String inputStr;
    if (Serial2.available())
    {
      char c = Serial2.read();
      inputStr += c;
      if (inputStr.endsWith("shit"))
      {
        if (inputStr[0] == 0x96)
        {
          if (inputStr.length() != sizeof(PIDConfig))
            continue;
          auto parsed = reinterpret_cast<const PIDConfig *>(inputStr.c_str());
          rightMotor.setPID(parsed->kp, parsed->ki, parsed->kd);
        }
        else if (inputStr[0] == 0x97)
        {
          if (inputStr.length() != sizeof(SpeedSet))
            continue;
          auto parsed = reinterpret_cast<const SpeedSet *>(inputStr.c_str());
          rightMotor.setSpeed(parsed->speed);
        }
        inputStr = "";
      }
    }
#else

    auto heading = Magneto::getHeading();

    IMU::IMUData imuData;
    IMU::getData(&imuData);

    auto leftSpeed = leftMotor.getSpeed();
    auto rightSpeed = rightMotor.getSpeed();

    tft.fillScreen(TFT_BLACK);
    tft.setCursor(0, 0);
    tft.print("Left: ");
    tft.println(leftSpeed);
    tft.print("Right: ");
    tft.println(rightSpeed);

#endif // PID_TUNING

    vTaskDelay(100 / portTICK_PERIOD_MS);

    // // Show the IMU data on the TFT
    // tft.drawPixel(cursor, 40 + imuData.accelX * 2, TFT_RED);
    // tft.drawPixel(cursor, 40 + imuData.accelY * 2, TFT_GREEN);
    // tft.drawPixel(cursor, 40 + imuData.accelZ * 2, TFT_CYAN);
    // cursor = cursor < 80 ? cursor + 1 : 0;
    // tft.drawFastVLine(cursor, 0, 80, TFT_BLACK);

    // Show the heading on the TFT
    // tft.fillCircle(120, 40, 20, TFT_BLACK);
    // tft.drawCircle(120, 40, 20, TFT_ORANGE);
    // tft.drawLine(120, 40, 120 + 20 * arm_cos_f32(heading * DEG_TO_RAD), 40 - 20 * arm_sin_f32(heading * DEG_TO_RAD), TFT_WHITE);
    // vTaskDelay(50 / portTICK_PERIOD_MS);

    // if (button1Pressed || (Serial2.available() && Serial2.read() == 'c'))
    // {
    //   tft.fillScreen(TFT_BLACK);
    //   tft.setCursor(0, 0);
    //   tft.println("Calibrating the compass");
    //   tft.println("Please rotate the device in all directions");
    //   Magneto::runCalibration(&tft);
    //   tft.println("Calibration done");
    //   vTaskDelay(1000 / portTICK_PERIOD_MS);
    //   button1Pressed = false;
    // }
  }
}

void setup(void)
{
  Serial2.setTx(UART2_TX_PIN);
  Serial2.setRx(UART2_RX_PIN);
  Serial2.begin(115200);

  ulog_init();
#ifdef PID_TUNING
  ulog_subscribe([](ulog_level_t severity, char *msg)
                 { Serial2.printf("%d [%s]: %s\n", millis(), ulog_level_name(severity), msg); }, ULOG_ERROR_LEVEL);
#else
  ulog_subscribe([](ulog_level_t severity, char *msg)
                 { Serial2.printf("%d [%s]: %s\n", millis(), ulog_level_name(severity), msg); }, ULOG_DEBUG_LEVEL);
#endif // PID_TUNING

  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_PINK);

  ULOG_INFO("I'm fucked up!");
  ULOG_INFO("Checking the peripherals: ");

  tft.println("Checking the peripherals: ");

  if (!IMU::begin())
  {
    ULOG_ERROR("IMU Initialization Error");
    tft.printf("IMU Initialization Error");
    while (1)
      ;
  }
  tft.println("IMU Initialized");

  if (!Magneto::begin())
  {
    ULOG_ERROR("Compass Initialization Error");
    tft.printf("Compass Initialization Error");
    while (1)
      ;
  }
  tft.println("Compass Initialized");
  tft.fillScreen(TFT_CYAN);

  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(BUTTON1_PIN, INPUT);
  pinMode(BUTTON2_PIN, INPUT);
  pinMode(BUTTON3_PIN, INPUT);

  SensorCollector::begin();
  xTaskCreate(app_main, "Main App", 1024, nullptr, 6, nullptr);
  vTaskStartScheduler();
}

void loop() {}