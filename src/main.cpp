
#include <Arduino.h>
#include <TFT_eSPI.h>
#include <BMI088.h>
#include <STM32FreeRTOS.h>
#include <CMSIS_DSP.h>

#include "Logger.hpp"
#include "SensorCollector.hpp"
#include "Magnetometer.hpp"
#include "IMU.hpp"
#include "config.h"

TFT_eSPI tft;

void app_main(void *)
{
  while (1)
  {
    static uint8_t cursor = 0;

    auto heading = Magneto::getHeading();
    // Logger::debug(("Heading: " + String(heading)));

    IMU::IMUData imuData;
    IMU::getData(&imuData);

    // Show the IMU data on the TFT
    tft.drawPixel(cursor, 40 + imuData.accelX * 2, TFT_RED);
    tft.drawPixel(cursor, 40 + imuData.accelY * 2, TFT_GREEN);
    tft.drawPixel(cursor, 40 + imuData.accelZ * 2, TFT_CYAN);
    cursor = cursor < 80 ? cursor + 1 : 0;
    tft.drawFastVLine(cursor, 0, 80, TFT_BLACK);

    // Show the heading on the TFT
    tft.fillCircle(120, 40, 20, TFT_BLACK);
    tft.drawCircle(120, 40, 20, TFT_ORANGE);
    tft.drawLine(120, 40, 120 + 20 * arm_cos_f32(heading * DEG_TO_RAD), 40 - 20 * arm_sin_f32(heading * DEG_TO_RAD), TFT_WHITE);
    vTaskDelay(50 / portTICK_PERIOD_MS);

    if (Serial2.available())
      if (Serial2.read() == 'c')
      {
        tft.fillScreen(TFT_BLACK);
        tft.setCursor(0, 0);
        tft.println("Calibrating the compass");
        tft.println("Please rotate the device in all directions");
        Magneto::runCalibration(&tft);
        tft.println("Calibration done");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
      }
  }
}

void setup(void)
{
  Logger::begin();

  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);

  Logger::info("I'm fucked up!");
  Logger::info("Checking the peripherals: ");

  tft.print("Checking the peripherals: ");

  if (!IMU::begin())
  {
    Logger::error("IMU Initialization Error");
    tft.printf("IMU Initialization Error");
    while (1)
      ;
  }

  if (!Magneto::begin())
  {
    Logger::error("Compass Initialization Error");
    tft.printf("Compass Initialization Error");
    while (1)
      ;
  }

  pinMode(PB4, OUTPUT);
  pinMode(PB5, OUTPUT);
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0, 0);

  SensorCollector::begin();
  xTaskCreate(app_main, "Main App", 1024, nullptr, 6, nullptr);
  vTaskStartScheduler();
}

void loop() {}