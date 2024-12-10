
#include <Arduino.h>
#include <TFT_eSPI.h>
#include <BMI088.h>
#include <STM32FreeRTOS.h>
#include <CMSIS_DSP.h>
#include <ulog.h>

#include "SensorCollector.h"
#include "Magnetometer.h"
#include "IMU.h"
#include <Motor.h>
#include "config.h"

TFT_eSPI tft;
void app_main(void *)
{

  bool button1Pressed = false;
  attachInterrupt(BUTTON1_PIN, [&button1Pressed]()
                  { button1Pressed = true; }, FALLING);

  Motor leftMotor(MOTOR1_IN1_PIN, MOTOR1_IN2_PIN, MOTOR1_EN_PIN, MOTOR1_ENC_A_PIN, MOTOR1_ENC_B_PIN, MOTOR1_ENC_TIMER);
  Motor rightMotor(MOTOR2_IN1_PIN, MOTOR2_IN2_PIN, MOTOR2_EN_PIN, MOTOR2_ENC_A_PIN, MOTOR2_ENC_B_PIN, MOTOR2_ENC_TIMER);

  while (1)
  {
    static uint8_t cursor = 0;

    auto heading = Magneto::getHeading();

    IMU::IMUData imuData;
    IMU::getData(&imuData);

    auto leftEnc = LL_TIM_GetCounter(MOTOR1_ENC_TIMER);
    auto rightEnc = LL_TIM_GetCounter(MOTOR2_ENC_TIMER);
    tft.setCursor(0, 0);
    tft.print("Left: ");
    tft.println(leftEnc);
    tft.print("Right: ");
    tft.println(rightEnc);
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
  ulog_subscribe([](ulog_level_t severity, char *msg)
                 { Serial2.printf("%d [%s]: %s\n", millis(), ulog_level_name(severity), msg); }, ULOG_DEBUG_LEVEL);

  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);

  ULOG_INFO("I'm fucked up!");
  ULOG_INFO("Checking the peripherals: ");

  tft.print("Checking the peripherals: ");

  // if (!IMU::begin())
  // {
  //   ULOG_ERROR("IMU Initialization Error");
  //   tft.printf("IMU Initialization Error");
  //   while (1)
  //     ;
  // }

  if (!Magneto::begin())
  {
    ULOG_ERROR("Compass Initialization Error");
    tft.printf("Compass Initialization Error");
    while (1)
      ;
  }

  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0, 0);

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