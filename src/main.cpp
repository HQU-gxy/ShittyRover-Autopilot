
#include <Arduino.h>
#include <TFT_eSPI.h> // Hardware-specific library

#include "Magnetometer.hpp"
#include "config.h"
#include <USBSerial.h>

TFT_eSPI tft = TFT_eSPI(); // Invoke custom library

Bmi088Accel accel(SPI, BMI088_ACCEL_CS);
Bmi088Gyro gyro(SPI, BMI088_GYRO_CS);

void setup(void) {
    SPI.begin();

    tft.init();
    tft.setRotation(3);
    tft.fillScreen(TFT_BLACK);

    Serial2.setTx(UART2_TX);
    Serial2.setRx(UART2_RX);
    Serial2.begin(115200);
    Serial2.println("I'm fucked up!");

    tft.print("Checking the peripherals: ");
    auto status = accel.begin();
    if (status < 0) {
        Serial2.println("Accel Initialization Error");
        Serial2.println(status);
        tft.printf("Accel Initialization Error: %d\n", status);
        while (1)
            ;
    }
    status = gyro.begin();
    if (status < 0) {
        Serial2.println("Gyro Initialization Error");
        Serial2.println(status);
        tft.printf("Gyro Initialization Error: %d\n", status);
        while (1)
            ;
    }
    if (!Magneto::begin(Magneto::MODE_CONTINUOUS, Magneto::DATARATE_50HZ,
                        Magneto::RANGE_2GA, Magneto::OVERSAMPLE_512)) {
        Serial2.println("Compass Initialization Error");
        tft.printf("Compass Initialization Error");
        while (1)
            ;
    }

    pinMode(PB4, OUTPUT);
    pinMode(PB5, OUTPUT);
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(0, 0);
}

void loop() {

    static uint8_t cursor = 0;
    /* read the accel */
    accel.readSensor();
    /* read the gyro */
    gyro.readSensor();

    float magData[3];
    Magneto::getGauss(magData);

    auto magStr = String(magData[0]) + ", " + String(magData[1]) + ", " +
                  String(magData[2]);

    Serial2.print("Mag X, Y, Z: ");
    Serial2.println(magStr);

    tft.drawPixel(30 + cursor, 40 + accel.getAccelX_mss() * 2, TFT_RED);
    tft.drawPixel(30 + cursor, 40 + accel.getAccelY_mss() * 2, TFT_GREEN);
    tft.drawPixel(30 + cursor, 40 + accel.getAccelZ_mss() * 2, TFT_CYAN);
    cursor = cursor < 100 ? cursor + 1 : 0;
    tft.drawFastVLine(30 + cursor, 0, 80, TFT_BLACK);
    delay(100);

    if (Serial2.available())
        if (Serial2.read() == 'c') {
            int16_t magCal[3];
            tft.fillScreen(TFT_BLACK);
            tft.setCursor(0, 0);
            Serial2.println("Calibrating the compass, please rotate the device in all directions");
            tft.println("Calibrating the compass");
            tft.println("Please rotate the device in all directions");
            Magneto::runCalibration(magCal);
            Magneto::setCalibrationData(magCal);
            Serial2.println("Calibration done");
            tft.println("Calibration done");
            delay(1000);
        }
}
