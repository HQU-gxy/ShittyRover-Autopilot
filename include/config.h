#pragma once

#include <Arduino.h>

// LED pins
constexpr uint8_t LED1 = PB4;
constexpr uint8_t LED2 = PB5;

// IMU pins
constexpr uint8_t BMI088_ACCEL_CS = PB1;
constexpr uint8_t BMI088_GYRO_CS = PB0;
constexpr uint8_t SPI1_SCK = PA5;
constexpr uint8_t SPI1_MISO = PA6;
constexpr uint8_t SPI1_MOSI = PA7;

// Compass pins
constexpr uint8_t I2C1_SDA = PB9;
constexpr uint8_t I2C1_SCL = PA15;

// Compass magnetic declination
constexpr float MAGNETIC_DECLINATION = -(4.0 + (38.0 / 60.0)) / (180 / PI);

// UART pins
constexpr uint8_t UART1_TX = PB7;
constexpr uint8_t UART1_RX = PB6;
constexpr uint8_t UART2_TX = PA2;
constexpr uint8_t UART2_RX = PA3;
constexpr uint8_t UART3_TX = PB10;
constexpr uint8_t UART3_RX = PB11;
